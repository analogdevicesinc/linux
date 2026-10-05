// SPDX-License-Identifier: GPL-2.0+
// Copyright IBM Corp 2019
/*
 * The DPS310 is a barometric pressure and temperature sensor.
 *
 * https://www.infineon.com/dgdl/?fileId=5546d462576f34750157750826c42242
 *
 * Temperature calculation:
 *   c0 * 0.5 + c1 * T_raw / kT °C
 */

#include <linux/cleanup.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define DPS310_DEV_NAME		"dps310"

#define DPS310_PRS_B0		0x00
#define DPS310_PRS_B1		0x01
#define DPS310_PRS_B2		0x02
#define DPS310_TMP_B0		0x03
#define DPS310_TMP_B1		0x04
#define DPS310_TMP_B2		0x05
#define DPS310_PRS_CFG		0x06
#define  DPS310_PRS_RATE_BITS	GENMASK(6, 4)
#define  DPS310_PRS_PRC_BITS	GENMASK(3, 0)
#define DPS310_TMP_CFG		0x07
#define  DPS310_TMP_RATE_BITS	GENMASK(6, 4)
#define  DPS310_TMP_PRC_BITS	GENMASK(3, 0)
#define  DPS310_TMP_EXT		BIT(7)
#define DPS310_MEAS_CFG		0x08
#define  DPS310_MEAS_CTRL_BITS	GENMASK(2, 0)
#define   DPS310_PRS_EN		BIT(0)
#define   DPS310_TEMP_EN	BIT(1)
#define   DPS310_BACKGROUND	BIT(2)
#define  DPS310_PRS_RDY		BIT(4)
#define  DPS310_TMP_RDY		BIT(5)
#define  DPS310_SENSOR_RDY	BIT(6)
#define  DPS310_COEF_RDY	BIT(7)
#define DPS310_CFG_REG		0x09
#define  DPS310_INT_HL		BIT(7)
#define  DPS310_TMP_SHIFT_EN	BIT(3)
#define  DPS310_PRS_SHIFT_EN	BIT(2)
#define  DPS310_FIFO_EN		BIT(1)
#define  DPS310_SPI_EN		BIT(0)
#define DPS310_RESET		0x0c
#define  DPS310_FIFO_FLUSH	BIT(7)
#define  DPS310_RESET_MAGIC	0x09
#define DPS310_COEF_BASE	0x10

/* Section 4.8: 32 shared entries. Stops when full, so late drains lose data */
#define DPS310_FIFO_DEPTH	32

/* Read back once the FIFO is empty */
#define DPS310_FIFO_EMPTY_VAL	0x800000

/* LSB tags which measurement produced the entry */
#define DPS310_FIFO_TAG_PRS	BIT(0)

#define DPS310_DRAIN_MIN_MS	20
#define DPS310_DRAIN_MAX_MS	(2 * MSEC_PER_SEC)

/* Make sure sleep time is <= 30ms for usleep_range */
#define DPS310_POLL_SLEEP_US(t)		min(30000, (t) / 8)
/* Silently handle error in rate value here */
#define DPS310_POLL_TIMEOUT_US(rc)	((rc) <= 0 ? 1000000 : 1000000 / (rc))

#define DPS310_PRS_BASE		DPS310_PRS_B0
#define DPS310_TMP_BASE		DPS310_TMP_B0

/*
 * These values (defined in the spec) indicate how to scale the raw register
 * values for each level of precision available.
 */
static const int scale_factors[] = {
	 524288,
	1572864,
	3670016,
	7864320,
	 253952,
	 516096,
	1040384,
	2088960,
};

struct dps310_data {
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock;	/* Lock for sequential HW access functions */

	s32 c0, c1;
	s32 c00, c10, c20, c30, c01, c11, c21;
	s32 pressure_raw;
	s32 temp_raw;
	bool timeout_recovery_failed;
	bool fifo_temp_valid;

	/* Used only while the FIFO is enabled */
	struct iio_dev *iio;
	struct delayed_work fifo_work;
	s32 *fifo_hold;
	unsigned int fifo_hold_max;
	unsigned int fifo_held;
	unsigned int watermark;
	unsigned int drain_interval_ms;
	s32 fifo_temp_raw;
};

enum dps310_fifo_entry {
	DPS310_FIFO_EMPTY,
	DPS310_FIFO_TEMP,
	DPS310_FIFO_PRESSURE,
};

enum dps310_scan_index {
	DPS310_SCAN_TEMP,
	DPS310_SCAN_PRESSURE,
	DPS310_SCAN_TIMESTAMP,
};

static const struct iio_chan_spec dps310_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = DPS310_SCAN_TEMP,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	},
	{
		.type = IIO_PRESSURE,
		/*
		 * _raw here is already compensated (section 4.9.1, which needs
		 * a temperature too) and in Pa; _scale converts to the kPa the
		 * ABI wants. _processed predates buffers and has to stay.
		 *
		 * Do not copy this into other drivers. A _raw that is not the
		 * raw register value is only tolerable because the alternative
		 * is losing resolution or breaking existing _processed users.
		 */
		.info_mask_separate = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |
			BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = DPS310_SCAN_PRESSURE,
		.scan_type = {
			.sign = 's',
			.realbits = 32,
			.storagebits = 32,
			.endianness = IIO_CPU,
		},
	},
	IIO_CHAN_SOFT_TIMESTAMP(DPS310_SCAN_TIMESTAMP),
};

/* To be called after checking the COEF_RDY bit in MEAS_CFG */
static int dps310_get_coefs(struct dps310_data *data)
{
	int rc;
	u8 coef[18];
	u32 c0, c1;
	u32 c00, c10, c20, c30, c01, c11, c21;

	/* Read all sensor calibration coefficients from the COEF registers. */
	rc = regmap_bulk_read(data->regmap, DPS310_COEF_BASE, coef,
			      sizeof(coef));
	if (rc < 0)
		return rc;

	/*
	 * Calculate temperature calibration coefficients c0 and c1. The
	 * numbers are 12-bit 2's complement numbers.
	 */
	c0 = (coef[0] << 4) | (coef[1] >> 4);
	data->c0 = sign_extend32(c0, 11);

	c1 = ((coef[1] & GENMASK(3, 0)) << 8) | coef[2];
	data->c1 = sign_extend32(c1, 11);

	/*
	 * Calculate pressure calibration coefficients. c00 and c10 are 20 bit
	 * 2's complement numbers, while the rest are 16 bit 2's complement
	 * numbers.
	 */
	c00 = (coef[3] << 12) | (coef[4] << 4) | (coef[5] >> 4);
	data->c00 = sign_extend32(c00, 19);

	c10 = ((coef[5] & GENMASK(3, 0)) << 16) | (coef[6] << 8) | coef[7];
	data->c10 = sign_extend32(c10, 19);

	c01 = (coef[8] << 8) | coef[9];
	data->c01 = sign_extend32(c01, 15);

	c11 = (coef[10] << 8) | coef[11];
	data->c11 = sign_extend32(c11, 15);

	c20 = (coef[12] << 8) | coef[13];
	data->c20 = sign_extend32(c20, 15);

	c21 = (coef[14] << 8) | coef[15];
	data->c21 = sign_extend32(c21, 15);

	c30 = (coef[16] << 8) | coef[17];
	data->c30 = sign_extend32(c30, 15);

	return 0;
}

/*
 * Some versions of the chip will read temperatures in the ~60C range when
 * it's actually ~20C. This is the manufacturer recommended workaround
 * to correct the issue. The registers used below are undocumented.
 */
static int dps310_temp_workaround(struct dps310_data *data)
{
	int rc;
	int reg;

	rc = regmap_read(data->regmap, 0x32, &reg);
	if (rc < 0)
		return rc;

	/*
	 * If bit 1 is set then the device is okay, and the workaround does not
	 * need to be applied
	 */
	if (reg & BIT(1))
		return 0;

	rc = regmap_write(data->regmap, 0x0e, 0xA5);
	if (rc)
		return rc;

	rc = regmap_write(data->regmap, 0x0f, 0x96);
	if (rc)
		return rc;

	rc = regmap_write(data->regmap, 0x62, 0x02);
	if (rc)
		return rc;

	rc = regmap_write(data->regmap, 0x0e, 0x00);
	if (rc)
		return rc;

	return regmap_write(data->regmap, 0x0f, 0x00);
}

static int dps310_startup(struct dps310_data *data)
{
	int rc;
	int ready;

	/*
	 * Set up pressure sensor in single sample, one measurement per second
	 * mode
	 */
	rc = regmap_write(data->regmap, DPS310_PRS_CFG, 0);
	if (rc)
		return rc;

	/*
	 * Set up external (MEMS) temperature sensor in single sample, one
	 * measurement per second mode
	 */
	rc = regmap_write(data->regmap, DPS310_TMP_CFG, DPS310_TMP_EXT);
	if (rc)
		return rc;

	/* Temp and pressure shifts are disabled when PRC <= 8 */
	rc = regmap_write_bits(data->regmap, DPS310_CFG_REG,
			       DPS310_PRS_SHIFT_EN | DPS310_TMP_SHIFT_EN, 0);
	if (rc)
		return rc;

	/* MEAS_CFG doesn't update correctly unless first written with 0 */
	rc = regmap_write_bits(data->regmap, DPS310_MEAS_CFG,
			       DPS310_MEAS_CTRL_BITS, 0);
	if (rc)
		return rc;

	/* Turn on temperature and pressure measurement in the background */
	rc = regmap_write_bits(data->regmap, DPS310_MEAS_CFG,
			       DPS310_MEAS_CTRL_BITS, DPS310_PRS_EN |
			       DPS310_TEMP_EN | DPS310_BACKGROUND);
	if (rc)
		return rc;

	/*
	 * Calibration coefficients required for reporting temperature.
	 * They are available 40ms after the device has started
	 */
	rc = regmap_read_poll_timeout(data->regmap, DPS310_MEAS_CFG, ready,
				      ready & DPS310_COEF_RDY, 10000, 40000);
	if (rc)
		return rc;

	rc = dps310_get_coefs(data);
	if (rc)
		return rc;

	return dps310_temp_workaround(data);
}

static int dps310_get_pres_precision(struct dps310_data *data, int *val)
{
	int reg_val, rc;

	rc = regmap_read(data->regmap, DPS310_PRS_CFG, &reg_val);
	if (rc < 0)
		return rc;

	*val = BIT(reg_val & GENMASK(2, 0));

	return 0;
}

static int dps310_get_temp_precision(struct dps310_data *data, int *val)
{
	int reg_val, rc;

	rc = regmap_read(data->regmap, DPS310_TMP_CFG, &reg_val);
	if (rc < 0)
		return rc;

	/*
	 * Scale factor is bottom 4 bits of the register, but 1111 is
	 * reserved so just grab bottom three
	 */
	*val = BIT(reg_val & GENMASK(2, 0));

	return 0;
}

static int dps310_set_pres_precision(struct dps310_data *data, int val)
	__must_hold(&data->lock)
{
	int rc;
	u8 shift_en;

	if (val < 0 || val > 128)
		return -EINVAL;

	shift_en = val >= 16 ? DPS310_PRS_SHIFT_EN : 0;
	rc = regmap_write_bits(data->regmap, DPS310_CFG_REG,
			       DPS310_PRS_SHIFT_EN, shift_en);
	if (rc)
		return rc;

	return regmap_update_bits(data->regmap, DPS310_PRS_CFG,
				  DPS310_PRS_PRC_BITS, ilog2(val));
}

static int dps310_set_temp_precision(struct dps310_data *data, int val)
	__must_hold(&data->lock)
{
	int rc;
	u8 shift_en;

	if (val < 0 || val > 128)
		return -EINVAL;

	shift_en = val >= 16 ? DPS310_TMP_SHIFT_EN : 0;
	rc = regmap_write_bits(data->regmap, DPS310_CFG_REG,
			       DPS310_TMP_SHIFT_EN, shift_en);
	if (rc)
		return rc;

	return regmap_update_bits(data->regmap, DPS310_TMP_CFG,
				  DPS310_TMP_PRC_BITS, ilog2(val));
}

static int dps310_set_pres_samp_freq(struct dps310_data *data, int freq)
	__must_hold(&data->lock)
{
	u8 val;

	if (freq < 0 || freq > 128)
		return -EINVAL;

	val = ilog2(freq) << 4;

	return regmap_update_bits(data->regmap, DPS310_PRS_CFG,
				  DPS310_PRS_RATE_BITS, val);
}

static int dps310_set_temp_samp_freq(struct dps310_data *data, int freq)
	__must_hold(&data->lock)
{
	u8 val;

	if (freq < 0 || freq > 128)
		return -EINVAL;

	val = ilog2(freq) << 4;

	return regmap_update_bits(data->regmap, DPS310_TMP_CFG,
				  DPS310_TMP_RATE_BITS, val);
}

static int dps310_get_pres_samp_freq(struct dps310_data *data, int *val)
{
	int reg_val, rc;

	rc = regmap_read(data->regmap, DPS310_PRS_CFG, &reg_val);
	if (rc < 0)
		return rc;

	*val = BIT((reg_val & DPS310_PRS_RATE_BITS) >> 4);

	return 0;
}

static int dps310_get_temp_samp_freq(struct dps310_data *data, int *val)
{
	int reg_val, rc;

	rc = regmap_read(data->regmap, DPS310_TMP_CFG, &reg_val);
	if (rc < 0)
		return rc;

	*val = BIT((reg_val & DPS310_TMP_RATE_BITS) >> 4);

	return 0;
}

static int dps310_get_pres_k(struct dps310_data *data, int *val)
{
	int reg_val, rc;

	rc = regmap_read(data->regmap, DPS310_PRS_CFG, &reg_val);
	if (rc < 0)
		return rc;

	*val = scale_factors[reg_val & GENMASK(2, 0)];

	return 0;
}

static int dps310_get_temp_k(struct dps310_data *data, int *val)
{
	int reg_val, rc;

	rc = regmap_read(data->regmap, DPS310_TMP_CFG, &reg_val);
	if (rc < 0)
		return rc;

	*val = scale_factors[reg_val & GENMASK(2, 0)];

	return 0;
}

static int dps310_reset_wait(struct dps310_data *data)
{
	int rc;

	rc = regmap_write(data->regmap, DPS310_RESET, DPS310_RESET_MAGIC);
	if (rc)
		return rc;

	/* Wait for device chip access: 15ms in specification */
	usleep_range(15000, 55000);
	return 0;
}

static int dps310_reset_reinit(struct dps310_data *data)
{
	int rc;

	rc = dps310_reset_wait(data);
	if (rc)
		return rc;

	return dps310_startup(data);
}

static int dps310_ready_status(struct dps310_data *data, int ready_bit, int timeout)
{
	int sleep = DPS310_POLL_SLEEP_US(timeout);
	int ready;

	return regmap_read_poll_timeout(data->regmap, DPS310_MEAS_CFG, ready, ready & ready_bit,
					sleep, timeout);
}

static int dps310_ready(struct dps310_data *data, int ready_bit, int timeout)
	__must_hold(&data->lock)
{
	int rc;

	rc = dps310_ready_status(data, ready_bit, timeout);
	if (rc) {
		if (rc == -ETIMEDOUT && !data->timeout_recovery_failed) {
			/* Reset and reinitialize the chip. */
			if (dps310_reset_reinit(data)) {
				data->timeout_recovery_failed = true;
			} else {
				/* Try again to get sensor ready status. */
				if (dps310_ready_status(data, ready_bit, timeout))
					data->timeout_recovery_failed = true;
				else
					return 0;
			}
		}

		return rc;
	}

	data->timeout_recovery_failed = false;
	return 0;
}

static int dps310_read_pres_raw_locked(struct dps310_data *data)
	__must_hold(&data->lock)
{
	int rc;
	int rate;
	int timeout;
	u8 val[3];

	rc = dps310_get_pres_samp_freq(data, &rate);
	if (rc)
		return rc;

	timeout = DPS310_POLL_TIMEOUT_US(rate);

	/* Poll for sensor readiness; base the timeout upon the sample rate. */
	rc = dps310_ready(data, DPS310_PRS_RDY, timeout);
	if (rc)
		return rc;

	rc = regmap_bulk_read(data->regmap, DPS310_PRS_BASE, val, sizeof(val));
	if (rc < 0)
		return rc;

	data->pressure_raw = sign_extend32(get_unaligned_be24(val), 23);

	return 0;
}

static int dps310_read_temp_ready(struct dps310_data *data)
	__must_hold(&data->lock)
{
	int rc;
	u8 val[3];

	rc = regmap_bulk_read(data->regmap, DPS310_TMP_BASE, val, sizeof(val));
	if (rc < 0)
		return rc;

	data->temp_raw = sign_extend32(get_unaligned_be24(val), 23);

	return 0;
}

static int dps310_read_temp_raw_locked(struct dps310_data *data)
	__must_hold(&data->lock)
{
	int rc;
	int rate;
	int timeout;

	rc = dps310_get_temp_samp_freq(data, &rate);
	if (rc)
		return rc;

	timeout = DPS310_POLL_TIMEOUT_US(rate);

	/* Poll for sensor readiness; base the timeout upon the sample rate. */
	rc = dps310_ready(data, DPS310_TMP_RDY, timeout);
	if (rc)
		return rc;

	return dps310_read_temp_ready(data);
}

/* Best effort: on error the previous temperature stands */
static void dps310_refresh_temp_locked(struct dps310_data *data)
	__must_hold(&data->lock)
{
	int rc;
	int t_ready;

	rc = regmap_read(data->regmap, DPS310_MEAS_CFG, &t_ready);
	if (rc)
		return;

	if (t_ready & DPS310_TMP_RDY)
		dps310_read_temp_ready(data);
}

static bool dps310_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DPS310_PRS_CFG:
	case DPS310_TMP_CFG:
	case DPS310_MEAS_CFG:
	case DPS310_CFG_REG:
	case DPS310_RESET:
	/* No documentation available on the registers below */
	case 0x0e:
	case 0x0f:
	case 0x62:
		return true;
	default:
		return false;
	}
}

static bool dps310_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case DPS310_PRS_B0:
	case DPS310_PRS_B1:
	case DPS310_PRS_B2:
	case DPS310_TMP_B0:
	case DPS310_TMP_B1:
	case DPS310_TMP_B2:
	case DPS310_MEAS_CFG:
	case 0x32:	/* No documentation available on this register */
		return true;
	default:
		return false;
	}
}

static int dps310_write_raw(struct iio_dev *iio,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long mask)
{
	struct dps310_data *data = iio_priv(iio);

	/* Reconfiguring mid-capture would change the values being captured */
	IIO_DEV_ACQUIRE_DIRECT_MODE(iio, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	ACQUIRE(mutex_intr, lock)(&data->lock);
	if (ACQUIRE_ERR(mutex_intr, &lock))
		return -EINTR;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->type) {
		case IIO_PRESSURE:
			return dps310_set_pres_samp_freq(data, val);

		case IIO_TEMP:
			return dps310_set_temp_samp_freq(data, val);

		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (chan->type) {
		case IIO_PRESSURE:
			return dps310_set_pres_precision(data, val);

		case IIO_TEMP:
			return dps310_set_temp_precision(data, val);

		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

/* Fixed point one for the compensation fractions */
#define DPS310_CALC_ONE		((s64)BIT(20))

/* *acc = (*acc + coef) * raw / k, with coef scaled to match *acc */
static bool dps310_horner_step(s64 *acc, s32 coef, s64 raw, s64 k)
{
	s64 prod;

	if (check_mul_overflow(*acc + coef * DPS310_CALC_ONE, raw, &prod))
		return false;

	*acc = div64_s64(prod, k);

	return true;
}

static int dps310_calculate_pressure(struct dps310_data *data, int *val)
	__must_hold(&data->lock)
{
	s64 p = data->pressure_raw;
	s64 t = data->temp_raw;
	s64 prs = 0;
	s64 tmp = 0;
	s64 pressure;
	int kp, kt;
	int rc;

	rc = dps310_get_pres_k(data, &kp);
	if (rc)
		return rc;

	rc = dps310_get_temp_k(data, &kt);
	if (rc)
		return rc;

	/* Section 4.9.1 of the DPS310 spec, Horner form to avoid overflow */
	if (!dps310_horner_step(&prs, data->c30, p, kp) ||
	    !dps310_horner_step(&prs, data->c20, p, kp) ||
	    !dps310_horner_step(&prs, data->c10, p, kp) ||
	    !dps310_horner_step(&tmp, data->c21, p, kp) ||
	    !dps310_horner_step(&tmp, data->c11, p, kp) ||
	    !dps310_horner_step(&tmp, data->c01, t, kt))
		return -EOVERFLOW;

	pressure = data->c00 + div64_s64(prs + tmp, DPS310_CALC_ONE);
	if (pressure < 0)
		return -ERANGE;

	*val = (int)min_t(s64, pressure, INT_MAX);

	return 0;
}

static int dps310_read_pressure_value(struct dps310_data *data, int *val)
{
	int rc;

	ACQUIRE(mutex_intr, lock)(&data->lock);
	if (ACQUIRE_ERR(mutex_intr, &lock))
		return -EINTR;

	rc = dps310_read_pres_raw_locked(data);
	if (rc)
		return rc;

	dps310_refresh_temp_locked(data);

	return dps310_calculate_pressure(data, val);
}

static int dps310_read_pressure(struct dps310_data *data, int *val, int *val2,
				long mask)
{
	int rc;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		rc = dps310_get_pres_samp_freq(data, val);
		if (rc)
			return rc;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_RAW:
		rc = dps310_read_pressure_value(data, val);
		if (rc)
			return rc;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		rc = dps310_read_pressure_value(data, val);
		if (rc)
			return rc;

		*val2 = 1000; /* Convert Pa to KPa per IIO ABI */
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_SCALE:
		/* The raw value is in Pa, the ABI wants kPa */
		*val = 1;
		*val2 = 1000;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		rc = dps310_get_pres_precision(data, val);
		if (rc)
			return rc;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int dps310_calculate_temp(struct dps310_data *data, int *val)
	__must_hold(&data->lock)
{
	s64 c0;
	s64 t;
	int kt, rc;

	rc = dps310_get_temp_k(data, &kt);
	if (rc)
		return rc;

	/* Obtain inverse-scaled offset */
	c0 = div_s64((s64)kt * (s64)data->c0, 2);

	/* Add the offset to the unscaled temperature */
	t = c0 + ((s64)data->temp_raw * (s64)data->c1);

	/* Convert to milliCelsius and scale the temperature */
	*val = (int)div_s64(t * 1000LL, kt);

	return 0;
}

static int dps310_read_temp_value(struct dps310_data *data, int *val)
{
	int rc;

	ACQUIRE(mutex_intr, lock)(&data->lock);
	if (ACQUIRE_ERR(mutex_intr, &lock))
		return -EINTR;

	rc = dps310_read_temp_raw_locked(data);
	if (rc)
		return rc;

	return dps310_calculate_temp(data, val);
}

static int dps310_read_temp(struct dps310_data *data, int *val, int *val2,
			    long mask)
{
	int rc;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		rc = dps310_get_temp_samp_freq(data, val);
		if (rc)
			return rc;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PROCESSED:
		rc = dps310_read_temp_value(data, val);
		if (rc)
			return rc;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		rc = dps310_get_temp_precision(data, val);
		if (rc)
			return rc;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int dps310_read_channel(struct dps310_data *data,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2, long mask)
{
	switch (chan->type) {
	case IIO_PRESSURE:
		return dps310_read_pressure(data, val, val2, mask);

	case IIO_TEMP:
		return dps310_read_temp(data, val, val2, mask);

	default:
		return -EINVAL;
	}
}

static int dps310_read_raw(struct iio_dev *iio,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct dps310_data *data = iio_priv(iio);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED: {
		/* This consumes the measurement the capture path reads */
		IIO_DEV_ACQUIRE_DIRECT_MODE(iio, claim);
		if (IIO_DEV_ACQUIRE_FAILED(claim))
			return -EBUSY;

		return dps310_read_channel(data, chan, val, val2, mask);
	}
	default:
		return dps310_read_channel(data, chan, val, val2, mask);
	}
}

static int dps310_fill_channels(struct dps310_data *data,
				const unsigned long *scan_mask,
				s32 channels[2])
	__must_hold(&data->lock)
{
	unsigned int i;
	int rc;

	/* Compensation needs a temperature, so it is sampled either way */
	rc = dps310_read_temp_raw_locked(data);
	if (rc)
		return rc;

	i = 0;
	if (test_bit(DPS310_SCAN_TEMP, scan_mask)) {
		/* Millidegrees Celsius */
		rc = dps310_calculate_temp(data, &channels[i++]);
		if (rc)
			return rc;
	}

	if (test_bit(DPS310_SCAN_PRESSURE, scan_mask)) {
		rc = dps310_read_pres_raw_locked(data);
		if (rc)
			return rc;

		/* Pascals, see the channel definition */
		rc = dps310_calculate_pressure(data, &channels[i++]);
		if (rc)
			return rc;
	}

	return 0;
}

static int dps310_fifo_hw_flush(struct dps310_data *data)
	__must_hold(&data->lock)
{
	return regmap_write(data->regmap, DPS310_RESET, DPS310_FIFO_FLUSH);
}

static int dps310_fifo_set_enable(struct dps310_data *data, bool enable)
	__must_hold(&data->lock)
{
	return regmap_assign_bits(data->regmap, DPS310_CFG_REG, DPS310_FIFO_EN,
				  enable);
}

/* No interrupt pin is wired in tree, so drain on a timer */
static unsigned int dps310_fifo_interval(struct dps310_data *data,
					 int prs_rate, int tmp_rate)
{
	unsigned int drain_interval_ms, fill_ms, want_ms;
	int scan_rate;

	fill_ms = MSEC_PER_SEC * DPS310_FIFO_DEPTH / (prs_rate + tmp_rate);

	/*
	 * There is no hardware watermark, so take it as the number of scans
	 * the user will wait for and drain at that rate. Scans come from
	 * whichever measurement drives them.
	 */
	if (test_bit(DPS310_SCAN_PRESSURE, data->iio->active_scan_mask))
		scan_rate = prs_rate;
	else
		scan_rate = tmp_rate;

	want_ms = data->watermark * MSEC_PER_SEC / scan_rate;

	drain_interval_ms = min(want_ms, fill_ms / 2);

	return clamp(drain_interval_ms, DPS310_DRAIN_MIN_MS, DPS310_DRAIN_MAX_MS);
}

/* Returns which measurement the entry came from, or a negative error */
static int dps310_fifo_read_entry(struct dps310_data *data, s32 *value)
	__must_hold(&data->lock)
{
	u8 val[3];
	s32 raw;
	int rc;

	/* Entries come out of the pressure registers whichever made them */
	rc = regmap_bulk_read(data->regmap, DPS310_PRS_BASE, val, sizeof(val));
	if (rc < 0)
		return rc;

	raw = get_unaligned_be24(val);
	if (raw == DPS310_FIFO_EMPTY_VAL)
		return DPS310_FIFO_EMPTY;

	*value = sign_extend32(raw, 23);

	return raw & DPS310_FIFO_TAG_PRS ? DPS310_FIFO_PRESSURE : DPS310_FIFO_TEMP;
}

static int dps310_fifo_push_scan(struct dps310_data *data, s32 temp_raw,
				 s32 pressure_raw)
	__must_hold(&data->lock)
{
	struct iio_dev *iio = data->iio;
	s32 channels[2] = { };
	unsigned int i;
	int rc;

	/* Direct-mode claim keeps sysfs reads off these */
	data->temp_raw = temp_raw;
	data->pressure_raw = pressure_raw;

	i = 0;
	if (test_bit(DPS310_SCAN_TEMP, iio->active_scan_mask)) {
		rc = dps310_calculate_temp(data, &channels[i++]);
		if (rc)
			return rc;
	}

	if (test_bit(DPS310_SCAN_PRESSURE, iio->active_scan_mask)) {
		rc = dps310_calculate_pressure(data, &channels[i++]);
		if (rc)
			return rc;
	}

	iio_push_to_buffers(iio, channels);

	return 0;
}

/* Pressure seen before any temperature, kept until one turns up */
static void dps310_fifo_hold(struct dps310_data *data, s32 pressure_raw)
	__must_hold(&data->lock)
{
	if (data->fifo_held < data->fifo_hold_max)
		data->fifo_hold[data->fifo_held++] = pressure_raw;
}

/* Returns scans pushed. One that fails to compensate is dropped. */
static unsigned int dps310_fifo_push_held(struct dps310_data *data)
	__must_hold(&data->lock)
{
	unsigned int pushed = 0;

	for (unsigned int i = 0; i < data->fifo_held; i++) {
		if (!dps310_fifo_push_scan(data, data->fifo_temp_raw,
					   data->fifo_hold[i]))
			pushed++;
	}

	data->fifo_held = 0;

	return pushed;
}

/*
 * Read the batch out before compensating it, so a pressure entry pairs with
 * the temperature preceding it rather than the last one in the batch. Stop
 * at max_scans, as reading an entry removes it from the FIFO. Zero means no
 * limit.
 *
 * Returns scans pushed.
 */
static int dps310_fifo_drain(struct dps310_data *data, unsigned int max_scans)
	__must_hold(&data->lock)
{
	bool pressure_enabled = test_bit(DPS310_SCAN_PRESSURE,
					 data->iio->active_scan_mask);
	u8 kind[DPS310_FIFO_DEPTH];
	s32 raw[DPS310_FIFO_DEPTH];
	unsigned int scans = 0;
	unsigned int pushed = 0;
	unsigned int cnt;
	bool is_pressure;
	int rc;

	for (cnt = 0; cnt < DPS310_FIFO_DEPTH; cnt++) {
		if (max_scans && scans >= max_scans)
			break;

		rc = dps310_fifo_read_entry(data, &raw[cnt]);
		if (rc < 0)
			return rc;

		if (rc == DPS310_FIFO_EMPTY)
			break;

		is_pressure = rc == DPS310_FIFO_PRESSURE;
		kind[cnt] = rc;

		/* Only the measurement that drives the scans counts */
		if (is_pressure == pressure_enabled)
			scans++;
	}

	for (unsigned int i = 0; i < cnt; i++) {
		if (kind[i] == DPS310_FIFO_TEMP) {
			data->fifo_temp_raw = raw[i];
			data->fifo_temp_valid = true;

			if (pressure_enabled)
				pushed += dps310_fifo_push_held(data);
			else if (!dps310_fifo_push_scan(data, raw[i], 0))
				pushed++;
			continue;
		}

		if (!pressure_enabled)
			continue;

		if (!data->fifo_temp_valid) {
			dps310_fifo_hold(data, raw[i]);
			continue;
		}

		if (!dps310_fifo_push_scan(data, data->fifo_temp_raw, raw[i]))
			pushed++;
	}

	return pushed;
}

static void dps310_fifo_work(struct work_struct *work)
{
	struct dps310_data *data = container_of(to_delayed_work(work),
						struct dps310_data, fifo_work);
	int rc;

	scoped_guard(mutex, &data->lock)
		rc = dps310_fifo_drain(data, 0);

	if (rc < 0)
		dev_dbg(&data->client->dev, "FIFO drain failed: %d\n", rc);

	schedule_delayed_work(&data->fifo_work,
			      msecs_to_jiffies(data->drain_interval_ms));
}

/*
 * First temperature is one temperature period away at most, which bounds the
 * pressure before it. Rates cannot change while the buffer runs.
 */
static int dps310_fifo_hold_alloc(struct dps310_data *data, int prs_rate,
				  int tmp_rate)
	__must_hold(&data->lock)
{
	data->fifo_temp_valid = false;
	data->fifo_held = 0;
	data->fifo_hold = NULL;
	data->fifo_hold_max = 0;

	if (!test_bit(DPS310_SCAN_PRESSURE, data->iio->active_scan_mask))
		return 0;

	data->fifo_hold_max = prs_rate / tmp_rate + 2;
	data->fifo_hold = kcalloc(data->fifo_hold_max, sizeof(*data->fifo_hold),
				  GFP_KERNEL);
	if (!data->fifo_hold)
		return -ENOMEM;

	return 0;
}

static int dps310_hwfifo_flush(struct iio_dev *iio, unsigned int count)
{
	struct dps310_data *data = iio_priv(iio);

	/* Holding the mode keeps active_scan_mask alive across the drain */
	IIO_DEV_GUARD_CURRENT_MODE(iio);

	/* A trigger drives the capture instead and leaves the FIFO empty */
	if (iio_device_get_current_mode(iio) != INDIO_BUFFER_SOFTWARE)
		return 0;

	guard(mutex)(&data->lock);

	return dps310_fifo_drain(data, count);
}

static int dps310_hwfifo_set_watermark(struct iio_dev *iio, unsigned int val)
{
	struct dps310_data *data = iio_priv(iio);

	data->watermark = clamp(val, 1, DPS310_FIFO_DEPTH);

	return 0;
}

static int dps310_fifo_start(struct dps310_data *data)
	__must_hold(&data->lock)
{
	int rc, prs_rate, tmp_rate;

	rc = dps310_get_pres_samp_freq(data, &prs_rate);
	if (rc)
		return rc;

	rc = dps310_get_temp_samp_freq(data, &tmp_rate);
	if (rc)
		return rc;

	data->drain_interval_ms = dps310_fifo_interval(data, prs_rate, tmp_rate);

	rc = dps310_fifo_hold_alloc(data, prs_rate, tmp_rate);
	if (rc)
		return rc;

	/* Drop whatever accumulated before enable */
	rc = dps310_fifo_hw_flush(data);
	if (rc) {
		kfree(data->fifo_hold);
		return rc;
	}

	rc = dps310_fifo_set_enable(data, true);
	if (rc) {
		kfree(data->fifo_hold);
		return rc;
	}

	schedule_delayed_work(&data->fifo_work,
			      msecs_to_jiffies(data->drain_interval_ms));

	return 0;
}

static int dps310_buffer_postenable(struct iio_dev *iio)
{
	struct dps310_data *data = iio_priv(iio);

	/* An attached trigger drives the capture instead, FIFO stays off */
	if (iio_device_get_current_mode(iio) == INDIO_BUFFER_TRIGGERED)
		return 0;

	/* Entries are not timestamped and the drain timer is no substitute */
	if (iio_scan_timestamp_enabled(iio))
		return -EINVAL;

	guard(mutex)(&data->lock);

	return dps310_fifo_start(data);
}

static int dps310_buffer_predisable(struct iio_dev *iio)
{
	struct dps310_data *data = iio_priv(iio);
	int rc;

	if (iio_device_get_current_mode(iio) == INDIO_BUFFER_TRIGGERED)
		return 0;

	cancel_delayed_work_sync(&data->fifo_work);

	guard(mutex)(&data->lock);

	kfree(data->fifo_hold);

	rc = dps310_fifo_set_enable(data, false);
	if (rc)
		return rc;

	return dps310_fifo_hw_flush(data);
}

static const struct iio_buffer_setup_ops dps310_buffer_setup_ops = {
	.postenable = dps310_buffer_postenable,
	.predisable = dps310_buffer_predisable,
};

static irqreturn_t dps310_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *iio = pf->indio_dev;
	struct dps310_data *data = iio_priv(iio);
	struct {
		s32 channels[2];
		aligned_s64 timestamp;
	} scan = { };
	int rc;

	mutex_lock(&data->lock);
	rc = dps310_fill_channels(data, iio->active_scan_mask, scan.channels);
	mutex_unlock(&data->lock);
	if (rc)
		goto err;

	iio_push_to_buffers_with_ts(iio, &scan, sizeof(scan), iio_get_time_ns(iio));

err:
	iio_trigger_notify_done(iio->trig);

	return IRQ_HANDLED;
}

static void dps310_reset(void *action_data)
{
	struct dps310_data *data = action_data;

	dps310_reset_wait(data);
}

static const struct regmap_config dps310_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = dps310_is_writeable_reg,
	.volatile_reg = dps310_is_volatile_reg,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 0x62, /* No documentation available on this register */
};

static const struct iio_info dps310_info = {
	.read_raw = dps310_read_raw,
	.write_raw = dps310_write_raw,
	.hwfifo_set_watermark = dps310_hwfifo_set_watermark,
	.hwfifo_flush_to_buffer = dps310_hwfifo_flush,
};

static int dps310_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct dps310_data *data;
	struct iio_dev *iio;
	int rc;

	iio = devm_iio_device_alloc(dev, sizeof(*data));
	if (!iio)
		return -ENOMEM;

	data = iio_priv(iio);
	data->client = client;
	data->iio = iio;
	data->watermark = 1;

	rc = devm_mutex_init(dev, &data->lock);
	if (rc)
		return rc;

	iio->name = DPS310_DEV_NAME;
	iio->channels = dps310_channels;
	iio->num_channels = ARRAY_SIZE(dps310_channels);
	iio->info = &dps310_info;
	iio->modes = INDIO_BUFFER_TRIGGERED | INDIO_BUFFER_SOFTWARE |
		     INDIO_DIRECT_MODE;

	data->regmap = devm_regmap_init_i2c(client, &dps310_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	/* Register to run the device reset when the device is removed */
	rc = devm_add_action_or_reset(dev, dps310_reset, data);
	if (rc)
		return rc;

	rc = dps310_startup(data);
	if (rc)
		return rc;

	/*
	 * The device measures continuously in background mode, so a triggered
	 * capture is just a read of the latest results. The setup ops run the
	 * FIFO drain when no trigger is attached. The trigger is not aligned
	 * with the measurements, so the timestamp is taken in the handler.
	 */
	rc = devm_iio_triggered_buffer_setup(dev, iio, NULL,
					     dps310_trigger_handler,
					     &dps310_buffer_setup_ops);
	if (rc)
		return rc;

	rc = devm_delayed_work_autocancel(dev, &data->fifo_work,
					  dps310_fifo_work);
	if (rc)
		return rc;

	rc = devm_iio_device_register(dev, iio);
	if (rc)
		return rc;

	i2c_set_clientdata(client, iio);

	return 0;
}

static const struct i2c_device_id dps310_id[] = {
	{ .name = DPS310_DEV_NAME },
	{ }
};
MODULE_DEVICE_TABLE(i2c, dps310_id);

static const struct acpi_device_id dps310_acpi_match[] = {
	{ .id = "IFX3100" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, dps310_acpi_match);

static struct i2c_driver dps310_driver = {
	.driver = {
		.name = DPS310_DEV_NAME,
		.acpi_match_table = dps310_acpi_match,
	},
	.probe = dps310_probe,
	.id_table = dps310_id,
};
module_i2c_driver(dps310_driver);

MODULE_AUTHOR("Joel Stanley <joel@jms.id.au>");
MODULE_DESCRIPTION("Infineon DPS310 pressure and temperature sensor");
MODULE_LICENSE("GPL v2");
