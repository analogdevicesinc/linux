// SPDX-License-Identifier: GPL-2.0+
/*
 * ADXL372 3-Axis Digital Accelerometer SPI driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

/* ADXL372 registers definition */
#define ADXL372_DEVID			0x00
#define ADXL372_DEVID_MST		0x01
#define ADXL372_PARTID			0x02
#define ADXL372_REVID			0x03
#define ADXL372_STATUS_1		0x04
#define ADXL372_STATUS_2		0x05
#define ADXL372_FIFO_ENTRIES_2		0x06
#define ADXL372_FIFO_ENTRIES_1		0x07
#define ADXL372_X_DATA_H		0x08
#define ADXL372_X_DATA_L		0x09
#define ADXL372_Y_DATA_H		0x0A
#define ADXL372_Y_DATA_L		0x0B
#define ADXL372_Z_DATA_H		0x0C
#define ADXL372_Z_DATA_L		0x0D
#define ADXL372_X_MAXPEAK_H		0x15
#define ADXL372_X_MAXPEAK_L		0x16
#define ADXL372_Y_MAXPEAK_H		0x17
#define ADXL372_Y_MAXPEAK_L		0x18
#define ADXL372_Z_MAXPEAK_H		0x19
#define ADXL372_Z_MAXPEAK_L		0x1A
#define ADXL372_OFFSET_X		0x20
#define ADXL372_OFFSET_Y		0x21
#define ADXL372_OFFSET_Z		0x22
#define ADXL372_X_THRESH_ACT_H		0x23
#define ADXL372_X_THRESH_ACT_L		0x24
#define ADXL372_Y_THRESH_ACT_H		0x25
#define ADXL372_Y_THRESH_ACT_L		0x26
#define ADXL372_Z_THRESH_ACT_H		0x27
#define ADXL372_Z_THRESH_ACT_L		0x28
#define ADXL372_TIME_ACT		0x29
#define ADXL372_X_THRESH_INACT_H	0x2A
#define ADXL372_X_THRESH_INACT_L	0x2B
#define ADXL372_Y_THRESH_INACT_H	0x2C
#define ADXL372_Y_THRESH_INACT_L	0x2D
#define ADXL372_Z_THRESH_INACT_H	0x2E
#define ADXL372_Z_THRESH_INACT_L	0x2F
#define ADXL372_TIME_INACT_H		0x30
#define ADXL372_TIME_INACT_L		0x31
#define ADXL372_X_THRESH_ACT2_H		0x32
#define ADXL372_X_THRESH_ACT2_L		0x33
#define ADXL372_Y_THRESH_ACT2_H		0x34
#define ADXL372_Y_THRESH_ACT2_L		0x35
#define ADXL372_Z_THRESH_ACT2_H		0x36
#define ADXL372_Z_THRESH_ACT2_L		0x37
#define ADXL372_HPF			0x38
#define ADXL372_FIFO_SAMPLES		0x39
#define ADXL372_FIFO_CTL		0x3A
#define ADXL372_INT1_MAP		0x3B
#define ADXL372_INT2_MAP		0x3C
#define ADXL372_TIMING			0x3D
#define ADXL372_MEASURE			0x3E
#define ADXL372_POWER_CTL		0x3F
#define ADXL372_SELF_TEST		0x40
#define ADXL372_RESET			0x41
#define ADXL372_FIFO_DATA		0x42

#define ADXL372_DEVID_VAL		0xAD
#define ADXL372_PARTID_VAL		0xFA
#define ADXL372_RESET_CODE		0x52

#define ADXL372_RD_FLAG_MSK(x)		((((x) & 0xFF) << 1) | 0x01)
#define ADXL372_WR_FLAG_MSK(x)		(((x) & 0xFF) << 1)

/* ADXL372_POWER_CTL */
#define ADXL372_POWER_CTL_MODE_MSK		GENMASK_ULL(1, 0)
#define ADXL372_POWER_CTL_MODE(x)		(((x) & 0x3) << 0)

/* ADXL372_MEASURE */
#define ADXL372_MEASURE_LINKLOOP_MSK		GENMASK_ULL(5, 4)
#define ADXL372_MEASURE_LINKLOOP_MODE(x)	(((x) & 0x3) << 4)
#define ADXL372_MEASURE_BANDWIDTH_MSK		GENMASK_ULL(2, 0)
#define ADXL372_MEASURE_BANDWIDTH_MODE(x)	(((x) & 0x7) << 0)

/* ADXL372_TIMING */
#define ADXL372_TIMING_ODR_MSK			GENMASK_ULL(7, 5)
#define ADXL372_TIMING_ODR_MODE(x)		(((x) & 0x7) << 5)

/* ADXL372_FIFO_CTL */
#define ADXL372_FIFO_CTL_FORMAT_MSK		GENMASK(5, 3)
#define ADXL372_FIFO_CTL_FORMAT_MODE(x)		(((x) & 0x7) << 3)
#define ADXL372_FIFO_CTL_MODE_MSK		GENMASK(2, 1)
#define ADXL372_FIFO_CTL_MODE_MODE(x)		(((x) & 0x3) << 1)
#define ADXL372_FIFO_CTL_SAMPLES_MSK		BIT(1)
#define ADXL372_FIFO_CTL_SAMPLES_MODE(x)	(((x) > 0xFF) ? 1 : 0)

/* ADXL372_STATUS_1 */
#define ADXL372_STATUS_1_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL372_STATUS_1_FIFO_RDY(x)		(((x) >> 1) & 0x1)
#define ADXL372_STATUS_1_FIFO_FULL(x)		(((x) >> 2) & 0x1)
#define ADXL372_STATUS_1_FIFO_OVR(x)		(((x) >> 3) & 0x1)
#define ADXL372_STATUS_1_USR_NVM_BUSY(x)	(((x) >> 5) & 0x1)
#define ADXL372_STATUS_1_AWAKE(x)		(((x) >> 6) & 0x1)
#define ADXL372_STATUS_1_ERR_USR_REGS(x)	(((x) >> 7) & 0x1)

/* ADXL372_INT1_MAP */
#define ADXL372_INT1_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL372_INT1_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL372_INT1_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL372_INT1_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL372_INT1_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL372_INT1_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_INT1_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL372_INT1_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_INT1_MAP_INACT_MSK		BIT(4)
#define ADXL372_INT1_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL372_INT1_MAP_ACT_MSK		BIT(5)
#define ADXL372_INT1_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL372_INT1_MAP_AWAKE_MSK		BIT(6)
#define ADXL372_INT1_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL372_INT1_MAP_LOW_MSK		BIT(7)
#define ADXL372_INT1_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

#define ADXL372_FIFO_SIZE			512

/*
 * At +/- 200g with 12-bit resolution, scale is computed as:
 * (200 + 200) * 9.81 / (2^12 - 1) = 0.958241
 */
#define ADXL372_USCALE	958241

enum adxl372_op_mode {
	ADXL372_STANDBY,
	ADXL372_WAKE_UP,
	ADXL372_INSTANT_ON,
	ADXL372_FULL_BW_MEASUREMENT,
};

enum adxl372_act_proc_mode {
	ADXL372_DEFAULT,
	ADXL372_LINKED,
	ADXL372_LOOPED,
};

enum adxl372_th_activity {
	ADXL372_ACTIVITY,
	ADXL372_ACTIVITY2,
	ADXL372_INACTIVITY,
};

enum adxl372_odr {
	ADXL372_ODR_400HZ,
	ADXL372_ODR_800HZ,
	ADXL372_ODR_1600HZ,
	ADXL372_ODR_3200HZ,
	ADXL372_ODR_6400HZ,
};

enum adxl372_bandwidth {
	ADXL372_BW_200HZ,
	ADXL372_BW_400HZ,
	ADXL372_BW_800HZ,
	ADXL372_BW_1600HZ,
	ADXL372_BW_3200HZ,
};

enum adxl372_fifo_format {
	ADXL372_XYZ_FIFO,
	ADXL372_X_FIFO,
	ADXL372_Y_FIFO,
	ADXL372_XY_FIFO,
	ADXL372_Z_FIFO,
	ADXL372_XZ_FIFO,
	ADXL372_YZ_FIFO,
	ADXL372_XYZ_PEAK_FIFO,
};

enum adxl372_fifo_mode {
	ADXL372_FIFO_BYPASSED,
	ADXL372_FIFO_STREAMED,
	ADXL372_FIFO_TRIGGERED,
	ADXL372_FIFO_OLD_SAVED
};

static const int adxl372_samp_freq_tbl[5] = {
	400, 800, 1600, 3200, 6400,
};

static const int adxl372_bw_freq_tbl[5] = {
	200, 400, 800, 1600, 3200,
};

#define ADXL372_ACCEL_CHANNEL(index, reg, axis) {			\
	.type = IIO_ACCEL,						\
	.address = reg,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
		BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),	\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 12,						\
		.storagebits = 16,					\
		.shift = 4,						\
		.endianness = IIO_CPU,					\
	},								\
}

static const struct iio_chan_spec adxl372_channels[] = {
	ADXL372_ACCEL_CHANNEL(0, ADXL372_X_DATA_H, X),
	ADXL372_ACCEL_CHANNEL(1, ADXL372_Y_DATA_H, Y),
	ADXL372_ACCEL_CHANNEL(2, ADXL372_Z_DATA_H, Z),
};

struct adxl372_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct iio_trigger		*dready_trig;
	enum adxl372_fifo_mode		fifo_mode;
	enum adxl372_fifo_format	fifo_format;
	enum adxl372_op_mode		op_mode;
	enum adxl372_act_proc_mode	act_proc_mode;
	enum adxl372_odr		odr;
	enum adxl372_bandwidth		bw;
	u8				fifo_set_size;
	u8				int1_bitmask;
	u8				int2_bitmask;
	u16				watermark;
	__be16				fifo_buf[512];
};

static int adxl372_read_fifo(struct adxl372_state *st, u16 fifo_entries)
{
	return regmap_bulk_read(st->regmap,
				ADXL372_RD_FLAG_MSK(ADXL372_FIFO_DATA),
				st->fifo_buf, fifo_entries * 2);
}

static int adxl372_read_axis(struct adxl372_state *st, u8 addr)
{
	__be16 regval;
	int ret;

	ret = regmap_bulk_read(st->regmap, ADXL372_RD_FLAG_MSK(addr),
			       &regval, sizeof(regval));
	if (ret < 0)
		return ret;

	return be16_to_cpu(regval);
}

static int adxl372_set_op_mode(struct adxl372_state *st,
			       enum adxl372_op_mode op_mode)
{
	int ret;

	ret = regmap_update_bits(st->regmap, ADXL372_POWER_CTL,
				 ADXL372_POWER_CTL_MODE_MSK,
				 ADXL372_POWER_CTL_MODE(op_mode));
	if (ret < 0)
		return ret;

	st->op_mode = op_mode;

	return ret;
}

static int adxl372_set_odr(struct adxl372_state *st,
			   enum adxl372_odr odr)
{
	int ret;

	ret = regmap_update_bits(st->regmap, ADXL372_TIMING,
				 ADXL372_TIMING_ODR_MSK,
				 ADXL372_TIMING_ODR_MODE(odr));
	if (ret < 0)
		return ret;

	st->odr = odr;

	return ret;
}

static int adxl372_find_closest_match(const int *array,
				      unsigned int size, int val)
{
	int i;

	for (i = 0; i < size; i++) {
		if (val <= array[i])
			return i;
	}

	return size - 1;
}

static int adxl372_set_bandwidth(struct adxl372_state *st,
				 enum adxl372_bandwidth bw)
{
	int ret;

	ret = regmap_update_bits(st->regmap, ADXL372_MEASURE,
				 ADXL372_MEASURE_BANDWIDTH_MSK,
				 ADXL372_MEASURE_BANDWIDTH_MODE(bw));
	if (ret < 0)
		return ret;

	st->bw = bw;

	return ret;
}

static int adxl372_set_act_proc_mode(struct adxl372_state *st,
				     enum adxl372_act_proc_mode mode)
{
	int ret;

	ret = regmap_update_bits(st->regmap,
				 ADXL372_MEASURE,
				 ADXL372_MEASURE_LINKLOOP_MSK,
				 ADXL372_MEASURE_LINKLOOP_MODE(mode));
	if (ret < 0)
		return ret;

	st->act_proc_mode = mode;

	return ret;
}

static int adxl372_set_activity_threshold(struct adxl372_state *st,
					  enum adxl372_th_activity act,
					  bool ref_en, bool enable,
					  unsigned int threshold)
{
	unsigned char buf[6];
	unsigned char th_reg_high_val, th_reg_low_val, th_reg_high_addr;

	/* scale factor is 100 mg/code */
	th_reg_high_val = (threshold / 100) >> 3;
	th_reg_low_val = ((threshold / 100) << 5) | (ref_en << 1) | enable;

	switch (act) {
	case ADXL372_ACTIVITY:
		th_reg_high_addr = ADXL372_X_THRESH_ACT_H;
		break;
	case ADXL372_ACTIVITY2:
		th_reg_high_addr = ADXL372_X_THRESH_ACT2_H;
		break;
	case ADXL372_INACTIVITY:
		th_reg_high_addr = ADXL372_X_THRESH_INACT_H;
		break;
	}

	buf[0] = th_reg_high_val;
	buf[1] = th_reg_low_val;
	buf[2] = th_reg_high_val;
	buf[3] = th_reg_low_val;
	buf[4] = th_reg_high_val;
	buf[5] = th_reg_low_val;

	return regmap_bulk_write(st->regmap,
				 ADXL372_WR_FLAG_MSK(th_reg_high_addr),
				 buf, ARRAY_SIZE(buf));
}

static int adxl372_set_interrupts(struct adxl372_state *st,
				  unsigned char int1_bitmask,
				  unsigned char int2_bitmask)
{
	unsigned char buf[2];
	int ret;

	buf[0] = int1_bitmask;
	buf[1] = int2_bitmask;

	/* INT1_MAP and INT2_MAP are adjacent registers */
	ret = regmap_bulk_write(st->regmap,
				ADXL372_WR_FLAG_MSK(ADXL372_INT1_MAP),
				buf, ARRAY_SIZE(buf));
	if (ret < 0)
		return ret;

	st->int1_bitmask = int1_bitmask;
	st->int2_bitmask = int2_bitmask;

	return ret;
}

static int adxl372_configure_fifo(struct adxl372_state *st)
{
	unsigned char buf[2];
	int ret;

	/* FIFO must be configured while in standby mode */
	ret = adxl372_set_op_mode(st, ADXL372_STANDBY);
	if (ret < 0)
		return ret;

	buf[0] = st->watermark & 0xFF;
	buf[1] = ADXL372_FIFO_CTL_FORMAT_MODE(st->fifo_format) |
		 ADXL372_FIFO_CTL_MODE_MODE(st->fifo_mode) |
		 ADXL372_FIFO_CTL_SAMPLES_MODE(st->watermark);

	/* FIFO_SAMPLES and FIFO_CTL are adjacent registers */
	ret = regmap_bulk_write(st->regmap,
				ADXL372_WR_FLAG_MSK(ADXL372_FIFO_SAMPLES),
				buf, ARRAY_SIZE(buf));
	if (ret < 0)
		return ret;

	return adxl372_set_op_mode(st, ADXL372_FULL_BW_MEASUREMENT);
}

static int adxl372_get_status(struct adxl372_state *st,
			      u8 *status1, u8 *status2,
			      u16 *fifo_entries)
{
	unsigned char buf[4];
	int ret;

	/* STATUS, STATUS2, FIFO_ENTRIES2 and FIFO_ENTRIES are adjacent regs */
	ret = regmap_bulk_read(st->regmap,
			       ADXL372_RD_FLAG_MSK(ADXL372_STATUS_1),
			       buf, ARRAY_SIZE(buf));
	if (ret < 0)
		return ret;

	*status1 = buf[0];
	*status2 = buf[1];
	/*
	 * FIFO_ENTRIES contains the least significant byte, and FIFO_ENTRIES2
	 * contains the two most significant bits
	 */
	*fifo_entries = ((buf[2] & 0x3) << 8) | buf[3];

	return ret;
}

static irqreturn_t adxl372_trigger_handler(int irq, void  *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adxl372_state *st = iio_priv(indio_dev);
	u8 status1, status2;
	u16 fifo_entries, i;
	int ret;

	ret = adxl372_get_status(st, &status1, &status2, &fifo_entries);
	if (ret < 0)
		goto err;

	if ((st->fifo_mode != ADXL372_FIFO_BYPASSED) &&
	    (ADXL372_STATUS_1_FIFO_FULL(status1))) {
		/*
		 * When reading data from multiple axes from the FIFO,
		 * to ensure that data is not overwritten and stored out
		 * of order at least one sample set must be left in the
		 * FIFO after every read.
		 */
		fifo_entries -= st->fifo_set_size;

		ret = adxl372_read_fifo(st, fifo_entries);
		if (ret < 0)
			goto err;

		for (i = 0; i < fifo_entries * 2; i += st->fifo_set_size * 2)
			iio_push_to_buffers(indio_dev, &st->fifo_buf[i]);
	}
err:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int adxl372_setup(struct adxl372_state *st)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(st->regmap, ADXL372_RD_FLAG_MSK(ADXL372_DEVID),
			  &regval);
	if (ret < 0)
		return ret;

	if (regval != ADXL372_DEVID_VAL) {
		dev_err(&st->spi->dev, "Invalid chip id %x\n", regval);
		return -ENODEV;
	}

	ret = adxl372_set_op_mode(st, ADXL372_STANDBY);
	if (ret < 0)
		return ret;

	/* Set threshold for activity detection to 500mg */
	ret = adxl372_set_activity_threshold(st, ADXL372_ACTIVITY,
					     true, true, 500);
	if (ret < 0)
		return ret;

	/* Set threshold for inactivity detection to 500mg */
	ret = adxl372_set_activity_threshold(st, ADXL372_INACTIVITY,
					     true, true, 500);
	if (ret < 0)
		return ret;

	/* Set activity processing in Looped mode */
	ret = adxl372_set_act_proc_mode(st, ADXL372_LOOPED);
	if (ret < 0)
		return ret;

	ret = adxl372_set_odr(st, ADXL372_ODR_6400HZ);
	if (ret < 0)
		return ret;

	ret = adxl372_set_bandwidth(st, ADXL372_BW_3200HZ);
	if (ret < 0)
		return ret;

	/* Set activity timer */
	ret = regmap_write(st->regmap,
			   ADXL372_WR_FLAG_MSK(ADXL372_TIME_ACT), 1);
	if (ret < 0)
		return ret;

	/* Set inactivity timer to 1s */
	ret = regmap_write(st->regmap,
			   ADXL372_WR_FLAG_MSK(ADXL372_TIME_INACT_L), 0x28);
	if (ret < 0)
		return ret;

	/* Set the mode of operation to full bandwidth measurement mode */
	return adxl372_set_op_mode(st, ADXL372_FULL_BW_MEASUREMENT);
}

static int adxl372_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adxl372_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, ADXL372_RD_FLAG_MSK(reg),
				   readval);
	else
		return regmap_write(st->regmap, ADXL372_WR_FLAG_MSK(reg),
				    writeval);
}

static int adxl372_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct adxl372_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = adxl372_read_axis(st, chan->address);
		iio_device_release_direct_mode(indio_dev);
		if (ret < 0)
			return ret;

		*val = sign_extend32(ret >> chan->scan_type.shift,
				     chan->scan_type.realbits - 1);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = ADXL372_USCALE;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adxl372_samp_freq_tbl[st->odr];
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*val = adxl372_bw_freq_tbl[st->bw];
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adxl372_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct adxl372_state *st = iio_priv(indio_dev);
	int odr_index, bw_index, ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		odr_index = adxl372_find_closest_match(adxl372_samp_freq_tbl,
					ARRAY_SIZE(adxl372_samp_freq_tbl),
					val);
		ret = adxl372_set_odr(st, odr_index);
		if (ret < 0)
			return ret;
		/*
		 * The maximum bandwidth is constrained to at most half of
		 * the ODR to ensure that the Nyquist criteria is not violated
		 */
		if (st->bw > odr_index)
			ret = adxl372_set_bandwidth(st, odr_index);

		return ret;
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		bw_index = adxl372_find_closest_match(adxl372_bw_freq_tbl,
					ARRAY_SIZE(adxl372_bw_freq_tbl),
					val);
		return adxl372_set_bandwidth(st, bw_index);
	default:
		return -EINVAL;
	}
}

static ssize_t adxl372_show_filter_freq_avail(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxl372_state *st = iio_priv(indio_dev);
	int i;
	size_t len = 0;

	for (i = 0; i <= st->odr; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "%d ", adxl372_bw_freq_tbl[i]);

	buf[len - 1] = '\n';

	return len;
}

static ssize_t adxl372_get_fifo_enabled(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxl372_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->fifo_mode);
}

static ssize_t adxl372_get_fifo_watermark(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adxl372_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->watermark);
}

static IIO_CONST_ATTR(hwfifo_watermark_min, "1");
static IIO_CONST_ATTR(hwfifo_watermark_max,
		      __stringify(ADXL372_FIFO_SIZE));
static IIO_DEVICE_ATTR(hwfifo_watermark, 0444,
		       adxl372_get_fifo_watermark, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_enabled, 0444,
		       adxl372_get_fifo_enabled, NULL, 0);

static const struct attribute *adxl372_fifo_attributes[] = {
	&iio_const_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_const_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	NULL,
};

static int adxl372_set_watermark(struct iio_dev *indio_dev, unsigned int val)
{
	struct adxl372_state *st  = iio_priv(indio_dev);

	if (val > ADXL372_FIFO_SIZE)
		val = ADXL372_FIFO_SIZE;

	st->watermark = val;

	return 0;
}

static int adxl372_buffer_postenable(struct iio_dev *indio_dev)
{
	struct adxl372_state *st = iio_priv(indio_dev);
	u8 fifo_set_size, accel_axis_en;
	int bit, ret;

	if (!st->watermark)
		return -EINVAL;

	ret = adxl372_set_interrupts(st,
				     ADXL372_INT1_MAP_FIFO_FULL_MSK,
				     0);
	if (ret < 0)
		return ret;

	fifo_set_size = 0;
	accel_axis_en = 0;
	for_each_set_bit(bit, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		accel_axis_en |= bit;
		fifo_set_size++;
	}

	switch (accel_axis_en) {
	case 0:
		st->fifo_format = ADXL372_X_FIFO;
		break;
	case 1:
		if (fifo_set_size == 1)
			st->fifo_format = ADXL372_Y_FIFO;
		else
			st->fifo_format = ADXL372_XY_FIFO;
		break;
	case 2:
		if (fifo_set_size == 1)
			st->fifo_format = ADXL372_Z_FIFO;
		else
			st->fifo_format = ADXL372_XZ_FIFO;
		break;
	default: /* case 3 */
		if (fifo_set_size == 3)
			st->fifo_format = ADXL372_XYZ_PEAK_FIFO;
		else
			st->fifo_format = ADXL372_YZ_FIFO;
		break;
	}

	/*
	 * The 512 FIFO samples can be allotted in several ways, such as:
	 * 170 sample sets of concurrent 3-axis data
	 * 256 sample sets of concurrent 2-axis data (user selectable)
	 * 512 sample sets of single-axis data
	 */
	if ((st->watermark * fifo_set_size) > ADXL372_FIFO_SIZE)
		st->watermark = (ADXL372_FIFO_SIZE  / fifo_set_size);

	st->fifo_set_size = fifo_set_size;
	st->fifo_mode = ADXL372_FIFO_STREAMED;

	ret = adxl372_configure_fifo(st);
	if (ret < 0) {
		st->fifo_mode = ADXL372_FIFO_BYPASSED;
		adxl372_set_interrupts(st, 0, 0);
	}

	return ret;
}

static int adxl372_buffer_predisable(struct iio_dev *indio_dev)
{
	struct adxl372_state *st = iio_priv(indio_dev);

	adxl372_set_interrupts(st, 0, 0);
	st->fifo_mode = ADXL372_FIFO_BYPASSED;
	adxl372_configure_fifo(st);

	return 0;
}

static const struct iio_buffer_setup_ops adxl372_buffer_ops = {
	.postenable = adxl372_buffer_postenable,
	.predisable = adxl372_buffer_predisable,
};

static int adxl372_dready_trig_set_state(struct iio_trigger *trig,
					 bool state)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct adxl372_state *st = iio_priv(indio_dev);
	unsigned long int mask = 0;

	if (state)
		mask = ADXL372_INT1_MAP_FIFO_FULL_MSK;

	return adxl372_set_interrupts(st, mask, 0);
}

static const struct iio_trigger_ops adxl372_trigger_ops = {
	.set_trigger_state = adxl372_dready_trig_set_state,
};

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("400 800 1600 3200 6400");
static IIO_DEVICE_ATTR(in_accel_filter_low_pass_3db_frequency_available,
		       0444, adxl372_show_filter_freq_avail, NULL, 0);

static struct attribute *adxl372_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_accel_filter_low_pass_3db_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group adxl372_attrs_group = {
	.attrs = adxl372_attributes,
};

static const struct iio_info adxl372_info = {
	.attrs = &adxl372_attrs_group,
	.read_raw = adxl372_read_raw,
	.write_raw = adxl372_write_raw,
	.debugfs_reg_access = &adxl372_reg_access,
	.hwfifo_set_watermark = adxl372_set_watermark,
};

static const struct regmap_config adxl372_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
};

static int adxl372_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adxl372_state *st;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	regmap = devm_regmap_init_spi(spi, &adxl372_spi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st->regmap = regmap;

	indio_dev->channels = adxl372_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl372_channels);
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &adxl372_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;

	ret = adxl372_setup(st);
	if (ret < 0) {
		dev_err(&st->spi->dev, "ADXL372 setup failed\n");
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(&st->spi->dev,
					      indio_dev, NULL,
					      adxl372_trigger_handler,
					      &adxl372_buffer_ops);
	if (ret < 0)
		return ret;

	st->dready_trig = devm_iio_trigger_alloc(&st->spi->dev,
			  "%s-dev%d",
			  indio_dev->name);
	if (st->dready_trig == NULL)
		return -ENOMEM;

	st->dready_trig->ops = &adxl372_trigger_ops;
	st->dready_trig->dev.parent = &st->spi->dev;
	iio_trigger_set_drvdata(st->dready_trig, indio_dev);
	ret = devm_iio_trigger_register(&st->spi->dev, st->dready_trig);
	if (ret < 0)
		return ret;

	indio_dev->trig = iio_trigger_get(st->dready_trig);

	ret = devm_request_threaded_irq(&st->spi->dev, st->spi->irq,
					iio_trigger_generic_data_rdy_poll,
					NULL,
					IRQF_TRIGGER_RISING |
					IRQF_ONESHOT,
					indio_dev->name,
					st->dready_trig);
	if (ret < 0)
		return ret;

	iio_buffer_set_attrs(indio_dev->buffer, adxl372_fifo_attributes);

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static const struct spi_device_id adxl372_id[] = {
	{ "adxl372", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, adxl372_id);

static struct spi_driver adxl372_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = adxl372_probe,
	.id_table = adxl372_id,
};

module_spi_driver(adxl372_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXL372 3-axis accelerometer driver");
MODULE_LICENSE("GPL v2");
