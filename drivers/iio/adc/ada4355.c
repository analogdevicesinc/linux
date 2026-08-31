// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices ADA4355/ADA4356 SPI ADC driver
 *
 * ADA4355 and ADA4356 share the same register map and CHIP_ID (0x8B).
 * The "adi,ada4356" compatible is handled identically to "adi,ada4355".
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/clk.h>

#include "cf_axi_adc.h"

/* SPI Register Map */
#define ADA4355_REG_CHIP_CONFIGURATION      0x00
#define ADA4355_REG_CHIP_ID                 0x01
#define ADA4355_REG_DEVICE_INDEX            0x05
#define ADA4355_REG_TRANFER                 0xFF
#define ADA4355_REG_POWER_MODES             0x08
#define ADA4355_REG_CLOCK                   0x09
#define ADA4355_REG_CLOCK_DIVIDE            0x0B
#define ADA4355_REG_TEST_MODE               0x0D
#define ADA4355_REG_OUTPUT_MODE             0x14
#define ADA4355_REG_OUTPUT_ADJUST           0x15
#define ADA4355_REG_OUTPUT_PHASE            0x16
#define ADA4355_REG_USER_PATT1_LSB          0x19
#define ADA4355_REG_USER_PATT1_MSB          0x1A
#define ADA4355_REG_USER_PATT2_LSB          0x1B
#define ADA4355_REG_USER_PATT2_MSB          0x1C
#define ADA4355_REG_SERIAL_OUT_DATA_CNTRL   0x21
#define ADA4355_REG_SERIAL_CHANNEL_STATUS   0x22
#define ADA4355_REG_RESOLUTION_SAMPLE_RATE  0x100
#define ADA4355_REG_USER_IN_OUT_CNTRL       0x101

/* ADA4355_REG_POWER_MODES (0x08) */
#define ADA4355_DIGITAL_RESET               GENMASK(1, 0)
#define ADA4355_NORMAL_OPERATION            0xFC

/* CHIP_ID — shared by ADA4355 and ADA4356 */
#define ADA4355_CHIP_ID                     0x8B

/* ADA4355_REG_TRANFER (0xFF) */
#define ADA4355_OVERRIDE                    BIT(0)

/* ADA4355_REG_RESOLUTION_SAMPLE_RATE: 125 MSPS override */
#define ADA4355_125_RATE                    0x06

/* ADA4355_REG_SERIAL_OUT_DATA_CNTRL */
#define ADA4355_DDR_TWO_LANE_BITWISE        0x20

/* ADA4355_REG_TEST_MODE values */
#define ADA4355_INPUT_SIGNALS               0x00
#define ADA4355_USER_INPUT                  0x48

/* ADA4355_REG_OUTPUT_MODE */
#define ADA4355_TWOSCOMP                    BIT(0)

/*
 * HDL custom register: word offset 0x32 → byte address 0x00C8
 * enable_error[2:0]:
 *   bit 0 = D0A lane data error monitoring
 *   bit 1 = D1A lane data error monitoring
 *   bit 2 = frame alignment error monitoring
 */
#define ADA4355_ENABLE_ERROR_MASK           0x00C8

/*
 * Zynq-7000 uses IDELAYE2, whose tap counter is 5 bits wide. Sweeping further
 * aliases every 32 taps, so find_opt() locks onto a repeating pattern and
 * reports success on a meaningless delay. UltraScale+ (IDELAYE3) is 9 bits and
 * needs 512 instead.
 */
#define IDELAY_NUM_TAPS 32
#define IDELAY_STEP     1
#define IDELAY_ENTRIES  (IDELAY_NUM_TAPS / IDELAY_STEP)

/* Frame lane sits above the data lanes in the up_delay_cntrl address space */
#define ADA4355_FRAME_DELAY_LANE            2

static const int ada4355_scale_table[][2] = {
	{2000, 0}, /* 2V differential range (±1V) for 1V reference */
};

struct ada4355_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clk;
	struct mutex		lock;
	unsigned int		num_lanes;

	/* Readback census for the setup transcript, see ada4355_write_verify() */
	unsigned int		rb_total;
	unsigned int		rb_mismatch;
	unsigned int		rb_ff;
};

static const struct regmap_config ada4355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static struct ada4355_state *ada4355_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	return conv->phy;
}

/*
 * Write a register, read it straight back, and log both. On the Quad ADA4356
 * FMC the SDO return path is dead, so every readback comes back 0xFF; the
 * census kept here lets ada4355_setup() tell "the part is mute" apart from
 * "this particular write did not stick", which the bare error codes cannot.
 * Self-clearing and write-only registers pass verify=false.
 */
static int ada4355_write_verify(struct ada4355_state *st, unsigned int reg,
				unsigned int val, bool verify, const char *name)
{
	unsigned int rb;
	int ret;

	ret = regmap_write(st->regmap, reg, val);
	if (ret) {
		dev_err(&st->spi->dev, "  W 0x%03X %-20s <= 0x%02X  WRITE FAILED (%d)\n",
			reg, name, val, ret);
		return ret;
	}

	if (!verify) {
		dev_info(&st->spi->dev, "  W 0x%03X %-20s <= 0x%02X  (no readback)\n",
			 reg, name, val);
		return 0;
	}

	ret = regmap_read(st->regmap, reg, &rb);
	if (ret) {
		dev_warn(&st->spi->dev, "  W 0x%03X %-20s <= 0x%02X  READ FAILED (%d)\n",
			 reg, name, val, ret);
		return 0;
	}

	st->rb_total++;
	if (rb != val)
		st->rb_mismatch++;
	if (rb == 0xFF)
		st->rb_ff++;

	dev_info(&st->spi->dev, "  W 0x%03X %-20s <= 0x%02X  RB 0x%02X  %s\n",
		 reg, name, val, rb, rb == val ? "ok" : "MISMATCH");

	return 0;
}

static int ada4355_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct ada4355_state *st = ada4355_get_data(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ada4355_get_scale(struct axiadc_converter *conv, int *val, int *val2)
{
	unsigned int tmp;

	tmp = (conv->chip_info->scale_table[0][0] * 1000000ULL) >>
	       conv->chip_info->channel[0].scan_type.realbits;
	*val = tmp / 1000000;
	*val2 = tmp % 1000000;

	return IIO_VAL_INT_PLUS_NANO;
}

static int ada4355_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ada4355_state *st = ada4355_get_data(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		return ada4355_get_scale(conv, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clk);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ada4355_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	return -EINVAL;
}

#define ADA4355_CHAN(_chan, _si, _bits, _sign, _shift) {		\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _si,						\
	.scan_type = {							\
		.sign = _sign,						\
		.realbits = _bits,					\
		.storagebits = 16,					\
		.shift = _shift,					\
	},								\
}

static const struct axiadc_chip_info ada4355_chip_info = {
	.name = "ADA4355",
	.id = ADA4355_CHIP_ID,
	.max_rate = 125000000UL,
	.scale_table = ada4355_scale_table,
	.num_scales = ARRAY_SIZE(ada4355_scale_table),
	.num_channels = 1,
	.channel[0] = ADA4355_CHAN(0, 0, 14, 's', 2),
};

static void ada4355_clk_disable(void *data)
{
	struct axiadc_converter *conv = data;

	clk_disable_unprepare(conv->clk);
}

static int find_opt(u8 *field, u32 size, u32 *ret_start)
{
	int i, cnt = 0, max_cnt = 0, start, max_start = 0;

	for (i = 0, start = -1; i < size; i++) {
		if (field[i] == 0) {
			if (start == -1)
				start = i;
			cnt++;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
			start = -1;
			cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}

	*ret_start = max_start;

	return max_cnt;
}

/*
 * Decode up_clock_mon: it counts adc_clk edges over a 65536-cycle window of the
 * 100 MHz AXI clock. A wrong or absent DCO shows up here long before it shows
 * up as a failed IDELAY sweep, so it is the first thing worth printing.
 */
static void ada4355_log_clk_mon(struct device *dev, struct axiadc_state *axi_adc_st)
{
	unsigned int freq = ADI_TO_CLK_FREQ(axiadc_read(axi_adc_st, ADI_REG_CLK_FREQ));
	unsigned int ratio = ADI_TO_CLK_RATIO(axiadc_read(axi_adc_st, ADI_REG_CLK_RATIO));
	u32 hz = (u32)(((u64)freq * ratio * 100000000ULL) >> 16);

	dev_info(dev, "  CLK_FREQ 0x%08X  CLK_RATIO 0x%08X  => adc_clk %u.%03u MHz\n",
		 freq, ratio, hz / 1000000, (hz / 1000) % 1000);

	if (!freq)
		dev_warn(dev, "  adc_clk is not running at all (no DCO reaching this instance)\n");
}

static void ada4355_log_sweep(struct device *dev, const char *what, const u8 *field)
{
	char buf[IDELAY_ENTRIES + 1];
	unsigned int i;

	for (i = 0; i < IDELAY_ENTRIES; i++)
		buf[i] = field[i] ? 'X' : '-';
	buf[IDELAY_ENTRIES] = '\0';

	dev_info(dev, "  %-10s taps 0-%u |%s|\n", what, IDELAY_ENTRIES - 1, buf);
}

static int ada4355_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct ada4355_state *st = ada4355_get_data(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct device *dev = &conv->spi->dev;
	u8 pn_status[3][IDELAY_ENTRIES];
	unsigned int opt_delay, s;
	int c;
	int ret;
	unsigned int reg_cntrl, ver, cfg;
	unsigned int i, delay, val, idx;
	bool cal_ok = true;

	ver = axiadc_read(axi_adc_st, ADI_AXI_REG_VERSION);
	cfg = axiadc_read(axi_adc_st, ADI_REG_CONFIG);

	dev_info(dev, "==== ada4355_post_setup: AXI core state ====\n");
	dev_info(dev, "  VERSION %u.%u.%u  ID 0x%08X  CONFIG 0x%08X%s\n",
		 ADI_AXI_PCORE_VER_MAJOR(ver), ADI_AXI_PCORE_VER_MINOR(ver),
		 ADI_AXI_PCORE_VER_PATCH(ver),
		 axiadc_read(axi_adc_st, ADI_AXI_REG_ID), cfg,
		 (cfg & ADI_DELAY_CONTROL_DISABLE) ? " [IDELAY CONTROL DISABLED]" : "");
	ada4355_log_clk_mon(dev, axi_adc_st);
	dev_info(dev, "  RSTN 0x%08X  CNTRL 0x%08X  STATUS 0x%08X\n",
		 axiadc_read(axi_adc_st, ADI_REG_RSTN),
		 axiadc_read(axi_adc_st, ADI_REG_CNTRL),
		 axiadc_read(axi_adc_st, ADI_REG_STATUS));

	/* Set number of lanes and assert sync */
	reg_cntrl = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	reg_cntrl |= ADI_NUM_LANES(st->num_lanes);
	reg_cntrl |= ADI_SYNC;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, reg_cntrl);
	dev_info(dev, "  CNTRL <= 0x%08X (num_lanes=%u, SYNC), RB 0x%08X\n",
		 reg_cntrl, st->num_lanes, axiadc_read(axi_adc_st, ADI_REG_CNTRL));

	axiadc_write(axi_adc_st, ADI_REG_CHAN_CNTRL(0), ADI_ENABLE);

	/* Frame lane IDELAY sweep: find widest passing window */
	axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, BIT(2));
	for (idx = 0, delay = 0; delay < IDELAY_NUM_TAPS; delay += IDELAY_STEP, idx++) {
		val = axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0));
		axiadc_write(axi_adc_st, ADI_REG_CHAN_STATUS(0), val);
		axiadc_write(axi_adc_st, ADI_REG_DELAY(ADA4355_FRAME_DELAY_LANE), delay);
		mdelay(1);
		pn_status[0][idx] =
			(axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0)) & ADI_PN_ERR) ? 1 : 0;
	}
	axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, 0);

	dev_info(dev, "---- IDELAY sweep ('-' = pass, 'X' = PN error) ----\n");
	ada4355_log_sweep(dev, "frame", &pn_status[0][0]);

	c = find_opt(&pn_status[0][0], IDELAY_ENTRIES, &s);
	if (c == 0) {
		dev_err(dev, "frame lane: no valid IDELAY window found\n");
		cal_ok = false;
	}
	opt_delay = (s + c / 2) * IDELAY_STEP;
	axiadc_write(axi_adc_st, ADI_REG_DELAY(ADA4355_FRAME_DELAY_LANE), opt_delay);
	dev_info(dev, "  frame      window [%u..%u] %d wide, delay %u, RB %u\n",
		 s, s + (c ? c - 1 : 0), c, opt_delay,
		 axiadc_read(axi_adc_st, ADI_REG_DELAY(ADA4355_FRAME_DELAY_LANE)));

	/* Data lane IDELAY sweep: one lane at a time */
	for (i = 0; i < st->num_lanes; i++) {
		axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, BIT(i));
		for (idx = 0, delay = 0; delay < IDELAY_NUM_TAPS; delay += IDELAY_STEP, idx++) {
			val = axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0));
			axiadc_write(axi_adc_st, ADI_REG_CHAN_STATUS(0), val);
			axiadc_write(axi_adc_st, ADI_REG_DELAY(i), delay);
			mdelay(1);
			pn_status[i][idx] =
				(axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0)) & ADI_PN_ERR) ? 1 : 0;
		}
		axiadc_write(axi_adc_st, ADA4355_ENABLE_ERROR_MASK, 0);
	}

	for (i = 0; i < st->num_lanes; i++) {
		char name[8];

		snprintf(name, sizeof(name), "lane %u", i);
		ada4355_log_sweep(dev, name, &pn_status[i][0]);

		c = find_opt(&pn_status[i][0], IDELAY_ENTRIES, &s);
		if (c == 0) {
			dev_err(dev, "lane %u: no valid IDELAY window found\n", i);
			cal_ok = false;
		}
		opt_delay = (s + c / 2) * IDELAY_STEP;
		axiadc_write(axi_adc_st, ADI_REG_DELAY(i), opt_delay);
		dev_info(dev, "  lane %u     window [%u..%u] %d wide, delay %u, RB %u\n",
			 i, s, s + (c ? c - 1 : 0), c, opt_delay,
			 axiadc_read(axi_adc_st, ADI_REG_DELAY(i)));
	}

	dev_info(dev, "  post-sweep STATUS 0x%08X  CHAN_STATUS(0) 0x%08X\n",
		 axiadc_read(axi_adc_st, ADI_REG_STATUS),
		 axiadc_read(axi_adc_st, ADI_REG_CHAN_STATUS(0)));

	if (cal_ok) {
		dev_info(dev, "==== IDELAY calibration complete ====\n");
	} else {
		dev_err(dev, "==== IDELAY calibration FAILED ====\n");
		/*
		 * An all-'X' sweep means the comparison never passed at any tap,
		 * which is a data/clock problem rather than a timing problem —
		 * distinguish those two for whoever reads the log.
		 */
		dev_err(dev, "all taps failing points at a missing DCO or a pattern mismatch, "
			     "not at IDELAY timing; check CLK_FREQ above and that 0xFFFC is "
			     "actually being driven\n");
	}

	axiadc_write(axi_adc_st, ADI_REG_CHAN_CNTRL(0), 0);

	/* Switch ADC back to normal input */
	ret = regmap_write(st->regmap, ADA4355_REG_TEST_MODE, ADA4355_INPUT_SIGNALS);
	if (ret) {
		dev_err(dev, "failed to restore TEST_MODE to normal input: %d\n", ret);
		return ret;
	}
	dev_info(dev, "ada4355_post_setup: TEST_MODE restored to normal input\n");

	return 0;
}

static int ada4355_setup(struct ada4355_state *st)
{
	struct device *dev = &st->spi->dev;
	unsigned int reg, id;
	int ret;
	struct gpio_desc *gpio_vld_en;

	st->rb_total = 0;
	st->rb_mismatch = 0;
	st->rb_ff = 0;

	dev_info(dev, "==== ada4355_setup: SPI register transcript ====\n");

	/* Digital reset: self-clearing, readback here would be meaningless */
	ret = ada4355_write_verify(st, ADA4355_REG_POWER_MODES,
				   ADA4355_DIGITAL_RESET, false, "POWER_MODES/rst");
	if (ret)
		return ret;

	/*
	 * First read of the session. Anything other than 0x00 here means the
	 * SDO path is suspect before a single config register has been touched.
	 */
	ret = regmap_read(st->regmap, ADA4355_REG_POWER_MODES, &reg);
	if (ret) {
		dev_err(dev, "  R 0x%03X POWER_MODES         READ FAILED (%d)\n",
			ADA4355_REG_POWER_MODES, ret);
		return ret;
	}
	dev_info(dev, "  R 0x%03X POWER_MODES         => 0x%02X (post-reset, expect 0x00)\n",
		 ADA4355_REG_POWER_MODES, reg);

	ret = ada4355_write_verify(st, ADA4355_REG_POWER_MODES,
				   0x00, true, "POWER_MODES");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_CHIP_CONFIGURATION,
				   0x00, true, "CHIP_CONFIGURATION");
	if (ret)
		return ret;

	/* Select all channels for subsequent register writes */
	ret = ada4355_write_verify(st, ADA4355_REG_DEVICE_INDEX,
				   0x02, true, "DEVICE_INDEX");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_SERIAL_CHANNEL_STATUS,
				   0x03, true, "SERIAL_CHAN_STATUS");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_DEVICE_INDEX,
				   0x31, true, "DEVICE_INDEX");
	if (ret)
		return ret;

	/* Verify chip identity — shared between ADA4355 and ADA4356 */
	ret = regmap_read(st->regmap, ADA4355_REG_CHIP_ID, &id);
	if (ret) {
		dev_err(dev, "  R 0x%03X CHIP_ID             READ FAILED (%d)\n",
			ADA4355_REG_CHIP_ID, ret);
		return ret;
	}
	dev_info(dev, "  R 0x%03X CHIP_ID             => 0x%02X (expect 0x%02X)\n",
		 ADA4355_REG_CHIP_ID, id, ADA4355_CHIP_ID);

	/* Quad ADA4356 FMC: SDO cannot return through the level shifter, so every
	 * readback is 0xFF. Writes still reach the part, so configure it blind.
	 */
	if (id != ADA4355_CHIP_ID)
		dev_warn(dev, "Unrecognized CHIP_ID 0x%02X, configuring blind\n", id);

	/* Enable DDR two-lane bitwise serial output */
	ret = ada4355_write_verify(st, ADA4355_REG_SERIAL_OUT_DATA_CNTRL,
				   ADA4355_DDR_TWO_LANE_BITWISE, true,
				   "SERIAL_OUT_DATA_CNTRL");
	if (ret)
		return ret;

	/* Commit register writes: self-clearing */
	ret = ada4355_write_verify(st, ADA4355_REG_TRANFER,
				   ADA4355_OVERRIDE, false, "TRANSFER");
	if (ret)
		return ret;

	/* User input mode with test pattern 0xFFFC for IDELAY calibration */
	ret = ada4355_write_verify(st, ADA4355_REG_TEST_MODE,
				   ADA4355_USER_INPUT, true, "TEST_MODE/user_in");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_USER_PATT1_MSB,
				   0xFF, true, "USER_PATT1_MSB");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_USER_PATT1_LSB,
				   0xFC, true, "USER_PATT1_LSB");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_USER_PATT2_MSB,
				   0xFF, true, "USER_PATT2_MSB");
	if (ret)
		return ret;

	ret = ada4355_write_verify(st, ADA4355_REG_USER_PATT2_LSB,
				   0xFC, true, "USER_PATT2_LSB");
	if (ret)
		return ret;

	/* Two's complement output format */
	ret = ada4355_write_verify(st, ADA4355_REG_OUTPUT_MODE,
				   ADA4355_TWOSCOMP, true, "OUTPUT_MODE/2scomp");
	if (ret)
		return ret;

	/* Fix sample rate at 125 MSPS */
	ret = ada4355_write_verify(st, ADA4355_REG_RESOLUTION_SAMPLE_RATE,
				   ADA4355_125_RATE, true, "RESOLUTION_SMP_RATE");
	if (ret)
		return ret;
	dev_warn(dev, "  NOTE: 0x100 written without a following TRANSFER (0xFF=0x01), so the "
		      "125 MSPS override is NOT latched yet\n");

	dev_info(dev, "==== transcript end: %u readbacks, %u mismatched, %u read 0xFF ====\n",
		 st->rb_total, st->rb_mismatch, st->rb_ff);

	if (st->rb_total && st->rb_ff == st->rb_total)
		dev_warn(dev, "every readback is 0xFF: SDO never drives the bus. Suspect the "
			      "SDO_1P8V level shifter (A-side VS on 1P8V_LDO), not the ADC\n");
	else if (st->rb_mismatch)
		dev_warn(dev, "%u of %u readbacks disagree: writes are reaching the part "
			      "selectively, so this is not a simple dead-SDO fault\n",
			 st->rb_mismatch, st->rb_total);
	else if (st->rb_total)
		dev_info(dev, "all %u readbacks verified: SPI path to this DUT is healthy\n",
			 st->rb_total);

	/* Optional GPIO: enable ADC output valid (board-specific, may be absent) */
	gpio_vld_en = devm_gpiod_get_optional(dev, "gpio-vld-en", GPIOD_OUT_LOW);
	if (IS_ERR(gpio_vld_en))
		return dev_err_probe(dev, PTR_ERR(gpio_vld_en),
				     "Failed to get gpio-vld-en\n");

	if (gpio_vld_en)
		dev_info(dev, "ada4355_setup: gpio-vld-en present, asserting high\n");
	else
		dev_info(dev, "ada4355_setup: gpio-vld-en absent (VLDEN is hard pulled to "
			      "3P3V_MAIN via R13 on this board, so this is expected)\n");
	gpiod_set_value_cansleep(gpio_vld_en, 1);

	dev_info(dev, "ada4355_setup: complete\n");
	return 0;
}

static int ada4355_properties_parse(struct ada4355_state *st)
{
	struct spi_device *spi = st->spi;
	unsigned int val;
	int ret;

	st->clk = devm_clk_get(&spi->dev, "adc_clk");
	if (IS_ERR(st->clk))
		return dev_err_probe(&spi->dev, PTR_ERR(st->clk),
				     "Failed to get adc_clk\n");

	ret = of_property_read_u32(spi->dev.of_node, "num_lanes", &val);
	st->num_lanes = ret ? 1 : val;

	return 0;
}

static int ada4355_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct axiadc_converter *conv;
	struct ada4355_state *st;
	struct regmap *regmap;
	int ret;

	dev_info(&spi->dev, "ada4355_probe: enter, DT node %pOF\n", spi->dev.of_node);
	dev_info(&spi->dev, "ada4355_probe: SPI bus %d, cs %d, mode 0x%x, %u bits, max_speed %u Hz\n",
		 spi->controller->bus_num, spi_get_chipselect(spi, 0),
		 spi->mode, spi->bits_per_word, spi->max_speed_hz);

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ada4355_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "ada4355_probe: regmap init failed %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	st = iio_priv(indio_dev);
	st->regmap = regmap;
	st->spi = spi;
	mutex_init(&st->lock);

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	/* cf_axi_adc core discovers the converter via spi drvdata */
	spi_set_drvdata(spi, conv);

	ret = ada4355_properties_parse(st);
	if (ret) {
		dev_err(&spi->dev, "ada4355_probe: properties_parse failed %d\n", ret);
		return ret;
	}

	dev_info(&spi->dev, "ada4355_probe: num_lanes=%u, adc_clk=%lu Hz\n",
		 st->num_lanes, clk_get_rate(st->clk));

	ret = clk_prepare_enable(st->clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ada4355_clk_disable, conv);
	if (ret)
		return ret;

	dev_info(&spi->dev, "ada4355_probe: calling ada4355_setup\n");
	ret = ada4355_setup(st);
	if (ret) {
		dev_err(&spi->dev, "ada4355_probe: setup failed %d\n", ret);
		return ret;
	}

	conv->spi        = st->spi;
	conv->clk        = st->clk;
	conv->chip_info  = &ada4355_chip_info;
	conv->reg_access = ada4355_reg_access;
	conv->read_raw   = ada4355_read_raw;
	conv->write_raw  = ada4355_write_raw;
	conv->post_setup = ada4355_post_setup;
	conv->phy        = st;

	return 0;
}

static const struct spi_device_id ada4355_id[] = {
	{ "ada4355", 0 },
	{ "ada4356", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ada4355_id);

static const struct of_device_id ada4355_of_match[] = {
	{ .compatible = "adi,ada4355" },
	{ .compatible = "adi,ada4356" },
	{}
};
MODULE_DEVICE_TABLE(of, ada4355_of_match);

static struct spi_driver ada4355_driver = {
	.driver = {
		.name = "ada4355",
		.of_match_table = ada4355_of_match,
	},
	.probe    = ada4355_probe,
	.id_table = ada4355_id,
};
module_spi_driver(ada4355_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_AUTHOR("Pop Ioan Daniel <pop.Ioan-daniel@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADA4355/ADA4356 current-input ADC");
MODULE_LICENSE("GPL");
