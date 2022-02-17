// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADAQ8092 driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include "cf_axi_adc.h"

/* ADAQ8092 Register Map */
#define ADAQ8092_REG_RESET		0x00
#define ADAQ8092_REG_POWERDOWN		0x01
#define ADAQ8092_REG_TIMING		0x02
#define ADAQ8092_REG_OUTPUT_MODE	0x03
#define ADAQ8092_REG_DATA_FORMAT	0x04

/* ADAQ8092_REG_RESET Bit Definition */
#define ADAQ8092_RESET			BIT(7)

/* ADAQ8092_REG_POWERDOWN Bit Definition */
#define ADAQ8092_POWERDOWN_MODE		GENMASK(1, 0)

/* ADAQ8092_REG_TIMING Bit Definition */
#define ADAQ8092_CLK_INVERT		BIT(3)
#define ADAQ8092_CLK_PHASE		GENMASK(2, 1)
#define ADAQ8092_CLK_DUTYCYCLE		BIT(0)

/* ADAQ8092_REG_OUTPUT_MODE Bit Definition */
#define ADAQ8092_ILVDS			GENMASK(6, 4)
#define ADAQ8092_TERMON			BIT(3)
#define ADAQ8092_OUTOFF			BIT(2)
#define ADAQ8092_OUTMODE		GENMASK(1, 0)

/* ADAQ8092_REG_DATA_FORMAT Bit Definition */
#define ADAQ8092_OUTTEST		GENMASK(5, 3)
#define ADAQ8092_ABP			BIT(2)
#define ADAQ8092_RAND			BIT(1)
#define ADAQ8092_TWOSCOMP		BIT(0)

/* ADAQ8092 Power Down Modes */
enum adaq8092_powerdown_modes {
	ADAQ8092_NORMAL_OP,
	ADAQ8092_CH1_NORMAL_CH2_NAP,
	ADAQ8092_CH1_CH2_NAP,
	ADAQ8092_SLEEP
};

/* ADAQ8092 Output Clock Invert */
enum adaq8092_clk_invert {
	ADAQ8092_CLK_POL_NORMAL,
	ADAQ8092_CLK_POL_INVERTED
};

/* ADAQ8092 Output Clock Phase Delay Bits */
enum adaq8092_clk_phase_delay {
	ADAQ8092_NO_DELAY,
	ADAQ8092_CLKOUT_DELAY_45DEG,
	ADAQ8092_CLKOUT_DELAY_90DEG,
	ADAQ8092_CLKOUT_DELAY_180DEG
};

/*ADAQ8092 Clock Duty Cycle Stabilizer */
enum adaq8092_clk_dutycycle {
	ADAQ8092_CLK_DC_STABILIZER_OFF,
	ADAQ8092_CLK_DC_STABILIZER_ON,
};

/* ADAQ8092 LVDS Output Current */
enum adaq8092_lvds_out_current {
	ADAQ8092_3M5A = 0,
	ADAQ8092_4MA = 1,
	ADAQ8092_4M5A = 2,
	ADAQ8092_3MA = 4,
	ADAQ8092_2M5A = 5,
	ADAQ8092_2M1A = 6,
	ADAQ8092_1M75 = 7
};

/* ADAQ8092 LVDS Internal Termination */
enum adaq8092_internal_term {
	ADAQ8092_TERM_OFF,
	ADAQ8092_TERM_ON
};

/* ADAQ8092 Digital Output */
enum adaq8092_dout_enable {
	ADAQ8092_DOUT_ON,
	ADAQ8092_DOUT_OFF
};

/* ADAQ8092 Digital Output Mode */
enum adaq8092_dout_modes {
	ADAQ8092_FULL_RATE_CMOS,
	ADAQ8092_DOUBLE_RATE_LVDS,
	ADAQ8092_DOUBLE_RATE_CMOS
};

/* ADAQ8092 Digital Test Pattern */
enum adaq8092_out_test_modes {
	ADAQ8092_TEST_OFF,
	ADAQ8092_TEST_ONES,
	ADAQ8092_TEST_ZEROS,
	ADAQ8092_TEST_CHECKERBOARD,
	ADAQ8092_TEST_ALTERNATING
};

/* ADAQ8092 Alternate Bit Polarity Mode */
enum adaq8092_alt_bit_pol {
	ADAQ8092_ALT_BIT_POL_OFF,
	ADAQ8092_ALT_BIT_POL_ON
};

/* ADAQ8092 Data Output Randomizer*/
enum adaq8092_data_rand {
	ADAQ8092_DATA_RAND_OFF,
	ADAQ8092_DATA_RAND_ON
};

enum adaq8092_twoscomp {
	ADAQ8092_OFFSET_BINARY,
	ADAQ8092_TWOS_COMPLEMENT
};

struct adaq8092_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct clk			*clkin;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex			lock;
	struct gpio_desc		*gpio_adc_pd1;
	struct gpio_desc		*gpio_adc_pd2;
	struct gpio_desc		*gpio_en_1p8;
	struct gpio_desc		*gpio_par_ser;
	enum adaq8092_powerdown_modes	pd_mode;
	enum adaq8092_clk_invert	clk_pol_mode;
	enum adaq8092_clk_phase_delay	clk_phase_mode;
	enum adaq8092_clk_dutycycle	clk_dc_mode;
	enum adaq8092_lvds_out_current	lvds_cur_mode;
	enum adaq8092_internal_term	lvds_term_mode;
	enum adaq8092_dout_enable	dout_en;
	enum adaq8092_dout_modes	dout_mode;
	enum adaq8092_out_test_modes	test_mode;
	enum adaq8092_alt_bit_pol	alt_bit_pol_en;
	enum adaq8092_data_rand		data_rand_en;
	enum adaq8092_twoscomp		twos_comp;
};

static const char * const adaq8092_pd_modes[] = {
	[ADAQ8092_NORMAL_OP] = "normal",
	[ADAQ8092_CH1_NORMAL_CH2_NAP] = "ch2_nap",
	[ADAQ8092_CH1_CH2_NAP] = "ch1_ch2_nap",
	[ADAQ8092_SLEEP] = "sleep"
};

static const char * const adaq8092_clk_pol_modes[] = {
	[ADAQ8092_CLK_POL_NORMAL] = "clk_pol_normal",
	[ADAQ8092_CLK_POL_INVERTED] = "clk_pol_inverted"
};

static const char * const adaq8092_clk_phase_modes[] = {
	[ADAQ8092_NO_DELAY] = "clk_phase_no_delay",
	[ADAQ8092_CLKOUT_DELAY_45DEG] = "clk_phase_45deg",
	[ADAQ8092_CLKOUT_DELAY_90DEG] = "clk_phase_90deg",
	[ADAQ8092_CLKOUT_DELAY_180DEG] = "clk_phase_180deg"
};

static const char * const adaq8092_clk_dc_modes[] = {
	[ADAQ8092_CLK_DC_STABILIZER_OFF] = "clk_dc_stabilizer_off",
	[ADAQ8092_CLK_DC_STABILIZER_ON] = "clk_dc_stabilizer_on"
};

static const char * const adaq8092_lvds_cur_modes[] = {
	[ADAQ8092_3M5A] = "lvds_current_3m5A",
	[ADAQ8092_4MA] = "lvds_current_4mA",
	[ADAQ8092_4M5A] = "lvds_current_4m5A",
	[ADAQ8092_3MA] = "lvds_current_3mA",
	[ADAQ8092_2M5A] = "lvds_current_2m5A",
	[ADAQ8092_2M1A] = "lvds_current_3m1A",
	[ADAQ8092_1M75] = "lvds_current_1m75A",
};

static const char * const adaq8092_lvds_term_modes[] = {
	[ADAQ8092_TERM_OFF] = "lvds_internal_termination_off",
	[ADAQ8092_TERM_ON] = "lvds_internal_termination_on"
};

static const char * const adaq8092_dout_en[] = {
	[ADAQ8092_DOUT_ON] = "digital_output_on",
	[ADAQ8092_DOUT_OFF] = "digital_output_off"
};

static const char * const adaq8092_dout_modes[] = {
	[ADAQ8092_FULL_RATE_CMOS] = "full_rate_cmos_output",
	[ADAQ8092_DOUBLE_RATE_LVDS] = "double_data_rate_lvds_output",
	[ADAQ8092_DOUBLE_RATE_CMOS] = "double_data_rate_cmos_output"
};

static const char * const adaq8092_test_modes[] = {
	[ADAQ8092_TEST_OFF] = "test_pattern_off",
	[ADAQ8092_TEST_ONES] = "test_all_digital_zero",
	[ADAQ8092_TEST_ZEROS] = "test_all_digital_one",
	[ADAQ8092_TEST_CHECKERBOARD] = "test_checkerboard",
	[ADAQ8092_TEST_ALTERNATING] = "test_alternating"
};

static const char * const adaq8092_alt_pol_en[] = {
	[ADAQ8092_ALT_BIT_POL_OFF] = "alternate_bit_polarity_off",
	[ADAQ8092_ALT_BIT_POL_ON] = "alternate_bit_polarity_on"
};

static const char * const adaq8092_data_rand_en[] = {
	[ADAQ8092_DATA_RAND_OFF] = "data_randomizer_off",
	[ADAQ8092_DATA_RAND_ON] = "data_randomizer_on"
};

static const char * const adaq8092_twos_comp_mode[] = {
	[ADAQ8092_OFFSET_BINARY] = "offset_binary",
	[ADAQ8092_TWOS_COMPLEMENT] = "twos_complement"
};

static const struct regmap_config adaq8092_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
	.max_register = 0x1A,
};

static struct adaq8092_state *adaq8092_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	conv = iio_device_get_drvdata(indio_dev);

	return conv->phy;
}

static int adaq8092_set_pd_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_POWERDOWN,
				 ADAQ8092_POWERDOWN_MODE,
				 FIELD_PREP(ADAQ8092_POWERDOWN_MODE, mode));
	if (ret)
		return ret;

	st->pd_mode = mode;

	return 0;
}

static int adaq8092_get_pd_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->pd_mode;
}

static int adaq8092_set_clk_pol_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_TIMING,
				 ADAQ8092_CLK_INVERT,
				 FIELD_PREP(ADAQ8092_CLK_INVERT, mode));
	if (ret)
		return ret;

	st->clk_pol_mode = mode;

	return 0;
}

static int adaq8092_get_clk_pol_mode(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->clk_pol_mode;
}

static int adaq8092_set_clk_phase_mode(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_TIMING,
				 ADAQ8092_CLK_PHASE,
				 FIELD_PREP(ADAQ8092_CLK_PHASE, mode));
	if (ret)
		return ret;

	st->clk_phase_mode = mode;

	return 0;
}

static int adaq8092_get_clk_phase_mode(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->clk_phase_mode;
}

static int adaq8092_set_clk_dc_mode(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_TIMING,
				 ADAQ8092_CLK_DUTYCYCLE,
				 FIELD_PREP(ADAQ8092_CLK_DUTYCYCLE, mode));
	if (ret)
		return ret;

	st->clk_dc_mode = mode;

	return 0;
}

static int adaq8092_get_clk_dc_mode(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->clk_dc_mode;
}

static int adaq8092_set_lvds_cur_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_OUTPUT_MODE,
				 ADAQ8092_ILVDS,
				 FIELD_PREP(ADAQ8092_ILVDS, mode));
	if (ret)
		return ret;

	st->lvds_cur_mode = mode;

	return 0;
}

static int adaq8092_get_lvds_cur_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->lvds_cur_mode;
}

static int adaq8092_set_lvds_term_mode(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_OUTPUT_MODE,
				 ADAQ8092_TERMON,
				 FIELD_PREP(ADAQ8092_TERMON, mode));
	if (ret)
		return ret;

	st->lvds_term_mode = mode;

	return 0;
}

static int adaq8092_get_lvds_term_mode(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->lvds_term_mode;
}

static int adaq8092_set_dout_en(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_OUTPUT_MODE,
				 ADAQ8092_OUTOFF,
				 FIELD_PREP(ADAQ8092_OUTOFF, mode));
	if (ret)
		return ret;

	st->dout_en = mode;

	return 0;
}

static int adaq8092_get_dout_en(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->dout_en;
}

static int adaq8092_set_dout_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_OUTPUT_MODE,
				 ADAQ8092_OUTMODE,
				 FIELD_PREP(ADAQ8092_OUTMODE, mode));
	if (ret)
		return ret;

	st->dout_mode = mode;

	return 0;
}

static int adaq8092_get_dout_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->dout_mode;
}

static int adaq8092_set_test_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_DATA_FORMAT,
				 ADAQ8092_OUTTEST,
				 FIELD_PREP(ADAQ8092_OUTTEST, mode));
	if (ret)
		return ret;

	st->test_mode = mode;

	return 0;
}

static int adaq8092_get_test_mode(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->test_mode;
}

static int adaq8092_set_alt_pol_en(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_DATA_FORMAT,
				 ADAQ8092_ABP,
				 FIELD_PREP(ADAQ8092_ABP, mode));
	if (ret)
		return ret;

	st->alt_bit_pol_en = mode;

	return 0;
}

static int adaq8092_get_alt_pol_en(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->alt_bit_pol_en;
}

static int adaq8092_set_data_rand_en(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_DATA_FORMAT,
				 ADAQ8092_RAND,
				 FIELD_PREP(ADAQ8092_RAND, mode));
	if (ret)
		return ret;

	st->data_rand_en = mode;

	return 0;
}

static int adaq8092_get_data_rand_en(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->data_rand_en;
}

static int adaq8092_set_twos_comp(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int mode)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_DATA_FORMAT,
				 ADAQ8092_TWOSCOMP,
				 FIELD_PREP(ADAQ8092_TWOSCOMP, mode));
	if (ret)
		return ret;

	st->twos_comp = mode;

	return 0;
}

static int adaq8092_get_twos_comp(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	return st->twos_comp;
}

static int adaq8092_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int write_val,
			       unsigned int *read_val)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	if (read_val)
		return regmap_read(st->regmap, reg, read_val);
	else
		return regmap_write(st->regmap, reg, write_val);
}

static const struct iio_enum adaq8092_pd_mode_enum = {
	.items = adaq8092_pd_modes,
	.num_items = ARRAY_SIZE(adaq8092_pd_modes),
	.get = adaq8092_get_pd_mode,
	.set = adaq8092_set_pd_mode,
};

static const struct iio_enum adaq8092_clk_pol_mode_enum = {
	.items = adaq8092_clk_pol_modes,
	.num_items = ARRAY_SIZE(adaq8092_clk_pol_modes),
	.get = adaq8092_get_clk_pol_mode,
	.set = adaq8092_set_clk_pol_mode,
};

static const struct iio_enum adaq8092_clk_phase_mode_enum = {
	.items = adaq8092_clk_phase_modes,
	.num_items = ARRAY_SIZE(adaq8092_clk_phase_modes),
	.get = adaq8092_get_clk_phase_mode,
	.set = adaq8092_set_clk_phase_mode,
};

static const struct iio_enum adaq8092_clk_dc_mode_enum = {
	.items = adaq8092_clk_dc_modes,
	.num_items = ARRAY_SIZE(adaq8092_clk_dc_modes),
	.get = adaq8092_get_clk_dc_mode,
	.set = adaq8092_set_clk_dc_mode,
};

static const struct iio_enum adaq8092_lvds_cur_mode_enum = {
	.items = adaq8092_lvds_cur_modes,
	.num_items = ARRAY_SIZE(adaq8092_lvds_cur_modes),
	.get = adaq8092_get_lvds_cur_mode,
	.set = adaq8092_set_lvds_cur_mode,
};

static const struct iio_enum adaq8092_lvds_term_mode_enum = {
	.items = adaq8092_lvds_term_modes,
	.num_items = ARRAY_SIZE(adaq8092_lvds_term_modes),
	.get = adaq8092_get_lvds_term_mode,
	.set = adaq8092_set_lvds_term_mode,
};

static const struct iio_enum adaq8092_dout_en_enum = {
	.items = adaq8092_dout_en,
	.num_items = ARRAY_SIZE(adaq8092_dout_en),
	.get = adaq8092_get_dout_en,
	.set = adaq8092_set_dout_en,
};

static const struct iio_enum adaq8092_dout_mode_enum = {
	.items = adaq8092_dout_modes,
	.num_items = ARRAY_SIZE(adaq8092_dout_modes),
	.get = adaq8092_get_dout_mode,
	.set = adaq8092_set_dout_mode,
};

static const struct iio_enum adaq8092_test_mode_enum = {
	.items = adaq8092_test_modes,
	.num_items = ARRAY_SIZE(adaq8092_test_modes),
	.get = adaq8092_get_test_mode,
	.set = adaq8092_set_test_mode,
};

static const struct iio_enum adaq8092_alt_pol_en_enum = {
	.items = adaq8092_alt_pol_en,
	.num_items = ARRAY_SIZE(adaq8092_alt_pol_en),
	.get = adaq8092_get_alt_pol_en,
	.set = adaq8092_set_alt_pol_en,
};

static const struct iio_enum adaq8092_data_rand_en_enum = {
	.items = adaq8092_data_rand_en,
	.num_items = ARRAY_SIZE(adaq8092_data_rand_en),
	.get = adaq8092_get_data_rand_en,
	.set = adaq8092_set_data_rand_en,
};

static const struct iio_enum adaq8092_twoscomp_enum = {
	.items = adaq8092_twos_comp_mode,
	.num_items = ARRAY_SIZE(adaq8092_twos_comp_mode),
	.get = adaq8092_get_twos_comp,
	.set = adaq8092_set_twos_comp,
};

static const struct iio_chan_spec_ext_info adaq8092_ext_info[] = {
	IIO_ENUM("pd_mode", IIO_SHARED_BY_ALL, &adaq8092_pd_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("pd_mode", IIO_SHARED_BY_ALL, &adaq8092_pd_mode_enum),
	IIO_ENUM("clk_pol_mode", IIO_SHARED_BY_ALL, &adaq8092_clk_pol_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("clk_pol_mode", IIO_SHARED_BY_ALL, &adaq8092_clk_pol_mode_enum),
	IIO_ENUM("clk_phase_mode", IIO_SHARED_BY_ALL, &adaq8092_clk_phase_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("clk_phase_mode", IIO_SHARED_BY_ALL, &adaq8092_clk_phase_mode_enum),
	IIO_ENUM("clk_dc_mode", IIO_SHARED_BY_ALL, &adaq8092_clk_dc_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("clk_dc_mode", IIO_SHARED_BY_ALL, &adaq8092_clk_dc_mode_enum),
	IIO_ENUM("lvds_cur_mode", IIO_SHARED_BY_ALL, &adaq8092_lvds_cur_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("lvds_cur_mode", IIO_SHARED_BY_ALL, &adaq8092_lvds_cur_mode_enum),
	IIO_ENUM("lvds_term_mode", IIO_SHARED_BY_ALL, &adaq8092_lvds_term_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("lvds_term_mode", IIO_SHARED_BY_ALL, &adaq8092_lvds_term_mode_enum),
	IIO_ENUM("dout_en", IIO_SHARED_BY_ALL, &adaq8092_dout_en_enum),
	IIO_ENUM_AVAILABLE_SHARED("dout_en", IIO_SHARED_BY_ALL, &adaq8092_dout_en_enum),
	IIO_ENUM("dout_mode", IIO_SHARED_BY_ALL, &adaq8092_dout_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("dout_mode", IIO_SHARED_BY_ALL, &adaq8092_dout_mode_enum),
	IIO_ENUM("test_mode", IIO_SHARED_BY_ALL, &adaq8092_test_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("test_mode", IIO_SHARED_BY_ALL, &adaq8092_test_mode_enum),
	IIO_ENUM("alt_bit_pol_en", IIO_SHARED_BY_ALL, &adaq8092_alt_pol_en_enum),
	IIO_ENUM_AVAILABLE_SHARED("alt_bit_pol_en", IIO_SHARED_BY_ALL, &adaq8092_alt_pol_en_enum),
	IIO_ENUM("data_rand_en", IIO_SHARED_BY_ALL, &adaq8092_data_rand_en_enum),
	IIO_ENUM_AVAILABLE_SHARED("data_rand_en", IIO_SHARED_BY_ALL, &adaq8092_data_rand_en_enum),
	IIO_ENUM("twos_complement", IIO_SHARED_BY_ALL, &adaq8092_twoscomp_enum),
	IIO_ENUM_AVAILABLE_SHARED("twos_complement", IIO_SHARED_BY_ALL, &adaq8092_twoscomp_enum),
	{ },
};

#define ADAQ8092_CHAN(_channel)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.address = _channel,					\
		.indexed = 1,						\
		.channel = _channel,					\
		.scan_index = _channel,					\
		.ext_info = adaq8092_ext_info,				\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 16,					\
			.storagebits = 16,				\
		},							\
	}

static const struct axiadc_chip_info conv_chip_info = {
	.name = "adaq8092_axi_adc",
	.max_rate = 105000000UL,
	.num_channels = 2,
	.channel[0] = ADAQ8092_CHAN(0),
	.channel[1] = ADAQ8092_CHAN(1),
};

static int adaq8092_properties_parse(struct adaq8092_state *st)
{
	struct spi_device *spi = st->spi;

	st->gpio_adc_pd1 = devm_gpiod_get(&st->spi->dev, "adc-pd1",
					  GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_adc_pd1))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_adc_pd1),
				     "failed to get the PD1 GPIO\n");

	st->gpio_adc_pd2 = devm_gpiod_get(&st->spi->dev, "adc-pd2",
					  GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_adc_pd2))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_adc_pd2),
				     "failed to get the PD2 GPIO\n");

	st->gpio_en_1p8 = devm_gpiod_get(&st->spi->dev, "en-1p8",
					 GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_en_1p8))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_en_1p8),
				     "failed to get the 1p8 GPIO\n");

	st->gpio_par_ser = devm_gpiod_get(&st->spi->dev, "par-ser",
					  GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_par_ser))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_par_ser),
				     "failed to get the Par/Ser GPIO\n");

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return dev_err_probe(&spi->dev, PTR_ERR(st->clkin),
				     "failed to get the input clock\n");

	return 0;
}

static void adaq8092_powerup(struct adaq8092_state *st)
{
	gpiod_set_value(st->gpio_par_ser, 0);
	gpiod_set_value(st->gpio_adc_pd1, 0);
	gpiod_set_value(st->gpio_adc_pd2, 0);
	gpiod_set_value(st->gpio_en_1p8, 0);

	msleep(1000);

	gpiod_set_value(st->gpio_en_1p8, 1);

	msleep(1000);

	gpiod_set_value(st->gpio_adc_pd1, 1);
	gpiod_set_value(st->gpio_adc_pd2, 1);
}

static void adaq8092_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int adaq8092_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	int i;

	for (i = 0; i < conv->chip_info->num_channels; i++)
		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ADI_ENABLE | ADI_FORMAT_ENABLE
			     | ADI_FORMAT_SIGNEXT);

	return 0;
}

static int adaq8092_init(struct adaq8092_state *st)
{
	struct spi_device *spi = st->spi;
	struct axiadc_converter	*conv;
	int ret;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	ret = adaq8092_properties_parse(st);
	if (ret)
		return ret;

	ret = clk_prepare_enable(st->clkin);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adaq8092_clk_disable, st->clkin);
	if (ret)
		return ret;

	conv->spi = st->spi;
	conv->clk = st->clkin;
	conv->chip_info = &conv_chip_info;
	conv->adc_output_mode = ADAQ8092_TWOS_COMPLEMENT;
	conv->reg_access = &adaq8092_reg_access;
	conv->post_setup = &adaq8092_post_setup;
	conv->phy = st;

	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	adaq8092_powerup(st);

	ret = regmap_write(st->regmap, ADAQ8092_REG_RESET,
			   FIELD_PREP(ADAQ8092_RESET, 1));
	if (ret)
		return ret;

	st->dout_mode = ADAQ8092_DOUBLE_RATE_LVDS;

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_OUTPUT_MODE, ADAQ8092_OUTMODE,
				 FIELD_PREP(ADAQ8092_OUTMODE, st->dout_mode));
	if (ret)
		return ret;

	st->twos_comp = ADAQ8092_TWOS_COMPLEMENT;

	return regmap_update_bits(st->regmap, ADAQ8092_REG_DATA_FORMAT, ADAQ8092_TWOSCOMP,
				  FIELD_PREP(ADAQ8092_TWOSCOMP, st->twos_comp));
}

static int adaq8092_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct adaq8092_state *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adaq8092_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st = iio_priv(indio_dev);
	st->regmap = regmap;
	st->spi = spi;

	mutex_init(&st->lock);

	return adaq8092_init(st);
}

static const struct spi_device_id adaq8092_id[] = {
	{ "adaq8092", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, adaq8092_id);

static const struct of_device_id adaq8092_of_match[] = {
	{ .compatible = "adi,adaq8092" },
	{},
};
MODULE_DEVICE_TABLE(of, adaq8092_of_match);

static struct spi_driver adaq8092_driver = {
	.driver = {
		.name = "adaq8092",
		.of_match_table = adaq8092_of_match,
	},
	.probe = adaq8092_probe,
	.id_table = adaq8092_id,
};
module_spi_driver(adaq8092_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com");
MODULE_DESCRIPTION("Analog Devices ADAQ8092");
MODULE_LICENSE("GPL v2");
