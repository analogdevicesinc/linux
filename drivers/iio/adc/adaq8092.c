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
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
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

/* Micellaneous Definitions */
#define ADAQ8092_SCRATCHPAD_VALUE	0xA
#define ADAQ8092_SCRATCHPAD_MSK		GENMASK(7, 4)
#define ADAQ8092_MAX_SAMPLING_FREQ	105000000
#define ADAQ8092_MIN_SAMPLING_FREQ	1000000

/* ADAQ8092 Digital Output Mode */
enum adaq8092_dout_modes {
	ADAQ8092_FULL_RATE_CMOS,
	ADAQ8092_DOUBLE_RATE_LVDS,
	ADAQ8092_DOUBLE_RATE_CMOS
};

/* ADAQ8092 Twos Complement Mode */
enum adaq8092_twoscomp {
	ADAQ8092_OFFSET_BINARY,
	ADAQ8092_TWOS_COMPLEMENT
};

/*ADAQ8092 Clock Duty Cycle Stabilizer */
enum adaq8092_clk_dutycycle {
	ADAQ8092_CLK_DC_STABILIZER_OFF,
	ADAQ8092_CLK_DC_STABILIZER_ON,
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

struct adaq8092_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct clk			*clkin;
	struct regulator		*avcc;
	struct regulator		*avdd;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex			lock;
	struct gpio_desc		*gpio_adc_pd1;
	struct gpio_desc		*gpio_adc_pd2;
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

static int adaq8092_update_dout_config(struct iio_dev *indio_dev, enum adaq8092_dout_modes mode)
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);
	unsigned int data, sdr_ddr_n;
	int ret;

	mutex_lock(&st->lock);
	switch (mode) {
	case ADAQ8092_FULL_RATE_CMOS:
		sdr_ddr_n = BIT(16);

		ret = regmap_write(st->regmap, ADAQ8092_REG_TIMING,
				   FIELD_PREP(ADAQ8092_CLK_INVERT, ADAQ8092_CLK_POL_NORMAL) |
				   FIELD_PREP(ADAQ8092_CLK_PHASE, ADAQ8092_NO_DELAY) |
				   FIELD_PREP(ADAQ8092_CLK_DUTYCYCLE, ADAQ8092_CLK_DC_STABILIZER_OFF));
		if (ret)
			goto exit;

		break;
	case ADAQ8092_DOUBLE_RATE_CMOS:
		sdr_ddr_n = 0;

		ret = regmap_write(st->regmap, ADAQ8092_REG_TIMING,
				   FIELD_PREP(ADAQ8092_CLK_INVERT, ADAQ8092_CLK_POL_INVERTED) |
				   FIELD_PREP(ADAQ8092_CLK_PHASE, ADAQ8092_CLKOUT_DELAY_45DEG) |
				   FIELD_PREP(ADAQ8092_CLK_DUTYCYCLE, ADAQ8092_CLK_DC_STABILIZER_ON));
		if (ret)
			goto exit;

		break;
	case ADAQ8092_DOUBLE_RATE_LVDS:
		sdr_ddr_n = 0;

		ret = regmap_write(st->regmap, ADAQ8092_REG_TIMING,
				   FIELD_PREP(ADAQ8092_CLK_INVERT, ADAQ8092_CLK_POL_INVERTED) |
				   FIELD_PREP(ADAQ8092_CLK_PHASE, ADAQ8092_NO_DELAY) |
				   FIELD_PREP(ADAQ8092_CLK_DUTYCYCLE, ADAQ8092_CLK_DC_STABILIZER_OFF));
		if (ret)
			goto exit;

		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	data = axiadc_read(axi_adc_st, ADI_REG_CNTRL);
	data &= ~BIT(16);
	data |= sdr_ddr_n;
	axiadc_write(axi_adc_st, ADI_REG_CNTRL, data);

	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_OUTPUT_MODE,
				 ADAQ8092_OUTMODE,
				 FIELD_PREP(ADAQ8092_OUTMODE, mode));

exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int adaq8092_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int write_val,
			       unsigned int *read_val)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	if (read_val)
		return regmap_read(st->regmap, reg, read_val);

	return regmap_write(st->regmap, reg, write_val);
}

#define ADAQ8092_CHAN(_channel)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.address = _channel,					\
		.indexed = 1,						\
		.channel = _channel,					\
		.scan_index = _channel,					\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 14,					\
			.storagebits = 16,				\
		},							\
	}

static const struct axiadc_chip_info conv_chip_info = {
	.name = "adaq8092",
	.max_rate = 105000000UL,
	.num_channels = 2,
	.channel[0] = ADAQ8092_CHAN(0),
	.channel[1] = ADAQ8092_CHAN(1),
};

static int adaq8092_read_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int *val, int *val2, long info)
{
	struct adaq8092_state *st = adaq8092_get_data(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clkin);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adaq8092_properties_parse(struct adaq8092_state *st)
{
	struct spi_device *spi = st->spi;

	st->gpio_adc_pd1 = devm_gpiod_get_optional(&st->spi->dev, "adc-pd1",
						   GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_adc_pd1))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_adc_pd1),
				     "failed to get the PD1 GPIO\n");

	st->gpio_adc_pd2 = devm_gpiod_get_optional(&st->spi->dev, "adc-pd2",
						   GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_adc_pd2))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_adc_pd2),
				     "failed to get the PD2 GPIO\n");

	st->avcc = devm_regulator_get_exclusive(&st->spi->dev, "avcc");
	if (IS_ERR(st->avcc))
		return dev_err_probe(&st->spi->dev, PTR_ERR(st->avcc),
				     "failed to get vcc regulator\n");

	st->avdd = devm_regulator_get_exclusive(&st->spi->dev, "avdd");
	if (IS_ERR(st->avdd))
		return dev_err_probe(&st->spi->dev, PTR_ERR(st->avdd),
				     "failed to get vdd regulator\n");

	st->clkin = devm_clk_get(&spi->dev, "clkin");
	if (IS_ERR(st->clkin))
		return dev_err_probe(&spi->dev, PTR_ERR(st->clkin),
				     "failed to get the input clock\n");

	return 0;
}

static void adaq8092_reg_disable(void *data)
{
	regulator_disable(data);
}

static int adaq8092_powerup(struct adaq8092_state *st)
{
	int ret;

	ret = regulator_enable(st->avcc);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&st->spi->dev, adaq8092_reg_disable,
				       st->avcc);
	if (ret)
		return ret;

	if (!st->gpio_adc_pd1 || !st->gpio_adc_pd2) {
		fsleep(500000);
		return regulator_enable(st->avdd);
	}

	ret = regulator_enable(st->avdd);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&st->spi->dev, adaq8092_reg_disable,
				       st->avdd);
	if (ret)
		return ret;

	fsleep(500000);

	gpiod_set_value_cansleep(st->gpio_adc_pd1, 1);

	gpiod_set_value_cansleep(st->gpio_adc_pd2, 1);

	return 0;
}

static void adaq8092_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int adaq8092_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axi_adc_st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	enum adaq8092_dout_modes mode;
	unsigned int data;
	int i, ret;

	data = axiadc_read(axi_adc_st, ADI_REG_CONFIG);
	data &= ADI_CMOS_OR_LVDS_N;

	if (data)
		mode = ADAQ8092_DOUBLE_RATE_CMOS;
	else
		mode = ADAQ8092_DOUBLE_RATE_LVDS;

	ret = adaq8092_update_dout_config(indio_dev, mode);
	if (ret)
		return ret;

	for (i = 0; i < conv->chip_info->num_channels; i++)
		axiadc_write(axi_adc_st, ADI_REG_CHAN_CNTRL(i), ADI_ENABLE | ADI_FORMAT_ENABLE
			     | ADI_FORMAT_SIGNEXT);

	return 0;
}

static int adaq8092_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct adaq8092_state *st;
	struct axiadc_converter	*conv;
	unsigned int data;
	int ret;

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

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	ret = adaq8092_properties_parse(st);
	if (ret)
		return ret;

	ret = clk_prepare_enable(st->clkin);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adaq8092_clk_disable,
				       st->clkin);
	if (ret)
		return ret;

	ret = adaq8092_powerup(st);
	if (ret)
		return ret;

	/* Check if connection is established */
	ret = regmap_update_bits(st->regmap, ADAQ8092_REG_POWERDOWN, ADAQ8092_SCRATCHPAD_MSK,
				 FIELD_PREP(ADAQ8092_SCRATCHPAD_MSK, ADAQ8092_SCRATCHPAD_VALUE));
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADAQ8092_REG_POWERDOWN, &data);
	if (ret)
		return ret;

	if (FIELD_GET(ADAQ8092_SCRATCHPAD_MSK, data) != ADAQ8092_SCRATCHPAD_VALUE) {
		dev_err(&spi->dev, "Spi communication failed!\n");
		return -EINVAL;
	}

	conv->spi = st->spi;
	conv->clk = st->clkin;
	conv->chip_info = &conv_chip_info;
	conv->adc_output_mode = ADAQ8092_TWOS_COMPLEMENT;
	conv->read_raw = &adaq8092_read_raw;
	conv->reg_access = &adaq8092_reg_access;
	conv->post_setup = &adaq8092_post_setup;
	conv->phy = st;

	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	ret = regmap_write(st->regmap, ADAQ8092_REG_RESET,
			   FIELD_PREP(ADAQ8092_RESET, 1));
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, ADAQ8092_REG_DATA_FORMAT, ADAQ8092_TWOSCOMP,
				  FIELD_PREP(ADAQ8092_TWOSCOMP, ADAQ8092_TWOS_COMPLEMENT));
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
