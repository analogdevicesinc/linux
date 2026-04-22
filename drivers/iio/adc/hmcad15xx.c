// SPDX-License-Identifier: GPL-2.0+
/*
 * HMCAD15XX Analog to digital converters driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/units.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/driver.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "cf_axi_adc.h"

/* HMCAD15XX register addresses */
#define HMCAD15XX_REG_RST		0x00
#define HMCAD15XX_REG_PD		0x0F
#define HMCAD15XX_REG_OPERATION_MODE	0x31
#define HMCAD15XX_REG_INP_SEL_ADC1_ADC2	0x3A
#define HMCAD15XX_REG_INP_SEL_ADC3_ADC4	0x3B
#define HMCAD15XX_REG_PHASE_DDR		0x42
#define HMCAD15XX_REG_BTC_MODE		0x46
#define HMCAD15XX_REG_LVDS_OUTPUT_MODE	0x53

/* HMCAD15XX_REG_RST (0x00) */
#define HMCAD15XX_RST_SW_RESET		0x0001

/* HMCAD15XX_REG_PD (0x0F) */
#define HMCAD15XX_PD_MSK		BIT(9)
#define HMCAD15XX_PD_POWER_DOWN		BIT(9)
#define HMCAD15XX_PD_POWER_UP		0

/* HMCAD15XX_REG_PHASE_DDR (0x42) */
#define HMCAD15XX_PHASE_DDR_MSK		GENMASK(15, 0)

/* HMCAD15XX_REG_LVDS_OUTPUT_MODE (0x53) */
#define HMCAD15XX_LVDS_OUTPUT_MODE_MSK		GENMASK(2, 0)
#define HMCAD15XX_LVDS_OUTPUT_8BIT		0x0
#define HMCAD15XX_LVDS_OUTPUT_12BIT		0x1
#define HMCAD15XX_LVDS_OUTPUT_DUAL_8BIT		0x4

/* HMCAD15XX_REG_OPERATION_MODE (0x31) - combined high_speed + clk_divide */
#define HMCAD15XX_OP_MODE_MSK			(GENMASK(9, 8) | GENMASK(2, 0))
#define HMCAD15XX_OP_MODE_SINGLE_CH		0x0001
#define HMCAD15XX_OP_MODE_DUAL_CH		0x0102
#define HMCAD15XX_OP_MODE_QUAD_CH		0x0204

/* HMCAD15XX_REG_INP_SEL */
#define HMCAD15XX_INP_SEL_ADC1_MSK		GENMASK(4, 1)
#define HMCAD15XX_INP_SEL_ADC2_MSK		GENMASK(12, 9)

#define HMCAD15XX_ADC_1_3_GET_SEL(x)	(__builtin_ctz(((x) & HMCAD15XX_INP_SEL_ADC1_MSK) >> 1))
#define HMCAD15XX_ADC_2_4_GET_SEL(x)	(__builtin_ctz(((x) & HMCAD15XX_INP_SEL_ADC2_MSK) >> 9))

#define HMCAD15XX_INP_1_3_SEL(x)	(((1 << (x)) & 0xf) << 1)
#define HMCAD15XX_INP_2_4_SEL(x)	(((1 << (x)) & 0xf) << 9)

/* HMCAD15XX_REG_BTC_MODE (0x46) */
#define HMCAD15XX_BTC_MODE_MSK			BIT(2)
#define HMCAD15XX_BTC_MODE_TWOS_COMPLEMENT	BIT(2)
#define HMCAD15XX_BTC_MODE_OFFSET_BINARY	0

/* Output mode for AXI ADC converter */
#define HMCAD15XX_OUTPUT_MODE_OFFSET_BINARY	0x00

enum hmcad15xx_clk_div {
	CLK_DIV_1 = 0,
	CLK_DIV_2 = 1,
	CLK_DIV_4 = 2,
};

enum hmcad15xx_input_select {
	IP1_IN1,
	IP2_IN2,
	IP3_IN3,
	IP4_IN4
};

enum hmcad15xx_resolution {
	RES_8BIT  = 0,
	RES_12BIT = 2,
	RES_14BIT = 1
};

static const int hmcad15xx_clk_div_value[] = {
	[CLK_DIV_1] = 1,
	[CLK_DIV_2] = 2,
	[CLK_DIV_4] = 4,
};

struct hmcad15xx_state {
	struct spi_device *spi;
	struct clk *clk;
	/* Protects against concurrent register access */
	struct mutex lock;
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_pd;
	unsigned long regs_hw[86];
	enum hmcad15xx_clk_div clk_div;
	unsigned int en_channels;
	unsigned int pol_mask;
	enum hmcad15xx_resolution resolution;
	const struct axiadc_chip_info *chip_info;
	__be32 d32;
};

static bool hmcad15xx_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct hmcad15xx_state *hmcad15xx_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if (hmcad15xx_has_axi_adc(&indio_dev->dev)) {
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int hmcad15xx_spi_reg_read(struct hmcad15xx_state *st, unsigned int addr,
				  unsigned int *val)
{
	*val = st->regs_hw[addr];
	return 0;
}

static int hmcad15xx_spi_reg_write(struct hmcad15xx_state *st,
				   unsigned int addr, unsigned int val)
{
	st->d32 = cpu_to_be32(((addr & 0xFF) << 24) | ((val & 0xFFFF) << 8));
	st->regs_hw[addr] = val;
	return spi_write(st->spi, &st->d32, sizeof(st->d32));
}

static int hmcad15xx_spi_write_mask(struct hmcad15xx_state *st,
				    unsigned int addr, unsigned long mask,
				    unsigned int val)
{
	unsigned int regval;
	int ret;

	ret = hmcad15xx_spi_reg_read(st, addr, &regval);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return hmcad15xx_spi_reg_write(st, addr, regval);
}

static int hmcad15xx_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				unsigned int writeval, unsigned int *readval)
{
	struct hmcad15xx_state *st = hmcad15xx_get_data(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval) {
		ret = hmcad15xx_spi_reg_read(st, reg, readval);
		if (ret < 0)
			goto exit;
		ret = 0;
	} else {
		ret = hmcad15xx_spi_reg_write(st, reg, writeval);
	}
exit:
	mutex_unlock(&st->lock);
	return ret;
}

static int hmcad15xx_set_input_select(struct iio_dev *dev,
				      const struct iio_chan_spec *chan,
				      unsigned int mode)
{
	struct hmcad15xx_state *st = hmcad15xx_get_data(dev);

	switch (chan->channel) {
	case 0:
		return hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC1_ADC2,
						HMCAD15XX_INP_SEL_ADC1_MSK,
						HMCAD15XX_INP_1_3_SEL(mode));
	case 1:
		return hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC1_ADC2,
						HMCAD15XX_INP_SEL_ADC2_MSK,
						HMCAD15XX_INP_2_4_SEL(mode));
	case 2:
		return hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC3_ADC4,
						HMCAD15XX_INP_SEL_ADC1_MSK,
						HMCAD15XX_INP_1_3_SEL(mode));
	case 3:
		return hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC3_ADC4,
						HMCAD15XX_INP_SEL_ADC2_MSK,
						HMCAD15XX_INP_2_4_SEL(mode));
	default:
		return -EINVAL;
	}
}

static int hmcad15xx_get_input_select(struct iio_dev *dev,
				      const struct iio_chan_spec *chan)
{
	struct hmcad15xx_state *st = hmcad15xx_get_data(dev);
	unsigned int regval;
	int ret;

	switch (chan->channel) {
	case 0:
		ret = hmcad15xx_spi_reg_read(st, HMCAD15XX_REG_INP_SEL_ADC1_ADC2, &regval);
		if (ret < 0)
			return ret;
		return HMCAD15XX_ADC_1_3_GET_SEL(regval);
	case 1:
		ret = hmcad15xx_spi_reg_read(st, HMCAD15XX_REG_INP_SEL_ADC1_ADC2, &regval);
		if (ret < 0)
			return ret;
		return HMCAD15XX_ADC_2_4_GET_SEL(regval);
	case 2:
		ret = hmcad15xx_spi_reg_read(st, HMCAD15XX_REG_INP_SEL_ADC3_ADC4, &regval);
		if (ret < 0)
			return ret;
		return HMCAD15XX_ADC_1_3_GET_SEL(regval);
	case 3:
		ret = hmcad15xx_spi_reg_read(st, HMCAD15XX_REG_INP_SEL_ADC3_ADC4, &regval);
		if (ret < 0)
			return ret;
		return HMCAD15XX_ADC_2_4_GET_SEL(regval);
	default:
		return -EINVAL;
	}
}

static const char * const hmcad15xx_input_select_iio_enum[] = {
	[IP1_IN1] = "IP1_IN1",
	[IP2_IN2] = "IP2_IN2",
	[IP3_IN3] = "IP3_IN3",
	[IP4_IN4] = "IP4_IN4"
};

static const struct iio_enum hmcad15xx_input_select_enum = {
	.items = hmcad15xx_input_select_iio_enum,
	.num_items = ARRAY_SIZE(hmcad15xx_input_select_iio_enum),
	.set = hmcad15xx_set_input_select,
	.get = hmcad15xx_get_input_select,
};

static ssize_t hmcad15xx_scale(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	const struct iio_chan_spec *chan = &indio_dev->channels[0];
	u64 scale;

	scale = div64_ul(1000000, BIT(chan->scan_type.realbits));
	return scnprintf(buf, PAGE_SIZE, "0.%012llu\n", scale);
}

static IIO_DEVICE_ATTR(in_voltage_scale, 0444, hmcad15xx_scale, NULL, 0);

static unsigned int hmcad15xx_get_sampling_freq(struct hmcad15xx_state *st)
{
	unsigned long clk_rate = clk_get_rate(st->clk);
	unsigned int clk_div = hmcad15xx_clk_div_value[st->clk_div];
	unsigned int valid_div;

	if (st->resolution == RES_14BIT)
		valid_div = 2;
	else
		valid_div = 1;

	return DIV_ROUND_CLOSEST(clk_rate, clk_div * valid_div);
}

static int hmcad15xx_read_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int *val, int *val2, long info)
{
	struct hmcad15xx_state *st = hmcad15xx_get_data(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = hmcad15xx_get_sampling_freq(st);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int hmcad15xx_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static struct iio_chan_spec_ext_info hmcad15xx_ext_info[] = {
	IIO_ENUM("input_select", IIO_SEPARATE, &hmcad15xx_input_select_enum),
	IIO_ENUM_AVAILABLE("input_select", IIO_SHARED_BY_TYPE,
			   &hmcad15xx_input_select_enum),
	{ },
};

static struct attribute *hmcad15xx_attributes[] = {
	&iio_dev_attr_in_voltage_scale.dev_attr.attr,
	NULL
};

static const struct attribute_group hmcad15xx_group = {
	.attrs = hmcad15xx_attributes,
};

static const struct iio_info hmcad15xx_info = {
	.attrs = &hmcad15xx_group,
	.read_raw = &hmcad15xx_read_raw,
	.write_raw = &hmcad15xx_write_raw,
	.debugfs_reg_access = &hmcad15xx_reg_access,
};

#define HMCAD15XX_CHAN(_idx, _realbits, _storagebits, _shift)	\
	{							\
		.type = IIO_VOLTAGE,				\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.address = _idx,				\
		.indexed = 1,					\
		.channel = _idx,				\
		.scan_index = _idx,				\
		.ext_info = hmcad15xx_ext_info,			\
		.scan_type = {					\
			.sign = 's',				\
			.realbits = _realbits,			\
			.storagebits = _storagebits,		\
			.shift = _shift,			\
		},						\
	}

static const struct axiadc_chip_info hmcad15xx_chip_info_8bit = {
	.id = 0,
	.name = "hmcad15xx_axi_adc",
	.num_channels = 4,
	.channel[0] = HMCAD15XX_CHAN(0, 8, 8, 0),
	.channel[1] = HMCAD15XX_CHAN(1, 8, 8, 0),
	.channel[2] = HMCAD15XX_CHAN(2, 8, 8, 0),
	.channel[3] = HMCAD15XX_CHAN(3, 8, 8, 0),
};

static const struct axiadc_chip_info hmcad15xx_chip_info_12bit = {
	.id = 0,
	.name = "hmcad15xx_axi_adc",
	.num_channels = 4,
	.channel[0] = HMCAD15XX_CHAN(0, 12, 16, 4),
	.channel[1] = HMCAD15XX_CHAN(1, 12, 16, 4),
	.channel[2] = HMCAD15XX_CHAN(2, 12, 16, 4),
	.channel[3] = HMCAD15XX_CHAN(3, 12, 16, 4),
};

static const struct axiadc_chip_info hmcad15xx_chip_info_14bit = {
	.id = 0,
	.name = "hmcad15xx_axi_adc",
	.num_channels = 4,
	.channel[0] = HMCAD15XX_CHAN(0, 14, 16, 2),
	.channel[1] = HMCAD15XX_CHAN(1, 14, 16, 2),
	.channel[2] = HMCAD15XX_CHAN(2, 14, 16, 2),
	.channel[3] = HMCAD15XX_CHAN(3, 14, 16, 2),
};

static const struct axiadc_chip_info *
hmcad15xx_get_chip_info(enum hmcad15xx_resolution resolution)
{
	switch (resolution) {
	case RES_14BIT:
		return &hmcad15xx_chip_info_14bit;
	case RES_12BIT:
		return &hmcad15xx_chip_info_12bit;
	case RES_8BIT:
	default:
		return &hmcad15xx_chip_info_8bit;
	}
}

/* HDL ADI_REG_CNTRL_3 bit fields for HMCAD15xx */
#define HMCAD15XX_HDL_MODE_MSK		GENMASK(4, 2)
#define HMCAD15XX_HDL_MODE(x)		(((x) & 0x7) << 2)
#define HMCAD15XX_HDL_RES_MSK		GENMASK(1, 0)

/* HDL ADI_REG_ADC_CONFIG_WR bit fields */
#define HMCAD15XX_HDL_POL_MSK		GENMASK(7, 0)

static int hmcad15xx_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axiadc_st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct hmcad15xx_state *st = conv->phy;
	unsigned int hdl_mode;
	unsigned int reg;

	switch (st->en_channels) {
	case 4:
		hdl_mode = 4;
		break;
	case 2:
		hdl_mode = 2;
		break;
	case 1:
	default:
		hdl_mode = 1;
		break;
	}

	reg = axiadc_read(axiadc_st, ADI_REG_CNTRL_3);
	reg &= ~(HMCAD15XX_HDL_MODE_MSK | HMCAD15XX_HDL_RES_MSK);
	reg |= HMCAD15XX_HDL_MODE(hdl_mode) | st->resolution;
	axiadc_write(axiadc_st, ADI_REG_CNTRL_3, reg);

	reg = axiadc_read(axiadc_st, ADI_REG_ADC_CONFIG_WR);
	reg &= ~HMCAD15XX_HDL_POL_MSK;
	reg |= st->pol_mask & HMCAD15XX_HDL_POL_MSK;
	axiadc_write(axiadc_st, ADI_REG_ADC_CONFIG_WR, reg);

	return 0;
}

static int hmcad15xx_register_axi_adc(struct hmcad15xx_state *st)
{
	struct axiadc_converter *conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->chip_info = st->chip_info;
	conv->adc_output_mode = HMCAD15XX_OUTPUT_MODE_OFFSET_BINARY;
	conv->reg_access = &hmcad15xx_reg_access;
	conv->write_raw = &hmcad15xx_write_raw;
	conv->read_raw = &hmcad15xx_read_raw;
	conv->post_setup = &hmcad15xx_post_setup;
	conv->attrs = &hmcad15xx_group;
	conv->phy = st;
	spi_set_drvdata(st->spi, conv);

	return 0;
}

static int hmcad15xx_probe(struct spi_device *spi)
{
	struct hmcad15xx_state *st;
	struct iio_dev *indio_dev;
	unsigned int resolution_bits;
	unsigned int regval;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	memset(st->regs_hw, 0x00, sizeof(st->regs_hw));

	st->clk = devm_clk_get_enabled(&spi->dev, "clk");
	if (IS_ERR(st->clk))
		return PTR_ERR(st->clk);

	st->spi = spi;

	st->en_channels = 1;
	ret = device_property_read_u32(&spi->dev, "adi,num-channels",
				       &st->en_channels);
	if (ret < 0 && ret != -EINVAL)
		return ret;

	resolution_bits = 8;
	ret = device_property_read_u32(&spi->dev, "adi,resolution",
				       &resolution_bits);
	if (ret < 0 && ret != -EINVAL)
		return ret;

	switch (resolution_bits) {
	case 8:
		st->resolution = RES_8BIT;
		break;
	case 12:
		st->resolution = RES_12BIT;
		break;
	case 14:
		st->resolution = RES_14BIT;
		break;
	default:
		dev_err(&spi->dev, "Invalid resolution %u (must be 8, 12, or 14)\n",
			resolution_bits);
		return -EINVAL;
	}

	st->chip_info = hmcad15xx_get_chip_info(st->resolution);

	ret = device_property_read_u32(&spi->dev, "adi,pol-mask", &st->pol_mask);
	if (ret < 0 && ret != -EINVAL)
		return ret;

	st->gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	st->gpio_pd = devm_gpiod_get_optional(&spi->dev, "pd", GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_pd))
		return PTR_ERR(st->gpio_pd);

	if (st->gpio_reset) {
		gpiod_set_value_cansleep(st->gpio_reset, 1);
		fsleep(100);
		gpiod_set_value_cansleep(st->gpio_reset, 0);
		fsleep(100);
	} else {
		ret = hmcad15xx_spi_reg_write(st, HMCAD15XX_REG_RST,
					      HMCAD15XX_RST_SW_RESET);
		if (ret < 0)
			return ret;
		fsleep(100);
	}

	if (st->gpio_pd) {
		gpiod_set_value_cansleep(st->gpio_pd, 1);
		fsleep(1660);
	} else {
		ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_PD,
					       HMCAD15XX_PD_MSK,
					       HMCAD15XX_PD_POWER_DOWN);
		if (ret < 0)
			return ret;
		fsleep(100);
	}

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_PHASE_DDR,
				       HMCAD15XX_PHASE_DDR_MSK, 0x00);
	if (ret < 0)
		return ret;

	switch (st->resolution) {
	case RES_14BIT:
		ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_LVDS_OUTPUT_MODE,
					       HMCAD15XX_LVDS_OUTPUT_MODE_MSK,
					       HMCAD15XX_LVDS_OUTPUT_DUAL_8BIT);
		break;
	case RES_12BIT:
		ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_LVDS_OUTPUT_MODE,
					       HMCAD15XX_LVDS_OUTPUT_MODE_MSK,
					       HMCAD15XX_LVDS_OUTPUT_12BIT);
		break;
	case RES_8BIT:
	default:
		ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_LVDS_OUTPUT_MODE,
					       HMCAD15XX_LVDS_OUTPUT_MODE_MSK,
					       HMCAD15XX_LVDS_OUTPUT_8BIT);
		break;
	}
	if (ret < 0)
		return ret;

	switch (st->en_channels) {
	case 1:
		regval = HMCAD15XX_OP_MODE_SINGLE_CH;
		st->clk_div = CLK_DIV_1;
		break;
	case 2:
		regval = HMCAD15XX_OP_MODE_DUAL_CH;
		st->clk_div = CLK_DIV_2;
		break;
	case 4:
	default:
		regval = HMCAD15XX_OP_MODE_QUAD_CH;
		st->clk_div = CLK_DIV_4;
		break;
	}

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_OPERATION_MODE,
				       HMCAD15XX_OP_MODE_MSK, regval);
	if (ret < 0)
		return ret;

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC1_ADC2,
				       HMCAD15XX_INP_SEL_ADC1_MSK,
				       HMCAD15XX_INP_1_3_SEL(IP4_IN4));
	if (ret < 0)
		return ret;

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC1_ADC2,
				       HMCAD15XX_INP_SEL_ADC2_MSK,
				       HMCAD15XX_INP_2_4_SEL(IP4_IN4));
	if (ret < 0)
		return ret;

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC3_ADC4,
				       HMCAD15XX_INP_SEL_ADC1_MSK,
				       HMCAD15XX_INP_1_3_SEL(IP4_IN4));
	if (ret < 0)
		return ret;

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_INP_SEL_ADC3_ADC4,
				       HMCAD15XX_INP_SEL_ADC2_MSK,
				       HMCAD15XX_INP_2_4_SEL(IP4_IN4));
	if (ret < 0)
		return ret;

	ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_BTC_MODE,
				       HMCAD15XX_BTC_MODE_MSK,
				       HMCAD15XX_BTC_MODE_TWOS_COMPLEMENT);
	if (ret < 0)
		return ret;

	if (st->gpio_pd) {
		gpiod_set_value_cansleep(st->gpio_pd, 0);
		fsleep(2000);
	} else {
		ret = hmcad15xx_spi_write_mask(st, HMCAD15XX_REG_PD,
					       HMCAD15XX_PD_MSK,
					       HMCAD15XX_PD_POWER_UP);
		if (ret < 0)
			return ret;
		fsleep(2000);
	}

	mutex_init(&st->lock);

	return hmcad15xx_register_axi_adc(st);
}

static const struct spi_device_id hmcad15xx_id[] = {
	{ "hmcad15xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, hmcad15xx_id);

static const struct of_device_id hmcad15xx_of_match[] = {
	{ .compatible = "adi,hmcad15xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, hmcad15xx_of_match);

static struct spi_driver hmcad15xx_driver = {
	.driver = {
		.name = "hmcad15xx",
		.of_match_table = hmcad15xx_of_match,
	},
	.probe = hmcad15xx_probe,
	.id_table = hmcad15xx_id,
};
module_spi_driver(hmcad15xx_driver);

MODULE_AUTHOR("Paul Pop <paul.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices HMCAD15XX ADC");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
