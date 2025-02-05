// SPDX-License-Identifier: GPL-2.0+
/*
 * AD7768 Analog to digital converters driver
 *
 * Copyright 2018 Analog Devices Inc.
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

/* AD7768 registers definition */
#define HMCAD15XX_RST                0x00
#define HMCAD15XX_PD                 0x0F
#define HMCAD15XX_EN_RAMP            0x25
#define HMCAD15XX_OPERATION_MODE     0x31
#define HMCAD15XX_INP_SEL_ADC1_ADC2  0x3A
#define HMCAD15XX_INP_SEL_ADC3_ADC4  0x3B
#define HMCAD15XX_PHASE_DDR          0x42
#define HMCAD15XX_BTC_MODE           0x46
#define HMCAD15XX_LVDS_PD_MODE       0x52
#define HMCAD15XX_LVDS_OUTPUT_MODE   0x53
#define HMCAD15XX_FS_CNTRL           0x55
#define HMCAD15XX_STARTUP_CTRL       0x56


#define HMCAD15XX_OUTPUT_MODE_RESOLUTION_MSK		GENMASK(2, 0)

/*HMCAD15XX_OPERATION_MODE*/
#define HMCAD15XX_OPERATION_MODE_PRECISION_MSK		BIT(3)
#define HMCAD15XX_OPERATION_MODE_HIGH_SPEED_MSK		GENMASK(2, 0)
#define HMCAD15XX_OPERATION_MODE_CLK_DIVIDE_MSK		GENMASK(9, 8)

/*HMCAD15XX_INP_SEL*/

#define HMCAD15XX_INP_SEL_ADC1_MSK 		   GENMASK(4, 1)
#define HMCAD15XX_INP_SEL_ADC2_MSK 		   GENMASK(12, 9)
#define HMCAD15XX_INP_SEL_ADC3_MSK 		   GENMASK(4, 1)
#define HMCAD15XX_INP_SEL_ADC4_MSK 		   GENMASK(12, 9)

/*HMCAD15XX_PD*/
#define HMCAD15XX_PD_MSK                    BIT(9)

#define HMCAD15XX__OUTPUT_MODE_OFFSET_BINARY	0x00


struct hmcad15xx_state {
	struct spi_device *spi;
	struct clk *clk;
	struct mutex lock;
	unsigned int sampling_freq;
	unsigned long		regs_hw[44];
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
		/* AXI ADC*/
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int hmcad15xx_spi_reg_read(struct hmcad15xx_state *st, unsigned int addr,
			       unsigned int *val)
{
	return st->regs_hw[addr];
}

static int hmcad15xx_spi_reg_write(struct hmcad15xx_state *st,
				unsigned int addr,
				unsigned int val)
{
	st->d32 = cpu_to_be32(((addr & 0xFF) << 16) | ((val & 0xFFFF) << 4));
    st->regs_hw[addr] = val;
	return spi_write(st->spi, &st->d32, sizeof(st->d32));
}

static int hmcad15xx_spi_write_mask(struct hmcad15xx_state *st,
				 unsigned int addr,
				 unsigned long int mask,
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

static int hmcad15xx_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
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

static int hmcad15xx_set_sampling_freq(struct iio_dev *dev,
				    unsigned int freq)
{
	struct hmcad15xx_state *st = hmcad15xx_get_data(dev);
	int ret = 0;

	if (!freq)
		return -EINVAL;

	mutex_lock(&st->lock);

	st->sampling_freq = freq;

	mutex_unlock(&st->lock);

	return ret;
}



static ssize_t hmcad15xx_scale(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	const struct iio_chan_spec *chan = &indio_dev->channels[0];

	u64 scale;

	scale = div64_ul(1000000,BIT(chan->scan_type.realbits));

	return scnprintf(buf, PAGE_SIZE, "0.%012llu\n", scale);
}

static IIO_DEVICE_ATTR(in_voltage_scale, 0644, hmcad15xx_scale, NULL, 0);

static int hmcad15xx_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct hmcad15xx_state *st = hmcad15xx_get_data(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(st->clk);
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
		return hmcad15xx_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static struct iio_chan_spec_ext_info hmcad15xx_ext_info[] = {
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

#define HMCAD15XX_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.address = index,					\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.ext_info = hmcad15xx_ext_info,				\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 8,					\
			.storagebits = 8,				\
		},							\
	}

static const struct axiadc_chip_info hmcad15xx_conv_chip_info = {
	.id = 0,
	.name = "hmcad15xx_axi_adc",
	.num_channels = 1,
	.channel[0] = HMCAD15XX_CHAN(0),
};


static const unsigned long hmcad15xx_available_scan_masks[]  = { 0xFF, 0x00 };

static int hmcad15xx_register_axi_adc(struct hmcad15xx_state *st)
{
	struct axiadc_converter	*conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->chip_info = st->chip_info;
	conv->adc_output_mode = HMCAD15XX__OUTPUT_MODE_OFFSET_BINARY;
	conv->reg_access = &hmcad15xx_reg_access;
	conv->write_raw = &hmcad15xx_write_raw;
	conv->read_raw = &hmcad15xx_read_raw;
	conv->attrs = &hmcad15xx_group;
	conv->phy = st;
	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	return 0;
}

static int hmcad15xx_probe(struct spi_device *spi)
{
	struct gpio_desc *gpio_reset;
    struct gpio_desc *gpio_pd;
	struct hmcad15xx_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return -ENODEV;

	st->clk = devm_clk_get_enabled(&spi->dev, "clk");
		if (IS_ERR(st->clk))
			return PTR_ERR(st->clk);


	st->spi = spi;


    	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		fsleep(2);
		gpiod_set_value_cansleep(gpio_reset, 0);
		fsleep(1660);
	}

    gpio_pd = devm_gpiod_get_optional(&spi->dev, "pd",
    				     GPIOD_OUT_HIGH);
    if (IS_ERR(gpio_pd))
    	return PTR_ERR(gpio_pd);






	 ret =  hmcad15xx_spi_write_mask(st, HMCAD15XX_LVDS_OUTPUT_MODE,
	 			                 HMCAD15XX_OUTPUT_MODE_RESOLUTION_MSK,
	 			                 0X0);
	ret =  hmcad15xx_spi_write_mask(st, HMCAD15XX_OPERATION_MODE,
			                 HMCAD15XX_OPERATION_MODE_HIGH_SPEED_MSK,
			                 0x0001);
 	if (gpio_pd) {
 		fsleep(2);
 		gpiod_set_value_cansleep(gpio_pd, 0);
 		fsleep(1660);
 	}

	ret =  hmcad15xx_spi_write_mask(st, HMCAD15XX_INP_SEL_ADC1_ADC2,
			                HMCAD15XX_INP_SEL_ADC1_MSK,
			                0x01);
	ret =  hmcad15xx_spi_write_mask(st, HMCAD15XX_INP_SEL_ADC1_ADC2,
		                 	HMCAD15XX_INP_SEL_ADC2_MSK,
		                 	0x01);
	ret =  hmcad15xx_spi_write_mask(st, HMCAD15XX_INP_SEL_ADC3_ADC4,
			                 HMCAD15XX_INP_SEL_ADC3_MSK,
			                 0x0001);
	ret =  hmcad15xx_spi_write_mask(st, HMCAD15XX_INP_SEL_ADC3_ADC4,
		                 	HMCAD15XX_INP_SEL_ADC4_MSK,
		                 	0x0001);

	mutex_init(&st->lock);


		ret = hmcad15xx_register_axi_adc(st);

	return ret;
}

static const struct spi_device_id hmcad15xx_id[] = {
	{"hmcad15xx", (kernel_ulong_t)&hmcad15xx_conv_chip_info},
	{},
};
MODULE_DEVICE_TABLE(spi, hmcad15xx_id);

static const struct of_device_id hmcad15xx_of_match[]  = {
	{ .compatible = "adi,hmcad15xx", .data = &hmcad15xx_conv_chip_info },
	{},
};
MODULE_DEVICE_TABLE(of, hmcad15xx_of_match);

static struct spi_driver hmcad15xx_driver = {
	.driver = {
		.name	= "hmcad15xx",
		.of_match_table = hmcad15xx_of_match,
	},
	.probe		= hmcad15xx_probe,
	.id_table	= hmcad15xx_id,
};
module_spi_driver(hmcad15xx_driver);

MODULE_AUTHOR("Paul Pop <paul.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices HMCAD15XX ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);