// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2015-2016 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 */

#include "ad5592r-base.h"

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>

#define AD5592R_GPIO_READBACK_EN	BIT(10)
#define AD5592R_LDAC_READBACK_EN	BIT(6)
#define AD5592R_GEN_CTRL_ADC_BUF_EN	BIT(8)

static int ad5592r_spi_wnop_r16(struct ad5592r_state *st, __be16 *buf)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	struct spi_transfer t = {
			.tx_buf	= &st->spi_msg_nop,
			.rx_buf	= buf,
			.len = 2
		};

	st->spi_msg_nop = 0; /* NOP */

	return spi_sync_transfer(spi, &t, 1);
}

static int ad5592r_write_dac(struct ad5592r_state *st, unsigned int chan, u16 value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);

	st->spi_msg = cpu_to_be16(BIT(15) | (chan << 12) | value);

	return spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
}

static int ad5592r_read_adc(struct ad5592r_state *st, unsigned int chan, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	int ret;

	st->spi_msg = cpu_to_be16((AD5592R_REG_ADC_SEQ << 11) | BIT(chan));

	ret = spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	/*
	 * Invalid data:
	 * See Figure 40. Single-Channel ADC Conversion Sequence
	 */
	ret = ad5592r_spi_wnop_r16(st, &st->spi_msg);
	if (ret)
		return ret;

	ret = ad5592r_spi_wnop_r16(st, &st->spi_msg);
	if (ret)
		return ret;

	*value = be16_to_cpu(st->spi_msg);

	return 0;
}

static int ad5592r_reg_write(struct ad5592r_state *st, u8 reg, u16 value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);

	st->spi_msg = cpu_to_be16((reg << 11) | value);

	return spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
}

static int ad5592r_reg_read(struct ad5592r_state *st, u8 reg, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	int ret;

	st->spi_msg = cpu_to_be16((AD5592R_REG_LDAC << 11) |
				   AD5592R_LDAC_READBACK_EN | (reg << 2));

	ret = spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	ret = ad5592r_spi_wnop_r16(st, &st->spi_msg);
	if (ret)
		return ret;

	*value = be16_to_cpu(st->spi_msg);

	return 0;
}

static int ad5592r_gpio_read(struct ad5592r_state *st, u8 *value)
{
	int ret;

	ret = ad5592r_reg_write(st, AD5592R_REG_GPIO_IN_EN,
				AD5592R_GPIO_READBACK_EN | st->gpio_in);
	if (ret)
		return ret;

	ret = ad5592r_spi_wnop_r16(st, &st->spi_msg);
	if (ret)
		return ret;

	*value = (u8) be16_to_cpu(st->spi_msg);

	return 0;
}

static const struct ad5592r_rw_ops ad5592r_rw_ops = {
	.write_dac = ad5592r_write_dac,
	.read_adc = ad5592r_read_adc,
	.reg_write = ad5592r_reg_write,
	.reg_read = ad5592r_reg_read,
	.gpio_read = ad5592r_gpio_read,
};

static ssize_t adc_buf_en_show(struct device *dev,
			       struct device_attribute *attr,
			       char *buf)
{
	struct iio_dev *indio = dev_get_drvdata(dev);
	struct ad5592r_state *st = iio_priv(indio);
	u16 reg;
	int ret;

	ret = ad5592r_reg_read(st, AD5592R_REG_CTRL, &reg);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", !!(reg & AD5592R_GEN_CTRL_ADC_BUF_EN));
}

static ssize_t adc_buf_en_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct iio_dev *indio = dev_get_drvdata(dev);
	struct ad5592r_state *st = iio_priv(indio);
	unsigned long val;
	u16 reg;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	ret = ad5592r_reg_read(st, AD5592R_REG_CTRL, &reg);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	if (val)
		reg |= AD5592R_GEN_CTRL_ADC_BUF_EN;
	else
		reg &= ~AD5592R_GEN_CTRL_ADC_BUF_EN;

	ret = ad5592r_reg_write(st, AD5592R_REG_CTRL, reg);
	mutex_unlock(&st->lock);
	if (ret)
		return ret;

	return count;
}

static IIO_DEVICE_ATTR(adc_buf_en, 0644, adc_buf_en_show, adc_buf_en_store, 0);

static struct attribute *ad5592r_sysfs_attrs[] = {
	&iio_dev_attr_adc_buf_en.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad5592r_sysfs_group = {
	.attrs = ad5592r_sysfs_attrs,
};

static const struct attribute_group *ad5592r_sysfs_groups[] = {
	&ad5592r_sysfs_group,
	NULL,
};

static int ad5592r_spi_probe(struct spi_device *spi)
{
	int ret;
	const struct spi_device_id *id = spi_get_device_id(spi);

	ret = ad5592r_probe(&spi->dev, id->name, &ad5592r_rw_ops, ad5592r_sysfs_groups);

	if (ret)
		return ret;

	return 0;
}

static void ad5592r_spi_remove(struct spi_device *spi)
{
	struct iio_dev *indio = dev_get_drvdata(&spi->dev);

	device_remove_groups(&indio->dev, ad5592r_sysfs_groups);
	ad5592r_remove(&spi->dev);
}

static const struct spi_device_id ad5592r_spi_ids[] = {
	{ .name = "ad5592r", },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5592r_spi_ids);

static const struct of_device_id ad5592r_of_match[] = {
	{ .compatible = "adi,ad5592r", },
	{},
};
MODULE_DEVICE_TABLE(of, ad5592r_of_match);

static const struct acpi_device_id ad5592r_acpi_match[] = {
	{"ADS5592", },
	{ },
};
MODULE_DEVICE_TABLE(acpi, ad5592r_acpi_match);

static struct spi_driver ad5592r_spi_driver = {
	.driver = {
		.name = "ad5592r",
		.of_match_table = ad5592r_of_match,
		.acpi_match_table = ad5592r_acpi_match,
	},
	.probe = ad5592r_spi_probe,
	.remove = ad5592r_spi_remove,
	.id_table = ad5592r_spi_ids,
};
module_spi_driver(ad5592r_spi_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_AD5592R);
