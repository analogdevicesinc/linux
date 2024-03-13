// SPDX-License-Identifier: GPL-2.0
/*
 * ADL5580 and similar Gain Amplifiers
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/bitfield.h>
#include <dt-bindings/iio/adi,adl5580.h>

/* Register: ADI_SPI_CONFIG */
#define ADL5580_REG_ADI_SPI_CONFIG 0x000
/* Bit Descriptions for ADI_SPI_CONFIG */
#define ADL5580_MSK_SOFTRESET_ BIT(7) /* Soft Reset */
#define ADL5580_MSK_LSB_FIRST_ BIT(6) /* LSB First */
#define ADL5580_MSK_ENDIAN_ BIT(5) /* Endian */
#define ADL5580_MSK_SDOACTIVE_ BIT(4) /* SDO Active */
#define ADL5580_MSK_SDOACTIVE BIT(3) /* SDO Active */
#define ADL5580_MSK_ENDIAN BIT(2) /* Endian */
#define ADL5580_MSK_LSB_FIRST BIT(1) /* LSB First */
#define ADL5580_MSK_SOFTRESET BIT(0) /* Soft Reset */

/* Register: SCRATCHPAD */
#define ADL5580_REG_SCRATCHPAD 0x00A
/* Bit Descriptions for SCRATCHPAD */
#define ADL5580_MSK_SCRATCHPAD GENMASK(7, 0) /* Scratch Pad */

/* Register: GEN_CTL0 */
#define ADL5580_REG_GEN_CTL0 0x100
/* Bit Descriptions for GEN_CTL0 */
#define ADL5580_MSK_PRG_OTRM GENMASK(7, 6) /* Output V CM */
#define ADL5580_PRG_OTRM(x) FIELD_PREP(ADL5580_MSK_PRG_OTRM, x)
#define ADL5580_MSK_MS_OTRM GENMASK(5, 4) /* V CM to internal or external and VCMO pin definition */
#define ADL5580_MS_OTRM(x) FIELD_PREP(ADL5580_MSK_MS_OTRM, x)
#define ADL5580_MSK_PRG_ITRM GENMASK(3, 2) /* Input V CM */
#define ADL5580_PRG_ITRM(x) FIELD_PREP(ADL5580_MSK_PRG_ITRM, x)
#define ADL5580_MSK_MS_ITRM GENMASK(1, 0) /* V CM to internal or external and VCMI pin definition */
#define ADL5580_MS_ITRM(x) FIELD_PREP(ADL5580_MSK_MS_ITRM, x)

/* Register: GEN_CTL1 */
#define ADL5580_REG_GEN_CTL1 0x101
/* Bit Descriptions for GEN_CTL1 */
#define ADL5580_MSK_PRG_CPEAK GENMASK(6, 4) /* C PEAK */
#define ADL5580_PRG_CPEAK(x) FIELD_PREP(ADL5580_MSK_PRG_CPEAK, x)
#define ADL5580_MSK_EN_AMP BIT(1) /* Enable Amplifier Block */
#define ADL5580_MSK_EN_REF BIT(0) /* Enable Reference Block */

/* Register: SPI_CTL */
#define ADL5580_REG_SPI_CTL 0x200
/* Bit Descriptions for SPI_CTL */
#define ADL5580_MSK_SPI_1P8_3P3_CTRL BIT(0) /* SPI Supply Control */

struct adl5580_state {
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	/* lock to protect against multiple access to the device and shared data */
	struct mutex lock;
};

static const struct regmap_config adl5580_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	.max_register = ADL5580_REG_SPI_CTL,
};

static const struct iio_chan_spec adl5580_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_ENABLE) |
					    BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.indexed = 1,
		.channel = 0,
	},
};

static int adl5580_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adl5580_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		*val = 10;
		*val2 = 0; /* No fractional part */
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_ENABLE:
		if (st->enable_gpio) {
			*val = gpiod_get_value(st->enable_gpio);
		} else {
			ret = regmap_read(st->regmap, ADL5580_REG_GEN_CTL1, val);
			if (ret)
				return ret;

			*val = !!(*val & ADL5580_MSK_EN_AMP);
		}
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adl5580_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adl5580_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return -EOPNOTSUPP;
	case IIO_CHAN_INFO_ENABLE:
		if (st->enable_gpio)
			gpiod_set_value(st->enable_gpio, val);
		else
			return regmap_update_bits(st->regmap,
					ADL5580_REG_GEN_CTL1,
					ADL5580_MSK_EN_AMP | ADL5580_MSK_EN_REF,
					val ? ADL5580_MSK_EN_AMP | ADL5580_MSK_EN_REF : 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int adl5580_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adl5580_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval)
		ret = regmap_read(st->regmap, reg, readval);
	else
		ret = regmap_write(st->regmap, reg, writeval);
	mutex_unlock(&st->lock);

	if (ret)
		return ret;

	return 0;
}

static struct iio_info adl5580_info = {
	.read_raw = adl5580_read_raw,
	.write_raw = adl5580_write_raw,
	.debugfs_reg_access = &adl5580_reg_access,
};

static int adl5580_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct gpio_desc *enable_gpio;
	struct regmap *regmap;
	struct adl5580_state *st;
	static const char * const regulator_names[] = {"avdd", "avcc"};
	u32 value, reg_val;
	bool en;
	int i, ret;

	enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(enable_gpio)) {
		dev_err(dev, "Failed to request enable GPIO\n");
		return PTR_ERR(enable_gpio);
	}

	regmap = devm_regmap_init_spi(spi, &adl5580_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return PTR_ERR(regmap);
	}

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct adl5580_state));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = dev;
	indio_dev->info = &adl5580_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adl5580_channels;
	indio_dev->num_channels = ARRAY_SIZE(adl5580_channels);

	st = iio_priv(indio_dev);
	st->regmap = regmap;
	st->enable_gpio = enable_gpio;
	mutex_init(&st->lock);

	for (i = 0; i < ARRAY_SIZE(regulator_names); i++) {
		ret = devm_regulator_get_enable_optional(dev, regulator_names[i]);
		if (ret < 0 && ret != -ENODEV)
			dev_err_probe(dev, ret, "error enabling regulator %s\n",
				      regulator_names[i]);
	}

	/* Soft Reset */
	ret = regmap_write(regmap, ADL5580_REG_ADI_SPI_CONFIG,
			   ADL5580_MSK_SOFTRESET_ | ADL5580_MSK_SOFTRESET);
	if (ret)
		return ret;
	ret = regmap_write(regmap, ADL5580_REG_ADI_SPI_CONFIG, 0);
	if (ret)
		return ret;

	en = device_property_read_bool(dev, "adi,spi-1p8-volt-enable");
	if (!ret)
		regmap_write(regmap, ADL5580_REG_SPI_CTL,
			     en ? 0 : ADL5580_MSK_SPI_1P8_3P3_CTRL);

	ret = regmap_write(regmap, ADL5580_REG_SCRATCHPAD, 0xAD);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADL5580_REG_SCRATCHPAD, &value);
	if (ret)
		return ret;

	if (value != 0xAD)
		dev_warn(dev, "Failed to read back scratchpad 0xAD != 0x%X\n", value);

	/*
	 * Read HW configuration values from devicetree and write to registers.
	 * Avoid read/modify/write SPI operations since there might be some
	 * users which can't handle SPI_3WIRE, so read might fail.
	 */

	ret = device_property_read_u32(dev, "adi,prg-otrm", &value);
	if (ret)
		reg_val = ADL5580_PRG_OTRM(ADL5580_OUT_CM_0P5V);
	else
		reg_val = ADL5580_PRG_OTRM(value);

	ret = device_property_read_u32(dev, "adi,ms-otrm", &value);
	if (ret)
		reg_val |= ADL5580_MS_OTRM(ADL5580_OUT_TERM_MODE3);
	else
		reg_val |= ADL5580_MS_OTRM(value);

	ret = device_property_read_u32(dev, "adi,prg-itrm", &value);
	if (ret)
		reg_val |= ADL5580_PRG_ITRM(ADL5580_IN_CM_1P75V);
	else
		reg_val |= ADL5580_PRG_ITRM(value);

	ret = device_property_read_u32(dev, "adi,ms-itrm", &value);
	if (ret)
		reg_val |= ADL5580_MS_ITRM(ADL5580_IN_TERM_MODE0);
	else
		reg_val |= ADL5580_MS_ITRM(value);

	ret = regmap_write(regmap, ADL5580_REG_GEN_CTL0, reg_val);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "adi,prg-cpeak", &value);
	if (ret)
		reg_val = ADL5580_PRG_CPEAK(3) | ADL5580_MSK_EN_AMP | ADL5580_MSK_EN_REF;
	else
		reg_val = ADL5580_PRG_CPEAK(value) | ADL5580_MSK_EN_AMP | ADL5580_MSK_EN_REF;

	ret = regmap_write(regmap, ADL5580_REG_GEN_CTL1, reg_val);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Failed to register IIO device\n");
		return ret;
	}

	return 0;
}

static const struct spi_device_id adl5580_id[] = {
	{ "adl5580", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, adl5580_id);

static const struct of_device_id adl5580_of_match[] = {
	{ .compatible = "adi,adl5580" },
	{ },
};
MODULE_DEVICE_TABLE(of, adl5580_of_match);

static struct spi_driver adl5580_driver = {
	.driver = {
		.name = "adl5580",
		.of_match_table = adl5580_of_match,
	},
	.probe = adl5580_probe,
	.id_table = adl5580_id,
};
module_spi_driver(adl5580_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("ADL5580 IIO Amplifier Driver");
MODULE_LICENSE("GPL");
