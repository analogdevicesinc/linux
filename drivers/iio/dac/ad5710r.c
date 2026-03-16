// SPDX-License-Identifier: GPL-2.0
/*
 * AD5710R/AD5711R 8-channel 12-/16-bit Configurable IDAC/VDAC
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kstrtox.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/units.h>

#include <dt-bindings/iio/addac/adi,ad74413r.h>

#define AD5710R_INTERFACE_CONFIG_A		0x00
#define AD5710R_OUTPUT_OPERATING_MODE_0		0x20
#define AD5710R_OUTPUT_OPERATING_MODE_1		0x21
#define AD5710R_OUTPUT_CONTROL_0		0x2A
#define AD5710R_REFERENCE_CONTROL_0		0x3C
#define AD5710R_SW_LDAC_TRIG_A			0xE5
#define AD5710R_INPUT_CH(chan)			(2 * (chan) + 0xEB)
#define AD5710R_CHN_VMODE_EN			0xFF

#define AD5710R_SLD_TRIG_A			BIT(7)
#define AD5710R_OUTPUT_CONTROL_RANGE		BIT(2)
#define AD5710R_REFERENCE_CONTROL_SEL		BIT(0)
#define AD5710R_REG_VAL_MASK			GENMASK(15, 0)
#define AD5710R_OP_MODE_CHAN_MSK(chan)		(GENMASK(1, 0) << 2 * (chan))
#define AD5710R_CHN_VMODE_EN_BIT(chan)		BIT(chan)

#define AD5710R_SW_RESET			(BIT(7) | BIT(0))
#define AD5710R_NORMAL_OP                       0
#define AD5710R_INTERNAL_VREF_mV		2500
#define AD5710R_INTERNAL_IREF_mA		50
#define AD5710R_LDAC_PULSE_US			100
#define AD5710R_NUM_CHANNELS			8

struct ad5710r_chip_info {
	const char *name;
	unsigned int resolution;
};

struct ad5710r_state {
	struct regmap *regmap;
	const struct ad5710r_chip_info *chip_info;
	struct iio_chan_spec *iio_channels;
	struct gpio_desc *ldac_gpio;
	int vref_mV;
	/*
	 * DMA (thus cache coherency maintenance) may require the transfer
	 * buffers to live in their own cache lines.
	 */
	__be16 buf __aligned(IIO_DMA_MINALIGN);
};

static ssize_t ad5710r_get_powerdown_mode(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  char *buf)
{
	struct ad5710r_state *st = iio_priv(indio_dev);
	unsigned int mode;
	int ret;

	ret = regmap_read(st->regmap, AD5710R_CHN_VMODE_EN, &mode);
	if (ret)
		return ret;

	if (mode & BIT(chan->channel))
		return sysfs_emit(buf, "15kohm_to_gnd\n");

	return sysfs_emit(buf, "high_z\n");
}

static ssize_t ad5710r_get_dac_powerdown(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct ad5710r_state *st = iio_priv(indio_dev);
	unsigned int mode;
	int ret;

	ret = regmap_read(st->regmap, chan->channel < 4 ?
			  AD5710R_OUTPUT_OPERATING_MODE_0 :
			  AD5710R_OUTPUT_OPERATING_MODE_1, &mode);
	if (ret)
		return ret;

	if (mode & AD5710R_OP_MODE_CHAN_MSK(chan->channel < 4 ? chan->channel :
					    chan->channel - 4))
		return sysfs_emit(buf, "1\n");

	return sysfs_emit(buf, "0\n");
}

static ssize_t ad5710r_set_dac_powerdown(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 const char *buf, size_t len)
{
	struct ad5710r_state *st = iio_priv(indio_dev);
	unsigned int reg, mask, val;
	bool powerdown;
	int ret;

	ret = kstrtobool(buf, &powerdown);
	if (ret)
		return ret;

	reg = chan->channel < 4 ? AD5710R_OUTPUT_OPERATING_MODE_0 :
	      AD5710R_OUTPUT_OPERATING_MODE_1;

	mask = chan->channel < 4 ? AD5710R_OP_MODE_CHAN_MSK(chan->channel) :
	       AD5710R_OP_MODE_CHAN_MSK(chan->channel - 4);

	val = field_prep(mask, powerdown);

	ret = regmap_update_bits(st->regmap, reg, mask, val);
	if (ret)
		return ret;

	return len;
}

static int ad5710r_trigger_hw_ldac(struct gpio_desc *ldac_gpio)
{
	gpiod_set_value_cansleep(ldac_gpio, 1);
	fsleep(AD5710R_LDAC_PULSE_US);
	gpiod_set_value_cansleep(ldac_gpio, 0);

	return 0;
}

static int ad5710r_dac_write(struct ad5710r_state *st, unsigned int chan,
			     unsigned int val)
{
	int ret;

	st->buf = cpu_to_be16(val << (16 - st->chip_info->resolution));

	ret = regmap_bulk_write(st->regmap, AD5710R_INPUT_CH(chan),
				&st->buf, sizeof(st->buf));
	if (ret)
		return ret;

	if (st->ldac_gpio)
		return ad5710r_trigger_hw_ldac(st->ldac_gpio);

	return regmap_set_bits(st->regmap, AD5710R_SW_LDAC_TRIG_A,
			       AD5710R_SLD_TRIG_A);
}

static int ad5710r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ad5710r_state *st = iio_priv(indio_dev);
	unsigned int mode;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read(st->regmap,
				       AD5710R_INPUT_CH(chan->channel),
				       &st->buf, sizeof(st->buf));
		if (ret)
			return ret;

		*val = be16_to_cpu(st->buf) >> (16 - st->chip_info->resolution);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = regmap_read(st->regmap, AD5710R_CHN_VMODE_EN, &mode);
		if (ret)
			return ret;

		if (mode & BIT(chan->channel))
			*val = st->vref_mV;
		else
			*val = AD5710R_INTERNAL_IREF_mA;

		*val2 = st->chip_info->resolution;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad5710r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad5710r_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > ((1 << st->chip_info->resolution) - 1))
			return -EINVAL;

		return ad5710r_dac_write(st, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int ad5710r_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct ad5710r_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_chan_spec_ext_info ad5710r_ext_info[] = {
	{
		.name = "powerdown",
		.shared = IIO_SEPARATE,
		.read = ad5710r_get_dac_powerdown,
		.write = ad5710r_set_dac_powerdown,
	},
	{
		.name = "powerdown_mode",
		.shared = IIO_SEPARATE,
		.read = ad5710r_get_powerdown_mode,
	},
	{ }
};

static const struct ad5710r_chip_info ad5710r_chip = {
	.name = "ad5710r",
	.resolution = 16,
};

static const struct ad5710r_chip_info ad5711r_chip = {
	.name = "ad5711r",
	.resolution = 12,
};

static int ad5710r_parse_channel_cfg(struct ad5710r_state *st, u8 *num_channels)
{
	struct device *dev = regmap_get_device(st->regmap);
	int ret, num_chan;
	u32 reg;

	num_chan = device_get_child_node_count(dev);
	if (!num_chan)
		return dev_err_probe(dev, -ENODEV, "No channels configured\n");

	st->iio_channels = devm_kcalloc(dev, num_chan,
					sizeof(*st->iio_channels),
					GFP_KERNEL);
	if (!st->iio_channels)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		u32 ch_func;
		enum iio_chan_type chan_type;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to read reg property of %pfwP\n", child);

		if (reg >= AD5710R_NUM_CHANNELS)
			return dev_err_probe(dev, -EINVAL,
					     "reg out of range in %pfwP\n", child);

		ret = fwnode_property_read_u32(child, "adi,ch-func", &ch_func);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Missing adi,ch-func property for %pfwP\n", child);

		switch (ch_func) {
		case CH_FUNC_VOLTAGE_OUTPUT:
			ret = regmap_set_bits(st->regmap, AD5710R_CHN_VMODE_EN,
					      AD5710R_CHN_VMODE_EN_BIT(reg));
			if (ret)
				return dev_err_probe(dev, ret,
						"Failed to set voltage output mode for %pfwP\n", child);

			chan_type = IIO_VOLTAGE;
			break;
		case CH_FUNC_CURRENT_OUTPUT:
			chan_type = IIO_CURRENT;
			break;
		default:
			return dev_err_probe(dev, -EINVAL,
					     "Invalid adi,ch-func %u for %pfwP\n",
					     ch_func, child);
		}

		st->iio_channels[reg] = (struct iio_chan_spec) {
			.type = chan_type,
			.channel = reg,
			.indexed = 1,
			.output = 1,
			.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					      BIT(IIO_CHAN_INFO_SCALE),
			.ext_info = ad5710r_ext_info,
		};

		/* Enable the channel in normal operation mode */
		if (reg < 4) {
			ret = regmap_update_bits(st->regmap, AD5710R_OUTPUT_OPERATING_MODE_0,
						 AD5710R_OP_MODE_CHAN_MSK(reg),
						 field_prep(AD5710R_OP_MODE_CHAN_MSK(reg),
						 AD5710R_NORMAL_OP));
			if (ret)
				return dev_err_probe(dev, ret,
						     "Failed to set normal operating mode for %pfwP\n", child);
		} else {
			ret = regmap_update_bits(st->regmap, AD5710R_OUTPUT_OPERATING_MODE_1,
						 AD5710R_OP_MODE_CHAN_MSK(reg - 4),
						 field_prep(AD5710R_OP_MODE_CHAN_MSK(reg - 4),
						 AD5710R_NORMAL_OP));
			if (ret)
				return dev_err_probe(dev, ret,
						"Failed to set normal operating mode for %pfwP\n", child);
		}
	}

	*num_channels = num_chan;

	return 0;
}

static int ad5710r_setup(struct ad5710r_state *st, int external_vref_uV, u8 *num_channels)
{
	struct device *dev = regmap_get_device(st->regmap);
	struct gpio_desc *reset_gpio;
	u8 range_multiplier;
	int ret;

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to get reset GPIO\n");

	if (reset_gpio) {
		/* Perform hardware reset */
		fsleep(1 * USEC_PER_MSEC);
		gpiod_set_value_cansleep(reset_gpio, 0);
	} else {
		/* Perform software reset */
		ret = regmap_update_bits(st->regmap, AD5710R_INTERFACE_CONFIG_A,
					 AD5710R_SW_RESET, AD5710R_SW_RESET);
		if (ret)
			return ret;
	}

	fsleep(10 * USEC_PER_MSEC);

	range_multiplier = 1;
	if (device_property_read_bool(dev, "adi,range-double")) {
		ret = regmap_set_bits(st->regmap, AD5710R_OUTPUT_CONTROL_0,
				      AD5710R_OUTPUT_CONTROL_RANGE);
		if (ret)
			return ret;

		range_multiplier = 2;
	}

	if (external_vref_uV) {
		st->vref_mV = range_multiplier * external_vref_uV / MILLI;
	} else {
		ret = regmap_set_bits(st->regmap, AD5710R_REFERENCE_CONTROL_0,
				      AD5710R_REFERENCE_CONTROL_SEL);
		if (ret)
			return ret;

		st->vref_mV = range_multiplier * AD5710R_INTERNAL_VREF_mV;
	}

	st->ldac_gpio = devm_gpiod_get_optional(dev, "ldac", GPIOD_OUT_LOW);
	if (IS_ERR(st->ldac_gpio))
		return dev_err_probe(dev, PTR_ERR(st->ldac_gpio),
				     "Failed to get ldac GPIO\n");

	ret = ad5710r_parse_channel_cfg(st, num_channels);
	if (ret)
		return ret;

	return 0;
}

static const struct regmap_config ad5710r_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = AD5710R_CHN_VMODE_EN,
};

static const struct iio_info ad5710r_info = {
	.read_raw = ad5710r_read_raw,
	.write_raw = ad5710r_write_raw,
	.debugfs_reg_access = ad5710r_reg_access,
};

static int ad5710r_probe(struct spi_device *spi)
{
	static const char * const regulators[] = { "vdd", "iovdd" };
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5710r_state *st;
	int ret, external_vref_uV;
	u8 num_channels;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regmap = devm_regmap_init_spi(spi, &ad5710r_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap");

	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return -ENODEV;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(regulators),
					     regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	external_vref_uV = devm_regulator_get_enable_read_voltage(dev, "ref");
	if (external_vref_uV < 0 && external_vref_uV != -ENODEV)
		return external_vref_uV;

	if (external_vref_uV == -ENODEV)
		external_vref_uV = 0;

	ret = ad5710r_setup(st, external_vref_uV, &num_channels);
	if (ret)
		return ret;

	indio_dev->name = st->chip_info->name;
	indio_dev->info = &ad5710r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->iio_channels;
	indio_dev->num_channels = num_channels;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad5710r_id[] = {
	{ "ad5710r", (kernel_ulong_t)&ad5710r_chip },
	{ "ad5711r", (kernel_ulong_t)&ad5711r_chip },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5710r_id);

static const struct of_device_id ad5710r_of_match[] = {
	{ .compatible = "adi,ad5710r", .data = &ad5710r_chip },
	{ .compatible = "adi,ad5711r", .data = &ad5711r_chip },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5710r_of_match);

static struct spi_driver ad5710r_driver = {
	.driver = {
		.name = "ad5710r",
		.of_match_table = ad5710r_of_match,
	},
	.probe = ad5710r_probe,
	.id_table = ad5710r_id,
};
module_spi_driver(ad5710r_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5710R/AD5711R DAC Driver");
MODULE_LICENSE("GPL");
