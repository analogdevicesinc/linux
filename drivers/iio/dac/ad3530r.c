// SPDX-License-Identifier: GPL-2.0
/*
 * AD3530R/AD3530 8-channel, 16-bit Voltage Output DAC Driver
 * AD3531R/AD3531 4-channel, 16-bit Voltage Output DAC Driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#define AD3530R_INTERFACE_CONFIG_A		0x00
#define AD3530R_OUTPUT_OPERATING_MODE_0		0x20
#define AD3530R_OUTPUT_OPERATING_MODE_1		0x21
#define AD3530R_OUTPUT_CONTROL_0		0x2A
#define AD3530R_REFERENCE_CONTROL_0		0x3C
#define AD3530R_MUX_OUT_SELECT			0x93
#define AD3530R_SW_LDAC_TRIG_A			0xE5
#define AD3530R_INPUT_CH(c)			(2 * (c) + 0xEB)

#define AD3531R_SW_LDAC_TRIG_A			0xDD
#define AD3531R_INPUT_CH(c)			(2 * (c) + 0xE3)

#define AD3530R_SW_LDAC_TRIG_MASK		BIT(7)
#define AD3530R_OUTPUT_CONTROL_MASK		BIT(2)
#define AD3530R_REFERENCE_CONTROL_MASK		BIT(0)
#define AD3530R_REG_VAL_MASK			GENMASK(15, 0)

#define AD3530R_SW_RESET			(BIT(7) | BIT(0))
#define AD3530R_MAX_CHANNELS			8
#define AD3531R_MAX_CHANNELS			4
#define AD3530R_CH(c)				(c)
#define AD3530R_32KOHM_POWERDOWN_MODE		3
#define AD3530R_INTERNAL_VREF_MV		2500
#define AD3530R_LDAC_PULSE_US			100

enum {
	AD3530R_MUXOUT_POWERED_DOWN,
	AD3530R_MUXOUT_VOUT0,
	AD3530R_MUXOUT_IOUT0_SOURCE,
	AD3530R_MUXOUT_IOUT0_SINK,
	AD3530R_MUXOUT_VOUT1,
	AD3530R_MUXOUT_IOUT1_SOURCE,
	AD3530R_MUXOUT_IOUT1_SINK,
	AD3530R_MUXOUT_VOUT2,
	AD3530R_MUXOUT_IOUT2_SOURCE,
	AD3530R_MUXOUT_IOUT2_SINK,
	AD3530R_MUXOUT_VOUT3,
	AD3530R_MUXOUT_IOUT3_SOURCE,
	AD3530R_MUXOUT_IOUT3_SINK,
	AD3530R_MUXOUT_VOUT4,
	AD3530R_MUXOUT_IOUT4_SOURCE,
	AD3530R_MUXOUT_IOUT4_SINK,
	AD3530R_MUXOUT_VOUT5,
	AD3530R_MUXOUT_IOUT5_SOURCE,
	AD3530R_MUXOUT_IOUT5_SINK,
	AD3530R_MUXOUT_VOUT6,
	AD3530R_MUXOUT_IOUT6_SOURCE,
	AD3530R_MUXOUT_IOUT6_SINK,
	AD3530R_MUXOUT_VOUT7,
	AD3530R_MUXOUT_IOUT7_SOURCE,
	AD3530R_MUXOUT_IOUT7_SINK,
	AD3530R_MUXOUT_DIE_TEMP,
	AD3530R_MUXOUT_AGND,
};

struct ad3530r_chan {
	unsigned int powerdown_mode;
	bool powerdown;
};

struct ad3530r_chip_info {
	const char *name;
	const struct iio_chan_spec *channels;
	int (*input_ch_reg)(unsigned int c);
	const int iio_chan;
	unsigned int num_channels;
	unsigned int sw_ldac_trig_reg;
};

struct ad3530r_state {
	struct regmap *regmap;
	/* lock to protect against multiple access to the device and shared data */
	struct mutex lock;
	struct ad3530r_chan chan[AD3530R_MAX_CHANNELS];
	const struct ad3530r_chip_info *chip_info;
	struct gpio_desc *ldac_gpio;
	int vref_mv;
	u8 ldac;
	bool range_multiplier;
};

static int ad3530r_input_ch_reg(unsigned int c)
{
	return AD3530R_INPUT_CH(c);
}

static int ad3531r_input_ch_reg(unsigned int c)
{
	return AD3531R_INPUT_CH(c);
}

static const char * const ad3530r_powerdown_modes[] = {
	"1kohm_to_gnd",
	"7.7kohm_to_gnd",
	"32kohm_to_gnd",
};

static int ad3530r_get_powerdown_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct ad3530r_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return st->chan[chan->channel].powerdown_mode - 1;
}

static int ad3530r_set_powerdown_mode(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int mode)
{
	struct ad3530r_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	st->chan[chan->channel].powerdown_mode = mode + 1;

	return 0;
}

static const struct iio_enum ad3530r_powerdown_mode_enum = {
	.items = ad3530r_powerdown_modes,
	.num_items = ARRAY_SIZE(ad3530r_powerdown_modes),
	.get = ad3530r_get_powerdown_mode,
	.set = ad3530r_set_powerdown_mode,
};

static ssize_t ad3530r_get_dac_powerdown(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 char *buf)
{
	struct ad3530r_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	return sysfs_emit(buf, "%d\n", st->chan[chan->channel].powerdown);
}

static ssize_t ad3530r_set_dac_powerdown(struct iio_dev *indio_dev,
					 uintptr_t private,
					 const struct iio_chan_spec *chan,
					 const char *buf, size_t len)
{
	struct ad3530r_state *st = iio_priv(indio_dev);
	int ret;
	unsigned int mask, val;
	bool powerdown;

	ret = kstrtobool(buf, &powerdown);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	switch (chan->channel) {
	case AD3530R_CH(0) ... AD3530R_CH(AD3531R_MAX_CHANNELS - 1):
		mask = GENMASK(chan->channel * 2 + 1, chan->channel * 2);
		val = (powerdown ? st->chan[chan->channel].powerdown_mode : 0)
		      << (chan->channel * 2);

		ret = regmap_update_bits(st->regmap,
					 AD3530R_OUTPUT_OPERATING_MODE_0,
					 mask, val);
		if (ret)
			return ret;

		st->chan[chan->channel].powerdown = powerdown;
		return len;
	case AD3530R_CH(AD3531R_MAX_CHANNELS) ...
	     AD3530R_CH(AD3530R_MAX_CHANNELS - 1):
		mask = GENMASK((chan->channel - 4) * 2 + 1,
			       (chan->channel - 4) * 2);
		val = (powerdown ? st->chan[chan->channel].powerdown_mode : 0)
		      << ((chan->channel - 4) * 2);

		ret = regmap_update_bits(st->regmap,
					 AD3530R_OUTPUT_OPERATING_MODE_1,
					 mask, val);
		if (ret)
			return ret;

		st->chan[chan->channel].powerdown = powerdown;
		return len;
	default:
		return -EINVAL;
	}
}

static const char * const ad3530r_muxout_select[] = {
	[AD3530R_MUXOUT_POWERED_DOWN] = "powered_down",
	[AD3530R_MUXOUT_VOUT0] = "vout0",
	[AD3530R_MUXOUT_IOUT0_SOURCE] = "iout0_source",
	[AD3530R_MUXOUT_IOUT0_SINK] = "iout0_sink",
	[AD3530R_MUXOUT_VOUT1] = "vout1",
	[AD3530R_MUXOUT_IOUT1_SOURCE] = "iout1_source",
	[AD3530R_MUXOUT_IOUT1_SINK] = "iout1_sink",
	[AD3530R_MUXOUT_VOUT2] = "vout2",
	[AD3530R_MUXOUT_IOUT2_SOURCE] = "iout2_source",
	[AD3530R_MUXOUT_IOUT2_SINK] = "iout2_sink",
	[AD3530R_MUXOUT_VOUT3] = "vout3",
	[AD3530R_MUXOUT_IOUT3_SOURCE] = "iout3_source",
	[AD3530R_MUXOUT_IOUT3_SINK] = "iout3_sink",
	[AD3530R_MUXOUT_VOUT4] = "vout4",
	[AD3530R_MUXOUT_IOUT4_SOURCE] = "iout4_source",
	[AD3530R_MUXOUT_IOUT4_SINK] = "iout4_sink",
	[AD3530R_MUXOUT_VOUT5] = "vout5",
	[AD3530R_MUXOUT_IOUT5_SOURCE] = "iout5_source",
	[AD3530R_MUXOUT_IOUT5_SINK] = "iout5_sink",
	[AD3530R_MUXOUT_VOUT6] = "vout6",
	[AD3530R_MUXOUT_IOUT6_SOURCE] = "iout6_source",
	[AD3530R_MUXOUT_IOUT6_SINK] = "iout6_sink",
	[AD3530R_MUXOUT_VOUT7] = "vout7",
	[AD3530R_MUXOUT_IOUT7_SOURCE] = "iout7_source",
	[AD3530R_MUXOUT_IOUT7_SINK] = "iout7_sink",
	[AD3530R_MUXOUT_DIE_TEMP] = "die_temp",
	[AD3530R_MUXOUT_AGND] = "agnd",
};

static int ad3530r_get_muxout_select(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan)
{
	struct ad3530r_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, AD3530R_MUX_OUT_SELECT, &val);
	if (ret)
		return ret;

	return val;
}

static int ad3530r_set_muxout_select(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int val)
{
	struct ad3530r_state *st = iio_priv(indio_dev);

	return regmap_write(st->regmap, AD3530R_MUX_OUT_SELECT, val);
}

static const struct iio_enum ad3530r_muxout_select_enum = {
	.items = ad3530r_muxout_select,
	.num_items = ARRAY_SIZE(ad3530r_muxout_select),
	.get = ad3530r_get_muxout_select,
	.set = ad3530r_set_muxout_select,
};

static int ad3530r_trigger_hw_ldac(struct gpio_desc *ldac_gpio)
{
	gpiod_set_value_cansleep(ldac_gpio, 0);
	fsleep(AD3530R_LDAC_PULSE_US);
	gpiod_set_value_cansleep(ldac_gpio, 1);

	return 0;
}

static int ad3530r_dac_write(struct ad3530r_state *st, unsigned int chan,
			     unsigned int val)
{
	int ret;
	__be16 reg_val;

	guard(mutex)(&st->lock);
	reg_val = cpu_to_be16(val);

	ret = regmap_bulk_write(st->regmap, st->chip_info->input_ch_reg(chan),
				&reg_val, sizeof(reg_val));
	if (ret)
		return ret;

	if (st->ldac_gpio)
		return ad3530r_trigger_hw_ldac(st->ldac_gpio);

	return regmap_update_bits(st->regmap, st->chip_info->sw_ldac_trig_reg,
				  AD3530R_SW_LDAC_TRIG_MASK,
				  FIELD_PREP(AD3530R_SW_LDAC_TRIG_MASK, 1));
}

static int ad3530r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ad3530r_state *st = iio_priv(indio_dev);
	int ret;
	__be16 reg_val;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read(st->regmap,
				       st->chip_info->input_ch_reg(chan->channel),
				       &reg_val, sizeof(reg_val));
		if (ret)
			return ret;

		*val = FIELD_GET(AD3530R_REG_VAL_MASK, be16_to_cpu(reg_val));

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = 16;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad3530r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad3530r_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > U16_MAX)
			return -EINVAL;

		return ad3530r_dac_write(st, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int ad3530r_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct ad3530r_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

#define AD3530R_CHAN_EXT_INFO(_name, _what, __shared, _read, _write) {	\
	.name = (_name),						\
	.read = (_read),						\
	.write = (_write),						\
	.private = (_what),						\
	.shared = (__shared),						\
}

static const struct iio_chan_spec_ext_info ad3530r_ext_info[] = {
	AD3530R_CHAN_EXT_INFO("powerdown", 0, IIO_SEPARATE,
			      ad3530r_get_dac_powerdown,
			      ad3530r_set_dac_powerdown),
	IIO_ENUM("powerdown_mode", IIO_SEPARATE, &ad3530r_powerdown_mode_enum),
	IIO_ENUM_AVAILABLE("powerdown_mode", IIO_SHARED_BY_TYPE,
			   &ad3530r_powerdown_mode_enum),
	IIO_ENUM("muxout_select", IIO_SHARED_BY_ALL, &ad3530r_muxout_select_enum),
	IIO_ENUM_AVAILABLE("muxout_select", IIO_SHARED_BY_ALL,
			   &ad3530r_muxout_select_enum),
	{ },
};

#define AD3530R_CHAN(_chan) {						\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _chan,						\
	.output = 1,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_SCALE),			\
	.ext_info = ad3530r_ext_info,					\
}

static const struct iio_chan_spec ad3530r_channels[] = {
	AD3530R_CHAN(0),
	AD3530R_CHAN(1),
	AD3530R_CHAN(2),
	AD3530R_CHAN(3),
	AD3530R_CHAN(4),
	AD3530R_CHAN(5),
	AD3530R_CHAN(6),
	AD3530R_CHAN(7),
};

static const struct iio_chan_spec ad3531r_channels[] = {
	AD3530R_CHAN(0),
	AD3530R_CHAN(1),
	AD3530R_CHAN(2),
	AD3530R_CHAN(3),
};

static const struct ad3530r_chip_info ad3530r_chip = {
	.name = "ad3530r",
	.channels = ad3530r_channels,
	.num_channels = ARRAY_SIZE(ad3530r_channels),
	.sw_ldac_trig_reg = AD3530R_SW_LDAC_TRIG_A,
	.input_ch_reg = ad3530r_input_ch_reg,
};

static const struct ad3530r_chip_info ad3531r_chip = {
	.name = "ad3531r",
	.channels = ad3531r_channels,
	.num_channels = ARRAY_SIZE(ad3531r_channels),
	.sw_ldac_trig_reg = AD3531R_SW_LDAC_TRIG_A,
	.input_ch_reg = ad3531r_input_ch_reg,
};

static int ad3530r_setup(struct ad3530r_state *st)
{
	const struct ad3530r_chip_info *chip_info = st->chip_info;
	struct device *dev = regmap_get_device(st->regmap);
	struct gpio_desc *reset_gpio;
	int i, ret;

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to get reset GPIO\n");

	if (reset_gpio) {
		/* Perform hardware reset */
		fsleep(1000);
		gpiod_set_value_cansleep(reset_gpio, 0);
	} else {
		/* Perform software reset */
		ret = regmap_update_bits(st->regmap, AD3530R_INTERFACE_CONFIG_A,
					 AD3530R_SW_RESET, AD3530R_SW_RESET);
		if (ret)
			return ret;
	}

	fsleep(10000);

	/* Set operating mode to normal operation. */
	ret = regmap_write(st->regmap, AD3530R_OUTPUT_OPERATING_MODE_0, 0);
	if (ret)
		return ret;

	if (chip_info->num_channels > AD3531R_MAX_CHANNELS) {
		ret = regmap_write(st->regmap, AD3530R_OUTPUT_OPERATING_MODE_1, 0);
		if (ret)
			return ret;
	}

	for (i = 0; i < chip_info->num_channels; i++)
		st->chan[i].powerdown_mode = AD3530R_32KOHM_POWERDOWN_MODE;

	st->ldac_gpio = devm_gpiod_get_optional(dev, "ldac", GPIOD_OUT_HIGH);
	if (IS_ERR(st->ldac_gpio))
		return dev_err_probe(dev, PTR_ERR(st->ldac_gpio),
				     "Failed to get ldac GPIO\n");

	if (device_property_present(dev, "adi,double-output-range")) {
		st->range_multiplier = true;

		return regmap_update_bits(st->regmap, AD3530R_OUTPUT_CONTROL_0,
					  AD3530R_OUTPUT_CONTROL_MASK,
					  FIELD_PREP(AD3530R_OUTPUT_CONTROL_MASK, 1));
	}

	return 0;
}

static const struct regmap_config ad3530r_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
};

static const struct iio_info ad3530r_info = {
	.read_raw = ad3530r_read_raw,
	.write_raw = ad3530r_write_raw,
	.debugfs_reg_access = &ad3530r_reg_access,
};

static int ad3530r_probe(struct spi_device *spi)
{
	static const char * const regulators[] = { "vdd", "iovdd" };
	const struct ad3530r_chip_info *chip_info;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad3530r_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regmap = devm_regmap_init_spi(spi, &ad3530r_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	chip_info = spi_get_device_match_data(spi);
	if (!chip_info)
		return -ENODEV;

	st->chip_info = chip_info;

	ret = ad3530r_setup(st);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(regulators),
					     regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_regulator_get_enable_read_voltage(dev, "ref");
	if (ret < 0 && ret != -ENODEV)
		return ret;

	if (ret >= 0) {
		st->vref_mv = st->range_multiplier ? 2 * ret / 1000 : ret / 1000;
	} else {
		/* Internal reference. */
		ret = regmap_update_bits(st->regmap, AD3530R_REFERENCE_CONTROL_0,
					 AD3530R_REFERENCE_CONTROL_MASK,
					 FIELD_PREP(AD3530R_REFERENCE_CONTROL_MASK, 1));
		if (ret)
			return ret;

		st->vref_mv = st->range_multiplier ?
			      2 * AD3530R_INTERNAL_VREF_MV :
			      AD3530R_INTERNAL_VREF_MV;
	}

	indio_dev->name = chip_info->name;
	indio_dev->info = &ad3530r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = chip_info->channels;
	indio_dev->num_channels = chip_info->num_channels;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad3530r_id[] = {
	{ "ad3530r", (kernel_ulong_t)&ad3530r_chip },
	{ "ad3531r", (kernel_ulong_t)&ad3531r_chip },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad3530r_id);

static const struct of_device_id ad3530r_of_match[] = {
	{ .compatible = "adi,ad3530r", .data = &ad3530r_chip },
	{ .compatible = "adi,ad3531r", .data = &ad3531r_chip },
	{ }
};
MODULE_DEVICE_TABLE(of, ad3530r_of_match);

static struct spi_driver ad3530r_driver = {
	.driver = {
		.name = "ad3530r",
		.of_match_table = ad3530r_of_match,
	},
	.probe = ad3530r_probe,
	.id_table = ad3530r_id,
};
module_spi_driver(ad3530r_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD3530R and Similar DACs Driver");
MODULE_LICENSE("GPL");
