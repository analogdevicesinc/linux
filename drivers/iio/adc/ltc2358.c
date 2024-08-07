// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LTC235x ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 * Authors:
 *	Kim Seer Paller <kimseer.paller@analog.com>
 *	John Erasmus Mari Geronimo <johnerasmusmari.geronimo@analog.com>
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#include "cf_axi_adc.h"

#define LTC235X_MAX_SOFTSPAN		7
#define LTC235X_TCNVH_NS		40
#define LTC235X_MIN_SAMP_FREQ		100000

enum ltc235x_supported_device_ids {
	ID_LTC2353_16,
	ID_LTC2353_18,
	ID_LTC2357_16,
	ID_LTC2357_18,
	ID_LTC2358_16,
	ID_LTC2358_18,
};

struct ltc235x_state {
	struct spi_device		*spi;
	struct clk			*clkin;
	struct pwm_device		*cnv_pwm;
	unsigned int			softspan[8];
	int				vref_mv;
	int				num_channels;
};

static struct ltc235x_state *ltc235x_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	conv = iio_device_get_drvdata(indio_dev);

	return conv->phy;
}

static int __ltc235x_set_sampling_freq(struct ltc235x_state *st,
					unsigned int freq)
{
	struct pwm_state cnv_state;

	pwm_get_state(st->cnv_pwm, &cnv_state);
	cnv_state.duty_cycle = LTC235X_TCNVH_NS;
	cnv_state.period = DIV_ROUND_CLOSEST_ULL(NANO, freq);

	return pwm_apply_state(st->cnv_pwm, &cnv_state);
}

static int ltc235x_set_sampling_freq(struct iio_dev *indio_dev,
					unsigned int freq)
{
	struct ltc235x_state *st = ltc235x_get_data(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct device *dev = &st->spi->dev;
	int ret, num_chan, max_freq;

	num_chan = conv->chip_info->num_channels;

	/*
	 * Determine the maximum sampling frequency based on the
	 * number of channels.
	 */
	switch (num_chan) {
	case 2:
		max_freq = 550000;
		break;
	case 4:
		max_freq = 350000;
		break;
	case 8:
		max_freq = 200000;
		break;
	default:
		return -EINVAL;
	}

	if (freq < LTC235X_MIN_SAMP_FREQ || freq > max_freq) {
		dev_err(dev, "Sampling frequency out of range\n");
		return -EINVAL;
	}

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = __ltc235x_set_sampling_freq(st, freq);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static void ltc235x_get_sampling_freq(struct iio_dev *indio_dev,
				      unsigned int *freq)
{
	struct ltc235x_state *st = ltc235x_get_data(indio_dev);
	struct pwm_state cnv_state;

	pwm_get_state(st->cnv_pwm, &cnv_state);
	*freq = DIV_ROUND_CLOSEST_ULL(NANO, cnv_state.period);
}

static int ltc235x_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct axiadc_state *axiadc_st = iio_priv(indio_dev);
	struct ltc235x_state *st = ltc235x_get_data(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		*val = axiadc_read(axiadc_st, ADI_REG_CHAN_RAW_DATA(chan->channel));

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv * 2;
		*val2 = conv->chip_info->resolution;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ltc235x_get_sampling_freq(indio_dev, val);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ltc235x_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ltc235x_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ltc235x_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				unsigned int writeval, unsigned int *readval)
{
	struct axiadc_state *axiadc_st = iio_priv(indio_dev);

	if (readval)
		*readval = axiadc_read(axiadc_st, reg);
	else
		axiadc_write(axiadc_st, reg, writeval);

	return 0;
}

#define LTC235X_CHAN(_chan, _idx) {					\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _chan,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _idx,						\
	.scan_type = {							\
		.sign = 'u',						\
		.realbits = 24,						\
		.storagebits = 32,					\
	},								\
}

static const struct axiadc_chip_info ltc235x_chip_info[] = {
	[ID_LTC2353_16] = {
		.name = "ltc2353-16",
		.resolution = 16,
		.num_channels = 2,
		.channel[0] = LTC235X_CHAN(0, 0),
		.channel[1] = LTC235X_CHAN(1, 1),
	},
	[ID_LTC2353_18] = {
		.name = "ltc2353-18",
		.resolution = 18,
		.num_channels = 2,
		.channel[0] = LTC235X_CHAN(0, 0),
		.channel[1] = LTC235X_CHAN(1, 1),
	},
	[ID_LTC2357_16] = {
		.name = "ltc2357-16",
		.resolution = 16,
		.num_channels = 4,
		.channel[0] = LTC235X_CHAN(0, 0),
		.channel[1] = LTC235X_CHAN(1, 1),
		.channel[2] = LTC235X_CHAN(2, 2),
		.channel[3] = LTC235X_CHAN(3, 3),
	},
	[ID_LTC2357_18] = {
		.name = "ltc2357-18",
		.resolution = 18,
		.num_channels = 4,
		.channel[0] = LTC235X_CHAN(0, 0),
		.channel[1] = LTC235X_CHAN(1, 1),
		.channel[2] = LTC235X_CHAN(2, 2),
		.channel[3] = LTC235X_CHAN(3, 3),
	},
	[ID_LTC2358_16] = {
		.name = "ltc2358-16",
		.resolution = 16,
		.num_channels = 8,
		.channel[0] = LTC235X_CHAN(0, 0),
		.channel[1] = LTC235X_CHAN(1, 1),
		.channel[2] = LTC235X_CHAN(2, 2),
		.channel[3] = LTC235X_CHAN(3, 3),
		.channel[4] = LTC235X_CHAN(4, 4),
		.channel[5] = LTC235X_CHAN(5, 5),
		.channel[6] = LTC235X_CHAN(6, 6),
		.channel[7] = LTC235X_CHAN(7, 7),
	},
	[ID_LTC2358_18] = {
		.name = "ltc2358-18",
		.resolution = 18,
		.num_channels = 8,
		.channel[0] = LTC235X_CHAN(0, 0),
		.channel[1] = LTC235X_CHAN(1, 1),
		.channel[2] = LTC235X_CHAN(2, 2),
		.channel[3] = LTC235X_CHAN(3, 3),
		.channel[4] = LTC235X_CHAN(4, 4),
		.channel[5] = LTC235X_CHAN(5, 5),
		.channel[6] = LTC235X_CHAN(6, 6),
		.channel[7] = LTC235X_CHAN(7, 7),
	},
};

static int ltc235x_channel_config(struct ltc235x_state *st)
{
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *child;
	unsigned int reg;
	int ret;

	device_for_each_child_node(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "Missing channel number\n");
		}

		if (reg >= st->num_channels) {
			fwnode_handle_put(child);
			return dev_err_probe(dev, -EINVAL,
					     "Invalid channel number\n");
		}

		st->softspan[reg] = 7;
		ret = fwnode_property_read_u32(child, "adi,softspan-code",
					       &st->softspan[reg]);
		if (!ret) {
			if (st->softspan[reg] > LTC235X_MAX_SOFTSPAN ||
						st->softspan[reg] < 0) {
				fwnode_handle_put(child);
				return dev_err_probe(dev, -EINVAL,
						     "Invalid softspan code\n");
			}
		}
	}

	return 0;
}

static int ltc235x_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axiadc_st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ltc235x_state *st = ltc235x_get_data(indio_dev);
	int i;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		axiadc_write(axiadc_st, ADI_SOFTSPAN(i), st->softspan[i]);
		axiadc_write(axiadc_st, ADI_REG_CHAN_CNTRL(i), ADI_ENABLE);
	}

	return 0;
}

static void ltc235x_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ltc235x_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ltc235x_probe(struct spi_device *spi)
{
	struct ltc235x_state *st;
	struct axiadc_converter *conv;
	struct regulator *vref;
	struct device *dev = &spi->dev;
	unsigned int samp_freq;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	conv = devm_kzalloc(dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	st->spi = spi;

	conv->chip_info = device_get_match_data(dev);
	if (!conv->chip_info)
		return dev_err_probe(dev, -EINVAL, "Failed to get chip info\n");

	st->num_channels = conv->chip_info->num_channels;

	ret = ltc235x_channel_config(st);
	if (ret)
		return ret;

	ret = clk_prepare_enable(st->clkin);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clkin\n");

	ret = devm_add_action_or_reset(dev, ltc235x_clk_disable, st->clkin);
	if (ret)
		return ret;

	vref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(vref)) {
		if (PTR_ERR(vref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(vref),
					     "Failed to get vref regulator\n");

		/* internal reference */
		st->vref_mv = 2048;
	} else {
		ret = regulator_enable(vref);
		if (ret)
			return dev_err_probe(dev, ret,
					"Failed to enable vref regulator\n");

		ret = devm_add_action_or_reset(dev, ltc235x_regulator_disable,
					       vref);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vref);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Failed to get vref\n");

		if (ret < 1250000 || ret > 2200000)
			return dev_err_probe(dev, -EINVAL,
						"Invalid vref voltage\n");

		st->vref_mv = ret * 2 / 1000;
	}

	samp_freq = LTC235X_MIN_SAMP_FREQ;
	st->cnv_pwm = devm_pwm_get(dev, "cnv");
	if (IS_ERR(st->cnv_pwm))
		return dev_err_probe(dev, PTR_ERR(st->cnv_pwm),
					"Failed to get CNV PWM\n");

	ret = __ltc235x_set_sampling_freq(st, samp_freq);
	if (ret)
		return ret;

	conv->spi = st->spi;
	conv->clk = st->clkin;
	conv->read_raw = &ltc235x_read_raw;
	conv->write_raw = &ltc235x_write_raw;
	conv->reg_access = &ltc235x_reg_access;
	conv->post_setup = &ltc235x_post_setup;
	conv->phy = st;
	spi_set_drvdata(st->spi, conv);

	return ret;
}

static const struct spi_device_id ltc235x_id[] = {
	{ "ltc2353-16", (kernel_ulong_t)&ltc235x_chip_info[ID_LTC2353_16] },
	{ "ltc2353-18", (kernel_ulong_t)&ltc235x_chip_info[ID_LTC2353_18] },
	{ "ltc2357-16", (kernel_ulong_t)&ltc235x_chip_info[ID_LTC2357_16] },
	{ "ltc2357-18", (kernel_ulong_t)&ltc235x_chip_info[ID_LTC2357_18] },
	{ "ltc2358-16", (kernel_ulong_t)&ltc235x_chip_info[ID_LTC2358_16] },
	{ "ltc2358-18", (kernel_ulong_t)&ltc235x_chip_info[ID_LTC2358_18] },
	{ },
};
MODULE_DEVICE_TABLE(spi, ltc235x_id);

static const struct of_device_id ltc235x_of_match[] = {
	{ .compatible = "adi,ltc2353-16",
			.data = &ltc235x_chip_info[ID_LTC2353_16] },
	{ .compatible = "adi,ltc2353-18",
			.data = &ltc235x_chip_info[ID_LTC2353_18] },
	{ .compatible = "adi,ltc2357-16",
			.data = &ltc235x_chip_info[ID_LTC2357_16] },
	{ .compatible = "adi,ltc2357-18",
			.data = &ltc235x_chip_info[ID_LTC2357_18] },
	{ .compatible = "adi,ltc2358-16",
			.data = &ltc235x_chip_info[ID_LTC2358_16] },
	{ .compatible = "adi,ltc2358-18",
			.data = &ltc235x_chip_info[ID_LTC2358_18] },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc235x_of_match);

static struct spi_driver ltc235x_driver = {
	.driver = {
		.name = "ltc235x",
		.of_match_table = ltc235x_of_match,
	},
	.probe = ltc235x_probe,
	.id_table = ltc235x_id,
};
module_spi_driver(ltc235x_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@nalog.com>");
MODULE_DESCRIPTION("Analog Devices LTC235X ADC");
MODULE_LICENSE("GPL v2");
