// SPDX-License-Identifier: GPL
/*
 * AD5940 SPI ADC driver
 *
 * Copyright (C) 2019 Song Qiang <songqiang1304521@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bsearch.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

#define AD5940_CHANNEL_AINP_MSK		GENMASK(5, 0)
#define AD5940_CHANNEL_AINP(x)		FIELD_PREP(AD5940_CHANNEL_AINP_MSK, x)
#define AD5940_CHANNEL_AINN_MSK		GENMASK(12, 8)
#define AD5940_CHANNEL_AINN(x)		FIELD_PREP(AD5940_CHANNEL_AINN_MSK, x)

#define AD5940_CHANNEL_NAME		0

struct ad5940_channel_config {
	u32 ain;
	const char *channel_name;
};

struct ad5940_state {
	struct spi_device *spi;

	u8 n_input;
	u8 p_input;

	int num_channels;
	struct ad5940_channel_config *channel_config;
};

static ssize_t ad5940_read_info(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				char *buf)
{
	struct ad5940_state *st = iio_priv(indio_dev);

	switch ((u32)private) {
	case AD5940_CHANNEL_NAME:
		return sprintf(buf, "%s\n",
			st->channel_config[chan->address].channel_name);
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec_ext_info ad4590_ext_info[] = {
	{
		.name = "name",
		.read = ad5940_read_info,
		.private = AD5940_CHANNEL_NAME,
		.shared = IIO_SEPARATE,
	},
	{ },
};

static const struct iio_chan_spec ad5940_channel_template = {
	.type = IIO_VOLTAGE,
	.differential = 1,
	.indexed = 1,
	.ext_info = ad4590_ext_info,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
};

static int ad5940_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = 1000;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 20;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad5940_info = {
	.read_raw = &ad5940_read_raw,
};

int cmp_u8(const void *a, const void *b)
{
	return (*(u8 *)a - *(u8 *)b);
	}

static int ad5940_check_channel_indexes(struct device *dev, u32 *ain)
{
	const u8 channel_p[] = {
		0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 19,
		20, 22, 23, 24, 25, 26, 31, 33, 35, 36
};
	const u8 channel_n[] = {
		0, 1, 2, 4, 5, 6, 7, 10, 11, 12, 14, 16, 17, 20
	};
	u8 *index;

	index = (u8 *) bsearch(&ain[0], channel_p, ARRAY_SIZE(channel_p),
				sizeof(u8), cmp_u8);
	if (!index) {
		dev_err(dev, "Positive input index not found.\n");
		return -EINVAL;
	}

	index = (u8 *) bsearch(&ain[1], channel_n, ARRAY_SIZE(channel_n),
				sizeof(u8), cmp_u8);
	if (!index) {
		dev_err(dev, "negtive input index not found.\n");
		return -EINVAL;
	}

	return 0;
}

static int ad5940_of_parse_channel_config(struct iio_dev *indio_dev,
					  struct device_node *np)
{
	struct ad5940_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *chan;
	struct device_node *child;
	u32 channel, ain[2];
	int ret;

	st->num_channels = of_get_available_child_count(np);
	if (!st->num_channels) {
		dev_err(indio_dev->dev.parent, "no channel children\n");
		return -ENODEV;
	}

	chan = devm_kcalloc(indio_dev->dev.parent, st->num_channels,
			    sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	st->channel_config = devm_kcalloc(indio_dev->dev.parent,
					  st->num_channels,
					  sizeof(*st->channel_config),
					  GFP_KERNEL);
	if (!st->channel_config)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = st->num_channels;

	for_each_available_child_of_node(np, child) {
		ret = of_property_read_u32(child, "reg", &channel);
		if (ret)
			goto err;

		ret = of_property_read_u32_array(child, "diff-channels",
						 ain, 2);
		if (ret)
			goto err;

		ret = of_property_read_string(child, "channel-name",
				&st->channel_config[channel].channel_name);
		if (ret)
			st->channel_config[channel].channel_name = "none-name";

		ret = ad5940_check_channel_indexes(indio_dev->dev.parent, ain);
		if (ret) {
			dev_err(indio_dev->dev.parent,
				"some input channel index does not exist: %d, %d, %d",
				channel, ain[0], ain[1]);
			goto err;
		}

		st->channel_config[channel].ain = AD5940_CHANNEL_AINP(ain[0]) |
						  AD5940_CHANNEL_AINN(ain[1]);

		*chan = ad5940_channel_template;
		chan->address = channel;
		chan->scan_index = channel;
		chan->channel = ain[0];
		chan->channel2 = ain[1];

		chan++;
	}

	return 0;
err:
	of_node_put(child);

	return ret;
}

static int ad5940_setup(struct ad5940_state *st)
{
	return 0;
}

static int ad5940_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad5940_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad5940_info;

	ret = ad5940_of_parse_channel_config(indio_dev, spi->dev.of_node);
	if (ret < 0)
		return ret;

	ret = ad5940_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad5940_dt_match[] = {
	{ .compatible = "adi,ad5940" },
	{},
};
MODULE_DEVICE_TABLE(of, ad5940_spi_ids);

static struct spi_driver ad5940_driver = {
	.driver = {
		.name = "ad5940",
		.of_match_table = ad5940_dt_match,
	},
	.probe = ad5940_probe,
};
module_spi_driver(ad5940_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("Analog Device AD5940 ADC driver");
MODULE_LICENSE("GPL");
