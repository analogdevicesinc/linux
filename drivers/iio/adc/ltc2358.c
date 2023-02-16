// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices LTC2358 ADC
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <asm/unaligned.h>
#include <linux/cache.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/init.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#define LTC2358_BYTES_PER_CH	3
#define LTC2358_CHANNEL_MSK     GENMASK(2, 0)
#define LTC2358_NUM_CH          7
#define LTC2358_NUM_CONFIG	7
#define T_CONVH			40
#define T_QUIET			20
#define CACHE_LINE_SIZE		L1_CACHE_BYTES
#define LTC2358_CREATE_CONFIG_WORD(channel, config_number, config_word)		\
	(*(config_word) |= ((u32)((config_number) & LTC2358_CHANNEL_MSK)	\
			<< ((channel) * LTC2358_BYTES_PER_CH)))

struct ltc2358_state {
	struct spi_device	*spi;
};

static int ltc2358_read(struct ltc2358_state *adc, u32 config_word,
			u8 *data_array, u8 channel, int *val)
{
	int ret;
	u8 tx_buff[24] __aligned(CACHE_LINE_SIZE) = {0};

	struct spi_transfer xfer = {
		.tx_buf = tx_buff,
		.rx_buf = data_array,
		.len = 24,
		.cs_change = 1,
		.delay = {
			.unit = SPI_DELAY_UNIT_NSECS,
			.value = T_QUIET
		},
		.cs_change_delay = {
			.unit = SPI_DELAY_UNIT_NSECS,
			.value = T_CONVH
		}
	};

	put_unaligned_be24(config_word, tx_buff);

	ret = spi_sync_transfer(adc->spi, &xfer, 1);
	if (ret)
		return ret;

	*val = get_unaligned_be24(&data_array[channel * LTC2358_BYTES_PER_CH]);

	return 0;
}

static int ltc2358_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long info)
{
	struct ltc2358_state *adc = iio_priv(indio_dev);
	u8 data_array[24] __aligned(CACHE_LINE_SIZE);
	u32 config_word = 0;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ltc2358_read(adc, config_word, data_array, chan->channel, val);
		if (ret)
			return ret;
		break;

	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

#define LTC2358_CHAN(name)						\
	struct iio_chan_spec name = {					\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	}

static const struct iio_info ltc2358_info = {
	.read_raw = ltc2358_read_raw,
};

static int ltc2358_parse_channels(struct ltc2358_state *adc,
				  struct iio_dev *indio_dev)
{
	struct iio_chan_spec *ltc2358_channels;
	struct device *dev = &adc->spi->dev;
	struct fwnode_handle *child;
	unsigned int num_ch, idx;
	int chan_idx = 0;
	u32 config_word = 0;
	int config_number, ret;

	num_ch = device_get_child_node_count(dev);
	if (!num_ch)
		return -EINVAL;

	ltc2358_channels = devm_kcalloc(dev, num_ch, sizeof(*ltc2358_channels),
					GFP_KERNEL);

	device_for_each_child_node(dev, child) {
		LTC2358_CHAN(chan);
		ret = device_property_read_u32(dev, "reg", &idx);
		if (ret < 0 || idx > LTC2358_NUM_CH) {
			fwnode_handle_put(child);
			dev_err(dev, "Invalid channel index %d.", idx);
			return -EINVAL;
		}

		chan.channel = idx;

		ret = device_property_read_u32(dev, "config-number", &config_number);
		if (ret < 0 || config_number > LTC2358_NUM_CONFIG) {
			fwnode_handle_put(child);
			dev_err(dev, "Invalid config number %d.", config_number);
			return -EINVAL;
		}

		LTC2358_CREATE_CONFIG_WORD(idx, config_number, &config_word);

		ltc2358_channels[chan_idx] = chan;
		chan_idx++;
	}

	indio_dev->channels = ltc2358_channels;
	indio_dev->num_channels = num_ch;

	return 0;
}

static int ltc2358_probe(struct spi_device *spi)
{
	struct ltc2358_state *adc;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	indio_dev->name = "ltc2358";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc2358_info;

	ret = ltc2358_parse_channels(adc, indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2358_of_match[] = {
	{ .compatible = "adi,ltc2358" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2358_of_match);

static struct spi_device_id ltc2358[] = {
	{"ltc2358", 0},
	{ }
};
MODULE_DEVICE_TABLE(spi, ltc2358);

static struct spi_driver ltc2358_driver = {
	.driver = {
		.name = "ltc2358",
		.of_match_table = ltc2358_of_match,
	},
	.probe = ltc2358_probe,
	.id_table = ltc2358,
};
module_spi_driver(ltc2358_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2358 ADC");
MODULE_LICENSE("GPL v2");
