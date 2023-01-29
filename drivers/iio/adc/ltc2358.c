// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices LTC2358 ADC
 *
 * Copyright 2023 Analog Devices Inc.
 */
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
#define T_CONVH			40
#define T_QUIET			20

struct ltc2358_state {
	struct spi_device	*spi;
	struct mutex		lock;
};

static void ltc2358_create_config_word(u8 channel, u8 config_number,
				       u32 *config_word)
{
	*config_word |= (u32)(config_number & LTC2358_CHANNEL_MSK)
			<< (channel * LTC2358_BYTES_PER_CH);
}

static int ltc2358_read(struct ltc2358_state *adc, u32 config_word,
			u8 data_array[24], u8 channel, int *val)
{
	int ret;
	u8 tx_buff[24] = {0};

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

	tx_buff[2] = config_word & 0xFF;
	tx_buff[1] = (config_word >> 8) & 0xFF;
	tx_buff[0] = config_word >> 16;

	ret = spi_sync_transfer(adc->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	*val = (data_array[channel * LTC2358_BYTES_PER_CH] << 16 |
		data_array[channel * LTC2358_BYTES_PER_CH + 1] << 8 |
		data_array[channel * LTC2358_BYTES_PER_CH + 2]);

	return 0;
}

static int ltc2358_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ltc2358_state *adc = iio_priv(indio_dev);
	u8 data_array[24];
	u32 config_word = 0;
	int ret;

	if (info == IIO_CHAN_INFO_RAW) {
		mutex_lock(&adc->lock);
		ret = ltc2358_read(adc, config_word, data_array,
				   chan->channel, val);
		mutex_unlock(&adc->lock);
		if (ret < 0)
			return ret;
	} else
		return -EINVAL;

	return IIO_VAL_INT;
}

static const struct iio_chan_spec ltc2358_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)
	}
};

static const struct iio_info ltc2358_info = {
	.read_raw = ltc2358_read_raw,
};

static int ltc2358_parse_dt(struct ltc2358_state *adc,
			    struct iio_dev *indio_dev)
{
	struct iio_chan_spec *ltc2358_channels;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	unsigned int num_ch, idx;
	int chan_idx = 0;
	u32 config_word = 0;
	int config_number, ret;

	fwnode = dev_fwnode(indio_dev->dev.parent);
	num_ch = device_get_child_node_count(indio_dev->dev.parent);
	if (num_ch == 0)
		return -EINVAL;

	ltc2358_channels = devm_kzalloc(indio_dev->dev.parent,
					num_ch * sizeof(struct iio_chan_spec),
					GFP_KERNEL);

	fwnode_for_each_child_node(fwnode, child) {
		ret = fwnode_property_read_u32(child, "reg", &idx);
		if (ret < 0 || idx > 7) {
			fwnode_handle_put(child);
			pr_err("Invalid channel index %d.", idx);
			return -EINVAL;
		}

		ltc2358_channels->channel = idx;

		ret = fwnode_property_read_u32(child, "config-number", &config_number);
		if (ret < 0 || config_number > 7) {
			fwnode_handle_put(child);
			pr_err("Invalid config number %d.", config_number);
			return -EINVAL;
		}

		ltc2358_create_config_word(chan_idx, config_number, &config_word);

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

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ltc2358";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ltc2358_info;

	ret = ltc2358_parse_dt(adc, indio_dev);
	if (ret < 0)
		return ret;

	ret = spi_setup(spi);
	if (ret < 0)
		return ret;

	spi_set_drvdata(spi, indio_dev);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static int ltc2358_remove(struct spi_device *spi)
{
	struct ltc2358_state *adc;
	struct iio_dev *indio_dev;

	indio_dev = spi_get_drvdata(spi);
	adc = iio_priv(indio_dev);

	return 0;
}

static const struct of_device_id ltc2358_of_match[] = {
	{ .compatible = "adi,ltc2358" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2358_of_match);

static struct spi_device_id ltc2358[] = {
	{"ltc2358", 0},
	{ },
};
MODULE_DEVICE_TABLE(spi, ltc2358);

static struct spi_driver ltc2358_driver = {
	.driver = {
		.name = "ltc2358",
		.of_match_table = ltc2358_of_match,
	},
	.probe = ltc2358_probe,
	.remove = ltc2358_remove,
	.id_table = ltc2358
};
module_spi_driver(ltc2358_driver);

MODULE_AUTHOR("Kim Seer Paller <kimseer.paller@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC2358 ADC");
MODULE_LICENSE("GPL v2");
