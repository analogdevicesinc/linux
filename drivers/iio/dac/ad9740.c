// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9740/AD9742/AD9744/AD9748
 * 8/10/12/14-Bit, 210 MSPS Digital-to-Analog Converters
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>

struct ad9740_chip_info {
	const char *name;
	unsigned int resolution;
	const struct iio_chan_spec *channels;
	int num_channels;
};

struct ad9740_state {
	struct device *dev;
	struct iio_backend *back;
	struct gpio_desc *reset_gpio;
	const struct ad9740_chip_info *chip_info;
	bool twos_complement;
	struct mutex lock;
};

static const char * const ad9740_data_sources[] = {
	[IIO_BACKEND_EXTERNAL] = "normal",
	[IIO_BACKEND_INTERNAL_CONTINUOUS_WAVE] = "dds",
	[IIO_BACKEND_INTERNAL_RAMP_16BIT] = "ramp",
};

static int ad9740_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad9740_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return iio_backend_data_stream_enable(st->back);
}

static int ad9740_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad9740_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return iio_backend_data_stream_disable(st->back);
}

static ssize_t ad9740_ext_info_get_data_source(struct iio_dev *indio_dev,
						uintptr_t private,
						const struct iio_chan_spec *chan,
						char *buf)
{
	struct ad9740_state *st = iio_priv(indio_dev);
	enum iio_backend_data_source type;
	int ret;

	ret = iio_backend_data_source_get(st->back, 0, &type);
	if (ret)
		return ret;

	if (type >= ARRAY_SIZE(ad9740_data_sources) || !ad9740_data_sources[type])
		return -EINVAL;

	return sysfs_emit(buf, "%s\n", ad9740_data_sources[type]);
}

static ssize_t ad9740_ext_info_set_data_source(struct iio_dev *indio_dev,
						uintptr_t private,
						const struct iio_chan_spec *chan,
						const char *buf, size_t len)
{
	struct ad9740_state *st = iio_priv(indio_dev);
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ad9740_data_sources); i++) {
		if (ad9740_data_sources[i] && sysfs_streq(buf, ad9740_data_sources[i])) {
			ret = iio_backend_data_source_set(st->back, 0, i);
			return ret ? ret : len;
		}
	}

	return -EINVAL;
}

static ssize_t ad9740_ext_info_get_data_source_available(struct iio_dev *indio_dev,
							  uintptr_t private,
							  const struct iio_chan_spec *chan,
							  char *buf)
{
	ssize_t l = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(ad9740_data_sources); i++)
		if (ad9740_data_sources[i])
			l += sysfs_emit_at(buf, l, "%s ", ad9740_data_sources[i]);

	buf[l - 1] = '\n';
	return l;
}

static const struct iio_chan_spec_ext_info ad9740_ext_info[] = {
	{
		.name = "data_source",
		.read = ad9740_ext_info_get_data_source,
		.write = ad9740_ext_info_set_data_source,
		.shared = IIO_SEPARATE,
	},
	{
		.name = "data_source_available",
		.read = ad9740_ext_info_get_data_source_available,
		.shared = IIO_SHARED_BY_ALL,
	},
	{ }
};

static int ad9740_setup(struct ad9740_state *st)
{
	struct iio_backend_data_fmt fmt = {
		.sign_extend = false,
		.enable = true,
	};
	int ret;

	if (st->twos_complement)
		fmt.type = IIO_BACKEND_TWOS_COMPLEMENT;
	else
		fmt.type = IIO_BACKEND_OFFSET_BINARY;

	ret = iio_backend_data_format_set(st->back, 0, &fmt);
	if (ret)
		return ret;

	return iio_backend_data_source_set(st->back, 0, IIO_BACKEND_EXTERNAL);
}

static const struct iio_buffer_setup_ops ad9740_buffer_setup_ops = {
	.postenable = ad9740_buffer_postenable,
	.predisable = ad9740_buffer_predisable,
};

#define AD9740_CHAN_DDS { \
	.type = IIO_ALTVOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.scan_index = -1, \
}

#define AD9740_CHAN_DATA(bits, shft) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.output = 1, \
	.ext_info = ad9740_ext_info, \
	.scan_type = { \
		.sign = 'u', \
		.realbits = (bits), \
		.storagebits = 16, \
		.shift = (shft), \
		.endianness = IIO_BE, \
	}, \
}

static const struct iio_chan_spec ad9748_channels[] = {
	AD9740_CHAN_DDS,
	AD9740_CHAN_DATA(8, 8),
};

static const struct iio_chan_spec ad9740_channels[] = {
	AD9740_CHAN_DDS,
	AD9740_CHAN_DATA(10, 6),
};

static const struct iio_chan_spec ad9742_channels[] = {
	AD9740_CHAN_DDS,
	AD9740_CHAN_DATA(12, 4),
};

static const struct iio_chan_spec ad9744_channels[] = {
	AD9740_CHAN_DDS,
	AD9740_CHAN_DATA(14, 0),
};

static const struct iio_info ad9740_info = {
};

static const struct ad9740_chip_info ad9748_chip_info = {
	.name = "ad9748",
	.resolution = 8,
	.channels = ad9748_channels,
	.num_channels = ARRAY_SIZE(ad9748_channels),
};

static const struct ad9740_chip_info ad9740_chip_info = {
	.name = "ad9740",
	.resolution = 10,
	.channels = ad9740_channels,
	.num_channels = ARRAY_SIZE(ad9740_channels),
};

static const struct ad9740_chip_info ad9742_chip_info = {
	.name = "ad9742",
	.resolution = 12,
	.channels = ad9742_channels,
	.num_channels = ARRAY_SIZE(ad9742_channels),
};

static const struct ad9740_chip_info ad9744_chip_info = {
	.name = "ad9744",
	.resolution = 14,
	.channels = ad9744_channels,
	.num_channels = ARRAY_SIZE(ad9744_channels),
};

static int ad9740_probe(struct platform_device *pdev)
{
	struct ad9740_state *st;
	struct iio_dev *indio_dev;
	struct iio_chan_spec *channels;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;

	st->chip_info = device_get_match_data(&pdev->dev);
	if (!st->chip_info)
		return dev_err_probe(&pdev->dev, -ENODEV, "Failed to get chip info\n");

	mutex_init(&st->lock);

	st->twos_complement = device_property_read_bool(&pdev->dev,
							"adi,twos-complement");

	st->back = devm_iio_backend_get(&pdev->dev, NULL);
	if (IS_ERR(st->back))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->back),
				     "Failed to get backend\n");

	ret = devm_iio_backend_enable(&pdev->dev, st->back);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to enable backend\n");

	st->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		msleep(10);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
		msleep(10);
	}

	channels = devm_kmemdup(&pdev->dev, st->chip_info->channels,
				sizeof(struct iio_chan_spec) * st->chip_info->num_channels,
				GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	ret = iio_backend_extend_chan_spec(st->back, &channels[0]);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to extend channel spec\n");

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad9740_buffer_setup_ops;
	indio_dev->channels = channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad9740_info;

	ret = devm_iio_backend_request_buffer(&pdev->dev, st->back, indio_dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "Failed to request backend buffer\n");

	ret = ad9740_setup(st);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "AD9740 setup failed\n");

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register IIO device\n");

	iio_backend_debugfs_add(st->back, indio_dev);

	return 0;
}

static const struct of_device_id ad9740_of_id[] = {
	{ .compatible = "adi,ad9748", .data = &ad9748_chip_info },
	{ .compatible = "adi,ad9740", .data = &ad9740_chip_info },
	{ .compatible = "adi,ad9742", .data = &ad9742_chip_info },
	{ .compatible = "adi,ad9744", .data = &ad9744_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9740_of_id);

static struct platform_driver ad9740_driver = {
	.driver = {
		.name = "ad9740",
		.of_match_table = ad9740_of_id,
	},
	.probe = ad9740_probe,
};
module_platform_driver(ad9740_driver);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("Analog Devices AD9740/AD9742/AD9744/AD9748 DAC Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BACKEND);
