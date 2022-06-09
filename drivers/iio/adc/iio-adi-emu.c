// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. Emulator Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/regmap.h>

#define REG_SCRATCH_PAD		0x01

#define REG_DEVICE_CONFIG	0x02
#define POWER_DOWN		BIT(5)

#define REG_CNVST		0x03
#define CNVST			BIT(0)

#define REG_CH0_DATA_HIGH	0x04
#define REG_CH0_DATA_LOW	0x05
#define REG_CH1_DATA_HIGH	0x06
#define REG_CH1_DATA_LOW	0x07

#define REG_TEST_MODE		0x08
#define TEST_MODE(x)		(((x) & 0x7) << 1)

enum adi_emu_test_mode {
	TEST_OFF,
	TEST_MIDSCALE,
	TEST_POSITIVE,
	TEST_NEGATIVE,
	TEST_RAMP,
};

struct adi_emu_priv {
	bool		enable;
	struct regmap	*regmap;
	unsigned int	test_mode;
};

static int adi_emu_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int high, low;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = priv->enable;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ret = regmap_write(priv->regmap, REG_CNVST, CNVST);
		if (ret)
			return ret;
		if (chan->channel) {
			ret = regmap_read(priv->regmap,
				REG_CH1_DATA_HIGH, &high);
			if (ret)
				return ret;
			ret = regmap_read(priv->regmap,
				REG_CH1_DATA_LOW, &low);
			if (ret)
				return ret;
		} else {
			ret = regmap_read(priv->regmap,
				REG_CH0_DATA_HIGH, &high);
			if (ret)
				return ret;
			ret = regmap_read(priv->regmap,
				REG_CH0_DATA_LOW, &low);
			if (ret)
				return ret;
		}
		*val = (high << 8) | low;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adi_emu_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_write(priv->regmap, REG_DEVICE_CONFIG,
			!val ? POWER_DOWN : 0);
		if (ret)
			return ret;
		priv->enable = val;
		return 0;
	}

	return -EINVAL;
}

static int adi_emu_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);

	if (readval)
		return regmap_read(priv->regmap, reg, readval);
	
	return regmap_write(priv->regmap, reg, writeval);
}

enum scratch_pad_iio_dev_attr {
	SCRATCH_PAD,
};

static ssize_t adi_emu_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	switch (this_attr->address) {
	case SCRATCH_PAD:
		regmap_read(priv->regmap, REG_SCRATCH_PAD, &reg_val);
		ret = sprintf(buf, "0x%x\n", reg_val);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t adi_emu_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	switch (this_attr->address) {
	case SCRATCH_PAD:
		ret = kstrtou32(buf, 16, &reg_val);
		if (ret < 0)
			break;
		ret = regmap_write(priv->regmap, REG_SCRATCH_PAD, reg_val);
		if (ret < 0)
			break;
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(scratch_pad, S_IRUGO | S_IWUSR,
		       adi_emu_show,
		       adi_emu_store,
		       SCRATCH_PAD);

static struct attribute *adi_emu_attributes[] = {
	&iio_dev_attr_scratch_pad.dev_attr.attr,
	NULL,
};

static const struct attribute_group adi_emu_attribute_group = {
	.attrs = adi_emu_attributes,
};

static const struct iio_info adi_emu_info = {
	.read_raw = &adi_emu_read_raw,
	.write_raw = &adi_emu_write_raw,
	.debugfs_reg_access = &adi_emu_reg_access,
	.attrs = &adi_emu_attribute_group,
};

static const char * const adi_emu_test_modes[] = {
	[TEST_OFF] = "off",
	[TEST_MIDSCALE] = "midscale",
	[TEST_POSITIVE] = "positive",
	[TEST_NEGATIVE] = "negative",
	[TEST_RAMP] = "ramp",
};

static int adi_emu_test_mode_write(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int item)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	int ret;

	ret = regmap_write(priv->regmap, REG_TEST_MODE,
		TEST_MODE(item));
	if (ret)
		return ret;

	if (priv->enable)
		priv->test_mode = item;

	return 0;
}

static int adi_emu_test_mode_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct adi_emu_priv *priv = iio_priv(indio_dev);

	return priv->test_mode;
}

static const struct iio_enum adi_emu_test_mode_enum = {
	.items = adi_emu_test_modes,
	.num_items = ARRAY_SIZE(adi_emu_test_modes),
	.set = adi_emu_test_mode_write,
	.get = adi_emu_test_mode_read,
};

static struct iio_chan_spec_ext_info adi_emu_ext_info[] = {
	IIO_ENUM("test_mode", IIO_SHARED_BY_ALL, &adi_emu_test_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("test_mode", IIO_SHARED_BY_ALL,
		&adi_emu_test_mode_enum),
	{},
};

static irqreturn_t adi_emu_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adi_emu_priv *priv = iio_priv(indio_dev);
	unsigned int high, low;
	u16 buf[2];
	int i = 0, ret;

	ret = regmap_write(priv->regmap, REG_CNVST, CNVST);
	if (ret)
		return ret;
	if (*indio_dev->active_scan_mask & BIT(0)) {
		ret = regmap_read(priv->regmap,
			REG_CH0_DATA_HIGH, &high);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap,
			REG_CH0_DATA_LOW, &low);
		if (ret)
			return ret;
		buf[i++] = (high << 8) | low;
	}
	if (*indio_dev->active_scan_mask & BIT(1)) {
		ret = regmap_read(priv->regmap,
			REG_CH1_DATA_HIGH, &high);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap,
			REG_CH1_DATA_LOW, &low);
		if (ret)
			return ret;
		buf[i] = (high << 8) | low;
	}

	iio_push_to_buffers(indio_dev, buf);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_chan_spec adi_emu_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = adi_emu_ext_info,
		.output = 0,
		.indexed = 1,
		.channel = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}, {
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = adi_emu_ext_info,
		.output = 0,
		.indexed = 1,
		.channel = 1,
		.scan_index = 1,
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 0,
			.endianness = IIO_LE,
		},
	}
};

static const struct regmap_config adi_emu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x8,
};

static int adi_emu_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adi_emu_priv *priv;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->enable = false;
	priv->test_mode = TEST_RAMP;
	priv->regmap = devm_regmap_init_spi(spi, &adi_emu_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	indio_dev->name = "iio-adi-emu";
	indio_dev->channels = adi_emu_channels;
	indio_dev->num_channels = ARRAY_SIZE(adi_emu_channels);
	indio_dev->info = &adi_emu_info;

	devm_iio_triggered_buffer_setup(&spi->dev, indio_dev, NULL,
			&adi_emu_trigger_handler, NULL);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver adi_emu_driver = {
	.driver = {
		.name = "iio-adi-emu",
	},
	.probe = adi_emu_probe,
};
module_spi_driver(adi_emu_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("IIO ADI Emulator Driver");
MODULE_LICENSE("GPL v2");

