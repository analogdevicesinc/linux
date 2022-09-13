// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Analog Devices, Inc.
 * Author: Ciprian Regus <ciprian.regus@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#define AD5752_INT_VREF			2500
#define AD5752_FRAME_SIZE		3
#define AD5752_MAX_CHANNELS		4
#define AD5752_MAX_RESOLUTION		16

#define AD5752_DATA_MASK(_lsb)		GENMASK(15, _lsb)

#define AD5752_RANGE_MASK		GENMASK(2, 0)

#define AD5752_REG_RD			BIT(7)

#define AD5752_CLEAR_FUNC		BIT(2)
#define AD5752_LOAD_FUNC		BIT(2) | BIT(0)
#define AD5752_NO_OP_FUNC		0x18

#define AD5752_PU_ADDR			0
#define AD5752_PU_MASK			GENMASK(3, 0)
#define AD5752_PU_CH(x)			BIT(x)
#define AD5752_INT_REF_MASK		BIT(4)
#define AD5752_INT_REF			BIT(4)

#define AD5752_DAC			0
#define AD5752_RANGE			BIT(0)
#define AD5752_PWR			BIT(1)
#define AD5752_CTRL			GENMASK(1, 0)

#define AD5752_REG_ADDR(reg, addr)	((reg << 3) | addr)

#define AD5752_CHANNEL(_channel)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = _channel,					\
		.output = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)    |	\
				      BIT(IIO_CHAN_INFO_SCALE)  |	\
				      BIT(IIO_CHAN_INFO_OFFSET)		\
	}					

enum ad5752_num_channels {
	AD5752_2_CHANNELS,
	AD5752_4_CHANNELS,
};

struct ad5752_span_tbl {
	int min;
	int max;
};

const struct ad5752_span_tbl ad5752_range[] = {
	{0, 5000000},
	{0, 10000000},
	{0, 10800000},
	{-5000000, 5000000},
	{-10000000, 10000000},
	{-10800000, 10800000},
};

enum AD5752_TYPE {
	AD5722,
	AD5732,
	AD5752,
	AD5724,
	AD5734,
	AD5754,
	AD5722R,
	AD5732R,
	AD5752R,
	AD5724R,
	AD5734R,
	AD5754R,
};

struct ad5752_chip_info {
	enum AD5752_TYPE chip_type;
	const char *name;
	u32 resolution;
	bool internal_vref;
	const u32 data_mask;
	const struct iio_chan_spec *channels;
	u32 num_channels;
};

const struct iio_chan_spec ad5752_channels[][AD5752_MAX_CHANNELS] = {
	[AD5752_2_CHANNELS] = {
		AD5752_CHANNEL(0),
		AD5752_CHANNEL(1),
	},
	[AD5752_4_CHANNELS] = {
		AD5752_CHANNEL(0),
		AD5752_CHANNEL(1),
		AD5752_CHANNEL(2),
		AD5752_CHANNEL(3),
	},
};

const struct ad5752_chip_info ad5752_chip_info_data[] = {
	[AD5722] = {
		.chip_type = AD5722,
		.name = "ad5722",
		.resolution = 12,
		.data_mask = AD5752_DATA_MASK(4),
		.internal_vref = false,
		.num_channels = 2,
		.channels = ad5752_channels[AD5752_2_CHANNELS],
	},
	[AD5732] = {
		.chip_type = AD5732,
		.name = "ad5732",
		.resolution = 14,
		.data_mask = AD5752_DATA_MASK(2),
		.internal_vref = false,
		.num_channels = 2,
		.channels = ad5752_channels[AD5752_2_CHANNELS],
	},
	[AD5752] = {
		.chip_type = AD5752,
		.name = "ad5752",
		.resolution = 16,
		.data_mask = AD5752_DATA_MASK(0),
		.internal_vref = false,
		.num_channels = 2,
		.channels = ad5752_channels[AD5752_2_CHANNELS],
	},
	[AD5724] = {
		.chip_type = AD5724,
		.name = "ad5724",
		.resolution = 12,
		.data_mask = AD5752_DATA_MASK(4),
		.internal_vref = false,
		.num_channels = 4,
		.channels = ad5752_channels[AD5752_4_CHANNELS],
	},
	[AD5734] = {
		.chip_type = AD5734,
		.name = "ad5734",
		.resolution = 14,
		.data_mask = AD5752_DATA_MASK(2),
		.internal_vref = false,
		.num_channels = 4,
		.channels = ad5752_channels[AD5752_4_CHANNELS],
	},
	[AD5754] = {
		.chip_type = AD5754,
		.name = "ad5754",
		.resolution = 16,
		.data_mask = AD5752_DATA_MASK(0),
		.internal_vref = false,
		.num_channels = 4,
		.channels = ad5752_channels[AD5752_4_CHANNELS],
	},
	[AD5722R] = {
		.chip_type = AD5722R,
		.name = "ad5722r",
		.resolution = 12,
		.data_mask = AD5752_DATA_MASK(4),
		.internal_vref = true,
		.num_channels = 2,
		.channels = ad5752_channels[AD5752_2_CHANNELS],
	},
	[AD5732R] = {
		.chip_type = AD5732R,
		.name = "ad5732r",
		.resolution = 14,
		.data_mask = AD5752_DATA_MASK(2),
		.internal_vref = true,
		.num_channels = 2,
		.channels = ad5752_channels[AD5752_2_CHANNELS],
	},
	[AD5752R] = {
		.chip_type = AD5752R,
		.name = "ad5752r",
		.resolution = 16,
		.data_mask = AD5752_DATA_MASK(0),
		.internal_vref = true,
		.num_channels = 2,
		.channels = ad5752_channels[AD5752_2_CHANNELS],
	},
	[AD5724R] = {
		.chip_type = AD5724R,
		.name = "ad5724r",
		.resolution = 12,
		.data_mask = AD5752_DATA_MASK(4),
		.internal_vref = true,
		.num_channels = 4,
		.channels = ad5752_channels[AD5752_4_CHANNELS],
	},
	[AD5734R] = {
		.chip_type = AD5734R,
		.name = "ad5734r",
		.resolution = 14,
		.data_mask = AD5752_DATA_MASK(2),
		.internal_vref = true,
		.num_channels = 4,
		.channels = ad5752_channels[AD5752_4_CHANNELS],
	},
	[AD5754R] = {
		.chip_type = AD5754R,
		.name = "ad5754r",
		.resolution = 16,
		.data_mask = AD5752_DATA_MASK(0),
		.internal_vref = true,
		.num_channels = 4,
		.channels = ad5752_channels[AD5752_4_CHANNELS],
	}
};

struct ad5752_state {
	struct mutex lock;
	
	struct regmap *regmap;
	struct spi_device *spi;
	struct device *dev;

	const struct ad5752_chip_info *chip_info;
	u32 dac_max_code;
	u32 range_idx;
	u32 vref;
	bool bipolar;
	u32 data_mask;
	u32 sub_lsb;

	u8 rx_buff[AD5752_FRAME_SIZE] ____cacheline_aligned;
	u8 tx_buff[AD5752_FRAME_SIZE] ____cacheline_aligned;
};

/*
 * The channel addresses for 2 channel chip variants are not sequential:
 *      A2 A1 A0 Channel
 *	0  0  0   DAC A
 *	0  1  0   DAC B
 * 
 * This is not the case for 4 channel chips:
 * 	A2 A1 A0 Channel
 *	0  0  0   DAC A
 *	0  0  1   DAC B
 *	0  1  0   DAC C
 *	0  1  1   DAC D
 */
static unsigned int ad5752_real_ch(struct ad5752_state *st, u32 channel, u32 *real_channel)
{
	switch (st->chip_info->num_channels) {
	case 2:
		if (channel == 0)
			*real_channel = 0;
		*real_channel = 2;
		break;
	case 4:
		*real_channel = channel;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad5752_get_output_range(struct ad5752_state *st, struct fwnode_handle *channel_node)
{
	u32 range[2], max;
	int min;
	int ret;
	u32 i;

	ret = fwnode_property_read_u32_array(channel_node, "output-range-microvolts",
					     range, 2);
	if (ret)
		return ret;

	min = range[0];
	max = range[1];

	for (i = 0; i < ARRAY_SIZE(ad5752_range); i++) {
		if (ad5752_range[i].min != min ||
			ad5752_range[i].max != max)
			continue;

		st->range_idx = i;
		if (min < 0)
			st->bipolar = true;

		return 0;
	}

	return -EINVAL;
}

static int ad5752_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad5752_state *st = context;

	struct spi_transfer tx = {
		.tx_buf = st->tx_buff,
		.len = 3,
	};

	st->tx_buff[0] = reg;
	put_unaligned_be16(val, &st->tx_buff[1]);

	return spi_sync_transfer(st->spi, &tx, 1);
};

static int ad5752_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad5752_state *st = context;
	struct spi_transfer tx_xfer[] = {
		{
			.tx_buf = st->tx_buff,
			.len = AD5752_FRAME_SIZE,
		},
	};
	struct spi_transfer rx_xfer[] = {
		{
			.tx_buf = st->tx_buff,
			.rx_buf = st->rx_buff,
			.len = AD5752_FRAME_SIZE,
		},
	};
	int ret;

	st->tx_buff[0] = AD5752_REG_RD | reg;
	ret = spi_sync_transfer(st->spi, tx_xfer, 1);
	if (ret)
		return ret;
	
	st->tx_buff[0] = AD5752_NO_OP_FUNC;
	st->tx_buff[1] = 0;
	st->tx_buff[2] = 0;
	ret = spi_sync_transfer(st->spi, rx_xfer, 1);
	if (ret)
		return ret;

	*val = get_unaligned_be16(&st->rx_buff[1]);

	return 0;
};

static const struct regmap_config ad5752_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.reg_write = ad5752_reg_write,
	.reg_read = ad5752_reg_read,
};

static int ad5752_set_dac_code(struct ad5752_state *st, u32 channel, u32 code)
{
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_write_bits(st->regmap, AD5752_REG_ADDR(AD5752_DAC, channel),
				st->chip_info->data_mask, code << st->sub_lsb);
	if (ret)
		goto unlock;
	ret = regmap_write_bits(st->regmap, AD5752_REG_ADDR(AD5752_CTRL, AD5752_LOAD_FUNC),
				st->chip_info->data_mask, 0);
unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad5752_enable_channels(struct ad5752_state *st)
{
	struct fwnode_handle *channel_node = NULL;
	u32 real_channel;
	u32 dac_value;
	u32 power_reg;
	u32 index;
	int ret;

	fwnode_for_each_available_child_node(dev_fwnode(st->dev), channel_node) {
		ret = fwnode_property_read_u32(channel_node, "reg", &index);
		if (ret) {
			dev_err(st->dev, "Failed to read channel reg: %d\n", ret);
			goto free_node;
		}
		if (index >= st->chip_info->num_channels) {
			dev_err(st->dev, "Channel index %u is too large\n", index);
			goto free_node;
		}

		ret = fwnode_property_read_u32(channel_node, "adi,init-dac-code", &dac_value);
		if (ret) {
			dev_err(st->dev, "No init dac value for channel %d\n", index);
			goto free_node;
		}

		ret = ad5752_real_ch(st, index, &real_channel);
		if (ret)
			goto free_node;

		ret = ad5752_get_output_range(st, channel_node);
		if (ret)
			goto free_node;

		/* Set the output range */
		ret = regmap_write_bits(st->regmap,
					AD5752_REG_ADDR(AD5752_RANGE, real_channel),
					AD5752_RANGE_MASK, st->range_idx);
		if (ret)
			goto free_node;
		ret = regmap_read(st->regmap, AD5752_REG_ADDR(AD5752_PWR, AD5752_PU_ADDR), &power_reg);
		if  (ret)
			return ret;
		/* Power up a channel */
		ret = regmap_update_bits(st->regmap, AD5752_REG_ADDR(AD5752_PWR, AD5752_PU_ADDR),
					 AD5752_PU_MASK, AD5752_PU_CH(real_channel) |
					 (power_reg & AD5752_PU_MASK));
		if (ret)
			goto free_node;
		mdelay(10);
		ret = ad5752_set_dac_code(st, real_channel, dac_value);
		if (ret)
			goto free_node;
	}

	return 0;

free_node:
	fwnode_handle_put(channel_node);

	return ret;
}

static int ad5752_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad5752_state *st = iio_priv(indio_dev);
	u32 real_channel;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > st->dac_max_code) {
			dev_err(st->dev, "Invalid DAC code %d\n", val);
			return -EINVAL;
		}
		ret = ad5752_real_ch(st, chan->channel, &real_channel);
		if (ret)
			return ret;

		return ad5752_set_dac_code(st, real_channel, val);
	default:
		return -EINVAL;
	}
}

static int ad5752_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long info)
{
	struct ad5752_state *st = iio_priv(indio_dev);
	const struct ad5752_span_tbl *range;
	u32 real_channel;
	u32 gain;
	int ret;

	ret = ad5752_real_ch(st, chan->channel, &real_channel);
	if (ret)
		return ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(st->regmap, real_channel, val);
		if (ret)
			return ret;
		break;
	case IIO_CHAN_INFO_SCALE:
		range = &ad5752_range[st->range_idx];
		gain = (range->max - range->min) / 2500;
		*val = st->vref * gain / 1000;
		*val2 = st->chip_info->resolution;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		if (st->bipolar)
			*val = -(st->chip_info->data_mask >> 1);
		else
			*val = 0;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
	
	return IIO_VAL_INT;
}

static int ad5752_int_vref_enable(struct ad5752_state *st)
{
	return regmap_update_bits(st->regmap, AD5752_REG_ADDR(AD5752_PWR, AD5752_PU_ADDR),
				  AD5752_INT_REF_MASK,
				  FIELD_PREP(AD5752_INT_REF_MASK, 1));
}

static void ad5752_disable_regulator(void *regulator)
{
	regulator_disable(regulator);
}

static const struct iio_info ad5752_info = {
	.read_raw = &ad5752_read_raw,
	.write_raw = &ad5752_write_raw,
};

static int ad5752_probe(struct spi_device *spi)
{
	struct regulator *vref_reg;
	struct iio_dev *indio_dev;
	struct ad5752_state *st;
	struct device *dev;
	int ret;

	dev = &spi->dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;
	st->dev = dev;
	st->chip_info = device_get_match_data(dev);
	if (!st->chip_info)
		dev_err(dev, "AD5752 probe err\n");
	mutex_init(&st->lock);

	st->regmap = devm_regmap_init(st->dev, NULL, st, &ad5752_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(st->dev, PTR_ERR(vref_reg),
				     "Regmap init error\n");
	st->bipolar = false;
	st->dac_max_code = (1 << st->chip_info->resolution) - 1;
	st->sub_lsb = AD5752_MAX_RESOLUTION - st->chip_info->resolution;

	/* Disable daisy-chain mode */
	ret = regmap_write_bits(st->regmap, AD5752_REG_ADDR(AD5752_CTRL, 1), GENMASK(15, 0), 1);
	if (ret)
		return ret;

	vref_reg = devm_regulator_get_optional(st->dev, "vref");
	if (IS_ERR(vref_reg)) {
		if (st->chip_info->internal_vref == false)
			return dev_err_probe(st->dev, PTR_ERR(vref_reg),
			       "Failed to get the vref regulator\n");

		st->vref = AD5752_INT_VREF;
		ret = ad5752_int_vref_enable(st);
		if (ret)
			return ret;
	} else {
		ret = regulator_enable(vref_reg);
		if (ret)
			return dev_err_probe(st->dev, PTR_ERR(vref_reg),
				"Failed to enable the vref regulator\n");
		
		ret = devm_add_action_or_reset(dev, ad5752_disable_regulator, vref_reg);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vref_reg);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Failed to get vref\n");
		
		st->vref = ret / 1000;
	}

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad5752_info;
	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;

	ret = ad5752_enable_channels(st);
	if (ret)
		return ret;

	return devm_iio_device_register(st->dev, indio_dev);
}

static int __init ad5752_register_driver(struct spi_driver *spi)
{
	return spi_register_driver(spi);
}

static int ad5752_unregister_driver(struct spi_driver *spi)
{
	spi_unregister_driver(spi);

	return 0;
}

static const struct of_device_id ad5752_dt_id[] = {
	{
		.compatible = "adi,ad5722",
		.data = &ad5752_chip_info_data[AD5722],
	},
	{
		.compatible = "adi,ad5732",
		.data = &ad5752_chip_info_data[AD5732],
	},
	{
		.compatible = "adi,ad5752",
		.data = &ad5752_chip_info_data[AD5752],
	},
	{
		.compatible = "adi,ad5724",
		.data = &ad5752_chip_info_data[AD5724],
	},
	{
		.compatible = "adi,ad5734",
		.data = &ad5752_chip_info_data[AD5734],
	},
	{
		.compatible = "adi,ad5754",
		.data = &ad5752_chip_info_data[AD5754],
	},
	{
		.compatible = "adi,ad5722r",
		.data = &ad5752_chip_info_data[AD5722R],
	},
	{
		.compatible = "adi,ad5732r",
		.data = &ad5752_chip_info_data[AD5732R],
	},
	{
		.compatible = "adi,ad5752r",
		.data = &ad5752_chip_info_data[AD5752R],
	},
	{
		.compatible = "adi,ad5724r",
		.data = &ad5752_chip_info_data[AD5724R],
	},
	{
		.compatible = "adi,ad5734r",
		.data = &ad5752_chip_info_data[AD5734R],
	},
	{
		.compatible = "adi,ad5754r",
		.data = &ad5752_chip_info_data[AD5754R],
	},
	{},
};
MODULE_DEVICE_TABLE(of, ad5752_dt_id);

static struct spi_driver ad5752_driver = {
	.driver = {
		.name = "ad5752",
		.of_match_table = ad5752_dt_id,
	},
	.probe = ad5752_probe,
};

module_driver(ad5752_driver,
	      ad5752_register_driver,
	      ad5752_unregister_driver);

MODULE_AUTHOR("Ciprian Regus <ciprian.regus@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5752 DAC");
MODULE_LICENSE("GPL v2");