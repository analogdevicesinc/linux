// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Analog Devices, Inc.
 * Author: Ana-Maria Cusco <ana-maria.cusco@analog.com>
 */

/* In development */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#include <asm/div64.h>
#include <asm/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include "ad4170.h"




enum ad4170_ref_select {
	AD4170_REF_REFIN1,
	AD4170_REF_REFIN2,
	AD4170_REF_REFOUT_AVSS,
	AD4170_REF_AVDD_AVSS,
	AD4170_REF_SEL_MAX
};

enum ad4170_iout {
	AD4170_IOUT_OFF,
	AD4170_IOUT_10MICROA,
	AD4170_IOUT_50MICROA,
	AD4170_IOUT_100MICROA,
	AD4170_IOUT_250MICROA,
	AD4170_IOUT_500MICROA,
	AD4170_IOUT_1000MICROA,
	AD4170_IOUT_1500MICROA,
	AD4170_IOUT_MAX
};

enum ad4170_burnout {
	AD4170_BURNOUT_OFF,
	AD4170_BURNOUT_100NA,
	AD4170_BURNOUT_2000NA,
	AD4170_BURNOUT_10000NA,
	AD4170_BURNOUT_MAX
};


enum ad4170_filter_mode {
	AD4170_FILTER_SINC5_AVG,
	AD4170_FILTER_SINC5 = 4,
	AD4170_FILTER_SINC3,
};

struct ad4170_chan_info {
	bool live;
	unsigned int cfg_slot;
	enum ad4170_ref_select refsel;
	bool bipolar;
	bool buf_positive;
	bool buf_negative;
	unsigned int vref_mv;
	unsigned int pga_bits;
	unsigned int odr;
	unsigned int odr_sel_bits;
	unsigned int filter_type;
};


struct ad4170_state {
	struct regmap			*regmap; 
	struct spi_device		*spi; 
	struct clk			*mclk; //common
	struct regulator_bulk_data	regulators[4]; //struct regulator*
	struct mutex 			 lock; //common
	struct ad4170_chan_info 	*chan_info; //common - dynamic vs static
	struct iio_chan_spec		*chans;
	unsigned int num_channels;

	u16			reg_read_tx_buf;
	u8			reg_write_tx_buf[5];
	u8			reg_read_rx_buf[3];
	
};


static int ad4170_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4170_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}


static const struct iio_chan_spec ad4170_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.differential = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_OFFSET) |
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_SCALE) |
					BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.scan_type = {
		.sign = 'u',
		.endianness = IIO_BE,
	},
};

static const struct iio_info ad4170_info = {
	// .read_raw = ad4170_read_raw,
	// .read_avail = ad4170_read_avail,
	// .write_raw = ad4170_write_raw,
	// .update_scan_mode = ad4170_update_scan_mode,
	 .debugfs_reg_access = ad4170_reg_access,
};


static int ad4170_soft_reset(struct ad4170_state *st)
{
	int ret;
	unsigned int reg = AD4170_INTERFACE_CONFIG_A_REG;

	ret = regmap_write(st->regmap, reg, AD4170_SW_RESET_MSK);
	if (ret){
		return ret;
	}

	/* AD4170-4 requires a minimum of 1 ms between any reset event and 
	a register read/write transaction */
	fsleep(1000);
	pr_err("%s: %d\n", __FUNCTION__, __LINE__);

	return 0;
}

static int ad4170_get_reg_size(struct ad4170_state *st, unsigned int reg,
			       unsigned int *size)
{
	if (reg >= ARRAY_SIZE(ad4170_reg_size))
		return -EINVAL;

	*size = ad4170_reg_size[reg];

	return 0;
}

static int ad4170_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4170_state *st = context;
	unsigned int size;
	int ret;

	ret = ad4170_get_reg_size(st, reg, &size);
	pr_err("%s: %d size=%d\n", __FUNCTION__, __LINE__,size);
	if (ret)
		return ret;

	st->reg_write_tx_buf[0] = (reg >> 8) & 0xFF; 
	st->reg_write_tx_buf[1] = reg & 0xFF; 

	switch (size) {
	case 3:
		put_unaligned_be24(val, &st->reg_write_tx_buf[2]);
		break;
	case 2:
		put_unaligned_be16(val, &st->reg_write_tx_buf[2]);
		break;
	case 1:
		st->reg_write_tx_buf[2] = val;
		break;
	default:
		return -EINVAL;
	}

	return spi_write(st->spi, st->reg_write_tx_buf, size + 2);
}

static int ad4170_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4170_state *st = context;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->reg_read_tx_buf,
			.len = 2,
		},
		{
			.rx_buf = st->reg_read_rx_buf,
		},
	};
	unsigned int size;
	int ret;

	ret = ad4170_get_reg_size(st, reg, &size);
	pr_err("%s: %d size=%d\n", __FUNCTION__, __LINE__,size);
	if (ret) 
		return ret;
	

	st->reg_read_tx_buf = AD4170_READ_MASK | reg;
	t[1].len = size;

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	switch (size) {
	case 3:
		*val = get_unaligned_be24(st->reg_read_rx_buf);
		break;
	case 2:
		*val = get_unaligned_be16(st->reg_read_rx_buf);
		break;
	case 1:
		*val = st->reg_read_rx_buf[0];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct regmap_config ad4170_regmap_config = {
	.reg_read = ad4170_reg_read,
	.reg_write = ad4170_reg_write,
};


static void ad4170_clk_disable_unprepare(void *clk)
{
	clk_disable_unprepare(clk);
}

static int ad4170_setup(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *child;
	int ret;


	ret = clk_prepare_enable(st->mclk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ad4170_clk_disable_unprepare,
				       st->mclk);
	if (ret)
		return ret;
	
	return 0;
	
}


static int ad4170_parse_fw_channel(struct iio_dev *indio_dev,
				   struct fwnode_handle *child)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct ad4170_chan_info *chan_info;
	struct iio_chan_spec *chan;
	u32 pins[2];
	int ret;
	unsigned int chan_index;;

	ret = fwnode_property_read_u32(child, "reg", &chan_index);
	if (ret)
		return ret;

	if (chan_index >= indio_dev->num_channels) {
			dev_err(indio_dev->dev.parent,
				"Channel index >= number of channels\n");
			ret = -EINVAL;
			return ret;
	}

	
	chan_info->bipolar = fwnode_property_read_bool(child, "bipolar");
		

	chan = &st->chans[chan_index];
	chan_info = &st->chan_info[chan_index];

	*chan = ad4170_channel_template;

	ret = fwnode_property_read_u32_array(child, "diff-channels", pins,
					     ARRAY_SIZE(pins));
	if (ret)
		return ret;

	chan->channel = pins[0];
	chan->channel2 = pins[1];


	// ret = ad4170_parse_fw_setup(st, child, &chan_info->setup);
	// if (ret)
	// 	return ret;


	return 0;

	
}

static int ad4170_parse_fw(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *chan;
	struct fwnode_handle *child;
	unsigned int num_channels;
	int ret;

	st->mclk = devm_clk_get_optional(dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);

	num_channels = device_get_child_node_count(dev);
	if (!num_channels)
		return dev_err_probe(&indio_dev->dev, -ENODEV,
				     "no channels defined\n");

	st->chans = devm_kcalloc(dev, num_channels, sizeof(*st->chans), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	st->chan_info = devm_kcalloc(dev, st->num_channels, 
				     sizeof(*st->chan_info), GFP_KERNEL);
	if (!st->chan_info )
		return -ENOMEM;

	indio_dev->channels = st->chans;
	indio_dev->num_channels = num_channels;

	device_for_each_child_node(dev, child) {
		ret = ad4170_parse_fw_channel(indio_dev, child);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}
	}

	return 0;
}

static void ad4170_disable_regulators(void *data)
{
	struct ad4170_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

static int ad4170_probe(struct spi_device *spi)
{

	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4170_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);
	st->spi = spi;

	indio_dev->name = AD4170_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4170_info;

	st->regmap = devm_regmap_init(dev, NULL, st, &ad4170_regmap_config);

	st->regulators[0].supply = "avdd";
	st->regulators[1].supply = "iovdd";
	st->regulators[2].supply = "refin1";
	st->regulators[3].supply = "refin2";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad4170_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	ret = ad4170_soft_reset(st);
	if (ret) 
		return ret;


	ret = ad4170_parse_fw(indio_dev);
	if (ret)
		return ret;

	// ret = ad4170_setup(indio_dev);
	// if (ret)
	// 	return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad4170_of_match[] = {
	{
		.compatible = "adi,ad4170",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4170_of_match);

static struct spi_driver ad4170_driver = {
	.driver = {
		.name = AD4170_NAME,
		.of_match_table = ad4170_of_match,
	},
	.probe = ad4170_probe,
};
module_spi_driver(ad4170_driver);

MODULE_AUTHOR("Ana-Maria Cusco <ana-maria.cuso@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4170 SPI driver");
MODULE_LICENSE("GPL");