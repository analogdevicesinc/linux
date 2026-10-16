// SPDX-License-Identifier: GPL-2.0+
/*
 * Vishay VCNL36829 Proximity Sensor driver
 *
 * Copyright (C) 2026 Liu Yufei <lyf98405@gmail.com>
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/device/devres.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h>
#include <linux/types.h>

#include <linux/iio/iio.h>

#define VCNL36829_PS_CONF1 0x00
#define VCNL36829_PS_ON_MSK BIT(0)
#define VCNL36829_PS_SD_MSK BIT(8)

#define VCNL36829_PS_CONF2 0x01
#define VCNL36829_PS_IT_MSK GENMASK(11, 8)

#define VCNL36829_PS_CONF3 0x02
#define VCNL36829_PS_CURRENT_EN_MSK BIT(3)
#define VCNL36829_PS_CURRENT_MSK GENMASK(2, 0)
#define VCNL36829_PD1_EN_MSK BIT(8)
#define VCNL36829_PD2_EN_MSK BIT(9)
#define VCNL36829_PD3_EN_MSK BIT(10)

#define VCNL36829_PS_CONF5 0x08

#define VCNL36829_DATA 0xF8
#define VCNL36829_DEV_ID 0xFA

#define VCNL36829_PS_CONF5_INIT 0x00A0
#define VCNL36829_PS_CURRENT_18MA 0x7
#define VCNL36829_PS_INT_TIME_25 0x1
#define VCNL36829_DEV_ID_VAL 0x29

static const int vcnl36829_it_range[] = { 0, 25, 0, 25, 0, 375 };

struct vcnl36829_data {
	struct regmap *regmap;
};

static bool vcnl36829_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case VCNL36829_DATA:
	case VCNL36829_DEV_ID:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config vcnl36829_regmap_config = {
	.name = "vcnl36829_regmap",
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = VCNL36829_DEV_ID,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
	.volatile_reg = vcnl36829_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

static int vcnl36829_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2, long mask)
{
	struct vcnl36829_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	struct regmap *regmap = data->regmap;
	unsigned int reg;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(regmap, chan->address, &reg);
		if (ret) {
			dev_err(dev, "Data read failed: %d\n", ret);
			return ret;
		}
		*val = reg;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_INT_TIME:
		ret = regmap_read(regmap, VCNL36829_PS_CONF2, &reg);
		if (ret) {
			dev_err(dev, "Data read failed: %d\n", ret);
			return ret;
		}
		/*
		 * PS_IT field 0 would mean 12.5us, which does not fit the
		 * 25us step used here. The driver never programs field 0:
		 * probe sets 25us and write_raw rejects anything below 25us,
		 * so the field is always >= 1 and val2 = field * 25 holds.
		 */
		*val = 0;
		*val2 = FIELD_GET(VCNL36829_PS_IT_MSK, reg) * 25;
		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static int vcnl36829_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val, int val2, long mask)
{
	struct vcnl36829_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME: {
		int ival = val2 / 25;
		int fval = val2 % 25;

		if (val != 0)
			return -EINVAL;
		if (ival < 1 || ival > 15)
			return -EINVAL;
		if (fval != 0)
			return -EINVAL;
		return regmap_update_bits(data->regmap,
					  VCNL36829_PS_CONF2,
					  VCNL36829_PS_IT_MSK,
					  FIELD_PREP(VCNL36829_PS_IT_MSK, ival));
	}

	default:
		return -EINVAL;
	}
}

static int vcnl36829_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type, int *length,
				long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		*length = 6;
		*vals = vcnl36829_it_range;
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_RANGE;

	default:
		return -EINVAL;
	}
}

static const struct iio_info vcnl36829_info = {
	.read_raw = vcnl36829_read_raw,
	.write_raw = vcnl36829_write_raw,
	.read_avail = vcnl36829_read_avail,
};

static const struct iio_chan_spec vcnl36829_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.address = VCNL36829_DATA,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_INT_TIME),
		.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_INT_TIME),
	}
};

static void vcnl36829_shutdown_action(void *data)
{
	struct vcnl36829_data *vcnl36829_data = data;

	regmap_update_bits(vcnl36829_data->regmap,
			   VCNL36829_PS_CONF1,
			   VCNL36829_PS_ON_MSK | VCNL36829_PS_SD_MSK,
			   FIELD_PREP(VCNL36829_PS_ON_MSK, 0) |
			   FIELD_PREP(VCNL36829_PS_SD_MSK, 1));
}

static int vcnl36829_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct vcnl36829_data *data;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	unsigned int reg;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_i2c(client, &vcnl36829_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Regmap setup failed\n");

	data = iio_priv(indio_dev);
	data->regmap = regmap;

	indio_dev->name = "vcnl36829";
	indio_dev->info = &vcnl36829_info;
	indio_dev->channels = vcnl36829_channels;
	indio_dev->num_channels = ARRAY_SIZE(vcnl36829_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vdd\n");

	ret = devm_regulator_get_enable(dev, "vddio");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable vddio\n");

	/*
	 * The datasheet does not specify a power-up settling time.
	 * Wait 3 ms after enabling the supplies before accessing the
	 * device, as a conservative value.
	 */
	fsleep(3 * USEC_PER_MSEC);

	ret = regmap_read(regmap, VCNL36829_DEV_ID, &reg);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read device ID\n");
	if ((reg & 0xFF) != VCNL36829_DEV_ID_VAL)
		dev_info(dev, "Unknown device ID 0x%x\n", reg & 0xFF);

	ret = regmap_update_bits(regmap, VCNL36829_PS_CONF2,
				 VCNL36829_PS_IT_MSK,
				 FIELD_PREP(VCNL36829_PS_IT_MSK, VCNL36829_PS_INT_TIME_25));
	if (ret)
		return dev_err_probe(dev, ret, "Could not configure PS_IT\n");

	ret = regmap_update_bits(regmap, VCNL36829_PS_CONF3,
				 VCNL36829_PS_CURRENT_EN_MSK |
				 VCNL36829_PS_CURRENT_MSK |
				 VCNL36829_PD1_EN_MSK |
				 VCNL36829_PD2_EN_MSK |
				 VCNL36829_PD3_EN_MSK,
				 FIELD_PREP(VCNL36829_PS_CURRENT_EN_MSK, 1) |
				 FIELD_PREP(VCNL36829_PS_CURRENT_MSK, VCNL36829_PS_CURRENT_18MA) |
				 FIELD_PREP(VCNL36829_PD1_EN_MSK, 1) |
				 FIELD_PREP(VCNL36829_PD2_EN_MSK, 1) |
				 FIELD_PREP(VCNL36829_PD3_EN_MSK, 1));
	if (ret)
		return dev_err_probe(dev, ret, "Could not configure VCSEL/PD\n");

	ret = regmap_write(regmap, VCNL36829_PS_CONF5, VCNL36829_PS_CONF5_INIT);
	if (ret)
		return dev_err_probe(dev, ret, "Could not set PS_CONF5\n");

	ret = regmap_update_bits(regmap, VCNL36829_PS_CONF1,
				 VCNL36829_PS_ON_MSK | VCNL36829_PS_SD_MSK,
				 FIELD_PREP(VCNL36829_PS_ON_MSK, 1) |
				 FIELD_PREP(VCNL36829_PS_SD_MSK, 0));
	if (ret)
		return dev_err_probe(dev, ret, "Could not set initial config\n");

	ret = devm_add_action_or_reset(dev, vcnl36829_shutdown_action, data);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id vcnl36829_id_table[] = {
	{ .name = "vcnl36829" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, vcnl36829_id_table);

static const struct of_device_id vcnl36829_of_match[] = {
	{ .compatible = "vishay,vcnl36829" },
	{ }
};
MODULE_DEVICE_TABLE(of, vcnl36829_of_match);

static struct i2c_driver vcnl36829_driver = {
	.probe = vcnl36829_probe,
	.id_table = vcnl36829_id_table,
	.driver = {
		.name = "vcnl36829",
		.of_match_table = vcnl36829_of_match,
	},
};
module_i2c_driver(vcnl36829_driver);

MODULE_DESCRIPTION("Vishay VCNL36829 proximity sensor driver");
MODULE_AUTHOR("Liu Yufei <lyf98405@gmail.com>");
MODULE_LICENSE("GPL");
