// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Texas Instruments TLA2528 ADC
 *
 * Copyright (C) 2020-2021 Rodolfo Giometti <giometti@enneenne.com>
 * Copyright (C) 2025 Maxime Chevallier <maxime.chevallier@bootlin.com>
 * Datasheet: https://www.ti.com/lit/gpn/tla2528
 */

#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/units.h>

#include <asm/byteorder.h>

#include <linux/iio/adc-helpers.h>
#include <linux/iio/iio.h>

#define TLA2528_OP_WRITE_REG		0x08
#define TLA2528_OP_READ_REG		0x10

#define TLA2528_GENERAL_CFG_ADR		0x01
#define TLA2528_GENERAL_CFG_RST		BIT(0)
#define TLA2528_CHANNEL_SEL_ADR		0x11
#define TLA2528_CHANNEL_SEL_MANUAL_CHID	GENMASK(3, 0)
#define TLA2528_AUTO_SEQ_CH_SEL_ADR	0x12

#define TLA2528_NUM_CHANNELS		8

struct tla2528 {
	struct i2c_client *client;
	struct regmap *regmap;
	int vref_mV;

	/* Protects manual channel selection */
	struct mutex lock;
};

static bool tla2528_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg == TLA2528_GENERAL_CFG_ADR;
}

static const struct regmap_config tla2528_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = TLA2528_OP_READ_REG,
	.write_flag_mask = TLA2528_OP_WRITE_REG,
	.max_register = TLA2528_AUTO_SEQ_CH_SEL_ADR,
	.volatile_reg = tla2528_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

static int tla2528_read(struct tla2528 *tla2528, u8 channel, int *val)
{
	__be16 data;
	int ret;

	guard(mutex)(&tla2528->lock);

	ret = regmap_update_bits(tla2528->regmap, TLA2528_CHANNEL_SEL_ADR,
				 TLA2528_CHANNEL_SEL_MANUAL_CHID, channel);
	if (ret)
		return ret;

	ret = i2c_master_recv(tla2528->client, (char *)&data, sizeof(data));
	if (ret < 0)
		return ret;
	if (ret != sizeof(data))
		return -EIO;

	*val = be16_to_cpu(data) >> 4;

	return 0;
}

static int tla2528_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct tla2528 *tla2528 = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = tla2528_read(tla2528, chan->channel, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = tla2528->vref_mV;
		*val2 = 12;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}
}

#define TLA2528_CHAN(chan) {					\
	.type = IIO_VOLTAGE,					\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	.indexed = 1,						\
}

static const struct iio_chan_spec tla2528_chan_template = TLA2528_CHAN(0);

static const struct iio_info tla2528_info = {
	.read_raw = tla2528_read_raw,
};

static int tla2528_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_chan_spec *channels;
	struct iio_dev *indio_dev;
	struct tla2528 *tla2528;
	struct regmap *map;
	int ret;

	/*
	 * We're reading the data with raw i2c reads, without passing an
	 * address like we would for smbus-style accesses.
	 */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	if (i2c_check_quirks(client->adapter, I2C_AQ_NO_CLK_STRETCH))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter does not support clock stretching\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*tla2528));
	if (!indio_dev)
		return -ENOMEM;

	tla2528 = iio_priv(indio_dev);
	tla2528->client = client;

	map = devm_regmap_init_i2c(client, &tla2528_regmap_config);
	if (IS_ERR(map))
		return PTR_ERR(map);

	tla2528->regmap = map;

	ret = devm_mutex_init(dev, &tla2528->lock);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(dev, "dvdd");
	if (ret)
		return ret;

	ret = devm_regulator_get_enable_read_voltage(dev, "avdd");
	if (ret < 0)
		return ret;

	tla2528->vref_mV = ret / (MICRO / MILLI);

	ret = devm_iio_adc_device_alloc_chaninfo_se(dev, &tla2528_chan_template,
						    TLA2528_NUM_CHANNELS - 1,
						    &channels);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get ADC channels\n");

	indio_dev->channels = channels;
	indio_dev->num_channels = ret;

	indio_dev->name = "tla2528";
	indio_dev->info = &tla2528_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = regmap_write(map, TLA2528_GENERAL_CFG_ADR, TLA2528_GENERAL_CFG_RST);
	if (ret)
		return ret;

	fsleep(5 * USEC_PER_MSEC);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id tla2528_id[] = {
	{ .name = "tla2528" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tla2528_id);

static const struct of_device_id tla2528_of_match[] = {
	{ .compatible = "ti,tla2528" },
	{ }
};
MODULE_DEVICE_TABLE(of, tla2528_of_match);

static struct i2c_driver tla2528_driver = {
	.driver = {
		.name = "tla2528",
		.of_match_table = tla2528_of_match,
	},
	.probe = tla2528_probe,
	.id_table = tla2528_id,
};
module_i2c_driver(tla2528_driver);

MODULE_AUTHOR("Maxime Chevallier <maxime.chevallier@bootlin.com>");
MODULE_AUTHOR("Rodolfo Giometti <giometti@enneenne.com>");
MODULE_DESCRIPTION("Texas Instruments TLA2528 ADC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_DRIVER");
