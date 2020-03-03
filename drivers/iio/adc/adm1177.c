/*
 * ADM1177 Hot Swap Controller and Digital Power Monitor with Soft Start Pin
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

/*  Command Byte Operations */
#define ADM1177_CMD_V_CONT	BIT(0)
#define ADM1177_CMD_V_ONCE	BIT(1)
#define ADM1177_CMD_I_CONT	BIT(2)
#define ADM1177_CMD_I_ONCE	BIT(3)
#define ADM1177_CMD_VRANGE	BIT(4)
#define ADM1177_CMD_STATUS_RD	BIT(6)

/* Extended Register */
#define ADM1177_REG_ALERT_EN	1
#define ADM1177_REG_ALERT_TH	2
#define ADM1177_REG_CONTROL	3

/* ADM1177_REG_ALERT_EN */
#define ADM1177_EN_ADC_OC1	BIT(0)
#define ADM1177_EN_ADC_OC4	BIT(1)
#define ADM1177_EN_HS_ALERT	BIT(2)
#define ADM1177_EN_OFF_ALERT	BIT(3)
#define ADM1177_CLEAR		BIT(4)

/* ADM1177_REG_CONTROL */
#define ADM1177_SWOFF		BIT(0)

#define ADM1177_BITS		12

struct adm1177_chip_info {
	struct i2c_client	*client;
	struct regulator	*reg;
	u8			command;
	u32			r_sense_mohm;
	u32			alert_threshold_ma;
	bool			vrange_high;
};

static int adm1177_read(struct adm1177_chip_info *chip, u8 num, u8 *data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	ret = i2c_master_recv(client, data, num);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	return 0;
}

static int adm1177_write_cmd(struct adm1177_chip_info *chip, u8 cmd)
{
	chip->command = cmd;
	return i2c_smbus_write_byte(chip->client, cmd);
}

static int adm1177_write_reg(struct adm1177_chip_info *chip, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(chip->client, reg, val);
}

static int adm1177_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct adm1177_chip_info *chip = iio_priv(indio_dev);
	u8 data[3];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		adm1177_read(chip, 3, data);
		switch (chan->type) {
		case IIO_VOLTAGE:
			*val = (data[0] << 4) | (data[2] >> 4);
			return IIO_VAL_INT;
		case IIO_CURRENT:
			*val = (data[1] << 4) | (data[2] & 0xF);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			if (chip->command & ADM1177_CMD_VRANGE)
				*val = 6650;
			else
				*val = 26350;

			*val2 = ADM1177_BITS;
			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_CURRENT:
			*val = 105840 / chip->r_sense_mohm;

			*val2 = ADM1177_BITS;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec adm1177_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = 0,
	},
	{
		.type = IIO_CURRENT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE),
		.indexed = 1,
		.channel = 0,
	},
};

static const struct iio_info adm1177_info = {
	.read_raw = &adm1177_read_raw,
};

static int adm1177_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct adm1177_chip_info *chip;
	struct iio_dev *indio_dev;

	if (!client->dev.of_node && !client->dev.platform_data) {
		dev_err(&client->dev, "No device tree node or pdata available.\n");
		return -EINVAL;
	}

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!indio_dev)
		return -ENOMEM;

	chip = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	chip->client = client;

	if (client->dev.of_node) {
		struct device_node *np = client->dev.of_node;

		of_property_read_u32(np, "adi,r-sense-mohm", &chip->r_sense_mohm);
		of_property_read_u32(np, "adi,shutdown-threshold-ma",
				     &chip->alert_threshold_ma);
		chip->vrange_high = of_property_read_bool(np,
					"adi,vrange-high-enable");
	} else {
		u32 *pdata = client->dev.platform_data; /* FIXME later */
		chip->r_sense_mohm = pdata[0];
		chip->alert_threshold_ma = pdata[1];
		chip->vrange_high = pdata[2];			
	}

	if (chip->alert_threshold_ma) {
		unsigned val;

		val = (chip->alert_threshold_ma * chip->r_sense_mohm * 0xFF) / 105840U;
		if (val > 0xFF) {
			dev_err(&client->dev,
				"Requested shutdown current %d mA above limit\n",
				chip->alert_threshold_ma);

			val = 0xFF;
		}
		adm1177_write_reg(chip, ADM1177_REG_ALERT_TH, val);
	}

	adm1177_write_cmd(chip, ADM1177_CMD_V_CONT |
				ADM1177_CMD_I_CONT |
				(chip->vrange_high ? 0 : ADM1177_CMD_VRANGE));

	indio_dev->name = id->name;
	indio_dev->channels = adm1177_channels;
	indio_dev->num_channels = ARRAY_SIZE(adm1177_channels);

	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &adm1177_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	return iio_device_register(indio_dev);
}

static int adm1177_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct i2c_device_id adm1177_ids[] = {
	{ "adm1177", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adm1177_ids);

static const struct of_device_id adm1177_dt_ids[] = {
	{ .compatible = "adi,adm1177" },
	{},
};
MODULE_DEVICE_TABLE(of, nau7802_dt_ids);

static struct i2c_driver adm1177_driver = {
	.driver = {
		.name = "adm1177-iio",
		.of_match_table = adm1177_dt_ids,
	},
	.probe = adm1177_probe,
	.remove = adm1177_remove,
	.id_table = adm1177_ids,
};
module_i2c_driver(adm1177_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADM1177 ADC driver");
MODULE_LICENSE("GPL v2");
