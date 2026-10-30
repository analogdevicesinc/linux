// SPDX-License-Identifier: GPL-2.0-only
/*
 * Benewake TF-Luna LiDAR distance sensor
 *
 * Copyright (C) 2026 Vidhu Sarwal <vidhu.linux@gmail.com>
 *
 * Datasheet: https://en.benewake.com/uploadfiles/2025/04/20250430174501874.pdf
 *
 * The I2C register map is in appendix III of the TF-Luna product manual
 * (SJ-PM-TF-Luna A05). Default 7-bit I2C address is 0x10.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/time.h>
#include <linux/types.h>

#include <asm/byteorder.h>

#include <linux/iio/iio.h>

#define TF_LUNA_REG_DIST		0x00
#define TF_LUNA_REG_TICK		0x06
#define TF_LUNA_REG_VERSION		0x0A
#define TF_LUNA_REG_MODE		0x23
#define TF_LUNA_REG_TRIG_ONE_SHOT	0x24
#define TF_LUNA_REG_SIGNATURE		0x3C

#define TF_LUNA_MODE_TRIGGER		0x01
#define TF_LUNA_TRIG_ONE_SHOT		0x01

#define TF_LUNA_SIGN			0x4c554e41	/* "LUNA" in ASCII */

/* Below this the sensor reports a dummy distance of 0 */
#define TF_LUNA_AMP_MIN			100
#define TF_LUNA_AMP_OVEREXPOSED		0xFFFF

/* A measurement takes 10-20 ms in trigger mode */
#define TF_LUNA_POLL_US			(2 * USEC_PER_MSEC)
#define TF_LUNA_TIMEOUT_US		(100 * USEC_PER_MSEC)

struct tf_luna_data {
	struct regmap *regmap;
	/* Serialises the trigger, poll and read of a single measurement */
	struct mutex lock;
};

/*
 * No register cache: the only configuration register written is MODE,
 * once in probe, the other write is the one-shot trigger, and the
 * registers read afterwards hold measurement data that keeps changing.
 * MODE is also read back from the hardware, since the sensor can
 * silently ignore a write that comes right after another one.
 */
static const struct regmap_config tf_luna_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = TF_LUNA_REG_SIGNATURE + 3,
};

static int tf_luna_read_tick(struct tf_luna_data *data, u16 *tick)
{
	__le16 val;
	int ret;

	ret = regmap_bulk_read(data->regmap, TF_LUNA_REG_TICK, &val, sizeof(val));
	if (ret)
		return ret;

	*tick = le16_to_cpu(val);
	return 0;
}

static int tf_luna_measure(struct tf_luna_data *data, int *dist)
{
	struct regmap *map = data->regmap;
	__le16 buf[2];	/* distance in cm, signal strength */
	u16 old_tick, tick, amp;
	int ret, err;

	guard(mutex)(&data->lock);

	ret = tf_luna_read_tick(data, &old_tick);
	if (ret)
		return ret;

	ret = regmap_write(map, TF_LUNA_REG_TRIG_ONE_SHOT, TF_LUNA_TRIG_ONE_SHOT);
	if (ret)
		return ret;

	/* The tick register is a timestamp, it changes with each measurement */
	ret = read_poll_timeout(tf_luna_read_tick, err, err || tick != old_tick,
				TF_LUNA_POLL_US, TF_LUNA_TIMEOUT_US, true,
				data, &tick);
	if (ret)
		return ret;
	if (err)
		return err;

	ret = regmap_bulk_read(map, TF_LUNA_REG_DIST, buf, sizeof(buf));
	if (ret)
		return ret;

	amp = le16_to_cpu(buf[1]);
	if (amp < TF_LUNA_AMP_MIN || amp == TF_LUNA_AMP_OVEREXPOSED)
		return -ENODATA;

	*dist = le16_to_cpu(buf[0]);
	return 0;
}

static int tf_luna_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct tf_luna_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = tf_luna_measure(data, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* Distance is in cm */
		*val = 0;
		*val2 = 10000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info tf_luna_info = {
	.read_raw = tf_luna_read_raw,
};

static const struct iio_chan_spec tf_luna_channels[] = {
	{
		.type = IIO_DISTANCE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int tf_luna_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct tf_luna_data *data;
	struct iio_dev *indio_dev;
	struct regmap *map;
	unsigned int mode;
	__be32 sign;
	u32 fourcc;
	u8 ver[3];
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &data->lock);
	if (ret)
		return ret;

	map = devm_regmap_init_i2c(client, &tf_luna_regmap_config);
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "Failed to initialize regmap\n");
	data->regmap = map;

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable vcc regulator\n");

	/* Boot time is not documented, about 350 ms was measured */
	msleep(500);

	ret = regmap_bulk_read(map, TF_LUNA_REG_SIGNATURE, &sign, sizeof(sign));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read signature\n");

	fourcc = be32_to_cpu(sign);
	if (fourcc != TF_LUNA_SIGN)
		dev_info(dev, "Unknown signature %p4ch, continuing\n", &fourcc);

	ret = regmap_bulk_read(map, TF_LUNA_REG_VERSION, ver, sizeof(ver));
	if (ret)
		return ret;

	dev_dbg(dev, "Firmware version %u.%u.%u\n", ver[2], ver[1], ver[0]);

	/*
	 * In continuous mode a read can catch the sensor updating its result
	 * registers unless it is synchronised to the data ready output. Use
	 * trigger mode and start each measurement on demand instead.
	 */
	ret = regmap_write(map, TF_LUNA_REG_MODE, TF_LUNA_MODE_TRIGGER);
	if (ret)
		return ret;

	/*
	 * The sensor silently drops writes it is not ready for, so check that
	 * the mode was set. A write takes up to 100 ms to read back.
	 */
	msleep(100);

	ret = regmap_read(map, TF_LUNA_REG_MODE, &mode);
	if (ret)
		return ret;

	if (mode != TF_LUNA_MODE_TRIGGER)
		return dev_err_probe(dev, -EIO, "Failed to set trigger mode\n");

	indio_dev->name = "tf-luna";
	indio_dev->info = &tf_luna_info;
	indio_dev->channels = tf_luna_channels;
	indio_dev->num_channels = ARRAY_SIZE(tf_luna_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id tf_luna_id[] = {
	{ .name = "tf-luna" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tf_luna_id);

static const struct of_device_id tf_luna_of_match[] = {
	{ .compatible = "benewake,tf-luna" },
	{ }
};
MODULE_DEVICE_TABLE(of, tf_luna_of_match);

static struct i2c_driver tf_luna_driver = {
	.driver = {
		.name = "tf-luna",
		.of_match_table = tf_luna_of_match,
	},
	.probe = tf_luna_probe,
	.id_table = tf_luna_id,
};
module_i2c_driver(tf_luna_driver);

MODULE_AUTHOR("Vidhu Sarwal <vidhu.linux@gmail.com>");
MODULE_DESCRIPTION("Benewake TF-Luna LiDAR distance sensor driver");
MODULE_LICENSE("GPL");
