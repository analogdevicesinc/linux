// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Sensirion SDP31, SDP32 and SDP33 differential pressure sensors
 *
 * Datasheet:
 * https://sensirion.com/media/documents/4D045D69/6375F34F/DP_DS_SDP3x_digital_D1.pdf
 */
#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/once.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#include <linux/iio/iio.h>

#define SDP31_CMD_TRIG_DP     0x362F
#define SDP31_MEAS_DELAY_MS   50
#define SDP31_POWERUP_TIME_MS 25   /* datasheet t_PU (max) */
#define SDP31_CRC8_POLY       0x31
#define SDP31_CRC8_INIT       0xff

DECLARE_CRC8_TABLE(sdp31_crc8_table);

struct sdp31_data {
	struct i2c_client *client;
	struct mutex lock; /* serializes access to the sensor */
	u16 dp_scale;
};

static int sdp31_send_cmd(struct i2c_client *client, u16 cmd)
{
	__be16 buf = cpu_to_be16(cmd);
	int ret;

	ret = i2c_master_send(client, (u8 *)&buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return (ret == sizeof(buf)) ? 0 : -EIO;
}

static int sdp31_check_crc(const u8 *word)
{
	if (crc8(sdp31_crc8_table, word, 2, SDP31_CRC8_INIT) != word[2])
		return -EIO;

	return 0;
}

/*
 * Trigger a differential-pressure measurement and read the first len bytes
 * of the result. A short read is ended by the usual I2C NACK + stop, so
 * reading fewer than the full nine bytes is fine.
 */
static int sdp31_trigger_read(struct sdp31_data *data, u8 *rx, int len)
{
	int ret;

	guard(mutex)(&data->lock);

	ret = sdp31_send_cmd(data->client, SDP31_CMD_TRIG_DP);
	if (ret)
		return ret;

	msleep(SDP31_MEAS_DELAY_MS);

	ret = i2c_master_recv(data->client, rx, len);
	if (ret < 0)
		return ret;

	return (ret == len) ? 0 : -EIO;
}

static int sdp31_read_measurement(struct sdp31_data *data,
				  s16 *pressure, s16 *temp)
{
	u8 rx[6];
	int ret;

	ret = sdp31_trigger_read(data, rx, sizeof(rx));
	if (ret)
		return ret;

	if (sdp31_check_crc(&rx[0]) || sdp31_check_crc(&rx[3]))
		return -EIO;

	*pressure = get_unaligned_be16(&rx[0]);
	*temp = get_unaligned_be16(&rx[3]);

	return 0;
}

static int sdp31_read_scale(struct sdp31_data *data, u16 *scale)
{
	u8 rx[9];
	int ret;

	ret = sdp31_trigger_read(data, rx, sizeof(rx));
	if (ret)
		return ret;

	if (sdp31_check_crc(&rx[6]))
		return -EIO;

	*scale = get_unaligned_be16(&rx[6]);

	return 0;
}

static int sdp31_read_raw(struct iio_dev *indio_dev,
			  struct iio_chan_spec const *chan,
			  int *val, int *val2, long mask)
{
	struct sdp31_data *data = iio_priv(indio_dev);
	s16 pressure, temp;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = sdp31_read_measurement(data, &pressure, &temp);
		if (ret)
			return ret;
		switch (chan->type) {
		case IIO_PRESSURE:
			*val = pressure;
			return IIO_VAL_INT;
		case IIO_TEMP:
			*val = temp;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_PRESSURE:
			*val = 1;
			*val2 = data->dp_scale * KILO;
			return IIO_VAL_FRACTIONAL;
		case IIO_TEMP:
			/* raw / 200 = degC, so raw * 5 = milli-degC */
			*val = 5;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static const struct iio_info sdp31_info = {
	.read_raw = sdp31_read_raw,
};

static const struct iio_chan_spec sdp31_channels[] = {
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int sdp31_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct sdp31_data *data;
	int ret;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulator\n");

	msleep(SDP31_POWERUP_TIME_MS);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;

	ret = devm_mutex_init(dev, &data->lock);
	if (ret)
		return ret;

	DO_ONCE(crc8_populate_msb, sdp31_crc8_table, SDP31_CRC8_POLY);

	ret = sdp31_read_scale(data, &data->dp_scale);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read from sensor\n");
	if (!data->dp_scale)
		return dev_err_probe(dev, -EINVAL, "invalid scale factor\n");

	indio_dev->name = "sdp31";
	indio_dev->info = &sdp31_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = sdp31_channels;
	indio_dev->num_channels = ARRAY_SIZE(sdp31_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id sdp31_id[] = {
	{ .name = "sdp31" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sdp31_id);

static const struct of_device_id sdp31_of_match[] = {
	{ .compatible = "sensirion,sdp31" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdp31_of_match);

static struct i2c_driver sdp31_driver = {
	.driver = {
		.name = "sdp31",
		.of_match_table = sdp31_of_match,
	},
	.probe = sdp31_probe,
	.id_table = sdp31_id,
};
module_i2c_driver(sdp31_driver);

MODULE_AUTHOR("Muhammad Abu Bakar");
MODULE_DESCRIPTION("Sensirion SDP3x differential pressure sensors");
MODULE_LICENSE("GPL");
