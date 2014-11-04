/*
 * AD5270, AD5271, AD5272, AD5274 Digital Potentiometers driver
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>

#define AD5270_CMD(x)    (((x) & 0x03F) << 10)
#define AD5270_DATA(x)   (((x) & 0x3FF) <<  0)

#define AD5270_CMD_NOP           0
#define AD5270_CMD_RDAC_WR       1
#define AD5270_CMD_RDAC_RD       2
#define AD5270_CMD_50TP_STORE    3
#define AD5270_CMD_SW_RST        4
#define AD5270_CMD_50TP_READ     5
#define AD5270_CMD_50TP_ADDR_RD  6
#define AD5270_CMD_CTRL_WR       7
#define AD5270_CMD_CTRL_RD       8
#define AD5270_CMD_PWR           9

#define AD5270_REG_CTRL_50TP_PRG_EN    (1 << 0)
#define AD5270_REG_CTRL_RDAC_WR_PRCT   (1 << 1)
#define AD5270_REG_CTRL_RDAC_PERFM_EN  (1 << 2)
#define AD5270_REG_CTRL_50TP_PRG_SUCC  (1 << 3)

#define AD5270_50TP_ADDRESS(x)  ((x) & 0x3F)

#define AD5270_REG_PWR_SHUTDOWN (1 << 0)

#define AD5270_SDO_TRISTATE 0x8001

#define NO_SHIFT 0

#define AD5270_F_SDO_TRISTATE (1 << 0) /* Features Tristate SDO pin */

struct ad5270_state;

/**
 * struct ad5270_chip_info - chip specific information
 * @name:		chip name
 * @channel:		channel specification
 * @num_channels:	number of channels
 * @features:		feature flags that the chip might have
 */

struct ad5270_chip_info {
	char *name;
	const struct iio_chan_spec channel[6];
	unsigned int num_channels;
	unsigned int features;
};

typedef int (*ad5270_write_func)(struct ad5270_state *st,
	unsigned int cmd, unsigned int data, unsigned int data_shift);
typedef int (*ad5270_read_func)(struct ad5270_state *st,
		unsigned int data_shift);

/**
 * struct ad5270_state - driver instance specific data
 * @dev:		the device for this driver instance
 * @chip_info:		chip model specific constants, available modes etc
 * @otp0_enabled:	set to true before programming chip's OTP memory
 * @shutdown:		chip software shutdown status
 * @rperform:		enable state of Resistor-Performance mode
 *
 * @write:		register write callback
 * @read:		register read callback
 */
struct ad5270_state {
	struct device *dev;
	const struct ad5270_chip_info *chip_info;
	bool otp0_enabled;
	bool shutdown;
	bool rperform;
	ad5270_write_func write;
	ad5270_read_func read;
};

enum ad5270_type {
	ID_AD5270,
	ID_AD5271,
	ID_AD5272,
	ID_AD5274,
};

enum ad5270_iio_dev_attr {
	OTP0,
	OTP0_EN,
	SHUTDOWN,
	RPERFORMANCE,
};

static int ad5270_sdo_tristate(struct ad5270_state *st)
{
	unsigned int cmd = AD5270_SDO_TRISTATE >> 10;
	unsigned int data = AD5270_SDO_TRISTATE;
	int ret;

	ret = st->write(st, cmd, data, NO_SHIFT);
	if (ret)
		return ret;

	return st->write(st, AD5270_CMD_NOP, 0, NO_SHIFT);
}

static int ad5270_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct ad5270_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&indio_dev->mlock);
		ret = st->write(st, AD5270_CMD_RDAC_RD, 0, NO_SHIFT);
		mutex_unlock(&indio_dev->mlock);
		if (ret)
			return ret;
		mutex_lock(&indio_dev->mlock);
		*val = st->read(st, chan->scan_type.shift);
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret) {
				mutex_unlock(&indio_dev->mlock);
				return ret;
			}
		}
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	default:
		break;
	}

	return ret;
}

static int ad5270_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct ad5270_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&indio_dev->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = st->write(st, AD5270_CMD_CTRL_WR,
			AD5270_REG_CTRL_RDAC_WR_PRCT |
			AD5270_REG_CTRL_RDAC_PERFM_EN * st->rperform,
			NO_SHIFT);
		if (ret)
			break;
		ret = st->write(st, AD5270_CMD_RDAC_WR, val,
			chan->scan_type.shift);
		if (ret)
			break;
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret)
				break;
		}
		break;
	default:
		break;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad5270_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad5270_state *st = iio_priv(indio_dev);
	int addr, val, ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case OTP0:
		ret = st->write(st, AD5270_CMD_50TP_ADDR_RD, 0, NO_SHIFT);
		if (ret)
			break;
		addr = st->read(st, NO_SHIFT);
		ret = st->write(st, AD5270_CMD_50TP_READ,
				AD5270_50TP_ADDRESS(addr), NO_SHIFT);
		if (ret)
			break;
		val = st->read(st,
			st->chip_info->channel[0].scan_type.shift);
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret)
				break;
		}
		ret = sprintf(buf, "%d\n", val);
		break;
	case OTP0_EN:
		ret = sprintf(buf, "%s\n",
			(st->otp0_enabled) ? "enabled" : "disabled");
		break;
	case SHUTDOWN:
		ret = sprintf(buf, "%s\n",
			(st->shutdown) ? "1" : "0");
		break;
	case RPERFORMANCE:
		ret = st->write(st, AD5270_CMD_CTRL_RD, 0, NO_SHIFT);
		if (ret)
			break;
		val = st->read(st, NO_SHIFT);
		st->rperform = !!(val & AD5270_REG_CTRL_RDAC_PERFM_EN);
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret)
				break;
		}
		ret = sprintf(buf, "%s\n",
			(st->rperform) ? "enabled" : "disabled");
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad5270_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct ad5270_state *st = iio_priv(indio_dev);
	const struct iio_chan_spec *chan;
	long readin;
	int reg, ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case OTP0:
		if (!st->otp0_enabled) {
			ret = -EPERM;
			break;
		}
		ret = kstrtol(buf, 10, &readin);
		chan = &st->chip_info->channel[0];
		ret = st->write(st, AD5270_CMD_CTRL_WR,
			AD5270_REG_CTRL_50TP_PRG_EN |
			AD5270_REG_CTRL_RDAC_WR_PRCT |
			AD5270_REG_CTRL_RDAC_PERFM_EN * st->rperform,
			NO_SHIFT);
		if (ret)
			break;
		ret = st->write(st, AD5270_CMD_RDAC_WR, readin,
			chan->scan_type.shift);
		if (ret)
			break;
		ret = st->write(st, AD5270_CMD_50TP_STORE, 0, NO_SHIFT);
		if (ret)
			break;
		msleep(400);
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret)
				break;
		}
		break;
	case OTP0_EN:
		if (!strncmp(buf, "enabled", sizeof("enabled") - 1))
			st->otp0_enabled = true;
		else
			st->otp0_enabled = false;
		break;
	case SHUTDOWN:
		ret = kstrtol(buf, 10, &readin);
		if (ret)
			break;

		ret = st->write(st, AD5270_CMD_PWR, (readin) ?
				AD5270_REG_PWR_SHUTDOWN : 0, NO_SHIFT);
		if (ret)
			break;
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret)
				break;
		}

		st->shutdown = !!readin;
		break;
	case RPERFORMANCE:
		ret = st->write(st, AD5270_CMD_CTRL_RD, 0, NO_SHIFT);
		if (ret)
			break;
		reg = st->read(st, NO_SHIFT);
		reg &= ~AD5270_REG_CTRL_RDAC_PERFM_EN;

		if (!strncmp(buf, "enabled", sizeof("enabled") - 1))
			reg |= AD5270_REG_CTRL_RDAC_PERFM_EN;

		ret = st->write(st, AD5270_CMD_CTRL_WR, reg, NO_SHIFT);
		if (ret)
			break;
		if (st->chip_info->features & AD5270_F_SDO_TRISTATE) {
			ret = ad5270_sdo_tristate(st);
			if (ret)
				break;
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(otp0, S_IRUGO | S_IWUSR,
			ad5270_show,
			ad5270_store,
			OTP0);

static IIO_DEVICE_ATTR(otp0_en, S_IRUGO | S_IWUSR,
			ad5270_show,
			ad5270_store,
			OTP0_EN);

static IIO_DEVICE_ATTR(shutdown, S_IRUGO | S_IWUSR,
			ad5270_show,
			ad5270_store,
			SHUTDOWN);
static IIO_DEVICE_ATTR(rperformance, S_IRUGO | S_IWUSR,
			ad5270_show,
			ad5270_store,
			RPERFORMANCE);

static struct attribute *ad5270_attribute[] = {
	&iio_dev_attr_otp0.dev_attr.attr,
	&iio_dev_attr_otp0_en.dev_attr.attr,
	&iio_dev_attr_shutdown.dev_attr.attr,
	&iio_dev_attr_rperformance.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad5270_attribute_group[] = {
	[ID_AD5270] = {
		.attrs = ad5270_attribute,
	},
	[ID_AD5271] = {
		.attrs = ad5270_attribute,
	},
	[ID_AD5272] = {
		.attrs = ad5270_attribute,
	},
	[ID_AD5274] = {
		.attrs = ad5270_attribute,
	},
};

static struct iio_info ad5270_info = {
	.read_raw = ad5270_read_raw,
	.write_raw = ad5270_write_raw,
	.attrs = NULL,
};

#define AD5270_CHANNEL(chan, bits, sft)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (bits),				\
		.storagebits = 16,				\
		.shift = sft,					\
	},							\
}

static const struct ad5270_chip_info chip_info_tbl[] = {
	[ID_AD5270] = {
		.name = "AD5270",
		.num_channels = 1,
		.channel[0] = AD5270_CHANNEL(0, 10, 0),
		.features = AD5270_F_SDO_TRISTATE,
	},
	[ID_AD5271] = {
		.name = "AD5271",
		.num_channels = 1,
		.channel[0] = AD5270_CHANNEL(0, 8, 2),
		.features = AD5270_F_SDO_TRISTATE,
	},
	[ID_AD5272] = {
		.name = "AD5272",
		.num_channels = 1,
		.channel[0] = AD5270_CHANNEL(0, 10, 0),
	},
	[ID_AD5274] = {
		.name = "AD5274",
		.num_channels = 1,
		.channel[0] = AD5270_CHANNEL(0, 8, 2),
	},
};

static int ad5270_setup(struct ad5270_state *st)
{
	int ret = 0;

	if (st->chip_info->features & AD5270_F_SDO_TRISTATE)
		ret = ad5270_sdo_tristate(st);

	return ret;
}

static int ad5270_probe(struct device *dev, enum ad5270_type chip_type,
			const char *name, ad5270_read_func read,
			ad5270_write_func write)
{
	struct ad5270_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	st->read = read;
	st->write = write;
	st->chip_info = &chip_info_tbl[chip_type];
	ad5270_info.attrs = &ad5270_attribute_group[chip_type];

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &ad5270_info;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad5270_setup(st);
	if (ret)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret)
		return ret;

	return 0;
}

static int ad5270_remove(struct device *dev)
{
	iio_device_unregister(dev_get_drvdata(dev));

	return 0;
}

#if IS_ENABLED(CONFIG_SPI_MASTER)

static int ad5270_spi_read(struct ad5270_state *st,
		unsigned int data_shift)
{
	struct spi_device *spi = to_spi_device(st->dev);
	u8 data[2];
	int ret;

	ret = spi_read(spi, &data, 2);
	if (ret < 0)
		return ret;

	return (AD5270_DATA((((int)data[0]) << 8) | data[1]) >> data_shift);
}

static int ad5270_spi_write(struct ad5270_state *st, unsigned int cmd,
		unsigned int data, unsigned int data_shift)
{
	struct spi_device *spi = to_spi_device(st->dev);
	u16 msg;

	msg = cpu_to_be16(AD5270_CMD(cmd) |
		(AD5270_DATA(data) << data_shift));
	return spi_write(spi, &msg, sizeof(msg));
}

static int ad5270_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);

	return ad5270_probe(&spi->dev, id->driver_data, id->name,
		ad5270_spi_read, ad5270_spi_write);
}

static int ad5270_spi_remove(struct spi_device *spi)
{
	return ad5270_remove(&spi->dev);
}

static const struct spi_device_id ad5270_spi_ids[] = {
	{"ad5270_iio", ID_AD5270},
	{"ad5271_iio", ID_AD5271},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5270_spi_ids);

static struct spi_driver ad5270_spi_driver = {
	.driver = {
		.name	= "ad5270",
		.owner	= THIS_MODULE,
	},
	.probe		= ad5270_spi_probe,
	.remove		= ad5270_spi_remove,
	.id_table	= ad5270_spi_ids,
};

static int __init ad5270_spi_register_driver(void)
{
	return spi_register_driver(&ad5270_spi_driver);
}

static void ad5270_spi_unregister_driver(void)
{
	spi_unregister_driver(&ad5270_spi_driver);
}

#else

static inline int ad5270_spi_register_driver(void) { return 0; }
static inline void ad5270_spi_unregister_driver(void) { }

#endif

#if IS_ENABLED(CONFIG_I2C)

static int ad5270_i2c_read(struct ad5270_state *st,
		unsigned int data_shift)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	char data[2];
	int ret;

	ret = i2c_master_recv(i2c, data, 2);
	if (ret < 0)
		return ret;

	return (AD5270_DATA((data[0] << 8) | data[1]) >> data_shift);
}

static int ad5270_i2c_write(struct ad5270_state *st, unsigned int cmd,
		unsigned int data, unsigned int data_shift)
{
	struct i2c_client *i2c = to_i2c_client(st->dev);
	u16 msg = cpu_to_be16(AD5270_CMD(cmd) |
		(AD5270_DATA(data) << data_shift));
	int ret;

	ret = i2c_master_send(i2c, (char *)&msg, sizeof(msg));

	return (ret < 0 ? ret : (ret != sizeof(msg)) ? -EIO : 0);
}

static int ad5270_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	return ad5270_probe(&i2c->dev, id->driver_data, id->name,
		ad5270_i2c_read, ad5270_i2c_write);
}

static int ad5270_i2c_remove(struct i2c_client *i2c)
{
	return ad5270_remove(&i2c->dev);
}

static const struct i2c_device_id ad5270_i2c_ids[] = {
	{"ad5272_iio", ID_AD5272},
	{"ad5274_iio", ID_AD5274},
	{}
};
MODULE_DEVICE_TABLE(i2c, ad5270_i2c_ids);

static struct i2c_driver ad5270_i2c_driver = {
	.driver = {
		.name	= "ad5270",
		.owner	= THIS_MODULE,
	},
	.probe		= ad5270_i2c_probe,
	.remove		= ad5270_i2c_remove,
	.id_table	= ad5270_i2c_ids,
};

static int __init ad5270_i2c_register_driver(void)
{
	return i2c_add_driver(&ad5270_i2c_driver);
}

static void __exit ad5270_i2c_unregister_driver(void)
{
	i2c_del_driver(&ad5270_i2c_driver);
}

#else

static inline int ad5270_i2c_register_driver(void) { return 0; }
static inline void ad5270_i2c_unregister_driver(void) { }

#endif

static int __init ad5270_init(void)
{
	int ret;

	ret = ad5270_spi_register_driver();
	if (ret)
		return ret;

	ret = ad5270_i2c_register_driver();
	if (ret) {
		ad5270_spi_unregister_driver();
		return ret;
	}

	return 0;
}
module_init(ad5270_init);

static void __exit ad5270_exit(void)
{
	ad5270_i2c_unregister_driver();
	ad5270_spi_unregister_driver();
}
module_exit(ad5270_exit);

MODULE_AUTHOR("Dan Nechita <dan.nechita@analog.com>");
MODULE_DESCRIPTION("AD5270 Digital potentiometer driver");
MODULE_LICENSE("GPL v2");
