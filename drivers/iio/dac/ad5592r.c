/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

/*
 * TODO:
 *     - Add support for using channels as GPIOs
 *     - support for AD5593R (I2C interface)
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#define AD5592R_REG_NOOP		0x0
#define AD5592R_REG_DAC_READBACK	0x1
#define AD5592R_REG_ADC_SEQ		0x2
#define AD5592R_REG_CTRL		0x3
#define AD5592R_REG_ADC_EN		0x4
#define AD5592R_REG_DAC_EN		0x5
#define AD5592R_REG_PULLDOWN		0x6
#define AD5592R_REG_LDAC		0x7
#define AD5592R_REG_GPIO_OUT_EN	0x8
#define AD5592R_REG_GPIO_SET		0x9
#define AD5592R_REG_GPIO_IN_EN		0xA
#define AD5592R_REG_PD			0xB
#define AD5592R_REG_OPEN_DRAIN		0xC
#define AD5592R_REG_TRISTATE		0xD
#define AD5592R_REG_RESET		0xF

enum ad5592r_type {
	ID_AD5592R,
};

enum ad5592r_channel_mode {
	AD5592R_MODE_UNUSED,
	AD5592R_MODE_DAC,
	AD5592R_MODE_ADC,
	AD5592R_MODE_GPIO_OUT,
	AD5592R_MODE_GPIO_IN,
	AD5592R_MODE_GPIO_OPEN_DRAIN,
	AD5592R_MODE_GPIO_TRISTATE,
};

struct ad5592r_chip_info {
	unsigned num_channels;
};

struct ad5592r_state {
	struct spi_device *spi;
	const struct ad5592r_chip_info *chip_info;
	enum ad5592r_channel_mode channel_info[8];
};

static int ad5592r_dac_write(struct spi_device *spi, unsigned chan, u16 value)
{
	u16 msg = cpu_to_be16(BIT(15) | (chan << 12) | value);
	return spi_write(spi, &msg, sizeof(msg));
}

static int ad5592r_adc_read(struct spi_device *spi, unsigned chan, u16 *value)
{
	u16 msg = cpu_to_be16((AD5592R_REG_ADC_SEQ << 11) | BIT(chan));
	int ret = spi_write(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	spi_read(spi, &msg, sizeof(msg)); /* Invalid data */

	ret = spi_read(spi, &msg, sizeof(msg));
	if (!ret && value) {
		msg = be16_to_cpu(msg);

		/* Bits 12-14 must correspond to the channel */
		if (((msg >> 12) & 0x7) != chan) {
			dev_err(&spi->dev, "Error while reading channel %u\n",
					chan);
			ret = -EIO;
		} else {
			*value = msg & GENMASK(11, 0);
		}
	}

	return ret;
}

static int ad5592r_reg_write(struct spi_device *spi, u8 reg, u8 value)
{
	u16 msg = cpu_to_be16((reg << 11) | value);
	return spi_write(spi, &msg, sizeof(msg));
}

static int ad5592r_reg_verify(struct spi_device *spi, u8 reg, u8 value)
{
	u16 msg = cpu_to_be16((AD5592R_REG_LDAC << 11) | BIT(6) | (reg << 2));
	int ret = spi_write(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	ret = spi_read(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	msg = be16_to_cpu(msg);
	return msg == value ? 0 : -EIO;
}

static int ad5592r_set_channel_modes(struct ad5592r_state *st)
{
	int ret;
	unsigned i;
	struct spi_device *spi = st->spi;
	struct iio_dev *iio_dev = dev_get_drvdata(&spi->dev);
	u8 pulldown = 0, open_drain = 0, tristate = 0,
	   dac = 0, adc = 0, gpio_in = 0, gpio_out = 0;

	for (i = 0; i < st->chip_info->num_channels; i++) {
		switch (st->channel_info[i]) {
		case AD5592R_MODE_DAC:
			dac |= BIT(i);

		/* fall-through */
		case AD5592R_MODE_ADC:
			adc |= BIT(i);
			break;

		case AD5592R_MODE_GPIO_OUT:
			gpio_out |= BIT(i);

		/* fall-through */
		case AD5592R_MODE_GPIO_IN:
			gpio_in |= BIT(i);
			break;

		case AD5592R_MODE_GPIO_OPEN_DRAIN:
			open_drain |= BIT(i);
			break;

		case AD5592R_MODE_GPIO_TRISTATE:
			tristate |= BIT(i);
			break;

		default:
			pulldown |= BIT(i);
			break;
		}
	}

	mutex_lock(&iio_dev->mlock);

	/* Configure pins that we use */
	ret = ad5592r_reg_write(spi, AD5592R_REG_DAC_EN, dac);
	if (ret)
		goto err_unlock;

	ret = ad5592r_reg_write(spi, AD5592R_REG_ADC_EN, adc);
	if (ret)
		goto err_unlock;

	ret = ad5592r_reg_write(spi, AD5592R_REG_GPIO_OUT_EN, gpio_out);
	if (ret)
		goto err_unlock;

	ret = ad5592r_reg_write(spi, AD5592R_REG_GPIO_IN_EN, gpio_in);
	if (ret)
		goto err_unlock;

	ret = ad5592r_reg_write(spi, AD5592R_REG_OPEN_DRAIN, open_drain);
	if (ret)
		goto err_unlock;

	ret = ad5592r_reg_write(spi, AD5592R_REG_TRISTATE, tristate);
	if (ret)
		goto err_unlock;

	/* Pull down unused pins to GND */
	ret = ad5592r_reg_write(spi, AD5592R_REG_PULLDOWN, pulldown);
	if (ret)
		goto err_unlock;

	/* Verify that we can read back at least one register */
	ret = ad5592r_reg_verify(spi, AD5592R_REG_ADC_EN, adc);

err_unlock:
	mutex_unlock(&iio_dev->mlock);
	return ret;
}

static int ad5592r_write_raw(struct iio_dev *iio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct ad5592r_state *st = iio_priv(iio_dev);
	int ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		mutex_lock(&iio_dev->mlock);
		ret = ad5592r_dac_write(st->spi, chan->address, val);
		mutex_unlock(&iio_dev->mlock);
		break;

	default:
		break;
	}

	return ret;
}

static int ad5592r_read_raw(struct iio_dev *iio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct ad5592r_state *st = iio_priv(iio_dev);
	int ret = -EINVAL;
	u16 read_val;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&iio_dev->mlock);
		ret = ad5592r_adc_read(st->spi, chan->channel, &read_val);
		mutex_unlock(&iio_dev->mlock);
		if (ret)
			return ret;

		dev_dbg(&st->spi->dev, "ADC read: 0x%04hX\n", read_val);
		*val = (int) read_val;
		return IIO_VAL_INT;

	default:
		break;
	}

	return ret;
}

static int ad5592r_reset(struct ad5592r_state *st)
{
	struct iio_dev *iio_dev = dev_get_drvdata(&st->spi->dev);
	u16 msg = cpu_to_be16((AD5592R_REG_RESET << 11) | 0x5ac);
	int ret;

	mutex_lock(&iio_dev->mlock);
	ret = spi_write(st->spi, &msg, sizeof(msg));
	if (!ret)
		udelay(250);

	mutex_unlock(&iio_dev->mlock);
	return ret;
}

static const struct ad5592r_chip_info ad5592r_chip_info_tbl[] = {
	[ID_AD5592R] = {
		.num_channels = 8,
	},
};

static const struct iio_info ad5592r_info = {
	.read_raw = ad5592r_read_raw,
	.write_raw = ad5592r_write_raw,
	.driver_module = THIS_MODULE,
};

static void ad5592r_setup_channel(struct iio_dev *iio_dev,
		struct iio_chan_spec *chan, bool output, unsigned id)
{
	chan->type = IIO_VOLTAGE;
	chan->indexed = 1;
	chan->output = output;
	chan->channel = id;
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
	chan->address = id;
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
}

static int ad5592r_alloc_channels(struct ad5592r_state *st)
{
	unsigned i, curr_channel = 0,
		 num_channels = st->chip_info->num_channels;
	struct device *dev = &st->spi->dev;
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct iio_chan_spec *channels;
	u8 channel_modes[8];
	int ret;

	ret = of_property_read_u8_array(dev->of_node, "channel-modes",
			channel_modes, num_channels);
	if (ret)
		return ret;

	channels = devm_kzalloc(dev,
			2 * num_channels * sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (i = 0; i < num_channels; i++) {
		switch (channel_modes[i]) {
		case AD5592R_MODE_DAC:
			ad5592r_setup_channel(iio_dev, &channels[curr_channel],
					true, curr_channel);
			curr_channel++;
			st->channel_info[i] = channel_modes[i];
			break;

		case AD5592R_MODE_ADC:
			ad5592r_setup_channel(iio_dev, &channels[curr_channel],
					false, curr_channel);
			curr_channel++;

		/* fall-through */
		default:
			st->channel_info[i] = channel_modes[i];
			continue;
		}
	}

	iio_dev->num_channels = curr_channel;
	iio_dev->channels = channels;
	return 0;
}

static int ad5592r_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *iio_dev;
	struct ad5592r_state *st;
	int ret;

	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!iio_dev)
		return -ENOMEM;

	dev_dbg(&spi->dev, "Probing AD5592R\n");

	st = iio_priv(iio_dev);
	st->spi = spi;
	st->chip_info = &ad5592r_chip_info_tbl[id->driver_data];
	dev_set_drvdata(&spi->dev, iio_dev);

	BUG_ON(st->chip_info->num_channels > 8);

	iio_dev->dev.parent = &spi->dev;
	iio_dev->name = id->name;
	iio_dev->info = &ad5592r_info;
	iio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad5592r_reset(st);
	if (ret)
		return ret;

	ret = ad5592r_alloc_channels(st);
	if (ret)
		return ret;

	ret = ad5592r_set_channel_modes(st);
	if (ret)
		return ret;

	ret = iio_device_register(iio_dev);
	if (ret)
		return ret;

	dev_dbg(&spi->dev, "Probed\n");
	return 0;
}

static int ad5592r_spi_remove(struct spi_device *spi)
{
	struct iio_dev *iio_dev = dev_get_drvdata(&spi->dev);
	struct ad5592r_state *st = iio_priv(iio_dev);

	iio_device_unregister(iio_dev);
	return 0;
}

static const struct spi_device_id ad5592r_spi_ids[] = {
	{"ad5592r", ID_AD5592R},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5592r_spi_ids);

static struct spi_driver ad5592r_spi_driver = {
	.driver = {
		   .name = "ad5592r",
		   .owner = THIS_MODULE,
	},
	.probe = ad5592r_spi_probe,
	.remove = ad5592r_spi_remove,
	.id_table = ad5592r_spi_ids,
};

module_spi_driver(ad5592r_spi_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
