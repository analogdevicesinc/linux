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

#include "ad5592r.h"

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>

static int ad5592r_set_channel_modes(struct ad5592r_state *st)
{
	const struct ad5592r_rw_ops *ops = st->ops;
	int ret;
	unsigned i;
	struct iio_dev *iio_dev = iio_priv_to_dev(st);
	u8 pulldown = 0, open_drain = 0, tristate = 0,
	   dac = 0, adc = 0, gpio_in = 0, gpio_out = 0;
	u16 read_back;

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
	ret = ops->reg_write(st, AD5592R_REG_DAC_EN, dac);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_ADC_EN, adc);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_GPIO_OUT_EN, gpio_out);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_GPIO_IN_EN, gpio_in);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_OPEN_DRAIN, open_drain);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_TRISTATE, tristate);
	if (ret)
		goto err_unlock;

	/* Pull down unused pins to GND */
	ret = ops->reg_write(st, AD5592R_REG_PULLDOWN, pulldown);
	if (ret)
		goto err_unlock;

	/* Verify that we can read back at least one register */
	ret = ops->reg_read(st, AD5592R_REG_ADC_EN, &read_back);
	if (!ret && (read_back & 0xff) != adc)
		ret = -EIO;

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
		ret = st->ops->dac_write(st, chan->address, val);
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
		ret = st->ops->adc_read(st, chan->channel, &read_val);
		mutex_unlock(&iio_dev->mlock);
		if (ret)
			return ret;

		if ((read_val >> 12 & 0x7) != chan->channel) {
			dev_err(st->dev, "Error while reading channel %u\n",
					chan->channel);
			return -EIO;
		}

		read_val &= GENMASK(11, 0);

		dev_dbg(st->dev, "ADC read: 0x%04hX\n", read_val);
		*val = (int) read_val;
		return IIO_VAL_INT;

	default:
		break;
	}

	return ret;
}

static int ad5592r_reset(struct ad5592r_state *st)
{
	struct iio_dev *iio_dev = iio_priv_to_dev(st);
	int ret;

	mutex_lock(&iio_dev->mlock);
	ret = st->ops->reg_write(st, AD5592R_REG_RESET, 0x5ac);
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
	struct iio_dev *iio_dev = iio_priv_to_dev(st);
	struct iio_chan_spec *channels;
	u8 channel_modes[8];
	int ret;

	ret = of_property_read_u8_array(st->dev->of_node, "channel-modes",
			channel_modes, num_channels);
	if (ret)
		return ret;

	channels = devm_kzalloc(st->dev,
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

int ad5592r_probe(struct device *dev, enum ad5592r_type type,
		const char *name, const struct ad5592r_rw_ops *ops)
{
	struct iio_dev *iio_dev;
	struct ad5592r_state *st;
	int ret;

	iio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!iio_dev)
		return -ENOMEM;

	dev_dbg(dev, "Probing AD5592R\n");

	st = iio_priv(iio_dev);
	st->dev = dev;
	st->ops = ops;
	st->chip_info = &ad5592r_chip_info_tbl[type];
	dev_set_drvdata(dev, iio_dev);

	BUG_ON(st->chip_info->num_channels > 8);

	iio_dev->dev.parent = dev;
	iio_dev->name = name;
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

	dev_dbg(dev, "Probed\n");
	return 0;
}
EXPORT_SYMBOL_GPL(ad5592r_probe);

int ad5592r_remove(struct device *dev)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);

	iio_device_unregister(iio_dev);
	return 0;
}
EXPORT_SYMBOL_GPL(ad5592r_remove);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
