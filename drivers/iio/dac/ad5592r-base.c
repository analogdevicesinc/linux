/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/gpio.h>

#include <dt-bindings/iio/adi,ad5592r.h>

#include "ad5592r-base.h"

#ifdef CONFIG_GPIOLIB

static struct ad5592r_state *gpiochip_to_ad5592r(struct gpio_chip *chip)
{
	return container_of(chip, struct ad5592r_state, gpiochip);
}

static int ad5592r_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ad5592r_state *st = gpiochip_to_ad5592r(chip);
	u8 val;
	int ret = 0;

	mutex_lock(&st->gpio_lock);

	if (st->gpio_out & BIT(offset))
		val = st->gpio_val;
	else
		ret = st->ops->gpio_read(st, &val);

	mutex_unlock(&st->gpio_lock);

	return (ret < 0) ? ret : !!(val & BIT(offset));
}

static void ad5592r_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct ad5592r_state *st = gpiochip_to_ad5592r(chip);

	mutex_lock(&st->gpio_lock);

	if (value)
		st->gpio_val |= BIT(offset);
	else
		st->gpio_val &= ~BIT(offset);

	st->ops->reg_write(st, AD5592R_REG_GPIO_SET, st->gpio_val);

	mutex_unlock(&st->gpio_lock);
}

static int ad5592r_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ad5592r_state *st = gpiochip_to_ad5592r(chip);
	int ret;

	mutex_lock(&st->gpio_lock);

	st->gpio_out &= ~BIT(offset);
	st->gpio_in |= BIT(offset);

	ret = st->ops->reg_write(st, AD5592R_REG_GPIO_OUT_EN, st->gpio_out);
	ret |= st->ops->reg_write(st, AD5592R_REG_GPIO_IN_EN, st->gpio_in);

	mutex_unlock(&st->gpio_lock);

	return ret;
}

static int ad5592r_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
					 int value)
{
	struct ad5592r_state *st = gpiochip_to_ad5592r(chip);
	int ret;

	mutex_lock(&st->gpio_lock);

	if (value)
		st->gpio_val |= BIT(offset);
	else
		st->gpio_val &= ~BIT(offset);

	st->gpio_in &= ~BIT(offset);
	st->gpio_out |= BIT(offset);

	ret = st->ops->reg_write(st, AD5592R_REG_GPIO_SET, st->gpio_val);
	ret |= st->ops->reg_write(st, AD5592R_REG_GPIO_OUT_EN, st->gpio_out);
	ret |= st->ops->reg_write(st, AD5592R_REG_GPIO_IN_EN, st->gpio_in);

	mutex_unlock(&st->gpio_lock);

	return ret;
}

static int ad5592r_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct ad5592r_state *st = gpiochip_to_ad5592r(chip);

	if (!(st->gpio_map & BIT(offset))) {
		dev_err(chip->dev, "GPIO %d is reserved by alternate function\n",
			offset);
		return -ENODEV;
	}

	if (offset >= chip->ngpio)
		return -EINVAL;

	return 0;
}


static int ad5592r_gpio_init(struct ad5592r_state *st)
{
	st->gpiochip.label = dev_name(st->dev);
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 8;
	st->gpiochip.dev = st->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad5592r_gpio_direction_input;
	st->gpiochip.direction_output = ad5592r_gpio_direction_output;
	st->gpiochip.get = ad5592r_gpio_get;
	st->gpiochip.set = ad5592r_gpio_set;
	st->gpiochip.request = ad5592r_gpio_request;
	st->gpiochip.owner = THIS_MODULE;

	mutex_init(&st->gpio_lock);

	return gpiochip_add(&st->gpiochip);
}

static void ad5592r_gpio_cleanup(struct ad5592r_state *st)
{
	gpiochip_remove(&st->gpiochip);
}

#else

static int ad5592r_gpio_init(struct ad5592r_state *st) { return 0 };
static void ad5592r_gpio_cleanup(struct ad5592r_state *st) { };

#endif

static int ad5592r_set_channel_modes(struct ad5592r_state *st)
{
	const struct ad5592r_rw_ops *ops = st->ops;
	int ret;
	unsigned i;
	struct iio_dev *iio_dev = iio_priv_to_dev(st);
	u8 pulldown = 0, open_drain = 0, tristate = 0,
	   dac = 0, adc = 0;
	u16 read_back;

	for (i = 0; i < st->num_channels; i++) {
		switch (st->channel_modes[i]) {
		case CH_MODE_DAC:
			dac |= BIT(i);
			break;

		case CH_MODE_ADC:
			adc |= BIT(i);
			break;

		case CH_MODE_DAC_AND_ADC:
			dac |= BIT(i);
			adc |= BIT(i);
			break;

		case CH_MODE_UNUSED_PULL_DOWN:
			pulldown |= BIT(i);
			break;

		case CH_MODE_UNUSED_OUT_TRISTATE:
			tristate |= BIT(i);
			break;

		case CH_MODE_UNUSED_OUT_LOW:
			st->gpio_out |= BIT(i);
			break;

		case CH_MODE_UNUSED_OUT_HIGH:
			st->gpio_out |= BIT(i);
			st->gpio_val |= BIT(i);
			break;

		case CH_MODE_GPIO_OPEN_DRAIN:
			open_drain |= BIT(i);

			/* fall-through */

		case CH_MODE_GPIO:
			st->gpio_map |= BIT(i);
			break;

		default:
			pulldown |= BIT(i);
			break;
		}
	}

	mutex_lock(&iio_dev->mlock);

	/* Pull down unused pins to GND */
	ret = ops->reg_write(st, AD5592R_REG_PULLDOWN, pulldown);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_TRISTATE, tristate);
	if (ret)
		goto err_unlock;

	/* Configure pins that we use */
	ret = ops->reg_write(st, AD5592R_REG_DAC_EN, dac);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_ADC_EN, adc);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_OPEN_DRAIN, open_drain);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_GPIO_SET, st->gpio_val);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_GPIO_OUT_EN, st->gpio_out);
	if (ret)
		goto err_unlock;

	ret = ops->reg_write(st, AD5592R_REG_GPIO_IN_EN, st->gpio_in);
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
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		/* Warn if we try to write to a ADC channel */
		WARN_ON(!chan->output);

		mutex_lock(&iio_dev->mlock);
		ret = st->ops->write_dac(st, chan->channel, val);
		if (!ret)
			st->cached_dac[chan->channel] = val;
		mutex_unlock(&iio_dev->mlock);
		return ret;

	default:
		return -EINVAL;
	}
}

static int ad5592r_read_raw(struct iio_dev *iio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct ad5592r_state *st = iio_priv(iio_dev);
	u16 read_val;
	int ret;

	mutex_lock(&iio_dev->mlock);

	switch (m) {
	case IIO_CHAN_INFO_RAW:

		if (!chan->output) {
			ret = st->ops->read_adc(st, chan->channel, &read_val);

			if (ret)
				goto unlock;

			if ((read_val >> 12 & 0x7) != chan->channel) {
				dev_err(st->dev, "Error while reading channel %u\n",
						chan->channel);
				ret = -EIO;
				goto unlock;
			}

			read_val &= GENMASK(11, 0);

		} else {
			read_val = st->cached_dac[chan->channel];
		}

		dev_dbg(st->dev, "Channel %u read: 0x%04hX\n",
				chan->channel, read_val);

		*val = (int) read_val;
		ret = IIO_VAL_INT;
		break;

	case IIO_CHAN_INFO_SCALE:

		if (st->reg) {
			ret = regulator_get_voltage(st->reg);
			if (ret < 0)
				goto unlock;

			*val = ret / 1000;
		} else {
			*val = 2500;
		}

		*val2 = chan->scan_type.realbits;
		ret = IIO_VAL_FRACTIONAL_LOG2;
		break;

	default:
		return -EINVAL;
	}

unlock:
	mutex_unlock(&iio_dev->mlock);
	return ret;
}

static int ad5592r_reset(struct ad5592r_state *st)
{
	struct gpio_desc *gpio;
	struct iio_dev *iio_dev = iio_priv_to_dev(st);

	gpio = devm_gpiod_get_optional(st->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	if (gpio) {
		udelay(1);
		gpiod_set_value(gpio, 1);
	} else {
		mutex_lock(&iio_dev->mlock);
		st->ops->reg_write(st, AD5592R_REG_RESET, 0xdac);
		mutex_unlock(&iio_dev->mlock);
	}

	udelay(250);

	return 0;
}

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
	chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				   BIT(IIO_CHAN_INFO_SCALE);
	chan->scan_type.sign = 'u';
	chan->scan_type.realbits = 12;
	chan->scan_type.storagebits = 16;
}

static int ad5592r_alloc_channels(struct ad5592r_state *st)
{
	unsigned i, curr_channel = 0,
		 num_channels = st->num_channels;
	struct iio_dev *iio_dev = iio_priv_to_dev(st);
	struct iio_chan_spec *channels;
	int ret;

	ret = device_property_read_u8_array(st->dev, "channel-modes",
			st->channel_modes, num_channels);
	if (ret)
		return ret;

	channels = devm_kzalloc(st->dev,
			2 * num_channels * sizeof(*channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	for (i = 0; i < num_channels; i++) {
		switch (st->channel_modes[i]) {
		case CH_MODE_DAC:
			ad5592r_setup_channel(iio_dev, &channels[curr_channel],
					true, i);
			curr_channel++;
			break;

		case CH_MODE_ADC:
			ad5592r_setup_channel(iio_dev, &channels[curr_channel],
					false, i);
			curr_channel++;
			break;

		case CH_MODE_DAC_AND_ADC:
			ad5592r_setup_channel(iio_dev, &channels[curr_channel],
					true, i);
			curr_channel++;
			ad5592r_setup_channel(iio_dev, &channels[curr_channel],
					false, i);
			curr_channel++;
			break;

		default:
			continue;
		}
	}

	iio_dev->num_channels = curr_channel;
	iio_dev->channels = channels;

	return 0;
}

int ad5592r_probe(struct device *dev, const char *name,
		const struct ad5592r_rw_ops *ops)
{
	struct iio_dev *iio_dev;
	struct ad5592r_state *st;
	int ret;

	iio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!iio_dev)
		return -ENOMEM;

	st = iio_priv(iio_dev);
	st->dev = dev;
	st->ops = ops;
	st->num_channels = 8;
	dev_set_drvdata(dev, iio_dev);

	st->reg = devm_regulator_get_optional(dev, "vref");
	if (PTR_ERR(st->reg) != -ENODEV)
		return PTR_ERR(st->reg);

	if (IS_ERR(st->reg)) {
		st->reg = NULL;
	} else {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	iio_dev->dev.parent = dev;
	iio_dev->name = name;
	iio_dev->info = &ad5592r_info;
	iio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad5592r_reset(st);
	if (ret)
		goto error_disable_reg;

	ret = ops->reg_write(st, AD5592R_REG_PD,
		     (st->reg == NULL) ? AD5592R_REG_PD_EN_REF : 0);
	if (ret)
		goto error_disable_reg;

	ret = ad5592r_alloc_channels(st);
	if (ret)
		goto error_disable_reg;

	ret = ad5592r_set_channel_modes(st);
	if (ret)
		goto error_disable_reg;

	ret = devm_iio_device_register(dev, iio_dev);
	if (ret)
		goto error_disable_reg;

	return ad5592r_gpio_init(st);

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}
EXPORT_SYMBOL_GPL(ad5592r_probe);

int ad5592r_remove(struct device *dev)
{
	struct iio_dev *iio_dev = dev_get_drvdata(dev);
	struct ad5592r_state *st = iio_priv(iio_dev);
	unsigned int i;

	/* Reset all channels */
	for (i = 0; i < ARRAY_SIZE(st->channel_modes); i++)
		st->channel_modes[i] = CH_MODE_UNUSED_PULL_DOWN;

	if (st->reg)
		regulator_disable(st->reg);

	if (st->gpio_map)
		ad5592r_gpio_cleanup(st);

	return ad5592r_set_channel_modes(st);
}
EXPORT_SYMBOL_GPL(ad5592r_remove);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
