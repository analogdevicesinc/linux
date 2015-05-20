/*
 * AD7172-2/AD7173-8/AD7175-2/AD7176-2 SPI ADC driver
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/adc/ad_sigma_delta.h>

#define AD7173_REG_COMMS		0x00
#define AD7173_REG_ADC_MODE		0x01
#define AD7173_REG_INTERFACE_MODE	0x02
#define AD7173_REG_CRC			0x03
#define AD7173_REG_DATA			0x04
#define AD7173_REG_GPIO			0x06
#define AD7173_REG_ID			0x07
#define AD7173_REG_CH(x)		(0x10 + (x))
#define AD7173_REG_SETUP(x)		(0x20 + (x))
#define AD7173_REG_FILTER(x)		(0x28 + (x))
#define AD7173_REG_OFFSET(x)		(0x30 + (x))
#define AD7173_REG_GAIN(x)		(0x38 + (x))

#define AD7173_CH_ENABLE		BIT(15)
#define AD7173_CH_SETUP_SEL(x)		((x) << 12)
#define AD7173_CH_SETUP_AINPOS(x)	((x) << 5)
#define AD7173_CH_SETUP_AINNEG(x)	(x)

#define AD7173_CH_ADDRESS(pos, neg) \
	(AD7173_CH_SETUP_AINPOS(pos) | AD7173_CH_SETUP_AINNEG(neg))

#define AD7172_ID			0x00d0
#define AD7173_ID			0x30d0
#define AD7175_ID			0x0cd0
#define AD7176_ID			0x0c90
#define AD7173_ID_MASK			0xfff0

#define AD7173_ADC_MODE_REF_EN		BIT(15)
#define AD7173_ADC_MODE_SING_CYC	BIT(13)
#define AD7173_ADC_MODE_MODE_MASK	0x70
#define AD7173_ADC_MODE_MODE(x)	((x) << 4)
#define AD7173_ADC_MODE_CLOCKSEL_MASK	0xc
#define AD7173_ADC_MODE_CLOCKSEL(x)	((x) << 2)

#define AD7173_GPIO_PDSW		BIT(14)
#define AD7173_GPIO_OP_EN2_3		BIT(13)
#define AD7173_GPIO_MUX_IO		BIT(12)
#define AD7173_GPIO_SYNC_EN		BIT(11)
#define AD7173_GPIO_ERR_EN		BIT(10)
#define AD7173_GPIO_ERR_DAT		BIT(9)
#define AD7173_GPIO_GP_DATA3		BIT(7)
#define AD7173_GPIO_GP_DATA2		BIT(6)
#define AD7173_GPIO_IP_EN1		BIT(5)
#define AD7173_GPIO_IP_EN0		BIT(4)
#define AD7173_GPIO_OP_EN1		BIT(3)
#define AD7173_GPIO_OP_EN0		BIT(2)
#define AD7173_GPIO_GP_DATA1		BIT(1)
#define AD7173_GPIO_GP_DATA0		BIT(0)

#define AD7173_SETUP_BIPOLAR		BIT(12)
#define AD7173_SETUP_REF_SEL_MASK	0x30
#define AD7173_SETUP_REF_SEL_AVDD1_AVSS	0x30
#define AD7173_SETUP_REF_SEL_INT_REF	0x20
#define AD7173_SETUP_REF_SEL_EXT_REF2	0x10
#define AD7173_SETUP_REF_SEL_EXT_REF	0x00

#define AD7173_FILTER_ODR0_MASK		0x1f

enum ad7173_ids {
	ID_AD7172_2,
	ID_AD7173_8,
	ID_AD7175_2,
	ID_AD7176_2,
};

struct ad7173_device_info {
	unsigned int id;
	unsigned int num_inputs;
	unsigned int num_channels;
	unsigned int num_configs;
	bool has_gp23;
	bool has_temp;
	unsigned int clock;

	unsigned int *sinc5_data_rates;
	unsigned int num_sinc5_data_rates;
};

struct ad7173_state {
	struct regulator *reg;
	unsigned int adc_mode;
	unsigned int interface_mode;

	struct ad_sigma_delta sd;

	const struct ad7173_device_info *info;

#ifdef CONFIG_GPIOLIB
	struct gpio_chip gpiochip;
	unsigned int gpio_reg;
	unsigned int gpio_23_mask;
#endif
};

static unsigned int ad7173_sinc5_data_rates[] = {
	6211000,
	6211000,
	6211000,
	6211000,
	6211000,
	6211000,
	5181000,
	4444000,
	3115000,
	2597000,
	1007000,
	 503800,
	 381000,
	 200300,
	 100500,
	  59520,
	  49680,
	  20010,
	  16333,
	  10000,
	   5000,
	   2500,
	   1250,
};

static unsigned int ad7175_sinc5_data_rates[] = {
	50000000,
	41667000,
	31250000,
	27778000,
	20833000,
	17857000,
	12500000,
	10000000,
	5000000,
	2500000,
	1000000,
	 500000,
	 397500,
	 200000,
	 100000,
	  59920,
	  49960,
	  20000,
	  16666,
	  10000,
	   5000,
};

static struct ad7173_device_info ad7173_device_info[] = {
	[ID_AD7172_2] = {
		.id = AD7172_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.has_gp23 = false,
		.has_temp = true,
		.clock = 2000000,
		.sinc5_data_rates = ad7173_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7173_sinc5_data_rates),
	},
	[ID_AD7173_8] = {
		.id = AD7173_ID,
		.num_inputs = 17,
		.num_channels = 16,
		.num_configs = 8,
		.has_gp23 = true,
		.has_temp = true,
		.clock = 2000000,
		.sinc5_data_rates = ad7173_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7173_sinc5_data_rates),
	},
	[ID_AD7175_2] = {
		.id = AD7175_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.has_gp23 = false,
		.has_temp = true,
		.clock = 16000000,
		.sinc5_data_rates = ad7175_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7175_sinc5_data_rates),
	},
	[ID_AD7176_2] = {
		.id = AD7176_ID,
		.num_inputs = 5,
		.num_channels = 4,
		.num_configs = 4,
		.has_gp23 = false,
		.has_temp = false,
		.clock = 16000000,
		.sinc5_data_rates = ad7175_sinc5_data_rates,
		.num_sinc5_data_rates = ARRAY_SIZE(ad7175_sinc5_data_rates),
	},
};

#ifdef CONFIG_GPIOLIB

static struct ad7173_state *gpiochip_to_ad7173(struct gpio_chip *chip)
{
	return container_of(chip, struct ad7173_state, gpiochip);
}

static int ad7173_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ad7173_state *st = gpiochip_to_ad7173(chip);
	unsigned int mask;
	unsigned int value;
	int ret;

	switch (offset) {
	case 0:
		mask = AD7173_GPIO_GP_DATA0;
		break;
	case 1:
		mask = AD7173_GPIO_GP_DATA1;
		break;
	default:
		return -EINVAL;
	}

	ret = ad_sd_read_reg(&st->sd, AD7173_REG_GPIO, 2, &value);
	if (ret)
		return ret;

	return (bool)(value & mask);
}

static int ad7173_gpio_update(struct ad7173_state *st, unsigned int set_mask,
	unsigned int clr_mask)
{
	st->gpio_reg |= set_mask;
	st->gpio_reg &= ~clr_mask;

	return ad_sd_write_reg(&st->sd, AD7173_REG_GPIO, 2, st->gpio_reg);
}

static void ad7173_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct ad7173_state *st = gpiochip_to_ad7173(chip);
	unsigned int mask, set_mask, clr_mask;

	switch (offset) {
	case 0:
		mask = AD7173_GPIO_GP_DATA0;
		break;
	case 1:
		mask = AD7173_GPIO_GP_DATA1;
		break;
	case 2:
		mask = AD7173_GPIO_GP_DATA2;
		break;
	case 3:
		mask = AD7173_GPIO_GP_DATA3;
		break;
	default:
		return;
	}

	if (value) {
		set_mask = mask;
		clr_mask = 0;
	} else {
		set_mask = 0;
		clr_mask = mask;
	}

	ad7173_gpio_update(st, set_mask, clr_mask);
}

static int ad7173_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct ad7173_state *st = gpiochip_to_ad7173(chip);
	unsigned int mask;

	switch (offset) {
	case 0:
		mask = AD7173_GPIO_IP_EN0;
		break;
	case 1:
		mask = AD7173_GPIO_IP_EN1;
		break;
	default:
		return -EINVAL;
	}

	return ad7173_gpio_update(st, mask, 0);
}

static int ad7173_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
	int value)
{
	struct ad7173_state *st = gpiochip_to_ad7173(chip);
	unsigned int set_mask, clr_mask, val_mask;

	switch (offset) {
	case 0:
		set_mask = AD7173_GPIO_OP_EN0;
		val_mask = AD7173_GPIO_GP_DATA0;
		break;
	case 1:
		set_mask = AD7173_GPIO_OP_EN1;
		val_mask = AD7173_GPIO_GP_DATA1;
		break;
	/* GP2 and GP3 can not be enabled independently */
	case 2:
		st->gpio_23_mask |= (1 << 2);
		set_mask = AD7173_GPIO_OP_EN2_3;
		val_mask = AD7173_GPIO_GP_DATA2;
		break;
	case 3:
		st->gpio_23_mask |= (1 << 3);
		set_mask = AD7173_GPIO_OP_EN2_3;
		val_mask = AD7173_GPIO_GP_DATA3;
		break;
	default:
		return -EINVAL;
	}

	if (value) {
		set_mask |= val_mask;
		clr_mask = 0;
	} else {
		clr_mask = val_mask;
	}

	return ad7173_gpio_update(st, set_mask, clr_mask);
}

static void ad7173_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct ad7173_state *st = gpiochip_to_ad7173(chip);
	unsigned int mask;

	switch (offset) {
	case 0:
		mask = AD7173_GPIO_OP_EN0 | AD7173_GPIO_IP_EN0;
		break;
	case 1:
		mask = AD7173_GPIO_OP_EN1 | AD7173_GPIO_IP_EN1;
		break;
	case 2:
		st->gpio_23_mask &= ~(1 << offset);
		if (st->gpio_23_mask != 0)
			return;
		mask = AD7173_GPIO_OP_EN2_3;
		break;
	default:
		return;
	}

	ad7173_gpio_update(st, 0, mask);
}

static int ad7173_gpio_init(struct ad7173_state *st)
{
	st->gpiochip.label = dev_name(&st->sd.spi->dev);
	st->gpiochip.base = -1;
	if (st->info->has_gp23)
		st->gpiochip.ngpio = 4;
	else
		st->gpiochip.ngpio = 2;
	st->gpiochip.parent = &st->sd.spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad7173_gpio_direction_input;
	st->gpiochip.direction_output = ad7173_gpio_direction_output;
	st->gpiochip.get = ad7173_gpio_get;
	st->gpiochip.set = ad7173_gpio_set;
	st->gpiochip.free = ad7173_gpio_free;
	st->gpiochip.owner = THIS_MODULE;

	return gpiochip_add(&st->gpiochip);
}

static void ad7173_gpio_cleanup(struct ad7173_state *st)
{
	gpiochip_remove(&st->gpiochip);
}

#else

static int ad7173_gpio_init(struct ad7173_state *st) { return 0 };
static void ad7173_gpio_cleanup(struct ad7173_state *st) { };

#endif

static struct ad7173_state *ad_sigma_delta_to_ad7173(struct ad_sigma_delta *sd)
{
	return container_of(sd, struct ad7173_state, sd);
}

static int ad7173_prepare_channel(struct ad_sigma_delta *sd, unsigned int slot,
	const struct iio_chan_spec *chan)
{
	unsigned int config;

	config = AD7173_SETUP_REF_SEL_INT_REF;

	if (chan->differential)
		config |= AD7173_SETUP_BIPOLAR;

	return ad_sd_write_reg(sd, AD7173_REG_SETUP(0), 2, config);
}

static int ad7173_set_channel(struct ad_sigma_delta *sd, unsigned int slot,
	unsigned int channel)
{
	struct ad7173_state *st = ad_sigma_delta_to_ad7173(sd);

	return ad_sd_write_reg(&st->sd, AD7173_REG_CH(0), 2,
		AD7173_CH_ENABLE | channel);
}

static int ad7173_set_mode(struct ad_sigma_delta *sd,
			   enum ad_sigma_delta_mode mode)
{
	struct ad7173_state *st = ad_sigma_delta_to_ad7173(sd);

	st->adc_mode &= ~AD7173_ADC_MODE_MODE_MASK;
	st->adc_mode |= AD7173_ADC_MODE_MODE(mode);

	return ad_sd_write_reg(&st->sd, AD7173_REG_ADC_MODE, 2, st->adc_mode);
}

static const struct ad_sigma_delta_info ad7173_sigma_delta_info = {
	.set_channel = ad7173_set_channel,
	.prepare_channel = ad7173_prepare_channel,
	.set_mode = ad7173_set_mode,
	.has_registers = true,
	.data_reg = AD7173_REG_DATA,
	.addr_shift = 0,
	.read_mask = BIT(6),
};

static int ad7173_setup(struct iio_dev *indio_dev)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	unsigned int id;
	uint8_t *buf;
	int ret;

	/* reset the serial interface */
	buf = kcalloc(8, sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memset(buf, 0xff, 8);
	ret = spi_write(st->sd.spi, buf, 8);
	kfree(buf);
	if (ret < 0)
		return ret;

	/* datasheet recommends a delay of at least 500us after reset */
	usleep_range(500, 1000);

	ret = ad_sd_read_reg(&st->sd, AD7173_REG_ID, 2, &id);
	if (ret)
		return ret;

	id &= AD7173_ID_MASK;
	if (id != st->info->id) {
		dev_err(&st->sd.spi->dev, "Unexpected device id: %x, expected: %x\n",
				id, st->info->id);
		return -ENODEV;
	}

	st->adc_mode |= AD7173_ADC_MODE_REF_EN | AD7173_ADC_MODE_SING_CYC;

	return 0;
}

static int ad7173_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = ad_sigma_delta_single_conversion(indio_dev, chan, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			*val = 250000000;
			*val2 = 800273203; /* (2**24 * 477) / 10 */
			return IIO_VAL_FRACTIONAL;
		} else {
			*val = 2500;
			if (chan->differential)
				*val2 = 23;
			else
				*val2 = 24;
			return IIO_VAL_FRACTIONAL_LOG2;
		}
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			*val = -874379;
		} else {
			if (chan->differential)
				*val = -(1 << (chan->scan_type.realbits - 1));
			else
				*val = 0;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ad_sd_read_reg(&st->sd, AD7173_REG_FILTER(0), 2, &reg);
		if (ret)
			return ret;

		reg &= AD7173_FILTER_ODR0_MASK;
		if (reg >= st->info->num_sinc5_data_rates)
			reg -= 1;

		*val = st->info->sinc5_data_rates[reg] / 1000;
		*val2 = (st->info->sinc5_data_rates[reg] % 1000) * 1000;

		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static int ad7173_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	unsigned int freq;
	unsigned int reg;
	unsigned int i;
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	if (iio_buffer_enabled(indio_dev)) {
		mutex_unlock(&indio_dev->mlock);
		return -EBUSY;
	}

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		freq = val * 1000 + val2 / 1000;

		for (i = 0; i < st->info->num_sinc5_data_rates - 1; i++) {
			if (freq >= st->info->sinc5_data_rates[i])
				break;
		}

		ret = ad_sd_read_reg(&st->sd, AD7173_REG_FILTER(0), 2, &reg);
		if (ret)
			break;
		reg &= ~AD7173_FILTER_ODR0_MASK;
		reg |= i;
		ret = ad_sd_write_reg(&st->sd, AD7173_REG_FILTER(0), 2, reg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&indio_dev->mlock);
	return ret;
}

static int ad7173_write_raw_get_fmt(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, long mask)
{
	return IIO_VAL_INT_PLUS_MICRO;
}

static const struct iio_info ad7173_info = {
	.read_raw = &ad7173_read_raw,
	.write_raw = &ad7173_write_raw,
	.write_raw_get_fmt = &ad7173_write_raw_get_fmt,
	.validate_trigger = ad_sd_validate_trigger,
};

static const struct iio_chan_spec ad7173_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 0,
	.address = AD7173_CH_ADDRESS(0, 0),
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE),
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.scan_index = 0,
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.shift = 0,
		.endianness = IIO_BE,
	},
};

static const struct iio_chan_spec ad7173_temp_channel_template = {
	.type = IIO_TEMP,
	.indexed = 1,
	.channel = 0,
	.address = AD7173_CH_ADDRESS(17, 18),
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_OFFSET),
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.scan_index = 0,
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.shift = 0,
		.endianness = IIO_BE,
	},
};

static int ad7173_of_parse_channel_config(struct iio_dev *indio_dev,
	struct device_node *np)
{
	struct ad7173_state *st = iio_priv(indio_dev);
	struct device_node *chan_node, *child;
	struct iio_chan_spec *chan;
	unsigned int num_ext_channels = 0;
	unsigned int num_channels = 0;
	unsigned int scan_index = 0;
	unsigned int chan_index = 0;

	chan_node = of_get_child_by_name(np, "adi,channels");
	if (chan_node)
		num_ext_channels = of_get_available_child_count(chan_node);

	num_channels = num_ext_channels;
	if (st->info->has_temp)
		num_channels++;

	if (num_channels == 0)
		return 0;

	chan = devm_kcalloc(indio_dev->dev.parent, sizeof(*chan), num_channels,
		GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = num_channels;

	if (st->info->has_temp) {
		*chan = ad7173_temp_channel_template;
		chan++;
		scan_index++;
	}

	if (!chan_node)
		return 0;

	for_each_available_child_of_node(chan_node, child) {
		uint32_t ain[2];
		int ret;

		ret = of_property_read_u32_array(child, "reg", ain, 2);
		if (ret) {
			of_node_put(chan_node);
			of_node_put(child);
			return ret;
		}

		if (ain[0] >= st->info->num_inputs ||
		    ain[1] >= st->info->num_inputs) {
			dev_err(indio_dev->dev.parent,
				"Input pin number out of range.\n");
			of_node_put(chan_node);
			of_node_put(child);
			return -EINVAL;
		}


		*chan = ad7173_channel_template;
		chan->address = AD7173_CH_ADDRESS(ain[0], ain[1]);
		chan->scan_index = scan_index;
		chan->channel = ain[0];
		chan->channel2 = ain[1];
		chan->differential = of_property_read_bool(child, "adi,bipolar");
		if (chan->differential)
			chan->info_mask_separate |= BIT(IIO_CHAN_INFO_OFFSET);

		chan_index++;
		scan_index++;
		chan++;
	}
	of_node_put(chan_node);

	return 0;
}

static int ad7173_probe(struct spi_device *spi)
{
	const struct spi_device_id *id;
	struct ad7173_state *st;
	struct iio_dev *indio_dev;
	int ret;

	if (!spi->irq) {
		dev_err(&spi->dev, "No IRQ specified\n");
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	id = spi_get_device_id(spi);
	st->info = &ad7173_device_info[id->driver_data];

	ad_sd_init(&st->sd, indio_dev, spi, &ad7173_sigma_delta_info);

	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad7173_info;

	ret = ad7173_of_parse_channel_config(indio_dev, spi->dev.of_node);
	if (ret)
		return ret;

	ret = ad_sd_setup_buffer_and_trigger(indio_dev);
	if (ret)
		goto error_disable_reg;

	ret = ad7173_setup(indio_dev);
	if (ret)
		goto error_remove_trigger;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_remove_trigger;

	return ad7173_gpio_init(st);

error_remove_trigger:
	ad_sd_cleanup_buffer_and_trigger(indio_dev);
error_disable_reg:
	return ret;
}

static int ad7173_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7173_state *st = iio_priv(indio_dev);

	ad7173_gpio_cleanup(st);

	iio_device_unregister(indio_dev);
	ad_sd_cleanup_buffer_and_trigger(indio_dev);

	return 0;
}

static const struct spi_device_id ad7173_id_table[] = {
	{ "ad7172-2", ID_AD7172_2 },
	{ "ad7173-8", ID_AD7173_8 },
	{ "ad7175-2", ID_AD7175_2 },
	{ "ad7176-2", ID_AD7176_2 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7173_id_table);

static struct spi_driver ad7173_driver = {
	.driver = {
		.name	= "ad7173",
		.owner	= THIS_MODULE,
	},
	.probe		= ad7173_probe,
	.remove		= ad7173_remove,
	.id_table	= ad7173_id_table,
};
module_spi_driver(ad7173_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafo.de>");
MODULE_DESCRIPTION("Analog Devices AD7172/AD7173/AD7175/AD7176 ADC driver");
MODULE_LICENSE("GPL v2");
