// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices ONE_BIT_ADC_DAC
 * Digital to Analog Converters driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>

enum ch_direction {
	CH_IN,
	CH_OUT,
};

struct one_bit_adc_dac_state {
	struct platform_device  *pdev;
	struct gpio_descs       *in_gpio_descs;
	struct gpio_descs       *out_gpio_descs;
};

 #define ONE_BIT_ADC_DAC_CHANNEL(idx, direction)			\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = idx,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.output = direction,					\
	}

static int one_bit_adc_dac_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	int in_num_ch = 0;
	int channel = chan->channel;

	if (st->in_gpio_descs)
		in_num_ch = st->in_gpio_descs->ndescs;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (channel < in_num_ch) {
			*val = gpiod_get_value_cansleep(
				st->in_gpio_descs->desc[channel]);
		} else {
			channel -= in_num_ch;
			*val = gpiod_get_value_cansleep(
				st->out_gpio_descs->desc[channel]);
		}
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int one_bit_adc_dac_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long info)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	int in_num_ch = 0;
	int channel = chan->channel;

	if (st->in_gpio_descs)
		in_num_ch = st->in_gpio_descs->ndescs;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (channel >= in_num_ch) {
			channel -= in_num_ch;
			gpiod_set_value_cansleep(
				st->out_gpio_descs->desc[channel], val);
		}

		return 0;
	default:
		return -EINVAL;
	}
}

static const struct iio_info one_bit_adc_dac_info = {
	.read_raw = &one_bit_adc_dac_read_raw,
	.write_raw = &one_bit_adc_dac_write_raw,
};

static int one_bit_adc_dac_set_ch(struct iio_dev *indio_dev,
					struct iio_chan_spec *channels,
					const char *propname,
					int num_ch,
					enum ch_direction direction,
					int offset)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	const char **gpio_names;
	int ret, i;

	if (num_ch <= 0)
		return 0;

	gpio_names = devm_kcalloc(indio_dev->dev.parent,
					num_ch,
					sizeof(char *),
					GFP_KERNEL);
	if (!gpio_names)
		return -ENOMEM;

	ret = device_property_read_string_array(&st->pdev->dev,
					propname,
					gpio_names,
					num_ch);
	if (ret < 0)
		return ret;

	for (i = 0; i < num_ch; i++) {
		channels[i] = (struct iio_chan_spec)ONE_BIT_ADC_DAC_CHANNEL(i +
							offset,
							direction);
		channels[i].extend_name = gpio_names[i];
	}

	return 0;
}

static int one_bit_adc_dac_parse_dt(struct iio_dev *indio_dev)
{
	struct one_bit_adc_dac_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *channels;
	int ret, in_num_ch = 0, out_num_ch = 0;

	st->in_gpio_descs = devm_gpiod_get_array_optional(&st->pdev->dev,
						"in", GPIOD_IN);
	if (IS_ERR(st->in_gpio_descs))
		return PTR_ERR(st->in_gpio_descs);

	if (st->in_gpio_descs)
		in_num_ch = st->in_gpio_descs->ndescs;

	st->out_gpio_descs = devm_gpiod_get_array_optional(&st->pdev->dev,
						"out", GPIOD_OUT_HIGH);
	if (IS_ERR(st->out_gpio_descs))
		return PTR_ERR(st->out_gpio_descs);

	if (st->out_gpio_descs)
		out_num_ch = st->out_gpio_descs->ndescs;

	channels = devm_kcalloc(indio_dev->dev.parent, (in_num_ch + out_num_ch),
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	ret = one_bit_adc_dac_set_ch(indio_dev, &channels[0],
					"in-gpio-names", in_num_ch,
					CH_IN, 0);
	if (ret < 0)
		return ret;

	ret = one_bit_adc_dac_set_ch(indio_dev, &channels[in_num_ch],
					"out-gpio-names", out_num_ch,
					CH_OUT, in_num_ch);
	if (ret < 0)
		return ret;

	indio_dev->channels = channels;
	indio_dev->num_channels = in_num_ch + out_num_ch;

	return 0;
}

static int one_bit_adc_dac_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct one_bit_adc_dac_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->pdev = pdev;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "one-bit-adc-dac";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &one_bit_adc_dac_info;

	ret = one_bit_adc_dac_parse_dt(indio_dev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, indio_dev);
	return devm_iio_device_register(indio_dev->dev.parent, indio_dev);
}

static const struct of_device_id one_bit_adc_dac_dt_match[] = {
	{ .compatible = "adi,one-bit-adc-dac" },
	{},
};

MODULE_DEVICE_TABLE(of, one_bit_adc_dac_dt_match);

static struct platform_driver one_bit_adc_dac_driver = {
	.driver = {
		.name = "one-bit-adc-dac",
		.of_match_table = one_bit_adc_dac_dt_match,
	},
	.probe = one_bit_adc_dac_probe,
};

module_platform_driver(one_bit_adc_dac_driver);

MODULE_AUTHOR("Cristian Pop <cristian.pop@analog.com>");
MODULE_DESCRIPTION("Analog Devices ONE_BIT_ADC_DAC");
MODULE_LICENSE("GPL v2");
