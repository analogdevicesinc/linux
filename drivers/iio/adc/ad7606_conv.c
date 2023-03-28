// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices ad7606C ADC driver for Parallel Interface
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/gpio/consumer.h>

#include <linux/fpga/adi-axi-common.h>

#include <linux/iio/iio.h>

#include "cf_axi_adc.h"

/* ADC Channel */
/* Enable ADC channel with sign extension in HDL regmap's REG_CHAN_CNTRL register */
#define ADI_ENABLE_FMT_SIGNEX		0x51

#define KHZ				1000
#define FULL_RESET_TIME			3300
#define SETUP_TIME			275

#define AD7606C_MULTIPLE_CHAN(_idx, _storagebits, _realbits)		\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.indexed = 1,						\
		.channel = _idx,					\
		.scan_index = _idx,					\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = _storagebits,			\
			.realbits = _realbits,				\
			.shift = 0,					\
		},							\
	}

enum ad7606_conv_id {
	ID_AD7606C_16,
	ID_AD7606C_18,
};

static const struct axiadc_chip_info conv_chip_info[] = {
	[ID_AD7606C_16] = {
		.name = "AD7606C-16",
		.num_channels = 8,
		.channel[0] = AD7606C_MULTIPLE_CHAN(0, 16, 16),
		.channel[1] = AD7606C_MULTIPLE_CHAN(1, 16, 16),
		.channel[2] = AD7606C_MULTIPLE_CHAN(2, 16, 16),
		.channel[3] = AD7606C_MULTIPLE_CHAN(3, 16, 16),
		.channel[4] = AD7606C_MULTIPLE_CHAN(4, 16, 16),
		.channel[5] = AD7606C_MULTIPLE_CHAN(5, 16, 16),
		.channel[6] = AD7606C_MULTIPLE_CHAN(6, 16, 16),
		.channel[7] = AD7606C_MULTIPLE_CHAN(7, 16, 16),
	},
	[ID_AD7606C_18] = {
		.name = "AD7606C-18",
		.num_channels = 8,
		.channel[0] = AD7606C_MULTIPLE_CHAN(0, 32, 18),
		.channel[1] = AD7606C_MULTIPLE_CHAN(1, 32, 18),
		.channel[2] = AD7606C_MULTIPLE_CHAN(2, 32, 18),
		.channel[3] = AD7606C_MULTIPLE_CHAN(3, 32, 18),
		.channel[4] = AD7606C_MULTIPLE_CHAN(4, 32, 18),
		.channel[5] = AD7606C_MULTIPLE_CHAN(5, 32, 18),
		.channel[6] = AD7606C_MULTIPLE_CHAN(6, 32, 18),
		.channel[7] = AD7606C_MULTIPLE_CHAN(7, 32, 18),
	},
};

struct ad7606_conv_state {
	const struct axiadc_chip_info		*device_info;
	struct gpio_desc			*adc_reset;
	struct pwm_device			*cnvst_n;
	struct clk				*ref_clk;

	/* protect against concurrent device accesses */
	struct mutex				lock;

	int					sampling_freq;
	unsigned long				ref_clk_rate;
};

static int ad7606_read_label(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, char *label)
{
	return sprintf(label, "voltage%d\n", chan->channel);
}

static int ad7606_conv_reg_access(struct iio_dev *indio_dev,
				  unsigned int reg, unsigned int writeval,
				  unsigned int *readval)
{
	struct axiadc_state *axiadc = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad7606_conv_state *ad7606_conv = conv->phy;

	mutex_lock(&ad7606_conv->lock);
	if (!readval)
		axiadc_write(axiadc, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(axiadc, reg & 0xFFFF);
	mutex_unlock(&ad7606_conv->lock);

	return 0;
}

static int ad7606_conv_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *scan_mask)
{
	struct axiadc_state *axiadc = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad7606_conv_state *ad7606_conv = conv->phy;
	unsigned int i, ctrl;

	mutex_lock(&ad7606_conv->lock);
	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(axiadc, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE_FMT_SIGNEX;
		else
			ctrl &= ~ADI_ENABLE_FMT_SIGNEX;

		axiadc_write(axiadc, ADI_REG_CHAN_CNTRL(i), ctrl);
	}
	mutex_unlock(&ad7606_conv->lock);

	return 0;
}

static int ad7606_conv_set_sampling_freq(struct ad7606_conv_state *ad7606_conv,
					 int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnvst_n_state;
	int ret;

	target = DIV_ROUND_CLOSEST_ULL(ad7606_conv->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL,
						  ad7606_conv->ref_clk_rate);
	cnvst_n_state.period = ref_clk_period_ps * target;
	cnvst_n_state.duty_cycle = ref_clk_period_ps;
	cnvst_n_state.phase = ref_clk_period_ps;
	cnvst_n_state.time_unit = PWM_UNIT_PSEC;
	cnvst_n_state.enabled = true;
	ret = pwm_apply_state(ad7606_conv->cnvst_n, &cnvst_n_state);
	if (ret)
		return ret;

	ad7606_conv->sampling_freq = DIV_ROUND_CLOSEST_ULL(ad7606_conv->ref_clk_rate, target);

	return 0;
}

static int ad7606_conv_gpio_request(struct platform_device *pdev,
				    struct ad7606_conv_state *ad7606_conv)
{
	ad7606_conv->adc_reset = devm_gpiod_get_optional(&pdev->dev, "reset",
							 GPIOD_OUT_LOW);

	return PTR_ERR_OR_ZERO(ad7606_conv->adc_reset);
}

static int ad7606_conv_full_reset(struct platform_device *pdev,
				  struct ad7606_conv_state *ad7606_conv)
{
	if (ad7606_conv->adc_reset) {
		gpiod_set_value(ad7606_conv->adc_reset, 1);
		ndelay(FULL_RESET_TIME); /* t_reset >= 3200ns */
		gpiod_set_value(ad7606_conv->adc_reset, 0);
		udelay(SETUP_TIME); /* Full reset t_device_setup >= 274ns */
		return 0;
	}

	return -ENODEV;
}

static int ad7606_conv_read_raw(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan, int *val,
				int *val2, long info)
{
	struct ad7606_conv_state *ad7606_conv = iio_priv(indio_dev);
	/* Scales are computed as 10000/32768 */
	const int scale_10_s_ended = 305176;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad7606_conv->sampling_freq;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		*val2 = scale_10_s_ended;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad7606_conv_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int val,
				 int val2, long info)
{
	struct ad7606_conv_state *ad7606_conv = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7606_conv_set_sampling_freq(ad7606_conv, val);
	default:
		return -EINVAL;
	}
}

static void ad7606_conv_pwm_disable(void *data)
{
	pwm_disable(data);
}

static void ad7606_conv_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct of_device_id ad7606_conv_of_match[] = {
	{
		.compatible = "axi-ad7606c-16",
		.data = &conv_chip_info[ID_AD7606C_16]
	},
	{
		.compatible = "axi-ad7606c-18",
		.data = &conv_chip_info[ID_AD7606C_18]
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad7606_conv_of_match);

static int ad7606_conv_probe(struct platform_device *pdev)
{
	struct axiadc_converter	*conv;
	struct ad7606_conv_state *st;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	conv = devm_kzalloc(&pdev->dev, sizeof(*conv), GFP_KERNEL);
	if (!conv)
		return -ENOMEM;

	st->device_info = device_get_match_data(&pdev->dev);
	if (!st->device_info)
		return -EINVAL;

	mutex_init(&st->lock);

	ret = ad7606_conv_gpio_request(pdev, st);
	if (ret)
		return ret;

	st->ref_clk = devm_clk_get(&pdev->dev, "ad7606_conv_clk");
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, ad7606_conv_clk_disable,
				       st->ref_clk);
	if (ret)
		return ret;

	st->ref_clk_rate = clk_get_rate(st->ref_clk);

	st->cnvst_n = devm_pwm_get(&pdev->dev, "cnvst_n");

	if (IS_ERR(st->cnvst_n))
		return PTR_ERR(st->cnvst_n);

	ret = devm_add_action_or_reset(&pdev->dev, ad7606_conv_pwm_disable,
				       st->cnvst_n);
	if (ret)
		return ret;

	ret = ad7606_conv_full_reset(pdev, st);
	if (ret)
		dev_warn(&pdev->dev, "failed to RESET: no RESET GPIO specified\n");

	ret = ad7606_conv_set_sampling_freq(st, 100 * KHZ);
	if (ret) {
		dev_err(&pdev->dev, "\nad7606_conv setup failed\n");
		return ret;
	}

	conv->dev = &pdev->dev;
	conv->clk = st->ref_clk;
	conv->chip_info = st->device_info;
	conv->read_label = &ad7606_read_label;
	conv->reg_access = &ad7606_conv_reg_access;
	conv->write_raw = &ad7606_conv_write_raw;
	conv->read_raw = &ad7606_conv_read_raw;
	conv->update_scan_mode = &ad7606_conv_update_scan_mode;
	conv->phy = st;

	/* Without this, the axi_adc won't find the converter data */
	dev_set_drvdata(&pdev->dev, conv);

	return 0;
}

static struct platform_driver ad7606_conv_driver = {
	.probe          = ad7606_conv_probe,
	.driver         = {
		.name   = "ad7606_conv",
		.owner = THIS_MODULE,
		.of_match_table = ad7606_conv_of_match,
	},
};
module_platform_driver(ad7606_conv_driver);

MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606X Parallel Interface ADC");
MODULE_LICENSE("Dual BSD/GPL");
