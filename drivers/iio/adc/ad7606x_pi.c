// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices ad7606x ADC driver for Parallel Interface
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include <linux/fpga/adi-axi-common.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			BIT(0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
/* Enable ADC channel with sign extension in HDL regmap's REG_CHAN_CNTRL register */
#define ADI_ENABLE_FMT_SIGNEX		0x51

#define KHZ				1000
#define SLEEP_TIME			3300
#define SETUP_TIME			275

#define AD7606X_MULTIPLE_CHAN(_idx, _storagebits, _realbits)		\
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

enum ad7606x_pi_id {
	ID_AD7606X_16,
	ID_AD7606X_18,
};

static const struct axiadc_chip_info conv_chip_info[] = {
	[ID_AD7606X_16] = {
		.name = "AD7606X-16",
		.num_channels = 8,
		.channel[0] = AD7606X_MULTIPLE_CHAN(0, 16, 16),
		.channel[1] = AD7606X_MULTIPLE_CHAN(1, 16, 16),
		.channel[2] = AD7606X_MULTIPLE_CHAN(2, 16, 16),
		.channel[3] = AD7606X_MULTIPLE_CHAN(3, 16, 16),
		.channel[4] = AD7606X_MULTIPLE_CHAN(4, 16, 16),
		.channel[5] = AD7606X_MULTIPLE_CHAN(5, 16, 16),
		.channel[6] = AD7606X_MULTIPLE_CHAN(6, 16, 16),
		.channel[7] = AD7606X_MULTIPLE_CHAN(7, 16, 16),
	},
	[ID_AD7606X_18] = {
		.name = "AD7606X-18",
		.num_channels = 8,
		.channel[0] = AD7606X_MULTIPLE_CHAN(0, 32, 18),
		.channel[1] = AD7606X_MULTIPLE_CHAN(1, 32, 18),
		.channel[2] = AD7606X_MULTIPLE_CHAN(2, 32, 18),
		.channel[3] = AD7606X_MULTIPLE_CHAN(3, 32, 18),
		.channel[4] = AD7606X_MULTIPLE_CHAN(4, 32, 18),
		.channel[5] = AD7606X_MULTIPLE_CHAN(5, 32, 18),
		.channel[6] = AD7606X_MULTIPLE_CHAN(6, 32, 18),
		.channel[7] = AD7606X_MULTIPLE_CHAN(7, 32, 18),
	},
};

struct ad7606x_pi_state {
	const	struct	axiadc_chip_info		*device_info;
	struct	iio_info				iio_info;
	struct	gpio_desc				*adc_serpar;
	struct	gpio_desc				*adc_refsel;
	struct	gpio_desc				*adc_reset;
	struct	gpio_desc				*adc_standby;
	struct	gpio_desc				*adc_range;
	struct	gpio_descs				*adc_os;
	struct	pwm_device				*cnvst_n;
	struct	clk					*ref_clk;

	/* protect against device accessse */
	struct	mutex					lock;

	int						sampling_freq;

	unsigned	int				pcore_version;
	unsigned	long				ref_clk_rate;
};

static int ad7606x_pi_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg, unsigned int writeval,
			     unsigned int *readval)
{
	struct axiadc_state *axiadc = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad7606x_pi_state *ad7606x_pi = conv->phy;

	mutex_lock(&ad7606x_pi->lock);
	if (!readval)
		axiadc_write(axiadc, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(axiadc, reg & 0xFFFF);
	mutex_unlock(&ad7606x_pi->lock);

	return 0;
}

static int ad7606x_pi_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	//struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);
	struct axiadc_state *axiadc = iio_priv(indio_dev);
	unsigned int i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(axiadc, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE_FMT_SIGNEX;
		else
			ctrl &= ~ADI_ENABLE_FMT_SIGNEX;

		axiadc_write(axiadc, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static int ad7606x_pi_set_sampling_freq(struct ad7606x_pi_state *ad7606x_pi, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state cnvst_n_state;
	int ret;

	target = DIV_ROUND_CLOSEST_ULL(ad7606x_pi->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL,
						  ad7606x_pi->ref_clk_rate);
	cnvst_n_state.period = ref_clk_period_ps * target;
	cnvst_n_state.duty_cycle = ref_clk_period_ps;
	cnvst_n_state.phase = ref_clk_period_ps;
	cnvst_n_state.time_unit = PWM_UNIT_PSEC;
	cnvst_n_state.enabled = true;
	ret = pwm_apply_state(ad7606x_pi->cnvst_n, &cnvst_n_state);
	if (ret)
		return ret;

	ad7606x_pi->sampling_freq = DIV_ROUND_CLOSEST_ULL(ad7606x_pi->ref_clk_rate, target);

	return 0;
}

static int ad7606x_pi_gpio_request(struct platform_device *pdev, struct ad7606x_pi_state *ad7606x_pi)
{
	ad7606x_pi->adc_serpar = devm_gpiod_get_optional(&pdev->dev, "adi,adc_serpar", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_serpar))
		return PTR_ERR(ad7606x_pi->adc_serpar);

	ad7606x_pi->adc_refsel = devm_gpiod_get_optional(&pdev->dev, "adi,adc_refsel", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_refsel))
		return PTR_ERR(ad7606x_pi->adc_refsel);

	ad7606x_pi->adc_reset = devm_gpiod_get_optional(&pdev->dev, "adi,adc_reset", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_reset))
		return PTR_ERR(ad7606x_pi->adc_reset);

	ad7606x_pi->adc_range = devm_gpiod_get_optional(&pdev->dev, "adi,adc_range", GPIOD_OUT_LOW);
	if (IS_ERR(ad7606x_pi->adc_range))
		return PTR_ERR(ad7606x_pi->adc_range);

	ad7606x_pi->adc_standby = devm_gpiod_get_optional(&pdev->dev, "adi,adc_standby", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x_pi->adc_standby))
		return PTR_ERR(ad7606x_pi->adc_standby);

	ad7606x_pi->adc_os = devm_gpiod_get_array_optional(&pdev->dev, "adi,adc_os", GPIOD_OUT_HIGH);
	if (IS_ERR(ad7606x_pi->adc_os))
		return PTR_ERR(ad7606x_pi->adc_os);

	return 0;
}

static int ad7606x_pi_full_reset(struct platform_device *pdev, struct ad7606x_pi_state *ad7606x_pi)
{
	int ret;

	ret = gpiod_direction_output(ad7606x_pi->adc_reset, 1);
	if (ret) {
		dev_err(&pdev->dev, "gpiod_direction_output to 1 failed");
		return ret;
	}

	ndelay(SLEEP_TIME);

	ret = gpiod_direction_output(ad7606x_pi->adc_reset, 0);
	if (ret) {
		dev_err(&pdev->dev, "gpiod_direction_output to 0 failed");
		return ret;
	}

	udelay(SETUP_TIME);

	return 0;
}

static int ad7606x_pi_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long info)
{
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);
	/* Scales are computed as 10000/32768 */
	const int scale_10_s_ended = 305176;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad7606x_pi->sampling_freq;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:

		*val = 0;
		*val2 = scale_10_s_ended;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad7606x_pi_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad7606x_pi_state *ad7606x_pi = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7606x_pi_set_sampling_freq(ad7606x_pi, val);

	default:
		return -EINVAL;
	}
}

static void ad7606x_pi_pwm_disable(void *data)
{
	pwm_disable(data);
}

static void ad7606x_pi_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct iio_info ad7606x_pi_iio_info = {
	.debugfs_reg_access = &ad7606x_pi_reg_access,
	.update_scan_mode = &ad7606x_pi_update_scan_mode,
	.read_raw = ad7606x_pi_read_raw,
	.write_raw = ad7606x_pi_write_raw,
};

static const struct of_device_id ad7606x_pi_of_match[] = {
	{
		.compatible = "ad7606x-pi-16",
		.data = &conv_chip_info[ID_AD7606X_16]
	},
	{
		.compatible = "ad7606x-pi-18",
		.data = &conv_chip_info[ID_AD7606X_18]
	},
	{}
};
MODULE_DEVICE_TABLE(of, ad7606x_pi_of_match);

static int ad7606x_pi_probe(struct platform_device *pdev)
{
	struct iio_dev			*indio_dev;
	struct axiadc_converter	*conv;
	struct ad7606x_pi_state		*st;
	int				ret;


	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	conv = devm_kzalloc(&pdev->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	mutex_init(&st->lock);

	ad7606x_pi_gpio_request(pdev, st);

	st->ref_clk = devm_clk_get(&pdev->dev, "ad7606x_pi_clk");
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, ad7606x_pi_clk_disable,
				       st->ref_clk);
	if (ret)
		return ret;

	st->ref_clk_rate = clk_get_rate(st->ref_clk);

	st->cnvst_n = devm_pwm_get(&pdev->dev, "cnvst_n");

	if (IS_ERR(st->cnvst_n))
		return PTR_ERR(st->cnvst_n);

	ret = devm_add_action_or_reset(&pdev->dev, ad7606x_pi_pwm_disable,
				       st->cnvst_n);
	if (ret)
		return ret;

	ad7606x_pi_full_reset(pdev, st);

	platform_set_drvdata(pdev, indio_dev);

	st->device_info = device_get_match_data(&pdev->dev);
	if (!st->device_info)
		return -EINVAL;

	st->iio_info = ad7606x_pi_iio_info;
	indio_dev->info = &st->iio_info;

	ret = ad7606x_pi_set_sampling_freq(st, 100 * KHZ);
	if (ret) {
		dev_err(&pdev->dev, "\nAD7606X setup failed\n");
		return ret;
	}

	conv->dev = &pdev->dev;
	//conv->clk = st->mclk;
	conv->clk = st->ref_clk;
	conv->chip_info = st->device_info;
	//conv->adc_output_mode = AD7768_OUTPUT_MODE_TWOS_COMPLEMENT; //TODO?
	//conv->reg_access = &ad7768_reg_access;
	//conv->write_raw = &ad7768_write_raw;
	//conv->read_raw = &ad7768_read_raw;
	conv->reg_access = &ad7606x_pi_reg_access;
	conv->write_raw = &ad7606x_pi_write_raw;
	conv->read_raw = &ad7606x_pi_read_raw;
	//conv->attrs = &ad7768_group;
	conv->phy = st;
	/* Without this, the axi_adc won't find the converter data */
	dev_set_drvdata(&pdev->dev, conv);

	return 0;
}

static struct platform_driver ad7606x_pi_driver = {
	.probe          = ad7606x_pi_probe,
	.driver         = {
		.name   = "ad7606x_pi",
		.owner = THIS_MODULE,
		.of_match_table = ad7606x_pi_of_match,
	},
};
module_platform_driver(ad7606x_pi_driver);

MODULE_AUTHOR("Alin-Tudor Sferle <alin-tudor.sferle@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7606X Parallel Interface ADC");
MODULE_LICENSE("Dual BSD/GPL");
