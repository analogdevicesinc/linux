// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Linear Technology LTC2387 ADC driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define LTC2387_VREF		4096
#define LTC2387_T_CNVH		8
#define LTC2387_T_FIRSTCLK	70

#define KHz 1000
#define MHz (1000 * KHz)

#define LTC2378_CHAN(resolution)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = 32,				\
			.realbits = resolution,				\
		},							\
	}

enum ltc2387_lane_modes {
	ONE_LANE = 0,
	TWO_LANES = 1
};

enum ltc2387_id {
	ID_LTC2387_16 = 1,
	ID_LTC2387_18
};

const unsigned int ltc2387_resolutions[] = {
	[ID_LTC2387_16] = 16,
	[ID_LTC2387_18] = 18
};

const unsigned int ltc2387_testpatterns[][2] = {
	[ID_LTC2387_16] = {
		[ONE_LANE] = 0b1010000001111111,
		[TWO_LANES] = 0b1100110000111111
	},
	[ID_LTC2387_18] = {
		[ONE_LANE] = 0b101000000111111100,
		[TWO_LANES] = 0b110011000011111100
	}
};

struct ltc2387_dev {
	struct mutex		lock;

	int			sampling_freq;
	unsigned int		resolution;
	unsigned int		vref_mv;
	unsigned int		id;
	enum ltc2387_lane_modes	lane_mode;
	struct gpio_desc 	*gpio_testpat;
	struct clk		*ref_clk;
	struct pwm_device	*clk_en;
	struct regulator	*vref;
	struct pwm_device	*cnv;
};


static const struct iio_chan_spec ltc2387_channels[] = {
	[ID_LTC2387_16] = LTC2378_CHAN(16),
	[ID_LTC2387_18] = LTC2378_CHAN(18)
};

static const struct of_device_id ltc2387_of_match[] = {
	{
		.compatible = "ltc2387-18",
		.data = (void *) ID_LTC2387_18
	},
	{
		.compatible = "ltc2387-16",
		.data = (void *) ID_LTC2387_16
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2387_of_match);

// static int ltc2387_set_conversion(struct ltc2387_dev *ltc, bool enabled)
// {
// 	struct pwm_state clk_en_state, cnv_state;
// 	struct pwm_capture clk_en_capture, cnv_capture;

// 	int ret;

// 	ret = pwm_capture(ltc->cnv, &cnv_capture, 0);
// 	if (ret < 0)
// 		return ret;
// 	cnv_state.period = cnv_capture.period;
// 	cnv_state.duty_cycle = cnv_capture.duty_cycle;
// 	cnv_state.offset = cnv_capture.offset;
// 	cnv_state.time_unit = cnv_capture.time_unit;
// 	cnv_state.enabled = enabled;
// 	ret = pwm_apply_state(ltc->cnv, &cnv_state);
// 	if (ret < 0)
// 		return ret;
// 	ret = pwm_capture(ltc->clk_en, &clk_en_capture, 0);
// 	if (ret < 0)
// 		return ret;
// 	clk_en_state.period = clk_en_capture.period;
// 	clk_en_state.duty_cycle = clk_en_capture.duty_cycle;
// 	clk_en_state.offset = clk_en_capture.offset;
// 	clk_en_state.time_unit = clk_en_capture.time_unit;
// 	clk_en_state.enabled = enabled;

// 	return pwm_apply_state(ltc->clk_en, &clk_en_state);
// }

static int ltc2387_set_sampling_freq(struct ltc2387_dev *ltc, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state clk_en_state, cnv_state;
	unsigned long ref_clk_rate;
	int ret, clk_en_time;

	ref_clk_rate = clk_get_rate(ltc->ref_clk);

	target = DIV_ROUND_CLOSEST_ULL(ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL, ref_clk_rate);
	cnv_state.period = ref_clk_period_ps * target;
	cnv_state.duty_cycle = ref_clk_period_ps;
	cnv_state.offset = ref_clk_period_ps;
	cnv_state.time_unit = PWM_UNIT_PSEC;
	cnv_state.enabled = true;

	ret = pwm_apply_state(ltc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	/* Gate the active period of the clock (see page 10-13 for both LTC's) */
	if (ltc->lane_mode == TWO_LANES)
		clk_en_time = DIV_ROUND_UP_ULL(ltc->resolution, 4);
	else
		clk_en_time = DIV_ROUND_UP_ULL(ltc->resolution, 2);
	clk_en_state.period = cnv_state.period;
	clk_en_state.duty_cycle = ref_clk_period_ps * clk_en_time;
	clk_en_state.offset = 0;
	clk_en_state.time_unit = PWM_UNIT_PSEC;
	clk_en_state.enabled = true;

	ret = pwm_apply_state(ltc->clk_en, &clk_en_state);
	if (ret < 0)
		return ret;

	ltc->sampling_freq = (int)DIV_ROUND_CLOSEST_ULL(ref_clk_rate, target);

	return 0;
}

static int ltc2387_setup(struct iio_dev *indio_dev)
{
	struct fwnode_handle *fwnode = dev_fwnode(indio_dev->dev.parent);
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	if (fwnode_property_present(fwnode, "adi,use-two-lanes"))
		ltc->lane_mode = TWO_LANES;
	
	return ltc2387_set_sampling_freq(ltc, 15*MHz);
	// return ltc2387_set_conversion(ltc, false);
}

static int ltc2387_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long info)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ltc->sampling_freq;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ltc2387_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ltc2387_set_sampling_freq(ltc, val);
	case IIO_CHAN_INFO_SCALE:

	default:
		return -EINVAL;
	}
}

static int ltc2387_buffer_preenable(struct iio_dev *indio_dev)
{
	// struct ltc2387_dev *ltc = iio_priv(indio_dev);

	return 0;
	// return ltc2387_set_conversion(ltc, true);
}

static int ltc2387_buffer_postdisable(struct iio_dev *indio_dev)
{
	// struct ltc2387_dev *ltc = iio_priv(indio_dev);

	return 0;
	// return ltc2387_set_conversion(ltc, false);
}

static int ltc2387_dma_submit(struct iio_dma_buffer_queue *queue,
			      struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static void ltc2387_clk_diasble(void *data)
{
	pwm_disable(data);
}

static void ltc2387_cnv_diasble(void *data)
{
	pwm_disable(data);
}

static void ltc2387_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ltc2387_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static const struct iio_dma_buffer_ops ltc2387_dma_buffer_ops = {
	.submit = ltc2387_dma_submit,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ltc2387_buffer_ops = {
	.preenable = &ltc2387_buffer_preenable,
	.postdisable = &ltc2387_buffer_postdisable,
};

static const struct iio_info ltc2387_info = {
	.read_raw = ltc2387_read_raw,
	.write_raw = ltc2387_write_raw,
};

static int ltc2387_probe(struct platform_device *pdev)
{
	const struct of_device_id	*device_id;
	struct iio_dev			*indio_dev;
	struct iio_buffer		*buffer;
	struct ltc2387_dev		*ltc;
	int				ret;
	int				id;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ltc));
	if (!indio_dev)
		return -ENOMEM;

	ltc = iio_priv(indio_dev);

	ltc->vref = devm_regulator_get_optional(&pdev->dev, "vref");
	if (!IS_ERR(ltc->vref)) {
		ret = regulator_enable(ltc->vref);
		if (ret) {
			dev_err(&pdev->dev, "Can't to enable vref regulator\n");
			return ret;
		}
		ret = regulator_get_voltage(ltc->vref);
		if (ret < 0)
			return ret;
		ltc->vref_mv = ret / 1000;
		ret = devm_add_action_or_reset(&pdev->dev,
					       ltc2387_regulator_disable,
					       ltc->vref);
		if (ret)
			return ret;
	} else {
		/* Internal vref is used */
		ltc->vref_mv = 4096;
	}

	// ltc->gpio_testpat = devm_gpiod_get_optional(&pdev->dev, "testpat",
	// 					    GPIOD_OUT_HIGH);
	// if (IS_ERR(ltc->gpio_testpat))
	// 	return PTR_ERR(ltc->gpio_testpat);

	ltc->ref_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(ltc->ref_clk))
		return PTR_ERR(ltc->ref_clk);

	ret = clk_prepare_enable(ltc->ref_clk);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_clk_disable,
				       ltc->ref_clk);
	if (ret)
		return ret;

	ltc->clk_en = devm_pwm_get(&pdev->dev, "clk_en");
	if (IS_ERR(ltc->clk_en))
		return PTR_ERR(ltc->clk_en);
	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_clk_diasble,
				       ltc->clk_en);

	ltc->cnv = devm_pwm_get(&pdev->dev, "cnv");
	if (IS_ERR(ltc->cnv))
		return PTR_ERR(ltc->cnv);
	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_cnv_diasble,
				       ltc->cnv);

	device_id = of_match_device(of_match_ptr(ltc2387_of_match), &pdev->dev);
	id = (int) device_id->data;

	ltc->id = id;
	ltc->resolution = ltc2387_resolutions[id];

	indio_dev->channels = &ltc2387_channels[id];
	indio_dev->num_channels = 1;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = "ltc2387";
	indio_dev->info = &ltc2387_info;
	indio_dev->setup_ops = &ltc2387_buffer_ops;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						"rx",
						&ltc2387_dma_buffer_ops,
						indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = ltc2387_setup(indio_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "\nltc2387 setup failed\n");
		return ret;
	}

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver ltc2387_driver = {
	.probe          = ltc2387_probe,
	.driver         = {
		.name   = "ltc2387",
		.of_match_table = of_match_ptr(ltc2387_of_match),
	},
};
module_platform_driver(ltc2387_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Linear Technology LTC2387 ADC");
MODULE_LICENSE("Dual BSD/GPL");
