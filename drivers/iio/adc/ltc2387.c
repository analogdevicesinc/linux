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
#include <linux/mod_devicetable.h>

#define LTC2387_VREF		4096
#define LTC2387_T_CNVH		8
#define LTC2387_T_FIRSTCLK	70

#define KHz 1000
#define MHz (1000 * KHz)

#define LTC2378_MULTIPLE_CHAN(_idx, _storagebits, _realbits, _shift)	\
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
			.shift = _shift,				\
		},							\
	}

#define LTC2378_CHAN(_realbits, _storagebits)				\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.scan_type = {						\
			.sign = 's',					\
			.storagebits = _storagebits,			\
			.realbits = _realbits,				\
		},							\
	}

enum ltc2387_lane_modes {
	ONE_LANE = 0,
	TWO_LANES = 1
};

enum ltc2387_id {
	ID_LTC2387_16,
	ID_LTC2387_16_X4,
	ID_LTC2387_18,
	ID_LTC2387_18_X4,
};

struct ltc2387_info {
	struct iio_chan_spec channels[4];
	unsigned int test_pattern[2];
	int num_channels;
	int resolution;
};

static const struct ltc2387_info ltc2387_infos[] = {
	[ID_LTC2387_16] = {
		.resolution = 16,
		.test_pattern = {
			[ONE_LANE] = 0b1010000001111111,
			[TWO_LANES] = 0b1100110000111111
		},
		.channels = {
			LTC2378_CHAN(16, 16),
		},
		.num_channels = 1,
	},
	[ID_LTC2387_16_X4] = {
		.resolution = 16,
		.test_pattern = {
			[ONE_LANE] = 0b1010000001111111,
			[TWO_LANES] = 0b1100110000111111
		},
		.channels = {
			LTC2378_MULTIPLE_CHAN(0, 64, 16, 0),
			LTC2378_MULTIPLE_CHAN(1, 64, 16, 16),
			LTC2378_MULTIPLE_CHAN(2, 64, 16, 32),
			LTC2378_MULTIPLE_CHAN(3, 64, 16, 48),
		},
		.num_channels = 4,
	},
	[ID_LTC2387_18] = {
		.resolution = 18,
		.test_pattern = {
			[ONE_LANE] = 0b101000000111111100,
			[TWO_LANES] = 0b110011000011111100
		},
		.channels = {
			LTC2378_CHAN(18, 32),
		},
		.num_channels = 1,
	},
	[ID_LTC2387_18_X4] = {
		.resolution = 18,
		.test_pattern = {
			[ONE_LANE] = 0b101000000111111100,
			[TWO_LANES] = 0b110011000011111100
		},
		.channels = {
			LTC2378_MULTIPLE_CHAN(0, 128, 32, 0),
			LTC2378_MULTIPLE_CHAN(1, 128, 32, 32),
			LTC2378_MULTIPLE_CHAN(2, 128, 32, 64),
			LTC2378_MULTIPLE_CHAN(3, 128, 32, 96),
		},
		.num_channels = 4,
	},
};

struct ltc2387_dev {
	const struct ltc2387_info *device_info;
	enum ltc2387_lane_modes lane_mode;
	struct gpio_desc *gpio_testpat;
	unsigned long ref_clk_rate;
	struct pwm_device *clk_en;
	struct regulator *vref;
	struct pwm_device *cnv;
	struct clk *ref_clk;

	unsigned int vref_mv;
	int sampling_freq;
};

static int ltc2387_set_sampling_freq(struct ltc2387_dev *ltc, int freq)
{
	unsigned long long target, ref_clk_period_ps;
	struct pwm_state clk_en_state, cnv_state;
	int ret, clk_en_time;

	target = DIV_ROUND_CLOSEST_ULL(ltc->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL,
						  ltc->ref_clk_rate);

	cnv_state = (struct pwm_state) {
		.period = ref_clk_period_ps * target,
		.duty_cycle = ref_clk_period_ps,
		.time_unit = PWM_UNIT_PSEC,
		.enabled = true,
	};

	ret = pwm_apply_state(ltc->cnv, &cnv_state);
	if (ret < 0)
		return ret;

	/* Gate the active period of the clock (see page 10-13 for both LTC's) */
	if (ltc->lane_mode == TWO_LANES)
		clk_en_time = DIV_ROUND_UP_ULL(ltc->device_info->resolution, 4);
	else
		clk_en_time = DIV_ROUND_UP_ULL(ltc->device_info->resolution, 2);

	clk_en_state = (struct pwm_state) {
		.period = cnv_state.period,
		.duty_cycle = ref_clk_period_ps * clk_en_time,
		.phase = cnv_state.phase + LTC2387_T_FIRSTCLK,
		.time_unit = PWM_UNIT_PSEC,
		.enabled = true,
	};

	ret = pwm_apply_state(ltc->clk_en, &clk_en_state);
	if (ret < 0)
		return ret;

	ltc->sampling_freq = DIV_ROUND_CLOSEST_ULL(ltc->ref_clk_rate, target);

	return 0;
}

static int ltc2387_setup(struct iio_dev *indio_dev)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;

	if (device_property_present(dev, "adi,use-two-lanes"))
		ltc->lane_mode = TWO_LANES;

	return ltc2387_set_sampling_freq(ltc, 15 * MHz);
}

static int ltc2387_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long info)
{
	struct ltc2387_dev *ltc = iio_priv(indio_dev);
	unsigned int temp;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ltc->sampling_freq;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		temp = regulator_get_voltage(ltc->vref);
		if (temp < 0)
			return temp;

		*val = (temp * 2) / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
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

	default:
		return -EINVAL;
	}
}

static void ltc2387_pwm_diasble(void *data)
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

static const struct iio_info ltc2387_info = {
	.read_raw = ltc2387_read_raw,
	.write_raw = ltc2387_write_raw,
};

static const struct of_device_id ltc2387_of_match[] = {
	{
		.compatible = "ltc2387-16",
		.data = &ltc2387_infos[ID_LTC2387_16]
	}, {
		.compatible = "ltc2387-16-x4",
		.data = &ltc2387_infos[ID_LTC2387_16_X4]
	}, {
		.compatible = "ltc2387-18",
		.data = &ltc2387_infos[ID_LTC2387_18]
	}, {
		.compatible = "ltc2387-18-x4",
		.data = &ltc2387_infos[ID_LTC2387_18_X4]
	}, {
		.compatible = "adaq2387-16",
		.data = &ltc2387_infos[ID_LTC2387_16]
	}, {
		.compatible = "adaq2387-18",
		.data = &ltc2387_infos[ID_LTC2387_18]
	},
	{}
};
MODULE_DEVICE_TABLE(of, ltc2387_of_match);

static int ltc2387_probe(struct platform_device *pdev)
{
	struct iio_dev			*indio_dev;
	struct ltc2387_dev		*ltc;
	int				ret;

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
		if (PTR_ERR(ltc->vref) != -ENODEV)
			return PTR_ERR(ltc->vref);

		ltc->vref_mv = LTC2387_VREF; /* Internal vref is used */
	}

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
	ltc->ref_clk_rate = clk_get_rate(ltc->ref_clk);

	ltc->clk_en = devm_pwm_get(&pdev->dev, "clk_en");
	if (IS_ERR(ltc->clk_en))
		return PTR_ERR(ltc->clk_en);

	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_pwm_diasble,
				       ltc->clk_en);
	if (ret)
		return ret;

	ltc->cnv = devm_pwm_get(&pdev->dev, "cnv");
	if (IS_ERR(ltc->cnv))
		return PTR_ERR(ltc->cnv);

	ret = devm_add_action_or_reset(&pdev->dev, ltc2387_pwm_diasble,
				       ltc->cnv);

	if (ret)
		return ret;

	ltc->device_info = device_get_match_data(&pdev->dev);
	if (!ltc->device_info)
		return -EINVAL;
	indio_dev->channels = ltc->device_info->channels;
	indio_dev->num_channels = ltc->device_info->num_channels;
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->info = &ltc2387_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
					      indio_dev, "rx",
					      IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return ret;

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
		.of_match_table = ltc2387_of_match,
	},
};
module_platform_driver(ltc2387_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Linear Technology LTC2387 ADC");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
