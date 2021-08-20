// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio/consumer.h>
#include <linux/mux/consumer.h>

#define HMC4069_DIV_MIN 2
#define HMC4069_DIV_MAX 32
#define HMC4069_DIV_TO_STATE(div_ratio) ((div_ratio) - 1)

static int hmc4069_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mux_control *mux;
	struct clk *ref, *vco;
	u32 ref_rate, vco_rate;
	u8 div_ratio;
	u8 state;
	int ret;

	mux = devm_mux_control_get(&pdev->dev, NULL);
	if (IS_ERR(mux)) {
		ret = PTR_ERR(mux);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get gpio control mux: %d\n", ret);
		return ret;
	}

	ref = devm_clk_get(dev, "ref");
	if (IS_ERR(ref)) {
		ret = PTR_ERR(ref);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get ref clock: %d\n", ret);
		return ret;
	}

	vco = devm_clk_get(dev, "vco");
	if (IS_ERR(vco)) {
		ret = PTR_ERR(vco);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get vco clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(ref);
	if (ret < 0) {
		dev_err(dev, "Failed to enable ref clock: %d\n", ret);
		return ret;
	}
	ref_rate = clk_get_rate(ref);

	ret = clk_prepare_enable(vco);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vco clock: %d\n", ret);
		goto error_vco_clk_prepare_enable;
	}
	vco_rate = clk_get_rate(vco);

	div_ratio = vco_rate / ref_rate;
	if (div_ratio < HMC4069_DIV_MIN || div_ratio > HMC4069_DIV_MAX) {
		ret = -EINVAL;
		dev_err(dev, "Div ratio out of range, min: %d, max: %d, "
			"calculated %d\n", HMC4069_DIV_MIN, HMC4069_DIV_MAX,
			div_ratio);
		goto error_invalid_rate;
	}
	state = HMC4069_DIV_TO_STATE(vco_rate / ref_rate);
	ret = mux_control_select(mux, state);
	if (ret) {
		dev_err(dev, "Failed to select mux state: %d\n", ret);
		goto error_mux_select;
	}

	return 0;

error_mux_select:
error_invalid_rate:
	clk_disable_unprepare(vco);
error_vco_clk_prepare_enable:
	clk_disable_unprepare(ref);

	return ret;
}

static const struct of_device_id hmc4069_of_match[] = {
	{ .compatible = "adi,hmc4069", },
	{ },
};
MODULE_DEVICE_TABLE(of, hmc4069_of_match);

static struct platform_driver hmc4069_driver = {
	.driver = {
		.name = "hmc4069",
		.of_match_table = hmc4069_of_match,
	},
	.probe = hmc4069_probe,
};
module_platform_driver(hmc4069_driver);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices HMC4069 driver");
MODULE_LICENSE("GPL v2");

