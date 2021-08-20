// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio/consumer.h>
#include <linux/mux/consumer.h>

struct hmc705 {
	struct device		*dev;
	struct mux_control	*mux;
	struct clk		*xref;
	struct clk_hw		clk_hw;
	struct mutex		lock;
	u8			div_ratio;
};

#define to_hmc_clk(_hw)	container_of(_hw, struct hmc705, clk_hw)

#define HMC705_DIV_MIN 1
#define HMC705_DIV_MAX 17
static const int hmc705_div_to_state[] = {
	[1] = 0b000001,
	[2] = 0b000010,
	[3] = 0b100010,
	[4] = 0b000100,
	[5] = 0b100100,
	[6] = 0b001000,
	[7] = 0b101000,
	[8] = 0b001100,
	[9] = 0b101100,
	[10] = 0b010000,
	[11] = 0b110000,
	[12] = 0b010100,
	[13] = 0b110100,
	[14] = 0b011000,
	[15] = 0b111000,
	[16] = 0b011100,
	[17] = 0b111100,
};

static u8 calculate_closest_div_ratio(u64 rate, u64 parent_rate)
{
	u8 div_rate = DIV_ROUND_CLOSEST_ULL(parent_rate, rate);

	if (div_rate < HMC705_DIV_MIN)
		div_rate = HMC705_DIV_MIN;
	else if (div_rate > HMC705_DIV_MAX)
		div_rate = HMC705_DIV_MAX;

	return div_rate;
}

static u64 calculate_rate(u64 parent_rate, u8 div_ratio)
{
	return DIV_ROUND_CLOSEST_ULL(parent_rate, div_ratio);
}

static int hmc705_clk_set_div_ratio(struct hmc705 *hmc, u8 div_ratio)
{
	u8 state;
	int ret;

	if (div_ratio < HMC705_DIV_MIN || div_ratio > HMC705_DIV_MAX) {
		dev_err(hmc->dev, "Div ratio out of range, min: %u, max: %u, "
			"given: %u\n", HMC705_DIV_MIN, HMC705_DIV_MAX,
			div_ratio);
		return -EINVAL;
	}

	state = hmc705_div_to_state[div_ratio];

	/**
	 * The state of the mux controller is set and then unset because it is
	 * necessary to not keep the mux locked. The state of the mux will be
	 * the same until the next _select because the default idle-state is
	 * MUX_IDLE_AS_IS.
	 */
	ret = mux_control_select(hmc->mux, state);
	if (ret) {
		dev_err(hmc->dev, "Failed to select mux state: %d\n", ret);
		return ret;
	}

	ret = mux_control_deselect(hmc->mux);
	if (ret) {
		dev_err(hmc->dev, "Failed to deselect mux state: %d\n", ret);
		return ret;
	}

	mutex_lock(&hmc->lock);
	hmc->div_ratio = div_ratio;
	mutex_unlock(&hmc->lock);

	return 0;
}

static unsigned long hmc705_clk_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct hmc705 *hmc = to_hmc_clk(hw);
	u8 div_ratio;

	mutex_lock(&hmc->lock);
	div_ratio = hmc->div_ratio;
	mutex_unlock(&hmc->lock);

	return calculate_rate(parent_rate, div_ratio);
}

static int hmc705_clk_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	struct hmc705 *hmc = to_hmc_clk(hw);
	u64 parent_rate = clk_get_rate(hmc->xref);
	u8 div_ratio = calculate_closest_div_ratio(req->rate, parent_rate);
	req->rate = calculate_rate(parent_rate, div_ratio);
	return 0;
}

static int hmc705_clk_set_rate(struct clk_hw *hw,
			       unsigned long rate,
			       unsigned long parent_rate)
{
	struct hmc705 *hmc = to_hmc_clk(hw);
	u8 div_ratio;
	int ret;

	if (!parent_rate)
		parent_rate = clk_get_rate(hmc->xref);

	div_ratio = calculate_closest_div_ratio(rate, parent_rate);
	ret = hmc705_clk_set_div_ratio(hmc, div_ratio);
	if (ret) {
		dev_err(hmc->dev, "Failed to set div ratio: %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct clk_ops hmc705_clk_ops = {
	.recalc_rate = hmc705_clk_recalc_rate,
	.determine_rate = hmc705_clk_determine_rate,
	.set_rate = hmc705_clk_set_rate,
};

enum ext_info {
	HMC705_FREQUENCY,
};

static ssize_t hmc705_read(struct iio_dev *indio_dev,
			   uintptr_t private,
			   const struct iio_chan_spec *chan,
			   char *buf)
{
	struct hmc705 *hmc = iio_priv(indio_dev);
	unsigned long long val;
	int ret = 0;

	switch ((u32)private) {
	case HMC705_FREQUENCY:
		val = clk_get_rate(hmc->clk_hw.clk);
		break;
	default:
		ret = -EINVAL;
		val = 0;
	}

	return ret < 0 ? ret : sprintf(buf, "%llu\n", val);
}

static ssize_t hmc705_write(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    const char *buf, size_t len)
{
	struct hmc705 *hmc = iio_priv(indio_dev);
	unsigned long long val;
	int ret;

	switch ((u32)private) {
	case HMC705_FREQUENCY:
		ret = kstrtoull(buf, 10, &val);
		if (ret)
			break;

		ret = clk_set_rate(hmc->clk_hw.clk, val);
		if (ret)
			dev_err(hmc->dev, "Failed to set clock rate to %llu"
				": %d\n", val, ret);
		break;
	default:
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static const struct iio_chan_spec_ext_info hmc705_ext_info[] = {
	{
		.name = "frequency",
		.read = hmc705_read,
		.write = hmc705_write,
		.private = HMC705_FREQUENCY,
		.shared = IIO_SEPARATE,
	},
	{},
};

#define HMC705_CHAN(index) { 		\
	.type = IIO_ALTVOLTAGE, 	\
	.output = 1, 			\
	.indexed = 1, 			\
	.channel = index, 		\
	.ext_info = hmc705_ext_info,	\
}

static struct iio_chan_spec hmc705_chan_spec[] = {
	HMC705_CHAN(0),
};

static const struct iio_info hmc705_info = {};

static int hmc705_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct clk_init_data init;
	struct iio_dev *indio_dev;
	const char *parent_name;
	struct hmc705 *hmc;
	struct clk *clk;
	u64 xref_rate;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*hmc));
	if (!indio_dev)
		return -ENOMEM;

	hmc = iio_priv(indio_dev);
	hmc->dev = dev;

	mutex_init(&hmc->lock);

	hmc->mux = devm_mux_control_get(&pdev->dev, NULL);
	if (IS_ERR(hmc->mux)) {
		ret = PTR_ERR(hmc->mux);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get gpio control mux: %d\n", ret);
		return ret;
	}

	hmc->xref = devm_clk_get(dev, "xref");
	if (IS_ERR(hmc->xref)) {
		ret = PTR_ERR(hmc->xref);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get xref clock: %d\n", ret);
		return ret;
	}
	xref_rate = clk_get_rate(hmc->xref);

	ret = clk_prepare_enable(hmc->xref);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xref clock: %d\n", ret);
		return ret;
	}

	indio_dev->dev.parent = dev;
	indio_dev->name = pdev->name;
	indio_dev->info = &hmc705_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = hmc705_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(hmc705_chan_spec);

	parent_name = __clk_get_name(hmc->xref);
	init.name = dev->of_node->name;
	init.ops = &hmc705_clk_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	hmc->clk_hw.init = &init;

	clk = devm_clk_register(dev, &hmc->clk_hw);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "Failed to register clock: %d\n", ret);
		goto error_clk_register;
	}

	ret = of_clk_add_provider(dev->of_node, of_clk_src_simple_get, clk);
	if (ret) {
		dev_err(dev, "Failed to add clock provider: %d\n", ret);
		goto error_clk_add_provider;
	}

	ret = clk_set_rate(hmc->clk_hw.clk, xref_rate);
	if (ret) {
		dev_err(dev, "Failed to set clock rate to %llu: %d\n",
			xref_rate, ret);
		goto error_clk_set_rate;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(dev, "Failed to register iio device: %d\n", ret);
		goto error_clk_iio_device_register;
	}

	return 0;

error_clk_iio_device_register:
	of_clk_del_provider(dev->of_node);
error_clk_set_rate:
error_clk_add_provider:
error_clk_register:
	clk_disable_unprepare(hmc->xref);

	return ret;
}

static const struct of_device_id hmc705_of_match[] = {
	{ .compatible = "adi,hmc705", },
	{ },
};
MODULE_DEVICE_TABLE(of, hmc705_of_match);

static struct platform_driver hmc705_driver = {
	.driver = {
		.name = "hmc705",
		.of_match_table = hmc705_of_match,
	},
	.probe = hmc705_probe,
};
module_platform_driver(hmc705_driver);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices HMC705 driver");
MODULE_LICENSE("GPL v2");

