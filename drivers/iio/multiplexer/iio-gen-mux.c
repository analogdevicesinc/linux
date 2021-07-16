// SPDX-License-Identifier: GPL-2.0
/*
 * IIO-GEN-MUX Generic MUX control via device attribute
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/mux/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/clk-provider.h>
#include <linux/clk.h>


struct gen_mux_state {
	struct device		*dev;
	struct mutex		lock; /* protect mux state */
	struct mux_control	*mux;
	unsigned int		sel;
	const char		**strings;
	unsigned int		max_states;
	struct clk_hw		hw;
	const char		**parent_names;
};

static u8 gen_mux_clk_mux_get_parent(struct clk_hw *hw)
{
	struct gen_mux_state *st = container_of(hw, struct gen_mux_state, hw);

	dev_dbg(st->dev, "%s:%d: %d", __func__, __LINE__, st->sel);

	return st->sel;
}

static int den_mux_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct gen_mux_state *st = container_of(hw, struct gen_mux_state, hw);

	dev_dbg(st->dev, "%s:%d: %d index %d",
		__func__, __LINE__, st->sel, index);

	return 0;
}

static int gen_mux_clk_mux_determine_rate(struct clk_hw *hw,
				      struct clk_rate_request *req)
{
	struct gen_mux_state *st = container_of(hw, struct gen_mux_state, hw);
	struct clk_hw *parent = clk_hw_get_parent_by_index(hw, st->sel);

	dev_dbg(st->dev, "%s:%d: %lu", __func__, __LINE__, req->rate);

	req->best_parent_hw = parent;
	req->best_parent_rate = req->rate;

	return 0;
}
const struct clk_ops clk_gen_mux_ops = {
	.get_parent = gen_mux_clk_mux_get_parent,
	.set_parent = den_mux_clk_mux_set_parent,
	.determine_rate = gen_mux_clk_mux_determine_rate,
};

static void gen_mux_of_clk_del_provider(void *data)
{
	struct device *dev = data;

	of_clk_del_provider(dev->of_node);
}

static int gen_mux_clk_driver_probe(struct platform_device *pdev,
	struct gen_mux_state *st)
{
	struct device_node *node = pdev->dev.of_node;
	unsigned int num_parents;
	struct clk_init_data init = {};
	struct clk_hw *hw;
	const char *clk_name = node->name;
	int ret;

	num_parents = of_clk_get_parent_count(node);

	if (num_parents) {
		st->parent_names = devm_kcalloc(&pdev->dev, num_parents,
					    sizeof(char *), GFP_KERNEL);
		if (!st->parent_names)
			return -ENOMEM;

		of_clk_parent_fill(node, st->parent_names, num_parents);
	} else {
		return 0;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &clk_gen_mux_ops;
	init.flags = 0;
	init.parent_names = st->parent_names;
	init.num_parents = num_parents;

	st->hw.init = &init;

	hw = &st->hw;

	ret = devm_clk_hw_register(&pdev->dev, hw);
	if (ret)
		return ret;

	ret = of_clk_add_provider(node, of_clk_src_simple_get, hw->clk);
	if (ret)
		return ret;

	return devm_add_action_or_reset(&pdev->dev,
		gen_mux_of_clk_del_provider, &pdev->dev);
}

static ssize_t gen_mux_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct gen_mux_state *st = iio_priv(indio_dev);
	int ret, index;

	index = __sysfs_match_string(st->strings, st->max_states, buf);
	if (index < 0)
		return -EINVAL;

	mutex_lock(&st->lock);
	ret = mux_control_select(st->mux, index);
	if (!ret) {
		st->sel = index;
		mux_control_deselect(st->mux);
	}

	if (st->parent_names)
		clk_set_parent(st->hw.clk,
			__clk_lookup(st->parent_names[index]));

	mutex_unlock(&st->lock);

	dev_dbg(indio_dev->dev.parent, "setting up the mux to %d\n", index);

	return ret ? ret : len;
}

static ssize_t gen_mux_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct gen_mux_state *st = iio_priv(indio_dev);
	int ret = 0, i;

	mutex_lock(&st->lock);
	switch ((u32)this_attr->address) {
	case 0:
		ret = sysfs_emit(buf, "%s\n", st->strings[st->sel]);
		break;
	case 1:
		for (i = 0; i < st->max_states; i++) {
			if (strlen(st->strings[i]))
				ret += sysfs_emit_at(buf, ret, "%s ",
					st->strings[i]);
		}
		/* replace last space with a newline */
		if (ret)
			buf[ret - 1] = '\n';
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static IIO_DEVICE_ATTR(mux_select, 0644, gen_mux_show, gen_mux_store, 0);
static IIO_DEVICE_ATTR(mux_select_available, 0644, gen_mux_show, NULL, 1);

static struct attribute *gen_mux_attributes[] = {
	&iio_dev_attr_mux_select.dev_attr.attr,
	&iio_dev_attr_mux_select_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group gen_mux_attribute_group = {
	.attrs = gen_mux_attributes,
};

static const struct iio_info gen_mux_info = {
	.attrs = &gen_mux_attribute_group,
};

/* Match table for of_platform binding */
static const struct of_device_id gen_mux_of_match[] = {
	{ .compatible = "adi,gen_mux", },
	{},
};
MODULE_DEVICE_TABLE(of, gen_mux_of_match);

static int gen_mux_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct gen_mux_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev = &pdev->dev;

	st->mux = devm_mux_control_get(&pdev->dev, NULL);
	if (IS_ERR(st->mux)) {
		if (PTR_ERR(st->mux) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get control-mux (%ld)\n",
				PTR_ERR(st->mux));

		return PTR_ERR(st->mux);
	}

	st->max_states = mux_control_states(st->mux);

	st->strings = devm_kcalloc(&pdev->dev, st->max_states,
		sizeof(*st->strings), GFP_KERNEL);
	if (!st->strings)
		return -ENOMEM;

	ret = of_property_read_string_array(pdev->dev.of_node,
		"mux-state-names", st->strings, st->max_states);
	if (ret < 0)
		return ret;

	mutex_init(&st->lock);

	indio_dev->name = "iio-gen-mux";
	indio_dev->info = &gen_mux_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = gen_mux_clk_driver_probe(pdev, st);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static struct platform_driver gen_mux_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = gen_mux_of_match,
	},
	.probe = gen_mux_probe,
};
module_platform_driver(gen_mux_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Generic MUX control via device attribute");
MODULE_LICENSE("GPL v2");
