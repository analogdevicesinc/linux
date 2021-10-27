// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * JESD204 Generic TOP device implementation
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

#define JESD204_OF_PREFIX	"adi,"
#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/jesd204-of.h>

#define JESD204_TOP_DEVICE_MAX_NUM_LINKS	16

struct jesd204_top_device_data {
	struct device *dev;
	struct jesd204_dev_data	jesd204_dev_data;
	struct jesd204_link links[JESD204_TOP_DEVICE_MAX_NUM_LINKS];
	u32 d; /* dummy */
};

static int jes204_parse_jesd_link_dt(struct jesd204_top_device_data *tdev,
				     struct device_node *np,
				     struct jesd204_link *link)
{
	struct device *dev = tdev->dev;
	int ret;

	JESD204_LNK_READ_DEVICE_ID(dev, np, link, &tdev->d, 0);
	JESD204_LNK_READ_OCTETS_PER_FRAME(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_FRAMES_PER_MULTIFRAME(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_SAMPLES_PER_CONVERTER_PER_FRAME(dev, np, link,
		&tdev->d, -1);
	JESD204_LNK_READ_HIGH_DENSITY(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_CONVERTER_RESOLUTION(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_BITS_PER_SAMPLE(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_NUM_CONVERTERS(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_CTRL_BITS_PER_SAMPLE(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_NUM_LANES(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_VERSION(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_SUBCLASS(dev, np, link, &tdev->d, -1);
	JESD204_LNK_READ_SCRAMBLING(dev, np, link, &tdev->d, 1);

	ret = of_property_read_u64(np, JESD204_OF_PREFIX "sample-rate",
		&link->sample_rate);
	if (ret || !link->sample_rate) {
		dev_err(dev, "Missing DT prop '%s' @ line %d ",
			JESD204_OF_PREFIX "sample-rate", __LINE__);
		return -EINVAL;
	}

	ret = of_property_read_u32(np, JESD204_OF_PREFIX "sample-rate-div",
		&link->sample_rate_div);
	if (ret)
		link->sample_rate_div = 1;

	return 0;
}

static int jesd204_link_init(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct jesd204_top_device_data *tdev = dev_get_drvdata(dev);

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	if (lnk->link_id > JESD204_TOP_DEVICE_MAX_NUM_LINKS)
		return -EFAULT;

	if (!tdev->links[lnk->link_id].num_lanes)
		return -EFAULT;

	jesd204_copy_link_params(lnk, &tdev->links[lnk->link_id]);

	lnk->sample_rate = tdev->links[lnk->link_id].sample_rate;
	lnk->sample_rate_div = tdev->links[lnk->link_id].sample_rate_div;

	if (lnk->jesd_version == JESD204_VERSION_C)
		lnk->jesd_encoder = JESD204_ENCODER_64B66B;
	else
		lnk->jesd_encoder = JESD204_ENCODER_8B10B;

	return JESD204_STATE_CHANGE_DONE;
}

/* Match table for of_platform binding */
static const struct of_device_id jesd204_top_device_of_match[] = {
	{ .compatible = "adi,jesd204-top-device-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, jesd204_top_device_of_match);

static int jesd204_top_device_probe(struct platform_device *pdev)
{
	struct jesd204_top_device_data *tdev;
	struct jesd204_dev *jdev;
	struct device_node *of_links, *of_link;
	int ret;
	u32 reg;

	if (!pdev->dev.of_node)
		return -ENODEV;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	tdev->dev = &pdev->dev;

	of_links = of_get_child_by_name(pdev->dev.of_node, "adi,jesd-links");
	if (!of_links)
		return -ENODEV;

	for_each_child_of_node(of_links, of_link) {
		ret = of_property_read_u32(of_link, "reg", &reg);
		if (ret || (!ret && reg >= JESD204_TOP_DEVICE_MAX_NUM_LINKS)) {
			dev_err(&pdev->dev, "Missing or invalid DT prop 'reg'\n");
			of_node_put(of_link);
			of_node_put(of_links);
			return -EINVAL;
		}
		ret = jes204_parse_jesd_link_dt(tdev, of_link, &tdev->links[reg]);
		if (!ret)
			tdev->jesd204_dev_data.max_num_links++;
	}

	of_node_put(of_links);

	tdev->jesd204_dev_data.state_ops[JESD204_OP_LINK_INIT].per_link =
		jesd204_link_init;

	dev_set_drvdata(&pdev->dev, tdev);

	jdev = devm_jesd204_dev_register(&pdev->dev, &tdev->jesd204_dev_data);
	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	return jesd204_fsm_start(jdev, JESD204_LINKS_ALL);
}

static struct platform_driver jesd204_top_device_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = jesd204_top_device_of_match,
	},
	.probe = jesd204_top_device_probe,
};
module_platform_driver(jesd204_top_device_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Generic jesd204 top-device");
MODULE_LICENSE("Dual BSD/GPL");
