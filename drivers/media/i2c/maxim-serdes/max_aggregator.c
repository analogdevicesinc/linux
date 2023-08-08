// SPDX-License-Identifier: GPL-2.0
/*
 * Maxim GMSL2 Aggregator Driver
 *
 * Copyright (C) 2023 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-subdev.h>

struct max_aggregator_device {
	struct device *dev;
	struct v4l2_subdev subdev;
	struct media_pad *pads;
	u32 nsinks;
	u32 nsources;
};

static int max_aggregator_parse_of(struct max_aggregator_device *max_aggregator)
{
	struct device_node *node = max_aggregator->dev->of_node;

	max_aggregator->nsinks = 1;
	of_property_read_u32(node, "max,num-sink-slots", &max_aggregator->nsinks);

	max_aggregator->nsources = 1;
	of_property_read_u32(node, "max,num-source-slots", &max_aggregator->nsources);

	return 0;
}

static struct v4l2_subdev_ops max_aggregator_ops = { };

static int max_aggregator_probe(struct platform_device *pdev)
{
	struct max_aggregator_device *max_aggregator;
	struct v4l2_subdev *subdev;
	unsigned int npads;
	unsigned int i;
	int ret;

	max_aggregator = devm_kzalloc(&pdev->dev, sizeof(*max_aggregator), GFP_KERNEL);
	if (!max_aggregator)
		return -ENOMEM;

	max_aggregator->dev = &pdev->dev;

	ret = max_aggregator_parse_of(max_aggregator);
	if (ret)
		return ret;

	npads = max_aggregator->nsinks + max_aggregator->nsources;
	max_aggregator->pads = devm_kzalloc(&pdev->dev, npads * sizeof(*max_aggregator->pads),
					    GFP_KERNEL);
	if (!max_aggregator->pads)
		return -ENOMEM;

	for (i = 0; i < max_aggregator->nsinks; ++i)
		max_aggregator->pads[i].flags = MEDIA_PAD_FL_SINK;

	for (; i < npads; ++i)
		max_aggregator->pads[i].flags = MEDIA_PAD_FL_SOURCE;

	subdev = &max_aggregator->subdev;
	v4l2_subdev_init(subdev, &max_aggregator_ops);
	subdev->dev = &pdev->dev;
	strlcpy(subdev->name, dev_name(&pdev->dev), sizeof(subdev->name));
	subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	subdev->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;

	ret = media_entity_pads_init(&subdev->entity, npads, max_aggregator->pads);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, max_aggregator);

	ret = v4l2_async_register_subdev(subdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register subdev\n");
		goto error;
	}

	return 0;

error:
	media_entity_cleanup(&subdev->entity);

	return ret;
}

static int max_aggregator_remove(struct platform_device *pdev)
{
	struct max_aggregator_device *max_aggregator = platform_get_drvdata(pdev);
	struct v4l2_subdev *subdev = &max_aggregator->subdev;

	v4l2_async_unregister_subdev(subdev);
	media_entity_cleanup(&subdev->entity);

	return 0;
}

static const struct of_device_id max_aggregator_of_id_table[] = {
	{ .compatible = "maxim,aggregator" },
	{ }
};
MODULE_DEVICE_TABLE(of, max_aggregator_of_id_table);

static struct platform_driver max_aggregator_driver = {
	.driver = {
		.name		= "max-aggregator",
		.of_match_table	= max_aggregator_of_id_table,
	},
	.probe			= max_aggregator_probe,
	.remove			= max_aggregator_remove,
};

module_platform_driver(max_aggregator_driver);
MODULE_LICENSE("GPL");
