/*
 * IMAGEON V4L2 bridge driver
 *
 * Copyright 2015 Analog Devices Inc.
 *  Author: Dragos Bogdan <dragos.bogdan@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_graph.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/i2c/adv7604.h>

#define INPUT_SUBDEV		0
#define OUTPUT_SUBDEV		1

struct imageon_subdev {
	struct fwnode_handle *fwnode;
	struct v4l2_subdev *subdev;
};

struct imageon_bridge {
	struct imageon_subdev imageon_subdev[2];
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_device media_dev;

	u8 input_edid_data[256];
	u8 input_edid_blocks;

	int irq;
};

static struct imageon_bridge *
	notifier_to_imageon_bridge(struct v4l2_async_notifier *n)
{
	return container_of(n, struct imageon_bridge, notifier);
}

static int imageon_bridge_load_input_edid(struct platform_device *pdev,
					  struct imageon_bridge *bridge)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "imageon_edid.bin", &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to load firmware: %d\n", ret);
		return ret;
	}

	if (fw->size > 256) {
		dev_err(&pdev->dev, "EDID firmware data too large\n");
		release_firmware(fw);
		bridge->input_edid_blocks = 0;
		return -EINVAL;
	}

	if (fw->size > 128)
		bridge->input_edid_blocks = 2;
	else
		bridge->input_edid_blocks = 1;

	memcpy(bridge->input_edid_data, fw->data, fw->size);

	release_firmware(fw);

	return 0;
}

static irqreturn_t imageon_bridge_hdmiio_int_handler(int irq, void *dev_id)
{
	struct imageon_bridge *bridge = dev_id;

	if (bridge->imageon_subdev[INPUT_SUBDEV].subdev)
		v4l2_subdev_call(bridge->imageon_subdev[INPUT_SUBDEV].subdev,
				 core, interrupt_service_routine, 0, NULL);

	if (bridge->imageon_subdev[OUTPUT_SUBDEV].subdev)
		v4l2_subdev_call(bridge->imageon_subdev[OUTPUT_SUBDEV].subdev,
				 core, interrupt_service_routine, 0, NULL);

	return IRQ_HANDLED;
}

static int imageon_bridge_async_bound(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *subdev,
				      struct v4l2_async_subdev *asd)
{
	struct fwnode_handle *subdev_node = of_fwnode_handle(subdev->dev->of_node);
	struct imageon_bridge *bridge = notifier_to_imageon_bridge(notifier);
	int ret;

	if (bridge->imageon_subdev[INPUT_SUBDEV].fwnode == subdev_node) {
		struct v4l2_subdev_edid edid = {
			.pad = 0,
			.start_block = 0,
			.blocks = bridge->input_edid_blocks,
			.edid = bridge->input_edid_data,
		};

		bridge->imageon_subdev[INPUT_SUBDEV].subdev = subdev;
		ret = v4l2_subdev_call(subdev, video, s_routing,
				       ADV76XX_PAD_HDMI_PORT_A, 0, 0);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(subdev, pad, set_edid, &edid);
		if (ret)
			return ret;
	}

	if (bridge->imageon_subdev[OUTPUT_SUBDEV].fwnode == subdev_node) {
		bridge->imageon_subdev[OUTPUT_SUBDEV].subdev = subdev;

		ret = v4l2_subdev_call(subdev, video, s_stream, 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int imageon_bridge_async_complete(struct v4l2_async_notifier *notifier)
{
	struct imageon_bridge *bridge = notifier_to_imageon_bridge(notifier);
	struct v4l2_subdev_format fmt;
	int ret;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = ADV7611_PAD_SOURCE;
	fmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;
	ret = v4l2_subdev_call(bridge->imageon_subdev[INPUT_SUBDEV].subdev,
			       pad, set_fmt, NULL, &fmt);
	if (ret)
		return ret;

	return 0;
}

static const struct v4l2_async_notifier_operations imageon_async_ops = {
	.bound = imageon_bridge_async_bound,
	.complete = imageon_bridge_async_complete,
};

static struct imageon_bridge *imageon_bridge_parse_dt(struct device *dev)
{
	struct imageon_bridge *bridge;
	struct device_node *ep = NULL;
	struct device_node *next;
	int index;

	bridge = devm_kzalloc(dev, sizeof(struct imageon_bridge), GFP_KERNEL);
	if (!bridge)
		return NULL;

	for (index = 0; index < 2; index++) {
		struct fwnode_handle *fwnode;

		next = of_graph_get_next_endpoint(dev->of_node, ep);
		if (!next)
			return NULL;

		ep = next;

		fwnode = of_fwnode_handle(of_graph_get_remote_port_parent(next));
		bridge->imageon_subdev[index].fwnode = fwnode;
	}

	return bridge;
}

static int imageon_bridge_notifier_init(struct imageon_bridge *bridge)
{
	struct imageon_subdev *input = &bridge->imageon_subdev[INPUT_SUBDEV];
	struct imageon_subdev *output = &bridge->imageon_subdev[OUTPUT_SUBDEV];
	struct v4l2_async_notifier *notifier = &bridge->notifier;
	struct v4l2_async_subdev *asd;

	v4l2_async_nf_init(notifier);

	asd = v4l2_async_nf_add_fwnode(notifier, input->fwnode,
				       struct v4l2_async_subdev);
	if (IS_ERR(asd))
		return PTR_ERR(asd);

	asd = v4l2_async_nf_add_fwnode(notifier, output->fwnode,
				       struct v4l2_async_subdev);
	if (IS_ERR(asd))
		return PTR_ERR(asd);

	notifier->ops = &imageon_async_ops;

	return v4l2_async_nf_register(&bridge->v4l2_dev, notifier);
}

static int imageon_bridge_probe(struct platform_device *pdev)
{
	struct imageon_bridge *bridge;
	int ret;

	bridge = imageon_bridge_parse_dt(&pdev->dev);
	if (!bridge)
		return -ENOMEM;

	bridge->irq = platform_get_irq(pdev, 0);
	if (bridge->irq > 0) {
		ret = request_threaded_irq(bridge->irq, NULL,
					   imageon_bridge_hdmiio_int_handler,
					   IRQF_ONESHOT | IRQF_TRIGGER_LOW,
					   dev_name(&pdev->dev), bridge);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request irq\n");
			return ret;
		}
	}

	ret = imageon_bridge_load_input_edid(pdev, bridge);
	if (ret < 0)
		goto err;

	bridge->media_dev.dev = &pdev->dev;
	strscpy(bridge->media_dev.model, "IMAGEON V4L2 Bridge",
		sizeof(bridge->media_dev.model));
	bridge->media_dev.hw_revision = 0;

	media_device_init(&bridge->media_dev);

	ret = media_device_register(&bridge->media_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register media_device\n");
		goto err;
	}
	bridge->v4l2_dev.mdev = &bridge->media_dev;

	snprintf(bridge->v4l2_dev.name, sizeof(bridge->v4l2_dev.name),
		 "imageon_v4l2_bridge");

	ret = v4l2_device_register(&pdev->dev, &bridge->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to register v4l2_device\n");
		goto err;
	}

	ret = imageon_bridge_notifier_init(bridge);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to initialize bridge notifier\n");
		goto imageon_notifier_clean;
	}

	ret = v4l2_device_register_subdev_nodes(&bridge->v4l2_dev);
	if (ret < 0)
		goto err;

	return 0;

imageon_notifier_clean:
	v4l2_async_nf_cleanup(&bridge->notifier);
err:
	if (bridge->irq > 0)
		free_irq(bridge->irq, pdev);
	return ret;
}

static int imageon_bridge_remove(struct platform_device *pdev)
{
	struct imageon_bridge *bridge = platform_get_drvdata(pdev);

	v4l2_async_nf_unregister(&bridge->notifier);
	v4l2_async_nf_cleanup(&bridge->notifier);
	v4l2_device_unregister(&bridge->v4l2_dev);
	media_device_unregister(&bridge->media_dev);
	if (bridge->irq > 0)
		free_irq(bridge->irq, pdev);

	return 0;
}

static const struct of_device_id imageon_bridge_of_match[] = {
	{ .compatible = "adi,imageon-v4l2-bridge-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, imageon_bridge_of_match);

static struct platform_driver imageon_bridge_driver = {
	.driver = {
		.name = "imageon-bridge",
		.owner = THIS_MODULE,
		.of_match_table = imageon_bridge_of_match,
	},
	.probe = imageon_bridge_probe,
	.remove = imageon_bridge_remove,
};
module_platform_driver(imageon_bridge_driver);

MODULE_DESCRIPTION("Imageon video bridge");
MODULE_LICENSE("GPL v2");
