/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/slab.h>
#include <linux/imx_rpmsg.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/uaccess.h>
#include <linux/virtio.h>

#define RPMSG_TIMEOUT 1000

enum key_cmd_type {
	KEY_RPMSG_SETUP,
	KEY_RPMSG_REPLY,
	KEY_RPMSG_NOTIFY,
};

enum keys_type {
	KEY_PRESS = 1,
	KEY_RELEASE,
	KEY_BOTH,
};

struct key_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 key_index;
	union {
		u8 event;
		u8 retcode;
	};
	u8 wakeup;
} __attribute__((packed));

struct rpmsg_keys_button {
	unsigned int code;
	enum keys_type type;
	int wakeup;
	struct input_dev *input;
};

struct rpmsg_keys_drvdata {
	struct input_dev *input;
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct key_rpmsg_data *msg;
	bool ack;
	struct pm_qos_request pm_qos_req;
	struct delayed_work keysetup_work;
	struct completion cmd_complete;
	int nbuttons;
	struct rpmsg_keys_button buttons[0];
};

static struct rpmsg_keys_drvdata *keys_rpmsg;

static int key_send_message(struct key_rpmsg_data *msg,
			struct rpmsg_keys_drvdata *info, bool ack)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	pm_qos_add_request(&info->pm_qos_req,
			PM_QOS_CPU_DMA_LATENCY, 0);

	if (ack) {
		info->ack = true;
		reinit_completion(&info->cmd_complete);
	}

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct key_rpmsg_data));
	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	if (ack) {
		err = wait_for_completion_timeout(&info->cmd_complete,
					msecs_to_jiffies(RPMSG_TIMEOUT));
		if (!err) {
			dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
			err = -ETIMEDOUT;
			goto err_out;
		}

		if (info->msg->retcode != 0) {
			dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n",
				info->msg->retcode);
			err = -EINVAL;
			goto err_out;
		}

		err = 0;
	}

err_out:
	info->ack = true;
	pm_qos_remove_request(&info->pm_qos_req);

	return err;
}

static int keys_rpmsg_cb(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 src)
{
	struct key_rpmsg_data *msg = (struct key_rpmsg_data *)data;

	if (msg->header.type == KEY_RPMSG_REPLY) {
		keys_rpmsg->msg = msg;
		complete(&keys_rpmsg->cmd_complete);
		return 0;
	} else if (msg->header.type == KEY_RPMSG_NOTIFY) {
		keys_rpmsg->msg = msg;
		keys_rpmsg->ack = false;
	} else
		dev_err(&keys_rpmsg->rpdev->dev, "wrong command type!\n");

	input_event(keys_rpmsg->input, EV_KEY, msg->key_index, msg->event);
	input_sync(keys_rpmsg->input);

	return 0;
}

static void keys_init_handler(struct work_struct *work)
{
	struct key_rpmsg_data msg;
	int i;

	/* setup keys */
	for (i = 0; i < keys_rpmsg->nbuttons; i++) {
		struct rpmsg_keys_button *button = &keys_rpmsg->buttons[i];

		msg.header.cate = IMX_RPMSG_KEY;
		msg.header.major = IMX_RMPSG_MAJOR;
		msg.header.minor = IMX_RMPSG_MINOR;
		msg.header.type = KEY_RPMSG_SETUP;
		msg.header.cmd = 0;
		msg.key_index = button->code;
		msg.wakeup = button->wakeup;
		msg.event = button->type;
		if (key_send_message(&msg, keys_rpmsg, true))
			dev_err(&keys_rpmsg->rpdev->dev,
				"key %d setup failed!\n", button->code);
	}
}

static int keys_rpmsg_probe(struct rpmsg_device *rpdev)
{
	keys_rpmsg->rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&keys_rpmsg->cmd_complete);

	INIT_DELAYED_WORK(&keys_rpmsg->keysetup_work,
				keys_init_handler);
	schedule_delayed_work(&keys_rpmsg->keysetup_work,
			msecs_to_jiffies(100));

	return 0;
}

static struct rpmsg_device_id keys_rpmsg_id_table[] = {
	{ .name	= "rpmsg-keypad-channel" },
	{ },
};

static struct rpmsg_driver keys_rpmsg_driver = {
	.drv.name	= "key_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= keys_rpmsg_id_table,
	.probe		= keys_rpmsg_probe,
	.callback	= keys_rpmsg_cb,
};

static struct rpmsg_keys_drvdata *
rpmsg_keys_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct rpmsg_keys_drvdata *ddata;
	struct rpmsg_keys_button *button;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node)
		return ERR_PTR(-ENODEV);

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	ddata = devm_kzalloc(dev,
			     sizeof(*ddata) + nbuttons *
			     sizeof(struct rpmsg_keys_button),
			     GFP_KERNEL);
	if (!ddata)
		return ERR_PTR(-ENOMEM);

	ddata->nbuttons = nbuttons;

	i = 0;
	for_each_child_of_node(node, pp) {
		button = &ddata->buttons[i++];

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
				button->code);
			return ERR_PTR(-EINVAL);
		}

		button->wakeup = !!of_get_property(pp, "rpmsg-key,wakeup",
							NULL);
		button->type = KEY_BOTH;
	}

	return ddata;
}

static int rpmsg_keys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rpmsg_keys_drvdata *ddata;
	int i, error;
	struct input_dev *input;

	ddata = rpmsg_keys_get_devtree_pdata(dev);
	if (IS_ERR(ddata))
		return PTR_ERR(ddata);

	input = devm_input_allocate_device(dev);
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	ddata->input = input;

	keys_rpmsg = ddata;
	platform_set_drvdata(pdev, ddata);

	input->name = pdev->name;
	input->phys = "rpmsg-keys/input0";
	input->dev.parent = &pdev->dev;

	input->id.bustype = BUS_HOST;

	for (i = 0; i < ddata->nbuttons; i++) {
		struct rpmsg_keys_button *button = &ddata->buttons[i];

		input_set_capability(input, EV_KEY, button->code);
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto err_out;
	}

	return register_rpmsg_driver(&keys_rpmsg_driver);
err_out:
	return error;
}

static const struct of_device_id rpmsg_keys_of_match[] = {
	{ .compatible = "fsl,rpmsg-keys", },
	{ },
};

MODULE_DEVICE_TABLE(of, rpmsg_keys_of_match);

static struct platform_driver rpmsg_keys_device_driver = {
	.probe		= rpmsg_keys_probe,
	.driver		=  {
		.name	= "rpmsg-keys",
		.of_match_table = of_match_ptr(rpmsg_keys_of_match)
	}
};

module_platform_driver(rpmsg_keys_device_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Robin Gong <yibin.gong@nxp.com>");
MODULE_DESCRIPTION("Keyboard driver based on rpmsg");
