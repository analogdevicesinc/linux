/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/imx_rpmsg.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/virtio.h>

#define IMX_RPMSG_GPIO_PER_PORT	32
#define RPMSG_TIMEOUT	1000

enum gpio_input_trigger_type {
	GPIO_RPMSG_TRI_IGNORE,
	GPIO_RPMSG_TRI_RISING,
	GPIO_RPMSG_TRI_FALLING,
	GPIO_RPMSG_TRI_BOTH_EDGE,
	GPIO_RPMSG_TRI_LOW_LEVEL,
	GPIO_RPMSG_TRI_HIGH_LEVEL,
};

enum gpio_rpmsg_header_type {
	GPIO_RPMSG_SETUP,
	GPIO_RPMSG_REPLY,
	GPIO_RPMSG_NOTIFY,
};

enum gpio_rpmsg_header_cmd {
	GPIO_RPMSG_INPUT_INIT,
	GPIO_RPMSG_OUTPUT_INIT,
	GPIO_RPMSG_INPUT_GET,
};

struct gpio_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 pin_idx;
	u8 port_idx;
	union {
		u8 event;
		u8 retcode;
		u8 value;
	} out;
	union {
		u8 wakeup;
		u8 value;
	} in;
} __packed __aligned(8);

struct imx_rpmsg_gpio_port {
	struct gpio_chip gc;
	struct gpio_rpmsg_data msg;
	int idx;
};

struct imx_gpio_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct gpio_rpmsg_data *notify_msg;
	struct gpio_rpmsg_data *reply_msg;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct mutex lock;
};

static struct imx_gpio_rpmsg_info gpio_rpmsg;

static int gpio_send_message(struct imx_rpmsg_gpio_port *port,
	struct gpio_rpmsg_data *msg, struct imx_gpio_rpmsg_info *info)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(&info->rpdev->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	pm_qos_add_request(&info->pm_qos_req,
			PM_QOS_CPU_DMA_LATENCY, 0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct gpio_rpmsg_data));

	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		goto err_out;
	}

	err = wait_for_completion_timeout(&info->cmd_complete,
				msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!err) {
		dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
		err = -ETIMEDOUT;
		goto err_out;
	}

	if (info->reply_msg->out.retcode != 0) {
		dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n",
			info->reply_msg->out.retcode);
		err = -EINVAL;
		goto err_out;
	}

	/* copy the reply message */
	memcpy(&port->msg, info->reply_msg, sizeof(*info->reply_msg));

	err = 0;

err_out:
	pm_qos_remove_request(&info->pm_qos_req);
	mutex_unlock(&info->lock);

	return err;
}

static int gpio_rpmsg_cb(struct rpmsg_device *rpdev,
	void *data, int len, void *priv, u32 src)
{
	struct gpio_rpmsg_data *msg = (struct gpio_rpmsg_data *)data;

	if (msg->header.type == GPIO_RPMSG_REPLY) {
		gpio_rpmsg.reply_msg = msg;
		complete(&gpio_rpmsg.cmd_complete);
	} else if (msg->header.type == GPIO_RPMSG_NOTIFY) {
		gpio_rpmsg.notify_msg = msg;
		/* TBD for interrupt handler */
	} else
		dev_err(&gpio_rpmsg.rpdev->dev, "wrong command type!\n");

	return 0;
}

static int imx_rpmsg_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg;
	int ret;

	memset(&msg, 0, sizeof(struct gpio_rpmsg_data));
	msg.header.cate = IMX_RPMSG_GPIO;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = GPIO_RPMSG_SETUP;
	msg.header.cmd = GPIO_RPMSG_INPUT_GET;
	msg.pin_idx = gpio;
	msg.port_idx = port->idx;

	ret = gpio_send_message(port, &msg, &gpio_rpmsg);
	if (!ret)
		return !!port->msg.in.value;

	return ret;
}

static int imx_rpmsg_gpio_direction_input(struct gpio_chip *gc,
					  unsigned int gpio)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg;

	memset(&msg, 0, sizeof(struct gpio_rpmsg_data));
	msg.header.cate = IMX_RPMSG_GPIO;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = GPIO_RPMSG_SETUP;
	msg.header.cmd = GPIO_RPMSG_INPUT_INIT;
	msg.pin_idx = gpio;
	msg.port_idx = port->idx;

	/* TBD: get event trigger and wakeup from GPIO descriptor */
	msg.out.event = GPIO_RPMSG_TRI_IGNORE;
	msg.in.wakeup = 0;

	return gpio_send_message(port, &msg, &gpio_rpmsg);
}

static inline void imx_rpmsg_gpio_direction_output_init(struct gpio_chip *gc,
		unsigned int gpio, int val, struct gpio_rpmsg_data *msg)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);

	msg->header.cate = IMX_RPMSG_GPIO;
	msg->header.major = IMX_RMPSG_MAJOR;
	msg->header.minor = IMX_RMPSG_MINOR;
	msg->header.type = GPIO_RPMSG_SETUP;
	msg->header.cmd = GPIO_RPMSG_OUTPUT_INIT;
	msg->pin_idx = gpio;
	msg->port_idx = port->idx;
	msg->out.value = val;
}

static void imx_rpmsg_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg;

	memset(&msg, 0, sizeof(struct gpio_rpmsg_data));
	imx_rpmsg_gpio_direction_output_init(gc, gpio, val, &msg);
	gpio_send_message(port, &msg, &gpio_rpmsg);
}

static int imx_rpmsg_gpio_direction_output(struct gpio_chip *gc,
					unsigned int gpio, int val)
{
	struct imx_rpmsg_gpio_port *port = gpiochip_get_data(gc);
	struct gpio_rpmsg_data msg;

	memset(&msg, 0, sizeof(struct gpio_rpmsg_data));
	imx_rpmsg_gpio_direction_output_init(gc, gpio, val, &msg);
	return gpio_send_message(port, &msg, &gpio_rpmsg);
}

static int gpio_rpmsg_probe(struct rpmsg_device *rpdev)
{
	gpio_rpmsg.rpdev = rpdev;
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&gpio_rpmsg.cmd_complete);
	mutex_init(&gpio_rpmsg.lock);

	return 0;
}

static struct rpmsg_device_id gpio_rpmsg_id_table[] = {
	{ .name = "rpmsg-io-channel" },
	{},
};

static struct rpmsg_driver gpio_rpmsg_driver = {
	.drv.name	= "gpio_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= gpio_rpmsg_id_table,
	.probe		= gpio_rpmsg_probe,
	.callback	= gpio_rpmsg_cb,
};

static int imx_rpmsg_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct imx_rpmsg_gpio_port *port;
	struct gpio_chip *gc;
	int ret;

	port = devm_kzalloc(&pdev->dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	ret = of_property_read_u32(np, "port_idx", &port->idx);
	if (ret)
		return ret;

	gc = &port->gc;
	gc->of_node = np;
	gc->parent = dev;
	gc->label = "imx-rpmsg-gpio";
	gc->ngpio = IMX_RPMSG_GPIO_PER_PORT;
	gc->base = of_alias_get_id(np, "gpio") * IMX_RPMSG_GPIO_PER_PORT;

	gc->direction_input = imx_rpmsg_gpio_direction_input;
	gc->direction_output = imx_rpmsg_gpio_direction_output;
	gc->get = imx_rpmsg_gpio_get;
	gc->set = imx_rpmsg_gpio_set;

	platform_set_drvdata(pdev, port);

	ret = devm_gpiochip_add_data(dev, gc, port);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct of_device_id imx_rpmsg_gpio_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-gpio" },
	{ /* sentinel */ }
};

static struct platform_driver imx_rpmsg_gpio_driver = {
	.driver	= {
		.name = "gpio-imx-rpmsg",
		.of_match_table = imx_rpmsg_gpio_dt_ids,
	},
	.probe = imx_rpmsg_gpio_probe,
};

static int __init gpio_imx_rpmsg_init(void)
{
	int ret;

	ret = register_rpmsg_driver(&gpio_rpmsg_driver);
	if (ret)
		return ret;

	return platform_driver_register(&imx_rpmsg_gpio_driver);
}
device_initcall(gpio_imx_rpmsg_init);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX7ULP rpmsg gpio driver");
MODULE_LICENSE("GPL v2");
