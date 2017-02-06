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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/uaccess.h>
#include <linux/virtio.h>

#define RPMSG_TIMEOUT 1000

#define PM_RPMSG_TYPE		2

enum pm_rpmsg_cmd {
	PM_RPMSG_MODE,
	PM_RPMSG_HEART_BEAT,
};

enum pm_rpmsg_power_mode {
	PM_RPMSG_HSRUN,
	PM_RPMSG_RUN,
	PM_RPMSG_VLPR,
	PM_RPMSG_WAIT,
	PM_RPMSG_VLPS,
	PM_RPMSG_VLLS,
	PM_RPMSG_SHUTDOWN,
};

struct pm_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct pm_rpmsg_data *msg;
	struct pm_qos_request pm_qos_req;
};

static struct pm_rpmsg_info pm_rpmsg;

static struct delayed_work heart_beat_work;

struct pm_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 data;
} __attribute__ ((packed));

static int pm_send_message(struct pm_rpmsg_data *msg,
			struct pm_rpmsg_info *info)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	pm_qos_add_request(&info->pm_qos_req,
		PM_QOS_CPU_DMA_LATENCY, 0);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct pm_rpmsg_data));

	pm_qos_remove_request(&info->pm_qos_req);

	return err;
}

void pm_vlls_notify_m4(bool enter)
{
	struct pm_rpmsg_data msg;

	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_MODE;
	msg.data = enter ? PM_RPMSG_VLLS : PM_RPMSG_RUN;

	pm_send_message(&msg, &pm_rpmsg);
}

void pm_shutdown_notify_m4(void)
{
	struct pm_rpmsg_data msg;

	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_MODE;
	msg.data = PM_RPMSG_SHUTDOWN;

	pm_send_message(&msg, &pm_rpmsg);

}

static void pm_heart_beat_work_handler(struct work_struct *work)
{
	struct pm_rpmsg_data msg;

	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_HEART_BEAT;
	msg.data = 0;

	pm_send_message(&msg, &pm_rpmsg);

	schedule_delayed_work(&heart_beat_work,
		msecs_to_jiffies(30000));
}

static int pm_rpmsg_probe(struct rpmsg_device *rpdev)
{
	pm_rpmsg.rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	INIT_DELAYED_WORK(&heart_beat_work,
		pm_heart_beat_work_handler);

	schedule_delayed_work(&heart_beat_work,
		msecs_to_jiffies(10000));

	pm_vlls_notify_m4(false);

	return 0;
}

static int pm_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	return 0;
}

static void pm_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "pm rpmsg driver is removed\n");
}

static struct rpmsg_device_id pm_rpmsg_id_table[] = {
	{ .name	= "rpmsg-life-cycle-channel" },
	{ },
};

static struct rpmsg_driver pm_rpmsg_driver = {
	.drv.name	= "pm_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= pm_rpmsg_id_table,
	.probe		= pm_rpmsg_probe,
	.callback	= pm_rpmsg_cb,
	.remove		= pm_rpmsg_remove,
};

static int __init pm_rpmsg_init(void)
{
	return register_rpmsg_driver(&pm_rpmsg_driver);
}

module_init(pm_rpmsg_init);

MODULE_DESCRIPTION("Freescale PM rpmsg driver");
MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_LICENSE("GPL");
