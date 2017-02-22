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
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/uaccess.h>
#include <linux/virtio.h>

#define RPMSG_TIMEOUT 1000

#define PM_RPMSG_TYPE		0
#define HEATBEAT_RPMSG_TYPE	2

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
	PM_RPMSG_REBOOT,
	PM_RPMSG_SHUTDOWN,
};

struct pm_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct pm_rpmsg_data *msg;
	struct pm_qos_request pm_qos_req;
	struct notifier_block restart_handler;
	struct completion cmd_complete;
	bool first_flag;
	struct mutex lock;
};

static struct pm_rpmsg_info pm_rpmsg;

static struct delayed_work heart_beat_work;

struct pm_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 data;
} __attribute__ ((packed));

static int pm_send_message(struct pm_rpmsg_data *msg,
			struct pm_rpmsg_info *info, bool ack)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	pm_qos_add_request(&info->pm_qos_req,
			PM_QOS_CPU_DMA_LATENCY, 0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct pm_rpmsg_data));

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

		if (info->msg->data != 0) {
			dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n",
				info->msg->data);
			err = -EINVAL;
			goto err_out;
		}

		err = 0;
	}

err_out:
	pm_qos_remove_request(&info->pm_qos_req);
	mutex_unlock(&info->lock);

	return err;
}

static int pm_vlls_notify_m4(bool enter)
{
	struct pm_rpmsg_data msg;

	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_MODE;
	msg.data = enter ? PM_RPMSG_VLLS : PM_RPMSG_RUN;

	return pm_send_message(&msg, &pm_rpmsg, true);
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

	pm_send_message(&msg, &pm_rpmsg, true);

}

void pm_reboot_notify_m4(void)
{
	struct pm_rpmsg_data msg;

	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_MODE;
	msg.data = PM_RPMSG_REBOOT;

	pm_send_message(&msg, &pm_rpmsg, true);

}

static void pm_heart_beat_work_handler(struct work_struct *work)
{
	struct pm_rpmsg_data msg;

	/* Notify M4 side A7 in RUN mode at boot time */
	if (pm_rpmsg.first_flag) {
		pm_vlls_notify_m4(false);
		pm_rpmsg.first_flag = false;
	}

	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = HEATBEAT_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_HEART_BEAT;
	msg.data = 0;
	pm_send_message(&msg, &pm_rpmsg, false);

	schedule_delayed_work(&heart_beat_work,
		msecs_to_jiffies(30000));
}

static int pm_restart_handler(struct notifier_block *this, unsigned long mode,
				void *cmd)
{
	pm_reboot_notify_m4();

	return NOTIFY_DONE;
}

static int pm_rpmsg_probe(struct rpmsg_device *rpdev)
{
	int ret;

	pm_rpmsg.rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	init_completion(&pm_rpmsg.cmd_complete);
	mutex_init(&pm_rpmsg.lock);

	INIT_DELAYED_WORK(&heart_beat_work,
		pm_heart_beat_work_handler);

	pm_rpmsg.first_flag = true;
	schedule_delayed_work(&heart_beat_work,
			msecs_to_jiffies(100));

	pm_rpmsg.restart_handler.notifier_call = pm_restart_handler;
	pm_rpmsg.restart_handler.priority = 128;
	ret = register_restart_handler(&pm_rpmsg.restart_handler);
	if (ret)
		dev_err(&rpdev->dev, "cannot register restart handler\n");

	return 0;
}

static int pm_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct pm_rpmsg_data *msg = (struct pm_rpmsg_data *)data;

	pm_rpmsg.msg = msg;

	complete(&pm_rpmsg.cmd_complete);

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

#ifdef CONFIG_PM_SLEEP
static int pm_heartbeat_suspend(struct device *dev)
{
	int err;

	err = pm_vlls_notify_m4(true);
	if (err)
		return err;

	cancel_delayed_work_sync(&heart_beat_work);

	return 0;
}

static int pm_heartbeat_resume(struct device *dev)
{
	int err;

	err = pm_vlls_notify_m4(true);
	if (err)
		return err;

	schedule_delayed_work(&heart_beat_work,
			msecs_to_jiffies(10000));

	return 0;
}
#endif

static int pm_heartbeat_probe(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, &pm_rpmsg);

	return register_rpmsg_driver(&pm_rpmsg_driver);
}

static const struct of_device_id pm_heartbeat_id[] = {
	{"fsl,heartbeat-rpmsg",},
	{},
};
MODULE_DEVICE_TABLE(of, pm_heartbeat_id);

static SIMPLE_DEV_PM_OPS(pm_heartbeat_ops, pm_heartbeat_suspend,
			 pm_heartbeat_resume);

static struct platform_driver pm_heartbeat_driver = {
	.driver = {
		.name = "heartbeat-rpmsg",
		.owner = THIS_MODULE,
		.of_match_table = pm_heartbeat_id,
		.pm = &pm_heartbeat_ops,
		},
	.probe = pm_heartbeat_probe,
};

module_platform_driver(pm_heartbeat_driver);

MODULE_DESCRIPTION("Freescale PM rpmsg driver");
MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_LICENSE("GPL");
