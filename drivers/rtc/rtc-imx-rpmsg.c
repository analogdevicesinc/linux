/*
 * Copyright 2017-2018 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
#include <linux/rpmsg.h>
#include <linux/rtc.h>

#define RPMSG_TIMEOUT 1000

#define RTC_RPMSG_SEND		0x0
#define RTC_RPMSG_RECEIVE	0x1
#define RTC_RPMSG_NOTIFY	0x2

enum rtc_rpmsg_cmd {
	RTC_RPMSG_SET_TIME,
	RTC_RPMSG_GET_TIME,
	RTC_RPMSG_SET_ALARM,
	RTC_RPMSG_GET_ALARM,
	RTC_RPMSG_ENABLE_ALARM,
};

struct rtc_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 reserved0;
	union {
		u8 reserved1;
		u8 ret;
	};
	union {
		u32 reserved2;
		u32 sec;
	};
	union {
		u8 enable;
		u8 reserved3;
	};
	union {
		u8 pending;
		u8 reserved4;
	};
} __attribute__ ((packed));

struct rtc_rpmsg_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct rtc_rpmsg_data *msg;
	struct pm_qos_request pm_qos_req;
	struct completion cmd_complete;
	struct mutex lock;
};

static struct rtc_rpmsg_info rtc_rpmsg;

struct imx_rpmsg_rtc_data {
	struct rtc_device *rtc;
	spinlock_t lock;
};

struct imx_rpmsg_rtc_data *data;

static int rtc_send_message(struct rtc_rpmsg_data *msg,
			struct rtc_rpmsg_info *info, bool ack)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			"rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	cpu_latency_qos_add_request(&info->pm_qos_req,
			0);

	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct rtc_rpmsg_data));

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

		if (info->msg->ret != 0) {
			dev_err(&info->rpdev->dev, "rpmsg not ack %d!\n",
				info->msg->ret);
			err = -EINVAL;
			goto err_out;
		}

		err = 0;
	}

err_out:
	cpu_latency_qos_remove_request(&info->pm_qos_req);
	mutex_unlock(&info->lock);

	return err;
}

static int imx_rpmsg_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_rpmsg_data msg;
	int ret;

	msg.header.cate = IMX_RPMSG_RTC;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RTC_RPMSG_SEND;
	msg.header.cmd = RTC_RPMSG_GET_TIME;

	ret = rtc_send_message(&msg, &rtc_rpmsg, true);
	if (ret)
		return ret;

	rtc_time64_to_tm(rtc_rpmsg.msg->sec, tm);

	return 0;
}

static int imx_rpmsg_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct rtc_rpmsg_data msg;
	unsigned long time;
	int ret;

	time = rtc_tm_to_time64(tm);

	msg.header.cate = IMX_RPMSG_RTC;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RTC_RPMSG_SEND;
	msg.header.cmd = RTC_RPMSG_SET_TIME;
	msg.sec = time;

	ret = rtc_send_message(&msg, &rtc_rpmsg, true);
	if (ret)
		return ret;

	return rtc_rpmsg.msg->ret;
}

static int imx_rpmsg_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_rpmsg_data msg;
	int ret;

	msg.header.cate = IMX_RPMSG_RTC;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RTC_RPMSG_SEND;
	msg.header.cmd = RTC_RPMSG_GET_ALARM;

	ret = rtc_send_message(&msg, &rtc_rpmsg, true);
	if (ret)
		return ret;

	rtc_time64_to_tm(rtc_rpmsg.msg->sec, &alrm->time);
	alrm->pending = rtc_rpmsg.msg->pending;

	return rtc_rpmsg.msg->ret;
}

static int imx_rpmsg_rtc_alarm_irq_enable(struct device *dev,
	unsigned int enable)
{
	struct rtc_rpmsg_data msg;
	int ret;

	msg.header.cate = IMX_RPMSG_RTC;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RTC_RPMSG_SEND;
	msg.header.cmd = RTC_RPMSG_ENABLE_ALARM;
	msg.enable = enable;

	ret = rtc_send_message(&msg, &rtc_rpmsg, true);
	if (ret)
		return ret;

	return rtc_rpmsg.msg->ret;
}

static int imx_rpmsg_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_rpmsg_data msg;
	unsigned long time;
	int ret;

	time = rtc_tm_to_time64(&alrm->time);

	msg.header.cate = IMX_RPMSG_RTC;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = RTC_RPMSG_SEND;
	msg.header.cmd = RTC_RPMSG_SET_ALARM;
	msg.sec = time;
	msg.enable = alrm->enabled;

	ret = rtc_send_message(&msg, &rtc_rpmsg, true);
	if (ret)
		return ret;

	return rtc_rpmsg.msg->ret;
}

static int imx_rpmsg_rtc_alarm_irq_update(void)
{
	rtc_update_irq(data->rtc, 1, RTC_IRQF);

	return 0;
}

static const struct rtc_class_ops imx_rpmsg_rtc_ops = {
	.read_time = imx_rpmsg_rtc_read_time,
	.set_time = imx_rpmsg_rtc_set_time,
	.read_alarm = imx_rpmsg_rtc_read_alarm,
	.set_alarm = imx_rpmsg_rtc_set_alarm,
	.alarm_irq_enable = imx_rpmsg_rtc_alarm_irq_enable,
};

static struct rpmsg_device_id rtc_rpmsg_id_table[] = {
	{ .name	= "rpmsg-rtc-channel" },
	{ },
};

static int imx_rpmsg_rtc_probe(struct platform_device *pdev)
{
	int ret = 0;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	device_init_wakeup(&pdev->dev, true);

	data->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
					&imx_rpmsg_rtc_ops, THIS_MODULE);
	if (IS_ERR(data->rtc)) {
		ret = PTR_ERR(data->rtc);
		dev_err(&pdev->dev, "failed to register rtc: %d\n", ret);
	}

	return ret;
}

static const struct of_device_id imx_rpmsg_rtc_dt_ids[] = {
	{ .compatible = "fsl,imx-rpmsg-rtc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_rpmsg_rtc_dt_ids);

static int rtc_rpmsg_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	struct rtc_rpmsg_data *msg = (struct rtc_rpmsg_data *)data;

	if (msg->header.type == RTC_RPMSG_RECEIVE) {
		rtc_rpmsg.msg = msg;
		complete(&rtc_rpmsg.cmd_complete);
		return 0;
	} else if (msg->header.type == RTC_RPMSG_NOTIFY) {
		rtc_rpmsg.msg = msg;
		imx_rpmsg_rtc_alarm_irq_update();
	} else
		dev_err(&rtc_rpmsg.rpdev->dev, "wrong command type!\n");

	return 0;
}

static void rtc_rpmsg_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "rtc rpmsg driver is removed\n");
}

#ifdef CONFIG_PM_SLEEP
static int imx_rpmsg_rtc_suspend(struct device *dev)
{
	return 0;
}

static int imx_rpmsg_rtc_suspend_noirq(struct device *dev)
{
	return 0;
}

static int imx_rpmsg_rtc_resume(struct device *dev)
{
	return 0;
}

static int imx_rpmsg_rtc_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops imx_rpmsg_rtc_pm_ops = {
	.suspend = imx_rpmsg_rtc_suspend,
	.suspend_noirq = imx_rpmsg_rtc_suspend_noirq,
	.resume = imx_rpmsg_rtc_resume,
	.resume_noirq = imx_rpmsg_rtc_resume_noirq,
};

#define IMX_RPMSG_RTC_PM_OPS	(&imx_rpmsg_rtc_pm_ops)

#else

#define IMX_RPMSG_RTC_PM_OPS	NULL

#endif

static struct platform_driver imx_rpmsg_rtc_driver = {
	.driver = {
		.name	= "imx_rpmsg_rtc",
		.pm	= IMX_RPMSG_RTC_PM_OPS,
		.of_match_table = imx_rpmsg_rtc_dt_ids,
	},
	.probe		= imx_rpmsg_rtc_probe,
};

static int rtc_rpmsg_probe(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
		rpdev->src, rpdev->dst);

	rtc_rpmsg.rpdev = rpdev;
	mutex_init(&rtc_rpmsg.lock);

	init_completion(&rtc_rpmsg.cmd_complete);
	return platform_driver_register(&imx_rpmsg_rtc_driver);
}

static struct rpmsg_driver rtc_rpmsg_driver = {
	.drv.name	= "rtc_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= rtc_rpmsg_id_table,
	.probe		= rtc_rpmsg_probe,
	.callback	= rtc_rpmsg_cb,
	.remove		= rtc_rpmsg_remove,
};

static int __init rtc_imx_rpmsg_init(void)
{
	return register_rpmsg_driver(&rtc_rpmsg_driver);
}
late_initcall(rtc_imx_rpmsg_init);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX RPMSG RTC Driver");
MODULE_LICENSE("GPL");
