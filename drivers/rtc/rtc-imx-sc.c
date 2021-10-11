// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2018 NXP.
 */

#include <dt-bindings/firmware/imx/rsrc.h>
#include <linux/arm-smccc.h>
#include <linux/firmware/imx/sci.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/suspend.h>

#define IMX_SC_TIMER_FUNC_GET_RTC_SEC1970	9
#define IMX_SC_TIMER_FUNC_SET_RTC_ALARM		8
#define IMX_SC_TIMER_FUNC_SET_RTC_TIME		6

#define IMX_SIP_SRTC			0xC2000002
#define IMX_SIP_SRTC_SET_TIME		0x0

static struct imx_sc_ipc *rtc_ipc_handle;
static struct rtc_device *imx_sc_rtc;
static bool readonly; /* true if not authorised to set time */

struct imx_sc_msg_timer_get_rtc_time {
	struct imx_sc_rpc_msg hdr;
	u32 time;
} __packed;

struct imx_sc_msg_timer_rtc_set_alarm {
	struct imx_sc_rpc_msg hdr;
	u16 year;
	u8 mon;
	u8 day;
	u8 hour;
	u8 min;
	u8 sec;
} __packed __aligned(4);

#define RTC_NORMAL_MODE		 0
#define RTC_IN_SUSPEND		 1
#define RTC_ABORT_SUSPEND	 2
static int rtc_state = RTC_NORMAL_MODE;

static int imx_sc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct imx_sc_msg_timer_get_rtc_time msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_TIMER;
	hdr->func = IMX_SC_TIMER_FUNC_GET_RTC_SEC1970;
	hdr->size = 1;

	ret = imx_scu_call_rpc(rtc_ipc_handle, &msg, true);
	if (ret) {
		dev_err(dev, "read rtc time failed, ret %d\n", ret);
		return ret;
	}

	rtc_time64_to_tm(msg.time, tm);

	return 0;
}

static int imx_sc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct arm_smccc_res res;

	if (readonly)
		return 0;

	/* pack 2 time parameters into 1 register, 16 bits for each */
	arm_smccc_smc(IMX_SIP_SRTC, IMX_SIP_SRTC_SET_TIME,
		      ((tm->tm_year + 1900) << 16) | (tm->tm_mon + 1),
		      (tm->tm_mday << 16) | tm->tm_hour,
		      (tm->tm_min << 16) | tm->tm_sec,
		      0, 0, 0, &res);

	return res.a0;
}

static int imx_sc_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	return imx_scu_irq_group_enable(IMX_SC_IRQ_GROUP_RTC, IMX_SC_IRQ_RTC, enable);
}

static int imx_sc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct imx_sc_msg_timer_rtc_set_alarm msg;
	struct imx_sc_rpc_msg *hdr = &msg.hdr;
	int ret;
	struct rtc_time *alrm_tm = &alrm->time;

	hdr->ver = IMX_SC_RPC_VERSION;
	hdr->svc = IMX_SC_RPC_SVC_TIMER;
	hdr->func = IMX_SC_TIMER_FUNC_SET_RTC_ALARM;
	hdr->size = 3;

	msg.year = alrm_tm->tm_year + 1900;
	msg.mon = alrm_tm->tm_mon + 1;
	msg.day = alrm_tm->tm_mday;
	msg.hour = alrm_tm->tm_hour;
	msg.min = alrm_tm->tm_min;
	msg.sec = alrm_tm->tm_sec;

	rtc_state = RTC_NORMAL_MODE;

	ret = imx_scu_call_rpc(rtc_ipc_handle, &msg, true);
	if (ret) {
		dev_err(dev, "set rtc alarm failed, ret %d\n", ret);
		return ret;
	}

	ret = imx_sc_rtc_alarm_irq_enable(dev, alrm->enabled);
	if (ret) {
		dev_err(dev, "enable rtc alarm failed, ret %d\n", ret);
		return ret;
	}

	return 0;
}

static const struct rtc_class_ops imx_sc_rtc_ops = {
	.read_time = imx_sc_rtc_read_time,
	.set_time = imx_sc_rtc_set_time,
	.set_alarm = imx_sc_rtc_set_alarm,
	.alarm_irq_enable = imx_sc_rtc_alarm_irq_enable,
};

static int imx_sc_rtc_suspend(struct device *dev)
{
	int err = 0;

	mutex_lock(&imx_sc_rtc->ops_lock);
	/* Abort suspend if the RTC wakeup interrupt triggered during suspend. */
	if (rtc_state == RTC_ABORT_SUSPEND)
		err = -EBUSY;

	rtc_state = RTC_NORMAL_MODE;
	mutex_unlock(&imx_sc_rtc->ops_lock);

	return err;
}

static int imx_sc_rtc_resume(struct device *dev)
{
	return 0;
}

static int imx_sc_rtc_pm_notify(struct notifier_block *nb,
					unsigned long event, void *group)
{
	mutex_lock(&imx_sc_rtc->ops_lock);
	switch (event) {
		case PM_SUSPEND_PREPARE:
			rtc_state = RTC_IN_SUSPEND;
			break;
		case PM_POST_SUSPEND:
			rtc_state = RTC_NORMAL_MODE;
			break;
		default:
			break;
	}
	mutex_unlock(&imx_sc_rtc->ops_lock);
	return 0;
}

static int imx_sc_rtc_alarm_notify(struct notifier_block *nb,
					unsigned long event, void *group)
{
	/* ignore non-rtc irq */
	if (!((event & IMX_SC_IRQ_RTC) && (*(u8 *)group == IMX_SC_IRQ_GROUP_RTC)))
		return 0;

	/* Abort the suspend process if the alarm expired during suspend. */
	mutex_lock(&imx_sc_rtc->ops_lock);
	if (rtc_state == RTC_IN_SUSPEND)
		rtc_state = RTC_ABORT_SUSPEND;
	mutex_unlock(&imx_sc_rtc->ops_lock);
	rtc_update_irq(imx_sc_rtc, 1, RTC_IRQF | RTC_AF);

	return 0;
}

static struct notifier_block imx_sc_rtc_alarm_sc_notifier = {
	.notifier_call = imx_sc_rtc_alarm_notify,
};

static struct notifier_block imx_sc_rtc_pm_notifier = {
	.notifier_call = imx_sc_rtc_pm_notify,
};

static int imx_sc_rtc_probe(struct platform_device *pdev)
{
	int ret;

	ret = imx_scu_get_handle(&rtc_ipc_handle);
	if (ret)
		return ret;

	device_init_wakeup(&pdev->dev, true);

	imx_sc_rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(imx_sc_rtc))
		return PTR_ERR(imx_sc_rtc);

	imx_sc_rtc->ops = &imx_sc_rtc_ops;
	imx_sc_rtc->range_min = 0;
	imx_sc_rtc->range_max = U32_MAX;

	ret = devm_rtc_register_device(imx_sc_rtc);
	if (ret)
		return ret;

	imx_scu_irq_register_notifier(&imx_sc_rtc_alarm_sc_notifier);
	/* Register for PM calls. */
	ret = register_pm_notifier(&imx_sc_rtc_pm_notifier);
	if(ret)
		pr_warn("iMX_SC_RTC: Cannot register suspend notifier, ret = %d\n", ret);

	if (of_property_read_bool(pdev->dev.of_node, "read-only")) {
		readonly = true;
		dev_info(&pdev->dev, "not allowed to change time\n");
	} else {
		readonly = false;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(imx_sc_rtc_pm_ops, imx_sc_rtc_suspend, imx_sc_rtc_resume);

static const struct of_device_id imx_sc_dt_ids[] = {
	{ .compatible = "fsl,imx8qxp-sc-rtc", },
	{ .compatible = "fsl,imx8qm-sc-rtc", },
	{}
};
MODULE_DEVICE_TABLE(of, imx_sc_dt_ids);

static struct platform_driver imx_sc_rtc_driver = {
	.driver = {
		.name	= "imx-sc-rtc",
		.of_match_table = imx_sc_dt_ids,
		.pm = &imx_sc_rtc_pm_ops,
	},
	.probe		= imx_sc_rtc_probe,
};
module_platform_driver(imx_sc_rtc_driver);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX System Controller RTC Driver");
MODULE_LICENSE("GPL");
