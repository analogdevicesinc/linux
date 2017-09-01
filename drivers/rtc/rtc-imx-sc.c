/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/arm-smccc.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <soc/imx/fsl_sip.h>
#include <soc/imx8/sc/sci.h>

sc_ipc_t timer_ipcHandle;

struct imx_sc_rtc_data {
	struct rtc_device *rtc;
	spinlock_t lock;
};

struct imx_sc_rtc_data *data;

static int imx_sc_rtc_alarm_sc_notify(struct notifier_block *nb, unsigned long event, void *dummy)
{
	u32 events = 0;

	rtc_update_irq(data->rtc, 1, events);

	return 0;
}

static struct notifier_block imx_sc_rtc_alarm_sc_notifier = {
	.notifier_call = imx_sc_rtc_alarm_sc_notify,
};

static int imx_sc_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	u32 time = 0;
	sc_err_t sciErr = SC_ERR_NONE;

	if (!timer_ipcHandle)
		return -ENODEV;

	sciErr = sc_timer_get_rtc_sec1970(timer_ipcHandle, &time);
	if (sciErr) {
		dev_err(dev, "failed to read time: %d\n", sciErr);
		return -EINVAL;
	}

	rtc_time_to_tm(time, tm);

	return 0;
}

static int imx_sc_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct arm_smccc_res res;

	/* pack 2 time parameters into 1 register, 16 bits for each */
	arm_smccc_smc(FSL_SIP_SRTC, FSL_SIP_SRTC_SET_TIME,
		((tm->tm_year + 1900) << 16) | (tm->tm_mon + 1),
		(tm->tm_mday << 16) | tm->tm_hour,
		(tm->tm_min << 16) | tm->tm_sec,
		0, 0, 0, &res);

	return res.a0;
}

static int imx_sc_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	return 0;
}

static int imx_sc_rtc_alarm_irq_enable(struct device *dev, unsigned int enable)
{
	return 0;
}

static int imx_sc_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	sc_err_t sciErr = SC_ERR_NONE;
	struct rtc_time *alrm_tm = &alrm->time;

	if (!timer_ipcHandle)
		return -ENODEV;

	sciErr = sc_timer_set_rtc_alarm(timer_ipcHandle,
		alrm_tm->tm_year + 1900,
		alrm_tm->tm_mon + 1,
		alrm_tm->tm_mday,
		alrm_tm->tm_hour,
		alrm_tm->tm_min,
		alrm_tm->tm_sec);

	return 0;
}

static const struct rtc_class_ops imx_sc_rtc_ops = {
	.read_time = imx_sc_rtc_read_time,
	.set_time = imx_sc_rtc_set_time,
	.read_alarm = imx_sc_rtc_read_alarm,
	.set_alarm = imx_sc_rtc_set_alarm,
	.alarm_irq_enable = imx_sc_rtc_alarm_irq_enable,
};

static int imx_sc_rtc_probe(struct platform_device *pdev)
{
	int ret = 0;
	uint32_t mu_id;
	sc_err_t sciErr;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	device_init_wakeup(&pdev->dev, true);
	data->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
					&imx_sc_rtc_ops, THIS_MODULE);
	if (IS_ERR(data->rtc)) {
		ret = PTR_ERR(data->rtc);
		dev_err(&pdev->dev, "failed to register rtc: %d\n", ret);
		goto error_rtc_device_register;
	}

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "can not obtain mu id: %d\n", sciErr);
		return sciErr;
	}

	sciErr = sc_ipc_open(&timer_ipcHandle, mu_id);

	if (sciErr != SC_ERR_NONE) {
		dev_err(&pdev->dev, "can not open mu channel to scu: %d\n", sciErr);
		return sciErr;
	};

	register_scu_notifier(&imx_sc_rtc_alarm_sc_notifier);

error_rtc_device_register:

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int imx_sc_rtc_suspend(struct device *dev)
{
	return 0;
}

static int imx_sc_rtc_suspend_noirq(struct device *dev)
{
	return 0;
}

static int imx_sc_rtc_resume(struct device *dev)
{
	return 0;
}

static int imx_sc_rtc_resume_noirq(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops imx_sc_rtc_pm_ops = {
	.suspend = imx_sc_rtc_suspend,
	.suspend_noirq = imx_sc_rtc_suspend_noirq,
	.resume = imx_sc_rtc_resume,
	.resume_noirq = imx_sc_rtc_resume_noirq,
};

#define IMX_SC_RTC_PM_OPS	(&imx_sc_rtc_pm_ops)

#else

#define IMX8_SC_RTC_PM_OPS	NULL

#endif

static const struct of_device_id imx_sc_dt_ids[] = {
	{ .compatible = "fsl,imx-sc-rtc", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_sc_dt_ids);

static struct platform_driver imx_sc_rtc_driver = {
	.driver = {
		.name	= "imx_sc_rtc",
		.pm	= IMX_SC_RTC_PM_OPS,
		.of_match_table = imx_sc_dt_ids,
	},
	.probe		= imx_sc_rtc_probe,
};
module_platform_driver(imx_sc_rtc_driver);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX SC RTC Driver");
MODULE_LICENSE("GPL");
