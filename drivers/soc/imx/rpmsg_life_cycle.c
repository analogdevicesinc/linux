// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <linux/cpu.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>
#include <linux/suspend.h>

#define RPMSG_TIMEOUT 1000

#define PM_RPMSG_TYPE		0

struct pm_rpmsg_data {
	struct imx_rpmsg_head header;
	u8 data;
	u8 reserved;
} __packed;

enum pm_rpmsg_cmd {
	PM_RPMSG_MODE,
};

enum pm_rpmsg_power_mode {
	PM_RPMSG_ACTIVE = 1,
	PM_RPMSG_SUSPEND = 5,
	PM_RPMSG_SHUTDOWN = 7,
};

static struct rpmsg_device *life_cycle_rpdev;
static struct completion cmd_complete;

static int rpmsg_life_cycle_notifier(struct notifier_block *nb,
		unsigned long action, void *unused)
{
	int ret;
#ifdef CONFIG_HOTPLUG_CPU
	int cpu;
#endif
	struct pm_rpmsg_data msg = {};

	/* return early if it is RESTART case */
	if (action == SYS_RESTART)
		return NOTIFY_DONE;

	/*
	 * unplug the non-boot cpu to make sure A35 cluster can be
	 * put into DPD mode without risk.
	 */
#ifdef CONFIG_HOTPLUG_CPU
	for_each_online_cpu(cpu) {
		if (cpu == cpumask_first(cpu_online_mask))
			continue;
		ret = remove_cpu(cpu);
		if (ret) {
			pr_info("unplug the non-boot cpu failed:%d\n", ret);
			return NOTIFY_BAD;
		}
	}
#endif
	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_MODE;
	msg.data = PM_RPMSG_SHUTDOWN;

	/* No ACK from M core */
	ret = rpmsg_send(life_cycle_rpdev->ept, &msg, sizeof(struct pm_rpmsg_data));

	if (ret) {
		pr_info("rpmsg send failed:%d\n", ret);
		return NOTIFY_BAD;
	}

	return NOTIFY_DONE;
};

static struct notifier_block rpmsg_life_cycle_nb = {
	.notifier_call = rpmsg_life_cycle_notifier,
};

static int rpmsg_life_cycle_cb(struct rpmsg_device *rpdev, void *data, int len,
				void *priv, u32 src)
{
	/* no need to handle the received msg, just complete */
	complete(&cmd_complete);

	return 0;
}

static int rpmsg_life_cycle_probe(struct rpmsg_device *rpdev)
{
	life_cycle_rpdev = rpdev;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	return register_reboot_notifier(&rpmsg_life_cycle_nb);
}

static struct rpmsg_device_id rpmsg_life_cycle_id_table[] = {
	{ .name = "rpmsg-life-cycle-channel" },
	{ },
};

static struct rpmsg_driver rpmsg_life_cycle_driver = {
	.drv.name = 	"rpmsg_life_cycle",
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_life_cycle_id_table,
	.probe		= rpmsg_life_cycle_probe,
	.callback	= rpmsg_life_cycle_cb,
};

static int __maybe_unused rpmsg_lifecycle_pm_notify(bool enter)
{
	struct pm_rpmsg_data msg = {};
	int ret;

	/* Only need to do lifecycle notify when APD enter mem(HW PD) mode */
	if (pm_suspend_target_state != PM_SUSPEND_MEM)
		return 0;

	/* Bypass if no lifecycle device */
	if (!life_cycle_rpdev)
		return 0;

	msg.data = enter ? PM_RPMSG_SUSPEND : PM_RPMSG_ACTIVE;
	msg.header.cate = IMX_RMPSG_LIFECYCLE;
	msg.header.major = IMX_RMPSG_MAJOR;
	msg.header.minor = IMX_RMPSG_MINOR;
	msg.header.type = PM_RPMSG_TYPE;
	msg.header.cmd = PM_RPMSG_MODE;

	reinit_completion(&cmd_complete);

	ret = rpmsg_send(life_cycle_rpdev->ept, &msg, sizeof(struct pm_rpmsg_data));
	if (ret) {
		dev_err(&life_cycle_rpdev->dev, "rpmsg send failed:%d\n", ret);
		return ret;
	}

	ret = wait_for_completion_timeout(&cmd_complete,
					msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!ret) {
		dev_err(&life_cycle_rpdev->dev, "rpmsg_send timeout!\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int __maybe_unused rpmsg_lifecycle_suspend_noirq(struct device *dev)
{
	return rpmsg_lifecycle_pm_notify(true);
}

static int __maybe_unused rpmsg_lifecycle_resume_noirq(struct device *dev)
{
	return rpmsg_lifecycle_pm_notify(false);
}

static const struct dev_pm_ops rpmsg_lifecyle_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(rpmsg_lifecycle_suspend_noirq,
			rpmsg_lifecycle_resume_noirq)
};

static int rpmsg_lifecycle_probe(struct platform_device *pdev)
{
	init_completion(&cmd_complete);

	return register_rpmsg_driver(&rpmsg_life_cycle_driver);
}

static const struct of_device_id rpmsg_lifecycle_id[] = {
	{ "nxp,rpmsg-lifecycle", },
	{},
};
MODULE_DEVICE_TABLE(of, rpmsg_lifecycle_id);

static struct platform_driver rpmsg_lifecycle_platform_driver = {
	.driver = {
		.name = "rpmsg-lifecycle",
		.owner = THIS_MODULE,
		.of_match_table = rpmsg_lifecycle_id,
		.pm = &rpmsg_lifecyle_ops,
	},
	.probe = rpmsg_lifecycle_probe,
};
module_platform_driver(rpmsg_lifecycle_platform_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP rpmsg life cycle driver");
MODULE_LICENSE("GPL v2");
