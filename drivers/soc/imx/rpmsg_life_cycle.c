// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 NXP
 */

#include <linux/cpu.h>
#include <linux/imx_rpmsg.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/rpmsg.h>

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
	PM_RPMSG_SHUTDOWN = 7,
};

static struct rpmsg_device *life_cycle_rpdev;

static int rpmsg_life_cycle_notifier(struct notifier_block *nb,
		unsigned long action, void *unused)
{
	int ret, cpu;
	struct pm_rpmsg_data msg;

	/* return early if it is RESTART case */
	if (action == SYS_RESTART)
		return NOTIFY_DONE;

	/*
	 * unplug the non-boot cpu to make sure A35 cluster can be
	 * put into DPD mode without risk.
	 */

	for_each_online_cpu(cpu) {
		if (cpu == cpumask_first(cpu_online_mask))
			continue;
		ret = remove_cpu(cpu);
		if (ret) {
			pr_info("unplug the non-boot cpu failed:%d\n", ret);
			return NOTIFY_BAD;
		}
	}

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

static int __init rpmsg_life_cycle_init(void)
{
	return register_rpmsg_driver(&rpmsg_life_cycle_driver);
};
module_init(rpmsg_life_cycle_init);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP rpmsg life cycle driver");
MODULE_LICENSE("GPL v2");
