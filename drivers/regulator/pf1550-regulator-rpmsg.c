/*
 * pf1550.c - regulator driver for the PF1550
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Robin Gong <yibin.gong@freescale.com>
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
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/rpmsg.h>
#include <linux/uaccess.h>
#include <linux/virtio.h>

#define PF1550_DBGFS
#define PF1550_MAX_REGULATOR 7
#define RPMSG_TIMEOUT 1000

enum pf1550_regs {
	PF1550_SW1,
	PF1550_SW2,
	PF1550_SW3,
	PF1550_VREFDDR,
	PF1550_LDO1,
	PF1550_LDO2,
	PF1550_LDO3,
};

enum pf1550_rpmsg_cmd {
	PF1550_ENABLE,
	PF1550_DISABLE,
	PF1550_IS_ENABLED,
	PF1550_SET_VOL,
	PF1550_GET_VOL,
	PF1550_GET_REG,
	PF1550_SET_REG,
};

enum pf1550_resp {
	PF1550_NONE,
	PF1550_SUCCESS,
	PF1550_DISABLED,
	PF1550_NOT_ALLOWED,
};

struct pf1550_regulator_info {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct pf1550_regulator_rpmsg *msg;
	struct completion cmd_complete;
	struct regulator_desc *regulators;
};

static struct pf1550_regulator_info pf1550_info;

struct pf1550_regulator_rpmsg {
	enum pf1550_rpmsg_cmd cmd;
	union {
		enum pf1550_regs regulator;
		u32 reg;
	};
	enum pf1550_resp response;
	union {
		u32 voltage; /* uV */
		u32 val;
	};
};

static int pf1550_send_message(struct pf1550_regulator_rpmsg *msg,
			       struct pf1550_regulator_info *info)
{
	int err;

	msg->response = PF1550_NONE;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			 "rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct pf1550_regulator_rpmsg));
	if (err) {
		dev_err(&info->rpdev->dev, "rpmsg_send failed: %d\n", err);
		return err;
	}
	/* wait response from rpmsg */
	reinit_completion(&info->cmd_complete);
	err = wait_for_completion_timeout(&info->cmd_complete,
					  msecs_to_jiffies(RPMSG_TIMEOUT));
	if (!err) {
		dev_err(&info->rpdev->dev, "rpmsg_send timeout!\n");
		return -ETIMEDOUT;
	}

	dev_dbg(&info->rpdev->dev, "cmd:%d, reg:%d, resp:%d.\n",
		  msg->cmd, msg->regulator, msg->response);

	return 0;
}

static int pf1550_enable(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;

	msg.cmd = PF1550_ENABLE;
	msg.regulator = reg->desc->id;

	return pf1550_send_message(&msg, info);
}

static int pf1550_disable(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;

	msg.cmd = PF1550_DISABLE;
	msg.regulator = reg->desc->id;

	return pf1550_send_message(&msg, info);
}

static int pf1550_is_enabled(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;
	int err;

	msg.cmd = PF1550_IS_ENABLED;
	msg.regulator = reg->desc->id;

	err = pf1550_send_message(&msg, info);
	if (err)
		return err;
	/* Here SUCCESS means ENABLED */
	if (info->msg->response == PF1550_SUCCESS)
		return 1;
	else
		return 0;
}

static int pf1550_set_voltage(struct regulator_dev *reg,
			      int minuV, int uV, unsigned *selector)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;
	int err;

	msg.cmd = PF1550_SET_VOL;
	msg.regulator = reg->desc->id;
	msg.voltage = uV;

	err = pf1550_send_message(&msg, info);
	if (err)
		return err;

	if (info->msg->response == PF1550_NOT_ALLOWED) {
		dev_err(info->dev, "Voltages not allowed to set to %d!\n", uV);
		return -EINVAL;
	}

	return 0;
}

static int pf1550_get_voltage(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;
	int err;

	msg.cmd = PF1550_GET_VOL;
	msg.regulator = reg->desc->id;
	msg.voltage = 0;

	err = pf1550_send_message(&msg, info);
	if (err)
		return err;

	return info->msg->voltage;
}

/* return the fix voltage */
static int pf1550_get_fix_voltage(struct regulator_dev *dev)
{
	return dev->desc->fixed_uV;
}

static struct regulator_ops pf1550_sw_ops = {
	.set_voltage = pf1550_set_voltage,
	.get_voltage = pf1550_get_voltage,
};

static struct regulator_ops pf1550_ldo_ops = {
	.enable = pf1550_enable,
	.disable = pf1550_disable,
	.is_enabled = pf1550_is_enabled,
	.set_voltage = pf1550_set_voltage,
	.get_voltage = pf1550_get_voltage,
};

static struct regulator_ops pf1550_fixed_ops = {
	.enable = pf1550_enable,
	.disable = pf1550_disable,
	.get_voltage = pf1550_get_fix_voltage,
	.is_enabled = pf1550_is_enabled,
};

static struct regulator_desc pf1550_regulators[PF1550_MAX_REGULATOR] = {
{
	.name = "SW1",
	.of_match = of_match_ptr("SW1"),
	.id = PF1550_SW1,
	.ops = &pf1550_sw_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "SW2",
	.of_match = of_match_ptr("SW2"),
	.id = PF1550_SW2,
	.ops = &pf1550_sw_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "SW3",
	.of_match = of_match_ptr("SW3"),
	.id = PF1550_SW3,
	.ops = &pf1550_sw_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "VREFDDR",
	.of_match = of_match_ptr("VREFDDR"),
	.id = PF1550_VREFDDR,
	.ops = &pf1550_fixed_ops,
	.type = REGULATOR_VOLTAGE,
	.fixed_uV = 1200000,
	.owner = THIS_MODULE,
},
{
	.name = "LDO1",
	.of_match = of_match_ptr("LDO1"),
	.id = PF1550_LDO1,
	.ops = &pf1550_ldo_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "LDO2",
	.of_match = of_match_ptr("LDO2"),
	.id = PF1550_LDO2,
	.ops = &pf1550_ldo_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "LDO3",
	.of_match = of_match_ptr("LDO3"),
	.id = PF1550_LDO3,
	.ops = &pf1550_ldo_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};


static int rpmsg_regulator_cb(struct rpmsg_device *rpdev, void *data, int len,
			      void *priv, u32 src)
{
	struct pf1550_regulator_rpmsg *msg = (struct pf1550_regulator_rpmsg *)data;


	dev_dbg(&rpdev->dev, "get from%d: cmd:%d, reg:%d, resp:%d.\n",
		  src, msg->cmd, msg->regulator, msg->response);

	pf1550_info.msg = msg;

	complete(&pf1550_info.cmd_complete);

	return 0;
}

static int rpmsg_regulator_probe(struct rpmsg_device *rpdev)
{
	pf1550_info.rpdev = rpdev;

	init_completion(&pf1550_info.cmd_complete);

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);
	return 0;
}

static void rpmsg_regulator_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "rpmsg regulator driver is removed\n");
}

static struct rpmsg_device_id rpmsg_regulator_id_table[] = {
	/* { .name	= "rpmsg-regulator-channel" }, */
	{ .name	= "rpmsg-openamp-demo-channel" },
	{ },
};

static struct rpmsg_driver rpmsg_regulator_driver = {
	.drv.name	= "regulator_rpmsg",
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsg_regulator_id_table,
	.probe		= rpmsg_regulator_probe,
	.callback	= rpmsg_regulator_cb,
	.remove		= rpmsg_regulator_remove,
};

#ifdef PF1550_DBGFS
#define MAX_REGS 0xff

/*
 * Alligned the below two functions as the same as regmap_map_read_file
 * and regmap_map_write_file in regmap-debugfs.c
 */
static ssize_t pf1550_registers_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i;
	struct platform_device *pdev = to_platform_device(dev);
	struct pf1550_regulator_info *info = platform_get_drvdata(pdev);
	struct pf1550_regulator_rpmsg msg;
	int err;
	size_t bufpos = 0, count = MAX_REGS * 7;

	for (i = 0; i < MAX_REGS; i++) {
		snprintf(buf + bufpos, count - bufpos, "%.*x: ", 2, i);
		bufpos += 4;

		msg.cmd = PF1550_GET_REG;
		msg.reg = i;

		err = pf1550_send_message(&msg, info);
		if (err)
			return err;

		if (info->msg->response != PF1550_SUCCESS) {
			dev_err(info->dev, "Get register failed %x, resp=%x!\n",
				i, info->msg->response);
			return -EINVAL;
		}

		snprintf(buf + bufpos, count - bufpos, "%.*x\n", 2,
			 info->msg->val);
		bufpos += 2;

		buf[bufpos++] = '\n';
	}

	return bufpos;
}

static ssize_t pf1550_register_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pf1550_regulator_info *info = platform_get_drvdata(pdev);
	struct pf1550_regulator_rpmsg msg;
	char *start = (char *)buf;
	unsigned long reg, value;
	int err;

	while (*start == ' ')
		start++;
	reg = simple_strtoul(start, &start, 16);

	while (*start == ' ')
		start++;
	if (kstrtoul(start, 16, &value))
		return -EINVAL;

	msg.cmd = PF1550_SET_REG;
	msg.reg = reg;
	msg.val = value;

	err = pf1550_send_message(&msg, info);
	if (err)
		return err;

	if (info->msg->response != PF1550_SUCCESS) {
		dev_err(info->dev, "set register failed %lx!\n", reg);
		return -EINVAL;
	}

	return size;
}

static DEVICE_ATTR(regs, 0644, pf1550_registers_show, pf1550_register_store);
#endif

static int pf1550_regulator_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int i;
	struct regulator_config config = { };

	if (!np)
		return -ENODEV;

	config.dev = &pdev->dev;
	config.driver_data = &pf1550_info;
	pf1550_info.dev = &pdev->dev;
	pf1550_info.regulators = pf1550_regulators;

	for (i = 0; i < ARRAY_SIZE(pf1550_regulators); i++) {
		struct regulator_dev *rdev;
		struct regulator_desc *desc;

		desc = &pf1550_info.regulators[i];
		rdev = devm_regulator_register(&pdev->dev, desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"Failed to initialize regulator-%d\n", i);
			return PTR_ERR(rdev);
		}
	}

#ifdef PF1550_DBGFS
	i = sysfs_create_file(&config.dev->kobj, &dev_attr_regs.attr);
	if (i) {
		dev_err(&pdev->dev, "Failed to create pf1550 debug sysfs.\n");
		return i;
	}
#endif
	platform_set_drvdata(pdev, &pf1550_info);

	return 0;
}

static const struct of_device_id pf1550_regulator_id[] = {
	{"fsl,pf1550-rpmsg",},
	{},
};

MODULE_DEVICE_TABLE(of, pf1550_regulator_id);

static struct platform_driver pf1550_regulator_driver = {
	.driver = {
		   .name = "pf1550-rpmsg",
		   .owner = THIS_MODULE,
		   .of_match_table = pf1550_regulator_id,
		   },
	.probe = pf1550_regulator_probe,
};

static int __init pf1550_rpmsg_init(void)
{
	return register_rpmsg_driver(&rpmsg_regulator_driver);
}

module_platform_driver(pf1550_regulator_driver);
module_init(pf1550_rpmsg_init);

MODULE_DESCRIPTION("Freescale PF1550 regulator rpmsg driver");
MODULE_AUTHOR("Robin Gong <yibin.gong@freescale.com>");
MODULE_LICENSE("GPL");
