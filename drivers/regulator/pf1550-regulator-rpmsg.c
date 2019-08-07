/*
 * pf1550.c - regulator driver for the PF1550
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright (C) 2017 NXP.
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
#include <linux/imx_rpmsg.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_qos.h>
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
	PF1550_SUCCESS,
	PF1550_FAILED,
	PF1550_UNSURPPORT,
};

enum pf1550_status {
	PF1550_DISABLED,
	PF1550_ENABLED,
};

struct pf1550_regulator_info {
	struct rpmsg_device *rpdev;
	bool is_ready;
	struct device *dev;
	struct pf1550_regulator_rpmsg *msg;
	struct completion cmd_complete;
	struct pm_qos_request pm_qos_req;
	struct mutex lock;
	struct regulator_desc *regulators;
};

static struct pf1550_regulator_info pf1550_info;

struct pf1550_regulator_rpmsg {
	/* common head */
	struct imx_rpmsg_head header;
	/* pmic structure */
	union {
		u8 regulator;
		u8 reg;
	};
	u8 response;
	u8 status;
	union {
		u32 voltage; /* uV */
		u32 val;
	};
} __attribute__ ((packed));

static int pf1550_send_message(struct pf1550_regulator_rpmsg *msg,
			       struct pf1550_regulator_info *info)
{
	int err;

	if (!info->rpdev) {
		dev_dbg(info->dev,
			 "rpmsg channel not ready, m4 image ready?\n");
		return -EINVAL;
	}

	mutex_lock(&info->lock);
	cpu_latency_qos_add_request(&info->pm_qos_req, 0);

	msg->header.cate = IMX_RPMSG_PMIC;
	msg->header.major = IMX_RMPSG_MAJOR;
	msg->header.minor = IMX_RMPSG_MINOR;
	msg->header.type = 0;

	/* wait response from rpmsg */
	reinit_completion(&info->cmd_complete);

	err = rpmsg_send(info->rpdev->ept, (void *)msg,
			    sizeof(struct pf1550_regulator_rpmsg));

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

	err = 0;

err_out:
	cpu_latency_qos_remove_request(&info->pm_qos_req);
	mutex_unlock(&info->lock);

	dev_dbg(&info->rpdev->dev, "cmd:%d, reg:%d, resp:%d.\n",
		  msg->header.cmd, msg->regulator, msg->response);

	return err;
}

static int pf1550_enable(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;

	msg.header.cmd = PF1550_ENABLE;
	msg.regulator = reg->desc->id;

	return pf1550_send_message(&msg, info);
}

static int pf1550_disable(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;

	msg.header.cmd = PF1550_DISABLE;
	msg.regulator = reg->desc->id;

	return pf1550_send_message(&msg, info);
}

static int pf1550_is_enabled(struct regulator_dev *reg)
{
	struct pf1550_regulator_info *info = rdev_get_drvdata(reg);
	struct pf1550_regulator_rpmsg msg;
	int err;

	msg.header.cmd = PF1550_IS_ENABLED;
	msg.regulator = reg->desc->id;

	err = pf1550_send_message(&msg, info);
	if (err)
		return err;
	/* Here SUCCESS means ENABLED */
	if (info->msg->status == PF1550_ENABLED)
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

	msg.header.cmd = PF1550_SET_VOL;
	msg.regulator = reg->desc->id;
	msg.voltage = minuV;

	err = pf1550_send_message(&msg, info);
	if (err)
		return err;

	if (info->msg->response == PF1550_UNSURPPORT) {
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

	msg.header.cmd = PF1550_GET_VOL;
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

static const int pf1550_ldo13_volts[] = {
	750000, 800000, 850000, 900000, 950000, 1000000, 1050000, 1100000,
	1150000, 1200000, 1250000, 1300000, 1350000, 1400000, 1450000, 1500000,
	1800000, 1900000, 2000000, 2100000, 2200000, 2300000, 2400000, 2500000,
	2600000, 2700000, 2800000, 2900000, 3000000, 3100000, 3200000, 3300000,
};

static struct regulator_ops pf1550_sw_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.set_voltage = pf1550_set_voltage,
	.get_voltage = pf1550_get_voltage,
};

static struct regulator_ops pf1550_ldo_ops = {
	.enable = pf1550_enable,
	.disable = pf1550_disable,
	.is_enabled = pf1550_is_enabled,
	.list_voltage = regulator_list_voltage_table,
	.set_voltage = pf1550_set_voltage,
	.get_voltage = pf1550_get_voltage,
};

static struct regulator_ops pf1550_ldo2_ops = {
	.enable = pf1550_enable,
	.disable = pf1550_disable,
	.is_enabled = pf1550_is_enabled,
	.list_voltage = regulator_list_voltage_linear,
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
	.n_voltages = (1387500 - 600000) / 12500 + 1,
	.min_uV = 600000,
	.uV_step = 12500,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "SW2",
	.of_match = of_match_ptr("SW2"),
	.id = PF1550_SW2,
	.ops = &pf1550_sw_ops,
	.n_voltages = (1387500 - 600000) / 12500 + 1,
	.min_uV = 600000,
	.uV_step = 12500,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "SW3",
	.of_match = of_match_ptr("SW3"),
	.id = PF1550_SW3,
	.ops = &pf1550_sw_ops,
	.n_voltages = (3300000 - 1800000) / 100000 + 1,
	.min_uV = 1800000,
	.uV_step = 100000,
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
	.n_voltages = ARRAY_SIZE(pf1550_ldo13_volts),
	.volt_table = pf1550_ldo13_volts,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "LDO2",
	.of_match = of_match_ptr("LDO2"),
	.id = PF1550_LDO2,
	.ops = &pf1550_ldo2_ops,
	.n_voltages = (3300000 - 1800000) / 100000 + 1,
	.min_uV = 1800000,
	.uV_step = 100000,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
{
	.name = "LDO3",
	.of_match = of_match_ptr("LDO3"),
	.id = PF1550_LDO3,
	.ops = &pf1550_ldo_ops,
	.n_voltages = ARRAY_SIZE(pf1550_ldo13_volts),
	.volt_table = pf1550_ldo13_volts,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
},
};


static int rpmsg_regulator_cb(struct rpmsg_device *rpdev, void *data, int len,
			      void *priv, u32 src)
{
	struct pf1550_regulator_rpmsg *msg = (struct pf1550_regulator_rpmsg *)data;

	dev_dbg(&rpdev->dev, "get from%d: cmd:%d, reg:%d, resp:%d.\n",
		  src, msg->header.cmd, msg->regulator, msg->response);

	pf1550_info.msg = msg;

	complete(&pf1550_info.cmd_complete);

	return 0;
}

static int rpmsg_regulator_probe(struct rpmsg_device *rpdev)
{
	pf1550_info.rpdev = rpdev;

	init_completion(&pf1550_info.cmd_complete);
	mutex_init(&pf1550_info.lock);

	pf1550_info.is_ready = true;

	dev_info(&rpdev->dev, "new channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);
	return 0;
}

static void rpmsg_regulator_remove(struct rpmsg_device *rpdev)
{
	dev_info(&rpdev->dev, "rpmsg regulator driver is removed\n");
}

static struct rpmsg_device_id rpmsg_regulator_id_table[] = {
	{ .name	= "rpmsg-regulator-channel" },
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
 * Aligned the below two functions as the same as regmap_map_read_file
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

		msg.header.cmd = PF1550_GET_REG;
		msg.reg = i;
		msg.val = 0;

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

	msg.header.cmd = PF1550_SET_REG;
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

	if (!pf1550_info.is_ready)
		return -EPROBE_DEFER;

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
