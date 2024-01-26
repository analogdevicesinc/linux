// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Analog Devices Trigger Routing Unit as used with the inter-core
 * communications driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *
 */

#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <linux/soc/adi/icc.h>

#define ADI_TRU_REG_GCTL		0x7f4
#define ADI_TRU_REG_MTR			0x7e0

#define ADI_TRU_DEFAULT_MAX_MASTER_ID		180
#define ADI_TRU_DEFAULT_MAX_SLAVE_ID		180

#define ARM_SMCCC_OWNER_ADI 51
#define ADI_SMC_FUNCID_TRU_TRIGGER 0

#define ADI_TRU_SMC_TRIGGER \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, ARM_SMCCC_SMC_32, \
		ARM_SMCCC_OWNER_ADI, ADI_SMC_FUNCID_TRU_TRIGGER)

struct adi_tru {
	void __iomem *ioaddr;
	struct device *dev;
	u32 max_master_id;
	u32 max_slave_id;
	bool use_smc;
};

/**
 * Device tree interface for other modules that need TRU access
 */
struct adi_tru *get_adi_tru_from_node(struct device *dev)
{
	struct platform_device *tru_pdev;
	struct device_node *tru_node;
	struct adi_tru *ret = NULL;

	tru_node = of_parse_phandle(dev->of_node, "adi,tru", 0);
	if (!tru_node) {
		dev_err(dev, "Missing adi,tru phandle in device tree\n");
		return ERR_PTR(-ENODEV);
	}

	tru_pdev = of_find_device_by_node(tru_node);
	if (!tru_pdev) {
		ret = ERR_PTR(-EPROBE_DEFER);
		goto cleanup;
	}

	ret = dev_get_drvdata(&tru_pdev->dev);
	if (!ret)
		ret = ERR_PTR(-EPROBE_DEFER);

cleanup:
	of_node_put(tru_node);
	return ret;
}
EXPORT_SYMBOL(get_adi_tru_from_node);

void put_adi_tru(struct adi_tru *tru)
{
	put_device(tru->dev);
}
EXPORT_SYMBOL(put_adi_tru);

int adi_tru_trigger_device(struct adi_tru *tru, struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 master = 0;

	if (of_property_read_u32(np, "adi,tru-master-id", &master)) {
		dev_err(tru->dev, "dts entry %s is missing a adi,tru-master-id",
			np->full_name);
		return -ENOENT;
	}

	return adi_tru_trigger(tru, master);
}
EXPORT_SYMBOL(adi_tru_trigger_device);

static int adi_tru_smc_trigger(struct adi_tru *tru, u32 master) {
	struct arm_smccc_res res;
	arm_smccc_smc(ADI_TRU_SMC_TRIGGER, master, 0, 0, 0, 0, 0, 0, &res);
	return (res.a0 == 0) ? 0 : -EINVAL;
}

int adi_tru_trigger(struct adi_tru *tru, u32 master)
{
	if (master == 0 || master > tru->max_master_id) {
		dev_err(tru->dev, "Invalid master ID to trigger %d\n", master);
		return -ERANGE;
	}

	if (tru->use_smc)
		return adi_tru_smc_trigger(tru, master);

	writel(master, tru->ioaddr + ADI_TRU_REG_MTR);
	return 0;
}
EXPORT_SYMBOL(adi_tru_trigger);

/**
 * Configure the given slave (i.e. TRU_SSR[n]) to be triggered by the given
 * master ID. The IDs found in the documentation, which appear to be 1-indexed
 * for masters (valid IDs are 1-182) and 0-indexed for slaves (start at SSR[0] and
 * count to SSR[187]) should be used as-is. There's effectively an ID 0 master
 * that is unused and undocumented.
 */
int adi_tru_set_trigger_by_id(struct adi_tru *tru, u32 master, u32 slave)
{
	if (slave > tru->max_slave_id) {
		dev_err(tru->dev, "Invalid slave ID %d passed to %s", slave, __func__);
		return -ERANGE;
	}

	if (master > tru->max_master_id || master == 0) {
		dev_err(tru->dev, "Invalid master ID %d passed to %s", slave, __func__);
		return -ERANGE;
	}

	if (!tru->use_smc) {
		dev_info(tru->dev, "Connecting master %d to slave %d\n", master, slave);
		writel(master, tru->ioaddr + (slave*4));
	}
	else {
		dev_err(tru->dev, "Cannot dynamically adjust TRU configuration when "
			"TRU control from OPTEE is enabled\n");
	}
	return 0;
}

/**
 * Configure by device tree nodes, which lets us have more flexible configurations,
 * although another layer of phandle may be needed on top:
 * a: a@0 {
 *  adi,tru-master-id = <100>;
 * };
 * b: b@0 {
 *  adi,tru-slave-id = <101>;
 * };
 * icc {
 *   adi,a = <&a>;
 *   adi,b = <&b>;
 * };
 * then parse the phandles and pass the node here, then put the node back.
 */
int adi_tru_set_trigger(struct adi_tru *tru, struct device_node *master,
	struct device_node *slave)
{
	u32 mid = 0;
	u32 sid = 0;

	if (of_property_read_u32(master, "adi,tru-master-id", &mid)) {
		dev_err(tru->dev, "dts entry %s is missing a adi,tru-master-id",
			master->full_name);
		return -ENOENT;
	}

	if (of_property_read_u32(slave, "adi,tru-slave-id", &sid)) {
		dev_err(tru->dev, "dts entry %s is missing a adi,tru-slave-id",
			slave->full_name);
		return -ENOENT;
	}

	return adi_tru_set_trigger_by_id(tru, mid, sid);
}

int adi_tru_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_tru *tru;
	struct resource *res;
	struct device_node *np = dev->of_node;
	struct device_node *child;
	void __iomem *base;
	u32 master, slave;
	int ret = 0;

	tru = devm_kzalloc(dev, sizeof(*tru), GFP_KERNEL);
	if (!tru)
		return -ENOMEM;

	tru->dev = dev;
	tru->use_smc = of_property_read_bool(np, "adi,use-smc");

	if (!tru->use_smc) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(dev, "Missing TRU base address (reg property in device tree)\n");
			return -ENODEV;
		}

		base = devm_ioremap(dev, res->start, resource_size(res));
		if (IS_ERR(base)) {
			dev_err(dev, "Cannot map TRU base address\n");
			return -PTR_ERR(base);
		}
		tru->ioaddr = base;
	}

	master = ADI_TRU_DEFAULT_MAX_MASTER_ID;
	if (of_property_read_u32(np, "adi,max-master-id", &master)) {
		dev_warn(dev, "Missing adi,max-master-id property, using default %d\n",
			ADI_TRU_DEFAULT_MAX_MASTER_ID);
	}
	tru->max_master_id = master;

	slave = ADI_TRU_DEFAULT_MAX_SLAVE_ID;
	if (of_property_read_u32(np, "adi,max-slave-id", &slave)) {
		dev_warn(dev, "Missing adi,max-slave-id property, using default %d\n",
			ADI_TRU_DEFAULT_MAX_SLAVE_ID);
	}
	tru->max_slave_id = slave;

	/* Do not try to configure the hardware if we need to use smcs to trigger
	 * because all of the TRU is restricted from access in that case
	 */
	if (!tru->use_smc) {
		/*
		 * Initialize statically defined triggers from the device tree
		 * as child nodes, for example something like this
		 * tru: tru@xyz {
		 *  a: channel@0 {
		 *   adi,tru-master-id = <100>;
		 *   adi,tru-slave-id = <101>;
		 *  };
		 *  b: channel@1 {
		 *   adi,tru-master-id = <102>;
		 *   adi,tru-slave-id = <103>;
		 *  };
		 * };
		 */
		child = NULL;
		while ((child = of_get_next_child(np, child))) {
			ret = adi_tru_set_trigger(tru, child, child);
			if (ret) {
				of_node_put(child);
				dev_err(dev, "Invalid static trigger map in TRU device tree entry\n");
				return ret;
			}
		}

		writel(0x01, tru->ioaddr + ADI_TRU_REG_GCTL);
	}

	dev_set_drvdata(dev, tru);
	return 0;
}

int adi_tru_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id adi_tru_dt_ids[] = {
	{ .compatible = "adi,trigger-routing-unit" },
	{},
};

MODULE_DEVICE_TABLE(of, adi_tru_dt_ids);

static struct platform_driver adi_tru_driver = {
	.probe = adi_tru_probe,
	.remove = adi_tru_remove,
	.driver = {
		.name = "adi-trigger-routing-unit",
		.of_match_table = of_match_ptr(adi_tru_dt_ids),
	},
};
module_platform_driver(adi_tru_driver);

MODULE_DESCRIPTION("ADI Trigger Routing Unit driver");
MODULE_LICENSE("GPL");
