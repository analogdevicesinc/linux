/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <soc/imx8/sc/sci.h>
#include <soc/imx8/soc.h>
#include <soc/imx/revision.h>

#define ANADIG_DIGPROG		0x6c

struct imx8_soc_data {
	char *name;
	u32 (*soc_revision)(void);
};

static u32 imx8_soc_id;
static u32 imx8_soc_rev = IMX_CHIP_REVISION_UNKNOWN;

static const struct imx8_soc_data *soc_data;

static inline void imx8_set_soc_revision(u32 rev)
{
	imx8_soc_rev = rev;
}

unsigned int imx8_get_soc_revision(void)
{
	return imx8_soc_rev;
}

static inline void imx8_set_soc_id(u32 id)
{
	imx8_soc_id = id;
}

inline bool cpu_is_imx8qm(void)
{
	return imx8_soc_id == IMX_SOC_IMX8QM;
}

inline bool cpu_is_imx8qxp(void)
{
	return imx8_soc_id == IMX_SOC_IMX8QXP;
}

inline bool cpu_is_imx8mq(void)
{
	return imx8_soc_id == IMX_SOC_IMX8MQ;
}

static u32 imx_init_revision_from_anatop(void)
{
	struct device_node *np;
	void __iomem *anatop_base;
	u32 digprog;
	u32 id, rev;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mq-anatop");
	anatop_base = of_iomap(np, 0);
	WARN_ON(!anatop_base);
	digprog = readl_relaxed(anatop_base + ANADIG_DIGPROG);
	iounmap(anatop_base);

	/*
	 * Bit [23:16] is the silicon ID
	 * Bit[7:4] is the base layer revision,
	 * Bit[3:0] is the metal layer revision
	 * e.g. 0x10 stands for Tapeout 1.0
	 */

	rev = digprog & 0xff;
	id = digprog >> 16 & 0xff;

	imx8_set_soc_id(id);
	imx8_set_soc_revision(rev);

	return rev;
}

static u32 imx_init_revision_from_scu(void)
{
	uint32_t mu_id;
	sc_err_t sc_err = SC_ERR_NONE;
	sc_ipc_t ipc_handle;
	u32 id, rev;

	sc_err = sc_ipc_getMuID(&mu_id);
	if (sc_err != SC_ERR_NONE) {
		WARN(1, "%s: Cannot obtain MU ID\n", __func__);

		return IMX_CHIP_REVISION_UNKNOWN;
	}

	sc_err = sc_ipc_open(&ipc_handle, mu_id);
	if (sc_err != SC_ERR_NONE) {
		WARN(1, "%s: Cannot open MU channel\n", __func__);

		return IMX_CHIP_REVISION_UNKNOWN;
	};

	sc_err = sc_misc_get_control(ipc_handle, SC_R_SC_PID0, SC_C_ID, &id);
	if (sc_err != SC_ERR_NONE) {
		WARN(1, "%s: Cannot get control\n", __func__);

		return IMX_CHIP_REVISION_UNKNOWN;
	};

	rev = (id >> 5) & 0xf;
	rev = (((rev >> 2) + 1) << 4) | (rev & 0x3);
	id &= 0x1f;

	imx8_set_soc_id(id);
	imx8_set_soc_revision(rev);

	return rev;
}

bool TKT340553_SW_WORKAROUND;

static u32 imx8qm_soc_revision(void)
{
	u32 rev = imx_init_revision_from_scu();

	if (rev == IMX_CHIP_REVISION_1_0)
		TKT340553_SW_WORKAROUND = true;

	return rev;
}

static u32 imx8qxp_soc_revision(void)
{
	return imx_init_revision_from_scu();
}

static u32 imx8mq_soc_revision(void)
{
	return imx_init_revision_from_anatop();
}

static struct imx8_soc_data imx8qm_soc_data = {
	.name = "i.MX8QM",
	.soc_revision = imx8qm_soc_revision,
};

static struct imx8_soc_data imx8qxp_soc_data = {
	.name = "i.MX8QXP",
	.soc_revision = imx8qxp_soc_revision,
};

static struct imx8_soc_data imx8mq_soc_data = {
	.name = "i.MX8MQ",
	.soc_revision = imx8mq_soc_revision,
};

static const struct of_device_id imx8_soc_match[] = {
	{ .compatible = "fsl,imx8qm", .data = &imx8qm_soc_data, },
	{ .compatible = "fsl,imx8qxp", .data = &imx8qxp_soc_data, },
	{ .compatible = "fsl,imx8mq", .data = &imx8mq_soc_data, },
	{ }
};

static int __init imx8_revision_init(void)
{
	struct device_node *root;
	const struct of_device_id *id;
	const char *machine;
	u32 rev = IMX_CHIP_REVISION_UNKNOWN;
	int ret;

	root = of_find_node_by_path("/");
	ret = of_property_read_string(root, "model", &machine);
	if (ret)
		return -ENODEV;

	id = of_match_node(imx8_soc_match, root);
	if (!id)
		return -ENODEV;

	of_node_put(root);

	soc_data = id->data;
	if (soc_data && soc_data->soc_revision)
		rev = soc_data->soc_revision();

	if (rev == IMX_CHIP_REVISION_UNKNOWN)
		pr_info("CPU identified as %s, unknown revision\n",
			soc_data->name);
	else
		pr_info("CPU identified as %s, silicon rev %d.%d\n",
			soc_data->name, (rev >> 4) & 0xf, rev & 0xf);

	return 0;
}
early_initcall(imx8_revision_init);

static int __init imx8_soc_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	u32 soc_rev;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Freescale i.MX";

	if (soc_data)
		soc_dev_attr->soc_id = soc_data->name;

	soc_rev = imx8_get_soc_revision();
	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%d.%d",
					   (soc_rev >> 4) & 0xf,
					    soc_rev & 0xf);
	if (!soc_dev_attr->revision)
		goto free_soc;

	of_property_read_string(of_root, "model", &soc_dev_attr->machine);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev))
		goto free_rev;

	return 0;

free_rev:
	kfree(soc_dev_attr->revision);
free_soc:
	kfree(soc_dev_attr);
	return -ENODEV;
}
device_initcall(imx8_soc_init);

static int __init imx8_register_cpufreq(void)
{
	if (of_machine_is_compatible("fsl,imx8mq"))
		platform_device_register_simple("imx8mq-cpufreq", -1, NULL, 0);
	else
		platform_device_register_simple("imx8-cpufreq", -1, NULL, 0);

	return 0;
}
late_initcall(imx8_register_cpufreq);
