/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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

#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <soc/imx8/sc/sci.h>
#include <soc/imx8/soc.h>
#include <soc/imx/revision.h>
#include <soc/imx/src.h>
#include <soc/imx/fsl_sip.h>

struct imx8_soc_data {
	char *name;
	u32 (*soc_revision)(void);
};

static u32 imx8_soc_id;
static u32 imx8_soc_rev = IMX_CHIP_REVISION_UNKNOWN;
static u64 imx8_soc_uid;

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

static u32 imx_init_revision_from_atf(void)
{
	struct arm_smccc_res res;
	u32 digprog;
	u32 id, rev;

	arm_smccc_smc(FSL_SIP_GET_SOC_INFO, 0, 0,
			0, 0, 0, 0, 0, &res);
	digprog = res.a0;

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
	u32 uid_l = 0, uid_h = 0;

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

	sc_err = sc_misc_get_control(ipc_handle, SC_R_SYSTEM, SC_C_ID, &id);
	if (sc_err != SC_ERR_NONE) {
		WARN(1, "%s: Cannot get control\n", __func__);

		return IMX_CHIP_REVISION_UNKNOWN;
	};

	rev = (id >> 5) & 0xf;
	rev = (((rev >> 2) + 1) << 4) | (rev & 0x3);
	id &= 0x1f;

	imx8_set_soc_id(id);
	imx8_set_soc_revision(rev);

	sc_misc_unique_id(ipc_handle, &uid_l, &uid_h);

	imx8_soc_uid = uid_h;
	imx8_soc_uid <<= 32;
	imx8_soc_uid |= uid_l;

	return rev;
}

bool TKT340553_SW_WORKAROUND;

static u32 imx8qm_soc_revision(void)
{
	u32 rev = imx_init_revision_from_scu();

	if (rev == IMX_CHIP_REVISION_1_0 || rev == IMX_CHIP_REVISION_1_1)
		TKT340553_SW_WORKAROUND = true;

	return rev;
}

static u32 imx8qxp_soc_revision(void)
{
	return imx_init_revision_from_scu();
}

#define OCOTP_UID_LOW	0x410
#define OCOTP_UID_HIGH	0x420

static u64 imx8mq_soc_get_soc_uid(void)
{
	struct device_node *np;
	void __iomem *base;

	u64 val = 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mq-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return val;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	val = readl_relaxed(base + OCOTP_UID_HIGH);
	val <<= 32;
	val |=  readl_relaxed(base + OCOTP_UID_LOW);

	iounmap(base);

put_node:
	of_node_put(np);
	return val;
}

static u32 imx8mq_soc_revision(void)
{
	imx8_soc_uid = imx8mq_soc_get_soc_uid();
	return imx_init_revision_from_atf();
}

static u32 imx8mm_soc_revision(void)
{
	imx8_soc_uid = imx8mq_soc_get_soc_uid();
	return imx_init_revision_from_atf();
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

static struct imx8_soc_data imx8mm_soc_data = {
	.name = "i.MX8MM",
	.soc_revision = imx8mm_soc_revision,
};

static const struct of_device_id imx8_soc_match[] = {
	{ .compatible = "fsl,imx8qm", .data = &imx8qm_soc_data, },
	{ .compatible = "fsl,imx8qxp", .data = &imx8qxp_soc_data, },
	{ .compatible = "fsl,imx8mq", .data = &imx8mq_soc_data, },
	{ .compatible = "fsl,imx8mm", .data = &imx8mm_soc_data, },
	/* Fixme: this is a hack for big/little xen guest, b0 no need this */
	{ .compatible = "xen,xenvm", .data = &imx8qm_soc_data, },
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

static ssize_t imx8_get_soc_uid(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%016llX\n", imx8_soc_uid);
}

static struct device_attribute imx8_uid =
	__ATTR(soc_uid, S_IRUGO, imx8_get_soc_uid, NULL);

static void __init imx8mq_noc_init(void)
{
	struct arm_smccc_res res;

	pr_info("Config NOC for VPU and CPU\n");

	arm_smccc_smc(FSL_SIP_NOC, FSL_SIP_NOC_PRIORITY, NOC_CPU_PRIORITY,
			0x80000300, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_err("Config NOC for CPU fail!\n");

	arm_smccc_smc(FSL_SIP_NOC, FSL_SIP_NOC_PRIORITY, NOC_VPU_PRIORITY,
			0x80000300, 0, 0, 0, 0, &res);
	if (res.a0)
		pr_err("Config NOC for VPU fail!\n");
}

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

	device_create_file(soc_device_to_device(soc_dev), &imx8_uid);

	if (of_machine_is_compatible("fsl,imx8mq"))
		imx8mq_noc_init();

	return 0;

free_rev:
	kfree(soc_dev_attr->revision);
free_soc:
	kfree(soc_dev_attr);
	return -ENODEV;
}
device_initcall(imx8_soc_init);

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_MKT_SEGMENT_SHIFT	6
#define OCOTP_CFG3_CONSUMER		0
#define OCOTP_CFG3_EXT_CONSUMER		1
#define OCOTP_CFG3_INDUSTRIAL		2
#define OCOTP_CFG3_AUTO			3

static void __init imx8mq_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx8mq-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_MKT_SEGMENT_SHIFT;
	val &= 0x3;

	switch (val) {
	case OCOTP_CFG3_CONSUMER:
		if (dev_pm_opp_disable(cpu_dev, 800000000))
			pr_warn("failed to disable 800MHz OPP!\n");
		if (dev_pm_opp_disable(cpu_dev, 1300000000))
			pr_warn("failed to disable 1.3GHz OPP!\n");
		break;
	case OCOTP_CFG3_INDUSTRIAL:
		if (dev_pm_opp_disable(cpu_dev, 1000000000))
			pr_warn("failed to disable 1GHz OPP!\n");
		if (dev_pm_opp_disable(cpu_dev, 1500000000))
			pr_warn("failed to disable 1.5GHz OPP!\n");
		break;
	default:
		/* consumer part for default */
		if (dev_pm_opp_disable(cpu_dev, 800000000))
			pr_warn("failed to disable 800MHz OPP!\n");
		if (dev_pm_opp_disable(cpu_dev, 1300000000))
			pr_warn("failed to disable 1.3GHz OPP!\n");
		break;
	}

	iounmap(base);

put_node:
	of_node_put(np);
}

static void __init imx8mq_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (dev_pm_opp_of_add_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	if (of_machine_is_compatible("fsl,imx8mq"))
		imx8mq_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static int __init imx8_register_cpufreq(void)
{
	if (of_machine_is_compatible("fsl,imx8mq") ||
		of_machine_is_compatible("fsl,imx8mm")) {
		imx8mq_opp_init();
		platform_device_register_simple("imx8mq-cpufreq", -1, NULL, 0);
	} else {
		platform_device_register_simple("imx8-cpufreq", -1, NULL, 0);
	}

	return 0;
}
late_initcall(imx8_register_cpufreq);

/* To indicate M4 enabled or not on i.MX8MQ */
static bool m4_is_enabled;
bool imx_src_is_m4_enabled(void)
{
	return m4_is_enabled;
}

int check_m4_enabled(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(FSL_SIP_SRC, FSL_SIP_SRC_M4_STARTED, 0,
		      0, 0, 0, 0, 0, &res);
	m4_is_enabled = !!res.a0;

	if (m4_is_enabled)
		printk("M4 is started\n");

	return 0;
}
