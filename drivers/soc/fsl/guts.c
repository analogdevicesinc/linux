// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Freescale QorIQ Platforms GUTS Driver
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 */

#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_fdt.h>
#include <linux/sys_soc.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/fsl/guts.h>

#define DCFG_CCSR	0
#define DCFG_DCSR	1

#define MAX_NUM_LANES	8
#define MAX_NUM_SERDES	2

struct fsl_soc_die_attr {
	char	*die;
	u32	svr;
	u32	mask;
};

struct fsl_soc_serdes_rcw_override {
	int offset;
	int width;
	int shift;
	int val;
};

struct fsl_soc_data {
	const char *sfp_compat;
	u32 uid_offset;
	int (*serdes_get_rcw_override)(int index, int lane,
				       enum lynx_lane_mode,
				       struct fsl_soc_serdes_rcw_override *override);
	void (*serdes_init_rcwcr)(int index);
};

enum qoriq_die {
	DIE_T4240,
	DIE_T1040,
	DIE_T2080,
	DIE_T1024,
	DIE_LS1043A,
	DIE_LS2080A,
	DIE_LS1088A,
	DIE_LS1012A,
	DIE_LS1046A,
	DIE_LS2088A,
	DIE_LS1021A,
	DIE_LX2160A,
	DIE_LS1028A,
	DIE_MAX,
};

/* SoC die attribute definition for QorIQ platform */
static const struct fsl_soc_die_attr fsl_soc_die[] = {
	/*
	 * Power Architecture-based SoCs T Series
	 */

	/* Die: T4240, SoC: T4240/T4160/T4080 */
	[DIE_T4240] =
	{ .die		= "T4240",
	  .svr		= 0x82400000,
	  .mask		= 0xfff00000,
	},
	/* Die: T1040, SoC: T1040/T1020/T1042/T1022 */
	[DIE_T1040] =
	{ .die		= "T1040",
	  .svr		= 0x85200000,
	  .mask		= 0xfff00000,
	},
	/* Die: T2080, SoC: T2080/T2081 */
	[DIE_T2080] =
	{ .die		= "T2080",
	  .svr		= 0x85300000,
	  .mask		= 0xfff00000,
	},
	/* Die: T1024, SoC: T1024/T1014/T1023/T1013 */
	[DIE_T1024] =
	{ .die		= "T1024",
	  .svr		= 0x85400000,
	  .mask		= 0xfff00000,
	},

	/*
	 * ARM-based SoCs LS Series
	 */

	/* Die: LS1043A, SoC: LS1043A/LS1023A */
	[DIE_LS1043A] =
	{ .die		= "LS1043A",
	  .svr		= 0x87920000,
	  .mask		= 0xffff0000,
	},
	/* Die: LS2080A, SoC: LS2080A/LS2040A/LS2085A */
	[DIE_LS2080A] =
	{ .die		= "LS2080A",
	  .svr		= 0x87010000,
	  .mask		= 0xff3f0000,
	},
	/* Die: LS1088A, SoC: LS1088A/LS1048A/LS1084A/LS1044A */
	[DIE_LS1088A] =
	{ .die		= "LS1088A",
	  .svr		= 0x87030000,
	  .mask		= 0xff3f0000,
	},
	/* Die: LS1012A, SoC: LS1012A */
	[DIE_LS1012A] =
	{ .die		= "LS1012A",
	  .svr		= 0x87040000,
	  .mask		= 0xffff0000,
	},
	/* Die: LS1046A, SoC: LS1046A/LS1026A */
	[DIE_LS1046A] =
	{ .die		= "LS1046A",
	  .svr		= 0x87070000,
	  .mask		= 0xffff0000,
	},
	/* Die: LS2088A, SoC: LS2088A/LS2048A/LS2084A/LS2044A */
	[DIE_LS2088A] =
	{ .die		= "LS2088A",
	  .svr		= 0x87090000,
	  .mask		= 0xff3f0000,
	},
	/* Die: LS1021A, SoC: LS1021A/LS1020A/LS1022A */
	[DIE_LS1021A] =
	{ .die		= "LS1021A",
	  .svr		= 0x87000000,
	  .mask		= 0xfff70000,
	},
	/* Die: LX2160A, SoC: LX2160A/LX2120A/LX2080A */
	[DIE_LX2160A] =
	{ .die          = "LX2160A",
	  .svr          = 0x87360000,
	  .mask         = 0xff3f0000,
	},
	/* Die: LS1028A, SoC: LS1028A */
	[DIE_LS1028A] =
	{ .die          = "LS1028A",
	  .svr          = 0x870b0000,
	  .mask         = 0xff3f0000,
	},
	{ },
};

static struct fsl_soc_guts {
	struct ccsr_guts __iomem *dcfg_ccsr;
	struct ccsr_guts __iomem *dcfg_dcsr;
	const struct fsl_soc_data *data;
	bool little_endian;
	u32 svr;
	enum lynx_lane_mode lane_mode[MAX_NUM_SERDES][MAX_NUM_LANES];
	bool rcwcr_init_done;
	struct mutex rcwcr_mutex; /* protects access to the RCWCR */
} soc;

static unsigned int fsl_guts_read(const void __iomem *reg)
{
	if (soc.little_endian)
		return ioread32(reg);
	return ioread32be(reg);
}

static void fsl_guts_write(void __iomem *reg, u32 val)
{
	if (soc.little_endian)
		iowrite32(val, reg);
	else
		iowrite32be(val, reg);
}

static int fsl_soc_die_match_one(u32 svr, const struct fsl_soc_die_attr *match)
{
	if (match->svr == (svr & match->mask))
		return 1;
	return 0;
}

static const struct fsl_soc_die_attr *fsl_soc_die_match(
	u32 svr, const struct fsl_soc_die_attr *matches)
{
	while (matches->svr) {
		if (fsl_soc_die_match_one(svr, matches))
			return matches;
		matches++;
	}
	return NULL;
}

void fsl_guts_lane_init(int serdes_idx, int lane, enum lynx_lane_mode lane_mode)
{
	soc.lane_mode[serdes_idx - 1][lane] = lane_mode;
}
EXPORT_SYMBOL(fsl_guts_lane_init);

int fsl_guts_lane_validate(int serdes_idx, int lane, enum lynx_lane_mode lane_mode)
{
	struct fsl_soc_serdes_rcw_override override;

	if (!fsl_soc_die_match_one(soc.svr, &fsl_soc_die[DIE_LS1088A]) &&
	    !fsl_soc_die_match_one(soc.svr, &fsl_soc_die[DIE_LS2088A]) &&
	    !fsl_soc_die_match_one(soc.svr, &fsl_soc_die[DIE_LS1046A]))
		return -EOPNOTSUPP;
	if (!soc.data)
		return -EOPNOTSUPP;
	if (!soc.data->serdes_get_rcw_override)
		return -EOPNOTSUPP;

	return soc.data->serdes_get_rcw_override(serdes_idx, lane, lane_mode, &override);
}
EXPORT_SYMBOL(fsl_guts_lane_validate);

int fsl_guts_lane_set_mode(int serdes_idx, int lane, enum lynx_lane_mode lane_mode)
{
	struct fsl_soc_serdes_rcw_override override;
	u32 old, tmp, mask;
	int err;

	if (!soc.data->serdes_get_rcw_override)
		return -EOPNOTSUPP;

	mutex_lock(&soc.rcwcr_mutex);

	if (soc.data->serdes_init_rcwcr)
		soc.data->serdes_init_rcwcr(serdes_idx);

	err = soc.data->serdes_get_rcw_override(serdes_idx, lane, lane_mode, &override);
	if (err)
		goto out;

	old = fsl_guts_read(&soc.dcfg_ccsr->rcwsr[override.offset]);
	if (override.width == 1)
		mask = BIT(override.shift);
	else
		mask = GENMASK(override.width - 1, 0) << override.shift;
	tmp = (old & ~mask) | (override.val << override.shift);
	fsl_guts_write(&soc.dcfg_dcsr->rcwsr[override.offset], tmp);

	soc.lane_mode[serdes_idx - 1][lane] = lane_mode;

	mutex_unlock(&soc.rcwcr_mutex);

	return 0;

out:
	mutex_unlock(&soc.rcwcr_mutex);

	return err;
}
EXPORT_SYMBOL(fsl_guts_lane_set_mode);

static u64 fsl_guts_get_soc_uid(const char *compat, unsigned int offset)
{
	struct device_node *np;
	void __iomem *sfp_base;
	u64 uid;

	np = of_find_compatible_node(NULL, NULL, compat);
	if (!np)
		return 0;

	sfp_base = of_iomap(np, 0);
	if (!sfp_base) {
		of_node_put(np);
		return 0;
	}

	uid = ioread32(sfp_base + offset);
	uid <<= 32;
	uid |= ioread32(sfp_base + offset + 4);

	iounmap(sfp_base);
	of_node_put(np);

	return uid;
}

static int ls1088a_serdes_get_rcw_override(int index, int lane,
					   enum lynx_lane_mode lane_mode,
					   struct fsl_soc_serdes_rcw_override *override)
{
	/* The RCW override procedure has to write to different registers
	 * depending on the SerDes block index.
	 */
	switch (index) {
	case 1:
		override->offset = 28;
		override->shift = 16 + 4 * (3 - lane);
		break;
	case 2:
		override->offset = 29;
		override->shift = 4 * (3 - lane);
		break;
	default:
		return -EINVAL;
	}
	override->width = 4;

	switch (lane_mode) {
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		override->val = 1;
		break;
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		override->val = 3;
		break;
	default:
		return -EOPNOTSUPP;
	};

	return 0;
}

static int ls1046a_serdes_get_rcw_override(int index, int lane,
					   enum lynx_lane_mode lane_mode,
					   struct fsl_soc_serdes_rcw_override *override)
{
	/* The RCW override procedure has to write to different registers
	 * depending on the SerDes block index.
	 */
	switch (index) {
	case 1:
		override->offset = 4;
		override->shift = 16 + 4 * lane;
		break;
	case 2:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
	override->width = 4;

	switch (lane_mode) {
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		override->val = 1;
		break;
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		override->val = 3;
		break;
	default:
		return -EOPNOTSUPP;
	};

	return 0;
}

static int ls2088a_serdes_get_rcw_override(int index, int lane,
					   enum lynx_lane_mode lane_mode,
					   struct fsl_soc_serdes_rcw_override *override)
{
	switch (index) {
	case 1:
		override->offset = 29;
		override->shift = 6 + (7 - lane);
		break;
	case 2:
		return -EOPNOTSUPP;
	default:
		return -EINVAL;
	}
	override->width = 1;

	switch (lane_mode) {
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		override->val = 1;
		break;
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		override->val = 0;
		break;
	default:
		return -EOPNOTSUPP;
	};

	return 0;
}

static void ls2088a_serdes_init_rcwcr(int serdes_idx)
{
	u32 reg;
	int i;

	if (serdes_idx != 1)
		return;
	if (soc.rcwcr_init_done)
		return;

	/* Enable SRDS_CLK_EN_SEL_XGMII_S1 */
	reg = fsl_guts_read(&soc.dcfg_ccsr->rcwsr[29]);
	reg = (reg & ~GENMASK(14, 6)) | BIT(14);

	/* We need to configure the initial state of all lanes for the SerDes block #1 */
	for (i = 0; i < MAX_NUM_LANES; i++) {
		if (soc.lane_mode[serdes_idx - 1][i] == LANE_MODE_10GBASER ||
		    soc.lane_mode[serdes_idx - 1][i] == LANE_MODE_10GBASEKR)
			reg |= BIT(6 + (7 - i));
	}
	fsl_guts_write(&soc.dcfg_dcsr->rcwsr[29], reg);
	soc.rcwcr_init_done = true;
}

static const struct fsl_soc_data ls1088a_data = {
	.serdes_get_rcw_override = ls1088a_serdes_get_rcw_override,
};

static const struct fsl_soc_data ls1046a_data = {
	.serdes_get_rcw_override = ls1046a_serdes_get_rcw_override,
};

static const struct fsl_soc_data ls2088a_data = {
	.serdes_get_rcw_override = ls2088a_serdes_get_rcw_override,
	.serdes_init_rcwcr = ls2088a_serdes_init_rcwcr,
};

static const struct fsl_soc_data ls1028a_data = {
	.sfp_compat = "fsl,ls1028a-sfp",
	.uid_offset = 0x21c,
};

/*
 * Table for matching compatible strings, for device tree
 * guts node, for Freescale QorIQ SOCs.
 */
static const struct of_device_id fsl_guts_of_match[] = {
	{ .compatible = "fsl,qoriq-device-config-1.0", },
	{ .compatible = "fsl,qoriq-device-config-2.0", },
	{ .compatible = "fsl,p1010-guts", },
	{ .compatible = "fsl,p1020-guts", },
	{ .compatible = "fsl,p1021-guts", },
	{ .compatible = "fsl,p1022-guts", },
	{ .compatible = "fsl,p1023-guts", },
	{ .compatible = "fsl,p2020-guts", },
	{ .compatible = "fsl,bsc9131-guts", },
	{ .compatible = "fsl,bsc9132-guts", },
	{ .compatible = "fsl,mpc8536-guts", },
	{ .compatible = "fsl,mpc8544-guts", },
	{ .compatible = "fsl,mpc8548-guts", },
	{ .compatible = "fsl,mpc8568-guts", },
	{ .compatible = "fsl,mpc8569-guts", },
	{ .compatible = "fsl,mpc8572-guts", },
	{ .compatible = "fsl,ls1021a-dcfg", },
	{ .compatible = "fsl,ls1043a-dcfg", },
	{ .compatible = "fsl,ls2080a-dcfg", .data = &ls2088a_data},
	{ .compatible = "fsl,ls1088a-dcfg", .data = &ls1088a_data},
	{ .compatible = "fsl,ls1012a-dcfg", },
	{ .compatible = "fsl,ls1046a-dcfg", .data = &ls1046a_data},
	{ .compatible = "fsl,lx2160a-dcfg", },
	{ .compatible = "fsl,ls1028a-dcfg", .data = &ls1028a_data},
	{}
};

static int __init fsl_guts_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	static struct soc_device *soc_dev;
	const struct fsl_soc_die_attr *soc_die;
	const struct of_device_id *match;
	const char *machine = NULL;
	struct device_node *np;
	u64 soc_uid = 0;
	int ret;

	np = of_find_matching_node_and_match(NULL, fsl_guts_of_match, &match);
	if (!np)
		return 0;
	soc.data = match->data;

	soc.dcfg_ccsr = of_iomap(np, DCFG_CCSR);
	if (!soc.dcfg_ccsr) {
		of_node_put(np);
		return -ENOMEM;
	}
	soc.dcfg_dcsr = of_iomap(np, DCFG_DCSR);

	soc.little_endian = of_property_read_bool(np, "little-endian");
	soc.svr = fsl_guts_read(&soc.dcfg_ccsr->svr);
	of_node_put(np);

	/* Register soc device */
	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	if (of_property_read_string(of_root, "model", &machine))
		of_property_read_string_index(of_root, "compatible", 0, &machine);
	if (machine) {
		soc_dev_attr->machine = kstrdup(machine, GFP_KERNEL);
		if (!soc_dev_attr->machine)
			goto err_nomem;
	}

	soc_die = fsl_soc_die_match(soc.svr, fsl_soc_die);
	if (soc_die) {
		soc_dev_attr->family = kasprintf(GFP_KERNEL, "QorIQ %s",
						 soc_die->die);
	} else {
		soc_dev_attr->family = kasprintf(GFP_KERNEL, "QorIQ");
	}
	if (!soc_dev_attr->family)
		goto err_nomem;

	soc_dev_attr->soc_id = kasprintf(GFP_KERNEL, "svr:0x%08x", soc.svr);
	if (!soc_dev_attr->soc_id)
		goto err_nomem;

	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%d.%d",
					   (soc.svr >>  4) & 0xf, soc.svr & 0xf);
	if (!soc_dev_attr->revision)
		goto err_nomem;

	if (soc.data)
		soc_uid = fsl_guts_get_soc_uid(soc.data->sfp_compat,
					       soc.data->uid_offset);
	if (soc_uid)
		soc_dev_attr->serial_number = kasprintf(GFP_KERNEL, "%016llX",
							soc_uid);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		ret = PTR_ERR(soc_dev);
		goto err;
	}

	mutex_init(&soc.rcwcr_mutex);

	pr_info("Machine: %s\n", soc_dev_attr->machine);
	pr_info("SoC family: %s\n", soc_dev_attr->family);
	pr_info("SoC ID: %s, Revision: %s\n",
		soc_dev_attr->soc_id, soc_dev_attr->revision);

	return 0;

err_nomem:
	ret = -ENOMEM;
	if (soc.dcfg_dcsr)
		iounmap(soc.dcfg_dcsr);
	iounmap(soc.dcfg_ccsr);
err:
	kfree(soc_dev_attr->machine);
	kfree(soc_dev_attr->family);
	kfree(soc_dev_attr->soc_id);
	kfree(soc_dev_attr->revision);
	kfree(soc_dev_attr->serial_number);
	kfree(soc_dev_attr);

	return ret;
}
core_initcall(fsl_guts_init);
