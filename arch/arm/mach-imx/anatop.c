// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include "common.h"
#include "hardware.h"

#define REG_SET		0x4
#define REG_CLR		0x8

#define ANADIG_ARM_PLL		0x60
#define ANADIG_DDR_PLL		0x70
#define ANADIG_SYS_PLL		0xb0
#define ANADIG_ENET_PLL		0xe0
#define ANADIG_AUDIO_PLL	0xf0
#define ANADIG_VIDEO_PLL	0x130

#define ANADIG_REG_2P5		0x130
#define ANADIG_REG_CORE		0x140
#define ANADIG_ANA_MISC0	0x150
#define ANADIG_ANA_MISC2	0x170
#define ANADIG_DIGPROG		0x260
#define ANADIG_DIGPROG_IMX6SL	0x280
#define ANADIG_DIGPROG_IMX7D	0x800

#define BM_ANADIG_REG_2P5_ENABLE_WEAK_LINREG	0x40000
#define BM_ANADIG_REG_2P5_ENABLE_PULLDOWN	0x8
#define BM_ANADIG_REG_CORE_FET_ODRIVE		0x20000000
#define BM_ANADIG_REG_CORE_REG1			(0x1f << 9)
#define BM_ANADIG_REG_CORE_REG2			(0x1f << 18)
#define BP_ANADIG_REG_CORE_REG2			(18)
#define BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG	0x1000
#define BM_ANADIG_ANA_MISC0_V2_STOP_MODE_CONFIG	0x800
#define BM_ANADIG_ANA_MISC0_V3_STOP_MODE_CONFIG	0xc00
#define BM_ANADIG_ANA_MISC2_REG1_STEP_TIME	(0x3 << 26)
#define BP_ANADIG_ANA_MISC2_REG1_STEP_TIME	(26)
/* Below MISC0_DISCON_HIGH_SNVS is only for i.MX6SL */
#define BM_ANADIG_ANA_MISC0_DISCON_HIGH_SNVS	0x2000
/* Since i.MX6SX, DISCON_HIGH_SNVS is changed to bit 12 */
#define BM_ANADIG_ANA_MISC0_V2_DISCON_HIGH_SNVS	0x1000

#define LDO_RAMP_UP_UNIT_IN_CYCLES      64 /* 64 cycles per step */
#define LDO_RAMP_UP_FREQ_IN_MHZ         24 /* cycle based on 24M OSC */

static struct regmap *anatop;

static void imx_anatop_enable_weak2p5(bool enable)
{
	u32 reg, val, mask;

	regmap_read(anatop, ANADIG_ANA_MISC0, &val);

	if (cpu_is_imx6sx() || cpu_is_imx6ul() || cpu_is_imx6ull() ||
	    cpu_is_imx6ulz() || cpu_is_imx6sll())
		mask = BM_ANADIG_ANA_MISC0_V3_STOP_MODE_CONFIG;
	else if (cpu_is_imx6sl())
		mask = BM_ANADIG_ANA_MISC0_V2_STOP_MODE_CONFIG;
	else
		mask = BM_ANADIG_ANA_MISC0_STOP_MODE_CONFIG;

	/* can only be enabled when stop_mode_config is clear. */
	reg = ANADIG_REG_2P5;
	reg += (enable && (val & mask) == 0) ? REG_SET : REG_CLR;
	regmap_write(anatop, reg, BM_ANADIG_REG_2P5_ENABLE_WEAK_LINREG);
}

static void imx_anatop_enable_fet_odrive(bool enable)
{
	regmap_write(anatop, ANADIG_REG_CORE + (enable ? REG_SET : REG_CLR),
		BM_ANADIG_REG_CORE_FET_ODRIVE);
}

static inline void imx_anatop_enable_2p5_pulldown(bool enable)
{
	regmap_write(anatop, ANADIG_REG_2P5 + (enable ? REG_SET : REG_CLR),
		BM_ANADIG_REG_2P5_ENABLE_PULLDOWN);
}

static inline void imx_anatop_disconnect_high_snvs(bool enable)
{
	if (cpu_is_imx6sx() || cpu_is_imx6ul() || cpu_is_imx6ull() ||
	    cpu_is_imx6ulz() || cpu_is_imx6sll())
		regmap_write(anatop, ANADIG_ANA_MISC0 +
			(enable ? REG_SET : REG_CLR),
			BM_ANADIG_ANA_MISC0_V2_DISCON_HIGH_SNVS);
	else
		regmap_write(anatop, ANADIG_ANA_MISC0 +
			(enable ? REG_SET : REG_CLR),
			BM_ANADIG_ANA_MISC0_DISCON_HIGH_SNVS);
}

static void imx_anatop_disable_pu(bool off)
{
	u32  val, soc, delay;
	if (off) {
		regmap_read(anatop, ANADIG_REG_CORE, &val);
		val &= ~BM_ANADIG_REG_CORE_REG1;
		regmap_write(anatop, ANADIG_REG_CORE, val);
	} else {
		/* track vddpu with vddsoc */
		regmap_read(anatop, ANADIG_REG_CORE, &val);
		soc = val & BM_ANADIG_REG_CORE_REG2;
		val &= ~BM_ANADIG_REG_CORE_REG1;
		val |= soc >> 9;
		regmap_write(anatop, ANADIG_REG_CORE, val);
		/* wait PU LDO ramp */
		regmap_read(anatop, ANADIG_ANA_MISC2, &val);
		val &= BM_ANADIG_ANA_MISC2_REG1_STEP_TIME;
		val >>= BP_ANADIG_ANA_MISC2_REG1_STEP_TIME;
		delay = (soc >> BP_ANADIG_REG_CORE_REG2) *
			(LDO_RAMP_UP_UNIT_IN_CYCLES << val) /
			LDO_RAMP_UP_FREQ_IN_MHZ + 1;
		udelay(delay);
	}
}

void imx_anatop_pre_suspend(void)
{
	if (cpu_is_imx7d()) {
		/* PLL and PFDs overwrite set */
		regmap_write(anatop, ANADIG_ARM_PLL + REG_SET, 1 << 20);
		regmap_write(anatop, ANADIG_DDR_PLL + REG_SET, 1 << 19);
		regmap_write(anatop, ANADIG_SYS_PLL + REG_SET, 0x1ff << 17);
		regmap_write(anatop, ANADIG_ENET_PLL + REG_SET, 1 << 13);
		regmap_write(anatop, ANADIG_AUDIO_PLL + REG_SET, 1 << 24);
		regmap_write(anatop, ANADIG_VIDEO_PLL + REG_SET, 1 << 24);
		return;
	}

	if (cpu_is_imx6q() && imx_get_soc_revision() >= IMX_CHIP_REVISION_2_0)
		imx_anatop_disable_pu(true);
	imx_anatop_enable_weak2p5(true);

	imx_anatop_enable_fet_odrive(true);

	if (cpu_is_imx6sl() || cpu_is_imx6sx() || cpu_is_imx6ul() ||
	    cpu_is_imx6ull() || cpu_is_imx6ulz() || cpu_is_imx6sll())
		imx_anatop_disconnect_high_snvs(true);
}

void imx_anatop_post_resume(void)
{
	if (cpu_is_imx7d()) {
		/* PLL and PFDs overwrite clear */
		regmap_write(anatop, ANADIG_ARM_PLL + REG_CLR, 1 << 20);
		regmap_write(anatop, ANADIG_DDR_PLL + REG_CLR, 1 << 19);
		regmap_write(anatop, ANADIG_SYS_PLL + REG_CLR, 0x1ff << 17);
		regmap_write(anatop, ANADIG_ENET_PLL + REG_CLR, 1 << 13);
		regmap_write(anatop, ANADIG_AUDIO_PLL + REG_CLR, 1 << 24);
		regmap_write(anatop, ANADIG_VIDEO_PLL + REG_CLR, 1 << 24);
		return;
	}

	if (cpu_is_imx6q() && imx_get_soc_revision() >= IMX_CHIP_REVISION_2_0)
		imx_anatop_disable_pu(false);

	imx_anatop_enable_weak2p5(false);

	imx_anatop_enable_fet_odrive(false);

	if (cpu_is_imx6sl() || cpu_is_imx6sx() || cpu_is_imx6ul() ||
	    cpu_is_imx6ull() || cpu_is_imx6ulz() || cpu_is_imx6sll())
		imx_anatop_disconnect_high_snvs(false);
}

void __init imx_init_revision_from_anatop(void)
{
	struct device_node *np, *src_np;
	void __iomem *anatop_base;
	void __iomem *src_base;
	unsigned int revision;
	u32 digprog, sbmr2 = 0;
	u16 offset = ANADIG_DIGPROG;
	u16 major_part, minor_part;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-anatop");
	anatop_base = of_iomap(np, 0);
	WARN_ON(!anatop_base);
	if (of_device_is_compatible(np, "fsl,imx6sl-anatop"))
		offset = ANADIG_DIGPROG_IMX6SL;
	if (of_device_is_compatible(np, "fsl,imx7d-anatop"))
		offset = ANADIG_DIGPROG_IMX7D;
	digprog = readl_relaxed(anatop_base + offset);
	iounmap(anatop_base);

	if ((digprog >> 16) == MXC_CPU_IMX6ULL) {
		src_np = of_find_compatible_node(NULL, NULL, "fsl,imx6ul-src");
		if (src_np) {
			src_base = of_iomap(src_np, 0);
			WARN_ON(!src_base);
			of_node_put(src_np);
			sbmr2 = readl_relaxed(src_base + 0x1c);
			iounmap(src_base);
		}
		if (sbmr2 & (1 << 6)) {
			digprog &= ~(0xff << 16);
			digprog |= (MXC_CPU_IMX6ULZ << 16);
		}
	}

	/*
	 * On i.MX7D digprog value match linux version format, so
	 * it needn't map again and we can use register value directly.
	 */
	if (of_device_is_compatible(np, "fsl,imx7d-anatop")) {
		revision = digprog & 0xff;
	} else {
		/*
		 * MAJOR: [15:8], the major silicon revison;
		 * MINOR: [7: 0], the minor silicon revison;
		 *
		 * please refer to the i.MX RM for the detailed
		 * silicon revison bit define.
		 * format the major part and minor part to match the
		 * linux kernel soc version format.
		 */
		major_part = (digprog >> 8) & 0xf;
		minor_part = digprog & 0xf;
		revision = ((major_part + 1) << 4) | minor_part;
	}
	of_node_put(np);

	mxc_set_cpu_type(digprog >> 16 & 0xff);
	imx_set_soc_revision(revision);
}

void __init imx_anatop_init(void)
{
	anatop = syscon_regmap_lookup_by_compatible("fsl,imx6q-anatop");
	if (IS_ERR(anatop))
		pr_err("%s: failed to find imx6q-anatop regmap!\n", __func__);
}
