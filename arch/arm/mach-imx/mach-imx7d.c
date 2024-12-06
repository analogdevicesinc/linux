// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 */
#include <linux/irqchip.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/regmap.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"

static struct property device_disabled = {
	.name = "status",
	.length = sizeof("disabled"),
	.value = "disabled",
};

static int bcm54220_phy_fixup(struct phy_device *dev)
{
	/* enable RXC skew select RGMII copper mode */
	phy_write(dev, 0x1e, 0x21);
	phy_write(dev, 0x1f, 0x7ea8);
	phy_write(dev, 0x1e, 0x2f);
	phy_write(dev, 0x1f, 0x71b7);

	return 0;
}

#define PHY_ID_BCM54220	0x600d8589

static void __init imx7d_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_BCM54220, 0xffffffff,
					   bcm54220_phy_fixup);
	}
}

static void __init imx7d_enet_mdio_fixup(void)
{
	struct regmap *gpr;

	/* The management data input/output (MDIO) bus where often high-speed,
	 * open-drain operation is required. i.MX7D TO1.0 ENET MDIO pin has no
	 * open drain as IC ticket number: TKT252980, i.MX7D TO1.1 fix the issue.
	 * GPR1[8:7] are reserved bits at TO1.0, there no need to add version check.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx7d-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR0, IMX7D_GPR0_ENET_MDIO_OPEN_DRAIN_MASK,
				   IMX7D_GPR0_ENET_MDIO_OPEN_DRAIN_MASK);
	else
		pr_err("failed to find fsl,imx7d-iomux-gpr regmap\n");
}

static void __init imx7d_enet_clk_sel(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx7d-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET_TX_CLK_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET_CLK_DIR_MASK, 0);
	} else {
		pr_err("failed to find fsl,imx7d-iomux-gpr regmap\n");
	}
}

static void __init imx7d_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx7d-fec", "fsl,imx7d-ocotp");
	imx7d_enet_mdio_fixup();
	imx7d_enet_phy_init();
	imx7d_enet_clk_sel();
}

static inline void imx7d_disable_arm_arch_timer(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "arm,armv7-timer");
	if (node) {
		pr_info("disable arm arch timer for nosmp!\n");
		of_add_property(node, &device_disabled);
	}
}

static void __init imx7d_init_machine(void)
{
	imx_anatop_init();
	imx7d_pm_init();
	imx7d_enet_init();
}

static void __init imx7d_init_late(void)
{
	imx7d_cpuidle_init();
	if (IS_ENABLED(CONFIG_ARM_IMX_CPUFREQ_DT))
		platform_device_register_simple("imx-cpufreq-dt", -1, NULL, 0);
}

static void __init imx7d_init_irq(void)
{
	imx_gpcv2_check_dt();
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
#ifndef CONFIG_SMP
	imx7d_disable_arm_arch_timer();
#endif
}

static void __init imx7d_map_io(void)
{
	debug_ll_io_init();
	imx7_pm_map_io();
	imx_busfreq_map_io();
}

static const char *const imx7d_dt_compat[] __initconst = {
	"fsl,imx7d",
	"fsl,imx7s",
	NULL,
};

DT_MACHINE_START(IMX7D, "Freescale i.MX7 Dual (Device Tree)")
	.map_io         = imx7d_map_io,
	.smp            = smp_ops(imx_smp_ops),
	.init_irq	= imx7d_init_irq,
	.init_machine	= imx7d_init_machine,
	.init_late      = imx7d_init_late,
	.dt_compat	= imx7d_dt_compat,
MACHINE_END
