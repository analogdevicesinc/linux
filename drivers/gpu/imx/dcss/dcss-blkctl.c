/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/device.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/of.h>

#include "dcss-prv.h"
#include <video/imx-dcss.h>

#define DCSS_BLKCTL_RESET_CTRL		0x00
#define   B_CLK_RESETN			BIT(0)
#define   APB_CLK_RESETN		BIT(1)
#define   P_CLK_RESETN			BIT(2)
#define   HDMI_RESETN			BIT(3)
#define   RTR_CLK_RESETN		BIT(4)
#define DCSS_BLKCTL_CONTROL0		0x10
#define   HDMI_MIPI_CLK_SEL		BIT(0)
#define   DISPMIX_REFCLK_SEL_POS	4
#define   DISPMIX_REFCLK_SEL_MASK	GENMASK(5, 4)
#define   DISPMIX_PIXCLK_SEL		BIT(8)
#define   HDMI_SRC_SECURE_EN		BIT(16)

static void __iomem *dcss_blkctl_reg;
static bool hdmi_output;

static void dcss_blkctl_clk_reset(u32 assert, u32 deassert)
{
	if (assert)
		dcss_clr(assert, dcss_blkctl_reg + DCSS_BLKCTL_RESET_CTRL);

	if (deassert)
		dcss_set(deassert, dcss_blkctl_reg + DCSS_BLKCTL_RESET_CTRL);
}

int dcss_blkctl_init(struct dcss_soc *dcss, unsigned long blkctl_base)
{
	struct device_node *node = dcss->dev->of_node;
	int len;
	const char *disp_dev;

	hdmi_output = false;

	dcss_blkctl_reg = devm_ioremap(dcss->dev, blkctl_base, SZ_4K);
	if (!dcss_blkctl_reg) {
		dev_err(dcss->dev, "unable to remap BLK CTRL base\n");
		return -ENOMEM;
	}

	disp_dev = of_get_property(node, "disp-dev", &len);
	if (!disp_dev || !strncmp(disp_dev, "hdmi_disp", 9))
		hdmi_output = true;

	if (hdmi_output)
		dcss_writel(0, dcss_blkctl_reg + DCSS_BLKCTL_CONTROL0);
	else
		dcss_writel(HDMI_MIPI_CLK_SEL | DISPMIX_PIXCLK_SEL,
			    dcss_blkctl_reg + DCSS_BLKCTL_CONTROL0);

	/* deassert clock domains resets */
	dcss_blkctl_clk_reset(0, B_CLK_RESETN | APB_CLK_RESETN |
				 P_CLK_RESETN | HDMI_RESETN | RTR_CLK_RESETN);

	return 0;
}

void dcss_blkctl_exit(struct dcss_soc *dcss)
{
	/* assert clock domains resets */
	dcss_blkctl_clk_reset(B_CLK_RESETN | APB_CLK_RESETN | P_CLK_RESETN |
			      HDMI_RESETN | RTR_CLK_RESETN, 0);
}

/* disabled only by cold reset/reboot */
void dcss_blkctl_hdmi_secure_src_en(struct dcss_soc *dcss)
{
	writel(HDMI_SRC_SECURE_EN, dcss_blkctl_reg + SET);
}
EXPORT_SYMBOL(dcss_blkctl_hdmi_secure_src_en);

