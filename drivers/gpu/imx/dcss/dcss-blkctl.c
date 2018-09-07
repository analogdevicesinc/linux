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
#include <linux/delay.h>
#include <soc/imx8/soc.h>

#include "dcss-prv.h"
#include <video/imx-dcss.h>

#define DCSS_BLKCTL_RESET_CTRL		0x00
#define   B_CLK_RESETN			BIT(0)
#define   APB_CLK_RESETN		BIT(1)
#define   P_CLK_RESETN			BIT(2)
#define   RTR_CLK_RESETN		BIT(3)
#define   HDMI_RESETN			BIT(4)
#define DCSS_BLKCTL_CONTROL0		0x10
#define   HDMI_MIPI_CLK_SEL		BIT(0)
#define   DISPMIX_REFCLK_SEL_POS	4
#define   DISPMIX_REFCLK_SEL_MASK	GENMASK(5, 4)
#define   DISPMIX_PIXCLK_SEL		BIT(8)
#define   HDMI_SRC_SECURE_EN		BIT(16)

#define B0_SILICON_ID			0x20

static struct dcss_debug_reg blkctl_debug_reg[] = {
	DCSS_DBG_REG(DCSS_BLKCTL_RESET_CTRL),
	DCSS_DBG_REG(DCSS_BLKCTL_CONTROL0),
};

struct dcss_blkctl_priv {
	struct dcss_soc *dcss;
	void __iomem *base_reg;

	bool hdmi_output;
	u32 clk_setting;
};

#ifdef CONFIG_DEBUG_FS
void dcss_blkctl_dump_regs(struct seq_file *s, void *data)
{
	struct dcss_soc *dcss = data;
	int j;

	seq_puts(s, ">> Dumping BLKCTL:\n");
	for (j = 0; j < ARRAY_SIZE(blkctl_debug_reg); j++)
		seq_printf(s, "%-35s(0x%04x) -> 0x%08x\n",
			   blkctl_debug_reg[j].name,
			   blkctl_debug_reg[j].ofs,
			   dcss_readl(dcss->blkctl_priv->base_reg +
				      blkctl_debug_reg[j].ofs));
}
#endif

static void dcss_blkctl_clk_reset(struct dcss_blkctl_priv *blkctl,
				  u32 assert, u32 deassert)
{
	if (assert)
		dcss_clr(assert, blkctl->base_reg + DCSS_BLKCTL_RESET_CTRL);

	if (deassert)
		dcss_set(deassert, blkctl->base_reg + DCSS_BLKCTL_RESET_CTRL);
}

void dcss_blkctl_cfg(struct dcss_soc *dcss)
{
	struct dcss_blkctl_priv *blkctl = dcss->blkctl_priv;

	if (blkctl->hdmi_output)
		dcss_writel((blkctl->clk_setting ^ HDMI_MIPI_CLK_SEL),
		    blkctl->base_reg + DCSS_BLKCTL_CONTROL0);
	else
		dcss_writel((blkctl->clk_setting ^ HDMI_MIPI_CLK_SEL) |
			    DISPMIX_PIXCLK_SEL,
			    blkctl->base_reg + DCSS_BLKCTL_CONTROL0);

	/* deassert clock domains resets */
	dcss_blkctl_clk_reset(blkctl, 0, 0xffffff);
}

int dcss_blkctl_init(struct dcss_soc *dcss, unsigned long blkctl_base)
{
	struct device_node *node = dcss->dev->of_node;
	int len;
	const char *disp_dev;
	struct dcss_blkctl_priv *blkctl;

	blkctl = devm_kzalloc(dcss->dev, sizeof(*blkctl), GFP_KERNEL);
	if (!blkctl)
		return -ENOMEM;

	blkctl->base_reg = devm_ioremap(dcss->dev, blkctl_base, SZ_4K);
	if (!blkctl->base_reg) {
		dev_err(dcss->dev, "unable to remap BLK CTRL base\n");
		return -ENOMEM;
	}

	blkctl->dcss = dcss;
	dcss->blkctl_priv = blkctl;

	disp_dev = of_get_property(node, "disp-dev", &len);
	if (!disp_dev || !strncmp(disp_dev, "hdmi_disp", 9))
		blkctl->hdmi_output = true;

	if (imx8_get_soc_revision() >= B0_SILICON_ID)
		blkctl->clk_setting = HDMI_MIPI_CLK_SEL;

	dcss_blkctl_cfg(dcss);

	return 0;
}

void dcss_blkctl_exit(struct dcss_soc *dcss)
{
	/* assert clock domains resets */
	dcss_blkctl_clk_reset(dcss->blkctl_priv,
			      B_CLK_RESETN | APB_CLK_RESETN | P_CLK_RESETN |
			      HDMI_RESETN | RTR_CLK_RESETN, 0);
}

/* disabled only by cold reset/reboot */
void dcss_blkctl_hdmi_secure_src_en(struct dcss_soc *dcss)
{
	struct dcss_blkctl_priv *blkctl = dcss->blkctl_priv;

	dcss_set(HDMI_SRC_SECURE_EN, blkctl->base_reg + DCSS_BLKCTL_CONTROL0);
}
EXPORT_SYMBOL(dcss_blkctl_hdmi_secure_src_en);

