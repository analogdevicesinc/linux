/*
 * Copyright (C) 2018 NXP
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

#include <linux/io.h>
#include <linux/delay.h>
#include <video/imx-dcss.h>
#include "dcss-prv.h"


struct dcss_pll_data {
	unsigned long vco1;
	unsigned long vco2;
	unsigned long fout;
	int refin;
	int ref_div;
	int fout_request;
	int fout_error;
	int r1;
	int f1;
	int r2;
	int f2;
	int q;
};

struct dcss_pll_priv {
	struct dcss_soc *dcss;
	void __iomem *base_reg;
	struct dcss_pll_data data;
};

static const int ref_sel[] = {
	25000000, 27000000, 27000000, 0
};

/* These are the specification limits for the SSCG PLL */
#define PLL_STAGE1_MIN_FREQ  1600000000
#define PLL_STAGE1_MAX_FREQ  2400000000

#define PLL_STAGE1_REF_MIN_FREQ  25000000
#define PLL_STAGE1_REF_MAX_FREQ  54000000

#define PLL_STAGE2_MIN_FREQ  1200000000
#define PLL_STAGE2_MAX_FREQ  2400000000

#define PLL_STAGE2_REF_MIN_FREQ  54000000
#define PLL_STAGE2_REF_MAX_FREQ  75000000

#define PLL_STAGE2_OUT_MIN_FREQ  20000000
#define PLL_STAGE2_OUT_MAX_FREQ  75000000

#define PLL_OUT_MAX_FREQ    600000000
#define PLL_OUT_MIN_FREQ     12000000

#define VIDEO_PLL2_CFG0		0x30360054
#define VIDEO_PLL2_CFG1		0x30360058
#define VIDEO_PLL2_CFG2		0x3036005c
#define VIDEO_PLL2_CFG0_OFFSET       0x0054
#define VIDEO_PLL2_CFG1_OFFSET       0x0058
#define VIDEO_PLL2_CFG2_OFFSET       0x005c

/* SYS PLL1/2/3 VIDEO PLL2 DRAM PLL */
#define SSCG_PLL_LOCK_MASK		BIT(31)
#define SSCG_PLL_CLKE_MASK		BIT(25)
#define SSCG_PLL_VIDEO_PLL2_CLKE_MASK	BIT(9)
#define SSCG_PLL_PD_MASK		BIT(7)
#define SSCG_PLL_BYPASS1_MASK		BIT(5)
#define SSCG_PLL_BYPASS2_MASK		BIT(4)
#define SSCG_PLL_LOCK_SEL_MASK		BIT(3)
#define SSCG_PLL_COUNTCLK_SEL_MASK	BIT(2)
#define SSCG_PLL_REFCLK_SEL_MASK	0x3
#define SSCG_PLL_REFCLK_SEL_OSC_25M	0
#define SSCG_PLL_REFCLK_SEL_OSC_27M	1
#define SSCG_PLL_REFCLK_SEL_HDMI_PHY_27M	2
#define SSCG_PLL_REFCLK_SEL_CLK_PN	3

#define SSCG_PLL_REF_DIVR1_MASK		(0x7 << 25)
#define SSCG_PLL_REF_DIVR1_SHIFT	25
#define SSCG_PLL_REF_DIVR1_VAL(n)	(((n) << 25) & SSCG_PLL_REF_DIVR1_MASK)
#define SSCG_PLL_REF_DIVR2_MASK		(0x3f << 19)
#define SSCG_PLL_REF_DIVR2_SHIFT	19
#define SSCG_PLL_REF_DIVR2_VAL(n)	(((n) << 19) & SSCG_PLL_REF_DIVR2_MASK)
#define SSCG_PLL_FEEDBACK_DIV_F1_MASK	(0x3f << 13)
#define SSCG_PLL_FEEDBACK_DIV_F1_SHIFT	13
#define SSCG_PLL_FEEDBACK_DIV_F1_VAL(n)	(((n) << 13) & \
					 SSCG_PLL_FEEDBACK_DIV_F1_MASK)
#define SSCG_PLL_FEEDBACK_DIV_F2_MASK	(0x3f << 7)
#define SSCG_PLL_FEEDBACK_DIV_F2_SHIFT	7
#define SSCG_PLL_FEEDBACK_DIV_F2_VAL(n)	(((n) << 7) & \
					 SSCG_PLL_FEEDBACK_DIV_F2_MASK)
#define SSCG_PLL_OUTPUT_DIV_VAL_MASK	(0x3f << 1)
#define SSCG_PLL_OUTPUT_DIV_VAL_SHIFT	1
#define SSCG_PLL_OUTPUT_DIV_VAL(n)	(((n) << 1) & \
					 SSCG_PLL_OUTPUT_DIV_VAL_MASK)
#define SSCG_PLL_FILTER_RANGE_MASK	0x1

static void pll_show(struct dcss_pll_priv *pll)
{
	dev_dbg(pll->dcss->dev,
		"vco1 %lu r %d f %d vco2 %lu (%lu) ref_div %d r %d f %d q %d out %lu",
		pll->data.vco1, pll->data.r1, pll->data.f1, pll->data.vco2,
		pll->data.vco2 / 2, pll->data.ref_div, pll->data.r2,
		pll->data.f2, pll->data.q, pll->data.fout);
}

static int pll2_check_match(struct dcss_pll_priv *pll,
			    struct dcss_pll_data *temp_data)
{
	if ((pll->data.vco2 <= PLL_STAGE2_MAX_FREQ) &&
	    (pll->data.vco2 >= PLL_STAGE2_MIN_FREQ)) {
		/* found new frequency */
		if (pll->data.fout_request == pll->data.fout) {
			*temp_data = pll->data;
			dev_dbg(pll->dcss->dev,
				"found exact match - vco1 %ld r %d f %d vco2 %ld (%ld) ref_div %d r %d f %d q %d out %lu\n",
				pll->data.vco1, pll->data.r1, pll->data.f1,
				pll->data.vco2, pll->data.vco2 / 2,
				pll->data.ref_div, pll->data.r2, pll->data.f2,
				pll->data.q, pll->data.fout);
			return 0;
		} else if (abs(pll->data.fout_error) >
			       abs(pll->data.fout - pll->data.fout_request)) {
			pll->data.fout_error = pll->data.fout -
				pll->data.fout_request;
			*temp_data = pll->data; /* copy pll */
			dev_dbg(pll->dcss->dev,
				"found new best match delta %d - vco1 %ld r %d f %d vco2 %ld (%ld) ref_div %d r %d f %d q %d out %lu\n",
				pll->data.fout_error, pll->data.vco1,
				pll->data.r1, pll->data.f1, pll->data.vco2,
				pll->data.vco2 / 2, pll->data.ref_div,
				pll->data.r2, pll->data.f2, pll->data.q,
				pll->data.fout);
		}
	}
	return -1;
}

static int pll2_find_match(struct dcss_pll_priv *pll,
			   struct dcss_pll_data *temp_data)
{
	const long r_max = 63;
	const long f_max = 63;
	const long q_max = 63;

	for (pll->data.r2 = 0; pll->data.r2 <= r_max; pll->data.r2++) {
		pll->data.ref_div = (pll->data.vco1 / (pll->data.r2 + 1));
		if ((pll->data.ref_div <= PLL_STAGE2_REF_MAX_FREQ) &&
		    (pll->data.ref_div >= PLL_STAGE2_REF_MIN_FREQ)) {
			for (pll->data.f2 = 0; pll->data.f2 <= f_max;
			     pll->data.f2++) {
				for (pll->data.q = 0; pll->data.q < q_max;
				     pll->data.q++) {
					pll->data.vco2 =
					    (pll->data.vco1 /
					    (pll->data.r2 + 1)) *
					    2 * (pll->data.f2 + 1);
					pll->data.fout =
					    pll->data.vco2 /
						(2 * (pll->data.q + 1));

					if (!pll2_check_match(pll, temp_data))
						return 0;
				}
			}
		}
	}
	return -1;
}
/*
 * pll1_find_match starts the iteration over all pll1 valid frequencies. For
 * every valid frequency pll2_find_match is called. Once an exact match is
 * found then match frequency is returned. If no exact match is found then the
 * closest match is returned.
 *
 */
static int pll1_find_match(struct dcss_pll_priv *pll)
{
	int ret;
	long ref_div;
	const long r_max = 7;
	const long f_max = 63;
	struct dcss_pll_data temp_data;

	temp_data.fout_error = PLL_OUT_MAX_FREQ;
	pll->data.fout_error = PLL_OUT_MAX_FREQ;

	for (pll->data.r1 = 0; pll->data.r1 <= r_max; pll->data.r1++) {
		ref_div = (pll->data.refin / (pll->data.r1 + 1));
		if ((ref_div <= PLL_STAGE1_REF_MAX_FREQ) &&
		    (ref_div >= PLL_STAGE1_REF_MIN_FREQ)) {
			for (pll->data.f1 = 0; pll->data.f1 <= f_max;
			      pll->data.f1++) {
				pll->data.vco1 =
				    (pll->data.refin / (pll->data.r1 + 1)) * 2 *
				    (pll->data.f1 + 1);
				if ((pll->data.vco1 <= PLL_STAGE1_MAX_FREQ) &&
				    (pll->data.vco1 >=  PLL_STAGE1_MIN_FREQ)) {
					/*pll_show(pll);*/
					ret = pll2_find_match(pll, &temp_data);
					/* exact match */
					if (ret == 0) {
						pll->data = temp_data;
						return 0;
					}
				}
			}
		}
	}

	/* no exact match */
	pll->data = temp_data;

	return -1;
}

/*
 * The PLL typical lifecycle is as follows:
 *     dcss_pll_init(0
 *          dcss_pll_set_rate() (optional default rate is 594 MHz)
 *          dcss_pll_enable()
 *          dcss_pll_disable()
 *     dcss_pll_exit(0
 *
 * To change the PLL rate:
 *          dcss_pll_disable()
 *          dcss_pll_set_rate()
 *          dcss_pll_enable()
 */

int dcss_pll_init(struct dcss_soc *dcss, unsigned long pll_base)
{
	struct dcss_pll_priv *pll;
	int f_actual;

	pll = devm_kzalloc(dcss->dev, sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return -ENOMEM;

	dcss->pll_priv = pll;
	pll->dcss = dcss;

	pll->base_reg = devm_ioremap(dcss->dev, pll_base, SZ_4K);
	if (!pll->base_reg) {
		dev_err(pll->dcss->dev,
			"pll: unable to map pll base at 0x%08lx\n",
			 pll_base);
		return -ENOMEM;
	}

	dcss_pll_set_rate(dcss, 594000000, SSCG_PLL_REFCLK_SEL_OSC_27M,
			  &f_actual);
	return 0;
}

void dcss_pll_exit(struct dcss_soc *dcss)
{
	dcss_pll_disable(dcss);
}

int dcss_pll_set_rate(struct dcss_soc *dcss, u32 freq, u32 ref_clk,
		      u32 *actual_freq)
{
	struct dcss_pll_priv *pll = dcss->pll_priv;
	void __iomem *reg = pll->base_reg;
	u32 pll_control_reg, val_cfg2;

	dev_dbg(pll->dcss->dev, "initial pll reg1 %x %x %x\n",
		 readl(reg + VIDEO_PLL2_CFG0_OFFSET),
		 readl(reg + VIDEO_PLL2_CFG1_OFFSET),
		 readl(reg + VIDEO_PLL2_CFG2_OFFSET));

	if (freq == 27000000) {
		/* use hdmi 27 mhz */
		pll_control_reg = readl(reg + VIDEO_PLL2_CFG0_OFFSET);
		pll_control_reg &= ~SSCG_PLL_REFCLK_SEL_MASK;
		writel(pll_control_reg, reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(pll->dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, pll_control_reg);

		pll_control_reg = readl(reg + VIDEO_PLL2_CFG0_OFFSET);
		pll_control_reg |= ref_clk & SSCG_PLL_REFCLK_SEL_MASK;
		writel(pll_control_reg, reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(pll->dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, pll_control_reg);
		pll->data.fout = freq;
	} else {
		/* these are settings generate  1188 MHz */
		int ref_freq;
		int match;

		if (ref_clk > SSCG_PLL_REFCLK_SEL_HDMI_PHY_27M) {
			dev_dbg(pll->dcss->dev,
				"%s(): ref_clk index is out of range!\n",
				__func__);
			ref_freq = ref_sel[ref_clk];
		} else
			ref_freq = ref_sel[SSCG_PLL_REFCLK_SEL_HDMI_PHY_27M];

		pll->data.refin = ref_freq;
		pll->data.fout_request = freq;

		match = pll1_find_match(pll);
		dev_dbg(dcss->dev, "pll_find_match found %s match at %lu\n",
			match == 0 ? "exact" : "best", pll->data.fout);

		pll_show(pll);

		/* reference 2 requires HDMI to be initialized */
		pll_control_reg = readl(reg + VIDEO_PLL2_CFG0_OFFSET);
		pll_control_reg &= ~SSCG_PLL_REFCLK_SEL_MASK;
		writel(pll_control_reg, reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(pll->dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, pll_control_reg);

		pll_control_reg = readl(reg + VIDEO_PLL2_CFG0_OFFSET);
		pll_control_reg |= ref_clk & SSCG_PLL_REFCLK_SEL_MASK;
		writel(pll_control_reg, reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(pll->dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, pll_control_reg);

		val_cfg2 = SSCG_PLL_REF_DIVR1_VAL(pll->data.r1) |
		    SSCG_PLL_REF_DIVR2_VAL(pll->data.r2) |
		    SSCG_PLL_FEEDBACK_DIV_F1_VAL(pll->data.f1) |
		    SSCG_PLL_FEEDBACK_DIV_F2_VAL(pll->data.f2) |
		    SSCG_PLL_OUTPUT_DIV_VAL(pll->data.q);

		writel(val_cfg2, reg + VIDEO_PLL2_CFG2_OFFSET);
		dev_dbg(pll->dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG2_OFFSET, val_cfg2);

	}
	*actual_freq = pll->data.fout;

	dev_info(pll->dcss->dev,
		 "Configured video pll 2 with ref_clk %d freq %d (actual %d)\n",
		ref_clk, freq, *actual_freq);

	return  0;
}

int dcss_pll_enable(struct dcss_soc *dcss)
{
	struct dcss_pll_priv *pll = dcss->pll_priv;
	void __iomem *base_reg = pll->base_reg;
	u32 reg;

	if (pll->data.fout != 27000000) {
		int lock_count = 0;

		/* Clear power down bit */
		reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
		reg &= ~SSCG_PLL_PD_MASK;
		writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, reg);

		/* Enable clk output  */
		reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
		reg |= SSCG_PLL_VIDEO_PLL2_CLKE_MASK;
		writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, reg);

		/* Clear bypass */
		reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
		reg &= ~SSCG_PLL_BYPASS1_MASK;
		writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, reg);

		udelay(100);

		reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
		reg &= ~SSCG_PLL_BYPASS2_MASK;
		writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, reg);
		/* Wait until lock */

		while (!(readl(base_reg + VIDEO_PLL2_CFG0_OFFSET) &
			 SSCG_PLL_LOCK_MASK)) {
			udelay(100);
			if (lock_count++ > (10 * 1000)) {
				dev_err(dcss->dev,
					"failed to lock video pll 2!\n");
				return 1;
			}
		}
	} else {
		/* use bypass mode for 27000000 */
		reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
		reg |= SSCG_PLL_BYPASS1_MASK;
		writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, reg);

		reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
		reg |= SSCG_PLL_BYPASS2_MASK;
		writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
		dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
			 VIDEO_PLL2_CFG0_OFFSET, reg);
	}

	return 0;
}

int dcss_pll_disable(struct dcss_soc *dcss)
{
	struct dcss_pll_priv *pll = dcss->pll_priv;
	void __iomem *base_reg = pll->base_reg;
	u32 reg;

	/* Disable clk output  */
	reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
	reg &= ~SSCG_PLL_VIDEO_PLL2_CLKE_MASK;
	writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
	dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
		 VIDEO_PLL2_CFG0_OFFSET, reg);

	/* Set power down bit */
	reg = readl(base_reg + VIDEO_PLL2_CFG0_OFFSET);
	reg |= SSCG_PLL_PD_MASK;
	writel(reg, base_reg + VIDEO_PLL2_CFG0_OFFSET);
	dev_dbg(dcss->dev, "pll reg offset %x data %x\n",
		 VIDEO_PLL2_CFG0_OFFSET, reg);

	return 0;
}
