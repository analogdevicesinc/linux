/*
 * IMX Display Mix GPR reset driver
 *
 * Copyright 2019 NXP
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
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <dt-bindings/reset/imx8mm-dispmix.h>
#include <dt-bindings/reset/imx8mn-dispmix.h>

#define DRIVER_NAME		"dispmix_reset_drv"

/* DISPMIX GPR registers */
#define DISPLAY_MIX_SFT_RSTN_CSR		0x00
#define DISPLAY_MIX_CLK_EN_CSR			0x00
#define GPR_MIPI_RESET_DIV			0x00

struct dispmix_reset_controller {
	struct reset_controller_dev rcdev;
	struct device *dev;
	struct regmap *rstcon;
	struct clk *ipg_clk;
	bool active_low;
};

struct dispmix_reset_entry {
	uint32_t reg_off;
	uint32_t bit_off;
};

struct dispmix_reset_pdata {
	const struct dispmix_reset_entry *resets;
	uint32_t nr_resets;
	const struct regmap_config *config;
};

#define RESET_ENTRY(id, reg, bit)			\
	[id] = { .reg_off = (reg), .bit_off = (bit) }

static const struct dispmix_reset_entry imx8mm_sft_rstn[] = {
	/* dispmix reset entry */
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_CHIP_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 0),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_IPG_HARD_ASYNC_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 1),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_CSI_HRESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 2),
	RESET_ENTRY(IMX8MM_CAMERA_PIXEL_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 3),
	RESET_ENTRY(IMX8MM_MIPI_CSI_I_PRESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 4),
	RESET_ENTRY(IMX8MM_MIPI_DSI_I_PRESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 5),
	RESET_ENTRY(IMX8MM_BUS_RSTN_BLK_SYNC,
		    DISPLAY_MIX_SFT_RSTN_CSR, 6),
};

static const struct dispmix_reset_entry imx8mm_clk_en[] = {
	/* dispmix clock enable entry */
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_CSI_HCLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  0),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_SPU_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  1),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_MEM_WRAPPER_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  2),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_IPG_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  3),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_IPG_CLK_S_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  4),
	RESET_ENTRY(IMX8MM_CSI_BRIDGE_IPG_CLK_S_RAW_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  5),
	RESET_ENTRY(IMX8MM_LCDIF_APB_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  6),
	RESET_ENTRY(IMX8MM_LCDIF_PIXEL_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  7),
	RESET_ENTRY(IMX8MM_MIPI_DSI_PCLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  8),
	RESET_ENTRY(IMX8MM_MIPI_DSI_CLKREF_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  9),
	RESET_ENTRY(IMX8MM_MIPI_CSI_ACLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR, 10),
	RESET_ENTRY(IMX8MM_MIPI_CSI_PCLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR, 11),
	RESET_ENTRY(IMX8MM_BUS_BLK_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR, 12),
};

static const struct dispmix_reset_entry imx8mm_mipi_rst[] = {
	/* mipi lanes reset entry */
	RESET_ENTRY(IMX8MM_MIPI_S_RESET,
		    GPR_MIPI_RESET_DIV, 16),
	RESET_ENTRY(IMX8MM_MIPI_M_RESET,
		    GPR_MIPI_RESET_DIV, 17),
};

static const struct dispmix_reset_entry imx8mn_sft_rstn[] = {
	/* dispmix reset entry */
	RESET_ENTRY(IMX8MN_MIPI_DSI_PCLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 0),
	RESET_ENTRY(IMX8MN_MIPI_DSI_CLKREF_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 1),
	RESET_ENTRY(IMX8MN_MIPI_CSI_PCLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 2),
	RESET_ENTRY(IMX8MN_MIPI_CSI_ACLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 3),
	RESET_ENTRY(IMX8MN_LCDIF_PIXEL_CLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 4),
	RESET_ENTRY(IMX8MN_LCDIF_APB_CLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 5),
	RESET_ENTRY(IMX8MN_ISI_PROC_CLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 6),
	RESET_ENTRY(IMX8MN_ISI_APB_CLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 7),
	RESET_ENTRY(IMX8MN_BUS_BLK_CLK_RESET,
		    DISPLAY_MIX_SFT_RSTN_CSR, 8),
};

static const struct dispmix_reset_entry imx8mn_clk_en[] = {
	/* dispmix clock enable entry */
	RESET_ENTRY(IMX8MN_MIPI_DSI_PCLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  0),
	RESET_ENTRY(IMX8MN_MIPI_DSI_CLKREF_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  1),
	RESET_ENTRY(IMX8MN_MIPI_CSI_PCLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  2),
	RESET_ENTRY(IMX8MN_MIPI_CSI_ACLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  3),
	RESET_ENTRY(IMX8MN_LCDIF_PIXEL_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  4),
	RESET_ENTRY(IMX8MN_LCDIF_APB_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  5),
	RESET_ENTRY(IMX8MN_ISI_PROC_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  6),
	RESET_ENTRY(IMX8MN_ISI_APB_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  7),
	RESET_ENTRY(IMX8MN_BUS_BLK_CLK_EN,
		    DISPLAY_MIX_CLK_EN_CSR,  8),
};

static const struct dispmix_reset_entry imx8mn_mipi_rst[] = {
	/* mipi lanes reset entry */
	RESET_ENTRY(IMX8MN_MIPI_S_RESET,
		    GPR_MIPI_RESET_DIV, 16),
	RESET_ENTRY(IMX8MN_MIPI_M_RESET,
		    GPR_MIPI_RESET_DIV, 17),
};

static const struct regmap_config sft_rstn_config = {
	.name = "sft_rstn",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x00,
};

static const struct regmap_config clk_en_config = {
	.name = "clk_en",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x00,
};

static const struct regmap_config mipi_rst_config = {
	.name = "mipi_rst",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = 0x00,
};

static const struct dispmix_reset_pdata imx8mm_sft_rstn_pdata = {
	.resets    = imx8mm_sft_rstn,
	.nr_resets = IMX8MM_DISPMIX_SFT_RSTN_NUM,
	.config    = &sft_rstn_config,
};

static const struct dispmix_reset_pdata imx8mm_clk_en_pdata = {
	.resets    = imx8mm_clk_en,
	.nr_resets = IMX8MM_DISPMIX_CLK_EN_NUM,
	.config    = &clk_en_config,
};

static const struct dispmix_reset_pdata imx8mm_mipi_rst_pdata = {
	.resets    = imx8mm_mipi_rst,
	.nr_resets = IMX8MM_MIPI_RESET_NUM,
	.config    = &mipi_rst_config,
};

static const struct dispmix_reset_pdata imx8mn_sft_rstn_pdata = {
	.resets    = imx8mn_sft_rstn,
	.nr_resets = IMX8MN_DISPMIX_SFT_RSTN_NUM,
	.config    = &sft_rstn_config,
};

static const struct dispmix_reset_pdata imx8mn_clk_en_pdata = {
	.resets    = imx8mn_clk_en,
	.nr_resets = IMX8MN_DISPMIX_CLK_EN_NUM,
	.config    = &clk_en_config,
};

static const struct dispmix_reset_pdata imx8mn_mipi_rst_pdata = {
	.resets    = imx8mn_mipi_rst,
	.nr_resets = IMX8MN_MIPI_RESET_NUM,
	.config    = &mipi_rst_config,
};

static const struct of_device_id dispmix_reset_dt_ids[] = {
	{
		.compatible = "fsl,imx8mm-dispmix-sft-rstn",
		.data = &imx8mm_sft_rstn_pdata,
	},
	{
		.compatible = "fsl,imx8mm-dispmix-clk-en",
		.data = &imx8mm_clk_en_pdata,
	},
	{
		.compatible = "fsl,imx8mm-dispmix-mipi-rst",
		.data = &imx8mm_mipi_rst_pdata,
	},
	{
		.compatible = "fsl,imx8mn-dispmix-sft-rstn",
		.data = &imx8mn_sft_rstn_pdata,
	},
	{
		.compatible = "fsl,imx8mn-dispmix-clk-en",
		.data = &imx8mn_clk_en_pdata,
	},
	{
		.compatible = "fsl,imx8mn-dispmix-mipi-rst",
		.data = &imx8mn_mipi_rst_pdata,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dispmix_reset_dt_ids);

static int dispmix_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct dispmix_reset_controller *drcdev;
	const struct of_device_id *of_id;
	const struct dispmix_reset_pdata *pdata;
	const struct dispmix_reset_entry *rstent;
	struct regmap *rstcon;

	if (id >= rcdev->nr_resets) {
		pr_info("dispmix reset: %lu is not a valid line\n", id);
		return -EINVAL;
	}

	drcdev = container_of(rcdev, struct dispmix_reset_controller, rcdev);
	of_id  = of_match_device(dispmix_reset_dt_ids, drcdev->dev);
	pdata = of_id->data;

	rstcon = drcdev->rstcon;
	rstent = &pdata->resets[id];

	pm_runtime_get_sync(drcdev->dev);
	regmap_update_bits(rstcon, rstent->reg_off,
			   1 << rstent->bit_off,
			   !drcdev->active_low << rstent->bit_off);
	pm_runtime_put(drcdev->dev);

	return 0;
}

static int dispmix_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	struct dispmix_reset_controller *drcdev;
	const struct of_device_id *of_id;
	const struct dispmix_reset_pdata *pdata;
	const struct dispmix_reset_entry *rstent;
	struct regmap *rstcon;

	if (id >= rcdev->nr_resets) {
		pr_info("dispmix reset: %lu is not a valid line\n", id);
		return -EINVAL;
	}

	drcdev = container_of(rcdev, struct dispmix_reset_controller, rcdev);
	of_id  = of_match_device(dispmix_reset_dt_ids, drcdev->dev);
	pdata = of_id->data;

	rstcon = drcdev->rstcon;
	rstent = &pdata->resets[id];

	pm_runtime_get_sync(drcdev->dev);
	regmap_update_bits(rstcon, rstent->reg_off,
			   1 << rstent->bit_off,
			   !!drcdev->active_low << rstent->bit_off);
	pm_runtime_put(drcdev->dev);

	return 0;
}

static const struct reset_control_ops dispmix_reset_ops = {
	.assert   = dispmix_reset_assert,
	.deassert = dispmix_reset_deassert,
};

static int dispmix_reset_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id;
	struct dispmix_reset_controller *drcdev;
	const struct dispmix_reset_pdata *pdata;
	struct resource *res;
	void __iomem *regs;
	struct regmap *regmap;
	struct clk *apb_clk;

	drcdev = devm_kzalloc(dev, sizeof(*drcdev), GFP_KERNEL);
	if (!drcdev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	apb_clk = devm_clk_get(dev, "disp_apb_root_clk");
	if (IS_ERR(apb_clk)) {
		dev_err(dev, "Unable to get disp apb clock\n");
		return PTR_ERR(apb_clk);
	}

	drcdev->active_low = of_property_read_bool(np, "active_low");

	of_id = of_match_device(dispmix_reset_dt_ids, dev);
	pdata = of_id->data;

	/* init mmio regmap */
	regmap = regmap_init_mmio_clk(NULL, NULL,
				      regs, pdata->config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to init mmio regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	drcdev->rstcon = regmap;

	platform_set_drvdata(pdev, drcdev);
	pm_runtime_enable(dev);

	/* register reset controller */
	drcdev->dev = dev;
	drcdev->rcdev.of_node = dev->of_node;
	drcdev->rcdev.owner = THIS_MODULE;
	drcdev->rcdev.nr_resets = pdata->nr_resets;
	drcdev->rcdev.ops = &dispmix_reset_ops;

	return devm_reset_controller_register(dev, &drcdev->rcdev);
}

static int dispmix_reset_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver dispmix_reset_driver = {
	.probe  = dispmix_reset_probe,
	.remove = dispmix_reset_remove,
	.driver = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(dispmix_reset_dt_ids),
	},
};

builtin_platform_driver(dispmix_reset_driver);

MODULE_DESCRIPTION("IMX Display Mix reset driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
