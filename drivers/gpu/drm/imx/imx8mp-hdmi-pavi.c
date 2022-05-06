// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2020 NXP
 *
 * Programe Video/Audio Interface between LCDIF and HDMI Ctrl in HDMIMIX
 *
 */

#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <drm/drm_fourcc.h>

#include "imx8mp-hdmi-pavi.h"

#define DRIVER_NAME "imx-hdmi-pavi"

#define HTX_PVI_CTRL         0x0
#define HTX_PVI_IRQ_MASK     0x04
#define HTX_TMG_GEN_DISP_LRC 0x10
#define HTX_TMG_GEN_DE_ULC   0x14
#define HTX_TMG_GEN_DE_LRC   0x18
#define HTX_TMG_GEN_HSYNC    0x1c
#define HTX_TMG_GEN_VSYNC    0x20
#define HTX_TMG_GEN_IRQ0     0x24
#define HTX_TMG_GEN_IRQ1     0x28
#define HTX_TMG_GEN_IRQ2     0x2c
#define HTX_TMG_GEN_IRQ3     0x30
#define HTX_TMG_GEN_CFG      0x40

#define HTX_PAI_CTRL        0x800
#define HTX_PAI_CTRL_EXT    0x804
#define HTX_PAI_FIELD_CTRL  0x808

#define HTX_PAI_CTRL_ENABLE 1


static struct imx8mp_hdmi_pavi *gpavi;

/* PAI APIs  */
void imx8mp_hdmi_pai_enable(int channel, int width, int rate, int non_pcm)
{
	/* PAI set */
	writel((0x3030000 | ((channel-1) << 8)),
			gpavi->base + HTX_PAI_CTRL_EXT);

	/* hbr */
	if (non_pcm && width == 32 && channel == 8 && rate == 192000)
		writel(0x004e77df, gpavi->base + HTX_PAI_FIELD_CTRL);
	else if (width == 32)
		writel(0x1c8c675b, gpavi->base + HTX_PAI_FIELD_CTRL);
	else
		writel(0x1c0c675b, gpavi->base + HTX_PAI_FIELD_CTRL);

	/* PAI start running */
	writel(HTX_PAI_CTRL_ENABLE, gpavi->base + HTX_PAI_CTRL);
}
EXPORT_SYMBOL(imx8mp_hdmi_pai_enable);

void imx8mp_hdmi_pai_disable(void)
{
	/* stop PAI */
	writel(0, gpavi->base + HTX_PAI_CTRL);
}
EXPORT_SYMBOL(imx8mp_hdmi_pai_disable);

/* PVI APIs  */
void imx8mp_hdmi_pvi_enable(const struct drm_display_mode *mode)
{
	writel(0x00000003, gpavi->base + HTX_PVI_IRQ_MASK);
	writel(0x08970464, gpavi->base + HTX_TMG_GEN_DISP_LRC);
	writel(0x00bf0029, gpavi->base + HTX_TMG_GEN_DE_ULC);
	writel(0x083f0460, gpavi->base + HTX_TMG_GEN_DE_LRC);
	writel(0x0897002b, gpavi->base + HTX_TMG_GEN_HSYNC);
	writel(0x04640004, gpavi->base + HTX_TMG_GEN_VSYNC);
	writel(0x000100ff, gpavi->base + HTX_TMG_GEN_IRQ0);
	writel(0x000100f0, gpavi->base + HTX_TMG_GEN_IRQ1);
	writel(0x00010315, gpavi->base + HTX_TMG_GEN_IRQ2);
	writel(0x00010207, gpavi->base + HTX_TMG_GEN_IRQ3);
	writel(0x84640000, gpavi->base + HTX_TMG_GEN_CFG);

	/* DE/VSYN/HSYNC pol */
	if ((mode->flags & DRM_MODE_FLAG_PVSYNC) &&
			(mode->flags & DRM_MODE_FLAG_PHSYNC)) {
		writel(0x00377004, gpavi->base + HTX_PVI_CTRL);
		writel(0x00377005, gpavi->base + HTX_PVI_CTRL);
	} else {
		writel(0x00311004, gpavi->base + HTX_PVI_CTRL);
		writel(0x00311005, gpavi->base + HTX_PVI_CTRL);
	}
}
EXPORT_SYMBOL(imx8mp_hdmi_pvi_enable);

void imx8mp_hdmi_pvi_disable(void)
{
	/* Stop PVI */
	writel(0x0, gpavi->base + HTX_PVI_CTRL);
}
EXPORT_SYMBOL(imx8mp_hdmi_pvi_disable);

void imx8mp_hdmi_pavi_powerup(void)
{
	clk_prepare_enable(gpavi->clk_pvi);
	clk_prepare_enable(gpavi->clk_pai);

	/* deassert pai reset */
	if (!gpavi->reset_pai)
		reset_control_deassert(gpavi->reset_pai);

	/* deassert pvi reset */
	if (!gpavi->reset_pvi)
		reset_control_deassert(gpavi->reset_pvi);
}
EXPORT_SYMBOL(imx8mp_hdmi_pavi_powerup);

void imx8mp_hdmi_pavi_powerdown(void)
{
	/* set pvi reset */
	if (!gpavi->reset_pvi)
		reset_control_assert(gpavi->reset_pvi);

	/* set pai reset */
	if (!gpavi->reset_pai)
		reset_control_assert(gpavi->reset_pai);

	clk_disable_unprepare(gpavi->clk_pai);
	clk_disable_unprepare(gpavi->clk_pvi);
}
EXPORT_SYMBOL(imx8mp_hdmi_pavi_powerdown);

struct imx8mp_hdmi_pavi *imx8mp_hdmi_pavi_init(void)
{
	return gpavi;
}
EXPORT_SYMBOL(imx8mp_hdmi_pavi_init);

static int imx8mp_hdmi_pavi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx8mp_hdmi_pavi *pavi;
	struct resource *res;

	dev_dbg(dev, "%s: probe begin\n", __func__);

	pavi = devm_kzalloc(dev, sizeof(*pavi), GFP_KERNEL);
	if (!pavi) {
		dev_err(dev, "Can't allocate 'imx8mp pavi' structure\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	pavi->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pavi->base))
		return PTR_ERR(pavi->base);

	pavi->clk_pvi = devm_clk_get(dev, "pvi_clk");
	if (IS_ERR(pavi->clk_pvi)) {
		dev_err(dev, "No pvi clock get\n");
		return -EPROBE_DEFER;
	}

	pavi->clk_pai = devm_clk_get(dev, "pai_clk");
	if (IS_ERR(pavi->clk_pai)) {
		dev_err(dev, "No pai clock get\n");
		return -EPROBE_DEFER;
	}

	pavi->reset_pai = devm_reset_control_get(dev, "pai_rst");
	if (IS_ERR(pavi->reset_pai)) {
		dev_err(pavi->dev, "No PAI reset\n");
		return -EPROBE_DEFER;
	}

	pavi->reset_pvi = devm_reset_control_get(dev, "pvi_rst");
	if (IS_ERR(pavi->reset_pvi)) {
		dev_err(pavi->dev, "No PVI reset\n");
		return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, pavi);

	gpavi = pavi;

	dev_dbg(dev, "%s: probe success\n", __func__);
	return 0;
}

static int imx8mp_hdmi_pavi_remove(struct platform_device *pdev)
{
	gpavi = NULL;
	return 0;
}

static const struct of_device_id imx8mp_hdmi_pavi_dt_ids[] = {
	{ .compatible = "fsl,imx8mp-hdmi-pavi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx8mp_hdmi_pavi_dt_ids);

struct platform_driver imx8mp_hdmi_pavi_driver = {
	.probe    = imx8mp_hdmi_pavi_probe,
	.remove   = imx8mp_hdmi_pavi_remove,
	.driver   = {
		.name = DRIVER_NAME,
		.of_match_table = imx8mp_hdmi_pavi_dt_ids,
	},
};

module_platform_driver(imx8mp_hdmi_pavi_driver);

MODULE_DESCRIPTION("NXP i.MX8MP HDMI PAI/PVI Mix driver");
MODULE_AUTHOR("Sandor Yu <Sandor.yu@nxp.com>");
MODULE_LICENSE("GPL");
