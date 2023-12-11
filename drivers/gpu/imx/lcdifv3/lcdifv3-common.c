// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Copyright 2019,2022 NXP
 */

#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/types.h>
#include <drm/drm_fourcc.h>
#include <video/imx-lcdifv3.h>
#include <video/videomode.h>

#include "lcdifv3-regs.h"

#define DRIVER_NAME "imx-lcdifv3"

struct lcdifv3_soc {
	struct device *dev;

	int irq;
	void __iomem *base;
	struct regmap *gpr;
	atomic_t rpm_suspended;

	struct clk *clk_pix;
	struct clk *clk_disp_axi;
	struct clk *clk_disp_apb;

	u32 thres_low_mul;
	u32 thres_low_div;
	u32 thres_high_mul;
	u32 thres_high_div;
};

struct lcdifv3_soc_pdata {
	bool hsync_invert;
	bool vsync_invert;
	bool de_invert;
};

struct lcdifv3_platform_reg {
	struct lcdifv3_client_platformdata pdata;
	char *name;
};

static struct lcdifv3_platform_reg client_reg[] = {
	{
		.pdata = { },
		.name  = "imx-lcdifv3-crtc",
	},
};

static struct lcdifv3_soc_pdata imx8mp_lcdif1_pdata = {
	.hsync_invert = false,
	.vsync_invert = false,
	.de_invert    = false,
};

static struct lcdifv3_soc_pdata imx8mp_lcdif2_pdata = {
	.hsync_invert = false,
	.vsync_invert = false,
	.de_invert    = false,
};

static const struct of_device_id imx_lcdifv3_dt_ids[] = {
	{ .compatible = "fsl,imx93-lcdif", },
	{ .compatible = "fsl,imx8mp-lcdif1", .data = &imx8mp_lcdif1_pdata, },
	{ .compatible = "fsl,imx8mp-lcdif2", .data = &imx8mp_lcdif2_pdata, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_lcdifv3_dt_ids);

static int lcdifv3_enable_clocks(struct lcdifv3_soc *lcdifv3)
{
	int ret;

	if (lcdifv3->clk_disp_axi) {
		ret = clk_prepare_enable(lcdifv3->clk_disp_axi);
		if (ret)
			return ret;
	}

	if (lcdifv3->clk_disp_apb) {
		ret = clk_prepare_enable(lcdifv3->clk_disp_apb);
		if (ret)
			goto disable_disp_axi;
	}

	ret = clk_prepare_enable(lcdifv3->clk_pix);
	if (ret)
		goto disable_disp_apb;

	return 0;

disable_disp_apb:
	if (lcdifv3->clk_disp_apb)
		clk_disable_unprepare(lcdifv3->clk_disp_apb);
disable_disp_axi:
	if (lcdifv3->clk_disp_axi)
		clk_disable_unprepare(lcdifv3->clk_disp_axi);

	return ret;
}

static void lcdifv3_disable_clocks(struct lcdifv3_soc *lcdifv3)
{
	clk_disable_unprepare(lcdifv3->clk_pix);

	if (lcdifv3->clk_disp_axi)
		clk_disable_unprepare(lcdifv3->clk_disp_axi);

	if (lcdifv3->clk_disp_apb)
		clk_disable_unprepare(lcdifv3->clk_disp_apb);
}

static void lcdifv3_enable_plane_panic(struct lcdifv3_soc *lcdifv3)
{
	u32 panic_thres, thres_low, thres_high;

	/* apb clock has been enabled */

	/* As suggestion, the thres_low should be 1/3 FIFO,
	 * and thres_high should be 2/3 FIFO (The FIFO size
	 * is 8KB = 512 * 128bit).
	 * threshold = n * 128bit (n: 0 ~ 511)
	 */
	thres_low  = DIV_ROUND_UP(511 * lcdifv3->thres_low_mul,
			lcdifv3->thres_low_div);
	thres_high = DIV_ROUND_UP(511 * lcdifv3->thres_high_mul,
			lcdifv3->thres_high_div);

	panic_thres = PANIC0_THRES_PANIC_THRES_LOW(thres_low)	|
		      PANIC0_THRES_PANIC_THRES_HIGH(thres_high);

	writel(panic_thres, lcdifv3->base + LCDIFV3_PANIC0_THRES);

	/* Enable Panic:
	 *
	 * As designed, the panic won't trigger an irq,
	 * so it is unnecessary to handle this as an irq
	 * and NoC + QoS modules will handle panic
	 * automatically.
	 */
	writel(INT_ENABLE_D1_PLANE_PANIC_EN,
	       lcdifv3->base + LCDIFV3_INT_ENABLE_D1);
}

int lcdifv3_vblank_irq_get(struct lcdifv3_soc *lcdifv3)
{
	return lcdifv3->irq;
}
EXPORT_SYMBOL(lcdifv3_vblank_irq_get);

/* TODO: use VS_BLANK or VSYNC? */
void lcdifv3_vblank_irq_enable(struct lcdifv3_soc *lcdifv3)
{
	uint32_t int_enable_d0;

	int_enable_d0 = readl(lcdifv3->base + LCDIFV3_INT_ENABLE_D0);
	int_enable_d0 |= INT_STATUS_D0_VS_BLANK;

	/* W1C */
	writel(INT_STATUS_D0_VS_BLANK,
	       lcdifv3->base + LCDIFV3_INT_STATUS_D0);
	/* enable */
	writel(int_enable_d0,
	       lcdifv3->base + LCDIFV3_INT_ENABLE_D0);
}
EXPORT_SYMBOL(lcdifv3_vblank_irq_enable);

void lcdifv3_vblank_irq_disable(struct lcdifv3_soc *lcdifv3)
{
	uint32_t int_enable_d0;

	int_enable_d0 = readl(lcdifv3->base + LCDIFV3_INT_ENABLE_D0);
	int_enable_d0 &= ~INT_STATUS_D0_VS_BLANK;

	/* disable */
	writel(int_enable_d0,
	       lcdifv3->base + LCDIFV3_INT_ENABLE_D0);
	/* W1C */
	writel(INT_STATUS_D0_VS_BLANK,
	       lcdifv3->base + LCDIFV3_INT_STATUS_D0);
}
EXPORT_SYMBOL(lcdifv3_vblank_irq_disable);

void lcdifv3_vblank_irq_clear(struct lcdifv3_soc *lcdifv3)
{
	/* W1C */
	writel(INT_STATUS_D0_VS_BLANK,
	       lcdifv3->base + LCDIFV3_INT_STATUS_D0);
}
EXPORT_SYMBOL(lcdifv3_vblank_irq_clear);

static uint32_t lcdifv3_get_bpp_from_fmt(uint32_t format)
{
	/* TODO: only support RGB for now */

	switch (format) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		return 16;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		return 32;
	default:
		/* unsupported format */
		return 0;
	}
}

/*
 * Get the bus format supported by LCDIF
 * according to drm fourcc format
 */
int lcdifv3_get_bus_fmt_from_pix_fmt(struct lcdifv3_soc *lcdifv3,
				     uint32_t format)
{
	uint32_t bpp;

	bpp = lcdifv3_get_bpp_from_fmt(format);
	if (!bpp)
		return -EINVAL;

	switch (bpp) {
	case 16:
		return MEDIA_BUS_FMT_RGB565_1X16;
	case 18:
		return MEDIA_BUS_FMT_RGB666_1X18;
	case 24:
	case 32:
		return MEDIA_BUS_FMT_RGB888_1X24;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(lcdifv3_get_bus_fmt_from_pix_fmt);

int lcdifv3_set_pix_fmt(struct lcdifv3_soc *lcdifv3, u32 format)
{
	uint32_t ctrldescl0_5 = 0;

	ctrldescl0_5 = readl(lcdifv3->base + LCDIFV3_CTRLDESCL0_5);

	ctrldescl0_5 &= ~(CTRLDESCL0_5_BPP(0xf) | CTRLDESCL0_5_YUV_FORMAT(0x3));

	switch (format) {
	case DRM_FORMAT_RGB565:
		ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP16_RGB565);
		break;
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP16_ARGB1555);
		break;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
		ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP32_ARGB8888);
		break;
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
		ctrldescl0_5 |= CTRLDESCL0_5_BPP(BPP32_ABGR8888);
		break;
	default:
		dev_err(lcdifv3->dev, "unsupported pixel format: %p4cc\n",
			&format);
		return -EINVAL;
	}

	writel(ctrldescl0_5,  lcdifv3->base + LCDIFV3_CTRLDESCL0_5);

	return 0;
}
EXPORT_SYMBOL(lcdifv3_set_pix_fmt);

void lcdifv3_set_bus_fmt(struct lcdifv3_soc *lcdifv3, u32 bus_format)
{
	uint32_t disp_para = 0;

	disp_para = readl(lcdifv3->base + LCDIFV3_DISP_PARA);

	/* clear line pattern bits */
	disp_para &= ~DISP_PARA_LINE_PATTERN(0xf);

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		disp_para |= DISP_PARA_LINE_PATTERN(LP_RGB565);
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		disp_para |= DISP_PARA_LINE_PATTERN(LP_RGB888_OR_YUV444);
		break;
	default:
		dev_err(lcdifv3->dev, "unknown bus format: %#x\n", bus_format);
		return;
	}

	/* config display mode: default is normal mode */
	disp_para &= ~DISP_PARA_DISP_MODE(3);
	disp_para |= DISP_PARA_DISP_MODE(0);

	writel(disp_para, lcdifv3->base + LCDIFV3_DISP_PARA);
}
EXPORT_SYMBOL(lcdifv3_set_bus_fmt);

void lcdifv3_set_fb_addr(struct lcdifv3_soc *lcdifv3, int id, u32 addr)
{
	switch (id) {
	case 0:
		/* primary plane */
		writel(addr, lcdifv3->base + LCDIFV3_CTRLDESCL_LOW0_4);
		break;
	default:
		/* TODO: add overlay support */
		return;
	}
}
EXPORT_SYMBOL(lcdifv3_set_fb_addr);

void lcdifv3_set_pitch(struct lcdifv3_soc *lcdifv3, unsigned int pitch)
{
	uint32_t ctrldescl0_3 = 0;

	/* config P_SIZE and T_SIZE:
	 * 1. P_SIZE and T_SIZE should never
	 *    be less than AXI bus width.
	 * 2. P_SIZE should never be less than T_SIZE.
	 */
	ctrldescl0_3 |= CTRLDESCL0_3_P_SIZE(2);
	ctrldescl0_3 |= CTRLDESCL0_3_T_SIZE(2);

	/* config pitch */
	ctrldescl0_3 |= CTRLDESCL0_3_PITCH(pitch);

	/* enable frame clear to clear FIFO data on
	 * every vsync blank period to make sure no
	 * dirty data exits to affect next frame
	 * display, otherwise some flicker issue may
	 * be observed in some cases.
	 */
	ctrldescl0_3 |= CTRLDESCL0_3_STATE_CLEAR_VSYNC;

	writel(ctrldescl0_3, lcdifv3->base + LCDIFV3_CTRLDESCL0_3);
}
EXPORT_SYMBOL(lcdifv3_set_pitch);


void lcdifv3_set_mode(struct lcdifv3_soc *lcdifv3, struct videomode *vmode)
{
	const struct of_device_id *of_id =
			of_match_device(imx_lcdifv3_dt_ids, lcdifv3->dev);
	const struct lcdifv3_soc_pdata *soc_pdata;
	u32 disp_size, hsyn_para, vsyn_para, vsyn_hsyn_width, ctrldescl0_1;

	if (unlikely(!of_id))
		return;
	soc_pdata = of_id->data;

	/* set pixel clock rate */
	clk_disable_unprepare(lcdifv3->clk_pix);
	clk_set_rate(lcdifv3->clk_pix, vmode->pixelclock);
	clk_prepare_enable(lcdifv3->clk_pix);

	/* config display timings */
	disp_size = DISP_SIZE_DELTA_Y(vmode->vactive) |
		    DISP_SIZE_DELTA_X(vmode->hactive);
	writel(disp_size, lcdifv3->base + LCDIFV3_DISP_SIZE);

	WARN_ON(!vmode->hback_porch || !vmode->hfront_porch);
	hsyn_para = HSYN_PARA_BP_H(vmode->hback_porch) |
		    HSYN_PARA_FP_H(vmode->hfront_porch);
	writel(hsyn_para, lcdifv3->base + LCDIFV3_HSYN_PARA);

	WARN_ON(!vmode->vback_porch || !vmode->vfront_porch);
	vsyn_para = VSYN_PARA_BP_V(vmode->vback_porch) |
		    VSYN_PARA_FP_V(vmode->vfront_porch);
	writel(vsyn_para, lcdifv3->base + LCDIFV3_VSYN_PARA);

	WARN_ON(!vmode->vsync_len || !vmode->hsync_len);
	vsyn_hsyn_width = VSYN_HSYN_WIDTH_PW_V(vmode->vsync_len) |
			  VSYN_HSYN_WIDTH_PW_H(vmode->hsync_len);
	writel(vsyn_hsyn_width, lcdifv3->base + LCDIFV3_VSYN_HSYN_WIDTH);

	/* config layer size */
	/* TODO: 32bits alignment for width */
	ctrldescl0_1 = CTRLDESCL0_1_HEIGHT(vmode->vactive) |
		       CTRLDESCL0_1_WIDTH(vmode->hactive);
	writel(ctrldescl0_1, lcdifv3->base + LCDIFV3_CTRLDESCL0_1);

	/* Polarities */
	if (soc_pdata) {
		if ((soc_pdata->hsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_HSYNC_HIGH) ||
		    (!soc_pdata->hsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_HSYNC_LOW))
			writel(CTRL_INV_HS, lcdifv3->base + LCDIFV3_CTRL_SET);
		else
			writel(CTRL_INV_HS, lcdifv3->base + LCDIFV3_CTRL_CLR);

		if ((soc_pdata->vsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_VSYNC_HIGH) ||
		    (!soc_pdata->vsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_VSYNC_LOW))
			writel(CTRL_INV_VS, lcdifv3->base + LCDIFV3_CTRL_SET);
		else
			writel(CTRL_INV_VS, lcdifv3->base + LCDIFV3_CTRL_CLR);

		if ((soc_pdata->de_invert &&
		     vmode->flags & DISPLAY_FLAGS_DE_HIGH) ||
		    (!soc_pdata->de_invert &&
		     vmode->flags & DISPLAY_FLAGS_DE_LOW))
			writel(CTRL_INV_DE, lcdifv3->base + LCDIFV3_CTRL_SET);
		else
			writel(CTRL_INV_DE, lcdifv3->base + LCDIFV3_CTRL_CLR);
	} else {
		if (vmode->flags & DISPLAY_FLAGS_HSYNC_LOW)
			writel(CTRL_INV_HS, lcdifv3->base + LCDIFV3_CTRL_SET);
		else
			writel(CTRL_INV_HS, lcdifv3->base + LCDIFV3_CTRL_CLR);
		if (vmode->flags & DISPLAY_FLAGS_VSYNC_LOW)
			writel(CTRL_INV_VS, lcdifv3->base + LCDIFV3_CTRL_SET);
		else
			writel(CTRL_INV_VS, lcdifv3->base + LCDIFV3_CTRL_CLR);
		if (vmode->flags & DISPLAY_FLAGS_DE_LOW)
			writel(CTRL_INV_DE, lcdifv3->base + LCDIFV3_CTRL_SET);
		else
			writel(CTRL_INV_DE, lcdifv3->base + LCDIFV3_CTRL_CLR);
	}

	if (vmode->flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
		writel(CTRL_INV_PXCK, lcdifv3->base + LCDIFV3_CTRL_SET);
	else
		writel(CTRL_INV_PXCK, lcdifv3->base + LCDIFV3_CTRL_CLR);
}
EXPORT_SYMBOL(lcdifv3_set_mode);

void lcdifv3_en_shadow_load(struct lcdifv3_soc *lcdifv3)
{
	u32 ctrldescl0_5;

	ctrldescl0_5 = readl(lcdifv3->base + LCDIFV3_CTRLDESCL0_5);
	ctrldescl0_5 |= CTRLDESCL0_5_SHADOW_LOAD_EN;

	writel(ctrldescl0_5, lcdifv3->base + LCDIFV3_CTRLDESCL0_5);
}
EXPORT_SYMBOL(lcdifv3_en_shadow_load);

void lcdifv3_enable_controller(struct lcdifv3_soc *lcdifv3)
{
	u32 disp_para, ctrldescl0_5;

	disp_para = readl(lcdifv3->base + LCDIFV3_DISP_PARA);
	ctrldescl0_5 = readl(lcdifv3->base + LCDIFV3_CTRLDESCL0_5);

	/* disp on */
	disp_para |= DISP_PARA_DISP_ON;
	writel(disp_para, lcdifv3->base + LCDIFV3_DISP_PARA);

	/* enable layer dma */
	ctrldescl0_5 |= CTRLDESCL0_5_EN;
	writel(ctrldescl0_5, lcdifv3->base + LCDIFV3_CTRLDESCL0_5);
}
EXPORT_SYMBOL(lcdifv3_enable_controller);

void lcdifv3_disable_controller(struct lcdifv3_soc *lcdifv3)
{
	u32 disp_para, ctrldescl0_5;

	disp_para = readl(lcdifv3->base + LCDIFV3_DISP_PARA);
	ctrldescl0_5 = readl(lcdifv3->base + LCDIFV3_CTRLDESCL0_5);

	/* disable dma */
	ctrldescl0_5 &= ~CTRLDESCL0_5_EN;
	writel(ctrldescl0_5, lcdifv3->base + LCDIFV3_CTRLDESCL0_5);

	/* dma config only takes effect at the end of
	 * one frame, so add delay to wait dma disable
	 * done before turn off disp.
	 */
	usleep_range(20000, 25000);

	/* disp off */
	disp_para &= ~DISP_PARA_DISP_ON;
	writel(disp_para, lcdifv3->base + LCDIFV3_DISP_PARA);
}
EXPORT_SYMBOL(lcdifv3_disable_controller);

static int platform_remove_device_fn(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static void platform_device_unregister_children(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, platform_remove_device_fn);
}

static DEFINE_MUTEX(lcdifv3_client_id_mutex);
static int lcdifv3_client_id;

static int lcdifv3_add_client_devices(struct lcdifv3_soc *lcdifv3)
{
	int ret = 0, i, id;
	struct device *dev = lcdifv3->dev;
	struct platform_device *pdev = NULL;
	struct device_node *of_node;

	for (i = 0; i < ARRAY_SIZE(client_reg); i++) {
		of_node = of_graph_get_port_by_id(dev->of_node, i);
		if (!of_node) {
			dev_info(dev, "no port@%d node in %s\n",
				 i, dev->of_node->full_name);
			continue;
		}
		of_node_put(of_node);

		mutex_lock(&lcdifv3_client_id_mutex);
		id = lcdifv3_client_id++;
		mutex_unlock(&lcdifv3_client_id_mutex);

		pdev = platform_device_alloc(client_reg[i].name, id);
		if (!pdev) {
			dev_err(dev, "Can't allocate port pdev\n");
			ret = -ENOMEM;
			goto err_register;
		}

		pdev->dev.parent = dev;
		client_reg[i].pdata.of_node = of_node;

		/* make child device 'dma_mask' to point to its
		 * coherent dma mask, otherwise later probe will
		 * print warning message: 'DMA mask not set'.
		 */
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

		ret = platform_device_add_data(pdev, &client_reg[i].pdata,
					       sizeof(client_reg[i].pdata));
		if (!ret)
			ret = platform_device_add(pdev);
		if (ret) {
			platform_device_put(pdev);
			goto err_register;
		}

		pdev->dev.of_node = of_node;
	}

	if (!pdev)
		return -ENODEV;

	return 0;

err_register:
	platform_device_unregister_children(to_platform_device(dev));
	return ret;
}

static int imx_lcdifv3_check_thres_value(u32 mul, u32 div)
{
	if (!div)
		return -EINVAL;

	if (mul > div)
		return -EINVAL;

	return 0;
}

static void imx_lcdifv3_of_parse_thres(struct lcdifv3_soc *lcdifv3)
{
	int ret;
	u32 thres_low[2], thres_high[2];
	struct device_node *np = lcdifv3->dev->of_node;

	/* default 'thres-low' value:  FIFO * 1/3;
	 * default 'thres-high' value: FIFO * 2/3.
	 */
	lcdifv3->thres_low_mul	= 1;
	lcdifv3->thres_low_div	= 3;
	lcdifv3->thres_high_mul	= 2;
	lcdifv3->thres_high_div	= 3;

	ret = of_property_read_u32_array(np, "thres-low", thres_low, 2);
	if (!ret) {
		/* check the value effectiveness */
		ret = imx_lcdifv3_check_thres_value(thres_low[0], thres_low[1]);
		if (!ret) {
			lcdifv3->thres_low_mul	= thres_low[0];
			lcdifv3->thres_low_div	= thres_low[1];
		}
	}

	ret = of_property_read_u32_array(np, "thres-high", thres_high, 2);
	if (!ret) {
		/* check the value effectiveness */
		ret = imx_lcdifv3_check_thres_value(thres_high[0], thres_high[1]);
		if (!ret) {
			lcdifv3->thres_high_mul	= thres_high[0];
			lcdifv3->thres_high_div	= thres_high[1];
		}
	}
}

static int imx_lcdifv3_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lcdifv3_soc *lcdifv3;
	struct resource *res;

	dev_dbg(dev, "%s: probe begin\n", __func__);

	lcdifv3 = devm_kzalloc(dev, sizeof(*lcdifv3), GFP_KERNEL);
	if (!lcdifv3) {
		dev_err(dev, "Can't allocate 'lcdifv3_soc' structure\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	lcdifv3->irq = platform_get_irq(pdev, 0);
	if (lcdifv3->irq < 0) {
		dev_err(dev, "No irq get, ret=%d\n", lcdifv3->irq);
		return lcdifv3->irq;
	}

	lcdifv3->clk_pix = devm_clk_get(dev, "pix");
	if (IS_ERR(lcdifv3->clk_pix)) {
		ret = PTR_ERR(lcdifv3->clk_pix);
		dev_err(dev, "No pix clock get: %d\n", ret);
		return ret;
	}

	lcdifv3->clk_disp_axi = devm_clk_get(dev, "disp-axi");
	if (IS_ERR(lcdifv3->clk_disp_axi))
		lcdifv3->clk_disp_axi = NULL;

	lcdifv3->clk_disp_apb = devm_clk_get(dev, "disp-apb");
	if (IS_ERR(lcdifv3->clk_disp_apb))
		lcdifv3->clk_disp_apb = NULL;

	lcdifv3->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(lcdifv3->base))
		return PTR_ERR(lcdifv3->base);

	if (of_device_is_compatible(np, "fsl,imx93-lcdif")) {
		lcdifv3->gpr = syscon_regmap_lookup_by_phandle(np, "fsl,gpr");
		if (IS_ERR(lcdifv3->gpr)) {
			ret = PTR_ERR(lcdifv3->gpr);
			dev_err(dev, "failed to get gpr: %d\n", ret);
			return ret;
		}
	}

	lcdifv3->dev = dev;

	imx_lcdifv3_of_parse_thres(lcdifv3);

	platform_set_drvdata(pdev, lcdifv3);

	atomic_set(&lcdifv3->rpm_suspended, 0);
	pm_runtime_enable(dev);
	atomic_inc(&lcdifv3->rpm_suspended);

	dev_dbg(dev, "%s: probe end\n", __func__);

	return lcdifv3_add_client_devices(lcdifv3);
}

static int imx_lcdifv3_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM
static int imx_lcdifv3_runtime_suspend(struct device *dev)
{
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(dev);

	if (atomic_inc_return(&lcdifv3->rpm_suspended) > 1)
		return 0;

	lcdifv3_disable_clocks(lcdifv3);

	release_bus_freq(BUS_FREQ_HIGH);

	/* clear LCDIF QoS and cache */
	if (of_device_is_compatible(dev->of_node, "fsl,imx93-lcdif"))
		regmap_write(lcdifv3->gpr, 0xc, 0x0);

	return 0;
}

static int imx_lcdifv3_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(dev);

	if (unlikely(!atomic_read(&lcdifv3->rpm_suspended))) {
		dev_warn(lcdifv3->dev, "Unbalanced %s!\n", __func__);
		return 0;
	}

	if (!atomic_dec_and_test(&lcdifv3->rpm_suspended))
		return 0;

	/* set LCDIF QoS and cache */
	if (of_device_is_compatible(dev->of_node, "fsl,imx93-lcdif"))
		regmap_write(lcdifv3->gpr, 0xc, 0x3712);

	request_bus_freq(BUS_FREQ_HIGH);

	ret = lcdifv3_enable_clocks(lcdifv3);
	if (ret) {
		release_bus_freq(BUS_FREQ_HIGH);
		return ret;
	}

	/* clear sw_reset */
	writel(CTRL_SW_RESET, lcdifv3->base + LCDIFV3_CTRL_CLR);

	/* enable plane FIFO panic */
	lcdifv3_enable_plane_panic(lcdifv3);

	return ret;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int imx_lcdifv3_suspend(struct device *dev)
{
	return imx_lcdifv3_runtime_suspend(dev);
}

static int imx_lcdifv3_resume(struct device *dev)
{
	return imx_lcdifv3_runtime_resume(dev);
}
#endif

static const struct dev_pm_ops imx_lcdifv3_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(imx_lcdifv3_suspend,
				     imx_lcdifv3_resume)
	SET_RUNTIME_PM_OPS(imx_lcdifv3_runtime_suspend,
			   imx_lcdifv3_runtime_resume, NULL)
};

struct platform_driver imx_lcdifv3_driver = {
	.probe    = imx_lcdifv3_probe,
	.remove   = imx_lcdifv3_remove,
	.driver   = {
		.name = DRIVER_NAME,
		.of_match_table = imx_lcdifv3_dt_ids,
		.pm = &imx_lcdifv3_pm_ops,
	},
};

module_platform_driver(imx_lcdifv3_driver);

MODULE_DESCRIPTION("NXP i.MX LCDIFV3 Display Controller driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
