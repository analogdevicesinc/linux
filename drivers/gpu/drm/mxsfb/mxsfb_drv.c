/*
 * Copyright (C) 2016 Marek Vasut <marex@denx.de>
 *
 * This code is based on drivers/video/fbdev/mxsfb.c :
 * Copyright (C) 2010 Juergen Beisert, Pengutronix
 * Copyright (C) 2008-2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/list.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_runtime.h>
#include <linux/reservation.h>
#include <linux/version.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_simple_kms_helper.h>

#include "mxsfb_drv.h"
#include "mxsfb_regs.h"

/* The eLCDIF max possible CRTCs */
#define MAX_CRTCS 1

enum mxsfb_devtype {
	MXSFB_V3,
	MXSFB_V4,
};

/*
 * When adding new formats, make sure to update the num_formats from
 * mxsfb_devdata below.
 */
static const uint32_t mxsfb_formats[] = {
	/* MXSFB_V3 */
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_RGB565,
	/* MXSFB_V4 */
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_RGBX8888,
	DRM_FORMAT_RGBA8888,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_ABGR1555,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_BGR565
};

static const struct mxsfb_devdata mxsfb_devdata[] = {
	[MXSFB_V3] = {
		.transfer_count	= LCDC_V3_TRANSFER_COUNT,
		.cur_buf	= LCDC_V3_CUR_BUF,
		.next_buf	= LCDC_V3_NEXT_BUF,
		.debug0		= LCDC_V3_DEBUG0,
		.hs_wdth_mask	= 0xff,
		.hs_wdth_shift	= 24,
		.ipversion	= 3,
		.flags		= MXSFB_FLAG_NULL,
		.num_formats	= 3,
	},
	[MXSFB_V4] = {
		.transfer_count	= LCDC_V4_TRANSFER_COUNT,
		.cur_buf	= LCDC_V4_CUR_BUF,
		.next_buf	= LCDC_V4_NEXT_BUF,
		.debug0		= LCDC_V4_DEBUG0,
		.hs_wdth_mask	= 0x3fff,
		.hs_wdth_shift	= 18,
		.ipversion	= 4,
		.flags		= MXSFB_FLAG_BUSFREQ,
		.num_formats	= ARRAY_SIZE(mxsfb_formats),
	},
};

static struct mxsfb_drm_private *
drm_pipe_to_mxsfb_drm_private(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct mxsfb_drm_private, pipe);
}

void mxsfb_enable_axi_clk(struct mxsfb_drm_private *mxsfb)
{
	if (mxsfb->clk_axi)
		clk_prepare_enable(mxsfb->clk_axi);
}

void mxsfb_disable_axi_clk(struct mxsfb_drm_private *mxsfb)
{
	if (mxsfb->clk_axi)
		clk_disable_unprepare(mxsfb->clk_axi);
}

/**
 * mxsfb_atomic_helper_check - validate state object
 * @dev: DRM device
 * @state: the driver state object
 *
 * On top of the drm imlementation drm_atomic_helper_check,
 * check if the bpp is changed, if so, signal mode_changed,
 * this will trigger disable/enable
 *
 * RETURNS:
 * Zero for success or -errno
 */
static int mxsfb_atomic_helper_check(struct drm_device *dev,
			    struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	int i, ret;

	ret = drm_atomic_helper_check(dev, state);
	if (ret)
		return ret;

	for_each_crtc_in_state(state, crtc, crtc_state, i) {
		struct drm_plane_state *primary_state;
		int old_bpp = 0;
		int new_bpp = 0;

		if (!crtc->primary || !crtc->primary->old_fb)
			continue;
		primary_state =
			drm_atomic_get_plane_state(state, crtc->primary);
		if (!primary_state || !primary_state->fb)
			continue;
		old_bpp = crtc->primary->old_fb->format->depth;
		new_bpp = primary_state->fb->format->depth;
		if (old_bpp != new_bpp) {
			crtc_state->mode_changed = true;
			DRM_DEBUG_ATOMIC(
				"[CRTC:%d:%s] mode changed, bpp %d->%d\n",
				crtc->base.id, crtc->name, old_bpp, new_bpp);
		}
	}
	return ret;
}

static const struct drm_mode_config_funcs mxsfb_mode_config_funcs = {
	.fb_create		= drm_fb_cma_create,
	.atomic_check		= mxsfb_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static void mxsfb_pipe_enable(struct drm_simple_display_pipe *pipe,
			      struct drm_crtc_state *crtc_state)
{
	struct drm_device *drm = pipe->encoder.dev;
	struct drm_connector *connector;
	struct mxsfb_drm_private *mxsfb = drm_pipe_to_mxsfb_drm_private(pipe);

	if (!mxsfb->connector) {
		list_for_each_entry(connector,
				    &drm->mode_config.connector_list,
				    head)
			if (connector->encoder == &(mxsfb->pipe.encoder)) {
				mxsfb->connector = connector;
				break;
			}
	}

	if (!mxsfb->connector) {
		dev_warn(drm->dev, "No connector attached, using default\n");
		mxsfb->connector = &mxsfb->panel_connector;
	}

	drm_crtc_vblank_on(&mxsfb->pipe.crtc);

	pm_runtime_get_sync(drm->dev);
	drm_panel_prepare(mxsfb->panel);
	mxsfb_crtc_enable(mxsfb);
	drm_panel_enable(mxsfb->panel);
}

static void mxsfb_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct mxsfb_drm_private *mxsfb = drm_pipe_to_mxsfb_drm_private(pipe);
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_device *drm = pipe->plane.dev;

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	drm_panel_disable(mxsfb->panel);
	mxsfb_crtc_disable(mxsfb);
	drm_panel_unprepare(mxsfb->panel);
	pm_runtime_put_sync(drm->dev);

	if (mxsfb->connector != &mxsfb->panel_connector)
		mxsfb->connector = NULL;
}

static void mxsfb_pipe_update(struct drm_simple_display_pipe *pipe,
			      struct drm_plane_state *plane_state)
{
	struct mxsfb_drm_private *mxsfb = drm_pipe_to_mxsfb_drm_private(pipe);

	mxsfb_plane_atomic_update(mxsfb, plane_state);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
static int mxsfb_pipe_prepare_fb(struct drm_simple_display_pipe *pipe,
				 struct drm_plane_state *plane_state)
{
	return drm_fb_cma_prepare_fb(&pipe->plane, plane_state);
}
#endif

static struct drm_simple_display_pipe_funcs mxsfb_funcs = {
	.enable		= mxsfb_pipe_enable,
	.disable	= mxsfb_pipe_disable,
	.update		= mxsfb_pipe_update,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	.prepare_fb	= mxsfb_pipe_prepare_fb,
#endif
};

static int mxsfb_load(struct drm_device *drm, unsigned long flags)
{
	struct platform_device *pdev = to_platform_device(drm->dev);
	struct mxsfb_drm_private *mxsfb;
	struct resource *res;
	u32 max_res[2] = {0, 0};
	int ret;

	mxsfb = devm_kzalloc(&pdev->dev, sizeof(*mxsfb), GFP_KERNEL);
	if (!mxsfb)
		return -ENOMEM;

	drm->dev_private = mxsfb;
	mxsfb->devdata = &mxsfb_devdata[pdev->id_entry->driver_data];
	mxsfb->dev = &pdev->dev;

	platform_set_drvdata(pdev, drm);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mxsfb->base = devm_ioremap_resource(drm->dev, res);
	if (IS_ERR(mxsfb->base))
		return PTR_ERR(mxsfb->base);

	mxsfb->clk = devm_clk_get(drm->dev, "pix");
	if (IS_ERR(mxsfb->clk))
		return PTR_ERR(mxsfb->clk);

	mxsfb->clk_axi = devm_clk_get(drm->dev, "axi");
	if (IS_ERR(mxsfb->clk_axi))
		mxsfb->clk_axi = NULL;

	mxsfb->clk_disp_axi = devm_clk_get(drm->dev, "disp_axi");
	if (IS_ERR(mxsfb->clk_disp_axi))
		mxsfb->clk_disp_axi = NULL;

	ret = dma_set_mask_and_coherent(drm->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = drm_vblank_init(drm, MAX_CRTCS);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to initialise vblank\n");
		return ret;
	}

	/* Modeset init */
	drm_mode_config_init(drm);

	ret = mxsfb_create_output(drm);
	if (ret < 0) {
		dev_err(drm->dev, "Failed to create outputs\n");
		return ret;
	}

	ret = drm_simple_display_pipe_init(drm, &mxsfb->pipe, &mxsfb_funcs,
			mxsfb_formats, mxsfb->devdata->num_formats, NULL,
			mxsfb->connector);
	if (ret < 0) {
		dev_err(drm->dev, "Cannot setup simple display pipe\n");
		return ret;
	}

	drm_crtc_vblank_off(&mxsfb->pipe.crtc);

	/*
	 * Attach panel only if there is one.
	 * If there is no panel attach, it must be a bridge. In this case, we
	 * need a reference to its connector for a proper initialization.
	 * We will do this check in pipe->enable(), since the connector won't
	 * be attached to an encoder until then.
	 */

	if (mxsfb->panel) {
		ret = drm_panel_attach(mxsfb->panel, mxsfb->connector);
		if (ret) {
			dev_err(drm->dev, "Cannot connect panel\n");
			return ret;
		}
	} else if (mxsfb->bridge) {
		ret = drm_simple_display_pipe_attach_bridge(&mxsfb->pipe,
				mxsfb->bridge);
		if (ret) {
			dev_err(drm->dev, "Cannot connect bridge\n");
			return ret;
		}
	}

	of_property_read_u32_array(drm->dev->of_node, "max-res",
				   &max_res[0], 2);
	if (!max_res[0])
		max_res[0] = MXSFB_MAX_XRES;
	if (!max_res[1])
		max_res[1] = MXSFB_MAX_YRES;

	drm->mode_config.min_width	= MXSFB_MIN_XRES;
	drm->mode_config.min_height	= MXSFB_MIN_YRES;
	drm->mode_config.max_width	= max_res[0];
	drm->mode_config.max_height	= max_res[1];
	drm->mode_config.funcs		= &mxsfb_mode_config_funcs;

	drm_mode_config_reset(drm);

	ret = drm_irq_install(drm, platform_get_irq(pdev, 0));

	if (ret < 0) {
		dev_err(drm->dev, "Failed to install IRQ handler\n");
		goto err_irq;
	}

	drm_kms_helper_poll_init(drm);

	mxsfb->fbdev = drm_fbdev_cma_init(drm, 32,
					  drm->mode_config.num_connector);
	if (IS_ERR(mxsfb->fbdev)) {
		ret = PTR_ERR(mxsfb->fbdev);
		mxsfb->fbdev = NULL;
		dev_err(drm->dev, "Failed to init FB CMA area\n");
		goto err_cma;
	}


	drm_helper_hpd_irq_event(drm);

	pm_runtime_enable(drm->dev);

	return 0;

err_cma:
	drm_irq_uninstall(drm);
err_irq:
	drm_panel_detach(mxsfb->panel);

	return ret;
}

static void mxsfb_unload(struct drm_device *drm)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;

	if (mxsfb->fbdev)
		drm_fbdev_cma_fini(mxsfb->fbdev);

	drm_kms_helper_poll_fini(drm);
	drm_mode_config_cleanup(drm);

	pm_runtime_get_sync(drm->dev);
	drm_irq_uninstall(drm);
	pm_runtime_put_sync(drm->dev);

	drm->dev_private = NULL;

	pm_runtime_disable(drm->dev);
}

static void mxsfb_lastclose(struct drm_device *drm)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;

	drm_fbdev_cma_restore_mode(mxsfb->fbdev);
}

static int mxsfb_enable_vblank(struct drm_device *drm, unsigned int crtc)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;

	/* Clear and enable VBLANK IRQ */
	mxsfb_enable_axi_clk(mxsfb);
	writel(CTRL1_CUR_FRAME_DONE_IRQ, mxsfb->base + LCDC_CTRL1 + REG_CLR);
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN, mxsfb->base + LCDC_CTRL1 + REG_SET);
	mxsfb_disable_axi_clk(mxsfb);

	return 0;
}

static void mxsfb_disable_vblank(struct drm_device *drm, unsigned int crtc)
{
	struct mxsfb_drm_private *mxsfb = drm->dev_private;

	/* Disable and clear VBLANK IRQ */
	mxsfb_enable_axi_clk(mxsfb);
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN, mxsfb->base + LCDC_CTRL1 + REG_CLR);
	writel(CTRL1_CUR_FRAME_DONE_IRQ, mxsfb->base + LCDC_CTRL1 + REG_CLR);
	mxsfb_disable_axi_clk(mxsfb);
}

static void mxsfb_irq_preinstall(struct drm_device *drm)
{
	mxsfb_disable_vblank(drm, 0);
}

static irqreturn_t mxsfb_irq_handler(int irq, void *data)
{
	struct drm_device *drm = data;
	struct mxsfb_drm_private *mxsfb = drm->dev_private;
	u32 reg;

	mxsfb_enable_axi_clk(mxsfb);

	reg = readl(mxsfb->base + LCDC_CTRL1);

	if (reg & CTRL1_CUR_FRAME_DONE_IRQ)
		drm_crtc_handle_vblank(&mxsfb->pipe.crtc);

	writel(CTRL1_CUR_FRAME_DONE_IRQ, mxsfb->base + LCDC_CTRL1 + REG_CLR);

	mxsfb_disable_axi_clk(mxsfb);

	return IRQ_HANDLED;
}

DEFINE_DRM_GEM_CMA_FOPS(fops);

static struct drm_driver mxsfb_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET |
				  DRIVER_PRIME | DRIVER_ATOMIC |
				  DRIVER_HAVE_IRQ,
	.lastclose		= mxsfb_lastclose,
	.irq_handler		= mxsfb_irq_handler,
	.irq_preinstall		= mxsfb_irq_preinstall,
	.irq_uninstall		= mxsfb_irq_preinstall,
	.enable_vblank		= mxsfb_enable_vblank,
	.disable_vblank		= mxsfb_disable_vblank,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.dumb_create		= drm_gem_cma_dumb_create,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,
	.fops	= &fops,
	.name	= "mxsfb-drm",
	.desc	= "MXSFB Controller DRM",
	.date	= "20160824",
	.major	= 1,
	.minor	= 0,
};

static const struct platform_device_id mxsfb_devtype[] = {
	{ .name = "imx23-fb", .driver_data = MXSFB_V3, },
	{ .name = "imx28-fb", .driver_data = MXSFB_V4, },
	{ .name = "imx6sx-fb", .driver_data = MXSFB_V4, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, mxsfb_devtype);

static const struct of_device_id mxsfb_dt_ids[] = {
	{ .compatible = "fsl,imx23-lcdif", .data = &mxsfb_devtype[0], },
	{ .compatible = "fsl,imx28-lcdif", .data = &mxsfb_devtype[1], },
	{ .compatible = "fsl,imx6sx-lcdif", .data = &mxsfb_devtype[2], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxsfb_dt_ids);

static int mxsfb_probe(struct platform_device *pdev)
{
	struct drm_device *drm;
	const struct of_device_id *of_id =
			of_match_device(mxsfb_dt_ids, &pdev->dev);
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	if (of_id)
		pdev->id_entry = of_id->data;

	drm = drm_dev_alloc(&mxsfb_driver, &pdev->dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);

	ret = mxsfb_load(drm, 0);
	if (ret)
		goto err_free;

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_unload;

	return 0;

err_unload:
	mxsfb_unload(drm);
err_free:
	drm_dev_unref(drm);

	return ret;
}

static int mxsfb_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unregister(drm);
	mxsfb_unload(drm);
	drm_dev_unref(drm);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxsfb_suspend(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(drm);
}

static int mxsfb_resume(struct device *dev)
{
	struct drm_device *drm = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(drm);
}
#endif

static const struct dev_pm_ops mxsfb_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mxsfb_suspend, mxsfb_resume)
};

static struct platform_driver mxsfb_platform_driver = {
	.probe		= mxsfb_probe,
	.remove		= mxsfb_remove,
	.id_table	= mxsfb_devtype,
	.driver	= {
		.name		= "mxsfb_drm",
		.of_match_table	= mxsfb_dt_ids,
		.pm		= &mxsfb_pm_ops,
	},
};

module_platform_driver(mxsfb_platform_driver);

MODULE_AUTHOR("Marek Vasut <marex@denx.de>");
MODULE_DESCRIPTION("Freescale MXS DRM/KMS driver");
MODULE_LICENSE("GPL");
