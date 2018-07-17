/*
 * Copyright 2018 NXP
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

#include <linux/component.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <video/imx-lcdif.h>
#include <video/videomode.h>

#include "imx-drm.h"
#include "lcdif-plane.h"
#include "lcdif-kms.h"

struct lcdif_crtc {
	struct device *dev;

	struct drm_crtc base;
	struct lcdif_plane *plane[2];

	int vbl_irq;
	u32 pix_fmt;		/* drm fourcc */
};

#define to_lcdif_crtc(crtc) container_of(crtc, struct lcdif_crtc, base)

static void lcdif_crtc_reset(struct drm_crtc *crtc)
{
	struct imx_crtc_state *state;

	if (crtc->state) {
		__drm_atomic_helper_crtc_destroy_state(crtc->state);

		state = to_imx_crtc_state(crtc->state);
		kfree(state);
		crtc->state = NULL;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return;

	crtc->state = &state->base;
	crtc->state->crtc = crtc;
}

static struct drm_crtc_state *lcdif_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct imx_crtc_state *state, *orig_state;

	if (WARN_ON(!crtc->state))
		return NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &state->base);

	orig_state = to_imx_crtc_state(crtc->state);
	state->bus_format = orig_state->bus_format;
	state->bus_flags = orig_state->bus_flags;
	state->di_hsync_pin = orig_state->di_hsync_pin;
	state->di_vsync_pin = orig_state->di_vsync_pin;

	return &state->base;
}

static void lcdif_crtc_destroy_state(struct drm_crtc *crtc,
				     struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_imx_crtc_state(state));
}

static int lcdif_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	return 0;
}

static void lcdif_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	drm_crtc_vblank_on(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static void lcdif_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	/* LCDIF doesn't have command buffer */
	return;
}

static void lcdif_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_crtc_state)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(crtc);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	if (imx_crtc_state->bus_flags & DRM_BUS_FLAG_DE_HIGH)
		vm.flags |= DISPLAY_FLAGS_DE_HIGH;
	else
		vm.flags |= DISPLAY_FLAGS_DE_LOW;

	if (imx_crtc_state->bus_flags & DRM_BUS_FLAG_PIXDATA_POSEDGE)
		vm.flags |= DISPLAY_FLAGS_PIXDATA_POSEDGE;
	else
		vm.flags |= DISPLAY_FLAGS_PIXDATA_NEGEDGE;

	pm_runtime_get_sync(lcdif_crtc->dev->parent);

	lcdif_set_mode(lcdif, &vm);

	/* defer the lcdif controller enable to plane update,
	 * since until then the lcdif config is complete to
	 * enable the controller to run actually.
	 */
}

static void lcdif_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_crtc_state *old_crtc_state)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(crtc);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	drm_crtc_vblank_off(crtc);

	lcdif_disable_controller(lcdif);

	pm_runtime_put(lcdif_crtc->dev->parent);
}

static const struct drm_crtc_helper_funcs lcdif_helper_funcs = {
	.atomic_check	= lcdif_crtc_atomic_check,
	.atomic_begin	= lcdif_crtc_atomic_begin,
	.atomic_flush	= lcdif_crtc_atomic_flush,
	.atomic_enable	= lcdif_crtc_atomic_enable,
	.atomic_disable	= lcdif_crtc_atomic_disable,
};

static int lcdif_enable_vblank(struct drm_crtc *crtc)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(crtc);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	lcdif_vblank_irq_enable(lcdif);
	enable_irq(lcdif_crtc->vbl_irq);

	return 0;
}

static void lcdif_disable_vblank(struct drm_crtc *crtc)
{
	struct lcdif_crtc *lcdif_crtc = to_lcdif_crtc(crtc);
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	disable_irq_nosync(lcdif_crtc->vbl_irq);
	lcdif_vblank_irq_disable(lcdif);
}

static const struct drm_crtc_funcs lcdif_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy    = drm_crtc_cleanup,
	.page_flip  = drm_atomic_helper_page_flip,
	.reset      = lcdif_crtc_reset,
	.atomic_duplicate_state = lcdif_crtc_duplicate_state,
	.atomic_destroy_state	= lcdif_crtc_destroy_state,
	.enable_vblank	= lcdif_enable_vblank,
	.disable_vblank = lcdif_disable_vblank,
};

static irqreturn_t lcdif_crtc_vblank_irq_handler(int irq, void *dev_id)
{
	struct lcdif_crtc *lcdif_crtc = dev_id;
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	drm_crtc_handle_vblank(&lcdif_crtc->base);

	lcdif_vblank_irq_clear(lcdif);

	return IRQ_HANDLED;
}

static int lcdif_crtc_init(struct lcdif_crtc *lcdif_crtc,
			   struct lcdif_client_platformdata *pdata,
			   struct drm_device *drm)
{
	int ret;
	struct lcdif_plane *primary = lcdif_crtc->plane[0];
	struct lcdif_soc *lcdif = dev_get_drvdata(lcdif_crtc->dev->parent);

	/* Primary plane
	 * The 'possible_crtcs' of primary plane will be
	 * recalculated during the 'crtc' initialization
	 * later.
	 */
	primary = lcdif_plane_init(drm, lcdif, 0, DRM_PLANE_TYPE_PRIMARY, 0);
	if (IS_ERR(primary))
		return PTR_ERR(primary);
	lcdif_crtc->plane[0] = primary;

	/* TODO: Overlay plane */

	lcdif_crtc->base.port = pdata->of_node;
	drm_crtc_helper_add(&lcdif_crtc->base, &lcdif_helper_funcs);
	ret = drm_crtc_init_with_planes(drm, &lcdif_crtc->base,
			&lcdif_crtc->plane[0]->base, NULL,
			&lcdif_crtc_funcs, NULL);
	if (ret) {
		dev_err(lcdif_crtc->dev, "failed to init crtc\n");
		goto primary_plane_deinit;
	}

	lcdif_crtc->vbl_irq = lcdif_vblank_irq_get(lcdif);
	WARN_ON(lcdif_crtc->vbl_irq < 0);

	ret = devm_request_irq(lcdif_crtc->dev, lcdif_crtc->vbl_irq,
			       lcdif_crtc_vblank_irq_handler, 0,
			       dev_name(lcdif_crtc->dev), lcdif_crtc);
	if (ret) {
		dev_err(lcdif_crtc->dev,
			"vblank irq request failed: %d\n", ret);
		goto primary_plane_deinit;
	}

	disable_irq(lcdif_crtc->vbl_irq);

	return 0;

primary_plane_deinit:
	lcdif_plane_deinit(drm, primary);

	return ret;
}

static int lcdif_crtc_bind(struct device *dev, struct device *master,
			   void *data)
{
	int ret;
	struct drm_device *drm = data;
	struct lcdif_crtc *lcdif_crtc;
	struct lcdif_client_platformdata *pdata = dev->platform_data;

	dev_dbg(dev, "%s: lcdif crtc bind begin\n", __func__);

	lcdif_crtc = devm_kzalloc(dev, sizeof(*lcdif_crtc), GFP_KERNEL);
	if (!lcdif_crtc)
		return -ENOMEM;

	lcdif_crtc->dev = dev;

	ret = lcdif_crtc_init(lcdif_crtc, pdata, drm);
	if (ret)
		return ret;

	if (!drm->mode_config.funcs)
		drm->mode_config.funcs = &lcdif_drm_mode_config_funcs;

	if (!drm->mode_config.helper_private)
		drm->mode_config.helper_private = &lcdif_drm_mode_config_helpers;

	/* limit the max width and height */
	drm->mode_config.max_width  = 1920;
	drm->mode_config.max_height = 1920;

	dev_set_drvdata(dev, lcdif_crtc);

	dev_dbg(dev, "%s: lcdif crtc bind end\n", __func__);

	return 0;
}

static void lcdif_crtc_unbind(struct device *dev, struct device *master,
			      void *data)
{
	struct drm_device *drm = data;
	struct lcdif_crtc *lcdif_crtc = dev_get_drvdata(dev);

	lcdif_plane_deinit(drm, lcdif_crtc->plane[0]);
}

static const struct component_ops lcdif_crtc_ops = {
	.bind   = lcdif_crtc_bind,
	.unbind = lcdif_crtc_unbind,
};

static int lcdif_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_dbg(&pdev->dev, "%s: lcdif crtc probe begin\n", __func__);

	if (!dev->platform_data) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	return component_add(dev, &lcdif_crtc_ops);
}

static int lcdif_crtc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &lcdif_crtc_ops);

	return 0;
}

static struct platform_driver lcdif_crtc_driver = {
	.probe  = lcdif_crtc_probe,
	.remove = lcdif_crtc_remove,
	.driver = {
		.name = "imx-lcdif-crtc",
	},
};
module_platform_driver(lcdif_crtc_driver);

MODULE_DESCRIPTION("NXP i.MX LCDIF DRM CRTC driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
