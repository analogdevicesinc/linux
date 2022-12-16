// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2019,2020,2022 NXP
 */

#include <linux/component.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_vblank.h>
#include <video/imx-lcdifv3.h>
#include <video/videomode.h>

#include "imx-drm.h"
#include "lcdifv3-plane.h"
#include "lcdifv3-kms.h"

struct lcdifv3_crtc {
	struct device *dev;

	struct drm_crtc base;
	struct lcdifv3_plane *plane[2];

	int vbl_irq;
	u32 pix_fmt;		/* drm fourcc */
};

#define to_lcdifv3_crtc(crtc) container_of(crtc, struct lcdifv3_crtc, base)

static void lcdifv3_crtc_reset(struct drm_crtc *crtc)
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

static struct drm_crtc_state *lcdifv3_crtc_duplicate_state(struct drm_crtc *crtc)
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

static void lcdifv3_crtc_destroy_state(struct drm_crtc *crtc,
				     struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_imx_crtc_state(state));
}

static int lcdifv3_crtc_atomic_check(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);

	/* Don't check 'bus_format' when CRTC is
	 * going to be disabled.
	 */
	if (!crtc_state->enable)
		return 0;

	/* For the commit that the CRTC is active
	 * without planes attached to it should be
	 * invalid.
	 */
	if (crtc_state->active && !crtc_state->plane_mask)
		return -EINVAL;

	/* check the requested bus format can be
	 * supported by LCDIF CTRC or not
	 */
	switch (imx_crtc_state->bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
	case MEDIA_BUS_FMT_RGB666_1X18:
	case MEDIA_BUS_FMT_RGB888_1X24:
		break;
	default:
		dev_err(lcdifv3_crtc->dev,
			"unsupported bus format: %#x\n",
			imx_crtc_state->bus_format);
		return -EINVAL;
	}

	return 0;
}

static void lcdifv3_crtc_atomic_begin(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
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

static void lcdifv3_crtc_atomic_flush(struct drm_crtc *crtc,
				      struct drm_atomic_state *state)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	/* kick shadow load for plane config */
	lcdifv3_en_shadow_load(lcdifv3);
}

static void lcdifv3_crtc_atomic_enable(struct drm_crtc *crtc,
				       struct drm_atomic_state *state)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, crtc->primary);
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	if (imx_crtc_state->bus_flags & DRM_BUS_FLAG_DE_HIGH)
		vm.flags |= DISPLAY_FLAGS_DE_HIGH;
	else
		vm.flags |= DISPLAY_FLAGS_DE_LOW;

	if (imx_crtc_state->bus_flags & DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE)
		vm.flags |= DISPLAY_FLAGS_PIXDATA_POSEDGE;
	else
		vm.flags |= DISPLAY_FLAGS_PIXDATA_NEGEDGE;

	pm_runtime_get_sync(lcdifv3_crtc->dev->parent);

	lcdifv3_set_mode(lcdifv3, &vm);

	/* config LCDIF output bus format */
	lcdifv3_set_bus_fmt(lcdifv3, imx_crtc_state->bus_format);

	/* update primary plane to avoid an initial corrupt frame */
	lcdifv3_set_pitch(lcdifv3, plane_state->fb->pitches[0]);
	lcdifv3_plane_atomic_update(crtc->primary, state);
	lcdifv3_en_shadow_load(lcdifv3);

	/* run LCDIFv3 */
	lcdifv3_enable_controller(lcdifv3);
}

static void lcdifv3_crtc_atomic_disable(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	drm_crtc_vblank_off(crtc);

	lcdifv3_disable_controller(lcdifv3);

	pm_runtime_put(lcdifv3_crtc->dev->parent);
}

static enum drm_mode_status lcdifv3_crtc_mode_valid(struct drm_crtc * crtc,
						    const struct drm_display_mode *mode)
{
	u8 vic;
	long rounded_rate;
	unsigned long pclk_rate;
	struct drm_display_mode *dmt, copy;
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	/* check CEA-861 mode */
	vic = drm_match_cea_mode(mode);
	if (vic)
		goto check_pix_clk;

	/* check DMT mode */
	dmt = drm_mode_find_dmt(crtc->dev, mode->hdisplay, mode->vdisplay,
				drm_mode_vrefresh(mode), false);
	if (dmt) {
		drm_mode_copy(&copy, dmt);
		drm_mode_destroy(crtc->dev, dmt);

		if (drm_mode_equal(mode, &copy))
			goto check_pix_clk;
	}

	return MODE_OK;

check_pix_clk:
	pclk_rate = mode->clock * 1000;

	rounded_rate = lcdifv3_pix_clk_round_rate(lcdifv3, pclk_rate);

	if (rounded_rate <= 0)
		return MODE_BAD;

	/* allow +/-0.5% HDMI pixel clock rate shift */
	if (rounded_rate < pclk_rate * 995 / 1000 ||
	    rounded_rate > pclk_rate * 1005 / 1000)
		return MODE_BAD;

	return MODE_OK;
}

static const struct drm_crtc_helper_funcs lcdifv3_helper_funcs = {
	.atomic_check	= lcdifv3_crtc_atomic_check,
	.atomic_begin	= lcdifv3_crtc_atomic_begin,
	.atomic_flush	= lcdifv3_crtc_atomic_flush,
	.atomic_enable	= lcdifv3_crtc_atomic_enable,
	.atomic_disable	= lcdifv3_crtc_atomic_disable,
	.mode_valid	= lcdifv3_crtc_mode_valid,
};

static int lcdifv3_enable_vblank(struct drm_crtc *crtc)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	lcdifv3_vblank_irq_enable(lcdifv3);
	enable_irq(lcdifv3_crtc->vbl_irq);

	return 0;
}

static void lcdifv3_disable_vblank(struct drm_crtc *crtc)
{
	struct lcdifv3_crtc *lcdifv3_crtc = to_lcdifv3_crtc(crtc);
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	disable_irq_nosync(lcdifv3_crtc->vbl_irq);
	lcdifv3_vblank_irq_disable(lcdifv3);
}

static const struct drm_crtc_funcs lcdifv3_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy    = drm_crtc_cleanup,
	.page_flip  = drm_atomic_helper_page_flip,
	.reset      = lcdifv3_crtc_reset,
	.atomic_duplicate_state = lcdifv3_crtc_duplicate_state,
	.atomic_destroy_state	= lcdifv3_crtc_destroy_state,
	.enable_vblank	= lcdifv3_enable_vblank,
	.disable_vblank = lcdifv3_disable_vblank,
};

static irqreturn_t lcdifv3_crtc_vblank_irq_handler(int irq, void *dev_id)
{
	struct lcdifv3_crtc *lcdifv3_crtc = dev_id;
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	drm_crtc_handle_vblank(&lcdifv3_crtc->base);

	lcdifv3_vblank_irq_clear(lcdifv3);

	return IRQ_HANDLED;
}

static int lcdifv3_crtc_init(struct lcdifv3_crtc *lcdifv3_crtc,
			     struct lcdifv3_client_platformdata *pdata,
			     struct drm_device *drm)
{
	int ret;
	struct lcdifv3_plane *primary = lcdifv3_crtc->plane[0];
	struct lcdifv3_soc *lcdifv3 = dev_get_drvdata(lcdifv3_crtc->dev->parent);

	/* Primary plane
	 * The 'possible_crtcs' of primary plane will be
	 * recalculated during the 'crtc' initialization
	 * later.
	 */
	primary = lcdifv3_plane_init(drm, lcdifv3, 0, DRM_PLANE_TYPE_PRIMARY, 0);
	if (IS_ERR(primary))
		return PTR_ERR(primary);
	lcdifv3_crtc->plane[0] = primary;

	/* TODO: Overlay plane */

	lcdifv3_crtc->base.port = pdata->of_node;
	drm_crtc_helper_add(&lcdifv3_crtc->base, &lcdifv3_helper_funcs);
	ret = drm_crtc_init_with_planes(drm, &lcdifv3_crtc->base,
			&lcdifv3_crtc->plane[0]->base, NULL,
			&lcdifv3_crtc_funcs, NULL);
	if (ret) {
		dev_err(lcdifv3_crtc->dev, "failed to init crtc\n");
		return ret;
	}

	lcdifv3_crtc->vbl_irq = lcdifv3_vblank_irq_get(lcdifv3);
	WARN_ON(lcdifv3_crtc->vbl_irq < 0);

	ret = devm_request_irq(lcdifv3_crtc->dev, lcdifv3_crtc->vbl_irq,
			       lcdifv3_crtc_vblank_irq_handler, 0,
			       dev_name(lcdifv3_crtc->dev), lcdifv3_crtc);
	if (ret) {
		dev_err(lcdifv3_crtc->dev,
			"vblank irq request failed: %d\n", ret);
		return ret;
	}

	disable_irq(lcdifv3_crtc->vbl_irq);

	return 0;
}

static int lcdifv3_crtc_bind(struct device *dev, struct device *master,
			   void *data)
{
	int ret;
	struct drm_device *drm = data;
	struct lcdifv3_crtc *lcdifv3_crtc = dev_get_drvdata(dev);
	struct lcdifv3_client_platformdata *pdata = dev->platform_data;

	dev_dbg(dev, "%s: lcdifv3 crtc bind begin\n", __func__);

	ret = lcdifv3_crtc_init(lcdifv3_crtc, pdata, drm);
	if (ret)
		return ret;

	if (!drm->mode_config.funcs)
		drm->mode_config.funcs = &lcdifv3_drm_mode_config_funcs;

	if (!drm->mode_config.helper_private)
		drm->mode_config.helper_private = &lcdifv3_drm_mode_config_helpers;

	/* limit the max width and height */
	drm->mode_config.max_width  = 4096;
	drm->mode_config.max_height = 4096;

	dev_dbg(dev, "%s: lcdifv3 crtc bind end\n", __func__);

	return 0;
}

static void lcdifv3_crtc_unbind(struct device *dev, struct device *master,
			      void *data)
{
	/* No special to be done */
}

static const struct component_ops lcdifv3_crtc_ops = {
	.bind   = lcdifv3_crtc_bind,
	.unbind = lcdifv3_crtc_unbind,
};

static int lcdifv3_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lcdifv3_crtc *lcdifv3_crtc;

	dev_dbg(&pdev->dev, "%s: lcdifv3 crtc probe begin\n", __func__);

	if (!dev->platform_data) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	lcdifv3_crtc = devm_kzalloc(dev, sizeof(*lcdifv3_crtc), GFP_KERNEL);
	if (!lcdifv3_crtc)
		return -ENOMEM;

	lcdifv3_crtc->dev = dev;
	dev_set_drvdata(dev, lcdifv3_crtc);

	return component_add(dev, &lcdifv3_crtc_ops);
}

static int lcdifv3_crtc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &lcdifv3_crtc_ops);

	return 0;
}

static struct platform_driver lcdifv3_crtc_driver = {
	.probe  = lcdifv3_crtc_probe,
	.remove = lcdifv3_crtc_remove,
	.driver = {
		.name = "imx-lcdifv3-crtc",
	},
};
module_platform_driver(lcdifv3_crtc_driver);

MODULE_DESCRIPTION("NXP i.MX LCDIFV3 DRM CRTC driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
