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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/component.h>
#include <linux/pm_runtime.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_atomic_helper.h>
#include <video/imx-dcss.h>

#include "dcss-kms.h"
#include "dcss-plane.h"
#include "imx-drm.h"
#include "dcss-crtc.h"

struct dcss_crtc {
	struct device *dev;
	struct drm_crtc		base;

	struct dcss_plane	*plane[3];

	int			irq;

	struct drm_property *alpha;
	struct drm_property *use_global;
	struct drm_property *dtrc_table_ofs;

	struct completion disable_completion;

	enum dcss_hdr10_nonlinearity opipe_nl;
	enum dcss_hdr10_gamut opipe_g;
	enum dcss_hdr10_pixel_range opipe_pr;
	u32 opipe_pix_format;
};

static void dcss_crtc_reset(struct drm_crtc *crtc)
{
	struct imx_crtc_state *state;

	if (crtc->state) {
		if (crtc->state->mode_blob)
			drm_property_unreference_blob(crtc->state->mode_blob);

		state = to_imx_crtc_state(crtc->state);
		memset(state, 0, sizeof(*state));
	} else {
		state = kzalloc(sizeof(*state), GFP_KERNEL);
		if (!state)
			return;
		crtc->state = &state->base;
	}

	state->base.crtc = crtc;
}

static struct drm_crtc_state *dcss_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct imx_crtc_state *state;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &state->base);

	WARN_ON(state->base.crtc != crtc);
	state->base.crtc = crtc;

	return &state->base;
}

static void dcss_crtc_destroy_state(struct drm_crtc *crtc,
				    struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_imx_crtc_state(state));
}

static int dcss_enable_vblank(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_soc *dcss = dev_get_drvdata(dcss_crtc->dev->parent);

	dcss_vblank_irq_enable(dcss, true);

	enable_irq(dcss_crtc->irq);

	return 0;
}

static void dcss_disable_vblank(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_soc *dcss = dev_get_drvdata(dcss_crtc->dev->parent);

	disable_irq_nosync(dcss_crtc->irq);

	dcss_vblank_irq_enable(dcss, false);
}

static const struct drm_crtc_funcs dcss_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = drm_crtc_cleanup,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = dcss_crtc_reset,
	.atomic_duplicate_state = dcss_crtc_duplicate_state,
	.atomic_destroy_state = dcss_crtc_destroy_state,
	.enable_vblank = dcss_enable_vblank,
	.disable_vblank = dcss_disable_vblank,
};

static int dcss_crtc_atomic_check(struct drm_crtc *crtc,
				  struct drm_crtc_state *state)
{
	/* TODO: other checks? */

	return 0;
}

static void dcss_crtc_atomic_begin(struct drm_crtc *crtc,
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

static void dcss_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_soc *dcss = dev_get_drvdata(dcss_crtc->dev->parent);

	if (dcss_dtg_is_enabled(dcss))
		dcss_ctxld_enable(dcss);
}

void dcss_crtc_setup_opipe(struct drm_crtc *crtc, struct drm_connector *conn,
			   u32 colorimetry, u32 eotf,
			   enum hdmi_quantization_range qr)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct drm_display_info *di = &conn->display_info;
	int vic;

	if ((colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_BT2020)) ||
	    (colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_BT2020_CONST_LUM)))
		dcss_crtc->opipe_g = G_REC2020;
	else if (colorimetry & BIT(HDMI_EXTENDED_COLORIMETRY_ADOBE_RGB))
		dcss_crtc->opipe_g = G_ADOBE_ARGB;
	else
		dcss_crtc->opipe_g = G_REC709;

	if (eotf & (1 << 2))
		dcss_crtc->opipe_nl = NL_REC2084;
	else
		dcss_crtc->opipe_nl = NL_REC709;

	if (qr == HDMI_QUANTIZATION_RANGE_FULL)
		dcss_crtc->opipe_pr = PR_FULL;
	else
		dcss_crtc->opipe_pr = PR_LIMITED;

	vic = drm_match_cea_mode(&crtc->state->adjusted_mode);

	/* FIXME: we should get the connector colorspace some other way */
	if (vic == 97 &&
	    (di->color_formats & DRM_COLOR_FORMAT_YCRCB420) &&
	    (di->bpc >= 10))
		dcss_crtc->opipe_pix_format = DRM_FORMAT_P010;
	else
		dcss_crtc->opipe_pix_format = DRM_FORMAT_ARGB8888;

	DRM_INFO("OPIPE_CFG: gamut = %d, nl = %d, pr = %d, pix_format = %d\n",
		 dcss_crtc->opipe_g, dcss_crtc->opipe_nl,
		 dcss_crtc->opipe_pr, dcss_crtc->opipe_pix_format);
}

int dcss_crtc_get_opipe_cfg(struct drm_crtc *crtc,
			    struct dcss_hdr10_pipe_cfg *opipe_cfg)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);

	opipe_cfg->pixel_format = dcss_crtc->opipe_pix_format;
	opipe_cfg->g = dcss_crtc->opipe_g;
	opipe_cfg->nl = dcss_crtc->opipe_nl;
	opipe_cfg->pr = dcss_crtc->opipe_pr;

	return 0;
}

static void dcss_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_soc *dcss = dev_get_drvdata(dcss_crtc->dev->parent);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	pm_runtime_get_sync(dcss_crtc->dev->parent);

	dcss_dtg_sync_set(dcss, &vm);

	dcss_ss_subsam_set(dcss, dcss_crtc->opipe_pix_format);
	dcss_ss_sync_set(dcss, &vm, mode->flags & DRM_MODE_FLAG_PHSYNC,
			 mode->flags & DRM_MODE_FLAG_PVSYNC);

	dcss_dtg_css_set(dcss, dcss_crtc->opipe_pix_format);

	dcss_ss_enable(dcss, true);
	dcss_dtg_enable(dcss, true, NULL);
	dcss_ctxld_enable(dcss);

	crtc->enabled = true;
}

static void dcss_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_crtc_state)
{
	struct dcss_crtc *dcss_crtc = container_of(crtc, struct dcss_crtc,
						   base);
	struct dcss_soc *dcss = dev_get_drvdata(dcss_crtc->dev->parent);

	drm_atomic_helper_disable_planes_on_crtc(old_crtc_state, false);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	drm_crtc_vblank_off(crtc);

	dcss_ss_enable(dcss, false);
	dcss_dtg_enable(dcss, false, &dcss_crtc->disable_completion);
	dcss_ctxld_enable(dcss);

	crtc->enabled = false;

	wait_for_completion_timeout(&dcss_crtc->disable_completion,
				    msecs_to_jiffies(100));

	pm_runtime_put_sync(dcss_crtc->dev->parent);
}

static const struct drm_crtc_helper_funcs dcss_helper_funcs = {
	.atomic_check = dcss_crtc_atomic_check,
	.atomic_begin = dcss_crtc_atomic_begin,
	.atomic_flush = dcss_crtc_atomic_flush,
	.atomic_enable = dcss_crtc_atomic_enable,
	.atomic_disable = dcss_crtc_atomic_disable,
};

static irqreturn_t dcss_crtc_irq_handler(int irq, void *dev_id)
{
	struct dcss_crtc *dcss_crtc = dev_id;
	struct dcss_soc *dcss = dev_get_drvdata(dcss_crtc->dev->parent);

	drm_crtc_handle_vblank(&dcss_crtc->base);

	dcss_vblank_irq_clear(dcss);

	return IRQ_HANDLED;
}

static int dcss_crtc_init(struct dcss_crtc *crtc,
			  struct dcss_client_platformdata *pdata,
			  struct drm_device *drm)
{
	struct dcss_soc *dcss = dev_get_drvdata(crtc->dev->parent);
	int ret;

	crtc->plane[0] = dcss_plane_init(drm, dcss, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_PRIMARY, 2);
	if (IS_ERR(crtc->plane[0]))
		return PTR_ERR(crtc->plane[0]);

	drm_crtc_helper_add(&crtc->base, &dcss_helper_funcs);
	ret = drm_crtc_init_with_planes(drm, &crtc->base, &crtc->plane[0]->base,
					NULL, &dcss_crtc_funcs, NULL);
	if (ret) {
		dev_err(crtc->dev, "failed to init crtc\n");
		return ret;
	}

	crtc->plane[1] = dcss_plane_init(drm, dcss, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_OVERLAY, 1);
	if (IS_ERR(crtc->plane[1]))
		crtc->plane[1] = NULL;

	crtc->plane[2] = dcss_plane_init(drm, dcss, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_OVERLAY, 0);
	if (IS_ERR(crtc->plane[2]))
		crtc->plane[2] = NULL;

	crtc->alpha = drm_property_create_range(drm, 0, "alpha", 0, 255);
	if (!crtc->alpha) {
		dev_err(crtc->dev, "cannot create alpha property\n");
		return -ENOMEM;
	}

	crtc->use_global = drm_property_create_range(drm, 0,
						     "use_global_alpha", 0, 1);
	if (!crtc->use_global) {
		dev_err(crtc->dev, "cannot create use_global property\n");
		return -ENOMEM;
	}

	crtc->dtrc_table_ofs = drm_property_create_range(drm, 0,
							 "dtrc_table_ofs", 0,
							 ULLONG_MAX);
	if (!crtc->dtrc_table_ofs) {
		dev_err(crtc->dev, "cannot create dtrc_table_ofs property\n");
		return -ENOMEM;
	}

	/* attach alpha property to channel 0 */
	drm_object_attach_property(&crtc->plane[0]->base.base,
				   crtc->alpha, 255);
	crtc->plane[0]->alpha_prop = crtc->alpha;

	drm_object_attach_property(&crtc->plane[0]->base.base,
				   crtc->use_global, 0);
	crtc->plane[0]->use_global_prop = crtc->use_global;

	/* attach DTRC table offsets property to overlay planes */
	drm_object_attach_property(&crtc->plane[1]->base.base,
				   crtc->dtrc_table_ofs, 0);
	crtc->plane[1]->dtrc_table_ofs_prop = crtc->dtrc_table_ofs;

	drm_object_attach_property(&crtc->plane[2]->base.base,
				   crtc->dtrc_table_ofs, 0);
	crtc->plane[2]->dtrc_table_ofs_prop = crtc->dtrc_table_ofs;

	crtc->irq = dcss_vblank_irq_get(dcss);
	if (crtc->irq < 0) {
		dev_err(crtc->dev, "unable to get vblank interrupt\n");
		return crtc->irq;
	}

	init_completion(&crtc->disable_completion);

	ret = devm_request_irq(crtc->dev, crtc->irq, dcss_crtc_irq_handler,
			       IRQF_TRIGGER_RISING, "dcss_drm", crtc);
	if (ret) {
		dev_err(crtc->dev, "irq request failed with %d.\n", ret);
		return ret;
	}

	disable_irq(crtc->irq);

	return 0;
}

static int dcss_crtc_bind(struct device *dev, struct device *master,
			  void *data)
{
	struct dcss_client_platformdata *pdata = dev->platform_data;
	struct drm_device *drm = data;
	struct dcss_crtc *crtc;
	int ret;

	crtc = devm_kzalloc(dev, sizeof(*crtc), GFP_KERNEL);
	if (!crtc)
		return -ENOMEM;

	crtc->dev = dev;

	ret = dcss_crtc_init(crtc, pdata, drm);
	if (ret)
		return ret;

	if (!drm->mode_config.funcs)
		drm->mode_config.funcs = &dcss_drm_mode_config_funcs;

	if (!drm->mode_config.helper_private)
		drm->mode_config.helper_private = &dcss_drm_mode_config_helpers;

	dev_set_drvdata(dev, crtc);

	return 0;
}

static void dcss_crtc_unbind(struct device *dev, struct device *master,
			     void *data)
{
}

static const struct component_ops dcss_crtc_ops = {
	.bind = dcss_crtc_bind,
	.unbind = dcss_crtc_unbind,
};

static int dcss_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!dev->platform_data) {
		dev_err(dev, "no platform data\n");
		return -EINVAL;
	}

	return component_add(dev, &dcss_crtc_ops);
}

static int dcss_crtc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dcss_crtc_ops);
	return 0;
}

static struct platform_driver dcss_crtc_driver = {
	.driver = {
		.name = "imx-dcss-crtc",
	},
	.probe = dcss_crtc_probe,
	.remove = dcss_crtc_remove,
};
module_platform_driver(dcss_crtc_driver);

MODULE_AUTHOR("Laurentiu Palcu <laurentiu.palcu@nxp.com>");
MODULE_DESCRIPTION("i.MX DCSS CRTC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-dcss-crtc");
