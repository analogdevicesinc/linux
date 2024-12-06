// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <drm/bridge/cdns-mhdp.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_vblank.h>

#include <linux/hdmi.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "dcss-dev.h"
#include "dcss-kms.h"

static void dcss_drm_crtc_reset(struct drm_crtc *crtc)
{
	struct dcss_crtc_state *state;

	if (crtc->state) {
		__drm_atomic_helper_crtc_destroy_state(crtc->state);

		state = to_dcss_crtc_state(crtc->state);
		kfree(state);
		crtc->state = NULL;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state) {
		crtc->state = &state->base;
		crtc->state->crtc = crtc;
	}
}

static struct drm_crtc_state *
dcss_drm_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct dcss_crtc_state *state, *copy;

	if (WARN_ON(!crtc->state))
		return NULL;

	copy = kzalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &copy->base);
	state = to_dcss_crtc_state(crtc->state);
	copy->output_encoding = state->output_encoding;
	copy->opipe_nl = state->opipe_nl;
	copy->opipe_g = state->opipe_g;
	copy->opipe_pr = state->opipe_pr;

	return &copy->base;
}

static void dcss_drm_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					       struct drm_crtc_state *state)
{
	struct dcss_crtc_state *dcss_crtc_state;

	if (state) {
		__drm_atomic_helper_crtc_destroy_state(state);
		dcss_crtc_state = to_dcss_crtc_state(state);
		kfree(dcss_crtc_state);
	}
}

static int dcss_enable_vblank(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = crtc->dev->dev_private;

	dcss_dtg_vblank_irq_enable(dcss->dtg, true);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, true);

	enable_irq(dcss_crtc->irq);

	return 0;
}

static void dcss_disable_vblank(struct drm_crtc *crtc)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	disable_irq_nosync(dcss_crtc->irq);

	dcss_dtg_vblank_irq_enable(dcss->dtg, false);

	if (!dcss_dtrc_is_running(dcss->dtrc) &&
	    dcss_crtc->disable_ctxld_kick_irq)
		dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, false);
}

static const struct drm_crtc_funcs dcss_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = drm_crtc_cleanup,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = dcss_drm_crtc_reset,
	.atomic_duplicate_state = dcss_drm_crtc_atomic_duplicate_state,
	.atomic_destroy_state = dcss_drm_crtc_atomic_destroy_state,
	.enable_vblank = dcss_enable_vblank,
	.disable_vblank = dcss_disable_vblank,
};

static void dcss_crtc_atomic_begin(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	drm_crtc_vblank_on(crtc);
}

static void dcss_crtc_atomic_flush(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		drm_crtc_arm_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	if (dcss_dtg_is_enabled(dcss->dtg))
		dcss_ctxld_enable(dcss->ctxld);
}

static void dcss_crtc_atomic_enable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state,
									      crtc);
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;
	struct dcss_crtc_state *dcss_crtc_state =
						to_dcss_crtc_state(crtc->state);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct drm_display_mode *old_mode = &old_crtc_state->adjusted_mode;
	struct videomode vm;

	drm_display_mode_to_videomode(mode, &vm);

	pm_runtime_get_sync(dcss->dev);

	vm.pixelclock = mode->crtc_clock * 1000;

	dcss_ss_subsam_set(dcss->ss, dcss_crtc_state->output_encoding);
	dcss_dtg_css_set(dcss->dtg, dcss_crtc_state->output_encoding);

	if (!drm_mode_equal(mode, old_mode) || !old_crtc_state->active) {
		dcss_dtg_sync_set(dcss->dtg, &vm);
		dcss_ss_sync_set(dcss->ss, &vm,
				 mode->flags & DRM_MODE_FLAG_PHSYNC,
				 mode->flags & DRM_MODE_FLAG_PVSYNC);
	}

	dcss_enable_dtg_and_ss(dcss);

	dcss_ctxld_enable(dcss->ctxld);

	/* Allow CTXLD kick interrupt to be disabled when VBLANK is disabled. */
	dcss_crtc->disable_ctxld_kick_irq = true;
}

static void dcss_crtc_atomic_disable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_crtc_state = drm_atomic_get_old_crtc_state(state,
									      crtc);
	struct dcss_crtc *dcss_crtc = to_dcss_crtc(crtc);
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct drm_display_mode *old_mode = &old_crtc_state->adjusted_mode;

	drm_atomic_helper_disable_planes_on_crtc(old_crtc_state, false);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	dcss_dtg_ctxld_kick_irq_enable(dcss->dtg, true);

	reinit_completion(&dcss->disable_completion);

	dcss_disable_dtg_and_ss(dcss);

	dcss_ctxld_enable(dcss->ctxld);

	if (!drm_mode_equal(mode, old_mode) || !crtc->state->active)
		if (!wait_for_completion_timeout(&dcss->disable_completion,
						 msecs_to_jiffies(100)))
			dev_err(dcss->dev, "Shutting off DTG timed out.\n");

	/*
	 * Do not shut off CTXLD kick interrupt when shutting VBLANK off. It
	 * will be needed to commit the last changes, before going to suspend.
	 */
	dcss_crtc->disable_ctxld_kick_irq = false;

	drm_crtc_vblank_off(crtc);

	pm_runtime_mark_last_busy(dcss->dev);
	pm_runtime_put_autosuspend(dcss->dev);
}

static enum drm_mode_status dcss_crtc_mode_valid(struct drm_crtc *crtc,
						 const struct drm_display_mode *mode)
{
	/*
	 * From DCSS perspective, dissallow any mode higher than
	 * 3840x2160 or 2160x3840.
	 */
	if (mode->hdisplay * mode->vdisplay > 3840 * 2160)
		return MODE_BAD;

	return MODE_OK;
}

static const struct drm_crtc_helper_funcs dcss_helper_funcs = {
	.atomic_begin = dcss_crtc_atomic_begin,
	.atomic_flush = dcss_crtc_atomic_flush,
	.atomic_enable = dcss_crtc_atomic_enable,
	.atomic_disable = dcss_crtc_atomic_disable,
	.mode_valid = dcss_crtc_mode_valid,
};

static irqreturn_t dcss_crtc_irq_handler(int irq, void *dev_id)
{
	struct dcss_crtc *dcss_crtc = dev_id;
	struct dcss_dev *dcss = dcss_crtc->base.dev->dev_private;

	if (!dcss_dtg_vblank_irq_valid(dcss->dtg))
		return IRQ_NONE;

	if (dcss_ctxld_is_flushed(dcss->ctxld))
		drm_crtc_handle_vblank(&dcss_crtc->base);

	dcss_dtg_vblank_irq_clear(dcss->dtg);

	return IRQ_HANDLED;
}

static void __dcss_crtc_setup_opipe_gamut(u32 colorspace,
					  const struct drm_display_mode *mode,
					  enum dcss_hdr10_gamut *g,
					  enum dcss_hdr10_nonlinearity *nl)
{
	u8 vic;

	switch (colorspace) {
	case DRM_MODE_COLORIMETRY_BT709_YCC:
	case DRM_MODE_COLORIMETRY_XVYCC_709:
		*g = G_REC709;
		*nl = NL_REC709;
		return;
	case DRM_MODE_COLORIMETRY_SMPTE_170M_YCC:
	case DRM_MODE_COLORIMETRY_XVYCC_601:
	case DRM_MODE_COLORIMETRY_SYCC_601:
	case DRM_MODE_COLORIMETRY_OPYCC_601:
		*g = G_REC601_NTSC;
		*nl = NL_REC709;
		return;
	case DRM_MODE_COLORIMETRY_BT2020_CYCC:
	case DRM_MODE_COLORIMETRY_BT2020_RGB:
	case DRM_MODE_COLORIMETRY_BT2020_YCC:
		*g = G_REC2020;
		*nl = NL_REC2084;
		return;
	case DRM_MODE_COLORIMETRY_OPRGB:
		*g = G_REC709;
		*nl = NL_SRGB;
		return;
	default:
		break;
	}

	/*
	 * If we reached this point, it means the default colorimetry is used.
	 */

	/* non-CEA mode, sRGB is used */
	vic = drm_match_cea_mode(mode);
	if (vic == 0) {
		*g = G_REC709;
		*nl = NL_SRGB;
		return;
	}

	/* use REC709 otherwise, by default */
	*g = G_REC709;
	*nl = NL_REC709;
}

static void __dcss_crtc_setup_opipe(struct drm_crtc_state *crtc_state,
				    struct drm_connector_state *conn_state)
{
	struct dcss_crtc_state *dcss_crtc_state = to_dcss_crtc_state(crtc_state);
	struct dcss_dev *dcss = crtc_state->crtc->dev->dev_private;
	enum hdmi_quantization_range qr;

	qr = drm_default_rgb_quant_range(&crtc_state->adjusted_mode);

	__dcss_crtc_setup_opipe_gamut(conn_state->colorspace,
				      &crtc_state->adjusted_mode,
				      &dcss_crtc_state->opipe_g,
				      &dcss_crtc_state->opipe_nl);

	dcss_crtc_state->opipe_pr = qr == HDMI_QUANTIZATION_RANGE_FULL ?
							PR_FULL : PR_LIMITED;

	dcss_crtc_state->output_encoding = DCSS_PIPE_OUTPUT_RGB;

	if (dcss->hdmi_output) {
		struct cdns_mhdp_device *mhdp_dev =
					container_of(conn_state->connector,
						     struct cdns_mhdp_device,
						     connector.base);

		switch (mhdp_dev->video_info.color_fmt) {
		case YCBCR_4_2_2:
			dcss_crtc_state->output_encoding =
							DCSS_PIPE_OUTPUT_YUV422;
			break;
		case YCBCR_4_2_0:
			dcss_crtc_state->output_encoding =
							DCSS_PIPE_OUTPUT_YUV420;
			break;
		case YCBCR_4_4_4:
			dcss_crtc_state->output_encoding =
							DCSS_PIPE_OUTPUT_YUV444;
			break;
		default:
			break;
		}
	}
}

int dcss_crtc_setup_opipe(struct drm_device *dev, struct drm_atomic_state *state)
{
	struct dcss_dev *dcss = dev->dev_private;
	struct dcss_kms_dev *dcss_kms =
				container_of(dev, struct dcss_kms_dev, base);
	struct drm_crtc_state *crtc_state;
	struct drm_connector *conn;
	struct drm_connector_state *conn_state;
	int i, ret;

	crtc_state = drm_atomic_get_crtc_state(state, &dcss_kms->crtc.base);
	if (WARN_ON(IS_ERR(crtc_state))) {
		ret = PTR_ERR(crtc_state);
		dev_dbg(dcss->dev, "failed to get CRTC state: %d\n", ret);
		return ret;
	}

	if (!dcss_drv_is_componentized(dcss->dev)) {
		conn_state = drm_atomic_get_connector_state(state,
							dcss_kms->connector);
		if (IS_ERR(conn_state)) {
			ret = PTR_ERR(conn_state);
			dev_dbg(dcss->dev,
				"failed to get connector state: %d\n", ret);
			return ret;
		}

		__dcss_crtc_setup_opipe(crtc_state, conn_state);
	} else {
		for_each_new_connector_in_state(state, conn, conn_state, i) {
			if (!conn_state->best_encoder)
				continue;

			if (!crtc_state->active)
				continue;

			__dcss_crtc_setup_opipe(crtc_state, conn_state);
		}
	}

	return 0;
}

int dcss_crtc_init(struct dcss_crtc *crtc, struct drm_device *drm)
{
	struct dcss_dev *dcss = drm->dev_private;
	struct platform_device *pdev = to_platform_device(dcss->dev);
	int ret;

	crtc->plane[0] = dcss_plane_init(drm, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_PRIMARY, 2);
	if (IS_ERR(crtc->plane[0]))
		return PTR_ERR(crtc->plane[0]);

	crtc->base.port = dcss->of_port;

	drm_crtc_helper_add(&crtc->base, &dcss_helper_funcs);
	ret = drm_crtc_init_with_planes(drm, &crtc->base, &crtc->plane[0]->base,
					NULL, &dcss_crtc_funcs, NULL);
	if (ret) {
		dev_err(dcss->dev, "failed to init crtc\n");
		return ret;
	}

	crtc->plane[1] = dcss_plane_init(drm, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_OVERLAY, 1);
	if (IS_ERR(crtc->plane[1]))
		crtc->plane[1] = NULL;

	crtc->plane[2] = dcss_plane_init(drm, drm_crtc_mask(&crtc->base),
					 DRM_PLANE_TYPE_OVERLAY, 0);
	if (IS_ERR(crtc->plane[2]))
		crtc->plane[2] = NULL;

	drm_plane_create_alpha_property(&crtc->plane[0]->base);

	crtc->irq = platform_get_irq_byname(pdev, "vblank");
	if (crtc->irq < 0)
		return crtc->irq;

	ret = request_irq(crtc->irq, dcss_crtc_irq_handler,
			  0, "dcss_drm", crtc);
	if (ret) {
		dev_err(dcss->dev, "irq request failed with %d.\n", ret);
		return ret;
	}

	disable_irq(crtc->irq);

	return 0;
}

void dcss_crtc_attach_color_mgmt_properties(struct dcss_crtc *crtc)
{
	int i;

	/* create color management properties only for video planes */
	for (i = 1; i < 3; i++) {
		if (crtc->plane[i]->type == DRM_PLANE_TYPE_PRIMARY)
			return;

		drm_plane_create_color_properties(&crtc->plane[i]->base,
					BIT(DRM_COLOR_YCBCR_BT601) |
					BIT(DRM_COLOR_YCBCR_BT709) |
					BIT(DRM_COLOR_YCBCR_BT2020),
					BIT(DRM_COLOR_YCBCR_FULL_RANGE) |
					BIT(DRM_COLOR_YCBCR_LIMITED_RANGE),
					DRM_COLOR_YCBCR_BT709,
					DRM_COLOR_YCBCR_FULL_RANGE);
	}
}

void dcss_crtc_deinit(struct dcss_crtc *crtc, struct drm_device *drm)
{
	free_irq(crtc->irq, crtc);
}
