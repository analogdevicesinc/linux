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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_atomic.h>
#include <drm/drm_gem_cma_helper.h>
#include <linux/dma-buf.h>
#include <linux/reservation.h>

#include "imx-drm.h"
#include "dcss-crtc.h"

static void dcss_drm_output_poll_changed(struct drm_device *drm)
{
	struct imx_drm_device *imxdrm = drm->dev_private;

	drm_fbdev_cma_hotplug_event(imxdrm->fbhelper);
}

static int dcss_drm_atomic_check(struct drm_device *drm,
				 struct drm_atomic_state *state)
{
	int ret;

	ret = drm_atomic_helper_check_modeset(drm, state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_planes(drm, state);
	if (ret)
		return ret;

	/*
	 * Check modeset again in case crtc_state->mode_changed is
	 * updated in plane's ->atomic_check callback.
	 */
	return drm_atomic_helper_check_modeset(drm, state);
}

static void dcss_kms_setup_output_pipe(struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_connector *connector;
	struct drm_connector_state *conn_state;
	struct drm_display_info *di;
	int i;

	for_each_connector_in_state(state, connector, conn_state, i) {
		if (!connector->state->best_encoder)
			continue;

		if (!connector->state->crtc->state->active ||
		    !drm_atomic_crtc_needs_modeset(connector->state->crtc->state))
			continue;

		crtc = connector->state->crtc;
		di = &connector->display_info;

		dcss_crtc_setup_opipe(crtc, connector, di->hdmi.colorimetry,
				      di->hdmi.hdr_panel_metadata.eotf,
				      HDMI_QUANTIZATION_RANGE_FULL);
	}
}

static void dcss_drm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	dcss_kms_setup_output_pipe(state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

const struct drm_mode_config_funcs dcss_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = dcss_drm_output_poll_changed,
	.atomic_check = dcss_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

struct drm_mode_config_helper_funcs dcss_drm_mode_config_helpers = {
	.atomic_commit_tail = dcss_drm_atomic_commit_tail,
};
