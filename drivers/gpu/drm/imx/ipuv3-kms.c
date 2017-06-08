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
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <linux/dma-buf.h>
#include <linux/reservation.h>
#include "imx-drm.h"

static void imx_drm_output_poll_changed(struct drm_device *drm)
{
	struct imx_drm_device *imxdrm = drm->dev_private;

	drm_fbdev_cma_hotplug_event(imxdrm->fbhelper);
}

static int imx_drm_atomic_check(struct drm_device *dev,
				struct drm_atomic_state *state)
{
	int ret;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret)
		return ret;

	/*
	 * Check modeset again in case crtc_state->mode_changed is
	 * updated in plane's ->atomic_check callback.
	 */
	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	/* Assign PRG/PRE channels and check if all constrains are satisfied. */
	ret = ipu_planes_assign_pre(dev, state);
	if (ret)
		return ret;

	return ret;
}

const struct drm_mode_config_funcs ipuv3_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = imx_drm_output_poll_changed,
	.atomic_check = imx_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void ipuv3_drm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;
	struct drm_plane *plane;
	struct drm_plane_state *old_plane_state, *new_plane_state;
	bool plane_disabling = false;
	int i;

	drm_atomic_helper_commit_modeset_disables(dev, state);

	drm_atomic_helper_commit_planes(dev, state,
				DRM_PLANE_COMMIT_ACTIVE_ONLY |
				DRM_PLANE_COMMIT_NO_DISABLE_AFTER_MODESET);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	for_each_oldnew_plane_in_state(state, plane, old_plane_state, new_plane_state, i) {
		if (drm_atomic_plane_disabling(old_plane_state, new_plane_state))
			plane_disabling = true;
	}

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);
}

struct drm_mode_config_helper_funcs ipuv3_drm_mode_config_helpers = {
	.atomic_commit_tail = ipuv3_drm_atomic_commit_tail,
};

