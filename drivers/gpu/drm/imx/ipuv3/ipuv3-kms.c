/*
 * Copyright 2019 NXP
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

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_vblank.h>
#include "imx-drm.h"
#include "ipuv3-plane.h"

static int ipuv3_drm_atomic_check(struct drm_device *dev,
				   struct drm_atomic_state *state)
{
	int ret;

	ret = drm_atomic_helper_check(dev, state);
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
	.fb_create = drm_gem_fb_create,
	.atomic_check = ipuv3_drm_atomic_check,
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

	/*
	 * The flip done wait is only strictly required by imx-drm if a deferred
	 * plane disable is in-flight. As the core requires blocking commits
	 * to wait for the flip it is done here unconditionally. This keeps the
	 * workitem around a bit longer than required for the majority of
	 * non-blocking commits, but we accept that for the sake of simplicity.
	 */
	drm_atomic_helper_wait_for_flip_done(dev, state);

	if (plane_disabling) {
		for_each_old_plane_in_state(state, plane, old_plane_state, i)
			ipu_plane_disable_deferred(plane);

	}

	drm_atomic_helper_commit_hw_done(state);
}

const struct drm_mode_config_helper_funcs ipuv3_drm_mode_config_helpers = {
	.atomic_commit_tail = ipuv3_drm_atomic_commit_tail,
};
