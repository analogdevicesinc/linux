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

static int dcss_drm_atomic_commit(struct drm_device *drm,
				  struct drm_atomic_state *state,
				  bool nonblock)
{
	struct drm_plane_state *plane_state;
	struct drm_plane *plane;
	struct dma_buf *dma_buf;
	int i;

	/*
	 * If the plane fb has an dma-buf attached, fish out the exclusive
	 * fence for the atomic helper to wait on.
	 */
	for_each_plane_in_state(state, plane, plane_state, i) {
		if ((plane->state->fb != plane_state->fb) && plane_state->fb) {
			dma_buf = drm_fb_cma_get_gem_obj(plane_state->fb,
							 0)->base.dma_buf;
			if (!dma_buf)
				continue;
			plane_state->fence =
				reservation_object_get_excl_rcu(dma_buf->resv);
		}
	}

	return drm_atomic_helper_commit(drm, state, nonblock);
}

static void dcss_drm_atomic_commit_tail(struct drm_atomic_state *state)
{
	struct drm_device *dev = state->dev;

	drm_atomic_helper_commit_modeset_disables(dev, state);

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
	.atomic_commit = dcss_drm_atomic_commit,
};

struct drm_mode_config_helper_funcs dcss_drm_mode_config_helpers = {
	.atomic_commit_tail = dcss_drm_atomic_commit_tail,
};
