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

struct dcss_drm_commit {
	struct work_struct work;
	struct drm_device *dev;
	struct drm_atomic_state *state;
};

static void dcss_drm_atomic_commit_tail(struct dcss_drm_commit *commit)
{
	struct drm_atomic_state *state = commit->state;
	struct drm_device *dev = commit->dev;
	struct imx_drm_device *imxdrm = dev->dev_private;

	drm_atomic_helper_wait_for_fences(dev, state, false);

	drm_atomic_helper_wait_for_dependencies(state);

	drm_atomic_helper_commit_modeset_disables(dev, state);

	dcss_kms_setup_output_pipe(state);

	drm_atomic_helper_commit_modeset_enables(dev, state);

	drm_atomic_helper_commit_planes(dev, state,
					DRM_PLANE_COMMIT_ACTIVE_ONLY);

	drm_atomic_helper_commit_hw_done(state);

	drm_atomic_helper_wait_for_vblanks(dev, state);

	drm_atomic_helper_cleanup_planes(dev, state);

	drm_atomic_helper_commit_cleanup_done(state);

	drm_atomic_state_put(state);

	spin_lock(&imxdrm->commit.wait.lock);
	imxdrm->commit.pending = false;
	wake_up_all_locked(&imxdrm->commit.wait);
	spin_unlock(&imxdrm->commit.wait.lock);

	kfree(commit);
}

static void dcss_commit_work(struct work_struct *work)
{
	struct dcss_drm_commit *commit = container_of(work,
						      struct dcss_drm_commit,
						      work);

	dcss_drm_atomic_commit_tail(commit);
}

static int dcss_drm_atomic_commit(struct drm_device *dev,
				  struct drm_atomic_state *state,
				  bool nonblock)
{
	int ret;
	struct imx_drm_device *imxdrm = dev->dev_private;
	struct dcss_drm_commit *commit;

	if (state->async_update) {
		ret = drm_atomic_helper_prepare_planes(dev, state);
		if (ret)
			return ret;

		drm_atomic_helper_async_commit(dev, state);
		drm_atomic_helper_cleanup_planes(dev, state);

		return 0;
	}

	commit = kzalloc(sizeof(*commit), GFP_KERNEL);
	if (!commit)
		return -ENOMEM;

	commit->dev = dev;
	commit->state = state;

	ret = drm_atomic_helper_setup_commit(state, nonblock);
	if (ret)
		goto err_free;

	INIT_WORK(&commit->work, dcss_commit_work);

	ret = drm_atomic_helper_prepare_planes(dev, state);
	if (ret)
		goto err_free;

	if (!nonblock) {
		ret = drm_atomic_helper_wait_for_fences(dev, state, true);
		if (ret)
			goto err;
	}

	spin_lock(&imxdrm->commit.wait.lock);
	ret = wait_event_interruptible_locked(imxdrm->commit.wait,
					      !imxdrm->commit.pending);
	if (ret == 0)
		imxdrm->commit.pending = true;
	spin_unlock(&imxdrm->commit.wait.lock);

	if (ret)
		goto err;

	ret = drm_atomic_helper_swap_state(state, true);
	if (ret)
		goto err;

	drm_atomic_state_get(state);
	if (nonblock)
		queue_work(imxdrm->wq, &commit->work);
	else
		dcss_drm_atomic_commit_tail(commit);

	return 0;

err:
	drm_atomic_helper_cleanup_planes(dev, state);

err_free:
	kfree(commit);
	return ret;
}

const struct drm_mode_config_funcs dcss_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = dcss_drm_output_poll_changed,
	.atomic_check = dcss_drm_atomic_check,
	.atomic_commit = dcss_drm_atomic_commit,
};
