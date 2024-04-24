// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2017-2020,2022,2023 NXP
 */

#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/slab.h>
#include <linux/sort.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_blend.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "dpu95.h"
#include "dpu95-crtc.h"
#include "dpu95-drv.h"
#include "dpu95-plane.h"

static int zpos_cmp(const void *a, const void *b)
{
	const struct drm_plane_state *sa = *(struct drm_plane_state **)a;
	const struct drm_plane_state *sb = *(struct drm_plane_state **)b;

	return sa->normalized_zpos - sb->normalized_zpos;
}

static int
dpu95_atomic_sort_planes_per_crtc(struct drm_crtc_state *crtc_state,
				  struct drm_plane_state **plane_states)
{
	struct drm_atomic_state *state = crtc_state->state;
	struct drm_plane *plane;
	int n = 0;

	drm_atomic_crtc_state_for_each_plane(plane, crtc_state) {
		struct drm_plane_state *plane_state =
			drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);
		plane_states[n++] = plane_state;
	}

	sort(plane_states, n, sizeof(*plane_states), zpos_cmp, NULL);

	return n;
}

static void
dpu95_atomic_set_top_plane_per_crtc(struct drm_plane_state **plane_states, int n)
{
	struct dpu95_plane_state *dpstate;
	int i;

	for (i = 0; i < n; i++) {
		dpstate = to_dpu95_plane_state(plane_states[i]);
		dpstate->is_top = (i == (n - 1)) ? true : false;
	}
}

static int dpu95_plane_alloc_hscaler(struct drm_plane *plane,
				     struct dpu95_fetchunit *fu,
				     unsigned int stream_id)
{
	struct dpu95_plane *dplane = to_dpu95_plane(plane);
	const struct dpu95_fetchunit_ops *fu_ops;
	const struct dpu95_hscaler_ops *hs_ops;
	struct dpu95_hscaler *hs;

	if (dplane->grp->hs_used) {
		dpu95_plane_dbg(plane, "failed to alloc HScaler on stream%u\n",
				stream_id);
		return -EINVAL;
	}

	fu_ops = dpu95_fu_get_ops(fu);
	hs = fu_ops->get_hscaler(fu);
	hs_ops = dpu95_hs_get_ops(hs);

	/* avoid HScaler hot migration */
	if (hs_ops->has_stream_id(hs) &&
	    hs_ops->get_stream_id(hs) != stream_id) {
		dpu95_plane_dbg(plane,
				"failed to hot migrate HScaler to stream%u\n",
				stream_id);
		return -EINVAL;
	}

	dplane->grp->hs_used = true;

	return 0;
}

static int
dpu95_atomic_assign_plane_source_per_crtc(struct dpu95_crtc *dpu_crtc,
					  struct drm_plane_state **plane_states,
					  int n, bool use_current_source)
{
	const struct dpu95_fetchunit_ops *fu_ops;
	unsigned int sid = dpu_crtc->stream_id;
	struct drm_plane_state *plane_state;
	struct dpu95_plane_state *dpstate;
	struct dpu95_layerblend *blend;
	u32 src_w, dst_w;
	union dpu95_plane_stage stage;
	struct dpu95_plane_grp *grp;
	struct dpu95_plane_res *res;
	struct dpu95_plane *dplane;
	struct drm_framebuffer *fb;
	struct dpu95_fetchunit *fu;
	struct drm_plane *plane;
	bool fb_is_packed_yuv422;
	struct list_head *node;
	bool found_fu;
	bool need_fe;
	bool need_hs;
	u32 cap_mask;
	int i, j;
	int ret;

	/* for active planes only */
	for (i = 0; i < n; i++) {
		plane_state = plane_states[i];
		dpstate = to_dpu95_plane_state(plane_state);
		plane = plane_state->plane;
		dplane = to_dpu95_plane(plane);
		fb = plane_state->fb;
		grp = dplane->grp;
		res = &grp->res;

		src_w = plane_state->src_w >> 16;
		dst_w = plane_state->crtc_w;

		fb_is_packed_yuv422 =
				drm_format_info_is_yuv_packed(fb->format) &&
				drm_format_info_is_yuv_sampling_422(fb->format);
		need_fe = fb->format->num_planes > 1;
		need_hs = src_w != dst_w;

		/*
		 * If modeset is not allowed, use the current source for
		 * the prone-to-put planes so that unnecessary updates and
		 * spurious EBUSY can be avoided.
		 */
		if (use_current_source) {
			fu_ops = dpu95_fu_get_ops(dpstate->source);
			fu_ops->set_inavailable(dpstate->source);

			if (need_hs)
				grp->hs_used = true;
			continue;
		}

		cap_mask = 0;
		if (need_fe)
			cap_mask |= DPU95_FETCHUNIT_CAP_USE_FETCHECO;
		if (need_hs)
			cap_mask |= DPU95_FETCHUNIT_CAP_USE_SCALER;
		if (fb_is_packed_yuv422)
			cap_mask |= DPU95_FETCHUNIT_CAP_PACKED_YUV422;

		/* assign source */
		found_fu = false;
		list_for_each(node, &grp->fu_list) {
			fu = dpu95_fu_get_from_list(node);

			fu_ops = dpu95_fu_get_ops(fu);

			/* available? */
			if (!fu_ops->is_available(fu))
				continue;

			/* enough capability? */
			if ((cap_mask & fu_ops->get_cap_mask(fu)) != cap_mask)
				continue;

			/* avoid fetchunit hot migration */
			if (fu_ops->has_stream_id(fu) &&
			    fu_ops->get_stream_id(fu) != sid)
				continue;

			if (need_hs) {
				ret = dpu95_plane_alloc_hscaler(plane, fu, sid);
				if (ret)
					return ret;
			}

			fu_ops->set_inavailable(fu);
			found_fu = true;
			break;
		}

		if (!found_fu) {
			dpu95_plane_dbg(plane,
					"failed to find fetchunit on stream%u\n",
					sid);
			return -EINVAL;
		}

		dpstate->source = fu;

		/* assign stage and blend */
		if (sid) {
			j = DPU95_HW_PLANES - (n - i);
			blend = res->lb[j];
			if (i == 0)
				stage.cf = grp->cf[sid];
			else
				stage.lb = res->lb[j - 1];
		} else {
			blend = res->lb[i];
			if (i == 0)
				stage.cf = grp->cf[sid];
			else
				stage.lb = res->lb[i - 1];
		}

		dpstate->stage = stage;
		dpstate->blend = blend;
	}

	return 0;
}

static int dpu95_atomic_assign_plane_source(struct drm_atomic_state *state,
					    u32 crtc_mask_prone_to_put,
					    bool prone_to_put)
{
	struct drm_plane_state **plane_states;
	struct drm_crtc_state *crtc_state;
	struct dpu95_crtc *dpu_crtc;
	bool use_current_source;
	struct drm_crtc *crtc;
	int ret, i, n;

	use_current_source = !state->allow_modeset && prone_to_put;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		/* Skip if no active plane. */
		if (crtc_state->plane_mask == 0)
			continue;

		if (prone_to_put !=
		    !!(drm_crtc_mask(crtc) & crtc_mask_prone_to_put))
			continue;

		dpu_crtc = to_dpu95_crtc(crtc);

		plane_states = kmalloc_array(DPU95_HW_PLANES,
					     sizeof(*plane_states), GFP_KERNEL);
		if (!plane_states) {
			ret = -ENOMEM;
			dpu95_crtc_dbg(crtc,
				       "failed to alloc plane state ptrs: %d\n",
				       ret);
			return ret;
		}

		n = dpu95_atomic_sort_planes_per_crtc(crtc_state, plane_states);
		if (n < 0) {
			dpu95_crtc_dbg(crtc, "failed to sort planes: %d\n", n);
			kfree(plane_states);
			return n;
		}

		dpu95_atomic_set_top_plane_per_crtc(plane_states, n);

		ret = dpu95_atomic_assign_plane_source_per_crtc(dpu_crtc,
					plane_states, n, use_current_source);
		if (ret) {
			dpu95_crtc_dbg(crtc,
				       "failed to assign resource to plane: %d\n",
				       ret);
			kfree(plane_states);
			return ret;
		}

		kfree(plane_states);
	}

	return 0;
}

static void dpu95_atomic_put_plane_state(struct drm_atomic_state *state,
					 struct drm_plane *plane)
{
	int index = drm_plane_index(plane);

	plane->funcs->atomic_destroy_state(plane, state->planes[index].state);
	state->planes[index].ptr = NULL;
	state->planes[index].state = NULL;
	state->planes[index].old_state = NULL;
	state->planes[index].new_state = NULL;

	drm_modeset_unlock(&plane->mutex);

	dpu95_plane_dbg(plane, "put state\n");
}

static void dpu95_atomic_put_crtc_state(struct drm_atomic_state *state,
					struct drm_crtc *crtc)
{
	int index = drm_crtc_index(crtc);

	crtc->funcs->atomic_destroy_state(crtc, state->crtcs[index].state);
	state->crtcs[index].ptr = NULL;
	state->crtcs[index].state = NULL;
	state->crtcs[index].old_state = NULL;
	state->crtcs[index].new_state = NULL;

	drm_modeset_unlock(&crtc->mutex);

	dpu95_crtc_dbg(crtc, "put state\n");
}

static void
dpu95_atomic_put_possible_states_per_crtc(struct drm_crtc_state *crtc_state)
{
	struct drm_plane_state *old_plane_state, *new_plane_state;
	struct dpu95_plane_state *old_dpstate, *new_dpstate;
	struct drm_atomic_state *state = crtc_state->state;
	struct drm_crtc *crtc = crtc_state->crtc;
	struct drm_plane *plane;

	drm_atomic_crtc_state_for_each_plane(plane, crtc_state) {
		old_plane_state = drm_atomic_get_old_plane_state(state, plane);
		new_plane_state = drm_atomic_get_new_plane_state(state, plane);

		old_dpstate = to_dpu95_plane_state(old_plane_state);
		new_dpstate = to_dpu95_plane_state(new_plane_state);

		/* Should be enough to check the below HW plane resources. */
		if (old_dpstate->stage.ptr != new_dpstate->stage.ptr ||
		    old_dpstate->source != new_dpstate->source ||
		    old_dpstate->blend != new_dpstate->blend)
			return;
	}

	drm_atomic_crtc_state_for_each_plane(plane, crtc_state)
		dpu95_atomic_put_plane_state(state, plane);

	dpu95_atomic_put_crtc_state(state, crtc);
}

static int dpu95_drm_atomic_check(struct drm_device *dev,
				  struct drm_atomic_state *state)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(dev);
	struct dpu95_plane_grp *plane_grp = &dpu_drm->dpu_plane_grp;
	const struct dpu95_fetchunit_ops *fu_ops;
	struct drm_crtc_state *crtc_state;
	struct dpu95_fetchunit *fu;
	u32 crtc_mask_prone_to_put;
	u32 crtc_mask_in_state = 0;
	struct list_head *node;
	struct drm_crtc *crtc;
	int ret, i;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	/* Set crtc_mask_in_state. */
	for_each_new_crtc_in_state(state, crtc, crtc_state, i)
		crtc_mask_in_state |= drm_crtc_mask(crtc);

	/*
	 * Those CRTCs not in the state for check are prone to put,
	 * because HW resources of their active planes are likely unchanged.
	 */
	crtc_mask_prone_to_put = dpu_drm->crtc_mask ^ crtc_mask_in_state;

	/*
	 * For those CRTCs prone to put, get their CRTC states as well,
	 * so that all relevant active plane states can be got when
	 * assigning HW resources to them later on.
	 */
	drm_for_each_crtc(crtc, dev) {
		if ((drm_crtc_mask(crtc) & crtc_mask_prone_to_put) == 0)
			continue;

		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);
	}

	/*
	 * Set all the fetchunits in the plane group to be available,
	 * so that they can be assigned to planes.
	 */
	list_for_each(node, &plane_grp->fu_list) {
		fu = dpu95_fu_get_from_list(node);
		fu_ops = dpu95_fu_get_ops(fu);
		fu_ops->set_available(fu);
	}

	plane_grp->hs_used = false;

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret) {
		drm_dbg_kms(dev, "failed to normalize zpos: %d\n", ret);
		return ret;
	}

	/*
	 * Assign HW resources to planes in question.
	 * It is likely to fail due to some reasons, e.g., no enough
	 * fetchunits, users ask for more features than the HW resources
	 * can provide, HW resource hot-migration bewteen CRTCs is needed.
	 *
	 * Do the assignment for the prone-to-put CRTCs first, as we want
	 * the planes of them to use the current sources if modeset is not
	 * allowed.
	 */
	ret = dpu95_atomic_assign_plane_source(state,
					       crtc_mask_prone_to_put, true);
	if (ret) {
		drm_dbg_kms(dev,
			    "failed to assign source to prone-to-put plane: %d\n",
			    ret);
		return ret;
	}
	ret = dpu95_atomic_assign_plane_source(state,
					       crtc_mask_prone_to_put, false);
	if (ret) {
		drm_dbg_kms(dev, "failed to assign source to plane: %d\n", ret);
		return ret;
	}

	/*
	 * To gain some performance, put those CRTC and plane states
	 * which can be put.
	 */
	drm_for_each_crtc(crtc, dev) {
		if (crtc_mask_prone_to_put & drm_crtc_mask(crtc)) {
			crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
			if (WARN_ON(!crtc_state))
				return -EINVAL;

			dpu95_atomic_put_possible_states_per_crtc(crtc_state);
		}
	}

	return drm_atomic_helper_check_planes(dev, state);
}

static const struct drm_mode_config_funcs dpu95_drm_mode_config_funcs = {
	.fb_create	= drm_gem_fb_create,
	.atomic_check	= dpu95_drm_atomic_check,
	.atomic_commit	= drm_atomic_helper_commit,
};

static int dpu95_kms_init_encoder_per_crtc(struct dpu95_drm_device *dpu_drm,
					   struct dpu95_crtc *dpu_crtc)
{
	struct drm_device *drm = &dpu_drm->base;
	struct drm_crtc *crtc = &dpu_crtc->base;
	struct device_node *ep, *remote;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_bridge *bridge;
	int ret = 0;

	ep = of_get_next_child(dpu_crtc->np, NULL);
	if (!ep) {
		drm_err(drm, "failed to find CRTC(%pOF) port's endpoint\n",
			dpu_crtc->np);
		return -ENODEV;
	}

	remote = of_graph_get_remote_port_parent(ep);
	if (!of_device_is_available(remote))
		goto out;

	bridge = of_drm_find_bridge(remote);
	if (!bridge) {
		ret = -EPROBE_DEFER;
		drm_dbg_kms(drm, "failed to find bridge for stream%u: %d\n",
			    dpu_crtc->stream_id, ret);
		goto out;
	}

	encoder = &dpu_drm->encoder[dpu_crtc->stream_id];
	ret = drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_NONE);
	if (ret) {
		drm_err(drm, "failed to initialize encoder for stream%u: %d\n",
			dpu_crtc->stream_id, ret);
		goto out;
	}

	encoder->possible_crtcs = drm_crtc_mask(crtc);

	ret = drm_bridge_attach(encoder, bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret) {
		drm_err(drm,
			"failed to attach bridge to encoder for stream%u: %d\n",
			dpu_crtc->stream_id, ret);
		goto out;
	}

	connector = drm_bridge_connector_init(drm, encoder);
	if (IS_ERR(connector)) {
		ret = PTR_ERR(connector);
		drm_err(drm, "failed to initialize bridge connector for stream%u: %d\n",
			dpu_crtc->stream_id, ret);
		goto out;
	}

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		drm_err(drm, "failed to attach encoder to connector for stream%u: %d\n",
			dpu_crtc->stream_id, ret);

out:
	of_node_put(remote);
	of_node_put(ep);
	return ret;
}

int dpu95_kms_prepare(struct dpu95_drm_device *dpu_drm)
{
	struct drm_device *drm = &dpu_drm->base;
	struct dpu95_plane *dpu_overlay;
	struct dpu95_crtc *dpu_crtc;
	int ret, i;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;

	drm->mode_config.min_width = 60;
	drm->mode_config.min_height = 60;
	drm->mode_config.max_width = 8192;
	drm->mode_config.max_height = 8192;
	drm->mode_config.funcs = &dpu95_drm_mode_config_funcs;
	drm->max_vblank_count = DPU95_FRAMEGEN_MAX_FRAME_INDEX;

	for (i = 0; i < DPU95_CRTCS; i++) {
		dpu_crtc = &dpu_drm->dpu_crtc[i];

		ret = dpu95_crtc_init(dpu_drm, dpu_crtc, i);
		if (ret)
			return ret;

		ret = dpu95_kms_init_encoder_per_crtc(dpu_drm, dpu_crtc);
		if (ret)
			return ret;
	}

	for (i = 0; i < DPU95_OVERLAYS; i++) {
		dpu_overlay = &dpu_drm->dpu_overlay[i];

		ret = dpu95_plane_initialize(dpu_drm, dpu_overlay,
					     dpu_drm->crtc_mask,
					     DRM_PLANE_TYPE_OVERLAY);
		if (ret) {
			drm_err(drm, "failed to init overlay plane%d: %d\n",
				i, ret);
			return ret;
		}
	}

	ret = drm_vblank_init(drm, DPU95_CRTCS);
	if (ret) {
		drm_err(drm, "failed to initialize vblank support: %d\n", ret);
		return ret;
	}

	drm_mode_config_reset(drm);

	drm_kms_helper_poll_init(drm);

	return 0;
}

void dpu95_kms_unprepare(struct dpu95_drm_device *dpu_drm)
{
	drm_kms_helper_poll_fini(&dpu_drm->base);
}
