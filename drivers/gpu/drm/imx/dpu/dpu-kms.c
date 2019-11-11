/*
 * Copyright 2017-2019 NXP
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
#include <drm/drm_blend.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <linux/sort.h>
#include <video/dpu.h>
#include "dpu-crtc.h"
#include "dpu-plane.h"
#include "imx-drm.h"

static const lb_prim_sel_t cf_stages[] = {LB_PRIM_SEL__CONSTFRAME0,
					  LB_PRIM_SEL__CONSTFRAME1};
static const lb_prim_sel_t stages[] = {LB_PRIM_SEL__LAYERBLEND0,
				       LB_PRIM_SEL__LAYERBLEND1,
				       LB_PRIM_SEL__LAYERBLEND2,
				       LB_PRIM_SEL__LAYERBLEND3};

static struct drm_plane_state **
dpu_atomic_alloc_tmp_planes_per_crtc(struct drm_device *dev)
{
	int total_planes = dev->mode_config.num_total_plane;
	struct drm_plane_state **states;

	states = kmalloc_array(total_planes, sizeof(*states), GFP_KERNEL);
	if (!states)
		return ERR_PTR(-ENOMEM);

	return states;
}

static int zpos_cmp(const void *a, const void *b)
{
	const struct drm_plane_state *sa = *(struct drm_plane_state **)a;
	const struct drm_plane_state *sb = *(struct drm_plane_state **)b;

	return sa->normalized_zpos - sb->normalized_zpos;
}

static int dpu_atomic_sort_planes_per_crtc(struct drm_crtc_state *crtc_state,
					   struct drm_plane_state **states)
{
	struct drm_atomic_state *state = crtc_state->state;
	struct drm_device *dev = state->dev;
	struct drm_plane *plane;
	int n = 0;

	drm_for_each_plane_mask(plane, dev, crtc_state->plane_mask) {
		struct drm_plane_state *plane_state =
			drm_atomic_get_plane_state(state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);
		states[n++] = plane_state;
	}

	sort(states, n, sizeof(*states), zpos_cmp, NULL);

	return n;
}

static void
dpu_atomic_compute_plane_base_per_crtc(struct drm_plane_state **states, int n)
{
	struct dpu_plane_state *dpstate;
	int i, left, right, top, bottom, tmp;

	/* compute the plane base */
	left   = states[0]->crtc_x;
	top    = states[0]->crtc_y;
	right  = states[0]->crtc_x + states[0]->crtc_w;
	bottom = states[0]->crtc_y + states[0]->crtc_h;

	for (i = 1; i < n; i++) {
		left = min(states[i]->crtc_x, left);
		top =  min(states[i]->crtc_y, top);

		tmp = states[i]->crtc_x + states[i]->crtc_w;
		right = max(tmp, right);

		tmp = states[i]->crtc_y + states[i]->crtc_h;
		bottom = max(tmp, bottom);
	}

	/* BTW, be smart to compute the layer offset */
	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);
		dpstate->layer_x = states[i]->crtc_x - left;
		dpstate->layer_y = states[i]->crtc_y - top;
	}

	/* finally, store the base in plane state */
	dpstate = to_dpu_plane_state(states[0]);
	dpstate->base_x = left;
	dpstate->base_y = top;
	dpstate->base_w = right - left;
	dpstate->base_h = bottom - top;
}

static void
dpu_atomic_set_top_plane_per_crtc(struct drm_plane_state **states, int n)
{
	struct dpu_plane_state *dpstate;
	int i;

	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);
		dpstate->is_top = (i == (n - 1)) ? true : false;
	}
}

static int get_total_asrc_num(unsigned long src_a_mask)
{
	int num = 0;
	int bit;

	for_each_set_bit(bit, &src_a_mask, 32)
		num++;

	return num;
}

static int
dpu_atomic_assign_plane_source_per_crtc(struct drm_plane_state **states, int n)
{
	struct dpu_plane_state *dpstate;
	struct dpu_plane *dplane;
	struct dpu_plane_grp *grp;
	struct drm_framebuffer *fb;
	struct dpu_fetchunit *fu;
	struct dpu_fetchunit *fe;
	struct dpu_hscaler *hs;
	struct dpu_vscaler *vs;
	lb_prim_sel_t stage;
	dpu_block_id_t blend;
	unsigned int sid, src_sid;
	unsigned int num_planes;
	int i, j, k = 0, m;
	int total_asrc_num;
	u32 src_a_mask, cap_mask, fe_mask, hs_mask, vs_mask;
	bool need_fetcheco, need_hscaler, need_vscaler;
	bool fmt_is_yuv;

	/* for active planes only */
	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);
		dplane = to_dpu_plane(states[i]->plane);
		fb = states[i]->fb;
		num_planes = fb->format->num_planes;
		fmt_is_yuv = drm_format_is_yuv(fb->format->format);
		grp = dplane->grp;

		sid = dplane->stream_id;

		need_fetcheco = (num_planes > 1);
		need_hscaler = (states[i]->src_w >> 16 != states[i]->crtc_w);
		need_vscaler = (states[i]->src_h >> 16 != states[i]->crtc_h);

		src_a_mask = grp->src_a_mask;
		fe_mask = 0;
		hs_mask = 0;
		vs_mask = 0;

		total_asrc_num = get_total_asrc_num(src_a_mask);

		/* assign source */
		mutex_lock(&grp->mutex);
		for (j = 0; j < total_asrc_num; j++) {
			k = ffs(src_a_mask) - 1;
			if (k < 0)
				return -EINVAL;

			fu = source_to_fu(&grp->res, sources[k]);
			if (!fu)
				return -EINVAL;

			/* avoid on-the-fly/hot migration */
			src_sid = fu->ops->get_stream_id(fu);
			if (src_sid && src_sid != BIT(sid))
				goto next;

			if (fetchunit_is_fetchdecode(fu)) {
				cap_mask = fetchdecode_get_vproc_mask(fu);

				if (need_fetcheco) {
					fe = fetchdecode_get_fetcheco(fu);

					/* avoid on-the-fly/hot migration */
					src_sid = fu->ops->get_stream_id(fe);
					if (src_sid && src_sid != BIT(sid))
						goto next;

					/* fetch unit has the fetcheco cap? */
					if (!dpu_vproc_has_fetcheco_cap(cap_mask))
						goto next;

					fe_mask =
					   dpu_vproc_get_fetcheco_cap(cap_mask);

					/* fetcheco available? */
					if (grp->src_use_vproc_mask & fe_mask)
						goto next;
				}

				if (need_hscaler) {
					hs = fetchdecode_get_hscaler(fu);

					/* avoid on-the-fly/hot migration */
					src_sid = hscaler_get_stream_id(hs);
					if (src_sid && src_sid != BIT(sid))
						goto next;

					/* fetch unit has the hscale cap */
					if (!dpu_vproc_has_hscale_cap(cap_mask))
						goto next;

					hs_mask =
					     dpu_vproc_get_hscale_cap(cap_mask);

					/* hscaler available? */
					if (grp->src_use_vproc_mask & hs_mask)
						goto next;
				}

				if (need_vscaler) {
					vs = fetchdecode_get_vscaler(fu);

					/* avoid on-the-fly/hot migration */
					src_sid = vscaler_get_stream_id(vs);
					if (src_sid && src_sid != BIT(sid))
						goto next;

					/* fetch unit has the vscale cap? */
					if (!dpu_vproc_has_vscale_cap(cap_mask))
						goto next;

					vs_mask =
					     dpu_vproc_get_vscale_cap(cap_mask);

					/* vscaler available? */
					if (grp->src_use_vproc_mask & vs_mask)
						goto next;
				}
			} else {
				if (fmt_is_yuv || need_fetcheco ||
				    need_hscaler || need_vscaler)
					goto next;
			}

			grp->src_a_mask &= ~BIT(k);
			grp->src_use_vproc_mask |= fe_mask | hs_mask | vs_mask;
			break;
next:
			src_a_mask &= ~BIT(k);
			fe_mask = 0;
			hs_mask = 0;
			vs_mask = 0;
		}
		mutex_unlock(&grp->mutex);

		if (j == total_asrc_num)
			return -EINVAL;

		dpstate->source = sources[k];

		/* assign stage and blend */
		if (sid) {
			m = grp->hw_plane_num - (n - i);
			stage = i ? stages[m - 1] : cf_stages[sid];
			blend = blends[m];
		} else {
			stage = i ? stages[i - 1] : cf_stages[sid];
			blend = blends[i];
		}

		dpstate->stage = stage;
		dpstate->blend = blend;
	}

	return 0;
}

static void
dpu_atomic_mark_pipe_states_prone_to_put_per_crtc(struct drm_crtc *crtc,
						u32 crtc_mask,
						struct drm_atomic_state *state,
						bool *puts)
{
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	bool found_pstate = false;
	int i;

	if ((crtc_mask & drm_crtc_mask(crtc)) == 0) {
		for_each_new_plane_in_state(state, plane, plane_state, i) {
			if (plane->possible_crtcs & drm_crtc_mask(crtc)) {
				found_pstate = true;
				break;
			}
		}

		if (!found_pstate)
			puts[drm_crtc_index(crtc)] = true;
	}
}

static void
dpu_atomic_put_plane_state(struct drm_atomic_state *state,
			   struct drm_plane *plane)
{
	int index = drm_plane_index(plane);

	plane->funcs->atomic_destroy_state(plane, state->planes[index].state);
	state->planes[index].ptr = NULL;
	state->planes[index].state = NULL;

	drm_modeset_unlock(&plane->mutex);
}

static void
dpu_atomic_put_crtc_state(struct drm_atomic_state *state,
			  struct drm_crtc *crtc)
{
	int index = drm_crtc_index(crtc);

	crtc->funcs->atomic_destroy_state(crtc, state->crtcs[index].state);
	state->crtcs[index].ptr = NULL;
	state->crtcs[index].state = NULL;

	drm_modeset_unlock(&crtc->mutex);
}

static void
dpu_atomic_put_possible_states_per_crtc(struct drm_crtc_state *crtc_state)
{
	struct drm_atomic_state *state = crtc_state->state;
	struct drm_crtc *crtc = crtc_state->crtc;
	struct drm_crtc_state *old_crtc_state = crtc->state;
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_state **old_dpstates;
	struct dpu_plane_state *old_dpstate, *new_dpstate;
	u32 active_mask = 0;
	int i;

	old_dpstates = crtc_state_get_dpu_plane_states(old_crtc_state);
	if (WARN_ON(!old_dpstates))
		return;

	for (i = 0; i < dplane->grp->hw_plane_num; i++) {
		old_dpstate = old_dpstates[i];
		if (!old_dpstate)
			continue;

		active_mask |= BIT(i);

		drm_atomic_crtc_state_for_each_plane(plane, crtc_state) {
			if (drm_plane_index(plane) !=
			    drm_plane_index(old_dpstate->base.plane))
				continue;

			plane_state =
				drm_atomic_get_existing_plane_state(state,
									plane);
			if (WARN_ON(!plane_state))
				return;

			new_dpstate = to_dpu_plane_state(plane_state);

			active_mask &= ~BIT(i);

			/*
			 * Should be enough to check the below real HW plane
			 * resources only.
			 * Vproc resources and things like layer_x/y should
			 * be fine.
			 */
			if (old_dpstate->stage  != new_dpstate->stage  ||
			    old_dpstate->source != new_dpstate->source ||
			    old_dpstate->blend  != new_dpstate->blend)
				return;
		}
	}

	/* pure software check */
	if (WARN_ON(active_mask))
		return;

	drm_atomic_crtc_state_for_each_plane(plane, crtc_state)
		dpu_atomic_put_plane_state(state, plane);

	dpu_atomic_put_crtc_state(state, crtc);
}

static int dpu_drm_atomic_check(struct drm_device *dev,
				struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct drm_plane *plane;
	struct dpu_plane *dpu_plane;
	struct drm_plane_state *plane_state;
	struct dpu_plane_state *dpstate;
	struct drm_framebuffer *fb;
	struct dpu_plane_grp *grp[MAX_DPU_PLANE_GRP];
	int ret, i, grp_id;
	int active_plane[MAX_DPU_PLANE_GRP];
	int active_plane_fetcheco[MAX_DPU_PLANE_GRP];
	int active_plane_hscale[MAX_DPU_PLANE_GRP];
	int active_plane_vscale[MAX_DPU_PLANE_GRP];
	bool pipe_states_prone_to_put[MAX_CRTC];
	u32 crtc_mask_in_state = 0;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret) {
		DRM_DEBUG_KMS("%s: failed to check modeset\n", __func__);
		return ret;
	}

	for (i = 0; i < MAX_CRTC; i++)
		pipe_states_prone_to_put[i] = false;

	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		active_plane[i] = 0;
		active_plane_fetcheco[i] = 0;
		active_plane_hscale[i] = 0;
		active_plane_vscale[i] = 0;
		grp[i] = NULL;
	}

	for_each_new_crtc_in_state(state, crtc, crtc_state, i)
		crtc_mask_in_state |= drm_crtc_mask(crtc);

	drm_for_each_crtc(crtc, dev) {
		dpu_atomic_mark_pipe_states_prone_to_put_per_crtc(crtc,
						crtc_mask_in_state, state,
						pipe_states_prone_to_put);

		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (WARN_ON(IS_ERR(crtc_state)))
			return PTR_ERR(crtc_state);

		drm_for_each_plane_mask(plane, dev, crtc_state->plane_mask) {
			plane_state = drm_atomic_get_plane_state(state, plane);
			if (IS_ERR(plane_state)) {
				DRM_DEBUG_KMS("failed to get plane state\n");
				return PTR_ERR(plane_state);
			}

			dpstate = to_dpu_plane_state(plane_state);
			fb = plane_state->fb;
			dpu_plane = to_dpu_plane(plane);
			grp_id = dpu_plane->grp->id;
			active_plane[grp_id]++;

			if (fb->format->num_planes > 1)
				active_plane_fetcheco[grp_id]++;

			if (plane_state->src_w >> 16 != plane_state->crtc_w)
				active_plane_hscale[grp_id]++;

			if (plane_state->src_h >> 16 != plane_state->crtc_h)
				active_plane_vscale[grp_id]++;

			if (grp[grp_id] == NULL)
				grp[grp_id] = dpu_plane->grp;
		}
	}

	/* enough resources? */
	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		if (!grp[i])
			continue;

		if (active_plane[i] > grp[i]->hw_plane_num) {
			DRM_DEBUG_KMS("no enough fetch units\n");
			return -EINVAL;
		}

		if (active_plane_fetcheco[i] > grp[i]->hw_plane_fetcheco_num) {
			DRM_DEBUG_KMS("no enough FetchEcos\n");
			return -EINVAL;
		}

		if (active_plane_hscale[i] > grp[i]->hw_plane_hscaler_num) {
			DRM_DEBUG_KMS("no enough Hscalers\n");
			return -EINVAL;
		}

		if (active_plane_vscale[i] > grp[i]->hw_plane_vscaler_num) {
			DRM_DEBUG_KMS("no enough Vscalers\n");
			return -EINVAL;
		}
	}

	/* initialize resource mask */
	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		if (!grp[i])
			continue;

		mutex_lock(&grp[i]->mutex);
		grp[i]->src_a_mask = grp[i]->src_mask;
		grp[i]->src_use_vproc_mask = 0;
		mutex_unlock(&grp[i]->mutex);
	}

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret)
		return ret;

	for_each_new_crtc_in_state(state, crtc, crtc_state, i) {
		struct drm_plane_state **states;
		int n;

		states = dpu_atomic_alloc_tmp_planes_per_crtc(dev);
		if (IS_ERR(states)) {
			DRM_DEBUG_KMS(
				"[CRTC:%d:%s] cannot alloc plane state ptrs\n",
					crtc->base.id, crtc->name);
			return PTR_ERR(states);
		}

		n = dpu_atomic_sort_planes_per_crtc(crtc_state, states);
		if (n < 0) {
			DRM_DEBUG_KMS("[CRTC:%d:%s] failed to sort planes\n",
					crtc->base.id, crtc->name);
			kfree(states);
			return n;
		}

		/* no active planes? */
		if (n == 0) {
			kfree(states);
			continue;
		}

		/* 'zpos = 0' means primary plane */
		if (states[0]->plane->type != DRM_PLANE_TYPE_PRIMARY) {
			DRM_DEBUG_KMS(
				"[CRTC:%d:%s] has only overlay plane(s)\n",
					crtc->base.id, crtc->name);
			kfree(states);
			return -EINVAL;
		}

		dpu_atomic_compute_plane_base_per_crtc(states, n);
		dpu_atomic_set_top_plane_per_crtc(states, n);

		ret = dpu_atomic_assign_plane_source_per_crtc(states, n);
		if (ret) {
			DRM_DEBUG_KMS("[CRTC:%d:%s] cannot assign plane rscs\n",
					crtc->base.id, crtc->name);
			kfree(states);
			return ret;
		}

		kfree(states);
	}

	drm_for_each_crtc(crtc, dev) {
		if (pipe_states_prone_to_put[drm_crtc_index(crtc)]) {
			crtc_state = drm_atomic_get_crtc_state(state, crtc);
			if (WARN_ON(IS_ERR(crtc_state)))
				return PTR_ERR(crtc_state);

			dpu_atomic_put_possible_states_per_crtc(crtc_state);
		}
	}

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret) {
		DRM_DEBUG_KMS("%s: failed to check planes\n", __func__);
		return ret;
	}

	return ret;
}

const struct drm_mode_config_funcs dpu_drm_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = dpu_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};
