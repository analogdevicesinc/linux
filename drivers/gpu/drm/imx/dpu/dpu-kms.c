/*
 * Copyright 2017-2018 NXP
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
#include <linux/sort.h>
#include <video/dpu.h>
#include "dpu-crtc.h"
#include "dpu-plane.h"
#include "imx-drm.h"

static void dpu_drm_output_poll_changed(struct drm_device *dev)
{
	struct imx_drm_device *imxdrm = dev->dev_private;

	drm_fbdev_cma_hotplug_event(imxdrm->fbhelper);
}

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

static int
dpu_atomic_compute_plane_base_per_crtc(struct drm_crtc_state *crtc_state,
				       struct drm_plane_state **states, int n,
				       bool use_pc)
{
	struct dpu_plane_state *dpstate;
	int i, left, right, top, bottom, tmp;
	int base_x, base_y, base_w, base_h;
	int half_hdisplay = crtc_state->adjusted_mode.hdisplay >> 1;
	bool lo, ro, bo;

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

	/* store the base in plane state */
	dpstate = to_dpu_plane_state(states[0]);
	base_x = left;
	base_y = top;
	base_w = right - left;
	base_h = bottom - top;
	dpstate->base_x = base_x;
	dpstate->base_y = base_y;
	dpstate->base_w = base_w;
	dpstate->base_h = base_h;

	if (!use_pc)
		return 0;

	/* compute left/right_layer/base_x/w if pixel combiner is needed */
	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);

		lo =  dpstate->left_src_w && !dpstate->right_src_w;
		ro = !dpstate->left_src_w &&  dpstate->right_src_w;
		bo =  dpstate->left_src_w &&  dpstate->right_src_w;

		if (lo || bo) {
			dpstate->left_layer_x = dpstate->layer_x;
			dpstate->right_layer_x = 0;
		} else if (ro) {
			dpstate->left_layer_x = 0;
			dpstate->right_layer_x =
					states[i]->crtc_x - half_hdisplay;
		}

		if (i)
			continue;

		if (base_x < half_hdisplay) {
			dpstate->left_base_x = base_x;
			dpstate->right_base_x = 0;

			if ((base_x + base_w) < half_hdisplay) {
				dpstate->left_base_w = base_w;
				dpstate->right_base_w = 0;
			} else {
				dpstate->left_base_w = half_hdisplay - base_x;
				dpstate->right_base_w =
						base_x + base_w - half_hdisplay;
			}
		} else {
			dpstate->left_base_x = 0;
			dpstate->right_base_x = base_x - half_hdisplay;

			dpstate->left_base_w = 0;
			dpstate->right_base_w = base_w;
		}
	}

	return 0;
}

static void
dpu_atomic_set_top_plane_per_crtc(struct drm_plane_state **states, int n,
				  bool use_pc)
{
	struct dpu_plane_state *dpstate;
	bool found_l_top = false, found_r_top = false;
	int i;

	for (i = n - 1; i >= 0; i--) {
		dpstate = to_dpu_plane_state(states[i]);
		if (use_pc) {
			if (dpstate->left_src_w && !found_l_top) {
				dpstate->is_left_top = true;
				found_l_top = true;
			} else {
				dpstate->is_left_top = false;
			}

			if (dpstate->right_src_w && !found_r_top) {
				dpstate->is_right_top = true;
				found_r_top = true;
			} else {
				dpstate->is_right_top = false;
			}
		} else {
			dpstate->is_top = (i == (n - 1)) ? true : false;
		}
	}
}

static int
dpu_atomic_assign_plane_source_per_crtc(struct drm_plane_state **states,
					int n, bool use_pc)
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
	int i, j, k, l, m;
	int total_asrc_num;
	int s0_layer_cnt = 0, s1_layer_cnt = 0;
	int s0_n = 0, s1_n = 0;
	u32 src_a_mask, cap_mask, fe_mask, hs_mask, vs_mask;
	bool need_fetcheco, need_hscaler, need_vscaler;
	bool fmt_is_yuv;
	bool alloc_aux_source;

	if (use_pc) {
		for (i = 0; i < n; i++) {
			dpstate = to_dpu_plane_state(states[i]);

			if (dpstate->left_src_w)
				s0_n++;

			if (dpstate->right_src_w)
				s1_n++;
		}
	} else {
		s0_n = n;
		s1_n = n;
	}

	/* for active planes only */
	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);
		dplane = to_dpu_plane(states[i]->plane);
		fb = states[i]->fb;
		num_planes = drm_format_num_planes(fb->format->format);
		fmt_is_yuv = drm_format_is_yuv(fb->format->format);
		grp = dplane->grp;
		alloc_aux_source = false;

		if (use_pc)
			sid = dpstate->left_src_w ? 0 : 1;
		else
			sid = dplane->stream_id;

again:
		if (alloc_aux_source)
			sid ^= 1;

		need_fetcheco = (num_planes > 1);
		need_hscaler = (states[i]->src_w >> 16 != states[i]->crtc_w);
		need_vscaler = (states[i]->src_h >> 16 != states[i]->crtc_h);

		total_asrc_num = 0;
		src_a_mask = grp->src_a_mask;
		fe_mask = 0;
		hs_mask = 0;
		vs_mask = 0;

		for (l = 0; l < (sizeof(grp->src_a_mask) * 8); l++) {
			if (grp->src_a_mask & BIT(l))
				total_asrc_num++;
		}

		/* assign source */
		mutex_lock(&grp->mutex);
		for (k = 0; k < total_asrc_num; k++) {
			m = ffs(src_a_mask) - 1;

			fu = source_to_fu(&grp->res, sources[m]);
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

			grp->src_a_mask &= ~BIT(m);
			grp->src_use_vproc_mask |= fe_mask | hs_mask | vs_mask;
			break;
next:
			src_a_mask &= ~BIT(m);
			fe_mask = 0;
			hs_mask = 0;
			vs_mask = 0;
		}
		mutex_unlock(&grp->mutex);

		if (k == total_asrc_num)
			return -EINVAL;

		if (alloc_aux_source)
			dpstate->aux_source = sources[m];
		else
			dpstate->source = sources[m];

		/* assign stage and blend */
		if (sid) {
			j = grp->hw_plane_num - (s1_n - s1_layer_cnt);
			stage = s1_layer_cnt ? stages[j - 1] : cf_stages[sid];
			blend = blends[j];

			s1_layer_cnt++;
		} else {
			stage = s0_layer_cnt ?
				stages[s0_layer_cnt - 1] : cf_stages[sid];
			blend = blends[s0_layer_cnt];

			s0_layer_cnt++;
		}

		if (alloc_aux_source) {
			dpstate->aux_stage = stage;
			dpstate->aux_blend = blend;
		} else {
			dpstate->stage = stage;
			dpstate->blend = blend;
		}

		if (dpstate->need_aux_source && !alloc_aux_source) {
			alloc_aux_source = true;
			goto again;
		}
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
		for_each_plane_in_state(state, plane, plane_state, i) {
			if (plane->possible_crtcs &
			    drm_crtc_mask(crtc)) {
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
			WARN_ON(!plane_state);

			new_dpstate = to_dpu_plane_state(plane_state);

			active_mask &= ~BIT(i);

			/*
			 * Should be enough to check the below real HW plane
			 * resources only.
			 * Vproc resources and things like layer_x/y should
			 * be fine.
			 */
			if (old_dpstate->stage  != new_dpstate->stage ||
			    old_dpstate->source != new_dpstate->source ||
			    old_dpstate->blend  != new_dpstate->blend ||
			    old_dpstate->aux_stage  != new_dpstate->aux_stage ||
			    old_dpstate->aux_source != new_dpstate->aux_source ||
			    old_dpstate->aux_blend  != new_dpstate->aux_blend)
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
	int half_hdisplay = 0;
	bool pipe_states_prone_to_put[MAX_CRTC];
	bool use_pc[MAX_DPU_PLANE_GRP];
	u32 crtc_mask_in_state = 0;

	ret = drm_atomic_helper_check_modeset(dev, state);
	if (ret)
		return ret;

	for (i = 0; i < MAX_CRTC; i++)
		pipe_states_prone_to_put[i] = false;

	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		active_plane[i] = 0;
		active_plane_fetcheco[i] = 0;
		active_plane_hscale[i] = 0;
		active_plane_vscale[i] = 0;
		use_pc[i] = false;
		grp[i] = NULL;
	}

	for_each_crtc_in_state(state, crtc, crtc_state, i)
		crtc_mask_in_state |= drm_crtc_mask(crtc);

	drm_for_each_crtc(crtc, dev) {
		struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
		struct imx_crtc_state *imx_crtc_state;
		struct dpu_crtc_state *dcstate;
		bool need_left, need_right, need_aux_source, use_pc_per_crtc;

		use_pc_per_crtc = false;

		dpu_atomic_mark_pipe_states_prone_to_put_per_crtc(crtc,
						crtc_mask_in_state, state,
						pipe_states_prone_to_put);

		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		imx_crtc_state = to_imx_crtc_state(crtc_state);
		dcstate = to_dpu_crtc_state(imx_crtc_state);

		if (crtc_state->enable) {
			if (use_pc[dpu_crtc->crtc_grp_id])
				return -EINVAL;

			if (crtc_state->adjusted_mode.clock >
					dpu_crtc->syncmode_min_prate ||
			    crtc_state->adjusted_mode.hdisplay >
					dpu_crtc->singlemode_max_width) {
				if (!dpu_crtc->has_pc)
					return -EINVAL;

				use_pc_per_crtc = true;
			}
		}

		if (use_pc_per_crtc) {
			use_pc[dpu_crtc->crtc_grp_id] = true;
			half_hdisplay = crtc_state->adjusted_mode.hdisplay >> 1;
		}

		dcstate->use_pc = use_pc_per_crtc;

		drm_for_each_plane_mask(plane, dev, crtc_state->plane_mask) {
			plane_state = drm_atomic_get_plane_state(state, plane);
			dpstate = to_dpu_plane_state(plane_state);
			fb = plane_state->fb;
			dpu_plane = to_dpu_plane(plane);
			grp_id = dpu_plane->grp->id;
			active_plane[grp_id]++;

			need_left = false;
			need_right = false;
			need_aux_source = false;

			if (use_pc_per_crtc) {
				if (plane_state->crtc_x < half_hdisplay)
					need_left = true;

				if ((plane_state->crtc_w +
				     plane_state->crtc_x) > half_hdisplay)
					need_right = true;

				if (need_left && need_right) {
					need_aux_source = true;
					active_plane[grp_id]++;
				}
			}

			if (need_left && need_right) {
				dpstate->left_crtc_w = half_hdisplay;
				dpstate->left_crtc_w -= plane_state->crtc_x;

				dpstate->left_src_w = dpstate->left_crtc_w;
			} else if (need_left) {
				dpstate->left_crtc_w = plane_state->crtc_w;
				dpstate->left_src_w = plane_state->src_w >> 16;
			} else {
				dpstate->left_crtc_w = 0;
				dpstate->left_src_w = 0;
			}

			if (need_right && need_left) {
				dpstate->right_crtc_w = plane_state->crtc_x +
					plane_state->crtc_w;
				dpstate->right_crtc_w -= half_hdisplay;

				dpstate->right_src_w = dpstate->right_crtc_w;
			} else if (need_right) {
				dpstate->right_crtc_w = plane_state->crtc_w;
				dpstate->right_src_w = plane_state->src_w >> 16;
			} else {
				dpstate->right_crtc_w = 0;
				dpstate->right_src_w = 0;
			}

			if (drm_format_num_planes(fb->format->format) > 1) {
				active_plane_fetcheco[grp_id]++;
				if (need_aux_source)
					active_plane_fetcheco[grp_id]++;
			}

			if (plane_state->src_w >> 16 != plane_state->crtc_w) {
				if (use_pc_per_crtc)
					return -EINVAL;

				active_plane_hscale[grp_id]++;
			}

			if (plane_state->src_h >> 16 != plane_state->crtc_h) {
				if (use_pc_per_crtc)
					return -EINVAL;

				active_plane_vscale[grp_id]++;
			}

			if (grp[grp_id] == NULL)
				grp[grp_id] = dpu_plane->grp;

			dpstate->need_aux_source = need_aux_source;
		}
	}

	/* enough resources? */
	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		if (grp[i]) {
			if (active_plane[i] > grp[i]->hw_plane_num)
				return -EINVAL;

			if (active_plane_fetcheco[i] >
			    grp[i]->hw_plane_fetcheco_num)
				return -EINVAL;

			if (active_plane_hscale[i] >
			    grp[i]->hw_plane_hscaler_num)
				return -EINVAL;

			if (active_plane_vscale[i] >
			    grp[i]->hw_plane_vscaler_num)
				return -EINVAL;
		}
	}

	/* clear resource mask */
	for (i = 0; i < MAX_DPU_PLANE_GRP; i++) {
		if (grp[i]) {
			mutex_lock(&grp[i]->mutex);
			grp[i]->src_a_mask = ~grp[i]->src_na_mask;
			grp[i]->src_use_vproc_mask = 0;
			mutex_unlock(&grp[i]->mutex);
		}
	}

	ret = drm_atomic_normalize_zpos(dev, state);
	if (ret)
		return ret;

	for_each_crtc_in_state(state, crtc, crtc_state, i) {
		struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
		struct drm_plane_state **states;
		int n;

		states = dpu_atomic_alloc_tmp_planes_per_crtc(dev);
		if (IS_ERR(states))
			return PTR_ERR(states);

		n = dpu_atomic_sort_planes_per_crtc(crtc_state, states);
		if (n < 0) {
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
			kfree(states);
			return -EINVAL;
		}

		ret = dpu_atomic_compute_plane_base_per_crtc(crtc_state, states,
					n, use_pc[dpu_crtc->crtc_grp_id]);
		if (ret) {
			kfree(states);
			return ret;
		}

		dpu_atomic_set_top_plane_per_crtc(states, n,
					use_pc[dpu_crtc->crtc_grp_id]);

		ret = dpu_atomic_assign_plane_source_per_crtc(states, n,
						use_pc[dpu_crtc->crtc_grp_id]);
		if (ret) {
			kfree(states);
			return ret;
		}

		kfree(states);

		if (pipe_states_prone_to_put[drm_crtc_index(crtc)])
			dpu_atomic_put_possible_states_per_crtc(crtc_state);
	}

	ret = drm_atomic_helper_check_planes(dev, state);
	if (ret)
		return ret;

	return ret;
}

const struct drm_mode_config_funcs dpu_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = dpu_drm_output_poll_changed,
	.atomic_check = dpu_drm_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};
