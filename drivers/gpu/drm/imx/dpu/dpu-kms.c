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

	return 0;
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

static int
dpu_atomic_assign_plane_source_per_crtc(struct drm_plane_state **states, int n)
{
	struct dpu_plane_state *dpstate;
	struct dpu_plane *dplane;
	struct dpu_plane_grp *grp;
	struct drm_framebuffer *fb;
	struct dpu_fetchdecode *fd;
	struct dpu_fetcheco *fe;
	struct dpu_fetchlayer *fl;
	struct dpu_fetchwarp *fw;
	struct dpu_hscaler *hs;
	struct dpu_vscaler *vs;
	unsigned int sid, src_sid;
	unsigned int num_planes;
	int i, j, k, l, m;
	int fu_id, fu_type;
	int total_asrc_num;
	u32 src_a_mask, cap_mask, fe_mask, hs_mask, vs_mask;
	bool need_fetcheco, need_hscaler, need_vscaler;
	bool fmt_is_yuv;

	/* for active planes only */
	for (i = 0; i < n; i++) {
		dpstate = to_dpu_plane_state(states[i]);
		dplane = to_dpu_plane(states[i]->plane);
		fb = states[i]->fb;
		num_planes = drm_format_num_planes(fb->format->format);
		fmt_is_yuv = drm_format_is_yuv(fb->format->format);
		grp = dplane->grp;
		sid = dplane->stream_id;

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

			fu_type = source_to_type(sources[m]);
			fu_id = source_to_id(sources[m]);

			switch (fu_type) {
			case DPU_PLANE_SRC_FL:
				fl = grp->res.fl[fu_id];

				if (fmt_is_yuv || need_fetcheco ||
				    need_hscaler || need_vscaler)
					goto next;

				/* avoid on-the-fly/hot migration */
				src_sid = fetchlayer_get_stream_id(fl);
				if (src_sid && src_sid != BIT(sid))
					goto next;
				break;
			case DPU_PLANE_SRC_FW:
				fw = grp->res.fw[fu_id];

				if (fmt_is_yuv || need_fetcheco ||
				    need_hscaler || need_vscaler)
					goto next;

				/* avoid on-the-fly/hot migration */
				src_sid = fetchwarp_get_stream_id(fw);
				if (src_sid && src_sid != BIT(sid))
					goto next;
				break;
			case DPU_PLANE_SRC_FD:
				fd = grp->res.fd[fu_id];

				/* avoid on-the-fly/hot migration */
				src_sid = fetchdecode_get_stream_id(fd);
				if (src_sid && src_sid != BIT(sid))
					goto next;

				cap_mask = fetchdecode_get_vproc_mask(fd);

				if (need_fetcheco) {
					fe = fetchdecode_get_fetcheco(fd);

					/* avoid on-the-fly/hot migration */
					src_sid = fetcheco_get_stream_id(fe);
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
					hs = fetchdecode_get_hscaler(fd);

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
					vs = fetchdecode_get_vscaler(fd);

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
				break;
			default:
				return -EINVAL;
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

		dpstate->source = sources[m];

		/* assign stage and blend */
		if (sid) {
			j = grp->hw_plane_num - (n - i);
			dpstate->stage = i ? stages[j - 1] : cf_stages[sid];
			dpstate->blend = blends[j];
		} else {
			dpstate->stage = i ? stages[i - 1] : cf_stages[sid];
			dpstate->blend = blends[i];
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
	const struct drm_plane_state *plane_state;
	struct dpu_plane_grp *grp[MAX_DPU_PLANE_GRP];
	int ret, i, grp_id;
	int active_plane[MAX_DPU_PLANE_GRP];
	int active_plane_fetcheco[MAX_DPU_PLANE_GRP];
	int active_plane_hscale[MAX_DPU_PLANE_GRP];
	int active_plane_vscale[MAX_DPU_PLANE_GRP];
	bool pipe_states_prone_to_put[MAX_CRTC];
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
		grp[i] = NULL;
	}

	for_each_crtc_in_state(state, crtc, crtc_state, i)
		crtc_mask_in_state |= drm_crtc_mask(crtc);

	drm_for_each_crtc(crtc, dev) {
		dpu_atomic_mark_pipe_states_prone_to_put_per_crtc(crtc,
						crtc_mask_in_state, state,
						pipe_states_prone_to_put);

		crtc_state = drm_atomic_get_crtc_state(state, crtc);
		if (IS_ERR(crtc_state))
			return PTR_ERR(crtc_state);

		drm_atomic_crtc_state_for_each_plane(plane, crtc_state)
			plane_state = drm_atomic_get_plane_state(state, plane);

		drm_atomic_crtc_state_for_each_plane_state(plane, plane_state,
							   crtc_state) {
			struct drm_framebuffer *fb = plane_state->fb;
			dpu_plane = to_dpu_plane(plane);
			grp_id = dpu_plane->grp->id;
			active_plane[grp_id]++;

			if (drm_format_num_planes(fb->format->format) > 1)
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

		ret = dpu_atomic_compute_plane_base_per_crtc(states, n);
		if (ret) {
			kfree(states);
			return ret;
		}

		dpu_atomic_set_top_plane_per_crtc(states, n);

		ret = dpu_atomic_assign_plane_source_per_crtc(states, n);
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

static int dpu_drm_atomic_commit(struct drm_device *dev,
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

	return drm_atomic_helper_commit(dev, state, nonblock);
}

const struct drm_mode_config_funcs dpu_drm_mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = dpu_drm_output_poll_changed,
	.atomic_check = dpu_drm_atomic_check,
	.atomic_commit = dpu_drm_atomic_commit,
};
