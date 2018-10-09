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
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <video/dpu.h>
#include <video/imx8-pc.h>
#include "dpu-crtc.h"
#include "dpu-kms.h"
#include "dpu-plane.h"
#include "imx-drm.h"

static inline struct dpu_plane_state **
alloc_dpu_plane_states(struct dpu_crtc *dpu_crtc)
{
	struct dpu_plane_state **states;

	states = kcalloc(dpu_crtc->hw_plane_num, sizeof(*states), GFP_KERNEL);
	if (!states)
		return ERR_PTR(-ENOMEM);

	return states;
}

struct dpu_plane_state **
crtc_state_get_dpu_plane_states(struct drm_crtc_state *state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);

	return dcstate->dpu_plane_states;
}

struct dpu_crtc *dpu_crtc_get_aux_dpu_crtc(struct dpu_crtc *dpu_crtc)
{
	struct drm_crtc *crtc = &dpu_crtc->base, *tmp_crtc;
	struct drm_device *dev = crtc->dev;
	struct dpu_crtc *aux_dpu_crtc = NULL;

	drm_for_each_crtc(tmp_crtc, dev) {
		if (tmp_crtc == crtc)
			continue;

		aux_dpu_crtc = to_dpu_crtc(tmp_crtc);

		if (dpu_crtc->crtc_grp_id == aux_dpu_crtc->crtc_grp_id)
			break;
	}

	BUG_ON(!aux_dpu_crtc);

	return aux_dpu_crtc;
}

static void dpu_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_extdst *plane_ed = res->ed[dplane->stream_id];
	struct dpu_extdst *aux_plane_ed = dpu_aux_ed_peek(plane_ed);
	struct dpu_extdst *m_plane_ed = NULL, *s_plane_ed;
	struct completion *shdld_done;
	struct completion *m_safety_shdld_done, *s_safety_shdld_done;
	struct completion *m_content_shdld_done, *s_content_shdld_done;
	struct completion *m_dec_shdld_done, *s_dec_shdld_done;
	unsigned long ret;

	drm_crtc_vblank_on(crtc);

	if (dcstate->use_pc) {
		tcon_enable_pc(dpu_crtc->tcon);

		if (extdst_is_master(plane_ed)) {
			m_plane_ed = plane_ed;
			s_plane_ed = aux_plane_ed;
		} else {
			m_plane_ed = aux_plane_ed;
			s_plane_ed = plane_ed;
		}
		extdst_pixengcfg_syncmode_master(m_plane_ed, true);
		extdst_pixengcfg_syncmode_master(s_plane_ed, false);
	} else {
		extdst_pixengcfg_syncmode_master(plane_ed, false);
	}

	enable_irq(dpu_crtc->safety_shdld_irq);
	enable_irq(dpu_crtc->content_shdld_irq);
	enable_irq(dpu_crtc->dec_shdld_irq);
	if (dcstate->use_pc) {
		enable_irq(aux_dpu_crtc->safety_shdld_irq);
		enable_irq(aux_dpu_crtc->content_shdld_irq);
		enable_irq(aux_dpu_crtc->dec_shdld_irq);
	}

	if (dcstate->use_pc) {
		framegen_enable_clock(dpu_crtc->stream_id ?
					dpu_crtc->aux_fg : dpu_crtc->fg);
		extdst_pixengcfg_sync_trigger(m_plane_ed);
		framegen_shdtokgen(dpu_crtc->m_fg);

		/* First turn on the slave stream, second the master stream. */
		framegen_enable(dpu_crtc->s_fg);
		framegen_enable(dpu_crtc->m_fg);

		if (dpu_crtc->stream_id) {
			m_safety_shdld_done  = &aux_dpu_crtc->safety_shdld_done;
			m_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
			m_dec_shdld_done     = &aux_dpu_crtc->dec_shdld_done;
			s_safety_shdld_done  = &dpu_crtc->safety_shdld_done;
			s_content_shdld_done = &dpu_crtc->content_shdld_done;
			s_dec_shdld_done     = &dpu_crtc->dec_shdld_done;
		} else {
			m_safety_shdld_done  = &dpu_crtc->safety_shdld_done;
			m_content_shdld_done = &dpu_crtc->content_shdld_done;
			m_dec_shdld_done     = &dpu_crtc->dec_shdld_done;
			s_safety_shdld_done  = &aux_dpu_crtc->safety_shdld_done;
			s_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
			s_dec_shdld_done     = &aux_dpu_crtc->dec_shdld_done;
		}

		ret = wait_for_completion_timeout(m_safety_shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for master safety shdld done timeout\n");
		ret = wait_for_completion_timeout(m_content_shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for master content shdld done timeout\n");
		ret = wait_for_completion_timeout(m_dec_shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for master dec shdld done timeout\n");

		ret = wait_for_completion_timeout(s_safety_shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for slave safety shdld done timeout\n");
		ret = wait_for_completion_timeout(s_content_shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for slave content shdld done timeout\n");
		ret = wait_for_completion_timeout(s_dec_shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for slave DEC shdld done timeout\n");
	} else {
		framegen_enable_clock(dpu_crtc->fg);
		extdst_pixengcfg_sync_trigger(plane_ed);
		extdst_pixengcfg_sync_trigger(dpu_crtc->ed);
		framegen_shdtokgen(dpu_crtc->fg);
		framegen_enable(dpu_crtc->fg);

		shdld_done = &dpu_crtc->safety_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for safety shdld done timeout\n");
		shdld_done = &dpu_crtc->content_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for content shdld done timeout\n");
		shdld_done = &dpu_crtc->dec_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "enable - wait for dec shdld done timeout\n");
	}

	disable_irq(dpu_crtc->safety_shdld_irq);
	disable_irq(dpu_crtc->content_shdld_irq);
	disable_irq(dpu_crtc->dec_shdld_irq);
	if (dcstate->use_pc) {
		disable_irq(aux_dpu_crtc->safety_shdld_irq);
		disable_irq(aux_dpu_crtc->content_shdld_irq);
		disable_irq(aux_dpu_crtc->dec_shdld_irq);
	}

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}

	/*
	 * TKT320590:
	 * Turn TCON into operation mode later after the first dumb frame is
	 * generated by DPU.  This makes DPR/PRG be able to evade the frame.
	 */
	if (dcstate->use_pc) {
		framegen_wait_for_frame_counter_moving(dpu_crtc->m_fg);
		tcon_set_operation_mode(dpu_crtc->m_tcon);
		framegen_wait_for_frame_counter_moving(dpu_crtc->s_fg);
		tcon_set_operation_mode(dpu_crtc->s_tcon);

		framegen_wait_for_secondary_syncup(dpu_crtc->m_fg);
		framegen_wait_for_secondary_syncup(dpu_crtc->s_fg);
	} else {
		framegen_wait_for_frame_counter_moving(dpu_crtc->fg);
		tcon_set_operation_mode(dpu_crtc->tcon);
	}
}

static void dpu_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *imx_crtc_state =
					to_imx_crtc_state(old_crtc_state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct drm_display_mode *adjusted_mode = &old_crtc_state->adjusted_mode;

	if (dcstate->use_pc) {
		tcon_disable_pc(dpu_crtc->tcon);

		/* First turn off the master stream, second the slave stream. */
		framegen_disable(dpu_crtc->m_fg);
		framegen_disable(dpu_crtc->s_fg);

		framegen_wait_done(dpu_crtc->m_fg, adjusted_mode);
		framegen_wait_done(dpu_crtc->s_fg, adjusted_mode);

		framegen_disable_clock(dpu_crtc->stream_id ?
					dpu_crtc->aux_fg : dpu_crtc->fg);
	} else {
		framegen_disable(dpu_crtc->fg);
		framegen_wait_done(dpu_crtc->fg, adjusted_mode);
		framegen_disable_clock(dpu_crtc->fg);
	}

	WARN_ON(!crtc->state->event);

	if (crtc->state->event) {
		spin_lock_irq(&crtc->dev->event_lock);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		spin_unlock_irq(&crtc->dev->event_lock);

		crtc->state->event = NULL;
	}

	drm_crtc_vblank_off(crtc);
}

static void dpu_drm_crtc_reset(struct drm_crtc *crtc)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *imx_crtc_state;
	struct dpu_crtc_state *state;

	if (crtc->state) {
		__drm_atomic_helper_crtc_destroy_state(crtc->state);

		imx_crtc_state = to_imx_crtc_state(crtc->state);
		state = to_dpu_crtc_state(imx_crtc_state);
		kfree(state->dpu_plane_states);
		kfree(state);
		crtc->state = NULL;
	}

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state) {
		crtc->state = &state->imx_crtc_state.base;
		crtc->state->crtc = crtc;

		state->dpu_plane_states = alloc_dpu_plane_states(dpu_crtc);
		if (IS_ERR(state->dpu_plane_states))
			kfree(state);
	}
}

static struct drm_crtc_state *
dpu_drm_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct imx_crtc_state *imx_crtc_state;
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_crtc_state *state, *copy;

	if (WARN_ON(!crtc->state))
		return NULL;

	copy = kzalloc(sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return NULL;

	copy->dpu_plane_states = alloc_dpu_plane_states(dpu_crtc);
	if (IS_ERR(copy->dpu_plane_states)) {
		kfree(copy);
		return NULL;
	}

	__drm_atomic_helper_crtc_duplicate_state(crtc,
					&copy->imx_crtc_state.base);
	imx_crtc_state = to_imx_crtc_state(crtc->state);
	state = to_dpu_crtc_state(imx_crtc_state);
	copy->use_pc = state->use_pc;

	return &copy->imx_crtc_state.base;
}

static void dpu_drm_crtc_destroy_state(struct drm_crtc *crtc,
				       struct drm_crtc_state *state)
{
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(state);
	struct dpu_crtc_state *dcstate;

	if (state) {
		__drm_atomic_helper_crtc_destroy_state(state);
		dcstate = to_dpu_crtc_state(imx_crtc_state);
		kfree(dcstate->dpu_plane_states);
		kfree(dcstate);
	}
}

static int dpu_enable_vblank(struct drm_crtc *crtc)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);

	enable_irq(dpu_crtc->vbl_irq);

	return 0;
}

static void dpu_disable_vblank(struct drm_crtc *crtc)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);

	disable_irq_nosync(dpu_crtc->vbl_irq);
}

static const struct drm_crtc_funcs dpu_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.destroy = drm_crtc_cleanup,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = dpu_drm_crtc_reset,
	.atomic_duplicate_state = dpu_drm_crtc_duplicate_state,
	.atomic_destroy_state = dpu_drm_crtc_destroy_state,
	.enable_vblank = dpu_enable_vblank,
	.disable_vblank = dpu_disable_vblank,
};

static irqreturn_t dpu_vbl_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	drm_crtc_handle_vblank(&dpu_crtc->base);

	return IRQ_HANDLED;
}

static irqreturn_t dpu_safety_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	complete(&dpu_crtc->safety_shdld_done);

	return IRQ_HANDLED;
}

static irqreturn_t dpu_content_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	complete(&dpu_crtc->content_shdld_done);

	return IRQ_HANDLED;
}

static irqreturn_t dpu_dec_shdld_irq_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;

	complete(&dpu_crtc->dec_shdld_done);

	return IRQ_HANDLED;
}

static int dpu_crtc_atomic_check(struct drm_crtc *crtc,
				 struct drm_crtc_state *crtc_state)
{
	struct drm_device *dev = crtc->dev;
	struct drm_encoder *encoder;
	struct drm_plane *plane;
	struct drm_plane_state *plane_state;
	struct dpu_plane_state *dpstate;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc_state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	struct videomode vm;
	unsigned long encoder_types = 0;
	u32 encoder_mask;
	int i = 0;

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		encoder_mask = 1 << drm_encoder_index(encoder);

		if (!(crtc_state->encoder_mask & encoder_mask))
			continue;

		encoder_types |= BIT(encoder->encoder_type);
	}

	if (crtc_state->enable && dcstate->use_pc) {
		if (encoder_types & BIT(DRM_MODE_ENCODER_LVDS))
			return -EINVAL;

		if (encoder_types & BIT(DRM_MODE_ENCODER_DSI))
			return -EINVAL;

		drm_display_mode_to_videomode(mode, &vm);
		if ((vm.hactive % 2)   || (vm.hfront_porch % 2) ||
		    (vm.hsync_len % 2) || (vm.hback_porch % 2))
			return -EINVAL;
	}

	/*
	 * cache the plane states so that the planes can be disabled in
	 * ->atomic_begin.
	 */
	drm_for_each_plane_mask(plane, crtc->dev, crtc_state->plane_mask) {
		plane_state =
			drm_atomic_get_plane_state(crtc_state->state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);

		dpstate = to_dpu_plane_state(plane_state);
		dcstate->dpu_plane_states[i++] = dpstate;
	}

	return 0;
}

static void dpu_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *imx_crtc_state =
					to_imx_crtc_state(old_crtc_state);
	struct dpu_crtc_state *old_dcstate = to_dpu_crtc_state(imx_crtc_state);
	int i;

	/*
	 * Disable all planes' resources in SHADOW only.
	 * Whether any of them would be disabled or kept running depends
	 * on new plane states' commit.
	 */
	for (i = 0; i < dpu_crtc->hw_plane_num; i++) {
		struct dpu_plane_state *old_dpstate;
		struct drm_plane_state *plane_state;
		struct dpu_plane *dplane;
		struct drm_plane *plane;
		struct dpu_plane_res *res;
		struct dpu_fetchunit *fu;
		struct dpu_fetchunit *fe = NULL;
		struct dpu_hscaler *hs = NULL;
		struct dpu_vscaler *vs = NULL;
		struct dpu_layerblend *lb;
		struct dpu_extdst *ed;
		extdst_src_sel_t ed_src;
		dpu_block_id_t blend, source;
		unsigned int stream_id;
		int lb_id;
		bool crtc_disabling_on_primary;
		bool release_aux_source;

		old_dpstate = old_dcstate->dpu_plane_states[i];
		if (!old_dpstate)
			continue;

		plane_state = &old_dpstate->base;
		dplane = to_dpu_plane(plane_state->plane);
		res = &dplane->grp->res;

		release_aux_source = false;
again:
		crtc_disabling_on_primary = false;

		if (old_dcstate->use_pc) {
			if (release_aux_source) {
				source = old_dpstate->aux_source;
				blend = old_dpstate->aux_blend;
				stream_id = 1;
			} else {
				source = old_dpstate->source;
				blend = old_dpstate->blend;
				stream_id = old_dpstate->left_src_w ? 0 : 1;
			}
		} else {
			source = old_dpstate->source;
			blend = old_dpstate->blend;
			stream_id = dplane->stream_id;
		}

		fu = source_to_fu(res, source);
		if (!fu)
			return;

		lb_id = blend_to_id(blend);
		if (lb_id < 0)
			return;

		lb = res->lb[lb_id];

		layerblend_pixengcfg_clken(lb, CLKEN__DISABLE);
		if (fetchunit_is_fetchdecode(fu)) {
			fe = fetchdecode_get_fetcheco(fu);
			hs = fetchdecode_get_hscaler(fu);
			vs = fetchdecode_get_vscaler(fu);
			hscaler_pixengcfg_clken(hs, CLKEN__DISABLE);
			vscaler_pixengcfg_clken(vs, CLKEN__DISABLE);
			hscaler_mode(hs, SCALER_NEUTRAL);
			vscaler_mode(vs, SCALER_NEUTRAL);
		}
		if (old_dpstate->is_top) {
			ed = res->ed[stream_id];
			ed_src = stream_id ?
				ED_SRC_CONSTFRAME1 : ED_SRC_CONSTFRAME0;
			extdst_pixengcfg_src_sel(ed, ed_src);
		}

		plane = old_dpstate->base.plane;
		if (!crtc->state->enable &&
		    plane->type == DRM_PLANE_TYPE_PRIMARY)
			crtc_disabling_on_primary = true;

		if (crtc_disabling_on_primary && old_dpstate->use_prefetch) {
			fu->ops->pin_off(fu);
			if (fetchunit_is_fetchdecode(fu) &&
			    fe->ops->is_enabled(fe))
				fe->ops->pin_off(fe);
		} else {
			fu->ops->disable_src_buf(fu);
			fu->ops->unpin_off(fu);
			if (fetchunit_is_fetchdecode(fu)) {
				fetchdecode_pixengcfg_dynamic_src_sel(fu,
								FD_SRC_DISABLE);
				fe->ops->disable_src_buf(fe);
				fe->ops->unpin_off(fe);
			}
		}

		if (old_dpstate->need_aux_source && !release_aux_source) {
			release_aux_source = true;
			goto again;
		}
	}
}

static void dpu_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_crtc_state *old_crtc_state)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc), *aux_dpu_crtc = NULL;
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct imx_crtc_state *old_imx_crtc_state =
					to_imx_crtc_state(old_crtc_state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct dpu_crtc_state *old_dcstate =
					to_dpu_crtc_state(old_imx_crtc_state);
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_extdst *ed = res->ed[dplane->stream_id], *aux_ed;
	struct completion *shdld_done;
	struct completion *m_content_shdld_done = NULL;
	struct completion *s_content_shdld_done = NULL;
	unsigned long ret;
	int i;
	bool need_modeset = drm_atomic_crtc_needs_modeset(crtc->state);

	if (!crtc->state->active && !old_crtc_state->active)
		return;

	if (dcstate->use_pc) {
		aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);

		if (dpu_crtc->stream_id) {
			m_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
			s_content_shdld_done = &dpu_crtc->content_shdld_done;
		} else {
			m_content_shdld_done = &dpu_crtc->content_shdld_done;
			s_content_shdld_done = &aux_dpu_crtc->content_shdld_done;
		}
	}

	if (!need_modeset) {
		enable_irq(dpu_crtc->content_shdld_irq);
		if (dcstate->use_pc)
			enable_irq(aux_dpu_crtc->content_shdld_irq);

		if (dcstate->use_pc) {
			if (extdst_is_master(ed)) {
				extdst_pixengcfg_sync_trigger(ed);
			} else {
				aux_ed = dpu_aux_ed_peek(ed);
				extdst_pixengcfg_sync_trigger(aux_ed);
			}
		} else {
			extdst_pixengcfg_sync_trigger(ed);
		}

		if (dcstate->use_pc) {
			shdld_done = m_content_shdld_done;
			ret = wait_for_completion_timeout(shdld_done, HZ);
			if (ret == 0)
				dev_warn(dpu_crtc->dev,
				      "flush - wait for master content shdld done timeout\n");

			shdld_done = s_content_shdld_done;
			ret = wait_for_completion_timeout(shdld_done, HZ);
			if (ret == 0)
				dev_warn(dpu_crtc->dev,
					"flush - wait for slave content shdld done timeout\n");
		} else {
			shdld_done = &dpu_crtc->content_shdld_done;
			ret = wait_for_completion_timeout(shdld_done, HZ);
			if (ret == 0)
				dev_warn(dpu_crtc->dev,
				      "flush - wait for content shdld done timeout\n");
		}

		disable_irq(dpu_crtc->content_shdld_irq);
		if (dcstate->use_pc)
			disable_irq(aux_dpu_crtc->content_shdld_irq);

		WARN_ON(!crtc->state->event);

		if (crtc->state->event) {
			spin_lock_irq(&crtc->dev->event_lock);
			drm_crtc_send_vblank_event(crtc, crtc->state->event);
			spin_unlock_irq(&crtc->dev->event_lock);

			crtc->state->event = NULL;
		}
	} else if (!crtc->state->active) {
		if (old_dcstate->use_pc) {
			if (extdst_is_master(ed)) {
				extdst_pixengcfg_sync_trigger(ed);
			} else {
				aux_ed = dpu_aux_ed_peek(ed);
				extdst_pixengcfg_sync_trigger(aux_ed);
			}
		} else {
			extdst_pixengcfg_sync_trigger(ed);
		}
	}

	for (i = 0; i < dpu_crtc->hw_plane_num; i++) {
		struct dpu_plane_state *old_dpstate;
		struct dpu_fetchunit *fu;
		struct dpu_fetchunit *fe;
		struct dpu_hscaler *hs;
		struct dpu_vscaler *vs;
		dpu_block_id_t source;
		bool aux_source_disable;

		old_dpstate = old_dcstate->dpu_plane_states[i];
		if (!old_dpstate)
			continue;

		aux_source_disable = false;
again:
		source = aux_source_disable ?
				old_dpstate->aux_source : old_dpstate->source;
		fu = source_to_fu(res, source);
		if (!fu)
			return;

		if (!fu->ops->is_enabled(fu) || fu->ops->is_pinned_off(fu))
			fu->ops->set_stream_id(fu, DPU_PLANE_SRC_DISABLED);

		if (fetchunit_is_fetchdecode(fu)) {
			fe = fetchdecode_get_fetcheco(fu);
			if (!fe->ops->is_enabled(fe) ||
			     fe->ops->is_pinned_off(fe))
				fe->ops->set_stream_id(fe,
							DPU_PLANE_SRC_DISABLED);

			hs = fetchdecode_get_hscaler(fu);
			if (!hscaler_is_enabled(hs))
				hscaler_set_stream_id(hs,
							DPU_PLANE_SRC_DISABLED);

			vs = fetchdecode_get_vscaler(fu);
			if (!vscaler_is_enabled(vs))
				vscaler_set_stream_id(vs,
							DPU_PLANE_SRC_DISABLED);
		}

		if (old_dpstate->need_aux_source && !aux_source_disable) {
			aux_source_disable = true;
			goto again;
		}
	}
}

static void dpu_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct drm_encoder *encoder;
	struct dpu_plane *dplane = to_dpu_plane(crtc->primary);
	struct dpu_plane_res *res = &dplane->grp->res;
	struct dpu_constframe *cf;
	struct dpu_disengcfg *dec;
	struct dpu_extdst *ed, *plane_ed;
	struct dpu_framegen *fg;
	struct dpu_tcon *tcon;
	struct dpu_store *st;
	extdst_src_sel_t ed_src;
	unsigned long encoder_types = 0;
	u32 encoder_mask;
	unsigned int stream_id;
	int crtc_hdisplay = dcstate->use_pc ?
			(mode->crtc_hdisplay >> 1) : mode->crtc_hdisplay;
	bool encoder_type_has_tmds = false;
	bool encoder_type_has_lvds = false;
	bool cfg_aux_pipe = false;

	dev_dbg(dpu_crtc->dev, "%s: mode->hdisplay: %d\n", __func__,
			mode->hdisplay);
	dev_dbg(dpu_crtc->dev, "%s: mode->vdisplay: %d\n", __func__,
			mode->vdisplay);
	if (dcstate->use_pc)
		dev_dbg(dpu_crtc->dev, "%s: use pixel combiner\n", __func__);

	list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
		encoder_mask = 1 << drm_encoder_index(encoder);

		if (!(crtc->state->encoder_mask & encoder_mask))
			continue;

		encoder_types |= BIT(encoder->encoder_type);
	}

	if (encoder_types & BIT(DRM_MODE_ENCODER_TMDS)) {
		encoder_type_has_tmds = true;
		dev_dbg(dpu_crtc->dev, "%s: encoder type has TMDS\n", __func__);
	}

	if (encoder_types & BIT(DRM_MODE_ENCODER_LVDS)) {
		encoder_type_has_lvds = true;
		dev_dbg(dpu_crtc->dev, "%s: encoder type has LVDS\n", __func__);
	}

again:
	if (cfg_aux_pipe) {
		cf = dpu_crtc->aux_cf;
		dec = dpu_crtc->aux_dec;
		ed = dpu_crtc->aux_ed;
		fg = dpu_crtc->aux_fg;
		tcon = dpu_crtc->aux_tcon;
		st = aux_dpu_crtc->st;
		stream_id = dpu_crtc->stream_id ^ 1;
	} else {
		cf = dpu_crtc->cf;
		dec = dpu_crtc->dec;
		ed = dpu_crtc->ed;
		fg = dpu_crtc->fg;
		tcon = dpu_crtc->tcon;
		st = dpu_crtc->st;
		stream_id = dpu_crtc->stream_id;
	}

	if (dcstate->use_pc) {
		store_pixengcfg_syncmode_fixup(st, true);
		framegen_syncmode_fixup(fg,
				framegen_is_master(fg) ? false : true);
		framegen_syncmode(fg, framegen_is_master(fg) ?
				FGSYNCMODE__MASTER : FGSYNCMODE__SLAVE_ONCE);
	} else {
		store_pixengcfg_syncmode_fixup(st, false);
		framegen_syncmode_fixup(fg, false);
		framegen_syncmode(fg, FGSYNCMODE__OFF);
	}

	framegen_cfg_videomode(fg, mode, dcstate->use_pc,
			encoder_type_has_tmds, encoder_type_has_lvds);
	framegen_displaymode(fg, FGDM__SEC_ON_TOP);

	framegen_panic_displaymode(fg, FGDM__TEST);

	tcon_cfg_videomode(tcon, mode, dcstate->use_pc);
	tcon_set_fmt(tcon, imx_crtc_state->bus_format);
	if (dpu_crtc->has_pc)
		tcon_configure_pc(tcon, stream_id, mode->crtc_hdisplay,
				dcstate->use_pc ? PC_COMBINE : PC_BYPASS, 0);

	disengcfg_polarity_ctrl(dec, mode->flags);

	constframe_framedimensions(cf, crtc_hdisplay, mode->crtc_vdisplay);

	ed_src = stream_id ? ED_SRC_CONSTFRAME5 : ED_SRC_CONSTFRAME4;
	extdst_pixengcfg_src_sel(ed, ed_src);

	plane_ed = res->ed[stream_id];
	ed_src = stream_id ? ED_SRC_CONSTFRAME1 : ED_SRC_CONSTFRAME0;
	extdst_pixengcfg_src_sel(plane_ed, ed_src);

	if (dcstate->use_pc && !cfg_aux_pipe) {
		cfg_aux_pipe = true;
		goto again;
	}
}

static const struct drm_crtc_helper_funcs dpu_helper_funcs = {
	.mode_set_nofb = dpu_crtc_mode_set_nofb,
	.atomic_check = dpu_crtc_atomic_check,
	.atomic_begin = dpu_crtc_atomic_begin,
	.atomic_flush = dpu_crtc_atomic_flush,
	.atomic_enable = dpu_crtc_atomic_enable,
	.atomic_disable = dpu_crtc_atomic_disable,
};

static void dpu_crtc_put_resources(struct dpu_crtc *dpu_crtc)
{
	if (!IS_ERR_OR_NULL(dpu_crtc->cf))
		dpu_cf_put(dpu_crtc->cf);
	if (!IS_ERR_OR_NULL(dpu_crtc->dec))
		dpu_dec_put(dpu_crtc->dec);
	if (!IS_ERR_OR_NULL(dpu_crtc->ed))
		dpu_ed_put(dpu_crtc->ed);
	if (!IS_ERR_OR_NULL(dpu_crtc->fg))
		dpu_fg_put(dpu_crtc->fg);
	if (!IS_ERR_OR_NULL(dpu_crtc->tcon))
		dpu_tcon_put(dpu_crtc->tcon);
}

static int dpu_crtc_get_resources(struct dpu_crtc *dpu_crtc)
{
	struct dpu_soc *dpu = dev_get_drvdata(dpu_crtc->dev->parent);
	unsigned int stream_id = dpu_crtc->stream_id;
	int ret;

	dpu_crtc->cf = dpu_cf_get(dpu, stream_id + 4);
	if (IS_ERR(dpu_crtc->cf)) {
		ret = PTR_ERR(dpu_crtc->cf);
		goto err_out;
	}
	dpu_crtc->aux_cf = dpu_aux_cf_peek(dpu_crtc->cf);

	dpu_crtc->dec = dpu_dec_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->dec)) {
		ret = PTR_ERR(dpu_crtc->dec);
		goto err_out;
	}
	dpu_crtc->aux_dec = dpu_aux_dec_peek(dpu_crtc->dec);

	dpu_crtc->ed = dpu_ed_get(dpu, stream_id + 4);
	if (IS_ERR(dpu_crtc->ed)) {
		ret = PTR_ERR(dpu_crtc->ed);
		goto err_out;
	}
	dpu_crtc->aux_ed = dpu_aux_ed_peek(dpu_crtc->ed);

	dpu_crtc->fg = dpu_fg_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->fg)) {
		ret = PTR_ERR(dpu_crtc->fg);
		goto err_out;
	}
	dpu_crtc->aux_fg = dpu_aux_fg_peek(dpu_crtc->fg);

	dpu_crtc->tcon = dpu_tcon_get(dpu, stream_id);
	if (IS_ERR(dpu_crtc->tcon)) {
		ret = PTR_ERR(dpu_crtc->tcon);
		goto err_out;
	}
	dpu_crtc->aux_tcon = dpu_aux_tcon_peek(dpu_crtc->tcon);

	if (stream_id) {
		dpu_crtc->m_cf   = dpu_crtc->aux_cf;
		dpu_crtc->m_dec  = dpu_crtc->aux_dec;
		dpu_crtc->m_ed   = dpu_crtc->aux_ed;
		dpu_crtc->m_fg   = dpu_crtc->aux_fg;
		dpu_crtc->m_tcon = dpu_crtc->aux_tcon;

		dpu_crtc->s_cf   = dpu_crtc->cf;
		dpu_crtc->s_dec  = dpu_crtc->dec;
		dpu_crtc->s_ed   = dpu_crtc->ed;
		dpu_crtc->s_fg   = dpu_crtc->fg;
		dpu_crtc->s_tcon = dpu_crtc->tcon;
	} else {
		dpu_crtc->m_cf   = dpu_crtc->cf;
		dpu_crtc->m_dec  = dpu_crtc->dec;
		dpu_crtc->m_ed   = dpu_crtc->ed;
		dpu_crtc->m_fg   = dpu_crtc->fg;
		dpu_crtc->m_tcon = dpu_crtc->tcon;

		dpu_crtc->s_cf   = dpu_crtc->aux_cf;
		dpu_crtc->s_dec  = dpu_crtc->aux_dec;
		dpu_crtc->s_ed   = dpu_crtc->aux_ed;
		dpu_crtc->s_fg   = dpu_crtc->aux_fg;
		dpu_crtc->s_tcon = dpu_crtc->aux_tcon;
	}

	return 0;
err_out:
	dpu_crtc_put_resources(dpu_crtc);

	return ret;
}

static int dpu_crtc_init(struct dpu_crtc *dpu_crtc,
	struct dpu_client_platformdata *pdata, struct drm_device *drm)
{
	struct dpu_soc *dpu = dev_get_drvdata(dpu_crtc->dev->parent);
	struct device *dev = dpu_crtc->dev;
	struct drm_crtc *crtc = &dpu_crtc->base;
	struct dpu_plane_grp *plane_grp = pdata->plane_grp;
	unsigned int stream_id = pdata->stream_id;
	int i, ret;

	init_completion(&dpu_crtc->safety_shdld_done);
	init_completion(&dpu_crtc->content_shdld_done);
	init_completion(&dpu_crtc->dec_shdld_done);

	dpu_crtc->stream_id = stream_id;
	dpu_crtc->crtc_grp_id = pdata->di_grp_id;
	dpu_crtc->hw_plane_num = plane_grp->hw_plane_num;
	dpu_crtc->has_pc = dpu_has_pc(dpu);
	dpu_crtc->syncmode_min_prate = dpu_get_syncmode_min_prate(dpu);
	dpu_crtc->singlemode_max_width = dpu_get_singlemode_max_width(dpu);
	dpu_crtc->st = pdata->st9;

	dpu_crtc->plane = devm_kcalloc(dev, dpu_crtc->hw_plane_num,
					sizeof(*dpu_crtc->plane), GFP_KERNEL);
	if (!dpu_crtc->plane)
		return -ENOMEM;

	ret = dpu_crtc_get_resources(dpu_crtc);
	if (ret) {
		dev_err(dev, "getting resources failed with %d.\n", ret);
		return ret;
	}

	plane_grp->res.fg[stream_id] = dpu_crtc->fg;
	dpu_crtc->plane[0] = dpu_plane_init(drm, 0, stream_id, plane_grp,
					DRM_PLANE_TYPE_PRIMARY);
	if (IS_ERR(dpu_crtc->plane[0])) {
		ret = PTR_ERR(dpu_crtc->plane[0]);
		dev_err(dev, "initializing plane0 failed with %d.\n", ret);
		goto err_put_resources;
	}

	crtc->port = pdata->of_node;
	drm_crtc_helper_add(crtc, &dpu_helper_funcs);
	ret = drm_crtc_init_with_planes(drm, crtc, &dpu_crtc->plane[0]->base, NULL,
			&dpu_crtc_funcs, NULL);
	if (ret) {
		dev_err(dev, "adding crtc failed with %d.\n", ret);
		goto err_put_resources;
	}

	for (i = 1; i < dpu_crtc->hw_plane_num; i++) {
		dpu_crtc->plane[i] = dpu_plane_init(drm,
					drm_crtc_mask(&dpu_crtc->base),
					stream_id, plane_grp,
					DRM_PLANE_TYPE_OVERLAY);
		if (IS_ERR(dpu_crtc->plane[i])) {
			ret = PTR_ERR(dpu_crtc->plane[i]);
			dev_err(dev, "initializing plane%d failed with %d.\n",
								i, ret);
			goto err_put_resources;
		}
	}

	dpu_crtc->vbl_irq = dpu_map_inner_irq(dpu, stream_id ?
				IRQ_DISENGCFG_FRAMECOMPLETE1 :
				IRQ_DISENGCFG_FRAMECOMPLETE0);
	irq_set_status_flags(dpu_crtc->vbl_irq, IRQ_DISABLE_UNLAZY);
	ret = devm_request_irq(dev, dpu_crtc->vbl_irq, dpu_vbl_irq_handler, 0,
				"imx_drm", dpu_crtc);
	if (ret < 0) {
		dev_err(dev, "vblank irq request failed with %d.\n", ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->vbl_irq);

	dpu_crtc->safety_shdld_irq = dpu_map_inner_irq(dpu, stream_id ?
			IRQ_EXTDST5_SHDLOAD : IRQ_EXTDST4_SHDLOAD);
	irq_set_status_flags(dpu_crtc->safety_shdld_irq, IRQ_DISABLE_UNLAZY);
	ret = devm_request_irq(dev, dpu_crtc->safety_shdld_irq,
				dpu_safety_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		dev_err(dev,
			"safety shadow load irq request failed with %d.\n",
			ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->safety_shdld_irq);

	dpu_crtc->content_shdld_irq = dpu_map_inner_irq(dpu, stream_id ?
			IRQ_EXTDST1_SHDLOAD : IRQ_EXTDST0_SHDLOAD);
	irq_set_status_flags(dpu_crtc->content_shdld_irq, IRQ_DISABLE_UNLAZY);
	ret = devm_request_irq(dev, dpu_crtc->content_shdld_irq,
				dpu_content_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		dev_err(dev,
			"content shadow load irq request failed with %d.\n",
			ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->content_shdld_irq);

	dpu_crtc->dec_shdld_irq = dpu_map_inner_irq(dpu, stream_id ?
			IRQ_DISENGCFG_SHDLOAD1 : IRQ_DISENGCFG_SHDLOAD0);
	irq_set_status_flags(dpu_crtc->dec_shdld_irq, IRQ_DISABLE_UNLAZY);
	ret = devm_request_irq(dev, dpu_crtc->dec_shdld_irq,
				dpu_dec_shdld_irq_handler, 0, "imx_drm",
				dpu_crtc);
	if (ret < 0) {
		dev_err(dev,
			"DEC shadow load irq request failed with %d.\n",
			ret);
		goto err_put_resources;
	}
	disable_irq(dpu_crtc->dec_shdld_irq);

	return 0;

err_put_resources:
	dpu_crtc_put_resources(dpu_crtc);

	return ret;
}

static int dpu_crtc_bind(struct device *dev, struct device *master, void *data)
{
	struct dpu_client_platformdata *pdata = dev->platform_data;
	struct drm_device *drm = data;
	struct dpu_crtc *dpu_crtc;
	int ret;

	dpu_crtc = devm_kzalloc(dev, sizeof(*dpu_crtc), GFP_KERNEL);
	if (!dpu_crtc)
		return -ENOMEM;

	dpu_crtc->dev = dev;

	ret = dpu_crtc_init(dpu_crtc, pdata, drm);
	if (ret)
		return ret;

	if (!drm->mode_config.funcs)
		drm->mode_config.funcs = &dpu_drm_mode_config_funcs;

	dev_set_drvdata(dev, dpu_crtc);

	return 0;
}

static void dpu_crtc_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct dpu_crtc *dpu_crtc = dev_get_drvdata(dev);

	dpu_crtc_put_resources(dpu_crtc);
}

static const struct component_ops dpu_crtc_ops = {
	.bind = dpu_crtc_bind,
	.unbind = dpu_crtc_unbind,
};

static int dpu_crtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	if (!dev->platform_data)
		return -EINVAL;

	return component_add(dev, &dpu_crtc_ops);
}

static int dpu_crtc_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dpu_crtc_ops);
	return 0;
}

static struct platform_driver dpu_crtc_driver = {
	.driver = {
		.name = "imx-dpu-crtc",
	},
	.probe = dpu_crtc_probe,
	.remove = dpu_crtc_remove,
};
module_platform_driver(dpu_crtc_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("i.MX DPU CRTC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-dpu-crtc");
