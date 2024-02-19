// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright 2017-2020,2022-2024 NXP
 */

#include <linux/irq.h>
#include <linux/irqflags.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_color_mgmt.h>
#include <drm/drm_encoder.h>

#include "dpu95.h"
#include "dpu95-crtc.h"
#include "dpu95-drv.h"
#include "dpu95-plane.h"

#define DPU95_CRTC_WAIT_FOR_COMPLETION_TIMEOUT(_name)			\
do {									\
	unsigned long ret;						\
	ret = wait_for_completion_timeout(&dpu_crtc->_name, HZ);	\
	if (ret == 0)							\
		dpu95_crtc_err(crtc, "%s: wait for " #_name " timeout\n",\
			       __func__);				\
} while (0)

#define DPU95_CRTC_CHECK_FRAMEGEN_PRIMARY_FIFO(fg)			\
do {									\
	typeof(fg) _fg = (fg);						\
	if (dpu95_fg_primary_requests_to_read_empty_fifo(_fg)) {	\
		dpu95_fg_primary_clear_channel_status(_fg);		\
		dpu95_crtc_err(crtc, "%s: FrameGen FIFO empty\n",	\
			       __func__);				\
	}								\
} while (0)

#define DPU95_CRTC_WAIT_FOR_FRAMEGEN_PRIMARY_SYNCUP(fg)			\
do {									\
	if (dpu95_fg_wait_for_primary_syncup(fg))			\
		dpu95_crtc_err(crtc,					\
			"%s: FrameGen primary channel isn't syncup\n",	\
			__func__);					\
} while (0)

static u32 dpu95_crtc_get_vblank_counter(struct drm_crtc *crtc)
{
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);

	if (pm_runtime_active(dpu_crtc->dpu->dev))
		return dpu95_fg_get_frame_index(dpu_crtc->fg);
	else
		return (u32)drm_crtc_vblank_count(crtc);
}

static int dpu95_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);

	enable_irq(dpu_crtc->dec_frame_complete_irq);

	return 0;
}

static void dpu95_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);

	disable_irq_nosync(dpu_crtc->dec_frame_complete_irq);
}

static irqreturn_t
dpu95_crtc_dec_frame_complete_irq_handler(int irq, void *dev_id)
{
	struct dpu95_crtc *dpu_crtc = dev_id;
	struct drm_crtc *crtc = &dpu_crtc->base;
	unsigned long flags;

	drm_crtc_handle_vblank(crtc);

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (dpu_crtc->event) {
		drm_crtc_send_vblank_event(crtc, dpu_crtc->event);
		dpu_crtc->event = NULL;
		drm_crtc_vblank_put(crtc);
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t dpu95_crtc_common_irq_handler(int irq, void *dev_id)
{
	struct dpu95_crtc *dpu_crtc = dev_id;
	struct drm_crtc *crtc = &dpu_crtc->base;

	if (irq == dpu_crtc->dec_seq_complete_irq) {
		complete(&dpu_crtc->dec_seq_complete_done);
	} else if (irq == dpu_crtc->dec_shdld_irq) {
		complete(&dpu_crtc->dec_shdld_done);
	} else if (irq == dpu_crtc->db_shdld_irq) {
		complete(&dpu_crtc->db_shdld_done);
	} else if (irq == dpu_crtc->ed_cont_shdld_irq) {
		complete(&dpu_crtc->ed_cont_shdld_done);
	} else {
		dpu95_crtc_err(crtc, "invalid CRTC irq(%u)\n", irq);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static const struct drm_crtc_funcs dpu95_crtc_funcs = {
	.reset			= drm_atomic_helper_crtc_reset,
	.destroy		= drm_crtc_cleanup,
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.atomic_duplicate_state	= drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_crtc_destroy_state,
	.get_vblank_counter	= dpu95_crtc_get_vblank_counter,
	.enable_vblank		= dpu95_crtc_enable_vblank,
	.disable_vblank		= dpu95_crtc_disable_vblank,
	.get_vblank_timestamp	= drm_crtc_vblank_helper_get_vblank_timestamp,
};

static void dpu95_crtc_queue_state_event(struct drm_crtc *crtc)
{
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		WARN_ON(drm_crtc_vblank_get(crtc));
		WARN_ON(dpu_crtc->event);
		dpu_crtc->event = crtc->state->event;
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);
}

static enum drm_mode_status
dpu95_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	if (mode->crtc_clock > DPU95_FRAMEGEN_MAX_CLOCK)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static int dpu95_crtc_pm_runtime_resume_and_get(struct dpu95_crtc *dpu_crtc)
{
	int ret;

	ret = pm_runtime_resume_and_get(dpu_crtc->dpu->dev);
	if (ret < 0) {
		dpu95_crtc_err(&dpu_crtc->base,
			       "failed to get device RPM: %d\n", ret);
		return ret;
	}

	return 0;
}

static void dpu95_crtc_pm_runtime_put(struct dpu95_crtc *dpu_crtc)
{
	int ret;

	ret = pm_runtime_put(dpu_crtc->dpu->dev);
	if (ret < 0)
		dpu95_crtc_err(&dpu_crtc->base,
			       "failed to put device RPM: %d\n", ret);
}

static void dpu95_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(crtc->dev);
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	struct dpu95_soc *dpu = dpu_crtc->dpu;
	struct drm_display_mode *adj = &crtc->state->adjusted_mode;
	struct drm_encoder *encoder = &dpu_drm->encoder[dpu_crtc->stream_id];
	bool enc_is_dsi = encoder->encoder_type == DRM_MODE_ENCODER_DSI;

	dpu95_crtc_dbg(crtc, "mode " DRM_MODE_FMT "\n", DRM_MODE_ARG(adj));

	/* request power-on when we start to set mode for CRTC */
	dpu95_crtc_pm_runtime_resume_and_get(dpu_crtc);

	dpu95_disable_display_pipeline_sync(dpu);

	dpu95_fg_syncmode(dpu_crtc->fg, FG_SYNCMODE_OFF);
	dpu95_fg_displaymode(dpu_crtc->fg, FG_DM_PRIM);
	dpu95_fg_panic_displaymode(dpu_crtc->fg, FG_DM_CONSTCOL);
	dpu95_fg_cfg_videomode(dpu_crtc->fg, adj, enc_is_dsi);
	dpu95_fg_shdtokgen_syncmode(dpu_crtc->fg, FG_SHDTOKGENSYNCMODE_LOCAL);

	/* black CRTC background */
	dpu95_cf_constantcolor_black(dpu_crtc->cf_cont);
	dpu95_cf_framedimensions(dpu_crtc->cf_cont,
				 adj->crtc_hdisplay, adj->crtc_vdisplay);

	dpu95_dt_polen_active_high(dpu_crtc->dt);
	if (enc_is_dsi) {
		dpu95_dt_polhs_active_low(dpu_crtc->dt);
		dpu95_dt_polvs_active_low(dpu_crtc->dt);
	} else {
		if (adj->flags & DRM_MODE_FLAG_PHSYNC)
			dpu95_dt_polhs_active_high(dpu_crtc->dt);
		else
			dpu95_dt_polhs_active_low(dpu_crtc->dt);

		if (adj->flags & DRM_MODE_FLAG_PVSYNC)
			dpu95_dt_polvs_active_high(dpu_crtc->dt);
		else
			dpu95_dt_polvs_active_low(dpu_crtc->dt);
	}
}

static int dpu95_crtc_atomic_check(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state;
	int ret;

	crtc_state = drm_atomic_get_new_crtc_state(state, crtc);

	/* force a mode set if the CRTC is changed to active */
	if (crtc_state->active_changed && crtc_state->active) {
		/*
		 * If mode_changed is set by us, call
		 * drm_atomic_helper_check_modeset() as it's Kerneldoc requires.
		 */
		if (!crtc_state->mode_changed) {
			crtc_state->mode_changed = true;

			ret = drm_atomic_helper_check_modeset(crtc->dev, state);
			if (ret)
				return ret;
		}
	}

	/* CRTC has to be enabled together with at least one plane. */
	if (crtc_state->active && !crtc_state->plane_mask) {
		dpu95_crtc_dbg(crtc, "%s: no plane is enabled\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void dpu95_crtc_atomic_begin(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct drm_crtc_state *old_crtc_state;
	struct drm_atomic_state *old_state;
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	struct drm_plane_state *old_plane_state;
	struct dpu95_plane_state *old_dpstate;
	struct dpu95_fetchunit *fu;
	const struct dpu95_fetchunit_ops *fu_ops;
	int i;

	old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);
	old_state = old_crtc_state->state;

	/* do nothing if planes keep being disabled */
	if (old_crtc_state->plane_mask == 0 && crtc->state->plane_mask == 0)
		return;

	/* request power-on when any plane starts to be active */
	if (old_crtc_state->plane_mask == 0 && crtc->state->plane_mask != 0)
		dpu95_crtc_pm_runtime_resume_and_get(dpu_crtc);

	/*
	 * Disable relevant planes' resources in SHADOW only.
	 * Whether any of them would be disabled or kept running depends
	 * on new plane states in the new global atomic state.
	 */
	for_each_old_plane_state_in_state(old_state, old_plane_state, i) {
		old_dpstate = to_dpu95_plane_state(old_plane_state);

		if (!old_plane_state->fb)
			continue;

		if (old_plane_state->crtc != crtc)
			continue;

		fu = old_dpstate->source;

		fu_ops = dpu95_fu_get_ops(fu);

		fu_ops->disable_src_buf(fu);

		if (old_dpstate->is_top)
			dpu95_ed_pec_src_sel(dpu_crtc->ed_cont,
					     DPU95_LINK_ID_NONE);
	}
}

static void dpu95_crtc_atomic_flush_hscaler(struct drm_crtc *crtc)
{
	struct dpu95_plane *dplane = to_dpu95_plane(crtc->primary);
	struct dpu95_hscaler *hs = dplane->grp->hs;
	const struct dpu95_hscaler_ops *hs_ops;

	hs_ops = dpu95_hs_get_ops(hs);
	if (!hs_ops->is_enabled(hs))
		hs_ops->set_no_stream_id(hs);
}

static void dpu95_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	bool need_modeset = drm_atomic_crtc_needs_modeset(crtc->state);
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	const struct dpu95_fetchunit_ops *fu_ops;
	struct drm_plane_state *old_plane_state;
	struct drm_crtc_state *old_crtc_state;
	struct dpu95_plane_state *old_dpstate;
	struct drm_atomic_state *old_state;
	struct dpu95_fetchunit *fu;
	int i;

	old_crtc_state = drm_atomic_get_old_crtc_state(state, crtc);
	old_state = old_crtc_state->state;

	if (old_crtc_state->plane_mask == 0 && crtc->state->plane_mask == 0) {
		/* Queue a pending vbl event if necessary. */
		if (!need_modeset && crtc->state->active)
			dpu95_crtc_queue_state_event(crtc);
		return;
	}

	/* Set no stream id for disabled fetchunits of relevant planes. */
	for_each_old_plane_state_in_state(old_state, old_plane_state, i) {
		old_dpstate = to_dpu95_plane_state(old_plane_state);

		if (!old_plane_state->fb)
			continue;

		if (old_plane_state->crtc != crtc)
			continue;

		fu = old_dpstate->source;

		fu_ops = dpu95_fu_get_ops(fu);

		if (!fu_ops->is_enabled(fu))
			fu_ops->set_no_stream_id(fu);
	}

	dpu95_crtc_atomic_flush_hscaler(crtc);

	if (!need_modeset && crtc->state->active) {
		enable_irq(dpu_crtc->ed_cont_shdld_irq);

		/*
		 * Flush plane(s) update out to display & queue a pending
		 * vbl event if necessary.
		 */
		dpu95_ed_pec_sync_trigger(dpu_crtc->ed_cont);

		DPU95_CRTC_WAIT_FOR_COMPLETION_TIMEOUT(ed_cont_shdld_done);

		disable_irq(dpu_crtc->ed_cont_shdld_irq);

		DPU95_CRTC_CHECK_FRAMEGEN_PRIMARY_FIFO(dpu_crtc->fg);

		dpu95_crtc_queue_state_event(crtc);
	} else {
		/*
		 * Simply flush and hope that any update takes effect
		 * if CRTC is disabled.  This helps for the case where
		 * migrating plane(s) from a disabled CRTC to the other
		 * CRTC.
		 */
		if (!crtc->state->active)
			dpu95_ed_pec_sync_trigger(dpu_crtc->ed_cont);
	}

	/* request power-off when all planes are off */
	if (old_crtc_state->plane_mask != 0 && crtc->state->plane_mask == 0)
		dpu95_crtc_pm_runtime_put(dpu_crtc);
}

static void dpu95_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_atomic_state *state)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(crtc->dev);
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	struct drm_encoder *encoder = &dpu_drm->encoder[dpu_crtc->stream_id];
	bool enc_is_dsi = encoder->encoder_type == DRM_MODE_ENCODER_DSI;

	drm_crtc_vblank_on(crtc);

	enable_irq(dpu_crtc->dec_shdld_irq);
	enable_irq(dpu_crtc->db_shdld_irq);
	enable_irq(dpu_crtc->ed_cont_shdld_irq);

	dpu95_fg_enable_clock(dpu_crtc->fg, enc_is_dsi);
	dpu95_ed_pec_sync_trigger(dpu_crtc->ed_cont);
	dpu95_db_shdtokgen(dpu_crtc->db);
	dpu95_fg_shdtokgen(dpu_crtc->fg);
	dpu95_fg_enable(dpu_crtc->fg);

	DPU95_CRTC_WAIT_FOR_COMPLETION_TIMEOUT(ed_cont_shdld_done);
	DPU95_CRTC_WAIT_FOR_COMPLETION_TIMEOUT(db_shdld_done);
	DPU95_CRTC_WAIT_FOR_COMPLETION_TIMEOUT(dec_shdld_done);

	disable_irq(dpu_crtc->ed_cont_shdld_irq);
	disable_irq(dpu_crtc->db_shdld_irq);
	disable_irq(dpu_crtc->dec_shdld_irq);

	DPU95_CRTC_WAIT_FOR_FRAMEGEN_PRIMARY_SYNCUP(dpu_crtc->fg);

	/* ignore initial empty primary pixel FIFO read status, just clear it */
	dpu95_fg_primary_clear_channel_status(dpu_crtc->fg);

	dpu95_crtc_queue_state_event(crtc);
}

static void dpu95_crtc_disable(struct drm_crtc *crtc)
{
	struct dpu95_drm_device *dpu_drm = to_dpu95_drm_device(crtc->dev);
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	struct drm_encoder *encoder = &dpu_drm->encoder[dpu_crtc->stream_id];
	bool enc_is_dsi = encoder->encoder_type == DRM_MODE_ENCODER_DSI;

	enable_irq(dpu_crtc->dec_seq_complete_irq);
	dpu95_fg_disable(dpu_crtc->fg);
	DPU95_CRTC_WAIT_FOR_COMPLETION_TIMEOUT(dec_seq_complete_done);
	disable_irq(dpu_crtc->dec_seq_complete_irq);

	dpu95_fg_disable_clock(dpu_crtc->fg, enc_is_dsi);

	drm_crtc_vblank_off(crtc);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event && !crtc->state->active) {
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	/* request power-off when CRTC is disabled */
	dpu95_crtc_pm_runtime_put(dpu_crtc);
}

static bool dpu95_crtc_get_scanout_position(struct drm_crtc *crtc,
					    bool in_vblank_irq,
					    int *vpos, int *hpos,
					    ktime_t *stime, ktime_t *etime,
					    const struct drm_display_mode *mode)
{
	struct dpu95_crtc *dpu_crtc = to_dpu95_crtc(crtc);
	int vdisplay = mode->crtc_vdisplay;
	int vtotal = mode->crtc_vtotal;
	bool reliable;
	int line;

	if (stime)
		*stime = ktime_get();

	if (pm_runtime_active(dpu_crtc->dpu->dev)) {
		/* line index starts with 0 for the first active output line */
		line = dpu95_fg_get_line_index(dpu_crtc->fg);

		if (line < vdisplay)
			/* active scanout area - positive */
			*vpos = line + 1;
		else
			/* inside vblank - negative */
			*vpos = line - (vtotal - 1);

		reliable = true;
	} else {
		*vpos = 0;
		reliable = false;
	}

	*hpos = 0;

	if (etime)
		*etime = ktime_get();

	return reliable;
}

static const struct drm_crtc_helper_funcs dpu95_helper_funcs = {
	.mode_valid		= dpu95_crtc_mode_valid,
	.mode_set_nofb		= dpu95_crtc_mode_set_nofb,
	.atomic_check		= dpu95_crtc_atomic_check,
	.atomic_begin		= dpu95_crtc_atomic_begin,
	.atomic_flush		= dpu95_crtc_atomic_flush,
	.atomic_enable		= dpu95_crtc_atomic_enable,
	.disable		= dpu95_crtc_disable,
	.get_scanout_position	= dpu95_crtc_get_scanout_position,
};

static int dpu95_crtc_get_resources(struct dpu95_crtc *dpu_crtc)
{
	struct dpu95_soc *dpu = dpu_crtc->dpu;
	struct {
		void **dpu_unit;
		void *(*get)(struct dpu95_soc *dpu95, unsigned int id);
	} resources[] = {
		{(void *)&dpu_crtc->cf_cont,	(void *)dpu95_cf_cont_get},
		{(void *)&dpu_crtc->ed_cont,	(void *)dpu95_ed_cont_get},
		{(void *)&dpu_crtc->fg,		(void *)dpu95_fg_get},
		{(void *)&dpu_crtc->db,		(void *)dpu95_db_get},
		{(void *)&dpu_crtc->dt,		(void *)dpu95_dt_get},
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(resources); i++) {
		*resources[i].dpu_unit =
				resources[i].get(dpu, dpu_crtc->stream_id);
		if (IS_ERR(*resources[i].dpu_unit)) {
			ret = PTR_ERR(*resources[i].dpu_unit);
			return ret;
		}
	}

	return 0;
}

static int dpu95_crtc_request_irq(struct dpu95_crtc *dpu_crtc,
				  unsigned int *crtc_irq,
				  unsigned int dpu_irq,
				  irqreturn_t (*irq_handler)(int irq,
							     void *dev_id))
{
	struct drm_crtc *crtc = &dpu_crtc->base;
	struct dpu95_soc *dpu = dpu_crtc->dpu;
	int ret;

	if (dpu_crtc->stream_id == 0)
		*crtc_irq = dpu95_map_disp_irq0(dpu, dpu_irq);
	else
		*crtc_irq = dpu95_map_disp_irq2(dpu, dpu_irq);

	irq_set_status_flags(*crtc_irq, IRQ_DISABLE_UNLAZY);
	ret = devm_request_irq(dpu_crtc->dpu->dev, *crtc_irq, irq_handler,
			       0, dev_name(dpu_crtc->dpu->dev), dpu_crtc);
	if (ret < 0) {
		dpu95_crtc_err(crtc, "failed to request irq(%u): %d\n",
			       *crtc_irq, ret);
		return ret;
	}
	disable_irq(*crtc_irq);

	return 0;
}

static int dpu95_crtc_request_irqs(struct dpu95_crtc *dpu_crtc)
{
	struct {
		unsigned int *crtc_irq;
		unsigned int dpu_irq;
		irqreturn_t (*irq_handler)(int irq, void *dev_id);
	} irqs[] = {
		{
			&dpu_crtc->dec_frame_complete_irq,
			dpu_crtc->dpu_dec_frame_complete_irq,
			dpu95_crtc_dec_frame_complete_irq_handler,
		}, {
			&dpu_crtc->dec_seq_complete_irq,
			dpu_crtc->dpu_dec_seq_complete_irq,
			dpu95_crtc_common_irq_handler,
		}, {
			&dpu_crtc->dec_shdld_irq,
			dpu_crtc->dpu_dec_shdld_irq,
			dpu95_crtc_common_irq_handler,
		}, {
			&dpu_crtc->db_shdld_irq,
			dpu_crtc->dpu_db_shdld_irq,
			dpu95_crtc_common_irq_handler,
		}, {
			&dpu_crtc->ed_cont_shdld_irq,
			dpu_crtc->dpu_ed_cont_shdld_irq,
			dpu95_crtc_common_irq_handler,
		},
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(irqs); i++) {
		ret = dpu95_crtc_request_irq(dpu_crtc,
					     irqs[i].crtc_irq, irqs[i].dpu_irq,
					     irqs[i].irq_handler);
		if (ret)
			return ret;
	}

	return 0;
}

int dpu95_crtc_init(struct dpu95_drm_device *dpu_drm,
		    struct dpu95_crtc *dpu_crtc, int stream_id)
{
	struct dpu95_plane_grp *plane_grp = &dpu_drm->dpu_plane_grp;
	struct drm_device *drm = &dpu_drm->base;
	struct drm_crtc *crtc = &dpu_crtc->base;
	struct dpu95_plane *dpu_primary;
	struct device *dev = drm->dev;
	struct device_node *port;
	int ret;

	port = of_graph_get_port_by_id(dev->of_node, stream_id);
	if (!port) {
		drm_err(drm, "failed to get port for stream%d\n", stream_id);
		return -ENODEV;
	}

	dpu_crtc->np = port;

	of_node_put(port);

	init_completion(&dpu_crtc->dec_seq_complete_done);
	init_completion(&dpu_crtc->dec_shdld_done);
	init_completion(&dpu_crtc->db_shdld_done);
	init_completion(&dpu_crtc->ed_cont_shdld_done);

	dpu_crtc->dpu = &dpu_drm->dpu_soc;
	dpu_crtc->stream_id = stream_id;

	if (stream_id == 0) {
		dpu_crtc->dpu_dec_frame_complete_irq	= DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0;
		dpu_crtc->dpu_dec_seq_complete_irq	= DPU95_IRQ_DISENGCFG_SEQCOMPLETE0;
		dpu_crtc->dpu_dec_shdld_irq		= DPU95_IRQ_DISENGCFG_SHDLOAD0;
		dpu_crtc->dpu_db_shdld_irq		= DPU95_IRQ_DOMAINBLEND0_SHDLOAD;
		dpu_crtc->dpu_ed_cont_shdld_irq		= DPU95_IRQ_EXTDST0_SHDLOAD;
	} else {
		dpu_crtc->dpu_dec_frame_complete_irq	= DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1;
		dpu_crtc->dpu_dec_seq_complete_irq	= DPU95_IRQ_DISENGCFG_SEQCOMPLETE1;
		dpu_crtc->dpu_dec_shdld_irq		= DPU95_IRQ_DISENGCFG_SHDLOAD1;
		dpu_crtc->dpu_db_shdld_irq		= DPU95_IRQ_DOMAINBLEND1_SHDLOAD;
		dpu_crtc->dpu_ed_cont_shdld_irq		= DPU95_IRQ_EXTDST1_SHDLOAD;
	}

	ret = dpu95_crtc_get_resources(dpu_crtc);
	if (ret) {
		drm_err(drm, "failed to get CRTC HW resources for stream%d: %d\n",
			stream_id, ret);
		return ret;
	}

	plane_grp->cf[stream_id] = dpu_crtc->cf_cont;
	plane_grp->ed[stream_id] = dpu_crtc->ed_cont;
	plane_grp->hs = dpu95_hs_get(dpu_crtc->dpu, 4);

	/* each CRTC has a primary plane */
	dpu_primary = &dpu_drm->dpu_primary[stream_id];
	ret = dpu95_plane_initialize(dpu_drm, dpu_primary, 0,
				     DRM_PLANE_TYPE_PRIMARY);
	if (ret) {
		drm_err(drm, "failed to init primary plane for stream%d: %d\n",
			stream_id, ret);
		return ret;
	}

	drm_crtc_helper_add(crtc, &dpu95_helper_funcs);

	ret = drm_crtc_init_with_planes(drm, crtc, &dpu_primary->base,
					NULL, &dpu95_crtc_funcs, NULL);
	if (ret) {
		drm_err(drm, "failed to add CRTC for stream%d: %d\n",
			stream_id, ret);
		return ret;
	}

	dpu_drm->crtc_mask |= drm_crtc_mask(crtc);

	ret = dpu95_crtc_pm_runtime_resume_and_get(dpu_crtc);
	if (ret < 0)
		return ret;

	ret = dpu95_crtc_request_irqs(dpu_crtc);

	dpu95_crtc_pm_runtime_put(dpu_crtc);

	return ret;
}
