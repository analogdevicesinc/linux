/*
 * Copyright 2019,2020 NXP
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
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_plane.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-crtc.h"

static void
dpu_enable_signature_roi(struct dpu_signature *sig, struct drm_rect *roi)
{
	signature_continuous_mode(sig, true);
	signature_win(sig, 0, roi->x1, roi->y1, roi->x2, roi->y2);
	signature_eval_win(sig, 0, true);
	signature_shdldreq(sig, 0x1);
}

static void dpu_disable_signature(struct dpu_signature *sig)
{
	signature_continuous_mode(sig, false);
	signature_wait_for_idle(sig);
	signature_eval_win(sig, 0, false);
}

static int dpu_crc_parse_source(const char *source_name, enum dpu_crc_source *s)
{
	if (!source_name) {
		*s = DPU_CRC_SRC_NONE;
	} else if (!strcmp(source_name, "auto")) {
		*s = DPU_CRC_SRC_FRAMEGEN;
	} else {
		return -EINVAL;
	}

	return 0;
}

int dpu_crtc_verify_crc_source(struct drm_crtc *crtc, const char *source_name,
			       size_t *values_cnt)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct imx_crtc_state *imx_crtc_state;
	struct dpu_crtc_state *dcstate;
	enum dpu_crc_source source;
	int ret;

	if (dpu_crc_parse_source(source_name, &source) < 0) {
		dev_dbg(dpu_crtc->dev, "unknown source %s\n", source_name);
		return -EINVAL;
	}

	ret = drm_modeset_lock_single_interruptible(&crtc->mutex);
	if (ret)
		return ret;

	imx_crtc_state = to_imx_crtc_state(crtc->state);
	dcstate = to_dpu_crtc_state(imx_crtc_state);
	*values_cnt = dcstate->use_pc ? 6 : 3;

	drm_modeset_unlock(&crtc->mutex);

	return ret;
}

int dpu_crtc_set_crc_source(struct drm_crtc *crtc, const char *source_name)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct drm_modeset_acquire_ctx ctx;
	struct drm_crtc_state *crtc_state;
	struct drm_atomic_state *state;
	enum dpu_crc_source source;
	int ret;

	if (dpu_crc_parse_source(source_name, &source) < 0) {
		dev_dbg(dpu_crtc->dev, "unknown source %s\n", source_name);
		return -EINVAL;
	}

	/* Perform an atomic commit to set the CRC source. */
	drm_modeset_acquire_init(&ctx, 0);

	state = drm_atomic_state_alloc(crtc->dev);
	if (!state) {
		ret = -ENOMEM;
		goto unlock;
	}

	state->acquire_ctx = &ctx;

retry:
	crtc_state = drm_atomic_get_crtc_state(state, crtc);
	if (!IS_ERR(crtc_state)) {
		struct imx_crtc_state *imx_crtc_state;
		struct dpu_crtc_state *dcstate;

		imx_crtc_state = to_imx_crtc_state(crtc_state);
		dcstate = to_dpu_crtc_state(imx_crtc_state);

		if ((dcstate->use_pc && crtc->crc.values_cnt != 6) ||
		    (!dcstate->use_pc && crtc->crc.values_cnt != 3)) {
			ret = -EINVAL;
			goto put;
		}

		dcstate->crc.source = source;
		dpu_crtc->use_dual_crc = dcstate->use_pc;

		ret = drm_atomic_commit(state);
	} else {
		ret = PTR_ERR(crtc_state);
	}

	if (ret == -EDEADLK) {
		drm_atomic_state_clear(state);
		drm_modeset_backoff(&ctx);
		goto retry;
	}

put:
	drm_atomic_state_put(state);

unlock:
	drm_modeset_drop_locks(&ctx);
	drm_modeset_acquire_fini(&ctx);

	return ret;
}

irqreturn_t dpu_crc_valid_irq_threaded_handler(int irq, void *dev_id)
{
	struct dpu_crtc *dpu_crtc = dev_id;
	struct dpu_signature *sig = dpu_crtc->sig;
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);
	bool dual_crc = dpu_crtc->use_dual_crc;
	unsigned long ret;
	uint32_t crcs[6] = {0, 0, 0, 0, 0, 0};

	dev_dbg(dpu_crtc->dev, "CRC valid irq threaded handler\n");

	signature_crc_value(sig, 0, &dpu_crtc->crc_red,
				    &dpu_crtc->crc_green,
				    &dpu_crtc->crc_blue);

	if (dual_crc && dpu_crtc->stream_id == 1) {
		complete(&aux_dpu_crtc->aux_crc_done);
		return IRQ_HANDLED;
	}

	crcs[2] = dpu_crtc->crc_red;
	crcs[1] = dpu_crtc->crc_green;
	crcs[0] = dpu_crtc->crc_blue;

	if (dual_crc && dpu_crtc->stream_id == 0) {
		ret = wait_for_completion_timeout(&dpu_crtc->aux_crc_done,
						  HZ / 20);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				"wait for auxiliary CRC done timeout\n");

		crcs[5] = aux_dpu_crtc->crc_red;
		crcs[4] = aux_dpu_crtc->crc_green;
		crcs[3] = aux_dpu_crtc->crc_blue;
	}

	drm_crtc_add_crc_entry(&dpu_crtc->base, false, 0, crcs);

	return IRQ_HANDLED;
}

void dpu_crtc_enable_crc_source(struct drm_crtc *crtc,
				enum dpu_crc_source source)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);
	struct imx_crtc_state *imx_crtc_state = to_imx_crtc_state(crtc->state);
	struct dpu_crtc_state *dcstate = to_dpu_crtc_state(imx_crtc_state);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;
	struct completion *shdld_done;
	struct drm_rect roi;
	bool dual_crc = dpu_crtc->use_dual_crc;
	unsigned long ret;

	if (source == DPU_CRC_SRC_NONE)
		return;

	if (dual_crc != dcstate->use_pc)
		return;

	if (dpu_crtc->crc_is_enabled)
		return;

	/* region of interest */
	roi.x1 = 0;
	roi.y1 = 0;
	roi.x2 = dual_crc ? mode->hdisplay >> 1 : mode->hdisplay;
	roi.y2 = mode->vdisplay;

	enable_irq(dpu_crtc->crc_valid_irq);
	enable_irq(dpu_crtc->crc_shdld_irq);
	disengcfg_sig_select(dpu_crtc->dec, DEC_SIG_SEL_FRAMEGEN);
	dpu_enable_signature_roi(dpu_crtc->sig, &roi);

	if (dual_crc) {
		aux_dpu_crtc->use_dual_crc = dual_crc;
		enable_irq(aux_dpu_crtc->crc_valid_irq);
		enable_irq(aux_dpu_crtc->crc_shdld_irq);
		disengcfg_sig_select(dpu_crtc->aux_dec, DEC_SIG_SEL_FRAMEGEN);
		dpu_enable_signature_roi(dpu_crtc->aux_sig, &roi);
	}

	shdld_done = &dpu_crtc->crc_shdld_done;
	ret = wait_for_completion_timeout(shdld_done, HZ);
	if (ret == 0)
		dev_warn(dpu_crtc->dev, "wait for CRC shdld done timeout\n");

	if (dual_crc) {
		shdld_done = &aux_dpu_crtc->crc_shdld_done;
		ret = wait_for_completion_timeout(shdld_done, HZ);
		if (ret == 0)
			dev_warn(dpu_crtc->dev,
				 "wait for auxiliary CRC shdld done timeout\n");
	}

	disable_irq(dpu_crtc->crc_shdld_irq);
	if (dual_crc)
		disable_irq(aux_dpu_crtc->crc_shdld_irq);

	dpu_crtc->crc_is_enabled = true;

	dev_dbg(dpu_crtc->dev, "enable CRC source\n");
}

void dpu_crtc_disable_crc_source(struct drm_crtc *crtc, bool dual_crc)
{
	struct dpu_crtc *dpu_crtc = to_dpu_crtc(crtc);
	struct dpu_crtc *aux_dpu_crtc = dpu_crtc_get_aux_dpu_crtc(dpu_crtc);

	if (!dpu_crtc->crc_is_enabled)
		return;

	dpu_disable_signature(dpu_crtc->sig);
	if (dual_crc)
		dpu_disable_signature(dpu_crtc->aux_sig);

	disable_irq(dpu_crtc->crc_valid_irq);
	if (dual_crc) {
		disable_irq(aux_dpu_crtc->crc_valid_irq);
		reinit_completion(&dpu_crtc->aux_crc_done);
	}

	dpu_crtc->crc_is_enabled = false;

	dev_dbg(dpu_crtc->dev, "disable CRC source\n");
}
