/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright 2017-2020,2022,2023 NXP
 */

#ifndef __DPU95_CRTC_H__
#define __DPU95_CRTC_H__

#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/of.h>

#include <drm/drm_crtc.h>
#include <drm/drm_print.h>
#include <drm/drm_vblank.h>

#include "dpu95.h"

#define dpu95_crtc_dbg(crtc, fmt, ...)					\
do {									\
	typeof(crtc) _crtc = (crtc);					\
	drm_dbg_kms(_crtc->dev, "[CRTC:%d:%s] " fmt,			\
		    _crtc->base.id, _crtc->name, ##__VA_ARGS__);	\
} while (0)

#define dpu95_crtc_err(crtc, fmt, ...)					\
do {									\
	typeof(crtc) _crtc = (crtc);					\
	drm_err(_crtc->dev, "[CRTC:%d:%s] " fmt,			\
		_crtc->base.id, _crtc->name, ##__VA_ARGS__);		\
} while (0)

struct dpu95_drm_device;

struct dpu95_crtc {
	struct drm_crtc			base;
	struct dpu95_soc		*dpu;
	struct device_node		*np;
	struct dpu95_constframe		*cf_cont;
	struct dpu95_extdst		*ed_cont;
	struct dpu95_framegen		*fg;
	struct dpu95_domainblend	*db;
	struct dpu95_dither		*dt;
	unsigned int			dpu_dec_frame_complete_irq;
	unsigned int			dpu_dec_seq_complete_irq;
	unsigned int			dpu_dec_shdld_irq;
	unsigned int			dpu_db_shdld_irq;
	unsigned int			dpu_ed_cont_shdld_irq;
	unsigned int			dec_frame_complete_irq;
	unsigned int			dec_seq_complete_irq;
	unsigned int			dec_shdld_irq;
	unsigned int			db_shdld_irq;
	unsigned int			ed_cont_shdld_irq;
	unsigned int			stream_id;
	struct completion		dec_seq_complete_done;
	struct completion		dec_shdld_done;
	struct completion		db_shdld_done;
	struct completion		ed_cont_shdld_done;
	struct drm_pending_vblank_event *event;
};

static inline struct dpu95_crtc *to_dpu95_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct dpu95_crtc, base);
}

int dpu95_crtc_init(struct dpu95_drm_device *dpu_drm,
		    struct dpu95_crtc *dpu_crtc, int stream_id);

#endif /* __DPU95_CRTC_H__ */
