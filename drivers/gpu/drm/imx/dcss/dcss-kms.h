/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 NXP.
 */

#ifndef _DCSS_KMS_H_
#define _DCSS_KMS_H_

#include <linux/kernel.h>

#include <drm/drm_atomic.h>
#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_encoder.h>

#include "dcss-dev.h"

struct dcss_plane {
	struct drm_plane base;

	uint64_t dtrc_table_ofs_val;
	struct drm_property *dtrc_table_ofs_prop;

	int ch_num;

	enum drm_plane_type type;
	bool use_dtrc;
};

struct dcss_crtc {
	struct drm_crtc		base;
	struct dcss_plane	*plane[3];
	int			irq;
	bool			disable_ctxld_kick_irq;
};

struct dcss_crtc_state {
	struct drm_crtc_state		base;
	enum dcss_pixel_pipe_output	output_encoding;
	enum dcss_hdr10_nonlinearity	opipe_nl;
	enum dcss_hdr10_gamut		opipe_g;
	enum dcss_hdr10_pixel_range	opipe_pr;
};

struct dcss_kms_dev {
	struct drm_device base;
	struct dcss_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector *connector;
};

static inline struct dcss_crtc *to_dcss_crtc(struct drm_crtc *crtc)
{
	return container_of(crtc, struct dcss_crtc, base);
}

static inline struct dcss_crtc_state *
to_dcss_crtc_state(struct drm_crtc_state *state)
{
	return container_of(state, struct dcss_crtc_state, base);
}

struct dcss_kms_dev *dcss_kms_attach(struct dcss_dev *dcss, bool componetized);
void dcss_kms_detach(struct dcss_kms_dev *kms, bool componetized);
void dcss_kms_shutdown(struct dcss_kms_dev *kms);
int dcss_crtc_setup_opipe(struct drm_device *dev,
			  struct drm_atomic_state *state);
int dcss_crtc_init(struct dcss_crtc *crtc, struct drm_device *drm);
void dcss_crtc_deinit(struct dcss_crtc *crtc, struct drm_device *drm);
struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos);
void dcss_crtc_attach_color_mgmt_properties(struct dcss_crtc *crtc);

#endif
