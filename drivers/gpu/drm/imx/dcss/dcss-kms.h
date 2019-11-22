/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019 NXP.
 */

#ifndef _DCSS_KMS_H_
#define _DCSS_KMS_H_

#include <drm/drm_encoder.h>

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
	struct drm_crtc_state	*state;

	struct dcss_plane	*plane[3];

	int			irq;

	bool disable_ctxld_kick_irq;

	bool output_is_yuv;
	enum dcss_hdr10_nonlinearity opipe_nl;
	enum dcss_hdr10_gamut opipe_g;
	enum dcss_hdr10_pixel_range opipe_pr;
};

struct dcss_kms_dev {
	struct drm_device base;
	struct dcss_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector *connector;
};

struct dcss_kms_dev *dcss_kms_attach(struct dcss_dev *dcss);
void dcss_kms_setup_opipe(struct drm_connector_state *conn_state);
void dcss_kms_detach(struct dcss_kms_dev *kms);
void dcss_kms_shutdown(struct dcss_kms_dev *kms);
int dcss_crtc_init(struct dcss_crtc *crtc, struct drm_device *drm);
void dcss_crtc_deinit(struct dcss_crtc *crtc, struct drm_device *drm);
struct dcss_plane *dcss_plane_init(struct drm_device *drm,
				   unsigned int possible_crtcs,
				   enum drm_plane_type type,
				   unsigned int zpos);
void dcss_crtc_attach_color_mgmt_properties(struct dcss_crtc *crtc);

#endif
