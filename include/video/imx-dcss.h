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

#ifndef __IMX_DCSS_H__
#define __IMX_DCSS_H__

#include <linux/types.h>
#include <video/videomode.h>
#include <drm/drm_rect.h>

struct dcss_soc;

struct dcss_client_platformdata {
	struct device_node *of_node;
};

/* COMMON */
int dcss_vblank_irq_get(struct dcss_soc *dcss);
void dcss_vblank_irq_enable(struct dcss_soc *dcss, bool en);
void dcss_vblank_irq_clear(struct dcss_soc *dcss);
enum dcss_color_space dcss_drm_fourcc_to_colorspace(u32 drm_fourcc);
void dcss_trace_write(u64 tag);


#define TAG(x)			((x) << 56)

#define TRACE_COMMON		TAG(0LL)
#define TRACE_DTG		TAG(1LL)
#define TRACE_SS		TAG(2LL)
#define TRACE_DPR		TAG(3LL)
#define TRACE_SCALER		TAG(4LL)
#define TRACE_CTXLD		TAG(5LL)
#define TRACE_DEC400D		TAG(6LL)
#define TRACE_DTRC		TAG(7LL)
#define TRACE_HDR10		TAG(8LL)
#define TRACE_RDSRC		TAG(9LL)
#define TRACE_WRSCL		TAG(10LL)

#define TRACE_DRM_CRTC		TAG(11LL)
#define TRACE_DRM_PLANE		TAG(12LL)
#define TRACE_DRM_KMS		TAG(13LL)

#define dcss_trace_module(mod_tag, val) dcss_trace_write((mod_tag) | (val));

/* BLKCTL */
void dcss_blkctl_hdmi_secure_src_en(struct dcss_soc *dcss);

/* DPR */
enum dcss_tile_type {
	TILE_LINEAR = 0,
	TILE_GPU_STANDARD,
	TILE_GPU_SUPER,
	TILE_VPU_YUV420,
	TILE_VPU_VP9,
};

enum dcss_pix_size {
	PIX_SIZE_8,
	PIX_SIZE_16,
	PIX_SIZE_32,
};

void dcss_dpr_set_res(struct dcss_soc *dcss, int ch_num, u32 xres, u32 yres,
		      u32 adj_w, u32 adj_h);
void dcss_dpr_addr_set(struct dcss_soc *dcss, int ch_num, u32 luma_base_addr,
		       u32 chroma_base_addr, u16 pitch);
void dcss_dpr_enable(struct dcss_soc *dcss, int ch_num, bool en);
void dcss_dpr_format_set(struct dcss_soc *dcss, int ch_num, u32 pix_format,
			 bool modifiers_present);
void dcss_dpr_tile_derive(struct dcss_soc *dcss,
			  int ch_num,
			  uint64_t modifier);
void dcss_dpr_set_rotation(struct dcss_soc *dcss, int ch_num, u32 rotation);

/* DTG */
void dcss_dtg_sync_set(struct dcss_soc *dcss, struct videomode *vm);
void dcss_dtg_plane_pos_set(struct dcss_soc *dcss, int ch_num,
			    int px, int py, int pw, int ph);
void dcss_dtg_enable(struct dcss_soc *dcss, bool en,
		     struct completion *dis_completion);
bool dcss_dtg_is_enabled(struct dcss_soc *dcss);
void dcss_dtg_ch_enable(struct dcss_soc *dcss, int ch_num, bool en);
void dcss_dtg_plane_alpha_set(struct dcss_soc *dcss, int ch_num,
			      u32 pix_format, int alpha, bool use_global_alpha);
bool dcss_dtg_global_alpha_changed(struct dcss_soc *dcss, int ch_num,
				   u32 pix_format, int alpha,
				   int use_global_alpha);
void dcss_dtg_css_set(struct dcss_soc *dcss, u32 pix_format);

/* SUBSAM */
void dcss_ss_sync_set(struct dcss_soc *dcss, struct videomode *vm,
		      bool phsync, bool pvsync);
void dcss_ss_subsam_set(struct dcss_soc *dcss, u32 pix_format);
void dcss_ss_enable(struct dcss_soc *dcss, bool en);

/* SCALER */
void dcss_scaler_enable(struct dcss_soc *dcss, int ch_num, bool en);
void dcss_scaler_setup(struct dcss_soc *dcss, int ch_num, u32 pix_format,
		       int src_xres, int src_yres, int dst_xres, int dst_yres,
		       u32 vrefresh_hz);
bool dcss_scaler_can_scale(struct dcss_soc *dcss, int ch_num,
			   int src_xres, int src_yres,
			   int dst_xres, int dst_yres);

/* CTXLD */
int dcss_ctxld_enable(struct dcss_soc *dcss);
bool dcss_ctxld_is_flushed(struct dcss_soc *dcss);

/* HDR10 */
enum dcss_hdr10_nonlinearity {
	NL_REC2084,
	NL_REC709,
	NL_BT1886,
	NL_2100HLG,
	NL_SRGB,
};

enum dcss_hdr10_pixel_range {
	PR_LIMITED,
	PR_FULL,
};

enum dcss_hdr10_gamut {
	G_REC2020,
	G_REC709,
	G_REC601_NTSC,
	G_REC601_PAL,
	G_ADOBE_ARGB,
};

struct dcss_hdr10_pipe_cfg {
	u32 pixel_format;
	enum dcss_hdr10_nonlinearity nl;
	enum dcss_hdr10_pixel_range pr;
	enum dcss_hdr10_gamut g;
};

void dcss_hdr10_setup(struct dcss_soc *dcss, int ch_num,
		      struct dcss_hdr10_pipe_cfg *ipipe_cfg,
		      struct dcss_hdr10_pipe_cfg *opipe_cfg);

/* DTRC */
void dcss_dtrc_bypass(struct dcss_soc *dcss, int ch_num);
void dcss_dtrc_set_res(struct dcss_soc *dcss, int ch_num, struct drm_rect *src,
		       struct drm_rect *old_src, u32 pixel_format);
void dcss_dtrc_addr_set(struct dcss_soc *dcss, int ch_num, u32 p1_ba, u32 p2_ba,
			uint64_t dec_table_ofs);
void dcss_dtrc_enable(struct dcss_soc *dcss, int ch_num, bool enable);
void dcss_dtrc_set_format_mod(struct dcss_soc *dcss, int ch_num, u64 modifier);

enum dcss_color_space {
	DCSS_COLORSPACE_RGB,
	DCSS_COLORSPACE_YUV,
	DCSS_COLORSPACE_UNKNOWN,
};

/* DEC400D */
void dcss_dec400d_set_format_mod(struct dcss_soc *dcss,
				 uint32_t fourcc,
				 uint32_t mod_idx,
				 uint64_t modifier);
void dcss_dec400d_bypass(struct dcss_soc *dcss);
void dcss_dec400d_shadow_trig(struct dcss_soc *dcss);
void dcss_dec400d_addr_set(struct dcss_soc *dcss,
			   uint32_t baddr,
			   uint32_t caddr);
void dcss_dec400d_read_config(struct dcss_soc *dcss,
			      uint32_t read_id,
			      bool compress_en,
			      uint32_t compress_format);
void dcss_dec400d_fast_clear_config(struct dcss_soc *dcss,
                                    uint32_t fc_value,
                                    bool enable);
void dcss_dec400d_enable(struct dcss_soc *dcss);
#endif /* __IMX_DCSS_H__ */
