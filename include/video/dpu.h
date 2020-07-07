/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020 NXP
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

#ifndef __DRM_DPU_H__
#define __DRM_DPU_H__

#include <drm/drm_crtc.h>
#include <drm/drm_modes.h>
#include <video/videomode.h>

struct dpu_soc;

enum dpu_irq {
	IRQ_STORE9_SHDLOAD		= 0,
	IRQ_STORE9_FRAMECOMPLETE	= 1,
	IRQ_STORE9_SEQCOMPLETE		= 2,
	IRQ_EXTDST0_SHDLOAD		= 3,
	IRQ_EXTDST0_FRAMECOMPLETE	= 4,
	IRQ_EXTDST0_SEQCOMPLETE		= 5,
	IRQ_EXTDST4_SHDLOAD		= 6,
	IRQ_EXTDST4_FRAMECOMPLETE	= 7,
	IRQ_EXTDST4_SEQCOMPLETE		= 8,
	IRQ_EXTDST1_SHDLOAD		= 9,
	IRQ_EXTDST1_FRAMECOMPLETE	= 10,
	IRQ_EXTDST1_SEQCOMPLETE		= 11,
	IRQ_EXTDST5_SHDLOAD		= 12,
	IRQ_EXTDST5_FRAMECOMPLETE	= 13,
	IRQ_EXTDST5_SEQCOMPLETE		= 14,
	IRQ_DISENGCFG_SHDLOAD0		= 15,
	IRQ_DISENGCFG_FRAMECOMPLETE0	= 16,
	IRQ_DISENGCFG_SEQCOMPLETE0	= 17,
	IRQ_FRAMEGEN0_INT0		= 18,
	IRQ_FRAMEGEN0_INT1		= 19,
	IRQ_FRAMEGEN0_INT2		= 20,
	IRQ_FRAMEGEN0_INT3		= 21,
	IRQ_SIG0_SHDLOAD		= 22,
	IRQ_SIG0_VALID			= 23,
	IRQ_SIG0_ERROR			= 24,
	IRQ_DISENGCFG_SHDLOAD1		= 25,
	IRQ_DISENGCFG_FRAMECOMPLETE1	= 26,
	IRQ_DISENGCFG_SEQCOMPLETE1	= 27,
	IRQ_FRAMEGEN1_INT0		= 28,
	IRQ_FRAMEGEN1_INT1		= 29,
	IRQ_FRAMEGEN1_INT2		= 30,
	IRQ_FRAMEGEN1_INT3		= 31,
	IRQ_SIG1_SHDLOAD		= 32,
	IRQ_SIG1_VALID			= 33,
	IRQ_SIG1_ERROR			= 34,
	IRQ_RESERVED			= 35,
	IRQ_CMDSEQ_ERROR		= 36,
	IRQ_COMCTRL_SW0			= 37,
	IRQ_COMCTRL_SW1			= 38,
	IRQ_COMCTRL_SW2			= 39,
	IRQ_COMCTRL_SW3			= 40,
	IRQ_FRAMEGEN0_PRIMSYNC_ON	= 41,
	IRQ_FRAMEGEN0_PRIMSYNC_OFF	= 42,
	IRQ_FRAMEGEN0_SECSYNC_ON	= 43,
	IRQ_FRAMEGEN0_SECSYNC_OFF	= 44,
	IRQ_FRAMEGEN1_PRIMSYNC_ON	= 45,
	IRQ_FRAMEGEN1_PRIMSYNC_OFF	= 46,
	IRQ_FRAMEGEN1_SECSYNC_ON	= 47,
	IRQ_FRAMEGEN1_SECSYNC_OFF	= 48,
};

typedef enum {
	ID_NONE		= 0x00,	/*  0 */
	ID_FETCHDECODE9	= 0x01,	/*  1 */
	ID_FETCHPERSP9	= 0x02,	/*  2 */
	ID_FETCHECO9	= 0x03,	/*  3 */
	ID_ROP9		= 0x04,	/*  4 */
	ID_CLUT9	= 0x05,	/*  5 */
	ID_MATRIX9	= 0x06,	/*  6 */
	ID_HSCALER9	= 0x07,	/*  7 */
	ID_VSCALER9	= 0x08,	/*  8 */
	ID_FILTER9	= 0x09,	/*  9 */
	ID_BLITBLEND9	= 0x0A,	/* 10 */
	ID_CONSTFRAME0	= 0x0C,	/* 12 */
	ID_CONSTFRAME4	= 0x0E,	/* 14 */
	ID_CONSTFRAME1	= 0x10,	/* 16 */
	ID_CONSTFRAME5	= 0x12,	/* 18 */
	ID_FETCHWARP2	= 0x14,	/* 20 */
	ID_FETCHECO2	= 0x15,	/* 21 */
	ID_FETCHDECODE0	= 0x16,	/* 22 */
	ID_FETCHECO0	= 0x17,	/* 23 */
	ID_FETCHDECODE1	= 0x18,	/* 24 */
	ID_FETCHECO1	= 0x19,	/* 25 */
	ID_FETCHLAYER0	= 0x1a, /* 26 */
	ID_MATRIX4	= 0x1B,	/* 27 */
	ID_HSCALER4	= 0x1C,	/* 28 */
	ID_VSCALER4	= 0x1D,	/* 29 */
	ID_MATRIX5	= 0x1E,	/* 30 */
	ID_HSCALER5	= 0x1F,	/* 31 */
	ID_VSCALER5	= 0x20,	/* 32 */
	ID_LAYERBLEND0	= 0x21,	/* 33 */
	ID_LAYERBLEND1	= 0x22,	/* 34 */
	ID_LAYERBLEND2	= 0x23,	/* 35 */
	ID_LAYERBLEND3	= 0x24,	/* 36 */
} dpu_block_id_t;

typedef enum {
	DEC_SIG_SEL_FRAMEGEN = 0,
	DEC_SIG_SEL_GAMMACOR,
	DEC_SIG_SEL_MATRIX,
	DEC_SIG_SEL_DITHER,
} dec_sig_sel_t;

typedef enum {
	ED_SRC_DISABLE		= ID_NONE,
	ED_SRC_BLITBLEND9	= ID_BLITBLEND9,
	ED_SRC_CONSTFRAME0	= ID_CONSTFRAME0,
	ED_SRC_CONSTFRAME1	= ID_CONSTFRAME1,
	ED_SRC_CONSTFRAME4	= ID_CONSTFRAME4,
	ED_SRC_CONSTFRAME5	= ID_CONSTFRAME5,
	ED_SRC_MATRIX4		= ID_MATRIX4,
	ED_SRC_HSCALER4		= ID_HSCALER4,
	ED_SRC_VSCALER4		= ID_VSCALER4,
	/* content stream(extdst 0/1) only */
	ED_SRC_MATRIX5		= ID_MATRIX5,
	ED_SRC_HSCALER5		= ID_HSCALER5,
	ED_SRC_VSCALER5		= ID_VSCALER5,
	/* content stream(extdst 0/1) only */
	ED_SRC_LAYERBLEND3	= ID_LAYERBLEND3,
	ED_SRC_LAYERBLEND2	= ID_LAYERBLEND2,
	ED_SRC_LAYERBLEND1	= ID_LAYERBLEND1,
	ED_SRC_LAYERBLEND0	= ID_LAYERBLEND0,
} extdst_src_sel_t;

typedef enum {
	SINGLE,	/* Reconfig pipeline after explicit trigger */
	AUTO,	/* Reconfig pipeline after every kick when idle */
} ed_sync_mode_t;

typedef enum {
	PSTATUS_EMPTY,
	PSTATUS_RUNNING,
	PSTATUS_RUNNING_RETRIGGERED,
	PSTATUS_RESERVED
} ed_pipeline_status_t;

typedef enum {
	SOFTWARE = 0,		/* kick generation by KICK field only */
	EXTERNAL = BIT(8),	/* kick signal from external allowed */
} ed_kick_mode_t;

typedef enum {
	FD_SRC_DISABLE		= ID_NONE,
	FD_SRC_FETCHECO0	= ID_FETCHECO0,
	FD_SRC_FETCHECO1	= ID_FETCHECO1,
	FD_SRC_FETCHECO2	= ID_FETCHECO2,
	FD_SRC_FETCHDECODE0	= ID_FETCHDECODE0,
	FD_SRC_FETCHDECODE1	= ID_FETCHDECODE1,
	FD_SRC_FETCHWARP2	= ID_FETCHWARP2,
} fd_dynamic_src_sel_t;

typedef enum {
	/* RL and RLAD decoder */
	FETCHTYPE__DECODE,
	/* fractional plane(8 layers) */
	FETCHTYPE__LAYER,
	/* arbitrary warping and fractional plane(8 layers) */
	FETCHTYPE__WARP,
	/* minimum feature set for alpha, chroma and coordinate planes */
	FETCHTYPE__ECO,
	/* affine, perspective and arbitrary warping */
	FETCHTYPE__PERSP,
	/* affine and arbitrary warping */
	FETCHTYPE__ROT,
	/* RL and RLAD decoder, reduced feature set */
	FETCHTYPE__DECODEL,
	/* fractional plane(8 layers), reduced feature set */
	FETCHTYPE__LAYERL,
	/* affine and arbitrary warping, reduced feature set */
	FETCHTYPE__ROTL,
} fetchtype_t;

typedef enum {
	/* No side-by-side synchronization. */
	FGSYNCMODE__OFF = 0,
	/* Framegen is master. */
	FGSYNCMODE__MASTER = 1 << 1,
	/* Runs in cyclic synchronization mode. */
	FGSYNCMODE__SLAVE_CYC = 2 << 1,
	/* Runs in one time synchronization mode. */
	FGSYNCMODE__SLAVE_ONCE = 3 << 1,
} fgsyncmode_t;

typedef enum {
	FGDM__BLACK,
	/* Constant Color Background is shown. */
	FGDM__CONSTCOL,
	FGDM__PRIM,
	FGDM__SEC,
	FGDM__PRIM_ON_TOP,
	FGDM__SEC_ON_TOP,
	/* White color background with test pattern is shown. */
	FGDM__TEST,
} fgdm_t;

typedef enum {
	HS_SRC_SEL__DISABLE		= ID_NONE,
	HS_SRC_SEL__MATRIX9		= ID_MATRIX9,
	HS_SRC_SEL__VSCALER9		= ID_VSCALER9,
	HS_SRC_SEL__FILTER9		= ID_FILTER9,
	HS_SRC_SEL__FETCHDECODE0	= ID_FETCHDECODE0,
	HS_SRC_SEL__FETCHDECODE1	= ID_FETCHDECODE1,
	HS_SRC_SEL__MATRIX4		= ID_MATRIX4,
	HS_SRC_SEL__VSCALER4		= ID_VSCALER4,
	HS_SRC_SEL__MATRIX5		= ID_MATRIX5,
	HS_SRC_SEL__VSCALER5		= ID_VSCALER5,
} hs_src_sel_t;

typedef enum {
	/* common options */
	LB_PRIM_SEL__DISABLE		= ID_NONE,
	LB_PRIM_SEL__BLITBLEND9		= ID_BLITBLEND9,
	LB_PRIM_SEL__CONSTFRAME0	= ID_CONSTFRAME0,
	LB_PRIM_SEL__CONSTFRAME1	= ID_CONSTFRAME1,
	LB_PRIM_SEL__CONSTFRAME4	= ID_CONSTFRAME4,
	LB_PRIM_SEL__CONSTFRAME5	= ID_CONSTFRAME5,
	LB_PRIM_SEL__MATRIX4		= ID_MATRIX4,
	LB_PRIM_SEL__HSCALER4		= ID_HSCALER4,
	LB_PRIM_SEL__VSCALER4		= ID_VSCALER4,
	LB_PRIM_SEL__MATRIX5		= ID_MATRIX5,
	LB_PRIM_SEL__HSCALER5		= ID_HSCALER5,
	LB_PRIM_SEL__VSCALER5		= ID_VSCALER5,
	/*
	 * special options:
	 * layerblend(n) has n special options,
	 * from layerblend0 to layerblend(n - 1), e.g.,
	 * layerblend3 has 3 special options -
	 * layerblend0/1/2.
	 */
	LB_PRIM_SEL__LAYERBLEND3	= ID_LAYERBLEND3,
	LB_PRIM_SEL__LAYERBLEND2	= ID_LAYERBLEND2,
	LB_PRIM_SEL__LAYERBLEND1	= ID_LAYERBLEND1,
	LB_PRIM_SEL__LAYERBLEND0	= ID_LAYERBLEND0,
} lb_prim_sel_t;

typedef enum {
	LB_SEC_SEL__DISABLE		= ID_NONE,
	LB_SEC_SEL__FETCHWARP2		= ID_FETCHWARP2,
	LB_SEC_SEL__FETCHDECODE0	= ID_FETCHDECODE0,
	LB_SEC_SEL__FETCHDECODE1	= ID_FETCHDECODE1,
	LB_SEC_SEL__MATRIX4		= ID_MATRIX4,
	LB_SEC_SEL__HSCALER4		= ID_HSCALER4,
	LB_SEC_SEL__VSCALER4		= ID_VSCALER4,
	LB_SEC_SEL__MATRIX5		= ID_MATRIX5,
	LB_SEC_SEL__HSCALER5		= ID_HSCALER5,
	LB_SEC_SEL__VSCALER5		= ID_VSCALER5,
	LB_SEC_SEL__FETCHLAYER0		= ID_FETCHLAYER0,
} lb_sec_sel_t;

typedef enum {
	PRIMARY,	/* background plane */
	SECONDARY,	/* foreground plane */
	BOTH,
} lb_shadow_sel_t;

typedef enum {
	LB_NEUTRAL,	/* Output is same as primary input. */
	LB_BLEND,
} lb_mode_t;

typedef enum {
	/* Constant 0 indicates frame or top field. */
	SCALER_ALWAYS0 = 0x0,
	/* Constant 1 indicates bottom field. */
	SCALER_ALWAYS1 = 0x1 << 12,
	/* Output field polarity is taken from input field polarity. */
	SCALER_INPUT = 0x2 << 12,
	/* Output field polarity toggles, starting with 0 after reset. */
	SCALER_TOGGLE = 0x3 << 12,
} scaler_field_mode_t;

typedef enum {
	/* pointer-sampling */
	SCALER_NEAREST = 0x0,
	/* box filter */
	SCALER_LINEAR = 0x100,
} scaler_filter_mode_t;

typedef enum {
	SCALER_DOWNSCALE = 0x0,
	SCALER_UPSCALE = 0x10,
} scaler_scale_mode_t;

typedef enum {
	/* Pixel by-pass the scaler, all other settings are ignored. */
	SCALER_NEUTRAL = 0x0,
	/* Scaler is active. */
	SCALER_ACTIVE = 0x1,
} scaler_mode_t;

typedef enum {
	VS_SRC_SEL__DISABLE		= ID_NONE,
	VS_SRC_SEL__MATRIX9		= ID_MATRIX9,
	VS_SRC_SEL__HSCALER9		= ID_HSCALER9,
	VS_SRC_SEL__FETCHDECODE0	= ID_FETCHDECODE0,
	VS_SRC_SEL__FETCHDECODE1	= ID_FETCHDECODE1,
	VS_SRC_SEL__MATRIX4		= ID_MATRIX4,
	VS_SRC_SEL__HSCALER4		= ID_HSCALER4,
	VS_SRC_SEL__MATRIX5		= ID_MATRIX5,
	VS_SRC_SEL__HSCALER5		= ID_HSCALER5,
} vs_src_sel_t;

#define CLKEN_MASK		(0x3 << 24)
#define CLKEN_MASK_SHIFT	24
typedef enum {
	CLKEN__DISABLE = 0x0,
	CLKEN__AUTOMATIC = 0x1,
	CLKEN__FULL = 0x3,
} pixengcfg_clken_t;

/* fetch unit types */
enum {
	FU_T_NA,
	FU_T_FD,
	FU_T_FE,
	FU_T_FL,
	FU_T_FW,
};

enum dpu_crc_source {
	DPU_CRC_SRC_NONE,
	DPU_CRC_SRC_FRAMEGEN,
	DPU_CRC_SRC_FRAMEGEN_ROI,
};

struct dpu_fetchunit;

struct dpu_fetchunit_ops {
	void (*set_burstlength)(struct dpu_fetchunit *fu,
			        unsigned int x_offset, unsigned int mt_w,
			        int bpp, dma_addr_t baddr, bool use_prefetch);

	void (*set_baseaddress)(struct dpu_fetchunit *fu, unsigned int width,
			        unsigned int x_offset, unsigned int y_offset,
			        unsigned int mt_w, unsigned int mt_h,
			        int bpp, dma_addr_t baddr);

	void (*set_src_bpp)(struct dpu_fetchunit *fu, int bpp);

	void (*set_src_stride)(struct dpu_fetchunit *fu,
			       unsigned int width, unsigned int x_offset,
			       unsigned int mt_w, int bpp, unsigned int stride,
			       dma_addr_t baddr, bool use_prefetch);

	void (*set_src_buf_dimensions)(struct dpu_fetchunit *fu,
				       unsigned int w, unsigned int h, u32 fmt,
				       bool deinterlace);

	void (*set_fmt)(struct dpu_fetchunit *fu, u32 fmt,
			enum drm_color_encoding color_encoding,
			enum drm_color_range color_range,
			bool deinterlace);

	void (*set_pixel_blend_mode)(struct dpu_fetchunit *fu,
				     unsigned int pixel_blend_mode, u16 alpha,
				     u32 fb_format);

	void (*enable_src_buf)(struct dpu_fetchunit *fu);
	void (*disable_src_buf)(struct dpu_fetchunit *fu);
	bool (*is_enabled)(struct dpu_fetchunit *fu);

	void (*set_framedimensions)(struct dpu_fetchunit *fu,
				    unsigned int w, unsigned int h,
				    bool deinterlace);

	void (*set_controltrigger)(struct dpu_fetchunit *fu);

	unsigned int (*get_stream_id)(struct dpu_fetchunit *fu);
	void (*set_stream_id)(struct dpu_fetchunit *fu, unsigned int id);
};

struct dpu_fetchunit {
	void __iomem *pec_base;
	void __iomem *base;
	char *name;
	struct mutex mutex;
	int id;
	int sub_id;	/* for fractional fetch units */
	int type;
	bool inuse;
	struct dpu_soc *dpu;
	/* see DPU_PLANE_SRC_xxx */
	unsigned int stream_id;
	struct dprc *dprc;
	const struct dpu_fetchunit_ops *ops;
};

int dpu_map_irq(struct dpu_soc *dpu, int irq);

/* Constant Frame Unit */
struct dpu_constframe;
void constframe_shden(struct dpu_constframe *cf, bool enable);
void constframe_framedimensions(struct dpu_constframe *cf, unsigned int w,
				unsigned int h);
void constframe_framedimensions_copy_prim(struct dpu_constframe *cf);
void constframe_constantcolor(struct dpu_constframe *cf, unsigned int r,
			      unsigned int g, unsigned int b, unsigned int a);
void constframe_controltrigger(struct dpu_constframe *cf, bool trigger);
struct dpu_constframe *dpu_cf_get(struct dpu_soc *dpu, int id);
void dpu_cf_put(struct dpu_constframe *cf);
struct dpu_constframe *dpu_aux_cf_peek(struct dpu_constframe *cf);

/* Display Engine Configuration Unit */
struct dpu_disengcfg;
void disengcfg_sig_select(struct dpu_disengcfg *dec, dec_sig_sel_t sig_sel);
struct dpu_disengcfg *dpu_dec_get(struct dpu_soc *dpu, int id);
void dpu_dec_put(struct dpu_disengcfg *dec);
struct dpu_disengcfg *dpu_aux_dec_peek(struct dpu_disengcfg *dec);

/* External Destination Unit */
struct dpu_extdst;
void extdst_pixengcfg_shden(struct dpu_extdst *ed, bool enable);
void extdst_pixengcfg_powerdown(struct dpu_extdst *ed, bool powerdown);
void extdst_pixengcfg_sync_mode(struct dpu_extdst *ed, ed_sync_mode_t mode);
void extdst_pixengcfg_reset(struct dpu_extdst *ed, bool reset);
void extdst_pixengcfg_div(struct dpu_extdst *ed, u16 div);
void extdst_pixengcfg_syncmode_master(struct dpu_extdst *ed, bool enable);
int extdst_pixengcfg_src_sel(struct dpu_extdst *ed, extdst_src_sel_t src);
void extdst_pixengcfg_sel_shdldreq(struct dpu_extdst *ed);
void extdst_pixengcfg_shdldreq(struct dpu_extdst *ed, u32 req_mask);
void extdst_pixengcfg_sync_trigger(struct dpu_extdst *ed);
void extdst_pixengcfg_trigger_sequence_complete(struct dpu_extdst *ed);
bool extdst_pixengcfg_is_sync_busy(struct dpu_extdst *ed);
ed_pipeline_status_t extdst_pixengcfg_pipeline_status(struct dpu_extdst *ed);
void extdst_shden(struct dpu_extdst *ed, bool enable);
void extdst_kick_mode(struct dpu_extdst *ed, ed_kick_mode_t mode);
void extdst_perfcountmode(struct dpu_extdst *ed, bool enable);
void extdst_gamma_apply_enable(struct dpu_extdst *ed, bool enable);
void extdst_kick(struct dpu_extdst *ed);
void extdst_cnt_err_clear(struct dpu_extdst *ed);
bool extdst_cnt_err_status(struct dpu_extdst *ed);
u32 extdst_last_control_word(struct dpu_extdst *ed);
void extdst_pixel_cnt(struct dpu_extdst *ed, u16 *x, u16 *y);
void extdst_last_pixel_cnt(struct dpu_extdst *ed, u16 *x, u16 *y);
u32 extdst_perfresult(struct dpu_extdst *ed);
bool extdst_is_master(struct dpu_extdst *ed);
struct dpu_extdst *dpu_ed_get(struct dpu_soc *dpu, int id);
void dpu_ed_put(struct dpu_extdst *ed);
struct dpu_extdst *dpu_aux_ed_peek(struct dpu_extdst *ed);

/* Fetch Decode Unit */
int fetchdecode_pixengcfg_dynamic_src_sel(struct dpu_fetchunit *fu,
					  fd_dynamic_src_sel_t src);
void fetchdecode_layeroffset(struct dpu_fetchunit *fd, unsigned int x,
			     unsigned int y);
void fetchdecode_clipoffset(struct dpu_fetchunit *fd, unsigned int x,
			    unsigned int y);
void fetchdecode_clipdimensions(struct dpu_fetchunit *fd, unsigned int w,
				unsigned int h);
void fetchdecode_rgb_constantcolor(struct dpu_fetchunit *fd,
					u8 r, u8 g, u8 b, u8 a);
void fetchdecode_yuv_constantcolor(struct dpu_fetchunit *fd,
					u8 y, u8 u, u8 v);
int fetchdecode_fetchtype(struct dpu_fetchunit *fd, fetchtype_t *type);
u32 fetchdecode_get_vproc_mask(struct dpu_fetchunit *fd);
bool fetchdecode_need_fetcheco(struct dpu_fetchunit *fd, u32 fmt);
struct dpu_fetchunit *dpu_fd_get(struct dpu_soc *dpu, int id);
void dpu_fd_put(struct dpu_fetchunit *fu);

/* Fetch ECO Unit */
void fetcheco_layeroffset(struct dpu_fetchunit *fu, unsigned int x,
			  unsigned int y);
void fetcheco_clipoffset(struct dpu_fetchunit *fu, unsigned int x,
			 unsigned int y);
void fetcheco_clipdimensions(struct dpu_fetchunit *fu, unsigned int w,
			     unsigned int h);
void fetcheco_frameresampling(struct dpu_fetchunit *fu, unsigned int x,
			      unsigned int y);
int fetcheco_fetchtype(struct dpu_fetchunit *fu, fetchtype_t *type);
dpu_block_id_t fetcheco_get_block_id(struct dpu_fetchunit *fu);
struct dpu_fetchunit *dpu_fe_get(struct dpu_soc *dpu, int id);
void dpu_fe_put(struct dpu_fetchunit *fu);

/* Fetch Layer Unit */
void fetchlayer_rgb_constantcolor(struct dpu_fetchunit *fu,
					u8 r, u8 g, u8 b, u8 a);
void fetchlayer_yuv_constantcolor(struct dpu_fetchunit *fu, u8 y, u8 u, u8 v);
int fetchlayer_fetchtype(struct dpu_fetchunit *fu, fetchtype_t *type);
struct dpu_fetchunit *dpu_fl_get(struct dpu_soc *dpu, int id);
void dpu_fl_put(struct dpu_fetchunit *fu);

/* Fetch Warp Unit */
void fetchwarp_rgb_constantcolor(struct dpu_fetchunit *fu,
				 u8 r, u8 g, u8 b, u8 a);
void fetchwarp_yuv_constantcolor(struct dpu_fetchunit *fu, u8 y, u8 u, u8 v);
int fetchwarp_fetchtype(struct dpu_fetchunit *fu, fetchtype_t *type);
struct dpu_fetchunit *dpu_fw_get(struct dpu_soc *dpu, int id);
void dpu_fw_put(struct dpu_fetchunit *fu);

/* Frame Generator Unit */
struct dpu_framegen;
void framegen_enable(struct dpu_framegen *fg);
void framegen_disable(struct dpu_framegen *fg);
void framegen_enable_pixel_link(struct dpu_framegen *fg);
void framegen_disable_pixel_link(struct dpu_framegen *fg);
void framegen_shdtokgen(struct dpu_framegen *fg);
void framegen_syncmode(struct dpu_framegen *fg, fgsyncmode_t mode);
void framegen_cfg_videomode(struct dpu_framegen *fg, struct drm_display_mode *m,
			    bool side_by_side, unsigned int encoder_type);
void framegen_pkickconfig(struct dpu_framegen *fg, bool enable);
void framegen_syncmode_fixup(struct dpu_framegen *fg, bool enable);
void framegen_displaymode(struct dpu_framegen *fg, fgdm_t mode);
void framegen_panic_displaymode(struct dpu_framegen *fg, fgdm_t mode);
void framegen_wait_done(struct dpu_framegen *fg, struct drm_display_mode *m);
void framegen_read_timestamp(struct dpu_framegen *fg,
			     u32 *frame_index, u32 *line_index);
void framegen_wait_for_frame_counter_moving(struct dpu_framegen *fg);
bool framegen_secondary_requests_to_read_empty_fifo(struct dpu_framegen *fg);
void framegen_secondary_clear_channel_status(struct dpu_framegen *fg);
bool framegen_secondary_is_syncup(struct dpu_framegen *fg);
void framegen_wait_for_secondary_syncup(struct dpu_framegen *fg);
void framegen_enable_clock(struct dpu_framegen *fg);
void framegen_disable_clock(struct dpu_framegen *fg);
bool framegen_is_master(struct dpu_framegen *fg);
bool framegen_is_slave(struct dpu_framegen *fg);
struct dpu_framegen *dpu_fg_get(struct dpu_soc *dpu, int id);
void dpu_fg_put(struct dpu_framegen *fg);
struct dpu_framegen *dpu_aux_fg_peek(struct dpu_framegen *fg);

/* Horizontal Scaler Unit */
struct dpu_hscaler;
int hscaler_pixengcfg_dynamic_src_sel(struct dpu_hscaler *hs, hs_src_sel_t src);
void hscaler_pixengcfg_clken(struct dpu_hscaler *hs, pixengcfg_clken_t clken);
void hscaler_shden(struct dpu_hscaler *hs, bool enable);
void hscaler_setup1(struct dpu_hscaler *hs, unsigned int src, unsigned int dst);
void hscaler_setup2(struct dpu_hscaler *hs, u32 phase_offset);
void hscaler_output_size(struct dpu_hscaler *hs, u32 line_num);
void hscaler_filter_mode(struct dpu_hscaler *hs, scaler_filter_mode_t m);
void hscaler_scale_mode(struct dpu_hscaler *hs, scaler_scale_mode_t m);
void hscaler_mode(struct dpu_hscaler *hs, scaler_mode_t m);
bool hscaler_is_enabled(struct dpu_hscaler *hs);
dpu_block_id_t hscaler_get_block_id(struct dpu_hscaler *hs);
unsigned int hscaler_get_stream_id(struct dpu_hscaler *hs);
void hscaler_set_stream_id(struct dpu_hscaler *hs, unsigned int id);
struct dpu_hscaler *dpu_hs_get(struct dpu_soc *dpu, int id);
void dpu_hs_put(struct dpu_hscaler *hs);

/* Layer Blend Unit */
struct dpu_layerblend;
int layerblend_pixengcfg_dynamic_prim_sel(struct dpu_layerblend *lb,
					  lb_prim_sel_t prim);
void layerblend_pixengcfg_dynamic_sec_sel(struct dpu_layerblend *lb,
					  lb_sec_sel_t sec);
void layerblend_pixengcfg_clken(struct dpu_layerblend *lb,
				pixengcfg_clken_t clken);
void layerblend_shden(struct dpu_layerblend *lb, bool enable);
void layerblend_shdtoksel(struct dpu_layerblend *lb, lb_shadow_sel_t sel);
void layerblend_shdldsel(struct dpu_layerblend *lb, lb_shadow_sel_t sel);
void layerblend_control(struct dpu_layerblend *lb, lb_mode_t mode);
void layerblend_blendcontrol(struct dpu_layerblend *lb, unsigned int zpos,
			     unsigned int pixel_blend_mode, u16 alpha);
void layerblend_position(struct dpu_layerblend *lb, int x, int y);
struct dpu_layerblend *dpu_lb_get(struct dpu_soc *dpu, int id);
void dpu_lb_put(struct dpu_layerblend *lb);

/* Signature Unit */
#define MAX_DPU_SIGNATURE_WIN_NUM	8
struct dpu_signature;
void signature_shden(struct dpu_signature *sig, bool enable);
void signature_shdldsel_local(struct dpu_signature *sig);
void signature_shdldsel_global(struct dpu_signature *sig);
void
signature_global_panic(struct dpu_signature *sig, unsigned int win, bool enable);
void
signature_local_panic(struct dpu_signature *sig, unsigned int win, bool enable);
void
signature_alpha_mask(struct dpu_signature *sig, unsigned int win, bool enable);
void signature_crc(struct dpu_signature *sig, unsigned int win, bool enable);
void
signature_eval_win(struct dpu_signature *sig, unsigned int win, bool enable);
void signature_win(struct dpu_signature *sig, unsigned int win,
		   int xul, int yul, int xlr, int ylr);
void signature_crc_value(struct dpu_signature *sig, unsigned int win,
			 u32 *red, u32 *green, u32 *blue);
void signature_shdldreq(struct dpu_signature *sig, u8 win_mask);
void signature_continuous_mode(struct dpu_signature *sig, bool enable);
void signature_kick(struct dpu_signature *sig);
bool signature_is_idle(struct dpu_signature *sig);
void signature_wait_for_idle(struct dpu_signature *sig);
bool signature_is_valid(struct dpu_signature *sig);
bool signature_is_error(struct dpu_signature *sig, u8 *err_win_mask);
struct dpu_signature *dpu_sig_get(struct dpu_soc *dpu, int id);
void dpu_sig_put(struct dpu_signature *sig);
struct dpu_signature *dpu_aux_sig_peek(struct dpu_signature *sig);

/* Store Unit */
struct dpu_store;
void store_pixengcfg_syncmode_fixup(struct dpu_store *st, bool enable);
struct dpu_store *dpu_st_get(struct dpu_soc *dpu, int id);
void dpu_st_put(struct dpu_store *st);

/* Timing Controller Unit */
struct dpu_tcon;
int tcon_set_fmt(struct dpu_tcon *tcon, u32 bus_format);
void tcon_set_operation_mode(struct dpu_tcon *tcon);
void tcon_cfg_videomode(struct dpu_tcon *tcon,
			struct drm_display_mode *m, bool side_by_side);
bool tcon_is_master(struct dpu_tcon *tcon);
bool tcon_is_slave(struct dpu_tcon *tcon);
void tcon_configure_pc(struct dpu_tcon *tcon, unsigned int di,
			unsigned int frame_width, u32 mode, u32 format);
void tcon_enable_pc(struct dpu_tcon *tcon);
void tcon_disable_pc(struct dpu_tcon *tcon);
struct dpu_tcon *dpu_tcon_get(struct dpu_soc *dpu, int id);
void dpu_tcon_put(struct dpu_tcon *tcon);
struct dpu_tcon *dpu_aux_tcon_peek(struct dpu_tcon *tcon);

/* Vertical Scaler Unit */
struct dpu_vscaler;
int vscaler_pixengcfg_dynamic_src_sel(struct dpu_vscaler *vs, vs_src_sel_t src);
void vscaler_pixengcfg_clken(struct dpu_vscaler *vs, pixengcfg_clken_t clken);
void vscaler_shden(struct dpu_vscaler *vs, bool enable);
void vscaler_setup1(struct dpu_vscaler *vs, u32 src, u32 dst, bool deinterlace);
void vscaler_setup2(struct dpu_vscaler *vs, bool deinterlace);
void vscaler_setup3(struct dpu_vscaler *vs, bool deinterlace);
void vscaler_setup4(struct dpu_vscaler *vs, u32 phase_offset);
void vscaler_setup5(struct dpu_vscaler *vs, u32 phase_offset);
void vscaler_output_size(struct dpu_vscaler *vs, u32 line_num);
void vscaler_field_mode(struct dpu_vscaler *vs, scaler_field_mode_t m);
void vscaler_filter_mode(struct dpu_vscaler *vs, scaler_filter_mode_t m);
void vscaler_scale_mode(struct dpu_vscaler *vs, scaler_scale_mode_t m);
void vscaler_mode(struct dpu_vscaler *vs, scaler_mode_t m);
bool vscaler_is_enabled(struct dpu_vscaler *vs);
dpu_block_id_t vscaler_get_block_id(struct dpu_vscaler *vs);
unsigned int vscaler_get_stream_id(struct dpu_vscaler *vs);
void vscaler_set_stream_id(struct dpu_vscaler *vs, unsigned int id);
struct dpu_vscaler *dpu_vs_get(struct dpu_soc *dpu, int id);
void dpu_vs_put(struct dpu_vscaler *vs);

struct dpu_fetchunit *fetchdecode_get_fetcheco(struct dpu_fetchunit *fu);
struct dpu_hscaler *fetchdecode_get_hscaler(struct dpu_fetchunit *fu);
struct dpu_vscaler *fetchdecode_get_vscaler(struct dpu_fetchunit *fu);

unsigned int dpu_get_syncmode_min_prate(struct dpu_soc *dpu);
unsigned int dpu_get_singlemode_max_width(struct dpu_soc *dpu);
unsigned int dpu_get_master_stream_id(struct dpu_soc *dpu);

bool dpu_vproc_has_fetcheco_cap(u32 cap_mask);
bool dpu_vproc_has_hscale_cap(u32 cap_mask);
bool dpu_vproc_has_vscale_cap(u32 cap_mask);

u32 dpu_vproc_get_fetcheco_cap(u32 cap_mask);
u32 dpu_vproc_get_hscale_cap(u32 cap_mask);
u32 dpu_vproc_get_vscale_cap(u32 cap_mask);

unsigned int fetchunit_burst_size_fixup_tkt343664(dma_addr_t baddr);
unsigned int
fetchunit_stride_fixup_tkt339017(unsigned int stride, unsigned int burst_size,
				 dma_addr_t baddr, bool nonzero_mod);
void fetchunit_get_dprc(struct dpu_fetchunit *fu, void *data);
void fetchunit_shden(struct dpu_fetchunit *fu, bool enable);
void fetchunit_baddr_autoupdate(struct dpu_fetchunit *fu, u8 layer_mask);
void fetchunit_shdldreq_sticky(struct dpu_fetchunit *fu, u8 layer_mask);
void fetchunit_set_burstlength(struct dpu_fetchunit *fu,
			       unsigned int x_offset, unsigned int mt_w,
			       int bpp, dma_addr_t baddr, bool use_prefetch);
void fetchunit_set_baseaddress(struct dpu_fetchunit *fu, unsigned int width,
			       unsigned int x_offset, unsigned int y_offset,
			       unsigned int mt_w, unsigned int mt_h,
			       int bpp, dma_addr_t baddr);
void fetchunit_set_src_bpp(struct dpu_fetchunit *fu, int bpp);
void fetchunit_set_src_stride(struct dpu_fetchunit *fu,
			      unsigned int width, unsigned int x_offset,
			      unsigned int mt_w, int bpp, unsigned int stride,
			      dma_addr_t baddr, bool use_prefetch);
void fetchunit_set_pixel_blend_mode(struct dpu_fetchunit *fu,
				    unsigned int pixel_blend_mode, u16 alpha,
				    u32 fb_format);
void fetchunit_enable_src_buf(struct dpu_fetchunit *fu);
void fetchunit_disable_src_buf(struct dpu_fetchunit *fu);
bool fetchunit_is_enabled(struct dpu_fetchunit *fu);
unsigned int fetchunit_get_stream_id(struct dpu_fetchunit *fu);
void fetchunit_set_stream_id(struct dpu_fetchunit *fu, unsigned int id);
bool fetchunit_is_fetchdecode(struct dpu_fetchunit *fu);
bool fetchunit_is_fetcheco(struct dpu_fetchunit *fu);
bool fetchunit_is_fetchlayer(struct dpu_fetchunit *fu);
bool fetchunit_is_fetchwarp(struct dpu_fetchunit *fu);

/*
 * to avoid on-the-fly/hot plane resource migration
 * between two display interfaces
 */
#define DPU_PLANE_SRC_TO_DISP_STREAM0	BIT(0)
#define DPU_PLANE_SRC_TO_DISP_STREAM1	BIT(1)
#define DPU_PLANE_SRC_DISABLED		0

struct dpu_plane_res {
	struct dpu_extdst	*ed[2];
	struct dpu_fetchunit	*fd[2];
	struct dpu_fetchunit	*fe[2];
	struct dpu_fetchunit	*fl[1];
	struct dpu_fetchunit	*fw[1];
	struct dpu_framegen	*fg[2];
	struct dpu_hscaler	*hs[2];
	struct dpu_layerblend	*lb[4];
	struct dpu_vscaler	*vs[2];
};

/*
 * Each DPU plane can be a primary plane or an overlay plane
 * of one of the DPU's two CRTCs.
 */
#define	DPU_PLANE_SRC_FL0_ID	BIT(0)
#define	DPU_PLANE_SRC_FW2_ID	BIT(1)
#define	DPU_PLANE_SRC_FD0_ID	BIT(2)
#define	DPU_PLANE_SRC_FD1_ID	BIT(3)

struct dpu_plane_grp {
	struct dpu_plane_res	res;
	unsigned int		hw_plane_num;
	unsigned int		hw_plane_fetcheco_num;
	unsigned int		hw_plane_hscaler_num;
	unsigned int		hw_plane_vscaler_num;
	unsigned int		id;
	bool			has_vproc;

	/* used when assigning plane source */
	struct mutex		mutex;
	u32			src_mask;
	u32			src_a_mask;
	u32			src_use_vproc_mask;
};

static inline struct dpu_plane_grp *plane_res_to_grp(struct dpu_plane_res *res)
{
	return container_of(res, struct dpu_plane_grp, res);
}

struct dpu_client_platformdata {
	const unsigned int	stream_id;
	unsigned int		di_grp_id;
	struct dpu_plane_grp	*plane_grp;

	/* Store9 could be shared bewteen display engine and blit engine */
	struct dpu_store	*st9;

	struct device_node	*of_node;
};
#endif /* __DRM_DPU_H__ */
