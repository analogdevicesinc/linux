/* SPDX-License-Identifier: GPL-2.0+ */

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020,2023 NXP
 */

#ifndef __DRM_DPU95_H__
#define __DRM_DPU95_H__

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include <drm/drm_color_mgmt.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_modes.h>

/* IRQ register */
#define INTERRUPTENABLE(n)		(0x8 + 0x4 * (n))
#define INTERRUPTPRESET(n)		(0x14 + 0x4 * (n))
#define INTERRUPTCLEAR(n)		(0x20 + 0x4 * (n))
#define INTERRUPTSTATUS(n)		(0x2c + 0x4 * (n))

/* Domain Mask register */
#define STORE9_DM			0x8
#define EXTDST0_DM			0xc
#define EXTDST4_DM			0x10
#define EXTDST1_DM			0x14
#define EXTDST5_DM			0x18
#define  ALLOW_ALL			0xffffffff

#define DISPLAY_STATIC			0x1c
#define  PIPELINE_SYNC			BIT(0)

#define EXTDST0_STATIC			0x20
#define EXTDST4_STATIC			0x24
#define EXTDST1_STATIC			0x28
#define EXTDST5_STATIC			0x2c
#define  MASTER				BIT(15)

#define DPU95_SAFETY_STREAM_OFFSET	4

/* shadow enable bit for several DPU units */
#define SHDEN				BIT(0)

/* Pixel Engine Configuration register fields */
#define CLKEN_MASK_SHIFT		24
#define CLKEN_MASK			(0x3 << 24)
#define CLKEN(n)			((n) << CLKEN_MASK_SHIFT)

/* Hscaler register fields */
#define SCALE_FACTOR_MASK		0xfffff
#define SCALE_FACTOR(n)			((n) & 0xfffff)
#define PHASE_OFFSET_MASK		0x1fffff
#define PHASE_OFFSET(n)			((n) & 0x1fffff)
#define OUTPUT_SIZE_MASK		0x3fff0000
#define OUTPUT_SIZE(n)			((((n) - 1) << 16) & OUTPUT_SIZE_MASK)
#define FILTER_MODE_MASK		0x100
#define FILTER_MODE(n)			((n) << 8)
#define SCALE_MODE_MASK			0x10
#define SCALE_MODE(n)			((n) << 4)
#define CTRL_MODE_MASK			BIT(0)

#define DPU95_FRAMEGEN_MAX_FRAME_INDEX		0x3ffff
#define DPU95_FRAMEGEN_MAX_CLOCK		300000	/* in KHz */

#define DPU95_FETCHUNIT_CAP_USE_FETCHECO	BIT(0)
#define DPU95_FETCHUNIT_CAP_USE_SCALER		BIT(1)
#define DPU95_FETCHUNIT_CAP_PACKED_YUV422	BIT(2)

struct dpu95_fetchunit;

enum dpu95_irq {
	DPU95_IRQ_STORE9_SHDLOAD		= 0,
	DPU95_IRQ_STORE9_FRAMECOMPLETE		= 1,
	DPU95_IRQ_STORE9_SEQCOMPLETE		= 2,
	DPU95_IRQ_EXTDST0_SHDLOAD		= 3,
	DPU95_IRQ_EXTDST0_FRAMECOMPLETE		= 4,
	DPU95_IRQ_EXTDST0_SEQCOMPLETE		= 5,
	DPU95_IRQ_EXTDST4_SHDLOAD		= 6,
	DPU95_IRQ_EXTDST4_FRAMECOMPLETE		= 7,
	DPU95_IRQ_EXTDST4_SEQCOMPLETE		= 8,
	DPU95_IRQ_EXTDST1_SHDLOAD		= 9,
	DPU95_IRQ_EXTDST1_FRAMECOMPLETE		= 10,
	DPU95_IRQ_EXTDST1_SEQCOMPLETE		= 11,
	DPU95_IRQ_EXTDST5_SHDLOAD		= 12,
	DPU95_IRQ_EXTDST5_FRAMECOMPLETE		= 13,
	DPU95_IRQ_EXTDST5_SEQCOMPLETE		= 14,
	DPU95_IRQ_DOMAINBLEND0_SHDLOAD		= 15,
	DPU95_IRQ_DOMAINBLEND0_FRAMECOMPLETE	= 16,
	DPU95_IRQ_DOMAINBLEND0_SEQCOMPLETE	= 17,
	DPU95_IRQ_DISENGCFG_SHDLOAD0		= 18,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0	= 19,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE0	= 20,
	DPU95_IRQ_FRAMEGEN0_INT0		= 21,
	DPU95_IRQ_FRAMEGEN0_INT1		= 22,
	DPU95_IRQ_FRAMEGEN0_INT2		= 23,
	DPU95_IRQ_FRAMEGEN0_INT3		= 24,
	DPU95_IRQ_SIG0_SHDLOAD			= 25,
	DPU95_IRQ_SIG0_VALID			= 26,
	DPU95_IRQ_SIG0_ERROR			= 27,
	DPU95_IRQ_SIG0_CLUSTER_ERROR		= 28,
	DPU95_IRQ_SIG0_CLUSTER_MATCH		= 29,
	DPU95_IRQ_SIG2_SHDLOAD			= 30,
	DPU95_IRQ_SIG2_VALID			= 31,
	DPU95_IRQ_SIG2_ERROR			= 32,
	DPU95_IRQ_SIG2_CLUSTER_ERROR		= 33,
	DPU95_IRQ_SIG2_CLUSTER_MATCH		= 34,
	DPU95_IRQ_IDHASH0_SHDLOAD		= 35,
	DPU95_IRQ_IDHASH0_VALID			= 36,
	DPU95_IRQ_IDHASH0_WINDOWN_ERROR		= 37,
	DPU95_IRQ_DOMAINBLEND1_SHDLOAD		= 38,
	DPU95_IRQ_DOMAINBLEND1_FRAMECOMPLETE	= 39,
	DPU95_IRQ_DOMAINBLEND1_SEQCOMPLETE	= 40,
	DPU95_IRQ_DISENGCFG_SHDLOAD1		= 41,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1	= 42,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE1	= 43,
	DPU95_IRQ_FRAMEGEN1_INT0		= 44,
	DPU95_IRQ_FRAMEGEN1_INT1		= 45,
	DPU95_IRQ_FRAMEGEN1_INT2		= 46,
	DPU95_IRQ_FRAMEGEN1_INT3		= 47,
	DPU95_IRQ_SIG1_SHDLOAD			= 48,
	DPU95_IRQ_SIG1_VALID			= 49,
	DPU95_IRQ_SIG1_ERROR			= 50,
	DPU95_IRQ_SIG1_CLUSTER_ERROR		= 51,
	DPU95_IRQ_SIG1_CLUSTER_MATCH		= 52,
	DPU95_IRQ_CMDSEQ_ERROR			= 53,
	DPU95_IRQ_COMCTRL_SW0			= 54,
	DPU95_IRQ_COMCTRL_SW1			= 55,
	DPU95_IRQ_COMCTRL_SW2			= 56,
	DPU95_IRQ_COMCTRL_SW3			= 57,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_ON		= 58,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_OFF	= 59,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_ON	= 60,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_OFF	= 61,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_ON	= 62,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_OFF	= 63,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_RISE	= 64,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_FAIL	= 65,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_ON	= 66,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_OFF	= 67,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_ON	= 68,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_OFF	= 69,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_RISE	= 70,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_FAIL	= 71,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_ON		= 72,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_OFF	= 73,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_ON	= 74,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_OFF	= 75,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_ON	= 76,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_OFF	= 77,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_RISE	= 78,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_FAIL	= 79,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_ON	= 80,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_OFF	= 81,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_ON	= 82,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_OFF	= 83,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_RISE	= 84,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_FAIL	= 85,
	DPU95_IRQ_COUNT				= 86,
};

enum dpu95_unit_type {
	DPU95_DISP,
	DPU95_BLIT,
};

enum dpu95_link_id {
	DPU95_LINK_ID_NONE		= 0x00,
	DPU95_LINK_ID_STORE9		= 0x01,
	DPU95_LINK_ID_EXTDST0		= 0x02,
	DPU95_LINK_ID_EXTDST4		= 0x03,
	DPU95_LINK_ID_EXTDST1		= 0x04,
	DPU95_LINK_ID_EXTDST5		= 0x05,
	DPU95_LINK_ID_FETCHECO9		= 0x07,
	DPU95_LINK_ID_HSCALER9		= 0x08,
	DPU95_LINK_ID_FILTER9		= 0x0a,
	DPU95_LINK_ID_CONSTFRAME0	= 0x0c,
	DPU95_LINK_ID_CONSTFRAME4	= 0x0d,
	DPU95_LINK_ID_CONSTFRAME1	= 0x10,
	DPU95_LINK_ID_CONSTFRAME5	= 0x11,
	DPU95_LINK_ID_LAYERBLEND1	= 0x14,
	DPU95_LINK_ID_LAYERBLEND2	= 0x15,
	DPU95_LINK_ID_LAYERBLEND3	= 0x16,
	DPU95_LINK_ID_LAYERBLEND4	= 0x17,
	DPU95_LINK_ID_LAYERBLEND5	= 0x18,
	DPU95_LINK_ID_LAYERBLEND6	= 0x19,
	DPU95_LINK_ID_FETCHLAYER0	= 0x1a,
	DPU95_LINK_ID_FETCHLAYER1	= 0x1b,
	DPU95_LINK_ID_FETCHYUV3		= 0x1c,
	DPU95_LINK_ID_FETCHYUV0		= 0x1d,
	DPU95_LINK_ID_FETCHECO0		= 0x1e,
	DPU95_LINK_ID_FETCHYUV1		= 0x1f,
	DPU95_LINK_ID_FETCHECO1		= 0x20,
	DPU95_LINK_ID_FETCHYUV2		= 0x21,
	DPU95_LINK_ID_FETCHECO2		= 0x22,
	DPU95_LINK_ID_MATRIX4		= 0x23,
	DPU95_LINK_ID_HSCALER4		= 0x24,
};

enum dpu95_db_modecontrol {
	DB_MODE_PRIMARY,
	DB_MODE_SECONDARY,
	DB_MODE_BLEND,
	DB_MODE_SIDEBYSIDE,
};

enum dpu95_db_alphamaskmode {
	DB_ALHPHAMASKMODE_PRIM,
	DB_ALHPHAMASKMODE_SEC,
	DB_ALHPHAMASKMODE_PRIM_OR_SEC,
	DB_ALHPHAMASKMODE_PRIM_AND_SEC,
	DB_ALHPHAMASKMODE_PRIM_INV,
	DB_ALHPHAMASKMODE_SEC_INV,
	DB_ALHPHAMASKMODE_PRIM_OR_SEC_INV,
	DB_ALHPHAMASKMODE_PRIM_AND_SEC_INV,
};

enum dpu95_fy_dynamic_src_sel {
	FY_SRC_DISABLE		= DPU95_LINK_ID_NONE,
	FY_SRC_FETCHECO0	= DPU95_LINK_ID_FETCHECO0,
	FY_SRC_FETCHECO1	= DPU95_LINK_ID_FETCHECO1,
	FY_SRC_FETCHECO2	= DPU95_LINK_ID_FETCHECO2,
	FY_SRC_FETCHECO9	= DPU95_LINK_ID_FETCHECO9,
};

enum dpu95_fy_dynamic_sec_sel {
	FY_SEC_DISABLE		= DPU95_LINK_ID_NONE,
	FY_SEC_FETCHYUV0	= DPU95_LINK_ID_FETCHYUV0,
	FY_SEC_FETCHYUV1	= DPU95_LINK_ID_FETCHYUV1,
	FY_SEC_FETCHYUV2	= DPU95_LINK_ID_FETCHYUV2,
	FY_SEC_FETCHYUV3	= DPU95_LINK_ID_FETCHYUV3,
};

enum dpu95_fg_syncmode {
	/* No side-by-side synchronization. */
	FG_SYNCMODE_OFF = 0x0,
	/* Framegen is master. */
	FG_SYNCMODE_MASTER = 0x1,
	/* Runs in cyclic synchronization mode. */
	FG_SYNCMODE_SLAVE_CYC = 0x2,
	/* Runs in one time synchronization mode. */
	FG_SYNCMODE_SLAVE_ONCE = 0x3,
	/* Runs in cyclic synchronization mode. Size is adpated. */
	FG_SYNCMODE_ADAPT_CYC = 0x6,
	/* Runs in one time synchronization mode. Size is adpated. */
	FG_SYNCMODE_ADAPT_ONCE = 0x7,
};

enum dpu95_fg_shdtokgensyncmode {
	/* Shadow token is generated by local FgSlr.ShdTokGen field. */
	FG_SHDTOKGENSYNCMODE_LOCAL,
	/* Shadow token is generated by FgSlr.ShdTokGen field of second Framegen. */
	FG_SHDTOKGENSYNCMODE_EXTERNAL,
};

enum dpu95_fg_dm {
	FG_DM_BLACK,
	/* Constant Color Background is shown. */
	FG_DM_CONSTCOL,
	FG_DM_PRIM,
	FG_DM_SEC,
	FG_DM_PRIM_ON_TOP,
	FG_DM_SEC_ON_TOP,
	/* White color background with test pattern is shown. */
	FG_DM_TEST,
};

enum dpu95_lb_mode {
	LB_NEUTRAL,	/* Output is same as primary input. */
	LB_BLEND,
};

enum dpu95_scaler_field_mode {
	/* Constant 0 indicates frame or top field. */
	SCALER_ALWAYS0,
	/* Constant 1 indicates bottom field. */
	SCALER_ALWAYS1,
	/* Output field polarity is taken from input field polarity. */
	SCALER_INPUT,
	/* Output field polarity toggles, starting with 0 after reset. */
	SCALER_TOGGLE,
};

enum dpu95_scaler_filter_mode {
	/* pointer-sampling */
	SCALER_NEAREST,
	/* box filter */
	SCALER_LINEAR,
};

enum dpu95_scaler_scale_mode {
	SCALER_DOWNSCALE,
	SCALER_UPSCALE,
};

enum dpu95_scaler_mode {
	/* Pixel by-pass the scaler, all other settings are ignored. */
	SCALER_NEUTRAL,
	/* Scaler is active. */
	SCALER_ACTIVE,
};

enum dpu95_pec_clken {
	CLKEN_DISABLE = 0x0,
	CLKEN_AUTOMATIC = 0x1,
	CLKEN_FULL = 0x3,
};

static enum dpu95_irq dpu_comctrl_irq[] = {
	DPU95_IRQ_COMCTRL_SW0,
	DPU95_IRQ_COMCTRL_SW1,
	DPU95_IRQ_COMCTRL_SW2,
	DPU95_IRQ_COMCTRL_SW3,
};
#define DPU95_COMCTRL_IRQ_IRQS		ARRAY_SIZE(dpu_comctrl_irq)

static enum dpu95_irq dpu_display_irq0[] = {
	DPU95_IRQ_EXTDST0_SHDLOAD,
	DPU95_IRQ_EXTDST0_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST0_SEQCOMPLETE,
	DPU95_IRQ_EXTDST4_SHDLOAD,
	DPU95_IRQ_EXTDST4_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST4_SEQCOMPLETE,
	DPU95_IRQ_DOMAINBLEND0_SHDLOAD,
	DPU95_IRQ_DOMAINBLEND0_FRAMECOMPLETE,
	DPU95_IRQ_DOMAINBLEND0_SEQCOMPLETE,
	DPU95_IRQ_DISENGCFG_SHDLOAD0,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE0,
	DPU95_IRQ_FRAMEGEN0_INT0,
	DPU95_IRQ_FRAMEGEN0_INT1,
	DPU95_IRQ_FRAMEGEN0_INT2,
	DPU95_IRQ_FRAMEGEN0_INT3,
	DPU95_IRQ_SIG0_SHDLOAD,
	DPU95_IRQ_SIG0_VALID,
	DPU95_IRQ_SIG0_ERROR,
	DPU95_IRQ_SIG0_CLUSTER_ERROR,
	DPU95_IRQ_SIG0_CLUSTER_MATCH,
	DPU95_IRQ_SIG2_SHDLOAD,
	DPU95_IRQ_SIG2_VALID,
	DPU95_IRQ_SIG2_ERROR,
	DPU95_IRQ_SIG2_CLUSTER_ERROR,
	DPU95_IRQ_SIG2_CLUSTER_MATCH,
	DPU95_IRQ_IDHASH0_SHDLOAD,
	DPU95_IRQ_IDHASH0_VALID,
	DPU95_IRQ_IDHASH0_WINDOWN_ERROR,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_ON,
	DPU95_IRQ_FRAMEGEN0_PRIMSYNC_OFF,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_ON,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW0_OFF,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_ON,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN0_OFF,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_RISE,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD0_FAIL,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_ON,
	DPU95_IRQ_FRAMEGEN0_OVERFLOW1_OFF,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_ON,
	DPU95_IRQ_FRAMEGEN0_UNDERRUN1_OFF,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_RISE,
	DPU95_IRQ_FRAMEGEN0_THRESHOLD1_FAIL,
};
#define DPU95_DISPLAY_IRQ0_IRQS		ARRAY_SIZE(dpu_display_irq0)

static enum dpu95_irq dpu_display_irq2[] = {
	DPU95_IRQ_EXTDST1_SHDLOAD,
	DPU95_IRQ_EXTDST1_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST1_SEQCOMPLETE,
	DPU95_IRQ_EXTDST5_SHDLOAD,
	DPU95_IRQ_EXTDST5_FRAMECOMPLETE,
	DPU95_IRQ_EXTDST5_SEQCOMPLETE,
	DPU95_IRQ_DOMAINBLEND1_SHDLOAD,
	DPU95_IRQ_DOMAINBLEND1_FRAMECOMPLETE,
	DPU95_IRQ_DOMAINBLEND1_SEQCOMPLETE,
	DPU95_IRQ_DISENGCFG_SHDLOAD1,
	DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1,
	DPU95_IRQ_DISENGCFG_SEQCOMPLETE1,
	DPU95_IRQ_FRAMEGEN1_INT0,
	DPU95_IRQ_FRAMEGEN1_INT1,
	DPU95_IRQ_FRAMEGEN1_INT2,
	DPU95_IRQ_FRAMEGEN1_INT3,
	DPU95_IRQ_SIG1_SHDLOAD,
	DPU95_IRQ_SIG1_VALID,
	DPU95_IRQ_SIG1_ERROR,
	DPU95_IRQ_SIG1_CLUSTER_ERROR,
	DPU95_IRQ_SIG1_CLUSTER_MATCH,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_ON,
	DPU95_IRQ_FRAMEGEN1_PRIMSYNC_OFF,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_ON,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW0_OFF,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_ON,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN0_OFF,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_RISE,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD0_FAIL,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_ON,
	DPU95_IRQ_FRAMEGEN1_OVERFLOW1_OFF,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_ON,
	DPU95_IRQ_FRAMEGEN1_UNDERRUN1_OFF,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_RISE,
	DPU95_IRQ_FRAMEGEN1_THRESHOLD1_FAIL,
};
#define DPU95_DISPLAY_IRQ2_IRQS		ARRAY_SIZE(dpu_display_irq2)

struct dpu95_soc {
	struct device			*dev;

	void __iomem			*comctrl_irq_reg;
	void __iomem			*dm_mask_reg;
	void __iomem			*disp_irq0_reg;
	void __iomem			*disp_irq2_reg;

	struct regmap			*regmap;

	struct clk			*clk_axi;
	struct clk			*clk_apb;
	struct clk			*clk_pix;
	struct clk			*clk_ocram;
	struct clk			*clk_ldb;
	struct clk			*clk_ldb_vco;

	int				comctrl_irq[DPU95_COMCTRL_IRQ_IRQS];
	int				disp_irq0[DPU95_DISPLAY_IRQ0_IRQS];
	int				disp_irq2[DPU95_DISPLAY_IRQ2_IRQS];

	struct irq_domain		*comctrl_irq_domain;
	struct irq_domain		*disp_irq0_domain;
	struct irq_domain		*disp_irq2_domain;

	struct dpu95_constframe		*cf[4];
	struct dpu95_domainblend	*db[2];
	struct dpu95_dither		*dt[2];
	struct dpu95_extdst		*ed[4];
	struct dpu95_fetchunit		*fe[4];
	struct dpu95_framegen		*fg[2];
	struct dpu95_fetchunit		*fl[2];
	struct dpu95_fetchunit		*fy[4];
	struct dpu95_hscaler		*hs[2];
	struct dpu95_layerblend		*lb[6];
};

struct dpu95_units {
	const unsigned int *ids;
	const enum dpu95_unit_type *types;
	const unsigned long *ofss;
	const unsigned long *aux_ofss;	/* PixEngCFG or PixEngPath */
	const unsigned int cnt;
	const char *name;

	/* software initialization */
	int (*init)(struct dpu95_soc *dpu, unsigned int index,
		    unsigned int id, enum dpu95_unit_type type,
		    unsigned long aux_base, unsigned long base);

	/* hardware initialization */
	void (*hw_init)(struct dpu95_soc *dpu, unsigned int index);
};

int dpu95_map_comctrl_irq(struct dpu95_soc *dpu, int irq);
int dpu95_map_disp_irq0(struct dpu95_soc *dpu, int irq);
int dpu95_map_disp_irq2(struct dpu95_soc *dpu, int irq);

void dpu95_irq_hw_init(struct dpu95_soc *dpu);

void dpu95_submodules_hw_init(struct dpu95_soc *dpu);

int dpu95_set_qos(struct dpu95_soc *dpu);

void dpu95_enable_display_pipeline_sync(struct dpu95_soc *dpu);
void dpu95_disable_display_pipeline_sync(struct dpu95_soc *dpu);

/* Constant Frame Unit */
struct dpu95_constframe;
enum dpu95_link_id dpu95_cf_get_link_id(struct dpu95_constframe *cf);
void dpu95_cf_framedimensions(struct dpu95_constframe *cf, unsigned int w,
			      unsigned int h);
void dpu95_cf_constantcolor_black(struct dpu95_constframe *cf);
struct dpu95_constframe *dpu95_cf_safe_get(struct dpu95_soc *dpu,
					   unsigned int stream_id);
struct dpu95_constframe *dpu95_cf_cont_get(struct dpu95_soc *dpu,
					   unsigned int stream_id);
void dpu95_cf_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_cf_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

/* Domain Blend Unit */
struct dpu95_domainblend;
void dpu95_db_shdtokgen(struct dpu95_domainblend *db);
void dpu95_db_modecontrol(struct dpu95_domainblend *db,
			  enum dpu95_db_modecontrol m);
void dpu95_db_alphamaskmode_enable(struct dpu95_domainblend *db);
void dpu95_db_alphamaskmode_disable(struct dpu95_domainblend *db);
struct dpu95_domainblend *dpu95_db_get(struct dpu95_soc *dpu95, int id);
void dpu95_db_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_db_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long unused, unsigned long base);

/* Dither Unit */
struct dpu95_dither;
void dpu95_dt_polhs_active_high(struct dpu95_dither *dt);
void dpu95_dt_polhs_active_low(struct dpu95_dither *dt);
void dpu95_dt_polvs_active_high(struct dpu95_dither *dt);
void dpu95_dt_polvs_active_low(struct dpu95_dither *dt);
void dpu95_dt_polen_active_high(struct dpu95_dither *dt);
void dpu95_dt_polen_active_low(struct dpu95_dither *dt);
struct dpu95_dither *dpu95_dt_get(struct dpu95_soc *dpu95, int id);
void dpu95_dt_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_dt_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long aux_base, unsigned long base);

/* External Destination Unit */
struct dpu95_extdst;
void dpu95_ed_pec_poweron(struct dpu95_extdst *ed);
void dpu95_ed_pec_src_sel(struct dpu95_extdst *ed, enum dpu95_link_id src);
void dpu95_ed_pec_sync_trigger(struct dpu95_extdst *ed);
struct dpu95_extdst *dpu95_ed_safe_get(struct dpu95_soc *dpu,
				       unsigned int stream_id);
struct dpu95_extdst *dpu95_ed_cont_get(struct dpu95_soc *dpu,
				       unsigned int stream_id);
void dpu95_ed_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_ed_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

/* Fetch ECO Unit */
struct dpu95_fetchunit *dpu95_fe_get(struct dpu95_soc *dpu, unsigned int id);
void dpu95_fe_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_fe_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

/* Frame Generator Unit */
struct dpu95_framegen;
void dpu95_fg_syncmode(struct dpu95_framegen *fg, enum dpu95_fg_syncmode mode);
void dpu95_fg_shdtokgen_syncmode(struct dpu95_framegen *fg,
				 enum dpu95_fg_shdtokgensyncmode mode);
void dpu95_fg_cfg_videomode(struct dpu95_framegen *fg,
			    struct drm_display_mode *m,
			    bool enc_is_dsi);
void dpu95_fg_displaymode(struct dpu95_framegen *fg, enum dpu95_fg_dm mode);
void dpu95_fg_panic_displaymode(struct dpu95_framegen *fg,
				enum dpu95_fg_dm mode);
void dpu95_fg_enable(struct dpu95_framegen *fg);
void dpu95_fg_disable(struct dpu95_framegen *fg);
void dpu95_fg_shdtokgen(struct dpu95_framegen *fg);
u32 dpu95_fg_get_frame_index(struct dpu95_framegen *fg);
int dpu95_fg_get_line_index(struct dpu95_framegen *fg);
bool dpu95_fg_primary_requests_to_read_empty_fifo(struct dpu95_framegen *fg);
void dpu95_fg_primary_clear_channel_status(struct dpu95_framegen *fg);
bool dpu95_fg_secondary_requests_to_read_empty_fifo(struct dpu95_framegen *fg);
void dpu95_fg_secondary_clear_channel_status(struct dpu95_framegen *fg);
int dpu95_fg_wait_for_primary_syncup(struct dpu95_framegen *fg);
int dpu95_fg_wait_for_secondary_syncup(struct dpu95_framegen *fg);
void dpu95_fg_enable_clock(struct dpu95_framegen *fg, bool enc_is_dsi);
void dpu95_fg_disable_clock(struct dpu95_framegen *fg, bool enc_is_dsi);
struct dpu95_framegen *dpu95_fg_get(struct dpu95_soc *dpu, unsigned int id);
void dpu95_fg_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_fg_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long unused, unsigned long base);

/* Fetch Layer Unit */
struct dpu95_fetchunit *dpu95_fl_get(struct dpu95_soc *dpu, unsigned int id);
void dpu95_fl_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_fl_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

/* Fetch YUV Unit */
struct dpu95_fetchunit *dpu95_fy_get(struct dpu95_soc *dpu, unsigned int id);
void dpu95_fy_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_fy_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

/* Horizontal Scaler Unit */
struct dpu95_hscaler;

struct dpu95_hscaler_ops {
	bool (*is_enabled)(struct dpu95_hscaler *hs);
	void (*set_stream_id)(struct dpu95_hscaler *hs, unsigned int stream_id);
	unsigned int (*get_stream_id)(struct dpu95_hscaler *hs);
	void (*set_no_stream_id)(struct dpu95_hscaler *hs);
	bool (*has_stream_id)(struct dpu95_hscaler *hs);
};

const struct dpu95_hscaler_ops *dpu95_hs_get_ops(struct dpu95_hscaler *hs);

enum dpu95_link_id dpu95_hs_get_link_id(struct dpu95_hscaler *hs);
void dpu95_hs_pec_dynamic_src_sel(struct dpu95_hscaler *hs,
				  enum dpu95_link_id src);
void dpu95_hs_pec_clken(struct dpu95_hscaler *hs, enum dpu95_pec_clken clken);
void dpu95_hs_setup1(struct dpu95_hscaler *hs,
		     unsigned int src_w, unsigned int dst_w);
void dpu95_hs_setup2(struct dpu95_hscaler *hs, u32 phase_offset);
void dpu95_hs_output_size(struct dpu95_hscaler *hs, u32 line_num);
void dpu95_hs_filter_mode(struct dpu95_hscaler *hs,
			  enum dpu95_scaler_filter_mode m);
void dpu95_hs_scale_mode(struct dpu95_hscaler *hs,
			 enum dpu95_scaler_scale_mode m);
void dpu95_hs_mode(struct dpu95_hscaler *hs, enum dpu95_scaler_mode m);
unsigned int dpu95_hs_get_id(struct dpu95_hscaler *hs);
struct dpu95_hscaler *dpu95_hs_get(struct dpu95_soc *dpu, unsigned int id);
void dpu95_hs_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_hs_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

/* Layer Blend Unit */
struct dpu95_layerblend;
enum dpu95_link_id dpu95_lb_get_link_id(struct dpu95_layerblend *lb);
void dpu95_lb_pec_dynamic_prim_sel(struct dpu95_layerblend *lb,
				   enum dpu95_link_id prim);
void dpu95_lb_pec_dynamic_sec_sel(struct dpu95_layerblend *lb,
				  enum dpu95_link_id sec);
void dpu95_lb_pec_clken(struct dpu95_layerblend *lb, enum dpu95_pec_clken clken);
void dpu95_lb_mode(struct dpu95_layerblend *lb, enum dpu95_lb_mode mode);
void dpu95_lb_blendcontrol(struct dpu95_layerblend *lb, unsigned int zpos,
			   unsigned int pixel_blend_mode, u16 alpha);
void dpu95_lb_position(struct dpu95_layerblend *lb, int x, int y);
unsigned int dpu95_lb_get_id(struct dpu95_layerblend *lb);
struct dpu95_layerblend *dpu95_lb_get(struct dpu95_soc *dpu, unsigned int id);
void dpu95_lb_hw_init(struct dpu95_soc *dpu, unsigned int index);
int dpu95_lb_init(struct dpu95_soc *dpu, unsigned int index,
		  unsigned int id, enum dpu95_unit_type type,
		  unsigned long pec_base, unsigned long base);

struct dpu95_fetchunit_ops {
	void (*set_pec_dynamic_src_sel)(struct dpu95_fetchunit *fu,
					enum dpu95_link_id src);

	bool (*is_enabled)(struct dpu95_fetchunit *fu);

	void (*set_stream_id)(struct dpu95_fetchunit *fu, unsigned int stream_id);

	unsigned int (*get_stream_id)(struct dpu95_fetchunit *fu);

	void (*set_no_stream_id)(struct dpu95_fetchunit *fu);

	bool (*has_stream_id)(struct dpu95_fetchunit *fu);

	void (*set_numbuffers)(struct dpu95_fetchunit *fu, unsigned int num);

	void (*set_burstlength)(struct dpu95_fetchunit *fu,
				unsigned int burst_length);

	void (*set_baseaddress)(struct dpu95_fetchunit *fu, dma_addr_t baddr);

	void (*set_src_stride)(struct dpu95_fetchunit *fu, unsigned int stride);

	void (*set_src_buf_dimensions)(struct dpu95_fetchunit *fu,
				       unsigned int w, unsigned int h,
				       const struct drm_format_info *format,
				       bool deinterlace);

	void (*set_fmt)(struct dpu95_fetchunit *fu,
			const struct drm_format_info *format,
			enum drm_color_encoding color_encoding,
			enum drm_color_range color_range,
			bool deinterlace);

	void (*set_pixel_blend_mode)(struct dpu95_fetchunit *fu,
				     unsigned int pixel_blend_mode, u16 alpha,
				     bool fb_format_has_alpha);

	void (*enable_src_buf)(struct dpu95_fetchunit *fu);
	void (*disable_src_buf)(struct dpu95_fetchunit *fu);

	void (*set_framedimensions)(struct dpu95_fetchunit *fu,
				    unsigned int w, unsigned int h,
				    bool deinterlace);

	struct dpu95_fetchunit *(*get_fetcheco)(struct dpu95_fetchunit *fu);
	struct dpu95_hscaler *(*get_hscaler)(struct dpu95_fetchunit *fu);

	void (*set_layerblend)(struct dpu95_fetchunit *fu,
			       struct dpu95_layerblend *lb);

	bool (*is_available)(struct dpu95_fetchunit *fu);
	void (*set_available)(struct dpu95_fetchunit *fu);
	void (*set_inavailable)(struct dpu95_fetchunit *fu);

	enum dpu95_link_id (*get_link_id)(struct dpu95_fetchunit *fu);

	u32 (*get_cap_mask)(struct dpu95_fetchunit *fu);

	const char *(*get_name)(struct dpu95_fetchunit *fu);
};

const struct dpu95_fetchunit_ops *dpu95_fu_get_ops(struct dpu95_fetchunit *fu);
struct dpu95_fetchunit *dpu95_fu_get_from_list(struct list_head *l);
void dpu95_fu_add_to_list(struct dpu95_fetchunit *fu, struct list_head *l);

struct dpu95_plane_res {
	struct dpu95_fetchunit	*fl[2];
	struct dpu95_fetchunit	*fy[4];
	struct dpu95_layerblend	*lb[6];
};

/*
 * fetchunit and layerblend resources of the plane group are
 * shared by the two CRTCs in DPU
 */
struct dpu95_plane_grp {
	struct dpu95_plane_res	res;
	struct list_head	fu_list;
	struct dpu95_constframe	*cf[2];
	struct dpu95_extdst	*ed[2];
	struct dpu95_hscaler	*hs;
	bool			hs_used;
};

#endif /* __DRM_DPU95_H__ */
