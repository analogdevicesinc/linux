/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
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
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <soc/imx8/sc/sci.h>
#include <video/dpu.h>
#include <video/imx8-pc.h>
#include <video/imx8-prefetch.h>
#include "dpu-prv.h"

static bool display_plane_video_proc = true;
module_param(display_plane_video_proc, bool, 0444);
MODULE_PARM_DESC(display_plane_video_proc,
		 "Enable video processing for display [default=true]");

#define DPU_CM_REG_DEFINE1(name1, name2)		\
static inline u32 name1(const struct cm_reg_ofs *ofs)	\
{							\
	return ofs->name2;				\
}

#define DPU_CM_REG_DEFINE2(name1, name2)		\
static inline u32 name1(const struct cm_reg_ofs *ofs,	\
			unsigned int n)			\
{							\
	return ofs->name2 + (4 * n);			\
}

DPU_CM_REG_DEFINE1(IPIDENTIFIER, ipidentifier);

#define DESIGNDELIVERYID_MASK		0xF0U
#define DESIGNDELIVERYID_SHIFT		4U

#define DESIGNMATURITYLEVEL_MASK	0xF00U
#define DESIGNMATURITYLEVEL_SHIFT	8U
enum design_maturity_level {
	/* Pre feasibility study. */
	DESIGNMATURITYLEVEL__PREFS	= 1 << DESIGNMATURITYLEVEL_SHIFT,
	/* Feasibility study. */
	DESIGNMATURITYLEVEL__FS		= 2 << DESIGNMATURITYLEVEL_SHIFT,
	/* Functionality complete. */
	DESIGNMATURITYLEVEL__R0		= 3 << DESIGNMATURITYLEVEL_SHIFT,
	/* Verification complete. */
	DESIGNMATURITYLEVEL__R1		= 4 << DESIGNMATURITYLEVEL_SHIFT,
};

#define IPEVOLUTION_MASK		0xF000U
#define IPEVOLUTION_SHIFT		12U

#define IPFEATURESET_MASK		0xF0000U
#define IPFEATURESET_SHIFT		16U
enum ip_feature_set {
	/* Minimal functionality (Eco). */
	IPFEATURESET__E = 1 << IPFEATURESET_SHIFT,
	/* Reduced functionality (Light). */
	IPFEATURESET__L = 2 << IPFEATURESET_SHIFT,
	/* Advanced functionality (Plus). */
	IPFEATURESET__P = 4 << IPFEATURESET_SHIFT,
	/* Extensive functionality (eXtensive). */
	IPFEATURESET__X = 5 << IPFEATURESET_SHIFT,
};

#define IPAPPLICATION_MASK		0xF00000U
#define IPAPPLICATION_SHIFT		20U
enum ip_application {
	/* Blit Engine only. */
	IPAPPLICATION__B = 1 << IPAPPLICATION_SHIFT,
	/* Blit Engine and Display Controller. */
	IPAPPLICATION__D = 2 << IPAPPLICATION_SHIFT,
	/* Display Controller only (with direct capture). */
	IPAPPLICATION__V = 3 << IPAPPLICATION_SHIFT,
	/*
	 * Blit Engine, Display Controller (with direct capture),
	 * Capture Controller (buffered capture) and Drawing Engine.
	 */
	IPAPPLICATION__G = 4 << IPAPPLICATION_SHIFT,
	/* Display Controller only. */
	IPAPPLICATION__C = 5 << IPAPPLICATION_SHIFT,
};

#define IPCONFIGURATION_MASK		0xF000000U
#define IPCONFIGURATION_SHIFT		24U
enum ip_configuration {
	/* Graphics core only (Module). */
	IPCONFIGURATION__M = 1 << IPCONFIGURATION_SHIFT,
	/* Subsystem including a graphics core (System). */
	IPCONFIGURATION__S = 2 << IPCONFIGURATION_SHIFT,
};

#define IPFAMILY_MASK			0xF0000000U
#define IPFAMILY_SHIFT			28U
enum ip_family {
	/* IMXDPU building block generation 2010. */
	IPFAMILY__IMXDPU2010 = 0,
	/* IMXDPU building block generation 2012. */
	IPFAMILY__IMXDPU2012 = 1 << IPFAMILY_SHIFT,
	/* IMXDPU building block generation 2013. */
	IPFAMILY__IMXDPU2013 = 2 << IPFAMILY_SHIFT,
};

DPU_CM_REG_DEFINE1(LOCKUNLOCK, lockunlock);
DPU_CM_REG_DEFINE1(LOCKSTATUS, lockstatus);
DPU_CM_REG_DEFINE2(USERINTERRUPTMASK, userinterruptmask);
DPU_CM_REG_DEFINE2(INTERRUPTENABLE, interruptenable);
DPU_CM_REG_DEFINE2(INTERRUPTPRESET, interruptpreset);
DPU_CM_REG_DEFINE2(INTERRUPTCLEAR, interruptclear);
DPU_CM_REG_DEFINE2(INTERRUPTSTATUS, interruptstatus);
DPU_CM_REG_DEFINE2(USERINTERRUPTENABLE, userinterruptenable);
DPU_CM_REG_DEFINE2(USERINTERRUPTPRESET, userinterruptpreset);
DPU_CM_REG_DEFINE2(USERINTERRUPTCLEAR, userinterruptclear);
DPU_CM_REG_DEFINE2(USERINTERRUPTSTATUS, userinterruptstatus);
DPU_CM_REG_DEFINE1(GENERALPURPOSE, generalpurpose);

static inline u32 dpu_cm_read(struct dpu_soc *dpu, unsigned int offset)
{
	return readl(dpu->cm_reg + offset);
}

static inline void dpu_cm_write(struct dpu_soc *dpu, u32 value,
				unsigned int offset)
{
	writel(value, dpu->cm_reg + offset);
}

/* Constant Frame Unit */
static const unsigned long cf_ofss[] = {0x4400, 0x5400, 0x4c00, 0x5c00};
static const unsigned long cf_pec_ofss_v1[] = {0x980, 0xa00, 0x9c0, 0xa40};
static const unsigned long cf_pec_ofss_v2[] = {0x960, 0x9e0, 0x9a0, 0xa20};

/* Display Engine Configuration Unit */
static const unsigned long dec_ofss_v1[] = {0x10000, 0x10020};
static const unsigned long dec_ofss_v2[] = {0xb400, 0xb420};

/* External Destination Unit */
static const unsigned long ed_ofss[] = {0x4800, 0x5800, 0x5000, 0x6000};
static const unsigned long ed_pec_ofss_v1[] = {0x9a0, 0xa20, 0x9e0, 0xa60};
static const unsigned long ed_pec_ofss_v2[] = {0x980, 0xa00, 0x9c0, 0xa40};

/* Fetch Decode Unit */
static const unsigned long fd_ofss_v1[] = {0x8c00, 0x9800, 0x7400, 0x7c00};
static const unsigned long fd_ofss_v2[] = {0x6c00, 0x7800};
static const unsigned long fd_pec_ofss_v1[] = {0xb60, 0xb80, 0xb00, 0xb20};
static const unsigned long fd_pec_ofss_v2[] = {0xa80, 0xaa0};

/* Fetch ECO Unit */
static const unsigned long fe_ofss_v1[] = {0x9400, 0xa000, 0x8800, 0x1c00};
static const unsigned long fe_ofss_v2[] = {0x7400, 0x8000, 0x6800, 0x1c00};
static const unsigned long fe_pec_ofss_v1[] = {0xb70, 0xb90, 0xb50, 0x870};
static const unsigned long fe_pec_ofss_v2[] = {0xa90, 0xab0, 0xa70, 0x850};

/* Frame Generator Unit */
static const unsigned long fg_ofss_v1[] = {0x10c00, 0x12800};
static const unsigned long fg_ofss_v2[] = {0xb800, 0xd400};

/* Fetch Layer Unit */
static const unsigned long fl_ofss_v1[] = {0xa400, 0xac00};
static const unsigned long fl_ofss_v2[] = {0x8400};
static const unsigned long fl_pec_ofss_v1[] = {0xba0, 0xbb0};
static const unsigned long fl_pec_ofss_v2[] = {0xac0};

/* Fetch Warp Unit */
static const unsigned long fw_ofss_v1[] = {0x8400};
static const unsigned long fw_ofss_v2[] = {0x6400};
static const unsigned long fw_pec_ofss_v1[] = {0xb40};
static const unsigned long fw_pec_ofss_v2[] = {0xa60};

/* Horizontal Scaler Unit */
static const unsigned long hs_ofss_v1[] = {0xbc00, 0xd000, 0x3000};
static const unsigned long hs_ofss_v2[] = {0x9000, 0x9c00, 0x3000};
static const unsigned long hs_pec_ofss_v1[] = {0xc00, 0xca0, 0x8e0};
static const unsigned long hs_pec_ofss_v2[] = {0xb00, 0xb60, 0x8c0};

/* Layer Blend Unit */
static const unsigned long lb_ofss_v1[] = {0xdc00, 0xe000, 0xe400, 0xe800,
					   0xec00, 0xf000, 0xf400};
static const unsigned long lb_ofss_v2[] = {0xa400, 0xa800, 0xac00, 0xb000};
static const unsigned long lb_pec_ofss_v1[] = {0xd00, 0xd20, 0xd40, 0xd60,
					       0xd80, 0xda0, 0xdc0};
static const unsigned long lb_pec_ofss_v2[] = {0xba0, 0xbc0, 0xbe0, 0xc00};

/* Store Unit */
static const unsigned long st_ofss_v1[] = {0x4000};
static const unsigned long st_ofss_v2[] = {0x4000};
static const unsigned long st_pec_ofss_v1[] = {0x960};
static const unsigned long st_pec_ofss_v2[] = {0x940};

/* Timing Controller Unit */
static const unsigned long tcon_ofss_v1[] = {0x12000, 0x13c00};
static const unsigned long tcon_ofss_v2[] = {0xcc00, 0xe800};

/* Vertical Scaler Unit */
static const unsigned long vs_ofss_v1[] = {0xc000, 0xd400, 0x3400};
static const unsigned long vs_ofss_v2[] = {0x9400, 0xa000, 0x3400};
static const unsigned long vs_pec_ofss_v1[] = {0xc20, 0xcc0, 0x900};
static const unsigned long vs_pec_ofss_v2[] = {0xb20, 0xb80, 0x8e0};

static const struct dpu_unit cfs_v1 = {
	.name = "ConstFrame",
	.num = ARRAY_SIZE(cf_ids),
	.ids = cf_ids,
	.pec_ofss = cf_pec_ofss_v1,
	.ofss = cf_ofss,
};

static const struct dpu_unit cfs_v2 = {
	.name = "ConstFrame",
	.num = ARRAY_SIZE(cf_ids),
	.ids = cf_ids,
	.pec_ofss = cf_pec_ofss_v2,
	.ofss = cf_ofss,
};

static const struct dpu_unit decs_v1 = {
	.name = "DisEngCfg",
	.num = ARRAY_SIZE(dec_ids),
	.ids = dec_ids,
	.pec_ofss = NULL,
	.ofss = dec_ofss_v1,
};

static const struct dpu_unit decs_v2 = {
	.name = "DisEngCfg",
	.num = ARRAY_SIZE(dec_ids),
	.ids = dec_ids,
	.pec_ofss = NULL,
	.ofss = dec_ofss_v2,
};

static const struct dpu_unit eds_v1 = {
	.name = "ExtDst",
	.num = ARRAY_SIZE(ed_ids),
	.ids = ed_ids,
	.pec_ofss = ed_pec_ofss_v1,
	.ofss = ed_ofss,
};

static const struct dpu_unit eds_v2 = {
	.name = "ExtDst",
	.num = ARRAY_SIZE(ed_ids),
	.ids = ed_ids,
	.pec_ofss = ed_pec_ofss_v2,
	.ofss = ed_ofss,
};

static const struct dpu_unit fds_v1 = {
	.name = "FetchDecode",
	.num = ARRAY_SIZE(fd_ids),
	.ids = fd_ids,
	.pec_ofss = fd_pec_ofss_v1,
	.ofss = fd_ofss_v1,
};

static const struct dpu_unit fds_v2 = {
	.name = "FetchDecode",
	.num = 2,
	.ids = fd_ids,
	.pec_ofss = fd_pec_ofss_v2,
	.ofss = fd_ofss_v2,
	.dprc_ids = fd_dprc_ids,
};

static const struct dpu_unit fes_v1 = {
	.name = "FetchECO",
	.num = ARRAY_SIZE(fe_ids),
	.ids = fe_ids,
	.pec_ofss = fe_pec_ofss_v1,
	.ofss = fe_ofss_v1,
};

static const struct dpu_unit fes_v2 = {
	.name = "FetchECO",
	.num = ARRAY_SIZE(fe_ids),
	.ids = fe_ids,
	.pec_ofss = fe_pec_ofss_v2,
	.ofss = fe_ofss_v2,
};

static const struct dpu_unit fgs_v1 = {
	.name = "FrameGen",
	.num = ARRAY_SIZE(fg_ids),
	.ids = fg_ids,
	.pec_ofss = NULL,
	.ofss = fg_ofss_v1,
};

static const struct dpu_unit fgs_v2 = {
	.name = "FrameGen",
	.num = ARRAY_SIZE(fg_ids),
	.ids = fg_ids,
	.pec_ofss = NULL,
	.ofss = fg_ofss_v2,
};

static const struct dpu_unit fls_v1 = {
	.name = "FetchLayer",
	.num = ARRAY_SIZE(fl_ids),
	.ids = fl_ids,
	.pec_ofss = fl_pec_ofss_v1,
	.ofss = fl_ofss_v1,
};

static const struct dpu_unit fls_v2 = {
	.name = "FetchLayer",
	.num = 1,
	.ids = fl_ids,
	.pec_ofss = fl_pec_ofss_v2,
	.ofss = fl_ofss_v2,
	.dprc_ids = fl_dprc_ids,
};

static const struct dpu_unit fws_v1 = {
	.name = "FetchWarp",
	.num = ARRAY_SIZE(fw_ids),
	.ids = fw_ids,
	.pec_ofss = fw_pec_ofss_v1,
	.ofss = fw_ofss_v1,
};

static const struct dpu_unit fws_v2 = {
	.name = "FetchWarp",
	.num = ARRAY_SIZE(fw_ids),
	.ids = fw_ids,
	.pec_ofss = fw_pec_ofss_v2,
	.ofss = fw_ofss_v2,
	.dprc_ids = fw_dprc_ids,
};

static const struct dpu_unit hss_v1 = {
	.name = "HScaler",
	.num = ARRAY_SIZE(hs_ids),
	.ids = hs_ids,
	.pec_ofss = hs_pec_ofss_v1,
	.ofss = hs_ofss_v1,
};

static const struct dpu_unit hss_v2 = {
	.name = "HScaler",
	.num = ARRAY_SIZE(hs_ids),
	.ids = hs_ids,
	.pec_ofss = hs_pec_ofss_v2,
	.ofss = hs_ofss_v2,
};

static const struct dpu_unit lbs_v1 = {
	.name = "LayerBlend",
	.num = ARRAY_SIZE(lb_ids),
	.ids = lb_ids,
	.pec_ofss = lb_pec_ofss_v1,
	.ofss = lb_ofss_v1,
};

static const struct dpu_unit lbs_v2 = {
	.name = "LayerBlend",
	.num = 4,
	.ids = lb_ids,
	.pec_ofss = lb_pec_ofss_v2,
	.ofss = lb_ofss_v2,
};

static const struct dpu_unit sts_v1 = {
	.name = "Store",
	.num = ARRAY_SIZE(st_ids),
	.ids = st_ids,
	.pec_ofss = st_pec_ofss_v1,
	.ofss = st_ofss_v1,
};

static const struct dpu_unit sts_v2 = {
	.name = "Store",
	.num = ARRAY_SIZE(st_ids),
	.ids = st_ids,
	.pec_ofss = st_pec_ofss_v2,
	.ofss = st_ofss_v2,
};

static const struct dpu_unit tcons_v1 = {
	.name = "TCon",
	.num = ARRAY_SIZE(tcon_ids),
	.ids = tcon_ids,
	.pec_ofss = NULL,
	.ofss = tcon_ofss_v1,
};

static const struct dpu_unit tcons_v2 = {
	.name = "TCon",
	.num = ARRAY_SIZE(tcon_ids),
	.ids = tcon_ids,
	.pec_ofss = NULL,
	.ofss = tcon_ofss_v2,
};

static const struct dpu_unit vss_v1 = {
	.name = "VScaler",
	.num = ARRAY_SIZE(vs_ids),
	.ids = vs_ids,
	.pec_ofss = vs_pec_ofss_v1,
	.ofss = vs_ofss_v1,
};

static const struct dpu_unit vss_v2 = {
	.name = "VScaler",
	.num = ARRAY_SIZE(vs_ids),
	.ids = vs_ids,
	.pec_ofss = vs_pec_ofss_v2,
	.ofss = vs_ofss_v2,
};

static const struct cm_reg_ofs cm_reg_ofs_v1 = {
	.ipidentifier = 0,
	.lockunlock = 0x80,
	.lockstatus = 0x84,
	.userinterruptmask = 0x88,
	.interruptenable = 0x94,
	.interruptpreset = 0xa0,
	.interruptclear = 0xac,
	.interruptstatus = 0xb8,
	.userinterruptenable = 0x100,
	.userinterruptpreset = 0x10c,
	.userinterruptclear = 0x118,
	.userinterruptstatus = 0x124,
	.generalpurpose = 0x200,
};

static const struct cm_reg_ofs cm_reg_ofs_v2 = {
	.ipidentifier = 0,
	.lockunlock = 0x40,
	.lockstatus = 0x44,
	.userinterruptmask = 0x48,
	.interruptenable = 0x50,
	.interruptpreset = 0x58,
	.interruptclear = 0x60,
	.interruptstatus = 0x68,
	.userinterruptenable = 0x80,
	.userinterruptpreset = 0x88,
	.userinterruptclear = 0x90,
	.userinterruptstatus = 0x98,
	.generalpurpose = 0x100,
};

static const unsigned int intsteer_map_v1[] = {
	/*  0    1    2    3    4    5    6    7 */	/*  0~31: int0 */
	  448, 449, 450,  64,  65,  66,  67,  68,
	/*  8    9   10   11   12   13   14   15 */
	   69,  70, 193, 194, 195, 196, 197, 320,
	/* 16   17   18   19   20   21   22   23 */
	  321, 322, 384, 385, 386,  NA, 323,  NA,
	/* 24   25   26   27   28   29   30   31 */
	  387,  71, 198,  72,  73,  74,  75,  76,
	/* 32   33   34   35   36   37   38   39 */	/* 32~63: int1 */
	   77,  78,  79,  80,  81, 199, 200, 201,
	/* 40   41   42   43   44   45   46   47 */
	  202, 203, 204, 205, 206, 207, 208, 324,
	/* 48   49   50   51   52   53   54   55 */
	  389,  NA,   0,   1,   2,   3,   4,  82,
	/* 56   57   58   59   60   61   62   63 */
	   83,  84,  85, 209, 210, 211, 212, 325,
	/* 64   65   66 */				/*   64+: int2 */
	  326, 390, 391,
};
static const unsigned long unused_irq_v1[] = {0x00a00000, 0x00020000,
					      0xfffffff8};

static const unsigned int intsteer_map_v2[] = {
	/*  0    1    2    3    4    5    6    7 */	/*  0~31: int0 */
	  448, 449, 450,  64,  65,  66,  67,  68,
	/*  8    9   10   11   12   13   14   15 */
	   69,  70, 193, 194, 195, 196, 197,  72,
	/* 16   17   18   19   20   21   22   23 */
	   73,  74,  75,  76,  77,  78,  79,  80,
	/* 24   25   26   27   28   29   30   31 */
	   81, 199, 200, 201, 202, 203, 204, 205,
	/* 32   33   34   35   36   37   38   39 */	/*   32+: int1 */
	  206, 207, 208,  NA,   0,   1,   2,   3,
	/* 40   41   42   43   44   45   46   47 */
	    4,  82,  83,  84,  85, 209, 210, 211,
	/* 48 */
	  212,
};
static const unsigned long unused_irq_v2[] = {0x00000000, 0xfffe0008};

static const unsigned int sw2hw_irq_map_v2[] = {
	/*  0    1    2    3    4    5    6    7 */
	    0,   1,   2,   3,   4,   5,   6,   7,
	/*  8    9   10   11   12   13   14   15 */
	    8,   9,  10,  11,  12,  13,  14,  NA,
	/* 16   17   18   19   20   21   22   23 */
	   NA,  NA,  NA,  NA,  NA,  NA,  NA,  NA,
	/* 24   25   26   27   28   29   30   31 */
	   NA,  NA,  NA,  15,  16,  17,  18,  19,
	/* 32   33   34   35   36   37   38   39 */
	   20,  21,  22,  23,  24,  25,  26,  27,
	/* 40   41   42   43   44   45   46   47 */
	   28,  29,  30,  31,  32,  33,  34,  NA,
	/* 48   49   50   51   52   53   54   55 */
	   NA,  NA,  36,  37,  38,  39,  40,  41,
	/* 56   57   58   59   60   61   62   63 */
	   42,  43,  44,  45,  46,  47,  48,  NA,
	/* 64   65   66 */
	   NA,  NA,  NA,
};

/* FIXME: overkill for some N/As, revive them when needed */
static const unsigned int sw2hw_block_id_map_v2[] = {
	/*   0     1     2     3     4     5     6     7 */
	  0x00,   NA,   NA, 0x03,   NA,   NA,   NA, 0x07,
	/*   8     9    10    11    12    13    14    15 */
	  0x08,   NA, 0x0a,   NA, 0x0c,   NA, 0x0e,   NA,
	/*  16    17    18    19    20    21    22    23 */
	  0x10,   NA, 0x12,   NA,   NA,   NA,   NA,   NA,
	/*  24    25    26    27    28    29    30    31 */
	    NA,   NA, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19,
	/*  32    33    34    35    36    37    38    39 */
	  0x1a,   NA,   NA, 0x1b, 0x1c, 0x1d,   NA,   NA,
	/*  40    41    42    43    44    45    46    47 */
	  0x1e, 0x1f, 0x20,   NA, 0x21, 0x22, 0x23, 0x24,
	/*  48    49    50    51    52    53    54    55 */
	    NA,   NA,   NA,   NA,   NA,   NA,   NA,   NA,
	/*  56    57    58    59    60    61    62    63 */
	    NA,   NA,   NA,   NA,   NA,   NA,   NA,   NA,
	/*  64    65    66    67 */
	    NA,   NA,   NA,   NA,
};

static const struct dpu_devtype dpu_type_v1 = {
	.cm_ofs = 0x0,
	.cfs = &cfs_v1,
	.decs = &decs_v1,
	.eds = &eds_v1,
	.fds = &fds_v1,
	.fes = &fes_v1,
	.fgs = &fgs_v1,
	.fls = &fls_v1,
	.fws = &fws_v1,
	.hss = &hss_v1,
	.lbs = &lbs_v1,
	.sts = &sts_v1,
	.tcons = &tcons_v1,
	.vss = &vss_v1,
	.cm_reg_ofs = &cm_reg_ofs_v1,
	.intsteer_map = intsteer_map_v1,
	.intsteer_map_size = ARRAY_SIZE(intsteer_map_v1),
	.unused_irq = unused_irq_v1,
	.plane_src_na_mask = 0xffffff80,
	.has_capture = true,
	.has_prefetch = false,
	.has_disp_sel_clk = false,
	.has_dual_ldb = false,
	.has_pc = false,
	.has_syncmode_fixup = false,
	.pixel_link_quirks = false,
	.pixel_link_nhvsync = false,
	.version = DPU_V1,
};

static const struct dpu_devtype dpu_type_v2_qm = {
	.cm_ofs = 0x0,
	.cfs = &cfs_v2,
	.decs = &decs_v2,
	.eds = &eds_v2,
	.fds = &fds_v2,
	.fes = &fes_v2,
	.fgs = &fgs_v2,
	.fls = &fls_v2,
	.fws = &fws_v2,
	.hss = &hss_v2,
	.lbs = &lbs_v2,
	.sts = &sts_v2,
	.tcons = &tcons_v2,
	.vss = &vss_v2,
	.cm_reg_ofs = &cm_reg_ofs_v2,
	.intsteer_map = intsteer_map_v2,
	.intsteer_map_size = ARRAY_SIZE(intsteer_map_v2),
	.unused_irq = unused_irq_v2,
	.sw2hw_irq_map = sw2hw_irq_map_v2,
	.sw2hw_block_id_map = sw2hw_block_id_map_v2,
	.plane_src_na_mask = 0xffffffe2,
	.has_capture = false,
	.has_prefetch = true,
	.has_disp_sel_clk = true,
	.has_dual_ldb = false,
	.has_pc = true,
	.has_syncmode_fixup = true,
	.syncmode_min_prate = 300000,
	.singlemode_max_width = 1920,
	.master_stream_id = 1,
	.pixel_link_quirks = true,
	.pixel_link_nhvsync = true,
	.version = DPU_V2,
};

static const struct dpu_devtype dpu_type_v2_qxp = {
	.cm_ofs = 0x0,
	.cfs = &cfs_v2,
	.decs = &decs_v2,
	.eds = &eds_v2,
	.fds = &fds_v2,
	.fes = &fes_v2,
	.fgs = &fgs_v2,
	.fls = &fls_v2,
	.fws = &fws_v2,
	.hss = &hss_v2,
	.lbs = &lbs_v2,
	.sts = &sts_v2,
	.tcons = &tcons_v2,
	.vss = &vss_v2,
	.cm_reg_ofs = &cm_reg_ofs_v2,
	.intsteer_map = intsteer_map_v2,
	.intsteer_map_size = ARRAY_SIZE(intsteer_map_v2),
	.unused_irq = unused_irq_v2,
	.sw2hw_irq_map = sw2hw_irq_map_v2,
	.sw2hw_block_id_map = sw2hw_block_id_map_v2,
	.plane_src_na_mask = 0xffffffe2,
	.has_capture = false,
	.has_prefetch = true,
	.has_disp_sel_clk = false,
	.has_dual_ldb = true,
	.has_pc = true,
	.has_syncmode_fixup = false,
	.syncmode_min_prate = UINT_MAX,	/* pc is unused */
	.singlemode_max_width = UINT_MAX,	/* pc is unused */
	.pixel_link_quirks = true,
	.pixel_link_nhvsync = true,
	.version = DPU_V2,
};

static const struct of_device_id dpu_dt_ids[] = {
	{
		.compatible = "fsl,imx8qm-dpu",
		.data = &dpu_type_v2_qm,
	}, {
		.compatible = "fsl,imx8qxp-dpu",
		.data = &dpu_type_v2_qxp,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, dpu_dt_ids);

bool dpu_has_pc(struct dpu_soc *dpu)
{
	return dpu->devtype->has_pc;
}
EXPORT_SYMBOL_GPL(dpu_has_pc);

unsigned int dpu_get_syncmode_min_prate(struct dpu_soc *dpu)
{
	if (dpu->devtype->has_pc)
		return dpu->devtype->syncmode_min_prate;
	else
		return UINT_MAX;
}
EXPORT_SYMBOL_GPL(dpu_get_syncmode_min_prate);

unsigned int dpu_get_singlemode_max_width(struct dpu_soc *dpu)
{
	if (dpu->devtype->has_pc)
		return dpu->devtype->singlemode_max_width;
	else
		return UINT_MAX;
}
EXPORT_SYMBOL_GPL(dpu_get_singlemode_max_width);

unsigned int dpu_get_master_stream_id(struct dpu_soc *dpu)
{
	if (dpu->devtype->has_pc)
		return dpu->devtype->master_stream_id;
	else
		return UINT_MAX;
}
EXPORT_SYMBOL_GPL(dpu_get_master_stream_id);

bool dpu_vproc_has_fetcheco_cap(u32 cap_mask)
{
	return !!(cap_mask & DPU_VPROC_CAP_FETCHECO);
}
EXPORT_SYMBOL_GPL(dpu_vproc_has_fetcheco_cap);

bool dpu_vproc_has_hscale_cap(u32 cap_mask)
{
	return !!(cap_mask & DPU_VPROC_CAP_HSCALE);
}
EXPORT_SYMBOL_GPL(dpu_vproc_has_hscale_cap);

bool dpu_vproc_has_vscale_cap(u32 cap_mask)
{
	return !!(cap_mask & DPU_VPROC_CAP_VSCALE);
}
EXPORT_SYMBOL_GPL(dpu_vproc_has_vscale_cap);

u32 dpu_vproc_get_fetcheco_cap(u32 cap_mask)
{
	return cap_mask & DPU_VPROC_CAP_FETCHECO;
}
EXPORT_SYMBOL_GPL(dpu_vproc_get_fetcheco_cap);

u32 dpu_vproc_get_hscale_cap(u32 cap_mask)
{
	return cap_mask & DPU_VPROC_CAP_HSCALE;
}
EXPORT_SYMBOL_GPL(dpu_vproc_get_hscale_cap);

u32 dpu_vproc_get_vscale_cap(u32 cap_mask)
{
	return cap_mask & DPU_VPROC_CAP_VSCALE;
}
EXPORT_SYMBOL_GPL(dpu_vproc_get_vscale_cap);

int dpu_format_horz_chroma_subsampling(u32 format)
{
	switch (format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
		return 2;
	default:
		return 1;
	}
}

int dpu_format_vert_chroma_subsampling(u32 format)
{
	switch (format) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		return 2;
	default:
		return 1;
	}
}

int dpu_format_num_planes(u32 format)
{
	switch (format) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		return 2;
	default:
		return 1;
	}
}

int dpu_format_plane_width(int width, u32 format, int plane)
{
	if (plane >= dpu_format_num_planes(format))
		return 0;

	if (plane == 0)
		return width;

	return width / dpu_format_horz_chroma_subsampling(format);
}

int dpu_format_plane_height(int height, u32 format, int plane)
{
	if (plane >= dpu_format_num_planes(format))
		return 0;

	if (plane == 0)
		return height;

	return height / dpu_format_vert_chroma_subsampling(format);
}

#define _DPU_UNITS_INIT(unit)						\
{									\
	const struct dpu_unit *us = devtype->unit##s;			\
	int i;								\
									\
	/* software check */						\
	if (WARN_ON(us->num > ARRAY_SIZE(unit##_ids)))			\
		return -EINVAL;						\
									\
	for (i = 0; i < us->num; i++)					\
		_dpu_##unit##_init(dpu, us->ids[i]);			\
}

static int
_dpu_submodules_init(struct dpu_soc *dpu, struct platform_device *pdev)
{
	const struct dpu_devtype *devtype = dpu->devtype;

	_DPU_UNITS_INIT(cf);
	_DPU_UNITS_INIT(dec);
	_DPU_UNITS_INIT(ed);
	_DPU_UNITS_INIT(fd);
	_DPU_UNITS_INIT(fe);
	_DPU_UNITS_INIT(fg);
	_DPU_UNITS_INIT(fl);
	_DPU_UNITS_INIT(fw);
	_DPU_UNITS_INIT(hs);
	_DPU_UNITS_INIT(lb);
	_DPU_UNITS_INIT(tcon);
	_DPU_UNITS_INIT(vs);

	return 0;
}

#define DPU_UNIT_INIT(dpu, base, unit, name, id, pec_ofs, ofs)		\
{									\
	int ret;							\
	ret = dpu_##unit##_init((dpu),	(id),				\
				(pec_ofs) ? (base) + (pec_ofs) : 0,	\
				(base) + (ofs));			\
	if (ret) {							\
		dev_err((dpu)->dev, "init %s%d failed with %d\n",	\
						(name), (id), ret);	\
		return ret;						\
	}								\
}

#define DPU_UNITS_INIT(unit)						\
{									\
	const struct dpu_unit *us = devtype->unit##s;			\
	int i;								\
									\
	/* software check */						\
	if (WARN_ON(us->num > ARRAY_SIZE(unit##_ids)))			\
		return -EINVAL;						\
									\
	for (i = 0; i < us->num; i++)					\
		DPU_UNIT_INIT(dpu, dpu_base, unit, us->name,		\
			      us->ids[i],				\
			      us->pec_ofss ? us->pec_ofss[i] : 0,	\
			      us->ofss[i]);				\
}

static int dpu_submodules_init(struct dpu_soc *dpu,
		struct platform_device *pdev, unsigned long dpu_base)
{
	const struct dpu_devtype *devtype = dpu->devtype;
	const struct dpu_unit *fds = devtype->fds;
	const struct dpu_unit *fls = devtype->fls;
	const struct dpu_unit *fws = devtype->fws;
	const struct dpu_unit *tcons = devtype->tcons;

	DPU_UNITS_INIT(cf);
	DPU_UNITS_INIT(dec);
	DPU_UNITS_INIT(ed);
	DPU_UNITS_INIT(fd);
	DPU_UNITS_INIT(fe);
	DPU_UNITS_INIT(fg);
	DPU_UNITS_INIT(fl);
	DPU_UNITS_INIT(fw);
	DPU_UNITS_INIT(hs);
	DPU_UNITS_INIT(lb);
	DPU_UNITS_INIT(st);
	DPU_UNITS_INIT(tcon);
	DPU_UNITS_INIT(vs);

	/* get DPR channel for submodules */
	if (devtype->has_prefetch) {
		struct dpu_fetchunit *fu;
		struct dprc *dprc;
		int i;

		for (i = 0; i < fds->num; i++) {
			dprc = dprc_lookup_by_phandle(dpu->dev,
						      "fsl,dpr-channels",
						      fds->dprc_ids[i]);
			if (!dprc)
				return -EPROBE_DEFER;

			fu = dpu_fd_get(dpu, i);
			fetchunit_get_dprc(fu, dprc);
			dpu_fd_put(fu);
		}

		for (i = 0; i < fls->num; i++) {
			dprc = dprc_lookup_by_phandle(dpu->dev,
						      "fsl,dpr-channels",
						      fls->dprc_ids[i]);
			if (!dprc)
				return -EPROBE_DEFER;

			fu = dpu_fl_get(dpu, i);
			fetchunit_get_dprc(fu, dprc);
			dpu_fl_put(fu);
		}

		for (i = 0; i < fws->num; i++) {
			dprc = dprc_lookup_by_phandle(dpu->dev,
						      "fsl,dpr-channels",
						      fws->dprc_ids[i]);
			if (!dprc)
				return -EPROBE_DEFER;

			fu = dpu_fw_get(dpu, fw_ids[i]);
			fetchunit_get_dprc(fu, dprc);
			dpu_fw_put(fu);
		}
	}

	/* get pixel combiner */
	if (devtype->has_pc) {
		struct dpu_tcon *tcon;
		struct pc *pc =
			pc_lookup_by_phandle(dpu->dev, "fsl,pixel-combiner");
		int i;

		if (!pc)
			return -EPROBE_DEFER;

		for (i = 0; i < tcons->num; i++) {
			tcon = dpu_tcon_get(dpu, i);
			tcon_get_pc(tcon, pc);
			dpu_tcon_put(tcon);
		}
	}

	return 0;
}

#define DPU_UNITS_ADDR_DBG(unit)					\
{									\
	const struct dpu_unit *us = devtype->unit##s;			\
	int i;								\
	for (i = 0; i < us->num; i++) {					\
		if (us->pec_ofss) {					\
			dev_dbg(&pdev->dev, "%s%d: pixengcfg @ 0x%08lx,"\
				" unit @ 0x%08lx\n", us->name,		\
				us->ids[i],				\
				dpu_base + us->pec_ofss[i],		\
				dpu_base + us->ofss[i]);		\
		} else {						\
			dev_dbg(&pdev->dev,				\
				"%s%d: unit @ 0x%08lx\n", us->name,	\
				us->ids[i], dpu_base + us->ofss[i]);	\
		}							\
	}								\
}

enum dpu_irq_line {
	DPU_IRQ_LINE_CM = 0,
	DPU_IRQ_LINE_STREAM0A = 1,
	DPU_IRQ_LINE_STREAM1A = 3,
	DPU_IRQ_LINE_RESERVED0 = 5,
	DPU_IRQ_LINE_RESERVED1 = 6,
	DPU_IRQ_LINE_BLIT = 7,
};

static inline unsigned int dpu_get_max_intsteer_num(enum dpu_irq_line irq_line)
{
	return 64 * (++irq_line) - 1;
}

static inline unsigned int dpu_get_min_intsteer_num(enum dpu_irq_line irq_line)
{
	return 64 * irq_line;
}

static void
dpu_inner_irq_handle(struct irq_desc *desc, enum dpu_irq_line irq_line)
{
	struct dpu_soc *dpu = irq_desc_get_handler_data(desc);
	const struct dpu_devtype *devtype = dpu->devtype;
	const struct cm_reg_ofs *ofs = devtype->cm_reg_ofs;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int i, virq, min_intsteer_num, max_intsteer_num;
	u32 status;

	chained_irq_enter(chip, desc);

	min_intsteer_num = dpu_get_min_intsteer_num(irq_line);
	max_intsteer_num = dpu_get_max_intsteer_num(irq_line);

	for (i = 0; i < devtype->intsteer_map_size; i++) {
		if (devtype->intsteer_map[i] >= min_intsteer_num &&
		    devtype->intsteer_map[i] <= max_intsteer_num) {
			status = dpu_cm_read(dpu,
					USERINTERRUPTSTATUS(ofs, i / 32));
			status &= dpu_cm_read(dpu,
					USERINTERRUPTENABLE(ofs, i / 32));

			if (status & BIT(i % 32)) {
				virq = irq_linear_revmap(dpu->domain, i);
				if (virq) {
					generic_handle_irq(virq);
				}
			}
		}
	}

	chained_irq_exit(chip, desc);
}

#define DPU_INNER_IRQ_HANDLER_DEFINE(name1, name2)		\
static void dpu_##name1##_irq_handler(struct irq_desc *desc)	\
{								\
	dpu_inner_irq_handle(desc, DPU_IRQ_LINE_##name2);		\
}

DPU_INNER_IRQ_HANDLER_DEFINE(cm, CM)
DPU_INNER_IRQ_HANDLER_DEFINE(stream0a, STREAM0A)
DPU_INNER_IRQ_HANDLER_DEFINE(stream1a, STREAM1A)
DPU_INNER_IRQ_HANDLER_DEFINE(reserved0, RESERVED0)
DPU_INNER_IRQ_HANDLER_DEFINE(reserved1, RESERVED1)
DPU_INNER_IRQ_HANDLER_DEFINE(blit, BLIT)

int dpu_map_inner_irq(struct dpu_soc *dpu, int irq)
{
	const unsigned int *sw2hw_irq_map = dpu->devtype->sw2hw_irq_map;
	int virq, mapped_irq;

	mapped_irq = sw2hw_irq_map ? sw2hw_irq_map[irq] : irq;
	if (WARN_ON(mapped_irq == NA))
		return -EINVAL;

	virq = irq_linear_revmap(dpu->domain, mapped_irq);
	if (!virq)
		virq = irq_create_mapping(dpu->domain, mapped_irq);

	return virq;
}
EXPORT_SYMBOL_GPL(dpu_map_inner_irq);

static int platform_remove_devices_fn(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static void platform_device_unregister_children(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, platform_remove_devices_fn);
}

struct dpu_platform_reg {
	struct dpu_client_platformdata pdata;
	const char *name;
};

static struct dpu_platform_reg client_reg[] = {
	{
		/* placeholder */
		.pdata = { },
		.name = "imx-dpu-csi",
	}, {
		/* placeholder */
		.pdata = { },
		.name = "imx-dpu-csi",
	}, {
		.pdata = {
			.stream_id = 0,
		},
		.name = "imx-dpu-crtc",
	}, {
		.pdata = {
			.stream_id = 1,
		},
		.name = "imx-dpu-crtc",
	}, {
		.pdata = { },
		.name = "imx-drm-dpu-bliteng",
	},
};

static DEFINE_MUTEX(dpu_client_id_mutex);
static int dpu_client_id;

static int dpu_get_plane_resource(struct dpu_soc *dpu,
				  struct dpu_plane_res *res)
{
	const struct dpu_unit *fds = dpu->devtype->fds;
	const struct dpu_unit *fls = dpu->devtype->fls;
	const struct dpu_unit *fws = dpu->devtype->fws;
	const struct dpu_unit *lbs = dpu->devtype->lbs;
	struct dpu_plane_grp *grp = plane_res_to_grp(res);
	int i;

	for (i = 0; i < ARRAY_SIZE(res->cf); i++) {
		res->cf[i] = dpu_cf_get(dpu, i);
		if (IS_ERR(res->cf[i]))
			return PTR_ERR(res->cf[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->ed); i++) {
		res->ed[i] = dpu_ed_get(dpu, i);
		if (IS_ERR(res->ed[i]))
			return PTR_ERR(res->ed[i]);
	}
	for (i = 0; i < fds->num; i++) {
		res->fd[i] = dpu_fd_get(dpu, i);
		if (IS_ERR(res->fd[i]))
			return PTR_ERR(res->fd[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fe); i++) {
		res->fe[i] = dpu_fe_get(dpu, i);
		if (IS_ERR(res->fe[i]))
			return PTR_ERR(res->fe[i]);
		grp->hw_plane_fetcheco_num = ARRAY_SIZE(res->fe);
	}
	for (i = 0; i < fls->num; i++) {
		res->fl[i] = dpu_fl_get(dpu, i);
		if (IS_ERR(res->fl[i]))
			return PTR_ERR(res->fl[i]);
	}
	for (i = 0; i < fws->num; i++) {
		res->fw[i] = dpu_fw_get(dpu, fw_ids[i]);
		if (IS_ERR(res->fw[i]))
			return PTR_ERR(res->fw[i]);
	}
	/* HScaler could be shared with capture. */
	if (display_plane_video_proc) {
		for (i = 0; i < ARRAY_SIZE(res->hs); i++) {
			res->hs[i] = dpu_hs_get(dpu, hs_ids[i]);
			if (IS_ERR(res->hs[i]))
				return PTR_ERR(res->hs[i]);
		}
		grp->hw_plane_hscaler_num = ARRAY_SIZE(res->hs);
	}
	for (i = 0; i < lbs->num; i++) {
		res->lb[i] = dpu_lb_get(dpu, i);
		if (IS_ERR(res->lb[i]))
			return PTR_ERR(res->lb[i]);
	}
	/* VScaler could be shared with capture. */
	if (display_plane_video_proc) {
		for (i = 0; i < ARRAY_SIZE(res->vs); i++) {
			res->vs[i] = dpu_vs_get(dpu, vs_ids[i]);
			if (IS_ERR(res->vs[i]))
				return PTR_ERR(res->vs[i]);
		}
		grp->hw_plane_vscaler_num = ARRAY_SIZE(res->vs);
	}

	grp->hw_plane_num = fds->num + fls->num + fws->num;

	return 0;
}

static void dpu_put_plane_resource(struct dpu_plane_res *res)
{
	struct dpu_plane_grp *grp = plane_res_to_grp(res);
	int i;

	for (i = 0; i < ARRAY_SIZE(res->cf); i++) {
		if (!IS_ERR_OR_NULL(res->cf[i]))
			dpu_cf_put(res->cf[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->ed); i++) {
		if (!IS_ERR_OR_NULL(res->ed[i]))
			dpu_ed_put(res->ed[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fd); i++) {
		if (!IS_ERR_OR_NULL(res->fd[i]))
			dpu_fd_put(res->fd[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fe); i++) {
		if (!IS_ERR_OR_NULL(res->fe[i]))
			dpu_fe_put(res->fe[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fl); i++) {
		if (!IS_ERR_OR_NULL(res->fl[i]))
			dpu_fl_put(res->fl[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fw); i++) {
		if (!IS_ERR_OR_NULL(res->fw[i]))
			dpu_fw_put(res->fw[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->hs); i++) {
		if (!IS_ERR_OR_NULL(res->hs[i]))
			dpu_hs_put(res->hs[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->lb); i++) {
		if (!IS_ERR_OR_NULL(res->lb[i]))
			dpu_lb_put(res->lb[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->vs); i++) {
		if (!IS_ERR_OR_NULL(res->vs[i]))
			dpu_vs_put(res->vs[i]);
	}

	grp->hw_plane_num = 0;
}

static int dpu_add_client_devices(struct dpu_soc *dpu)
{
	const struct dpu_devtype *devtype = dpu->devtype;
	struct device *dev = dpu->dev;
	struct dpu_platform_reg *reg;
	struct dpu_plane_grp *plane_grp;
	struct dpu_store *st9 = NULL;
	size_t client_num, reg_size;
	int i, id, ret;

	client_num = ARRAY_SIZE(client_reg);
	if (!devtype->has_capture)
		client_num -= 2;

	reg = devm_kcalloc(dev, client_num, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENODEV;

	plane_grp = devm_kzalloc(dev, sizeof(*plane_grp), GFP_KERNEL);
	if (!plane_grp)
		return -ENODEV;

	mutex_init(&plane_grp->mutex);

	mutex_lock(&dpu_client_id_mutex);
	id = dpu_client_id;
	dpu_client_id += client_num;
	mutex_unlock(&dpu_client_id_mutex);

	reg_size = client_num * sizeof(struct dpu_platform_reg);
	if (devtype->has_capture)
		memcpy(reg, client_reg, reg_size);
	else
		memcpy(reg, &client_reg[2], reg_size);

	plane_grp->src_na_mask = devtype->plane_src_na_mask;
	plane_grp->id = id / client_num;
	plane_grp->has_vproc = display_plane_video_proc;

	ret = dpu_get_plane_resource(dpu, &plane_grp->res);
	if (ret)
		goto err_get_plane_res;

	/*
	 * Store9 is shared bewteen display engine(for sync mode
	 * fixup) and blit engine.
	 */
	if (devtype->has_syncmode_fixup) {
		st9 = dpu_st_get(dpu, 9);
		if (IS_ERR(st9)) {
			ret = PTR_ERR(st9);
			goto err_get_plane_res;
		}
	}

	for (i = 0; i < client_num; i++) {
		struct platform_device *pdev;
		struct device_node *of_node = NULL;
		bool is_disp, is_bliteng;

		if (devtype->has_capture) {
			is_bliteng = (i == 4) ? true : false;
			is_disp = (!is_bliteng) && ((i / 2) ? true : false);
		} else {
			is_bliteng = (i == 2) ? true : false;
			is_disp = !is_bliteng;
		}

		if (is_bliteng) {
			/* As bliteng has no of_node, so to use dpu's. */
			of_node = dev->of_node;
		} else {
			/*
			 * Associate subdevice with the
			 * corresponding port node.
			 */
			of_node = of_graph_get_port_by_id(dev->of_node, i);
			if (!of_node) {
				dev_info(dev, "no port@%d node in %s, not using %s%d\n",
					i, dev->of_node->full_name,
					is_disp ? "DISP" : "CSI", i % 2);
				continue;
			}
		}

		if (is_disp) {
			reg[i].pdata.plane_grp = plane_grp;
			reg[i].pdata.di_grp_id = plane_grp->id;
			reg[i].pdata.st9 = st9;
		}

		pdev = platform_device_alloc(reg[i].name, id++);
		if (!pdev) {
			ret = -ENOMEM;
			goto err_register;
		}

		pdev->dev.parent = dev;

		reg[i].pdata.of_node = of_node;
		ret = platform_device_add_data(pdev, &reg[i].pdata,
					       sizeof(reg[i].pdata));
		if (!ret)
			ret = platform_device_add(pdev);
		if (ret) {
			platform_device_put(pdev);
			goto err_register;
		}
	}

	return 0;

err_register:
	platform_device_unregister_children(to_platform_device(dev));
	if (devtype->has_syncmode_fixup)
		dpu_st_put(st9);
err_get_plane_res:
	dpu_put_plane_resource(&plane_grp->res);

	return ret;
}

#define IRQSTEER_CHANnCTL	0x0
#define IRQSTEER_CHANnCTL_CH(n)	BIT(n)
#define IRQSTEER_CHANnMASK(n)	((n) + 4)
#define LINE_TO_MASK_OFFSET(n)	((15 - ((n) / 32)) * 4)
#define LINE_TO_MASK_SHIFT(n)	((n) % 32)

static void dpu_inner_irq_gc_mask_set_bit(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	struct dpu_soc *dpu = gc->domain->host_data;
	unsigned long flags;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	spin_lock_irqsave(&dpu->intsteer_lock, flags);
	if (++dpu->intsteer_usecount == 1)
		/* assuming fast I/O regmap */
		regmap_write(dpu->intsteer_regmap, IRQSTEER_CHANnCTL,
				IRQSTEER_CHANnCTL_CH(0));
	spin_unlock_irqrestore(&dpu->intsteer_lock, flags);
	*ct->mask_cache |= mask;
	irq_reg_writel(gc, *ct->mask_cache, ct->regs.mask);
	irq_gc_unlock(gc);
}

static void dpu_inner_irq_gc_mask_clr_bit(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	struct dpu_soc *dpu = gc->domain->host_data;
	unsigned long flags;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	spin_lock_irqsave(&dpu->intsteer_lock, flags);
	if (!--dpu->intsteer_usecount) {
		WARN(dpu->intsteer_usecount < 0,
			"intsteer usecount %d is less than zero",
			dpu->intsteer_usecount);
		regmap_write(dpu->intsteer_regmap, IRQSTEER_CHANnCTL, 0);
	}
	spin_unlock_irqrestore(&dpu->intsteer_lock, flags);
	*ct->mask_cache &= ~mask;
	irq_reg_writel(gc, *ct->mask_cache, ct->regs.mask);
	irq_gc_unlock(gc);
}

static void
dpu_inner_intsteer_enable_line(struct dpu_soc *dpu, unsigned int line)
{
	unsigned int offset = LINE_TO_MASK_OFFSET(line);
	unsigned int shift = LINE_TO_MASK_SHIFT(line);

	regmap_update_bits(dpu->intsteer_regmap, IRQSTEER_CHANnMASK(offset),
			   BIT(shift), BIT(shift));
}

static void dpu_inner_intsteer_enable_lines(struct dpu_soc *dpu)
{
	const struct dpu_devtype *devtype = dpu->devtype;
	int i;

	for (i = 0; i < devtype->intsteer_map_size; i++) {
		if (devtype->intsteer_map[i] == NA)
			continue;

		dpu_inner_intsteer_enable_line(dpu, devtype->intsteer_map[i]);
	}
}

static int dpu_inner_irq_init(struct dpu_soc *dpu)
{
	const struct dpu_devtype *devtype = dpu->devtype;
	const struct cm_reg_ofs *ofs = devtype->cm_reg_ofs;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int ret, i;

	dpu_inner_intsteer_enable_lines(dpu);

	dpu->domain = irq_domain_add_linear(dpu->dev->of_node,
					    devtype->intsteer_map_size,
					    &irq_generic_chip_ops, dpu);
	if (!dpu->domain) {
		dev_err(dpu->dev, "failed to add irq domain\n");
		return -ENODEV;
	}

	ret = irq_alloc_domain_generic_chips(dpu->domain, 32, 1, "DPU",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dpu->dev, "failed to alloc generic irq chips\n");
		irq_domain_remove(dpu->domain);
		return ret;
	}

	for (i = 0; i < devtype->intsteer_map_size; i += 32) {
		/* Mask and clear all interrupts */
		dpu_cm_write(dpu, 0,
					USERINTERRUPTENABLE(ofs, i / 32));
		dpu_cm_write(dpu, ~devtype->unused_irq[i / 32],
					USERINTERRUPTCLEAR(ofs, i / 32));
		dpu_cm_write(dpu, 0,
					INTERRUPTENABLE(ofs, i / 32));
		dpu_cm_write(dpu, ~devtype->unused_irq[i / 32],
					INTERRUPTCLEAR(ofs, i / 32));

		/* Set all interrupts to user mode */
		dpu_cm_write(dpu, ~devtype->unused_irq[i / 32],
					USERINTERRUPTMASK(ofs, i / 32));

		gc = irq_get_domain_generic_chip(dpu->domain, i);
		gc->reg_base = dpu->cm_reg;
		gc->unused = devtype->unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = dpu_inner_irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = dpu_inner_irq_gc_mask_set_bit;
		ct->regs.ack = USERINTERRUPTCLEAR(ofs, i / 32);
		ct->regs.mask = USERINTERRUPTENABLE(ofs, i / 32);
	}

#define DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(name)	\
irq_set_chained_handler_and_data(dpu->irq_##name, dpu_##name##_irq_handler, dpu)

	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(cm);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(stream0a);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(stream1a);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(reserved0);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(reserved1);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA1(blit);

	return 0;
}

static void dpu_inner_irq_exit(struct dpu_soc *dpu)
{
	const struct dpu_devtype *devtype = dpu->devtype;
	unsigned int i, irq;

#define DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(name)	\
irq_set_chained_handler_and_data(dpu->irq_##name, NULL, NULL)

	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(cm);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(stream0a);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(stream1a);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(reserved0);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(reserved1);
	DPU_INNER_IRQ_SET_CHAINED_HANDLER_AND_DATA2(blit);

	for (i = 0; i < devtype->intsteer_map_size; i++) {
		irq = irq_linear_revmap(dpu->domain, i);
		if (irq)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(dpu->domain);
}

static irqreturn_t dpu_dpr0_irq_handler(int irq, void *desc)
{
	struct dpu_soc *dpu = desc;
	const struct dpu_unit *fls = dpu->devtype->fls;
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < fls->num; i++) {
		fu = dpu->fl_priv[i];
		dprc_irq_handle(fu->dprc);
	}

	return IRQ_HANDLED;
}

static irqreturn_t dpu_dpr1_irq_handler(int irq, void *desc)
{
	struct dpu_soc *dpu = desc;
	const struct dpu_unit *fds = dpu->devtype->fds;
	const struct dpu_unit *fws = dpu->devtype->fws;
	struct dpu_fetchunit *fu;
	int i;

	for (i = 0; i < fds->num; i++) {
		fu = dpu->fd_priv[i];
		dprc_irq_handle(fu->dprc);
	}

	for (i = 0; i < fws->num; i++) {
		fu = dpu->fw_priv[i];
		dprc_irq_handle(fu->dprc);
	}

	return IRQ_HANDLED;
}

static void dpu_debug_ip_identity(struct dpu_soc *dpu)
{
	struct device *dev = dpu->dev;
	const struct cm_reg_ofs *ofs = dpu->devtype->cm_reg_ofs;
	u32 reg;
	int id = 0;

	reg = dpu_cm_read(dpu, IPIDENTIFIER(ofs));

	dev_dbg(dev, "%d) Maturatiy level:\n", ++id);
	switch (reg & DESIGNMATURITYLEVEL_MASK) {
	case DESIGNMATURITYLEVEL__PREFS:
		dev_dbg(dev, "\tPre feasibility study.\n");
		break;
	case DESIGNMATURITYLEVEL__FS:
		dev_dbg(dev, "\tFeasibility study.\n");
		break;
	case DESIGNMATURITYLEVEL__R0:
		dev_dbg(dev, "\tFunctionality complete.\n");
		break;
	case DESIGNMATURITYLEVEL__R1:
		dev_dbg(dev, "\tVerification complete.\n");
		break;
	default:
		dev_dbg(dev, "\tUnknown.\n");
		break;
	}

	dev_dbg(dev, "%d) IP feature set:\n", ++id);
	switch (reg & IPFEATURESET_MASK) {
	case IPFEATURESET__E:
		dev_dbg(dev, "\tMinimal functionality (Eco).\n");
		break;
	case IPFEATURESET__L:
		dev_dbg(dev, "\tReduced functionality (Light).\n");
		break;
	case IPFEATURESET__P:
		dev_dbg(dev, "\tAdvanced functionality (Plus).\n");
		break;
	case IPFEATURESET__X:
		dev_dbg(dev, "\tExtensive functionality (eXtensive).\n");
		break;
	default:
		dev_dbg(dev, "\tUnknown.\n");
		break;
	}

	dev_dbg(dev, "%d) IP application:\n", ++id);
	switch (reg & IPAPPLICATION_MASK) {
	case IPAPPLICATION__B:
		dev_dbg(dev, "\tBlit engine only.\n");
		break;
	case IPAPPLICATION__D:
		dev_dbg(dev, "\tBlit engine and display controller.\n");
		break;
	case IPAPPLICATION__V:
		dev_dbg(dev, "\tDisplay controller only "
					"(with direct capture).\n");
		break;
	case IPAPPLICATION__G:
		dev_dbg(dev, "\tBlit engine, display controller "
					"(with direct capture),\n"
				   "\tcapture controller (buffered capture) "
				   "and drawing engine.\n");
		break;
	case IPAPPLICATION__C:
		dev_dbg(dev, "\tDisplay controller only.\n");
		break;
	default:
		dev_dbg(dev, "\tUnknown.\n");
		break;
	}

	dev_dbg(dev, "%d) IP configuration:\n", ++id);
	switch (reg & IPCONFIGURATION_MASK) {
	case IPCONFIGURATION__M:
		dev_dbg(dev, "\tGraphics core only (Module).\n");
		break;
	case IPCONFIGURATION__S:
		dev_dbg(dev, "\tSubsystem including a graphics core "
							"(System).\n");
		break;
	default:
		dev_dbg(dev, "\tUnknown.\n");
		break;
	}

	dev_dbg(dev, "%d) IP family:\n", ++id);
	switch (reg & IPFAMILY_MASK) {
	case IPFAMILY__IMXDPU2010:
		dev_dbg(dev, "\tBuilding block generation 2010.\n");
		break;
	case IPFAMILY__IMXDPU2012:
		dev_dbg(dev, "\tBuilding block generation 2012.\n");
		break;
	case IPFAMILY__IMXDPU2013:
		dev_dbg(dev, "\tBuilding block generation 2013.\n");
		break;
	default:
		dev_dbg(dev, "\tUnknown.\n");
		break;
	}
}

/* FIXME: initialize pixel link in a proper manner */
static void dpu_pixel_link_init(int id)
{
	sc_err_t sciErr;
	sc_ipc_t ipcHndl = 0;
	u32 mu_id;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_err("Cannot obtain MU ID\n");
		return;
	}

	sciErr = sc_ipc_open(&ipcHndl, mu_id);
	if (sciErr != SC_ERR_NONE) {
		pr_err("sc_ipc_open failed! (sciError = %d)\n", sciErr);
		return;
	}

	if (id == 0) {
		/* SC_C_KACHUNK_CNT is for blit */
		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_KACHUNK_CNT, 32);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_KACHUNK_CNT sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_ADDR, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_PXL_LINK_MST1_ADDR sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_ENB, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_PXL_LINK_MST1_ENB sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST1_VLD, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_PXL_LINK_MST1_VLD sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST2_ADDR, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_PXL_LINK_MST2_ADDR sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST2_ENB, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_PXL_LINK_MST2_ENB sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_PXL_LINK_MST2_VLD, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_PXL_LINK_MST2_VLD sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_SYNC_CTRL0, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_SYNC_CTRL0 sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_0, SC_C_SYNC_CTRL1, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_0:SC_C_SYNC_CTRL1 sc_misc_set_control failed! (sciError = %d)\n", sciErr);
	} else if (id == 1) {
		/* SC_C_KACHUNK_CNT is for blit */
		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_KACHUNK_CNT, 32);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_KACHUNK_CNT sc_misc_set_control failed! (sciError = %d)\n", sciErr);
		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_PXL_LINK_MST1_ADDR, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_PXL_LINK_MST1_ADDR sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_PXL_LINK_MST1_ENB, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_PXL_LINK_MST1_ENB sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_PXL_LINK_MST1_VLD, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_PXL_LINK_MST1_VLD sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_PXL_LINK_MST2_ADDR, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_PXL_LINK_MST2_ADDR sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_PXL_LINK_MST2_ENB, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_PXL_LINK_MST2_ENB sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_PXL_LINK_MST2_VLD, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_PXL_LINK_MST2_VLD sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_SYNC_CTRL0, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_SYNC_CTRL0 sc_misc_set_control failed! (sciError = %d)\n", sciErr);

		sciErr = sc_misc_set_control(ipcHndl, SC_R_DC_1, SC_C_SYNC_CTRL1, 0);
		if (sciErr != SC_ERR_NONE)
			pr_err("SC_R_DC_1:SC_C_SYNC_CTRL1 sc_misc_set_control failed! (sciError = %d)\n", sciErr);
	}

	sc_ipc_close(mu_id);
}

static int dpu_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(dpu_dt_ids, &pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct dpu_soc *dpu;
	struct resource *res;
	unsigned long dpu_base;
	const struct dpu_devtype *devtype;
	int ret;

	devtype = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dpu_base = res->start;

	dpu = devm_kzalloc(&pdev->dev, sizeof(*dpu), GFP_KERNEL);
	if (!dpu)
		return -ENODEV;

	dpu->dev = &pdev->dev;
	dpu->devtype = devtype;
	dpu->id = of_alias_get_id(np, "dpu");

	/* inner irqs */
	dpu->irq_cm = platform_get_irq(pdev, 0);
	dpu->irq_stream0a = platform_get_irq(pdev, 1);
	dpu->irq_stream1a = platform_get_irq(pdev, 3);
	dpu->irq_reserved0 = platform_get_irq(pdev, 5);
	dpu->irq_reserved1 = platform_get_irq(pdev, 6);
	dpu->irq_blit = platform_get_irq(pdev, 7);

	dev_dbg(dpu->dev, "irq_cm: %d\n", dpu->irq_cm);
	dev_dbg(dpu->dev, "irq_stream0a: %d, irq_stream1a: %d\n",
			dpu->irq_stream0a, dpu->irq_stream1a);
	dev_dbg(dpu->dev, "irq_reserved0: %d, irq_reserved1: %d\n",
			dpu->irq_reserved0, dpu->irq_reserved1);
	dev_dbg(dpu->dev, "irq_blit: %d\n", dpu->irq_blit);

	if (dpu->irq_cm < 0 ||
	    dpu->irq_stream0a < 0 || dpu->irq_stream1a < 0 ||
	    dpu->irq_reserved0 < 0 || dpu->irq_reserved1 < 0 ||
	    dpu->irq_blit < 0)
		return -ENODEV;

	dpu->intsteer_regmap = syscon_regmap_lookup_by_phandle(np, "intsteer");
	if (IS_ERR(dpu->intsteer_regmap)) {
		dev_err(dpu->dev, "failed to get intsteer regmap\n");
		return PTR_ERR(dpu->intsteer_regmap);
	}

	/* DPR irqs */
	if (dpu->devtype->has_prefetch) {
		dpu->irq_dpr0 = platform_get_irq(pdev, 8);
		dpu->irq_dpr1 = platform_get_irq(pdev, 9);

		dev_dbg(dpu->dev, "irq_dpr0: %d\n", dpu->irq_dpr0);
		dev_dbg(dpu->dev, "irq_dpr1: %d\n", dpu->irq_dpr1);

		if (dpu->irq_dpr0 < 0 || dpu->irq_dpr1 < 0)
			return -ENODEV;

		ret = devm_request_irq(dpu->dev, dpu->irq_dpr0,
				dpu_dpr0_irq_handler, 0, pdev->name, dpu);
		if (ret) {
			dev_err(dpu->dev, "request dpr0 interrupt failed\n");
			return ret;
		}

		ret = devm_request_irq(dpu->dev, dpu->irq_dpr1,
				dpu_dpr1_irq_handler, 0, pdev->name, dpu);
		if (ret) {
			dev_err(dpu->dev, "request dpr1 interrupt failed\n");
			return ret;
		}
	}

	spin_lock_init(&dpu->lock);
	spin_lock_init(&dpu->intsteer_lock);

	dev_dbg(dpu->dev, "Common: 0x%08lx\n", dpu_base + devtype->cm_ofs);
	DPU_UNITS_ADDR_DBG(cf);
	DPU_UNITS_ADDR_DBG(dec);
	DPU_UNITS_ADDR_DBG(ed);
	DPU_UNITS_ADDR_DBG(fd);
	DPU_UNITS_ADDR_DBG(fe);
	DPU_UNITS_ADDR_DBG(fg);
	DPU_UNITS_ADDR_DBG(fl);
	DPU_UNITS_ADDR_DBG(fw);
	DPU_UNITS_ADDR_DBG(hs);
	DPU_UNITS_ADDR_DBG(lb);
	DPU_UNITS_ADDR_DBG(st);
	DPU_UNITS_ADDR_DBG(tcon);
	DPU_UNITS_ADDR_DBG(vs);

	dpu->cm_reg = devm_ioremap(dpu->dev, dpu_base + devtype->cm_ofs, SZ_1K);
	if (!dpu->cm_reg)
		return -ENOMEM;

	platform_set_drvdata(pdev, dpu);

	ret = dpu_inner_irq_init(dpu);
	if (ret)
		goto failed_inner_irq;

	ret = dpu_submodules_init(dpu, pdev, dpu_base);
	if (ret)
		goto failed_submodules_init;

	ret = dpu_add_client_devices(dpu);
	if (ret) {
		dev_err(dpu->dev, "adding client devices failed with %d\n",
					ret);
		goto failed_add_clients;
	}

	dpu_debug_ip_identity(dpu);

	if (devtype->pixel_link_quirks)
		dpu_pixel_link_init(dpu->id);

	dev_info(dpu->dev, "driver probed\n");

	return 0;

failed_add_clients:
failed_submodules_init:
	dpu_inner_irq_exit(dpu);
failed_inner_irq:
	return ret;
}

static int dpu_remove(struct platform_device *pdev)
{
	struct dpu_soc *dpu = platform_get_drvdata(pdev);

	platform_device_unregister_children(pdev);
	dpu_inner_irq_exit(dpu);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dpu_suspend(struct device *dev)
{
	/*
	 * The dpu core driver currently depends on the client drivers
	 * to do suspend operations to leave dpu a cleaned up state
	 * machine status before the system enters sleep mode.
	 */
	return 0;
}

static int dpu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dpu_soc *dpu = platform_get_drvdata(pdev);

	dpu_inner_intsteer_enable_lines(dpu);

	if (dpu->devtype->pixel_link_quirks)
		dpu_pixel_link_init(dpu->id);

	_dpu_submodules_init(dpu, pdev);

	return 0;
}
#endif

static const struct dev_pm_ops dpu_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(dpu_suspend, dpu_resume)
};

static struct platform_driver dpu_driver = {
	.driver = {
		.pm = &dpu_pm_ops,
		.name = "dpu-core",
		.of_match_table = dpu_dt_ids,
	},
	.probe = dpu_probe,
	.remove = dpu_remove,
};

module_platform_driver(dpu_driver);

MODULE_DESCRIPTION("i.MX DPU driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
