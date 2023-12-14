// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP context registers/memory setting helpers
 *
 * Copyright 2023-2024 NXP
 * Author: Aymen Sghaier (aymen.sghaier@nxp.com)
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/nxp_neoisp.h>

#include "neoisp.h"
#include "neoisp_regs.h"
#include "neoisp_ctx.h"

/*
 * This is the initial set of parameters setup by driver upon a streamon ioctl for INPUT0 node.
 * It could be updated later by the driver depending on input/output formats setup by userspace
 * and also if fine tuned parameters are provided by a third party (IPA).
 */
struct neoisp_meta_params_s neoisp_default_params = {
	.features_cfg = {
		.hdr_decompress_input0_cfg = 1,
		.hdr_decompress_input1_cfg = 1,
		.obwb0_cfg = 1,
		.obwb1_cfg = 1,
		.obwb2_cfg = 1,
		.hdr_merge_cfg = 1,
		.rgbir_cfg = 1,
		.stat_cfg = 1,
		.ir_compress_cfg = 1,
		.bnr_cfg = 1,
		.vignetting_ctrl_cfg = 1,
		.ctemp_cfg = 1,
		.demosaic_cfg = 1,
		.rgb2yuv_cfg = 1,
		.dr_comp_cfg = 1,
		.nr_cfg = 1,
		.af_cfg = 1,
		.ee_cfg = 1,
		.df_cfg = 1,
		.convmed_cfg = 1,
		.cas_cfg = 1,
		.gcm_cfg = 1,
		.vignetting_table_cfg = 1,
		.drc_global_tonemap_cfg = 1,
		.drc_local_tonemap_cfg = 1,
	},
	.regs = {
	.decompress_input0 = { .ctrl_enable = 1,
		.knee_point1 = (1 << 16) - 1, /* default ibpp is 16 */
		.knee_ratio0 = 1 << 4,
		.knee_ratio4 = 1 << 4,
	},
	.decompress_input1 = { .ctrl_enable = 0 },
	.obwb[0] = {
		.ctrl_obpp = 3,
		.r_ctrl_gain = 1 << 8,
		.r_ctrl_offset = 0,
		.gr_ctrl_gain = 1 << 8,
		.gr_ctrl_offset = 0,
		.gb_ctrl_gain = 1 << 8,
		.gb_ctrl_offset = 0,
		.b_ctrl_gain = 1 << 8,
		.b_ctrl_offset = 0,
		},
	.obwb[1] = {
		.ctrl_obpp = 2,
		.r_ctrl_gain = 1 << 8,
		.r_ctrl_offset = 0,
		.gr_ctrl_gain = 1 << 8,
		.gr_ctrl_offset = 0,
		.gb_ctrl_gain = 1 << 8,
		.gb_ctrl_offset = 0,
		.b_ctrl_gain = 1 << 8,
		.b_ctrl_offset = 0,
		},
	.obwb[2] = {
		.ctrl_obpp = 3,
		.r_ctrl_gain = 1 << 8,
		.r_ctrl_offset = 0,
		.gr_ctrl_gain = 1 << 8,
		.gr_ctrl_offset = 0,
		.gb_ctrl_gain = 1 << 8,
		.gb_ctrl_offset = 0,
		.b_ctrl_gain = 1 << 8,
		.b_ctrl_offset = 0,
		},
	.hdr_merge = { .ctrl_enable = 0, },
	.rgbir = { .ctrl_enable = 0, },
	.stat = {},
	.ir_compress = { .ctrl_enable = 0, },
	.bnr = {
		.ctrl_enable = 1,
		.ctrl_debug = 0,
		.ctrl_obpp = 3,
		.ctrl_nhood = 0,
		.ypeak_peak_outsel = 0,
		.ypeak_peak_sel = 0,
		.ypeak_peak_low = 1 << 7,
		.ypeak_peak_high = 1 << 8,
		.yedge_th0_edge_th0 = 20,
		.yedge_scale_scale = 1 << 10,
		.yedge_scale_shift = 10,
		.yedges_th0_edge_th0 = 20,
		.yedges_scale_scale = 1 << 10,
		.yedges_scale_shift = 10,
		.yedgea_th0_edge_th0 = 20,
		.yedgea_scale_scale = 1 << 10,
		.yedgea_scale_shift = 10,
		.yluma_x_th0_th = 20,
		.yluma_y_th_luma_y_th0 = 10,
		.yluma_y_th_luma_y_th1 = 1 << 8,
		.yluma_scale_scale = 1 << 10,
		.yluma_scale_shift = 10,
		.yalpha_gain_gain = 1 << 8,
		.yalpha_gain_offset = 0,
		.cpeak_peak_outsel = 0,
		.cpeak_peak_sel = 0,
		.cpeak_peak_low = 1 << 7,
		.cpeak_peak_high = 1 << 8,
		.cedge_th0_edge_th0 = 10,
		.cedge_scale_scale = 1 << 10,
		.cedge_scale_shift = 10,
		.cedges_th0_edge_th0 = 20,
		.cedges_scale_scale = 1 << 10,
		.cedges_scale_shift = 10,
		.cedgea_th0_edge_th0 = 20,
		.cedgea_scale_scale = 1 << 10,
		.cedgea_scale_shift = 10,
		.cluma_x_th0_th = 20,
		.cluma_y_th_luma_y_th0 = 10,
		.cluma_y_th_luma_y_th1 = 0,
		.cluma_scale_scale = 1 << 10,
		.cluma_scale_shift = 10,
		.calpha_gain_gain = 1 << 8,
		.calpha_gain_offset = 0,
		.stretch_gain = 32 << 8,
	},
	.vignetting_ctrl = { .ctrl_enable = 0, },
	.ctemp  = { .ctrl_enable = 0, },
	.demosaic = {
		.ctrl_fmt = 0,
		.activity_ctl_alpha = 1 << 8,
		.activity_ctl_act_ratio = 1 << 8,
		.dynamics_ctl0_strengthg = 1 << 8,
		.dynamics_ctl0_strengthc = 1 << 8,
		.dynamics_ctl2_max_impact = 1 << 7,
	},
	.rgb2yuv = {
		.gain_ctrl_rgain = 1 << 8,
		.gain_ctrl_bgain = 1 << 8,
		.mat_rxcy = {
			{76, 148,  29},
			{-36, -73, 111},
			{157, -130, -26},
			},
		.csc_offsets = {0, 0, 0},
		},
	.drc = {
		.gbl_gain_gain = 1 << 8,
		.lcl_stretch_stretch = 1 << 8,
		.alpha_alpha = 1 << 8,
		},
	.nrc = { .ctrl_enable = 0, },
	.afc = {{}}, /* bypass */
	.eec = { .ctrl_enable = 0, },
	.dfc = { .ctrl_enable = 0, },
	.convf = { .ctrl_flt = 0, }, /* bypassed */
	.cas = {
		.gain_shift = 0,
		.gain_scale = 1,
		.corr_corr = 0,
		.offset_offset = 0,
		},
	.gcm = {
		.imat_rxcy = {
			{256, 0, 292},
			{256, -101, -149},
			{256, 520, 0},
			},
		.ioffsets = {0, 0, 0},
		.omat_rxcy = {
			{256, 0, 0},
			{0, 256, 0},
			{0, 0, 256},
			},
		.ooffsets = {0, 0, 0},
		.gamma0_gamma0 = 106, /* 1/2.4 x 256 */
		.gamma0_offset0 = 0,
		.gamma1_gamma1 = 106, /* 1/2.4 x 256 */
		.gamma1_offset1 = 0,
		.gamma2_gamma2 = 106, /* 1/2.4 x 256 */
		.gamma2_offset2 = 0,
		.mat_confg_sign_confg = 1,
		},
	},
};

static inline void ctx_blk_write(uint32_t field, __u32 *ptr, __u32 *dest)
{
	__u32 woffset, wcount;

	woffset = ISP_GET_OFF(field);
	wcount = ISP_GET_WSZ(field);
	if (IS_ERR_OR_NULL(ptr) || IS_ERR_OR_NULL(dest)) {
		pr_err("Invalid pointer for memcpy block !");
		return;
	}
	memcpy(&dest[woffset], ptr, wcount * sizeof(__u32));
}

static void neoisp_set_hdr_decompress0(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_CTRL_CAM0_IDX],
			NEO_CTRL_CAM0_ENABLE_SET(p->decompress_input0.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_POINT1_CAM0_IDX],
			p->decompress_input0.knee_point1);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_POINT2_CAM0_IDX],
			p->decompress_input0.knee_point2);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_POINT3_CAM0_IDX],
			p->decompress_input0.knee_point3);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_POINT4_CAM0_IDX],
			p->decompress_input0.knee_point4);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_OFFSET0_CAM0_IDX],
			p->decompress_input0.knee_offset0);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_OFFSET1_CAM0_IDX],
			p->decompress_input0.knee_offset1);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_OFFSET2_CAM0_IDX],
			p->decompress_input0.knee_offset2);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_OFFSET3_CAM0_IDX],
			p->decompress_input0.knee_offset3);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_OFFSET4_CAM0_IDX],
			p->decompress_input0.knee_offset4);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_IDX],
			NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_RATIO0_SET(p->decompress_input0.knee_ratio0)
			| NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_RATIO1_SET(p->decompress_input0.knee_ratio1));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_IDX],
			NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_RATIO2_SET(p->decompress_input0.knee_ratio2)
			| NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_RATIO3_SET(p->decompress_input0.knee_ratio3));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_RATIO4_CAM0_IDX],
			p->decompress_input0.knee_ratio4);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0_IDX],
			p->decompress_input0.knee_npoint0);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_NPOINT1_CAM0_IDX],
			p->decompress_input0.knee_npoint1);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_NPOINT2_CAM0_IDX],
			p->decompress_input0.knee_npoint2);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_NPOINT3_CAM0_IDX],
			p->decompress_input0.knee_npoint3);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS0_KNEE_NPOINT4_CAM0_IDX],
			p->decompress_input0.knee_npoint4);
}

static void neoisp_set_hdr_decompress1(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_CTRL_CAM0_IDX],
			NEO_CTRL_CAM0_ENABLE_SET(p->decompress_input1.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_POINT1_CAM0_IDX],
			p->decompress_input1.knee_point1);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_POINT2_CAM0_IDX],
			p->decompress_input1.knee_point2);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_POINT3_CAM0_IDX],
			p->decompress_input1.knee_point3);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_POINT4_CAM0_IDX],
			p->decompress_input1.knee_point4);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_OFFSET0_CAM0_IDX],
			p->decompress_input1.knee_offset0);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_OFFSET1_CAM0_IDX],
			p->decompress_input1.knee_offset1);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_OFFSET2_CAM0_IDX],
			p->decompress_input1.knee_offset2);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_OFFSET3_CAM0_IDX],
			p->decompress_input1.knee_offset3);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_OFFSET4_CAM0_IDX],
			p->decompress_input1.knee_offset4);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_RATIO01_CAM0_IDX],
			NEO_HDR_DECOMPRESS1_KNEE_RATIO01_CAM0_RATIO0_SET(p->decompress_input1.knee_ratio0)
			| NEO_HDR_DECOMPRESS1_KNEE_RATIO01_CAM0_RATIO1_SET(p->decompress_input1.knee_ratio1));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_RATIO23_CAM0_IDX],
			NEO_HDR_DECOMPRESS1_KNEE_RATIO23_CAM0_RATIO2_SET(p->decompress_input1.knee_ratio2)
			| NEO_HDR_DECOMPRESS1_KNEE_RATIO23_CAM0_RATIO3_SET(p->decompress_input1.knee_ratio3));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_RATIO4_CAM0_IDX],
			p->decompress_input1.knee_ratio4);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_NPOINT0_CAM0_IDX],
			p->decompress_input1.knee_npoint0);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_NPOINT1_CAM0_IDX],
			p->decompress_input1.knee_npoint1);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_NPOINT2_CAM0_IDX],
			p->decompress_input1.knee_npoint2);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_NPOINT3_CAM0_IDX],
			p->decompress_input1.knee_npoint3);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_DECOMPRESS1_KNEE_NPOINT4_CAM0_IDX],
			p->decompress_input1.knee_npoint4);
}

static void neoisp_set_ob_wb0(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB0_CTRL_CAM0_IDX],
			NEO_OB_WB0_CTRL_CAM0_OBPP_SET(p->obwb[0].ctrl_obpp));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB0_R_CTRL_CAM0_IDX],
			NEO_OB_WB0_R_CTRL_CAM0_OFFSET_SET(p->obwb[0].r_ctrl_offset)
			| NEO_OB_WB0_R_CTRL_CAM0_GAIN_SET(p->obwb[0].r_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB0_GR_CTRL_CAM0_IDX],
			NEO_OB_WB0_GR_CTRL_CAM0_OFFSET_SET(p->obwb[0].gr_ctrl_offset)
			| NEO_OB_WB0_GR_CTRL_CAM0_GAIN_SET(p->obwb[0].gr_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB0_GB_CTRL_CAM0_IDX],
			NEO_OB_WB0_GB_CTRL_CAM0_OFFSET_SET(p->obwb[0].gb_ctrl_offset)
			| NEO_OB_WB0_GB_CTRL_CAM0_GAIN_SET(p->obwb[0].gb_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB0_B_CTRL_CAM0_IDX],
			NEO_OB_WB0_B_CTRL_CAM0_OFFSET_SET(p->obwb[0].b_ctrl_offset)
			| NEO_OB_WB0_B_CTRL_CAM0_GAIN_SET(p->obwb[0].b_ctrl_gain));
}

static void neoisp_set_ob_wb1(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB1_CTRL_CAM0_IDX],
			NEO_OB_WB1_CTRL_CAM0_OBPP_SET(p->obwb[1].ctrl_obpp));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB1_R_CTRL_CAM0_IDX],
			NEO_OB_WB1_R_CTRL_CAM0_OFFSET_SET(p->obwb[1].r_ctrl_offset)
			| NEO_OB_WB1_R_CTRL_CAM0_GAIN_SET(p->obwb[1].r_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB1_GR_CTRL_CAM0_IDX],
			NEO_OB_WB1_GR_CTRL_CAM0_OFFSET_SET(p->obwb[1].gr_ctrl_offset)
			| NEO_OB_WB1_GR_CTRL_CAM0_GAIN_SET(p->obwb[1].gr_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB1_GB_CTRL_CAM0_IDX],
			NEO_OB_WB1_GB_CTRL_CAM0_OFFSET_SET(p->obwb[1].gb_ctrl_offset)
			| NEO_OB_WB1_GB_CTRL_CAM0_GAIN_SET(p->obwb[1].gb_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB1_B_CTRL_CAM0_IDX],
			NEO_OB_WB1_B_CTRL_CAM0_OFFSET_SET(p->obwb[1].b_ctrl_offset)
			| NEO_OB_WB1_B_CTRL_CAM0_GAIN_SET(p->obwb[1].b_ctrl_gain));
}

static void neoisp_set_ob_wb2(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB2_CTRL_CAM0_IDX],
			NEO_OB_WB2_CTRL_CAM0_OBPP_SET(p->obwb[2].ctrl_obpp));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB2_R_CTRL_CAM0_IDX],
			NEO_OB_WB2_R_CTRL_CAM0_OFFSET_SET(p->obwb[2].r_ctrl_offset)
			| NEO_OB_WB2_R_CTRL_CAM0_GAIN_SET(p->obwb[2].r_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB2_GR_CTRL_CAM0_IDX],
			NEO_OB_WB2_GR_CTRL_CAM0_OFFSET_SET(p->obwb[2].gr_ctrl_offset)
			| NEO_OB_WB2_GR_CTRL_CAM0_GAIN_SET(p->obwb[2].gr_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB2_GB_CTRL_CAM0_IDX],
			NEO_OB_WB2_GB_CTRL_CAM0_OFFSET_SET(p->obwb[2].gb_ctrl_offset)
			| NEO_OB_WB2_GB_CTRL_CAM0_GAIN_SET(p->obwb[2].gb_ctrl_gain));
	regmap_field_write(neoispd->regs.fields[NEO_OB_WB2_B_CTRL_CAM0_IDX],
			NEO_OB_WB2_B_CTRL_CAM0_OFFSET_SET(p->obwb[2].b_ctrl_offset)
			| NEO_OB_WB2_B_CTRL_CAM0_GAIN_SET(p->obwb[2].b_ctrl_gain));
}

static void neoisp_set_hdr_merge(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_CTRL_CAM0_IDX],
			NEO_HDR_MERGE_CTRL_CAM0_ENABLE_SET(p->hdr_merge.ctrl_enable)
			| NEO_HDR_MERGE_CTRL_CAM0_MOTION_FIX_EN_SET(p->hdr_merge.ctrl_motion_fix_en)
			| NEO_HDR_MERGE_CTRL_CAM0_BLEND_3X3_SET(p->hdr_merge.ctrl_blend_3x3)
			| NEO_HDR_MERGE_CTRL_CAM0_GAIN1BPP_SET(p->hdr_merge.ctrl_gain1bpp)
			| NEO_HDR_MERGE_CTRL_CAM0_GAIN0BPP_SET(p->hdr_merge.ctrl_gain0bpp)
			| NEO_HDR_MERGE_CTRL_CAM0_OBPP_SET(p->hdr_merge.ctrl_obpp));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_GAIN_OFFSET_CAM0_IDX],
			NEO_HDR_MERGE_GAIN_OFFSET_CAM0_OFFSET1_SET(p->hdr_merge.gain_offset_offset1)
			| NEO_HDR_MERGE_GAIN_OFFSET_CAM0_OFFSET0_SET(p->hdr_merge.gain_offset_offset0));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_GAIN_SCALE_CAM0_IDX],
			NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE1_SET(p->hdr_merge.gain_scale_scale1)
			| NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE0_SET(p->hdr_merge.gain_scale_scale0));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_GAIN_SHIFT_CAM0_IDX],
			NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT1_SET(p->hdr_merge.gain_shift_shift1)
			| NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT0_SET(p->hdr_merge.gain_shift_shift0));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_LUMA_TH_CAM0_IDX], p->hdr_merge.luma_th_th0);
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_LUMA_SCALE_CAM0_IDX],
			NEO_HDR_MERGE_LUMA_SCALE_CAM0_SCALE_SET(p->hdr_merge.luma_scale_scale)
			| NEO_HDR_MERGE_LUMA_SCALE_CAM0_SHIFT_SET(p->hdr_merge.luma_scale_shift)
			| NEO_HDR_MERGE_LUMA_SCALE_CAM0_THSHIFT_SET(p->hdr_merge.luma_scale_thshift));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_DOWNSCALE_CAM0_IDX],
			NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE1_SET(p->hdr_merge.downscale_imgscale1)
			| NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE0_SET(p->hdr_merge.downscale_imgscale0));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_UPSCALE_CAM0_IDX],
			NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE1_SET(p->hdr_merge.upscale_imgscale1)
			| NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE0_SET(p->hdr_merge.upscale_imgscale0));
	regmap_field_write(neoispd->regs.fields[NEO_HDR_MERGE_POST_SCALE_CAM0_IDX], p->hdr_merge.post_scale_scale);
}

static void neoisp_set_rgbir(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CTRL_CAM0_IDX],
			NEO_RGBIR_CTRL_CAM0_ENABLE_SET(p->rgbir.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CCM0_CAM0_IDX], p->rgbir.ccm0_ccm);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CCM1_CAM0_IDX], p->rgbir.ccm1_ccm);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CCM2_CAM0_IDX], p->rgbir.ccm2_ccm);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CCM0_TH_CAM0_IDX],
			p->rgbir.ccm0_th_threshold);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CCM1_TH_CAM0_IDX],
			p->rgbir.ccm1_th_threshold);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_CCM2_TH_CAM0_IDX],
			p->rgbir.ccm2_th_threshold);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_ROI0_POS_CAM0_IDX],
			NEO_RGBIR_ROI0_POS_CAM0_XPOS_SET(p->rgbir.roi[0].xpos)
			| NEO_RGBIR_ROI0_POS_CAM0_YPOS_SET(p->rgbir.roi[0].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_ROI0_SIZE_CAM0_IDX],
			NEO_RGBIR_ROI0_SIZE_CAM0_WIDTH_SET(p->rgbir.roi[0].width)
			| NEO_RGBIR_ROI0_SIZE_CAM0_HEIGHT_SET(p->rgbir.roi[0].height));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_ROI1_POS_CAM0_IDX],
			NEO_RGBIR_ROI1_POS_CAM0_XPOS_SET(p->rgbir.roi[1].xpos)
			| NEO_RGBIR_ROI1_POS_CAM0_YPOS_SET(p->rgbir.roi[1].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_ROI1_SIZE_CAM0_IDX],
			NEO_RGBIR_ROI1_SIZE_CAM0_WIDTH_SET(p->rgbir.roi[1].width)
			| NEO_RGBIR_ROI1_SIZE_CAM0_HEIGHT_SET(p->rgbir.roi[1].height));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_HIST0_CTRL_CAM0_IDX],
			NEO_HIST_CTRL_CAM0_OFFSET_SET(p->rgbir.hists[0].hist_ctrl_offset)
			| NEO_HIST_CTRL_CAM0_CHANNEL_SET(p->rgbir.hists[0].hist_ctrl_channel)
			| NEO_HIST_CTRL_CAM0_PATTERN_SET(p->rgbir.hists[0].hist_ctrl_pattern)
			| NEO_HIST_CTRL_CAM0_DIR_INPUT1_DIF_SET(p->rgbir.hists[0].hist_ctrl_dir_input1_dif)
			| NEO_HIST_CTRL_CAM0_LIN_INPUT1_LOG_SET(p->rgbir.hists[0].hist_ctrl_lin_input1_log));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_HIST0_SCALE_CAM0_IDX],
			p->rgbir.hists[0].hist_scale_scale);
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_HIST1_CTRL_CAM0_IDX],
			NEO_HIST_CTRL_CAM0_OFFSET_SET(p->rgbir.hists[1].hist_ctrl_offset)
			| NEO_HIST_CTRL_CAM0_CHANNEL_SET(p->rgbir.hists[1].hist_ctrl_channel)
			| NEO_HIST_CTRL_CAM0_PATTERN_SET(p->rgbir.hists[1].hist_ctrl_pattern)
			| NEO_HIST_CTRL_CAM0_DIR_INPUT1_DIF_SET(p->rgbir.hists[1].hist_ctrl_dir_input1_dif)
			| NEO_HIST_CTRL_CAM0_LIN_INPUT1_LOG_SET(p->rgbir.hists[1].hist_ctrl_lin_input1_log));
	regmap_field_write(neoispd->regs.fields[NEO_RGBIR_HIST1_SCALE_CAM0_IDX],
			p->rgbir.hists[1].hist_scale_scale);
}

static void neoisp_set_stat_hists(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_STAT_ROI0_POS_CAM0_IDX],
			NEO_STAT_ROI0_POS_CAM0_XPOS_SET(p->stat.roi0.xpos)
			| NEO_STAT_ROI0_POS_CAM0_YPOS_SET(p->stat.roi0.ypos));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_ROI0_SIZE_CAM0_IDX],
			NEO_STAT_ROI0_SIZE_CAM0_WIDTH_SET(p->stat.roi0.width)
			| NEO_STAT_ROI0_SIZE_CAM0_HEIGHT_SET(p->stat.roi0.height));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_ROI1_POS_CAM0_IDX],
			NEO_STAT_ROI0_POS_CAM0_XPOS_SET(p->stat.roi1.xpos)
			| NEO_STAT_ROI0_POS_CAM0_YPOS_SET(p->stat.roi1.ypos));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_ROI1_SIZE_CAM0_IDX],
			NEO_STAT_ROI0_SIZE_CAM0_WIDTH_SET(p->stat.roi1.width)
			| NEO_STAT_ROI0_SIZE_CAM0_HEIGHT_SET(p->stat.roi1.height));

	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST0_CTRL_CAM0_IDX],
			NEO_HIST_CTRL_CAM0_OFFSET_SET(p->stat.hists[0].hist_ctrl_offset)
			| NEO_HIST_CTRL_CAM0_CHANNEL_SET(p->stat.hists[0].hist_ctrl_channel)
			| NEO_HIST_CTRL_CAM0_PATTERN_SET(p->stat.hists[0].hist_ctrl_pattern)
			| NEO_HIST_CTRL_CAM0_DIR_INPUT1_DIF_SET(p->stat.hists[0].hist_ctrl_dir_input1_dif)
			| NEO_HIST_CTRL_CAM0_LIN_INPUT1_LOG_SET(p->stat.hists[0].hist_ctrl_lin_input1_log));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST0_SCALE_CAM0_IDX],
			p->stat.hists[0].hist_scale_scale);
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST1_CTRL_CAM0_IDX],
			NEO_HIST_CTRL_CAM0_OFFSET_SET(p->stat.hists[1].hist_ctrl_offset)
			| NEO_HIST_CTRL_CAM0_CHANNEL_SET(p->stat.hists[1].hist_ctrl_channel)
			| NEO_HIST_CTRL_CAM0_PATTERN_SET(p->stat.hists[1].hist_ctrl_pattern)
			| NEO_HIST_CTRL_CAM0_DIR_INPUT1_DIF_SET(p->stat.hists[1].hist_ctrl_dir_input1_dif)
			| NEO_HIST_CTRL_CAM0_LIN_INPUT1_LOG_SET(p->stat.hists[1].hist_ctrl_lin_input1_log));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST1_SCALE_CAM0_IDX],
			p->stat.hists[1].hist_scale_scale);
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST2_CTRL_CAM0_IDX],
			NEO_HIST_CTRL_CAM0_OFFSET_SET(p->stat.hists[2].hist_ctrl_offset)
			| NEO_HIST_CTRL_CAM0_CHANNEL_SET(p->stat.hists[2].hist_ctrl_channel)
			| NEO_HIST_CTRL_CAM0_PATTERN_SET(p->stat.hists[2].hist_ctrl_pattern)
			| NEO_HIST_CTRL_CAM0_DIR_INPUT1_DIF_SET(p->stat.hists[2].hist_ctrl_dir_input1_dif)
			| NEO_HIST_CTRL_CAM0_LIN_INPUT1_LOG_SET(p->stat.hists[2].hist_ctrl_lin_input1_log));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST2_SCALE_CAM0_IDX],
			p->stat.hists[2].hist_scale_scale);
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST3_CTRL_CAM0_IDX],
			NEO_HIST_CTRL_CAM0_OFFSET_SET(p->stat.hists[3].hist_ctrl_offset)
			| NEO_HIST_CTRL_CAM0_CHANNEL_SET(p->stat.hists[3].hist_ctrl_channel)
			| NEO_HIST_CTRL_CAM0_PATTERN_SET(p->stat.hists[3].hist_ctrl_pattern)
			| NEO_HIST_CTRL_CAM0_DIR_INPUT1_DIF_SET(p->stat.hists[3].hist_ctrl_dir_input1_dif)
			| NEO_HIST_CTRL_CAM0_LIN_INPUT1_LOG_SET(p->stat.hists[3].hist_ctrl_lin_input1_log));
	regmap_field_write(neoispd->regs.fields[NEO_STAT_HIST3_SCALE_CAM0_IDX],
			p->stat.hists[3].hist_scale_scale);
}

static void neoisp_set_ir_compress(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_CTRL_CAM0_IDX],
			NEO_IR_COMPRESS_CTRL_CAM0_ENABLE_SET(p->ir_compress.ctrl_enable)
			| NEO_IR_COMPRESS_CTRL_CAM0_OBPP_SET(p->ir_compress.ctrl_obpp));
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_POINT1_CAM0_IDX],
			p->ir_compress.knee_point1_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_POINT2_CAM0_IDX],
			p->ir_compress.knee_point2_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_POINT3_CAM0_IDX],
			p->ir_compress.knee_point3_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_POINT4_CAM0_IDX],
			p->ir_compress.knee_point4_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_OFFSET0_CAM0_IDX],
			p->ir_compress.knee_offset0_offset);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_OFFSET1_CAM0_IDX],
			p->ir_compress.knee_offset1_offset);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_OFFSET2_CAM0_IDX],
			p->ir_compress.knee_offset2_offset);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_OFFSET3_CAM0_IDX],
			p->ir_compress.knee_offset3_offset);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_OFFSET4_CAM0_IDX],
			p->ir_compress.knee_offset4_offset);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_RATIO01_CAM0_IDX],
			NEO_IR_COMPRESS_KNEE_RATIO01_CAM0_RATIO0_SET(p->ir_compress.knee_ratio01_ratio0)
			| NEO_IR_COMPRESS_KNEE_RATIO01_CAM0_RATIO1_SET(p->ir_compress.knee_ratio01_ratio1));
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_RATIO23_CAM0_IDX],
			NEO_IR_COMPRESS_KNEE_RATIO23_CAM0_RATIO2_SET(p->ir_compress.knee_ratio23_ratio2)
			| NEO_IR_COMPRESS_KNEE_RATIO23_CAM0_RATIO3_SET(p->ir_compress.knee_ratio23_ratio3));
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_RATIO4_CAM0_IDX],
			p->ir_compress.knee_ratio4_ratio4);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_NPOINT0_CAM0_IDX],
			p->ir_compress.knee_npoint0_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_NPOINT1_CAM0_IDX],
			p->ir_compress.knee_npoint1_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_NPOINT2_CAM0_IDX],
			p->ir_compress.knee_npoint2_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_NPOINT3_CAM0_IDX],
			p->ir_compress.knee_npoint3_kneepoint);
	regmap_field_write(neoispd->regs.fields[NEO_IR_COMPRESS_KNEE_NPOINT4_CAM0_IDX],
			p->ir_compress.knee_npoint4_kneepoint);
}

static void neoisp_set_color_temp(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CTRL_CAM0_IDX],
			NEO_COLOR_TEMP_CTRL_CAM0_IBPP_SET(p->ctemp.ctrl_ibpp)
			| NEO_COLOR_TEMP_CTRL_CAM0_CSCON_SET(p->ctemp.ctrl_cscon)
			| NEO_COLOR_TEMP_CTRL_CAM0_ENABLE_SET(p->ctemp.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_ROI_POS_CAM0_IDX],
			NEO_COLOR_TEMP_ROI_POS_CAM0_XPOS_SET(p->ctemp.roi.xpos)
			| NEO_COLOR_TEMP_ROI_POS_CAM0_YPOS_SET(p->ctemp.roi.ypos));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_ROI_SIZE_CAM0_IDX],
			NEO_COLOR_TEMP_ROI_SIZE_CAM0_WIDTH_SET(p->ctemp.roi.width)
			| NEO_COLOR_TEMP_ROI_SIZE_CAM0_HEIGHT_SET(p->ctemp.roi.height));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_REDGAIN_CAM0_IDX],
			NEO_COLOR_TEMP_REDGAIN_CAM0_MIN_SET(p->ctemp.redgain_min)
			| NEO_COLOR_TEMP_REDGAIN_CAM0_MAX_SET(p->ctemp.redgain_max));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_BLUEGAIN_CAM0_IDX],
			NEO_COLOR_TEMP_BLUEGAIN_CAM0_MIN_SET(p->ctemp.bluegain_min)
			| NEO_COLOR_TEMP_BLUEGAIN_CAM0_MAX_SET(p->ctemp.bluegain_max));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_POINT1_CAM0_IDX],
			NEO_COLOR_TEMP_POINT1_CAM0_BLUE_SET(p->ctemp.point1_blue)
			| NEO_COLOR_TEMP_POINT1_CAM0_RED_SET(p->ctemp.point1_red));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_POINT2_CAM0_IDX],
			NEO_COLOR_TEMP_POINT2_CAM0_BLUE_SET(p->ctemp.point2_blue)
			| NEO_COLOR_TEMP_POINT2_CAM0_RED_SET(p->ctemp.point2_red));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_HOFFSET_CAM0_IDX],
			NEO_COLOR_TEMP_HOFFSET_CAM0_RIGHT_SET(p->ctemp.hoffset_right)
			| NEO_COLOR_TEMP_HOFFSET_CAM0_LEFT_SET(p->ctemp.hoffset_left));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_VOFFSET_CAM0_IDX],
			NEO_COLOR_TEMP_VOFFSET_CAM0_UP_SET(p->ctemp.voffset_up)
			| NEO_COLOR_TEMP_VOFFSET_CAM0_DOWN_SET(p->ctemp.voffset_down));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_IDX],
			NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_SLOPE_L_SET(p->ctemp.point1_slope_slope_l)
			| NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_SLOPE_R_SET(p->ctemp.point1_slope_slope_r));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_IDX],
			NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_SLOPE_L_SET(p->ctemp.point2_slope_slope_l)
			| NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_SLOPE_R_SET(p->ctemp.point2_slope_slope_r));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_LUMA_TH_CAM0_IDX],
			NEO_COLOR_TEMP_LUMA_TH_CAM0_THL_SET(p->ctemp.luma_th_thl)
			| NEO_COLOR_TEMP_LUMA_TH_CAM0_THH_SET(p->ctemp.luma_th_thh));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CSC_MAT0_CAM0_IDX],
			NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C0_SET(p->ctemp.csc_matrix[0][0])
			| NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C1_SET(p->ctemp.csc_matrix[0][1]));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CSC_MAT1_CAM0_IDX],
			NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C0_SET(p->ctemp.csc_matrix[0][2])
			| NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C1_SET(p->ctemp.csc_matrix[1][0]));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CSC_MAT2_CAM0_IDX],
			NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C0_SET(p->ctemp.csc_matrix[1][1])
			| NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C1_SET(p->ctemp.csc_matrix[1][2]));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CSC_MAT3_CAM0_IDX],
			NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C0_SET(p->ctemp.csc_matrix[2][0])
			| NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C1_SET(p->ctemp.csc_matrix[2][1]));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CSC_MAT4_CAM0_IDX], p->ctemp.csc_matrix[2][2]);
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_R_GR_OFFSET_CAM0_IDX],
			NEO_COLOR_TEMP_R_GR_OFFSET_CAM0_OFFSET0_SET(p->ctemp.offsets[0])
			| NEO_COLOR_TEMP_R_GR_OFFSET_CAM0_OFFSET1_SET(p->ctemp.offsets[1]));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_GB_B_OFFSET_CAM0_IDX],
			NEO_COLOR_TEMP_GB_B_OFFSET_CAM0_OFFSET0_SET(p->ctemp.offsets[2])
			| NEO_COLOR_TEMP_GB_B_OFFSET_CAM0_OFFSET1_SET(p->ctemp.offsets[3]));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_STAT_BLK_SIZE0_IDX],
			NEO_COLOR_TEMP_STAT_BLK_SIZE0_XSIZE_SET(p->ctemp.stat_blk_size0_xsize)
			| NEO_COLOR_TEMP_STAT_BLK_SIZE0_YSIZE_SET(p->ctemp.stat_blk_size0_ysize));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI0_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[0].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[0].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[0].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[0].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI1_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[1].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[1].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[1].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[1].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI2_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[2].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[2].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[2].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[2].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI3_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[3].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[3].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[3].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[3].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI4_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[4].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[4].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[4].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[4].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI5_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[5].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[5].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[5].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[5].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI6_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[6].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[6].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[6].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[6].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI7_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[7].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[7].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[7].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[7].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI8_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[8].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[8].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[8].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[8].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_CROI9_POS_CAM0_IDX],
			NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW_SET(p->ctemp.color_rois[9].pos_roverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH_SET(p->ctemp.color_rois[9].pos_roverg_high)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW_SET(p->ctemp.color_rois[9].pos_boverg_low)
			| NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH_SET(p->ctemp.color_rois[9].pos_boverg_high));
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_GR_AVG_IN_CAM0_IDX], p->ctemp.gr_avg_in_gr_agv);
	regmap_field_write(neoispd->regs.fields[NEO_COLOR_TEMP_GB_AVG_IN_CAM0_IDX], p->ctemp.gb_avg_in_gb_agv);
}

static void neoisp_set_bnr(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CTRL_CAM0_IDX],
			NEO_BNR_CTRL_CAM0_OBPP_SET(p->bnr.ctrl_obpp)
			| NEO_BNR_CTRL_CAM0_DEBUG_SET(p->bnr.ctrl_debug)
			| NEO_BNR_CTRL_CAM0_NHOOD_SET(p->bnr.ctrl_nhood)
			| NEO_BNR_CTRL_CAM0_ENABLE_SET(p->bnr.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YPEAK_CAM0_IDX],
			NEO_BNR_YPEAK_CAM0_PEAK_LOW_SET(p->bnr.ypeak_peak_low)
			| NEO_BNR_YPEAK_CAM0_PEAK_SEL_SET(p->bnr.ypeak_peak_sel)
			| NEO_BNR_YPEAK_CAM0_PEAK_HIGH_SET(p->bnr.ypeak_peak_high)
			| NEO_BNR_YPEAK_CAM0_PEAK_OUTSEL_SET(p->bnr.ypeak_peak_outsel));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YEDGE_TH0_CAM0_IDX], p->bnr.yedge_th0_edge_th0);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YEDGE_SCALE_CAM0_IDX],
			NEO_BNR_YEDGE_SCALE_CAM0_SCALE_SET(p->bnr.yedge_scale_scale)
			| NEO_BNR_YEDGE_SCALE_CAM0_SHIFT_SET(p->bnr.yedge_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YEDGES_TH0_CAM0_IDX], p->bnr.yedges_th0_edge_th0);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YEDGES_SCALE_CAM0_IDX],
			NEO_BNR_YEDGES_SCALE_CAM0_SCALE_SET(p->bnr.yedges_scale_scale)
			| NEO_BNR_YEDGES_SCALE_CAM0_SHIFT_SET(p->bnr.yedges_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YEDGEA_TH0_CAM0_IDX], p->bnr.yedgea_th0_edge_th0);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YEDGEA_SCALE_CAM0_IDX],
			NEO_BNR_YEDGEA_SCALE_CAM0_SCALE_SET(p->bnr.yedgea_scale_scale)
			| NEO_BNR_YEDGEA_SCALE_CAM0_SHIFT_SET(p->bnr.yedgea_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YLUMA_X_TH0_CAM0_IDX], p->bnr.yluma_x_th0_th);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YLUMA_Y_TH_CAM0_IDX],
			NEO_BNR_YLUMA_Y_TH_CAM0_LUMA_Y_TH0_SET(p->bnr.yluma_y_th_luma_y_th0)
			| NEO_BNR_YLUMA_Y_TH_CAM0_LUMA_Y_TH1_SET(p->bnr.yluma_y_th_luma_y_th1));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YLUMA_SCALE_CAM0_IDX],
			NEO_BNR_YLUMA_SCALE_CAM0_SCALE_SET(p->bnr.yluma_scale_scale)
			| NEO_BNR_YLUMA_SCALE_CAM0_SHIFT_SET(p->bnr.yluma_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_YALPHA_GAIN_CAM0_IDX],
			NEO_BNR_YALPHA_GAIN_CAM0_GAIN_SET(p->bnr.yalpha_gain_gain)
			| NEO_BNR_YALPHA_GAIN_CAM0_OFFSET_SET(p->bnr.yalpha_gain_offset));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CPEAK_CAM0_IDX],
			NEO_BNR_CPEAK_CAM0_PEAK_LOW_SET(p->bnr.cpeak_peak_low)
			| NEO_BNR_CPEAK_CAM0_PEAK_SEL_SET(p->bnr.cpeak_peak_sel)
			| NEO_BNR_CPEAK_CAM0_PEAK_HIGH_SET(p->bnr.cpeak_peak_high)
			| NEO_BNR_CPEAK_CAM0_PEAK_OUTSEL_SET(p->bnr.cpeak_peak_outsel));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CEDGE_TH0_CAM0_IDX], p->bnr.cedge_th0_edge_th0);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CEDGE_SCALE_CAM0_IDX],
			NEO_BNR_CEDGE_SCALE_CAM0_SCALE_SET(p->bnr.cedge_scale_scale)
			| NEO_BNR_CEDGE_SCALE_CAM0_SHIFT_SET(p->bnr.cedge_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CEDGES_TH0_CAM0_IDX], p->bnr.cedges_th0_edge_th0);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CEDGES_SCALE_CAM0_IDX],
			NEO_BNR_CEDGES_SCALE_CAM0_SCALE_SET(p->bnr.cedges_scale_scale)
			| NEO_BNR_CEDGES_SCALE_CAM0_SHIFT_SET(p->bnr.cedges_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CEDGEA_TH0_CAM0_IDX], p->bnr.cedgea_th0_edge_th0);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CEDGEA_SCALE_CAM0_IDX],
			NEO_BNR_CEDGEA_SCALE_CAM0_SCALE_SET(p->bnr.cedgea_scale_scale)
			| NEO_BNR_CEDGEA_SCALE_CAM0_SHIFT_SET(p->bnr.cedgea_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CLUMA_X_TH0_CAM0_IDX], p->bnr.cluma_x_th0_th);
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CLUMA_Y_TH_CAM0_IDX],
			NEO_BNR_CLUMA_Y_TH_CAM0_LUMA_Y_TH0_SET(p->bnr.cluma_y_th_luma_y_th0)
			| NEO_BNR_CLUMA_Y_TH_CAM0_LUMA_Y_TH1_SET(p->bnr.cluma_y_th_luma_y_th1));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CLUMA_SCALE_CAM0_IDX],
			NEO_BNR_CLUMA_SCALE_CAM0_SCALE_SET(p->bnr.cluma_scale_scale)
			| NEO_BNR_CLUMA_SCALE_CAM0_SHIFT_SET(p->bnr.cluma_scale_shift));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_CALPHA_GAIN_CAM0_IDX],
			NEO_BNR_CALPHA_GAIN_CAM0_GAIN_SET(p->bnr.calpha_gain_gain)
			| NEO_BNR_CALPHA_GAIN_CAM0_OFFSET_SET(p->bnr.calpha_gain_offset));
	regmap_field_write(neoispd->regs.fields[NEO_BNR_STRETCH_CAM0_IDX], p->bnr.stretch_gain);
}

static void neoisp_set_vignetting(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_VIGNETTING_CTRL_CAM0_IDX],
			NEO_VIGNETTING_CTRL_CAM0_ENABLE_SET(p->vignetting_ctrl.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_VIGNETTING_BLK_CONF_CAM0_IDX],
			NEO_VIGNETTING_BLK_CONF_CAM0_COLS_SET(p->vignetting_ctrl.blk_conf_cols)
			| NEO_VIGNETTING_BLK_CONF_CAM0_ROWS_SET(p->vignetting_ctrl.blk_conf_rows));
	regmap_field_write(neoispd->regs.fields[NEO_VIGNETTING_BLK_SIZE_CAM0_IDX],
			NEO_VIGNETTING_BLK_SIZE_CAM0_XSIZE_SET(p->vignetting_ctrl.blk_size_xsize)
			| NEO_VIGNETTING_BLK_SIZE_CAM0_YSIZE_SET(p->vignetting_ctrl.blk_size_ysize));
	regmap_field_write(neoispd->regs.fields[NEO_VIGNETTING_BLK_STEPY_CAM0_IDX],
			NEO_VIGNETTING_BLK_STEPY_CAM0_STEP_SET(p->vignetting_ctrl.blk_stepy_step));
	regmap_field_write(neoispd->regs.fields[NEO_VIGNETTING_BLK_STEPX_CAM0_IDX],
			NEO_VIGNETTING_BLK_STEPX_CAM0_STEP_SET(p->vignetting_ctrl.blk_stepx_step));
}

static void neoisp_set_demosaic(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_DEMOSAIC_CTRL_CAM0_IDX],
			NEO_DEMOSAIC_CTRL_CAM0_FMT_SET(p->demosaic.ctrl_fmt));
	regmap_field_write(neoispd->regs.fields[NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_IDX],
			NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_ALPHA_SET(p->demosaic.activity_ctl_alpha)
			| NEO_DEMOSAIC_ACTIVITY_CTL_CAM0_ACT_RATIO_SET(p->demosaic.activity_ctl_act_ratio));
	regmap_field_write(neoispd->regs.fields[NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_IDX],
			NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHG_SET(p->demosaic.dynamics_ctl0_strengthg)
			| NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHC_SET(p->demosaic.dynamics_ctl0_strengthc));
	regmap_field_write(neoispd->regs.fields[NEO_DEMOSAIC_DYNAMICS_CTL2_CAM0_IDX],
			NEO_DEMOSAIC_DYNAMICS_CTL0_CAM0_STRENGTHC_SET(p->demosaic.dynamics_ctl2_max_impact));
}

static void neoisp_set_rgb_to_yuv(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_IDX],
			NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_RGAIN_SET(p->rgb2yuv.gain_ctrl_rgain)
			| NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_BGAIN_SET(p->rgb2yuv.gain_ctrl_bgain));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_MAT0_CAM0_IDX],
			NEO_RGB_TO_YUV_MAT0_CAM0_R0C0_SET(p->rgb2yuv.mat_rxcy[0][0])
			| NEO_RGB_TO_YUV_MAT0_CAM0_R0C1_SET(p->rgb2yuv.mat_rxcy[0][1]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_MAT1_CAM0_IDX],
			NEO_RGB_TO_YUV_MAT1_CAM0_R0C2_SET(p->rgb2yuv.mat_rxcy[0][2]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_MAT2_CAM0_IDX],
			NEO_RGB_TO_YUV_MAT2_CAM0_R1C0_SET(p->rgb2yuv.mat_rxcy[1][0])
			| NEO_RGB_TO_YUV_MAT2_CAM0_R1C1_SET(p->rgb2yuv.mat_rxcy[1][1]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_MAT3_CAM0_IDX],
			NEO_RGB_TO_YUV_MAT3_CAM0_R1C2_SET(p->rgb2yuv.mat_rxcy[1][2]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_MAT4_CAM0_IDX],
			NEO_RGB_TO_YUV_MAT4_CAM0_R2C0_SET(p->rgb2yuv.mat_rxcy[2][0])
			| NEO_RGB_TO_YUV_MAT4_CAM0_R2C1_SET(p->rgb2yuv.mat_rxcy[2][1]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_MAT5_CAM0_IDX],
			NEO_RGB_TO_YUV_MAT5_CAM0_R2C2_SET(p->rgb2yuv.mat_rxcy[2][2]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_OFFSET0_CAM0_IDX],
			NEO_RGB_TO_YUV_OFFSET0_CAM0_OFFSET_SET(p->rgb2yuv.csc_offsets[0]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_OFFSET1_CAM0_IDX],
			NEO_RGB_TO_YUV_OFFSET1_CAM0_OFFSET_SET(p->rgb2yuv.csc_offsets[1]));
	regmap_field_write(neoispd->regs.fields[NEO_RGB_TO_YUV_OFFSET2_CAM0_IDX],
			NEO_RGB_TO_YUV_OFFSET2_CAM0_OFFSET_SET(p->rgb2yuv.csc_offsets[2]));
}


static void neoisp_set_drc(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_DRC_ROI0_POS_CAM0_IDX],
			NEO_DRC_ROI0_POS_CAM0_XPOS_SET(p->drc.roi0.xpos)
			| NEO_DRC_ROI0_POS_CAM0_YPOS_SET(p->drc.roi0.ypos));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_ROI0_SIZE_CAM0_IDX],
			NEO_DRC_ROI0_SIZE_CAM0_WIDTH_SET(p->drc.roi0.width)
			| NEO_DRC_ROI0_SIZE_CAM0_HEIGHT_SET(p->drc.roi0.height));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_ROI1_POS_CAM0_IDX],
			NEO_DRC_ROI1_POS_CAM0_XPOS_SET(p->drc.roi1.xpos)
			| NEO_DRC_ROI1_POS_CAM0_YPOS_SET(p->drc.roi1.ypos));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_ROI1_SIZE_CAM0_IDX],
			NEO_DRC_ROI1_SIZE_CAM0_WIDTH_SET(p->drc.roi1.width)
			| NEO_DRC_ROI1_SIZE_CAM0_HEIGHT_SET(p->drc.roi1.height));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_GROI_SUM_SHIFT_CAM0_IDX],
			NEO_DRC_GROI_SUM_SHIFT_CAM0_SHIFT0_SET(p->drc.groi_sum_shift_shift0)
			| NEO_DRC_GROI_SUM_SHIFT_CAM0_SHIFT1_SET(p->drc.groi_sum_shift_shift1));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_GBL_GAIN_CAM0_IDX],
			NEO_DRC_GBL_GAIN_CAM0_GAIN_SET(p->drc.gbl_gain_gain));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_LCL_BLK_SIZE_CAM0_IDX],
			NEO_DRC_LCL_BLK_SIZE_CAM0_XSIZE_SET(p->drc.lcl_blk_size_xsize)
			| NEO_DRC_LCL_BLK_SIZE_CAM0_YSIZE_SET(p->drc.lcl_blk_size_ysize));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_LCL_STRETCH_CAM0_IDX],
			NEO_DRC_LCL_STRETCH_CAM0_STRETCH_SET(p->drc.lcl_stretch_stretch)
			| NEO_DRC_LCL_STRETCH_CAM0_OFFSET_SET(p->drc.lcl_stretch_offset));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_LCL_BLK_STEPY_CAM0_IDX],
			NEO_DRC_LCL_BLK_STEPY_CAM0_STEP_SET(p->drc.lcl_blk_stepy_step));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_LCL_BLK_STEPX_CAM0_IDX],
			NEO_DRC_LCL_BLK_STEPX_CAM0_STEP_SET(p->drc.lcl_blk_stepx_step));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_LCL_SUM_SHIFT_CAM0_IDX],
			NEO_DRC_LCL_SUM_SHIFT_CAM0_SHIFT_SET(p->drc.lcl_sum_shift_shift));
	regmap_field_write(neoispd->regs.fields[NEO_DRC_ALPHA_CAM0_IDX],
			NEO_DRC_ALPHA_CAM0_ALPHA_SET(p->drc.alpha_alpha));
}

static void neoisp_set_nr(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_NR_CTRL_CAM0_IDX],
			NEO_NR_CTRL_CAM0_DEBUG_SET(p->nrc.ctrl_debug)
			| NEO_NR_CTRL_CAM0_ENABLE_SET(p->nrc.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_NR_BLEND_SCALE_CAM0_IDX],
			NEO_NR_BLEND_SCALE_CAM0_SCALE_SET(p->nrc.blend_scale_scale)
			| NEO_NR_BLEND_SCALE_CAM0_SHIFT_SET(p->nrc.blend_scale_shift)
			| NEO_NR_BLEND_SCALE_CAM0_GAIN_SET(p->nrc.blend_scale_gain));
	regmap_field_write(neoispd->regs.fields[NEO_NR_BLEND_TH0_CAM0_IDX],
			NEO_NR_BLEND_TH0_CAM0_TH_SET(p->nrc.blend_th0_th));
}

static void neoisp_set_df(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_DF_CTRL_CAM0_IDX],
			NEO_DF_CTRL_CAM0_DEBUG_SET(p->dfc.ctrl_debug)
			| NEO_DF_CTRL_CAM0_ENABLE_SET(p->dfc.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_DF_TH_SCALE_CAM0_IDX],
			NEO_DF_TH_SCALE_CAM0_SCALE_SET(p->dfc.th_scale_scale));
	regmap_field_write(neoispd->regs.fields[NEO_DF_BLEND_SHIFT_CAM0_IDX],
			NEO_DF_BLEND_SHIFT_CAM0_SHIFT_SET(p->dfc.blend_shift_shift));
	regmap_field_write(neoispd->regs.fields[NEO_DF_BLEND_TH0_CAM0_IDX],
			NEO_DF_BLEND_TH0_CAM0_TH_SET(p->dfc.blend_th0_th));
}

static void neoisp_set_ee(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_EE_CTRL_CAM0_IDX],
			NEO_EE_CTRL_CAM0_DEBUG_SET(p->eec.ctrl_debug)
			| NEO_EE_CTRL_CAM0_ENABLE_SET(p->eec.ctrl_enable));
	regmap_field_write(neoispd->regs.fields[NEO_EE_CORING_CAM0_IDX],
			NEO_EE_CORING_CAM0_CORING_SET(p->eec.coring_coring));
	regmap_field_write(neoispd->regs.fields[NEO_EE_CLIP_CAM0_IDX],
			NEO_EE_CLIP_CAM0_CLIP_SET(p->eec.clip_clip));
	regmap_field_write(neoispd->regs.fields[NEO_EE_MASKGAIN_CAM0_IDX],
			NEO_EE_MASKGAIN_CAM0_GAIN_SET(p->eec.maskgain_gain));
}

static void neoisp_set_convmed(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_CCONVMED_CTRL_CAM0_IDX],
			NEO_CCONVMED_CTRL_CAM0_FLT_SET(p->convf.ctrl_flt));
}

static void neoisp_set_cas(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_CAS_GAIN_CAM0_IDX],
			NEO_CAS_GAIN_CAM0_SCALE_SET(p->cas.gain_scale)
			| NEO_CAS_GAIN_CAM0_SHIFT_SET(p->cas.gain_shift));
	regmap_field_write(neoispd->regs.fields[NEO_CAS_CORR_CAM0_IDX],
			NEO_CAS_CORR_CAM0_CORR_SET(p->cas.corr_corr));
	regmap_field_write(neoispd->regs.fields[NEO_CAS_OFFSET_CAM0_IDX],
			NEO_CAS_OFFSET_CAM0_OFFSET_SET(p->cas.offset_offset));
}

void neoisp_set_gcm(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IMAT0_CAM0_IDX],
			NEO_GCM_IMAT0_CAM0_R0C0_SET(p->gcm.imat_rxcy[0][0])
			| NEO_GCM_IMAT0_CAM0_R0C1_SET(p->gcm.imat_rxcy[0][1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IMAT1_CAM0_IDX],
			NEO_GCM_IMAT1_CAM0_R0C2_SET(p->gcm.imat_rxcy[0][2]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IMAT2_CAM0_IDX],
			NEO_GCM_IMAT2_CAM0_R1C0_SET(p->gcm.imat_rxcy[1][0])
			| NEO_GCM_IMAT2_CAM0_R1C1_SET(p->gcm.imat_rxcy[1][1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IMAT3_CAM0_IDX],
			NEO_GCM_IMAT3_CAM0_R1C2_SET(p->gcm.imat_rxcy[1][2]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IMAT4_CAM0_IDX],
			NEO_GCM_IMAT4_CAM0_R2C0_SET(p->gcm.imat_rxcy[2][0])
			| NEO_GCM_IMAT4_CAM0_R2C1_SET(p->gcm.imat_rxcy[2][1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IMAT5_CAM0_IDX],
			NEO_GCM_IMAT5_CAM0_R2C2_SET(p->gcm.imat_rxcy[2][2]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IOFFSET0_CAM0_IDX],
			NEO_GCM_IOFFSET0_CAM0_OFFSET0_SET(p->gcm.ioffsets[0]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IOFFSET1_CAM0_IDX],
			NEO_GCM_IOFFSET1_CAM0_OFFSET1_SET(p->gcm.ioffsets[1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_IOFFSET2_CAM0_IDX],
			NEO_GCM_IOFFSET2_CAM0_OFFSET2_SET(p->gcm.ioffsets[2]));

	regmap_field_write(neoispd->regs.fields[NEO_GCM_OMAT0_CAM0_IDX],
			NEO_GCM_OMAT0_CAM0_R0C0_SET(p->gcm.omat_rxcy[0][0])
			| NEO_GCM_OMAT0_CAM0_R0C1_SET(p->gcm.omat_rxcy[0][1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OMAT1_CAM0_IDX],
			NEO_GCM_OMAT1_CAM0_R0C2_SET(p->gcm.omat_rxcy[0][2]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OMAT2_CAM0_IDX],
			NEO_GCM_OMAT2_CAM0_R1C0_SET(p->gcm.omat_rxcy[1][0])
			| NEO_GCM_OMAT2_CAM0_R1C1_SET(p->gcm.omat_rxcy[1][1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OMAT3_CAM0_IDX],
			NEO_GCM_OMAT3_CAM0_R1C2_SET(p->gcm.omat_rxcy[1][2]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OMAT4_CAM0_IDX],
			NEO_GCM_OMAT4_CAM0_R2C0_SET(p->gcm.omat_rxcy[2][0])
			| NEO_GCM_OMAT4_CAM0_R2C1_SET(p->gcm.omat_rxcy[2][1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OMAT5_CAM0_IDX],
			NEO_GCM_OMAT5_CAM0_R2C2_SET(p->gcm.omat_rxcy[2][2]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OOFFSET0_CAM0_IDX],
			NEO_GCM_OOFFSET0_CAM0_OFFSET0_SET(p->gcm.ooffsets[0]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OOFFSET1_CAM0_IDX],
			NEO_GCM_OOFFSET1_CAM0_OFFSET1_SET(p->gcm.ooffsets[1]));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_OOFFSET2_CAM0_IDX],
			NEO_GCM_OOFFSET2_CAM0_OFFSET2_SET(p->gcm.ooffsets[2]));

	regmap_field_write(neoispd->regs.fields[NEO_GCM_GAMMA0_CAM0_IDX],
			NEO_GCM_GAMMA0_CAM0_GAMMA0_SET(p->gcm.gamma0_gamma0)
			| NEO_GCM_GAMMA0_CAM0_OFFSET0_SET(p->gcm.gamma0_offset0));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_GAMMA1_CAM0_IDX],
			NEO_GCM_GAMMA1_CAM0_GAMMA1_SET(p->gcm.gamma1_gamma1)
			| NEO_GCM_GAMMA1_CAM0_OFFSET1_SET(p->gcm.gamma1_offset1));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_GAMMA2_CAM0_IDX],
			NEO_GCM_GAMMA2_CAM0_GAMMA2_SET(p->gcm.gamma2_gamma2)
			| NEO_GCM_GAMMA2_CAM0_OFFSET2_SET(p->gcm.gamma2_offset2));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_BLKLVL0_CTRL_CAM0_IDX],
			NEO_GCM_BLKLVL0_CTRL_CAM0_OFFSET0_SET(p->gcm.blklvl0_ctrl_offset0)
			| NEO_GCM_BLKLVL0_CTRL_CAM0_GAIN0_SET(p->gcm.blklvl0_ctrl_gain0));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_BLKLVL1_CTRL_CAM0_IDX],
			NEO_GCM_BLKLVL1_CTRL_CAM0_OFFSET1_SET(p->gcm.blklvl1_ctrl_offset1)
			| NEO_GCM_BLKLVL1_CTRL_CAM0_GAIN1_SET(p->gcm.blklvl1_ctrl_gain1));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_BLKLVL2_CTRL_CAM0_IDX],
			NEO_GCM_BLKLVL2_CTRL_CAM0_OFFSET2_SET(p->gcm.blklvl2_ctrl_offset2)
			| NEO_GCM_BLKLVL2_CTRL_CAM0_GAIN2_SET(p->gcm.blklvl2_ctrl_gain2));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_LOWTH_CTRL01_CAM0_IDX],
			NEO_GCM_LOWTH_CTRL01_CAM0_THRESHOLD0_SET(p->gcm.lowth_ctrl01_threshold0)
			| NEO_GCM_LOWTH_CTRL01_CAM0_THRESHOLD1_SET(p->gcm.lowth_ctrl01_threshold1));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_LOWTH_CTRL2_CAM0_IDX],
			NEO_GCM_LOWTH_CTRL2_CAM0_THRESHOLD2_SET(p->gcm.lowth_ctrl2_threshold2));
	regmap_field_write(neoispd->regs.fields[NEO_GCM_MAT_CONFG_CAM0_IDX],
			NEO_GCM_MAT_CONFG_CAM0_SIGN_CONFG_SET(p->gcm.mat_confg_sign_confg));
}

static void neoisp_set_autofocus(struct neoisp_reg_params_s *p, struct neoisp_dev_s *neoispd)
{
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI0_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI0_POS_CAM0_XPOS_SET(p->afc.af_roi[0].xpos)
			| NEO_AUTOFOCUS_ROI0_POS_CAM0_YPOS_SET(p->afc.af_roi[0].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI0_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI0_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[0].width)
			| NEO_AUTOFOCUS_ROI0_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[0].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI1_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI1_POS_CAM0_XPOS_SET(p->afc.af_roi[1].xpos)
			| NEO_AUTOFOCUS_ROI1_POS_CAM0_YPOS_SET(p->afc.af_roi[1].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI1_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI1_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[1].width)
			| NEO_AUTOFOCUS_ROI1_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[1].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI2_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI2_POS_CAM0_XPOS_SET(p->afc.af_roi[2].xpos)
			| NEO_AUTOFOCUS_ROI2_POS_CAM0_YPOS_SET(p->afc.af_roi[2].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI2_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI2_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[2].width)
			| NEO_AUTOFOCUS_ROI2_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[2].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI3_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI3_POS_CAM0_XPOS_SET(p->afc.af_roi[3].xpos)
			| NEO_AUTOFOCUS_ROI3_POS_CAM0_YPOS_SET(p->afc.af_roi[3].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI3_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI3_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[3].width)
			| NEO_AUTOFOCUS_ROI3_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[3].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI4_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI4_POS_CAM0_XPOS_SET(p->afc.af_roi[4].xpos)
			| NEO_AUTOFOCUS_ROI4_POS_CAM0_YPOS_SET(p->afc.af_roi[4].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI4_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI4_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[4].width)
			| NEO_AUTOFOCUS_ROI4_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[4].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI5_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI5_POS_CAM0_XPOS_SET(p->afc.af_roi[5].xpos)
			| NEO_AUTOFOCUS_ROI5_POS_CAM0_YPOS_SET(p->afc.af_roi[5].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI5_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI5_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[5].width)
			| NEO_AUTOFOCUS_ROI5_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[5].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI6_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI6_POS_CAM0_XPOS_SET(p->afc.af_roi[6].xpos)
			| NEO_AUTOFOCUS_ROI6_POS_CAM0_YPOS_SET(p->afc.af_roi[6].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI6_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI6_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[6].width)
			| NEO_AUTOFOCUS_ROI6_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[6].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI7_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI7_POS_CAM0_XPOS_SET(p->afc.af_roi[7].xpos)
			| NEO_AUTOFOCUS_ROI7_POS_CAM0_YPOS_SET(p->afc.af_roi[7].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI7_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI7_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[7].width)
			| NEO_AUTOFOCUS_ROI7_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[7].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI8_POS_CAM0_IDX],
			NEO_AUTOFOCUS_ROI8_POS_CAM0_XPOS_SET(p->afc.af_roi[8].xpos)
			| NEO_AUTOFOCUS_ROI8_POS_CAM0_YPOS_SET(p->afc.af_roi[8].ypos));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_ROI8_SIZE_CAM0_IDX],
			NEO_AUTOFOCUS_ROI8_SIZE_CAM0_WIDTH_SET(p->afc.af_roi[8].width)
			| NEO_AUTOFOCUS_ROI8_SIZE_CAM0_HEIGHT_SET(p->afc.af_roi[8].height));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_IDX],
			NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF0_SET(p->afc.fil0_coeffs[0])
			| NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF1_SET(p->afc.fil0_coeffs[1])
			| NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF2_SET(p->afc.fil0_coeffs[2])
			| NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF3_SET(p->afc.fil0_coeffs[3]));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_IDX],
			NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF4_SET(p->afc.fil0_coeffs[4])
			| NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF5_SET(p->afc.fil0_coeffs[5])
			| NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF6_SET(p->afc.fil0_coeffs[6])
			| NEO_AUTOFOCUS_FIL0_COEFFS1_CAM0_COEFF7_SET(p->afc.fil0_coeffs[7]));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL0_COEFFS2_CAM0_IDX],
			NEO_AUTOFOCUS_FIL0_COEFFS2_CAM0_COEFF8_SET(p->afc.fil0_coeffs[8]));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_IDX],
			NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF0_SET(p->afc.fil1_coeffs[0])
			| NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF1_SET(p->afc.fil1_coeffs[1])
			| NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF2_SET(p->afc.fil1_coeffs[2])
			| NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF3_SET(p->afc.fil1_coeffs[3]));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_IDX],
			NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF4_SET(p->afc.fil1_coeffs[4])
			| NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF5_SET(p->afc.fil1_coeffs[5])
			| NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF6_SET(p->afc.fil1_coeffs[6])
			| NEO_AUTOFOCUS_FIL1_COEFFS1_CAM0_COEFF7_SET(p->afc.fil1_coeffs[7]));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL1_COEFFS2_CAM0_IDX],
			NEO_AUTOFOCUS_FIL1_COEFFS2_CAM0_COEFF8_SET(p->afc.fil1_coeffs[8]));
	regmap_field_write(neoispd->regs.fields[NEO_AUTOFOCUS_FIL1_SHIFT_CAM0_IDX],
			NEO_AUTOFOCUS_FIL1_SHIFT_CAM0_SHIFT_SET(p->afc.fil1_shift_shift));
}

static void neoisp_set_mem_vignetting_table(struct neoisp_mem_params_s *p, __u32 *dest)
{
	ctx_blk_write(NEO_VIGNETTING_TABLE_MAP, (__u32 *)p->vt.vignetting_table, dest);
}

static void neoisp_set_mem_global_tonemap(struct neoisp_mem_params_s *p, __u32 *dest)
{
	ctx_blk_write(NEO_DRC_GLOBAL_TONEMAP_MAP, (__u32 *)p->gtm.drc_global_tonemap, dest);
}

static void neoisp_set_mem_local_tonemap(struct neoisp_mem_params_s *p, __u32 *dest)
{
	ctx_blk_write(NEO_DRC_LOCAL_TONEMAP_MAP, (__u32 *)p->ltm.drc_local_tonemap, dest);
}

int neoisp_set_params(struct neoisp_dev_s *neoispd, struct neoisp_meta_params_s *p, bool force)
{
	__u32 *mem = (__u32 *)neoispd->mmio_tcm;

	/* update selected blocks wrt feature config flag */
	if (force || p->features_cfg.hdr_decompress_input0_cfg)
		neoisp_set_hdr_decompress0(&p->regs, neoispd);
	if (force || p->features_cfg.hdr_decompress_input1_cfg)
		neoisp_set_hdr_decompress1(&p->regs, neoispd);
	if (force || p->features_cfg.obwb0_cfg)
		neoisp_set_ob_wb0(&p->regs, neoispd);
	if (force || p->features_cfg.obwb1_cfg)
		neoisp_set_ob_wb1(&p->regs, neoispd);
	if (force || p->features_cfg.obwb2_cfg)
		neoisp_set_ob_wb2(&p->regs, neoispd);
	if (force || p->features_cfg.hdr_merge_cfg)
		neoisp_set_hdr_merge(&p->regs, neoispd);
	if (force || p->features_cfg.rgbir_cfg)
		neoisp_set_rgbir(&p->regs, neoispd);
	if (force || p->features_cfg.stat_cfg)
		neoisp_set_stat_hists(&p->regs, neoispd);
	if (force || p->features_cfg.ir_compress_cfg)
		neoisp_set_ir_compress(&p->regs, neoispd);
	if (force || p->features_cfg.bnr_cfg)
		neoisp_set_bnr(&p->regs, neoispd);
	if (force || p->features_cfg.vignetting_ctrl_cfg)
		neoisp_set_vignetting(&p->regs, neoispd);
	if (force || p->features_cfg.ctemp_cfg)
		neoisp_set_color_temp(&p->regs, neoispd);
	if (force || p->features_cfg.demosaic_cfg)
		neoisp_set_demosaic(&p->regs, neoispd);
	if (force || p->features_cfg.rgb2yuv_cfg)
		neoisp_set_rgb_to_yuv(&p->regs, neoispd);
	if (force || p->features_cfg.dr_comp_cfg)
		neoisp_set_drc(&p->regs, neoispd);
	if (force || p->features_cfg.nr_cfg)
		neoisp_set_nr(&p->regs, neoispd);
	if (force || p->features_cfg.af_cfg)
		neoisp_set_autofocus(&p->regs, neoispd);
	if (force || p->features_cfg.ee_cfg)
		neoisp_set_ee(&p->regs, neoispd);
	if (force || p->features_cfg.df_cfg)
		neoisp_set_df(&p->regs, neoispd);
	if (force || p->features_cfg.convmed_cfg)
		neoisp_set_convmed(&p->regs, neoispd);
	if (force || p->features_cfg.cas_cfg)
		neoisp_set_cas(&p->regs, neoispd);
	if (force || p->features_cfg.gcm_cfg)
		neoisp_set_gcm(&p->regs, neoispd);
	if (force || p->features_cfg.vignetting_table_cfg)
		neoisp_set_mem_vignetting_table(&p->mems, mem);
	if (force || p->features_cfg.drc_global_tonemap_cfg)
		neoisp_set_mem_global_tonemap(&p->mems, mem);
	if (force || p->features_cfg.drc_local_tonemap_cfg)
		neoisp_set_mem_local_tonemap(&p->mems, mem);

	return 0;
}

/*
 * neoisp_program_ctx is used to write all parameters to registers and memory
 */
int neoisp_program_ctx(struct neoisp_dev_s *neoispd, __u32 ctx_id)
{
	struct neoisp_meta_params_s *params = &neoispd->node_group[ctx_id].params[VB2_MAX_FRAME];

	return neoisp_set_params(neoispd, params, true);
}

/*
 * neoisp_update_ctx is used to update parameters to a saved context with ctx_id index
 */
int neoisp_update_ctx(struct neoisp_dev_s *neoispd, __u32 ctx_id, struct neoisp_meta_params_s *new)
{
	struct neoisp_meta_params_s *par = &neoispd->node_group[ctx_id].params[VB2_MAX_FRAME];

	/* update selected blocks wrt feature config flag */
	if (new->features_cfg.hdr_decompress_input0_cfg)
		memcpy(&par->regs.decompress_input0, &new->regs.decompress_input0,
				sizeof(new->regs.decompress_input0));
	if (new->features_cfg.hdr_decompress_input1_cfg)
		memcpy(&par->regs.decompress_input1, &new->regs.decompress_input1,
				sizeof(new->regs.decompress_input1));
	if (new->features_cfg.obwb0_cfg)
		memcpy(&par->regs.obwb[0], &new->regs.obwb[0], sizeof(new->regs.obwb[0]));
	if (new->features_cfg.obwb1_cfg)
		memcpy(&par->regs.obwb[1], &new->regs.obwb[1], sizeof(new->regs.obwb[1]));
	if (new->features_cfg.obwb2_cfg)
		memcpy(&par->regs.obwb[2], &new->regs.obwb[2], sizeof(new->regs.obwb[2]));
	if (new->features_cfg.hdr_merge_cfg)
		memcpy(&par->regs.hdr_merge, &new->regs.hdr_merge, sizeof(new->regs.hdr_merge));
	if (new->features_cfg.rgbir_cfg)
		memcpy(&par->regs.rgbir, &new->regs.rgbir, sizeof(new->regs.rgbir));
	if (new->features_cfg.stat_cfg)
		memcpy(&par->regs.stat, &new->regs.stat, sizeof(new->regs.stat));
	if (new->features_cfg.ir_compress_cfg)
		memcpy(&par->regs.ir_compress, &new->regs.ir_compress,
				sizeof(new->regs.ir_compress));
	if (new->features_cfg.bnr_cfg)
		memcpy(&par->regs.bnr, &new->regs.bnr, sizeof(new->regs.bnr));
	if (new->features_cfg.vignetting_ctrl_cfg)
		memcpy(&par->regs.vignetting_ctrl, &new->regs.vignetting_ctrl,
				sizeof(new->regs.vignetting_ctrl));
	if (new->features_cfg.ctemp_cfg)
		memcpy(&par->regs.ctemp, &new->regs.ctemp, sizeof(new->regs.ctemp));
	if (new->features_cfg.demosaic_cfg)
		memcpy(&par->regs.demosaic, &new->regs.demosaic, sizeof(new->regs.demosaic));
	if (new->features_cfg.rgb2yuv_cfg)
		memcpy(&par->regs.rgb2yuv, &new->regs.rgb2yuv, sizeof(new->regs.rgb2yuv));
	if (new->features_cfg.dr_comp_cfg)
		memcpy(&par->regs.drc, &new->regs.drc, sizeof(new->regs.drc));
	if (new->features_cfg.nr_cfg)
		memcpy(&par->regs.nrc, &new->regs.nrc, sizeof(new->regs.nrc));
	if (new->features_cfg.af_cfg)
		memcpy(&par->regs.afc, &new->regs.afc, sizeof(new->regs.afc));
	if (new->features_cfg.ee_cfg)
		memcpy(&par->regs.eec, &new->regs.eec, sizeof(new->regs.eec));
	if (new->features_cfg.df_cfg)
		memcpy(&par->regs.dfc, &new->regs.dfc, sizeof(new->regs.dfc));
	if (new->features_cfg.convmed_cfg)
		memcpy(&par->regs.convf, &new->regs.convf, sizeof(new->regs.convf));
	if (new->features_cfg.cas_cfg)
		memcpy(&par->regs.cas, &new->regs.cas, sizeof(new->regs.cas));
	if (new->features_cfg.gcm_cfg)
		memcpy(&par->regs.gcm, &new->regs.gcm, sizeof(new->regs.gcm));
	if (new->features_cfg.vignetting_table_cfg)
		memcpy(&par->mems.vt, &new->mems.vt, sizeof(new->mems.vt));
	if (new->features_cfg.drc_global_tonemap_cfg)
		memcpy(&par->mems.gtm, &new->mems.gtm, sizeof(new->mems.gtm));
	if (new->features_cfg.drc_local_tonemap_cfg)
		memcpy(&par->mems.ltm, &new->mems.ltm, sizeof(new->mems.ltm));

	return 0;
}
