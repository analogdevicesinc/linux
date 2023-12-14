/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * NXP NEOISP userspace API
 * Reference i.MX95 Applications Processor Reference Manual
 *
 * Copyright 2023-2024 NXP
 */

#ifndef UAPI_NXP_NEOISP_H
#define UAPI_NXP_NEOISP_H

#include <linux/types.h>

/**
 * enum neoisp_version - NXP NEO ISP variants
 *
 * @NEO_ISP_V1: first version used in imx95
 */
enum neoisp_version_e {
	NEO_ISP_V1 = 1,
	NEO_ISP_VFUTURE,
};

/**
 * struct neoisp_feat_ctrl_s - ISP features control flags
 *
 *  one bit per feature
 *   - 1: update feature
 *   - 0: do not update feature
 */
struct neoisp_feat_ctrl_s {
	__u32 hdr_decompress_input0_cfg : 1;
	__u32 hdr_decompress_input1_cfg : 1;
	__u32 obwb0_cfg : 1;
	__u32 obwb1_cfg : 1;
	__u32 obwb2_cfg : 1;
	__u32 hdr_merge_cfg : 1;
	__u32 rgbir_cfg : 1;
	__u32 stat_cfg : 1;
	__u32 ir_compress_cfg : 1;
	__u32 bnr_cfg : 1;
	__u32 vignetting_ctrl_cfg : 1;
	__u32 ctemp_cfg : 1;
	__u32 demosaic_cfg : 1;
	__u32 rgb2yuv_cfg : 1;
	__u32 dr_comp_cfg : 1;
	__u32 nr_cfg : 1;
	__u32 af_cfg : 1;
	__u32 ee_cfg : 1;
	__u32 df_cfg : 1;
	__u32 convmed_cfg : 1;
	__u32 cas_cfg : 1;
	__u32 gcm_cfg : 1;
	__u32 vignetting_table_cfg : 1;
	__u32 drc_global_tonemap_cfg : 1;
	__u32 drc_local_tonemap_cfg : 1;
};

/**
 * HDR Decompression (hdr_decompress)
 */
struct neoisp_hdr_decompress0_cfg_s {
	__u8 ctrl_enable;
	__u16 knee_point1;
	__u16 knee_point2;
	__u16 knee_point3;
	__u16 knee_point4;
	__u16 knee_offset0;
	__u16 knee_offset1;
	__u16 knee_offset2;
	__u16 knee_offset3;
	__u16 knee_offset4;
	__u16 knee_ratio0;
	__u16 knee_ratio1;
	__u16 knee_ratio2;
	__u16 knee_ratio3;
	__u16 knee_ratio4;
	__u32 knee_npoint0;
	__u32 knee_npoint1;
	__u32 knee_npoint2;
	__u32 knee_npoint3;
	__u32 knee_npoint4;
};

struct neoisp_hdr_decompress1_cfg_s {
	__u8 ctrl_enable;
	__u16 knee_point1;
	__u16 knee_point2;
	__u16 knee_point3;
	__u16 knee_point4;
	__u16 knee_offset0;
	__u16 knee_offset1;
	__u16 knee_offset2;
	__u16 knee_offset3;
	__u16 knee_offset4;
	__u16 knee_ratio0;
	__u16 knee_ratio1;
	__u16 knee_ratio2;
	__u16 knee_ratio3;
	__u16 knee_ratio4;
	__u16 knee_npoint0;
	__u16 knee_npoint1;
	__u16 knee_npoint2;
	__u16 knee_npoint3;
	__u16 knee_npoint4;
};

/**
 * Optical Black Correction and White Balance (obwb)
 */
#define NEO_OBWB_CNT (3)

struct neoisp_obwb_cfg_s {
	__u8 ctrl_obpp;
	__u16 r_ctrl_gain;
	__u16 r_ctrl_offset;
	__u16 gr_ctrl_gain;
	__u16 gr_ctrl_offset;
	__u16 gb_ctrl_gain;
	__u16 gb_ctrl_offset;
	__u16 b_ctrl_gain;
	__u16 b_ctrl_offset;
};

/**
 * HDR Merge (hdr_merge)
 */
struct neoisp_hdr_merge_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_motion_fix_en;
	__u8 ctrl_blend_3x3;
	__u8 ctrl_gain1bpp;
	__u8 ctrl_gain0bpp;
	__u8 ctrl_obpp;
	__u16 gain_offset_offset1;
	__u16 gain_offset_offset0;
	__u16 gain_scale_scale1;
	__u16 gain_scale_scale0;
	__u8 gain_shift_shift1;
	__u8 gain_shift_shift0;
	__u16 luma_th_th0;
	__u16 luma_scale_scale;
	__u8 luma_scale_shift;
	__u8 luma_scale_thshift;
	__u8 downscale_imgscale1;
	__u8 downscale_imgscale0;
	__u8 upscale_imgscale1;
	__u8 upscale_imgscale0;
	__u8 post_scale_scale;
};

/**
 * RGBIR to RGGB, IR (rgbir)
 */
struct neoisp_roi_cfg_s {
	__u16 xpos;
	__u16 ypos;
	__u16 width;
	__u16 height;
};

struct neoisp_stat_hist_cfg_s {
	__u16 hist_ctrl_offset;
	__u8 hist_ctrl_channel;
	__u8 hist_ctrl_pattern;
	__u8 hist_ctrl_dir_input1_dif;
	__u8 hist_ctrl_lin_input1_log;
	__u32 hist_scale_scale;
};

#define NEO_RGBIR_ROI_CNT       (2)
#define NEO_RGBIR_STAT_HIST_CNT (2)

struct neoisp_rgbir_cfg_s {
	__u8 ctrl_enable;
	__u16 ccm0_ccm;
	__u16 ccm1_ccm;
	__u16 ccm2_ccm;
	__u32 ccm0_th_threshold;
	__u32 ccm1_th_threshold;
	__u32 ccm2_th_threshold;
	struct neoisp_roi_cfg_s roi[NEO_RGBIR_ROI_CNT];
	struct neoisp_stat_hist_cfg_s hists[NEO_RGBIR_STAT_HIST_CNT];
};

/**
 * Color temperature (ctemp)
 */
#define NEO_CTEMP_COLOR_ROIS_CNT            (10)
#define NEO_CTEMP_CSC_MATRIX_SIZE           (3)
#define NEO_CTEMP_CSC_OFFSET_VECTOR_SIZE    (4)

struct neoisp_ctemp_roi_desc_s {
	__u8 pos_roverg_low;
	__u8 pos_roverg_high;
	__u8 pos_boverg_low;
	__u8 pos_boverg_high;
};

struct neoisp_ctemp_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_cscon;
	__u8 ctrl_ibpp;
	__u16 luma_th_thl;
	__u16 luma_th_thh;
	struct neoisp_roi_cfg_s roi;
	__u8 redgain_min;
	__u8 redgain_max;
	__u8 bluegain_min;
	__u8 bluegain_max;
	__u8 point1_blue;
	__u8 point1_red;
	__u8 point2_blue;
	__u8 point2_red;
	__u8 hoffset_right;
	__u8 hoffset_left;
	__u8 voffset_up;
	__u8 voffset_down;
	__s16 point1_slope_slope_l;
	__s16 point1_slope_slope_r;
	__s16 point2_slope_slope_l;
	__s16 point2_slope_slope_r;
	__s16 csc_matrix[NEO_CTEMP_CSC_MATRIX_SIZE][NEO_CTEMP_CSC_MATRIX_SIZE];
	__u16 offsets[NEO_CTEMP_CSC_OFFSET_VECTOR_SIZE];
	__u16 stat_blk_size0_xsize;
	__u16 stat_blk_size0_ysize;
	struct neoisp_ctemp_roi_desc_s color_rois[NEO_CTEMP_COLOR_ROIS_CNT];
	__u32 gr_avg_in_gr_agv;
	__u32 gb_avg_in_gb_agv;
};

/**
 * Statistics and Histogram (stat)
 */
#define NEO_STAT_HIST_CNT (4)

struct neoisp_stat_cfg_s {
	struct neoisp_roi_cfg_s roi0;
	struct neoisp_roi_cfg_s roi1;
	struct neoisp_stat_hist_cfg_s hists[NEO_STAT_HIST_CNT];
};

/**
 * IR Compression (ir_compress)
 */
struct neoisp_ir_compress_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_obpp;
	__u32 knee_point1_kneepoint;
	__u32 knee_point2_kneepoint;
	__u32 knee_point3_kneepoint;
	__u32 knee_point4_kneepoint;
	__u32 knee_offset0_offset;
	__u32 knee_offset1_offset;
	__u32 knee_offset2_offset;
	__u32 knee_offset3_offset;
	__u32 knee_offset4_offset;
	__u16 knee_ratio01_ratio0;
	__u16 knee_ratio01_ratio1;
	__u16 knee_ratio23_ratio2;
	__u16 knee_ratio23_ratio3;
	__u16 knee_ratio4_ratio4;
	__u16 knee_npoint0_kneepoint;
	__u16 knee_npoint1_kneepoint;
	__u16 knee_npoint2_kneepoint;
	__u16 knee_npoint3_kneepoint;
	__u16 knee_npoint4_kneepoint;
};

/**
 * Bayer Noise Reduction (bnr)
 */
struct neoisp_bnr_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_debug;
	__u8 ctrl_obpp;
	__u8 ctrl_nhood;
	__u8 ypeak_peak_outsel;
	__u8 ypeak_peak_sel;
	__u16 ypeak_peak_low;
	__u16 ypeak_peak_high;
	__u32 yedge_th0_edge_th0;
	__u16 yedge_scale_scale;
	__u8 yedge_scale_shift;
	__u32 yedges_th0_edge_th0;
	__u16 yedges_scale_scale;
	__u8 yedges_scale_shift;
	__u32 yedgea_th0_edge_th0;
	__u16 yedgea_scale_scale;
	__u8 yedgea_scale_shift;
	__u32 yluma_x_th0_th;
	__u16 yluma_y_th_luma_y_th0;
	__u16 yluma_y_th_luma_y_th1;
	__u16 yluma_scale_scale;
	__u8 yluma_scale_shift;
	__u16 yalpha_gain_gain;
	__u16 yalpha_gain_offset;
	__u8 cpeak_peak_outsel;
	__u8 cpeak_peak_sel;
	__u16 cpeak_peak_low;
	__u16 cpeak_peak_high;
	__u32 cedge_th0_edge_th0;
	__u16 cedge_scale_scale;
	__u8 cedge_scale_shift;
	__u32 cedges_th0_edge_th0;
	__u16 cedges_scale_scale;
	__u8 cedges_scale_shift;
	__u32 cedgea_th0_edge_th0;
	__u16 cedgea_scale_scale;
	__u8 cedgea_scale_shift;
	__u32 cluma_x_th0_th;
	__u16 cluma_y_th_luma_y_th0;
	__u16 cluma_y_th_luma_y_th1;
	__u16 cluma_scale_scale;
	__u8 cluma_scale_shift;
	__u16 calpha_gain_gain;
	__u16 calpha_gain_offset;
	__u16 stretch_gain;
};

/**
 * Vignetting
 */
struct neoisp_vignetting_ctrl_cfg_s {
	__u8 ctrl_enable;
	__u8 blk_conf_rows;
	__u8 blk_conf_cols;
	__u16 blk_size_ysize;
	__u16 blk_size_xsize;
	__u16 blk_stepy_step;
	__u16 blk_stepx_step;
};

/**
 * Debayer (demosaic)
 */

struct neoisp_demosaic_cfg_s {
	__u8 ctrl_fmt;
	__u16 activity_ctl_alpha;
	__u16 activity_ctl_act_ratio;
	__u16 dynamics_ctl0_strengthg;
	__u16 dynamics_ctl0_strengthc;
	__u16 dynamics_ctl2_max_impact;
};

/**
 * RGB to YUV (rgb2yuv)
 */
#define NEO_RGB2YUV_MATRIX_SIZE (3)

struct neoisp_rgb2yuv_cfg_s {
	__u16 gain_ctrl_rgain;
	__u16 gain_ctrl_bgain;
	__s16 mat_rxcy[NEO_RGB2YUV_MATRIX_SIZE][NEO_RGB2YUV_MATRIX_SIZE];
	__s32 csc_offsets[NEO_RGB2YUV_MATRIX_SIZE];
};

/**
 * Dynamic Range Compression (dr_comp)
 */

struct neoisp_dr_comp_cfg_s {
	struct neoisp_roi_cfg_s roi0;
	struct neoisp_roi_cfg_s roi1;
	__u8 groi_sum_shift_shift0;
	__u8 groi_sum_shift_shift1;
	__u16 gbl_gain_gain;
	__u16 lcl_blk_size_xsize;
	__u16 lcl_blk_size_ysize;
	__u16 lcl_stretch_offset;
	__u16 lcl_stretch_stretch;
	__u16 lcl_blk_stepx_step;
	__u16 lcl_blk_stepy_step;
	__u8 lcl_sum_shift_shift;
	__u16 alpha_alpha;
};

/**
 * Noise Reduction (nr)
 */
struct neoisp_nr_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_debug;
	__u8 blend_scale_gain;
	__u8 blend_scale_shift;
	__u16 blend_scale_scale;
	__u32 blend_th0_th;
};

/**
 * AutoFocus (af)
 */
#define NEO_AF_ROIS_CNT     (9)
#define NEO_AF_FILTERS_CNT  (9)

struct neoisp_af_cfg_s {
	struct neoisp_roi_cfg_s af_roi[NEO_AF_ROIS_CNT];
	__s8 fil0_coeffs[NEO_AF_FILTERS_CNT];
	__u8 fil0_shift_shift;
	__s8 fil1_coeffs[NEO_AF_FILTERS_CNT];
	__u8 fil1_shift_shift;
};

/**
 * Edge Enhancement (ee)
 */
struct neoisp_ee_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_debug;
	__u8 maskgain_gain;
	__u32 coring_coring;
	__u32 clip_clip;
};

/**
 * Direction Filter (df)
 */
struct neoisp_df_cfg_s {
	__u8 ctrl_enable;
	__u8 ctrl_debug;
	__u8 blend_shift_shift;
	__u32 th_scale_scale;
	__u32 blend_th0_th;
};

/**
 * Color Convolution and Median Filter (convmed)
 */
struct neoisp_convmed_cfg_s {
	__u8 ctrl_flt;
};

/**
 * Color Adaptive Saturation (cas)
 */
struct neoisp_cas_cfg_s {
	__u8 gain_shift;
	__u16 gain_scale;
	__u16 corr_corr;
	__u16 offset_offset;
};
/**
 * Gamma Correction Matrix (gcm)
 */
#define NEO_GAMMA_MATRIX_SIZE   (3)
#define NEO_GAMMA_OFFSETS_SIZE  (3)

struct neoisp_gcm_cfg_s {
	__s16 imat_rxcy[NEO_GAMMA_MATRIX_SIZE][NEO_GAMMA_MATRIX_SIZE];
	__s16 ioffsets[NEO_GAMMA_OFFSETS_SIZE];
	__s16 omat_rxcy[NEO_GAMMA_MATRIX_SIZE][NEO_GAMMA_MATRIX_SIZE];
	__s16 ooffsets[NEO_GAMMA_OFFSETS_SIZE];
	__u16 gamma0_gamma0;
	__u16 gamma0_offset0;
	__u16 gamma1_gamma1;
	__u16 gamma1_offset1;
	__u16 gamma2_gamma2;
	__u16 gamma2_offset2;
	__u16 blklvl0_ctrl_gain0;
	__s16 blklvl0_ctrl_offset0;
	__u16 blklvl1_ctrl_gain1;
	__s16 blklvl1_ctrl_offset1;
	__u16 blklvl2_ctrl_gain2;
	__s16 blklvl2_ctrl_offset2;
	__u16 lowth_ctrl01_threshold0;
	__u16 lowth_ctrl01_threshold1;
	__u16 lowth_ctrl2_threshold2;
	__u8 mat_confg_sign_confg;
};

/**
 * ISP uapi params structure
 */
struct neoisp_reg_params_s {
	/* Pipeline 1 */
	struct neoisp_hdr_decompress0_cfg_s decompress_input0;
	struct neoisp_hdr_decompress1_cfg_s decompress_input1;
	struct neoisp_obwb_cfg_s obwb[NEO_OBWB_CNT];
	struct neoisp_hdr_merge_cfg_s hdr_merge;
	struct neoisp_rgbir_cfg_s rgbir;
	struct neoisp_stat_cfg_s stat;
	struct neoisp_ir_compress_cfg_s ir_compress;
	struct neoisp_bnr_cfg_s bnr;
	struct neoisp_vignetting_ctrl_cfg_s vignetting_ctrl;
	struct neoisp_ctemp_cfg_s ctemp;

	/* Pipeline 2 */
	struct neoisp_demosaic_cfg_s demosaic;
	struct neoisp_rgb2yuv_cfg_s rgb2yuv;
	struct neoisp_dr_comp_cfg_s drc;
	/* denoising pipeline */
	struct neoisp_nr_cfg_s nrc;
	struct neoisp_af_cfg_s afc;
	struct neoisp_ee_cfg_s eec;
	struct neoisp_df_cfg_s dfc;
	struct neoisp_convmed_cfg_s convf;
	struct neoisp_cas_cfg_s cas;
	struct neoisp_gcm_cfg_s gcm;
};

#define NEO_VIGNETTING_TABLE_SIZE   (3072)
#define NEO_DRC_GLOBAL_TONEMAP_SIZE (416)
#define NEO_DRC_LOCAL_TONEMAP_SIZE  (1024)

/**
 * vignetting table
 */
struct neoisp_vignetting_table_mem_params_s {
	__u16 vignetting_table[NEO_VIGNETTING_TABLE_SIZE];
};

/**
 * DRC Global Tonemap
 */
struct neoisp_drc_global_tonemap_mem_params_s {
	__u16 drc_global_tonemap[NEO_DRC_GLOBAL_TONEMAP_SIZE];
};

/**
 * DRC Local Tonemap
 */
struct neoisp_drc_local_tonemap_mem_params_s {
	__u8 drc_local_tonemap[NEO_DRC_LOCAL_TONEMAP_SIZE];
};

struct neoisp_mem_params_s {
	struct neoisp_vignetting_table_mem_params_s vt;
	struct neoisp_drc_global_tonemap_mem_params_s gtm;
	struct neoisp_drc_local_tonemap_mem_params_s ltm;
};

struct neoisp_meta_params_s {
	__u32 frame_id;
	struct neoisp_feat_ctrl_s features_cfg;
	struct neoisp_reg_params_s regs;
	struct neoisp_mem_params_s mems;
};

/**
 * Statistics
 */
#define NEO_CTEMP_REG_STATS_CROIS_CNT   (10)
#define NEO_AF_REG_STATS_ROIS_CNT       (9)
#define NEO_CTEMP_R_SUM_CNT             (64)
#define NEO_CTEMP_G_SUM_CNT             (64)
#define NEO_CTEMP_B_SUM_CNT             (64)
#define NEO_CTEMP_PIX_CNT_CNT           (64)
#define NEO_RGBIR_HIST_CNT              (256)
#define NEO_HIST_STAT_CNT               (512)
#define NEO_DRC_LOCAL_SUM_CNT           (256)
#define NEO_DRC_GLOBAL_HIST_ROI_CNT     (416)

struct neoisp_reg_stats_crois_s {
	__u32 pixcnt_pixcnt;
	__u32 sumred_sum;
	__u32 sumgreen_sum;
	__u32 sumblue_sum;
};

struct neoisp_ctemp_reg_stats_s {
	__u32 cnt_white_white;
	__u32 sumr_sum_l; /* split low and high to avoid padding and keep aligned with hw */
	__u32 sumr_sum_h;
	__u32 sumg_sum_l;
	__u32 sumg_sum_h;
	__u32 sumb_sum_l;
	__u32 sumb_sum_h;
	__u32 sumrg_sum_l;
	__u32 sumrg_sum_h;
	__u32 sumbg_sum_l;
	__u32 sumbg_sum_h;
	struct neoisp_reg_stats_crois_s crois[NEO_CTEMP_REG_STATS_CROIS_CNT];
	__u32 gr_gb_cnt_cnt;
	__u32 gr_sum_sum;
	__u32 gb_sum_sum;
	__u32 gr2_sum_sum;
	__u32 gb2_sum_sum;
	__u32 grgb_max_max;
	__u32 grgb_max2_max;
	__u32 grgb_sum_sum;
};

struct neoisp_drc_reg_stats_s {
	__u32  groi0_sum_val;
	__u32  groi1_sum_val;
};

struct neoisp_af_reg_stats_sums_s {
	__u32 sum0;
	__u32 sum1;
};

struct neoisp_af_reg_stats_s {
	struct neoisp_af_reg_stats_sums_s rois[NEO_AF_REG_STATS_ROIS_CNT];
};

struct neoisp_bnr_reg_stats_s {
	__u32 edge_stat_edge_pixels;
	__u32 edges_stat_edge_pixels;
};

struct neoisp_nr_reg_stats_s {
	__u32 edgecnt_val;
};

struct neoisp_ee_reg_stats_s {
	__u32 edgecnt_val;
};

struct neoisp_df_reg_stats_s {
	__u32 edgecnt_val;
};


struct neoisp_reg_stats_s {
	struct neoisp_ctemp_reg_stats_s ct;
	struct neoisp_drc_reg_stats_s drc;
	struct neoisp_af_reg_stats_s af;
	struct neoisp_bnr_reg_stats_s bnr;
	struct neoisp_nr_reg_stats_s nr;
	struct neoisp_ee_reg_stats_s ee;
	struct neoisp_df_reg_stats_s df;
};

struct neoisp_ctemp_mem_stats_s {
	__u32 ctemp_r_sum[NEO_CTEMP_R_SUM_CNT];
	__u32 ctemp_g_sum[NEO_CTEMP_G_SUM_CNT];
	__u32 ctemp_b_sum[NEO_CTEMP_B_SUM_CNT];
	__u16 ctemp_pix_cnt[NEO_CTEMP_PIX_CNT_CNT];
};

struct neoisp_rgbir_mem_stats_s {
	__u32 rgbir_hist[NEO_RGBIR_HIST_CNT];
};

struct neoisp_hist_mem_stats_s {
	__u32 hist_stat[NEO_HIST_STAT_CNT];
};

struct neoisp_drc_mem_stats_s {
	__u32 drc_local_sum[NEO_DRC_LOCAL_SUM_CNT];
	__u32 drc_global_hist_roi0[NEO_DRC_GLOBAL_HIST_ROI_CNT];
	__u32 drc_global_hist_roi1[NEO_DRC_GLOBAL_HIST_ROI_CNT];
};


struct neoisp_mem_stats_s {
	struct neoisp_ctemp_mem_stats_s ctemp;
	struct neoisp_rgbir_mem_stats_s rgbir;
	struct neoisp_hist_mem_stats_s hist;
	struct neoisp_drc_mem_stats_s drc;
};

struct neoisp_meta_stats_s {
	struct neoisp_reg_stats_s regs;
	struct neoisp_mem_stats_s mems;
};

#endif /* UAPI_NXP_NEOISP_H */
