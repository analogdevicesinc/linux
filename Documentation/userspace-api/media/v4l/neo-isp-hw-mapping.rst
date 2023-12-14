.. SPDX-License-Identifier: GPL-2.0

NXP NEO ISP uapi structures mapping
====================================

This is a list of neo-isp uapi structures mapped to hardware registers/fields

Hardware registers are based on neo_regs.h file generated from RTL sources that
could be easily mapped to neo isp reference manual tables.

Left side struture members are defined in nxp_neoisp.h uapi header file

Hardware registers are in the right side and in capital letters

 * HDR Decompression

======================================= =======================================
struct neob_isp_hdr_decompress_cfg_s    [DECOMPRESS0, DECOMPRESS1]
ctrl_enable                             NEO_HDR_DECOMPRESS0_CTRL_CAM0_ENABLE
knee_point1                             NEO_HDR_DECOMPRESS0_KNEE_POINT1_CAM0
knee_point2                             NEO_HDR_DECOMPRESS0_KNEE_POINT2_CAM0
knee_point3                             NEO_HDR_DECOMPRESS0_KNEE_POINT3_CAM0
knee_point4                             NEO_HDR_DECOMPRESS0_KNEE_POINT4_CAM0
knee_offset0                            NEO_HDR_DECOMPRESS0_KNEE_OFFSET0_CAM0
knee_offset1                            NEO_HDR_DECOMPRESS0_KNEE_OFFSET1_CAM0
knee_offset2                            NEO_HDR_DECOMPRESS0_KNEE_OFFSET2_CAM0
knee_offset3                            NEO_HDR_DECOMPRESS0_KNEE_OFFSET3_CAM0
knee_offset4                            NEO_HDR_DECOMPRESS0_KNEE_OFFSET4_CAM0
knee_ratio0                             NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_RATIO0
knee_ratio1                             NEO_HDR_DECOMPRESS0_KNEE_RATIO01_CAM0_RATIO1
knee_ratio2                             NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_RATIO2
knee_ratio3                             NEO_HDR_DECOMPRESS0_KNEE_RATIO23_CAM0_RATIO3
knee_ratio4                             NEO_HDR_DECOMPRESS0_KNEE_RATIO4_CAM0_RATIO4
knee_npoint0                            NEO_HDR_DECOMPRESS0_KNEE_NPOINT0_CAM0
knee_npoint1                            NEO_HDR_DECOMPRESS0_KNEE_NPOINT1_CAM0
knee_npoint2                            NEO_HDR_DECOMPRESS0_KNEE_NPOINT2_CAM0
knee_npoint3                            NEO_HDR_DECOMPRESS0_KNEE_NPOINT3_CAM0
knee_npoint4                            NEO_HDR_DECOMPRESS0_KNEE_NPOINT4_CAM0
======================================= =======================================

 * Optical Black Correction and White Balancing

=============================== ===============================================
struct neob_isp_obwb_cfg_s      [WB0, WB1, WB2]
ctrl_obpp                       NEO_OB_WB0_CTRL_CAM0_OBPP
r_ctrl_gain                     NEO_OB_WB0_R_CTRL_CAM0_GAIN
r_ctrl_offset                   NEO_OB_WB0_R_CTRL_CAM0_OFFSET
gr_ctrl_gain                    NEO_OB_WB0_GR_CTRL_CAM0_GAIN
gr_ctrl_offset                  NEO_OB_WB0_GR_CTRL_CAM0_OFFSET
gb_ctrl_gain                    NEO_OB_WB0_GB_CTRL_CAM0_GAIN
gb_ctrl_offset                  NEO_OB_WB0_GB_CTRL_CAM0_OFFSET
b_ctrl_gain                     NEO_OB_WB0_B_CTRL_CAM0_GAIN
b_ctrl_offset                   NEO_OB_WB0_B_CTRL_CAM0_OFFSET
=============================== ===============================================

 * HDR Merge

=================================== ===========================================
struct neob_isp_hdr_merge_cfg_s
ctrl_enable                         NEO_HDR_MERGE_CTRL_CAM0_ENABLE
ctrl_motion_fix_en                  NEO_HDR_MERGE_CTRL_CAM0_MOTION_FIX_EN
ctrl_blend_3x3                      NEO_HDR_MERGE_CTRL_CAM0_BLEND_3X3
ctrl_gain1bpp                       NEO_HDR_MERGE_CTRL_CAM0_GAIN1BPP
ctrl_gain0bpp                       NEO_HDR_MERGE_CTRL_CAM0_GAIN0BPP
ctrl_obpp                           NEO_HDR_MERGE_CTRL_CAM0_OBPP
gain_offset_offset1                 NEO_HDR_MERGE_GAIN_OFFSET_CAM0_OFFSET1
gain_offset_offset0                 NEO_HDR_MERGE_GAIN_OFFSET_CAM0_OFFSET0
gain_scale_scale1                   NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE1
gain_scale_scale0                   NEO_HDR_MERGE_GAIN_SCALE_CAM0_SCALE0
gain_shift_shift1                   NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT1
gain_shift_shift0                   NEO_HDR_MERGE_GAIN_SHIFT_CAM0_SHIFT0
luma_th_th0                         NEO_HDR_MERGE_LUMA_TH_CAM0_TH0
luma_scale_scale                    NEO_HDR_MERGE_LUMA_SCALE_CAM0_SCALE
luma_scale_shift                    NEO_HDR_MERGE_LUMA_SCALE_CAM0_SHIFT
luma_scale_thshift                  NEO_HDR_MERGE_LUMA_SCALE_CAM0_THSHIFT
downscale_imgscale1                 NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE1
downscale_imgscale0                 NEO_HDR_MERGE_DOWNSCALE_CAM0_IMGSCALE0
upscale_imgscale1                   NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE1
upscale_imgscale0                   NEO_HDR_MERGE_UPSCALE_CAM0_IMGSCALE0
post_scale_scale                    NEO_HDR_MERGE_POST_SCALE_CAM0_SCALE
=================================== ===========================================

 * RGBIR to RGGB, IR

=============================== ===============================================
struct neob_isp_rgbir_cfg_s
ctrl_enable                     NEO_RGBIR_CTRL_CAM0_ENABLE
ccm0_ccm                        NEO_RGBIR_CCM0_CAM0_CCM
ccm1_ccm                        NEO_RGBIR_CCM1_CAM0_CCM
ccm2_ccm                        NEO_RGBIR_CCM2_CAM0_CCM
ccm0_th_threshold               NEO_RGBIR_CCM0_TH_CAM0_THRESHOLD
ccm1_th_threshold               NEO_RGBIR_CCM1_TH_CAM0_THRESHOLD
ccm2_th_threshold               NEO_RGBIR_CCM2_TH_CAM0_THRESHOLD
struct nes_isp_roi_cfg_s        [ROI0, ROI1]
  - xpos                        NEO_RGBIR_ROI0_POS_CAM0_XPOS
  - ypos                        NEO_RGBIR_ROI0_POS_CAM0_YPOS
  - width                       NEO_RGBIR_ROI0_SIZE_CAM0_WIDTH
  - height                      NEO_RGBIR_ROI0_SIZE_CAM0_HEIGHT
struct neo_isp_stat_hist_cfg_s  [HIST0, HIST1]
  - hist_ctrl_offset            NEO_RGBIR_HIST0_CTRL_CAM0_OFFSET
  - hist_ctrl_channel           NEO_RGBIR_HIST0_CTRL_CAM0_CHANNEL
  - hist_ctrl_pattern           NEO_RGBIR_HIST0_CTRL_CAM0_PATTERN
  - hist_ctrl_dir_vs_dif        NEO_RGBIR_HIST0_CTRL_CAM0_DIR_VS_DIF
  - hist_ctrl_lin_vs_log        NEO_RGBIR_HIST0_CTRL_CAM0_LIN_VS_LOG
  - hist_scale_scale            NEO_RGBIR_HIST0_SCALE_CAM0_SCALE
=============================== ===============================================

 * Color Temperature

=============================== ===============================================
struct neob_isp_ctemp_cfg_s
ctrl_enable                     NEO_COLOR_TEMP_CTRL_CAM0_ENABLE
ctrl_cscon                      NEO_COLOR_TEMP_CTRL_CAM0_CSCON
ctrl_ibpp                       NEO_COLOR_TEMP_CTRL_CAM0_IBPP
luma_th_thl                     NEO_COLOR_TEMP_LUMA_TH_CAM0_THL
luma_th_thh                     NEO_COLOR_TEMP_LUMA_TH_CAM0_THH
struct roi_cfg_s
  - xpos                        NEO_COLOR_TEMP_ROI_POS_CAM0_XPOS
  - ypos                        NEO_COLOR_TEMP_ROI_POS_CAM0_YPOS
  - width                       NEO_COLOR_TEMP_ROI_SIZE_CAM0_WIDTH
  - height                      NEO_COLOR_TEMP_ROI_SIZE_CAM0_HEIGHT
redgain_min                     NEO_COLOR_TEMP_REDGAIN_CAM0_MIN
redgain_max                     NEO_COLOR_TEMP_REDGAIN_CAM0_MAX
bluegain_min                    NEO_COLOR_TEMP_BLUEGAIN_CAM0_MIN
bluegain_max                    NEO_COLOR_TEMP_BLUEGAIN_CAM0_MAX
point1_blue                     NEO_COLOR_TEMP_POINT1_CAM0_BLUE
point1_red                      NEO_COLOR_TEMP_POINT1_CAM0_RED
point2_blue                     NEO_COLOR_TEMP_POINT2_CAM0_BLUE
point2_red                      NEO_COLOR_TEMP_POINT2_CAM0_RED
hoffset_right                   NEO_COLOR_TEMP_HOFFSET_CAM0_RIGHT
hoffset_left                    NEO_COLOR_TEMP_HOFFSET_CAM0_LEFT
voffset_up                      NEO_COLOR_TEMP_VOFFSET_CAM0_UP
voffset_down                    NEO_COLOR_TEMP_VOFFSET_CAM0_DOWN
point1_slope_slope_l            NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_SLOPE_L
point1_slope_slope_r            NEO_COLOR_TEMP_POINT1_SLOPE_CAM0_SLOPE_R
point2_slope_slope_l            NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_SLOPE_L
point2_slope_slope_r            NEO_COLOR_TEMP_POINT2_SLOPE_CAM0_SLOPE_R
csc_mat_rxcy[3][3]              NEO_COLOR_TEMP_CSC_MAT0_CAM0_R0C0

                                NEO_COLOR_TEMP_CSC_MAT0_CAM0_R2C2
offsets[4]                      NEO_COLOR_TEMP_R_GR_OFFSET_CAM0_OFFSET0

                                NEO_COLOR_TEMP_GB_B_OFFSET_CAM0_OFFSET1
gr_avg_in_gr_agv                NEO_COLOR_TEMP_GR_AVG_IN_CAM0_GR_AGV
gb_avg_in_gb_agv                NEO_COLOR_TEMP_GB_AVG_IN_CAM0_GB_AGV
struct neo_isp_ctemp_roi_desc_s [CROI0, .. CROI9]
  - pos_roverg_low              NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_LOW
  - pos_roverg_high             NEO_COLOR_TEMP_CROI0_POS_CAM0_ROVERG_HIGH
  - pos_boverg_low              NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_LOW
  - pos_boverg_high             NEO_COLOR_TEMP_CROI0_POS_CAM0_BOVERG_HIGH
=============================== ===============================================

 * Stat Config

================================= =============================================
struct neob_isp_stat_cfg_s
struct roi_cfg_s                  [ROI0, ROI1]
  - xpos                          NEO_STAT_ROI0_POS_CAM0_XPOS
  - ypos                          NEO_STAT_ROI0_POS_CAM0_YPOS
  - width                         NEO_STAT_ROI0_SIZE_CAM0_WIDTH
  - height                        NEO_STAT_ROI0_SIZE_CAM0_HEIGHT
struct neo_isp_stat_hist_cfg_s    [HIST0, .. HIST3]
  - hist_ctrl_offset              NEO_STAT_HIST0_CTRL_CAM0_OFFSET
  - hist_ctrl_channel             NEO_STAT_HIST0_CTRL_CAM0_CHANNEL
  - hist_ctrl_pattern             NEO_STAT_HIST0_CTRL_CAM0_PATTERN
  - hist_ctrl_dir_vs_dif          NEO_STAT_HIST0_CTRL_CAM0_DIR_VS_DIF
  - hist_ctrl_lin_vs_log          NEO_STAT_HIST0_CTRL_CAM0_LIN_VS_LOG
  - hist_scale_scale              NEO_STAT_HIST0_SCALE_CAM0_SCALE
================================= =============================================

 * IR Compression

=================================== ===========================================
struct neob_isp_ir_compress_cfg_s
ctrl_enable                         NEO_IR_COMPRESS_CTRL_ENABLE
ctrl_obpp                           NEO_IR_COMPRESS_CTRL_OBPP
knee_point1_kneepoint               NEO_IR_COMPRESS_KNEE_POINT1_KNEEPOINT
knee_point2_kneepoint               NEO_IR_COMPRESS_KNEE_POINT2_KNEEPOINT
knee_point3_kneepoint               NEO_IR_COMPRESS_KNEE_POINT3_KNEEPOINT
knee_point4_kneepoint               NEO_IR_COMPRESS_KNEE_POINT4_KNEEPOINT
knee_offset0_offset                 NEO_IR_COMPRESS_KNEE_OFFSET0_OFFSET
knee_offset1_offset                 NEO_IR_COMPRESS_KNEE_OFFSET1_OFFSET
knee_offset2_offset                 NEO_IR_COMPRESS_KNEE_OFFSET2_OFFSET
knee_offset3_offset                 NEO_IR_COMPRESS_KNEE_OFFSET3_OFFSET
knee_offset4_offset                 NEO_IR_COMPRESS_KNEE_OFFSET4_OFFSET
knee_ratio01_ratio0                 NEO_IR_COMPRESS_KNEE_RATIO01_RATIO0
knee_ratio01_ratio1                 NEO_IR_COMPRESS_KNEE_RATIO01_RATIO1
knee_ratio23_ratio2                 NEO_IR_COMPRESS_KNEE_RATIO23_RATIO2
knee_ratio23_ratio3                 NEO_IR_COMPRESS_KNEE_RATIO23_RATIO3
knee_ratio4_ratio4                  NEO_IR_COMPRESS_KNEE_RATIO4_RATIO4
knee_npoint0_kneepoint              NEO_IR_COMPRESS_KNEE_NPOINT0_KNEEPOINT
knee_npoint1_kneepoint              NEO_IR_COMPRESS_KNEE_NPOINT1_KNEEPOINT
knee_npoint2_kneepoint              NEO_IR_COMPRESS_KNEE_NPOINT2_KNEEPOINT
knee_npoint3_kneepoint              NEO_IR_COMPRESS_KNEE_NPOINT3_KNEEPOINT
knee_npoint4_kneepoint              NEO_IR_COMPRESS_KNEE_NPOINT4_KNEEPOINT
=================================== ===========================================

 * Bayer Noise Reduction

=============================== ===============================================
struct neob_isp_bnr_cfg_s
ctrl_enable                     NEO_BNR_CTRL_ENABLE
ctrl_debug                      NEO_BNR_CTRL_DEBUG
ctrl_obpp                       NEO_BNR_CTRL_OBPP
ctrl_nhood                      NEO_BNR_CTRL_NHOOD
ypeak_peak_outsel               NEO_BNR_YPEAK_PEAK_OUTSEL
ypeak_peak_sel                  NEO_BNR_YPEAK_PEAK_SEL
ypeak_peak_low                  NEO_BNR_YPEAK_PEAK_LOW
ypeak_peak_high                 NEO_BNR_YPEAK_PEAK_HIGH
yedge_th0_edge_th0              NEO_BNR_YEDGE_TH0_EDGE_TH0
yedge_scale_scale               NEO_BNR_YEDGE_SCALE_SCALE
yedge_scale_shift               NEO_BNR_YEDGE_SCALE_SHIFT
yedges_th0_edge_th0             NEO_BNR_YEDGES_TH0_EDGE_TH0
yedges_scale_scale              NEO_BNR_YEDGES_SCALE_SCALE
yedges_scale_shift              NEO_BNR_YEDGES_SCALE_SHIFT
yedgea_th0_edge_th0             NEO_BNR_YEDGEA_TH0_EDGE_TH0
yedgea_scale_scale              NEO_BNR_YEDGEA_SCALE_SCALE
yedgea_scale_shift              NEO_BNR_YEDGEA_SCALE_SHIFT
yluma_x_th0_th                  NEO_BNR_YLUMA_X_TH0_TH
yluma_y_th_luma_y_th0           NEO_BNR_YLUMA_Y_TH_LUMA_Y_TH0
yluma_y_th_luma_y_th1           NEO_BNR_YLUMA_Y_TH_LUMA_Y_TH1
yluma_scale_scale               NEO_BNR_YLUMA_SCALE_SCALE
yluma_scale_shift               NEO_BNR_YLUMA_SCALE_SHIFT
yalpha_gain_gain                NEO_BNR_YALPHA_GAIN_GAIN
yalpha_gain_offset              NEO_BNR_YALPHA_GAIN_OFFSET
cpeak_peak_outsel               NEO_BNR_CPEAK_PEAK_OUTSEL
cpeak_peak_sel                  NEO_BNR_CPEAK_PEAK_SEL
cpeak_peak_low                  NEO_BNR_CPEAK_PEAK_LOW
cpeak_peak_high                 NEO_BNR_CPEAK_PEAK_HIGH
cedge_th0_edge_th0              NEO_BNR_CEDGE_TH0_EDGE_TH0
cedge_scale_scale               NEO_BNR_CEDGE_SCALE_SCALE
cedge_scale_shift               NEO_BNR_CEDGE_SCALE_SHIFT
cedges_th0_edge_th0             NEO_BNR_CEDGES_TH0_EDGE_TH0
cedges_scale_scale              NEO_BNR_CEDGES_SCALE_SCALE
cedges_scale_shift              NEO_BNR_CEDGES_SCALE_SHIFT
cedgea_th0_edge_th0             NEO_BNR_CEDGEA_TH0_EDGE_TH0
cedgea_scale_scale              NEO_BNR_CEDGEA_SCALE_SCALE
cedgea_scale_shift              NEO_BNR_CEDGEA_SCALE_SHIFT
cluma_x_th0_th                  NEO_BNR_CLUMA_X_TH0_TH
cluma_y_th_luma_y_th0           NEO_BNR_CLUMA_Y_TH_LUMA_Y_TH0
cluma_y_th_luma_y_th1           NEO_BNR_CLUMA_Y_TH_LUMA_Y_TH1
cluma_scale_scale               NEO_BNR_CLUMA_SCALE_SCALE
cluma_scale_shift               NEO_BNR_CLUMA_SCALE_SHIFT
calpha_gain_gain                NEO_BNR_CALPHA_GAIN_GAIN
calpha_gain_offset              NEO_BNR_CALPHA_GAIN_OFFSET
stretch_gain                    NEO_BNR_STRETCH_GAIN
=============================== ===============================================

 * Vignetting

======================================= =======================================
struct neob_isp_vignetting_ctrl_cfg_s
ctrl_enable                             NEO_VIGNETTING_CTRL_ENABLE
blk_conf_rows                           NEO_VIGNETTING_BLK_CONF_ROWS
blk_conf_cols                           NEO_VIGNETTING_BLK_CONF_COLS
blk_size_ysize                          NEO_VIGNETTING_BLK_SIZE_YSIZE
blk_size_xsize                          NEO_VIGNETTING_BLK_SIZE_XSIZE
blk_stepy_step                          NEO_VIGNETTING_BLK_STEPY_STEP
blk_stepx_step                          NEO_VIGNETTING_BLK_STEPX_STEP
======================================= =======================================

 * Demosaic

=============================== ===============================================
struct neob_isp_demosaic_cfg_s
ctrl_fmt                        NEO_DEMOSAIC_CTRL_FMT
activity_ctl_alpha              NEO_DEMOSAIC_ACTIVITY_CTL_ALPHA
activity_ctl_act_ratio          NEO_DEMOSAIC_ACTIVITY_CTL_ACT_RATIO
dynamics_ctl0_strengthg         NEO_DEMOSAIC_DYNAMICS_CTL0_STRENGTHG
dynamics_ctl0_strengthc         NEO_DEMOSAIC_DYNAMICS_CTL0_STRENGTHC
dynamics_ctl2_max_impact        NEO_DEMOSAIC_DYNAMICS_CTL2_MAX_IMPACT
=============================== ===============================================

 * RGB to YUV

=============================== ===============================================
struct neo_isp_rgb2yuv_cfg_s
gain_ctrl_rgain                 NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_RGAIN
gain_ctrl_bgain                 NEO_RGB_TO_YUV_GAIN_CTRL_CAM0_BGAIN
mat_rxcy[3][3]                  NEO_RGB_TO_YUV_MAT0_CAM0_R0C0

                                NEO_RGB_TO_YUV_MAT0_CAM0_R2C2
csc_offsets[3]                  NEO_RGB_TO_YUV_OFFSET0_CAM0_OFFSET

                                NEO_RGB_TO_YUV_OFFSET2_CAM0_OFFSET
=============================== ===============================================

 * Dynamic Range Compression (dr_comp)

=============================== ===============================================
struct neo_isp_dr_comp_cfg_s
struct neo_isp_glob_roi_cfg_s   [ROI0, ROI1]
  - xpos                        NEO_DRC_ROI0_POS_CAM0_XPOS
  - ypos                        NEO_DRC_ROI0_POS_CAM0_YPOS
  - width                       NEO_DRC_ROI0_SIZE_CAM0_WIDTH
  - height                      NEO_DRC_ROI1_SIZE_CAM0_HEIGHT
groi_sum_shift_shift0           NEO_DRC_GROI_SUM_SHIFT_SHIFT0
groi_sum_shift_shift1           NEO_DRC_GROI_SUM_SHIFT_SHIFT1
gbl_gain_gain_mask              NEO_DRC_GBL_GAIN_GAIN_MASK
lcl_blk_size_xsize              NEO_DRC_LCL_BLK_SIZE_XSIZE
lcl_blk_size_ysize              NEO_DRC_LCL_BLK_SIZE_YSIZE
lcl_stretch_offset              NEO_DRC_LCL_STRETCH_OFFSET
lcl_stretch_stretch             NEO_DRC_LCL_STRETCH_STRETCH
lcl_blk_stepx_step              NEO_DRC_LCL_BLK_STEPX_STEP
lcl_blk_stepy_step              NEO_DRC_LCL_BLK_STEPY_STEP
lcl_sum_shift_shift             NEO_DRC_LCL_SUM_SHIFT_SHIFT
alpha_alpha                     NEO_DRC_ALPHA_ALPHA
=============================== ===============================================

 * Noise Reduction

=============================== ===============================================
struct neo_isp_nr_cfg_s
ctrl_enable                     NEO_NR_CTRL_ENABLE
ctrl_debug                      NEO_NR_CTRL_DEBUG
blend_scale_gain                NEO_NR_BLEND_SCALE_GAIN
blend_scale_shift               NEO_NR_BLEND_SCALE_SHIFT
blend_scale_scale               NEO_NR_BLEND_SCALE_SCALE
blend_th0_th                    NEO_NR_BLEND_TH0_TH
=============================== ===============================================

 * AutoFocus

=============================== ===============================================
struct neo_isp_af_cfg_s
struct neo_isp_roi_cfg_s        [ROI0, .. ROI8]
  - xpos                        NEO_AUTOFOCUS_ROI0_POS_CAM0_XPOS
  - ypos                        NEO_AUTOFOCUS_ROI0_POS_CAM0_YPOS
  - width                       NEO_AUTOFOCUS_ROI0_SIZE_CAM0_WIDTH
  - height                      NEO_AUTOFOCUS_ROI1_SIZE_CAM0_HEIGHT
fil0_coeffs[9]                  NEO_AUTOFOCUS_FIL0_COEFFS0_CAM0_COEFF0

                                NEO_AUTOFOCUS_FIL0_COEFFS2_CAM0_COEFF8
fil0_shift_shift                NEO_AUTOFOCUS_FIL0_SHIFT_CAM0_SHIFT
fil1_coeffs[9]                  NEO_AUTOFOCUS_FIL1_COEFFS0_CAM0_COEFF0

                                NEO_AUTOFOCUS_FIL1_COEFFS2_CAM0_COEFF8
fil1_shift_shift                NEO_AUTOFOCUS_FIL1_SHIFT_CAM0_SHIFT
=============================== ===============================================

 * Edge Enhancement

=============================== ===============================================
struct neo_isp_ee_cfg_s
ctrl_enable                     NEO_EE_CTRL_ENABLE
ctrl_debug                      NEO_EE_CTRL_DEBUG
maskgain_gain                   NEO_EE_MASKGAIN_GAIN
coring_coring                   NEO_EE_CORING_CORING
clip_clip                       NEO_EE_CLIP_CLIP
=============================== ===============================================

 * Direction Filter

=============================== ===============================================
struct neo_isp_df_cfg_s
ctrl_enable                     NEO_DF_CTRL_ENABLE
ctrl_debug                      NEO_DF_CTRL_DEBUG
blend_shift_shift               NEO_DF_BLEND_SHIFT_SHIFT
th_scale_scale                  NEO_DF_TH_SCALE_SCALE
blend_th0_th                    NEO_DF_BLEND_TH0_TH
=============================== ===============================================

 * Color Convolution and Median Filter

=============================== ===============================================
ctrl_flt                        NEO_CCONVMED_CTRL_CAM0_FLT
=============================== ===============================================

 * Color Adaptive Saturation

=============================== ===============================================
struct neo_isp_cas_cfg_s
gain_shift                      NEO_CAS_GAIN_CAM0_SHIFT
gain_scale                      NEO_CAS_GAIN_CAM0_SCALE
corr_corr                       NEO_CAS_CORR_CAM0_CORR
offset_offset                   NEO_CAS_OFFSET_CAM0_OFFSET
=============================== ===============================================

 * Gamma Correction Matrix

=============================== ===============================================
struct neo_isp_gcm_cfg_s
imat_rxcy[3][3]                 NEO_GCM_IMAT0_CAM0_R0C0

                                NEO_GCM_IMAT5_CAM0_R2C2
ioffsets[3]                     NEO_GCM_IOFFSET0_CAM0_OFFSET0

                                NEO_GCM_IOFFSET2_CAM0_OFFSET2
omat_rxcy[3][3]                 NEO_GCM_OMAT0_CAM0_R0C0

                                NEO_GCM_OMAT5_CAM0_R2C2
ooffsets[3]                     NEO_GCM_OOFFSET0_CAM0_OFFSET0

                                NEO_GCM_OOFFSET2_CAM0_OFFSET2
gamma0_gamma0                   NEO_GCM_GAMMA0_GAMMA0
gamma0_offset0                  NEO_GCM_GAMMA0_OFFSET0
gamma1_gamma1                   NEO_GCM_GAMMA1_GAMMA1
gamma1_offset1                  NEO_GCM_GAMMA1_OFFSET1
gamma2_gamma2                   NEO_GCM_GAMMA2_GAMMA2
gamma2_offset2                  NEO_GCM_GAMMA2_OFFSET2
blklvl0_ctrl_gain0              NEO_GCM_BLKLVL0_CTRL_GAIN0
blklvl0_ctrl_offset0            NEO_GCM_BLKLVL0_CTRL_OFFSET0
blklvl1_ctrl_gain1              NEO_GCM_BLKLVL1_CTRL_GAIN1
blklvl1_ctrl_offset1            NEO_GCM_BLKLVL1_CTRL_OFFSET1
blklvl2_ctrl_gain2              NEO_GCM_BLKLVL2_CTRL_GAIN2
blklvl2_ctrl_offset2            NEO_GCM_BLKLVL2_CTRL_OFFSET2
lowth_ctrl01_threshold0         NEO_GCM_LOWTH_CTRL01_THRESHOLD0
lowth_ctrl01_threshold1         NEO_GCM_LOWTH_CTRL01_THRESHOLD1
lowth_ctrl2_threshold2          NEO_GCM_LOWTH_CTRL2_THRESHOLD2
mat_confg_sign_confg            NEO_GCM_MAT_CONFG_SIGN_CONFG
=============================== ===============================================
