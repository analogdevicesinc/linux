/***************************************************
  Copyright (c) 2015 Amphion Semiconductor Ltd
                All rights reserved.
 ***************************************************
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 ****************************************************

  Author    : VCodec FW team
  File name : mvd_reg_map.h
  Function  : Register map file

 ************************************************/

#ifndef _MVD_REG_MAP_H_
#define _MVD_REG_MAP_H_

/////////////////////////////////////////////////////////////////
// HIF address index
//

typedef struct MvdHwRegHifMapTag
{
  // 0x00 --> 0x0f
  MvdHwReg control;
  MvdHwReg status;
  MvdHwReg config;
  MvdHwReg config2;
  MvdHwReg test;
  MvdHwReg host_interrupt_enable;
  MvdHwReg interrupt_status;
  MvdHwReg fast_interrupt_enable;
  MvdHwReg dq_unused[4];
  MvdHwReg security;
  MvdHwReg format_disable;
  MvdHwReg system_data;
  MvdHwReg spp_security;
  // 0x10 --> 0x1f
  MvdHwReg dfe_security;
  MvdHwReg hif_unused[15];

  // the rest are not used
  // MvdHwReg unused_0x60[96];

} MvdHwRegHifMap;


/////////////////////////////////////////////////////////////////
// SIF address index
//

typedef struct MvdHwRegSifMapTag
{
  // 0x00 --> 0x0f
  MvdHwReg semaphore_read;      // R
  MvdHwReg intr_mask_read;      // R
  MvdHwReg semaphore_set;       // W
  MvdHwReg semaphore_clr;       // W
  MvdHwReg intr_mask_set;       // W
  MvdHwReg intr_mask_clr;       // W
  MvdHwReg host_data0_reg;      // R
  MvdHwReg msd_data0_reg;       // R/W
  MvdHwReg host_data1_reg;      // R
  MvdHwReg msd_data1_reg;       // R/W
  MvdHwReg frame_active_count;
  MvdHwReg slice_active_count;
  MvdHwReg rbsp_bytes_count;
  MvdHwReg sib_wait_count;
  MvdHwReg dpb_read_count;
  MvdHwReg mpr_wait_count;
  // 0x10 --> 0x1f
  MvdHwReg intr3_control;       // W/R
  MvdHwReg dpb_fs_size;         // R/W
  MvdHwReg dpb_frm_size;        // R/W
  MvdHwReg dpb_fs_size_ext;     // R/W
  MvdHwReg rsb_mprr_mprc_base;  // R/W
  MvdHwReg rsb_bsd_dbfc_base;   // R/W
  MvdHwReg rsb_pwt_pxd_base;    // R/W
  MvdHwReg frm_dangling;        // W
  MvdHwReg rsb_dfe_base;        // R/W
  MvdHwReg dpb_lut_load;        // W
  MvdHwReg bbb_crc;             // R
  MvdHwReg load_dpb_numb;       // W
  MvdHwReg dpb_frm_size_ext;    // R/W
  MvdHwReg dec_status;          // R
  MvdHwReg mbq_full_count;
  MvdHwReg mbq_empty_count;
  // 0x20 --> 0x2f
  MvdHwReg dispq_push[16];
  // 0x30 --> 0x3f
  MvdHwReg intr3_status;
  MvdHwReg intr3_force;
  MvdHwReg bs2rbsp_status;      // R
  MvdHwReg bs2rbsp_feed_control;// W/R
  MvdHwReg bs2rbsp_scode;       // R
  MvdHwReg bs2rbsp_scdctrl;     // W/R
  MvdHwReg rpr_wr_uv_offset;
  MvdHwReg dispd_cnt_tag;       // R
  MvdHwReg frame_request;       // R
  MvdHwReg intr2_control;       // W/R
  MvdHwReg intr2_status;        // R/W
  MvdHwReg intr2_force;
  MvdHwReg control;             // W/R
  MvdHwReg intr_status;         // R/W
  MvdHwReg intr_force;
  MvdHwReg soft_reset;          // W
  // 0x40 --> 0x4f
  MvdHwReg pes_setup;           // W/R
  MvdHwReg pes_status;          // W/R
  MvdHwReg pes_ctrl;            // W
  MvdHwReg pes_set_state;       // W
  MvdHwReg pes_peek_data;       // R
  MvdHwReg pes_read_data;       // R
  MvdHwReg pes_pts;             // R
  MvdHwReg pes_dts;             // R
  MvdHwReg bs_read;             // R
  MvdHwReg dpbmc_setup;         // W
  MvdHwReg dpbmc_crc_data;      // R
  MvdHwReg rsb_ctrl_stat;
  MvdHwReg ext_rsb_base;
  MvdHwReg ext_xfr_param;
  MvdHwReg vmif_rsb_arb_ctrl;
  MvdHwReg vmif_stride;

  // 0x50 --> 0x5f
  MvdHwReg vmif_uv_offset;
  MvdHwReg vmif_options;
  MvdHwReg vmif_base_offset;
  MvdHwReg vmif_mbi_cache;
  MvdHwReg lmem_config;
  MvdHwReg qp_acc;
  MvdHwReg vmif_mbi_xmem;
  MvdHwReg vmif_4kcache_stride;
  MvdHwReg dpbmc_setup2;
  MvdHwReg rpr_rd_uv_offset;
  MvdHwReg slow_feed_wt_val;
  MvdHwReg unused_0x5b;
  MvdHwReg unused_0x5c;
  MvdHwReg dpbmc_crc2_data;     // R
  MvdHwReg errgen_config;       // W
  MvdHwReg errgen_seeds;        // W
  // 0x60 --> 0x6f
  MvdHwReg bsdma_command;
  MvdHwReg bsdma_options;
  MvdHwReg bsdma_status;
  MvdHwReg bsdma_des;
  MvdHwReg bsdma_bwp;
  MvdHwReg bsdma_brp;
  MvdHwReg bsdma_bsa;
  MvdHwReg bsdma_bea;
  MvdHwReg bsdma_blw;
  MvdHwReg bsdma_bup;
  MvdHwReg bsdma_peek;
  MvdHwReg bsdma_bhw;
  MvdHwReg bsdma_trc;
  MvdHwReg bsdma_ext_options;
  MvdHwReg bsdma_lvl;
  MvdHwReg unused_0x6f;
  // 0x70 --> 0x7f
  // MvdHwReg unused_16[16];
} MvdHwRegSifMap;

/////////////////////////////////////////////////////////////////
// Context Reg Map
//

#define MVD_CTX_POC_RESTORE_WORDS     62

typedef struct MvdHwRegCtxMapTag
{
  // 0x00 --> 0x0f
  MvdHwReg ctx_command;
  MvdHwReg ctx_next_sf;
  MvdHwReg ctx_spp_command;
  MvdHwReg align_7[4];
  MvdHwReg ctx_status;
  MvdHwReg align_8[8];

  // 0x10 --> 0x1f
  MvdHwReg ctx_addr_spp[16];    /*  PreParser (if it is configured in) */
//  MvdHwReg align_9[4];

  // 0x20 --> 0x2f
  MvdHwReg ctx_addr_bbb[2];     /* Active BBB                          */
  MvdHwReg align_10[14];

  // 0x30 --> 0x3f
  MvdHwReg ctx_addr_bsi[13];    /* SCD/PES/FIFO                        */
  MvdHwReg align_11[3];

  // 0x40 --> 0x7f
  MvdHwReg h264_poc_restore[MVD_CTX_POC_RESTORE_WORDS];
  MvdHwReg unused_poc[64-MVD_CTX_POC_RESTORE_WORDS];

} MvdHwRegCtxMap;


/////////////////////////////////////////////////////////////////
// Resampler Reg Map
//

typedef struct MvdHwRegRprMapTag
{
  // 0x0 --> 0xf
  MvdHwReg rpr_status_addr;   // R
  MvdHwReg rpr_ctl_start;
  MvdHwReg rpr_in_size;
  MvdHwReg rpr_out_size;
  MvdHwReg rpr_fill_value;
  MvdHwReg rpr_ax_ini_y;
  MvdHwReg rpr_ax_ini_uv;
  MvdHwReg rpr_ax_inc;
  MvdHwReg rpr_iuyl_num_y;
  MvdHwReg rpr_iuyl_num_uv;
  MvdHwReg rpr_iuyl_inc;
  MvdHwReg rpr_options_addr;
  MvdHwReg csc_status;
  MvdHwReg csc_ctrl_start;
  MvdHwReg csc_fs_size;
  MvdHwReg csc_fs_idc;
  MvdHwReg csc_fs_stride;
  MvdHwReg rpr_rd_fs_stride;
  MvdHwReg rpr_wr_fs_stride;
  MvdHwReg dpv_status;
  MvdHwReg dpv_ctrl_start;
  MvdHwReg dpv_fs_size;
  MvdHwReg dpv_init_val;
  MvdHwReg dpv_crc_val;

  // approx 1KB space not used (1KB - rpr register)
  // MsdHwReg unused_0x90[240];

} MvdHwRegRprMap;

/////////////////////////////////////////////////////////////////
// RC4 Decryption Reg Map
//

typedef struct MvdHwRegRC4MapTag
{

  MvdHwReg Key0Word0;
  MvdHwReg Key0Word1;
  MvdHwReg Key0Word2;
  MvdHwReg Key0Word3;
  MvdHwReg Key1Word0;
  MvdHwReg Key1Word1;
  MvdHwReg Key1Word2;
  MvdHwReg Key1Word3;
  MvdHwReg RC4Enable;
  MvdHwReg unused_rc4_0[12-9];
  MvdHwReg RC4Ctx0;
  MvdHwReg RC4Ctx1;
  MvdHwReg RC4Ctx2;
  MvdHwReg unused_rc4_1;

} MvdHwRegRC4Map;


/////////////////////////////////////////////////////////////////
// Black Bar Detector Reg Map
//
/*-- BBD registers (common to all formats)                            */

typedef struct MvdHwRegBbdMapTag
{
  MvdHwReg bbd_cfg_ctrl;
  MvdHwReg bbd_logo_width;
  MvdHwReg bbd_pix_thr;
  MvdHwReg bbd_s_thr_row;
  MvdHwReg bbd_p_thr_row;
  MvdHwReg bbd_s_thr_logo_row;
  MvdHwReg bbd_p_thr_logo_row;
  MvdHwReg bbd_s_thr_col;
  MvdHwReg bbd_p_thr_col;
  MvdHwReg bbd_chr_thr;
  MvdHwReg bbd_excl_win_mb;
  MvdHwReg unused;
  MvdHwReg bbd_hor_active;
  MvdHwReg bbd_ver_active;
  MvdHwReg bbd_logo_active;
  MvdHwReg bbd_botprev_active;
  MvdHwReg bbd_min_col_proj;
  MvdHwReg bbd_min_row_proj;


} MvdHwRegBbdMap;

/////////////////////////////////////////////////////////////////
// Debug Reg Map
//

typedef struct MvdHwRegDbgMapTag
{
  // 0x0 --> 0x1f
  union
  {
    MvdHwReg msk0_lo;
    MvdHwReg fifo0;
    MvdHwReg status0;
  } fifo0_up;
  union
  {
    MvdHwReg msk0_hi;
    MvdHwReg fifo1;
    MvdHwReg status1;
  } fifo1_up;
  union
  {
    MvdHwReg msk1_lo;
    MvdHwReg fifo2;
    MvdHwReg status2;
  } fifo2_up;
  union
  {
    MvdHwReg msk1_hi;
    MvdHwReg fifo3;
    MvdHwReg status3;
  } fifo3_up;
  union
  {
    MvdHwReg msk2_lo;
    MvdHwReg fifo4;
    MvdHwReg status4;
  } fifo4_up;
  union
  {
    MvdHwReg msk2_hi;
    MvdHwReg fifo5;
    MvdHwReg status5;
  } fifo5_up;
  union
  {
    MvdHwReg mode;
    MvdHwReg fifo6;
    MvdHwReg status6;
  } fifo6_up;
  union
  {
    MvdHwReg unused2;
    MvdHwReg fifo7;
    MvdHwReg status7;
  } fifo7_up;
  union
  {
    MvdHwReg crc0;
    MvdHwReg fifo8;
    MvdHwReg status8;
  } crc0_up;
  union
  {
    MvdHwReg crc1;
    MvdHwReg fifo9;
    MvdHwReg status9;
  } crc1_up;
  union
  {
    MvdHwReg crc2;
    MvdHwReg fifo10;
    MvdHwReg status10;
  } crc2_up;
  union
  {
    MvdHwReg crc3;
    MvdHwReg fifo11;
    MvdHwReg status11;
  } crc3_up;
  union
  {
    MvdHwReg crc4;
    MvdHwReg fifo12;
    MvdHwReg status12;
  } crc4_up;
  union
  {
    MvdHwReg crc5;
    MvdHwReg fifo13;
    MvdHwReg status13;
  } crc5_up;
  union
  {
    MvdHwReg crc6;
    MvdHwReg fifo14;
    MvdHwReg status14;
  } crc6_up;
  union
  {
    MvdHwReg crc7;
    MvdHwReg clr_crc;
    MvdHwReg fifo15;
    MvdHwReg status15;
  } crc7_up;
  union
  {
    MvdHwReg crc8;
    MvdHwReg fifo16;
    MvdHwReg status16;
  } crc8_up;
  union
  {
    MvdHwReg crc9;
    MvdHwReg fifo17;
    MvdHwReg status17;
  } crc9_up;
  union
  {
    MvdHwReg crc10;
  } crc10_up;
  union
  {
    MvdHwReg crc11;
  } crc11_up;

  MvdHwReg crc12;
  MvdHwReg crc13;
  MvdHwReg crc14;
  MvdHwReg dbg_unused_31to23[9];

} MvdHwRegDbgMap;

/////////////////////////////////////////////////////////////////
//Command Queues Reg Map
//

typedef struct MvdHwRegCqMapTag
{

  MvdHwReg cq_status;
  MvdHwReg cq_grp;
  MvdHwReg cq_clr;
  MvdHwReg cq_data;
  MvdHwReg cq_data_last;
  MvdHwReg cq_data_mask;
  MvdHwReg cq_data_test;
  MvdHwReg cq_opts;
  MvdHwReg cq_push;
  MvdHwReg cq_dummy;
  MvdHwReg cq_cmdq_base;
  MvdHwReg cq_cmdq_amax;
  MvdHwReg cq_cmdq_wptr;
  MvdHwReg cq_cmdq_rptr;
  MvdHwReg cq_cmdq_level;
  MvdHwReg cq_rdq_base;
  MvdHwReg cq_rdq_amax;
  MvdHwReg cq_rdq_wptr;
  MvdHwReg cq_rdq_rptr;
  MvdHwReg cq_rdq_level;
  MvdHwReg cq_poll_param;
  MvdHwReg cq_grp_level;
  MvdHwReg cq_exec;
  MvdHwReg cq_lwm;
  MvdHwReg cq_dataq_base;
  MvdHwReg cq_dataq_amax;
  MvdHwReg cq_dataq_rptr;
  MvdHwReg cq_special_en;
  MvdHwReg cq_wdata_opt;
  MvdHwReg cq_sgrp_base_addr;
  MvdHwReg cq_sgrp_num_cmds;
  MvdHwReg cq_status2;

} MvdHwRegCqMap;


/////////////////////////////////////////////////////////////////
// Stream Preparser Reg Map
//

typedef struct MvdHwRegSppMapTag
{
  // 0x0 --> 0x07
  MvdHwReg bbb_showbits;        // R
  MvdHwReg bbb_status;          // R
  MvdHwReg config;              // W/R
  MvdHwReg bs2rbsp_status;      // R
  MvdHwReg bs2rbsp_feed_control;// W/R
  MvdHwReg bs2rbsp_scode;       // R
  MvdHwReg bs2rbsp_scdctrl;     // W/R
  MvdHwReg bbb_crc;             // R

  // 0x08 --> 0x0f
  MvdHwReg pes_setup;           // W/R
  MvdHwReg pes_status;          // W/R
  MvdHwReg pes_ctrl;            // W
  MvdHwReg pes_set_state;       // W
  MvdHwReg pes_peek_data;       // R
  MvdHwReg pes_read_data;       // R
  MvdHwReg pes_pts;             // R
  MvdHwReg pes_dts;             // R

  // 0x10 --> 0x1f
  MvdHwReg bsdma_command;
  MvdHwReg bsdma_options;
  MvdHwReg bsdma_status;
  MvdHwReg bsdma_des;
  MvdHwReg bsdma_bwp;
  MvdHwReg bsdma_brp;
  MvdHwReg bsdma_bsa;
  MvdHwReg bsdma_bea;
  MvdHwReg bsdma_blw;
  MvdHwReg bsdma_bup;
  MvdHwReg bsdma_peek;
  MvdHwReg bsdma_bhw;
  MvdHwReg bsdma_trc;
  MvdHwReg bsdma_ext_options;
  MvdHwReg bsdma_lvl;
  MvdHwReg unused_0x1f;

  // 0x20 --> 0x3f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;

  // 0x40 --> 0x4f
  MvdHwReg egd_ue;
  MvdHwReg egd_se;
  MvdHwReg egd_me_intra;
  MvdHwReg egd_me_inter;
  MvdHwReg data_reg;            /* R/W */
  MvdHwReg num_bits_pulled;     /* R   */
  MvdHwReg bit_puller;
  MvdHwReg unused_spp1[16-7];

  // 0x50 --> 0x5f
  MvdHwReg Key0Word0;
  MvdHwReg Key0Word1;
  MvdHwReg Key0Word2;
  MvdHwReg Key0Word3;
  MvdHwReg Key1Word0;
  MvdHwReg Key1Word1;
  MvdHwReg Key1Word2;
  MvdHwReg Key1Word3;
  MvdHwReg RC4Enable;
  MvdHwReg unused_rc4_0[12-9];
  MvdHwReg RC4Ctx0;
  MvdHwReg RC4Ctx1;
  MvdHwReg RC4Ctx2;
  MvdHwReg unused_rc4_1;

} MvdHwRegSppMap;

/////////////////////////////////////////////////////////////////
// Stream Frame Buffer Compression Reg Map
//

typedef struct MvdHwRegFbcMapTag
{
  // 0x0 --> 0x0f
  MvdHwReg fbc_start;
  MvdHwReg fbc_config;
  MvdHwReg fbc_param1;
  MvdHwReg fbc_param2;
  MvdHwReg fbc_ctb_tile_px4;
  MvdHwReg fbc_ctb_tile_py4;
  MvdHwReg fbc_ctb_tile_px8;
  MvdHwReg fbc_ctb_tile_py8;
  MvdHwReg fbc_ctb_tile_px9_y10;
  MvdHwReg fbc_ot_offset;
  MvdHwReg fbc_field_offset;
  MvdHwReg fbc_uv_offset;
  MvdHwReg fbc_field_list;
  MvdHwReg fbc_luma_pad_list;
  MvdHwReg fbc_chroma_pad_list;
  MvdHwReg fbc_write_count;
  MvdHwReg fbc_read_count_0;
  MvdHwReg fbc_read_count_1;
  MvdHwReg fbc_write_chksum;
  MvdHwReg fbc_read_chksum_0;
  MvdHwReg fbc_read_chksum_1;
  MvdHwReg fbc_param3;
  MvdHwReg fbc_fs_height;
  MvdHwReg fbc_read_dbg_gen;
  MvdHwReg fbc_read_dbg_blk;
  MvdHwReg fbd_ot_offset;
  MvdHwReg fbd_fld_offset;
  MvdHwReg fbd_uv_offset;
  MvdHwReg fbc_unused[62-28];
  MvdHwReg unused_fbc_0[2];
} MvdHwRegFbcMap;


/////////////////////////////////////////////////////////////////
// Decoupled Front End Register Map
// ( 1K incl debug registers )

typedef struct MvdHwRegDfeMapTag
{
  // 0x0 --> 0x07
  MvdHwReg bbb_showbits;        // R
  MvdHwReg bbb_status;          // R
  MvdHwReg config;              // W/R
  MvdHwReg bs2rbsp_status;      // R
  MvdHwReg bs2rbsp_feed_control;// W/R
  MvdHwReg bs2rbsp_scode;       // R
  MvdHwReg bs2rbsp_scdctrl;     // W/R
  MvdHwReg bbb_crc;             // R

  // 0x08 --> 0x0f
  MvdHwReg pes_setup;           // W/R
  MvdHwReg pes_status;          // W/R
  MvdHwReg pes_ctrl;            // W
  MvdHwReg pes_set_state;       // W
  MvdHwReg pes_peek_data;       // R
  MvdHwReg pes_read_data;       // R
  MvdHwReg pes_pts;             // R
  MvdHwReg pes_dts;             // R

  // 0x10 --> 0x1f
  MvdHwReg bsdma_command;
  MvdHwReg bsdma_options;
  MvdHwReg bsdma_status;
  MvdHwReg bsdma_des;
  MvdHwReg bsdma_bwp;
  MvdHwReg bsdma_brp;
  MvdHwReg bsdma_bsa;
  MvdHwReg bsdma_bea;
  MvdHwReg bsdma_blw;
  MvdHwReg bsdma_bup;
  MvdHwReg bsdma_peek;
  MvdHwReg bsdma_bhw;
  MvdHwReg bsdma_trc;
  MvdHwReg bsdma_ext_options;
  MvdHwReg bsdma_lvl;
  MvdHwReg unused_0x1f;

  /* BSD address, 0x20 --> 0x5f */
  MvdHwReg bsd_options;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_ctb_coord;
  MvdHwReg bsd_slice_start;
  MvdHwReg bsd_dec_param1;
  MvdHwReg bsd_dec_param2;
  MvdHwReg bsd_dec_ctb_cnt;
  MvdHwReg bsd_dec_err_ctb_cnt;
  MvdHwReg bsd_dec_ctb_qp_sum;

  MvdHwReg Unused_9;
  MvdHwReg Unused_A;

  MvdHwReg bsd_ue;
  MvdHwReg bsd_se;
  MvdHwReg Unused_D;
  MvdHwReg Unused_E;
  MvdHwReg bsd_data;

  MvdHwReg Unused_10;
  MvdHwReg bsd_ctb_tile_px4;
  MvdHwReg bsd_ctb_tile_py4;
  MvdHwReg bsd_ctb_tile_px8;
  MvdHwReg bsd_ctb_tile_py8;
  MvdHwReg bsd_ctb_tile_px9_y10;
  MvdHwReg bsd_sc_list_usage;
  MvdHwReg bsd_sc_list_base;
  MvdHwReg bsd_sc_list_idx[4];
  MvdHwReg bsd_ei_flags_mask;
  MvdHwReg bsd_bit_puller;
  MvdHwReg bsd_is_trailing;
  MvdHwReg bsd_bbb_status;

  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;

  /* 0x60 --> 0x6f */
  MvdHwReg Key0Word0;
  MvdHwReg Key0Word1;
  MvdHwReg Key0Word2;
  MvdHwReg Key0Word3;
  MvdHwReg Key1Word0;
  MvdHwReg Key1Word1;
  MvdHwReg Key1Word2;
  MvdHwReg Key1Word3;
  MvdHwReg RC4Enable;
  MvdHwReg unused_rc4_0[12-9];
  MvdHwReg RC4Ctx0;
  MvdHwReg RC4Ctx1;
  MvdHwReg RC4Ctx2;
  MvdHwReg unused_rc4_1;

  /* 0x70-0x7f */
  MvdHwReg dfe_ctx_cmd;
  MvdHwReg unused_dfe_ctx_1;
  MvdHwReg dfe_ctx_addr_bbb[2];     /* Active BBB                          */
  MvdHwReg unused_dfe_ctx_2[12];

  /* 0x80 --> 0x8f */
  MvdHwReg dfe_ctx_addr_bsi[13];    /* SCD/PES/FIFO                        */
  MvdHwReg unused_dfe_ctx_3[3];

  /* 0x90 --> 0x9F */
  MvdHwReg dfe_img_cfg;
  MvdHwReg dfe_slice_cfg;
  MvdHwReg dfe_frm_size;
  MvdHwReg dfe_status;
  MvdHwReg dfe_frm_act_count;
  MvdHwReg dfe_slice_act_count;
  MvdHwReg dfe_rbsp_bytes_count;
  MvdHwReg dfe_sib_wait_count;
  MvdHwReg dfe_bsd_wait_count;
  MvdHwReg unused_dfe[16-9];

  /* 0xA0 -> 0xBF */
  MvdHwRegCqMap DFECQ;

  /* 0xC0 -> 0xDF */
  MvdHwReg unused_0xc0_0xdf[32];

  /* 0xE0 -> 0xFF */
  MvdHwReg dfe_dbg[32];

} MvdHwRegDfeMap;

/////////////////////////////////////////////////////////////////
// Decoupled Back End Register Map
// ( 4K incl debug registers )

typedef struct MvdHwRegDbeMapTag
{

  /* TOP address, 0x00 --> 0x3f */
  MvdHwReg image_config;            // R/W
  MvdHwReg slice_config;            // R/W
  MvdHwReg top_ctb_tile_info;       // R/W
  MvdHwReg mbi_wr_base;             // R/W
  MvdHwReg mbi_rd_base;             // R/W

  MvdHwReg dec_cfg;                 // R/W             Addr in CQ : 0x605
  MvdHwReg dbg_cfg;                 // R/W
  MvdHwReg dbf1_cfg;                // R/W
  MvdHwReg dbf2_cfg;                // R/W
  MvdHwReg dbe_frm_size;            // R/W
  MvdHwReg prb_pack_base;           // R/W
  MvdHwReg prb_bin_cfg;             // R/W
  MvdHwReg dcp_bin_cfg;             // R/W

  MvdHwReg unused_top[64-13];

  /* DBE address, 0x40 --> 0x7F */
  MvdHwReg dbe_status;                 // R
  MvdHwReg dbe_frm_act_count;          // R
  MvdHwReg dbe_slice_act_count;        // R
  MvdHwReg dbe_wait_count;             // R
  MvdHwReg dbe_fetch_crc;              // R
  MvdHwReg dbe_mpr_prx_wait;           // R
  MvdHwReg dbe_pxd_prx_wait;           // R
  MvdHwReg dbe_fch_plq_wait;           // R
  MvdHwReg dbe_pxd_plq_wait;           // R
  MvdHwReg dbe_mpr_wait_count;         // R
  MvdHwReg dbe_fch_words_count;        // R
  MvdHwReg unused_dbe[64-11];

  /* MPR address, 0x80 --> 0xBF */
  MvdHwReg mpr_config;
  MvdHwReg mpr_slice_param;

  MvdHwReg Unused_82_9F[30];

  MvdHwReg mpr_list[32];

  MvdHwReg Unused_0xC0_FF[64];

  /* CQ address, 0x100 --> 0x11F */
  MvdHwRegCqMap DBECQ;
  MvdHwReg Unused_0x120_13F[32];

  MvdHwReg Unused_0x140_17F[64];

  MvdHwReg mpr_dbg;
  MvdHwReg pxd_dbg;
  MvdHwReg dbf_dbg;
  
  /* Not quite sure what debug is in here */
  MvdHwReg Unused_DbeDbg[61];

  MvdHwReg Unused_0x1C0_1FF[64];

} MvdHwRegDbeMap;


/////////////////////////////////////////////////////////////////
// H.264 Reg Map
//

typedef struct MvdHwRegH264MapTag
{
  //------- TOP address, 0x00 --> 0x3f
  MvdHwReg options;                 // R/W
  MvdHwReg mbi_wr_base;             // R/W
  MvdHwReg mbi_rd_base;             // R/W
  MvdHwReg bli_wr_base;             // R/W
  MvdHwReg bli_wr_stride;           // R/W
  MvdHwReg unused_top[64-5];
  //------- BSD address, 0x40 --> 0x7f
  // 0x40 --> 0x4f
  MvdHwReg bsd_dec_options;         // W/R
  MvdHwReg bsd_dec_status;          // R
  MvdHwReg bsd_curr_coord;
  MvdHwReg bsd_slice_start;
  MvdHwReg bsd_slice_param1;
  MvdHwReg bsd_slice_param2;
  MvdHwReg qm_dec_start;
  MvdHwReg qm_dec_status;
  MvdHwReg qm_load_start;
  MvdHwReg qm_load_value;
  MvdHwReg bsd_pwt_dec;             // W
  MvdHwReg bsd_egd_ue;
  MvdHwReg bsd_egd_se;
  MvdHwReg bsd_egd_me_intra;
  MvdHwReg bsd_egd_me_inter;
  MvdHwReg bsd_data_reg;            // R/W
  // 0x50 --> 0x5f
  MvdHwReg ibb_buf_mode;
  MvdHwReg ibb_buf_ctrl;
  MvdHwReg ibb_buf_status;
  MvdHwReg bsd_image_init;
  MvdHwReg unused_bsd_0[4];
  MvdHwReg bsd_pause;
  MvdHwReg unused_bsd[16-9-4];
  MvdHwReg bsd_err_mask;
  MvdHwReg bsd_bit_puller;
  MvdHwReg bsd_is_trailing;
  MvdHwReg bsd_bbb_status;          // R
  // 0x60 --> 0x7f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;
  //------- MPR/PXD address
  // 0x80 --> 0x9f
  MvdHwReg mpr_top_poc;
  MvdHwReg mpr_bot_poc;
  MvdHwReg unused_mpr[32-2];
  // 0xa0 --> 0xbf
  MvdHwReg mpr_list[32];
  //------- DBF address
  // 0xc0 --> 0xff
  MvdHwReg unused_dbf[64];
  // 0x100 --> 0x1bf not used
  MvdHwReg unsued_spare_128[128];
  MvdHwReg bsd_debug;
  MvdHwReg pxd_debug;
  MvdHwReg arb_wr_debug;
  MvdHwReg arb_rd_debug;
  MvdHwReg arb_mbi_debug;
  MvdHwReg unsued_spare[59];

} MvdHwRegH264Map;

/////////////////////////////////////////////////////////////////
// VC-1 Reg Map
//

typedef struct MvdHwRegVc1dMapTag
{
  //------- TOP address, 0x00 --> 0x3f
  MvdHwReg unused_gen64[64];
  //-- BSP address, 0x40 --> 0x7f
  // 0x40 --> 0x5f
  union
  {
    MvdHwReg bsp_byte_align;
    MvdHwReg bsp_getbits[32];
  } get_bits;
  // 0x60 --> 0x6f
  MvdHwReg bsp_status;
  MvdHwReg bsp_control;
  MvdHwReg bsp_showbits;
  MvdHwReg bsp_showbitsflipped;
  MvdHwReg bsp_ignorebbblevel;
  MvdHwReg unused_bsp11[11];
  // 0x70 --> 0x7f
  MvdHwReg bsp_hdrvlc_ptype;
  MvdHwReg bsp_hdrvlc_pptype;
  MvdHwReg bsp_hdrvlc_mvrange;
  MvdHwReg bsp_hdrvlc_mvmode;
  MvdHwReg bsp_hdrvlc_mvmode2;
  MvdHwReg bsp_hdrvlc_dmvrange;
  MvdHwReg bsp_hdrvlc_bppmode;
  MvdHwReg bsp_hdrvlc_bppvlc2;
  MvdHwReg bsp_hdrvlc_bppvlc6;
  MvdHwReg bsp_hdrvlc_bfract;
  MvdHwReg bsp_hdrvlc_refdist;
  MvdHwReg unused_bsp5[5];
  // 0x80 --> 0x8f
  MvdHwReg spr_general;
  MvdHwReg spr_stream_format1;
  MvdHwReg spr_coded_size;
  MvdHwReg spr_stream_format2;
  MvdHwReg spr_entrypoint1;
  MvdHwReg spr_range_map;
  MvdHwReg spr_frame_type;
  MvdHwReg spr_recon_control;
  MvdHwReg spr_mv_control;
  MvdHwReg spr_int_comp_fwd_topnorm;
  MvdHwReg spr_ref_bfraction;
  MvdHwReg spr_blk_control;
  MvdHwReg spr_trans_data;
  MvdHwReg spr_vop_dquant;
  MvdHwReg spr_curref_frm_id;
  MvdHwReg spr_curdisp_frm_id;
  // 0x90 --> 0x9f
  MvdHwReg spr_fwdref_frm_id;
  MvdHwReg spr_bwdref_frm_id;
  MvdHwReg spr_fieldref_ctrl_id;
  MvdHwReg spr_auxfrmctrl;
  MvdHwReg spr_imgstruct;
  MvdHwReg spr_alt_frame_type;
  MvdHwReg spr_int_comp_fwd_bot;
  MvdHwReg spr_int_comp_bwd_top;
  MvdHwReg spr_int_comp_bwd_bot;
  MvdHwReg unused_spr7[7];
  // 0xa0 --> 0xbf
  MvdHwReg unused_spr32[32];
  // 0xc0 --> 0xcf
  MvdHwReg mbd_status;
  MvdHwReg mbd_frm_start;
  MvdHwReg mbd_fwdebug;
  MvdHwReg mbd_mbqdebug;
  MvdHwReg mbd_mprdebug;
  MvdHwReg mbd_masdebug;
  MvdHwReg mbd_bppdebug;
  MvdHwReg mbd_dpbmcdebug;
  MvdHwReg mbd_gendebug;
  MvdHwReg mbd_fw_rw;
  MvdHwReg unused_6[6];
  // 0xd0 --> 0xdf
  MvdHwReg mbd_error_control;
  MvdHwReg unused_mbd15[15];
  // 0xe0 --> 0xff;
  MvdHwReg unused_mbd32[32];
  //------- BPP address
  // 0x100 --> 0x10f
  MvdHwReg bpp_control_status;
  MvdHwReg bpp_datain_status;
  MvdHwReg bpp_datain_value;
  MvdHwReg bpp_datain_possize;
  MvdHwReg bpp_bcachetag0;
  MvdHwReg bpp_bcachetag1;
  MvdHwReg bpp_bcachetag2;
  MvdHwReg unused_0x107[9];
  // 0x110 --> 0x1bf not used
  MvdHwReg unsued_spare[176];

} MvdHwRegVc1dMap;

/////////////////////////////////////////////////////////////////
// MPEG-2 Reg Map
//

typedef struct MvdHwRegMp2dMapTag
{
  //------- TOP address, 0x00 --> 0x3f
  // 0x00 --> 0x3f
  MvdHwReg top_seq_param;
  MvdHwReg top_frame_ptr;
  MvdHwReg top_pic_start;
  MvdHwReg unused_top[64-3];
  //------- BSD address, 0x40 --> 0x7f
  // 0x40 --> 0x5f
  MvdHwReg bsd_show_bits;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_bbb_status;
  MvdHwReg bsd_qm_status;
  MvdHwReg bsd_qm_load;
  MvdHwReg bsd_pic_param;
  MvdHwReg bsd_slice_start;
  MvdHwReg bsd_curr_mbcoord;
  MvdHwReg unused_bsd[32-8];
  // 0x60 --> 0x7f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;
  // 0x80 --> 0xff
  MvdHwReg unused_128[128];
  //------- DBF address
  // 0x100 --> 0x17f
  MvdHwReg unused_dbf16[16];
  MvdHwReg dbf_ctrl0;
  MvdHwReg dbf_ctrl1;
  MvdHwReg dbf_ctrl2;
  MvdHwReg dbf_ctrl3;
  MvdHwReg unused_dbf[128-20];
  //------ DEBUG address
  // 0x180 --> 0x1bf
  MvdHwReg debug_bsd;
  MvdHwReg debug_pxd_idct;
  MvdHwReg debug_wr_arbiter;
  MvdHwReg debug_rd_arbiter;
  MvdHwReg unused_debug[64-4];

} MvdHwRegMp2dMap;

/////////////////////////////////////////////////////////////////
// AVS Reg Map
//

//  `define AVSD_APB_ADDR_TOP_PREFIX      3'h1  // 0x40
//  `define AVSD_APB_ADDR_BSD_PREFIX      3'h0  // 0x00
//  `define AVSD_APB_ADDR_MPR_PREFIX      3'h2  // 0x80
#define MVD_AVSD_TOP_MIN_ADDR            0x00
#define MVD_AVSD_BSD_MIN_ADDR            0x40
#define MVD_AVSD_MPR_MIN_ADDR            0x80

//-- Top level Register address
#define MVD_AVSD_TOP_SEQ_PARAM_ADDR      0x00  // W/R, Chroma422
#define MVD_AVSD_TOP_FRAME_PTR_ADDR      0x01  // W/R, cur_dec_ptr, bwd_ref_ptr, fwd_ref_ptr
#define MVD_AVSD_TOP_PICT_START_ADDR     0x02  // W/R, is_2nd_fld, top_fild_1st, coding_type, pict_struct
#define MVD_AVSD_TOP_PICT_PARAM_ADDR     0x03  // W/R

//-- BSD Register address
#define MVD_AVSD_BSD_SHOW_BITS_ADDR      0x40  // R, show available bits (up to 31), lsb aligned
#define MVD_AVSD_BSD_DEC_STATUS_ADDR     0x41  // R, returns {1'b0, vld_mb_y, 1'b0, vld_mb_x, VldState, vld_error, img_in_prog, slice_in_prog}
#define MVD_AVSD_BSD_BBB_STATUS_ADDR     0x42  // R, returns {6'd0, bits_ei_flag, bbb_underflow, 6'd0, bbb_is_stuffing, bits_ended, 10'd0, num_bits_in_bbb}
#define MVD_AVSD_BSD_SCEPB_CTRL_ADDR     0x43  // W/R, {scepb_disab, scepb_delay}
#define MVD_AVSD_BSD_EGDUE_START_ADDR    0x44  // R, start UE decode, [0]: ready in 3 ticks, [1]: out of range(dec not performed)
#define MVD_AVSD_BSD_EGDSE_START_ADDR    0x45  // R, start SE decode, [0]: ready in 3 ticks, [1]: out of range(dec not performed)
#define MVD_AVSD_BSD_SLC_START_ADDR      0x46  // W/R, {24'd0, vld_recover, vld_new_mbrow}
#define MVD_AVSD_BSD_CURR_MBCOORD_ADDR   0x47  // R/W
#define MVD_AVSD_BSD_EGD_VALUE_ADDR      0x48  // R, exp-golumn decode value
#define MVD_AVSD_BSD_ERRCHK_MASK_ADDR    0x49  // R/W
#define MVD_AVSD_BSD_ERROR_STAT_ADDR     0x4a  // R, cleard at start of a slice

#define MVD_AVSD_BSD_BYTE_ALIGN_ADDR     0x60
#define MVD_AVSD_BSD_GETBITS_MIN_ADDR    0x60  // MVD_AVSD_BSD_ALIGN_BYTE_ADDR
#define MVD_AVSD_BSD_GETBITS_MAX_ADDR    0x7f  // 31 bits

//-- MPR register address
#define MVD_AVSD_MPR_WT_PARAM0_ADDR      0x80  // R/W
#define MVD_AVSD_MPR_WT_PARAM1_ADDR      0x81  // R/W
#define MVD_AVSD_MPR_WT_PARAM2_ADDR      0x82  // R/W
#define MVD_AVSD_MPR_WT_PARAM3_ADDR      0x83  // R/W
#define MVD_AVSD_MPR_BWD_DIST_ADDR       0x84  // when P picture, use bwd0,1 as fwd0,1
#define MVD_AVSD_MPR_FWD_DIST_ADDR       0x85  // when P picture, use fwd0,1 as fwd2,3
#define MVD_AVSD_MPR_REF01_DIST_ADDR     0x86
#define MVD_AVSD_MPR_REF23_DIST_ADDR     0x87
#define MVD_AVSD_MPR_BWD_DISTIDX_ADDR    0x88  // when P picture, use bwd0,1 as fwd0,1
#define MVD_AVSD_MPR_FWD_DISTIDX_ADDR    0x89  // when P picture, use fwd0,1 as fwd2,3
#define MVD_AVSD_MPR_REF01_DISTIDX_ADDR  0x8a
#define MVD_AVSD_MPR_REF23_DISTIDX_ADDR  0x8b
#define MVD_AVSD_MPR_CONFIG_OPTS_ADDR    0x8c

typedef struct MvdHwRegAvsdMapTag {
  //------- TOP address, 0x00 --> 0x3f
  // 0x00 --> 0x3f
  MvdHwReg top_seq_param;
  MvdHwReg top_frame_ptr;
  MvdHwReg top_pict_start;
  MvdHwReg top_pict_param;
  MvdHwReg unused_top[64-4];
  //------- BSD address, 0x40 --> 0x7f
  // 0x40 --> 0x5f
  MvdHwReg bsd_show_bits;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_bbb_status;
  MvdHwReg bsd_scepb_ctrl;
  MvdHwReg bsd_egdue_start;
  MvdHwReg bsd_egdse_start;
  MvdHwReg bsd_slc_start;
  MvdHwReg bsd_curr_mbcoord;
  MvdHwReg bsd_egd_value;
  MvdHwReg bsd_errchk_mask;
  MvdHwReg bsd_error_stat;
  MvdHwReg bsd_qp_delta_uv;
  MvdHwReg bsd_wqm1_low;
  MvdHwReg bsd_wqm1_high;
  MvdHwReg bsd_wqm2_low;
  MvdHwReg bsd_wqm2_high;
  MvdHwReg bsd_qp_ctr;
  MvdHwReg unused_bsd[32-17];
  //MvdHwReg unused_bsd[32-11];
  // 0x60 --> 0x7f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;
  //------- MPR address
  // 0x80 --> 0xbf
  MvdHwReg mpr_wt_param[4];
  MvdHwReg mpr_bwd_dist;
  MvdHwReg mpr_fwd_dist;
  MvdHwReg mpr_ref01_dist;
  MvdHwReg mpr_ref23_dist;
  MvdHwReg mpr_bwd_distidx;
  MvdHwReg mpr_fwd_distidx;
  MvdHwReg mpr_ref01_distidx;
  MvdHwReg mpr_ref23_distidx;
  MvdHwReg mpr_config_opts;
  MvdHwReg unused_mpr[64-13];
  // 0xc0 --> 0x1bf not used
  MvdHwReg unsued_spare[256];

} MvdHwRegAvsdMap;

/////////////////////////////////////////////////////////////////
// ASPD/DIVX/JPEG Reg Map
//

//`define ASPD_APB_ADDR_TOP_PREFIX      3'h0     // 0x00
//`define ASPD_APB_ADDR_BSD_PREFIX      3'h1     // 0x40
//`define ASPD_APB_ADDR_MPR_PREFIX      3'h2     // 0x80
//`define ASPD_APB_ADDR_PXD_PREFIX      3'h3     // 0xc0
//`define ASPD_APB_ADDR_DBF_PREFIX      3'h4     // 0x100
#define MVD_ASPD_TOP_MIN_ADDR            0x00
#define MVD_ASPD_BSD_MIN_ADDR            0x40
#define MVD_ASPD_MPR_MIN_ADDR            0x80
#define MVD_ASPD_DBF_MIN_ADDR            0x100

//-- Top level Register address
#define MVD_ASPD_TOP_IMG_START_ADDR      0x00
#define MVD_ASPD_TOP_VO_PARAM_ADDR       0x01
#define MVD_ASPD_TOP_FRAME_PTR_ADDR      0x02   // W/R, cur_dec_ptr, bwd_ref_ptr, fwd_ref_ptr
#define MVD_ASPD_TOP_VPARAM1_ADDR        0x03
#define MVD_ASPD_TOP_VPARAM2_ADDR        0x04
#define MVD_ASPD_TOP_VPARAM3_ADDR        0x05
#define MVD_ASPD_TOP_VO_PARAM_EXT_ADDR   0x06

//-- BSD Register address
#define MVD_ASPD_BSD_SHOW_BITS_ADDR      0x40   // R, show available bits (up to 31), lsb aligned
#define MVD_ASPD_BSD_DEC_STATUS_ADDR     0x41   // R, returns {1'b0, vld_mb_y, 1'b0, vld_mb_x, VldState, vld_error, img_in_prog, slice_in_prog}
#define MVD_ASPD_BSD_BBB_STATUS_ADDR     0x42   // R, returns {6'd0, bits_ei_flag, bbb_underflow, 6'd0, bbb_is_stuffing, bits_ended, 10'd0, num_bits_in_bbb}
#define MVD_ASPD_BSD_SLC_START_ADDR      0x46   // W/R, {24'd0, vld_recover, vld_new_mbrow}
#define MVD_ASPD_BSD_CURR_MBCOORD_ADDR   0x47   // R/W
#define MVD_ASPD_BSD_OPTIONS_ADDR        0x49   // R/W

// JPEG DC Predictor registers
#define MVD_ASPB_BSD_JPEG_CTX0_ADDR      0x4a   // R/W, must be backup when doing multiple JPEG images
#define MVD_ASPB_BSD_JPEG_CTX1_ADDR      0x4b   // R/W, must be backup when doing multiple JPEG images

#define MVD_ASPD_BSD_BYTE_ALIGN_ADDR     0x60
#define MVD_ASPD_BSD_GETBITS_MIN_ADDR    0x60   // MVD_ASPD_BSD_ALIGN_BYTE_ADDR
#define MVD_ASPD_BSD_GETBITS_MAX_ADDR    0x7f   // 31 bits

//-- MPR GMC register address
#define MVD_ASPD_GMC_ATLUM_X0_ADDR       0x90   // R/W, atlum->X0, 18 bits, signed
#define MVD_ASPD_GMC_ATLUM_Y0_ADDR       0x91   // R/W, atlum->Y0, 18 bits, signed
#define MVD_ASPD_GMC_ATLUM_YXXX_ADDR     0x92   // R/W, {atlum->YX, atlum->XX}, 16 bits each, signed
#define MVD_ASPD_GMC_ATLUM_XYYY_ADDR     0x93   // R/W, {atlum->XY, atlum->YY}, 16 bits each, signed
#define MVD_ASPD_GMC_ATCHR_X0_ADDR       0x94   // R/W, atchr->X0, 28 bits, signed
#define MVD_ASPD_GMC_ATCHR_Y0_ADDR       0x95   // R/W, atchr->Y0, 28 bits, signed
#define MVD_ASPD_GMC_ATCHR_YXXX_ADDR     0x96   // R/W, {atchr->YX, atchr->XX}, 16 bits each, signed
#define MVD_ASPD_GMC_ATCHR_XYYY_ADDR     0x97   // R/W, {atchr->XY, atchr->YY}, 16 bits each, signed
#define MVD_ASPD_GMC_PARAM_ADDR          0x98   // R/W, {5'd0, fcode_gmc, 6'd0, warp_accuracy, 3'd0, atchr_shift, 3'd0, atlum_shift}

#define MVD_ASPD_MPR_OPTS_ADDR           0x99   // R/W  {26'b0,mpr_opts}

#define ASPD_PXD_STATUS_ADDR             0xc0
#define ASPD_PXD_QM_CTRL_ADDR            0xc1
#define ASPD_PXD_WR_QMDATA_ADDR          0xc2

//-- DBF registers -- these are the same as those for MP2D
#define MVD_ASPD_DBF_CTRL0_ADDR          0x110  // Kevin Lim's POSTP_DEBLK_CTRL0
#define MVD_ASPD_DBF_CTRL1_ADDR          0x111  // Kevin Lim's POSTP_DEBLK_CTRL1
#define MVD_ASPD_DBF_CTRL2_ADDR          0x112  // Kevin Lim's POSTP_DEBLK_CTRL2
#define MVD_ASPD_DBF_CTRL3_ADDR          0x113  // Kevin Lim's POSTP_DEBLK_CTRL3

typedef struct MvdHwRegAspdMapTag
{
  //------- TOP address, 0x00 --> 0x3f
  // 0x00 --> 0x3f
  MvdHwReg top_img_start;
  MvdHwReg top_vo_param;
  MvdHwReg top_frame_ptr;
  MvdHwReg top_vparam1;
  MvdHwReg top_vparam2;
  MvdHwReg top_vparam3;
  MvdHwReg top_vo_param_ext;
  MvdHwReg top_vparam4;
  MvdHwReg top_vparam5;
  MvdHwReg unused_top[64-9];
  //------- BSD address, 0x40 --> 0x7f
  // 0x40 --> 0x5f
  MvdHwReg bsd_show_bits;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_bbb_status;
  MvdHwReg bsd_scepb_ctrl;
  MvdHwReg unused_0x04;
  MvdHwReg unused_0x05;
  MvdHwReg bsd_slc_start;
  MvdHwReg bsd_curr_mbcoord;
  MvdHwReg unused_0x08;
  MvdHwReg bsd_options;
  MvdHwReg bsd_jpeg_ctx0;
  MvdHwReg bsd_jpeg_ctx1;
  MvdHwReg unused_bsd[32-12];
  // 0x60 --> 0x7f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;
  //------- MPR/PXD/GMC address
  // 0x80 --> 0xbf
  MvdHwReg unused_mpr16[16];
  MvdHwReg gmc_atlum_x0;
  MvdHwReg gmc_atlum_y0;
  MvdHwReg gmc_atlum_yxxx;
  MvdHwReg gmc_atlum_xyyy;
  MvdHwReg gmc_atchr_x0;
  MvdHwReg gmc_atchr_y0;
  MvdHwReg gmc_atchr_yxxx;
  MvdHwReg gmc_atchr_xyyy;
  MvdHwReg gmc_param;
  MvdHwReg mpr_opts;
  MvdHwReg unused_mpr38[64-16-10];
  // 0xc0 --> 0xff
  MvdHwReg pxd_qm_status;
  MvdHwReg pxd_qm_ctrl;
  MvdHwReg pxd_qm_data;
  MvdHwReg unused_pxd61[61];
  //------- DBF address
  // 0x100 --> 0x17f
  MvdHwReg unused_dbf16[16];
  MvdHwReg dbf_ctrl0;
  MvdHwReg dbf_ctrl1;
  MvdHwReg dbf_ctrl2;
  MvdHwReg dbf_ctrl3;
  MvdHwReg unused_dbf108[108];
  // 0x180 --> 0x1bf not used
  MvdHwReg dbg_a;
  MvdHwReg dbg_b;
  MvdHwReg dbg_arb_wr;
  MvdHwReg dbg_arb_rd;
  MvdHwReg dbg_arb_mbi;
  MvdHwReg unsued_spare[59];

} MvdHwRegAspdMap;

/////////////////////////////////////////////////////////////////
// On2 Reg Map
//

typedef struct MvdHwRegOn2dMapTag {
  //------- TOP address, 0x00 --> 0x3f
  // 0x00 --> 0x3f
  MvdHwReg top_img_start;
  MvdHwReg top_param1;
  MvdHwReg top_frame_ptr;
  MvdHwReg top_param2;
  MvdHwReg top_param3;
  MvdHwReg top_param4;
  MvdHwReg top_param5;
  MvdHwReg top_param6;
  MvdHwReg top_param7;
  MvdHwReg top_param8;
  MvdHwReg top_param9;
  MvdHwReg top_param10;
  MvdHwReg top_param11;
  MvdHwReg top_param12;
  MvdHwReg top_param13;
  MvdHwReg top_param14;
  MvdHwReg top_param15;
  MvdHwReg top_param16;
  MvdHwReg top_param17;
  MvdHwReg top_param18;
  MvdHwReg top_param19;
  MvdHwReg top_param20;
  MvdHwReg top_param21;
  MvdHwReg top_param22;
  MvdHwReg top_param23;
  MvdHwReg top_param24;
  MvdHwReg top_param25;
  MvdHwReg top_param26;
  MvdHwReg unused_top[64-28];

  //------- BSD address, 0x40 --> 0x7f
  // 0x40 --> 0x5f
  MvdHwReg bsd_show_bits;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_bbb_status;
  MvdHwReg unused_0x03;
  MvdHwReg bsd_mbd_status;
  MvdHwReg unused_0x05;
  MvdHwReg bsd_slc_start;
  MvdHwReg bsd_curr_mbcoord;
  MvdHwReg unused_0x08;
  MvdHwReg bsd_options;
  MvdHwReg bsd_arith_config;
  MvdHwReg unused_0x0B;
  MvdHwReg bsd_scan_start;
  MvdHwReg bsd_scan_data;
  MvdHwReg bsd_prob_start;
  MvdHwReg bsd_prob_data;
  MvdHwReg bsd_prob_status;
  MvdHwReg unused_0x11;
  MvdHwReg unused_0x12;
  MvdHwReg unused_0x13;
  MvdHwReg unused_0x14;
  MvdHwReg unused_0x15;
  MvdHwReg unused_0x16;
  MvdHwReg unused_0x17;
  MvdHwReg debug_status;
  MvdHwReg debug_data0;
  MvdHwReg debug_data1;
  MvdHwReg unused_bsd[32-27];
  // 0x60 --> 0x7f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;
  // the rest are not used
} MvdHwRegOn2dMap;

/////////////////////////////////////////////////////////////////
// RVid Reg Map
//

typedef struct MvdHwRegRvidMapTag
{
  //------- TOP address, 0x00 --> 0x3f
  // 0x00 --> 0x3f
  MvdHwReg top_img_start;
  MvdHwReg top_param1;
  MvdHwReg top_frame_ptr;
  MvdHwReg top_param2;
  MvdHwReg top_param3;
  MvdHwReg top_param4;
  MvdHwReg unused_top[64-6];

  //------- BSD address, 0x40 --> 0x7f
  // 0x40 --> 0x5f
  MvdHwReg bsd_show_bits;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_bbb_status;
  MvdHwReg unused_0x03;
  MvdHwReg bsd_mbd_status;
  MvdHwReg unused_0x05;
  MvdHwReg bsd_slc_start;
  MvdHwReg bsd_curr_mbcoord;
  MvdHwReg unused_0x08;
  MvdHwReg bsd_options;
  MvdHwReg bsd_rsb_missing;
  MvdHwReg bsd_mb_qp;
  MvdHwReg unused_bsd[32-12];
  // 0x60 --> 0x7f
  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;
  // the rest are not used
} MvdHwRegRvidMap;

/////////////////////////////////////////////////////////////////
// HEVC Reg Map
//

typedef struct MvdHwRegHevcMapTag
{

  /* TOP address, 0x00 --> 0x3f */
  MvdHwReg image_config;            // R/W
  MvdHwReg slice_config;            // R/W
  MvdHwReg top_ctb_tile_info;       // R/W
  MvdHwReg mbi_wr_base;             // R/W
  MvdHwReg mbi_rd_base;             // R/W

  MvdHwReg dec_cfg;                 // R/W
  MvdHwReg dbg_cfg;                 // R/W
  MvdHwReg dbf1_cfg;                // R/W
  MvdHwReg dbf2_cfg;                // R/W
  MvdHwReg dbe_frm_size;            // R/W
  MvdHwReg prb_pack_base;           // R/W
  MvdHwReg prb_bin_cfg;             // R/W
  MvdHwReg dcp_bin_cfg;             // R/W

  MvdHwReg unused_top[64-13];

  /* BSD address, 0x40 --> 0x7f */
  MvdHwReg bsd_options;
  MvdHwReg bsd_dec_status;
  MvdHwReg bsd_ctb_coord;
  MvdHwReg bsd_slice_start;
  MvdHwReg bsd_dec_param1;
  MvdHwReg bsd_dec_param2;
  MvdHwReg bsd_dec_ctb_cnt;
  MvdHwReg bsd_dec_err_ctb_cnt;
  MvdHwReg bsd_dec_ctb_qp_sum;

  MvdHwReg bsd_plq_levels;
  MvdHwReg Unused_A;

  MvdHwReg bsd_ue;
  MvdHwReg bsd_se;
  MvdHwReg Unused_D;
  MvdHwReg Unused_E;
  MvdHwReg bsd_data;

  MvdHwReg Unused_10;
  MvdHwReg bsd_ctb_tile_px4;
  MvdHwReg bsd_ctb_tile_py4;
  MvdHwReg bsd_ctb_tile_px8;
  MvdHwReg bsd_ctb_tile_py8;
  MvdHwReg bsd_ctb_tile_px9_y10;
  MvdHwReg bsd_sc_list_usage;
  MvdHwReg bsd_sc_list_base;
  MvdHwReg bsd_sc_list_idx[4];
  MvdHwReg bsd_ei_flags_mask;
  MvdHwReg bsd_bit_puller;
  MvdHwReg bsd_is_trailing;
  MvdHwReg bsd_bbb_status;

  union
  {
    MvdHwReg bsd_byte_align;
    MvdHwReg bsd_getbits[32];
  } get_bits;

  /* MPR address, 0x80 --> 0xBF */
  MvdHwReg mpr_config;
  MvdHwReg mpr_slice_param;

  MvdHwReg Unused_82_9F[30];

  MvdHwReg mpr_list[32];

  MvdHwReg Unused_0xC0_FF[64];

  /* dfe address, 0x100 --> 0x103 */
  MvdHwReg dfe_status;
  MvdHwReg dfe_num_pr_bits;
  MvdHwReg dfe_row_crc;
  MvdHwReg dfe_num_bins_used;
  MvdHwReg Unused_DFE[60];

  MvdHwReg Unused_0x140_17F[64];

  MvdHwReg mpr_dbg;
  MvdHwReg pxd_dbg;
  MvdHwReg dbf_dbg;
  MvdHwReg wr_arb_dbg; 
  MvdHwReg rd_arb_dbg;
  MvdHwReg mbi_rd_arb_dbg;
  MvdHwReg mbi_wr_arb_dbg;
  MvdHwReg Unused_DBG[64-7];

} MvdHwRegHevcMap;

///////////////////////////////////////////////////////////////////////////////////////
// Top address mapping struct
//
// The base decoder address to be passed should be that of the HIF registers
// not the RSB registers
//
// Minor non-backwards compatible address map change between Kronos
// and Krome...

/* *********************************************************** */
/* Kronos                                                      */
/*                                                             */
/* MvdHwReg       hif[1024];                                   */
/* MvdHwReg       sif[128];                                    */
/* MvdHwReg       ctx[128];                                    */
/* MvdHwReg       rpr[32];                                     */
/* MvdHwReg       unused[32];                                  */
/* MvdHwReg       bbd[32];       -- BL only --                 */
/* MvdHwReg       dbg[32];       -- EL only --                 */
/* MvdHwReg       spp[128];                                    */
/* MvdHwReg       dec[512-64];                                 */
/*                                                             */
/* *********************************************************** */

/* *********************************************************** */
/* Krome                                                       */
/*                                                             */
/* Base Malone:                    Enhancement Malone          */
/*                                                             */
/* MvdHwReg       hif[1024];       MvdHwReg       hif[1024];   */
/* MvdHwReg       sif[128];        MvdHwReg       sif[128];    */
/* MvdHwReg       ctx[128];        MvdHwReg       ctx[128];    */
/* MvdHwReg       rpr[32];         MvdHwReg       rpr[32];     */
/* MvdHwReg       unused[32];      MvdHwReg       unused[32];  */
/* MvdHwReg       bbd[32];         MvdHwReg       dbg[32];     */
/* MvdHwReg       cq[32];          MvdHwReg       cq[32];      */
/* MvdHwReg       spp[128];        MvdHwReg       spp[128];    */
/* MvdHwReg       dec[512-64];     MvdHwReg       dec[512-64]; */
/*                                                             */
/* *********************************************************** */

typedef struct MvdHwRegMapTag
{
  //------------- 8KB MSD address -------------
  /* 4KB HIF address space */
  union
  {
    MvdHwReg       hif[1024];
    MvdHwRegHifMap hif_map;
  } HifMap;

  /* 0.5KB SIF address space */
  union
  {
    MvdHwReg       sif[128];
    MvdHwRegSifMap sif_map;
  } SifMap;

  /* 0.5KB CTX address space */
  union
  {
    MvdHwReg       ctx[128];
    MvdHwRegCtxMap ctx_map;
  } CtxMap;

  /* 128 B RPR address space */
  union
  {
    MvdHwReg       rpr[32];
    MvdHwRegRprMap rpr_map;
  } RprMap;

  /* 128 B RC4 address space */
  union
  {
    MvdHwReg       RC4[32];
    MvdHwRegRC4Map RC4_map;
  } RC4Map;

  /* 128 B DBG address space */
  union
  {
    MvdHwReg       dbg[32];
    MvdHwRegDbgMap dbg_map;
  } DbgMap;

  /* 128 B CQ address space */
  union
  {
    MvdHwReg       cq[32];
    MvdHwRegCqMap  cq_map;
    MvdHwRegDbgMap dbg_map;
  } CqMap;

  /* 0.5KB SPP address space */
  union
  {
    MvdHwReg       spp[128];
    MvdHwRegSppMap spp_map;
  } SppMap;

  /* 1.75KB decoder address space */
  union
  {
    MvdHwReg        dec[512-64];
    MvdHwRegH264Map h264_map;
    MvdHwRegVc1dMap vc1d_map;
    MvdHwRegMp2dMap mp2d_map;
    MvdHwRegAvsdMap avsd_map;
    MvdHwRegAspdMap aspd_map;
    MvdHwRegRvidMap rvid_map;
    MvdHwRegOn2dMap on2d_map;
    MvdHwRegHevcMap hevc_map;
  } DecMap;

  /* 0.25kB BBD address space */
  union
  {
    MvdHwReg        bbd[64];
    MvdHwRegBbdMap  bbd_map;
    MvdHwRegDbgMap  dbg_map;
  } BbdMap;

  /* 4KB Decoupled unit space */
  union
  {
    MvdHwReg        dcp[1024];
    MvdHwRegDfeMap  dfe_map;
    MvdHwRegDbeMap  dbe_map[2];
  } DcpMap;

  /* 0.25 KB FBC address space */
  union
  {
    MvdHwReg        fbc[64];
    MvdHwRegFbcMap  fbc_map;
  } FbcMap;

  /* 3.75 KB Special address */
  MvdHwReg spc[1024 - 64];

} MvdHwRegMap;

///////////////////////////////////////////////////////////////////////////////////////
// DPV address map
//

typedef struct
{
  MvdHwReg YFrameCRC;
  MvdHwReg YTopFieldCRC;
  MvdHwReg YBotFieldCRC;
  MvdHwReg UVFrameCRC;
  MvdHwReg UVTopFieldCRC;
  MvdHwReg UVBotFieldCRC;

} MvdHwRegA3CRCRegMap;


typedef struct
{
  MvdHwReg YWordLower;
  MvdHwReg UWordLower;
  MvdHwReg VWordLower;
  MvdHwReg YWordUpper;
  MvdHwReg UWordUpper;
  MvdHwReg VWordUpper;

} MvdHwRegMD5RegMap;

typedef struct
{
  MvdHwReg YFrameCRC;
  MvdHwReg UFrameCRC;
  MvdHwReg VFrameCRC;

} MvdHwRegHashCRCRegMap;

typedef struct
{
  MvdHwReg YCheckSum;
  MvdHwReg UCheckSum;
  MvdHwReg VCheckSum;

} MvdHwRegHashChkSumRegMap;

typedef struct
{                                         //DCSN_CFG_BASE_MMIO | 0x1FC000  => 0xE07FC000
  MvdHwReg Control;
  MvdHwReg Status;
  MvdHwReg DTLReadCalib;
  MvdHwReg CropTopLeft;
  MvdHwReg CropBotRight;
  MvdHwReg LumaBase;
  MvdHwReg ChromaBase;

  union
  {
    MvdHwReg                 Digest[0x6];
    MvdHwRegA3CRCRegMap      A3CRC;
    MvdHwRegMD5RegMap        MD5;
    MvdHwRegHashCRCRegMap    HashCRC;
    MvdHwRegHashChkSumRegMap HashCheckSum;

  } DigestMap0;

  MvdHwReg Stride;

  /* For code readability - this is added as a union       */
  /* but in fact the registers are valid only in MD5 mode  */
  union
  {
    MvdHwReg                 Digest[0x6];
    MvdHwRegA3CRCRegMap      A3CRC;
    MvdHwRegMD5RegMap        MD5;
    MvdHwRegHashCRCRegMap    HashCRC;
    MvdHwRegHashChkSumRegMap HashCheckSum;

  } DigestMap1;

  MvdHwReg Scratch[0x4];

} MvdHwDPVRegMap;

#endif /* _MVD_REG_MAP_H_ */

/* End of file */
