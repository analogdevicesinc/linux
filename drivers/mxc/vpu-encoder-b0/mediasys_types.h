/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MEDIASYS_TYPES_H_
#define _MEDIASYS_TYPES_H_

typedef unsigned int u_int32;
typedef unsigned char u_int8;
typedef unsigned long u_int64;
typedef unsigned int BOOL;
typedef int int32;
#define FALSE 0
#define TRUE 1
#define VPU_MAX_NUM_STREAMS 4
#define VID_API_NUM_STREAMS 4
#define VID_API_MAX_BUF_PER_STR 3
#define VID_API_MAX_NUM_MVC_VIEWS 4
#define MEDIAIP_MAX_NUM_MALONES 2
#define MEDIAIP_MAX_NUM_MALONE_IRQ_PINS 2
#define MEDIAIP_MAX_NUM_WINDSORS 1
#define MEDIAIP_MAX_NUM_WINDSOR_IRQ_PINS 2
#define MEDIAIP_MAX_NUM_CMD_IRQ_PINS 2
#define MEDIAIP_MAX_NUM_MSG_IRQ_PINS 1
#define MEDIAIP_MAX_NUM_TIMER_IRQ_PINS 4
#define MEDIAIP_MAX_NUM_TIMER_IRQ_SLOTS 4
#define VID_API_COMMAND_LIMIT    64
#define VID_API_MESSAGE_LIMIT    256

#define API_CMD_AVAILABLE            0x0
#define API_CMD_INCOMPLETE           0x1
#define API_CMD_BUFFER_ERROR         0x2
#define API_CMD_UNAVAILABLE          0x3
#define API_MSG_AVAILABLE            0x0
#define API_MSG_INCOMPLETE           0x1
#define API_MSG_BUFFER_ERROR         0x2
#define API_MSG_UNAVAILABLE          0x3
#define MEDIAIP_ENC_USER_DATA_WORDS  16
#define MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES 0x6
#define MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES 0x3

typedef enum {
	GTB_ENC_CMD_NOOP        = 0x0,
	GTB_ENC_CMD_STREAM_START,
	GTB_ENC_CMD_FRAME_ENCODE,
	GTB_ENC_CMD_FRAME_SKIP,
	GTB_ENC_CMD_STREAM_STOP,
	GTB_ENC_CMD_PARAMETER_UPD,
	GTB_ENC_CMD_TERMINATE,
	GTB_ENC_CMD_SNAPSHOT,
	GTB_ENC_CMD_ROLL_SNAPSHOT,
	GTB_ENC_CMD_LOCK_SCHEDULER,
	GTB_ENC_CMD_UNLOCK_SCHEDULER,
	GTB_ENC_CMD_CONFIGURE_CODEC,
	GTB_ENC_CMD_DEAD_MARK
} GTB_ENC_CMD;

typedef enum {
	VID_API_ENC_EVENT_RESET_DONE = 0x1,
	VID_API_ENC_EVENT_START_DONE,
	VID_API_ENC_EVENT_STOP_DONE,
	VID_API_ENC_EVENT_TERMINATE_DONE,
	VID_API_ENC_EVENT_FRAME_INPUT_DONE,
	VID_API_ENC_EVENT_FRAME_DONE,
	VID_API_ENC_EVENT_FRAME_RELEASE,
	VID_API_ENC_EVENT_PARA_UPD_DONE,
	VID_API_ENC_EVENT_MEM_REQUEST

} ENC_TB_API_ENC_EVENT;

typedef enum {
	MEDIAIP_ENC_PIC_TYPE_B_FRAME = 0,
	MEDIAIP_ENC_PIC_TYPE_P_FRAME,
	MEDIAIP_ENC_PIC_TYPE_I_FRAME,
	MEDIAIP_ENC_PIC_TYPE_IDR_FRAME,
	MEDIAIP_ENC_PIC_TYPE_BI_FRAME

} MEDIAIP_ENC_PIC_TYPE, *pMEDIAIP_ENC_PIC_TYPE;

typedef struct {
	u_int32                   uMemPhysAddr;
	u_int32                   uMemVirtAddr;
	u_int32                   uMemSize;
} MEDIAIP_ENC_MEM_RESOURCE, *pMEDIAIP_ENC_MEM_RESOURCE;

typedef struct {
	u_int32                    uEncFrmSize;
	u_int32                    uEncFrmNum;
	u_int32                    uRefFrmSize;
	u_int32                    uRefFrmNum;
	u_int32                    uActBufSize;
	u_int32                    uAlignmentMask;
} MEDIAIP_ENC_MEM_REQ_DATA, *pMEDIAIP_ENC_MEM_REQ_DATA;

typedef struct {
	MEDIAIP_ENC_MEM_RESOURCE  tEncFrameBuffers[MEDIAIP_MAX_NUM_WINDSOR_SRC_FRAMES];
	MEDIAIP_ENC_MEM_RESOURCE  tRefFrameBuffers[MEDIAIP_MAX_NUM_WINDSOR_REF_FRAMES];
	MEDIAIP_ENC_MEM_RESOURCE  tActFrameBufferArea;
} MEDIAIP_ENC_MEM_POOL, *pMEDIAIP_ENC_MEM_POOL;

///////////////////////////////////////////
// MEDIAIP_ENC_PIC_TYPE

typedef struct {
	u_int32              uFrameID;
	u_int32              uPicEncodDone;
	MEDIAIP_ENC_PIC_TYPE ePicType;
	u_int32              uSkippedFrame;
	u_int32              uErrorFlag;
	u_int32              uPSNR;
	u_int32              uFlushDone;
	u_int32              uMBy;
	u_int32              uMBx;
	u_int32              uFrameSize;
	u_int32              uFrameEncTtlCycles;
	u_int32              uFrameEncTtlFrmCycles;
	u_int32              uFrameEncTtlSlcCycles;
	u_int32              uFrameEncTtlEncCycles;
	u_int32              uFrameEncTtlHmeCycles;
	u_int32              uFrameEncTtlDsaCycles;
	u_int32              uFrameEncFwCycles;
	u_int32              uFrameCrc;
	u_int32              uNumInterrupts_1;
	u_int32              uNumInterrupts_2;
	u_int32              uH264POC;
	u_int32              uRefInfo;
	u_int32              uPicNum;
	u_int32              uPicActivity;
	u_int32              uSceneChange;
	u_int32              uMBStats;
	u_int32              uEncCacheCount0;
	u_int32              uEncCacheCount1;
	u_int32              uMtlWrStrbCnt;
	u_int32              uMtlRdStrbCnt;
	u_int32              uStrBuffWrPtr;
	u_int32              uDiagnosticEvents;

	u_int32              uProcIaccTotRdCnt;
	u_int32              uProcDaccTotRdCnt;
	u_int32              uProcDaccTotWrCnt;
	u_int32              uProcDaccRegRdCnt;
	u_int32              uProcDaccRegWrCnt;
	u_int32              uProcDaccRngRdCnt;
	u_int32              uProcDaccRngWrCnt;

} MEDIAIP_ENC_PIC_INFO, *pMEDIAIP_ENC_PIC_INFO;

typedef enum {
	MEDIAIP_PLAYMODE_CONNECTIVITY = 0,
	MEDIAIP_PLAYMODE_BROADCAST,
	MEDIAIP_PLAYMODE_BROADCAST_DSS,
	MEDIAIP_PLAYMODE_LAST = MEDIAIP_PLAYMODE_BROADCAST_DSS

} MEDIA_IP_PLAYMODE;

typedef struct {
	u_int32 wptr;
	u_int32 rptr;
	u_int32 start;
	u_int32 end;

} BUFFER_DESCRIPTOR_TYPE, *pBUFFER_DESCRIPTOR_TYPE;

typedef struct {
	u_int32	uWrPtr;
	u_int32	uRdPtr;
	u_int32	uStart;
	u_int32	uEnd;
	u_int32	uLo;
	u_int32	uHi;

} MediaIPFW_Video_BufDesc;

typedef struct {
	u_int32 uCfgCookie;

	u_int32 uNumMalones;
	u_int32 uMaloneBaseAddress[MEDIAIP_MAX_NUM_MALONES];
	u_int32 uHifOffset[MEDIAIP_MAX_NUM_MALONES];
	u_int32 uMaloneIrqPin[MEDIAIP_MAX_NUM_MALONES][MEDIAIP_MAX_NUM_MALONE_IRQ_PINS];
	u_int32 uMaloneIrqTarget[MEDIAIP_MAX_NUM_MALONES][MEDIAIP_MAX_NUM_MALONE_IRQ_PINS];

	u_int32 uNumWindsors;
	u_int32 uWindsorBaseAddress[MEDIAIP_MAX_NUM_WINDSORS];
	u_int32 uWindsorIrqPin[MEDIAIP_MAX_NUM_WINDSORS][MEDIAIP_MAX_NUM_WINDSOR_IRQ_PINS];
	u_int32 uWindsorIrqTarget[MEDIAIP_MAX_NUM_WINDSORS][MEDIAIP_MAX_NUM_WINDSOR_IRQ_PINS];

	u_int32 uCmdIrqPin[MEDIAIP_MAX_NUM_CMD_IRQ_PINS];
	u_int32 uCmdIrqTarget[MEDIAIP_MAX_NUM_CMD_IRQ_PINS];

	u_int32 uMsgIrqPin[MEDIAIP_MAX_NUM_MSG_IRQ_PINS];
	u_int32 uMsgIrqTarget[MEDIAIP_MAX_NUM_MSG_IRQ_PINS];

	u_int32 uSysClkFreq;
	u_int32 uNumTimers;
	u_int32 uTimerBaseAddr;
	u_int32 uTimerIrqPin[MEDIAIP_MAX_NUM_TIMER_IRQ_PINS];
	u_int32 uTimerIrqTarget[MEDIAIP_MAX_NUM_TIMER_IRQ_PINS];
	u_int32 uTimerSlots[MEDIAIP_MAX_NUM_TIMER_IRQ_SLOTS];

	u_int32 uGICBaseAddr;
	u_int32 uUartBaseAddr;

	u_int32 uDPVBaseAddr;
	u_int32 uDPVIrqPin;
	u_int32 uDPVIrqTarget;

	u_int32 uPixIfBaseAddr;

	u_int32 pal_trace_level;
	u_int32 pal_trace_destination;

	u_int32 pal_trace_level1;
	u_int32 pal_trace_destination1;

	u_int32 uHeapBase;
	u_int32 uHeapSize;

	u_int32 uFSLCacheBaseAddr;

} MEDIAIP_FW_SYSTEM_CONFIG, *pMEDIAIP_FW_SYSTEM_CONFIG;

typedef struct {
	u_int32   uFrameID;
	u_int32   uLumaBase;
	u_int32   uChromaBase;
	u_int32   uParamIdx;

} MEDIAIP_ENC_YUV_BUFFER_DESC, *pMEDIAIP_ENC_YUV_BUFFER_DESC;

typedef struct {
	u_int32 use_ame;

	u_int32 cme_mvx_max;
	u_int32 cme_mvy_max;
	u_int32 ame_prefresh_y0;
	u_int32 ame_prefresh_y1;
	u_int32 fme_min_sad;
	u_int32 cme_min_sad;

	u_int32 fme_pred_int_weight;
	u_int32 fme_pred_hp_weight;
	u_int32 fme_pred_qp_weight;
	u_int32 fme_cost_weight;
	u_int32 fme_act_thold;
	u_int32 fme_sad_thold;
	u_int32 fme_zero_sad_thold;

	u_int32 fme_lrg_mvx_lmt;
	u_int32 fme_lrg_mvy_lmt;
	u_int32 fme_force_mode;
	u_int32 fme_force4mvcost;
	u_int32 fme_force2mvcost;

	u_int32 h264_inter_thrd;

	u_int32 i16x16_mode_cost;
	u_int32 i4x4_mode_lambda;
	u_int32 i8x8_mode_lambda;

	u_int32 inter_mod_mult;
	u_int32 inter_sel_mult;
	u_int32 inter_bid_cost;
	u_int32 inter_bwd_cost;
	u_int32 inter_4mv_cost;
	int32   one_mv_i16_cost;
	int32   one_mv_i4x4_cost;
	int32   one_mv_i8x8_cost;
	int32   two_mv_i16_cost;
	int32   two_mv_i4x4_cost;
	int32   two_mv_i8x8_cost;
	int32   four_mv_i16_cost;
	int32   four_mv_i4x4_cost;
	int32   four_mv_i8x8_cost;

	u_int32 intra_pred_enab;
	u_int32 intra_chr_pred;
	u_int32 intra16_pred;
	u_int32 intra4x4_pred;
	u_int32 intra8x8_pred;

	u_int32 cb_base;
	u_int32 cb_size;
	u_int32 cb_head_room;

	u_int32 mem_page_width;
	u_int32 mem_page_height;
	u_int32 mem_total_size;
	u_int32 mem_chunk_phys_addr;
	u_int32 mem_chunk_virt_addr;
	u_int32 mem_chunk_size;
	u_int32 mem_y_stride;
	u_int32 mem_uv_stride;

	u_int32 split_wr_enab;
	u_int32 split_wr_req_size;
	u_int32 split_rd_enab;
	u_int32 split_rd_req_size;

} MEDIAIP_ENC_CALIB_PARAMS, *pMEDIAIP_ENC_CALIB_PARAMS;

typedef struct {
	u_int32 ParamChange;

	u_int32 start_frame;                // These variables are for debugging purposes only
	u_int32 end_frame;

	u_int32 userdata_enable;
	u_int32 userdata_id[4];
	u_int32 userdata_message[MEDIAIP_ENC_USER_DATA_WORDS];
	u_int32 userdata_length;

	u_int32 h264_profile_idc;
	u_int32 h264_level_idc;
	u_int32 h264_au_delimiter;          // Enable the use of Access Unit Delimiters
	u_int32 h264_seq_end_code;          // Enable the use of Sequence End Codes
	u_int32 h264_recovery_points;       // Enable the use of Recovery Points (must be with a fixed GOP structure)
	u_int32 h264_vui_parameters;        // Enable the use of VUI parameters (for rate control purposes)
	u_int32 h264_aspect_ratio_present;
	u_int32 h264_aspect_ratio_sar_width;
	u_int32 h264_aspect_ratio_sar_height;
	u_int32 h264_overscan_present;
	u_int32 h264_video_type_present;
	u_int32 h264_video_format;
	u_int32 h264_video_full_range;
	u_int32 h264_video_colour_descriptor;
	u_int32 h264_video_colour_primaries;
	u_int32 h264_video_transfer_char;
	u_int32 h264_video_matrix_coeff;
	u_int32 h264_chroma_loc_info_present;
	u_int32 h264_chroma_loc_type_top;
	u_int32 h264_chroma_loc_type_bot;
	u_int32 h264_timing_info_present;
	u_int32 h264_buffering_period_present;
	u_int32 h264_low_delay_hrd_flag;

	u_int32 aspect_ratio;
	u_int32 test_mode;                  // Automated firmware test mode
	u_int32 dsa_test_mode;              // Automated test mode for the DSA.
	u_int32 fme_test_mode;              // Automated test mode for the fme

	u_int32 cbr_row_mode;               //0: FW mode; 1: HW mode
	u_int32 windsor_mode;               //0: normal mode; 1: intra only mode; 2: intra+0MV mode
	u_int32 encode_mode;                // H264, VC1, MPEG2, DIVX
	u_int32 frame_width;                // display width
	u_int32 frame_height;               // display height
	u_int32 enc_frame_width;            // encoding width, should be 16-pix align
	u_int32 enc_frame_height;           // encoding height, should be 16-pix aligned for progressive and 32-pix aligned for interlace
	u_int32 frame_rate_num;
	u_int32 frame_rate_den;

	u_int32 vi_field_source;              // vi input source is frame or field
	u_int32 vi_frame_width;
	u_int32 vi_frame_height;
	u_int32 crop_frame_width;
	u_int32 crop_frame_height;
	u_int32 crop_x_start_posn;
	u_int32 crop_y_start_posn;
	u_int32 mode422;
	u_int32 mode_yuy2;
	u_int32 dsa_luma_en;
	u_int32 dsa_chroma_en;
	u_int32 dsa_ext_hfilt_en;
	u_int32 dsa_di_en;
	u_int32 dsa_di_top_ref;
	u_int32 dsa_vertf_disable;   // disable the vertical filter.
	u_int32 dsa_disable_pwb;
	u_int32 dsa_hor_phase;
	u_int32 dsa_ver_phase;

	u_int32 dsa_iac_enable;      // IAC / DSA cannot operate independently in FW so this variable controls
	u_int32 iac_sc_threshold;
	u_int32 iac_vm_threshold;
	u_int32 iac_skip_mode;
	u_int32 iac_grp_width;
	u_int32 iac_grp_height;

	u_int32 rate_control_mode;
	u_int32 rate_control_resolution;
	u_int32 buffer_size;
	u_int32 buffer_level_init;
	u_int32 buffer_I_bit_budget;

	u_int32 top_field_first;

	u_int32 intra_lum_qoffset;
	u_int32 intra_chr_qoffset;
	u_int32 inter_lum_qoffset;
	u_int32 inter_chr_qoffset;
	u_int32 use_def_scaling_mtx;

	u_int32 inter_8x8_enab;
	u_int32 inter_4x4_enab;

	u_int32 fme_enable_qpel;
	u_int32 fme_enable_hpel;
	u_int32 fme_nozeromv;               // can force the FME not to do the (0,0) search.
	u_int32 fme_predmv_en;
	u_int32 fme_pred_2mv4mv;
	u_int32 fme_smallsadthresh;

	u_int32 ame_en_lmvc;
	u_int32 ame_x_mult;
	u_int32 cme_enable_4mv;             // Enable the use of 4MV partitioning
	u_int32 cme_enable_1mv;
	u_int32 hme_enable_16x8mv;
	u_int32 hme_enable_8x16mv;
	u_int32 cme_mv_weight;              // CME motion vector decisions are made by combining these
	u_int32 cme_mv_cost;                // cost and weight variables
	u_int32 ame_mult_mv;
	u_int32 ame_shift_mv;

	u_int32 hme_forceto1mv_en;
	u_int32 hme_2mv_cost;               // the cost of choosing a 2MV mode over 1MV.
	u_int32 hme_pred_mode;
	u_int32 hme_sc_rnge;
	u_int32 hme_sw_rnge;

	// for windsor pes , add by fulin
	u_int32 output_format;     // 0: output ES; 1: output PES
	u_int32 timestamp_enab;    // 0: have timestamps in all frame; 1: have timestamps in I and P frame; 2: have timestamps only in I frame
	u_int32 initial_PTS_enab;  // if enabled , use following value,else compute by fw
	u_int32 initial_PTS;       // the initial value of PTS in the first frame (ms)

} MEDIAIP_ENC_CONFIG_PARAMS, *pMEDIAIP_ENC_CONFIG_PARAMS;

typedef struct {
	u_int32 ParamChange;

	u_int32 gop_length;

	u_int32 rate_control_bitrate;
	u_int32 rate_control_bitrate_min;
	u_int32 rate_control_bitrate_max;
	u_int32 rate_control_content_models;
	u_int32 rate_control_iframe_maxsize; // Maximum size of I frame generated by BPM in comparison to ideal (/4)
	u_int32 rate_control_qp_init;
	u_int32 rate_control_islice_qp;
	u_int32 rate_control_pslice_qp;
	u_int32 rate_control_bslice_qp;

	u_int32 adaptive_quantization;      // Enable the use of activity measures from VIPP in QP assignment
	u_int32 aq_variance;
	u_int32 cost_optimization;          // Enable picture/frame level adjustments of the cost parameters by FW.
	u_int32 fdlp_mode;                  // Frequency-domain low-pass filter control, 0: off, 1-4: specific, 5: adaptive
	u_int32 enable_isegbframes;         // Enable the use of B frames in the first segment of a GOP
	u_int32 enable_adaptive_keyratio;   // Enable the use of an adaptive I to P/B ratio (aims to reduce distortion)
	u_int32 keyratio_imin;              // Clamps applied to picture size ratios
	u_int32 keyratio_imax;
	u_int32 keyratio_pmin;
	u_int32 keyratio_pmax;
	u_int32 keyratio_bmin;
	u_int32 keyratio_bmax;
	int32   keyratio_istep;
	int32   keyratio_pstep;
	int32   keyratio_bstep;

	u_int32 enable_paff;                // Enable Picture Adaptive Frame/Field
	u_int32 enable_b_frame_ref;         // Enable B frame as references
	u_int32 enable_adaptive_gop;        // Enable an adaptive GOP structure
	u_int32 enable_closed_gop;          // Enable a closed GOP structure
									  // i.e. if enabled, the first consecutive B frames following
									  // an I frame in each GOP will be intra or backwards only coded
									  // and do not rely on previous reference pictures.
	u_int32 open_gop_refresh_freq;      // Controls the insertion of closed GOP's (or IDR GOP's in H.264)
	u_int32 enable_adaptive_sc;         // Enable adaptive scene change GOP structure (0:off, 1:adaptive, 2:IDR)
	u_int32 enable_fade_detection;      // Enable fade detection and associated motion estimation restrictions
	int32   fade_detection_threshold;   // Threshold at which the activity slope indicates a possible fading event
	u_int32 enable_repeat_b;            // Enalbe the repeated B frame mode at CBR
	u_int32 enable_low_delay_b;         // Use low delay-b frames with an IPPPP style GOP

} MEDIAIP_ENC_STATIC_PARAMS, *pMEDIAIP_ENC_STATIC_PARAMS;

typedef struct {
	u_int32 ParamChange;

	u_int32 rows_per_slice;

	u_int32 mbaff_enable;                // Macroblock adaptive frame/field enable
	u_int32 dbf_enable;                  // Enable the deblocking filter

	u_int32 field_source;                // progressive/interlaced control
	u_int32 gop_b_length;                // Number of B frames between anchor frames
									  //  (only to be changed at a GOP segment boundary)
	u_int32 mb_group_size;               // Number of macroblocks normally assigned to a group
									  // (implications for performance, interrupts and rate control)

	u_int32 cbr_rows_per_group;

	u_int32 skip_enable;                 // Enable the use of skipped macroblocks

	u_int32 pts_bits_0_to_31;            // TO BE REMOVED...
	u_int32 pts_bit_32;

	u_int32 rm_expsv_cff;
	u_int32 const_ipred;
	int32 chr_qp_offset;
	u_int32 intra_mb_qp_offset;

	u_int32 h264_cabac_init_method;
	u_int32 h264_cabac_init_idc;
	u_int32 h264_cabac_enable;                 // Main and stream

	int32 alpha_c0_offset_div2;
	int32 beta_offset_div2;

	u_int32 intra_prefresh_y0; // for setting intra limits for prog refresh.
	u_int32 intra_prefresh_y1;

	u_int32 dbg_dump_rec_src;

} MEDIAIP_ENC_DYN_PARAMS, *pMEDIAIP_ENC_DYN_PARAMS;

typedef struct {
	MEDIAIP_ENC_CALIB_PARAMS   Calib;
	MEDIAIP_ENC_CONFIG_PARAMS  Config;
	MEDIAIP_ENC_STATIC_PARAMS  Static;
	MEDIAIP_ENC_DYN_PARAMS     Dynamic;
} MEDIAIP_ENC_EXPERT_MODE_PARAM, *pMEDIAIP_ENC_EXPERT_MODE_PARAM;

typedef enum {
	MEDIAIP_ENC_FMT_H264 = 0,
	MEDIAIP_ENC_FMT_VC1,
	MEDIAIP_ENC_FMT_MPEG2,
	MEDIAIP_ENC_FMT_MPEG4SP,
	MEDIAIP_ENC_FMT_H263,
	MEDIAIP_ENC_FMT_MPEG1,
	MEDIAIP_ENC_FMT_SHORT_HEADER,
	MEDIAIP_ENC_FMT_NULL

} MEDIAIP_ENC_FMT;

typedef enum {
	MEDIAIP_ENC_PROF_MPEG2_SP = 0,
	MEDIAIP_ENC_PROF_MPEG2_MP,
	MEDIAIP_ENC_PROF_MPEG2_HP,
	MEDIAIP_ENC_PROF_H264_BP,
	MEDIAIP_ENC_PROF_H264_MP,
	MEDIAIP_ENC_PROF_H264_HP,
	MEDIAIP_ENC_PROF_MPEG4_SP,
	MEDIAIP_ENC_PROF_MPEG4_ASP,
	MEDIAIP_ENC_PROF_VC1_SP,
	MEDIAIP_ENC_PROF_VC1_MP,
	MEDIAIP_ENC_PROF_VC1_AP

} MEDIAIP_ENC_PROFILE;

typedef enum {
	MEDIAIP_ENC_BITRATECONTROLMODE_VBR          = 0x00000001,
	MEDIAIP_ENC_BITRATECONTROLMODE_CBR          = 0x00000002,
	MEDIAIP_ENC_BITRATECONTROLMODE_CONSTANT_QP  = 0x00000004   /* Only in debug mode */

} MEDIAIP_ENC_BITRATE_MODE, *pMEDIAIP_ENC_BITRATE_MODE;

typedef struct {
	MEDIAIP_ENC_FMT           eCodecMode;
	MEDIAIP_ENC_PROFILE       eProfile;

	MEDIAIP_ENC_MEM_RESOURCE  tEncMemDesc;

	u_int32                   uFrameRate;
	u_int32                   uSrcStride;
	u_int32                   uSrcWidth;
	u_int32                   uSrcHeight;
	u_int32                   uSrcOffset_x;
	u_int32                   uSrcOffset_y;
	u_int32                   uSrcCropWidth;
	u_int32                   uSrcCropHeight;
	u_int32                   uOutWidth;
	u_int32                   uOutHeight;
	u_int32                   uIFrameInterval;
	u_int32                   uGopBLength;
	u_int32                   uLowLatencyMode;

	MEDIAIP_ENC_BITRATE_MODE  eBitRateMode;
	u_int32                   uTargetBitrate;
	u_int32                   uMaxBitRate;
	u_int32                   uMinBitRate;
	u_int32                   uInitSliceQP;

} MEDIAIP_ENC_PARAM, *pMEDIAIP_ENC_PARAM;

typedef struct {
	u_int32   uFrameID;
	u_int32   uErrorFlag;   //Error type
	u_int32   uMBy;
	u_int32   uMBx;
	u_int32   uReserved[12];

} ENC_ENCODING_STATUS, *pENC_ENCODING_STATUS;

typedef struct {
	u_int32   uFrameID;
	u_int32   uDsaCyle;
	u_int32   uMBy;
	u_int32   uMBx;
	u_int32   uReserved[4];

} ENC_DSA_STATUS_t, *pENC_DSA_STATUS_t;

typedef struct {
	u_int32                                  pEncYUVBufferDesc;
	u_int32                                  pEncStreamBufferDesc;
	u_int32                                  pEncExpertModeParam;
	u_int32                                  pEncParam;
	u_int32                                  pEncMemPool;
	/* Status information for master to read */
	u_int32                                  pEncEncodingStatus;
	u_int32                                  pEncDSAStatus;
} MEDIA_ENC_API_CONTROL_INTERFACE, *pMEDIA_ENC_API_CONTROL_INTERFACE;

typedef struct {
	u_int32                                FwExecBaseAddr;
	u_int32                                FwExecAreaSize;
	BUFFER_DESCRIPTOR_TYPE                 StreamCmdBufferDesc;
	BUFFER_DESCRIPTOR_TYPE                 StreamMsgBufferDesc;
	u_int32                                StreamCmdIntEnable[VID_API_NUM_STREAMS];
	u_int32                                FWVersion;
	u_int32                                uMVDFWOffset;
	u_int32                                uMaxEncoderStreams;
	u_int32                                pEncCtrlInterface[VID_API_NUM_STREAMS];
	MEDIAIP_FW_SYSTEM_CONFIG               sSystemCfg;
	u_int32                                uApiVersion;
} ENC_RPC_HOST_IFACE, *pENC_RPC_HOST_IFACE;

#define SCB_XREG_SLV_BASE                               0x00000000
#define SCB_SCB_BLK_CTRL                                0x00070000
#define SCB_BLK_CTRL_XMEM_RESET_SET                     0x00000090
#define SCB_BLK_CTRL_CACHE_RESET_SET                    0x000000A0
#define SCB_BLK_CTRL_CACHE_RESET_CLR                    0x000000A4
#define SCB_BLK_CTRL_SCB_CLK_ENABLE_SET                 0x00000100

#define XMEM_CONTROL                                    0x00041000

#define DEC_MFD_XREG_SLV_BASE                           0x00180000

#define MFD_HIF                                         0x0001C000
#define MFD_HIF_MSD_REG_INTERRUPT_STATUS                0x00000018
#define MFD_SIF                                         0x0001D000
#define MFD_SIF_CTRL_STATUS                             0x000000F0
#define MFD_SIF_INTR_STATUS                             0x000000F4
#define MFD_MCX                                         0x00020800
#define MFD_MCX_OFF                                     0x00000020

#define MFD_BLK_CTRL                                    0x00030000
#define MFD_BLK_CTRL_MFD_SYS_RESET_SET                  0x00000000
#define MFD_BLK_CTRL_MFD_SYS_RESET_CLR                  0x00000004
#define MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET           0x00000100
#define MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_CLR           0x00000104

#endif
