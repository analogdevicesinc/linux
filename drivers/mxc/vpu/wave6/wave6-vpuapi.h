/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - helper definitions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef VPUAPI_H_INCLUDED
#define VPUAPI_H_INCLUDED

#include <linux/kfifo.h>
#include <linux/idr.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ctrls.h>
#include "wave6-vpuerror.h"
#include "wave6-vpuconfig.h"
#include "wave6-vdi.h"
#include "wave6-vpu-ctrl.h"

enum product_id {
	PRODUCT_ID_617,
	PRODUCT_ID_627,
	PRODUCT_ID_637,
	PRODUCT_ID_NONE,
};

struct vpu_attr;

enum vpu_instance_type {
	VPU_INST_TYPE_DEC = 0,
	VPU_INST_TYPE_ENC = 1
};

enum vpu_instance_state {
	VPU_INST_STATE_NONE = 0,
	VPU_INST_STATE_OPEN = 1,
	VPU_INST_STATE_INIT_SEQ = 2,
	VPU_INST_STATE_PIC_RUN = 3,
	VPU_INST_STATE_SEEK = 4,
	VPU_INST_STATE_STOP = 5
};

#define WAVE6_MAX_FBS 31

#define WAVE6_DEC_HEVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN(_w, 256) / 16) * (ALIGN(_h, 64) / 16) * 1 * 16)
#define WAVE6_DEC_AVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN(_w, 64) / 16) * (ALIGN(_h, 16) / 16) * 5 * 16)
#define WAVE6_DEC_VP9_MVCOL_BUF_SIZE_1(_w, _h) \
	((ALIGN(_w, 64) / 64) * ((ALIGN(_h, 64) / 64) * 32 * 16))
#define WAVE6_DEC_VP9_MVCOL_BUF_SIZE_2(_w, _h) \
	((ALIGN(_w, 256) / 256) * ((ALIGN(_h, 64) / 64) * 16 * 16))
#define WAVE6_DEC_AV1_MVCOL_BUF_SIZE_1(_w, _h) \
	(((ALIGN(_w, 64) / 64 * 16 * 16) + (ALIGN(_w, 256) / 64 * 8 * 16)) * (ALIGN(_h, 64) / 64))
#define WAVE6_DEC_AV1_MVCOL_BUF_SIZE_2(_w, _h) \
	((ALIGN(_w, 128) / 64) * (ALIGN(_h, 128) / 64) * 4 * 128 + 656 * 16)
#define WAVE6_AV1_DEFAULT_CDF_BUF_SIZE (48 * 1024)
#define WAVE6_VP9_SEGMAP_BUF_SIZE(_w, _h) \
	((ALIGN(_w, 64) / 64) * (ALIGN(_h, 64) / 64) * 64)
#define WAVE6_FBC_LUMA_TABLE_SIZE(_w, _h) \
	(ALIGN(_w, 256) * ALIGN(_h, 64) / 32)
#define WAVE6_FBC_CHROMA_TABLE_SIZE(_w, _h) \
	(ALIGN((_w / 2), 256) * ALIGN(_h, 64) / 32)
#define WAVE6_ENC_AV1_MVCOL_BUF_SIZE (12 * 1024)
#define WAVE6_ENC_AVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN(_w, 512) / 512) * (ALIGN(_h, 16) / 16) * 16)
#define WAVE6_ENC_HEVC_MVCOL_BUF_SIZE(_w, _h) \
	((ALIGN(_w, 64) / 64) * (ALIGN(_h, 64) / 64) * 128)
#define WAVE6_ENC_SUBSAMPLED_SIZE(_w, _h) \
	(ALIGN((_w / 4), 16) * ALIGN((_h / 4), 32))

/*
 * common struct and definition
 */
enum cod_std {
	STD_AVC = 0,
	STD_VC1 = 1,
	STD_MPEG2 = 2,
	STD_MPEG4 = 3,
	STD_H263 = 4,
	STD_DIV3 = 5,
	STD_RV = 6,
	STD_AVS = 7,
	STD_THO = 9,
	STD_VP3 = 10,
	STD_VP8 = 11,
	STD_HEVC = 12,
	STD_VP9 = 13,
	STD_AVS2 = 14,
	STD_AV1 = 16,
	STD_MAX
};

enum wave_std {
	W_HEVC_DEC = 0x00,
	W_HEVC_ENC = 0x01,
	W_AVC_DEC = 0x02,
	W_AVC_ENC = 0x03,
	W_VP9_DEC = 0x16,
	W_AVS2_DEC = 0x18,
	W_AV1_DEC = 0x1A,
	W_AV1_ENC = 0x1B,
	STD_UNKNOWN = 0xFF
};

enum SET_PARAM_OPTION {
	OPT_COMMON = 0, /* SET_PARAM command option for encoding sequence */
	OPT_CUSTOM_GOP = 1, /* SET_PARAM command option for setting custom GOP */
	OPT_CUSTOM_HEADER = 2, /* SET_PARAM command option for setting custom VPS/SPS/PPS */
	OPT_VUI = 3, /* SET_PARAM command option for encoding VUI */
	OPT_CHANGE_PARAM = 0x10,
};

enum SET_PARAM_ENABLE {
	ENABLE_SET_QUANT_PARAM               = (1 << 27),
	ENABLE_SET_QROUND_OFFSET             = (1 << 26),
	ENABLE_SET_BG_PARAM                  = (1 << 22),
	ENABLE_SET_RDO_BIAS_PARAM            = (1 << 21),
	ENABLE_SET_VUI_HRD_PARAM             = (1 << 20),
	ENABLE_SET_RC_VBV_BUFFER_SIZE        = (1 << 14),
	ENABLE_SET_RC_MAX_BITRATE            = (1 << 13),
	ENABLE_SET_MIN_MAX_QP                = (1 << 12),
	ENABLE_SET_RC_PARAM                  = (1 << 11),
	ENABLE_SET_RC_TARGET_RATE            = (1 << 10),
	ENABLE_SET_SLICE_PARAM               = (1 <<  6),
	ENABLE_SET_RDO_PARAM                 = (1 <<  5),
	ENABLE_SET_TEMPORAL_QP_PARAM         = (1 <<  4),
	ENABLE_SET_INTRA_PARAM               = (1 <<  2),
	ENABLE_SET_GOP_PARAM                 = (1 <<  1),
	ENABLE_SET_PPS_PARAM                 = (1 <<  0),
};

enum DEC_PIC_HDR_OPTION {
	INIT_SEQ_NORMAL = 0x01,
	INIT_SEQ_W_THUMBNAIL = 0x11,
};

enum DEC_PIC_OPTION {
	DEC_PIC_NORMAL = 0x00, /* it is normal mode of DEC_PIC command. */
	DEC_PIC_W_THUMBNAIL = 0x10, /* thumbnail mode (skip non-IRAP without reference reg.) */
	SKIP_NON_IRAP = 0x11, /* it skips to decode non-IRAP pictures. */
	SKIP_NON_REF_PIC = 0x13
};

/************************************************************************/
/* PROFILE & LEVEL */
/************************************************************************/
/* HEVC */
#define HEVC_PROFILE_MAIN 1
#define HEVC_PROFILE_MAIN10 2
#define HEVC_PROFILE_STILLPICTURE 3
#define HEVC_PROFILE_MAIN10_STILLPICTURE 2

/* H.264 profile for encoder*/
#define H264_PROFILE_BP 1
#define H264_PROFILE_MP 2
#define H264_PROFILE_EXTENDED 3
#define H264_PROFILE_HP 4
#define H264_PROFILE_HIGH10 5

/* decoding_refresh_type */
#define DEC_REFRESH_TYPE_NON_IRAP 0
#define DEC_REFRESH_TYPE_IDR 2

#define DEFAULT_NUM_TICKS_POC_DIFF 100
#define H264_VUI_SAR_IDC_EXTENDED 255

/**
 * \brief parameters of DEC_SET_SEQ_CHANGE_MASK
 */
#define SEQ_CHANGE_ENABLE_PROFILE BIT(5)
#define SEQ_CHANGE_CHROMA_FORMAT_IDC BIT(15) /* AV1 */
#define SEQ_CHANGE_ENABLE_SIZE BIT(16)
#define SEQ_CHANGE_INTER_RES_CHANGE BIT(17) /* VP9 */
#define SEQ_CHANGE_ENABLE_BITDEPTH BIT(18)
#define SEQ_CHANGE_ENABLE_DPB_COUNT BIT(19)

#define SEQ_CHANGE_ENABLE_ALL_VP9 (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_INTER_RES_CHANGE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_HEVC (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AVS2 (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AVC (SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define SEQ_CHANGE_ENABLE_ALL_AV1 (SEQ_CHANGE_ENABLE_PROFILE | \
		SEQ_CHANGE_CHROMA_FORMAT_IDC | \
		SEQ_CHANGE_ENABLE_SIZE | \
		SEQ_CHANGE_ENABLE_BITDEPTH | \
		SEQ_CHANGE_ENABLE_DPB_COUNT)

#define DEC_NOTI_FLAG_NO_FB 0x2
#define DEC_NOTI_FLAG_SEQ_CHANGE 0x1

#define RECON_IDX_FLAG_ENC_END -1
#define RECON_IDX_FLAG_ENC_DELAY -2
#define RECON_IDX_FLAG_HEADER_ONLY -3
#define RECON_IDX_FLAG_CHANGE_PARAM -4

#define DEC_USERDATA_FLAG_VUI 2
#define DEC_USERDATA_FLAG_RECOVERY_POINT 9

enum codec_command {
	ENABLE_ROTATION,
	ENABLE_MIRRORING,
	SET_MIRROR_DIRECTION,
	SET_ROTATION_ANGLE,
	ENABLE_DEC_THUMBNAIL_MODE,
	DEC_RESET_FRAMEBUF_INFO,
	DEC_GET_SEQ_INFO,
};

enum cb_cr_order {
	CBCR_ORDER_NORMAL,
	CBCR_ORDER_REVERSED
};

enum mirror_direction {
	MIRDIR_NONE, /* no mirroring */
	MIRDIR_VER, /* vertical mirroring */
	MIRDIR_HOR, /* horizontal mirroring */
	MIRDIR_HOR_VER /* horizontal and vertical mirroring */
};

enum chroma_format {
	YUV400,
	YUV420,
	YUV422,
	YUV444,
};

enum frame_buffer_format {
	FORMAT_ERR = -1,
	FORMAT_420 = 0, /* 8bit */
	FORMAT_422, /* 8bit */
	FORMAT_224, /* 8bit */
	FORMAT_444, /* 8bit */
	FORMAT_400, /* 8bit */

	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_420_P10_16BIT_MSB = 5, /* lsb |000000xx|xxxxxxxx | msb */
	FORMAT_420_P10_16BIT_LSB, /* lsb |xxxxxxx |xx000000 | msb */
	FORMAT_420_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_420_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	/* 4:2:2 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_422_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_422_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_422_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_422_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	/* 4:4:4 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_444_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_444_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_444_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_444_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	/* 4:0:0 packed format */
	/* little endian perspective */
	/* | addr 0 | addr 1 | */
	FORMAT_400_P10_16BIT_MSB, /* lsb |000000xx |xxxxxxxx | msb */
	FORMAT_400_P10_16BIT_LSB, /* lsb |xxxxxxxx |xx000000 | msb */
	FORMAT_400_P10_32BIT_MSB, /* lsb |00xxxxxxxxxxxxxxxxxxxxxxxxxxx| msb */
	FORMAT_400_P10_32BIT_LSB, /* lsb |xxxxxxxxxxxxxxxxxxxxxxxxxxx00| msb */

	FORMAT_YUYV, /* 8bit packed format : Y0U0Y1V0 Y2U1Y3V1 ... */
	FORMAT_YUYV_P10_16BIT_MSB,
	FORMAT_YUYV_P10_16BIT_LSB,
	FORMAT_YUYV_P10_32BIT_MSB,
	FORMAT_YUYV_P10_32BIT_LSB,

	FORMAT_YVYU, /* 8bit packed format : Y0V0Y1U0 Y2V1Y3U1 ... */
	FORMAT_YVYU_P10_16BIT_MSB,
	FORMAT_YVYU_P10_16BIT_LSB,
	FORMAT_YVYU_P10_32BIT_MSB,
	FORMAT_YVYU_P10_32BIT_LSB,

	FORMAT_UYVY, /* 8bit packed format : U0Y0V0Y1 U1Y2V1Y3 ... */
	FORMAT_UYVY_P10_16BIT_MSB,
	FORMAT_UYVY_P10_16BIT_LSB,
	FORMAT_UYVY_P10_32BIT_MSB,
	FORMAT_UYVY_P10_32BIT_LSB,

	FORMAT_VYUY, /* 8bit packed format : V0Y0U0Y1 V1Y2U1Y3 ... */
	FORMAT_VYUY_P10_16BIT_MSB,
	FORMAT_VYUY_P10_16BIT_LSB,
	FORMAT_VYUY_P10_32BIT_MSB,
	FORMAT_VYUY_P10_32BIT_LSB,

	FORMAT_RGB_32BIT_PACKED = 90,
	FORMAT_YUV444_32BIT_PACKED,
	FORMAT_RGB_P10_32BIT_PACKED,
	FORMAT_YUV444_P10_32BIT_PACKED,

	FORMAT_RGB_24BIT_PACKED = 95,
	FORMAT_YUV444_24BIT_PACKED,
	FORMAT_YUV444_24BIT,

	FORMAT_MAX,
};

enum packed_format_num {
	NOT_PACKED = 0,
	PACKED_YUYV,
	PACKED_YVYU,
	PACKED_UYVY,
	PACKED_VYUY,
};

enum wave6_interrupt_bit {
	INT_WAVE6_INIT_VPU = 0,
	INT_WAVE6_WAKEUP_VPU = 1,
	INT_WAVE6_SLEEP_VPU = 2,
	INT_WAVE6_CREATE_INSTANCE = 3,
	INT_WAVE6_FLUSH_INSTANCE = 4,
	INT_WAVE6_DESTROY_INSTANCE = 5,
	INT_WAVE6_INIT_SEQ = 6,
	INT_WAVE6_SET_FRAMEBUF = 7,
	INT_WAVE6_DEC_PIC = 8,
	INT_WAVE6_ENC_PIC = 8,
	INT_WAVE6_ENC_SET_PARAM = 9,
	INT_WAVE6_SET_DISP_BUF = 10,
	INT_WAVE6_UPDATE_FB = 11,
	INT_WAVE6_BSBUF_EMPTY = 15,
	INT_WAVE6_BSBUF_FULL = 15,
};

enum pic_type {
	PIC_TYPE_I = 0, /* I picture */
	PIC_TYPE_KEY = 0, /* KEY frame for AV1*/
	PIC_TYPE_P = 1, /* P picture */
	PIC_TYPE_INTER = 1, /* inter frame for AV1*/
	PIC_TYPE_B = 2, /* B picture (except VC1) */
	PIC_TYPE_REPEAT = 2, /* repeat frame (VP9 only) */
	PIC_TYPE_AV1_INTRA = 2, /* intra only frame (AV1 only) */
	PIC_TYPE_VC1_BI = 2, /* VC1 BI picture (VC1 only) */
	PIC_TYPE_VC1_B = 3, /* VC1 B picture (VC1 only) */
	PIC_TYPE_D = 3,
	PIC_TYPE_S = 3,
	PIC_TYPE_AVS2_F = 3, /* F picture in AVS2 */
	PIC_TYPE_AV1_SWITCH = 3, /* switch frame (AV1 only) */
	PIC_TYPE_VC1_P_SKIP = 4, /* VC1 P skip picture (VC1 only) */
	PIC_TYPE_MP4_P_SKIP_NOT_CODED = 4, /* not coded P picture in MPEG4 packed mode */
	PIC_TYPE_AVS2_S = 4, /* S picture in AVS2 */
	PIC_TYPE_IDR = 5, /* H.264/H.265 IDR picture */
	PIC_TYPE_AVS2_G = 5, /* G picture in AVS2 */
	PIC_TYPE_AVS2_GB = 6, /* GB picture in AVS2 */
	PIC_TYPE_MAX /* no meaning */
};

enum enc_force_pic_type {
	ENC_FORCE_PIC_TYPE_I = 0,
	ENC_FORCE_PIC_TYPE_P = 1,
	ENC_FORCE_PIC_TYPE_B = 2, // not used
	ENC_FORCE_PIC_TYPE_IDR = 3,
	ENC_FORCE_PIC_TYPE_DISABLED = 4,
};

enum bit_stream_mode {
	BS_MODE_INTERRUPT,
	BS_MODE_RESERVED, /* reserved for the future */
	BS_MODE_PIC_END,
};

enum display_mode {
	DISP_MODE_DISP_ORDER, /* VPU returns the display frame buffer by display order */
	DISP_MODE_DEC_ORDER, /* VPU returns the display frame buffer by decoding order */
};

enum sw_reset_mode {
	SW_RESET_SAFETY,
	SW_RESET_FORCE,
	SW_RESET_ON_BOOT
};

enum tiled_map_type {
	LINEAR_FRAME_MAP = 0, /* linear frame map type */
	COMPRESSED_FRAME_MAP = 17, /* compressed frame map type*/
};

#define DECODE_ALL_TEMPORAL_LAYERS 0
#define DECODE_ALL_SPATIAL_LAYERS 0

enum temporal_id_mode {
	TEMPORAL_ID_MODE_ABSOLUTE,
	TEMPORAL_ID_MODE_RELATIVE,
};

enum aux_buffer_type {
	AUX_BUF_FBC_Y_TBL,
	AUX_BUF_FBC_C_TBL,
	AUX_BUF_MV_COL,
	AUX_BUF_DEF_CDF,
	AUX_BUF_SEG_MAP,
	AUX_BUF_PRE_ENT,
	AUX_BUF_SUB_SAMPLE,
	AUX_BUF_TYPE_MAX,
};

enum intra_refresh_mode {
	INTRA_REFRESH_NONE = 0,
	INTRA_REFRESH_ROW = 1,
	INTRA_REFRESH_COLUMN = 2,
};

enum dec_update_fb_state {
	UPDATE_FB_STATE_ALLOC_FBC = BIT(0),
	UPDATE_FB_STATE_ALLOC_MV = BIT(1),
	UPDATE_FB_STATE_ALLOC_DISP = BIT(2),
	UPDATE_FB_STATE_SEQ_CHANGE = BIT(3),
};

struct vpu_attr {
	u32 product_id; /* the product ID */
	char product_name[8]; /* the product name in ascii code */
	u32 product_version; /* the product version number */
	u32 fw_version; /* the F/W version */
	u32 fw_revision;
	u32 support_decoders; /* bitmask: see <<vpuapi_h_cod_std>> */
	u32 support_encoders; /* bitmask: see <<vpuapi_h_cod_std>> */
	u32 support_bitstream_mode;
	bool support_avc10bit_enc;
	bool support_hevc10bit_enc;
	u32 support_endian_mask; /* A variable of supported endian mode in product */
	bool support_dual_core; /* this indicates whether a product has two vcores. */
	bool support_command_queue;
};

struct frame_buffer {
	dma_addr_t buf_y;
	dma_addr_t buf_cb;
	dma_addr_t buf_cr;
	enum tiled_map_type map_type;
	unsigned int stride; /* A horizontal stride for given frame buffer */
	unsigned int width; /* A width for given frame buffer */
	unsigned int height; /* A height for given frame buffer */
	int index;
	u32 luma_bitdepth: 4;
	u32 chroma_bitdepth: 4;
	u32 chroma_format_idc: 2;
};

struct vpu_rect {
	u32 left; /* A horizontal pixel offset of top-left corner of rectangle from (0, 0) */
	u32 top; /* A vertical pixel offset of top-left corner of rectangle from (0, 0) */
	u32 right; /* A horizontal pixel offset of bottom-right corner of rectangle from (0, 0) */
	u32 bottom; /* A vertical pixel offset of bottom-right corner of rectangle from (0, 0) */
};

struct timestamp_info {
	u32 hour;
	u32 min;
	u32 sec;
	u32 ms;
};

struct sar_info {
	u32 enable;
	u32 idc;
	u32 width;
	u32 height;
};

struct aux_buffer {
	int index;
	int size;
	dma_addr_t addr;
};

struct aux_buffer_info {
	int num;
	struct aux_buffer *buf_array;
	enum aux_buffer_type type;
};

struct dec_aux_buffer_size_info {
	int width;
	int height;
	enum aux_buffer_type type;
};

struct enc_aux_buffer_size_info {
	int width;
	int height;
	enum aux_buffer_type type;
	enum mirror_direction mirror_direction;
	int rotation_angle;
};

struct instance_buffer {
	dma_addr_t temp_base; /* It indicates the start address of temp buffer. */
	u32 temp_size; /* It indicates the size of temp buffer. */
	dma_addr_t sec_base_core0; /* It indicates the start address of secondary-axi buffer. */
	u32 sec_size_core0; /* It indicates the size of secondary-axi buffer. */
	dma_addr_t work_base; /* It indicates the start address of work buffer. */
	u32 work_size; /* It indicates the size of work buffer. */
	dma_addr_t ar_base; /* It indicates the start address of AR table buffer. (Encoder only). */
};

struct report_cycle {
	u32 host_cmd_s; /* Start Tick of DEC_PIC/ENC_PIC host command for the picture */
	u32 host_cmd_e; /* End tick of DEC_PIC/ENC_PIC host command for the picture (until host get the result for the picture) */
	u32 proc_s; /* Start tick of processing hw block for the picture */
	u32 proc_e; /* End tick of processing hw block for the picture */
	u32 vpu_s; /* Start tick of decoding/encoding for the picture */
	u32 vpu_e; /* End tick of decoding/encoding for the picture */
	u32 frame_cycle; /* The total cycle of host command for the picture */
	u32 proc_cycle; /* The total cycle of processing for the picture */
	u32 vpu_cycle; /* The total cycle of decoding/encoding for the picture except processing cycle */
};

/*
 * decode struct and definition
 */

struct dec_open_param {
	enum cb_cr_order cbcr_order;
	enum endian_mode frame_endian;
	enum endian_mode stream_endian;
	enum bit_stream_mode bitstream_mode;
	enum display_mode disp_mode;
	bool enable_non_ref_fbc_write;
	int av1_format;
	u32 ext_addr_vcpu: 8;
	bool is_secure_inst;
	u32 inst_priority: 5;
	struct instance_buffer inst_buffer;
};

struct dec_user_data {
	u32 header;
	u32 num;
	dma_addr_t buf_addr;
	u32 size; /* this is the size of user data. */
	bool buf_full;
};

struct dec_user_data_entry {
	u32 offset;
	u32 size;
};

struct dec_initial_info {
	s32 pic_width;
	s32 pic_height;
	s32 f_rate_numerator; /* the numerator part of frame rate fraction */
	s32 f_rate_denominator; /* the denominator part of frame rate fraction */
	struct vpu_rect pic_crop_rect;
	u32 min_frame_buffer_count; /* between 1 to 16 */
	u32 req_mv_buffer_count; /* between 1 to 16 */
	s32 frame_buf_delay;
	s32 profile;
	s32 level;
	u32 tier;
	bool is_ext_sar;
	u32 aspect_rate_info;
	s32 bit_rate;
	u32 chroma_format_idc; /* A chroma format indicator */
	u32 luma_bitdepth; /* A bit-depth of luma sample */
	u32 chroma_bitdepth; /* A bit-depth of chroma sample */
	u32 err_reason;
	s32 warn_info;
	dma_addr_t rd_ptr; /* A read pointer of bitstream buffer */
	dma_addr_t wr_ptr; /* A write pointer of bitstream buffer */
	unsigned int sequence_no;
	struct dec_user_data user_data;
};

#define WAVE_SKIPMODE_WAVE_NONE 0
#define WAVE_SKIPMODE_NON_IRAP 1
#define WAVE_SKIPMODE_NON_REF 2

struct dec_param {
	s32 skipframe_mode;
	bool cra_as_bla_flag;
	bool disable_film_grain;
	struct timestamp_info timestamp;
};

struct h265_rp_sei {
	unsigned int exist;
	int recovery_poc_cnt; /* recovery_poc_cnt */
	int exact_match_flag; /* exact_match_flag */
	int broken_link_flag; /* broken_link_flag */
};

struct av1_info {
	int allow_screen_content_tools; /* it indicates whether screen content tool is enabled. */
	int allow_intra_bc; /* it indicates whether intra block copy is enabled. */
};

struct dec_output_info {
	int nal_type;
	int pic_type;
	int num_of_err_m_bs;
	int num_of_tot_m_bs;
	int num_of_err_m_bs_in_disp;
	int num_of_tot_m_bs_in_disp;
	struct vpu_rect rc_display;
	int disp_pic_width;
	int disp_pic_height;
	struct vpu_rect rc_decoded;
	int dec_pic_width;
	int dec_pic_height;
	struct av1_info av1_info;
	int decoded_poc;
	int display_poc;
	struct h265_rp_sei h265_rp_sei;
	struct dec_user_data user_data;
	int rd_ptr; /* A stream buffer read pointer for the current decoder instance */
	int wr_ptr; /* A stream buffer write pointer for the current decoder instance */
	dma_addr_t byte_pos_frame_start;
	dma_addr_t byte_pos_frame_end;
	dma_addr_t frame_decoded_addr;
	dma_addr_t frame_display_addr;
	int frame_cycle; /* this variable reports the number of cycles for processing a frame. */
	int error_reason;
	int warn_info;
	unsigned int sequence_no;
	struct report_cycle cycle;
	dma_addr_t release_disp_frame_addr[WAVE6_MAX_FBS];
	dma_addr_t disp_frame_addr[WAVE6_MAX_FBS];
	struct timestamp_info timestamp;
	u32 notification_flag;
	u32 release_disp_frame_num: 5;
	u32 disp_frame_num: 5;
	u32 ctu_size: 2;
	u32 frame_display_flag: 1;
	u32 frame_decoded_flag: 1;
	u32 stream_end_flag: 1;
	u32 output_flag: 1;
	u32 last_frame_in_au: 1;
	u32 decoding_success: 1;
};

struct dec_update_fb_info {
	enum dec_update_fb_state state;
	struct dec_initial_info sequence;
	/**
	 * this variable reports that sequence has been changed while H.264/AVC stream decoding.
	 * if it is 1, HOST application can get the new sequence information by calling
	 * vpu_dec_get_initial_info() or wave6_vpu_dec_issue_seq_init().
	 *
	 * for H.265/HEVC decoder, each bit has a different meaning as follows.
	 *
	 * sequence_changed[5] : it indicates that the profile_idc has been changed.
	 * sequence_changed[16] : it indicates that the resolution has been changed.
	 * sequence_changed[19] : it indicates that the required number of frame buffer has
	 * been changed.
	 */
	u32 sequence_changed;
	int fbc_index;
	int mv_index;
	u32 release_disp_frame_num: 5;
	dma_addr_t release_disp_frame_addr[WAVE6_MAX_FBS];
};

/*
 * encode struct and definition
 */

#define MAX_CUSTOM_LAMBDA_NUM 52
#define MAX_NUM_TEMPORAL_LAYER 7
#define MAX_GOP_NUM 8
#define MAX_NUM_CHANGEABLE_TEMPORAL_LAYER 4

struct custom_gop_pic_param {
	int pic_type; /* A picture type of nth picture in the custom GOP */
	int poc_offset; /* A POC of nth picture in the custom GOP */
	int pic_qp; /* A quantization parameter of nth picture in the custom GOP */
	int use_multi_ref_p; /* use multiref pic for P picture. valid only if PIC_TYPE is P */
	int ref_poc_l0; /* A POC of reference L0 of nth picture in the custom GOP */
	int ref_poc_l1; /* A POC of reference L1 of nth picture in the custom GOP */
	int temporal_id; /* A temporal ID of nth picture in the custom GOP */
};

struct custom_gop_param {
	int custom_gop_size; /* the size of custom GOP (0~8) */
	struct custom_gop_pic_param pic_param[MAX_GOP_NUM];
};

struct wave_custom_map_opt {
	int roi_avg_qp; /* it sets an average QP of ROI map. */
	int custom_roi_map_enable; /* it enables ROI map. */
	int custom_lambda_map_enable; /* it enables custom lambda map. */
	int custom_mode_map_enable;
	int custom_coef_drop_enable;
	dma_addr_t custom_map_addr;
};

struct temporal_layer_param {
	u32 en_change_qp: 1;
	u32 qp_i;
	u32 qp_p;
	u32 qp_b;
};

struct av1_color_param {
	u32 chroma_sample_position;
	u32 color_range;
	u32 matrix_coefficients;
	u32 transfer_characteristics;
	u32 color_primaries;
	u32 color_description_present_flag;
};

struct dec_scaler_info {
	bool enable;
	int width;
	int height;
	u32 scale_mode;
};

struct enc_scaler_info {
	bool enable;
	int width;
	int height;
	int coef_mode;
};

struct enc_sei_nal_info {
	u32 prefix_sei_nal_enable;
	u32 prefix_sei_nal_data_size;
	dma_addr_t prefix_sei_nal_addr;
	u32 suffix_sei_nal_enable;
	u32 suffix_sei_nal_data_size;
	dma_addr_t suffix_sei_nal_addr;
};

struct enc_wave_param {
	u32 internal_bit_depth;
	u32 decoding_refresh_type;
	u32 idr_period;
	u32 intra_period;
	u32 gop_preset_idx;
	u32 frame_rate;
	u32 enc_bit_rate;
	u32 vbv_buffer_size;
	u32 hvs_qp_scale_div2;
	u32 max_delta_qp;
	int rc_initial_qp;
	u32 rc_update_speed;
	u32 max_bit_rate;
	u32 rc_mode;
	u32 rc_initial_level;
	u32 pic_rc_max_dqp;
	u32 bg_th_diff;
	u32 bg_th_mean_diff;
	int bg_delta_qp;
	u32 strong_intra_smoothing;
	u32 constrained_intra_pred;
	u32 intra_trans_skip;
	u32 intra_refresh_mode;
	u32 intra_refresh_arg;
	u32 me_center;
	u32 lf_cross_slice_boundary_flag;
	int beta_offset_div2;
	int tc_offset_div2;
	u32 lf_sharpness;
	u32 qp;
	u32 min_qp_i;
	u32 max_qp_i;
	u32 min_qp_p;
	u32 max_qp_p;
	u32 min_qp_b;
	u32 max_qp_b;
	int y_dc_qp_delta;
	int u_dc_qp_delta;
	int v_dc_qp_delta;
	int u_ac_qp_delta;
	int v_ac_qp_delta;
	int cb_qp_offset;
	int cr_qp_offset;
	u32 q_round_intra;
	u32 q_round_inter;
	int lambda_dqp_intra;
	int lambda_dqp_inter;
	u32 slice_mode;
	u32 slice_arg;
	u32 level;
	u32 tier;
	u32 profile;
	u32 col_tile_num;
	u32 row_tile_num;
	struct vpu_rect conf_win;
	u32 forced_idr_header;
	u16 custom_lambda_ssd[MAX_CUSTOM_LAMBDA_NUM];
	u16 custom_lambda_sad[MAX_CUSTOM_LAMBDA_NUM];
	struct custom_gop_param gop_param;
	struct temporal_layer_param temp_layer[MAX_NUM_CHANGEABLE_TEMPORAL_LAYER];
	u32 temp_layer_cnt;
	u32 report_mv_histo_threshold0;
	u32 report_mv_histo_threshold1;
	u32 report_mv_histo_threshold2;
	u32 report_mv_histo_threshold3;
	enum endian_mode custom_map_endian;
	u32 num_units_in_tick;
	u32 time_scale;
	u32 num_ticks_poc_diff_one;
	struct av1_color_param av1_color;
	u32 max_intra_pic_bit;
	u32 max_inter_pic_bit;

	/* flags */
	u32 en_long_term: 1;
	u32 en_rate_control: 1;
	u32 en_transform8x8: 1;
	u32 en_hvs_qp: 1;
	u32 en_bg_detect: 1;
	u32 en_temporal_mvp: 1;
	u32 en_cabac: 1;
	u32 en_dbk: 1;
	u32 en_sao: 1;
	u32 en_cdef: 1;
	u32 en_wiener: 1;
	u32 en_scaling_list: 1;
	u32 en_adaptive_round: 1;
	u32 en_qp_map: 1;
	u32 en_mode_map: 1;
	u32 en_q_round_offset: 1;
	u32 en_still_picture: 1;
	u32 en_custom_lambda: 1;
	u32 en_report_mv_histo: 1;
	u32 dis_coef_clear: 1;
	u32 en_cu_level_rate_control: 1;
};

struct hevc_vui_param {
	u32 aspect_ratio_info_present_flag: 1;
	u32 aspect_ratio_idc: 8;
	u32 reserved_0: 23;
	u32 sar_width: 16;
	u32 sar_height: 16;
	u32 overscan_info_present_flag: 1;
	u32 overscan_appropriate_flag: 1;
	u32 video_signal_type_present_flag: 1;
	u32 video_format: 3;
	u32 video_full_range_flag: 1;
	u32 colour_description_present_flag: 1;
	u32 colour_primaries: 8;
	u32 transfer_characteristics: 8;
	u32 matrix_coeffs: 8;
	u32 chroma_loc_info_present_flag: 1;
	u32 chroma_sample_loc_type_top_field: 8;
	u32 chroma_sample_loc_type_bottom_field: 8;
	u32 neutral_chroma_indication_flag: 1;
	u32 field_seq_flag: 1;
	u32 frame_field_info_present_flag: 1;
	u32 default_display_window_flag: 1;
	u32 vui_timing_info_present_flag: 1;
	u32 vui_poc_proportional_to_timing_flag: 1;
	u32 vui_hrd_parameters_present_flag: 1;
	u32 bitstream_restriction_flag: 1;
	u32 tiles_fixed_structure_flag: 1;
	u32 motion_vectors_over_pic_boundaries_flag: 1;
	u32 restricted_ref_pic_lists_flag: 1;
	u32 reserved_1: 4;
	u32 vui_num_units_in_tick: 32;
	u32 vui_time_scale: 32;
	u32 min_spatial_segmentation_idc: 12;
	u32 max_bytes_per_pic_denom: 5;
	u32 max_bits_per_mincu_denom: 5;
	u32 log2_max_mv_length_horizontal: 5;
	u32 log2_max_mv_length_vertical: 5;
	u32 vui_num_ticks_poc_diff_one_minus1: 32;
};

struct avc_vui_param {
	u32 aspect_ratio_info_present_flag: 1;
	u32 aspect_ratio_idc: 8;
	u32 reserved_0: 23;
	u32 sar_width: 16;
	u32 sar_height: 16;
	u32 overscan_info_present_flag: 1;
	u32 overscan_appropriate_flag: 1;
	u32 video_signal_type_present_flag: 1;
	u32 video_format: 3;
	u32 video_full_range_flag: 1;
	u32 colour_description_present_flag: 1;
	u32 colour_primaries: 8;
	u32 transfer_characteristics: 8;
	u32 matrix_coeffs: 8;
	u32 chroma_loc_info_present_flag: 1;
	u32 chroma_sample_loc_type_top_field: 8;
	u32 chroma_sample_loc_type_bottom_field: 8;
	u32 timing_info_present_flag: 1;
	u32 reserved_1: 14;
	u32 num_units_in_tick: 32;
	u32 time_scale: 32;
	u32 fixed_frame_rate_flag: 1;
	u32 nal_hrd_parameters_present_flag: 1;
	u32 vcl_hrd_parameters_present_flag: 1;
	u32 low_delay_hrd_flag: 1;
	u32 pic_struct_present_flag: 1;
	u32 bitstream_restriction_flag: 1;
	u32 motion_vectors_over_pic_boundaries_flag: 1;
	u32 max_bytes_per_pic_denom: 8;
	u32 max_bits_per_mb_denom: 8;
	u32 reserved_2: 9;
	u32 log2_max_mv_length_horizontal: 8;
	u32 log2_max_mv_length_vertical: 8;
	u32 max_num_reorder_frames: 8;
	u32 max_dec_frame_buffering: 8;
};

struct enc_open_param {
	int pic_width; /* the width of a picture to be encoded in unit of sample. */
	int pic_height; /* the height of a picture to be encoded in unit of sample. */
	struct enc_wave_param wave_param;
	enum cb_cr_order cbcr_order;
	enum endian_mode stream_endian;
	enum endian_mode source_endian;
	bool line_buf_int_en;
	enum packed_format_num packed_format; /* <<vpuapi_h_packed_format_num>> */
	enum frame_buffer_format src_format;
	enum frame_buffer_format output_format;
	bool enable_pts; /* an enable flag to report PTS(presentation timestamp) */
	bool enable_non_ref_fbc_write;
	bool enc_hrd_rbsp_in_vps; /* it encodes the HRD syntax rbsp into VPS. */
	u32 hrd_rbsp_data_size; /* the bit size of the HRD rbsp data */
	dma_addr_t hrd_rbsp_data_addr; /* the address of the HRD rbsp data */
	bool enc_vui_rbsp;
	u32 vui_rbsp_data_size; /* the bit size of the VUI rbsp data */
	dma_addr_t vui_rbsp_data_addr; /* the address of the VUI rbsp data */
	u32 ext_addr_vcpu: 8;
	bool is_secure_inst;
	u32 inst_priority: 5;
	struct instance_buffer inst_buffer;
	bool enc_aud;
};

struct enc_change_param {
	u32 enable;
	u32 enc_bit_rate;
	u32 max_bit_rate;
};

struct enc_initial_info {
	u32 min_frame_buffer_count; /* minimum number of frame buffer */
	u32 min_src_frame_count; /* minimum number of source buffer */
	u32 req_mv_buffer_count; /* between 1 to 16 */
	int max_latency_pictures; /* maximum number of picture latency */
	int err_reason; /* error information */
	int warn_info; /* warn information */
};

struct enc_code_opt {
	u32 implicit_header_encode: 1;
	u32 encode_vcl: 1; /* A flag to encode VCL nal unit explicitly */
	u32 encode_vps: 1; /* A flag to encode VPS nal unit explicitly */
	u32 encode_sps: 1; /* A flag to encode SPS nal unit explicitly */
	u32 encode_pps: 1; /* A flag to encode PPS nal unit explicitly */
	u32 encode_eos: 1;
	u32 encode_eob: 1;
};

struct enc_csc_param {
	u32 format_order;
	s32 coef_ry;
	s32 coef_gy;
	s32 coef_by;
	s32 coef_rcb;
	s32 coef_gcb;
	s32 coef_bcb;
	s32 coef_rcr;
	s32 coef_gcr;
	s32 coef_bcr;
	u32 offset_y;
	u32 offset_cb;
	u32 offset_cr;
};

struct enc_param {
	struct frame_buffer *source_frame;
	unsigned int skip_picture: 1;
	dma_addr_t pic_stream_buffer_addr;
	int pic_stream_buffer_size;
	int force_pic_qp_enable; /* flag used to force picture quantization parameter */
	int force_pic_qp_i;
	int force_pic_qp_p;
	int force_pic_qp_b;
	bool force_pic_type_enable; /* A flag to use a force picture type */
	int force_pic_type;
	int src_idx; /* A source frame buffer index */
	int src_end_flag;
	struct enc_code_opt code_option;
	u32 use_cur_src_as_longterm_pic;
	u32 use_longterm_ref;
	u64 pts; /* the presentation timestamp (PTS) of input source */
	struct wave_custom_map_opt custom_map_opt;
	u32 update_last_2bit;
	u32 last_2bit_data;
	struct enc_csc_param csc;
	struct timestamp_info timestamp;
	u32 intra_4x4;
	struct enc_sei_nal_info sei_nal;
};

struct enc_report_fme_sum {
	u32 lower_x0;
	u32 higher_x0;
	u32 lower_y0;
	u32 higher_y0;
	u32 lower_x1;
	u32 higher_x1;
	u32 lower_y1;
	u32 higher_y1;
};

struct enc_report_mv_histo {
	u32 cnt0;
	u32 cnt1;
	u32 cnt2;
	u32 cnt3;
	u32 cnt4;
};

struct enc_output_info {
	dma_addr_t bitstream_buffer;
	u32 bitstream_size; /* the byte size of encoded bitstream */
	int bitstream_wrap_around;
	int pic_type; /* <<vpuapi_h_pic_type>> */
	int num_of_slices; /* the number of slices of the currently being encoded picture */
	int recon_frame_index;
	struct frame_buffer recon_frame;
	dma_addr_t rd_ptr;
	dma_addr_t wr_ptr;
	int pic_skipped; /* whether the current encoding has been skipped or not */
	int num_of_intra; /* the number of intra coded block */
	int num_of_merge; /* the number of merge block in 8x8 */
	int num_of_skip_block; /* the number of skip block in 8x8 */
	int avg_ctu_qp; /* the average value of CTU q_ps */
	int enc_pic_byte; /* the number of encoded picture bytes */
	int enc_gop_pic_idx; /* the GOP index of the currently encoded picture */
	int enc_pic_poc; /* the POC(picture order count) of the currently encoded picture */
	int enc_src_idx; /* the source buffer index of the currently encoded picture */
	int enc_vcl_nut;
	int enc_pic_cnt; /* the encoded picture number */
	int error_reason; /* the error reason of the currently encoded picture */
	int warn_info; /* the warning information of the currently encoded picture */
	int frame_cycle; /* the param for reporting the cycle number of encoding one frame.*/
	u64 pts;
	u32 pic_distortion_low;
	u32 pic_distortion_high;
	u32 non_ref_pic_flag: 1;
	u32 encoding_success: 1;
	struct enc_report_fme_sum fme_sum;
	struct enc_report_mv_histo mv_histo;
	struct report_cycle cycle;
	struct timestamp_info timestamp;
	dma_addr_t src_y_addr;
	dma_addr_t custom_map_addr;
	dma_addr_t prefix_sei_nal_addr;
	dma_addr_t suffix_sei_nal_addr;
};

enum ENC_PIC_CODE_OPTION {
	CODEOPT_ENC_HEADER_IMPLICIT = BIT(0),
	CODEOPT_ENC_VCL = BIT(1), /* A flag to encode VCL nal unit explicitly */
};

enum GOP_PRESET_IDX {
	PRESET_IDX_CUSTOM_GOP = 0, /* user defined GOP structure */
	PRESET_IDX_ALL_I = 1, /* all intra, gopsize = 1 */
	PRESET_IDX_IPP = 2, /* consecutive P, cyclic gopsize = 1 */
	PRESET_IDX_IBBB = 3, /* consecutive B, cyclic gopsize = 1 */
	PRESET_IDX_IBPBP = 4, /* gopsize = 2 */
	PRESET_IDX_IBBBP = 5, /* gopsize = 4 */
	PRESET_IDX_IPPPP = 6, /* consecutive P, cyclic gopsize = 4 */
	PRESET_IDX_IBBBB = 7, /* consecutive B, cyclic gopsize = 4 */
	PRESET_IDX_RA_IB = 8, /* random access, cyclic gopsize = 8 */
	PRESET_IDX_IPP_SINGLE = 9, /* consecutive P, cyclic gopsize = 1, with single ref */
	PRESET_IDX_MAX,
};

struct sec_axi_info {
	u32 use_ip_enable: 1;
	u32 use_lf_row_enable: 1;
	u32 use_enc_rdo_enable: 1;
	u32 use_enc_lf_enable: 1;
};

struct dec_info {
	struct dec_open_param open_param;
	struct dec_initial_info initial_info;
	dma_addr_t stream_wr_ptr;
	dma_addr_t stream_rd_ptr;
	bool stream_end_flag;
	struct vpu_buf vb_mv[WAVE6_MAX_FBS];
	struct vpu_buf vb_fbc_y_tbl[WAVE6_MAX_FBS];
	struct vpu_buf vb_fbc_c_tbl[WAVE6_MAX_FBS];
	struct vpu_buf vb_def_cdf;
	struct vpu_buf vb_seg_map;
	struct vpu_buf vb_pre_ent;
	struct frame_buffer disp_buf[WAVE6_MAX_FBS];
	int stride;
	bool initial_info_obtained;
	struct sec_axi_info sec_axi_info;
	dma_addr_t user_data_buf_addr;
	u32 user_data_enable;
	u32 user_data_buf_size;
	struct dec_output_info dec_out_info[WAVE6_MAX_FBS];
	bool thumbnail_mode;
	int seq_change_mask;
	u32 cycle_per_tick;
	enum frame_buffer_format wtl_format;
};

struct enc_info {
	struct enc_open_param open_param;
	struct enc_change_param change_param;
	struct enc_initial_info initial_info;
	int num_frame_buffers;
	int stride;
	bool rotation_enable;
	bool mirror_enable;
	enum mirror_direction mirror_direction;
	int rotation_angle;
	bool initial_info_obtained;
	struct sec_axi_info sec_axi_info;
	bool line_buf_int_en;
	struct vpu_buf vb_mv[WAVE6_MAX_FBS]; /* col_mv buffer */
	struct vpu_buf vb_fbc_y_tbl[WAVE6_MAX_FBS]; /* FBC luma table buffer */
	struct vpu_buf vb_fbc_c_tbl[WAVE6_MAX_FBS]; /* FBC chroma table buffer */
	struct vpu_buf vb_sub_sam_buf[WAVE6_MAX_FBS]; /* sub-sampled buffer for ME */
	struct vpu_buf vb_def_cdf;
	u64 cur_pts; /* current timestamp in 90_k_hz */
	u64 pts_map[32]; /* PTS mapped with source frame index */
	u32 cycle_per_tick;
	u32 width;
	u32 height;
	struct enc_scaler_info scaler_info;
	int color_format;
};

struct vpu_device {
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_m2m_dev *m2m_dev;
	struct video_device *video_dev_dec;
	struct video_device *video_dev_enc;
	struct mutex dev_lock; /* the lock for the src,dst v4l2 queues */
	struct mutex hw_lock; /* lock hw configurations */
	int irq;
	enum product_id	product;
	u32 fw_version;
	u32 fw_revision;
	u32 hw_version;
	struct vpu_attr	attr;
	u32 last_performance_cycles;
	void __iomem *reg_base;
	struct device *ctrl;
	int product_code;
	struct clk_bulk_data *clks;
	int num_clks;
	unsigned long vpu_clk_rate;
	struct completion irq_done;
	struct kfifo irq_status;
	struct delayed_work task_timer;
	struct wave6_vpu_entity entity;
	struct dentry *debugfs;
};

struct vpu_instance;

struct vpu_instance_ops {
	int (*start_process)(struct vpu_instance *inst);
	void (*finish_process)(struct vpu_instance *inst);
};

struct vpu_performance_info {
	ktime_t ts_first;
	ktime_t ts_last;
	s64 latency_first;
	s64 latency_max;
	s64 min_process_time;
	s64 max_process_time;
	u64 total_sw_time;
	u64 total_hw_time;
};

struct vpu_instance {
	struct v4l2_fh v4l2_fh;
	struct v4l2_ctrl_handler v4l2_ctrl_hdl;
	struct vpu_device *dev;

	struct v4l2_pix_format_mplane src_fmt;
	struct v4l2_pix_format_mplane dst_fmt;
	struct v4l2_rect crop;
	enum v4l2_colorspace colorspace;
	enum v4l2_xfer_func xfer_func;
	enum v4l2_ycbcr_encoding ycbcr_enc;
	enum v4l2_quantization quantization;

	enum vpu_instance_state state;
	enum vpu_instance_state state_in_seek;
	enum vpu_instance_type type;
	const struct vpu_instance_ops *ops;

	enum wave_std std;
	u32 id;
	union {
		struct enc_info enc_info;
		struct dec_info dec_info;
	} *codec_info;
	struct frame_buffer frame_buf[WAVE6_MAX_FBS];
	struct vpu_buf frame_vbuf[WAVE6_MAX_FBS];
	u32 queued_src_buf_num;
	u32 queued_dst_buf_num;
	u32 processed_buf_num;
	u32 error_buf_num;
	u32 sequence;
	bool next_buf_last;
	bool cbcr_interleave;
	bool nv21;
	bool eos;

	struct vpu_buf aux_vbuf[AUX_BUF_TYPE_MAX][WAVE6_MAX_FBS];
	struct vpu_buf work_vbuf;
	struct vpu_buf temp_vbuf;
	struct vpu_buf ar_vbuf;
	struct vpu_buf vui_vbuf;
	bool thumbnail_mode;
	enum display_mode disp_mode;

	unsigned int rot_angle;
	unsigned int mirror_direction;
	unsigned int frame_rate;
	unsigned int rc_mode;
	struct enc_wave_param enc_param;
	struct dec_scaler_info scaler_info;
	bool force_key_frame;
	bool error_recovery;
	struct sar_info sar;
	u64 total_frames;
	u64 total_frame_cycle;
	struct workqueue_struct *workqueue;
	struct work_struct init_task;
	atomic_t start_init_seq;

	struct vpu_performance_info performance;

	u32 dynamic_bit_rate;
	u32 dynamic_max_bit_rate;
	struct dentry *debugfs;
};

void wave6_vdi_writel(struct vpu_device *vpu_device, unsigned int addr, unsigned int data);
unsigned int wave6_vdi_readl(struct vpu_device *vpu_dev, unsigned int addr);
int wave6_vdi_clear_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);
int wave6_vdi_allocate_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);
int wave6_vdi_write_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb, size_t offset,
			   u8 *data, int len, int endian);
void wave6_vdi_free_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb);

int wave6_vpu_dec_open(struct vpu_instance *inst, struct dec_open_param *pop);
int wave6_vpu_dec_close(struct vpu_instance *inst, u32 *fail_res);
int wave6_vpu_dec_issue_seq_init(struct vpu_instance *inst);
int wave6_vpu_dec_complete_seq_init(struct vpu_instance *inst, struct dec_initial_info *info);
int wave6_vpu_dec_get_aux_buffer_size(struct vpu_instance *inst,
					  struct dec_aux_buffer_size_info info,
					  uint32_t *size);
int wave6_vpu_dec_register_aux_buffer(struct vpu_instance *inst, struct aux_buffer_info info);
int wave6_vpu_dec_register_frame_buffer_ex(struct vpu_instance *inst, int num_of_dec_fbs,
					   int stride, int height, int map_type);
int wave6_vpu_dec_register_display_buffer_ex(struct vpu_instance *inst, struct frame_buffer fb);
int wave6_vpu_dec_update_frame_buffer(struct vpu_instance *inst, struct frame_buffer *fb, int mv_index);
int wave6_vpu_dec_get_update_frame_buffer_info(struct vpu_instance *inst,
					       struct dec_update_fb_info *info);
int wave6_vpu_dec_start_one_frame(struct vpu_instance *inst, struct dec_param *param,
				  u32 *res_fail);
int wave6_vpu_dec_get_output_info(struct vpu_instance *inst, struct dec_output_info *info);
int wave6_vpu_dec_set_rd_ptr(struct vpu_instance *inst, dma_addr_t addr, bool update_wr_ptr);
int wave6_vpu_dec_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);
int wave6_vpu_dec_get_bitstream_buffer(struct vpu_instance *inst, dma_addr_t *prd_prt,
				       dma_addr_t *pwr_ptr, uint32_t *size);
int wave6_vpu_dec_update_bitstream_buffer(struct vpu_instance *inst, int size);
int wave6_vpu_dec_flush_instance(struct vpu_instance *inst);

int wave6_vpu_enc_open(struct vpu_instance *inst, struct enc_open_param *enc_op_param);
int wave6_vpu_enc_close(struct vpu_instance *inst, u32 *fail_res);
int wave6_vpu_enc_issue_seq_init(struct vpu_instance *inst);
int wave6_vpu_enc_update_seq(struct vpu_instance *inst);
int wave6_vpu_enc_complete_seq_init(struct vpu_instance *inst, struct enc_initial_info *info);
int wave6_vpu_enc_complete_seq_update(struct vpu_instance *inst, struct enc_initial_info *info);
int wave6_vpu_enc_get_aux_buffer_size(struct vpu_instance *inst,
				      struct enc_aux_buffer_size_info info,
				      uint32_t *size);
int wave6_vpu_enc_register_aux_buffer(struct vpu_instance *inst, struct aux_buffer_info info);
int wave6_vpu_enc_register_frame_buffer_ex(struct vpu_instance *inst, int num, unsigned int stride,
					int height, enum tiled_map_type map_type);
int wave6_vpu_enc_start_one_frame(struct vpu_instance *inst, struct enc_param *param,
				  u32 *fail_res);
int wave6_vpu_enc_get_output_info(struct vpu_instance *inst, struct enc_output_info *info);
int wave6_vpu_enc_give_command(struct vpu_instance *inst, enum codec_command cmd, void *parameter);

const char *wave6_vpu_instance_state_name(u32 state);
void wave6_vpu_set_instance_state(struct vpu_instance *inst, u32 state);
void wave6_vpu_wait_active(struct vpu_instance *inst);
void *wave6_vpu_get_sram(struct vpu_instance *vpu_inst, dma_addr_t *dma_addr, u32 *size);
u64 wave6_cycle_to_ns(struct vpu_device *vpu_dev, u64 cycle);
#endif
