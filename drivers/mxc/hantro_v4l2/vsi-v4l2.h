/*
 *    public header file for vsi v4l2 driver and daemon.
 *
 *    Copyright (c) 2019, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License, version 2, as
 *    published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License version 2 for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 at the following locations:
 *    https://opensource.org/licenses/gpl-2.0.php
 */

#ifndef VSI_V4L2_H
#define VSI_V4L2_H

#define MAX_STREAMS 200
#define MAX_GOP_SIZE 8
#define MAX_INTRA_PIC_RATE 0x7fffffff
#define NO_RESPONSE_SEQID 0xFFFFFFFE
#define SEQID_UPLIMT 0x7FFFFFFE

#define OUTF_BASE	0x3ffff000L
#define VSI_DAEMON_FNAME	"vsi_daemon_ctrl"
#define VSI_DAEMON_PATH	"/usr/bin/vsidaemon"
#define VSI_DAEMON_DEVMAJOR	100

/* some common defines between driver and daemon */
#define DEFAULTLEVEL	0

#define FRAMETYPE_I			(1<<1)
#define FRAMETYPE_P			(1<<2)
#define FRAMETYPE_B			(1<<3)
#define LAST_BUFFER_FLAG	(1<<4)
#define FORCE_IDR			(1<<5)
#define UPDATE_INFO			(1<<6)
#define ERROR_BUFFER_FLAG		(1 << 7)

#define VSI_V4L2_MAX_ROI_REGIONS			8
#define VSI_V4L2_MAX_ROI_REGIONS_H1		2
#define VSI_V4L2_MAX_IPCM_REGIONS			2

/******************	communication with v4l2 driver. ***********/

struct vsi_v4l2_dev_info {
	s32 dec_corenum;
	s32 enc_corenum;
	s32 enc_isH1;
	u32 max_dec_resolution;
	ulong decformat;		//hw_dec_formats
	ulong encformat;		//hw_enc_formats
};

/* daemon ioctl id definitions */
#define VSIV4L2_IOCTL_BASE			'd'
#define VSI_IOCTL_CMD_BASE		_IO(VSIV4L2_IOCTL_BASE, 0x44)

/* user space daemno should use this ioctl to initial HW info to v4l2 driver */
#define VSI_IOCTL_CMD_INITDEV		_IOW(VSIV4L2_IOCTL_BASE, 45, struct vsi_v4l2_dev_info)
/* end of daemon ioctl id definitions */

/*these two enum have same sequence, identical to the table vsi_coded_fmt[] in vsi-v4l2-config.c */
enum hw_enc_formats {
	ENC_HAS_HEVC = 0,
	ENC_HAS_H264,
	ENC_HAS_JPEG,
	ENC_HAS_VP8,
	ENC_HAS_VP9,
	ENC_HAS_AV1,
	ENC_FORMATS_MAX,
};

enum hw_dec_formats {
	DEC_HAS_HEVC = 0,
	DEC_HAS_H264,
	DEC_HAS_JPEG,
	DEC_HAS_VP8,
	DEC_HAS_VP9,
	DEC_HAS_AV1,

	DEC_HAS_MPEG2,
	DEC_HAS_MPEG4,
	DEC_HAS_H263,
	DEC_HAS_VC1_G,
	DEC_HAS_VC1_L,
	DEC_HAS_RV,
	DEC_HAS_AVS2,
	DEC_HAS_XVID,
	DEC_HAS_CSC,
	DEC_FORMATS_MAX
};
/*above two enum have same sequence, identical to the table vsi_coded_fmt[] in vsi-v4l2-config.c */

enum v4l2_daemon_cmd_id {
	/*  every command should mark which kind of parameters is valid.
	 *      For example, V4L2_DAEMON_VIDIOC_BUF_RDY can contains input or output buffers.
	 *          also it can contains other parameters.  */
	V4L2_DAEMON_VIDIOC_STREAMON = 0,//for streamon and start
	V4L2_DAEMON_VIDIOC_BUF_RDY,
	V4L2_DAEMON_VIDIOC_CMD_STOP, //this is for flush.
	V4L2_DAEMON_VIDIOC_DESTROY_ENC,	//enc destroy
	V4L2_DAEMON_VIDIOC_ENC_RESET,	//enc reset, as in spec
	//above are enc cmds

	V4L2_DAEMON_VIDIOC_FAKE,//fake command.

	/*Below is for decoder*/
	V4L2_DAEMON_VIDIOC_S_EXT_CTRLS,
	V4L2_DAEMON_VIDIOC_RESET_BITRATE,
	V4L2_DAEMON_VIDIOC_CHANGE_RES,
	V4L2_DAEMON_VIDIOC_G_FMT,
	V4L2_DAEMON_VIDIOC_S_SELECTION,
	V4L2_DAEMON_VIDIOC_S_FMT,
	V4L2_DAEMON_VIDIOC_PACKET, // tell daemon a frame is ready.
	V4L2_DAEMON_VIDIOC_STREAMON_CAPTURE,//for streamon and start
	V4L2_DAEMON_VIDIOC_STREAMON_OUTPUT,
	V4L2_DAEMON_VIDIOC_STREAMOFF_CAPTURE,
	V4L2_DAEMON_VIDIOC_STREAMOFF_OUTPUT,
	V4L2_DAEMON_VIDIOC_CMD_START,
	V4L2_DAEMON_VIDIOC_FRAME,
	V4L2_DAEMON_VIDIOC_DESTROY_DEC,

	V4L2_DAEMON_VIDIOC_EXIT,		//daemon should exit itself
	V4L2_DAEMON_VIDIOC_PICCONSUMED,
	V4L2_DAEMON_VIDIOC_CROPCHANGE,
	V4L2_DAEMON_VIDIOC_WARNONOPTION,
	V4L2_DAEMON_VIDIOC_STREAMOFF_CAPTURE_DONE,
	V4L2_DAEMON_VIDIOC_STREAMOFF_OUTPUT_DONE,
	V4L2_DAEMON_VIDIOC_TOTAL_AMOUNT,
};

enum v4l2_daemon_codec_fmt {
	/*enc format, identical to VCEncVideoCodecFormat except name*/
	V4L2_DAEMON_CODEC_ENC_HEVC = 0,
	V4L2_DAEMON_CODEC_ENC_H264,
	V4L2_DAEMON_CODEC_ENC_AV1,
	V4L2_DAEMON_CODEC_ENC_VP8,
	V4L2_DAEMON_CODEC_ENC_VP9,
	V4L2_DAEMON_CODEC_ENC_MPEG2,
	V4L2_DAEMON_CODEC_ENC_JPEG,

	/*dec format*/
	V4L2_DAEMON_CODEC_DEC_HEVC,
	V4L2_DAEMON_CODEC_DEC_H264,
	V4L2_DAEMON_CODEC_DEC_JPEG,
	V4L2_DAEMON_CODEC_DEC_VP9,
	V4L2_DAEMON_CODEC_DEC_MPEG2,
	V4L2_DAEMON_CODEC_DEC_MPEG4,
	V4L2_DAEMON_CODEC_DEC_VP8,
	V4L2_DAEMON_CODEC_DEC_H263,
	V4L2_DAEMON_CODEC_DEC_VC1_G,
	V4L2_DAEMON_CODEC_DEC_VC1_L,
	V4L2_DAEMON_CODEC_DEC_RV,
	V4L2_DAEMON_CODEC_DEC_AVS2,
	V4L2_DAEMON_CODEC_DEC_XVID,
	V4L2_DAEMON_CODEC_UNKNOW_TYPE,
};

enum vsi_v4l2dec_pixfmt {
	VSI_V4L2_DECOUT_DEFAULT,
	VSI_V4L2_DEC_PIX_FMT_NV12,
	VSI_V4L2_DEC_PIX_FMT_400,
	VSI_V4L2_DEC_PIX_FMT_411SP,
	VSI_V4L2_DEC_PIX_FMT_422SP,
	VSI_V4L2_DEC_PIX_FMT_444SP,

	VSI_V4L2_DECOUT_DTRC,
	VSI_V4L2_DECOUT_P010,
	VSI_V4L2_DECOUT_NV12_10BIT,
	VSI_V4L2_DECOUT_DTRC_10BIT,
	VSI_V4L2_DECOUT_RFC,
	VSI_V4L2_DECOUT_RFC_10BIT,
};

enum {
	DAEMON_OK = 0,						// no error.
	DAEMON_ENC_FRAME_READY = 1,			// frame encoded
	DAEMON_ENC_FRAME_ENQUEUE = 2,		// frame enqueued

	DAEMON_ERR_INST_CREATE = -1,		// inst_init() failed.
	DAEMON_ERR_SIGNAL_CONFIG = -2,		// sigsetjmp() failed.
	DAEMON_ERR_DAEMON_MISSING = -3,     // daemon is not alive.
	DAEMON_ERR_NO_MEM = -4,				// no mem, used also by driver.

	DAEMON_ERR_ENC_PARA = -100,			// Parameters Error.
	DAEMON_ERR_ENC_NOT_SUPPORT = -101,	// Not Support Error.
	DAEMON_ERR_ENC_INTERNAL = -102,		// Ctrlsw reported Error.
	DAEMON_ERR_ENC_BUF_MISSED = -103,	// No desired input buffer.
	DAEMON_ERR_ENC_FATAL_ERROR = -104,	// Fatal error.

	DAEMON_ERR_DEC_FATAL_ERROR = -200,	// Fatal error.
	DAEMON_ERR_DEC_METADATA_ONLY = -201,	// CMD_STOP after metadata-only.
};

//warn type attached in V4L2_DAEMON_VIDIOC_WARNONOPTION message. Stored in msg.error member
enum {
	UNKONW_WARNTYPE = -1,		//not known warning type
	WARN_ROIREGION,			//(part of)roi region can not work with media setting and be ignored by enc
	WARN_IPCMREGION,			//(part of)ipcm region can not work with media setting and be ignored by enc
	WARN_LEVEL,				//current level cant't work with media setting and be updated by enc
};

struct v4l2_daemon_enc_buffers {
	/*IO*/
	s32 inbufidx;	 //from v4l2 driver, don't modify it
	s32 outbufidx;	 //-1:invalid, other:valid.

	dma_addr_t busLuma;
	s32 busLumaSize;
	dma_addr_t busChromaU;
	s32 busChromaUSize;
	dma_addr_t busChromaV;
	s32 busChromaVSize;

	dma_addr_t busLumaOrig;
	dma_addr_t busChromaUOrig;
	dma_addr_t busChromaVOrig;

	dma_addr_t busOutBuf;
	u32 outBufSize;

	u32 bytesused;	//valid bytes in buffer from user app.
	s64 timestamp;
};

struct v4l2_daemon_enc_general_cmd {
	s32 valid;//0:invalid, 1:valid.

	/*frame property*/
	s32 outputRateNumer;      /* Output frame rate numerator */
	s32 outputRateDenom;      /* Output frame rate denominator */
	s32 inputRateNumer;      /* Input frame rate numerator */
	s32 inputRateDenom;      /* Input frame rate denominator */

	s32 firstPic;
	s32 lastPic;

	s32 width;      //encode width
	s32 height;      //encode height
	s32 lumWidthSrc;      //input width
	s32 lumHeightSrc;      //input height

	s32 inputFormat;  //input format
	s32 bitPerSecond;

	s32 rotation;      //prep
	s32 mirror;
	s32 horOffsetSrc;
	s32 verOffsetSrc;
	s32 colorConversion;
	s32 scaledWidth;
	s32 scaledHeight;
	s32 scaledOutputFormat;

	s32 codecFormat;
};

struct v4l2_daemon_enc_h26x_cmd {
	s32 valid;//0:invalid, 1:valid.
	s32 byteStream;      //byteStream

	s32 enableCabac;      /* [0,1] H.264 entropy coding mode, 0 for CAVLC, 1 for CABAC */
	s32 cabacInitFlag;      //cabacInitFlag

	s32 profile;              /*main profile or main still picture profile*/
	s32 tier;               /*main tier or high tier*/
	s32 avclevel;              /*h264 main profile level*/
	s32 hevclevel;              /*hevc main profile level*/

	u32 strong_intra_smoothing_enabled_flag;      // intra setup

	s32 cirStart;  //cir
	s32 cirInterval;

	s32 intraAreaEnable; //intra area
	s32 intraAreaTop;
	s32 intraAreaLeft;
	s32 intraAreaBottom;
	s32 intraAreaRight;

	s32 pcm_loop_filter_disabled_flag;

	s32 ipcmAreaEnable[VSI_V4L2_MAX_IPCM_REGIONS];
	s32 ipcmAreaTop[VSI_V4L2_MAX_IPCM_REGIONS];    //ipcm area 1, 2
	s32 ipcmAreaLeft[VSI_V4L2_MAX_IPCM_REGIONS];
	s32 ipcmAreaBottom[VSI_V4L2_MAX_IPCM_REGIONS];
	s32 ipcmAreaRight[VSI_V4L2_MAX_IPCM_REGIONS];

	s32 ipcmMapEnable;    //ipcm map
	u8 *ipcmMapBuf;

	s32 skipMapEnable;      //skip map
	s32 skipMapBlockUnit;
	u8 *skipMapBuf;

	s32 roiAreaEnable[VSI_V4L2_MAX_ROI_REGIONS]; //8 roi for H2, 2 roi for H1
	s32 roiAreaTop[VSI_V4L2_MAX_ROI_REGIONS];
	s32 roiAreaLeft[VSI_V4L2_MAX_ROI_REGIONS];
	s32 roiAreaBottom[VSI_V4L2_MAX_ROI_REGIONS];
	s32 roiAreaRight[VSI_V4L2_MAX_ROI_REGIONS];
	s32 roiDeltaQp[VSI_V4L2_MAX_ROI_REGIONS];    //roiQp has higher priority than roiDeltaQp
	s32 roiQp[VSI_V4L2_MAX_ROI_REGIONS];    //only H2 use it

	u32 roiMapDeltaQpBlockUnit;//roimap cuctrl
	u32 roiMapDeltaQpEnable;
	u32 RoiCuCtrlVer;
	u32 RoiQpDeltaVer;
	u8 *roiMapDeltaQpBuf;
	u8 *cuCtrlInfoBuf;

	/* Rate control parameters */
	s32 hrdConformance;
	s32 cpbSize;
	s32 intraPicRate;   /* IDR interval */
	s32 vbr; /* Variable Bit Rate Control by qpMin */
	s32 qpHdr;
	s32 qpHdrI_h26x;  // for 264/5 I frame QP
	s32 qpHdrP_h26x;  // for 264/5 P frame PQ
	s32 qpMin_h26x;
	s32 qpMax_h26x;
	s32 qpHdrI_vpx;  // for vpx I frame QP
	s32 qpHdrP_vpx;  // for vpx P frame PQ
	s32 qpMin_vpx;
	s32 qpMax_vpx;
	s32 qpMinI;
	s32 qpMaxI;
	s32 bitVarRangeI;
	s32 bitVarRangeP;
	s32 bitVarRangeB;
	u32 u32StaticSceneIbitPercent;
	s32 tolMovingBitRate;/*tolerance of max Moving bit rate */
	s32 monitorFrames;/*monitor frame length for moving bit rate*/
	s32 picRc;
	s32 ctbRc;
	s32 blockRCSize;
	u32 rcQpDeltaRange;
	u32 rcBaseMBComplexity;
	s32 picSkip;
	s32 picQpDeltaMin;
	s32 picQpDeltaMax;
	s32 ctbRcRowQpStep;
	s32 tolCtbRcInter;
	s32 tolCtbRcIntra;
	s32 bitrateWindow;
	s32 intraQpDelta;
	s32 fixedIntraQp;
	s32 bFrameQpDelta;
	s32 disableDeblocking;
	s32 enableSao;
	s32 tc_Offset;
	s32 beta_Offset;
	s32 chromaQpOffset;

	s32 smoothPsnrInGOP;      //smooth psnr
	s32 sliceSize;      //multi slice

	s32 enableDeblockOverride;      //deblock
	s32 deblockOverride;

	s32 enableScalingList;      //scale list

	u32 compressor;      //rfc

	s32 interlacedFrame;//pregress/interlace
	s32 fieldOrder;      //field order
	s32 ssim;

	s32 sei;      //sei
	s8 *userData;

	u32 gopSize;      //gop
	s8 *gopCfg;
	u32 gopLowdelay;
	s32 outReconFrame;

	u32 longTermGap;      //longterm
	u32 longTermGapOffset;
	u32 ltrInterval;
	s32 longTermQpDelta;

	s32 gdrDuration;      //gdr

	s32 bitDepthLuma;      //10 bit
	s32 bitDepthChroma;

	u32 enableOutputCuInfo;      //cu info

	u32 rdoLevel;      //rdo

	s32 constChromaEn;/* constant chroma control */
	u32 constCb;
	u32 constCr;

	s32 skip_frame_enabled_flag;   /*for skip frame encoding ctr*/
	s32 skip_frame_poc;

	/* HDR10 */
	u32 hdr10_display_enable;
	u32 hdr10_dx0;
	u32 hdr10_dy0;
	u32 hdr10_dx1;
	u32 hdr10_dy1;
	u32 hdr10_dx2;
	u32 hdr10_dy2;
	u32 hdr10_wx;
	u32 hdr10_wy;
	u32 hdr10_maxluma;
	u32 hdr10_minluma;

	u32 hdr10_lightlevel_enable;
	u32 hdr10_maxlight;
	u32 hdr10_avglight;

	u32 hdr10_color_enable;
	u32 hdr10_primary;
	u32 hdr10_transfer;
	u32 hdr10_matrix;

	u32 RpsInSliceHeader;
	u32 P010RefEnable;
	u32 vui_timing_info_enable;

	u32 picOrderCntType;
	u32 log2MaxPicOrderCntLsb;
	u32 log2MaxFrameNum;

	u32 lookaheadDepth;
	u32 halfDsInput;
	u32 cuInfoVersion;
	u32 parallelCoreNum;

	u32 force_idr;

	u32 vuiVideoSignalTypePresentFlag;//1
	u32 vuiVideoFormat;               //default 5
	s32 videoRange;
	u32 vuiColorDescripPresentFlag;    //1 if elems below exist
	u32 vuiColorPrimaries;
	u32 vuiTransferCharacteristics;
	u32 vuiMatrixCoefficients;

	u32 idrHdr;
};

struct v4l2_daemon_enc_jpeg_cmd {
	s32 valid;//0:invalid, 1:valid.
	s32 restartInterval;
	s32 frameType;
	s32 partialCoding;
	s32 codingMode;
	s32 markerType;
	s32 qLevel;
	s32 unitsType;
	s32 xdensity;
	s32 ydensity;
	s32 thumbnail;
	s32 widthThumb;
	s32 heightThumb;
	s32 lumWidthSrcThumb;
	s32 lumHeightSrcThumb;
	s32 horOffsetSrcThumb;
	s32 verOffsetSrcThumb;
	s32 write;
	s32 comLength;
	s32 mirror;
	s32 formatCustomizedType;
	s32 constChromaEn;
	u32 constCb;
	u32 constCr;
	s32 losslessEnable;
	s32 predictMode;
	s32 ptransValue;
	u32 bitPerSecond;
	u32 mjpeg;
	s32 rcMode;
	s32 picQpDeltaMin;
	s32 picQpDeltaMax;
	u32 qpmin;
	u32 qpmax;
	s32 fixedQP;
	u32 exp_of_input_alignment;
	u32 streamBufChain;
	u32 streamMultiSegmentMode;
	u32 streamMultiSegmentAmount;
};

struct v4l2_daemon_enc_params {
	struct v4l2_daemon_enc_buffers io_buffer;
	struct v4l2_daemon_enc_general_cmd general;
	union {
		struct v4l2_daemon_enc_h26x_cmd enc_h26x_cmd;
		struct v4l2_daemon_enc_jpeg_cmd enc_jpeg_cmd;
	} specific;
};

struct v4l2_daemon_dec_buffers {
	/*IO*/
	s32 inbufidx;	 //from v4l2 driver, don't modify it
	s32 outbufidx;	 //-1:invalid, other:valid.

	dma_addr_t busInBuf;
	u32 inBufSize;
	s32 inputFormat;  //input format
	s32 srcwidth;      //encode width
	s32 srcheight;      //encode height
//infer output
	dma_addr_t busOutBuf;	//for Y or YUV
	s32    OutBufSize;
	dma_addr_t busOutBufUV;
	s32    OutUVBufSize;
	s32 outBufFormat;
	s32 output_width;
	s32 output_height;
	s32 output_wstride;
	s32 output_hstride;
	s32 outputPixelDepth;

	dma_addr_t rfc_luma_offset;
	dma_addr_t rfc_chroma_offset;

	u32 bytesused;	//valid bytes in buffer from user app.
	s64 timestamp;

	s32 no_reordering_decoding;
	s32 securemode_on;
};

//stub struct
struct v4l2_daemon_dec_pp_cfg {
	u32 x;
	u32 y;
	u32 width;
	u32 height;
};

struct v4l2_vpu_hdr10_meta {
	u32 hasHdr10Meta;
	u32 redPrimary[2];
	u32 greenPrimary[2];
	u32 bluePrimary[2];
	u32 whitePoint[2];
	u32 maxMasteringLuminance;
	u32 minMasteringLuminance;
	u32 maxContentLightLevel;
	u32 maxFrameAverageLightLevel;
};

struct v4l2_daemon_dec_info {
	u32 frame_width;
	u32 frame_height;
	u32 bit_depth;
	struct {
		u32 left;
		u32 top;
		u32 width;
		u32 height;
	} visible_rect;
	u32 needed_dpb_nums;
	u32 dpb_buffer_size;
	uint32_t pic_wstride;
	struct v4l2_daemon_dec_pp_cfg pp_params;
	u32 colour_description_present_flag;
	u32 matrix_coefficients;
	u32 colour_primaries;
	u32 transfer_characteristics;
	u32 video_range;
	enum vsi_v4l2dec_pixfmt src_pix_fmt;
	struct v4l2_vpu_hdr10_meta vpu_hdr10_meta;
};

struct v4l2_daemon_pic_info {
	u32 width;
	u32 height;
	u32 crop_left;
	u32 crop_top;
	u32 crop_width;
	u32 crop_height;
	uint32_t pic_wstride;
};

struct v4l2_daemon_dec_resochange_params {
	struct v4l2_daemon_dec_buffers io_buffer;
	struct v4l2_daemon_dec_info dec_info;
};

struct v4l2_daemon_dec_pictureinfo_params {
	//v4l2_daemon_dec_buffers io_buffer;
	struct v4l2_daemon_pic_info pic_info;
};

struct v4l2_daemon_dec_params {
	union {
		struct v4l2_daemon_dec_buffers io_buffer;
		struct v4l2_daemon_dec_resochange_params dec_info;
		struct v4l2_daemon_dec_pictureinfo_params pic_info;
	};
//	struct TBCfg general;
};

struct vsi_v4l2_msg_hdr {
	s32 size;
	s32 error;
	ulong seq_id;
	ulong inst_id;
	enum v4l2_daemon_cmd_id cmd_id;
	enum v4l2_daemon_codec_fmt codec_fmt;
	s32 param_type;
};

struct vsi_v4l2_msg {
	s32 size;
	s32 error;
	ulong seq_id;
	ulong inst_id;
	enum v4l2_daemon_cmd_id cmd_id;
	enum v4l2_daemon_codec_fmt codec_fmt;
	u32 param_type;
	// above part must be identical to vsi_v4l2_msg_hdr

	union {
		struct v4l2_daemon_enc_params enc_params;
		struct v4l2_daemon_dec_params dec_params;
	} params;
};

#endif	//#ifndef VSI_V4L2_H


