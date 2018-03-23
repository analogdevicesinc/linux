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

typedef enum {
	FRAME_ALLOC = 0,
	FRAME_FREE,
	FRAME_DECODED,
	FRAME_READY,
	FRAME_RELEASE,
} FRAME_BUFFER_STAT;

typedef enum {
	VPU_APP,
	VPU_DRIVER,
	VPU_DECODER,
} FRAME_BUFFER_OWNER;

typedef enum {
	MEDIAIP_FRAME_REQ = 0,
	MEDIAIP_MBI_REQ,
	MEDIAIP_DCP_REQ,
	MEDIAIP_REQ_LAST = MEDIAIP_DCP_REQ

} MEDIAIP_MEM_REQ;

typedef struct {
	u_int32          uNum;
	MEDIAIP_MEM_REQ  eType;
} MEDIA_PLAYER_FSREQ;

typedef struct {
	u_int32          uFSIdx;
	MEDIAIP_MEM_REQ  eType;
	BOOL             bNotDisplayed;
} MEDIA_PLAYER_FSREL;

typedef enum {
	/* Non-Stream Specific messages   */
	MEDIA_PLAYER_API_MODE_INVALID     = 0x00,
	MEDIA_PLAYER_API_MODE_PARSE_STEP  = 0x01,
	MEDIA_PLAYER_API_MODE_DECODE_STEP = 0x02,
	MEDIA_PLAYER_API_MODE_CONTINUOUS  = 0x03
} MEDIA_PLAYER_API_MODE;

typedef enum {
	/* Non-Stream Specific messages   */
	MEDIA_PLAYER_FS_CTRL_MODE_INTERNAL    = 0x00,
	MEDIA_PLAYER_FS_CTRL_MODE_EXTERNAL    = 0x01
} MEDIA_PLAYER_FS_CTRL_MODE;

typedef enum {
	/* Non-Stream Specific messages   */
	VID_API_CMD_NULL              = 0x00,
	VID_API_CMD_PARSE_NEXT_SEQ    = 0x01,
	VID_API_CMD_PARSE_NEXT_I      = 0x02,
	VID_API_CMD_PARSE_NEXT_IP     = 0x03,
	VID_API_CMD_PARSE_NEXT_ANY    = 0x04,
	VID_API_CMD_DEC_PIC           = 0x05,
	VID_API_CMD_UPDATE_ES_WR_PTR  = 0x06,
	VID_API_CMD_UPDATE_ES_RD_PTR  = 0x07,
	VID_API_CMD_UPDATE_UDATA      = 0x08,
	VID_API_CMD_GET_FSINFO        = 0x09,
	VID_API_CMD_SKIP_PIC          = 0x0a,
	VID_API_CMD_DEC_CHUNK         = 0x0b,
	VID_API_CMD_START             = 0x10,
	VID_API_CMD_STOP              = 0x11,
	VID_API_CMD_ABORT             = 0x12,
	VID_API_CMD_RST_BUF           = 0x13,
	VID_API_CMD_FS_RELEASE        = 0x15,
	VID_API_CMD_MEM_REGION_ATTACH = 0x16,
	VID_API_CMD_MEM_REGION_DETACH = 0x17,
	VID_API_CMD_MVC_VIEW_SELECT   = 0x18,
	VID_API_CMD_FS_ALLOC          = 0x19,
	VID_API_CMD_DBG_GET_STATUS    = 0x1C,
	VID_API_CMD_DBG_START_LOG     = 0x1D,
	VID_API_CMD_DBG_STOP_LOG      = 0x1E,
	VID_API_CMD_DBG_DUMP_LOG      = 0x1F,
	/* Begin Encode CMDs */
	VID_API_CMD_YUV_READY         = 0x20,
#if BOOT_ARCH == REBOOT
	VID_API_CMD_SNAPSHOT          = 0xAA,
	VID_API_CMD_ROLL_SNAPSHOT     = 0xAB,
	VID_API_CMD_LOCK_SCHEDULER    = 0xAC,
	VID_API_CMD_UNLOCK_SCHEDULER  = 0xAD,
#endif
	VID_API_CMD_CQ_FIFO_DUMP      = 0xAE,
	VID_API_CMD_DBG_FIFO_DUMP     = 0xAF,
	VID_API_CMD_SVC_ILP           = 0xBB,
	VID_API_CMD_INVALID           = 0xFF

} TB_API_DEC_CMD;

typedef enum {
	/* Non-Stream Specific messages    */
	VID_API_EVENT_NULL            = 0x00,
	VID_API_EVENT_RESET_DONE      = 0x01,
	VID_API_EVENT_SEQ_HDR_FOUND   = 0x02,
	VID_API_EVENT_PIC_HDR_FOUND   = 0x03,
	VID_API_EVENT_PIC_DECODED     = 0x04,
	VID_API_EVENT_FIFO_LOW        = 0x05,
	VID_API_EVENT_FIFO_HIGH       = 0x06,
	VID_API_EVENT_FIFO_EMPTY      = 0x07,
	VID_API_EVENT_FIFO_FULL       = 0x08,
	VID_API_EVENT_BS_ERROR        = 0x09,
	VID_API_EVENT_UDATA_FIFO_UPTD = 0x0A,
	VID_API_EVENT_RES_CHANGE      = 0x0B,
	VID_API_EVENT_FIFO_OVF        = 0x0C,
	VID_API_EVENT_CHUNK_DECODED   = 0x0D,
	VID_API_EVENT_REQ_FRAME_BUFF  = 0x10,
	VID_API_EVENT_FRAME_BUFF_RDY  = 0x11,
	VID_API_EVENT_REL_FRAME_BUFF  = 0x12,
	VID_API_EVENT_STR_BUF_RST     = 0x13,
	VID_API_EVENT_RET_PING        = 0x14,      /* Temp here - rationalise debug events at bottom */
	VID_API_EVENT_QMETER          = 0x15,
	VID_API_EVENT_STR_FMT_CHANGE  = 0x16,
	VID_API_EVENT_MIPS_XCPT       = 0x17,
	VID_API_EVENT_START_DONE      = 0x18,
	VID_API_EVENT_STOPPED         = 0x19,
	VID_API_EVENT_ABORT_DONE      = 0x1A,
	VID_API_EVENT_FINISHED        = 0x1B,
	VID_API_EVENT_DBG_STAT_UPDATE = 0x1C,
	VID_API_EVENT_DBG_LOG_STARTED = 0x1D,
	VID_API_EVENT_DBG_LOG_STOPPED = 0x1E,
	VID_API_EVENT_DBG_LOG_UPDATED = 0x1F,
	VID_API_EVENT_DBG_MSG_DEC     = 0x20,
	VID_API_EVENT_DEC_SC_ERR      = 0x21,
	VID_API_EVENT_CQ_FIFO_DUMP    = 0x22,
	VID_API_EVENT_DBG_FIFO_DUMP   = 0x23,
	VID_API_EVENT_DEC_CHECK_RES   = 0x24,
	VID_API_EVENT_DEC_CFG_INFO    = 0x25,
	VID_API_EVENT_INVALID         = 0xFF

} TB_API_DEC_EVENT;

typedef enum {
	MEDIAIP_PLAYMODE_CONNECTIVITY = 0,
	MEDIAIP_PLAYMODE_BROADCAST,
	MEDIAIP_PLAYMODE_BROADCAST_DSS,
	MEDIAIP_PLAYMODE_LAST = MEDIAIP_PLAYMODE_BROADCAST_DSS

} MEDIA_IP_PLAYMODE;

typedef enum {
	MEDIA_IP_FMT_NULL        = 0x0,
	MEDIA_IP_FMT_AVC         = 0x1,
	MEDIA_IP_FMT_VC1         = 0x2,
	MEDIA_IP_FMT_MP2         = 0x3,
	MEDIA_IP_FMT_AVS         = 0x4,
	MEDIA_IP_FMT_ASP         = 0x5,
	MEDIA_IP_FMT_JPG         = 0x6,
	MEDIA_IP_FMT_RV          = 0x7,
	MEDIA_IP_FMT_VP6         = 0x8,
	MEDIA_IP_FMT_SPK         = 0x9,
	MEDIA_IP_FMT_VP8         = 0xA,
	MEDIA_IP_FMT_MVC         = 0xB,
	MEDIA_IP_FMT_VP3         = 0xC,
	MEDIA_IP_FMT_HEVC        = 0xD,
	MEDIA_IP_FMT_AUTO_DETECT = 0xAD00,
	MEDIA_IP_FMT_ALL         = (int)0xAAAAAAAA,
	MEDIA_IP_FMT_UNSUPPORTED = (int)0xFFFFFFFF,
	MEDIA_IP_FMT_LAST = MEDIA_IP_FMT_UNSUPPORTED

} MEDIA_IP_FORMAT;

typedef enum {
	VSys_FrmtNull = 0x0,
	VSys_AvcFrmt  = 0x1,
	VSys_Mp2Frmt  = 0x2,
	VSys_Vc1Frmt  = 0x3,
	VSys_AvsFrmt  = 0x4,
	VSys_AspFrmt  = 0x5,
	VSys_JpgFrmt  = 0x6,
	VSys_RvFrmt   = 0x7,
	VSys_Vp6Frmt  = 0x8,
	VSys_SpkFrmt  = 0x9,
	VSys_Vp8Frmt  = 0xA,
	VSys_HevcFrmt = 0xB,
	VSys_LastFrmt = VSys_HevcFrmt
} TB_API_DEC_FMT;

typedef struct {
	u_int32 bTopFldFirst;
	u_int32 bRptFstField;
	u_int32 uDispVerRes;
	u_int32	uDispHorRes;
	u_int32	uCentreVerOffset;
	u_int32	uCentreHorOffset;
	u_int32 uCropLeftRightOffset;
	u_int32 uCropTopBotOffset;

} MediaIPFW_Video_PicDispInfo;

typedef struct MediaIPFW_PicPerfInfo {
	u_int32 uMemCRC;
	u_int32 uBSCRC;
	u_int32 uSlcActiveCnt;
	u_int32 uIBEmptyCnt;
	u_int32 uBaseMemCRC;

	u_int32 uBaseCRCSkip;
	u_int32 uBaseCRCDrop;
	BOOL    bBaseCRCValid;

	u_int32 uCRC0;
	u_int32 uCRC1;
	u_int32 uCRC2;
	u_int32 uCRC3;
	u_int32 uCRC4;
	u_int32 uCRC5;

	u_int32 uFrameActCount;
	u_int32 uRbspBytesCount;
	u_int32 uDpbReadCount;
	u_int32 uMprWaitCount;
	u_int32 uAccQP;
	u_int32 uCacheStat;
	u_int32 mbq_full;
	u_int32 mbq_empty;
	u_int32 slice_cnt;
	u_int32 mb_count;

	u_int32 uTotalTime_us;
	u_int32 uTotalFwTime_us;

	u_int32 uProcIaccTotRdCnt;
	u_int32 uProcDaccTotRdCnt;
	u_int32 uProcDaccTotWrCnt;
	u_int32 uProcDaccRegRdCnt;
	u_int32 uProcDaccRegWrCnt;
	u_int32 uProcDaccRngRdCnt;
	u_int32 uProcDaccRngWrCnt;

} MediaIPFW_Video_PicPerfInfo;

typedef struct {
	u_int32 mb_count;
	u_int32 slice_cnt;

	/* Front End Metrics */
	u_int32 uDFEBinsUsed;
	u_int32 uDFECycleCount;
	u_int32 uDFESliceCycleCount;
	u_int32 uDFEIBWaitCount;
	u_int32 uDFENumBytes;

	u_int32 uProcIaccTotRdCnt;
	u_int32 uProcDaccTotRdCnt;
	u_int32 uProcDaccTotWrCnt;
	u_int32 uProcDaccRegRdCnt;
	u_int32 uProcDaccRegWrCnt;
	u_int32 uProcDaccRngRdCnt;
	u_int32 uProcDaccRngWrCnt;

	/* Back End metrics */
	u_int32 uNumBEUsed;
	u_int32 uTotalTime_us;
	u_int32 uTotalFwTime_us;
	u_int32 uDBECycleCount[0x2];
	u_int32 uDBESliceCycleCount[0x2];
	u_int32 uDBEMprWaitCount[0x2];
	u_int32 uDBEWaitCount[0x2];
	u_int32 uDBECRC[0x2];
	u_int32 uDBETotalTime_us[0x2];

	u_int32 uDBEMPRPRXWaitCount[0x2];
	u_int32 uDBEPXDPRXWaitCount[0x2];
	u_int32 uDBEFCHPLQWaitCount[0x2];
	u_int32 uDBEPXDPLQWaitCount[0x2];

	u_int32 uDBEFchWordsCount[0x2];
	u_int32 uDBEDpbCRC[0x2];
	u_int32 uDBEDpbReadCount[0x2];
	u_int32 uDBECacheStats[0x2];

} MediaIPFW_Video_PicPerfDcpInfo, *pMediaIPFW_Video_PicPerfDcpInfo;

typedef struct {
	u_int32 uPicType;
	u_int32 uPicStruct;
	u_int32 bLastPicNPF;
	u_int32 uPicStAddr;
	u_int32 uFrameStoreID;
	MediaIPFW_Video_PicDispInfo    DispInfo;
	MediaIPFW_Video_PicPerfInfo    PerfInfo;
	MediaIPFW_Video_PicPerfDcpInfo PerfDcpInfo;
	u_int32 bUserDataAvail;
	u_int32 uPercentInErr;

	u_int32 uBbdHorActive;
	u_int32 uBbdVerActive;
	u_int32 uBbdLogoActive;
	u_int32 uBbdBotPrev;
	u_int32 uBbdMinColPrj;
	u_int32 uBbdMinRowPrj;
	u_int32 uFSBaseAddr;

	/* Only for RealVideo RPR */
	u_int32 uRprPicWidth;
	u_int32 uRprPicHeight;

	/*only for divx3*/
	u_int32 uFrameRate;

} MediaIPFW_Video_PicInfo;


typedef struct {
	u_int32 bClosedGop;
	u_int32 bBrokenLink;
} MediaIPFW_Video_GopInfo;

typedef struct {
	u_int32 uIQuant;
	u_int32 uIQuantAvail;
	u_int32 uGopBitRate;
	u_int32 uGopBitRateAvail;

} MediaIPFW_Video_QMeterInfo;

typedef struct {
	u_int32                  pPicInfoArrayBase;
	u_int32 uNumSizeDescriptors;
} MediaIPFW_Video_PicInfoBuffTabDesc;

typedef struct {
	u_int32                  pGopInfoArrayBase;
	u_int32 uNumSizeDescriptors;
} MediaIPFW_Video_GopInfoBuffTabDesc;

typedef struct {
	u_int32                     pQMeterInfoArrayBase;
	u_int32 uNumSizeDescriptors;
} MediaIPFW_Video_QMeterInfoTabDesc;

typedef struct {
	u_int32	uMemChunkBase;
	u_int32	uMemChunkSize;

} MediaIPFW_Video_FrameBuffer;

typedef struct {
	u_int32	uUDataBase;
	u_int32	uUDataTotalSize;
	u_int32 uUDataSlotSize;

} MediaIPFW_Video_UData;

typedef struct {
	u_int32	uDecStatusLogBase;
	u_int32	uDecStatusLogSize;
	u_int32 uDTVLogBase[VID_API_NUM_STREAMS];
	u_int32 uDTVLogSize[VID_API_NUM_STREAMS];

} MediaIPFW_Video_DbgLogDesc;

typedef struct {
	u_int32 uDTVLogBase[VID_API_NUM_STREAMS];
	u_int32 uDTVLogSize[VID_API_NUM_STREAMS];

} MediaIPFW_Video_EngAccessLogDesc;

typedef struct MediaIPFW_FrameStore {
	u_int32 uFrameStoreLumaBase;
	u_int32 uFrameStoreChromaBase;

} MediaIPFW_Video_FrameStore;

typedef struct {
	u_int32 uAddrFirstDescriptor;
	u_int32 uNumSizeDescriptors;

} MediaIPFW_Video_StreamBuffTabDesc;

typedef struct {
	u_int32 uAddrFirstDescriptor;
	u_int32 uNumSizeDescriptors;
} MediaIPFW_Video_UserDataBuffTabDesc;

typedef struct {
	u_int32 uNumRefFrms;
	u_int32 uNumDPBFrms;
	u_int32 uNumDFEAreas;
	u_int32 uColorDesc;
	u_int32 uProgressive;
	u_int32 uVerRes;
	u_int32 uHorRes;
	u_int32 uParWidth;
	u_int32 uParHeight;
	u_int32 FrameRate;
	u_int32 UDispAspRatio;
	u_int32 uLevelIDC;
	u_int32 uVerDecodeRes;
	u_int32 uHorDecodeRes;
	u_int32 uOverScan;
	u_int32 uChromaFmt;
	u_int32 uPAFF;
	u_int32 uMBAFF;
	u_int32 uBitDepthLuma;
	u_int32 uBitDepthChroma;
	u_int32 uMVCNumViews;
	u_int32 uMVCViewList[VID_API_MAX_NUM_MVC_VIEWS];
	u_int32 uFBCInUse;

} MediaIPFW_Video_SeqInfo;

typedef struct {
	u_int32                  pSeqInfoArrayBase;
	u_int32 uNumSizeDescriptors;
} MediaIPFW_Video_SeqInfoBuffTabDesc;

typedef struct {
	u_int32 wptr;
	u_int32 rptr;
	u_int32 start;
	u_int32 end;

} BUFFER_DESCRIPTOR_TYPE, *pBUFFER_DESCRIPTOR_TYPE;

typedef struct {
	volatile u_int32 wptr;
	volatile u_int32 rptr;
	volatile u_int32 start;
	volatile u_int32 end;
	volatile u_int32 LWM;

} STREAM_BUFFER_DESCRIPTOR_TYPE, *pSTREAM_BUFFER_DESCRIPTOR_TYPE;

typedef struct {
	u_int32 uRotationAngle;
	u_int32 uHorizScaleFactor;
	u_int32 uVertScaleFactor;
	u_int32 uRotationMode;
	u_int32 uRGBMode;
	u_int32 uChunkMode; /* 0 ~ 1 */
	u_int32 uLastChunk; /* 0 ~ 1 */
	u_int32 uChunkRows; /* 0 ~ 255 */
	u_int32 uNumBytes;
	u_int32 uJpgCropXStart;
	u_int32 uJpgCropYStart;
	u_int32 uJpgCropWidth;
	u_int32 uJpgCropHeight;
	u_int32 uJpgMjpegMode;
	u_int32 uJpgMjpegInterlaced;

} MediaIPFW_Video_JpegParams;

typedef struct {
	u_int32                     pJpegParamArrayBase;
	u_int32                    uNumSizeDescriptors;

} MediaIPFW_Video_JpegParamTabDesc;

typedef struct {
	u_int32 uDispImm;
	u_int32 uFourCC;
	u_int32 uCodecVersion;
	u_int32 uFrameRate;
	u_int32 bbd_logo_width;
	u_int32 bbd_lum_thr;
	u_int32 bbd_coring;
	u_int32 bbd_s_thr_row;
	u_int32 bbd_p_thr_row;
	u_int32 bbd_s_thr_logo_row;
	u_int32 bbd_p_thr_logo_row;
	u_int32 bbd_s_thr_col;
	u_int32 bbd_p_thr_col;
	u_int32 bbd_chr_thr_row;
	u_int32 bbd_chr_thr_col;
	u_int32 bbd_uv_mid_level;
	u_int32 bbd_excl_win_mb_left;
	u_int32 bbd_excl_win_mb_right;

} MediaIPFW_Video_CodecParams;

typedef struct {
	u_int32 uFramePitch;

} MediaIPFW_Video_PitchInfo;

typedef struct {
	u_int32	uWrPtr;
	u_int32	uRdPtr;
	u_int32	uStart;
	u_int32	uEnd;
	u_int32	uLo;
	u_int32	uHi;

} MediaIPFW_Video_BufDesc;

typedef struct {
	u_int32                      pCodecParamArrayBase;
	u_int32                      uNumSizeDescriptors;

} MediaIPFW_Video_CodecParamTabDesc;

typedef struct {
	u_int32 uRC4Key[0x8];
	u_int32 uMemObfuscVal;

} MediaIPFW_Video_Encrypt_Info, *pMediaIPFW_Video_Encrypt_Info;

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
	u_int32                                FwExecBaseAddr;
	u_int32                                FwExecAreaSize;
	MediaIPFW_Video_BufDesc                StreamCmdBufferDesc;
	MediaIPFW_Video_BufDesc                StreamMsgBufferDesc;
	u_int32                                StreamCmdIntEnable[VID_API_NUM_STREAMS];
	MediaIPFW_Video_PitchInfo              StreamPitchInfo[VID_API_NUM_STREAMS];
	u_int32                                StreamConfig[VID_API_NUM_STREAMS];
	MediaIPFW_Video_CodecParamTabDesc	   CodecParamTabDesc;                       /* TODO-KMC  should we just go ahead and remove the concept of tabdesc? It is basicaly a bad coding style used for pinkys anyway */
	MediaIPFW_Video_JpegParamTabDesc       JpegParamTabDesc;
#ifdef COREPLAY_API
	pBUFFER_DESCRIPTOR_TYPE                pStreamBuffDesc[VID_API_NUM_STREAMS][VID_API_MAX_BUF_PER_STR];
#else
	u_int32                                pStreamBuffDesc[VID_API_NUM_STREAMS][VID_API_MAX_BUF_PER_STR];
#endif
	MediaIPFW_Video_SeqInfoBuffTabDesc     SeqInfoTabDesc;
	MediaIPFW_Video_PicInfoBuffTabDesc     PicInfoTabDesc;
	MediaIPFW_Video_GopInfoBuffTabDesc     GopInfoTabDesc;
	MediaIPFW_Video_QMeterInfoTabDesc      QMeterInfoTabDesc;
	u_int32                                StreamError[VID_API_NUM_STREAMS];
	u_int32                                FWVersion;
	u_int32                                uMVDMipsOffset;
	u_int32                                uMaxDecoderStreams;
	MediaIPFW_Video_DbgLogDesc             DbgLogDesc;
	MediaIPFW_Video_FrameBuffer            StreamFrameBuffer[VID_API_NUM_STREAMS];
	MediaIPFW_Video_FrameBuffer            StreamDCPBuffer[VID_API_NUM_STREAMS];
	MediaIPFW_Video_UData                  UDataBuffer[VID_API_NUM_STREAMS];
	MediaIPFW_Video_BufDesc                DebugBufferDesc;
	MediaIPFW_Video_BufDesc                EngAccessBufferDesc[VID_API_NUM_STREAMS];
	u_int32                                ptEncryptInfo[VID_API_NUM_STREAMS];
	MEDIAIP_FW_SYSTEM_CONFIG               sSystemCfg;
	u_int32                                uApiVersion;
} DEC_RPC_HOST_IFACE, *pDEC_RPC_HOST_IFACE;

//x means source data , y means destination data
#define VID_STREAM_CONFIG_FORMAT_MASK            0x0000000F
#define VID_STREAM_CONFIG_FORMAT_POS             0
#define VID_STREAM_CONFIG_FORMAT_SET(x, y)          (*y = (*y | ((x << VID_STREAM_CONFIG_FORMAT_POS)&VID_STREAM_CONFIG_FORMAT_MASK)))

#define VID_STREAM_CONFIG_STRBUFIDX_MASK         0x00000300
#define VID_STREAM_CONFIG_STRBUFIDX_POS          8
#define VID_STREAM_CONFIG_STRBUFIDX_SET(x, y)       (*y = (*y | ((x << VID_STREAM_CONFIG_STRBUFIDX_POS)&VID_STREAM_CONFIG_STRBUFIDX_MASK)))

#define VID_STREAM_CONFIG_NOSEQ_MASK             0x00000400
#define VID_STREAM_CONFIG_NOSEQ_POS              10
#define VID_STREAM_CONFIG_NOSEQ_SET(x, y)          (*y = (*y | ((x << VID_STREAM_CONFIG_NOSEQ_POS)&VID_STREAM_CONFIG_NOSEQ_MASK)))

#define VID_STREAM_CONFIG_DEBLOCK_MASK           0x00000800
#define VID_STREAM_CONFIG_DEBLOCK_POS            11
#define VID_STREAM_CONFIG_DEBLOCK_SET(x, y)         (*y = (*y | ((x << VID_STREAM_CONFIG_DEBLOCK_POS)&VID_STREAM_CONFIG_DEBLOCK_MASK)))

#define VID_STREAM_CONFIG_DERING_MASK            0x00001000
#define VID_STREAM_CONFIG_DERING_POS             12
#define VID_STREAM_CONFIG_DERING_SET(x, y)          (*y = (*y | ((x << VID_STREAM_CONFIG_DERING_POS)&VID_STREAM_CONFIG_DERING_MASK)))

#define VID_STREAM_CONFIG_IBWAIT_MASK            0x00002000
#define VID_STREAM_CONFIG_IBWAIT_POS             13
#define VID_STREAM_CONFIG_IBWAIT_SET(x, y)          (*y = (*y | ((x << VID_STREAM_CONFIG_IBWAIT_POS)&VID_STREAM_CONFIG_IBWAIT_MASK)))

#define VID_STREAM_CONFIG_FBC_MASK               0x00004000
#define VID_STREAM_CONFIG_FBC_POS                14
#define VID_STREAM_CONFIG_FBC_SET(x, y)             (*y = (*y | ((x << VID_STREAM_CONFIG_FBC_POS)&VID_STREAM_CONFIG_FBC_MASK)))

#define VID_STREAM_CONFIG_PLAY_MODE_MASK         0x00030000
#define VID_STREAM_CONFIG_PLAY_MODE_POS          16
#define VID_STREAM_CONFIG_PLAY_MODE_SET(x, y)       (*y = (*y | ((x << VID_STREAM_CONFIG_PLAY_MODE_POS)&VID_STREAM_CONFIG_PLAY_MODE_MASK)))

#define VID_STREAM_CONFIG_ENABLE_DCP_MASK      0x00100000
#define VID_STREAM_CONFIG_ENABLE_DCP_POS       20
#define VID_STREAM_CONFIG_ENABLE_DCP_SET(x, y)    (*y = (*y | ((x << VID_STREAM_CONFIG_ENABLE_DCP_POS)&VID_STREAM_CONFIG_ENABLE_DCP_MASK)))

#define VID_STREAM_CONFIG_NUM_STR_BUF_MASK       0x00600000
#define VID_STREAM_CONFIG_NUM_STR_BUF_POS        21
#define VID_STREAM_CONFIG_NUM_STR_BUF_SET(x, y)     (*y = (*y | ((x << VID_STREAM_CONFIG_NUM_STR_BUF_POS)&VID_STREAM_CONFIG_NUM_STR_BUF_MASK)))

#define VID_STREAM_CONFIG_MALONE_USAGE_MASK      0x01800000
#define VID_STREAM_CONFIG_MALONE_USAGE_POS       23
#define VID_STREAM_CONFIG_MALONE_USAGE_SET(x, y)    (*y = (*y | ((x << VID_STREAM_CONFIG_MALONE_USAGE_POS)&VID_STREAM_CONFIG_MALONE_USAGE_MASK)))

#define VID_STREAM_CONFIG_MULTI_VID_MASK         0x02000000
#define VID_STREAM_CONFIG_MULTI_VID_POS          25
#define VID_STREAM_CONFIG_MULTI_VID_SET(x, y)       (*y = (*y | ((x << VID_STREAM_CONFIG_MULTI_VID_POS)&VID_STREAM_CONFIG_MULTI_VID_MASK)))

#define VID_STREAM_CONFIG_OBFUSC_EN_MASK         0x04000000
#define VID_STREAM_CONFIG_OBFUSC_EN_POS          26
#define VID_STREAM_CONFIG_OBFUSC_EN_SET(x, y)       (*y = (*y | ((x << VID_STREAM_CONFIG_OBFUSC_EN_POS)&VID_STREAM_CONFIG_OBFUSC_EN_MASK)))

#define VID_STREAM_CONFIG_RC4_EN_MASK            0x08000000
#define VID_STREAM_CONFIG_RC4_EN_POS             27
#define VID_STREAM_CONFIG_RC4_EN_SET(x, y)          (*y = (*y | ((x << VID_STREAM_CONFIG_RC4_EN_POS)&VID_STREAM_CONFIG_RC4_EN_MASK)))

#define VID_STREAM_CONFIG_MCX_MASK               0x10000000
#define VID_STREAM_CONFIG_MCX_POS                28
#define VID_STREAM_CONFIG_MCX_SET(x, y)             (*y = (*y | ((x << VID_STREAM_CONFIG_MCX_POS)&VID_STREAM_CONFIG_MCX_MASK)))

#define VID_STREAM_CONFIG_PES_MASK               0x20000000
#define VID_STREAM_CONFIG_PES_POS                29
#define VID_STREAM_CONFIG_PES_SET(x, y)             ((*y = (*y | ((x << VID_STREAM_CONFIG_PES_POS)&VID_STREAM_CONFIG_PES_MASK))))

#define VID_STREAM_CONFIG_NUM_DBE_MASK           0x40000000
#define VID_STREAM_CONFIG_NUM_DBE_POS            30
#define VID_STREAM_CONFIG_NUM_DBE_SET(x, y)         (*y = (*y | ((x << VID_STREAM_CONFIG_NUM_DBE_POS)&VID_STREAM_CONFIG_NUM_DBE_MASK)))

#define VID_STREAM_CONFIG_FS_CTRL_MODE_MASK   0x80000000
#define VID_STREAM_CONFIG_FS_CTRL_MODE_POS    31
#define VID_STREAM_CONFIG_FS_CTRL_MODE_SET(x, y) (*y = (*y | ((x << VID_STREAM_CONFIG_FS_CTRL_MODE_POS)&VID_STREAM_CONFIG_FS_CTRL_MODE_MASK)))

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
