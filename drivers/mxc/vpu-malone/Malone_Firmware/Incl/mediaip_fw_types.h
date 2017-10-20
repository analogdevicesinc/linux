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

  Filename:        mediaip_fw_types.h
  Description:     Contains structure definitions common
                   to multiple modules / layers
  Author:          Media IP FW team - Belfast / Shanghai

 ****************************************************/

#ifndef _MEDIAIP_FW_TYPES_H_
#define _MEDIAIP_FW_TYPES_H_

#include "basetype.h"
#include "mediaip_fw_defines.h"

//////////////////////////////////////////////////////////////
// Generic Stream format

typedef enum
{
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

//////////////////////////////////////////////////////////////
// Generic picture Structure

typedef enum
{
  MEDIAIP_TOP_FIELD,
  MEDIAIP_BOT_FIELD,
  MEDIAIP_TWO_FIELDS,
  MEDIAIP_FRAME_PICTURE

} MEDIAIP_PIC_STRUCT;

//////////////////////////////////////////////////////////////
// Generic picture type

typedef enum
{
  MEDIAIP_I_PICTYPE = 1,
  MEDIAIP_P_PICTYPE,
  MEDIAIP_B_PICTYPE,
  MEDIAIP_UNDEF_PICTYPE

} MEDIAIP_PIC_TYPE;

//////////////////////////////////////////////////////////////
// Stream Layer

typedef enum
{
  MEDIAIP_BASE_LAYER = 0,
  MEDIAIP_ENHANCEMENT_LAYER

} MEDIAIP_LAYER;

//////////////////////////////////////////////////////////////
// Display module - sync state

typedef enum
{
  DISP_SYNC_UNINITIALISED = 0,
  DISP_SYNC_ACQUIRING,
  DISP_SYNC_ACQUIRED,
  DISP_SYNC_LOST,
  DISP_SYNC_LOST_INVALID,
  DISP_SYNC_LOST_INSANE

} MEDIAIP_DISP_SYNC_STATE;

//////////////////////////////////////////////////////////////
// PES HW state

typedef enum
{
  PES_STATE_NORMAL = 0,
  PES_STATE_PTS_BACKTOBACK,
  PES_STATE_PTS_WITH_SLICE,
  PES_STATE_PTS_WITH_PIC

} MEDIAIP_PES_STATE;

//////////////////////////////////////////////////////////////
// Playback mode

typedef enum
{
  MEDIAIP_PLAYMODE_CONNECTIVITY = 0,
  MEDIAIP_PLAYMODE_BROADCAST,
  MEDIAIP_PLAYMODE_BROADCAST_DSS,
  MEDIAIP_PLAYMODE_LAST = MEDIAIP_PLAYMODE_BROADCAST_DSS

} MEDIA_IP_PLAYMODE;

//////////////////////////////////////////////////////////////
// System mode

typedef enum
{
  MEDIAIP_SYSMODE_DTV = 0,
  MEDIAIP_SYSMODE_STB,
  MEDIAIP_SYSMODE_LAST = MEDIAIP_SYSMODE_STB

} MEDIA_IP_SYSMODE;

//////////////////////////////////////////////////////////////
// Encoder Output Info
typedef struct
{
  u_int32    uStrIndex;
  BOOL       bLastFrame;
  BOOL       bEndOfGOP;
  u_int32    uEsStruct;
  u_int32    uMsbPTS;
  u_int32    uPTS;
  u_int32    uVesSize;
  u_int32    uCbFrmStartAddr;

} MEDIAIP_ENCODER_OUTPUT_INFO;

//////////////////////////////////////////////////////////////
// YUV Output Info

#ifndef HOST_BUILD
#ifndef _MSC_VER
typedef enum
{
  YUV420 = 0x0,
  YUV422,
  YUV400

} MEDIAIP_FW_FRAME_FMT;
#endif // _MSC_VER
#endif

//////////////////////////////////////////////////////////////
// Frame Display info

typedef struct
{
  u_int32 top_field_first;
  u_int32 repeat_first_field;
  u_int32 disp_vert_res;
  u_int32 disp_horiz_res;
  u_int32 centre_vert_offset;
  u_int32 centre_horiz_offset;

} MEDIAIP_PIC_DISP_INFO;

//////////////////////////////////////////////////////////////
// Frame info

typedef struct
{
  MEDIAIP_PIC_TYPE pic_type;
  u_int32          pic_struct;
  u_int32          pic_st_addr;
  u_int32          last_pic_npf;
  u_int32          user_data_avail;
  u_int32          frame_store_id;
  u_int32          uPercentInErr;
  u_int32          view_id;         /* H264 MVC */
  u_int32          uVOIdx;          /* H264 MVC */
#if MVC_SUPPORT == MVC_ENABLED
  u_int32          uViewId;
#endif
  u_int32          uSkipInProg;

} MEDIAIP_PIC_INFO;

//////////////////////////////////////////////////////////////
// Memory request type enum

typedef enum
{
  MEDIAIP_FRAME_REQ = 0,
  MEDIAIP_MBI_REQ,
  MEDIAIP_DCP_REQ,
  MEDIAIP_REQ_LAST = MEDIAIP_DCP_REQ

} MEDIAIP_MEM_REQ;

//////////////////////////////////////////////////////////////
// Memory request ctrl

typedef struct
{
  u_int32          uLayerIdx;
  MEDIAIP_MEM_REQ  eType;

} MEDIAIP_MEM_REQ_CTRL, *pMEDIAIP_MEM_REQ_CTRL;

//////////////////////////////////////////////////////////////
// Memory release ctrl

typedef struct
{
  u_int32          uLayerIdx;
  u_int32          uFSIdx;
  MEDIAIP_MEM_REQ  eType;

} MEDIAIP_MEM_REL_CTRL, *pMEDIAIP_MEM_REL_CTRL;

////////////////////////////////////////////////////////
// Media IP mem alloc struct

typedef struct
{
  MEDIAIP_MEM_REQ  eType;
  u_int32          uLayerIdx;

  u_int32          uFSHandle;
  u_int32          uBaseAddr;
  u_int32          uStride;
  u_int32          uChromaOffset;

} MEDIAIP_MEM_ALLOC_CTRL, *pMEDIAIP_MEM_ALLOC_CTRL;

//////////////////////////////////////////////////////////////
// Decoder metrics info

typedef struct
{
  u_int32 uDpbmcCrc;
  u_int32 uFrameActiveCount;
  u_int32 uSliceActiveCount;
  u_int32 uRbspBytesCount;
  u_int32 uSibWaitCount;
  u_int32 uDpbReadCount;
  u_int32 uMprWaitCount;
  u_int32 uBBBCrc;
  u_int32 uAccQP;
  u_int32 uCacheStat;
  u_int32 uCRCSkip;
  u_int32 uCRCDrop;
  BOOL    bCRCValid;

  u_int32 uBaseDpbMcCrc;
  u_int32 uByteStreamCrc;

  u_int32 uCRC0;
  u_int32 uCRC1;
  u_int32 uCRC2;
  u_int32 uCRC3;
  u_int32 uCRC4;
  u_int32 uCRC5;
  u_int32 uCRC6;
  u_int32 uCRC7;
  u_int32 uCRC8;
  u_int32 uCRC9;
  u_int32 uCRC10;
  u_int32 uCRC11;
  u_int32 uCRC12;
  u_int32 uCRC13;
  u_int32 uCRC14;

  u_int32 mbq_full;
  u_int32 mbq_empty;
  u_int32 slice_cnt;
  u_int32 mb_count;

  u_int32 uTotalTime_us;
  u_int32 uTotalFwTime_us;

  u_int32 uSTCAtFrameDone;

  u_int32 uProcIaccTotRdCnt;
  u_int32 uProcDaccTotRdCnt;
  u_int32 uProcDaccTotWrCnt;
  u_int32 uProcDaccRegRdCnt;
  u_int32 uProcDaccRegWrCnt;
  u_int32 uProcDaccRngRdCnt;
  u_int32 uProcDaccRngWrCnt;

} MEDIAIP_DEC_METRICS_INFO, *pMEDIAIP_DEC_METRICS_INFO;

//////////////////////////////////////////////////////////////
// GOP info

typedef struct
{
  u_int32 closed_gop;
  u_int32 broken_link;

} MEDIAIP_GOP_INFO;

//////////////////////////////////////////////////////////////
// Stream frequency

typedef enum
{
  MEDIAIP_FR_UNKNOWN  = 0,
  MEDIAIP_FR_23_97_HZ = 1,
  MEDIAIP_FR_24_HZ    = 2,
  MEDIAIP_FR_25_HZ    = 3,
  MEDIAIP_FR_29_97_HZ = 4,
  MEDIAIP_FR_30_HZ    = 5,
  MEDIAIP_FR_50_HZ    = 6,
  MEDIAIP_FR_59_94_HZ = 7,
  MEDIAIP_FR_60_HZ    = 8,
  MEDIAIP_FR_7P992_HZ = 9,
  MEDIAIP_FR_8_HZ     = 10,
  MEDIAIP_FR_8P33_HZ  = 11,
  MEDIAIP_FR_9P99_HZ  = 12,
  MEDIAIP_FR_10_HZ    = 13,
  MEDIAIP_FR_11P988_HZ= 14,
  MEDIAIP_FR_12_HZ    = 15,
  MEDIAIP_FR_12P5_HZ  = 16,
  MEDIAIP_FR_14P985_HZ= 17,
  MEDIAIP_FR_15_HZ    = 18

} MEDIAIP_OUTPUT_FREQ;

//////////////////////////////////////////////////////////////
// Scan info

typedef enum
{
  MEDIAIP_PROGRESSIVE,
  MEDIAIP_INTERLACE

} MEDIAIP_SCAN_INFO;

//////////////////////////////////////////////////////////////
// Pipeline type enumeration

typedef enum
{
  MEDIAIP_VIDEO_DECODE_PIPELINE = 0x0,
  MEDIAIP_VIDEO_ENCODE_PIPELINE,
  MEDIAIP_VIDEO_TRANSCODE_PIPELINE,
  MEDIAIP_VIDEO_TRANSDISP_PIPELINE,
  MEDIAIP_VIDEO_NULL_PIPELINE

} MEDIAIP_VIDEO_PIPELINE_TYPE;

//////////////////////////////////////////////////////////////
// Decode info

/* TODO-KMC */
/* Why do these structures break the protocol in case used for naming ? */

typedef enum
{
   MediaIPFW_DEC_CodingTypeI = 1,
   MediaIPFW_DEC_CodingTypeP,
   MediaIPFW_DEC_CodingTypeAny,
   MediaIPFW_DEC_CodingTypeIP,
   MediaIPFW_DEC_CodingTypeSkip,
   MediaIPFW_DEC_CodingTypeLast

} MediaIPFW_DEC_CodingType;

//////////////////////////////////////////////////////////////
// Decode Command Mode

typedef enum
{
  MediaIPFW_DEC_ModeSplit = 0,
  MediaIPFW_DEC_ModeNoSplit

} MediaIPFW_DEC_Mode;

//////////////////////////////////////////////////////////////
// Tag structures

typedef enum
{
  TAG_FREE = 0,
  TAG_IN_USE

} TAG_STATUS;

typedef struct
{

  u_int32 pts;             //  TSP aspect of the structure
  u_int32 dts;
  u_int32 pts_flags;
#ifdef PES_INPUT_ENABLE
#else
  u_int32 maturity_address;
#endif
  u_int32 scode_loc;      // Possibly this variable could be combined with maturity_address
                          // They are the same thing, but come from very different places
  BOOL    bValid;

} DEMUX_INFO, *pDEMUX_INFO;

// This struct is AVC specific - needs to be generic or at least cast as generic
typedef struct
{

  u_int32 sei_flags;        // Display aspect
  u_int32 sei_pic_struct;

} DISP_INFO, *pDISP_INFO;

typedef struct
{
  u_int32 fs_idc;
  u_int32 uUseFrameCRC;
  u_int32 uBotFirst;
  u_int32 uTopData[2];
  u_int32 uBotData[2];
  u_int32 uReported;

  u_int32 uDpbmcCRC;    // Internal MC interface CRC
  u_int32 uBsCRC;       // Byte Stream CRC

  u_int32 uDPBLevel;   // DPB Level at frame start

  u_int16 fs_is_used;
  u_int16 Monochrome;

  u_int32 control;

} CRC_INFO, *pCRC_INFO;

typedef struct
{
  u_int32 frame_size_inmbs;   // {Size[15:0],Height[7:0],Width[7:0]}
  u_int32 bs_ib_empty;
  u_int32 frm_dec_active;
  u_int32 slc_dec_active;
  u_int32 stc_at_decode;
  u_int32 stc_at_display;
  u_int32 num_slc_overlapped;
  u_int32 num_mb_overlapped;
  u_int32 luma_address;
  u_int32 cpb_min_level;
  u_int32 rbsp_byte_count;
  u_int32 dpb_read_count;
  u_int32 mpr_wait_count;
  u_int32 slice_count;
  u_int16 drm_pull_count;
  u_int16 drm_pap_pts_cnt;
  u_int16 drm_pap_skip_cnt;
  u_int16 drm_pap_dang_cnt;

} PERF_INFO, *pPERF_INFO;

typedef struct
{
  u_int32 uNumSlices;
  u_int32 uSumQP;

}QMETER_INFO, *pQMETER_INFO;

typedef struct
{

  DEMUX_INFO  TspInfo;
  DISP_INFO   DispInfo;
  CRC_INFO    CRCInfo;
  TAG_STATUS  Status;          // General info on the structure - free or taken...
  QMETER_INFO QMeterInfo;
  u_int32     uPercentMBInErr;
#ifdef PERF_DEBUG
  PERF_INFO   PerfInfo;
#endif

} TAG, *pTAG;

typedef struct
{

  u_int32 uLastTSP;
  u_int32 uLastNonTSP;
  u_int32 uInUse;

} TAG_CTRL, *pTAG_CTRL;


// Film Grain technology support
typedef struct
{
  BOOL    bFGS_Flag;
  u_int32 log2_scale_factor;
  u_int32 pic_order_count;
  u_int32 pic_order_offset;
  u_int16 fgs_lut[3][256];

}USERDATA_FILMGRAIN_CONTROL;

#define DIAG_ARRAY_SIZE 26

typedef struct
{

  int32    nAvcDiagState;
  int32    nTSPDiagState;
  u_int32  uFailurePt;
  u_int32  uFailData[4];
  int32    nLastCall;
  int32    nDecodeBegan;
  int32    nEngStart;
  u_int32  uLastStart;
  int32    DPBCount;
  int32    DPBAnomaly;
  int32    DPBAnomalyCnt;
  int32    nEntryIndex;
  int32    nEntryArray[DIAG_ARRAY_SIZE];
  int32    nExitIndex;
  int32    nExitArray[DIAG_ARRAY_SIZE];

} DiagInfo, *pDiagInfo;


////////////////////////////////////////////////////////
// Frame store structure -
// Common to multiple modules will hold all information
// which needs to be transferred between any two modules
// in relation to a picture buffer

typedef struct mediaip_fw_frame_store
{
  struct mediaip_fw_frame_store * pNext;     // Not entirely sure I need this - we will see as the architecture evolves...

  /* Frame store description */
  u_int32 uBaseAddr;                         // Base Address - Luma base for a image store, address for MBI or BLI
  u_int32 uChromaOffset;
  u_int32 uPitch;
  u_int32 uDimsMBs;

  // Module usage
  u_int32 uModUsage;                         // 1 bit set for each module that considers this frame store to be in use by it
                                             // For a real time display / transcode situation this could be more than 2

  // Last Frame flag
  BOOL bLastFrame;                           // Flag to indicate whether this is the last frame store decoded by the decoder.
                                             // Important when stopping a transcode pipeline or reaching the last frame of a bitstream
                                             // so that we can tell the encoder not to expect any more frames and hence empty it's
                                             // pipeline.

  // Display params
  u_int32 uDispData;                         // Bottom field first etc - normally to be filled in by decode module

  // Frame store
  u_int32 uFSID;                             // An ID used to cross reference with a local structure - filled in by the capture unit, decoder or
                                             // image capture unit, etc

  pTAG    pFSTag;                            // A pointer to a tag for the frame store - normally allocated by the decoder so that unit should
                                             // set the pointer

  USERDATA_FILMGRAIN_CONTROL ud_filmgrain;   // Filmgrain support structure

  // Want to give anything that may be managed by the resource layer a generic resource tag!
  // Until I can be bothered working out how to do this I will go with the 3 below...

  u_int8  uDynBuffType;                      // What is the buffer type of the currently allocayted frame store
                                             // Could be worked out from uFSInternBufNum - Remember if we have to
                                             // hold a frames over a channel change not all uDynBuffType members
                                             // will be the same in each stream's group of frame stores

  u_int8  uFSInternBufStart;
  u_int8  uFSInternBufNum;

} MEDIAIP_FW_FRAME_STORE;

typedef MEDIAIP_FW_FRAME_STORE  MEDIAIP_FW_ENCODER_REF_FRAME_STORE;
typedef MEDIAIP_FW_FRAME_STORE  MEDIAIP_FW_ENCODER_FRAME_ACT_STORE;


////////////////////////////////////////////////////////
// MetaData store structure -
// Defines an area which may be passed to a base module for its own internal use

typedef struct
{
  u_int32 uMetadataBase;
  u_int32 uMetadataSize;

} MEDIAIP_FW_METADATA_STORE, *pMEDIAIP_FW_METADATA_STORE;

////////////////////////////////////////////////////////
// Stream Frequency enumeration -


typedef enum {

                STREAM_FR_UNKNOWN  = 0,
                STREAM_FR_23_97_HZ = 1,
                STREAM_FR_24_HZ    = 2,
                STREAM_FR_25_HZ    = 3,
                STREAM_FR_29_97_HZ = 4,
                STREAM_FR_30_HZ    = 5,
                STREAM_FR_50_HZ    = 6,
                STREAM_FR_59_94_HZ = 7,
                STREAM_FR_60_HZ    = 8

             } STREAM_SCAN_FREQ;

////////////////////////////////////////////////////////
// Digital Encode Mode ( at the tv encoder)

typedef enum {
                MODE_240p        = 0x0,
                MODE_240i        = 0x1,
                MODE_480p        = 0x2,
                MODE_NTSC        = 0x3,
                MODE_PALp        = 0x4,
                MODE_PAL         = 0x5,
                MODE_720p        = 0x6,
                MODE_720p_50Hz   = 0x7,
                MODE_720i        = 0x8,
                MODE_720i_25Hz   = 0x9,
                MODE_720i_50Hz   = 0xa,
                MODE_720i_60Hz   = 0xb,
                MODE_1080p       = 0xc,
                MODE_1080p_25Hz  = 0xd,
                MODE_1080i       = 0xe,
                MODE_1080i_25Hz  = 0xf,
                MODE_2Kp         = 0x10,
                MODE_1080p_24Hz  = 0x11,
                MODE_UNKNOWN     = 0xff

             } TV_ENC_MODE;

////////////////////////////////////////////////////////
// Stream Display parameters

typedef struct
{
  u_int32              uTargetLevel;
  u_int32              uHorDecodeRes;
  u_int32              uVerDecodeRes;
  u_int32              uHorDispRes;
  u_int32              uVerDispRes;
  u_int32              uDispWidthMBs;
  u_int32              uDispHeightMBs;
  u_int32              uBufWidthMBs;
  u_int32              uBufHeightMBs;
  u_int32              uAspectRatio;
  u_int32              uSizeMBs;
  u_int32              uYUVFmt;
  u_int32              uScanFormat;
  STREAM_SCAN_FREQ     sFreq;
  u_int32              uNumRefFrms;
  u_int32              uNumDPBFrms;
  u_int32              uNumDFEAreas;
  u_int32              uProfLevelIDC;
  MEDIAIP_OUTPUT_FREQ  eOutputFreq;
  MEDIAIP_SCAN_INFO    eScanInfo;
  u_int32              uAR;
  u_int32              uColorDesc;
  u_int32              uBitDepthLuma;
  u_int32              uBitDepthChroma;
  u_int32              uNumViews;
  u_int32              uViewList;

} MEDIAIP_FW_STREAM_DISPLAY_PARAMS;


////////////////////////////////////////////////////////
// Frame decoded parameters

typedef struct
{
  BOOL                 bTopFieldFirst;
  BOOL                 bRepeatFirstField;
  MEDIAIP_PIC_TYPE     ePicType;
  MEDIAIP_PIC_STRUCT   ePicStruct;

  u_int32              uFSHandle;

  u_int32              uPicStartAddr;
  u_int32              uPercentMBsInErr;

  u_int32              uPTSLo;
  u_int32              uPTSHi;
  BOOL                 bPTSValid;

  BOOL                 bQMeterValid;
  u_int32              uQMeter;
  BOOL                 bGopBitRateAvail;
  u_int32              uGopBitRate;

  u_int32              bTemporalRef;

  u_int32              uBbdHorActive;
  u_int32              uBbdVerActive;
  u_int32              uBbdLogoActive;
  u_int32              uBbdBotPrev;
  u_int32              uBbdMinColPrj;
  u_int32              uBbdMinRowPrj;

  u_int32              uFSBaseAddr;
  BOOL                 bDangling;
  BOOL                 bTopFieldPresent;

  /* Only for RealVideo RPR */
  u_int32              uRprPicWidth;
  u_int32              uRprPicHeight;

  /* Only for DivX3         */
  u_int32              uFrameRate;

  /* For decode time yuv gathering */
  u_int32              ulTopLumBaseAddr;
  u_int32              ulTopChrBaseAddr;
  u_int32              ulBotLumBaseAddr;
  u_int32              ulBotChrBaseAddr;

  u_int32              ulStride;

  void *               pAltView;
  u_int32              uMVCTargetViewIdx;
} MEDIAIP_FW_FRAME_DEC_PARAMS , *pMEDIAIP_FW_FRAME_DEC_PARAMS;

////////////////////////////////////////////////////////
// SVC Seq parameters

typedef struct
{
  u_int32                          uNumValidLayers;
  MEDIAIP_FW_STREAM_DISPLAY_PARAMS tLayerParams[MEDIAIP_MAX_SVC_DID];

} MEDIAIP_FW_SVC_STREAM_PARAMS;

//////////////////////////////////////////////////////////////
// IRQ Event data

typedef struct
{
  void *  pMVDHw;
  u_int32 uIrqStatus[0x3];

} MEDIAIP_FW_IRQ_DATA, *pMEDIAIP_FW_IRQ_DATA;


//////////////////////////////////////////////////////////////
// CMD Event data

typedef struct
{
  u_int32 uCmdData;

}MEDIAIP_FW_CMD_DATA, *pMEDIAIP_FW_CMD_DATA;

//////////////////////////////////////////////////////////////
// Callback Request Data

typedef struct
{
  u_int32 uReason;

}MEDIAIP_FW_CBACK_DATA, *pMEDIAIP_FW_CBACK_DATA;

////////////////////////////////////////////////////////
// Interface descriptor
// Generic descriptor type as initially defined by Tempest
// module

typedef struct
{

  volatile u_int32 address;
  volatile u_int32 parameters;

} INTERFACE_DESCRIPTOR_TYPE, *pINTERFACE_DESCRIPTOR_TYPE;

////////////////////////////////////////////////////////
// Buffer descriptor
// Generic buffer descriptor as initially defined by Tempest
// module

typedef struct
{
  u_int32 descriptor_number;
  u_int32 descriptor_size;
  u_int32 descriptor_ptr;

} BUFFER_TBL_DESCRIPTOR, *BUFFER_TBL_DESCRIPTOR_PTR;

////////////////////////////////////////////////////////
// Buffer descriptor
// Generic buffer descriptor as initially defined by Tempest
// module

typedef struct
{
  u_int32 wptr;
  u_int32 rptr;
  u_int32 start;
  u_int32 end;

} BUFFER_DESCRIPTOR_TYPE, *pBUFFER_DESCRIPTOR_TYPE;

////////////////////////////////////////////////////////
// Stream Buffer descriptor
// Specific buffer descriptor for stream data

typedef struct
{
  volatile u_int32 wptr;
  volatile u_int32 rptr;
  volatile u_int32 start;
  volatile u_int32 end;
  volatile u_int32 LWM;

} STREAM_BUFFER_DESCRIPTOR_TYPE, *pSTREAM_BUFFER_DESCRIPTOR_TYPE;

#ifdef TCODE_API_TEST
// Defined specific for PCI ,for its access unit is 64 bits
typedef struct
{
  volatile u_int32 wptr;
  volatile u_int32 wptr_res;
  volatile u_int32 rptr;
  volatile u_int32 rptr_res;
  volatile u_int32 start;
  volatile u_int32 end;

} BUF_DES_PCI_SHARE_TYPE, BUF_DES_PCI_SHARE_TYPE_PTR;

#endif
////////////////////////////////////////////////////////
// Stream Buffer descriptor
// Buffer descriptor as used by the encoder
// module

typedef struct
{

  u_int32 stream_index;
  u_int32 stream_buffer_size;
  u_int32 stream_buffer_base;

} STREAM_BUFFER_DESCRIPTOR, *STREAM_BUFFER_DESCRIPTOR_PTR;

typedef STREAM_BUFFER_DESCRIPTOR MEDIAIP_FW_ENCODER_STREAM_BUFFER_DESC;

////////////////////////////////////////////////////////
// PTS debug descriptor
// This descriptor is as initially defined by Tempest
// module

typedef struct
{
  volatile u_int32 pts_lo;
  volatile u_int32 stc_lo;
  volatile   int32 error;
  volatile u_int32 flags;

} PTS_DEBUG_DESCRIPTOR_TYPE, *pPTS_DEBUG_DESCRIPTOR_TYPE;

////////////////////////////////////////////////////////
// PTS descriptor
// This descriptor is as initially defined by Tempest
// module

typedef struct
{
  volatile u_int32 pts_lo;
  volatile u_int32 dts_lo;
  volatile   int32 flags;
  volatile u_int32 maturity_addr;

} PTS_DESCRIPTOR_TYPE, *pPTS_DESCRIPTOR_TYPE;

//////////////////////////////////////////////////////////////
// Video Stream Status structure
//
// Note : I am putting this in the communal include file because we may wish to let
// module base FW update such a structure directly as opposed to sending messages
// back to the public FW level...
// We may also wish to lump all module's status into this one structure

// Why are these volatiles? Should consider removing their volatility!!...
typedef struct
{
  //////////////////////
  // Pipeline variables
  volatile u_int32 run;
  volatile u_int32 pause;

  //////////////////////
  // Decoder variables
  volatile u_int32 mpeg_format;
  volatile u_int32 DSS;
  volatile u_int32 native_sd;
  volatile u_int32 stop;
  volatile u_int32 stop_pending;
  volatile u_int32 stream_terminate_type;
  volatile u_int32 ES;
  volatile u_int32 PAL;
  volatile u_int32 CRC;
  volatile u_int32 update_offset;
  volatile u_int32 pcr_sent;
  volatile u_int32 PVRMode;
  volatile u_int32 malone_mp2_dbdrng_en;
  volatile u_int32 uWaitFlush;

  volatile u_int32 pvr_type;    /* These are new - need to be fixed */
  volatile u_int32 valid_frame_mask;

  //////////////////////
  // Display variables
  volatile u_int32 force_interlace;
  volatile u_int32 sync_stc;
  volatile int32   nTrinityDisc;
  volatile u_int32 uTrinityDiscPCRLo;
  volatile u_int32 display_mode_status;
  volatile u_int32 sync;

} MEDIAIP_FW_STREAM_STATUS, *MEDIAIP_FW_STREAM_STATUS_PTR;

//////////////////////////////////////////////////////////////
// Stream Descriptor Structure
//
// Currently this is basically the stream descriptor structure
// from the decoder FW interface with the host SW
// This needs to evolve to suit our needs better
#if ( TARGET_PLATFORM == GENTB_PLATFORM )

typedef struct
{
  volatile u_int32 Control;
  volatile u_int32 Buffer_Indices;
  volatile u_int32 Message_Filter;
  volatile u_int32 Extended_Message_Filter;
  volatile u_int32 Tempest_Command_Filter;
  volatile u_int32 Status;
  volatile u_int32 Bitstream_Error;
  volatile u_int32 Skip_Count;
  volatile u_int32 Repeat_Count;
  volatile u_int32 Frame_Count;
  volatile u_int32 Bitstream_Timeout;
  volatile u_int32 Max_Frame_Dpb_Size;                     // For dynamic frame allocation this actually means the start of the allocated memory
  volatile u_int32 Decode_Buffer_Count;                    // For dynamic frame allocation this actually means the size of allocated memory
  volatile u_int32 Buffer_Pitch;                           // For dynamic frame allocation this actually means the memory controller tile width
  volatile INTERFACE_DESCRIPTOR_TYPE DPB_Base_Address_dec; // For dynamic frame allocation this actually means the MB info address
  volatile u_int32 PTS_DTS_Offset;
  volatile u_int32 Internal_PTS_Delay;
  volatile u_int32 Sync_Window_Start;
  volatile u_int32 Sync_Window_End;
  volatile u_int32 Sync_Sanity_Limit;
  volatile u_int32 trick_speed;
  INTERFACE_DESCRIPTOR_TYPE user_data_desc;

  INTERFACE_DESCRIPTOR_TYPE  FrameStoreDesc;
  INTERFACE_DESCRIPTOR_TYPE  frame_buffer_control_desc;

  INTERFACE_DESCRIPTOR_TYPE  pvr_command_list_desc;
  volatile u_int32           Pad[2];

  pPTS_DESCRIPTOR_TYPE       pts_table_desc;
  pPTS_DEBUG_DESCRIPTOR_TYPE pts_debug_table_desc;

  volatile u_int32           PVR_Control;
  volatile u_int32           RAI;

  // For PecosB onwards this is the security control word
  volatile u_int32           security_control;
  // Status variables for a stream
  volatile u_int32           sd_changed_control;
  volatile u_int32           ud_type_mask;
  MEDIAIP_FW_STREAM_STATUS   stream_status;

#if TARGET_APP == VIDEO_TRANS
  volatile u_int32 FrameStoreCount;
  // Second set of image buffers ( only required in a dual MC situation )
  volatile u_int32 FrameStoreCount2;
  volatile INTERFACE_DESCRIPTOR_TYPE  FrameStoreDesc2;

  // Encoder Reference buffers
  volatile u_int32 EncRefCount;
  volatile INTERFACE_DESCRIPTOR_TYPE  EncRefDesc;

  // Encoder Image Activity buffers
  volatile u_int32 EncActCount;
  volatile INTERFACE_DESCRIPTOR_TYPE  EncActDesc;

  // Stream buffer descriptor index - for encoder and decoder
  volatile u_int32                    uDecStrmBufIdx;          // Current decode FW expects that the buffer used will match the SDIndex
  volatile u_int32                    uEncOutBufIdx;

  // Only enc, vamux interDesc used at present to pass params
  volatile INTERFACE_DESCRIPTOR_TYPE  uDecParamSet;
  volatile INTERFACE_DESCRIPTOR_TYPE  uEncParamSet;
  volatile INTERFACE_DESCRIPTOR_TYPE  uVamuxParamSet;

  // Hardware Interface to system layer
  INTERFACE_DESCRIPTOR_TYPE           system_config_desc;

#ifdef TCODE_API_TEST
  volatile INTERFACE_DESCRIPTOR_TYPE  uBitRateDispSet;
#endif

  volatile u_int32                    streamdes_id;

#else

#if PLAYMODE == STB
  volatile u_int32                    streamdes_id;
#endif
#endif

} STREAM_DESCRIPTOR, *STREAM_DESCRIPTOR_PTR;

#else

typedef struct
{
  volatile u_int32 Control;                   // 0
  volatile u_int32 Buffer_Indices;
  volatile u_int32 Message_Filter;
  volatile u_int32 Extended_Message_Filter;
  volatile u_int32 Tempest_Command_Filter;
  volatile u_int32 Status;
  volatile u_int32 Bitstream_Error;
  volatile u_int32 Skip_Count;
  volatile u_int32 Repeat_Count;
  volatile u_int32 Frame_Count;
  volatile u_int32 Bitstream_Timeout;
  volatile u_int32 Max_Frame_Dpb_Size;
  volatile u_int32 Buffer_Pitch;
  volatile u_int32 Decode_Buffer_Count;
  volatile u_int32 PTS_DTS_Offset;
  volatile u_int32 Internal_PTS_Delay;
  volatile u_int32 Sync_Window_Start;
  volatile u_int32 Sync_Window_End;
  volatile u_int32 Sync_Sanity_Limit;
  volatile u_int32 trick_speed;                 // 20

  // Image buffers
  volatile u_int32 FrameStoreCount;
  volatile INTERFACE_DESCRIPTOR_TYPE  FrameStoreDesc;

  // Second set of image buffers ( only required in a dual MC situation )
  volatile u_int32 FrameStoreCount2;
  volatile INTERFACE_DESCRIPTOR_TYPE  FrameStoreDesc2;

  // Encoder Reference buffers
  volatile u_int32 EncRefCount;
  volatile INTERFACE_DESCRIPTOR_TYPE  EncRefDesc;

  // Encoder Image Activity buffers
  volatile u_int32 EncActCount;                       // 30
  volatile INTERFACE_DESCRIPTOR_TYPE  EncActDesc;

  // Stream buffer descriptor index - for encoder and decoder
  volatile u_int32                    uDecStrmBufIdx;          // Current decode FW expects that the buffer used will match the SDIndex
  volatile u_int32                    uEncOutBufIdx;

  // Only enc, vamux interDesc used at present to pass params
  volatile INTERFACE_DESCRIPTOR_TYPE  uDecParamSet;
  volatile INTERFACE_DESCRIPTOR_TYPE  uEncParamSet;
  volatile INTERFACE_DESCRIPTOR_TYPE  uVamuxParamSet;

  INTERFACE_DESCRIPTOR_TYPE           user_data_desc;
  INTERFACE_DESCRIPTOR_TYPE           frame_buffer_attribute_desc;     // Address of the attribute table used by the display process
                                                                       // Who fills this in?? Do I give a shite in module architecture?
  INTERFACE_DESCRIPTOR_TYPE           frame_buffer_control_desc;
  BUFFER_DESCRIPTOR_TYPE              pvr_command_list_desc;
  pPTS_DESCRIPTOR_TYPE                pts_table_desc;
  pPTS_DEBUG_DESCRIPTOR_TYPE          pts_debug_table_desc;

  // Hardware Interface to system layer
  INTERFACE_DESCRIPTOR_TYPE           system_config_desc;

  // Status variables for a stream
  volatile u_int32                    sd_changed_control;
  MEDIAIP_FW_STREAM_STATUS            stream_status;

  volatile u_int32                    RAI;


#ifdef DRM_AV_SYNC_DEBUG
  volatile u_int32 DRMPath_Take_Snapshot;                                // Only used when DRM_AV_SYNC_DEBUG defined but included
  volatile u_int32 DRMPath_Snapshot_Addr;                                // in all builds to enable more transparent api
  volatile u_int32 DRMPath_Snapshot_Index;                               // operations
#endif

#ifdef TCODE_API_TEST
  volatile INTERFACE_DESCRIPTOR_TYPE  uBitRateDispSet;
#endif

  volatile u_int32                    streamdes_id;

  /* Following have been removed from official descriptor area
  INTERFACE_DESCRIPTOR_TYPE  sps_td;
  INTERFACE_DESCRIPTOR_TYPE  pps_td;
  INTERFACE_DESCRIPTOR_TYPE  spds_td;
  */

} STREAM_DESCRIPTOR, *STREAM_DESCRIPTOR_PTR;

#endif

#define  CRC_DEBUG_NUM  1024
#define  PERF_DEBUG_NUM 2048  // 68secs
#define  CRC_FRAME      0
#define  CRC_FIELD_TOP  1
#define  CRC_FIELD_BOT  3

typedef struct
{

  u_int32 abort_type;
  u_int32 mode;
  u_int32 link_reg;
  u_int32 stack_pointer;

}ExceptionDesc, pExceptionDesc;

#ifndef VIDEO_SOFTWARE
/* Structure below needs to be in shared memory header file for SW apps */
typedef enum
{
  MVDDecoderOK = 0x0,
  MVDDecoderServiceIrq,
  MVDDecoderCmdPending,
  MVDDecoderFWLoop,
  MVDDecoderHWActive,
  MVDDecoderHWActiveStreamStall,
  MVDDecoderHWActiveMemStall,
  MVDDecoderFWHWDeadlock,
  MVDDecoderFWHWAutoRec,
  MVDDecoderHWBSDMAIssue,
  MVDDecoderUnknown,
  MVDDecoderLast = MVDDecoderUnknown

} MEDIAIP_FW_DECODER_STATUS;

#endif

typedef struct

{

  u_int32 size;               // Size of this structure
  u_int32 index;              // Current index into array
  u_int32 loops;              // Number of times we looped
  u_int32 num_frames;         // Number of frames in a loop
  u_int32 frame_size_inmbs;   // {Size[15:0],Height[7:0],Width[7:0]}

  u_int32 not_first_time;
  u_int32 prev_vsync_count;
  u_int32 worst_frame;
  u_int32 loop_key;           // Value used to decide when stream has looped
  u_int32 skip_start;


  u_int32 luma_mismatches;    //
  u_int32 chroma_mismatches;  //
  u_int32 dpbmc_mismatches;   //
  u_int32 bs_mismatches;      //
  u_int32 skip_count;         //
  u_int32 repeat_count;       //
  u_int32 loop_lu_crc;        // Merge of CRCs
  u_int32 loop_ch_crc;        // Merge of CRCs

  // Most recent captured Info
  u_int32 crc_type[PERF_DEBUG_NUM];
  u_int32 idr_pic[PERF_DEBUG_NUM];
  u_int32 poc[PERF_DEBUG_NUM];
  u_int32 luma_crc[PERF_DEBUG_NUM];
  u_int32 chroma_crc[PERF_DEBUG_NUM];
  u_int32 dpbmc_crc[PERF_DEBUG_NUM];          // Only valid for BBV & above
  u_int32 bs_crc[PERF_DEBUG_NUM];             // Only valid for BBV & above

  // Mismatches & skips
  u_int32 luma_mismatch[PERF_DEBUG_NUM];
  u_int32 chroma_mismatch[PERF_DEBUG_NUM];
  u_int32 dpbmc_mismatch[PERF_DEBUG_NUM];     // Only valid for BBV & above
  u_int32 bs_mismatch[PERF_DEBUG_NUM];        // Only valid for BBV & above

  u_int32 mismatch_fs_idc[32];
  u_int32 skip[PERF_DEBUG_NUM];
  u_int32 repeat[PERF_DEBUG_NUM];


  // Performance metrics & the associated CRCs for first loop
  u_int32 crc_type_l0[PERF_DEBUG_NUM];
  u_int32 idr_pic_l0[PERF_DEBUG_NUM];
  u_int32 poc_l0[PERF_DEBUG_NUM];
  u_int32 luma_crc_l0[PERF_DEBUG_NUM];
  u_int32 chroma_crc_l0[PERF_DEBUG_NUM];
  u_int32 dpbmc_crc_l0[PERF_DEBUG_NUM];       // Only valid for BBV & above
  u_int32 bs_crc_l0[PERF_DEBUG_NUM];          // Only valid for BBV & above

  // Max of metrics
  u_int32 max_slc_dec_active;
  u_int32 max_dpb_read_count;
  u_int32 max_mpr_wait_count;
  u_int32 max_rbsp_byte_count;
  u_int32 max_bs_ib_empty;
  u_int32 max_slice_count;
  u_int32 max_mb_overlapped;
  u_int32 min_stc_diff;
  u_int32 max_frm_dec_active;
  u_int32 max_frm_dec_active_2;
  u_int32 max_frm_dec_active_4;
  u_int32 max_frm_dec_active_8;

  // Max of metrics assiciated with worst frame
  u_int32 mfda_bs_ib_empty;
  u_int32 mfda_mpr_wait_count;
  u_int32 mfda_dpb_read_count;
  u_int32 mfda_rbsp_byte_count;
  u_int32 mfda_slice_count;
  u_int32 mfda_slc_dec_active;
  u_int32 mfda_mb_overlapped;

  // Will effectively be averages
  u_int32 Av_frm_dec_active;
  u_int32 Av_slc_dec_active;
  u_int32 Av_mpr_wait_count;
  u_int32 Av_dpb_read_count;
  u_int32 Av_rbsp_byte_count;
  u_int32 Av_bs_ib_empty;
  u_int32 Av_slice_count;
  u_int32 Av_mb_overlapped;

  u_int32 stc_diff[PERF_DEBUG_NUM];
  u_int32 frm_dec_active[PERF_DEBUG_NUM];
  u_int32 bs_ib_empty[PERF_DEBUG_NUM];
  u_int32 slc_dec_active[PERF_DEBUG_NUM];
  u_int32 num_slc_overlapped[PERF_DEBUG_NUM]; // Only valid for BBV & above
  u_int32 num_mb_overlapped[PERF_DEBUG_NUM];  // Only valid for BBV & above
  u_int32 cpb_min_level[PERF_DEBUG_NUM];
  u_int32 rbsp_byte_count[PERF_DEBUG_NUM];
  u_int32 dpb_read_count[PERF_DEBUG_NUM];
  u_int32 mpr_wait_count[PERF_DEBUG_NUM];
  u_int16 drm_pull_count[PERF_DEBUG_NUM];
  u_int16 drm_pap_pts_cnt[PERF_DEBUG_NUM];
  u_int16 drm_pap_skip_cnt[PERF_DEBUG_NUM];
  u_int16 drm_pap_dang_cnt[PERF_DEBUG_NUM];

  u_int32 initial_dpb_level[PERF_DEBUG_NUM];  // DPB level at frame start

} PerfDebug;

typedef struct

{

  u_int32 loops;              // Number of times we looped
  u_int32 loop_frames;        // Number of frames in a loop
  u_int32 num_frames;         // Position in current loop
  u_int32 loop_dpbmc_crc;     // Merge of CRCs
  u_int32 loop_stream_crc;    // Merge of CRCs
  u_int32 LoopCyCntD1024;     // Sum of frm_dec_active/1024
  u_int32 MaxDecActive;       // Max frm_dec_active
  u_int32 not_first_time;
  u_int32 first_crc;
  u_int32 WorstFrame;

} DecDebug;


///////////////////////////////////////////
// System Configuration structure

typedef struct
{
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


///////////////////////////////////////////
// Encoder types to be globally used across
// all encode / transcode apps and components

///////////////////////////////////////////
// MEDIAIP_ENC_FMT

typedef  enum
{
  MEDIAIP_ENC_FMT_H264 = 0,
  MEDIAIP_ENC_FMT_VC1,
  MEDIAIP_ENC_FMT_MPEG2,
  MEDIAIP_ENC_FMT_MPEG4SP,
  MEDIAIP_ENC_FMT_H263,
  MEDIAIP_ENC_FMT_MPEG1,
  MEDIAIP_ENC_FMT_SHORT_HEADER,
  MEDIAIP_ENC_FMT_NULL

} MEDIAIP_ENC_FMT;

///////////////////////////////////////////
// MEDIAIP_ENC_PROFILE

typedef enum
{
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

///////////////////////////////////////////
// MEDIAIP_ENC_CONFIG_CODEC_PARAMETER

typedef struct
{
  MEDIAIP_ENC_FMT      eCodecMode;
  MEDIAIP_ENC_PROFILE  eProfile;
  u_int32              uReserved[2];

} MEDIAIP_ENC_CONFIG_CODEC_PARAMETER, *pMEDIAIP_ENC_CONFIG_CODEC_PARAMETER;  //8 words

///////////////////////////////////////////
// MEDIAIP_ENC_PIXEL_FORMAT

typedef enum
{
  MEDIAIP_ENC_PLANAR = 0,
  MEDIAIP_ENC_SEMIPLANAR

} MEDIAIP_ENC_PIXEL_FORMAT;

///////////////////////////////////////////
// MEDIAIP_ENC_CHROMA_FMT

typedef  enum
{
  MODE_420=0,
  MODE_422,
  MODE_444

} MEDIAIP_ENC_CHROMA_FMT;

///////////////////////////////////////////
// MEDIAIP_ENC_FRAME_STRUCT

typedef struct
{
  u_int32  uSrcFieldModeForDsa;
  u_int32  uDstFieldModeForEnc;
  u_int32  uReserved[2];

} MEDIAIP_ENC_FRAME_STRUCT, *pMEDIAIP_ENC_FRAME_STRUCT;

///////////////////////////////////////////
// MEDIAIP_ENC_FRAME_DIMENSIONS

typedef struct
{
  /* Input picture features */
  u_int32    uSrcWidth;
  u_int32    uSrcHeight;
  u_int32    uYStride;
  u_int32    uUVStride;

  /* Clipping information   */
  u_int32    uClipFlag;     /*Note: 0: don't clip, 1: clip.   encode only a part of input frame      */
  u_int32    uOffset_x;
  u_int32    uOffset_y;
  u_int32    uClipPicWidth;
  u_int32    uClipPicHeight;

  /* Output picture features */
  u_int32    uDstWidth;
  u_int32    uDstHeight;
  u_int32    uTileMode;     /* 0: scan raster mode  1: tile mode.                                     */
  u_int32    uTileWidth;    /* Note: In tile mode, uYStride should be calculated depend on uTileWidth */
  u_int32    uTileHeight;

} MEDIAIP_ENC_FRAME_DIMENSIONS, *pMEDIAIP_ENC_FRAME_DIMENSIONS;

///////////////////////////////////////////
// MEDIAIP_ENC_BITRATE_MODE

typedef enum
{
  MEDIAIP_ENC_BITRATECONTROLMODE_VBR          = 0x00000001,
  MEDIAIP_ENC_BITRATECONTROLMODE_CBR          = 0x00000002,
  MEDIAIP_ENC_BITRATECONTROLMODE_CONSTANT_QP  = 0x00000004   /* Only in debug mode */

} MEDIAIP_ENC_BITRATE_MODE, *pMEDIAIP_ENC_BITRATE_MODE;

///////////////////////////////////////////
// MEDIAIP_ENC_BITRATE_CONTROL

typedef struct
{
  MEDIAIP_ENC_BITRATE_MODE  eBitRateMode;
  u_int32                   uTargetBitrate;
  u_int32                   uMaxBitRate;
  u_int32                   uMinBitRate;      /* Requested by Windsor, for soft encoder, it is useless */
  u_int32                   uSliceQP;

} MEDIAIP_ENC_BITRATE_CONTROL, *pMEDIAIP_ENC_BITRATE_CONTROL;

///////////////////////////////////////////
// MEDIAIP_ENC_GOP_STRUCTURE

typedef struct
{
  u_int32 uFrameRateNum;
  u_int32 uFrameRateDen;
  u_int32 uIFrameInterval;
  u_int32 uGopBLength;       /* How many B frames between I or P frames,  max is 4 for Windsor */
  u_int32 uLowLatencyMode;   /* Switch off scene change mode, no B frame, only in VBR Mode     */
  /* TODO-KMC */
  /* Remove the crap below */
  u_int32 reserved[3];

} MEDIAIP_ENC_GOP_STRUCTURE, *pMEDIAIP_ENC_GOP_STRUCTURE;

///////////////////////////////////////////
// MEDIAIP_ENC_STREAM_PARAMETER

typedef struct
{
  MEDIAIP_ENC_CHROMA_FMT        eVideoFormat;
  MEDIAIP_ENC_PIXEL_FORMAT      ePixelFormat;

  MEDIAIP_ENC_GOP_STRUCTURE     mGOPStructure;
  MEDIAIP_ENC_FRAME_STRUCT      mFrameStructure;
  MEDIAIP_ENC_FRAME_DIMENSIONS  mFrameSize;
  MEDIAIP_ENC_BITRATE_CONTROL   mBitRateControl;
  u_int32                       uExpertModeEnable;    /* Enable expert mode            */
  u_int32                       uMemChunkAddr;        /* Start address of memory chunk */
  u_int32                       uMemChunkSize;        /* Size of memory chunk          */
  u_int32                       uStreamFinish;
  /* TODO-KMC */
  /* Remove the crap below */
  u_int32                       reserved[64 - 40];

} MEDIAIP_ENC_STREAM_PARAMETER, *pMEDIAIP_ENC_STREAM_PARAMETER;


///////////////////////////////////////////
// MEDIAIP_ENC_YUV_BUFFER_DESC

typedef struct
{
  u_int32   uFrameID;
  u_int32   uLumaBase;
  u_int32   uChromaBase;
  u_int32   uParamIdx;

} MEDIAIP_ENC_YUV_BUFFER_DESC, *pMEDIAIP_ENC_YUV_BUFFER_DESC;

///////////////////////////////////////////
// eMEDIAIP_ENC_PIC_TYPE

typedef  enum
{
  MEDIAIP_ENC_PIC_TYPE_B_FRAME = 0,
  MEDIAIP_ENC_PIC_TYPE_P_FRAME,
  MEDIAIP_ENC_PIC_TYPE_I_FRAME,
  MEDIAIP_ENC_PIC_TYPE_IDR_FRAME,
  MEDIAIP_ENC_PIC_TYPE_BI_FRAME

} MEDIAIP_ENC_PIC_TYPE, *pMEDIAIP_ENC_PIC_TYPE;

///////////////////////////////////////////
// MEDIAIP_ENC_PIC_TYPE

typedef struct
{
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

///////////////////////////////////////////
// MEDIAIP_ENC_DSA_INFO

typedef struct
{
  u_int32  uHeight;
  u_int32  uWidth;
  u_int32  uCropWidth;
  u_int32  uCropHeight;
  u_int32  uCropPixelXOffset;
  u_int8 * pImgBuffer;
  u_int32  uStride;
  u_int32  uOffset2Chroma;
  u_int32  uOffset2Decimate;

} MEDIAIP_ENC_DSA_INFO, *pMEDIAIP_ENC_DSA_INFO;

///////////////////////////////////////////
// MEDIAIP_ENC_PIC_PARAM_UPD

typedef struct
{
  /* Each bit indicate corresponding parameter should be updated */
  u_int32    uMaskflag;

  /* ENC_GOP_STRUCTURE variables            */
  u_int32    uFrameRateNum;
  u_int32    uFrameRateDen;

  /* MEDIAIP_ENC_FRAME_STRUCT variables     */
  u_int32    uSrcFieldModeForDsa;
  u_int32    uDstFieldModeForEnc;

  /* MEDIAIP_ENC_FRAME_DIMENSIONS variables */
  u_int32    uSrcWidth;
  u_int32    uSrcHeight;
  u_int32    uYStride;
  u_int32    uUVStride;
  u_int32    uOffset_x;
  u_int32    uOffset_y;
  u_int32    uClipPicWidth;
  u_int32    uClipPicHeight;
  u_int32    uDstWidth;
  u_int32    uDstHeight;
  u_int32    uIFrameInterval;
  u_int32    uGopBLength;
  u_int32    uLowLatencyMode;

  /* MEDIAIP_ENC_BITRATE_CONTROL  variables */
  MEDIAIP_ENC_BITRATE_MODE  eBitRateMode;
  u_int32                   uTargetBitrate;
  u_int32                   uMaxBitRate;
  u_int32                   uMinBitRate;
  u_int32                   uSliceQP;
  u_int32                   uReserved[32-23];

} MEDIAIP_ENC_PIC_PARAM_UPD, *pMEDIAIP_ENC_PIC_PARAM_UPD;


///////////////////////////////////////////
// MEDIAIP_ENC_CALIB_PARAMS
//
// Encoder Hardware calibration parameters

typedef struct
{
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
  u_int32 mem_chunk_addr;
  u_int32 mem_chunk_size;
  u_int32 mem_y_stride;
  u_int32 mem_uv_stride;

  u_int32 split_wr_enab;
  u_int32 split_wr_req_size;
  u_int32 split_rd_enab;
  u_int32 split_rd_req_size;

} MEDIAIP_ENC_CALIB_PARAMS, *pMEDIAIP_ENC_CALIB_PARAMS;

///////////////////////////////////////////
// MEDIAIP_ENC_CONFIG_PARAMS
//
// Stream-specific configuration parameters

typedef struct
{
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


///////////////////////////////////////////
// MEDIAIP_ENC_STATIC_PARAMS
//
// Static parameters ( may change at the GOP level )

typedef struct
{
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

} MEDIAIP_ENC_STATIC_PARAMS, *pMEDIAIP_ENC_STATIC_PARAMS;

///////////////////////////////////////////
// MEDIAIP_ENC_DYN_PARAMS
//
// Dynamic parameters (may change at the frame level)

typedef struct
{
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

///////////////////////////////////////////
// MEDIAIP_ENC_EXPERT_MODE_PARAM

typedef struct
{
  MEDIAIP_ENC_CALIB_PARAMS   Calib;
  MEDIAIP_ENC_CONFIG_PARAMS  Config;
  MEDIAIP_ENC_STATIC_PARAMS  Static;
  MEDIAIP_ENC_DYN_PARAMS     Dynamic;
} MEDIAIP_ENC_EXPERT_MODE_PARAM, *pMEDIAIP_ENC_EXPERT_MODE_PARAM;

///////////////////////////////////////////
// MEDIAIP_ENC_PARAM

typedef struct
{
  MEDIAIP_ENC_FMT           eCodecMode;
  MEDIAIP_ENC_PROFILE       eProfile;

  u_int32                   uMemChunkAddr;
  u_int32                   uMemChunkSize;

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

#endif /* _MEDIAIP_FW_TYPES_H_ */

/* End of File */
