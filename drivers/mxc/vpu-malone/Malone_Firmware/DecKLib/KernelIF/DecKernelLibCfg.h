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

  Filename:        DecLibCfg.h
  Description:     Decoder Library Configuration
  Author:          Media IP FW team (Belfast & Shanghai)

 *******************************************************/

#ifndef _DECODER_LIB_CFG_H_
#define _DECODER_LIB_CFG_H_

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#ifndef VPU_KERNEL_BUILD
#include "video_subsystem.h"
#endif
#include "mediaip_fw_types.h"

/////////////////////////////////////////////////////////////////////////////////
//  Global Macros
/////////////////////////////////////////////////////////////////////////////////

/* Local defines cast to those in global cfg file */
#define   DECODERLIB_MAX_MALONES           MEDIAIP_MAX_NUM_MALONES
#define   DECODERLIB_MAX_DBE_UNITS         0x2

/* These don't really need to be passed in the cfg methinks... */
#define   DECODERLIB_NUM_STREAMS           NUM_DECODER_STREAMS
#define   DECODERLIB_MAX_NUM_FRAMES        MAX_NUM_FRAMES_PER_STREAM
#if HEVC_JVT_MODEL < 92
#define   DECODERLIB_MAX_MBI_FRAMES        0x12
#else
#define   DECODERLIB_MAX_MBI_FRAMES        0x11
#endif
#define   DECODERLIB_MAX_DFE_AREAS         0x1
#define   DECODERLIB_MAX_NUM_OVLP_FRMS     1
#define   DECODERLIB_METADATA_AREA_NULL    0xFF
#define   DECODERLIB_NUM_EVENTS_PER_STREAM 4
#define   DECODERLIB_NUM_CMDS_PER_STREAM   4

#define   DECODERLIB_MAX_MVC_DPID          1
#define   DECODERLIB_MAX_MVC_TARGET_VIEWS  2
#define   DECODERLIB_MAX_MVC_VIEWS         4

#define   DECODERLIB_MAX_DPVS                0x1
#define   DECODERLIB_PIXIF_MAX_UPIX_TARGETS  0x2
#define   DECODERLIB_PIXIF_MAX_FBC_TARGETS   0x2
#define   DECODERLIB_MAX_STREAM_LEVELS       DECODERLIB_MAX_MVC_VIEWS


#define   DECODERLIB_MAX_STR_BUFFERS       DECODERLIB_MAX_STREAM_LEVELS
#define   DECODERLIB_RC4_CONTEXT_VALS      66

#if ( TARGET_LEVEL == HAPS ) || ( TARGET_LEVEL == SIMULATION )
#define   DECODERLIB_NUM_DBG_FIFOS 26
#else
#define   DECODERLIB_NUM_DBG_FIFOS 1
#endif

#ifdef DECLIB_ENABLE_DCP
#define   DECODERLIB_MAX_CQ_PER_MALONE     0x3
#else
#define   DECODERLIB_MAX_CQ_PER_MALONE     0x1
#endif

// Enable processing of PAFF streams
// If defined Field frame storage choice is made at a picture level
// otherwise it is made as a sequence level
//#define PIXIF_STORE_AS_PAFF

//-------------------------------------------------
// Some options for testing different HW Configs
//-------------------------------------------------
//#define FSLCACHE0_BYPASS
#define ALLOW_CHROMA_DP
#define ALLOW_MPS_ALIGN
#define ALLOW_OFFSET_FS
//#define FORCE_UNCACHED_8x8
//-------------------------------------------------

#endif /* _DECODER_LIB_CFG_H_ */

/* End of File */
