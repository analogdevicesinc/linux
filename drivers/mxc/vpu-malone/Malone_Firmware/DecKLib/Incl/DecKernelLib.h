/////////////////////////////////////////////////////////////////////////////////
/// @author    Copyright (c) 2015 Amphion Semiconductor Ltd
/////////////////////////////////////////////////////////////////////////////////
// The code contained herein is licensed under the GNU General Public
// License. You may obtain a copy of the GNU General Public License
// Version 2 or later at the following locations:
//
// http://www.opensource.org/licenses/gpl-license.html
// http://www.gnu.org/copyleft/gpl.html
////////////////////////////////////////////////////////////////////////////////
//
/// @file      DecLib.h
/// @brief     Main DecLib public header file
/// @ingroup   DecLib
/// @defgroup  DecLib DecLib API
/// @{
///            Decoder Library API level - Exported header file
///            called by the Player level and passing data back
///            via registered callbacks
/// @}
//
/////////////////////////////////////////////////////////////////////////////////
// $Id:
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#include "basetype.h"
#include "mediaip_fw_types.h"
#include "pal.h"
#include "DecKernelLibCfg.h"

#ifndef _DECODER_KLIB_H_
#define _DECODER_KLIB_H_

/////////////////////////////////////////////////////////////////////////////////
//  Global Macros
/////////////////////////////////////////////////////////////////////////////////

#define DECLIB_FSID_INVALID         0xbad

#define DECLIB_DBG_SESSION_STATE    0x1
#define DECLIB_DBG_DECODER_STATE    0x2
#define DECLIB_DBG_SYNTAX_ELEMENT   0x4
#define DECLIB_DBG_REG_ACCESS       0x8
#define DECLIB_DBG_CQ_FIFO          0x10

#define DECLIB_DBG_REG_HIF          0x1
#define DECLIB_DBG_REG_SIF          0x2
#define DECLIB_DBG_REG_CTX          0x4
#define DECLIB_DBG_REG_RPR          0x8
#define DECLIB_DBG_REG_SPP          0x10
#define DECLIB_DBG_REG_DEC          0x20
#define DECLIB_DBG_REG_CQ           0x40
#define DECLIB_DBG_REG_RSB          0x80
#define DECLIB_DBG_REG_DFE          0x80

#ifdef MVD_DIAG_LOG_REG_ACCESS
#define DECLIB_DBG_MASK             0x1F
#define DECLIB_DBG_REG_MASK         0x1FF
#else
#define DECLIB_DBG_MASK             0x7
#define DECLIB_DBG_REG_MASK         0x0
#endif

/////////////////////////////////////////////////////////////////////////////////
//  Global Types
/////////////////////////////////////////////////////////////////////////////////

typedef struct
{
  u_int32 uMalIdx;
  u_int32 uIrqStatus[0x3];

} DECODER_KERNEL_LIB_ISR_EVENT_DATA;

/////////////////////////////////////////////////////////////////////////////////
//  Global Function definitions
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
/// Decoder Event info callback

typedef void ( * DecKernelLib_Isr_Callback_t )( DECODER_KERNEL_LIB_ISR_EVENT_DATA *ptEventData );

/////////////////////////////////////////////////////////////////////////////////
//  Global Configuration Type
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
/// DecoderLib Configuration structure

typedef struct
{
  /* Malone hardware details */
  u_int32                         uNumMalones;
  u_int32                         uMaloneHifOffset[DECODERLIB_MAX_MALONES];
  uint_addr                       uMaloneBaseAddr[DECODERLIB_MAX_MALONES];
  u_int32                         uMaloneIrqPin[DECODERLIB_MAX_MALONES][0x2];
  u_int32                         uNumDPVUnits;        /* 0 or 1 - could infer this from base address but for clarity */
  uint_addr                       uDPVBaseAddr;
  u_int32                         uDPVIrqPin;
} DECODERLIB_KERNEL_CFG, * pDECODERLIB_KERNEL_CFG;

/////////////////////////////////////////////////////////////////////////////////
//  Global Function Prototypes
/////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS decoder_kernel_lib_init ( DECODERLIB_KERNEL_CFG * pCfg );

MEDIAIP_FW_STATUS decoder_kernel_lib_term ( void );

MEDIAIP_FW_STATUS decoder_kernel_lib_register_isr_callback ( u_int32 uMalIdx,
                                                             DecKernelLib_Isr_Callback_t pfCallback );


#endif /* _DECODER_KLIB_H_ */

/* End of File */
