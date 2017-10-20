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

  Filename:        DecoderLibPrivate.c
  Description:     Decoder Library API level
  Author:          Kyle McAdoo - Media IP FW Team ( Belfast & Shanghai )

 ***************************************************/

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#include "basetype.h"
#include "mediaip_fw_types.h"
#include "pal.h"
#include "DecKernelLibPrivate.h"
#include "DecKernelLibHWControl.h"

/////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////

u_int32                            uMvdKernelIrqPin[DECODERLIB_MAX_MALONES][0x2];

/////////////////////////////////////////////////////////////////////////////////
//  Extern function prototypes
/////////////////////////////////////////////////////////////////////////////////

extern DEC_KERNEL_LIB              gDecKernelLib;
extern MALONE_KERNEL_HW_SESSION    gMvdKernelHw[(DECODERLIB_MAX_MALONES + 1)];

extern MEDIAIP_IRQ_RETCODE mvd_kernel_hw_primary_isr ( u_int32 irq_val );
extern MEDIAIP_IRQ_RETCODE mvd_kernel_hw_secondary_isr ( u_int32 irq_val );

/////////////////////////////////////////////////////////////////////////////////
//  Code
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    internal_decoder_kernel_lib_init                                 //
//                                                                                //
//  DESCRIPTION: This function initialises the decoder library                    //
//               more notes to follow when it works...                            //
//                                                                                //
//  INPUTS:      None                                                             //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     MEDIAIP_FW_STATUS_                                               //
//                  OK             - Success.                                     //
//                  BAD_PARAMETER  - Configuration structure not initialised      //
//                  ALREADY_INIT   -                                              //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     This function must be called from non-interrupt context          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS internal_decoder_kernel_lib_init ( DECODERLIB_KERNEL_CFG * pCfg )
{
  MEDIAIP_FW_STATUS RetCode = MEDIAIP_FW_STATUS_OK;

  /* Parse interrupt config */
  internal_decoder_kernel_lib_parse_cfg ( pCfg,
                                          FALSE );

  /* Setup hardware view of Malone */
  mvd_kernel_hw_control_init ( pCfg );                                          
                                          
  return RetCode;
}

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    internal_decoder_kernel_lib_register_isr_callback                //
//                                                                                //
//  DESCRIPTION: This function registers handlers for event processing            //
//               more notes to follow when it works...                            //
//                                                                                //
//  INPUTS:      hHandle - handle for the stream with which to register the       //
//                         callbacks                                              //
//               pfCallback -  Function ponter to be executed when a decoder      //
//                             event is raised                                    //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     None.                                                            //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     Unknown if there are any constraints here as yet                 //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

void internal_decoder_kernel_lib_register_isr_callback ( u_int32                     uMalIdx,
                                                         DecKernelLib_Isr_Callback_t pfCallback
                                                       )
{
  gDecKernelLib.pfCallback[uMalIdx] = pfCallback;
}

///////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    internal_decoder_kernel_lib_parse_cfg                            //
//                                                                                //
//  DESCRIPTION: Update DecoderLib structs with info from the configuration       //
//                                                                                //
//  INPUTS:      pCfg   - The DecLib configuration                                //
//               bCheck - A control variable which allows prints on changes in    //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     None.                                                            //
//                                                                                //
//  NOTES:       Set bCheck in snapshot restarts to see what may be different     //
//               in the configuration of the generation and restart platforms     //
//                                                                                //
//  CONTEXT:     Call from thread context                                         //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

void internal_decoder_kernel_lib_parse_cfg ( DECODERLIB_KERNEL_CFG * pCfg,
                                             bool             bCheck
                                           )
{
  PAL_PFNISR        pFnDecodeIsr;
  u_int32           uMalIdx;

  gDecKernelLib.uNumMalones = pCfg->uNumMalones;

  for ( uMalIdx = 0x0; uMalIdx < gDecKernelLib.uNumMalones; uMalIdx++ )
  {
    uMvdKernelIrqPin[uMalIdx][0x0] = pCfg->uMaloneIrqPin[uMalIdx][0x0];
    uMvdKernelIrqPin[uMalIdx][0x1] = pCfg->uMaloneIrqPin[uMalIdx][0x1];
  }

  /* Finally claim the interrupts */
  pFnDecodeIsr = ( PAL_PFNISR )mvd_kernel_hw_primary_isr;
  pal_int_register ( uMvdKernelIrqPin[0x0][0x0], pFnDecodeIsr, FALSE );
  pFnDecodeIsr = ( PAL_PFNISR )mvd_kernel_hw_secondary_isr;

  /* This is a safety check in case a lazy player does not setup the second irq */
  if ( uMvdKernelIrqPin[0x0][0x0] != uMvdKernelIrqPin[0x0][0x1] )
  {
    pal_int_register ( uMvdKernelIrqPin[0x0][0x1], pFnDecodeIsr, FALSE );
  }

  if ( gDecKernelLib.uNumMalones > 0x1 )
  {
    pFnDecodeIsr = ( PAL_PFNISR )mvd_kernel_hw_primary_isr;
    pal_int_register ( uMvdKernelIrqPin[0x1][0x0], pFnDecodeIsr, FALSE );
    pFnDecodeIsr = ( PAL_PFNISR )mvd_kernel_hw_secondary_isr;

    if ( uMvdKernelIrqPin[0x1][0x0] != uMvdKernelIrqPin[0x1][0x1] )
    {
      pal_int_register ( uMvdKernelIrqPin[0x1][0x1], pFnDecodeIsr, FALSE );
    }
  }
}


/* End of file */
