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

  Filename:        DecoderLib.c
  Description:     Decoder Library API level
  Author:          Media IP FW team (Belfast & Shanghai)

 ****************************************************/

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#include "basetype.h"
#include "mediaip_fw_types.h"
#include "pal.h"
#include "DecKernelLib.h"
#include "DecKernelLibPrivate.h"
#include "DecKernelLibCfg.h"

/////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////

DEC_KERNEL_LIB gDecKernelLib = { 0x0 };

/////////////////////////////////////////////////////////////////////////////////
//  Code
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    decoder_kernel_lib_init                                          //
//                                                                                //
//  DESCRIPTION: This function initialises the decoder library                    //
//               to be called from kernel space                                   //
//               It's only purpose is to post interrupts via callback to a queue  //
//               for processing in userspace                                      //
//                                                                                //
//  INPUTS:      pCfg - Pointer to a static configuration structure for the       //
//                      library                                                   //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     MEDIAIP_FW_STATUS_                                               //
//                  OK             - Success.                                     //
//                  ALREADY_INIT   - This function has already been called and no //
//                                   subsequent call to decoderlib_term made      //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     This function must be called from non-interrupt context          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS decoder_kernel_lib_init ( DECODERLIB_KERNEL_CFG * pCfg )
{
  MEDIAIP_FW_STATUS eRetCode = MEDIAIP_FW_STATUS_OK;

  if ( gDecKernelLib.bInit == FALSE )
  {
    gDecKernelLib.bInit = TRUE;

    /* Make all necessary init calls */
    internal_decoder_kernel_lib_init ( pCfg );
  }
  else
  {
    eRetCode = MEDIAIP_FW_STATUS_ALREADY_INIT;
  }

  return eRetCode;

}

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    decoderlib_register_event_callback                               //
//                                                                                //
//  DESCRIPTION: This function registers a handler for reporting decoder events   //
//                                                                                //
//  INPUTS:      hHandle - handle for the stream with which to register the       //
//                         callbacks                                              //
//               pfCallback -  Function ponter to be executed when a decoder      //
//                             event is raised                                    //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     MEDIAIP_FW_STATUS_                                               //
//                  OK             - Success.                                     //
//                  BAD_HANDLE     - hHandle is not a valid decoder library       //
//                                   handle                                       //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     Unknown if there are any constraints here as yet                 //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS decoder_kernel_lib_register_isr_callback ( u_int32                     uMalIdx,
                                                             DecKernelLib_Isr_Callback_t pfCallback )
{

  if ( gDecKernelLib.bInit == FALSE )
  {
    pal_trace( DECODER_TL_WARNING, "DECODER_LIB: decoder_kernel_lib_register_isr_callback : Invalid Malone\n", uMalIdx );

    return MEDIAIP_FW_STATUS_BAD_HANDLE;
  }

  internal_decoder_kernel_lib_register_isr_callback ( uMalIdx,
                                                      pfCallback );

  return MEDIAIP_FW_STATUS_OK;

}

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    decoderlib_term                                                  //
//                                                                                //
//  DESCRIPTION: Deinitialises decode library                                     //
//                                                                                //
//  INPUTS:      None                                                             //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     MEDIAIP_FW_STATUS_                                               //
//                  OK             - Success.                                     //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     This function must be called from non-interrupt context          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS decoder_kernel_lib_term ( )
{
  MEDIAIP_FW_STATUS eRetCode = MEDIAIP_FW_STATUS_OK;

  gDecKernelLib.bInit = FALSE;

  return eRetCode;

}

