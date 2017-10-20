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

  Filename:        DecKernelPrivate.h
  Description:     Decoder Library Private header file - not
                   for inclusion by code outside of decoder lib.
  Author:          Media IP FW team (Belfast & Shanghai)

 *******************************************************/

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#include "DecKernelLib.h"

/* Include NO other files here */

#ifndef _DECODER_KERN_LIB_PRIV_H_
#define _DECODER_KERN_LIB_PRIV_H_


/////////////////////////////////////////////////////////////////////////////////
//  Global Macros
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Global Structures
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// DecoderLib Context structure

typedef struct decoderlib_inst
{
  bool                            bInit;
  u_int32                         uNumMalones;
  /* One per Malone, kernel lib only has concept of Malone hardware, not */
  /* the individual streams running on it!                                */
  DecKernelLib_Isr_Callback_t     pfCallback[DECODERLIB_MAX_MALONES]; 

} DEC_KERNEL_LIB;

/////////////////////////////////////////////////////////////////////////////////
//  Function Prototypes
/////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS internal_decoder_kernel_lib_init ( DECODERLIB_KERNEL_CFG * pCfg );

void internal_decoder_kernel_lib_register_isr_callback ( u_int32                     uMalIdx,
                                                         DecKernelLib_Isr_Callback_t pfCallback
                                                       );

 void internal_decoder_kernel_lib_parse_cfg ( DECODERLIB_KERNEL_CFG * pCfg,
                                              bool             bCheck
                                            );
                                                       
#endif /* _DECODER_KERN_LIB_PRIV_H_ */

/* End of File */
