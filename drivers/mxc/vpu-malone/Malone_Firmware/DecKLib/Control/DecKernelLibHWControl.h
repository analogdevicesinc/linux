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

  Filename:        DecLibHWControl.h
  Description:

 **************************************************/

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#ifndef _DECODER_LIB_HW_CONTROL_H_
#define _DECODER_LIB_HW_CONTROL_H_

#include "mvd_types.h"
#include "mvd_reg_map.h"
#include "DecKernelLib.h"

/////////////////////////////////////////////////////////////////////////////////
//  Global Macros
/////////////////////////////////////////////////////////////////////////////////

#define MALONE_HW_1 0                         /* Identifiers for the Malone units     */
#define MALONE_HW_2 1
#define MALONE_SW   DECODERLIB_MAX_MALONES    /* Sw Malone does not exist! Simply a   */
                                              /* control structure for carrying out   */
                                              /* commands which do not need a HW unit */
#define MALONE_SW_IRQ 0xdeaf
                                              
#define DECODERLIB_ISR_QU_SIZE          ( DECODERLIB_MAX_MALONES * 4 )
#define DECODERLIB_SECONDARY_ISR_EVENT  0x80000000
#define DECODERLIB_EVENT_MASK           0x7FFFFFFF

#define DECODERLIB_FORCEIRQ_BIT_SET( uVal )     (( uVal & 0x1 ) << 30 )
#define DECODERLIB_FORCEIRQ_BIT_GET( uVal )     (( uVal >> 30 ) & 0x1 )
#define DECODERLIB_FORCEDFEIRQ_BIT_SET( uVal )  (( uVal & 0x1 ) << 29 )
#define DECODERLIB_FORCEDFEIRQ_BIT_GET( uVal )  (( uVal >> 29 ) & 0x1 )                                              
                                              
/////////////////////////////////////////////////////////////////////////////////
//  Global Types
/////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// Decoder Library HW State

typedef enum
{
  MALONE_INACTIVE  = 0,
  MALONE_ACTIVE,
  MALONE_BE_ACTIVE

} MALONE_STATE;

//////////////////////////////////////////////////////////////
// Decoder Library HW Control Context Info

typedef struct
{

  MALONE_STATE eState;
  u_int32      uStrID;

  u_int32      uForceFIQ;
  u_int32      uForceDFEFIQ;
  u_int32      uChipVer;                /* Hold "malone" version to be filled in from Malone HW register   */
  u_int32      uChipMnrVer;             /* Hold "malone" minor version to be filled in from Malone HW register   */
  u_int32      uChipSubVer;             /* Hold metal version to be filled in by higher level control      */
  u_int32      uHWFeatures;
  u_int32      uForceCacheFlush;        /* Cache reset issue workaround purpose for Kronos and Fusion RevA */
  u_int32      bPreparser;              /* Set TRUE to indicate a preparser is in place                    */
  u_int32      bCQ;                     /* Set TRUE to indicate a command queue is in place                */
  u_int32      bDCP;                    /* Set TRUE to indicate unit can operate in decoupled mode         */
  u_int32      uNumDBEs;                /* Indicates the number of decoupled back ends available           */
  u_int32      bFBC;                    /* Set TRUE to indicate Frame buffer Compression support           */
  u_int32      uRSBSize;
  u_int32      uMaloneID;
  u_int32      bPixIf;
  u_int32      bXBUS;

  u_int32      uCQProcType[DECODERLIB_MAX_CQ_PER_MALONE]; /* Specifies what type of processing each of the CQ units is doing */

  MALONE_STATE eDFEState;                                 /* If the front end of the HW is active in isolation - specifies state */
  u_int8       usDFEStrID;                                /* If the front end of the HW is active in isolation - specifies       */
                                                          /* which stream is assigned                                            */

  u_int32      uDmaMemSize;
  void       * pDmaMemArea;  
                                                          
  MvdHwRegMap     * msd_regp;
  MvdHwRegHifMap  * hif_regp;
  MvdHwRegSifMap  * sif_regp;
  MvdHwRegCtxMap  * ctx_regp;
  MvdHwReg        * rsb_regp;
  MvdHwRegRprMap  * rpr_regp;
  MvdHwRegSppMap  * spp_regp;
  MvdHwRegH264Map * avc_regp;
  MvdHwRegMp2dMap * mp2d_regp;
  MvdHwRegAvsdMap * avsd_regp;
  MvdHwRegAspdMap * aspd_regp;
  MvdHwRegVc1dMap * vc1d_regp;
  MvdHwRegAspdMap * jpgd_regp;
  MvdHwRegOn2dMap * on2d_regp;
  MvdHwRegRvidMap * rvid_regp;
  MvdHwRegHevcMap * hevc_regp;
  MvdHwRegBbdMap  * bbd_regp;
  MvdHwRegDbgMap  * dbg_regp;
  MvdHwRegCqMap   * cq_regp;
  MvdHwRegRC4Map  * rc4_regp;
  MvdHwRegDfeMap  * dfe_regp;
  MvdHwRegDbeMap  * dbe_regp[DECODERLIB_MAX_DBE_UNITS];

} MALONE_KERNEL_HW_SESSION, *pMALONE_KERNEL_HW_SESSION;

/////////////////////////////////////////////////////////////////////////////////
//  Global Function definitions
/////////////////////////////////////////////////////////////////////////////////


void mvd_kernel_hw_control_init ( DECODERLIB_KERNEL_CFG * pCfg );

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_kernel_hw_set_focus                                            //
//                                                                                //
//  DESCRIPTION: Changes focus to the selected Malone HW unit                     //
//                                                                                //
//  INPUTS:      uMaloneID - The ID of the Malone                                 //
//                                                                                //
//  OUTPUTS:     ppMVDHw   - Pointer to a HW session structure for the specified  //
//                           Malone                                               //
//                                                                                //
//  RETURNS:     None.                                                            //
//                                                                                //
//  NOTES:       Only makes sense in multi-malone configs                         //
//                                                                                //
//  CONTEXT:     This function may be called from any context                     //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

void mvd_kernel_hw_set_focus ( u_int32 uMaloneID, pMALONE_KERNEL_HW_SESSION * ppMVDHw );

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_kernel_hw_init_handles                                       //
//                                                                                //
//  DESCRIPTION: Initialise the HW session handles                                //
//                                                                                //
//  INPUTS:      pCfg      - A pointer to the DecLib Config structure             //
//               bSoftInit - If set, the FW pointers get established - leave      //
//                           FALSE when restarting from a snapshot                //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     None.                                                            //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     This function must be called from non-interrupt context          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

void mvd_kernel_hw_init_handles ( DECODERLIB_KERNEL_CFG * pCfg,
                                  bool             bSoftInit );

#endif /* _DECODER_LIB_HW_CONTROL_H_ */

/* End of file */
