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

  Author    : Media IP FW team ( Belfast and Shanghai )
  File name : DecLibHWControl.c
  Notes     : Processes commands from decoder lib scheduler
              Establishes context for and makes calls to
              the base decoders
              This file provides the hardware facing aspect to
              the interface. It is part of a group with
              DecLibStreamControl.c which provides the
              HW funtionality

 ******************************************************/

/////////////////////////////////////////////////////////////////////////////////
//  Header Files
/////////////////////////////////////////////////////////////////////////////////

#include "basetype.h"
#include "mediaip_fw_types.h"
#include "pal.h"

#include "mvd_types.h"
#include "mvd_reg_map.h"
#include "mvd_sif_control.h"

#include "DecKernelLibPrivate.h"
#include "DecKernelLibHWControl.h"

/////////////////////////////////////////////////////////////////////////////////
//  Extern Function Prototypes
/////////////////////////////////////////////////////////////////////////////////

extern MEDIAIP_IRQ_RETCODE mvd_kernel_hw_primary_isr ( u_int32 irq_val );
extern MEDIAIP_IRQ_RETCODE mvd_kernel_hw_secondary_isr ( u_int32 irq_val );

/////////////////////////////////////////////////////////////////////////////////
//  Private Function Prototypes
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////;

extern DEC_KERNEL_LIB              gDecKernelLib;
       MALONE_KERNEL_HW_SESSION    gMvdKernelHw[(DECODERLIB_MAX_MALONES + 1)];
       pMALONE_KERNEL_HW_SESSION   pgMVDKernelHw;

u_int32                    gMaloneList[(DECODERLIB_MAX_MALONES + 1)] = { MALONE_HW_1,
#if DECODERLIB_MAX_MALONES > 1
                                                                         MALONE_HW_2,
#endif
                                                                         MALONE_SW };

static u_int32      uSWMaloneRegSpace[2048];   /* Until we are sure we have eliminated register access */
                                               /* for functions using the 'SW Malone', point them here */

extern u_int32      uDecLibIrqPin[DECODERLIB_MAX_MALONES][0x2];

/////////////////////////////////////////////////////////////////////////////////
//  Code
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_kernel_hw_control_init                                         //
//                                                                                //
//  DESCRIPTION: Perform all HW initialisation and shared structure setup         //
//                                                                                //
//  INPUTS:      uMaloneID - The ID of the Malone for which the info is to        //
//                           be restored                                          //
//               pCtxArea  - Pointer to area of memory from which data is to be   //
//                           read                                                 //
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

void mvd_kernel_hw_control_init ( DECODERLIB_KERNEL_CFG * pCfg )
{
  /* Init the handles                             */
  /* This sets up the reg pointers so do it first */
  mvd_kernel_hw_init_handles ( pCfg,
                               TRUE );
  
  /* Default global pointers to a 1st Malone focus  */
  mvd_kernel_hw_set_focus ( MALONE_HW_1, &pgMVDKernelHw  );

}


////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_kernel_hw_set_malone_instance                                  //
//                                                                                //
//  DESCRIPTION: Look for the most appropriate Malone instance to service         //
//               the command for a specified stream                               //
//                                                                                //
//  INPUTS:      uStrId   - The Stream ID                                         //
//               bSWCmd   - is the command to be carried out for this stream      //
//                          capable of being handled without using the HW engine? //
//               uHWIndex - HW Index suggested by scheduler                       //
//               bUseSch  - Set to guarantee scheduler suggested unit is used     //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     The most suitable Malone ID to be used                           //
//                                                                                //
//  NOTES:       This function must return a value.                               //
//                                                                                //
//  CONTEXT:     This function must be called from non-interrupt context          //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

u_int32 mvd_kernel_hw_set_malone_instance ( u_int32 uStrId,
                                            bool    bSWCmd,
                                            u_int32 uHWIndex,
                                            bool    bUseSch
                                          )
{
  u_int32 uIdx      = 0;

  if ( bUseSch )
  {
    return uHWIndex;
  }

  for ( uIdx = 0; uIdx < gDecKernelLib.uNumMalones; uIdx++ )
  {
    /* Even if there is no command active, this is a valid check as */
    /* it avoids need for a context change if it matches            */
    if ( gMvdKernelHw[uIdx].uStrID == uStrId )
    {
      return uIdx;
    }
  }

  if ( bSWCmd ) return MALONE_SW;

  /* Look for free Malone - this function should not be called */
  /* unless this is a SW command or there is a free Malone     */

  for ( uIdx = 0; uIdx < gDecKernelLib.uNumMalones; uIdx++ )
  {
    /* Even if there is no command active, this is a valid check as */
    /* it avoids need for a context change if it matches            */
    if ( gMvdKernelHw[uIdx].eState == MALONE_INACTIVE )
    {
      break;
    }
  }

  /* Do not assign this stream and do NOT make it active yet */
  /* Only make this malone active when the command is issued */
  /* This is necessary so we we can trigger context switches */

  /* We should set the global pointer though so calls to sif */
  /* etc can get correct functions!!                         */

  mvd_kernel_hw_set_focus ( uIdx, &pgMVDKernelHw );

  return uIdx;

}

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

void mvd_kernel_hw_set_focus ( u_int32 uMaloneID, pMALONE_KERNEL_HW_SESSION * ppMVDHw )
{
  *ppMVDHw = ( MALONE_KERNEL_HW_SESSION * )&gMvdKernelHw[uMaloneID];
}



////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_kernel_hw_init_handles                                         //
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
                                  bool                    bSoftInit
                                )
{
  u_int32            uIdx;
  pMALONE_KERNEL_HW_SESSION pMVDHw;
  MvdHwRegMap       *pMvdReg;

  for ( uIdx = 0; uIdx < (gDecKernelLib.uNumMalones + 1); uIdx++ )
  {
    /* If we have set up all the HW handles then move to the SW */
    /* handle and set it up                                     */
    if ( uIdx == gDecKernelLib.uNumMalones ) uIdx = DECODERLIB_MAX_MALONES;
    
    pMVDHw                 = &gMvdKernelHw[uIdx];
    
    if ( bSoftInit )
    {
      pMVDHw->eState       = MALONE_INACTIVE;
      pMVDHw->uStrID       = 0;
      pMVDHw->uForceFIQ    = 0;
      pMVDHw->uForceDFEFIQ = 0;

      /* Malone ID that the instance uses, normally fixed... */
      pMVDHw->uMaloneID    = gMaloneList[uIdx];
    }

    if ( pMVDHw->uMaloneID == DECODERLIB_MAX_MALONES )
    {
      pMvdReg = ( MvdHwRegMap * )uSWMaloneRegSpace;
    }
    else if ( pMVDHw->uMaloneID == MALONE_HW_2 )
    {
      pMvdReg = ( MvdHwRegMap * ) ( pCfg->uMaloneBaseAddr[0x1] + pCfg->uMaloneHifOffset[0x1] );
    }
    else
    {
      pMvdReg = ( MvdHwRegMap * ) ( pCfg->uMaloneBaseAddr[0x0] + pCfg->uMaloneHifOffset[0x0] );
    }

    if ( pMVDHw->uMaloneID == MALONE_SW )
    {
      pMVDHw->msd_regp      = ( MvdHwRegMap     * )pMvdReg;
      pMVDHw->hif_regp      = ( MvdHwRegHifMap  * )pMvdReg;
      pMVDHw->sif_regp      = ( MvdHwRegSifMap  * )pMvdReg;
      pMVDHw->ctx_regp      = ( MvdHwRegCtxMap  * )pMvdReg;
      pMVDHw->rsb_regp      = ( MvdHwReg        * )pMvdReg;
      pMVDHw->rpr_regp      = ( MvdHwRegRprMap  * )pMvdReg;
      pMVDHw->spp_regp      = ( MvdHwRegSppMap  * )pMvdReg;
      pMVDHw->avc_regp      = ( MvdHwRegH264Map * )pMvdReg;
      pMVDHw->mp2d_regp     = ( MvdHwRegMp2dMap * )pMvdReg;
      pMVDHw->avsd_regp     = ( MvdHwRegAvsdMap * )pMvdReg;
      pMVDHw->aspd_regp     = ( MvdHwRegAspdMap * )pMvdReg;
      pMVDHw->vc1d_regp     = ( MvdHwRegVc1dMap * )pMvdReg;
      pMVDHw->jpgd_regp     = ( MvdHwRegAspdMap * )pMvdReg;
      pMVDHw->rvid_regp     = ( MvdHwRegRvidMap * )pMvdReg;
      pMVDHw->hevc_regp     = ( MvdHwRegHevcMap * )pMvdReg;
      pMVDHw->on2d_regp     = ( MvdHwRegOn2dMap * )pMvdReg;
      pMVDHw->cq_regp       = ( MvdHwRegCqMap   * )pMvdReg;
      pMVDHw->rc4_regp      = ( MvdHwRegRC4Map  * )pMvdReg;
      pMVDHw->dfe_regp      = ( MvdHwRegDfeMap  * )pMvdReg;
      pMVDHw->dbe_regp[0x0] = ( MvdHwRegDbeMap  * )pMvdReg;
      pMVDHw->dbe_regp[0x1] = ( MvdHwRegDbeMap  * )pMvdReg;

    }
    else
    {
      pMVDHw->msd_regp       = pMvdReg;
      pMVDHw->hif_regp       = &( pMvdReg->HifMap.hif_map );
      pMVDHw->sif_regp       = &( pMvdReg->SifMap.sif_map );
      pMVDHw->ctx_regp       = &( pMvdReg->CtxMap.ctx_map );
      pMVDHw->rpr_regp       = &( pMvdReg->RprMap.rpr_map );
      pMVDHw->spp_regp       = &( pMvdReg->SppMap.spp_map );
      pMVDHw->avc_regp       = &( pMvdReg->DecMap.h264_map );
      pMVDHw->mp2d_regp      = &( pMvdReg->DecMap.mp2d_map );
      pMVDHw->aspd_regp      = &( pMvdReg->DecMap.aspd_map );
      pMVDHw->avsd_regp      = &( pMvdReg->DecMap.avsd_map );
      pMVDHw->vc1d_regp      = &( pMvdReg->DecMap.vc1d_map );
      pMVDHw->jpgd_regp      = &( pMvdReg->DecMap.aspd_map );
      pMVDHw->on2d_regp      = &( pMvdReg->DecMap.on2d_map );
      pMVDHw->rvid_regp      = &( pMvdReg->DecMap.rvid_map );
      pMVDHw->hevc_regp      = &( pMvdReg->DecMap.hevc_map );
      pMVDHw->bbd_regp       = &( pMvdReg->BbdMap.bbd_map );
      pMVDHw->cq_regp        = &( pMvdReg->CqMap.cq_map );
      pMVDHw->rc4_regp       = &( pMvdReg->RC4Map.RC4_map );
      pMVDHw->dfe_regp       = &( pMvdReg->DcpMap.dfe_map );
      pMVDHw->dbe_regp[0x0]  = &( pMvdReg->DcpMap.dbe_map[0x0] );
      pMVDHw->dbe_regp[0x1]  = &( pMvdReg->DcpMap.dbe_map[0x1] );

      if ( pMVDHw->uMaloneID == MALONE_HW_2 )
      {
        pMVDHw->rsb_regp = ( MvdHwReg * ) pCfg->uMaloneBaseAddr[0x1]  ;
      }
      else
      {
        pMVDHw->rsb_regp = ( MvdHwReg * ) pCfg->uMaloneBaseAddr[0x0] ;
      }
    }
  }
}

/* End of File */
