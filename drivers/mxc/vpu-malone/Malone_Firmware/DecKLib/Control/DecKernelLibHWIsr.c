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

  File name : DecLibHWIsr.c
  Notes     : Provides the HW Interrupt Service Routines

 *******************************************************/

/////////////////////////////////////////////////////////////////////////////////
//  Header Files
/////////////////////////////////////////////////////////////////////////////////
 
#include "basetype.h"
#include "mediaip_fw_types.h"           
#include "pal.h"

#include "mvd.h"
#include "mvd_types.h"
#include "mvd_reg_map.h"
#include "mvd_sif_control.h"

#include "DecKernelLibHWControl.h"
#include "DecKernelLibPrivate.h"

/////////////////////////////////////////////////////////////////////////////////
//  Global Variables                                                            
/////////////////////////////////////////////////////////////////////////////////

extern pMALONE_KERNEL_HW_SESSION pgMVDKernelHw;                       
extern u_int32                   uMvdKernelIrqPin[DECODERLIB_MAX_MALONES][0x2]; 
extern DEC_KERNEL_LIB            gDecKernelLib;

/////////////////////////////////////////////////////////////////////////////////
//  Global Macros                                                            
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Private Prototypes
/////////////////////////////////////////////////////////////////////////////////

void  mvd_kernel_hw_isr_event ( u_int32   uMaloneIdx,
                                u_int32 * puIrqStatus );

/////////////////////////////////////////////////////////////////////////////////
//  Extern Prototypes
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Code                                                     
/////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_hw_sesh_isr                                                  // 
//                                                                                //
//  DESCRIPTION: Malone HW ISR                                                    // 
//                                                                                //
//  INPUTS:      irq_val - Identification Parameter from which source of call     //
//                         may be ascertained                                     //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     MEDIAIP_IRQ_RETCODE                                              //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     This function may be called from any context                     //
//               Since it is an isr, that may be strange but it can be called to  //
//               handle SW commands in thread context only                        //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

#define CTX_DBG(x)

MEDIAIP_IRQ_RETCODE mvd_kernel_hw_primary_isr ( u_int32 irq_val )
{
  u_int32        sif_irq_status[0x3] = { 0x0 };
  u_int32        uActMaloneID;
#if ( TARGET_APP == GTB_DEC ) || ( TARGET_APP == MEDIA_DEC ) || ( TARGET_APP == VIDEO_TRANS ) || ( TARGET_APP == GTB_TRANS )
  u_int32        uIrqTarget0, uIrqTarget1;
#endif
  u_int32        uForceFIQ = 0, uForceDFEFIQ = 0; 
  u_int32 uHEVCActive;

  /* Future Proof... */
  /* If we have two levels of interrupt or we process interrupts in thread context */
  /* we need to save and restore HW focus                                          */
  pMALONE_KERNEL_HW_SESSION pLocMVDHw = pgMVDKernelHw;  
   
  /* Work out which Malone instance caused this isr and set HW focus accordingly   */

#if ( TARGET_APP == GTB_DEC ) || ( TARGET_APP == MEDIA_DEC ) || ( TARGET_APP == VIDEO_TRANS ) || ( TARGET_APP == GTB_TRANS )

  pal_int_get_irq_line ( uMvdKernelIrqPin[0x0][0x0],
                         &uIrqTarget0
                       );
  pal_int_get_irq_line ( uMvdKernelIrqPin[0x0][0x1],
                         &uIrqTarget1
                       );

  uActMaloneID = ( irq_val == MALONE_SW_IRQ )                              ? MALONE_SW   :
                 (( irq_val == uIrqTarget0 ) | ( irq_val == uIrqTarget1 )) ? MALONE_HW_1 :     
                 MALONE_HW_2;

#else

  uActMaloneID = ( irq_val == MALONE_SW_IRQ )                                                            ? MALONE_SW   :
                 (( irq_val == uMvdKernelIrqPin[0x0][0x0] ) | ( irq_val == uMvdKernelIrqPin[0x0][0x1] )) ? MALONE_HW_1 :     
                 MALONE_HW_2;

#endif

  mvd_kernel_hw_set_focus ( uActMaloneID, &pgMVDKernelHw );

  /* work out if HEVC active, as if not we don't care about irq_status3 DBE*/
  MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_CTX, pgMVDKernelHw->ctx_regp->ctx_status, uHEVCActive );
  
  if(uHEVCActive&0x400)
    uHEVCActive = 1;
  else
    uHEVCActive = 0;
  

  
#if 1  
  MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->msd_data0_reg, uForceFIQ    );
  MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->msd_data1_reg, uForceDFEFIQ );
  
  if ( uForceFIQ )
  {
    MVD_REG_WRITE( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->msd_data0_reg, 0 );
  }
  
  if ( uForceDFEFIQ )
  {
    MVD_REG_WRITE( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->msd_data1_reg, 0 );
  }
#else  
  MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->msd_data0_reg, uData );
  
  MVD_REG_WRITE( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->msd_data0_reg, 0 );
  
  uForceFIQ    = ( uData & 0x1 ) ? 1 : 0;
  uForceDFEFIQ = ( uData & 0x2 ) ? 1 : 0;
#endif
  
  if ( uActMaloneID != MALONE_SW )
  {  
    u_int32 uClearVal;
    
    /* Break down sif IRQ sources */
    MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr_status, sif_irq_status[0x0] );
    MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] );
    MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr3_status, sif_irq_status[0x2] );

    pal_trace( DECODER_TL_INFO, "mvd_kernel_hw_primary_isr(%d): opening sif_irq_status  = 0x%x\n", irq_val, sif_irq_status[0x0]);
    pal_trace( DECODER_TL_INFO, "mvd_kernel_hw_primary_isr(%d): opening sif_irq_status[0x1] = 0x%x\n", irq_val, sif_irq_status[0x1]);
    pal_trace( DECODER_TL_INFO, "mvd_kernel_hw_primary_isr(%d): opening sif_irq_status[0x2] = 0x%x\n", irq_val, sif_irq_status[0x2]);

    /* Clear the interrupt bits we're going to handle here */

#ifdef FW_PES_PARSE_AS_FIQ
    
    uClearVal = sif_irq_status[0x0] & ( MSD_SIF_INTR_BSDMA_BIT      |
                                        MSD_SIF_INTR_FORCE_EXIT_BIT |
                                        MSD_SIF_INTR_DISPQ_PULL_BIT |
                                        MSD_SIF_INTR_SEMAPHORE_BIT  |
                                        MSD_SIF_INTR_IMAGE_DONE_BIT |
                                        MSD_SIF_INTR_SLICE_DONE_BIT |
                                        MSD_SIF_INTR_FORCE_ENTRY_BIT 
                                      );

    MVD_REG_WRITE( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr_status, uClearVal );
    
#else

    uClearVal = sif_irq_status[0x0] & ( MSD_SIF_INTR_BSDMA_BIT       |
                                        MSD_SIF_INTR_PES_BIT         |
                                        MSD_SIF_INTR_SCODE_FOUND_BIT |
                                        MSD_SIF_INTR_DISPQ_PULL_BIT  |
                                        MSD_SIF_INTR_SEMAPHORE_BIT   |
                                        MSD_SIF_INTR_IMAGE_DONE_BIT  |
                                        MSD_SIF_INTR_SLICE_DONE_BIT  |
                                        MSD_SIF_INTR_FORCE_ENTRY_BIT
                                      );

    MVD_REG_WRITE( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr_status, uClearVal );
    
#endif /* FW_PES_PARSE_AS_FIQ */

    /* Read it back to ensure write has made it to Malone        */
    MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr_status, uClearVal );
    
    /* Also re-clear the INTC bit which brought us in here since */
    /* it will have been reset.                                  */
    pal_int_clear ( irq_val,
                    TRUE 
                  );

#ifdef FW_PARSE_PES
#ifdef FW_PES_PARSE_AS_FIQ

    /* Pes FW handling done at higher level         */
    /* Re-maps the force-exit ISR as the startcode  */
    
    /* TODO-KMC */
    /* Does this mean a normal engine startcode is mapped as a force-exit? I guess it does */
    /* So if we were parsing PES as an FIQ we would not actually be in this function if it */
    /* were a PES startcode and so the below event must be a normal startcode?             */
    
    if ( sif_irq_status[0x0] & MSD_SIF_INTR_FORCE_EXIT_BIT )
    {
      sif_irq_status[0x0] |= MSD_SIF_INTR_SCODE_FOUND_BIT;
    }
#else
    /* PES isr */
    if ( sif_irq_status[0x0] & MSD_SIF_INTR_SCODE_FOUND_BIT )
    {
      u_int32 scode_status;
    
      MVD_REG_READ( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->bs2rbsp_status, scode_status );
      
      if ( scode_status & MSD_SIF_BS2RBSP_GOT_PESSCODE )
      {
        sesh_pes_isr ( irq_val );
      }
    
      /* If it was just a PES startcode, then don't tell the engine  */
      /* that there was a startcode, as will get handled later       */
      if (( scode_status & MSD_SIF_BS2RBSP_GOT_SCODE ) == 0 )
      {
        sif_irq_status[0x0] &= ~(MSD_SIF_INTR_SCODE_FOUND_BIT);
      }
    }
#endif
#endif

    if ( sif_irq_status[0x1] & ( MSD_SIF_INTR2_CSC_BIT ) )
    {      
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] & ( MSD_SIF_INTR2_CSC_BIT ) );   
    }

    if ( sif_irq_status[0x1] & ( MSD_SIF_INTR2_CQ_BIT ) )
    {      
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] & ( MSD_SIF_INTR2_CQ_BIT ) );   
    }

    if ( sif_irq_status[0x1] & ( MSD_SIF_INTR2_DFE_DONE_BIT ) )
    {
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] & ( MSD_SIF_INTR2_DFE_DONE_BIT ) );
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] & ( MSD_SIF_INTR2_DFE_SLC_DONE_BIT ) );      
    }

    if ( sif_irq_status[0x1] & MSD_SIF_INTR2_DFE_SLC_DONE_BIT )
    {      
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] & ( MSD_SIF_INTR2_DFE_SLC_DONE_BIT ) ); 
    }  

    if ( sif_irq_status[0x2] & MSD_SIF_INTR3_DBE0_CQ_BIT )
    {      
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr3_status, sif_irq_status[0x2] & ( MSD_SIF_INTR3_DBE0_CQ_BIT ) );   
    }
 
    if ( sif_irq_status[0x2] & MSD_SIF_INTR3_DBE1_CQ_BIT )
    {      
      MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr3_status, sif_irq_status[0x2] & ( MSD_SIF_INTR3_DBE1_CQ_BIT ) );        
    }

    if ( sif_irq_status[0x2] & 
           ( MSD_SIF_INTR3_DBE0_DONE_BIT | MSD_SIF_INTR3_DBE1_DONE_BIT )
       )
    {
      u_int32 uMask = ( MSD_SIF_INTR3_DBE0_DONE_BIT | 
                        MSD_SIF_INTR3_DBE1_DONE_BIT   );
      
      if (( sif_irq_status[0x0] & MSD_SIF_INTR_IMAGE_DONE_BIT )||(uHEVCActive==0))
      {
        MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr3_status, sif_irq_status[0x2] & uMask );
      }
      else
      {
        sif_irq_status[0x2] &= ~uMask;
      }
    } 
      
    if ( sif_irq_status[0x2] & 
           ( MSD_SIF_INTR3_DBE0_SLC_DONE_BIT | MSD_SIF_INTR3_DBE1_SLC_DONE_BIT )
       )
    {
      u_int32 uMask = ( MSD_SIF_INTR3_DBE0_SLC_DONE_BIT | 
                        MSD_SIF_INTR3_DBE1_SLC_DONE_BIT   );
    
      if (( sif_irq_status[0x0] & MSD_SIF_INTR_SLICE_DONE_BIT )|| (uHEVCActive==0))
      {
        MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr3_status, sif_irq_status[0x2] & uMask );
      }
      else
      {
        sif_irq_status[0x2] &= ~uMask;
      }
    }       
  }
  else /* We wish to process a command without accessing HW */
  {
    sif_irq_status[0x0] = 0;
  }
    
  pal_trace( DECODER_TL_DEBUG, "-> mvd_kernel_hw_primary_isr( ) : uForceFIQ = 0x%x\n", uForceFIQ );     

  /* Decode engine IRQs */
  if (( uActMaloneID == MALONE_SW )    ||
      ( uForceFIQ )     ||
      ( uForceDFEFIQ )  ||
      ( sif_irq_status[0x0] & ( MSD_SIF_INTR_IMAGE_DONE_BIT | MSD_SIF_INTR_SLICE_DONE_BIT | MSD_SIF_INTR_SCODE_FOUND_BIT | MSD_SIF_INTR_FORCE_ENTRY_BIT | MSD_SIF_INTR_BSDMA_BIT )) ||
      ( sif_irq_status[0x1] & ( MSD_SIF_INTR2_CSC_BIT       | MSD_SIF_INTR2_DFE_DONE_BIT  | MSD_SIF_INTR2_DFE_SLC_DONE_BIT | MSD_SIF_INTR2_CQ_BIT )) ||  
      ( sif_irq_status[0x2] & ( MSD_SIF_INTR3_DBE0_DONE_BIT | MSD_SIF_INTR3_DBE1_DONE_BIT | MSD_SIF_INTR3_DBE0_SLC_DONE_BIT | MSD_SIF_INTR3_DBE1_SLC_DONE_BIT | MSD_SIF_INTR3_DBE0_CQ_BIT | MSD_SIF_INTR3_DBE1_CQ_BIT ))
     )
  {
    pal_trace ( DECODER_TL_DEBUG, 
                "-> mvd_kernel_hw_primary_isr( ) : sif_irq_status[0x0] = 0x%x : sif_irq_status[0x1] = 0x%x : sif_irq_status[0x2] = 0x%x : uForceFIQ  = 0x%x\n",
                sif_irq_status[0x0], 
                sif_irq_status[0x1], 
                sif_irq_status[0x2], 
                uForceFIQ
              );
              
    /* Only pass events not serviced inline in this isr */
            
    sif_irq_status[0x0] &= ( MSD_SIF_INTR_IMAGE_DONE_BIT | MSD_SIF_INTR_SLICE_DONE_BIT | MSD_SIF_INTR_SCODE_FOUND_BIT | MSD_SIF_INTR_FORCE_ENTRY_BIT | MSD_SIF_INTR_BSDMA_BIT );
    sif_irq_status[0x1] &= ( MSD_SIF_INTR2_CSC_BIT       | MSD_SIF_INTR2_DFE_DONE_BIT  | MSD_SIF_INTR2_DFE_SLC_DONE_BIT | MSD_SIF_INTR2_CQ_BIT);  
    sif_irq_status[0x2] &= ( MSD_SIF_INTR3_DBE0_DONE_BIT | MSD_SIF_INTR3_DBE1_DONE_BIT | MSD_SIF_INTR3_DBE0_SLC_DONE_BIT | MSD_SIF_INTR3_DBE1_SLC_DONE_BIT | MSD_SIF_INTR3_DBE0_CQ_BIT | MSD_SIF_INTR3_DBE1_CQ_BIT );

    /* When passing to thread - explicitly set that a force irq has been set */
    /* so that the thread function knows after which interrupt it can reset  */
    /* pMVDHw->uForceFIQ                                                     */
    
    sif_irq_status[0x0] |= DECODERLIB_FORCEIRQ_BIT_SET( uForceFIQ );
    sif_irq_status[0x0] |= DECODERLIB_FORCEDFEIRQ_BIT_SET( uForceDFEFIQ );
    
    if (( sif_irq_status[0x0] ) ||
        ( sif_irq_status[0x1] ) ||
        ( sif_irq_status[0x2] )
       )
    {
      mvd_kernel_hw_isr_event ( uActMaloneID,
                                sif_irq_status );
    }
  }

  /* Future Proof - Restore pgMVDKernelHw */
  pgMVDKernelHw = pLocMVDHw;
  
  return MEDIAIP_FW_STATUS_OK;

}

////////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    mvd_kernel_hw_secondary_isr                                        // 
//                                                                                //
//  DESCRIPTION: Malone HW ISR 2                                                  // 
//                                                                                //
//  INPUTS:      irq_val - Identification Parameter from which source of call     //
//                         may be ascertained                                     //
//                                                                                //
//  OUTPUTS:     None.                                                            //
//                                                                                //
//  RETURNS:     MEDIAIP_IRQ_RETCODE                                              //
//                                                                                //
//  NOTES:       None.                                                            //
//                                                                                //
//  CONTEXT:     This function may be called from any context                     //
//               Since it is an isr, that may be strange but it can be called to  //
//               handle SW commands in thread context only                        //
//                                                                                //
////////////////////////////////////////////////////////////////////////////////////

MEDIAIP_IRQ_RETCODE mvd_kernel_hw_secondary_isr ( u_int32 irq_val )
{
  u_int32 uActMaloneID;
  u_int32 uTemp;
  u_int32 sif_irq_status[0x3] = { 0x0 };
#if ( TARGET_APP == GTB_DEC ) || ( TARGET_APP == MEDIA_DEC ) || ( TARGET_APP == VIDEO_TRANS ) || ( TARGET_APP == GTB_TRANS )
  u_int32 uIrqTarget0, uIrqTarget1;
#endif
 
  /* Future Proof... */
  /* If we have two levels of interrupt we need to save and restore HW focus    */
  pMALONE_KERNEL_HW_SESSION pLocMVDHw = pgMVDKernelHw;  
 
  /* Work out which Malone instance cause this isr and set HW focus accordingly */  

#if ( TARGET_APP == GTB_DEC ) || ( TARGET_APP == MEDIA_DEC ) || ( TARGET_APP == VIDEO_TRANS ) || ( TARGET_APP == GTB_TRANS )

  pal_int_get_irq_line ( uMvdKernelIrqPin[0x0][0x0],
                         &uIrqTarget0
                       );
  pal_int_get_irq_line ( uMvdKernelIrqPin[0x0][0x1],
                         &uIrqTarget1
                       );

  uActMaloneID = ( irq_val == MALONE_SW_IRQ )                              ? MALONE_SW   :
                 (( irq_val == uIrqTarget0 ) | ( irq_val == uIrqTarget1 )) ? MALONE_HW_1 :     
                 MALONE_HW_2;

#else

  uActMaloneID = ( irq_val == MALONE_SW_IRQ )                                                            ? MALONE_SW   :
                 (( irq_val == uMvdKernelIrqPin[0x0][0x0] ) | ( irq_val == uMvdKernelIrqPin[0x0][0x1] )) ? MALONE_HW_1 :     
                 MALONE_HW_2;

#endif

  mvd_kernel_hw_set_focus ( uActMaloneID, &pgMVDKernelHw );
  
  if ( uActMaloneID != MALONE_SW )
  {  
    /* Break down sif IRQ sources      */
    APB_REG_READ( pgMVDKernelHw->sif_regp->intr_status, sif_irq_status[0x0] );
    
    /* Clear those we will handle here */
    MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SIF*/, pgMVDKernelHw->sif_regp->intr_status, sif_irq_status[0x0] & MSD_SIF_INTR2_EXTENSION_BIT_2 );
   
    /* Read it back to ensure write has made it to Malone        */
    /* Using uActMaloneID as read dsetination as it is no longer */
    /* required                                                  */
    APB_REG_READ( pgMVDKernelHw->sif_regp->intr_status, uTemp );
    
    /* Also re-clear the INTC bit which brought us in here since */
    /* it will have been reset.                                  */
    pal_int_clear ( irq_val,
                    TRUE );

    if ( sif_irq_status[0x0] & MSD_SIF_INTR2_EXTENSION_BIT_2 ) 
    {       
      /* Break down extension IRQ sources */
      APB_REG_READ( pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1]);
    
      /* Clear those we will handle here  */
      MVD_REG_WRITE( pgMVDKernelHw, DECLIB_DBG_REG_SIF, pgMVDKernelHw->sif_regp->intr2_status, sif_irq_status[0x1] & ( MSD_SIF_CTRL2_SPP_SCODE_INTR_ENAB_BIT |
                                                                                                                 MSD_SIF_CTRL2_SPP_PESSC_INTR_ENAB_BIT |
                                                                                                                 MSD_SIF_CTRL2_SPP_BSDMA_INTR_ENAB_BIT ));

      /* Bugzilla 237 - PES Interupt enable inadvertantly set through a RMW of sif_regp->control */
      if ( sif_irq_status[0x1] & MSD_SIF_CTRL2_SPP_PESSC_INTR_ENAB_BIT )
      {
        u_int32 pes_status;

        APB_REG_READ ( pgMVDKernelHw->spp_regp->pes_status, pes_status );

        if ( MSD_SIF_PES_STATUS_GET_STATE( pes_status ) == pes_WaitClear )
        {
          /* Pending DTS/PTS looks like it is locked out reception of another startcode */
          MVD_REG_WRITE( pgMVDKernelHw, 0x0 /*DECLIB_DBG_REG_SPP*/, pgMVDKernelHw->spp_regp->pes_ctrl, 0x0 );
        }
      }

      sif_irq_status[0x0] |= DECODERLIB_SECONDARY_ISR_EVENT;

      mvd_kernel_hw_isr_event ( uActMaloneID,
                                sif_irq_status );
      
    }
    
    /* What other interrupts might we wish to handle in a seperate context? */
    /* Only PES is known so far                                             */
     
  }  

  /* Future Proof - Restore pgMVDKernelHw */
  pgMVDKernelHw = pLocMVDHw;

  return MEDIAIP_FW_STATUS_OK;

}  

////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    internal_decoder_kernel_lib_isr_event                        //
//                                                                            //
//  DESCRIPTION: Callback function for base decoder to schedule the           //
//               processing of the isr in thread context                      //
//                                                                            //
//  INPUTS:      None                                                         //
//                                                                            //
//  OUTPUTS:     None                                                         //
//                                                                            //
//  RETURNS:     None                                                         //
//                                                                            //
//  NOTES:       Will be called in ISR or thread context by base decoders     //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////

void  mvd_kernel_hw_isr_event ( u_int32            uMaloneIdx,
                                u_int32 *          puIrqStatus )
{
  DECODER_KERNEL_LIB_ISR_EVENT_DATA sData;
  
  sData.uMalIdx         = uMaloneIdx;
  sData.uIrqStatus[0x0] = puIrqStatus[0x0];
  sData.uIrqStatus[0x1] = puIrqStatus[0x1];
  sData.uIrqStatus[0x2] = puIrqStatus[0x2];
  
  gDecKernelLib.pfCallback[uMaloneIdx] ( &sData );
  
}

/* End of file */
