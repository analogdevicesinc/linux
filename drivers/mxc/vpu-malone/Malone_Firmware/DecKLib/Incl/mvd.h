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

  Author    : Media IP FW team
  File name : mvd.h
  Notes     : Replaces misnamed "global.h"

 ***********************************************/

#ifndef _MVD_H_
#define _MVD_H_

/////////////////////////////////////////////////////////////////////////////////
//  Header files
/////////////////////////////////////////////////////////////////////////////////

#include "basetype.h"
#include "DecKernelLibCfg.h"

/////////////////////////////////////////////////////////////////////////////////
//  Global Variables
/////////////////////////////////////////////////////////////////////////////////
extern u_int32 uMvdKernelIrqPin[DECODERLIB_MAX_MALONES][0x2];

/////////////////////////////////////////////////////////////////////////////////
//  Macros
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// Register access model
//

///////////////////////////////////////////////////
// This is the option used for most builds

#ifndef MVD_DEBUG_REG_ACCESS
#if TARGET_LEVEL == CMODEL
  extern u_int32 dec_model_malone_reg_write ( u_int32 * puRegAddr, u_int32   uWrtData );
  extern u_int32 dec_model_malone_reg_read  ( u_int32 * puRegAddr, u_int32   uReadData );
  
	#define APB_REGISTER_WRITE(a,b) dec_model_malone_reg_write ( ( u_int32 * )a, ( u_int32  )b )
  #define APB_REGISTER_READ(a,b)  b = *a; dec_model_malone_reg_read  ( ( u_int32 * )a, ( u_int32  )b ) 
  #define APB_REG_WRITE(a,b)      dec_model_malone_reg_write ( ( u_int32 * )&a, ( u_int32  )b )
  #define APB_REG_READ(a,b)       b = a; dec_model_malone_reg_read  ( ( u_int32 * )&a,  ( u_int32 )b ) 
#else  



#if 0
#include "VPU_debug.h"
//static u_int32 c;

#define APB_REGISTER_WRITE(a,b) c = b; *a = c; printk("write reg 0x%p, val 0x%x, in %s\n", a, c, __FUNCTION__)
#define APB_REGISTER_READ(a,b)  c = *a; b = c; printk("read reg 0x%p, val 0x%x, in %s\n", a, c, __FUNCTION__)
#define APB_REG_WRITE(a,b)      c = b; a = c; printk("write reg 0x%p, val 0x%x, in %s\n", &a, c, __FUNCTION__)
#define APB_REG_READ(a,b)       c = a; b = c; printk("read reg 0x%p, val 0x%x, in %s\n", &a, c, __FUNCTION__)
#else

  #define APB_REGISTER_WRITE(a,b) *a = b
  #define APB_REGISTER_READ(a,b)  b = *a
  #define APB_REG_WRITE(a,b)      a = b
  #define APB_REG_READ(a,b)       b = a
#endif  

#endif
#else
  // Store reg acesses in simple debug array for dumping from debugger
  extern void decoderlib_debug_reg( volatile u_int32 * puRegAddr, u_int32 uVal);
  #define APB_REGISTER_WRITE(a,b) *a = b
  #define APB_REGISTER_READ(a,b)  b = *a
  #define APB_REG_WRITE(a,b)      decoderlib_debug_reg(( u_int32 * )&a,b)
  #define APB_REG_READ(a,b)       b = a
#endif


#ifdef MVD_DIAG_LOG_REG_ACCESS

#define DIAG_CQ_ID DECLIB_DBG_REG_CQ

extern void    decoderlib_dbg_core_log_write ( volatile u_int32 * puRegAddr, u_int32 uVal, void * pMVDHw, u_int32 uAuxData, bool bActive );
extern void    decoderlib_dbg_core_log_read  ( volatile u_int32 * puRegAddr, u_int32 *puVal, void * pMVDHw, u_int32 uAuxData, bool bActive );
extern u_int32 decoderlib_dbg_get_reg_mask ( u_int32 uStrIdx );

#define MVD_REG_WRITE(pMvdHw,aux,addr,val)     decoderlib_dbg_core_log_write ( ( u_int32 * )&addr, val, ( void * )pMvdHw, aux, gbLogRegAccessActive )
#define MVD_REG_READ(pMvdHw,aux,addr,val)      decoderlib_dbg_core_log_read  ( ( u_int32 * )&addr, &val, ( void * )pMvdHw, 0x0, gbLogRegAccessActive )
#define MVD_ADDRESS_WRITE(pMvdHw,aux,addr,val) decoderlib_dbg_core_log_write ( ( u_int32 * )addr, val, ( void * )pMvdHw, aux, gbLogRegAccessActive )
#define MVD_ADDRESS_READ(pMvdHw,aux,addr,val)  decoderlib_dbg_core_log_read  ( ( u_int32 * )addr,  &val, ( void * )pMvdHw, 0x0, gbLogRegAccessActive )

#else

#define MVD_REG_WRITE(pMvdHw,aux,addr,val)     APB_REG_WRITE(addr,val)
#define MVD_REG_READ(pMvdHw,aux,addr,val)      APB_REG_READ(addr,val)
#define MVD_ADDRESS_WRITE(pMvdHw,aux,addr,val) APB_REGISTER_WRITE(addr,val)
#define MVD_ADDRESS_READ(pMvdHw,aux,addr,val)  APB_REGISTER_READ(addr,val)

#endif /* MVD_DIAG_LOG_REG_WRITES */

#define APB_REGISTER_POLL(s,p,v,m)
#define APB_SEEK_EVENT(e)
#define APB_REG_SET(a,m,b)  ((a)) = ((((a)) & ~(m)) | ((b) & (m)))

///////////////////////////
// System functions
//

///////////////////////////
// MULT_U(a,b)
//
// a * b

#define MULT_U(a,b) (a)*(b)

///////////////////////////
// LDIV_MOD_U(a,b)
//
// a / b,  mod = a % b

#define LDIV_MOD_U(a,b,mod) (a)/(b), mod = (a)%(b)

///////////////////////////
// MEMORY_BLOCK_COPY(a,b,c,d)
//
// a : int - 1 for load, 0: store
// b : int - size of transfer in bytes
// c : void * - source address
// d : void * - destination address

#define MEMORY_BLOCK_COPY(a,b,c,d) pal_memcpy(d,c,b)

//////////////////////////////////////////////////
// Implementation running defines
//

#define FORCE_DECODE_HANDLE_IRQ   { \
                                    pgCtrlMVDHw->uForceFIQ = 1; \
                                    if ( pgCtrlMVDHw->uMaloneID == 0 ) \
                                    { \
                                      pal_int_set ( uDecLibIrqPin[0x0][0x0] ); \
                                    } \
                                    else \
                                    {  \
                                      pal_int_set ( uDecLibIrqPin[0x1][0x0] ); \
                                    } \
                                  }

#endif /* _MVD_H_ */

/* End of file */
