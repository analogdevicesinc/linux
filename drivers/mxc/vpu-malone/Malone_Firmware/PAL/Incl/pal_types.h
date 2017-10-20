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

  Filename:        pal_types.h
  Description:     Public header file for PAL type definitions
                   Common between all platforms
  Author:          Media IP FW team (Belfast)

 ****************************************************/

#ifndef _PAL_TYPES_H_
#define _PAL_TYPES_H_

/////////////////////////////////////////////////////////////////////////////////
//  Header Files
/////////////////////////////////////////////////////////////////////////////////

#include "basetype.h"
#include "status_codes.h"
#include "trace_types.h"

/////////////////////////////////////////////////////////////////////////////////
//  Macros
/////////////////////////////////////////////////////////////////////////////////

// Define the magic cookie
#define PAL_CONFIG_MAGIC        0x434C4150      // "PALC", little endian

// Values for pal_trace_destination
#define PAL_TRACE_TO_DEVNULL      0
#define PAL_TRACE_TO_UART         1
#define PAL_TRACE_TO_MESSAGE      2
#define PAL_TRACE_TO_CIRCULARBUF  3

#if ( TARGET_PLATFORM == GENTB_PLATFORM ) || ( TARGET_PLATFORM == WIN_LIB ) || ( TARGET_PLATFORM == GEN_TB_ENC )

#define PAL_CONFIG_MAX_IRQS          0x12
#define PAL_CONFIG_MAX_MALONES       0x2
#define PAL_CONFIG_MAX_WINDSORS      0x1
#define PAL_CONFIG_MAX_TIMER_IRQS    0x4
#define PAL_CONFIG_MAX_TIMER_SLOTS   0x4

/* Define the entry locations in the irq vector */
#define PAL_IRQ_MALONE0_LOW      0x0
#define PAL_IRQ_MALONE0_HI       0x1
#define PAL_IRQ_MALONE1_LOW      0x2
#define PAL_IRQ_MALONE1_HI       0x3
#define PAL_IRQ_WINDSOR_LOW      0x4
#define PAL_IRQ_WINDSOR_HI       0x5
#define PAL_IRQ_HOST_CMD_LO      0x6
#define PAL_IRQ_HOST_CMD_HI      0x7
#define PAL_IRQ_HOST_MSG         0x9
#define PAL_IRQ_DPV              0xA
#define PAL_IRQ_TIMER_0          0xE
#define PAL_IRQ_TIMER_1          0xF
#define PAL_IRQ_TIMER_2          0x10
#define PAL_IRQ_TIMER_3          0x11

#else

#define PAL_CONFIG_MAX_INITS    4               // Number of init slots
#define PAL_CONFIG_MAX_IRQS     2               // Number of incoming irq lines supported

#endif /* TARGET_PLATFORM == TB_PLATFORM */

/////////////////////////////////////////////////////////////////////////////////
//  Structure definitions
/////////////////////////////////////////////////////////////////////////////////

#if OSAL == NO_AL
/* Function pointer types */
typedef u_int32  PAL_TIMER_ID;
typedef u_int32  PAL_CRIT_STATE;

typedef MEDIAIP_IRQ_RETCODE (*PAL_PFNISR)(u_int32);
typedef void                (*PAL_PFNTIMER)(PAL_TIMER_ID, void *);

#endif

typedef u_int32  PAL_PERF_ID;

/////////////////////////////////////////////////////////////////////////////////
//  PAL Configuration structure

#if ( TARGET_PLATFORM == GENTB_PLATFORM ) || ( TARGET_PLATFORM == WIN_LIB ) || ( TARGET_PLATFORM == GEN_TB_ENC )

typedef struct _PALConfig
{
  u_int32             uPalConfigMagicCookie;

  u_int32             uGICBaseAddr;
  u_int32             uIrqLines[PAL_CONFIG_MAX_IRQS];
  u_int32             uIrqTarget[PAL_CONFIG_MAX_IRQS];

  u_int32             uUartBaseAddr;

  u_int32             uSysClkFreq;
  u_int32             uNumTimers;
  u_int32             uTimerBaseAddr;
  u_int32             uTimerSlots[PAL_CONFIG_MAX_TIMER_SLOTS];

  /* Do we need this in the PAL config? Only for checking mmu setup  */
  /* perhaps - otherwise its more naturtal home is in the DECLIB_CFG */
  /* structure                                                       */
  u_int32             uNumMalones;
  u_int32             uMaloneBaseAddr[PAL_CONFIG_MAX_MALONES];
  u_int32             uHifOffset[PAL_CONFIG_MAX_MALONES];

  u_int32             uNumWindsors;
  u_int32             uWindsorBaseAddr[PAL_CONFIG_MAX_WINDSORS];

  u_int32             uDPVBaseAddr;
  u_int32             uPixIfAddr;

  u_int32             pal_trace_level;
//  u_int32             pal_trace_destination;
//  u_int32             pal_trace_CBDescAddr[3];		// 3 separate circular buffers for PAL_TRACE_TO_CIRCULARBUF
   						                                    // 0: normal  1: irq  2: fiq
  u_int32             uHeapBase;
  u_int32             uHeapSize;

  u_int32             uFSLCacheBaseAddr;

} sPALConfig, *psPALConfig;

#else

typedef struct _PALConfig
{
  u_int32             pal_config_magic_cookie;
  u_int32             cmd_irq_line[PAL_CONFIG_MAX_IRQS];
  u_int32             cmd_irq_clear_addr[PAL_CONFIG_MAX_INITS];
  u_int32             cmd_irq_clear_mask[PAL_CONFIG_MAX_INITS];
  u_int32             cmd_irq_clear_val[PAL_CONFIG_MAX_INITS];
  u_int32             msg_irq_init_addr[PAL_CONFIG_MAX_INITS];
  u_int32             msg_irq_init_mask[PAL_CONFIG_MAX_INITS];
  u_int32             msg_irq_init_val[PAL_CONFIG_MAX_INITS];
  u_int32             msg_irq_raise_addr;
  u_int32             msg_irq_raise_mask;
  u_int32             msg_irq_raise_val;
  u_int32             uart_init_addr[PAL_CONFIG_MAX_INITS];
  u_int32             uart_init_mask[PAL_CONFIG_MAX_INITS];
  u_int32             uart_init_val[PAL_CONFIG_MAX_INITS];
  u_int32             uart_check_addr;
  u_int32             uart_check_mask;
  u_int32             uart_check_val;
  u_int32             uart_put_addr;
  u_int32             pal_trace_level;
  u_int32             pal_trace_destination;
  MEDIAIP_TRACE_FLAGS pal_trace_flags;         // Currently 5 words
  u_int32             pal_trace_CBDescAddr[3];		// 3 separate circular buffers for PAL_TRACE_TO_CIRCULARBUF
   						                                    // 0: normal  1: irq  2: fiq
} sPALConfig, *psPALConfig;

#endif /*  TARGET_PLATFORM == TB_PLATFORM */


#endif /* _PAL_TYPES_H_ */


/* End of File */
