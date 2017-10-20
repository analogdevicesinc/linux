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
  Filename    :    pal.h
  Description :    Public header file for the
                   Platform Abstraction Layer
  Author:          Media IP FW team (Belfast)

 ***************************************************/

#ifndef _PAL_H_
#define _PAL_H_

/////////////////////////////////////////////////////////////////////////////////
//  Header Files
/////////////////////////////////////////////////////////////////////////////////

#ifndef VPU_KERNEL_BUILD
#include "stdio.h"
#include "video_subsystem.h"
#include "pal_os_al.h"
/* For va_list */
#include <stdarg.h>
#endif
#include "basetype.h"
#include "pal_types.h"

#if ( TARGET_APP == VPU_TEST_APP )
/* thread, semaphore and queue funcitons */
#include "pal_linux_map.h"
#endif

/* For buffer descriptor */
#include "mediaip_fw_types.h"

/* For va_list */
#include <stdarg.h>

/////////////////////////////////////////////////////////////////////////////////
//  Function prototypes
/////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////
// Initialisation Functions
//

#if RTOS == NONE

MEDIAIP_FW_STATUS pal_initialise ( psPALConfig pconfig );

#else

void pal_early_initialise ( void );

MEDIAIP_FW_STATUS pal_initialise ( void );

#endif

//////////////////////////////////////////////////
// Assert function
//
void pal_assert_impl(u_int32 uAssertPC, u_int32 uAssertInfo);

///////////////////////////////////////////////////
// Critical section functions
//

MEDIAIP_FW_STATUS pal_critical_section_begin ( PAL_CRIT_STATE *pState );

MEDIAIP_FW_STATUS pal_critical_section_end ( PAL_CRIT_STATE PreviousState );

///////////////////////////////////////////////////
// Interrupt Functions
//

MEDIAIP_FW_STATUS pal_int_register ( u_int32     dwIntID,
                                     PAL_PFNISR  pfnHandler,
                                     BOOL        bFIQ );

MEDIAIP_FW_STATUS pal_int_enable ( u_int32 dwIntID );

void pal_int_set ( u_int32 dwIntID );

#if  ( TARGET_PLATFORM == GENTB_PLATFORM ) || ( TARGET_PLATFORM == GEN_TB_ENC ) || ( TARGET_PLATFORM == WIN_LIB )

void pal_int_clear ( u_int32 dwIntID,
                     BOOL    bDirect );

MEDIAIP_FW_STATUS pal_int_get_irq_line ( u_int32 uFWIrq,
                                         u_int32 *puIrqLine );

#else

void pal_int_clear ( u_int32 dwIntID );

#endif

void pal_int_clear_host ( u_int32 dwIntID );


///////////////////////////////////////////////////
// Processor Cache Control Functions
//

MEDIAIP_FW_STATUS pal_clean_d_cache ( void );

MEDIAIP_FW_STATUS pal_disable_d_cache ( void );

void              pal_wait_for_interrupt ( void );

///////////////////////////////////////////////////
// Malone Cache Control Functions
//
void pal_set_malone_cache ( u_int32 uMalID );

///////////////////////////////////////////////////
// C Runtime Wrappers
//

MEDIAIP_FW_STATUS pal_memcpy ( void *pDest,
                               const void *pSrc,
                               u_int32 uSize );

void pal_memset ( void *pDest, int32 nChar, u_int32 uCount );

BOOL pal_memcompare ( void *pArea1, void *pArea2, u_int32 uSizeInWords );

///////////////////////////////////////////////////
// Hardware Timer Service APIs
//

MEDIAIP_FW_STATUS pal_timer_create ( PAL_PFNTIMER     pfnCallback,
                                     void *           pUserData,
                                     PAL_TIMER_ID *   pTimer );

MEDIAIP_FW_STATUS pal_timer_destroy( PAL_TIMER_ID Timer );

///////////////////////////////////////////////////
// Perf Counter APIs
//

MEDIAIP_FW_STATUS pal_perf_counter_create ( const char *  pszName,
                                            PAL_PERF_ID * pPCId );

MEDIAIP_FW_STATUS pal_perf_counter_destroy ( PAL_PERF_ID PCId );

MEDIAIP_FW_STATUS pal_perf_counter_start ( PAL_PERF_ID PCId );

MEDIAIP_FW_STATUS pal_perf_counter_stop ( PAL_PERF_ID PCId );

MEDIAIP_FW_STATUS pal_perf_counter_pause_control ( PAL_PERF_ID PCId , bool bStartPause);

MEDIAIP_FW_STATUS pal_perf_counter_read ( PAL_PERF_ID PerfId,
                                          u_int32 *   puCountVal );

///////////////////////////////////////////////////
// Trace / Error / Message log functions
//

/* Error logging */
#if !(DEBUG_CAPS == FULL_DEBUG ) && (ENABLE_TRACE_IN_RELEASE == NO)

/* Non-debug case */
MEDIAIP_FW_STATUS pal_error_log ( u_int32 uError );

#else

/* Debug case - wrap in macros so that we can add file and line number automatically */
#ifdef _MSC_VER
int pal_debug_error_log (
#else
MEDIAIP_FW_STATUS pal_debug_error_log (
#endif
                  u_int32 uError,
                  char    *pszFile,
                  int32   nLineNum );
#ifndef _MSC_VER
#define pal_error_log(x) pal_debug_error_log((x), __FILE__, __LINE__)
#endif // _MSC_VER
#endif

/* Size of trace print buffer        */
#define FW_PRT_BUFF_SIZE       512

#ifdef DISABLE_TRACE
/* Declare pal_trace as an empty statement and cast to void to avoid a "no-effect" warning. */
/* This soaks up the trailing semi-colon and avoids leaving them dangling.                  */
#define pal_trace(...) (void)(0)
#else
#if ( TARGET_APP == VPU_TEST_APP )
#ifdef NXP_MX_REAL_TARGET
#define pal_trace(flags, fmt, arg...) dprintf(LVL_FUNC, fmt, ## arg)
#else
void pal_trace ( u_int32 uFlags, const char *psz_format, ...);
#endif
#else
void pal_trace ( u_int32 uFlags, const char *psz_format, ...);
#endif
#endif

int pal_vsnprintf ( char *str, int size, const char *format, va_list args );

int pal_sprintf ( char *str, int size, const char *psz_format, ...);

#ifdef PAL_DEBUG_LOG
void pal_debug_log ( u_int32 uCode );
#else
#define pal_debug_log(...) (void)(0)
#endif

MEDIAIP_FW_STATUS pal_trace_set_level ( u_int32               uLevel,
                                        BOOL                  bTimestamp,
                                        MEDIAIP_TRACE_FLAGS * pFlags );

MEDIAIP_FW_STATUS pal_trace_set_module_flag ( u_int32               uModuleID,
                                              BOOL                  bEnable,
                                              MEDIAIP_TRACE_FLAGS * pFlags );

MEDIAIP_FW_STATUS pal_trace_is_module_enabled ( u_int32  uModuleID,
                                                BOOL   * pbEnabled );

void pal_checkpoint_str(char *pMsg );
void pal_checkpoint_hex(unsigned uData);
#define CHECKPOINT_STR pal_checkpoint_str
#define CHECKPOINT_HEX pal_checkpoint_hex


///////////////////////////////////////////////////
// Clock functions - often very platform specific
//

#ifdef PAL_CLOCK_API

MEDIAIP_FW_STATUS pal_malone_clock_reg_init ( void );

MEDIAIP_FW_STATUS pal_malone_clock_enable_common ( bool bEnable );

MEDIAIP_FW_STATUS pal_malone_clock_enable_avc ( bool bEnable );

MEDIAIP_FW_STATUS pal_malone_clock_enable_vc1 ( bool bEnable );

MEDIAIP_FW_STATUS pal_malone_clock_enable_mpg ( bool bEnable );

MEDIAIP_FW_STATUS pal_malone_clock_enable_avs ( bool bEnable );

#endif /* PAL_CLOCK_API */

///////////////////////////////////////////////////
// Miscellaneous functions
//

void pal_fatal_exit_internal ( u_int32 uCosmicConstant,
                               char  * pszFilename,
                               int     iLineNum );

//#define pal_fatal_exit(x)    pal_fatal_exit_internal((x), __FILE__, __LINE__)
#ifdef VPU_KERNEL_BUILD
#define pal_fatal_exit(x)    while (1) {printk("pal_fatal_exit in %s file %s line %d\n", __FUNCTION__, __FILE__, __LINE__);}
#else
#define pal_fatal_exit(x)    while (1) {printf("pal_fatal_exit in %s file %s line %d\n", __FUNCTION__, __FILE__, __LINE__);}
#endif



u_int32 pal_find_highest_bit ( u_int32 uValue );

extern u_int32 _return_pc ( void );

#define pal_return_pc _return_pc

u_int32 pal_get_fw_base ( void );

u_int32 pal_get_target_version ( void );

///////////////////////////////////////////////////
// Memory management abstraction functions
//
#if ( TARGET_APP == VPU_TEST_APP )
/* sPALMemDesc Added by NXP for their PAL implementation */
typedef struct {
	u_int32 size;
	u_int32 phy_addr;
	uint_addr virt_addr;
#ifdef USE_ION
	int32 ion_buf_fd;
#endif
} sPALMemDesc, *psPALMemDesc;

MEDIAIP_FW_STATUS pal_get_phy_buf(psPALMemDesc pbuf);
MEDIAIP_FW_STATUS pal_free_phy_buf(psPALMemDesc pbuf);
#endif

u_int32 pal_va2pa ( u_int32 * pAddr );

u_int32 * pal_return_uncached_addr ( u_int32 * puAddress );

u_int32 * pal_return_cacheable_addr ( u_int32 * puAddress );

u_int32 * pal_return_mmu_bypass_addr ( u_int32 * puAddress );

u_int32 pal_read_uncached ( u_int32 * puAddress );

#if RTOS != NONE
/* Need to add in the prototypes for the PAL ftns which invoke an OS */
/* Abstraction layer call                                            */

#include "pal_os_al.h"

#endif

///////////////////////////////////////////////////
// Miscellaneous macros
//

#ifdef PERF_MEASURE_ENABLE_ENC
u_int32 GetCountVal();
u_int32 SetCountVal(u_int32 CountVal);
#endif

#if ( TARGET_APP == VPU_TEST_APP )
#define INT_ID_MALONE_LOW 0
#define INT_ID_MALONE_HI 1
#define INT_ID_MAX 2
extern PAL_PFNISR int_handlers[INT_ID_MAX];
#endif

#endif /* _PAL_H_ */

/* End of File */
