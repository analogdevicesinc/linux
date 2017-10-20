/***********************************************
 * Copyright (c) 2015 Amphion Semiconductor Ltd *
 ***********************************************
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 ****************************************************
 *
 * Filename:        pal_linux_map.h
 * Description:     Maps the abstraction layer type from the
 *                  PAL to Linux defines, though a variation of CNXT KAL 
 *						for implementing NXP PAL implementaiton layer
 * Author:          Media IP FW team (Belfast)
 *
 ******************************************************************************
 * $Id: 
 ******************************************************************************/

/* Note : At the moment I have done no real comparison and merge of the 
          different abstarction layers supported in the PAL layer so this
          file is simply a direct map - currently its only ftn is to 
          remove compiler warnings and to get the code structure correct for
          the future
*/

#ifndef _PAL_LINUX_MAP_H_
#define _PAL_LINUX_MAP_H_

#include "status_codes.h"

/*******************/
/* Resource Limits */
/*******************/

#if 0
/* Maximum length of an OS object name string */
#define PAL_MAX_OBJ_NAME_LENGTH   CNXT_KAL_MAX_OBJ_NAME_LENGTH
#endif

#define PAL_NO_WAIT               0
#define PAL_WAIT_FOREVER          ((u_int32)-1)

#if 0
/* Macro to assign task priority based on RTOS */
#define PAL_THREAD_PRIO(Prio, UCOS_Prio) CNXT_KAL_THREAD_PRIO(Prio, UCOS_Prio)
         
/* Task Priority Limits. */
#define PAL_DEFAULT_PRIORITY    CNXT_KAL_DEFAULT_PRIORITY 
#define PAL_MAX_THREAD_PRIORITY CNXT_KAL_MAX_THREAD_PRIORITY
#define PAL_MIN_THREAD_PRIORITY CNXT_KAL_MIN_THREAD_PRIORITY
#endif

/* Object identifiers, KAL originated */
typedef u_int32        CNXT_QUEUE_ID;
typedef u_int32        CNXT_THREAD_ID;
typedef u_int32        CNXT_SEM_ID;
typedef u_int32        CNXT_MUTEX_ID;
typedef u_int32        CNXT_POOL_ID;
typedef u_int16        CNXT_EVENTS;
typedef u_int32        CNXT_TICK_ID;
typedef u_int32        CNXT_TIMER_ID;
typedef bool           CNXT_CRIT_STATE;

#define  PAL_QUEUE_ID   CNXT_QUEUE_ID    
#define  PAL_THREAD_ID  CNXT_THREAD_ID 
#define  PAL_SEM_ID     CNXT_SEM_ID    
#define  PAL_POOL_ID    CNXT_POOL_ID   
#define  PAL_EVENTS     CNXT_EVENTS    
#define  PAL_TICK_ID    CNXT_TICK_ID   
#define  PAL_TIMER_ID   CNXT_TIMER_ID  
#define  PAL_CRIT_STATE CNXT_CRIT_STATE


typedef enum
{
  PAL_CB_LOW_PRIORITY,
  PAL_CB_LOW_PRIORITY_NO_BLOCK,
  PAL_CB_HIGH_PRIORITY,
  PAL_CB_HIGH_PRIORITY_NO_BLOCK,
  PAL_CB_PRIORITY_LAST = PAL_CB_HIGH_PRIORITY_NO_BLOCK
} PAL_CB_PRIORITY;

#define PAL_PFNTHREAD         PFNTHREAD
#define PAL_PFNISR            PFNISR
#define PAL_PFNTHREADCALLBACK PFNTHREADCALLBACK
typedef void             (*PFNTHREAD)(int, void **);
typedef MEDIAIP_FW_STATUS (*PFNISR)(u_int32);
typedef void             (*PFNTHREADCALLBACK)(u_int32, u_int32, void *);





/*****************************************************************/
/** PAL functions exporting OS abstraction layer functionality  **/
/*****************************************************************/

MEDIAIP_FW_STATUS pal_thread_create ( PAL_PFNTHREAD  pfnEntryPoint,
                                      int            nArgC,
                                      void           **ppArgV,
                                      u_int32        uStackSize,
                                      u_int8         uPrio,
                                      const char     *pszName,
                                      PAL_THREAD_ID  *pId );

MEDIAIP_FW_STATUS pal_thread_terminate ( PAL_THREAD_ID *pId );

MEDIAIP_FW_STATUS pal_make_async_thread_callback (
                                                   PAL_PFNTHREADCALLBACK pfnCallback, 
                                                   PAL_CB_PRIORITY Priority, 
                                                   u_int32 uParam1, 
                                                   u_int32 uParam2, 
                                                   void *pData );
                                                    

                                                    

////////////////////////////////////////////////////////////////////////////////
// Semaphore functions
////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS pal_sem_create (  u_int32 uInitialValue,
		const char *pszName,
		PAL_SEM_ID *pSem);                                                    
                                                    
////////////////////////////////////////////////////////////////////////////////
// Queue functions
////////////////////////////////////////////////////////////////////////////////

MEDIAIP_FW_STATUS pal_qu_create ( unsigned int nMaxElements, 
                                  const char *pszName, 
                                  PAL_QUEUE_ID *pQuId );

MEDIAIP_FW_STATUS pal_qu_destroy ( PAL_QUEUE_ID QuId );

MEDIAIP_FW_STATUS pal_qu_send ( PAL_QUEUE_ID QuId, 
                                void         *pMessage );

MEDIAIP_FW_STATUS pal_qu_receive ( PAL_QUEUE_ID QuId, 
                                   u_int32      uTimeoutMs, 
                                   void         *pMessage );


#endif /* _PAL_CNXT_KAL_MAP_H_ */

/* End of File */
