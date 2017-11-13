/***************************************************
 *   Copyright (c) 2015 Amphion Semiconductor Ltd
 *   All rights reserved.
 ****************************************************
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 ****************************************************
 *
 *   Filename    :    pal.c
 *   Description :    Code implementing Platform Abstraction Layer
 *         
 *   Author      :    Media IP FW team (Belfast)
 *
 ****************************************************/

/////////////////////////////////////////////////////////////////////////////////
////  Header Files
///////////////////////////////////////////////////////////////////////////////////

#ifndef VPU_KERNEL_BUILD
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/errno.h>
#include "VPU_io.h"
#else
#undef ARM
#undef SUCCESS
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>

#endif
#include "pal.h"
#include "VPU_regdef.h"
#include "VPU_debug.h"
#include "mvd.h"


///////////////////////////////////////////////////////////////////////////////////
////  External Function Prototypes
///////////////////////////////////////////////////////////////////////////////////

// The function declared in pal.h is set to be empty 
// and need to be defined later

//volatile u_int32    guPlayerFlag[MEDIA_PLAYER_NUM_STREAMS];

#ifdef VPU_KERNEL_BUILD
extern void __iomem *vpu_base;
#define VPU_REG_WR(reg, val) writel(val, vpu_base + reg)
#define VPU_REG_RD(reg) readl(vpu_base + reg)
#else
#define VPU_REG_WR(reg, val) VpuWriteReg(reg, val)
#define VPU_REG_RD(reg) VpuReadReg(reg)
#endif

PAL_PFNISR int_handlers[INT_ID_MAX];

#ifdef ENABLE_CRIT_SECTIONS
static int ulCriticalNesting = 0;
#endif

/////////////////////////////////////////////////////////////////////////////////////
// Assert function, implementation to be defined but we want to use this to trap logical errors
//
bool gbAssertExit = FALSE;
void pal_assert_impl(u_int32 uAssertPC, u_int32 uAssertInfo)
{
	/* AssertInfo could be the offset address of the calling function or something else? */
	while(gbAssertExit==FALSE)
	{
		/* wait to allow debug */
	}
}


MEDIAIP_FW_STATUS pal_critical_section_begin ( PAL_CRIT_STATE *pState )
{
#ifdef ENABLE_CRIT_SECTIONS
	//ENTER_FUNC();
	if(ulCriticalNesting>0)
	  dprintf(LVL_FUNC, "Nested %d\n", ulCriticalNesting);
	// FIXME: might need MFD_HIF_MSD_REG_HOST_INTERRUPT_ENABLE 0x1000
	*pState = VPU_REG_RD((DEC_MFD_XREG_SLV_BASE + MFD_HIF + MFD_HIF_MSD_REG_FAST_INTERRUPT_ENABLE));
	VPU_REG_WR((DEC_MFD_XREG_SLV_BASE + MFD_HIF + MFD_HIF_MSD_REG_FAST_INTERRUPT_ENABLE), *pState & ~0x20);
	//dprintf(LVL_FUNC, "save to State 0x%lx, expect 0x20 outside irq or 0 inside irq\n", *pState);
	ulCriticalNesting++;
#endif

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_critical_section_end ( PAL_CRIT_STATE PreviousState )
{
#ifdef ENABLE_CRIT_SECTIONS
	ulCriticalNesting--;
	//ENTER_FUNC();
	//dprintf(LVL_FUNC, "Nest %d\n", ulCriticalNesting);
	//dprintf(LVL_FUNC, "restore from State 0x%lx, expect 0x20 outside irq or 0 inside irq\n", PreviousState);
	VPU_REG_WR((DEC_MFD_XREG_SLV_BASE + MFD_HIF + MFD_HIF_MSD_REG_FAST_INTERRUPT_ENABLE), PreviousState);
#endif  
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_int_register ( u_int32     dwIntID,
		PAL_PFNISR  pfnHandler,
		BOOL        bFIQ )
{
	ENTER_FUNC();
	if (pfnHandler == NULL)
		return MEDIAIP_FW_STATUS_BAD_PARAMETER;

	if (dwIntID == PAL_IRQ_MALONE0_LOW) {
		int_handlers[INT_ID_MALONE_LOW] = pfnHandler;
	} else if (dwIntID == PAL_IRQ_MALONE0_HI) {
		int_handlers[INT_ID_MALONE_HI] = pfnHandler;
	} else {
		err_msg("wrong dwIntID 0x%x\n", dwIntID);
		return MEDIAIP_FW_STATUS_INT_NOT_HANDLED;
	}
	dprintf(LVL_FUNC, "pal pfnHandler 0x%p\n", pfnHandler);

	return MEDIAIP_FW_STATUS_OK;
}


MEDIAIP_FW_STATUS pal_int_enable ( u_int32 dwIntID )
{
	ENTER_FUNC();
	return MEDIAIP_FW_STATUS_OK;
}

void pal_int_set ( u_int32 dwIntID )
{
	ENTER_FUNC();
	dprintf(LVL_FUNC, "dwIntID %d\n", dwIntID);

	if (dwIntID != PAL_IRQ_MALONE0_LOW) {
		err_msg("ERROR: not PAL_IRQ_MALONE0_LOW!!!\n");
		EXIT_FUNC();
		return;
	}

	VPU_REG_WR((DEC_MFD_XREG_SLV_BASE + MFD_SIF + MFD_SIF_INTR_FORCE), 0x100);

	EXIT_FUNC();
}

void pal_int_clear ( u_int32 dwIntID,
		BOOL    bDirect )
{
	ENTER_FUNC();
}

MEDIAIP_FW_STATUS pal_int_get_irq_line ( u_int32 uFWIrq,
		u_int32 *puIrqLine )
{
	ENTER_FUNC();
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_memcpy ( void *pDest,
		const void *pSrc,
		u_int32 uSize )
{
	ENTER_FUNC();
	if (pDest == NULL || pSrc == NULL)
		return MEDIAIP_FW_STATUS_BAD_PARAMETER;

	memcpy(pDest, pSrc, uSize);
	return MEDIAIP_FW_STATUS_OK;
}

void pal_memset ( void *pDest, int32 nChar, u_int32 uCount )
{
	ENTER_FUNC();
	dprintf(LVL_FUNC, "pDest 0x%p, nChar %d, uCount %d\n", pDest, nChar, uCount);
	if (pDest == NULL)
		return;

	memset(pDest, nChar, uCount);
}

BOOL pal_memcompare ( void *pArea1, void *pArea2, u_int32 uSizeInWords )
{
	u_int32 i;
	u_int32 *ptr0, *ptr1;
	u_int32 uChange;

	ENTER_FUNC();
	if (pArea1 == NULL || pArea2 == NULL)
		return FALSE;

	ptr0 = ( u_int32 * ) pArea1;
	ptr1 = ( u_int32 * ) pArea2;

	for ( i = 0, uChange = 0; ( i < uSizeInWords ) && ( uChange == 0 ); i++ )
	{
		if ( ptr0[i] != ptr1[i] )
		{
			uChange = 1;
		}
	}

	return ( uChange ) ? TRUE : FALSE;
}

MEDIAIP_FW_STATUS pal_timer_create ( PAL_PFNTIMER     pfnCallback,
		void *           pUserData,
		PAL_TIMER_ID *   pTimer )
{
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_timer_destroy( PAL_TIMER_ID Timer )
{
	return MEDIAIP_FW_STATUS_OK;
}

////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    pal_memalloc                                                 //
//                                                                            //
//  DESCRIPTION:                                                              //
//     allocates size bytes and returns a pointer to the allocated  memory.   //
//     The memory is not cleared.                                             //
//                                                                            //
//  INPUT PARAMETERS:                                                         //
//     uSize      - Size of memory in bytes to alloc                          //
//                                                                            //
//  OUTPUT PARAMETERS:                                                        //
//                                                                            //
//  RETURN VALUES:                                                            //
//                                                                            //
//  NOTES:                                                                    //
//                                                                            //
//  CONTEXT:                                                                  //
//     This function may be called from any context                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
#ifndef VPU_KERNEL_BUILD

void * pal_memalloc ( size_t uSize )
{
  void * pPtr = malloc ( uSize );
  
  return pPtr;
}
#endif
////////////////////////////////////////////////////////////////////////////////
//  FUNCTION:    pal_memfree                                                  //
//                                                                            //
//  DESCRIPTION:                                                              //
//     frees the memory space pointed to by ptr, which must have been         //
//     returned by a previous call to malloc(), calloc() or realloc().        // 
//     Otherwise, or if free(ptr) has already  been called before, undefined  //
//     behaviour occurs. If ptr is NULL, no operation is performed.           //
//                                                                            //
//                                                                            //
//  INPUT PARAMETERS:                                                         //
//     pPtr      - Pointer to memory to free                                  //
//                                                                            //
//  OUTPUT PARAMETERS:                                                        //
//                                                                            //
//  RETURN VALUES:                                                            //
//                                                                            //
//  NOTES:                                                                    //
//                                                                            //
//  CONTEXT:                                                                  //
//     This function may be called from any context                           //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////
#ifndef VPU_KERNEL_BUILD

void pal_memfree ( void * pPtr )
{
  free ( pPtr );
}
#endif
u_int32 pal_find_highest_bit ( u_int32 uValue )
{
	u_int32 mask = 0x80000000;
	u_int32 ret = 31;

	ENTER_FUNC();

	while (mask && ((uValue & mask) == 0)) {
		mask >>= 1;
		ret--;
	}

	return ret;
}

MEDIAIP_FW_STATUS pal_perf_counter_create ( const char *  pszName,
		PAL_PERF_ID * pPCId )
{
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_perf_counter_destroy ( PAL_PERF_ID PCId )
{
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_perf_counter_start ( PAL_PERF_ID PCId )
{
	return MEDIAIP_FW_STATUS_OK;
}


MEDIAIP_FW_STATUS pal_perf_counter_pause_control ( PAL_PERF_ID PCId , bool bStartPause)
{
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_perf_counter_read ( PAL_PERF_ID PerfId,
		u_int32 *   puCountVal )
{
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_malone_clock_reg_init ( void )
{
	return MEDIAIP_FW_STATUS_OK;
}

static void mfd_clock_enable(unsigned int mask, bool bEnable)
{
	if (bEnable)
		VPU_REG_WR((DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_SET), mask);
	else
		VPU_REG_WR((DEC_MFD_XREG_SLV_BASE + MFD_BLK_CTRL + MFD_BLK_CTRL_MFD_SYS_CLOCK_ENABLE_CLR), mask);
}

MEDIAIP_FW_STATUS pal_malone_clock_enable_common ( bool bEnable )
{
	ENTER_FUNC();
	mfd_clock_enable(0x10, bEnable);
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_malone_clock_enable_avc ( bool bEnable )
{
	ENTER_FUNC();
	mfd_clock_enable(0x1, bEnable);
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_malone_clock_enable_vc1 ( bool bEnable )
{
	ENTER_FUNC();
	mfd_clock_enable(0x2, bEnable);
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_malone_clock_enable_mpg ( bool bEnable )
{
	ENTER_FUNC();
	mfd_clock_enable(0x4, bEnable);
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_malone_clock_enable_avs ( bool bEnable )
{
	ENTER_FUNC();
	mfd_clock_enable(0x8, bEnable);
	return MEDIAIP_FW_STATUS_OK;
}

u_int32 pal_get_target_version ( void )
{
	return 0;
}

#ifndef VPU_KERNEL_BUILD
MEDIAIP_FW_STATUS pal_get_phy_buf(psPALMemDesc pbuf)
{
	vpu_mem_desc mem_desc = {0};
	int ret;

	ENTER_FUNC();

	mem_desc.size = pbuf->size;
	ret = IOGetPhyMem(&mem_desc);
	if (ret) {
		err_msg("Unable to obtain physical mem\n");
		return MEDIAIP_FW_STATUS_RESOURCE_ERROR;
	}

	if (IOGetVirtMem(&mem_desc) == -1) {
		err_msg("Unable to obtain virtual mem\n");
		IOFreePhyMem(&mem_desc);
		return MEDIAIP_FW_STATUS_RESOURCE_ERROR;
	}

	pbuf->phy_addr = mem_desc.phy_addr;
	pbuf->virt_addr = mem_desc.virt_uaddr;
#ifdef USE_ION
	pbuf->ion_buf_fd = mem_desc.ion_buf_fd;
#endif

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_free_phy_buf(psPALMemDesc pbuf)
{
	vpu_mem_desc mem_desc = {0};

	ENTER_FUNC();

	mem_desc.size = pbuf->size;
	mem_desc.phy_addr = pbuf->phy_addr;
	mem_desc.virt_uaddr = pbuf->virt_addr;
#ifdef USE_ION
	mem_desc.ion_buf_fd = pbuf->ion_buf_fd;
#endif

	IOFreeVirtMem(&mem_desc);
	IOFreePhyMem(&mem_desc);

	return MEDIAIP_FW_STATUS_OK;
}
#endif

u_int32 pal_va2pa ( u_int32 * pAddr )
{
	/* CAUTION: pAddr shall be physical address already*/
	dprintf(LVL_FUNC, "pAddr 0x%p shall be physical!!!\n", pAddr);
	return (u_int32)(uint_addr)pAddr;
}

u_int32 * pal_return_uncached_addr ( u_int32 * puAddress )
{
	dprintf(LVL_FUNC, "puAddress 0x%p\n", puAddress);
	return puAddress;
}

u_int32 * pal_return_cacheable_addr ( u_int32 * puAddress )
{
	dprintf(LVL_FUNC, "puAddress 0x%p\n", puAddress);
	return puAddress;
}

u_int32 pal_read_uncached ( u_int32 * puAddress )
{
	dprintf(LVL_FUNC, "puAddress 0x%p\n", puAddress);

	if (puAddress == NULL)
		return MEDIAIP_FW_STATUS_BAD_PARAMETER;

	return (*puAddress);
}

int pal_vsnprintf ( char *str, int size, const char *format, va_list args )
{
	ENTER_FUNC();
	return 0;
}

int pal_sprintf ( char *str, int size, const char *psz_format, ...)
{
	ENTER_FUNC();
	return 0;
}

static void mfd_cache_clock_enable(unsigned int enable)
{
	ENTER_FUNC();
	VPU_REG_WR((SCB_XREG_SLV_BASE + SCB_SCB_BLK_CTRL + SCB_BLK_CTRL_SCB_CLK_ENABLE_SET),enable);
}

void pal_set_malone_cache ( u_int32 uMalID )
{
	ENTER_FUNC();
	mfd_cache_clock_enable(0xE);
}

#define MAX_QUEUE_NUM 10
static PAL_QUEUE_ID gQuId = 0;

MEDIAIP_FW_STATUS pal_sem_create (  u_int32 uInitialValue,
		const char *pszName,
		PAL_SEM_ID *pSem)
{
	/* CAUTION: if in use */
	ENTER_FUNC();
	return MEDIAIP_FW_STATUS_OK;
}

#ifdef VPU_KERNEL_BUILD

static struct kfifo irq_fifo[MAX_QUEUE_NUM];
static spinlock_t irq_lock[MAX_QUEUE_NUM];
static struct task_struct *msg_thread;
static wait_queue_head_t irq_wq[MAX_QUEUE_NUM];

MEDIAIP_FW_STATUS pal_thread_create ( PAL_PFNTHREAD  pfnEntryPoint,
		int        nArgC,
		void       **ppArgV,
		u_int32    uStackSize,
		u_int8     uPrio,
		const char *pszName,
		PAL_THREAD_ID  *pId )
{
	typedef int (*INTFUNC)(void *);

	/* CAUTION: shall not enter twice due to global msg_thread */
	ENTER_FUNC();
	//struct task_struct *msg_thread;
	msg_thread = kthread_run((INTFUNC)pfnEntryPoint , NULL, pszName);
	if(IS_ERR( msg_thread ))
		return MEDIAIP_FW_STATUS_FAILURE;

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_thread_terminate ( PAL_THREAD_ID *pId )
{
	ENTER_FUNC();

	kthread_stop(msg_thread);

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_create ( unsigned int nMaxElements,
		const char *pszName,
		PAL_QUEUE_ID *pQuId )
{
	/* CAUTION: message length shall be 4 * sizeof(uint_addr) */
	ENTER_FUNC();
	*pQuId = gQuId;
	spin_lock_init(&irq_lock[*pQuId]);
	init_waitqueue_head(&irq_wq[*pQuId]);
	if(kfifo_alloc(&irq_fifo[*pQuId],
				nMaxElements * 4 * sizeof(uint_addr),
				GFP_KERNEL))
	{
		err_msg("fail to alloc fifo in pal\n");
		return MEDIAIP_FW_STATUS_FAILURE;
	}

	gQuId++;
	if( MAX_QUEUE_NUM == gQuId)
		gQuId=0;
	dprintf(LVL_FUNC, "create QuId:%d\n",
			*pQuId
			);
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_destroy ( PAL_QUEUE_ID QuId )
{
	ENTER_FUNC();
	dprintf(LVL_FUNC, "destory QuId:%d\n",
			QuId
			);
	kfifo_free(&irq_fifo[QuId]);

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_send ( PAL_QUEUE_ID QuId,
		void         *pMessage)
{
	u_int32 retval;
	ENTER_FUNC();
	dprintf(LVL_FUNC, "QuId %d\n", QuId);

	/* CAUTION: if message is not 4 * sizeof(uint_addr) */
	retval = kfifo_in_locked(&irq_fifo[QuId], pMessage, 4 * sizeof(uint_addr),&irq_lock[QuId]);
	dprintf(LVL_FUNC, "message send: 0x%llx, 0x%llx, 0x%llx, 0x%llx\n", *(uint_addr *)pMessage, *((uint_addr *)pMessage+1), *((uint_addr *)pMessage+2), *((uint_addr *)pMessage+3));
	if(retval != 4*sizeof(uint_addr))
		return MEDIAIP_FW_STATUS_FAILURE;
	wake_up(&irq_wq[QuId]);

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_receive ( PAL_QUEUE_ID QuId,
		u_int32      uTimeoutMs,
		void         *pMessage )
{
	u_int32 retval;
	long ret;

	ENTER_FUNC();
	dprintf(LVL_FUNC, "QuId %d\n", QuId);
/*	while (kfifo_len(&irq_fifo) < sizeof(*pMessage)) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}*/
	ret = wait_event_interruptible_timeout(irq_wq[QuId],
			kfifo_len(&irq_fifo[QuId])>=4*sizeof(uint_addr)
			/* || kthread_should_stop() */,
			//uTimeoutMs
			msecs_to_jiffies(uTimeoutMs)
			);

	if (ret == 0) {
		dprintf(LVL_FUNC, "timeout %d ms\n", uTimeoutMs);
		return MEDIAIP_FW_STATUS_TIMEOUT;
	}	else if (ret == -ERESTARTSYS) {
		dprintf(LVL_FUNC, "ERROR interrupted by a signal!!\n");
		return MEDIAIP_FW_STATUS_FAILURE;
	}	else {
		dprintf(LVL_FUNC, "%ld returned from wait event\n", ret);
	}

	//if(kthread_should_stop())
	//	return MEDIAIP_FW_STATUS_STOPPED;

	if (kfifo_len(&irq_fifo[QuId])>=4*sizeof(uint_addr))
	{
		retval = kfifo_out_locked(&irq_fifo[QuId], pMessage, 4*sizeof(uint_addr),&irq_lock[QuId]);
		dprintf(LVL_FUNC, "message receive: 0x%llx, 0x%llx, 0x%llx, 0x%llx\n", *(uint_addr *)pMessage, *((uint_addr *)pMessage+1), *((uint_addr *)pMessage+2), *((uint_addr *)pMessage+3));
	}
	else
	{
		dprintf(LVL_FUNC, "ERROR interrupted by a signal!!\n");
	}

	return MEDIAIP_FW_STATUS_OK;
}

#else

#define FN_MSG_FIFO "/dev/shm/vpu_msg_fifo"
static char fn_fifo[MAX_QUEUE_NUM][64] = {0};
static int fd_send[MAX_QUEUE_NUM] = {0};
static int fd_receive[MAX_QUEUE_NUM] = {0};

MEDIAIP_FW_STATUS pal_thread_create ( PAL_PFNTHREAD  pfnEntryPoint,
		int        nArgC,
		void       **ppArgV,
		u_int32    uStackSize,
		u_int8     uPrio,
		const char *pszName,
		PAL_THREAD_ID  *pId )
{
	int err;
	pthread_t msg_tid;

	ENTER_FUNC();
	err = pthread_create(&msg_tid, NULL, (void *)pfnEntryPoint, NULL);
	if (err)
		return MEDIAIP_FW_STATUS_FAILURE;

	*pId = msg_tid;
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_thread_terminate ( PAL_THREAD_ID *pId )
{
	pthread_t msg_tid = *pId;

	ENTER_FUNC();

	if (msg_tid == 0)
		return MEDIAIP_FW_STATUS_BAD_PARAMETER;

	pthread_cancel(msg_tid);
	pthread_join(msg_tid, NULL);

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_create ( unsigned int nMaxElements,
		const char *pszName,
		PAL_QUEUE_ID *pQuId )
{
	/* CAUTION: message length shall be 4 * sizeof(uint_addr) */
	ENTER_FUNC();
	*pQuId = gQuId;
	sprintf(fn_fifo[gQuId], "%s%d", FN_MSG_FIFO, gQuId);
	if(access(fn_fifo[gQuId], F_OK) == -1)
	{
		if(mkfifo(fn_fifo[gQuId], 0777))
		{
			err_msg("failed to alloc fifo %s in pal\n", fn_fifo[gQuId]);
			return MEDIAIP_FW_STATUS_FAILURE;
		}
		dprintf(LVL_FUNC, "created fifo %s\n", fn_fifo[gQuId]);
	} else {
		dprintf(LVL_FUNC, "exist fifo %s\n", fn_fifo[gQuId]);
	}

	gQuId++;
	if( MAX_QUEUE_NUM == gQuId)
		gQuId=0;
	dprintf(LVL_FUNC, "create QuId:%d\n",
			*pQuId
			);
	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_destroy ( PAL_QUEUE_ID QuId )
{
	ENTER_FUNC();
	dprintf(LVL_FUNC, "destory QuId:%d\n",
			QuId
			);
	if (fd_send[QuId]) {
		close(fd_send[QuId]);
		fd_send[QuId] = 0;
	}
	if (fd_receive[QuId]) {
		close(fd_receive[QuId]);
		fd_receive[QuId] = 0;
	}

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_send ( PAL_QUEUE_ID QuId,
		void         *pMessage)
{
	u_int32 retval;
	ENTER_FUNC();
	dprintf(LVL_FUNC, "QuId %d\n", QuId);

	/* CAUTION: if message is not 4 * sizeof(uint_addr) */
	if (fd_send[QuId] == 0) {
		fd_send[QuId] = open(fn_fifo[QuId], O_WRONLY);
		if (fd_send[QuId] == -1) {
			err_msg("failed to open fifo %s to send\n", fn_fifo[QuId]);
			return MEDIAIP_FW_STATUS_FAILURE;
		} else {
			dprintf(LVL_FUNC, "opened fifo %s to send\n", fn_fifo[QuId]);
		}
	}
	retval = write(fd_send[QuId], pMessage, 4 * sizeof(uint_addr));
	if(retval != 4*sizeof(uint_addr))
	{
		err_msg("%s\n", strerror(errno));
		return MEDIAIP_FW_STATUS_FAILURE;
	}
	dprintf(LVL_FUNC, "message send: 0x%lx, 0x%lx, 0x%lx, 0x%lx\n", *(uint_addr *)pMessage, *((uint_addr *)pMessage+1), *((uint_addr *)pMessage+2), *((uint_addr *)pMessage+3));

	return MEDIAIP_FW_STATUS_OK;
}

MEDIAIP_FW_STATUS pal_qu_receive ( PAL_QUEUE_ID QuId,
		u_int32      uTimeoutMs,
		void         *pMessage )
{
	u_int32 retval;

	ENTER_FUNC();
	dprintf(LVL_FUNC, "QuId %d\n", QuId);
	// TODO: time out case
	if (fd_receive[QuId] == 0) {
		fd_receive[QuId] = open(fn_fifo[QuId], O_RDONLY);
		if (fd_receive[QuId] == -1) {
			err_msg("failed to open fifo %s to receive\n", fn_fifo[QuId]);
			return MEDIAIP_FW_STATUS_FAILURE;
		} else {
			dprintf(LVL_FUNC, "opened fifo %s to receive\n", fn_fifo[QuId]);
		}
	}
	retval = read(fd_receive[QuId], pMessage, 4*sizeof(uint_addr));
	if(retval != 4*sizeof(uint_addr))
	{
		err_msg("%s\n", strerror(errno));
		return MEDIAIP_FW_STATUS_FAILURE;
	}
	dprintf(LVL_FUNC, "message receive: 0x%lx, 0x%lx, 0x%lx, 0x%lx\n", *(uint_addr *)pMessage, *((uint_addr *)pMessage+1), *((uint_addr *)pMessage+2), *((uint_addr *)pMessage+3));

	return MEDIAIP_FW_STATUS_OK;
}
#endif

