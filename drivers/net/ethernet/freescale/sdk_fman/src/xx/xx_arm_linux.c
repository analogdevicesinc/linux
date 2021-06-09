/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************//**
 @File          xx_arm_linux.c

 @Description   XX routines implementation for Linux.
*//***************************************************************************/
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/ptrace.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/smp.h>
#include <linux/of.h>
#include <linux/irqdomain.h>

#include <linux/workqueue.h>

#ifdef BIGPHYSAREA_ENABLE
#include <linux/bigphysarea.h>
#endif /* BIGPHYSAREA_ENABLE */

//#include <sysdev/fsl_soc.h>
#include <asm/pgtable.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/atomic.h>
#include <asm/string.h>
#include <asm/byteorder.h>
#include <asm/page.h>

#include "error_ext.h"
#include "std_ext.h"
#include "list_ext.h"
#include "mm_ext.h"
#include "sys_io_ext.h"
#include "xx.h"


#define __ERR_MODULE__      MODULE_UNKNOWN

#ifdef BIGPHYSAREA_ENABLE
#define MAX_ALLOCATION_SIZE     128 * 1024 /* Maximum size allocated with kmalloc is 128K */


/* TODO: large allocations => use big phys area */
/******************************************************************************
 * routine:     get_nr_pages
 *
 * description:
 *     calculates the number of memory pages for a given size (in bytes)
 *
 * arguments:
 *     size       - the number of bytes
 *
 * return code:
 *     The number of pages
 *
 *****************************************************************************/
static __inline__ uint32_t get_nr_pages (uint32_t size)
{
    return (uint32_t)((size >> PAGE_SHIFT) + (size & PAGE_SHIFT ? 1 : 0));
}

static bool in_big_phys_area (uint32_t addr)
{
    uint32_t base, size;

    bigphysarea_get_details (&base, &size);
    return ((addr >= base) && (addr < base + size));
}
#endif /* BIGPHYSAREA_ENABLE */

void * xx_Malloc(uint32_t n)
{
    void        *a;
    uint32_t    flags;

    flags = XX_DisableAllIntr();
#ifdef BIGPHYSAREA_ENABLE
    if (n >= MAX_ALLOCATION_SIZE)
        a = (void*)bigphysarea_alloc_pages(get_nr_pages(n), 0, GFP_ATOMIC);
    else
#endif /* BIGPHYSAREA_ENABLE */
    a = (void *)kmalloc((uint32_t)n, GFP_ATOMIC);
    if (!a)
        XX_Print("No memory for XX_Malloc\n");
    XX_RestoreAllIntr(flags);

    return a;
}

void xx_Free(void *p)
{
#ifdef BIGPHYSAREA_ENABLE
    if (in_big_phys_area ((uint32_t)p))
        bigphysarea_free_pages(p);
    else
#endif /* BIGPHYSAREA_ENABLE */
    kfree(p);
}

void XX_Exit(int status)
{
    WARN(1, "\n\nFMD: fatal error, driver can't go on!!!\n\n");
}

#define BUF_SIZE    512
void XX_Print(char *str, ...)
{
    va_list args;
#ifdef CONFIG_SMP
    char buf[BUF_SIZE];
#endif /* CONFIG_SMP */

    va_start(args, str);
#ifdef CONFIG_SMP
    if (vsnprintf (buf, BUF_SIZE, str, args) >= BUF_SIZE)
        printk(KERN_WARNING "Illegal string to print!\n    more than %d characters.\n\tString was not printed completelly.\n", BUF_SIZE);
    printk(KERN_CRIT "cpu %d: %s",  raw_smp_processor_id(), buf);
#else
    vprintk(str, args);
#endif /* CONFIG_SMP */
    va_end(args);
}

void XX_Fprint(void *file, char *str, ...)
{
    va_list args;
#ifdef CONFIG_SMP
    char buf[BUF_SIZE];
#endif /* CONFIG_SMP */

    va_start(args, str);
#ifdef CONFIG_SMP
    if (vsnprintf (buf, BUF_SIZE, str, args) >= BUF_SIZE)
        printk(KERN_WARNING "Illegal string to print!\n    more than %d characters.\n\tString was not printed completelly.\n", BUF_SIZE);
    printk (KERN_CRIT "cpu %d: %s", smp_processor_id(), buf);

#else
    vprintk(str, args);
#endif /* CONFIG_SMP */
    va_end(args);
}

#ifdef DEBUG_XX_MALLOC
typedef void (*t_ffn)(void *);
typedef struct {
    t_ffn       f_free;
    void        *mem;
    char        *fname;
    int         fline;
    uint32_t    size;
    t_List      node;
} t_MemDebug;
#define MEMDBG_OBJECT(p_List) LIST_OBJECT(p_List, t_MemDebug, node)

LIST(memDbgLst);


void * XX_MallocDebug(uint32_t size, char *fname, int line)
{
    void       *mem;
    t_MemDebug *p_MemDbg;

    p_MemDbg = (t_MemDebug *)xx_Malloc(sizeof(t_MemDebug));
    if (p_MemDbg == NULL)
        return NULL;

    mem = xx_Malloc(size);
    if (mem == NULL)
    {
        XX_Free(p_MemDbg);
        return NULL;
    }

    INIT_LIST(&p_MemDbg->node);
    p_MemDbg->f_free = xx_Free;
    p_MemDbg->mem    = mem;
    p_MemDbg->fname  = fname;
    p_MemDbg->fline  = line;
    p_MemDbg->size   = size+sizeof(t_MemDebug);
    LIST_AddToTail(&p_MemDbg->node, &memDbgLst);

    return mem;
}

void * XX_MallocSmartDebug(uint32_t size,
                           int      memPartitionId,
                           uint32_t align,
                           char     *fname,
                           int      line)
{
    void       *mem;
    t_MemDebug *p_MemDbg;

    p_MemDbg = (t_MemDebug *)XX_Malloc(sizeof(t_MemDebug));
    if (p_MemDbg == NULL)
        return NULL;

    mem = xx_MallocSmart((uint32_t)size, memPartitionId, align);
    if (mem == NULL)
    {
        XX_Free(p_MemDbg);
        return NULL;
    }

    INIT_LIST(&p_MemDbg->node);
    p_MemDbg->f_free = xx_FreeSmart;
    p_MemDbg->mem    = mem;
    p_MemDbg->fname  = fname;
    p_MemDbg->fline  = line;
    p_MemDbg->size   = size+sizeof(t_MemDebug);
    LIST_AddToTail(&p_MemDbg->node, &memDbgLst);

    return mem;
}

static void debug_free(void *mem)
{
    t_List      *p_MemDbgLh = NULL;
    t_MemDebug  *p_MemDbg;
    bool        found = FALSE;

    if (LIST_IsEmpty(&memDbgLst))
    {
        REPORT_ERROR(MAJOR, E_ALREADY_FREE, ("Unbalanced free (0x%08x)", mem));
        return;
    }

    LIST_FOR_EACH(p_MemDbgLh, &memDbgLst)
    {
        p_MemDbg = MEMDBG_OBJECT(p_MemDbgLh);
        if (p_MemDbg->mem == mem)
        {
            found = TRUE;
            break;
        }
    }

    if (!found)
    {
        REPORT_ERROR(MAJOR, E_NOT_FOUND,
                     ("Attempt to free unallocated address (0x%08x)",mem));
        dump_stack();
        return;
    }

    LIST_Del(p_MemDbgLh);
    p_MemDbg->f_free(mem);
    p_MemDbg->f_free(p_MemDbg);
}

void XX_FreeSmart(void *p)
{
    debug_free(p);
}


void XX_Free(void *p)
{
    debug_free(p);
}

#else /* not DEBUG_XX_MALLOC */
void * XX_Malloc(uint32_t size)
{
    return xx_Malloc(size);
}

void * XX_MallocSmart(uint32_t size, int memPartitionId, uint32_t alignment)
{
    return xx_MallocSmart(size,memPartitionId, alignment);
}

void XX_FreeSmart(void *p)
{
    xx_FreeSmart(p);
}


void XX_Free(void *p)
{
    xx_Free(p);
}
#endif /* not DEBUG_XX_MALLOC */


#if (defined(REPORT_EVENTS) && (REPORT_EVENTS > 0))
void XX_EventById(uint32_t event, t_Handle appId, uint16_t flags, char *msg)
{
    e_Event eventCode = (e_Event)event;

    UNUSED(eventCode);
    UNUSED(appId);
    UNUSED(flags);
    UNUSED(msg);
}
#endif /* (defined(REPORT_EVENTS) && ... */


uint32_t XX_DisableAllIntr(void)
{
    unsigned long flags;

#ifdef local_irq_save_nort
    local_irq_save_nort(flags);
#else
    local_irq_save(flags);
#endif

    return (uint32_t)flags;
}

void XX_RestoreAllIntr(uint32_t flags)
{
#ifdef local_irq_restore_nort
    local_irq_restore_nort((unsigned long)flags);
#else
    local_irq_restore((unsigned long)flags);
#endif
}

t_Error XX_Call( uint32_t qid, t_Error (* f)(t_Handle), t_Handle id, t_Handle appId, uint16_t flags )
{
    UNUSED(qid);
    UNUSED(appId);
    UNUSED(flags);

    return f(id);
}

int XX_IsICacheEnable(void)
{
    return TRUE;
}

int XX_IsDCacheEnable(void)
{
    return TRUE;
}


typedef struct {
    t_Isr       *f_Isr;
    t_Handle    handle;
} t_InterruptHandler;


t_Handle interruptHandlers[0x00010000];

static irqreturn_t LinuxInterruptHandler (int irq, void *dev_id)
{
    t_InterruptHandler *p_IntrHndl = (t_InterruptHandler *)dev_id;
    p_IntrHndl->f_Isr(p_IntrHndl->handle);
    return IRQ_HANDLED;
}

t_Error XX_SetIntr(int irq, t_Isr *f_Isr, t_Handle handle)
{
    const char *device;
    t_InterruptHandler *p_IntrHndl;

    device = GetDeviceName(irq);
    if (device == NULL)
        RETURN_ERROR(MAJOR, E_INVALID_VALUE, ("Interrupt source - %d", irq));

    p_IntrHndl = (t_InterruptHandler *)XX_Malloc(sizeof(t_InterruptHandler));
    if (p_IntrHndl == NULL)
        RETURN_ERROR(MAJOR, E_NO_MEMORY, NO_MSG);
    p_IntrHndl->f_Isr = f_Isr;
    p_IntrHndl->handle = handle;
    interruptHandlers[irq] = p_IntrHndl;

    if (request_irq(GetDeviceIrqNum(irq), LinuxInterruptHandler, 0, device, p_IntrHndl) < 0)
        RETURN_ERROR(MAJOR, E_BUSY, ("Can't get IRQ %s\n", device));
    disable_irq(GetDeviceIrqNum(irq));

    return E_OK;
}

t_Error XX_FreeIntr(int irq)
{
    t_InterruptHandler *p_IntrHndl = interruptHandlers[irq];
    free_irq(GetDeviceIrqNum(irq), p_IntrHndl);
    XX_Free(p_IntrHndl);
    interruptHandlers[irq] = 0;
    return E_OK;
}

t_Error XX_EnableIntr(int irq)
{
    enable_irq(GetDeviceIrqNum(irq));
    return E_OK;
}

t_Error XX_DisableIntr(int irq)
{
    disable_irq(GetDeviceIrqNum(irq));
    return E_OK;
}


/*****************************************************************************/
/*                       Tasklet Service Routines                            */
/*****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
typedef struct
{
    t_Handle            h_Data;
    void                (*f_Callback) (void *);
    struct delayed_work dwork;
} t_Tasklet;

static void GenericTaskletCallback(struct work_struct *p_Work)
{
    t_Tasklet *p_Task = container_of(p_Work, t_Tasklet, dwork.work);

    p_Task->f_Callback(p_Task->h_Data);
}
#endif    /* LINUX_VERSION_CODE */


t_TaskletHandle XX_InitTasklet (void (*routine)(void *), void *data)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    struct work_struct *p_Task;
    p_Task = (struct work_struct *)XX_Malloc(sizeof(struct work_struct));
    INIT_WORK(p_Task, routine, data);
#else
    t_Tasklet *p_Task = (t_Tasklet *)XX_Malloc(sizeof(t_Tasklet));
    p_Task->h_Data = data;
    p_Task->f_Callback = routine;
    INIT_DELAYED_WORK(&p_Task->dwork, GenericTaskletCallback);
#endif    /* LINUX_VERSION_CODE */

    return (t_TaskletHandle)p_Task;
}


void XX_FreeTasklet (t_TaskletHandle h_Tasklet)
{
    if (h_Tasklet)
        XX_Free(h_Tasklet);
}

int XX_ScheduleTask(t_TaskletHandle h_Tasklet, int immediate)
{
    int ans;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    if (immediate)
        ans = schedule_work(h_Tasklet);
    else
        ans = schedule_delayed_work(h_Tasklet, 1);
#else
    if (immediate)
        ans = schedule_delayed_work(&((t_Tasklet *)h_Tasklet)->dwork, 0);
    else
        ans = schedule_delayed_work(&((t_Tasklet *)h_Tasklet)->dwork, HZ);
#endif /* LINUX_VERSION_CODE */

    return ans;
}

void XX_FlushScheduledTasks(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    flush_scheduled_tasks();
#else
    flush_scheduled_work();
#endif    /* LINUX_VERSION_CODE */
}

int XX_TaskletIsQueued(t_TaskletHandle h_Tasklet)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    return (int)(((struct work_struct *)h_Tasklet)->pending);
#else
    return (int)delayed_work_pending(&((t_Tasklet *)h_Tasklet)->dwork);
#endif    /* LINUX_VERSION_CODE */
}

void XX_SetTaskletData(t_TaskletHandle h_Tasklet, t_Handle data)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
    ((struct tq_struct *)h_Tasklet)->data = data;
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    ((struct work_struct *)h_Tasklet)->data = data;
#else
    ((t_Tasklet *)h_Tasklet)->h_Data = data;
#endif    /* LINUX_VERSION_CODE */
}

t_Handle XX_GetTaskletData(t_TaskletHandle h_Tasklet)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    return (t_Handle)(((struct work_struct *)h_Tasklet)->data);
#else
    return ((t_Tasklet *)h_Tasklet)->h_Data;
#endif    /* LINUX_VERSION_CODE */
}


/*****************************************************************************/
/*                         Spinlock Service Routines                         */
/*****************************************************************************/

t_Handle XX_InitSpinlock(void)
{
    spinlock_t *p_Spinlock = (spinlock_t *)XX_Malloc(sizeof(spinlock_t));
    if (!p_Spinlock)
        return NULL;

    spin_lock_init(p_Spinlock);

    return (t_Handle)p_Spinlock;
}

void XX_FreeSpinlock(t_Handle h_Spinlock)
{
    if (h_Spinlock)
        XX_Free(h_Spinlock);
}

void XX_LockSpinlock(t_Handle h_Spinlock)
{
    spin_lock((spinlock_t *)h_Spinlock);
}

void XX_UnlockSpinlock(t_Handle h_Spinlock)
{
    spin_unlock((spinlock_t *)h_Spinlock);
}

uint32_t XX_LockIntrSpinlock(t_Handle h_Spinlock)
{
    unsigned long intrFlags;
    spin_lock_irqsave((spinlock_t *)h_Spinlock, intrFlags);
    return intrFlags;
}

void XX_UnlockIntrSpinlock(t_Handle h_Spinlock, uint32_t intrFlags)
{
     spin_unlock_irqrestore((spinlock_t *)h_Spinlock, (unsigned long)intrFlags);
}


/*****************************************************************************/
/*                        Timers Service Routines                            */
/*****************************************************************************/
/* The time now is in mili sec. resolution */
uint32_t XX_CurrentTime(void)
{
    return (jiffies*1000)/HZ;
}

#if 0
t_Handle XX_CreateTimer(void)
{
    struct timer_list *p_Timer = (struct timer_list *)XX_Malloc(sizeof(struct timer_list));
    if (p_Timer)
    {
        memset(p_Timer, 0, sizeof(struct timer_list));
        init_timer(p_Timer);
    }
    return (t_Handle)p_Timer;
}

void XX_FreeTimer(t_Handle h_Timer)
{
    if (h_Timer)
        XX_Free(h_Timer);
}

void XX_StartTimer(t_Handle h_Timer,
                   uint32_t msecs,
                   bool     periodic,
                   void     (*f_TimerExpired)(t_Handle),
                   t_Handle h_Arg)
{
    int                 tmp_jiffies = (msecs*HZ)/1000;
    struct timer_list   *p_Timer = (struct timer_list *)h_Timer;

    SANITY_CHECK_RETURN((periodic == FALSE), E_NOT_SUPPORTED);

    p_Timer->function = (void (*)(unsigned long))f_TimerExpired;
    p_Timer->data = (unsigned long)h_Arg;
    if ((msecs*HZ)%1000)
        tmp_jiffies++;
    p_Timer->expires = (jiffies + tmp_jiffies);

    add_timer((struct timer_list *)h_Timer);
}

void XX_SetTimerData(t_Handle h_Timer, t_Handle data)
{
    struct timer_list   *p_Timer = (struct timer_list *)h_Timer;

    p_Timer->data = (unsigned long)data;
}

t_Handle XX_GetTimerData(t_Handle h_Timer)
{
    struct timer_list   *p_Timer = (struct timer_list *)h_Timer;

    return (t_Handle)p_Timer->data;
}

uint32_t   XX_GetExpirationTime(t_Handle h_Timer)
{
    struct timer_list   *p_Timer = (struct timer_list *)h_Timer;

    return (uint32_t)p_Timer->expires;
}

void XX_StopTimer(t_Handle h_Timer)
{
    del_timer((struct timer_list *)h_Timer);
}

void XX_ModTimer(t_Handle h_Timer, uint32_t msecs)
{
    int tmp_jiffies = (msecs*HZ)/1000;

    if ((msecs*HZ)%1000)
        tmp_jiffies++;
    mod_timer((struct timer_list *)h_Timer, jiffies + tmp_jiffies);
}

int XX_TimerIsActive(t_Handle h_Timer)
{
  return timer_pending((struct timer_list *)h_Timer);
}
#endif

uint32_t XX_Sleep(uint32_t msecs)
{
    int tmp_jiffies = (msecs*HZ)/1000;

    if ((msecs*HZ)%1000)
        tmp_jiffies++;
    return schedule_timeout(tmp_jiffies);
}

/*BEWARE!!!!! UDelay routine is BUSY WAITTING!!!!!*/
void XX_UDelay(uint32_t usecs)
{
    udelay(usecs);
}

/* TODO: verify that these are correct */
#define MSG_BODY_SIZE       512
typedef t_Error (t_MsgHandler) (t_Handle h_Mod, uint32_t msgId, uint8_t msgBody[MSG_BODY_SIZE]);
typedef void (t_MsgCompletionCB) (t_Handle h_Arg, uint8_t msgBody[MSG_BODY_SIZE]);
t_Error XX_SendMessage(char                 *p_DestAddr,
                       uint32_t             msgId,
                       uint8_t              msgBody[MSG_BODY_SIZE],
                       t_MsgCompletionCB    *f_CompletionCB,
                       t_Handle             h_CBArg);

typedef struct {
    char            *p_Addr;
    t_MsgHandler    *f_MsgHandlerCB;
    t_Handle        h_Mod;
    t_List          node;
} t_MsgHndlr;
#define MSG_HNDLR_OBJECT(ptr)  LIST_OBJECT(ptr, t_MsgHndlr, node)

LIST(msgHndlrList);

static void EnqueueMsgHndlr(t_MsgHndlr *p_MsgHndlr)
{
    uint32_t   intFlags;

    intFlags = XX_DisableAllIntr();
    LIST_AddToTail(&p_MsgHndlr->node, &msgHndlrList);
    XX_RestoreAllIntr(intFlags);
}
/* TODO: add this for multi-platform support
static t_MsgHndlr * DequeueMsgHndlr(void)
{
    t_MsgHndlr *p_MsgHndlr = NULL;
    uint32_t   intFlags;

    intFlags = XX_DisableAllIntr();
    if (!LIST_IsEmpty(&msgHndlrList))
    {
        p_MsgHndlr = MSG_HNDLR_OBJECT(msgHndlrList.p_Next);
        LIST_DelAndInit(&p_MsgHndlr->node);
    }
    XX_RestoreAllIntr(intFlags);

    return p_MsgHndlr;
}
*/
static t_MsgHndlr * FindMsgHndlr(char *p_Addr)
{
    t_MsgHndlr  *p_MsgHndlr;
    t_List      *p_Pos;

    LIST_FOR_EACH(p_Pos, &msgHndlrList)
    {
        p_MsgHndlr = MSG_HNDLR_OBJECT(p_Pos);
        if (strstr(p_MsgHndlr->p_Addr, p_Addr))
            return p_MsgHndlr;
    }

    return NULL;
}

t_Error XX_RegisterMessageHandler   (char *p_Addr, t_MsgHandler *f_MsgHandlerCB, t_Handle h_Mod)
{
    t_MsgHndlr  *p_MsgHndlr;
    uint32_t    len;

    p_MsgHndlr = (t_MsgHndlr*)XX_Malloc(sizeof(t_MsgHndlr));
    if (!p_MsgHndlr)
        RETURN_ERROR(MINOR, E_NO_MEMORY, ("message handler object!!!"));
    memset(p_MsgHndlr, 0, sizeof(t_MsgHndlr));

    len = strlen(p_Addr);
    p_MsgHndlr->p_Addr = (char*)XX_Malloc(len+1);
    strncpy(p_MsgHndlr->p_Addr,p_Addr, (uint32_t)(len+1));

    p_MsgHndlr->f_MsgHandlerCB = f_MsgHandlerCB;
    p_MsgHndlr->h_Mod = h_Mod;
    INIT_LIST(&p_MsgHndlr->node);
    EnqueueMsgHndlr(p_MsgHndlr);

    return E_OK;
}

t_Error XX_UnregisterMessageHandler (char *p_Addr)
{
    t_MsgHndlr *p_MsgHndlr = FindMsgHndlr(p_Addr);
    if (!p_MsgHndlr)
        RETURN_ERROR(MINOR, E_NO_DEVICE, ("message handler not found in list!!!"));

    LIST_Del(&p_MsgHndlr->node);
    XX_Free(p_MsgHndlr->p_Addr);
    XX_Free(p_MsgHndlr);

    return E_OK;
}

t_Error XX_SendMessage(char                 *p_DestAddr,
                       uint32_t             msgId,
                       uint8_t              msgBody[MSG_BODY_SIZE],
                       t_MsgCompletionCB    *f_CompletionCB,
                       t_Handle             h_CBArg)
{
    t_Error     ans;
    t_MsgHndlr  *p_MsgHndlr = FindMsgHndlr(p_DestAddr);
    if (!p_MsgHndlr)
        RETURN_ERROR(MINOR, E_NO_DEVICE, ("message handler not found in list!!!"));

    ans = p_MsgHndlr->f_MsgHandlerCB(p_MsgHndlr->h_Mod, msgId, msgBody);

    if (f_CompletionCB)
        f_CompletionCB(h_CBArg, msgBody);

    return ans;
}

t_Error XX_IpcRegisterMsgHandler(char                   addr[XX_IPC_MAX_ADDR_NAME_LENGTH],
                                 t_IpcMsgHandler        *f_MsgHandler,
                                 t_Handle               h_Module,
                                 uint32_t               replyLength)
{
    UNUSED(addr);UNUSED(f_MsgHandler);UNUSED(h_Module);UNUSED(replyLength);
    return E_OK;
}

t_Error XX_IpcUnregisterMsgHandler(char addr[XX_IPC_MAX_ADDR_NAME_LENGTH])
{
    UNUSED(addr);
    return E_OK;
}


t_Error XX_IpcSendMessage(t_Handle           h_Session,
                          uint8_t            *p_Msg,
                          uint32_t           msgLength,
                          uint8_t            *p_Reply,
                          uint32_t           *p_ReplyLength,
                          t_IpcMsgCompletion *f_Completion,
                          t_Handle           h_Arg)
{
    UNUSED(h_Session); UNUSED(p_Msg); UNUSED(msgLength); UNUSED(p_Reply);
    UNUSED(p_ReplyLength); UNUSED(f_Completion); UNUSED(h_Arg);
    return E_OK;
}

t_Handle XX_IpcInitSession(char destAddr[XX_IPC_MAX_ADDR_NAME_LENGTH],
                           char srcAddr[XX_IPC_MAX_ADDR_NAME_LENGTH])
{
    UNUSED(destAddr); UNUSED(srcAddr);
    return E_OK;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,22)
int GetDeviceIrqNum(int irq)
{
    struct device_node  *iPar;
    struct irq_domain   *irqHost;
    uint32_t            hwIrq;

    /* Get the interrupt controller */
    iPar = of_find_node_by_name(NULL, "mpic");
    hwIrq = 0;

    ASSERT_COND(iPar != NULL);
    /* Get the irq host */
    irqHost = irq_find_host(iPar);
    of_node_put(iPar);

    /* Create irq mapping */
    return irq_create_mapping(irqHost, hwIrq);
}
#else
#error "kernel not supported!!!"
#endif    /* LINUX_VERSION_CODE */

void * XX_PhysToVirt(physAddress_t addr)
{
    return UINT_TO_PTR(SYS_PhysToVirt((uint64_t)addr));
}

physAddress_t XX_VirtToPhys(void * addr)
{
    return (physAddress_t)SYS_VirtToPhys(PTR_TO_UINT(addr));
}

void * xx_MallocSmart(uint32_t size, int memPartitionId, uint32_t alignment)
{
    uintptr_t   *returnCode, tmp;

    if (alignment < sizeof(uintptr_t))
        alignment = sizeof(uintptr_t);
    size += alignment + sizeof(returnCode);
    tmp = (uintptr_t)xx_Malloc(size);
    if (tmp == 0)
        return NULL;
    returnCode = (uintptr_t*)((tmp + alignment + sizeof(returnCode)) & ~((uintptr_t)alignment - 1));
    *(returnCode - 1) = tmp;

    return (void*)returnCode;
}

void xx_FreeSmart(void *p)
{
    xx_Free((void*)(*((uintptr_t *)(p) - 1)));
}
