/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2018 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2018 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


#ifndef __gc_hal_kernel_h_
#define __gc_hal_kernel_h_

#include "gc_hal.h"
#include "gc_hal_kernel_hardware.h"
#include "gc_hal_driver.h"
#include "gc_hal_kernel_mutex.h"
#include "gc_hal_metadata.h"
#include "gc_hal_kernel_buffer.h"


#if gcdENABLE_VG
#include "gc_hal_kernel_vg.h"
#endif

#if gcdSECURITY || gcdENABLE_TRUST_APPLICATION
#include "gc_hal_security_interface.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
***** New MMU Defination *******************************************************/
#define gcdMMU_MTLB_SHIFT           22
#define gcdMMU_STLB_4K_SHIFT        12
#define gcdMMU_STLB_64K_SHIFT       16

#define gcdMMU_MTLB_BITS            (32 - gcdMMU_MTLB_SHIFT)
#define gcdMMU_PAGE_4K_BITS         gcdMMU_STLB_4K_SHIFT
#define gcdMMU_STLB_4K_BITS         (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_4K_BITS)
#define gcdMMU_PAGE_64K_BITS        gcdMMU_STLB_64K_SHIFT
#define gcdMMU_STLB_64K_BITS        (32 - gcdMMU_MTLB_BITS - gcdMMU_PAGE_64K_BITS)

#define gcdMMU_MTLB_ENTRY_NUM       (1 << gcdMMU_MTLB_BITS)
#define gcdMMU_MTLB_SIZE            (gcdMMU_MTLB_ENTRY_NUM << 2)
#define gcdMMU_STLB_4K_ENTRY_NUM    (1 << gcdMMU_STLB_4K_BITS)
#define gcdMMU_STLB_4K_SIZE         (gcdMMU_STLB_4K_ENTRY_NUM << 2)
#define gcdMMU_PAGE_4K_SIZE         (1 << gcdMMU_STLB_4K_SHIFT)
#define gcdMMU_STLB_64K_ENTRY_NUM   (1 << gcdMMU_STLB_64K_BITS)
#define gcdMMU_STLB_64K_SIZE        (gcdMMU_STLB_64K_ENTRY_NUM << 2)
#define gcdMMU_PAGE_64K_SIZE        (1 << gcdMMU_STLB_64K_SHIFT)

#define gcdMMU_MTLB_MASK            (~((1U << gcdMMU_MTLB_SHIFT)-1))
#define gcdMMU_STLB_4K_MASK         ((~0U << gcdMMU_STLB_4K_SHIFT) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_4K_MASK         (gcdMMU_PAGE_4K_SIZE - 1)
#define gcdMMU_STLB_64K_MASK        ((~((1U << gcdMMU_STLB_64K_SHIFT)-1)) ^ gcdMMU_MTLB_MASK)
#define gcdMMU_PAGE_64K_MASK        (gcdMMU_PAGE_64K_SIZE - 1)

/* Page offset definitions. */
#define gcdMMU_OFFSET_4K_BITS       (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_4K_BITS)
#define gcdMMU_OFFSET_4K_MASK       ((1U << gcdMMU_OFFSET_4K_BITS) - 1)
#define gcdMMU_OFFSET_16K_BITS      (32 - gcdMMU_MTLB_BITS - gcdMMU_STLB_16K_BITS)
#define gcdMMU_OFFSET_16K_MASK      ((1U << gcdMMU_OFFSET_16K_BITS) - 1)

#define gcdMMU_MTLB_ENTRY_HINTS_BITS 6
#define gcdMMU_MTLB_ENTRY_STLB_MASK  (~((1U << gcdMMU_MTLB_ENTRY_HINTS_BITS) - 1))

#define gcdMMU_MTLB_PRESENT         0x00000001
#define gcdMMU_MTLB_EXCEPTION       0x00000002
#define gcdMMU_MTLB_4K_PAGE         0x00000000

#define gcdMMU_STLB_PRESENT         0x00000001
#define gcdMMU_STLB_EXCEPTION       0x00000002
#define gcdMMU_STLB_4K_PAGE         0x00000000

/*******************************************************************************
***** Stuck Dump Level ********************************************************/

/* Dump nonthing when stuck happens. */
#define gcvSTUCK_DUMP_NONE          0

/* Dump GPU state and memory near stuck point. */
#define gcvSTUCK_DUMP_NEARBY_MEMORY 1

/* Beside gcvSTUCK_DUMP_NEARBY_MEMORY, dump context buffer and user command buffer. */
#define gcvSTUCK_DUMP_USER_COMMAND  2

/* Beside gcvSTUCK_DUMP_USER_COMMAND, commit will be stall
** to make sure command causing stuck isn't missed. */
#define gcvSTUCK_DUMP_STALL_COMMAND 3

/* Beside gcvSTUCK_DUMP_USER_COMMAND, dump kernel command buffer. */
#define gcvSTUCK_DUMP_ALL_COMMAND   4

/*******************************************************************************
***** Process Secure Cache ****************************************************/

#define gcdSECURE_CACHE_LRU         1
#define gcdSECURE_CACHE_LINEAR      2
#define gcdSECURE_CACHE_HASH        3
#define gcdSECURE_CACHE_TABLE       4

#define gcvPAGE_TABLE_DIRTY_BIT_OTHER   (1 << 0)
#define gcvPAGE_TABLE_DIRTY_BIT_FE      (1 << 1)

typedef struct _gcskLOGICAL_CACHE * gcskLOGICAL_CACHE_PTR;
typedef struct _gcskLOGICAL_CACHE   gcskLOGICAL_CACHE;
struct _gcskLOGICAL_CACHE
{
    /* Logical address. */
    gctPOINTER                      logical;

    /* DMAable address. */
    gctUINT32                       dma;

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
    /* Pointer to the previous and next hash tables. */
    gcskLOGICAL_CACHE_PTR           nextHash;
    gcskLOGICAL_CACHE_PTR           prevHash;
#endif

#if gcdSECURE_CACHE_METHOD != gcdSECURE_CACHE_TABLE
    /* Pointer to the previous and next slot. */
    gcskLOGICAL_CACHE_PTR           next;
    gcskLOGICAL_CACHE_PTR           prev;
#endif

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LINEAR
    /* Time stamp. */
    gctUINT64                       stamp;
#endif
};

typedef struct _gcskSECURE_CACHE * gcskSECURE_CACHE_PTR;
typedef struct _gcskSECURE_CACHE
{
    /* Cache memory. */
    gcskLOGICAL_CACHE               cache[1 + gcdSECURE_CACHE_SLOTS];

    /* Last known index for LINEAR mode. */
    gcskLOGICAL_CACHE_PTR           cacheIndex;

    /* Current free slot for LINEAR mode. */
    gctUINT32                       cacheFree;

    /* Time stamp for LINEAR mode. */
    gctUINT64                       cacheStamp;

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
    /* Hash table for HASH mode. */
    gcskLOGICAL_CACHE              hash[256];
#endif
}
gcskSECURE_CACHE;

/*******************************************************************************
***** Process Database Management *********************************************/

typedef enum _gceDATABASE_TYPE
{
    gcvDB_VIDEO_MEMORY = 1,             /* Video memory created. */
    gcvDB_COMMAND_BUFFER,               /* Command Buffer. */
    gcvDB_NON_PAGED,                    /* Non paged memory. */
    gcvDB_CONTIGUOUS,                   /* Contiguous memory. */
    gcvDB_SIGNAL,                       /* Signal. */
    gcvDB_VIDEO_MEMORY_LOCKED,          /* Video memory locked. */
    gcvDB_CONTEXT,                      /* Context */
    gcvDB_IDLE,                         /* GPU idle. */
    gcvDB_MAP_MEMORY,                   /* Map memory */
    gcvDB_MAP_USER_MEMORY,              /* Map user memory */
    gcvDB_SHBUF,                        /* Shared buffer. */

    gcvDB_NUM_TYPES,
}
gceDATABASE_TYPE;

#define gcdDATABASE_TYPE_MASK           0x000000FF
#define gcdDB_VIDEO_MEMORY_TYPE_MASK    0x0000FF00
#define gcdDB_VIDEO_MEMORY_TYPE_SHIFT   8

#define gcdDB_VIDEO_MEMORY_POOL_MASK    0x00FF0000
#define gcdDB_VIDEO_MEMORY_POOL_SHIFT   16

typedef struct _gcsDATABASE_RECORD *    gcsDATABASE_RECORD_PTR;
typedef struct _gcsDATABASE_RECORD
{
    /* Pointer to kernel. */
    gckKERNEL                           kernel;

    /* Pointer to next database record. */
    gcsDATABASE_RECORD_PTR              next;

    /* Type of record. */
    gceDATABASE_TYPE                    type;

    /* Data for record. */
    gctPOINTER                          data;
    gctPHYS_ADDR                        physical;
    gctSIZE_T                           bytes;
}
gcsDATABASE_RECORD;

typedef struct _gcsDATABASE *           gcsDATABASE_PTR;
typedef struct _gcsDATABASE
{
    /* Pointer to next entry is hash list. */
    gcsDATABASE_PTR                     next;
    gctSIZE_T                           slot;

    /* Process ID. */
    gctUINT32                           processID;

    /* Open-Close ref count */
    gctPOINTER                          refs;

    /* Already mark for delete and cannot reenter */
    gctBOOL                             deleted;

    /* Sizes to query. */
    gcsDATABASE_COUNTERS                vidMem;
    gcsDATABASE_COUNTERS                nonPaged;
    gcsDATABASE_COUNTERS                contiguous;
    gcsDATABASE_COUNTERS                mapUserMemory;
    gcsDATABASE_COUNTERS                mapMemory;

    gcsDATABASE_COUNTERS                vidMemType[gcvSURF_NUM_TYPES];
    /* Counter for each video memory pool. */
    gcsDATABASE_COUNTERS                vidMemPool[gcvPOOL_NUMBER_OF_POOLS];
    gctPOINTER                          counterMutex;

    /* Idle time management. */
    gctUINT64                           lastIdle;
    gctUINT64                           idle;

    /* Pointer to database. */
    gcsDATABASE_RECORD_PTR              list[48];

#if gcdSECURE_USER
    /* Secure cache. */
    gcskSECURE_CACHE                    cache;
#endif

    gctPOINTER                          handleDatabase;
    gctPOINTER                          handleDatabaseMutex;

#if gcdPROCESS_ADDRESS_SPACE
    gckMMU                              mmu;
#endif
}
gcsDATABASE;

typedef struct _gcsFDPRIVATE *          gcsFDPRIVATE_PTR;
typedef struct _gcsFDPRIVATE
{
    gctINT                              (* release) (gcsFDPRIVATE_PTR Private);
}
gcsFDPRIVATE;

typedef struct _gcsRECORDER * gckRECORDER;


/* Create a process database that will contain all its allocations. */
gceSTATUS
gckKERNEL_CreateProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID
    );

/* Add a record to the process database. */
gceSTATUS
gckKERNEL_AddProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gceDATABASE_TYPE Type,
    IN gctPOINTER Pointer,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Size
    );

/* Remove a record to the process database. */
gceSTATUS
gckKERNEL_RemoveProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gceDATABASE_TYPE Type,
    IN gctPOINTER Pointer
    );

/* Destroy the process database. */
gceSTATUS
gckKERNEL_DestroyProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID
    );

/* Find a record to the process database. */
gceSTATUS
gckKERNEL_FindProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 ThreadID,
    IN gceDATABASE_TYPE Type,
    IN gctPOINTER Pointer,
    OUT gcsDATABASE_RECORD_PTR Record
    );

/* Query the process database. */
gceSTATUS
gckKERNEL_QueryProcessDB(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctBOOL LastProcessID,
    IN gceDATABASE_TYPE Type,
    OUT gcuDATABASE_INFO * Info
    );

/* Dump the process database. */
gceSTATUS
gckKERNEL_DumpProcessDB(
    IN gckKERNEL Kernel
    );

/* Dump the video memory usage for process specified. */
gceSTATUS
gckKERNEL_DumpVidMemUsage(
    IN gckKERNEL Kernel,
    IN gctINT32 ProcessID
    );

gceSTATUS
gckKERNEL_FindDatabase(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctBOOL LastProcessID,
    OUT gcsDATABASE_PTR * Database
    );

gceSTATUS
gckKERNEL_FindHandleDatbase(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    OUT gctPOINTER * HandleDatabase,
    OUT gctPOINTER * HandleDatabaseMutex
    );

gceSTATUS
gckKERNEL_GetProcessMMU(
    IN gckKERNEL Kernel,
    OUT gckMMU * Mmu
    );

gceSTATUS
gckMMU_FlatMapping(
    IN gckMMU Mmu,
    IN gctUINT32 Physical,
    IN gctUINT32 NumPages
    );

gceSTATUS
gckMMU_GetPageEntry(
    IN gckMMU Mmu,
    IN gctUINT32 Address,
    IN gctUINT32_PTR *PageTable
    );

gceSTATUS
gckMMU_FreePagesEx(
    IN gckMMU Mmu,
    IN gctUINT32 Address,
    IN gctSIZE_T PageCount
    );

gceSTATUS
gckMMU_AttachHardware(
    IN gckMMU Mmu,
    IN gckHARDWARE Hardware
    );

void
gckMMU_DumpRecentFreedAddress(
    IN gckMMU Mmu
    );

gceSTATUS
gckKERNEL_CreateIntegerDatabase(
    IN gckKERNEL Kernel,
    OUT gctPOINTER * Database
    );

gceSTATUS
gckKERNEL_DestroyIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctPOINTER Database
    );

gceSTATUS
gckKERNEL_AllocateIntegerId(
    IN gctPOINTER Database,
    IN gctPOINTER Pointer,
    OUT gctUINT32 * Id
    );

gceSTATUS
gckKERNEL_FreeIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id
    );

gceSTATUS
gckKERNEL_QueryIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id,
    OUT gctPOINTER * Pointer
    );

/* Pointer rename  */
gctUINT32
gckKERNEL_AllocateNameFromPointer(
    IN gckKERNEL Kernel,
    IN gctPOINTER Pointer
    );

gctPOINTER
gckKERNEL_QueryPointerFromName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    );

gceSTATUS
gckKERNEL_DeleteName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    );

#if gcdSECURE_USER
/* Get secure cache from the process database. */
gceSTATUS
gckKERNEL_GetProcessDBCache(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    OUT gcskSECURE_CACHE_PTR * Cache
    );
#endif

/*******************************************************************************
********* Timer Management ****************************************************/
typedef struct _gcsTIMER *           gcsTIMER_PTR;
typedef struct _gcsTIMER
{
    /* Start and Stop time holders. */
    gctUINT64                           startTime;
    gctUINT64                           stopTime;
}
gcsTIMER;

/******************************************************************************\
********************************** Structures **********************************
\******************************************************************************/

/* gckDB object. */
struct _gckDB
{
    /* Database management. */
    gcsDATABASE_PTR             db[16];
    gctPOINTER                  dbMutex;
    gcsDATABASE_PTR             freeDatabase;
    gcsDATABASE_RECORD_PTR      freeRecord;
    gcsDATABASE_PTR             lastDatabase;
    gctUINT32                   lastProcessID;
    gctUINT64                   lastIdle;
    gctUINT64                   idleTime;
    gctUINT64                   lastSlowdown;
    gctUINT64                   lastSlowdownIdle;
    gctPOINTER                  nameDatabase;
    gctPOINTER                  nameDatabaseMutex;

    gctPOINTER                  pointerDatabase;
    gctPOINTER                  pointerDatabaseMutex;

    gcsLISTHEAD                 onFaultVidmemList;
    gctPOINTER                  onFaultVidmemListMutex;
};

typedef struct _gckVIRTUAL_BUFFER * gckVIRTUAL_BUFFER_PTR;
typedef struct _gckVIRTUAL_BUFFER
{
    gctPHYS_ADDR                physical;
    gctPOINTER                  userLogical;
    gctPOINTER                  kernelLogical;
    gctSIZE_T                   bytes;
    gctSIZE_T                   pageCount;
    gctPOINTER                  pageTable;
    gctUINT32                   gpuAddress;
    gctUINT                     pid;
    gckKERNEL                   kernel;
#if gcdPROCESS_ADDRESS_SPACE
    gckMMU                      mmu;
#endif
}
gckVIRTUAL_BUFFER;

typedef struct _gckVIRTUAL_COMMAND_BUFFER * gckVIRTUAL_COMMAND_BUFFER_PTR;
typedef struct _gckVIRTUAL_COMMAND_BUFFER
{
    gckVIRTUAL_BUFFER               virtualBuffer;
    gckVIRTUAL_COMMAND_BUFFER_PTR   next;
    gckVIRTUAL_COMMAND_BUFFER_PTR   prev;
}
gckVIRTUAL_COMMAND_BUFFER;

/* gckKERNEL object. */
struct _gckKERNEL
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Core */
    gceCORE                     core;

    /* Pointer to gckHARDWARE object. */
    gckHARDWARE                 hardware;

    /* Pointer to gckCOMMAND object. */
    gckCOMMAND                  command;

    /* Pointer to gckEVENT object. */
    gckEVENT                    eventObj;

    /* Pointer to context. */
    gctPOINTER                  context;

    /* Pointer to gckMMU object. */
    gckMMU                      mmu;

    /* Arom holding number of clients. */
    gctPOINTER                  atomClients;

#if VIVANTE_PROFILER
    /* Enable profiling */
    gctBOOL                     profileEnable;
    /* Clear profile register or not*/
    gctBOOL                     profileCleanRegister;
#endif

#ifdef QNX_SINGLE_THREADED_DEBUGGING
    gctPOINTER                  debugMutex;
#endif

    /* Database management. */
    gckDB                       db;
    gctBOOL                     dbCreated;

    gctUINT64                   resetTimeStamp;

    /* Pointer to gckEVENT object. */
    gcsTIMER                    timers[8];
    gctUINT32                   timeOut;

#if gcdENABLE_VG
    gckVGKERNEL                 vg;
#endif

    /* Virtual command buffer list. */
    gckVIRTUAL_COMMAND_BUFFER_PTR virtualBufferHead;
    gckVIRTUAL_COMMAND_BUFFER_PTR virtualBufferTail;
    gctPOINTER                    virtualBufferLock;

    /* Enable virtual command buffer. */
    gctBOOL                     virtualCommandBuffer;

#if gcdDVFS
    gckDVFS                     dvfs;
#endif

#if gcdLINUX_SYNC_FILE
    gctHANDLE                   timeline;
#endif

    /* Enable recovery. */
    gctBOOL                     recovery;

    /* Level of dump information after stuck. */
    gctUINT                     stuckDump;

#if gcdSECURITY || gcdENABLE_TRUST_APPLICATION
    gctUINT32                   securityChannel;
#endif

    /* Timer to monitor GPU stuck. */
    gctPOINTER                  monitorTimer;

    /* Flag to quit monitor timer. */
    gctBOOL                     monitorTimerStop;

    /* Monitor states. */
    gctBOOL                     monitoring;
    gctUINT32                   lastCommitStamp;
    gctUINT32                   timer;
    gctUINT32                   restoreAddress;
    gctINT32                    restoreMask;

    /* 3DBLIT */
    gckASYNC_COMMAND            asyncCommand;
    gckEVENT                    asyncEvent;

    /* Pointer to gckDEVICE object. */
    gckDEVICE                   device;

    gctUINT                     chipID;

    gctUINT32                   contiguousBaseAddress;
    gctUINT32                   externalBaseAddress;
};

struct _FrequencyHistory
{
    gctUINT32                   frequency;
    gctUINT32                   count;
};

/* gckDVFS object. */
struct _gckDVFS
{
    gckOS                       os;
    gckHARDWARE                 hardware;
    gctPOINTER                  timer;
    gctUINT32                   pollingTime;
    gctBOOL                     stop;
    gctUINT32                   totalConfig;
    gctUINT32                   loads[8];
    gctUINT8                    currentScale;
    struct _FrequencyHistory    frequencyHistory[16];
};

typedef struct _gcsFENCE * gckFENCE;
typedef struct _gcsFENCE
{
    /* Pointer to required object. */
    gckKERNEL                   kernel;

    /* Fence location. */
    gctPHYS_ADDR                physical;
    gctPHYS_ADDR                physHandle;
    gctPOINTER                  logical;
    gctUINT32                   address;

    gcsLISTHEAD                 waitingList;
    gctPOINTER                  mutex;
}
gcsFENCE;

/* A sync point attached to fence. */
typedef struct _gcsFENCE_SYNC * gckFENCE_SYNC;
typedef struct _gcsFENCE_SYNC
{
    /* Stamp of commit access this node. */
    gctUINT64                   commitStamp;

    /* Attach to waiting list. */
    gcsLISTHEAD                 head;

    gctPOINTER                  signal;

    gctBOOL                     inList;
}
gcsFENCE_SYNC;

/* gckCOMMAND object. */
struct _gckCOMMAND
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to required object. */
    gckKERNEL                   kernel;
    gckOS                       os;

    /* Number of bytes per page. */
    gctUINT32                   pageSize;

    /* Current pipe select. */
    gcePIPE_SELECT              pipeSelect;

    /* Command queue running flag. */
    gctBOOL                     running;

    /* Idle flag and commit stamp. */
    gctBOOL                     idle;
    gctUINT64                   commitStamp;

    /* Command queue mutex. */
    gctPOINTER                  mutexQueue;

    /* Context switching mutex. */
    gctPOINTER                  mutexContext;

    /* Context sequence mutex. */
    gctPOINTER                  mutexContextSeq;

    /* Command queue power semaphore. */
    gctPOINTER                  powerSemaphore;

    /* Current command queue. */
    struct _gcskCOMMAND_QUEUE
    {
        gctSIGNAL               signal;
        gctPHYS_ADDR            physical;
        gctPOINTER              logical;
        gctUINT32               address;
    }
    queues[gcdCOMMAND_QUEUES];

    gctPHYS_ADDR                virtualMemory;
    gctPHYS_ADDR                physHandle;
    gctUINT32                   physical;
    gctPOINTER                  logical;
    gctUINT32                   address;
    gctUINT32                   offset;
    gctINT                      index;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gctUINT                     wrapCount;
#endif

    /* The command queue is new. */
    gctBOOL                     newQueue;

    /* Context management. */
    gckCONTEXT                  currContext;
    gctPOINTER                  stateMap;

    /* Pointer to last WAIT command. */
    gctUINT32                   waitPhysical;
    gctPOINTER                  waitLogical;
    gctUINT32                   waitAddress;
    gctUINT32                   waitSize;
    gctUINT32                   waitOffset;

    /* Command buffer alignment. */
    gctUINT32                   alignment;
    gctUINT32                   reservedHead;

    /* Commit counter. */
    gctPOINTER                  atomCommit;

    /* Kernel process ID. */
    gctUINT32                   kernelProcessID;

    /* End Event signal. */
    gctSIGNAL                   endEventSignal;

#if gcdSECURE_USER
    /* Hint array copy buffer. */
    gctBOOL                     hintArrayAllocated;
    gctUINT                     hintArraySize;
    gctUINT32_PTR               hintArray;
#endif

#if gcdPROCESS_ADDRESS_SPACE
    gckMMU                      currentMmu;
#endif

#if gcdRECORD_COMMAND
    gckRECORDER                 recorder;
#endif

    gctPOINTER                  kList;

    gckFENCE                    fence;

    /* For getting state from async command buffer. */
    gckASYNC_COMMAND            asyncCommand;

    gctBOOL                     dummyDraw;

    /* a copy in kernel space for current committing command buffer
     * avoid occupyting stack space.
     */
    struct _gcoCMDBUF           _commandBufferObject;
};

typedef struct _gcsEVENT *      gcsEVENT_PTR;

/* Structure holding one event to be processed. */
typedef struct _gcsEVENT
{
    /* Pointer to next event in queue. */
    gcsEVENT_PTR                next;

    /* Event information. */
    gcsHAL_INTERFACE            info;

    /* Process ID owning the event. */
    gctUINT32                   processID;

#ifdef __QNXNTO__
    /* Kernel. */
    gckKERNEL                   kernel;
#endif

    gctBOOL                     fromKernel;
}
gcsEVENT;

/* Structure holding a list of events to be processed by an interrupt. */
typedef struct _gcsEVENT_QUEUE * gcsEVENT_QUEUE_PTR;
typedef struct _gcsEVENT_QUEUE
{
    /* Time stamp. */
    gctUINT64                   stamp;

    /* Source of the event. */
    gceKERNEL_WHERE             source;

    /* Pointer to head of event queue. */
    gcsEVENT_PTR                head;

    /* Pointer to tail of event queue. */
    gcsEVENT_PTR                tail;

    /* Next list of events. */
    gcsEVENT_QUEUE_PTR          next;

    /* Current commit stamp. */
    gctUINT64                   commitStamp;
}
gcsEVENT_QUEUE;

/*
    gcdREPO_LIST_COUNT defines the maximum number of event queues with different
    hardware module sources that may coexist at the same time. Only two sources
    are supported - gcvKERNEL_COMMAND and gcvKERNEL_PIXEL. gcvKERNEL_COMMAND
    source is used only for managing the kernel command queue and is only issued
    when the current command queue gets full. Since we commit event queues every
    time we commit command buffers, in the worst case we can have up to three
    pending event queues:
        - gcvKERNEL_PIXEL
        - gcvKERNEL_COMMAND (queue overflow)
        - gcvKERNEL_PIXEL
*/
#define gcdREPO_LIST_COUNT      3

/* gckEVENT object. */
struct _gckEVENT
{
    /* The object. */
    gcsOBJECT                   object;

    /* Pointer to required objects. */
    gckOS                       os;
    gckKERNEL                   kernel;

    /* Pointer to gckASYNC_COMMAND object. */
    gckASYNC_COMMAND            asyncCommand;

    /* Time stamp. */
    gctUINT64                   stamp;
    gctUINT32                   lastCommitStamp;

    /* Queue mutex. */
    gctPOINTER                  eventQueueMutex;

    /* Array of event queues. */
    gcsEVENT_QUEUE              queues[29];
    gctINT32                    freeQueueCount;
    gctUINT8                    lastID;

    /* Pending events. */
    gctPOINTER                  pending;

    /* List of free event structures and its mutex. */
    gcsEVENT_PTR                freeEventList;
    gctSIZE_T                   freeEventCount;
    gctPOINTER                  freeEventMutex;

    /* Event queues. */
    gcsEVENT_QUEUE_PTR          queueHead;
    gcsEVENT_QUEUE_PTR          queueTail;
    gcsEVENT_QUEUE_PTR          freeList;
    gcsEVENT_QUEUE              repoList[gcdREPO_LIST_COUNT];
    gctPOINTER                  eventListMutex;

    gctPOINTER                  submitTimer;

#if gcdINTERRUPT_STATISTIC
    gctPOINTER                  interruptCount;
#endif

    gctINT                      notifyState;
};

/* Free all events belonging to a process. */
gceSTATUS
gckEVENT_FreeProcess(
    IN gckEVENT Event,
    IN gctUINT32 ProcessID
    );

gceSTATUS
gckEVENT_Stop(
    IN gckEVENT Event,
    IN gctUINT32 ProcessID,
    IN gctPHYS_ADDR Handle,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctUINT32 Address,
    IN gctSIGNAL Signal,
    IN OUT gctUINT32 * waitSize
    );

typedef struct _gcsLOCK_INFO * gcsLOCK_INFO_PTR;
typedef struct _gcsLOCK_INFO
{
    gctUINT32                   GPUAddresses[gcdMAX_GPU_COUNT];
    gctPOINTER                  pageTables[gcdMAX_GPU_COUNT];
    gctUINT32                   lockeds[gcdMAX_GPU_COUNT];
    gckKERNEL                   lockKernels[gcdMAX_GPU_COUNT];
    gckMMU                      lockMmus[gcdMAX_GPU_COUNT];
}
gcsLOCK_INFO;

typedef struct _gcsGPU_MAP * gcsGPU_MAP_PTR;
typedef struct _gcsGPU_MAP
{
    gctINT                      pid;
    gcsLOCK_INFO                lockInfo;
    gcsGPU_MAP_PTR              prev;
    gcsGPU_MAP_PTR              next;
}
gcsGPU_MAP;

/* gcuVIDMEM_NODE structure. */
typedef union _gcuVIDMEM_NODE
{
    /* Allocated from gckVIDMEM. */
    struct _gcsVIDMEM_NODE_VIDMEM
    {
        /* Owner of this node. */
        gckVIDMEM               memory;

        /* Dual-linked list of nodes. */
        gcuVIDMEM_NODE_PTR      next;
        gcuVIDMEM_NODE_PTR      prev;

        /* Dual linked list of free nodes. */
        gcuVIDMEM_NODE_PTR      nextFree;
        gcuVIDMEM_NODE_PTR      prevFree;

        /* Information for this node. */
        gctSIZE_T               offset;
        gctSIZE_T               bytes;
        gctUINT32               alignment;

#ifdef __QNXNTO__
        /* Client virtual address. */
        gctPOINTER              logical;
#endif

        /* Locked counter. */
        gctINT32                locked;

        /* Memory pool. */
        gcePOOL                 pool;
        gctUINT32               physical;

        /* Process ID owning this memory. */
        gctUINT32               processID;

#if gcdENABLE_VG
        gctPOINTER              kernelVirtual;
#endif
    }
    VidMem;

    /* Allocated from gckOS. */
    struct _gcsVIDMEM_NODE_VIRTUAL
    {
        /* Pointer to gckKERNEL object. */
        gckKERNEL               kernel;

        /* Information for this node. */
        /* Contiguously allocated? */
        gctBOOL                 contiguous;
        /* mdl record pointer... a kmalloc address. Process agnostic. */
        gctPHYS_ADDR            physical;
        gctSIZE_T               bytes;
        /* do_mmap_pgoff address... mapped per-process. */
        gctPOINTER              logical;

#if gcdENABLE_VG
        /* Physical address of this node, only meaningful when it is contiguous. */
        gctUINT64               physicalAddress;

        /* Kernel logical of this node. */
        gctPOINTER              kernelVirtual;
#endif

        /* Customer private handle */
        gctUINT32               gid;

        /* Page table information. */
        /* Used only when node is not contiguous */
        gctSIZE_T               pageCount;

        /* Used only when node is not contiguous */
        gctPOINTER              pageTables[gcdMAX_GPU_COUNT];
        /* Actual physical address */
        gctUINT32               addresses[gcdMAX_GPU_COUNT];

        /* Locked counter. */
        gctINT32                lockeds[gcdMAX_GPU_COUNT];

        /* Surface type. */
        gceSURF_TYPE            type;

        /* Secure GPU virtual address. */
        gctBOOL                 secure;

        gctBOOL                 onFault;

        gcsLISTHEAD             head;
    }
    Virtual;
}
gcuVIDMEM_NODE;

/* gckVIDMEM object. */
struct _gckVIDMEM
{
    /* Object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* mdl record pointer... a kmalloc address. Process agnostic. */
    gctPHYS_ADDR                physical;

    /* Information for this video memory heap. */
    gctUINT32                   baseAddress;
    gctSIZE_T                   bytes;
    gctSIZE_T                   freeBytes;
    gctSIZE_T                   minFreeBytes;

    /* Mapping for each type of surface. */
    gctINT                      mapping[gcvSURF_NUM_TYPES];

    /* Sentinel nodes for up to 8 banks. */
    gcuVIDMEM_NODE              sentinel[8];

    /* Allocation threshold. */
    gctSIZE_T                   threshold;

    /* The heap mutex. */
    gctPOINTER                  mutex;
};

typedef struct _gcsVIDMEM_NODE
{
    _VIV_VIDMEM_METADATA        metadata;

    /* Pointer to gcuVIDMEM_NODE. */
    gcuVIDMEM_NODE_PTR          node;

    /* Pointer to gckKERNEL object. */
    gckKERNEL                   kernel;

    /* Mutex to protect node. */
    gctPOINTER                  mutex;

    /* Reference count. */
    gctPOINTER                  reference;

    /* Name for client to import. */
    gctUINT32                   name;

    /* dma_buf */
    gctPOINTER                  dmabuf;

#if gcdPROCESS_ADDRESS_SPACE
    /* Head of mapping list. */
    gcsGPU_MAP_PTR              mapHead;

    /* Tail of mapping list. */
    gcsGPU_MAP_PTR              mapTail;

    gctPOINTER                  mapMutex;
#endif

    /* Surface Type. */
    gceSURF_TYPE                type;

    /* Pool from which node is allocated. */
    gcePOOL                     pool;

    gcsFENCE_SYNC               sync[gcvENGINE_GPU_ENGINE_COUNT];

    /* For DRM usage */
    gctUINT64                   timeStamp;
    gckVIDMEM_NODE              tsNode;
    gctUINT32                   tilingMode;
    gctUINT32                   tsMode;
    gctUINT64                   clearValue;
}
gcsVIDMEM_NODE;

typedef struct _gcsVIDMEM_HANDLE * gckVIDMEM_HANDLE;
typedef struct _gcsVIDMEM_HANDLE
{
    /* Pointer to gckVIDMEM_NODE. */
    gckVIDMEM_NODE              node;

    /* Handle for current process. */
    gctUINT32                   handle;

    /* Reference count for this handle. */
    gctPOINTER                  reference;
}
gcsVIDMEM_HANDLE;

typedef struct _gcsSHBUF * gcsSHBUF_PTR;
typedef struct _gcsSHBUF
{
    /* ID. */
    gctUINT32                   id;

    /* Reference count. */
    gctPOINTER                  reference;

    /* Data size. */
    gctUINT32                   size;

    /* Data. */
    gctPOINTER                  data;
}
gcsSHBUF;

typedef struct _gcsCORE_INFO
{
    gceHARDWARE_TYPE            type;
    gceCORE                     core;
    gckKERNEL                   kernel;
    gctUINT                     chipID;
}
gcsCORE_INFO;

typedef struct _gcsCORE_LIST
{
    gckKERNEL                   kernels[gcvCORE_COUNT];
    gctUINT32                   num;
}
gcsCORE_LIST;

/* A gckDEVICE is a group of cores (gckKERNEL in software). */
typedef struct _gcsDEVICE
{
    gcsCORE_INFO                coreInfoArray[gcvCORE_COUNT];
    gctUINT32                   coreNum;
    gcsCORE_LIST                map[gcvHARDWARE_NUM_TYPES];
    gceHARDWARE_TYPE            defaultHwType;

    gckOS                       os;

    /* Process resource database. */
    gckDB                       database;

    /* Same hardware type shares one MMU. */
    gckMMU                      mmus[gcvHARDWARE_NUM_TYPES];

    /* Mutex to make sure stuck dump for multiple cores doesn't interleave. */
    gctPOINTER                  stuckDumpMutex;

    /* Mutex for multi-core combine mode command submission */
    gctPOINTER                  commitMutex;
}
gcsDEVICE;

gceSTATUS
gckVIDMEM_HANDLE_Allocate(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node,
    OUT gctUINT32 * Handle
    );

gceSTATUS
gckVIDMEM_HANDLE_Reference(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle
    );

gceSTATUS
gckVIDMEM_HANDLE_Dereference(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle
    );

gceSTATUS
gckVIDMEM_NODE_Allocate(
    IN gckKERNEL Kernel,
    IN gcuVIDMEM_NODE_PTR VideoNode,
    IN gceSURF_TYPE Type,
    IN gcePOOL Pool,
    IN gctUINT32 * Handle
    );

gceSTATUS
gckVIDMEM_NODE_LockCPU(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    OUT gctPOINTER * Logical
    );

gceSTATUS
gckVIDMEM_NODE_UnlockCPU(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    OUT gctPOINTER Logical
    );

gceSTATUS
gckVIDMEM_Node_Lock(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node,
    OUT gctUINT32 *Address
    );

gceSTATUS
gckVIDMEM_NODE_Unlock(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node,
    IN gctUINT32 ProcessID
    );

gceSTATUS
gckVIDMEM_NODE_Reference(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node
    );

gceSTATUS
gckVIDMEM_NODE_Dereference(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node
    );

gceSTATUS
gckVIDMEM_NODE_Export(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    IN gctINT32 Flags,
    OUT gctPOINTER *DmaBuf,
    OUT gctINT32 *FD
    );

gceSTATUS
gckVIDMEM_NODE_Name(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    OUT gctUINT32 * Name
    );

gceSTATUS
gckVIDMEM_NODE_Import(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name,
    OUT gctUINT32 * Handle
    );

gceSTATUS
gckVIDMEM_NODE_GetFd(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    OUT gctINT * Fd
    );

gceSTATUS
gckVIDMEM_HANDLE_LookupAndReference(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    OUT gckVIDMEM_NODE * Node
    );

gceSTATUS
gckVIDMEM_HANDLE_Lookup(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle,
    OUT gckVIDMEM_NODE * Node
    );

gceSTATUS
gckVIDMEM_NODE_WrapUserMemory(
    IN gckKERNEL Kernel,
    IN gcsUSER_MEMORY_DESC_PTR Desc,
    OUT gctUINT32 * Handle,
    OUT gctUINT64 * Bytes
    );

gceSTATUS
gckVIDMEM_FindVIDMEM(
    IN gckKERNEL Kernel,
    IN gctUINT32 HardwareAddress,
    OUT gcuVIDMEM_NODE_PTR * Node,
    OUT gctUINT32_PTR PageTableEntryValue
    );

gceSTATUS
gckVIDMEM_QueryNodes(
    IN gckKERNEL Kernel,
    IN gcePOOL   Pool,
    OUT gctINT32 *Count,
    OUT gcuVIDMEM_NODE_PTR *Nodes
    );

#if gcdPROCESS_ADDRESS_SPACE
gceSTATUS
gckEVENT_DestroyMmu(
    IN gckEVENT Event,
    IN gckMMU Mmu,
    IN gceKERNEL_WHERE FromWhere
    );
#endif

typedef struct _gcsADDRESS_AREA * gcsADDRESS_AREA_PTR;
typedef struct _gcsADDRESS_AREA
{
    /* Page table information. */
    gctSIZE_T                   pageTableSize;
    gctPHYS_ADDR                pageTablePhysical;
    gctUINT32_PTR               pageTableLogical;
    gctUINT32                   pageTableEntries;

    /* Free entries. */
    gctUINT32                   heapList;
    gctBOOL                     freeNodes;

    gctUINT32                   dynamicMappingStart;
    gctUINT32                   dynamicMappingEnd;

    gctUINT32_PTR               mapLogical;
}
gcsADDRESS_AREA;

/* gckMMU object. */
struct _gckMMU
{
    /* The object. */
    gcsOBJECT                   object;

    /* Pointer to gckOS object. */
    gckOS                       os;

    /* Pointer to gckHARDWARE object. */
    gckHARDWARE                 hardware;

    /* The page table mutex. */
    gctPOINTER                  pageTableMutex;

    /* Master TLB information. */
    gctSIZE_T                   mtlbSize;
    gctPHYS_ADDR                mtlbPhysical;
    gctUINT32_PTR               mtlbLogical;
    gctUINT32                   mtlbEntries;

    gctPOINTER                  staticSTLB;
    gctBOOL                     enabled;

#if gcdPROCESS_ADDRESS_SPACE
    gctPOINTER                  pageTableDirty[gcdMAX_GPU_COUNT];
    gctPOINTER                  stlbs;
#endif

    gctPOINTER                  safePageLogical;
    gctPHYS_ADDR                safePagePhysical;
    gctUINT32                   safeAddress;
    gctSIZE_T                   safePageSize;

    /* physBase,physSize flat mapping area. */
    gctUINT32                   flatMappingRangeCount;
    gcsFLAT_MAPPING_RANGE       flatMappingRanges[gcdMAX_FLAT_MAPPING_COUNT];

    /* List of hardware which uses this MMU. */
    gcsLISTHEAD                 hardwareList;

    struct _gckQUEUE            recentFreedAddresses;

    gcsADDRESS_AREA             area[gcvADDRESS_AREA_COUNT];

    gctUINT32                   contiguousBaseAddress;
    gctUINT32                   externalBaseAddress;
};

typedef struct _gcsASYNC_COMMAND
{
    gckOS                           os;
    gckHARDWARE                     hardware;
    gckKERNEL                       kernel;

    gctPOINTER                      mutex;
    gcsFE                           fe;

    gctUINT32                       reservedTail;
    gctUINT64                       commitStamp;

    gckFENCE                        fence;

    gctPOINTER                      kList;
}
gcsASYNC_COMMAND;

gceSTATUS
gckOS_CreateKernelMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical
    );

gceSTATUS
gckOS_DestroyKernelMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical
    );

gceSTATUS
gckOS_CreateKernelVirtualMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical,
    OUT gctSIZE_T * PageCount
    );

gceSTATUS
gckOS_DestroyKernelVirtualMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctPOINTER Logical
    );

gceSTATUS
gckOS_CreateUserVirtualMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER * Logical,
    OUT gctSIZE_T * PageCount
    );

gceSTATUS
gckOS_DestroyUserVirtualMapping(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T Bytes,
    IN gctPOINTER Logical
    );

gceSTATUS
gckOS_GetFd(
    IN gctSTRING Name,
    IN gcsFDPRIVATE_PTR Private,
    OUT gctINT *Fd
    );

/*******************************************************************************
**
**  gckOS_ReadMappedPointer
**
**  Read pointer mapped from user pointer which returned by gckOS_MapUserPointer.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gctPOINTER Address
**          Pointer returned by gckOS_MapUserPointer.
**
**      gctUINT32_PTR Data
**          Pointer to hold 32 bits data.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckOS_ReadMappedPointer(
    IN gckOS Os,
    IN gctPOINTER Address,
    IN gctUINT32_PTR Data
    );

gceSTATUS
gckKERNEL_AllocateVirtualCommandBuffer(
    IN gckKERNEL Kernel,
    IN gctBOOL InUserSpace,
    IN OUT gctSIZE_T * Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctPOINTER * Logical
    );

gceSTATUS
gckKERNEL_DestroyVirtualCommandBuffer(
    IN gckKERNEL Kernel,
    IN gctSIZE_T Bytes,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical
    );

gceSTATUS
gckKERNEL_AllocateVirtualMemory(
    IN gckKERNEL Kernel,
    IN gctBOOL NonPaged,
    IN gctBOOL InUserSpace,
    IN OUT gctSIZE_T * Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctPOINTER * Logical
    );

gceSTATUS
gckKERNEL_FreeVirtualMemory(
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical,
    IN gctBOOL NonPaged
    );

gceSTATUS
gckKERNEL_GetGPUAddress(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    IN gctBOOL InUserSpace,
    IN gctPHYS_ADDR Physical,
    OUT gctUINT32 * Address
    );

gceSTATUS
gckKERNEL_QueryGPUAddress(
    IN gckKERNEL Kernel,
    IN gctUINT32 GpuAddress,
    OUT gckVIRTUAL_COMMAND_BUFFER_PTR * Buffer
    );

gceSTATUS
gckKERNEL_AttachProcess(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach
    );

gceSTATUS
gckKERNEL_AttachProcessEx(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach,
    IN gctUINT32 PID
    );

#if gcdSECURE_USER
gceSTATUS
gckKERNEL_MapLogicalToPhysical(
    IN gckKERNEL Kernel,
    IN gcskSECURE_CACHE_PTR Cache,
    IN OUT gctPOINTER * Data
    );

gceSTATUS
gckKERNEL_FlushTranslationCache(
    IN gckKERNEL Kernel,
    IN gcskSECURE_CACHE_PTR Cache,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    );
#endif

gceSTATUS
gckHARDWARE_QueryIdle(
    IN gckHARDWARE Hardware,
    OUT gctBOOL_PTR IsIdle
    );

gceSTATUS
gckHARDWARE_WaitFence(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT64 FenceData,
    IN gctUINT32 FenceAddress,
    OUT gctUINT32 *Bytes
    );

gceSTATUS
gckHARDWARE_AddressInHardwareFuncions(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Address,
    OUT gctPOINTER *Pointer
    );

gceSTATUS
gckHARDWARE_UpdateContextID(
    IN gckHARDWARE Hardware
    );

#if gcdSECURITY
gceSTATUS
gckKERNEL_SecurityOpen(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPU,
    OUT gctUINT32 *Channel
    );

/*
** Close a security service channel
*/
gceSTATUS
gckKERNEL_SecurityClose(
    IN gctUINT32 Channel
    );

/*
** Security service interface.
*/
gceSTATUS
gckKERNEL_SecurityCallService(
    IN gctUINT32 Channel,
    IN OUT gcsTA_INTERFACE * Interface
    );

gceSTATUS
gckKERNEL_SecurityStartCommand(
    IN gckKERNEL Kernel
    );

gceSTATUS
gckKERNEL_SecurityAllocateSecurityMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 Bytes,
    OUT gctUINT32 * Handle
    );

gceSTATUS
gckKERNEL_SecurityExecute(
    IN gckKERNEL Kernel,
    IN gctPOINTER Buffer,
    IN gctUINT32 Bytes
    );

gceSTATUS
gckKERNEL_SecurityMapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 *PhysicalArray,
    IN gctUINT32 PageCount,
    OUT gctUINT32 * GPUAddress
    );

gceSTATUS
gckKERNEL_SecurityUnmapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPUAddress,
    IN gctUINT32 PageCount
    );

#endif

#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckKERNEL_SecurityOpen(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPU,
    OUT gctUINT32 *Channel
    );

/*
** Close a security service channel
*/
gceSTATUS
gckKERNEL_SecurityClose(
    IN gctUINT32 Channel
    );

/*
** Security service interface.
*/
gceSTATUS
gckKERNEL_SecurityCallService(
    IN gctUINT32 Channel,
    IN OUT gcsTA_INTERFACE * Interface
    );

gceSTATUS
gckKERNEL_SecurityStartCommand(
    IN gckKERNEL Kernel,
    IN gctUINT32 Address,
    IN gctUINT32 Bytes
    );

gceSTATUS
gckKERNEL_SecurityMapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 *PhysicalArray,
    IN gctPHYS_ADDR_T Physical,
    IN gctUINT32 PageCount,
    OUT gctUINT32 * GPUAddress
    );

gceSTATUS
gckKERNEL_SecurityUnmapMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 GPUAddress,
    IN gctUINT32 PageCount
    );

gceSTATUS
gckKERNEL_SecurityDumpMMUException(
    IN gckKERNEL Kernel
    );

gceSTATUS
gckKERNEL_ReadMMUException(
    IN gckKERNEL Kernel,
    IN gctUINT32_PTR MMUStatus,
    IN gctUINT32_PTR MMUException
    );

gceSTATUS
gckKERNEL_HandleMMUException(
    IN gckKERNEL Kernel,
    IN gctUINT32 MMUStatus,
    IN gctPHYS_ADDR_T Physical,
    IN gctUINT32 GPUAddres
    );
#endif

gceSTATUS
gckKERNEL_CreateShBuffer(
    IN gckKERNEL Kernel,
    IN gctUINT32 Size,
    OUT gctSHBUF * ShBuf
    );

gceSTATUS
gckKERNEL_DestroyShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    );

gceSTATUS
gckKERNEL_MapShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    );

gceSTATUS
gckKERNEL_WriteShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount
    );

gceSTATUS
gckKERNEL_ReadShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount,
    OUT gctUINT32 * BytesRead
    );


/******************************************************************************\
******************************* gckCONTEXT Object *******************************
\******************************************************************************/

gceSTATUS
gckCONTEXT_Construct(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gctUINT32 ProcessID,
    OUT gckCONTEXT * Context
    );

gceSTATUS
gckCONTEXT_Destroy(
    IN gckCONTEXT Context
    );

gceSTATUS
gckCONTEXT_Update(
    IN gckCONTEXT Context,
    IN gctUINT32 ProcessID,
    IN gcsSTATE_DELTA_PTR StateDelta
    );

gceSTATUS
gckCONTEXT_MapBuffer(
    IN gckCONTEXT Context,
    OUT gctUINT32 *Physicals,
    OUT gctUINT64 *Logicals,
    OUT gctUINT32 *Bytes
    );

void
gckQUEUE_Enqueue(
    IN gckQUEUE LinkQueue,
    IN gcuQUEUEDATA *Data
    );

void
gckQUEUE_GetData(
    IN gckQUEUE LinkQueue,
    IN gctUINT32 Index,
    OUT gcuQUEUEDATA ** Data
    );

gceSTATUS
gckQUEUE_Allocate(
    IN gckOS Os,
    IN gckQUEUE Queue,
    IN gctUINT32 Size
    );

gceSTATUS
gckQUEUE_Free(
    IN gckOS Os,
    IN gckQUEUE Queue
    );

/******************************************************************************\
****************************** gckRECORDER Object ******************************
\******************************************************************************/
gceSTATUS
gckRECORDER_Construct(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    OUT gckRECORDER * Recorder
    );

gceSTATUS
gckRECORDER_Destory(
    IN gckOS Os,
    IN gckRECORDER Recorder
    );

void
gckRECORDER_AdvanceIndex(
    gckRECORDER Recorder,
    gctUINT64   CommitStamp
    );

void
gckRECORDER_Record(
    gckRECORDER Recorder,
    gctUINT8_PTR CommandBuffer,
    gctUINT32 CommandBytes,
    gctUINT8_PTR ContextBuffer,
    gctUINT32 ContextBytes
    );

void
gckRECORDER_Dump(
    gckRECORDER Recorder
    );

gceSTATUS
gckRECORDER_UpdateMirror(
    gckRECORDER Recorder,
    gctUINT32 State,
    gctUINT32 Data
    );

/******************************************************************************\
*************************** gckASYNC_COMMAND Object ****************************
\******************************************************************************/
gceSTATUS
gckASYNC_COMMAND_Construct(
    IN gckKERNEL Kernel,
    OUT gckASYNC_COMMAND * Command
    );

gceSTATUS
gckASYNC_COMMAND_Destroy(
    IN gckASYNC_COMMAND Command
    );

gceSTATUS
gckASYNC_COMMAND_Commit(
    IN gckASYNC_COMMAND Command,
    IN gcoCMDBUF CommandBuffer,
    IN gcsQUEUE_PTR EventQueue
    );

gceSTATUS
gckASYNC_COMMAND_EnterCommit(
    IN gckASYNC_COMMAND Command
    );

gceSTATUS
gckASYNC_COMMAND_ExitCommit(
    IN gckASYNC_COMMAND Command
    );

gceSTATUS
gckASYNC_COMMAND_Execute(
    IN gckASYNC_COMMAND Command,
    IN gctUINT32 Start,
    IN gctUINT32 End
    );

void
gcsLIST_Init(
    gcsLISTHEAD_PTR Node
    );

void
gcsLIST_Add(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    );

void
gcsLIST_AddTail(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    );

void
gcsLIST_Del(
    gcsLISTHEAD_PTR Node
    );

gctBOOL
gcsLIST_Empty(
    gcsLISTHEAD_PTR Head
    );

#define gcmkLIST_FOR_EACH(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

#define gcmkLIST_FOR_EACH_SAFE(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
        pos = n, n = pos->next)

gceSTATUS
gckFENCE_Create(
    IN gckOS Os,
    IN gckKERNEL Kernel,
    OUT gckFENCE * Fence
    );

gceSTATUS
gckFENCE_Destory(
    IN gckOS Os,
    OUT gckFENCE Fence
    );

gceSTATUS
gckFENCE_Signal(
    IN gckOS Os,
    IN gckFENCE Fence
    );

gceSTATUS
gckDEVICE_Construct(
    IN gckOS Os,
    OUT gckDEVICE * Device
    );

gceSTATUS
gckDEVICE_AddCore(
    IN gckDEVICE Device,
    IN gceCORE Core,
    IN gctUINT chipID,
    IN gctPOINTER Context,
    IN gckKERNEL * Kernel
    );

gceSTATUS
gckDEVICE_Destroy(
    IN gckOS Os,
    IN gckDEVICE Device
    );

gceSTATUS
gckDEVICE_Dispatch(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    );

gceSTATUS
gckDEVICE_GetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU *Mmu
    );

gceSTATUS
gckDEVICE_SetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU Mmu
    );

gceSTATUS
gckDEVICE_QueryGPUAddress(
    IN gckDEVICE Device,
    IN gckKERNEL Kernel,
    IN gctUINT32 GPUAddress,
    OUT gckVIRTUAL_COMMAND_BUFFER_PTR * Buffer
    );

#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckKERNEL_MapInTrustApplicaiton(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical,
    IN gctUINT32 GPUAddress,
    IN gctSIZE_T PageCount
    );
#endif

#if gcdSECURITY || gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckOS_OpenSecurityChannel(
    IN gckOS Os,
    IN gceCORE Core,
    OUT gctUINT32 *Channel
    );

gceSTATUS
gckOS_CloseSecurityChannel(
    IN gctUINT32 Channel
    );

gceSTATUS
gckOS_CallSecurityService(
    IN gctUINT32 Channel,
    IN gcsTA_INTERFACE * Interface
    );

gceSTATUS
gckOS_InitSecurityChannel(
    OUT gctUINT32 Channel
    );

gceSTATUS
gckOS_AllocatePageArray(
    IN gckOS Os,
    IN gctPHYS_ADDR Physical,
    IN gctSIZE_T PageCount,
    OUT gctPOINTER * PageArrayLogical,
    OUT gctPHYS_ADDR * PageArrayPhysical
    );
#endif

#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_kernel_h_ */
