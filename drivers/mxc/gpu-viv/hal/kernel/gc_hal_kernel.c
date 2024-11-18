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


#include "gc_hal_kernel_precomp.h"

#if gcdDEC_ENABLE_AHB
#include "viv_dec300_main.h"
#endif

#define _GC_OBJ_ZONE    gcvZONE_KERNEL

/*******************************************************************************
***** Version Signature *******************************************************/

#define _gcmTXT2STR(t) #t
#define gcmTXT2STR(t) _gcmTXT2STR(t)
const char * _VERSION = "\n\0$VERSION$"
                        gcmTXT2STR(gcvVERSION_MAJOR) "."
                        gcmTXT2STR(gcvVERSION_MINOR) "."
                        gcmTXT2STR(gcvVERSION_PATCH) ":"
                        gcmTXT2STR(gcvVERSION_BUILD) "$\n";

/******************************************************************************\
******************************* gckKERNEL API Code ******************************
\******************************************************************************/

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
#define gcmDEFINE2TEXT(d) #d
gctCONST_STRING _DispatchText[] =
{
    gcmDEFINE2TEXT(gcvHAL_QUERY_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_IDENTITY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_FREQUENCY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_NON_PAGED_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_FREE_NON_PAGED_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_CONTIGUOUS_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_FREE_CONTIGUOUS_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_RELEASE_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_MAP_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNMAP_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_MAP_USER_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNMAP_USER_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_LOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_UNLOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_EVENT_COMMIT),
    gcmDEFINE2TEXT(gcvHAL_USER_SIGNAL),
    gcmDEFINE2TEXT(gcvHAL_SIGNAL),
    gcmDEFINE2TEXT(gcvHAL_WRITE_DATA),
    gcmDEFINE2TEXT(gcvHAL_COMMIT),
    gcmDEFINE2TEXT(gcvHAL_STALL),
    gcmDEFINE2TEXT(gcvHAL_READ_REGISTER),
    gcmDEFINE2TEXT(gcvHAL_WRITE_REGISTER),
    gcmDEFINE2TEXT(gcvHAL_GET_PROFILE_SETTING),
    gcmDEFINE2TEXT(gcvHAL_SET_PROFILE_SETTING),
    gcmDEFINE2TEXT(gcvHAL_PROFILE_REGISTERS_2D),
    gcmDEFINE2TEXT(gcvHAL_READ_ALL_PROFILE_REGISTERS_PART1),
    gcmDEFINE2TEXT(gcvHAL_READ_ALL_PROFILE_REGISTERS_PART2),
    gcmDEFINE2TEXT(gcvHAL_READ_PROFILER_REGISTER_SETTING),
    gcmDEFINE2TEXT(gcvHAL_SET_POWER_MANAGEMENT_STATE),
    gcmDEFINE2TEXT(gcvHAL_QUERY_POWER_MANAGEMENT_STATE),
    gcmDEFINE2TEXT(gcvHAL_GET_BASE_ADDRESS),
    gcmDEFINE2TEXT(gcvHAL_SET_IDLE),
    gcmDEFINE2TEXT(gcvHAL_QUERY_KERNEL_SETTINGS),
    gcmDEFINE2TEXT(gcvHAL_RESET),
    gcmDEFINE2TEXT(gcvHAL_MAP_PHYSICAL),
    gcmDEFINE2TEXT(gcvHAL_DEBUG),
    gcmDEFINE2TEXT(gcvHAL_CACHE),
    gcmDEFINE2TEXT(gcvHAL_TIMESTAMP),
    gcmDEFINE2TEXT(gcvHAL_DATABASE),
    gcmDEFINE2TEXT(gcvHAL_VERSION),
    gcmDEFINE2TEXT(gcvHAL_CHIP_INFO),
    gcmDEFINE2TEXT(gcvHAL_ATTACH),
    gcmDEFINE2TEXT(gcvHAL_DETACH),
    gcmDEFINE2TEXT(gcvHAL_SET_TIMEOUT),
    gcmDEFINE2TEXT(gcvHAL_GET_FRAME_INFO),
    gcmDEFINE2TEXT(gcvHAL_DUMP_GPU_PROFILE),
    gcmDEFINE2TEXT(gcvHAL_QUERY_COMMAND_BUFFER),
    gcmDEFINE2TEXT(gcvHAL_COMMIT_DONE),
    gcmDEFINE2TEXT(gcvHAL_DUMP_GPU_STATE),
    gcmDEFINE2TEXT(gcvHAL_DUMP_EVENT),
    gcmDEFINE2TEXT(gcvHAL_ALLOCATE_VIRTUAL_COMMAND_BUFFER),
    gcmDEFINE2TEXT(gcvHAL_FREE_VIRTUAL_COMMAND_BUFFER),
    gcmDEFINE2TEXT(gcvHAL_SET_FSCALE_VALUE),
    gcmDEFINE2TEXT(gcvHAL_GET_FSCALE_VALUE),
    gcmDEFINE2TEXT(gcvHAL_EXPORT_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_NAME_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_IMPORT_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_RESET_TIME_STAMP),
    gcmDEFINE2TEXT(gcvHAL_READ_REGISTER_EX),
    gcmDEFINE2TEXT(gcvHAL_WRITE_REGISTER_EX),
    gcmDEFINE2TEXT(gcvHAL_CREATE_NATIVE_FENCE),
    gcmDEFINE2TEXT(gcvHAL_WAIT_NATIVE_FENCE),
    gcmDEFINE2TEXT(gcvHAL_DESTROY_MMU),
    gcmDEFINE2TEXT(gcvHAL_SHBUF),
    gcmDEFINE2TEXT(gcvHAL_GET_GRAPHIC_BUFFER_FD),
    gcmDEFINE2TEXT(gcvHAL_SET_VIDEO_MEMORY_METADATA),
    gcmDEFINE2TEXT(gcvHAL_GET_VIDEO_MEMORY_FD),
    gcmDEFINE2TEXT(gcvHAL_CONFIG_POWER_MANAGEMENT),
    gcmDEFINE2TEXT(gcvHAL_WRAP_USER_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_WAIT_FENCE),
#if gcdDEC_ENABLE_AHB
    gcmDEFINE2TEXT(gcvHAL_DEC300_READ),
    gcmDEFINE2TEXT(gcvHAL_DEC300_WRITE),
    gcmDEFINE2TEXT(gcvHAL_DEC300_FLUSH),
    gcmDEFINE2TEXT(gcvHAL_DEC300_FLUSH_WAIT),
#endif
    gcmDEFINE2TEXT(gcvHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY),
    gcmDEFINE2TEXT(gcvHAL_QUERY_CHIP_OPTION),
};
#endif

#if gcdGPU_TIMEOUT && gcdINTERRUPT_STATISTIC
void
_MonitorTimerFunction(
    gctPOINTER Data
    )
{
    gckKERNEL kernel = (gckKERNEL)Data;
    gctINT32 pendingInterrupt;
    gctBOOL reset = gcvFALSE;
    gctINT32 mask;
    gctUINT32 advance = kernel->timeOut/2;

#if gcdENABLE_VG
    if (kernel->core == gcvCORE_VG)
    {
        return;
    }
#endif

    if (kernel->monitorTimerStop)
    {
        /* Stop. */
        return;
    }

    gckOS_AtomGet(kernel->os, kernel->eventObj->interruptCount, &pendingInterrupt);

    if (pendingInterrupt < 0)
    {
        gctINT i = 0 - pendingInterrupt;
        gctINT pendingMask;

        gcmkVERIFY_OK(gckOS_AtomGet(
            kernel->os,
            kernel->hardware->pendingEvent,
            &pendingMask
            ));

        gcmkPRINT("[galcore]: Number of pending interrupt is %d mask is %x",
                  pendingInterrupt, pendingMask);

        while (i--)
        {
            /* Ignore counting which should not exist. */
            gckOS_AtomIncrement(kernel->os, kernel->eventObj->interruptCount, &pendingInterrupt);
        }

        gckOS_AtomGet(kernel->os, kernel->eventObj->interruptCount, &pendingInterrupt);
    }

    if (kernel->monitoring == gcvFALSE)
    {
        if (pendingInterrupt)
        {
            /* Begin to mointor GPU state. */
            kernel->monitoring = gcvTRUE;

            /* Record current state. */
            kernel->lastCommitStamp = kernel->eventObj->lastCommitStamp;
            kernel->restoreAddress  = kernel->hardware->lastWaitLink;
            gcmkVERIFY_OK(gckOS_AtomGet(
                kernel->os,
                kernel->hardware->pendingEvent,
                &kernel->restoreMask
                ));

            /* Clear timeout. */
            kernel->timer = 0;
        }
    }
    else
    {
        if (pendingInterrupt)
        {
            gcmkVERIFY_OK(gckOS_AtomGet(
                kernel->os,
                kernel->hardware->pendingEvent,
                &mask
                ));

            if (kernel->eventObj->lastCommitStamp == kernel->lastCommitStamp
             && kernel->hardware->lastWaitLink    == kernel->restoreAddress
             && mask                              == kernel->restoreMask
            )
            {
                /* GPU state is not changed, accumlate timeout. */
                kernel->timer += advance;

                if (kernel->timer >= kernel->timeOut)
                {
                    /* GPU stuck, trigger reset. */
                    reset = gcvTRUE;
                }
            }
            else
            {
                /* GPU state changed, cancel current timeout.*/
                kernel->monitoring = gcvFALSE;
            }
        }
        else
        {
            /* GPU finish all jobs, cancel current timeout*/
            kernel->monitoring = gcvFALSE;
        }
    }

    if (reset)
    {
        gckKERNEL_Recovery(kernel);

        /* Work in this timeout is done. */
        kernel->monitoring = gcvFALSE;
    }

    gcmkVERIFY_OK(gckOS_StartTimer(kernel->os, kernel->monitorTimer, advance));
}
#endif

#if gcdPROCESS_ADDRESS_SPACE
gceSTATUS
_MapCommandBuffer(
    IN gckKERNEL Kernel
    )
{
    gceSTATUS status;
    gctUINT32 i;
    gctPHYS_ADDR_T physical;
    gctUINT32 address;
    gckMMU mmu;

    gcmkONERROR(gckKERNEL_GetProcessMMU(Kernel, &mmu));

    for (i = 0; i < gcdCOMMAND_QUEUES; i++)
    {
        gcmkONERROR(gckOS_GetPhysicalAddress(
            Kernel->os,
            Kernel->command->queues[i].logical,
            &physical
            ));

        gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, physical, &physical));

        gcmkSAFECASTPHYSADDRT(address, physical);

        gcmkONERROR(gckMMU_FlatMapping(mmu, address, 1));
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}
#endif

void
_DumpDriverConfigure(
    IN gckKERNEL Kernel
    )
{
    gcmkPRINT_N(0, "**************************\n");
    gcmkPRINT_N(0, "***   GPU DRV CONFIG   ***\n");
    gcmkPRINT_N(0, "**************************\n");

    gcmkPRINT("Galcore version %d.%d.%d.%d\n",
              gcvVERSION_MAJOR, gcvVERSION_MINOR, gcvVERSION_PATCH, gcvVERSION_BUILD);

    gckOS_DumpParam();
}

void
_DumpState(
    IN gckKERNEL Kernel
    )
{
    /* Dump GPU Debug registers. */
    gcmkVERIFY_OK(gckHARDWARE_DumpGPUState(Kernel->hardware));

    gcmkVERIFY_OK(gckCOMMAND_DumpExecutingBuffer(Kernel->command));

    /* Dump Pending event. */
    gcmkVERIFY_OK(gckEVENT_Dump(Kernel->eventObj));

    /* Dump Process DB. */
    gcmkVERIFY_OK(gckKERNEL_DumpProcessDB(Kernel));

#if gcdRECORD_COMMAND
    /* Dump record. */
    gckRECORDER_Dump(Kernel->command->recorder);
#endif
}

static gceHARDWARE_TYPE
_GetHardwareType(
    IN gckKERNEL Kernel
    )
{
    gceHARDWARE_TYPE type;
    gcmkHEADER();
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

#if gcdENABLE_VG
    if (Kernel->vg)
    {
        type =  gcvHARDWARE_VG;
    }
    else
#endif
    {
        type = Kernel->hardware->type;
    }

    gcmkFOOTER_ARG("type=%d", type);
    return type;
}

gceSTATUS
_SetRecovery(
    IN gckKERNEL Kernel,
    IN gctBOOL  Recovery,
    IN gctUINT32 StuckDump
    )
{
    Kernel->recovery = Recovery;

    if (Recovery == gcvFALSE)
    {
        /* Dump stuck information if Recovery is disabled. */
        Kernel->stuckDump = gcmMAX(StuckDump, gcvSTUCK_DUMP_USER_COMMAND);
    }

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckKERNEL_Construct
**
**  Construct a new gckKERNEL object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to an gckOS object.
**
**      gceCORE Core
**          Specified core.
**
**      IN gctPOINTER Context
**          Pointer to a driver defined context.
**
**      IN gckDB SharedDB,
**          Pointer to a shared DB.
**
**  OUTPUT:
**
**      gckKERNEL * Kernel
**          Pointer to a variable that will hold the pointer to the gckKERNEL
**          object.
*/

gceSTATUS
gckKERNEL_Construct(
    IN gckOS Os,
    IN gceCORE Core,
    IN gctUINT ChipID,
    IN gctPOINTER Context,
    IN gckDEVICE Device,
    IN gckDB SharedDB,
    OUT gckKERNEL * Kernel
    )
{
    gckKERNEL kernel = gcvNULL;
    gceSTATUS status;
    gctSIZE_T i;
    gctPOINTER pointer = gcvNULL;
    gctUINT32 recovery;
    gctUINT32 stuckDump;

    gcmkHEADER_ARG("Os=0x%x Context=0x%x", Os, Context);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

    /* Allocate the gckKERNEL object. */
    gcmkONERROR(gckOS_Allocate(Os,
                               gcmSIZEOF(struct _gckKERNEL),
                               &pointer));

    /* Zero the object. */
    gckOS_ZeroMemory(pointer, gcmSIZEOF(struct _gckKERNEL));

    kernel = pointer;

    /* Initialize the gckKERNEL object. */
    kernel->object.type = gcvOBJ_KERNEL;
    kernel->os          = Os;
    kernel->core        = Core;
    kernel->device      = Device;
    kernel->chipID      = ChipID;

#if gcdENABLE_TRUST_APPLICATION
    /* Connect to security service for this GPU. */
    gcmkONERROR(gckKERNEL_SecurityOpen(kernel, kernel->core, &kernel->securityChannel));
#endif

    if (SharedDB == gcvNULL)
    {
        gcmkONERROR(gckOS_Allocate(Os,
                                   gcmSIZEOF(struct _gckDB),
                                   &pointer));

        kernel->db               = pointer;
        kernel->dbCreated        = gcvTRUE;
        kernel->db->freeDatabase = gcvNULL;
        kernel->db->freeRecord   = gcvNULL;
        kernel->db->dbMutex      = gcvNULL;
        kernel->db->lastDatabase = gcvNULL;
        kernel->db->idleTime     = 0;
        kernel->db->lastIdle     = 0;
        kernel->db->lastSlowdown = 0;

        for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
        {
            kernel->db->db[i] = gcvNULL;
        }

        /* Construct a database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->dbMutex));

        /* Construct a video memory name database. */
        gcmkONERROR(gckKERNEL_CreateIntegerDatabase(kernel, &kernel->db->nameDatabase));

        /* Construct a video memory name database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->nameDatabaseMutex));

        /* Construct a pointer name database. */
        gcmkONERROR(gckKERNEL_CreateIntegerDatabase(kernel, &kernel->db->pointerDatabase));

        /* Construct a pointer name database mutex. */
        gcmkONERROR(gckOS_CreateMutex(Os, &kernel->db->pointerDatabaseMutex));

        /* Initialize on fault vidmem list. */
        gcsLIST_Init(&kernel->db->onFaultVidmemList);
    }
    else
    {
        kernel->db               = SharedDB;
        kernel->dbCreated        = gcvFALSE;
    }

    for (i = 0; i < gcmCOUNTOF(kernel->timers); ++i)
    {
        kernel->timers[i].startTime = 0;
        kernel->timers[i].stopTime = 0;
    }

    /* Save context. */
    kernel->context = Context;

    /* Construct atom holding number of clients. */
    kernel->atomClients = gcvNULL;
    gcmkONERROR(gckOS_AtomConstruct(Os, &kernel->atomClients));

    kernel->recovery  = gcvTRUE;
    kernel->stuckDump = gcvSTUCK_DUMP_NONE;

    /* Override default recovery and stuckDump setting. */
    status = gckOS_QueryOption(Os, "recovery", &recovery);

    if (gcmIS_SUCCESS(status))
    {
        status = gckOS_QueryOption(Os, "stuckDump", &stuckDump);

        gcmkASSERT(status == gcvSTATUS_OK);

        _SetRecovery(kernel, recovery, stuckDump);
    }

    /* Need the kernel reference before gckKERNEL_Construct() completes.
       gckOS_MapPagesEx() is called to map kernel virtual command buffers. */
    *Kernel = kernel;

    kernel->virtualBufferHead =
        kernel->virtualBufferTail = gcvNULL;

    gcmkONERROR(
        gckOS_CreateMutex(Os, (gctPOINTER)&kernel->virtualBufferLock));

#if gcdENABLE_VG
    kernel->vg = gcvNULL;

    if (Core == gcvCORE_VG)
    {
        gctUINT32 contiguousBase;
        gctUINT32 contiguousSize = 0;

        /* Construct the gckMMU object. */
        gcmkONERROR(
            gckVGKERNEL_Construct(Os, Context, kernel, &kernel->vg));

        kernel->timeOut = gcdGPU_TIMEOUT;

        status = gckOS_QueryOption(Os, "contiguousBase", &contiguousBase);

        if (gcmIS_SUCCESS(status))
        {
            status = gckOS_QueryOption(Os, "contiguousSize", &contiguousSize);
        }

        if (gcmIS_SUCCESS(status) && contiguousSize)
        {
            gctUINT64 gpuContiguousBase;

            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(Os, contiguousBase, &gpuContiguousBase));

            gcmkSAFECASTPHYSADDRT(kernel->contiguousBaseAddress, gpuContiguousBase);
        }
    }
    else
#endif
    {
        /* Construct the gckHARDWARE object. */
        gcmkONERROR(
            gckHARDWARE_Construct(Os, kernel->core, &kernel->hardware));

        /* Set pointer to gckKERNEL object in gckHARDWARE object. */
        kernel->hardware->kernel = kernel;

        kernel->timeOut = kernel->hardware->type == gcvHARDWARE_2D
                        ? gcdGPU_2D_TIMEOUT
                        : gcdGPU_TIMEOUT
                        ;

        /* Initialize virtual command buffer. */
#if gcdALLOC_CMD_FROM_RESERVE || gcdSECURITY || gcdDISABLE_GPU_VIRTUAL_ADDRESS || !USE_KERNEL_VIRTUAL_BUFFERS
        kernel->virtualCommandBuffer = gcvFALSE;
#else
        kernel->virtualCommandBuffer = kernel->hardware->options.enableMMU;
#endif

#if gcdSHARED_PAGETABLE
        /* Construct the gckMMU object. */
        gcmkONERROR(
            gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));
#else
        if (Device == gcvNULL)
        {
            /* Construct the gckMMU object. */
            gcmkONERROR(
                gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));
        }
        else
        {
            gcmkONERROR(gckDEVICE_GetMMU(Device, kernel->hardware->type, &kernel->mmu));

            if (kernel->mmu == gcvNULL)
            {
                gcmkONERROR(
                    gckMMU_Construct(kernel, gcdMMU_SIZE, &kernel->mmu));

                gcmkONERROR(
                    gckDEVICE_SetMMU(Device, kernel->hardware->type, kernel->mmu));
            }
        }

        gcmkVERIFY_OK(gckMMU_AttachHardware(kernel->mmu, kernel->hardware));
#endif

        kernel->contiguousBaseAddress = kernel->mmu->contiguousBaseAddress;
        kernel->externalBaseAddress   = kernel->mmu->externalBaseAddress;

        /* Construct the gckCOMMAND object. */
        gcmkONERROR(
            gckCOMMAND_Construct(kernel, &kernel->command));

        if (gckHARDWARE_IsFeatureAvailable(kernel->hardware, gcvFEATURE_ASYNC_BLIT))
        {
            /* Construct the gckASYNC_COMMAND object for BLT engine. */
            gcmkONERROR(gckASYNC_COMMAND_Construct(kernel, &kernel->asyncCommand));

            /* Construct gckEVENT for BLT. */
            gcmkONERROR(gckEVENT_Construct(kernel, &kernel->asyncEvent));

            kernel->asyncEvent->asyncCommand = kernel->asyncCommand;

            kernel->command->asyncCommand = kernel->asyncCommand;
        }

        /* Construct the gckEVENT object. */
        gcmkONERROR(
            gckEVENT_Construct(kernel, &kernel->eventObj));

        gcmkVERIFY_OK(gckOS_GetTime(&kernel->resetTimeStamp));

        gcmkONERROR(gckHARDWARE_PrepareFunctions(kernel->hardware));

        /* Initialize the hardware. */
        gcmkONERROR(
            gckHARDWARE_InitializeHardware(kernel->hardware));

#if gcdDVFS
        if (gckHARDWARE_IsFeatureAvailable(kernel->hardware,
                                           gcvFEATURE_DYNAMIC_FREQUENCY_SCALING))
        {
            gcmkONERROR(gckDVFS_Construct(kernel->hardware, &kernel->dvfs));
            gcmkONERROR(gckDVFS_Start(kernel->dvfs));
        }
#endif

#if COMMAND_PROCESSOR_VERSION == 1
        /* Start the command queue. */
        gcmkONERROR(gckCOMMAND_Start(kernel->command));
#endif
    }

#if VIVANTE_PROFILER
    /* Initialize profile setting */
    kernel->profileEnable = gcvFALSE;
    kernel->profileCleanRegister = gcvTRUE;
#endif

#if gcdLINUX_SYNC_FILE
    gcmkONERROR(gckOS_CreateSyncTimeline(Os, Core, &kernel->timeline));
#endif

#if gcdSECURITY
    /* Connect to security service for this GPU. */
    gcmkONERROR(gckKERNEL_SecurityOpen(kernel, kernel->core, &kernel->securityChannel));
#endif

#if gcdGPU_TIMEOUT && gcdINTERRUPT_STATISTIC
    if (kernel->timeOut)
    {
        gcmkVERIFY_OK(gckOS_CreateTimer(
            Os,
            (gctTIMERFUNCTION)_MonitorTimerFunction,
            (gctPOINTER)kernel,
            &kernel->monitorTimer
            ));

        kernel->monitoring  = gcvFALSE;

        kernel->monitorTimerStop = gcvFALSE;

        gcmkVERIFY_OK(gckOS_StartTimer(
            Os,
            kernel->monitorTimer,
            100
            ));
    }
#endif

    /* Return pointer to the gckKERNEL object. */
    *Kernel = kernel;

    /* Success. */
    gcmkFOOTER_ARG("*Kernel=0x%x", *Kernel);
    return gcvSTATUS_OK;

OnError:
    *Kernel = gcvNULL;

    if (kernel != gcvNULL)
    {
        gckKERNEL_Destroy(kernel);
    }

    /* Return the error. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_Destroy
**
**  Destroy an gckKERNEL object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object to destroy.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_Destroy(
    IN gckKERNEL Kernel
    )
{
    gctSIZE_T i;
    gcsDATABASE_PTR database, databaseNext;
    gcsDATABASE_RECORD_PTR record, recordNext;

    gcmkHEADER_ARG("Kernel=0x%x", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
#if QNX_SINGLE_THREADED_DEBUGGING
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->debugMutex));
#endif

    /* Destroy the database. */
    if (Kernel->dbCreated)
    {
        for (i = 0; i < gcmCOUNTOF(Kernel->db->db); ++i)
        {
            if (Kernel->db->db[i] != gcvNULL)
            {
                gcmkVERIFY_OK(
                    gckKERNEL_DestroyProcessDB(Kernel, Kernel->db->db[i]->processID));
            }
        }

        /* Free all databases. */
        for (database = Kernel->db->freeDatabase;
             database != gcvNULL;
             database = databaseNext)
        {
            databaseNext = database->next;

            if (database->counterMutex)
            {
                gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, database->counterMutex));
            }

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, database));
        }

        if (Kernel->db->lastDatabase != gcvNULL)
        {
            if (Kernel->db->lastDatabase->counterMutex)
            {
                gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->lastDatabase->counterMutex));
            }

            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel->db->lastDatabase));
        }

        /* Free all database records. */
        for (record = Kernel->db->freeRecord; record != gcvNULL; record = recordNext)
        {
            recordNext = record->next;
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, record));
        }

        if (Kernel->db->dbMutex)
        {
            /* Destroy the database mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->dbMutex));
        }

        if (Kernel->db->nameDatabase)
        {
            /* Destroy video memory name database. */
            gcmkVERIFY_OK(gckKERNEL_DestroyIntegerDatabase(Kernel, Kernel->db->nameDatabase));
        }

        if (Kernel->db->nameDatabaseMutex)
        {
            /* Destroy video memory name database mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->nameDatabaseMutex));
        }

        if (Kernel->db->pointerDatabase)
        {
            /* Destroy id-pointer database. */
            gcmkVERIFY_OK(gckKERNEL_DestroyIntegerDatabase(Kernel, Kernel->db->pointerDatabase));
        }

        if (Kernel->db->pointerDatabaseMutex)
        {
            /* Destroy id-pointer database mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
        }

        if (Kernel->db)
        {
            /* Destroy the database. */
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel->db));
        }

        /* Notify stuck timer to quit. */
        Kernel->monitorTimerStop = gcvTRUE;
    }

    if (Kernel->monitorTimer)
    {
        /* Stop and destroy monitor timer. */
        gcmkVERIFY_OK(gckOS_StopTimer(Kernel->os, Kernel->monitorTimer));
        gcmkVERIFY_OK(gckOS_DestroyTimer(Kernel->os, Kernel->monitorTimer));
    }

#if gcdENABLE_VG
    if (Kernel->vg)
    {
        gcmkVERIFY_OK(gckVGKERNEL_Destroy(Kernel->vg));
    }
    else
#endif
    {
        if (Kernel->command)
        {
            /* Destroy the gckCOMMNAND object. */
            gcmkVERIFY_OK(gckCOMMAND_Destroy(Kernel->command));
        }

        if (Kernel->asyncCommand)
        {
            gcmkVERIFY_OK(gckASYNC_COMMAND_Destroy(Kernel->asyncCommand));
        }

        if (Kernel->asyncEvent)
        {
            gcmkVERIFY_OK(gckEVENT_Destroy(Kernel->asyncEvent));
        }

        if (Kernel->eventObj)
        {
            /* Destroy the gckEVENT object. */
            gcmkVERIFY_OK(gckEVENT_Destroy(Kernel->eventObj));
        }

        gcmkVERIFY_OK(gckHARDWARE_DestroyFunctions(Kernel->hardware));

        if (Kernel->mmu)
        {
#if gcdSHARED_PAGETABLE
            /* Destroy the gckMMU object. */
            gcmkVERIFY_OK(gckMMU_Destroy(Kernel->mmu));
#else
            if (Kernel->mmu->hardware == Kernel->hardware)
            {
                /* Destroy the gckMMU object. */
                gcmkVERIFY_OK(gckMMU_Destroy(Kernel->mmu));
            }
#endif
        }

        if (Kernel->hardware)
        {
            /* Destroy the gckHARDWARE object. */
            gcmkVERIFY_OK(gckHARDWARE_Destroy(Kernel->hardware));
        }
    }

    if (Kernel->atomClients)
    {
        /* Detsroy the client atom. */
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, Kernel->atomClients));
    }

    if (Kernel->virtualBufferLock)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, Kernel->virtualBufferLock));
    }

#if gcdDVFS
    if (Kernel->dvfs)
    {
        gcmkVERIFY_OK(gckDVFS_Stop(Kernel->dvfs));
        gcmkVERIFY_OK(gckDVFS_Destroy(Kernel->dvfs));
    }
#endif

#if gcdLINUX_SYNC_FILE
    if (Kernel->timeline)
    {
        gcmkVERIFY_OK(gckOS_DestroySyncTimeline(Kernel->os, Kernel->timeline));
    }
#endif

#if gcdSECURITY
    if (Kernel->securityChannel)
    {
        gcmkVERIFY_OK(gckKERNEL_SecurityClose(Kernel->securityChannel));
    }
#endif

    /* Mark the gckKERNEL object as unknown. */
    Kernel->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckKERNEL object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Kernel));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  _AllocateMemory
**
**  Private function to walk all required memory pools to allocate the requested
**  amount of video memory.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
gceSTATUS
gckKERNEL_AllocateLinearMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN OUT gcePOOL * Pool,
    IN gctSIZE_T Bytes,
    IN gctUINT32 Alignment,
    IN gceSURF_TYPE Type,
    IN gctUINT32 Flag,
    OUT gctUINT32 * Node
    )
{
    gcePOOL pool;
    gceSTATUS status;
    gckVIDMEM videoMemory;
    gctINT loopCount;
    gcuVIDMEM_NODE_PTR node = gcvNULL;
    gctBOOL tileStatusInVirtual;
    gctBOOL contiguous = gcvFALSE;
    gctBOOL cacheable = gcvFALSE;
    gctBOOL secure = gcvFALSE;
    gctBOOL fastPools = gcvFALSE;
    gctBOOL hasFastPools = gcvFALSE;
    gctSIZE_T bytes = Bytes;
    gctUINT32 handle = 0;
    gceDATABASE_TYPE type;

    gcmkHEADER_ARG("Kernel=0x%x *Pool=%d Bytes=%lu Alignment=%lu Type=%d",
                   Kernel, *Pool, Bytes, Alignment, Type);

    gcmkVERIFY_ARGUMENT(Pool != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes != 0);

    /* Get basic type. */
    Type &= 0xFF;

    /* Check flags. */
    contiguous = Flag & gcvALLOC_FLAG_CONTIGUOUS;
    cacheable  = Flag & gcvALLOC_FLAG_CACHEABLE;
    secure     = Flag & gcvALLOC_FLAG_SECURITY;
    if (Flag & gcvALLOC_FLAG_FAST_POOLS)
    {
        fastPools = gcvTRUE;
        Flag &= ~gcvALLOC_FLAG_FAST_POOLS;
    }

#if gcdALLOC_ON_FAULT
    if (Type == gcvSURF_RENDER_TARGET)
    {
        Flag |= gcvALLOC_FLAG_ALLOC_ON_FAULT;
    }
#endif

    if (Flag & gcvALLOC_FLAG_ALLOC_ON_FAULT)
    {
        *Pool = gcvPOOL_VIRTUAL;
    }

    if (Flag & gcvALLOC_FLAG_DMABUF_EXPORTABLE)
    {
        gctSIZE_T pageSize = 0;
        gckOS_GetPageSize(Kernel->os, &pageSize);

        /* Usually, the exported dmabuf might be later imported to DRM,
        ** while DRM requires input size to be page aligned.
        */
        Bytes = gcmALIGN(Bytes, pageSize);
    }

AllocateMemory:

    /* Get initial pool. */
    switch (pool = *Pool)
    {
    case gcvPOOL_DEFAULT:
    case gcvPOOL_LOCAL:
        pool      = gcvPOOL_LOCAL_INTERNAL;
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    case gcvPOOL_UNIFIED:
        pool      = gcvPOOL_SYSTEM;
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    case gcvPOOL_CONTIGUOUS:
        loopCount = (gctINT) gcvPOOL_NUMBER_OF_POOLS;
        break;

    default:
        loopCount = 1;
        break;
    }

    while (loopCount-- > 0)
    {
        if (pool == gcvPOOL_VIRTUAL)
        {
            /* Create a gcuVIDMEM_NODE for virtual memory. */
            gcmkONERROR(
                gckVIDMEM_ConstructVirtual(Kernel, Flag | gcvALLOC_FLAG_NON_CONTIGUOUS, Bytes, &node));

            bytes = node->Virtual.bytes;
            node->Virtual.type = Type;

            /* Success. */
            break;
        }

        else
        if (pool == gcvPOOL_CONTIGUOUS)
        {
#if gcdCONTIGUOUS_SIZE_LIMIT
            if (Bytes > gcdCONTIGUOUS_SIZE_LIMIT && contiguous == gcvFALSE)
            {
                status = gcvSTATUS_OUT_OF_MEMORY;
            }
            else
#endif
            {
                /* Create a gcuVIDMEM_NODE from contiguous memory. */
                status = gckVIDMEM_ConstructVirtual(
                            Kernel,
                            Flag | gcvALLOC_FLAG_CONTIGUOUS,
                            Bytes,
                            &node);
            }

            if (gcmIS_SUCCESS(status))
            {
                bytes = node->Virtual.bytes;
                node->Virtual.type = Type;

                /* Memory allocated. */
                break;
            }
        }

        else
        /* gcvPOOL_SYSTEM can't be cacheable. */
        if (cacheable == gcvFALSE && secure == gcvFALSE)
        {
            /* Get pointer to gckVIDMEM object for pool. */
            status = gckKERNEL_GetVideoMemoryPool(Kernel, pool, &videoMemory);

            if (gcmIS_SUCCESS(status))
            {
                /* Allocate memory. */
#if defined(gcdLINEAR_SIZE_LIMIT)
                /* 512 KB */
                if (Bytes > gcdLINEAR_SIZE_LIMIT)
                {
                    status = gcvSTATUS_OUT_OF_MEMORY;
                }
                else
#endif
                {
                    hasFastPools = gcvTRUE;
                    status = gckVIDMEM_AllocateLinear(Kernel,
                                                      videoMemory,
                                                      Bytes,
                                                      Alignment,
                                                      Type,
                                                      (*Pool == gcvPOOL_SYSTEM),
                                                      &node);
                }

                if (gcmIS_SUCCESS(status))
                {
                    /* Memory allocated. */
                    node->VidMem.pool = pool;
                    bytes = node->VidMem.bytes;
                    break;
                }
            }
        }

        if (pool == gcvPOOL_LOCAL_INTERNAL)
        {
            /* Advance to external memory. */
            pool = gcvPOOL_LOCAL_EXTERNAL;
        }

        else
        if (pool == gcvPOOL_LOCAL_EXTERNAL)
        {
            /* Advance to contiguous system memory. */
            pool = gcvPOOL_SYSTEM;
        }

        else
        if (pool == gcvPOOL_SYSTEM)
        {
            /* Do not go ahead to try relative slow pools */
            if (fastPools && hasFastPools)
            {
                status = gcvSTATUS_OUT_OF_MEMORY;
                break;
            }

            /* Advance to contiguous memory. */
            pool = gcvPOOL_CONTIGUOUS;
        }

        else
        if (pool == gcvPOOL_CONTIGUOUS)
        {
#if gcdENABLE_VG
            if (Kernel->vg)
            {
                tileStatusInVirtual = gcvFALSE;
            }
            else
#endif
            {
                tileStatusInVirtual =
                    gckHARDWARE_IsFeatureAvailable(Kernel->hardware,
                                                   gcvFEATURE_MC20);
            }

            if (Type == gcvSURF_TILE_STATUS && tileStatusInVirtual != gcvTRUE)
            {
                gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
            }

            if (contiguous)
            {
                break;
            }

            /* Advance to virtual memory. */
            pool = gcvPOOL_VIRTUAL;
        }

        else
        {
            /* Out of pools. */
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }

    if (node == gcvNULL)
    {
        if (contiguous)
        {
            /* Broadcast OOM message. */
            status = gckOS_Broadcast(Kernel->os, Kernel->hardware, gcvBROADCAST_OUT_OF_MEMORY);

            if (gcmIS_SUCCESS(status))
            {
                /* Get some memory. */
                gckOS_Delay(gcvNULL, 1);
                goto AllocateMemory;
            }
        }

        /* Nothing allocated. */
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    /* Allocate handle for this video memory. */
    gcmkONERROR(
        gckVIDMEM_NODE_Allocate(Kernel, node, Type, pool, &handle));

    /* Return node and pool used for allocation. */
    *Node = handle;
    *Pool = pool;

    /* Encode surface type and pool to database type. */
    type = gcvDB_VIDEO_MEMORY
         | (Type << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
         | (pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    /* Record in process db. */
    gcmkONERROR(
            gckKERNEL_AddProcessDB(Kernel,
                                   ProcessID,
                                   type,
                                   gcmINT2PTR(handle),
                                   gcvNULL,
                                   bytes));


    /* Return status. */
    gcmkFOOTER_ARG("*Pool=%d *Node=0x%x", *Pool, *Node);
    return gcvSTATUS_OK;

OnError:
    if (handle)
    {
        /* Destroy handle allocated. */
        gcmkVERIFY_OK(gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, handle));
    }

    if (node)
    {
        /* Free video memory allocated. */
        gcmkVERIFY_OK(gckVIDMEM_Free(Kernel, node));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_ReleaseVideoMemory
**
**  Release handle of a video memory.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          ProcessID of current process.
**
**      gctUINT32 Handle
**          Handle of video memory.
**
**  OUTPUT:
**
**          Nothing.
*/
gceSTATUS
gckKERNEL_ReleaseVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Handle
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject;
    gceDATABASE_TYPE type;

    gcmkHEADER_ARG("Kernel=0x%08X ProcessID=%d Handle=%d",
                   Kernel, ProcessID, Handle);

    gcmkONERROR(
        gckVIDMEM_HANDLE_Lookup(Kernel, ProcessID, Handle, &nodeObject));

    type = gcvDB_VIDEO_MEMORY
         | (nodeObject->type << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
         | (nodeObject->pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    gcmkONERROR(
        gckKERNEL_RemoveProcessDB(Kernel,
            ProcessID,
            type,
            gcmINT2PTR(Handle)));

    gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, Handle);

    gckVIDMEM_NODE_Dereference(Kernel, nodeObject);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_LockVideoMemory
**
**      Lock a video memory node. It will generate a cpu virtual address used
**      by software and a GPU address used by GPU.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gceCORE Core
**          GPU to which video memory is locked.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
gceSTATUS
gckKERNEL_LockVideoMemory(
    IN gckKERNEL Kernel,
    IN gceCORE Core,
    IN gctUINT32 ProcessID,
    IN gctBOOL FromUser,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gcuVIDMEM_NODE_PTR node   = gcvNULL;
    gctBOOL locked            = gcvFALSE;
    gctBOOL asynchronous      = gcvFALSE;
#ifndef __QNXNTO__
    gctPOINTER pointer        = gcvNULL;
#endif

    gcmkHEADER_ARG("Kernel=0x%08X ProcessID=%d",
                   Kernel, ProcessID);

    gcmkONERROR(
        gckVIDMEM_HANDLE_LookupAndReference(Kernel,
                Interface->u.LockVideoMemory.node,
                &nodeObject));

    node = nodeObject->node;

    Interface->u.LockVideoMemory.gid = 0;

    /* Lock video memory. */
    gcmkONERROR(
            gckVIDMEM_Lock(Kernel,
                nodeObject,
                Interface->u.LockVideoMemory.cacheable,
                &Interface->u.LockVideoMemory.address,
                &Interface->u.LockVideoMemory.gid,
                &Interface->u.LockVideoMemory.physicalAddress));

    locked = gcvTRUE;

    if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
    {
        /* Map video memory address into user space. */
#ifdef __QNXNTO__
        if (node->VidMem.logical == gcvNULL)
        {
            gcmkONERROR(
                    gckKERNEL_MapVideoMemory(Kernel,
                        FromUser,
                        Interface->u.LockVideoMemory.address,
                        ProcessID,
                        node->VidMem.bytes,
                        &node->VidMem.logical));
        }
        gcmkASSERT(node->VidMem.logical != gcvNULL);

        Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(node->VidMem.logical);
#else
        gcmkONERROR(
                gckKERNEL_MapVideoMemoryEx(Kernel,
                    Core,
                    FromUser,
                    Interface->u.LockVideoMemory.address,
                    node->VidMem.pool,
                    &pointer));

        Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(pointer);
#endif
    }
    else
    {
        Interface->u.LockVideoMemory.memory = gcmPTR_TO_UINT64(node->Virtual.logical);

        /* Success. */
        status = gcvSTATUS_OK;
    }

#if gcdPROCESS_ADDRESS_SPACE
    gcmkONERROR(gckVIDMEM_Node_Lock(
        Kernel,
        nodeObject,
        &Interface->u.LockVideoMemory.address
        ));
#endif


#if gcdSECURE_USER
    /* Return logical address as physical address. */
    Interface->u.LockVideoMemory.address =
        (gctUINT32)(Interface->u.LockVideoMemory.memory);
#endif
    gcmkONERROR(
        gckKERNEL_AddProcessDB(Kernel,
            ProcessID, gcvDB_VIDEO_MEMORY_LOCKED,
            gcmINT2PTR(Interface->u.LockVideoMemory.node),
            gcvNULL,
            0));

    gckVIDMEM_HANDLE_Reference(
        Kernel, ProcessID, (gctUINT32)Interface->u.LockVideoMemory.node);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (locked)
    {
        /* Roll back the lock. */
        gcmkVERIFY_OK(gckVIDMEM_Unlock(Kernel,
                    nodeObject,
                    gcvSURF_TYPE_UNKNOWN,
                    &asynchronous));

        if (gcvTRUE == asynchronous)
        {
            /* Bottom Half */
            gcmkVERIFY_OK(gckVIDMEM_Unlock(Kernel,
                        nodeObject,
                        gcvSURF_TYPE_UNKNOWN,
                        gcvNULL));
        }
    }

    if (nodeObject != gcvNULL)
    {
        gckVIDMEM_NODE_Dereference(Kernel, nodeObject);
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_UnlockVideoMemory
**
**      Unlock a video memory node.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          ProcessID of current process.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
gceSTATUS
gckKERNEL_UnlockVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject;
    gcuVIDMEM_NODE_PTR node;
    gctSIZE_T bytes;

    gcmkHEADER_ARG("Kernel=0x%08X ProcessID=%d",
                   Kernel, ProcessID);

    Interface->u.UnlockVideoMemory.pool = gcvPOOL_UNKNOWN;
    Interface->u.UnlockVideoMemory.bytes = 0;

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
        Kernel,
        ProcessID,
        (gctUINT32)Interface->u.UnlockVideoMemory.node,
        &nodeObject));

    node = nodeObject->node;
    bytes = (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
          ? node->VidMem.bytes
          : node->Virtual.bytes;

    /* Unlock video memory. */
    gcmkONERROR(gckVIDMEM_Unlock(
        Kernel,
        nodeObject,
        Interface->u.UnlockVideoMemory.type,
        &Interface->u.UnlockVideoMemory.asynchroneous));

#if gcdSECURE_USER
    /* Flush the translation cache for virtual surfaces. */
    if (node->VidMem.memory->object.type != gcvOBJ_VIDMEM)
    {
        gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(Kernel,
                    cache,
                    node->Virtual.logical,
                    bytes));
    }
#endif

    Interface->u.UnlockVideoMemory.pool = nodeObject->pool;
    Interface->u.UnlockVideoMemory.bytes = (gctUINT)bytes;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_BottomHalfUnlockVideoMemory
**
**  Unlock video memory from gpu.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          Process ID owning this memory.
**
**      gctPOINTER Pointer
**          Video memory to be unlock.
*/
gceSTATUS
gckKERNEL_BottomHalfUnlockVideoMemory(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Node
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;

    /* Remove record from process db. */
    gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
        Kernel,
        ProcessID,
        gcvDB_VIDEO_MEMORY_LOCKED,
        gcmINT2PTR(Node)));

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(
        Kernel,
        ProcessID,
        Node,
        &nodeObject));

    gckVIDMEM_HANDLE_Dereference(Kernel, ProcessID, Node);

    /* Unlock video memory. */
    gcmkONERROR(gckVIDMEM_Unlock(
        Kernel,
        nodeObject,
        gcvSURF_TYPE_UNKNOWN,
        gcvNULL));

    gcmkONERROR(gckVIDMEM_NODE_Dereference(
        Kernel,
        nodeObject));

    return gcvSTATUS_OK;

OnError:
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_SetVidMemMetadata
**
**  Set/Get metadata to/from gckVIDMEM_NODE object.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 ProcessID
**          ProcessID of current process.
**
**  INOUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a interface structure
*/
#if defined(CONFIG_DMA_SHARED_BUFFER)
#include <linux/dma-buf.h>

gceSTATUS
gckKERNEL_SetVidMemMetadata(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    INOUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckVIDMEM_NODE nodeObj = gcvNULL;

    gcmkHEADER_ARG("Kernel=0x%X ProcessID=%d", Kernel, ProcessID);

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(Kernel, ProcessID, Interface->u.SetVidMemMetadata.node, &nodeObj));

    if (Interface->u.SetVidMemMetadata.readback)
    {
        Interface->u.SetVidMemMetadata.ts_fd            = nodeObj->metadata.ts_fd;
        Interface->u.SetVidMemMetadata.fc_enabled       = nodeObj->metadata.fc_enabled;
        Interface->u.SetVidMemMetadata.fc_value         = nodeObj->metadata.fc_value;
        Interface->u.SetVidMemMetadata.fc_value_upper   = nodeObj->metadata.fc_value_upper;
        Interface->u.SetVidMemMetadata.compressed       = nodeObj->metadata.compressed;
        Interface->u.SetVidMemMetadata.compress_format  = nodeObj->metadata.compress_format;
    }
    else
    {
#ifdef gcdANDROID
        if (nodeObj->metadata.ts_address == 0 && nodeObj->tsNode != NULL)
        {
            gctUINT32 Address = 0;
            gctUINT32 Gid = 0;
            gctUINT64 PhysicalAddress = 0;

            gcmkONERROR(gckVIDMEM_Lock(Kernel,
                    nodeObj->tsNode,
                    gcvFALSE,
                    &Address,
                    &Gid,
                    &PhysicalAddress));

            nodeObj->metadata.ts_address = (
                    PhysicalAddress + Kernel->hardware->baseAddress);
            gcmkONERROR(gckVIDMEM_Unlock(Kernel,
                    nodeObj->tsNode,
                    gcvSURF_TYPE_UNKNOWN,
                    gcvNULL));
        }
#else
        nodeObj->metadata.ts_fd             = Interface->u.SetVidMemMetadata.ts_fd;

        if (nodeObj->metadata.ts_fd >= 0)
        {
            nodeObj->metadata.ts_dma_buf    = dma_buf_get(nodeObj->metadata.ts_fd);

            if (IS_ERR(nodeObj->metadata.ts_dma_buf))
            {
                gcmkONERROR(gcvSTATUS_NOT_FOUND);
            }

            dma_buf_put(nodeObj->metadata.ts_dma_buf);
        }
        else
        {
            nodeObj->metadata.ts_dma_buf    = NULL;
        }
#endif

        nodeObj->metadata.fc_enabled        = Interface->u.SetVidMemMetadata.fc_enabled;
        nodeObj->metadata.fc_value          = Interface->u.SetVidMemMetadata.fc_value;
        nodeObj->metadata.fc_value_upper    = Interface->u.SetVidMemMetadata.fc_value_upper;
        nodeObj->metadata.compressed        = Interface->u.SetVidMemMetadata.compressed;
        nodeObj->metadata.compress_format   = Interface->u.SetVidMemMetadata.compress_format;
    }

OnError:
    gcmkFOOTER();
    return status;
}

#else

gceSTATUS
gckKERNEL_SetVidMemMetadata(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    INOUT gcsHAL_INTERFACE * Interface
    )
{
    gcmkFATAL("The kernel did NOT support CONFIG_DMA_SHARED_BUFFER");
    return gcvSTATUS_NOT_SUPPORTED;
}
#endif

/*******************************************************************************
**
**  gckKERNEL_QueryVidMemPoolNodes
**
**  Loop all databases to query used memory nodes of a specific pool.
**
**  INPUT:
**
**  Pool            The memory pool for query.

**  TotalSize       Total size of the used contiguous memory.
**
**  MemoryBlocks    Used contiguous memory block info.
**
**  NumMaxBlock     The count of memory blocks array provided by the caller.
**
**  NumBlocks       The actual number of memory blocks returned.
**
**  OUTPUT:
**
**      Error status. Should always be gcvSTATUS_SUCCESS since a query should always succeed.
*/
gceSTATUS
gckKERNEL_QueryVidMemPoolNodes(
    gckKERNEL            Kernel,
    gcePOOL              Pool,
    gctUINT32          * TotalSize,
    gcsContiguousBlock * MemoryBlocks,
    gctUINT32            NumMaxBlocks,
    gctUINT32          * NumBlocks
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctINT i;
    gctINT bcount;
    gctUINT32 num_blocks = 0;
    gctSIZE_T total_size = 0;
    gckVIDMEM memory;
    gcuVIDMEM_NODE_PTR nodes, node;

    do
    {
        /* Get the heap and nodes. */
        status = gckKERNEL_GetVideoMemoryPool(Kernel, Pool, &memory);
        if (status != gcvSTATUS_OK)
            break;

        status = gckVIDMEM_QueryNodes(Kernel, Pool, &bcount, &nodes);
        if (status != gcvSTATUS_OK)
            break;

        /* Iterate all nodes. */
        for (i = 0; i < bcount; i++)
        {
            node = nodes[i].VidMem.next;
            do
            {
                if (node == gcvNULL)
                {
                    break;
                }

                /* Is it in the "free" list? */
                if (node->VidMem.nextFree == gcvNULL)
                {
                    if (num_blocks < NumMaxBlocks)
                    {
                        MemoryBlocks[num_blocks].ptr = (gctUINT32)node->VidMem.offset + memory->baseAddress;
                        MemoryBlocks[num_blocks].size = node->VidMem.bytes;
                    }
                    total_size += node->VidMem.bytes;
                    num_blocks++;
                }

                node = node->VidMem.next;
            } while (node != &nodes[i]);
        }
    }
    while (gcvFALSE);

    if (TotalSize != gcvNULL)
        *TotalSize = (gctUINT32)total_size;

    if (NumBlocks != gcvNULL)
        *NumBlocks = num_blocks;

    return status;
}

gceSTATUS
gckKERNEL_QueryDatabase(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status;
    gctINT i;

    gceDATABASE_TYPE type[3] = {
        gcvDB_VIDEO_MEMORY | (gcvPOOL_SYSTEM << gcdDB_VIDEO_MEMORY_POOL_SHIFT),
        gcvDB_VIDEO_MEMORY | (gcvPOOL_CONTIGUOUS << gcdDB_VIDEO_MEMORY_POOL_SHIFT),
        gcvDB_VIDEO_MEMORY | (gcvPOOL_VIRTUAL << gcdDB_VIDEO_MEMORY_POOL_SHIFT),
    };

    gcmkHEADER();

    /* Query video memory. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_VIDEO_MEMORY,
                                 &Interface->u.Database.vidMem));

    /* Query non-paged memory. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_NON_PAGED,
                                 &Interface->u.Database.nonPaged));

    /* Query contiguous memory. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_CONTIGUOUS,
                                 &Interface->u.Database.contiguous));

    /* Query GPU idle time. */
    gcmkONERROR(
        gckKERNEL_QueryProcessDB(Kernel,
                                 Interface->u.Database.processID,
                                 !Interface->u.Database.validProcessID,
                                 gcvDB_IDLE,
                                 &Interface->u.Database.gpuIdle));
    for (i = 0; i < 3; i++)
    {
        /* Query each video memory pool. */
        gcmkONERROR(
            gckKERNEL_QueryProcessDB(Kernel,
                                     Interface->u.Database.processID,
                                     !Interface->u.Database.validProcessID,
                                     type[i],
                                     &Interface->u.Database.vidMemPool[i]));
    }

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gckKERNEL_DumpVidMemUsage(Kernel, Interface->u.Database.processID);
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_ConfigPowerManagement(
    IN gckKERNEL Kernel,
    IN OUT gcsHAL_INTERFACE * Interface
)
{
    gceSTATUS status;
    gctBOOL enable = Interface->u.ConfigPowerManagement.enable;

    gcmkHEADER();

    gcmkONERROR(gckHARDWARE_SetPowerManagement(Kernel->hardware, enable));

    if (enable == gcvFALSE)
    {
        gcmkONERROR(
            gckHARDWARE_SetPowerManagementState(Kernel->hardware, gcvPOWER_ON));
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckKERNEL_CacheOperation(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Node,
    IN gceCACHEOPERATION Operation,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gcuVIDMEM_NODE_PTR node = gcvNULL;
    void *memHandle;

    gcmkHEADER_ARG("Kernel=%p pid=%u Node=%u op=%d Logical=%p Bytes=0x%lx",
                   Kernel, ProcessID, Node, Operation, Logical, Bytes);

    gcmkONERROR(gckVIDMEM_HANDLE_Lookup(Kernel,
                                        ProcessID,
                                        Node,
                                        &nodeObject));

    node = nodeObject->node;

    if (node->VidMem.memory->object.type == gcvOBJ_VIDMEM)
    {
        static gctBOOL printed;

        if (!printed)
        {
            printed = gcvTRUE;
            gcmkPRINT("[galcore]: %s: Flush Video Memory", __FUNCTION__);
        }

        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }
    else
    {
        memHandle = node->Virtual.physical;
    }

    switch (Operation)
    {
    case gcvCACHE_FLUSH:
        /* Clean and invalidate the cache. */
        status = gckOS_CacheFlush(Kernel->os,
                                  ProcessID,
                                  memHandle,
                                  gcvINVALID_PHYSICAL_ADDRESS,
                                  Logical,
                                  Bytes);
        break;
    case gcvCACHE_CLEAN:
        /* Clean the cache. */
        status = gckOS_CacheClean(Kernel->os,
                                  ProcessID,
                                  memHandle,
                                  gcvINVALID_PHYSICAL_ADDRESS,
                                  Logical,
                                  Bytes);
        break;
    case gcvCACHE_INVALIDATE:
        /* Invalidate the cache. */
        status = gckOS_CacheInvalidate(Kernel->os,
                                       ProcessID,
                                       memHandle,
                                       gcvINVALID_PHYSICAL_ADDRESS,
                                       Logical,
                                       Bytes);
        break;

    case gcvCACHE_MEMORY_BARRIER:
        status = gckOS_MemoryBarrier(Kernel->os, Logical);
        break;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        break;
    }

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_WaitFence(
    IN gckKERNEL Kernel,
    IN gctUINT32 Handle,
    IN gctUINT32 TimeOut
    )
{
    gceSTATUS status;
    gckVIDMEM_NODE node;
    gctUINT32 processID;
    gckCOMMAND command = Kernel->command;
    gckASYNC_COMMAND asyncCommand = Kernel->asyncCommand;
    gckFENCE fence = gcvNULL;
    gctUINT i;

    gckOS_GetProcessID(&processID);

    gcmkONERROR(gckVIDMEM_HANDLE_LookupAndReference(Kernel, Handle, &node));

    /* Wait for fence of all engines. */
    for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
    {
        gckFENCE_SYNC sync = &node->sync[i];

        if (i == gcvENGINE_RENDER)
        {
            fence = command->fence;
        }
        else
        {
            fence = asyncCommand->fence;
        }

#if USE_KERNEL_VIRTUAL_BUFFERS
        if (Kernel->virtualCommandBuffer)
        {
            gckVIRTUAL_COMMAND_BUFFER_PTR commandBuffer = (gckVIRTUAL_COMMAND_BUFFER_PTR) fence->physical;

            fence->physHandle = commandBuffer->virtualBuffer.physical;
        }
        else
#endif
        {
            fence->physHandle = fence->physical;
        }

        gcmkONERROR(gckOS_CacheInvalidate(
            Kernel->os,
            0,
            fence->physHandle,
            0,
            fence->logical,
            8
            ));

        if (sync->commitStamp <= *(gctUINT64_PTR)fence->logical)
        {
            continue;
        }
        else
        {
            gckOS_Signal(Kernel->os, sync->signal, gcvFALSE);

            gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, &fence->mutex, gcvINFINITE));

            /* Add to waiting list. */
            gcsLIST_AddTail(&sync->head, &fence->waitingList);

            gcmkASSERT(sync->inList == gcvFALSE);

            sync->inList = gcvTRUE;

            gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, &fence->mutex));

            /* Wait. */
            status = gckOS_WaitSignal(Kernel->os, sync->signal, gcvTRUE, TimeOut);

            gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, &fence->mutex, gcvINFINITE));

            if (sync->inList)
            {
                gcsLIST_Del(&sync->head);
                sync->inList = gcvFALSE;
            }

            gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, &fence->mutex));
        }
    }

    gckVIDMEM_NODE_Dereference(Kernel, node);

OnError:
    return status;
}

#ifdef __linux__

typedef struct _gcsGRRAPHIC_BUFFER_PARCLE
{
    gcsFDPRIVATE base;
    gckKERNEL kernel;

    gckVIDMEM_NODE node[3];
    gctSHBUF  shBuf;
    gctINT32  signal;
}
gcsGRAPHIC_BUFFER_PARCLE;

static void
_ReleaseGraphicBuffer(
    gckKERNEL Kernel,
    gcsGRAPHIC_BUFFER_PARCLE * Parcle
    )
{
    gctUINT i;

    for (i = 0; i < 3; i++)
    {
        if (Parcle->node[i])
        {
            gckVIDMEM_NODE_Dereference(Kernel, Parcle->node[i]);
        }
    }

    if (Parcle->shBuf)
    {
        gckKERNEL_DestroyShBuffer(Kernel, Parcle->shBuf);
    }

    if (Parcle->signal)
    {
        gckOS_DestroyUserSignal(Kernel->os, Parcle->signal);
    }

    gcmkOS_SAFE_FREE(Kernel->os, Parcle);
}

static gctINT
_FdReleaseGraphicBuffer(
    gcsFDPRIVATE_PTR Private
    )
{
    gcsGRAPHIC_BUFFER_PARCLE * parcle = (gcsGRAPHIC_BUFFER_PARCLE *) Private;

    _ReleaseGraphicBuffer(parcle->kernel, parcle);
    return 0;
}

static gceSTATUS
_GetGraphicBufferFd(
    IN gckKERNEL Kernel,
    IN gctUINT32 ProcessID,
    IN gctUINT32 Node[3],
    IN gctUINT64 ShBuf,
    IN gctUINT64 Signal,
    OUT gctINT32 * Fd
    )
{
    gceSTATUS status;
    gctUINT i;
    gcsGRAPHIC_BUFFER_PARCLE * parcle = gcvNULL;

    gcmkONERROR(gckOS_Allocate(
        Kernel->os,
        gcmSIZEOF(gcsGRAPHIC_BUFFER_PARCLE),
        (gctPOINTER *)&parcle
        ));

    gckOS_ZeroMemory(parcle, sizeof(gcsGRAPHIC_BUFFER_PARCLE));

    parcle->base.release = _FdReleaseGraphicBuffer;
    parcle->kernel = Kernel;

    for (i = 0; i < 3; i++)
    {
        if (Node[i] != 0)
        {
            gcmkONERROR(
                gckVIDMEM_HANDLE_LookupAndReference(Kernel, Node[i], &parcle->node[i]));
        }
    }

    if (ShBuf)
    {
        gctSHBUF shBuf = gcmUINT64_TO_PTR(ShBuf);

        gcmkONERROR(gckKERNEL_MapShBuffer(Kernel, shBuf));
        parcle->shBuf = shBuf;
    }

    if (Signal)
    {
        gctSIGNAL signal = gcmUINT64_TO_PTR(Signal);

        gcmkONERROR(
            gckOS_MapSignal(Kernel->os,
                            signal,
                            (gctHANDLE)(gctUINTPTR_T)ProcessID,
                            &signal));

        parcle->signal= (gctINT32)Signal;
    }

    gcmkONERROR(gckOS_GetFd("viv-gr", &parcle->base, Fd));

    return gcvSTATUS_OK;

OnError:
    if (parcle)
    {
        _ReleaseGraphicBuffer(Kernel, parcle);
    }
    return status;
}
#endif

/*******************************************************************************
**
**  gckKERNEL_Dispatch
**
**  Dispatch a command received from the user HAL layer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL FromUser
**          whether the call is from the user space.
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that defines the command to
**          be dispatched.
**
**  OUTPUT:
**
**      gcsHAL_INTERFACE * Interface
**          Pointer to a gcsHAL_INTERFACE structure that receives any data to be
**          returned.
*/
gceSTATUS
gckKERNEL_Dispatch(
    IN gckKERNEL Kernel,
    IN gckDEVICE Device,
    IN gctBOOL FromUser,
    IN OUT gcsHAL_INTERFACE * Interface
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPHYS_ADDR physical = gcvNULL;
    gctSIZE_T bytes;
    gctPOINTER logical = gcvNULL;
#if (gcdENABLE_3D || gcdENABLE_2D)
    gckCONTEXT context = gcvNULL;
#endif
    gckKERNEL kernel = Kernel;
    gctUINT32 processID;
#if gcdSECURE_USER
    gcskSECURE_CACHE_PTR cache;
    gctPOINTER logical;
#endif
#if !USE_NEW_LINUX_SIGNAL
    gctSIGNAL   signal;
#endif
    gckVIRTUAL_COMMAND_BUFFER_PTR buffer;

    gctBOOL powerMutexAcquired = gcvFALSE;
    gctBOOL commitMutexAcquired = gcvFALSE;

    gcmkHEADER_ARG("Kernel=0x%x FromUser=%d Interface=0x%x",
                   Kernel, FromUser, Interface);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Interface != gcvNULL);

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                   "Dispatching command %d (%s)",
                   Interface->command, _DispatchText[Interface->command]);
#endif
#if QNX_SINGLE_THREADED_DEBUGGING
    gckOS_AcquireMutex(Kernel->os, Kernel->debugMutex, gcvINFINITE);
#endif

    /* Get the current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

#if gcdSECURE_USER
    gcmkONERROR(gckKERNEL_GetProcessDBCache(Kernel, processID, &cache));
#endif

    /* Dispatch on command. */
    switch (Interface->command)
    {
    case gcvHAL_GET_BASE_ADDRESS:
        /* Get base address. */
        Interface->u.GetBaseAddress.baseAddress = Kernel->hardware->baseAddress;
        Interface->u.GetBaseAddress.flatMappingRangeCount = Kernel->mmu->flatMappingRangeCount;
        if (Kernel->mmu->flatMappingRangeCount)
        {
            gckOS_MemCopy(Interface->u.GetBaseAddress.flatMappingRanges, Kernel->mmu->flatMappingRanges,
                gcmSIZEOF(gcsFLAT_MAPPING_RANGE) * Kernel->mmu->flatMappingRangeCount);
        }
        break;

    case gcvHAL_QUERY_VIDEO_MEMORY:
        /* Query video memory size. */
        gcmkONERROR(gckKERNEL_QueryVideoMemory(Kernel, Interface));
        break;

    case gcvHAL_QUERY_CHIP_IDENTITY:
        /* Query chip identity. */
        gcmkONERROR(
            gckHARDWARE_QueryChipIdentity(
                Kernel->hardware,
                &Interface->u.QueryChipIdentity));
        break;

    case gcvHAL_QUERY_CHIP_FREQUENCY:
        /* Query chip clock. */
        gcmkONERROR(
            gckHARDWARE_QueryFrequency(Kernel->hardware));

        Interface->u.QueryChipFrequency.mcClk = Kernel->hardware->mcClk;
        Interface->u.QueryChipFrequency.shClk = Kernel->hardware->shClk;
        break;

    case gcvHAL_MAP_MEMORY:
        physical = gcmINT2PTR(Interface->u.MapMemory.physical);

        /* Map memory. */
        gcmkONERROR(
            gckKERNEL_MapMemory(Kernel,
                                physical,
                                (gctSIZE_T) Interface->u.MapMemory.bytes,
                                &logical));

        Interface->u.MapMemory.logical = gcmPTR_TO_UINT64(logical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_MAP_MEMORY,
                                   logical,
                                   physical,
                                   (gctSIZE_T) Interface->u.MapMemory.bytes));
        break;

    case gcvHAL_UNMAP_MEMORY:
        physical = gcmINT2PTR(Interface->u.UnmapMemory.physical);

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_MAP_MEMORY,
                                      gcmUINT64_TO_PTR(Interface->u.UnmapMemory.logical)));

        /* Unmap memory. */
        gcmkONERROR(
            gckKERNEL_UnmapMemory(Kernel,
                                  physical,
                                  (gctSIZE_T) Interface->u.UnmapMemory.bytes,
                                  gcmUINT64_TO_PTR(Interface->u.UnmapMemory.logical),
                                  processID));
        break;

    case gcvHAL_ALLOCATE_NON_PAGED_MEMORY:
        bytes = (gctSIZE_T) Interface->u.AllocateNonPagedMemory.bytes;

        /* Allocate non-paged memory. */
        gcmkONERROR(
            gckOS_AllocateNonPagedMemory(
                Kernel->os,
                FromUser,
                gcvALLOC_FLAG_CONTIGUOUS,
                &bytes,
                &physical,
                &logical));

        Interface->u.AllocateNonPagedMemory.bytes    = bytes;
        Interface->u.AllocateNonPagedMemory.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateNonPagedMemory.physical = gcmPTR_TO_NAME(physical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_NON_PAGED,
                                   logical,
                                   gcmINT2PTR(Interface->u.AllocateNonPagedMemory.physical),
                                   bytes));
        break;

    case gcvHAL_ALLOCATE_VIRTUAL_COMMAND_BUFFER:
        bytes = (gctSIZE_T) Interface->u.AllocateVirtualCommandBuffer.bytes;

        gcmkONERROR(
            gckKERNEL_AllocateVirtualCommandBuffer(
                Kernel,
                FromUser,
                &bytes,
                &physical,
                &logical));

        Interface->u.AllocateVirtualCommandBuffer.bytes    = bytes;
        Interface->u.AllocateVirtualCommandBuffer.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateVirtualCommandBuffer.physical = gcmPTR_TO_NAME(physical);

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_COMMAND_BUFFER,
                                   logical,
                                   gcmINT2PTR(Interface->u.AllocateVirtualCommandBuffer.physical),
                                   bytes));
        break;

    case gcvHAL_FREE_NON_PAGED_MEMORY:
        physical = gcmNAME_TO_PTR(Interface->u.FreeNonPagedMemory.physical);

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_NON_PAGED,
                                      gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

        /* Unmap user logical out of physical memory first. */
        gcmkONERROR(gckOS_UnmapUserLogical(Kernel->os,
                                           physical,
                                           (gctSIZE_T) Interface->u.FreeNonPagedMemory.bytes,
                                           gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

        /* Free non-paged memory. */
        gcmkONERROR(
            gckOS_FreeNonPagedMemory(Kernel->os,
                                     (gctSIZE_T) Interface->u.FreeNonPagedMemory.bytes,
                                     physical,
                                     gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

#if gcdSECURE_USER
        gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(
            Kernel,
            cache,
            gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical),
            (gctSIZE_T) Interface->u.FreeNonPagedMemory.bytes));
#endif

        gcmRELEASE_NAME(Interface->u.FreeNonPagedMemory.physical);
        break;

    case gcvHAL_ALLOCATE_CONTIGUOUS_MEMORY:
        bytes = (gctSIZE_T) Interface->u.AllocateContiguousMemory.bytes;

        /* Allocate contiguous memory. */
        gcmkONERROR(gckOS_AllocateContiguous(
            Kernel->os,
            FromUser,
            &bytes,
            &physical,
            &logical));

        Interface->u.AllocateContiguousMemory.bytes    = bytes;
        Interface->u.AllocateContiguousMemory.logical  = gcmPTR_TO_UINT64(logical);
        Interface->u.AllocateContiguousMemory.physical = gcmPTR_TO_NAME(physical);

        gcmkONERROR(gckHARDWARE_ConvertLogical(
            Kernel->hardware,
            logical,
            gcvTRUE,
            &Interface->u.AllocateContiguousMemory.address));

        gcmkVERIFY_OK(gckKERNEL_AddProcessDB(
            Kernel,
            processID, gcvDB_CONTIGUOUS,
            logical,
            gcmINT2PTR(Interface->u.AllocateContiguousMemory.physical),
            bytes));
        break;

    case gcvHAL_FREE_CONTIGUOUS_MEMORY:
        physical = gcmNAME_TO_PTR(Interface->u.FreeContiguousMemory.physical);

        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                                      processID, gcvDB_CONTIGUOUS,
                                      gcmUINT64_TO_PTR(Interface->u.FreeNonPagedMemory.logical)));

        /* Unmap user logical out of physical memory first. */
        gcmkONERROR(gckOS_UnmapUserLogical(Kernel->os,
                                           physical,
                                           (gctSIZE_T) Interface->u.FreeContiguousMemory.bytes,
                                           gcmUINT64_TO_PTR(Interface->u.FreeContiguousMemory.logical)));

        /* Free contiguous memory. */
        gcmkONERROR(
            gckOS_FreeContiguous(Kernel->os,
                                 physical,
                                 gcmUINT64_TO_PTR(Interface->u.FreeContiguousMemory.logical),
                                 (gctSIZE_T) Interface->u.FreeContiguousMemory.bytes));

#if gcdSECURE_USER
        gcmkVERIFY_OK(gckKERNEL_FlushTranslationCache(
            Kernel,
            cache,
            gcmUINT64_TO_PTR(Interface->u.FreeContiguousMemory.logical),
            (gctSIZE_T) Interface->u.FreeContiguousMemory.bytes));
#endif

        gcmRELEASE_NAME(Interface->u.FreeContiguousMemory.physical);
        break;

    case gcvHAL_ALLOCATE_VIDEO_MEMORY:

        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);

        break;

    case gcvHAL_ALLOCATE_LINEAR_VIDEO_MEMORY:
        /* Allocate memory. */
        gcmkONERROR(
            gckKERNEL_AllocateLinearMemory(Kernel, processID,
                            &Interface->u.AllocateLinearVideoMemory.pool,
                            Interface->u.AllocateLinearVideoMemory.bytes,
                            Interface->u.AllocateLinearVideoMemory.alignment,
                            Interface->u.AllocateLinearVideoMemory.type,
                            Interface->u.AllocateLinearVideoMemory.flag,
                            &Interface->u.AllocateLinearVideoMemory.node));
        break;

    case gcvHAL_RELEASE_VIDEO_MEMORY:
        /* Release video memory. */
        gcmkONERROR(gckKERNEL_ReleaseVideoMemory(
            Kernel, processID,
            (gctUINT32)Interface->u.ReleaseVideoMemory.node
            ));
    break;

    case gcvHAL_LOCK_VIDEO_MEMORY:
        /* Lock video memory. */
        gcmkONERROR(gckKERNEL_LockVideoMemory(Kernel, Kernel->core, processID, FromUser, Interface));
        break;

    case gcvHAL_UNLOCK_VIDEO_MEMORY:
        /* Unlock video memory. */
        gcmkONERROR(gckKERNEL_UnlockVideoMemory(Kernel, processID, Interface));
        break;

    case gcvHAL_BOTTOM_HALF_UNLOCK_VIDEO_MEMORY:
        gcmkERR_BREAK(gckKERNEL_BottomHalfUnlockVideoMemory(Kernel, processID,
                                                            Interface->u.BottomHalfUnlockVideoMemory.node));
        break;

    case gcvHAL_EVENT_COMMIT:
        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                Kernel->device->commitMutex,
                gcvINFINITE
                ));

            commitMutexAcquired = gcvTRUE;
        }
        /* Commit an event queue. */
        if (Interface->engine == gcvENGINE_BLT)
        {
            if (!gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_ASYNC_BLIT))
            {
                gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
            }

            gcmkONERROR(gckEVENT_Commit(
                Kernel->asyncEvent, gcmUINT64_TO_PTR(Interface->u.Event.queue), gcvFALSE));
        }
        else
        {
            gcmkONERROR(gckEVENT_Commit(
                Kernel->eventObj, gcmUINT64_TO_PTR(Interface->u.Event.queue), gcvFALSE));
        }

        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
            commitMutexAcquired = gcvFALSE;
        }
        break;

    case gcvHAL_COMMIT:
        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                Kernel->device->commitMutex,
                gcvINFINITE
                ));
            commitMutexAcquired = gcvTRUE;
        }

        /* Commit a command and context buffer. */
        if (Interface->engine == gcvENGINE_BLT)
        {
            gctUINT64 *commandBuffers = gcmUINT64_TO_PTR(Interface->u.Commit.commandBuffer);

            if (!gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_ASYNC_BLIT))
            {
                gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
            }

            gcmkONERROR(gckASYNC_COMMAND_Commit(
                Kernel->asyncCommand,
                gcmUINT64_TO_PTR(commandBuffers[0]),
                gcmUINT64_TO_PTR(Interface->u.Commit.queue)
                ));

            gcmkONERROR(gckEVENT_Commit(
                Kernel->asyncEvent,
                gcmUINT64_TO_PTR(Interface->u.Commit.queue),
                gcvFALSE
                ));
        }
        else
        {
            gctUINT32 i;

            if (Interface->u.Commit.count > 1 && Interface->engine == gcvENGINE_RENDER)
            {
                for (i = 0; i < Interface->u.Commit.count; i++)
                {
                    gceHARDWARE_TYPE type = Interface->hardwareType;
                    gckKERNEL kernel = Device->map[type].kernels[i];

                    gcmkONERROR(gckOS_Broadcast(kernel->os,
                                                kernel->hardware,
                                                gcvBROADCAST_GPU_COMMIT));
                }
            }

            status = gckCOMMAND_Commit(Kernel->command,
                Interface->u.Commit.contexts[0] ?
                gcmNAME_TO_PTR(Interface->u.Commit.contexts[0]) : gcvNULL,
                gcmUINT64_TO_PTR(Interface->u.Commit.commandBuffers[0]),
                gcmUINT64_TO_PTR(Interface->u.Commit.deltas[0]),
                processID,
                Interface->u.Commit.shared,
                Interface->u.Commit.index,
                &Interface->u.Commit.commitStamp,
                &Interface->u.Commit.contextSwitched
                );

            if (status != gcvSTATUS_INTERRUPTED)
            {
                gcmkONERROR(status);
            }

            /* Force an event if powerManagement is on. */
            status = gckEVENT_Commit(Kernel->eventObj,
                                     gcmUINT64_TO_PTR(Interface->u.Commit.queue),
                                     Kernel->hardware->options.powerManagement);

            if (status != gcvSTATUS_INTERRUPTED)
            {
                gcmkONERROR(status);
            }

            if (Interface->u.Commit.count > 1 && Interface->engine == gcvENGINE_RENDER)
            {
                for (i = 1; i < Interface->u.Commit.count; i++)
                {
                    gceHARDWARE_TYPE type = Interface->hardwareType;
                    gckKERNEL kernel = Device->map[type].kernels[i];

                    status = gckCOMMAND_Commit(kernel->command,
                        Interface->u.Commit.contexts[i] ?
                        gcmNAME_TO_PTR(Interface->u.Commit.contexts[i]) : gcvNULL,
                        Interface->u.Commit.commandBuffers[i] ?
                        gcmUINT64_TO_PTR(Interface->u.Commit.commandBuffers[i]) : gcmUINT64_TO_PTR(Interface->u.Commit.commandBuffers[0]),
                        gcmUINT64_TO_PTR(Interface->u.Commit.deltas[i]),
                        processID,
                        Interface->u.Commit.shared,
                        Interface->u.Commit.commandBuffers[i] ?
                        Interface->u.Commit.index : i,
                        &Interface->u.Commit.commitStamp,
                        &Interface->u.Commit.contextSwitched
                        );

                    if (status != gcvSTATUS_INTERRUPTED)
                    {
                        gcmkONERROR(status);
                    }

                    /* Force an event if powerManagement is on. */
                    status = gckEVENT_Commit(kernel->eventObj,
                                             gcvNULL,
                                             kernel->hardware->options.powerManagement);

                    if (status != gcvSTATUS_INTERRUPTED)
                    {
                        gcmkONERROR(status);
                    }
                }
            }

            for (i = 0; i < Interface->u.Commit.count; i++)
            {
                gceHARDWARE_TYPE type = Interface->hardwareType;
                gckKERNEL kernel = Device->map[type].kernels[i];

                if  ((kernel->hardware->options.gpuProfiler == gcvTRUE) &&
                     (kernel->profileEnable == gcvTRUE))
                {
                    gcmkONERROR(gckCOMMAND_Stall(kernel->command, gcvTRUE));

                    if (kernel->command->currContext)
                    {
                        gcmkONERROR(gckHARDWARE_UpdateContextProfile(
                                    kernel->hardware,
                                    kernel->command->currContext));
                    }
                }
            }
        }

        if (!Interface->commitMutex)
        {
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
            commitMutexAcquired = gcvFALSE;
        }
        break;

    case gcvHAL_STALL:
        /* Stall the command queue. */
        gcmkONERROR(gckCOMMAND_Stall(Kernel->command, gcvFALSE));

        break;

    case gcvHAL_MAP_USER_MEMORY:

        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);

        break;

    case gcvHAL_UNMAP_USER_MEMORY:

        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);

        break;

#if !USE_NEW_LINUX_SIGNAL
    case gcvHAL_USER_SIGNAL:
        /* Dispatch depends on the user signal subcommands. */
        switch(Interface->u.UserSignal.command)
        {
        case gcvUSER_SIGNAL_CREATE:
            /* Create a signal used in the user space. */
            gcmkONERROR(
                gckOS_CreateUserSignal(Kernel->os,
                                       Interface->u.UserSignal.manualReset,
                                       &Interface->u.UserSignal.id));

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_SIGNAL,
                                       gcmINT2PTR(Interface->u.UserSignal.id),
                                       gcvNULL,
                                       0));
            break;

        case gcvUSER_SIGNAL_DESTROY:
            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));

            /* Destroy the signal. */
            gcmkONERROR(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));
            break;

        case gcvUSER_SIGNAL_SIGNAL:
            /* Signal the signal. */
            gcmkONERROR(
                gckOS_SignalUserSignal(Kernel->os,
                                       Interface->u.UserSignal.id,
                                       Interface->u.UserSignal.state));
            break;

        case gcvUSER_SIGNAL_WAIT:
            /* Wait on the signal. */
            status = gckOS_WaitUserSignal(Kernel->os,
                                          Interface->u.UserSignal.id,
                                          Interface->u.UserSignal.wait);
            break;

        case gcvUSER_SIGNAL_MAP:
            gcmkONERROR(
                gckOS_MapSignal(Kernel->os,
                               (gctSIGNAL)(gctUINTPTR_T)Interface->u.UserSignal.id,
                               (gctHANDLE)(gctUINTPTR_T)processID,
                               &signal));

            gcmkVERIFY_OK(
                gckKERNEL_AddProcessDB(Kernel,
                                       processID, gcvDB_SIGNAL,
                                       gcmINT2PTR(Interface->u.UserSignal.id),
                                       gcvNULL,
                                       0));
            break;

        case gcvUSER_SIGNAL_UNMAP:
            gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
                Kernel,
                processID, gcvDB_SIGNAL,
                gcmINT2PTR(Interface->u.UserSignal.id)));

            /* Destroy the signal. */
            gcmkONERROR(
                gckOS_DestroyUserSignal(Kernel->os,
                                        Interface->u.UserSignal.id));
            break;

        default:
            /* Invalid user signal command. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        break;
#endif

    case gcvHAL_SET_POWER_MANAGEMENT_STATE:
        /* Set the power management state. */
        gcmkONERROR(
            gckHARDWARE_SetPowerManagementState(
                Kernel->hardware,
                Interface->u.SetPowerManagement.state));
        break;

    case gcvHAL_QUERY_POWER_MANAGEMENT_STATE:
        /* Chip is not idle. */
        Interface->u.QueryPowerManagement.isIdle = gcvFALSE;

        /* Query the power management state. */
        gcmkONERROR(gckHARDWARE_QueryPowerManagementState(
            Kernel->hardware,
            &Interface->u.QueryPowerManagement.state));

        /* Query the idle state. */
        gcmkONERROR(
            gckHARDWARE_QueryIdle(Kernel->hardware,
                                  &Interface->u.QueryPowerManagement.isIdle));
        break;

    case gcvHAL_READ_REGISTER:
#if gcdREGISTER_ACCESS_FROM_USER
        {
            gceCHIPPOWERSTATE power;

            gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE));
            powerMutexAcquired = gcvTRUE;
            gcmkONERROR(gckHARDWARE_QueryPowerManagementState(Kernel->hardware,
                                                              &power));
            if (power == gcvPOWER_ON)
            {
                /* Read a register. */
                gcmkONERROR(gckOS_ReadRegisterEx(
                    Kernel->os,
                    Kernel->core,
                    Interface->u.ReadRegisterData.address,
                    &Interface->u.ReadRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.ReadRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
            powerMutexAcquired = gcvFALSE;
        }
#else
        /* No access from user land to read registers. */
        Interface->u.ReadRegisterData.data = 0;
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_WRITE_REGISTER:
#if gcdREGISTER_ACCESS_FROM_USER
        {
            gceCHIPPOWERSTATE power;

            gcmkONERROR(gckOS_AcquireMutex(Kernel->os, Kernel->hardware->powerMutex, gcvINFINITE));
            powerMutexAcquired = gcvTRUE;
            gcmkONERROR(gckHARDWARE_QueryPowerManagementState(Kernel->hardware,
                                                                  &power));
            if (power == gcvPOWER_ON)
            {
                /* Write a register. */
                gcmkONERROR(
                    gckOS_WriteRegisterEx(Kernel->os,
                                          Kernel->core,
                                          Interface->u.WriteRegisterData.address,
                                          Interface->u.WriteRegisterData.data));
            }
            else
            {
                /* Chip is in power-state. */
                Interface->u.WriteRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;
            }
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
            powerMutexAcquired = gcvFALSE;
        }
#else
        /* No access from user land to write registers. */
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_READ_ALL_PROFILE_REGISTERS_PART1:
        /* Read profile data according to the context. */
        gcmkONERROR(
            gckHARDWARE_QueryContextProfile(
                Kernel->hardware,
                Kernel->profileCleanRegister,
                gcmNAME_TO_PTR(Interface->u.RegisterProfileData_part1.context),
                &Interface->u.RegisterProfileData_part1.Counters,
                gcvNULL));
        break;
    case gcvHAL_READ_ALL_PROFILE_REGISTERS_PART2:
        /* Read profile data according to the context. */
        gcmkONERROR(
            gckHARDWARE_QueryContextProfile(
                Kernel->hardware,
                Kernel->profileCleanRegister,
                gcmNAME_TO_PTR(Interface->u.RegisterProfileData_part2.context),
                gcvNULL,
                &Interface->u.RegisterProfileData_part2.Counters));
        break;

    case gcvHAL_GET_PROFILE_SETTING:
#if VIVANTE_PROFILER
        /* Get profile setting */
        Interface->u.GetProfileSetting.enable = Kernel->profileEnable;
#endif

        status = gcvSTATUS_OK;
        break;

    case gcvHAL_SET_PROFILE_SETTING:
#if VIVANTE_PROFILER
        /* Set profile setting */
        if(Kernel->hardware->options.gpuProfiler)
        {
            Kernel->profileEnable = Interface->u.SetProfileSetting.enable;

            if (Kernel->profileEnable)
            {
                gcmkONERROR(gckHARDWARE_InitProfiler(Kernel->hardware));
            }

        }
        else
        {
            status = gcvSTATUS_NOT_SUPPORTED;
            break;
        }
#endif

        status = gcvSTATUS_OK;
        break;

    case gcvHAL_READ_PROFILER_REGISTER_SETTING:
        Kernel->profileCleanRegister = Interface->u.SetProfilerRegisterClear.bclear;
        status = gcvSTATUS_OK;
        break;

    case gcvHAL_QUERY_KERNEL_SETTINGS:
        /* Get kernel settings. */
        gcmkONERROR(
            gckKERNEL_QuerySettings(Kernel,
                                    &Interface->u.QueryKernelSettings.settings));
        break;

    case gcvHAL_RESET:
        /* Reset the hardware. */
        gcmkONERROR(
            gckHARDWARE_Reset(Kernel->hardware));
        break;

    case gcvHAL_DEBUG:
        /* Set debug level and zones. */
        if (Interface->u.Debug.set)
        {
            gckOS_SetDebugLevel(Interface->u.Debug.level);
            gckOS_SetDebugZones(Interface->u.Debug.zones,
                                Interface->u.Debug.enable);
        }

        if (Interface->u.Debug.message[0] != '\0')
        {
            /* Print a message to the debugger. */
            if (Interface->u.Debug.type == gcvMESSAGE_TEXT)
            {
               gckOS_DumpBuffer(Kernel->os,
                                Interface->u.Debug.message,
                                gcmSIZEOF(Interface->u.Debug.message),
                                gcvDUMP_BUFFER_FROM_USER,
                                gcvTRUE);
            }
            else
            {
               gckOS_DumpBuffer(Kernel->os,
                                Interface->u.Debug.message,
                                Interface->u.Debug.messageSize,
                                gcvDUMP_BUFFER_FROM_USER,
                                gcvTRUE);
            }
        }
        status = gcvSTATUS_OK;
        break;

    case gcvHAL_DUMP_GPU_STATE:
        {
            gceCHIPPOWERSTATE power;

            _DumpDriverConfigure(Kernel);

            gcmkONERROR(gckHARDWARE_QueryPowerManagementState(
                Kernel->hardware,
                &power
                ));

            if (power == gcvPOWER_ON)
            {
                Interface->u.ReadRegisterData.data = 1;

                _DumpState(Kernel);
            }
            else
            {
                Interface->u.ReadRegisterData.data = 0;
                status = gcvSTATUS_CHIP_NOT_READY;

                gcmkPRINT("[galcore]: Can't dump state if GPU isn't POWER ON.");
            }
        }
        break;

    case gcvHAL_DUMP_EVENT:
        break;

    case gcvHAL_CACHE:
        logical = gcmUINT64_TO_PTR(Interface->u.Cache.logical);
        bytes = (gctSIZE_T) Interface->u.Cache.bytes;

        gcmkONERROR(gckKERNEL_CacheOperation(Kernel,
                                             processID,
                                             Interface->u.Cache.node,
                                             Interface->u.Cache.operation,
                                             logical,
                                             bytes));
        break;

    case gcvHAL_TIMESTAMP:
        /* Check for invalid timer. */
        if ((Interface->u.TimeStamp.timer >= gcmCOUNTOF(Kernel->timers))
        ||  (Interface->u.TimeStamp.request != 2))
        {
            Interface->u.TimeStamp.timeDelta = 0;
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        /* Return timer results and reset timer. */
        {
            gcsTIMER_PTR timer = &(Kernel->timers[Interface->u.TimeStamp.timer]);
            gctUINT64 timeDelta = 0;

            if (timer->stopTime < timer->startTime )
            {
                Interface->u.TimeStamp.timeDelta = 0;
                gcmkONERROR(gcvSTATUS_TIMER_OVERFLOW);
            }

            timeDelta = timer->stopTime - timer->startTime;

            /* Check truncation overflow. */
            Interface->u.TimeStamp.timeDelta = (gctINT32) timeDelta;
            /*bit0~bit30 is available*/
            if (timeDelta>>31)
            {
                Interface->u.TimeStamp.timeDelta = 0;
                gcmkONERROR(gcvSTATUS_TIMER_OVERFLOW);
            }

            status = gcvSTATUS_OK;
        }
        break;

    case gcvHAL_DATABASE:
        gcmkONERROR(gckKERNEL_QueryDatabase(Kernel, processID, Interface));
        break;

    case gcvHAL_VERSION:
        Interface->u.Version.major = gcvVERSION_MAJOR;
        Interface->u.Version.minor = gcvVERSION_MINOR;
        Interface->u.Version.patch = gcvVERSION_PATCH;
        Interface->u.Version.build = gcvVERSION_BUILD;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                       "KERNEL version %d.%d.%d build %u",
                       gcvVERSION_MAJOR, gcvVERSION_MINOR,
                       gcvVERSION_PATCH, gcvVERSION_BUILD);
#endif
        break;

    case gcvHAL_CHIP_INFO:
        /* Only if not support multi-core */
        Interface->u.ChipInfo.count = 1;
        Interface->u.ChipInfo.types[0] = Kernel->hardware->type;
        break;

#if (gcdENABLE_3D || gcdENABLE_2D)
    case gcvHAL_ATTACH:
        /* Attach user process. */
        gcmkONERROR(
            gckCOMMAND_Attach(Kernel->command,
                              &context,
                              &bytes,
                              &Interface->u.Attach.numStates,
                              processID));

        Interface->u.Attach.maxState = bytes;
        Interface->u.Attach.context = gcmPTR_TO_NAME(context);

        if (Interface->u.Attach.map == gcvTRUE)
        {
            gcmkVERIFY_OK(
                gckCONTEXT_MapBuffer(context,
                                     Interface->u.Attach.physicals,
                                     Interface->u.Attach.logicals,
                                     &Interface->u.Attach.bytes));
        }

        gcmkVERIFY_OK(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_CONTEXT,
                                   gcmINT2PTR(Interface->u.Attach.context),
                                   gcvNULL,
                                   0));
        break;
#endif

    case gcvHAL_DETACH:
        gcmkVERIFY_OK(
            gckKERNEL_RemoveProcessDB(Kernel,
                              processID, gcvDB_CONTEXT,
                              gcmINT2PTR(Interface->u.Detach.context)));

        /* Detach user process. */
        gcmkONERROR(
            gckCOMMAND_Detach(Kernel->command,
                              gcmNAME_TO_PTR(Interface->u.Detach.context)));

        gcmRELEASE_NAME(Interface->u.Detach.context);
        gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
             Kernel->device->commitMutex,
             gcvINFINITE
             ));

        commitMutexAcquired = gcvTRUE;

        gcmkONERROR(gckEVENT_Submit(
            Kernel->eventObj,
            gcvTRUE,
            gcvFALSE
            ));

        gcmkONERROR(gckOS_ReleaseMutex(Kernel->os,
            Kernel->device->commitMutex
            ));

        commitMutexAcquired = gcvFALSE;
        break;

    case gcvHAL_GET_FRAME_INFO:
        gcmkONERROR(gckHARDWARE_GetFrameInfo(
                    Kernel->hardware,
                    gcmUINT64_TO_PTR(Interface->u.GetFrameInfo.frameInfo)));
        break;

    case gcvHAL_DUMP_GPU_PROFILE:
        gcmkONERROR(gckHARDWARE_DumpGpuProfile(Kernel->hardware));
        break;

    case gcvHAL_SET_FSCALE_VALUE:
#if gcdENABLE_FSCALE_VAL_ADJUST
        status = gckHARDWARE_SetFscaleValue(Kernel->hardware,
                                            Interface->u.SetFscaleValue.value);
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;
    case gcvHAL_GET_FSCALE_VALUE:
#if gcdENABLE_FSCALE_VAL_ADJUST
        status = gckHARDWARE_GetFscaleValue(Kernel->hardware,
                                            &Interface->u.GetFscaleValue.value,
                                            &Interface->u.GetFscaleValue.minValue,
                                            &Interface->u.GetFscaleValue.maxValue);
#else
        status = gcvSTATUS_NOT_SUPPORTED;
#endif
        break;

    case gcvHAL_EXPORT_VIDEO_MEMORY:
        /* Unlock video memory. */
        gcmkONERROR(gckVIDMEM_NODE_Export(Kernel,
                                          Interface->u.ExportVideoMemory.node,
                                          Interface->u.ExportVideoMemory.flags,
                                          gcvNULL,
                                          &Interface->u.ExportVideoMemory.fd));
        break;

    case gcvHAL_NAME_VIDEO_MEMORY:
        gcmkONERROR(gckVIDMEM_NODE_Name(Kernel,
                                        Interface->u.NameVideoMemory.handle,
                                        &Interface->u.NameVideoMemory.name));
        break;

    case gcvHAL_IMPORT_VIDEO_MEMORY:
        gcmkONERROR(gckVIDMEM_NODE_Import(Kernel,
                                          Interface->u.ImportVideoMemory.name,
                                          &Interface->u.ImportVideoMemory.handle));

        gcmkONERROR(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID, gcvDB_VIDEO_MEMORY,
                                   gcmINT2PTR(Interface->u.ImportVideoMemory.handle),
                                   gcvNULL,
                                   0));
        break;

    case gcvHAL_SET_VIDEO_MEMORY_METADATA:
        gcmkONERROR(gckKERNEL_SetVidMemMetadata(Kernel, processID, Interface));
        break;

    case gcvHAL_GET_VIDEO_MEMORY_FD:
        gcmkONERROR(gckVIDMEM_NODE_GetFd(
            Kernel,
            Interface->u.GetVideoMemoryFd.handle,
            &Interface->u.GetVideoMemoryFd.fd
            ));

        /* No need to add it to processDB because OS will release all fds when
        ** process quits.
        */
        break;

    case gcvHAL_QUERY_RESET_TIME_STAMP:
        Interface->u.QueryResetTimeStamp.timeStamp = Kernel->resetTimeStamp;
        Interface->u.QueryResetTimeStamp.contextID = Kernel->hardware->contextID;
        break;

    case gcvHAL_FREE_VIRTUAL_COMMAND_BUFFER:
        buffer = (gckVIRTUAL_COMMAND_BUFFER_PTR)gcmNAME_TO_PTR(Interface->u.FreeVirtualCommandBuffer.physical);

        gcmkVERIFY_OK(gckKERNEL_RemoveProcessDB(
            Kernel,
            processID,
            gcvDB_COMMAND_BUFFER,
            gcmUINT64_TO_PTR(Interface->u.FreeVirtualCommandBuffer.logical)));

        gcmkONERROR(gckOS_DestroyUserVirtualMapping(
            Kernel->os,
            buffer->virtualBuffer.physical,
            (gctSIZE_T)Interface->u.FreeVirtualCommandBuffer.bytes,
            gcmUINT64_TO_PTR(Interface->u.FreeVirtualCommandBuffer.logical)));

        gcmkONERROR(gckKERNEL_DestroyVirtualCommandBuffer(
            Kernel,
            (gctSIZE_T)Interface->u.FreeVirtualCommandBuffer.bytes,
            (gctPHYS_ADDR)buffer,
            gcmUINT64_TO_PTR(Interface->u.FreeVirtualCommandBuffer.logical)));

        gcmRELEASE_NAME(Interface->u.FreeVirtualCommandBuffer.physical);
        break;

#if gcdLINUX_SYNC_FILE
    case gcvHAL_CREATE_NATIVE_FENCE:
        {
            gctINT fenceFD;
            gctSIGNAL signal =
                gcmUINT64_TO_PTR(Interface->u.CreateNativeFence.signal);

            gcmkONERROR(
                gckOS_CreateNativeFence(Kernel->os,
                                        Kernel->timeline,
                                        signal,
                                        &fenceFD));

            Interface->u.CreateNativeFence.fenceFD = fenceFD;
        }
        break;

    case gcvHAL_WAIT_NATIVE_FENCE:
        {
            gctINT fenceFD;
            gctUINT32 timeout;

            fenceFD = Interface->u.WaitNativeFence.fenceFD;
            timeout = Interface->u.WaitNativeFence.timeout;

            gcmkONERROR(
                gckOS_WaitNativeFence(Kernel->os,
                                      Kernel->timeline,
                                      fenceFD,
                                      timeout));
        }
        break;
#endif

    case gcvHAL_SHBUF:
        {
            gctSHBUF shBuf;
            gctPOINTER uData;
            gctUINT32 bytes;

            switch (Interface->u.ShBuf.command)
            {
            case gcvSHBUF_CREATE:
                bytes = Interface->u.ShBuf.bytes;

                /* Create. */
                gcmkONERROR(gckKERNEL_CreateShBuffer(Kernel, bytes, &shBuf));

                Interface->u.ShBuf.id = gcmPTR_TO_UINT64(shBuf);

                gcmkVERIFY_OK(
                    gckKERNEL_AddProcessDB(Kernel,
                                           processID,
                                           gcvDB_SHBUF,
                                           shBuf,
                                           gcvNULL,
                                           0));
                break;

            case gcvSHBUF_DESTROY:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);

                /* Check db first to avoid illegal destroy in the process. */
                gcmkONERROR(
                    gckKERNEL_RemoveProcessDB(Kernel,
                                              processID,
                                              gcvDB_SHBUF,
                                              shBuf));

                gcmkONERROR(gckKERNEL_DestroyShBuffer(Kernel, shBuf));
                break;

            case gcvSHBUF_MAP:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);

                /* Map for current process access. */
                gcmkONERROR(gckKERNEL_MapShBuffer(Kernel, shBuf));

                gcmkVERIFY_OK(
                    gckKERNEL_AddProcessDB(Kernel,
                                           processID,
                                           gcvDB_SHBUF,
                                           shBuf,
                                           gcvNULL,
                                           0));
                break;

            case gcvSHBUF_WRITE:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);
                uData = gcmUINT64_TO_PTR(Interface->u.ShBuf.data);
                bytes = Interface->u.ShBuf.bytes;

                /* Write. */
                gcmkONERROR(
                    gckKERNEL_WriteShBuffer(Kernel, shBuf, uData, bytes));
                break;

            case gcvSHBUF_READ:
                shBuf = gcmUINT64_TO_PTR(Interface->u.ShBuf.id);
                uData = gcmUINT64_TO_PTR(Interface->u.ShBuf.data);
                bytes = Interface->u.ShBuf.bytes;

                /* Read. */
                gcmkONERROR(
                    gckKERNEL_ReadShBuffer(Kernel,
                                           shBuf,
                                           uData,
                                           bytes,
                                           &bytes));

                /* Return copied size. */
                Interface->u.ShBuf.bytes = bytes;
                break;

            default:
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
                break;
            }
        }
        break;

#ifdef __linux__
    case gcvHAL_GET_GRAPHIC_BUFFER_FD:
        gcmkONERROR(_GetGraphicBufferFd(
            Kernel,
            processID,
            Interface->u.GetGraphicBufferFd.node,
            Interface->u.GetGraphicBufferFd.shBuf,
            Interface->u.GetGraphicBufferFd.signal,
            &Interface->u.GetGraphicBufferFd.fd
            ));
        break;
#endif


    case gcvHAL_CONFIG_POWER_MANAGEMENT:
        gcmkONERROR(gckKERNEL_ConfigPowerManagement(Kernel, Interface));
        break;

    case gcvHAL_WRAP_USER_MEMORY:
        gcmkONERROR(gckVIDMEM_NODE_WrapUserMemory(Kernel,
                                                  &Interface->u.WrapUserMemory.desc,
                                                  &Interface->u.WrapUserMemory.node,
                                                  &Interface->u.WrapUserMemory.bytes));

        gcmkONERROR(
            gckKERNEL_AddProcessDB(Kernel,
                                   processID,
                                   gcvDB_VIDEO_MEMORY,
                                   gcmINT2PTR(Interface->u.WrapUserMemory.node),
                                   gcvNULL,
                                   0));
        break;

    case gcvHAL_WAIT_FENCE:
        gcmkONERROR(gckKERNEL_WaitFence(
            Kernel,
            Interface->u.WaitFence.handle,
            Interface->u.WaitFence.timeOut
            ));
        break;

    case gcvHAL_DEVICE_MUTEX:
        if (Interface->u.DeviceMutex.isMutexLocked)
        {
            gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                Kernel->device->commitMutex,
                gcvINFINITE
                ));
        }
        else
        {
            gcmkONERROR(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
        }
        break;

#if gcdDEC_ENABLE_AHB
    case gcvHAL_DEC300_READ:
        gcmkONERROR(viv_dec300_read(
            Interface->u.DEC300Read.enable,
            Interface->u.DEC300Read.readId,
            Interface->u.DEC300Read.format,
            Interface->u.DEC300Read.strides,
            Interface->u.DEC300Read.is3D,
            Interface->u.DEC300Read.isMSAA,
            Interface->u.DEC300Read.clearValue,
            Interface->u.DEC300Read.isTPC,
            Interface->u.DEC300Read.isTPCCompressed,
            Interface->u.DEC300Read.surfAddrs,
            Interface->u.DEC300Read.tileAddrs
            ));
        break;

    case gcvHAL_DEC300_WRITE:
        gcmkONERROR(viv_dec300_write(
            Interface->u.DEC300Write.enable,
            Interface->u.DEC300Write.readId,
            Interface->u.DEC300Write.writeId,
            Interface->u.DEC300Write.format,
            Interface->u.DEC300Write.surfAddr,
            Interface->u.DEC300Write.tileAddr
            ));
        break;

    case gcvHAL_DEC300_FLUSH:
        gcmkONERROR(viv_dec300_flush(0));
        break;

    case gcvHAL_DEC300_FLUSH_WAIT:
        gcmkONERROR(viv_dec300_flush_done(&Interface->u.DEC300FlushWait.done));
        break;
#endif


    case gcvHAL_QUERY_CHIP_OPTION:
        /* Query chip options. */
        gcmkONERROR(
            gckHARDWARE_QueryChipOptions(
                Kernel->hardware,
                &Interface->u.QueryChipOptions));
        break;

    default:
        /* Invalid command. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

OnError:
    /* Save status. */
    Interface->status = status;

#if QNX_SINGLE_THREADED_DEBUGGING
    gckOS_ReleaseMutex(Kernel->os, Kernel->debugMutex);
#endif

    if (powerMutexAcquired == gcvTRUE)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->hardware->powerMutex));
    }

    if (commitMutexAcquired == gcvTRUE)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->device->commitMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckKERNEL_AttachProcess
**
**  Attach or detach a process.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL Attach
**          gcvTRUE if a new process gets attached or gcFALSE when a process
**          gets detatched.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_AttachProcess(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach
    )
{
    gceSTATUS status;
    gctUINT32 processID;

    gcmkHEADER_ARG("Kernel=0x%x Attach=%d", Kernel, Attach);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Get current process ID. */
    gcmkONERROR(gckOS_GetProcessID(&processID));

    gcmkONERROR(gckKERNEL_AttachProcessEx(Kernel, Attach, processID));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**  gckKERNEL_AttachProcessEx
**
**  Attach or detach a process with the given PID. Can be paired with gckKERNEL_AttachProcess
**     provided the programmer is aware of the consequences.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL Attach
**          gcvTRUE if a new process gets attached or gcFALSE when a process
**          gets detatched.
**
**      gctUINT32 PID
**          PID of the process to attach or detach.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_AttachProcessEx(
    IN gckKERNEL Kernel,
    IN gctBOOL Attach,
    IN gctUINT32 PID
    )
{
    gceSTATUS status;
    gctINT32 old;

    gcmkHEADER_ARG("Kernel=0x%x Attach=%d PID=%d", Kernel, Attach, PID);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Attach)
    {
        /* Increment the number of clients attached. */
        gcmkONERROR(
            gckOS_AtomIncrement(Kernel->os, Kernel->atomClients, &old));

        if (old == 0)
        {
#if gcdENABLE_VG
            if (Kernel->vg == gcvNULL)
#endif
            {
                gcmkONERROR(gckOS_Broadcast(Kernel->os,
                                            Kernel->hardware,
                                            gcvBROADCAST_FIRST_PROCESS));
            }
        }

        if (Kernel->dbCreated)
        {
            /* Create the process database. */
            gcmkONERROR(gckKERNEL_CreateProcessDB(Kernel, PID));
        }

#if gcdPROCESS_ADDRESS_SPACE
        /* Map kernel command buffer in the process's own MMU. */
        gcmkONERROR(_MapCommandBuffer(Kernel));
#endif
    }
    else
    {
        if (Kernel->dbCreated)
        {
            /* Clean up the process database. */
            gcmkONERROR(gckKERNEL_DestroyProcessDB(Kernel, PID));

            /* Save the last know process ID. */
            Kernel->db->lastProcessID = PID;
        }

#if gcdENABLE_VG
        if (Kernel->vg == gcvNULL)
#endif
        {
            status = gckEVENT_Submit(Kernel->eventObj, gcvTRUE, gcvFALSE);

            if (status == gcvSTATUS_INTERRUPTED && Kernel->eventObj->submitTimer)
            {
                gcmkONERROR(gckOS_StartTimer(Kernel->os,
                                             Kernel->eventObj->submitTimer,
                                             1));
            }
            else
            {
                gcmkONERROR(status);
            }
        }

        /* Decrement the number of clients attached. */
        gcmkONERROR(
            gckOS_AtomDecrement(Kernel->os, Kernel->atomClients, &old));

        if (old == 1)
        {
#if gcdENABLE_VG
            if (Kernel->vg == gcvNULL)
#endif
            {
                /* Last client detached, switch to SUSPEND power state. */
                gcmkONERROR(gckOS_Broadcast(Kernel->os,
                                            Kernel->hardware,
                                            gcvBROADCAST_LAST_PROCESS));
            }

            /* Flush the debug cache. */
            gcmkDEBUGFLUSH(~0U);
        }
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

#if gcdSECURE_USER
gceSTATUS
gckKERNEL_MapLogicalToPhysical(
    IN gckKERNEL Kernel,
    IN gcskSECURE_CACHE_PTR Cache,
    IN OUT gctPOINTER * Data
    )
{
    gceSTATUS status;
    static gctBOOL baseAddressValid = gcvFALSE;
    static gctUINT32 baseAddress;
    gctBOOL needBase;
    gcskLOGICAL_CACHE_PTR slot;

    gcmkHEADER_ARG("Kernel=0x%x Cache=0x%x *Data=0x%x",
                   Kernel, Cache, gcmOPT_POINTER(Data));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (!baseAddressValid)
    {
        /* Get base address. */
        gcmkONERROR(gckHARDWARE_GetBaseAddress(Kernel->hardware, &baseAddress));

        baseAddressValid = gcvTRUE;
    }

    /* Does this state load need a base address? */
    gcmkONERROR(gckHARDWARE_NeedBaseAddress(Kernel->hardware,
                                            ((gctUINT32_PTR) Data)[-1],
                                            &needBase));

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LRU
    {
        gcskLOGICAL_CACHE_PTR next;
        gctINT i;

        /* Walk all used cache slots. */
        for (i = 1, slot = Cache->cache[0].next, next = gcvNULL;
             (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
             ++i, slot = slot->next
        )
        {
            if (slot->logical == *Data)
            {
                /* Bail out. */
                next = slot;
                break;
            }
        }

        /* See if we had a miss. */
        if (next == gcvNULL)
        {
            /* Use the tail of the cache. */
            slot = Cache->cache[0].prev;

            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, slot->dma, &slot->dma));
        }

        /* Move slot to head of list. */
        if (slot != Cache->cache[0].next)
        {
            /* Unlink. */
            slot->prev->next = slot->next;
            slot->next->prev = slot->prev;

            /* Move to head of chain. */
            slot->prev       = &Cache->cache[0];
            slot->next       = Cache->cache[0].next;
            slot->prev->next = slot;
            slot->next->prev = slot;
        }
    }
#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LINEAR
    {
        gctINT i;
        gcskLOGICAL_CACHE_PTR next = gcvNULL;
        gcskLOGICAL_CACHE_PTR oldestSlot = gcvNULL;
        slot = gcvNULL;

        if (Cache->cacheIndex != gcvNULL)
        {
            /* Walk the cache forwards. */
            for (i = 1, slot = Cache->cacheIndex;
                 (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
                 ++i, slot = slot->next)
            {
                if (slot->logical == *Data)
                {
                    /* Bail out. */
                    next = slot;
                    break;
                }

                /* Determine age of this slot. */
                if ((oldestSlot       == gcvNULL)
                ||  (oldestSlot->stamp > slot->stamp)
                )
                {
                    oldestSlot = slot;
                }
            }

            if (next == gcvNULL)
            {
                /* Walk the cache backwards. */
                for (slot = Cache->cacheIndex->prev;
                     (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
                     ++i, slot = slot->prev)
                {
                    if (slot->logical == *Data)
                    {
                        /* Bail out. */
                        next = slot;
                        break;
                    }

                    /* Determine age of this slot. */
                    if ((oldestSlot       == gcvNULL)
                    ||  (oldestSlot->stamp > slot->stamp)
                    )
                    {
                        oldestSlot = slot;
                    }
                }
            }
        }

        /* See if we had a miss. */
        if (next == gcvNULL)
        {
            if (Cache->cacheFree != 0)
            {
                slot = &Cache->cache[Cache->cacheFree];
                gcmkASSERT(slot->logical == gcvNULL);

                ++ Cache->cacheFree;
                if (Cache->cacheFree >= gcmCOUNTOF(Cache->cache))
                {
                    Cache->cacheFree = 0;
                }
            }
            else
            {
                /* Use the oldest cache slot. */
                gcmkASSERT(oldestSlot != gcvNULL);
                slot = oldestSlot;

                /* Unlink from the chain. */
                slot->prev->next = slot->next;
                slot->next->prev = slot->prev;

                /* Append to the end. */
                slot->prev       = Cache->cache[0].prev;
                slot->next       = &Cache->cache[0];
                slot->prev->next = slot;
                slot->next->prev = slot;
            }

            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, slot->dma, &slot->dma));
        }

        /* Save time stamp. */
        slot->stamp = ++ Cache->cacheStamp;

        /* Save current slot for next lookup. */
        Cache->cacheIndex = slot;
    }
#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
    {
        gctINT i;
        gctUINT32 data = gcmPTR2INT32(*Data);
        gctUINT32 key, index;
        gcskLOGICAL_CACHE_PTR hash;

        /* Generate a hash key. */
        key   = (data >> 24) + (data >> 16) + (data >> 8) + data;
        index = key % gcmCOUNTOF(Cache->hash);

        /* Get the hash entry. */
        hash = &Cache->hash[index];

        for (slot = hash->nextHash, i = 0;
             (slot != gcvNULL) && (i < gcdSECURE_CACHE_SLOTS);
             slot = slot->nextHash, ++i
        )
        {
            if (slot->logical == (*Data))
            {
                break;
            }
        }

        if (slot == gcvNULL)
        {
            /* Grab from the tail of the cache. */
            slot = Cache->cache[0].prev;

            /* Unlink slot from any hash table it is part of. */
            if (slot->prevHash != gcvNULL)
            {
                slot->prevHash->nextHash = slot->nextHash;
            }
            if (slot->nextHash != gcvNULL)
            {
                slot->nextHash->prevHash = slot->prevHash;
            }

            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, slot->dma, &slot->dma));

            if (hash->nextHash != gcvNULL)
            {
                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                               "Hash Collision: logical=0x%x key=0x%08x",
                               *Data, key);
            }

            /* Insert the slot at the head of the hash list. */
            slot->nextHash     = hash->nextHash;
            if (slot->nextHash != gcvNULL)
            {
                slot->nextHash->prevHash = slot;
            }
            slot->prevHash     = hash;
            hash->nextHash     = slot;
        }

        /* Move slot to head of list. */
        if (slot != Cache->cache[0].next)
        {
            /* Unlink. */
            slot->prev->next = slot->next;
            slot->next->prev = slot->prev;

            /* Move to head of chain. */
            slot->prev       = &Cache->cache[0];
            slot->next       = Cache->cache[0].next;
            slot->prev->next = slot;
            slot->next->prev = slot;
        }
    }
#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_TABLE
    {
        gctUINT32 index = (gcmPTR2INT32(*Data) % gcdSECURE_CACHE_SLOTS) + 1;

        /* Get cache slot. */
        slot = &Cache->cache[index];

        /* Check for cache miss. */
        if (slot->logical != *Data)
        {
            /* Initialize the cache line. */
            slot->logical = *Data;

            /* Map the logical address to a DMA address. */
            gcmkONERROR(
                gckOS_GetPhysicalAddress(Kernel->os, *Data, &slot->dma));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, slot->dma, &slot->dma));
        }
    }
#endif

    /* Return DMA address. */
    *Data = gcmINT2PTR(slot->dma + (needBase ? baseAddress : 0));

    /* Success. */
    gcmkFOOTER_ARG("*Data=0x%08x", *Data);
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_FlushTranslationCache(
    IN gckKERNEL Kernel,
    IN gcskSECURE_CACHE_PTR Cache,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes
    )
{
    gctINT i;
    gcskLOGICAL_CACHE_PTR slot;
    gctUINT8_PTR ptr;

    gcmkHEADER_ARG("Kernel=0x%x Cache=0x%x Logical=0x%x Bytes=%lu",
                   Kernel, Cache, Logical, Bytes);

    /* Do we need to flush the entire cache? */
    if (Logical == gcvNULL)
    {
        /* Clear all cache slots. */
        for (i = 1; i <= gcdSECURE_CACHE_SLOTS; ++i)
        {
            Cache->cache[i].logical  = gcvNULL;

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
            Cache->cache[i].nextHash = gcvNULL;
            Cache->cache[i].prevHash = gcvNULL;
#endif
}

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
        /* Zero the hash table. */
        for (i = 0; i < gcmCOUNTOF(Cache->hash); ++i)
        {
            Cache->hash[i].nextHash = gcvNULL;
        }
#endif

        /* Reset the cache functionality. */
        Cache->cacheIndex = gcvNULL;
        Cache->cacheFree  = 1;
        Cache->cacheStamp = 0;
    }

    else
    {
        gctUINT8_PTR low  = (gctUINT8_PTR) Logical;
        gctUINT8_PTR high = low + Bytes;

#if gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LRU
        gcskLOGICAL_CACHE_PTR next;

        /* Walk all used cache slots. */
        for (i = 1, slot = Cache->cache[0].next;
             (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
             ++i, slot = next
        )
        {
            /* Save pointer to next slot. */
            next = slot->next;

            /* Test if this slot falls within the range to flush. */
            ptr = (gctUINT8_PTR) slot->logical;
            if ((ptr >= low) && (ptr < high))
            {
                /* Unlink slot. */
                slot->prev->next = slot->next;
                slot->next->prev = slot->prev;

                /* Append slot to tail of cache. */
                slot->prev       = Cache->cache[0].prev;
                slot->next       = &Cache->cache[0];
                slot->prev->next = slot;
                slot->next->prev = slot;

                /* Mark slot as empty. */
                slot->logical = gcvNULL;
            }
        }

#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_LINEAR
        gcskLOGICAL_CACHE_PTR next;

        for (i = 1, slot = Cache->cache[0].next;
             (i <= gcdSECURE_CACHE_SLOTS) && (slot->logical != gcvNULL);
             ++i, slot = next)
        {
            /* Save pointer to next slot. */
            next = slot->next;

            /* Test if this slot falls within the range to flush. */
            ptr = (gctUINT8_PTR) slot->logical;
            if ((ptr >= low) && (ptr < high))
            {
                /* Test if this slot is the current slot. */
                if (slot == Cache->cacheIndex)
                {
                    /* Move to next or previous slot. */
                    Cache->cacheIndex = (slot->next->logical != gcvNULL)
                                      ? slot->next
                                      : (slot->prev->logical != gcvNULL)
                                      ? slot->prev
                                      : gcvNULL;
                }

                /* Unlink slot from cache. */
                slot->prev->next = slot->next;
                slot->next->prev = slot->prev;

                /* Insert slot to head of cache. */
                slot->prev       = &Cache->cache[0];
                slot->next       = Cache->cache[0].next;
                slot->prev->next = slot;
                slot->next->prev = slot;

                /* Mark slot as empty. */
                slot->logical = gcvNULL;
                slot->stamp   = 0;
            }
        }

#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_HASH
        gctINT j;
        gcskLOGICAL_CACHE_PTR hash, next;

        /* Walk all hash tables. */
        for (i = 0, hash = Cache->hash;
             i < gcmCOUNTOF(Cache->hash);
             ++i, ++hash)
        {
            /* Walk all slots in the hash. */
            for (j = 0, slot = hash->nextHash;
                 (j < gcdSECURE_CACHE_SLOTS) && (slot != gcvNULL);
                 ++j, slot = next)
            {
                /* Save pointer to next slot. */
                next = slot->next;

                /* Test if this slot falls within the range to flush. */
                ptr = (gctUINT8_PTR) slot->logical;
                if ((ptr >= low) && (ptr < high))
                {
                    /* Unlink slot from hash table. */
                    if (slot->prevHash == hash)
                    {
                        hash->nextHash = slot->nextHash;
                    }
                    else
                    {
                        slot->prevHash->nextHash = slot->nextHash;
                    }

                    if (slot->nextHash != gcvNULL)
                    {
                        slot->nextHash->prevHash = slot->prevHash;
                    }

                    /* Unlink slot from cache. */
                    slot->prev->next = slot->next;
                    slot->next->prev = slot->prev;

                    /* Append slot to tail of cache. */
                    slot->prev       = Cache->cache[0].prev;
                    slot->next       = &Cache->cache[0];
                    slot->prev->next = slot;
                    slot->next->prev = slot;

                    /* Mark slot as empty. */
                    slot->logical  = gcvNULL;
                    slot->prevHash = gcvNULL;
                    slot->nextHash = gcvNULL;
                }
            }
        }

#elif gcdSECURE_CACHE_METHOD == gcdSECURE_CACHE_TABLE
        gctUINT32 index;

        /* Loop while inside the range. */
        for (i = 1; (low < high) && (i <= gcdSECURE_CACHE_SLOTS); ++i)
        {
            /* Get index into cache for this range. */
            index = (gcmPTR2INT32(low) % gcdSECURE_CACHE_SLOTS) + 1;
            slot  = &Cache->cache[index];

            /* Test if this slot falls within the range to flush. */
            ptr = (gctUINT8_PTR) slot->logical;
            if ((ptr >= low) && (ptr < high))
            {
                /* Remove entry from cache. */
                slot->logical = gcvNULL;
            }

            /* Next block. */
            low += gcdSECURE_CACHE_SLOTS;
        }
#endif
    }

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}
#endif

/*******************************************************************************
**
**  gckKERNEL_Recovery
**
**  Try to recover the GPU from a fatal error.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_Recovery(
    IN gckKERNEL Kernel
    )
{
    gceSTATUS status;
    gckEVENT eventObj;
    gckHARDWARE hardware;
#if gcdSECURE_USER
    gctUINT32 processID;
    gcskSECURE_CACHE_PTR cache;
#endif
    gctUINT32 mask = 0;
    gctUINT32 i = 0, count = 0;
#if gcdINTERRUPT_STATISTIC
    gctINT32 oldValue;
#endif

    gcmkHEADER_ARG("Kernel=0x%x", Kernel);

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Grab gckEVENT object. */
    eventObj = Kernel->eventObj;
    gcmkVERIFY_OBJECT(eventObj, gcvOBJ_EVENT);

    /* Grab gckHARDWARE object. */
    hardware = Kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

#if gcdSECURE_USER
    /* Flush the secure mapping cache. */
    gcmkONERROR(gckOS_GetProcessID(&processID));
    gcmkONERROR(gckKERNEL_GetProcessDBCache(Kernel, processID, &cache));
    gcmkONERROR(gckKERNEL_FlushTranslationCache(Kernel, cache, gcvNULL, 0));
#endif

    if (Kernel->stuckDump == gcvSTUCK_DUMP_NONE)
    {
        gcmkPRINT("[galcore]: GPU[%d] hang, automatic recovery.", Kernel->core);
    }
    else
    {
        gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, Kernel->device->stuckDumpMutex, gcvINFINITE));

        _DumpDriverConfigure(Kernel);
        _DumpState(Kernel);

        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->device->stuckDumpMutex));
    }

    if (Kernel->recovery == gcvFALSE)
    {
        gcmkPRINT("[galcore]: Stop driver to keep scene.");

        /* Stop monitor timer. */
        Kernel->monitorTimerStop = gcvTRUE;

        /* Success. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    /* Issuing a soft reset for the GPU. */
    gcmkONERROR(gckHARDWARE_Reset(hardware));

    mask = Kernel->restoreMask;

    for (i = 0; i < 32; i++)
    {
        if (mask & (1 << i))
        {
            count++;
        }
    }

    /* Handle all outstanding events now. */
    gcmkONERROR(gckOS_AtomSet(Kernel->os, eventObj->pending, mask));

#if gcdINTERRUPT_STATISTIC
    while (count--)
    {
        gcmkONERROR(gckOS_AtomDecrement(
            Kernel->os,
            eventObj->interruptCount,
            &oldValue
            ));
    }

    gckOS_AtomClearMask(Kernel->hardware->pendingEvent, mask);
#endif

    gcmkONERROR(gckEVENT_Notify(eventObj, 1));

    gcmkVERIFY_OK(gckOS_GetTime(&Kernel->resetTimeStamp));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_OpenUserData
**
**  Get access to the user data.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL NeedCopy
**          The flag indicating whether or not the data should be copied.
**
**      gctPOINTER StaticStorage
**          Pointer to the kernel storage where the data is to be copied if
**          NeedCopy is gcvTRUE.
**
**      gctPOINTER UserPointer
**          User pointer to the data.
**
**      gctSIZE_T Size
**          Size of the data.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Pointer to the kernel pointer that will be pointing to the data.
*/
gceSTATUS
gckKERNEL_OpenUserData(
    IN gckKERNEL Kernel,
    IN gctBOOL NeedCopy,
    IN gctPOINTER StaticStorage,
    IN gctPOINTER UserPointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG(
        "Kernel=0x%08X NeedCopy=%d StaticStorage=0x%08X "
        "UserPointer=0x%08X Size=%lu KernelPointer=0x%08X",
        Kernel, NeedCopy, StaticStorage, UserPointer, Size, KernelPointer
        );

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(!NeedCopy || (StaticStorage != gcvNULL));
    gcmkVERIFY_ARGUMENT(UserPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    if (NeedCopy)
    {
        /* Copy the user data to the static storage. */
        gcmkONERROR(gckOS_CopyFromUserData(
            Kernel->os, StaticStorage, UserPointer, Size
            ));

        /* Set the kernel pointer. */
        * KernelPointer = StaticStorage;
    }
    else
    {
        gctPOINTER pointer = gcvNULL;

        /* Map the user pointer. */
        gcmkONERROR(gckOS_MapUserPointer(
            Kernel->os, UserPointer, Size, &pointer
            ));

        /* Set the kernel pointer. */
        * KernelPointer = pointer;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_CloseUserData
**
**  Release resources associated with the user data connection opened by
**  gckKERNEL_OpenUserData.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctBOOL NeedCopy
**          The flag indicating whether or not the data should be copied.
**
**      gctBOOL FlushData
**          If gcvTRUE, the data is written back to the user.
**
**      gctPOINTER UserPointer
**          User pointer to the data.
**
**      gctSIZE_T Size
**          Size of the data.
**
**  OUTPUT:
**
**      gctPOINTER * KernelPointer
**          Kernel pointer to the data.
*/
gceSTATUS
gckKERNEL_CloseUserData(
    IN gckKERNEL Kernel,
    IN gctBOOL NeedCopy,
    IN gctBOOL FlushData,
    IN gctPOINTER UserPointer,
    IN gctSIZE_T Size,
    OUT gctPOINTER * KernelPointer
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPOINTER pointer;

    gcmkHEADER_ARG(
        "Kernel=0x%08X NeedCopy=%d FlushData=%d "
        "UserPointer=0x%08X Size=%lu KernelPointer=0x%08X",
        Kernel, NeedCopy, FlushData, UserPointer, Size, KernelPointer
        );

    /* Validate the arguemnts. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(UserPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(KernelPointer != gcvNULL);
    gcmkVERIFY_ARGUMENT(Size > 0);

    /* Get a shortcut to the kernel pointer. */
    pointer = * KernelPointer;

    if (pointer != gcvNULL)
    {
        if (NeedCopy)
        {
            if (FlushData)
            {
                gcmkONERROR(gckOS_CopyToUserData(
                    Kernel->os, * KernelPointer, UserPointer, Size
                    ));
            }
        }
        else
        {
            /* Unmap record from kernel memory. */
            gcmkONERROR(gckOS_UnmapUserPointer(
                Kernel->os,
                UserPointer,
                Size,
                * KernelPointer
                ));
        }

        /* Reset the kernel pointer. */
        * KernelPointer = gcvNULL;
    }

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_AllocateVirtualCommandBuffer(
    IN gckKERNEL Kernel,
    IN gctBOOL InUserSpace,
    IN OUT gctSIZE_T * Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctPOINTER * Logical
    )
{
    gceSTATUS                       status;
    gckOS                           os = Kernel->os;
    gckVIRTUAL_COMMAND_BUFFER_PTR   buffer;

    gcmkHEADER_ARG("Os=0x%X InUserSpace=%d *Bytes=%lu",
        os, InUserSpace, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes != gcvNULL);
    gcmkVERIFY_ARGUMENT(*Bytes > 0);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    gcmkONERROR(
        gckOS_Allocate(
            os,
            sizeof(gckVIRTUAL_COMMAND_BUFFER),
            (gctPOINTER)&buffer
            ));

    gcmkONERROR(gckOS_ZeroMemory(buffer, sizeof(gckVIRTUAL_COMMAND_BUFFER)));

    gcmkONERROR(
        gckKERNEL_AllocateVirtualMemory(
            Kernel,
            gcvFALSE,
            InUserSpace,
            Bytes,
            (gctPHYS_ADDR *)&buffer,
            Logical
            ));

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, Kernel->virtualBufferLock, gcvINFINITE));

    if (Kernel->virtualBufferHead == gcvNULL)
    {
        Kernel->virtualBufferHead =
        Kernel->virtualBufferTail = buffer;
    }
    else
    {
        buffer->prev = Kernel->virtualBufferTail;
        Kernel->virtualBufferTail->next = buffer;
        Kernel->virtualBufferTail = buffer;
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, Kernel->virtualBufferLock));

    *Physical = buffer;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(gckOS_Free(os, buffer));

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_DestroyVirtualCommandBuffer(
    IN gckKERNEL Kernel,
    IN gctSIZE_T Bytes,
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical
    )
{
    gckOS                           os;
    gckKERNEL                       kernel;
    gckVIRTUAL_COMMAND_BUFFER_PTR   buffer = (gckVIRTUAL_COMMAND_BUFFER_PTR)Physical;

    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(buffer != gcvNULL);

    kernel = buffer->virtualBuffer.kernel;
    os = kernel->os;

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, kernel->virtualBufferLock, gcvINFINITE));

    if (buffer == kernel->virtualBufferHead)
    {
        if ((kernel->virtualBufferHead = buffer->next) == gcvNULL)
        {
            kernel->virtualBufferTail = gcvNULL;
        }
    }
    else
    {
        buffer->prev->next = buffer->next;

        if (buffer == kernel->virtualBufferTail)
        {
            kernel->virtualBufferTail = buffer->prev;
        }
        else
        {
            buffer->next->prev = buffer->prev;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, kernel->virtualBufferLock));

    gcmkVERIFY_OK(
        gckKERNEL_FreeVirtualMemory(
        Physical,
        Logical,
        gcvFALSE
        ));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_AllocateVirtualMemory(
    IN gckKERNEL Kernel,
    IN gctBOOL NonPaged,
    IN gctBOOL InUserSpace,
    IN OUT gctSIZE_T * Bytes,
    OUT gctPHYS_ADDR * Physical,
    OUT gctPOINTER * Logical
    )
{
    gckOS os                             = Kernel->os;
    gceSTATUS status;
    gctPOINTER logical                   = gcvNULL;
    gctSIZE_T pageCount;
    gctSIZE_T bytes                      = *Bytes;
    gckVIRTUAL_BUFFER_PTR buffer         = gcvNULL;
    gckMMU mmu                           = gcvNULL;
    gctUINT32 allocFlag                  = 0;

    gcmkHEADER_ARG("Os=0x%X InUserSpace=%d *Bytes=%lu",
        os, InUserSpace, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes != gcvNULL);
    gcmkVERIFY_ARGUMENT(*Bytes > 0);
    gcmkVERIFY_ARGUMENT(Physical != gcvNULL);
    gcmkVERIFY_ARGUMENT(Logical != gcvNULL);

    if (*Physical == gcvNULL)
    {
        gcmkONERROR(gckOS_Allocate(os,
            sizeof(gckVIRTUAL_BUFFER),
            (gctPOINTER)&buffer));

        gcmkONERROR(gckOS_ZeroMemory(buffer, sizeof(gckVIRTUAL_BUFFER)));
    }
    else
    {
        buffer = *Physical;
    }

    buffer->bytes = bytes;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
    allocFlag = gcvALLOC_FLAG_CACHEABLE;
#endif

    if (NonPaged)
    {
        gcmkONERROR(gckOS_AllocateNonPagedMemory(
            os,
            InUserSpace,
            allocFlag | gcvALLOC_FLAG_CONTIGUOUS,
            &bytes,
            &buffer->physical,
            &logical
            ));
    }
    else
    {
        gcmkONERROR(gckOS_AllocatePagedMemoryEx(
            os,
            allocFlag | gcvALLOC_FLAG_NON_CONTIGUOUS,
            bytes,
            gcvNULL,
            &buffer->physical
            ));
    }

    if (NonPaged)
    {
        gctSIZE_T pageSize;
        gcmkONERROR(gckOS_GetPageSize(os, &pageSize));

        pageCount = (bytes + pageSize - 1) / pageSize;

        if (InUserSpace)
        {
            *Logical =
                buffer->userLogical = logical;
        }
        else
        {
            *Logical =
                buffer->kernelLogical = logical;
        }
    }
    else
    {
        if (InUserSpace)
        {
            gcmkONERROR(gckOS_CreateUserVirtualMapping(os,
                buffer->physical,
                bytes,
                &logical,
                &pageCount));

            *Logical =
                buffer->userLogical = logical;
        }
        else
        {
            gcmkONERROR(gckOS_CreateKernelVirtualMapping(os,
                buffer->physical,
                bytes,
                &logical,
                &pageCount));

            *Logical =
                buffer->kernelLogical = logical;
        }

    }

    buffer->pageCount = pageCount;
    buffer->kernel = Kernel;

    gcmkONERROR(gckOS_GetProcessID(&buffer->pid));

#if gcdPROCESS_ADDRESS_SPACE
    gcmkONERROR(gckKERNEL_GetProcessMMU(Kernel, &mmu));
    buffer->mmu = mmu;
#else
    mmu = Kernel->mmu;
#endif

    gcmkONERROR(gckMMU_AllocatePages(mmu,
        pageCount,
        &buffer->pageTable,
        &buffer->gpuAddress));

#if gcdENABLE_TRUST_APPLICATION
    if (Kernel->hardware->options.secureMode == gcvSECURE_IN_TA)
    {
        gcmkONERROR(gckKERNEL_MapInTrustApplicaiton(
            Kernel,
            logical,
            buffer->physical,
            buffer->gpuAddress,
            pageCount
            ));
    }
    else
#endif
    {
        gcmkONERROR(gckOS_MapPagesEx(os,
            Kernel->core,
            buffer->physical,
            pageCount,
            buffer->gpuAddress,
            buffer->pageTable,
            gcvFALSE,
            gcvSURF_TYPE_UNKNOWN
            ));
    }

    gcmkONERROR(gckMMU_Flush(mmu, gcvSURF_INDEX));

    if (*Physical == gcvNULL)
        *Physical = buffer;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
        "gpuAddress = %x pageCount = %d kernelLogical = %x userLogical=%x",
        buffer->gpuAddress, buffer->pageCount,
        buffer->kernelLogical, buffer->userLogical);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (buffer && buffer->gpuAddress)
    {
        gcmkVERIFY_OK(
            gckMMU_FreePages(mmu, gcvFALSE, buffer->gpuAddress, buffer->pageTable, buffer->pageCount));
    }

    if (NonPaged && buffer->physical)
    {
        gcmkVERIFY_OK(gckOS_FreeNonPagedMemory(
            os,
            bytes,
            buffer->physical,
            logical
            ));
    }
    else
    {
        if (buffer && buffer->userLogical)
        {
            gcmkVERIFY_OK(
                gckOS_DestroyUserVirtualMapping(os,
                buffer->physical,
                bytes,
                (NonPaged ? 0 : buffer->userLogical)));
        }

        if (buffer && buffer->kernelLogical)
        {
            gcmkVERIFY_OK(
                gckOS_DestroyKernelVirtualMapping(os,
                buffer->physical,
                bytes,
                (NonPaged ? 0 : buffer->kernelLogical)));
        }

        if (buffer && buffer->physical)
        {
            gcmkVERIFY_OK(gckOS_FreePagedMemory(os, buffer->physical, bytes));
        }
    }

    if (*Physical == gcvNULL)
        gcmkVERIFY_OK(gckOS_Free(os, buffer));

    /* Return the status. */
    gcmkFOOTER();
    return status;

}

gceSTATUS
gckKERNEL_FreeVirtualMemory(
    IN gctPHYS_ADDR Physical,
    IN gctPOINTER Logical,
    IN gctBOOL NonPaged
    )
{
    gckOS os;
    gckKERNEL kernel;
    gckMMU mmu;
    gckVIRTUAL_BUFFER_PTR buffer = (gckVIRTUAL_BUFFER_PTR)Physical;

    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(buffer != gcvNULL);

    kernel = buffer->kernel;
    os = kernel->os;

#if gcdPROCESS_ADDRESS_SPACE
    gcmkONERROR(gckKERNEL_GetProcessMMU(Kernel, &mmu));
#else
    mmu = kernel->mmu;
#endif

    if (!buffer->userLogical && !NonPaged)
    {
        gcmkVERIFY_OK(gckOS_DestroyKernelVirtualMapping(os,
            buffer->physical,
            buffer->bytes,
            Logical));
    }

    gcmkVERIFY_OK(
        gckMMU_FreePages(mmu, gcvFALSE, buffer->gpuAddress, buffer->pageTable, buffer->pageCount));

    gcmkVERIFY_OK(gckOS_UnmapPages(os, buffer->pageCount, buffer->gpuAddress));

    if (NonPaged)
    {
        gcmkVERIFY_OK(gckOS_FreeNonPagedMemory(
            os,
            buffer->bytes,
            buffer->physical,
            Logical
            ));
    }
    else
    {
        gcmkVERIFY_OK(gckOS_FreePagedMemory(os, buffer->physical, buffer->bytes));
    }

    gcmkVERIFY_OK(gckOS_Free(os, buffer));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_GetGPUAddress(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    IN gctBOOL InUserSpace,
    IN gctPHYS_ADDR Physical,
    OUT gctUINT32 * Address
    )
{
    gckVIRTUAL_BUFFER_PTR buffer = Physical;
    gctPOINTER start;

    gcmkHEADER_ARG("Logical = %x InUserSpace=%d.", Logical, InUserSpace);

    if (InUserSpace)
    {
        start = buffer->userLogical;
    }
    else
    {
        start = buffer->kernelLogical;
    }

    gcmkASSERT(Logical >= start
           && (Logical < (gctPOINTER)((gctUINT8_PTR)start + buffer->bytes)));

    * Address = buffer->gpuAddress + (gctUINT32)((gctUINT8_PTR)Logical - (gctUINT8_PTR)start);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_QueryGPUAddress(
    IN gckKERNEL Kernel,
    IN gctUINT32 GpuAddress,
    OUT gckVIRTUAL_COMMAND_BUFFER_PTR * Buffer
    )
{
    gckVIRTUAL_COMMAND_BUFFER_PTR buffer;
    gctUINT32 start;
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, Kernel->virtualBufferLock, gcvINFINITE));

    /* Walk all command buffers. */
    for (buffer = Kernel->virtualBufferHead; buffer != gcvNULL; buffer = buffer->next)
    {
        start = (gctUINT32)buffer->virtualBuffer.gpuAddress;

        if (GpuAddress >= start && GpuAddress <= (start - 1 + buffer->virtualBuffer.pageCount * 4096))
        {
            /* Find a range matched. */
            *Buffer = buffer;
            status = gcvSTATUS_OK;
            break;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->virtualBufferLock));

    return status;
}

static void
gckQUEUE_Dequeue(
    IN gckQUEUE LinkQueue
    )
{
    gcmkASSERT(LinkQueue->count == LinkQueue->size);

    LinkQueue->count--;
    LinkQueue->front = (LinkQueue->front + 1) % gcdLINK_QUEUE_SIZE;
}

void
gckQUEUE_Enqueue(
    IN gckQUEUE LinkQueue,
    IN gcuQUEUEDATA *Data
    )
{
    gcuQUEUEDATA * datas = LinkQueue->datas;

    if (LinkQueue->count == LinkQueue->size)
    {
        gckQUEUE_Dequeue(LinkQueue);
    }

    gcmkASSERT(LinkQueue->count < LinkQueue->size);

    LinkQueue->count++;

    datas[LinkQueue->rear] = *Data;

    LinkQueue->rear = (LinkQueue->rear + 1) % LinkQueue->size;
}

void
gckQUEUE_GetData(
    IN gckQUEUE LinkQueue,
    IN gctUINT32 Index,
    OUT gcuQUEUEDATA ** Data
    )
{
    gcuQUEUEDATA * datas = LinkQueue->datas;

    gcmkASSERT(Index >= 0 && Index < LinkQueue->size);

    *Data = &datas[(Index + LinkQueue->front) % LinkQueue->size];
}

gceSTATUS
gckQUEUE_Allocate(
    IN gckOS Os,
    IN gckQUEUE Queue,
    IN gctUINT32 Size
    )
{
    gceSTATUS status;

    gcmkONERROR(gckOS_Allocate(
        Os,
        gcmSIZEOF(struct _gckLINKDATA) * Size,
        (gctPOINTER *)&Queue->datas
        ));

    Queue->size = Size;

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckQUEUE_Free(
    IN gckOS Os,
    IN gckQUEUE Queue
    )
{
    if (Queue->datas)
    {
        gcmkVERIFY_OK(gckOS_Free(Os, (gctPOINTER)Queue->datas));
    }

    return gcvSTATUS_OK;
}

/******************************************************************************\
*************************** Pointer - ID translation ***************************
\******************************************************************************/
#define gcdID_TABLE_LENGTH 1024
typedef struct _gcsINTEGERDB * gckINTEGERDB;
typedef struct _gcsINTEGERDB
{
    gckOS                       os;
    gctPOINTER*                 table;
    gctPOINTER                  mutex;
    gctUINT32                   tableLen;
    gctUINT32                   currentID;
    gctUINT32                   unused;
}
gcsINTEGERDB;

gceSTATUS
gckKERNEL_CreateIntegerDatabase(
    IN gckKERNEL Kernel,
    OUT gctPOINTER * Database
    )
{
    gceSTATUS status;
    gckINTEGERDB database = gcvNULL;

    gcmkHEADER_ARG("Kernel=0x%08X Datbase=0x%08X", Kernel, Database);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Database != gcvNULL);

    /* Allocate a database. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os, gcmSIZEOF(gcsINTEGERDB), (gctPOINTER *)&database));

    gcmkONERROR(gckOS_ZeroMemory(database, gcmSIZEOF(gcsINTEGERDB)));

    /* Allocate a pointer table. */
    gcmkONERROR(gckOS_Allocate(
        Kernel->os, gcmSIZEOF(gctPOINTER) * gcdID_TABLE_LENGTH, (gctPOINTER *)&database->table));

    gcmkONERROR(gckOS_ZeroMemory(database->table, gcmSIZEOF(gctPOINTER) * gcdID_TABLE_LENGTH));

    /* Allocate a database mutex. */
    gcmkONERROR(gckOS_CreateMutex(Kernel->os, &database->mutex));

    /* Initialize. */
    database->currentID = 0;
    database->unused = gcdID_TABLE_LENGTH;
    database->os = Kernel->os;
    database->tableLen = gcdID_TABLE_LENGTH;

    *Database = database;

    gcmkFOOTER_ARG("*Database=0x%08X", *Database);
    return gcvSTATUS_OK;

OnError:
    /* Rollback. */
    if (database)
    {
        if (database->table)
        {
            gcmkOS_SAFE_FREE(Kernel->os, database->table);
        }

        gcmkOS_SAFE_FREE(Kernel->os, database);
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_DestroyIntegerDatabase(
    IN gckKERNEL Kernel,
    IN gctPOINTER Database
    )
{
    gckINTEGERDB database = Database;

    gcmkHEADER_ARG("Kernel=0x%08X Datbase=0x%08X", Kernel, Database);

    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Database != gcvNULL);

    /* Destroy pointer table. */
    gcmkOS_SAFE_FREE(Kernel->os, database->table);

    /* Destroy database mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, database->mutex));

    /* Destroy database. */
    gcmkOS_SAFE_FREE(Kernel->os, database);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckKERNEL_AllocateIntegerId(
    IN gctPOINTER Database,
    IN gctPOINTER Pointer,
    OUT gctUINT32 * Id
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gctUINT32 i, unused, currentID, tableLen;
    gctPOINTER * table;
    gckOS os = database->os;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Database=0x%08X Pointer=0x%08X", Database, Pointer);

    gcmkVERIFY_ARGUMENT(Id != gcvNULL);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (database->unused < 1)
    {
        /* Extend table. */
        gcmkONERROR(
            gckOS_Allocate(os,
                           gcmSIZEOF(gctPOINTER) * (database->tableLen + gcdID_TABLE_LENGTH),
                           (gctPOINTER *)&table));

        gcmkONERROR(gckOS_ZeroMemory(table + database->tableLen,
                                     gcmSIZEOF(gctPOINTER) * gcdID_TABLE_LENGTH));

        /* Copy data from old table. */
        gckOS_MemCopy(table,
                      database->table,
                      database->tableLen * gcmSIZEOF(gctPOINTER));

        gcmkOS_SAFE_FREE(os, database->table);

        /* Update databse with new allocated table. */
        database->table = table;
        database->currentID = database->tableLen;
        database->tableLen += gcdID_TABLE_LENGTH;
        database->unused += gcdID_TABLE_LENGTH;
    }

    table = database->table;
    currentID = database->currentID;
    tableLen = database->tableLen;
    unused = database->unused;

    /* Connect id with pointer. */
    table[currentID] = Pointer;

    *Id = currentID + 1;

    /* Update the currentID. */
    if (--unused > 0)
    {
        for (i = 0; i < tableLen; i++)
        {
            if (++currentID >= tableLen)
            {
                /* Wrap to the begin. */
                currentID = 0;
            }

            if (table[currentID] == gcvNULL)
            {
                break;
            }
        }
    }

    database->table = table;
    database->currentID = currentID;
    database->tableLen = tableLen;
    database->unused = unused;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    acquired = gcvFALSE;

    gcmkFOOTER_ARG("*Id=%d", *Id);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_FreeIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gckOS os = database->os;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Database=0x%08X Id=%d", Database, Id);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (!(Id > 0 && Id <= database->tableLen))
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    Id -= 1;

    database->table[Id] = gcvNULL;

    if (database->unused++ == 0)
    {
        database->currentID = Id;
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckKERNEL_QueryIntegerId(
    IN gctPOINTER Database,
    IN gctUINT32 Id,
    OUT gctPOINTER * Pointer
    )
{
    gceSTATUS status;
    gckINTEGERDB database = Database;
    gctPOINTER pointer;
    gckOS os = database->os;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Database=0x%08X Id=%d", Database, Id);
    gcmkVERIFY_ARGUMENT(Pointer != gcvNULL);

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, database->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (!(Id > 0 && Id <= database->tableLen))
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    Id -= 1;

    pointer = database->table[Id];

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    acquired = gcvFALSE;

    if (pointer)
    {
        *Pointer = pointer;
    }
    else
    {
        gcmkONERROR(gcvSTATUS_NOT_FOUND);
    }

    gcmkFOOTER_ARG("*Pointer=0x%08X", *Pointer);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, database->mutex));
    }

    gcmkFOOTER();
    return status;
}


gctUINT32
gckKERNEL_AllocateNameFromPointer(
    IN gckKERNEL Kernel,
    IN gctPOINTER Pointer
    )
{
    gceSTATUS status;
    gctUINT32 name;
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=0x%X Pointer=0x%X", Kernel, Pointer);

    gcmkONERROR(
        gckKERNEL_AllocateIntegerId(database, Pointer, &name));

    gcmkFOOTER_ARG("name=%d", name);
    return name;

OnError:
    gcmkFOOTER();
    return 0;
}

gctPOINTER
gckKERNEL_QueryPointerFromName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    )
{
    gceSTATUS status;
    gctPOINTER pointer = gcvNULL;
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=0x%X Name=%d", Kernel, Name);

    /* Lookup in database to get pointer. */
    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Name, &pointer));

    gcmkFOOTER_ARG("pointer=0x%X", pointer);
    return pointer;

OnError:
    gcmkFOOTER();
    return gcvNULL;
}

gceSTATUS
gckKERNEL_DeleteName(
    IN gckKERNEL Kernel,
    IN gctUINT32 Name
    )
{
    gctPOINTER database = Kernel->db->pointerDatabase;

    gcmkHEADER_ARG("Kernel=0x%X Name=0x%X", Kernel, Name);

    /* Free name if exists. */
    gcmkVERIFY_OK(gckKERNEL_FreeIntegerId(database, Name));

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
***** Shared Buffer ************************************************************
*******************************************************************************/

/*******************************************************************************
**
**  gckKERNEL_CreateShBuffer
**
**  Create shared buffer.
**  The shared buffer can be used across processes. Other process needs call
**  gckKERNEL_MapShBuffer before use it.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctUINT32 Size
**          Specify the shared buffer size.
**
**  OUTPUT:
**
**      gctSHBUF * ShBuf
**          Pointer to hold return shared buffer handle.
*/
gceSTATUS
gckKERNEL_CreateShBuffer(
    IN gckKERNEL Kernel,
    IN gctUINT32 Size,
    OUT gctSHBUF * ShBuf
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf = gcvNULL;

    gcmkHEADER_ARG("Kernel=0x%X, Size=%u", Kernel, Size);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Size == 0)
    {
        /* Invalid size. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }
    else if (Size > 1024)
    {
        /* Limite shared buffer size. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    /* Create a shared buffer structure. */
    gcmkONERROR(
        gckOS_Allocate(Kernel->os,
                       sizeof (gcsSHBUF),
                       (gctPOINTER *)&shBuf));

    /* Initialize shared buffer. */
    shBuf->id        = 0;
    shBuf->reference = gcvNULL;
    shBuf->size      = Size;
    shBuf->data      = gcvNULL;

    /* Allocate integer id for this shared buffer. */
    gcmkONERROR(
        gckKERNEL_AllocateIntegerId(Kernel->db->pointerDatabase,
                                    shBuf,
                                    &shBuf->id));

    /* Allocate atom. */
    gcmkONERROR(gckOS_AtomConstruct(Kernel->os, &shBuf->reference));

    /* Set default reference count to 1. */
    gcmkVERIFY_OK(gckOS_AtomSet(Kernel->os, shBuf->reference, 1));

    /* Return integer id. */
    *ShBuf = (gctSHBUF)(gctUINTPTR_T)shBuf->id;

    gcmkFOOTER_ARG("*ShBuf=%u", shBuf->id);
    return gcvSTATUS_OK;

OnError:
    /* Error roll back. */
    if (shBuf != gcvNULL)
    {
        if (shBuf->id != 0)
        {
            gcmkVERIFY_OK(
                gckKERNEL_FreeIntegerId(Kernel->db->pointerDatabase,
                                        shBuf->id));
        }

        gcmkOS_SAFE_FREE(Kernel->os, shBuf);
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_DestroyShBuffer
**
**  Destroy shared buffer.
**  This will decrease reference of specified shared buffer and do actual
**  destroy when no reference on it.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be destroyed.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_DestroyShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf;
    gctINT32 oldValue = 0;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Kernel=0x%X ShBuf=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Acquire mutex. */
    gcmkONERROR(
        gckOS_AcquireMutex(Kernel->os,
                           Kernel->db->pointerDatabaseMutex,
                           gcvINFINITE));
    acquired = gcvTRUE;

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    /* Decrease the reference count. */
    gckOS_AtomDecrement(Kernel->os, shBuf->reference, &oldValue);

    if (oldValue == 1)
    {
        /* Free integer id. */
        gcmkVERIFY_OK(
            gckKERNEL_FreeIntegerId(Kernel->db->pointerDatabase,
                                    shBuf->id));

        /* Free atom. */
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, shBuf->reference));

        if (shBuf->data)
        {
            gcmkOS_SAFE_FREE(Kernel->os, shBuf->data);
            shBuf->data = gcvNULL;
        }

        /* Free the shared buffer. */
        gcmkOS_SAFE_FREE(Kernel->os, shBuf);
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(
        gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(
            gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_MapShBuffer
**
**  Map shared buffer into this process so that it can be used in this process.
**  This will increase reference count on the specified shared buffer.
**  Call gckKERNEL_DestroyShBuffer to dereference.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be mapped.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_MapShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf;
    gctINT32 oldValue = 0;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Kernel=0x%X ShBuf=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Acquire mutex. */
    gcmkONERROR(
        gckOS_AcquireMutex(Kernel->os,
                           Kernel->db->pointerDatabaseMutex,
                           gcvINFINITE));
    acquired = gcvTRUE;

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    /* Increase the reference count. */
    gckOS_AtomIncrement(Kernel->os, shBuf->reference, &oldValue);

    /* Release the mutex. */
    gcmkVERIFY_OK(
        gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(
            gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_WriteShBuffer
**
**  Write user data into shared buffer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be written to.
**
**      gctPOINTER UserData
**          User mode pointer to hold the source data.
**
**      gctUINT32 ByteCount
**          Specify number of bytes to write. If this is larger than
**          shared buffer size, gcvSTATUS_INVALID_ARGUMENT is returned.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckKERNEL_WriteShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf = gcvNULL;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Kernel=0x%X ShBuf=%u UserData=0x%X ByteCount=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf, UserData, ByteCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Acquire mutex. */
    gcmkONERROR(
        gckOS_AcquireMutex(Kernel->os,
                           Kernel->db->pointerDatabaseMutex,
                           gcvINFINITE));
    acquired = gcvTRUE;

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    if ((ByteCount > shBuf->size) ||
        (ByteCount == 0) ||
        (UserData  == gcvNULL))
    {
        /* Exceeds buffer max size or invalid. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (shBuf->data == gcvNULL)
    {
        /* Allocate buffer data when first time write. */
        gcmkONERROR(gckOS_Allocate(Kernel->os, ByteCount, &shBuf->data));
    }

    /* Copy data from user. */
    gcmkONERROR(
        gckOS_CopyFromUserData(Kernel->os,
                               shBuf->data,
                               UserData,
                               ByteCount));

    /* Release the mutex. */
    gcmkVERIFY_OK(
        gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (shBuf && shBuf->data)
    {
        gcmkOS_SAFE_FREE(Kernel->os, shBuf->data);
        shBuf->data = gcvNULL;
    }

    if (acquired)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(
            gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckKERNEL_ReadShBuffer
**
**  Read data from shared buffer and copy to user pointer.
**
**  INPUT:
**
**      gckKERNEL Kernel
**          Pointer to an gckKERNEL object.
**
**      gctSHBUF ShBuf
**          Specify the shared buffer to be read from.
**
**      gctPOINTER UserData
**          User mode pointer to save output data.
**
**      gctUINT32 ByteCount
**          Specify number of bytes to read.
**          If this is larger than shared buffer size, only avaiable bytes are
**          copied. If smaller, copy requested size.
**
**  OUTPUT:
**
**      gctUINT32 * BytesRead
**          Pointer to hold how many bytes actually read from shared buffer.
*/
gceSTATUS
gckKERNEL_ReadShBuffer(
    IN gckKERNEL Kernel,
    IN gctSHBUF ShBuf,
    IN gctPOINTER UserData,
    IN gctUINT32 ByteCount,
    OUT gctUINT32 * BytesRead
    )
{
    gceSTATUS status;
    gcsSHBUF_PTR shBuf;
    gctUINT32 bytes;
    gctBOOL acquired = gcvFALSE;

    gcmkHEADER_ARG("Kernel=0x%X ShBuf=%u UserData=0x%X ByteCount=%u",
                   Kernel, (gctUINT32)(gctUINTPTR_T) ShBuf, UserData, ByteCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(ShBuf != gcvNULL);

    /* Acquire mutex. */
    gcmkONERROR(
        gckOS_AcquireMutex(Kernel->os,
                           Kernel->db->pointerDatabaseMutex,
                           gcvINFINITE));
    acquired = gcvTRUE;

    /* Find shared buffer structure. */
    gcmkONERROR(
        gckKERNEL_QueryIntegerId(Kernel->db->pointerDatabase,
                                 (gctUINT32)(gctUINTPTR_T)ShBuf,
                                 (gctPOINTER)&shBuf));

    gcmkASSERT(shBuf->id == (gctUINT32)(gctUINTPTR_T)ShBuf);

    if (shBuf->data == gcvNULL)
    {
        *BytesRead = 0;

        /* No data in shared buffer, skip copy. */
        status = gcvSTATUS_SKIP;
        goto OnError;
    }
    else if (ByteCount == 0)
    {
        /* Invalid size to read. */
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    /* Determine bytes to copy. */
    bytes = (ByteCount < shBuf->size) ? ByteCount : shBuf->size;

    /* Copy data to user. */
    gcmkONERROR(
        gckOS_CopyToUserData(Kernel->os,
                             shBuf->data,
                             UserData,
                             bytes));

    /* Return copied size. */
    *BytesRead = bytes;

    /* Release the mutex. */
    gcmkVERIFY_OK(
        gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    acquired = gcvFALSE;

    gcmkFOOTER_ARG("*BytesRead=%u", bytes);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
    {
        /* Release the mutex. */
        gcmkVERIFY_OK(
            gckOS_ReleaseMutex(Kernel->os, Kernel->db->pointerDatabaseMutex));
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************\
*************************** List Helper *****************************************
\*******************************************************************************/

static void
_ListAdd(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Prev,
    gcsLISTHEAD_PTR Next
    )
{
    Next->prev = New;
    New->next  = Next;
    New->prev  = Prev;
    Prev->next = New;
}

void
_ListDel(
    gcsLISTHEAD_PTR Prev,
    gcsLISTHEAD_PTR Next
    )
{
    Next->prev = Prev;
    Prev->next = Next;
}

void
gcsLIST_Init(
    gcsLISTHEAD_PTR Node
    )
{
    Node->prev = Node;
    Node->next = Node;
}

void
gcsLIST_Add(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    )
{
    _ListAdd(New, Head, Head->next);
}

void
gcsLIST_AddTail(
    gcsLISTHEAD_PTR New,
    gcsLISTHEAD_PTR Head
    )
{
    _ListAdd(New, Head->prev, Head);
}

void
gcsLIST_Del(
    gcsLISTHEAD_PTR Node
    )
{
    _ListDel(Node->prev, Node->next);
}

gctBOOL
gcsLIST_Empty(
    gcsLISTHEAD_PTR Head
    )
{
    return Head->next == Head;
}

/*******************************************************************************\
********************************* Fence *****************************************
\*******************************************************************************/

gceSTATUS
gckFENCE_Create(
    IN gckOS Os,
    IN gckKERNEL Kernel,
    OUT gckFENCE * Fence
    )
{
    gceSTATUS status;
    gckFENCE fence = gcvNULL;
    gctSIZE_T pageSize = 4096;

    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcsFENCE), (gctPOINTER *)&fence));
    gcmkONERROR(gckOS_ZeroMemory(fence, gcmSIZEOF(gcsFENCE)));
    gcmkONERROR(gckOS_CreateMutex(Os, (gctPOINTER *)&fence->mutex));

    fence->kernel = Kernel;

#if USE_KERNEL_VIRTUAL_BUFFERS
    if (Kernel->virtualCommandBuffer)
    {
        gcmkONERROR(gckKERNEL_AllocateVirtualMemory(
            Kernel,
            gcvFALSE,
            gcvFALSE,
            &pageSize,
            &fence->physical,
            &fence->logical
            ));

        gcmkONERROR(gckKERNEL_GetGPUAddress(
            Kernel,
            fence->logical,
            gcvFALSE,
            fence->physical,
            &fence->address
            ));
    }
    else
#endif
    {
        gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

        gcmkONERROR(gckOS_AllocateNonPagedMemory(
            Os,
            gcvFALSE,
            allocFlag,
            &pageSize,
            &fence->physical,
            &fence->logical
            ));

        gcmkONERROR(gckHARDWARE_ConvertLogical(
            Kernel->hardware,
            fence->logical,
            gcvFALSE,
            &fence->address
            ));

        gcmkONERROR(gckMMU_FillFlatMapping(
            Kernel->mmu, fence->address, pageSize
            ));
    }

    gcsLIST_Init(&fence->waitingList);

    *Fence = fence;

    return gcvSTATUS_OK;
OnError:
    if (fence)
    {
        gckFENCE_Destory(Os, fence);
    }

    return status;
}

gceSTATUS
gckFENCE_Destory(
    IN gckOS Os,
    OUT gckFENCE Fence
    )
{
    if (Fence->mutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Fence->mutex));
    }

    if (Fence->logical)
    {
#if USE_KERNEL_VIRTUAL_BUFFERS
        if (Fence->kernel->virtualCommandBuffer)
        {
            gcmkVERIFY_OK(gckKERNEL_FreeVirtualMemory(
                Fence->physical,
                Fence->logical,
                gcvFALSE
                ));
        }
        else
#endif
        {
            gcmkVERIFY_OK(gckOS_FreeNonPagedMemory(
                Os,
                4096,
                Fence->physical,
                Fence->logical
                ));
        }
    }

    gcmkOS_SAFE_FREE(Os, Fence);

    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckFENCE_Signal
**
**  Signal all completed nodes.
**
**
*/
gceSTATUS
gckFENCE_Signal(
    IN gckOS Os,
    IN gckFENCE Fence
    )
{
    gcsLISTHEAD_PTR list = &Fence->waitingList;
    gcsLISTHEAD_PTR nodeHead, nodeTemp;
    gckFENCE_SYNC sync;
    gckOS os = Os;
    gctUINT64 stamp = *(gctUINT64 *)Fence->logical;

    gcmkVERIFY_OK(gckOS_AcquireMutex(os, Fence->mutex, gcvINFINITE));

    gcmkLIST_FOR_EACH_SAFE(nodeHead, nodeTemp, list)
    {
        sync = gcmCONTAINEROF(nodeHead, _gcsFENCE_SYNC, head);

        /* Signal all nodes which are complete. */
        if (sync->commitStamp <= stamp && sync->inList)
        {
            /* Signal. */
            gckOS_Signal(os, sync->signal, gcvTRUE);

            /* Remove from wait list. */
            gcsLIST_Del(nodeHead);

            /* Mark node not in waiting list. */
            sync->inList = gcvFALSE;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, Fence->mutex));

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Construct(
    IN gckOS Os,
    OUT gckDEVICE * Device
    )
{
    gceSTATUS status;
    gckDEVICE device;
    gctUINT i;

    gcmkHEADER();

    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcsDEVICE), (gctPOINTER *)&device));

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        device->coreInfoArray[i].type = gcvHARDWARE_INVALID;
    }
    device->defaultHwType = gcvHARDWARE_INVALID;

    gckOS_ZeroMemory(device, gcmSIZEOF(gcsDEVICE));

    gcmkONERROR(gckOS_CreateMutex(Os, &device->stuckDumpMutex));
    gcmkONERROR(gckOS_CreateMutex(Os, &device->commitMutex));

    device->os = Os;

    *Device = device;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:

    if (device != gcvNULL)
    {
        gckDEVICE_Destroy(Os, device);
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckDEVICE_AddCore(
    IN gckDEVICE Device,
    IN gceCORE Core,
    IN gctUINT ChipID,
    IN gctPOINTER Context,
    IN gckKERNEL * Kernel
    )
{
    gceSTATUS status;
    gcsCORE_INFO * info = Device->coreInfoArray;
    gceHARDWARE_TYPE type = (gceHARDWARE_TYPE)((gctUINT)gcvHARDWARE_INVALID);
    gctUINT32 index = Device->coreNum;
    gctUINT32 i;
    gcsCORE_LIST *coreList;
    gceHARDWARE_TYPE kernelType;
    gceHARDWARE_TYPE defaultHwType;
    gckKERNEL kernel;

    gcmkASSERT(Device->coreNum < gcvCORE_COUNT);

    if (Core >= gcvCORE_MAJOR && Core <= gcvCORE_3D_MAX)
    {
        /* Chip ID is only used for 3D cores. */
        if (ChipID == gcvCHIP_ID_DEFAULT)
        {
            /* Apply default chipID if it is not set. */
            ChipID = Core;
        }
    }

    /* Construct gckKERNEL for this core. */
    gcmkONERROR(gckKERNEL_Construct(
        Device->os, Core, ChipID, Context, Device, Device->database, Kernel));

    kernel = *Kernel;

    if (Device->database == gcvNULL)
    {
        Device->database = kernel->db;
    }

    kernelType =  _GetHardwareType(kernel);

    if (kernelType >= gcvHARDWARE_NUM_TYPES)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    info[index].type = kernelType;
    info[index].core = Core;
    info[index].kernel = kernel;
    info[index].chipID = ChipID;

    if (index == 0)
    {
        /* First core, map all type/core to it. */
        for (; type != gcvHARDWARE_NUM_TYPES; type = (gceHARDWARE_TYPE)((gctUINT)type + 1))
        {
            Device->map[type].num = 0;

            for (i = 0 ; i < 4; i++)
            {
                Device->map[type].kernels[i] = kernel;
            }
        }
    }

    /* Get core list of this type. */
    coreList = &Device->map[kernelType];

    /* Setup gceHARDWARE_TYPE to gceCORE mapping. */
    coreList->kernels[coreList->num++] = kernel;

    defaultHwType = kernelType;
    if (kernelType == gcvHARDWARE_3D2D)
    {
        coreList = &Device->map[gcvHARDWARE_3D];
        coreList->kernels[coreList->num++] = kernel;
        defaultHwType = gcvHARDWARE_3D;
    }

    /* Advance total core number. */
    Device->coreNum++;

    /* Default HW type was chosen: 3D > 2D > VG */
    if (Device->defaultHwType == gcvHARDWARE_INVALID)
    {
        Device->defaultHwType = defaultHwType;
    }
    else if (Device->defaultHwType > defaultHwType)
    {
        Device->defaultHwType = defaultHwType;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckDEVICE_ChipInfo(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
    gctUINT i;
    gcsCORE_INFO * info = Device->coreInfoArray;

    for (i = 0; i < Device->coreNum; i++)
    {
        Interface->u.ChipInfo.types[i] = info[i].type;
        Interface->u.ChipInfo.ids[i] = info[i].chipID;
    }

    Interface->u.ChipInfo.count = Device->coreNum;

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Version(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
    Interface->u.Version.major = gcvVERSION_MAJOR;
    Interface->u.Version.minor = gcvVERSION_MINOR;
    Interface->u.Version.patch = gcvVERSION_PATCH;
    Interface->u.Version.build = gcvVERSION_BUILD;
#if gcmIS_DEBUG(gcdDEBUG_TRACE)
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_KERNEL,
                   "KERNEL version %d.%d.%d build %u",
                   gcvVERSION_MAJOR, gcvVERSION_MINOR,
                   gcvVERSION_PATCH, gcvVERSION_BUILD);
#endif

    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_Destroy(
    IN gckOS Os,
    IN gckDEVICE Device
    )
{
    gctINT i;
    gcsCORE_INFO * info = Device->coreInfoArray;

    for (i = Device->coreNum - 1; i >= 0 ; i--)
    {
        if (info[i].kernel != gcvNULL)
        {
            gckKERNEL_Destroy(info[i].kernel);
        }
    }

    if (Device->commitMutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Device->commitMutex));
    }
    if (Device->stuckDumpMutex)
    {
        gcmkVERIFY_OK(gckOS_DeleteMutex(Os, Device->stuckDumpMutex));
    }

    gcmkOS_SAFE_FREE(Os, Device);

    return gcvSTATUS_OK;
}

static gceSTATUS
gckDEVICE_SetTimeOut(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
#if gcdGPU_TIMEOUT
    gckKERNEL kernel;
    gctUINT i;
    gceHARDWARE_TYPE type = Interface->hardwareType;
    gcsCORE_LIST *coreList;

    coreList = &Device->map[type];

    for (i = 0; i < coreList->num; i++)
    {
        kernel = coreList->kernels[i];

        kernel->timeOut = Interface->u.SetTimeOut.timeOut;
    }
#endif

    return gcvSTATUS_OK;
}


gceSTATUS
gckDEVICE_Dispatch(
    IN gckDEVICE Device,
    IN gcsHAL_INTERFACE_PTR Interface
    )
{
    gceSTATUS status = gcvSTATUS_NOT_SUPPORTED;
    gckKERNEL kernel;
    gceHARDWARE_TYPE type = Interface->hardwareType;
    gctUINT32 coreIndex = Interface->coreIndex;

    switch (Interface->command)
    {
    case gcvHAL_CHIP_INFO:
        status = gckDEVICE_ChipInfo(Device, Interface);
        break;

    case gcvHAL_VERSION:
        status = gckDEVICE_Version(Device, Interface);
        break;

    case gcvHAL_SET_TIMEOUT:
        status = gckDEVICE_SetTimeOut(Device, Interface);
        break;

    default:
        status = gcvSTATUS_NOT_SUPPORTED;
        break;
    }

    if (gcmIS_SUCCESS(status))
    {
        /* Dispatch handled in this layer. */
        Interface->status = status;
    }
    else
    {
        /* Need go through gckKERNEL dispatch. */
        kernel = Device->map[type].kernels[coreIndex];


#if gcdENABLE_VG
        if (kernel->vg)
        {
            status = gckVGKERNEL_Dispatch(kernel, gcvTRUE, Interface);
        }
        else
#endif
        {
            status = gckKERNEL_Dispatch(kernel, Device, gcvTRUE, Interface);
        }

        /* Interface->status is handled in gckKERNEL_Dispatch(). */
    }

    return status;
}

gceSTATUS
gckDEVICE_GetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU *Mmu
    )
{
    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(Type < gcvHARDWARE_NUM_TYPES);

    *Mmu = Device->mmus[Type];

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckDEVICE_SetMMU(
    IN gckDEVICE Device,
    IN gceHARDWARE_TYPE Type,
    IN gckMMU Mmu
    )
{
    gcmkHEADER();
    gcmkVERIFY_ARGUMENT(Type < gcvHARDWARE_NUM_TYPES);

    Device->mmus[Type] = Mmu;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckDEVICE_QueryGPUAddress
**
**  Search GPUAddress in other core's address space, whose type is same as current
**  core. It is used to find correct command buffer which is shared by mulitple
**  core.
**
*/
gceSTATUS
gckDEVICE_QueryGPUAddress(
    IN gckDEVICE Device,
    IN gckKERNEL Kernel,
    IN gctUINT32 GPUAddress,
    OUT gckVIRTUAL_COMMAND_BUFFER_PTR * Buffer
    )
{
    gceSTATUS status = gcvSTATUS_NOT_FOUND;
    gctUINT i;
    gceHARDWARE_TYPE kernelType;

    kernelType = _GetHardwareType(Kernel);

    if (Device != gcvNULL)
    {
        for (i = 0; i < Device->coreNum; i++)
        {
            if (Device->coreInfoArray[i].type == kernelType)
            {
                /* Search other core's command buffer list whose type is same. */
                status = gckKERNEL_QueryGPUAddress(
                            Device->coreInfoArray[i].kernel, GPUAddress, Buffer);

                if (gcmIS_SUCCESS(status))
                {
                    break;
                }
            }
        }
    }
    else
    {
        status = gckKERNEL_QueryGPUAddress(Kernel, GPUAddress, Buffer);
    }

    return status;
}

#if gcdENABLE_TRUST_APPLICATION
gceSTATUS
gckKERNEL_MapInTrustApplicaiton(
    IN gckKERNEL Kernel,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical,
    IN gctUINT32 GPUAddress,
    IN gctSIZE_T PageCount
    )
{
    gceSTATUS status;
    gctUINT32 * physicalArrayLogical = gcvNULL;
    gctSIZE_T bytes;
    gctPOINTER logical = Logical;
    gctUINT32 i;
    gctSIZE_T pageSize;
    gctUINT32 pageMask;

    gcmkHEADER();

    gcmkVERIFY_OK(gckOS_GetPageSize(Kernel->os, &pageSize));

    pageMask = (gctUINT32)pageSize - 1;

    bytes = PageCount * gcmSIZEOF(gctUINT32);

     gcmkONERROR(gckOS_Allocate(
        Kernel->os,
        bytes,
        (gctPOINTER *)&physicalArrayLogical
        ));

    /* Fill in physical array. */
    for (i = 0; i < PageCount; i++)
    {
        gctPHYS_ADDR_T phys;
        status = gckOS_PhysicalToPhysicalAddress(
            Kernel->os,
            Physical,
            i * 4096,
            &phys
            );

        if (status == gcvSTATUS_NOT_SUPPORTED)
        {
            gcmkONERROR(gckOS_GetPhysicalAddress(
                Kernel->os,
                logical,
                &phys
                ));

            gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(Kernel->os, phys, &phys));
        }

        phys &= ~pageMask;

        gcmkSAFECASTPHYSADDRT(physicalArrayLogical[i], phys);

        logical = (gctUINT8_PTR)logical + 4096;
    }

    gcmkONERROR(gckKERNEL_SecurityMapMemory(
        Kernel,
        physicalArrayLogical,
        0,
        (gctUINT32)PageCount,
        &GPUAddress
        ));

    gcmkVERIFY_OK(gckOS_Free(
        Kernel->os,
        physicalArrayLogical
        ))

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if(physicalArrayLogical != gcvNULL)
        gcmkVERIFY_OK(gckOS_Free(
            Kernel->os,
            (gctPOINTER)physicalArrayLogical
            ));
    gcmkFOOTER();
    return status;
}
#endif

/*******************************************************************************
***** Test Code ****************************************************************
*******************************************************************************/

