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


#ifndef __gc_hal_kernel_device_h_
#define __gc_hal_kernel_device_h_

#include "gc_hal_kernel_debugfs.h"
#include "gc_hal_ta.h"

typedef struct _gcsDEVICE_CONSTRUCT_ARGS
{
    gctBOOL             recovery;
    gctUINT             stuckDump;
    gctUINT             gpu3DMinClock;

    gctBOOL             contiguousRequested;
    gcsPLATFORM*        platform;
    gctBOOL             mmu;
    gctBOOL             registerMemMapped;
    gctPOINTER             registerMemAddress;
#if gcdDEC_ENABLE_AHB
    gctUINT32           registerMemBaseDEC300;
    gctSIZE_T           registerMemSizeDEC300;
#endif
    gctINT              irqs[gcvCORE_COUNT];
    gctUINT             registerBases[gcvCORE_COUNT];
    gctUINT             registerSizes[gcvCORE_COUNT];
    gctBOOL             powerManagement;
    gctBOOL             gpuProfiler;
    gctUINT             chipIDs[gcvCORE_COUNT];
}
gcsDEVICE_CONSTRUCT_ARGS;

/******************************************************************************\
************************** gckGALDEVICE Structure ******************************
\******************************************************************************/

typedef struct _gckGALDEVICE
{
    /* Objects. */
    gckOS               os;
    gckKERNEL           kernels[gcdMAX_GPU_COUNT];

    gcsPLATFORM*        platform;

    /* Attributes. */
    gctSIZE_T           internalSize;
    gctPHYS_ADDR        internalPhysical;
    gctUINT32           internalPhysicalName;
    gctPOINTER          internalLogical;
    gckVIDMEM           internalVidMem;

    gctUINT32           externalBase;
    gctSIZE_T           externalSize;
    gctPHYS_ADDR        externalPhysical;
    gctUINT32           externalPhysicalName;
    gctPOINTER          externalLogical;
    gckVIDMEM           externalVidMem;

    gctPHYS_ADDR_T      contiguousBase;
    gctSIZE_T           contiguousSize;

    gckVIDMEM           contiguousVidMem;
    gctPOINTER          contiguousLogical;
    gctPHYS_ADDR        contiguousPhysical;
    gctUINT32           contiguousPhysicalName;

    gctSIZE_T           systemMemorySize;
    gctUINT32           systemMemoryBaseAddress;
    gctPOINTER          registerBases[gcdMAX_GPU_COUNT];
    gctSIZE_T           registerSizes[gcdMAX_GPU_COUNT];

    gctUINT32           baseAddress;
    gctUINT32           physBase;
    gctUINT32           physSize;

    /* By request_mem_region. */
    gctUINT32           requestedRegisterMemBases[gcdMAX_GPU_COUNT];
    gctSIZE_T           requestedRegisterMemSizes[gcdMAX_GPU_COUNT];

    /* By request_mem_region. */
    gctUINT32           requestedContiguousBase;
    gctSIZE_T           requestedContiguousSize;

    /* IRQ management. */
    gctINT              irqLines[gcdMAX_GPU_COUNT];
    gctBOOL             isrInitializeds[gcdMAX_GPU_COUNT];

    /* Thread management. */
    struct task_struct  *threadCtxts[gcdMAX_GPU_COUNT];
    struct semaphore    semas[gcdMAX_GPU_COUNT];
    gctBOOL             threadInitializeds[gcdMAX_GPU_COUNT];
    gctBOOL             killThread;

    /* Signal management. */
    gctINT              signal;

    /* States before suspend. */
    gceCHIPPOWERSTATE   statesStored[gcdMAX_GPU_COUNT];

    /* Device Debug File System Entry in kernel. */
    struct _gcsDEBUGFS_Node * dbgNode;

    gcsDEBUGFS_DIR      debugfsDir;

    gckDEVICE           device;

    gcsDEVICE_CONSTRUCT_ARGS args;

    /* gctsOs object for trust application. */
    gctaOS              taos;

#if gcdENABLE_DRM
    void*               drm;
#endif
}
* gckGALDEVICE;

typedef struct _gcsHAL_PRIVATE_DATA
{
    gckGALDEVICE        device;
    /*
     * 'fput' schedules actual work in '__fput' in a different thread.
     * So the process opens the device may not be the same as the one that
     * closes it.
     */
    gctUINT32           pidOpen;
    gctBOOL             isLocked;
}
gcsHAL_PRIVATE_DATA, * gcsHAL_PRIVATE_DATA_PTR;

gceSTATUS gckGALDEVICE_Setup_ISR(
    IN gceCORE Core
    );

gceSTATUS gckGALDEVICE_Setup_ISR_VG(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Release_ISR(
    IN gceCORE Core
    );

gceSTATUS gckGALDEVICE_Release_ISR_VG(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Start_Threads(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Stop_Threads(
    gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Stop(
    gckGALDEVICE Device
    );

gceSTATUS gckGALDEVICE_Construct(
    IN gctINT IrqLine,
    IN gctUINT32 RegisterMemBase,
    IN gctSIZE_T RegisterMemSize,
    IN gctINT IrqLine2D,
    IN gctUINT32 RegisterMemBase2D,
    IN gctSIZE_T RegisterMemSize2D,
    IN gctINT IrqLineVG,
    IN gctUINT32 RegisterMemBaseVG,
    IN gctSIZE_T RegisterMemSizeVG,
    IN gctUINT32 ContiguousBase,
    IN gctSIZE_T ContiguousSize,
    IN gctUINT32 ExternalBase,
    IN gctSIZE_T ExternalSize,
    IN gctSIZE_T BankSize,
    IN gctINT FastClear,
    IN gctINT Compression,
    IN gctUINT32 PhysBaseAddr,
    IN gctUINT32 PhysSize,
    IN gctINT Signal,
    IN gctUINT LogFileSize,
    IN gctINT PowerManagement,
    IN gctINT GpuProfiler,
    IN gcsDEVICE_CONSTRUCT_ARGS * Args,
    OUT gckGALDEVICE *Device
    );

gceSTATUS gckGALDEVICE_Destroy(
    IN gckGALDEVICE Device
    );

static gcmINLINE gckKERNEL
_GetValidKernel(
    gckGALDEVICE Device
    )
{
    if (Device->kernels[gcvCORE_MAJOR])
    {
        return Device->kernels[gcvCORE_MAJOR];
    }
    else
    if (Device->kernels[gcvCORE_2D])
    {
        return Device->kernels[gcvCORE_2D];
    }
    else
    if (Device->kernels[gcvCORE_VG])
    {
        return Device->kernels[gcvCORE_VG];
    }
    else
    {
        gcmkASSERT(gcvFALSE);
        return gcvNULL;
    }
}

#endif /* __gc_hal_kernel_device_h_ */
