/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
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
*    Copyright (C) 2014 - 2022 Vivante Corporation
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


#ifndef _gc_hal_kernel_platform_h_
#define _gc_hal_kernel_platform_h_
#include <linux/mm.h>
#include <linux/platform_device.h>
#if USE_LINUX_PCIE
#    include <linux/pci.h>
#endif

#if USE_LINUX_PCIE
#    define gcdMAX_PCIE_BAR 6
#endif

typedef struct _gcsMODULE_PARAMETERS {
    gctUINT                 chipIDs[gcvCORE_COUNT];
    gctINT                  irqs[gcdGLOBAL_CORE_COUNT];
    gctPHYS_ADDR_T          registerBases[gcdGLOBAL_CORE_COUNT];
    gctSIZE_T               registerSizes[gcdGLOBAL_CORE_COUNT];
    gctPOINTER              registerBasesMapped[gcdGLOBAL_CORE_COUNT];

    gctINT                  irq2Ds[gcdGLOBAL_2D_COUNT];
    gctPHYS_ADDR_T          register2DBases[gcdGLOBAL_2D_COUNT];
    gctSIZE_T               register2DSizes[gcdGLOBAL_2D_COUNT];
    gctPOINTER              register2DBasesMapped[gcdGLOBAL_2D_COUNT];

    gctINT                  irqVG;
    gctPHYS_ADDR_T          registerVGBase;
    gctSIZE_T               registerVGSize;
    gctPOINTER              registerVGBaseMapped;

    /* Contiguous memory pool. */
    gctPHYS_ADDR_T          contiguousBase;
    gctSIZE_T               contiguousSize;
    gctPHYS_ADDR_T          contiguousBases[gcdSYSTEM_RESERVE_COUNT];
    gctSIZE_T               contiguousSizes[gcdSYSTEM_RESERVE_COUNT];
    gctBOOL                 contiguousRequested;

    /* External memory pool. */
    gctPHYS_ADDR_T          externalBase[gcdDEVICE_COUNT];
    gctSIZE_T               externalSize[gcdDEVICE_COUNT];

    /* External memory pool. */
    gctPHYS_ADDR_T          exclusiveBase[gcdDEVICE_COUNT];
    gctSIZE_T               exclusiveSize[gcdDEVICE_COUNT];

    /* Per-core SRAM. */
    gctPHYS_ADDR_T          sRAMBases[gcdGLOBAL_CORE_COUNT][gcvSRAM_INTER_COUNT];
    gctUINT32               sRAMSizes[gcdGLOBAL_CORE_COUNT][gcvSRAM_INTER_COUNT];

    /* Shared SRAM. */
    gctPHYS_ADDR_T          extSRAMBases[gcvSRAM_EXT_COUNT];
    gctUINT32               extSRAMSizes[gcvSRAM_EXT_COUNT];
#if USE_LINUX_PCIE
    gctINT                  bars[gcdGLOBAL_CORE_COUNT];
    gctUINT32               regOffsets[gcdGLOBAL_CORE_COUNT];
    gctINT                  bar2Ds[gcdGLOBAL_2D_COUNT];
    gctUINT32               reg2DOffsets[gcdGLOBAL_2D_COUNT];
    gctINT                  barVG;
    gctUINT32               regVGOffset;
    gctINT32                sRAMBars[gcvSRAM_EXT_COUNT];
    gctINT32                sRAMOffsets[gcvSRAM_EXT_COUNT];
#endif

    gctUINT                 platformIDs[gcdDEVICE_COUNT];
    gctUINT                 hwDevCounts[gcdPLATFORM_COUNT];
    gctUINT                 devCoreCounts[gcdDEVICE_COUNT];
    gctUINT                 dev2DCoreCounts[gcdDEVICE_COUNT];
    gctUINT                 devMemIDs[gcdDEVICE_COUNT];
    gctUINT                 devSysMemIDs[gcdDEVICE_COUNT];
    gctUINT                 devSRAMIDs[gcdDEVICE_COUNT];
    gctUINT                 devCount;

    gctBOOL                 sRAMRequested;
    gctUINT32               sRAMLoopMode;

    gctPHYS_ADDR_T          baseAddress;
    gctSIZE_T               physSize;
    gctSIZE_T               bankSize;

    gctUINT                 recovery;
    gctINT                  powerManagement;

    gctINT                  enableMmu;
    gctINT                  fastClear;
    gceCOMPRESSION_OPTION   compression;
    gctUINT                 gpu3DMinClock;
    gctUINT                 userClusterMasks[gcdMAX_MAJOR_CORE_COUNT];
    gctUINT                 smallBatch;

    /* Debug or other information. */
    gctUINT                 stuckDump;
    gctUINT                 softReset;
    /* device type, 0 for char device, 1 for misc device. */
    gctUINT                 deviceType;
    gctUINT                 showArgs;

    /* mmu page table pool, 0 mean auto, 1 means virtual. */
    gctUINT                 mmuPageTablePool;

    /* mmu page table pool, 1 mean auto, others means specific pool. */
    gctUINT                 mmuCmdPool;

    gctUINT                 mmuDynamicMap;
    gctUINT                 allMapInOne;

    gctUINT                 isrPoll;

    /* APB register offset to the register base address. */
    gctUINT64               registerAPB;

    /* Enabled NN clusters number */
    gctUINT                 enableNN;

    /* Struct device array. */
    struct device           *devices[gcdDEVICE_COUNT];

    /* GPU timeout for operation. */
    gctUINT                 gpuTimeout;
} gcsMODULE_PARAMETERS;

typedef struct _gcsDMA_TRANS_INFO {
    gctPOINTER src_node;
    gctPOINTER dst_node;
    gctSIZE_T offset;
    gctSIZE_T bytes;
    gctUINT32 reason;
} gcsDMA_TRANS_INFO;

typedef struct _gcsPLATFORM gcsPLATFORM;

typedef struct _gcsPLATFORM_OPERATIONS {
    /**********************************************************************
     **
     **  adjustParam
     **
     **  Override content of arguments, if a argument is not changed here,
     **  it will keep as default value or value set by insmod command line.
     */
    gceSTATUS
    (*adjustParam)(IN gcsPLATFORM *Platform, OUT gcsMODULE_PARAMETERS *Args);

    /**********************************************************************
     **
     **  getPower
     **
     **  Prepare power and clock operation.
     */
    gceSTATUS
    (*getPower)(IN gcsPLATFORM *Platform);

    /**********************************************************************
     **
     **  putPower
     **
     **  Finish power and clock operation.
     */
    gceSTATUS
    (*putPower)(IN gcsPLATFORM *Platform);

    /**********************************************************************
     **
     **  setPower
     **
     **  Set power state of specified GPU.
     **
     **  INPUT:
     **
     **      gceCORE GPU
     **          GPU neeed to config.
     **
     **      gceBOOL Enable
     **          Enable or disable power.
     */
    gceSTATUS
    (*setPower)(IN gcsPLATFORM *Platform,
                IN gctUINT32    DevIndex,
                IN gceCORE      GPU,
                IN gctBOOL      Enable);

    /**********************************************************************
     **
     **  setClock
     **
     **  Set clock state of specified GPU.
     **
     **  INPUT:
     **
     **      gceCORE GPU
     **          GPU neeed to config.
     **
     **      gceBOOL Enable
     **          Enable or disable clock.
     */
    gceSTATUS
    (*setClock)(IN gcsPLATFORM *Platform,
                IN gctUINT32    DevIndex,
                IN gceCORE      GPU,
                IN gctBOOL      Enable);

    /**********************************************************************
     **
     **  reset
     **
     **  Reset GPU outside.
     **
     **  INPUT:
     **
     **      gceCORE GPU
     **          GPU neeed to reset.
     */
    gceSTATUS
    (*reset)(IN gcsPLATFORM *Platform, IN gctUINT32 DevIndex, IN gceCORE GPU);

    /**********************************************************************
     **
     **  getGPUPhysical
     **
     **  Convert CPU physical address to GPU physical address if they are
     **  different.
     */
    gceSTATUS
    (*getGPUPhysical)(IN gcsPLATFORM     *Platform,
                      IN gctPHYS_ADDR_T   CPUPhysical,
                      OUT gctPHYS_ADDR_T *GPUPhysical);

    /**********************************************************************
     **
     **  getCPUPhysical
     **
     **  Convert GPU physical address to CPU physical address if they are
     **  different.
     */
    gceSTATUS
    (*getCPUPhysical)(IN gcsPLATFORM     *Platform,
                      IN gctPHYS_ADDR_T   GPUPhysical,
                      OUT gctPHYS_ADDR_T *CPUPhysical);

    /**********************************************************************
     **
     **  adjustProt
     **
     **  Override Prot flag when mapping paged memory to userspace.
     */
    gceSTATUS
    (*adjustProt)(IN struct vm_area_struct *vma);

    /**********************************************************************
     **
     **  shrinkMemory
     **
     **  Do something to collect memory, eg, act as oom killer.
     */
    gceSTATUS
    (*shrinkMemory)(IN gcsPLATFORM *Platform);

    /**********************************************************************
     **
     ** getPolicyID
     **
     ** Get policyID for a specified surface type.
     */
    gceSTATUS
    (*getPolicyID)(IN gcsPLATFORM   *Platform,
                   IN gceVIDMEM_TYPE Type,
                   OUT gctUINT32_PTR PolicyID,
                   OUT gctUINT32_PTR AXIConfig);

    /**********************************************************************
     **
     **  _ExternalCacheOperation
     **
     **  External device cache operation, if support. If the core has any
     **  additional caches they must be invalidated after this function
     **  returns.
     **  If the core does not have any additional caches the
     **  externalCacheOperation in the platform->ops should remain NULL.
     **  The function may be called by multiple thread, so need to add
     **  mutex in the callback when there is shared resource.
     **
     **  INPUT:
     **      gceCACHEOPERATION Operation
     **          Cache Operation: gcvCACHE_FLUSH, gcvCACHE_CLEAN or
     **          gcvCACHE_INVALIDATE.
     **
     **  OUTPUT:
     **
     **      Nothing.
     */
    void
    (*externalCacheOperation)(IN gcsPLATFORM      *Platform,
                              IN gceCACHEOPERATION Operation);

    /**********************************************************************
     **
     ** dmaCopy
     **
     ** copy videm memory by dma
     */
    gceSTATUS
    (*dmaCopy)(IN gctPOINTER Object, IN gcsDMA_TRANS_INFO *Info);

    /**********************************************************************
     **
     ** dmaInit
     **
     ** Init platform specified dma
     */
    gceSTATUS
    (*dmaInit)(gcsPLATFORM *Platform);

    /**********************************************************************
     **
     ** dmaExit
     **
     ** deInit platform dma
     */
    gceSTATUS
    (*dmaExit)(gcsPLATFORM *Platform);

#if gcdENABLE_MP_SWITCH
    /**********************************************************************
     ** switchCoreCount
     **
     ** Switch the core count according to specific conditions.
     **
     */
    gceSTATUS
    (*switchCoreCount)(IN gcsPLATFORM *Platform, OUT gctUINT32 *Count);
#endif
} gcsPLATFORM_OPERATIONS;

struct _gcsPLATFORM {
    struct platform_device *device;
    struct platform_driver *driver;

    const char             *name;
    gcsPLATFORM_OPERATIONS *ops;

    gckDEVICE               dev;

    /* PLATFORM specific flags */
    gctUINT32               flagBits;

    /* Real-time core count. */
    gctUINT32               coreCount;

    void                   *priv;
    /* Module special parameters */
    gcsMODULE_PARAMETERS    params;
};

int gckPLATFORM_Init(struct platform_driver *pdrv, gcsPLATFORM **platform);
int gckPLATFORM_Terminate(gcsPLATFORM *platform);

#endif
