/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2020 Vivante Corporation
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
*    Copyright (C) 2014 - 2020 Vivante Corporation
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


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_allocator.h"

#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mman.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,16,0)
#include <linux/dma-direct.h>
#endif

#define _GC_OBJ_ZONE    gcvZONE_OS

typedef struct _gcsCMA_PRIV * gcsCMA_PRIV_PTR;
typedef struct _gcsCMA_PRIV {
    atomic_t cmasize;
    gctBOOL cmaLimitRequest;
}
gcsCMA_PRIV;

struct mdl_cma_priv {
    gctPOINTER kvaddr;
    dma_addr_t physical;
};

static int gc_cma_usage_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckALLOCATOR Allocator = node->device;
    gcsCMA_PRIV_PTR priv = Allocator->privateData;
    long long size = (long long)atomic_read(&priv->cmasize);

    seq_printf(m, "type        n pages        bytes\n");
    seq_printf(m, "cma      %10llu %12llu\n", size, size * PAGE_SIZE);

    return 0;
}

static gcsINFO InfoList[] =
{
    {"cmausage", gc_cma_usage_show},
};

static void
_CMAAllocatorDebugfsInit(
    IN gckALLOCATOR Allocator,
    IN gckDEBUGFS_DIR Root
    )
{
    gcmkVERIFY_OK(
        gckDEBUGFS_DIR_Init(&Allocator->debugfsDir, Root->root, "cma"));

    gcmkVERIFY_OK(gckDEBUGFS_DIR_CreateFiles(
        &Allocator->debugfsDir,
        InfoList,
        gcmCOUNTOF(InfoList),
        Allocator
        ));
}

static void
_CMAAllocatorDebugfsCleanup(
    IN gckALLOCATOR Allocator
    )
{
    gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(
        &Allocator->debugfsDir,
        InfoList,
        gcmCOUNTOF(InfoList)
        ));

    gckDEBUGFS_DIR_Deinit(&Allocator->debugfsDir);
}

static gceSTATUS
_CMAFSLAlloc(
    IN gckALLOCATOR Allocator,
    INOUT PLINUX_MDL Mdl,
    IN gctSIZE_T NumPages,
    IN gctUINT32 Flags
    )
{
    gceSTATUS status;
    u32 gfp = GFP_KERNEL | gcdNOWARN;
    gcsCMA_PRIV_PTR priv = (gcsCMA_PRIV_PTR)Allocator->privateData;

    struct mdl_cma_priv *mdl_priv=gcvNULL;
    gckOS os = Allocator->os;

    gcmkHEADER_ARG("Mdl=%p NumPages=0x%zx", Mdl, NumPages);

    if (os->allocatorLimitMarker && !(Flags & gcvALLOC_FLAG_CMA_PREEMPT))
    {
        if (Flags & gcvALLOC_FLAG_CMA_LIMIT)
        {
            priv->cmaLimitRequest = gcvTRUE;
        }
        else if (priv->cmaLimitRequest == gcvTRUE)
        {
            gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        }
    }

    gcmkONERROR(gckOS_Allocate(os, sizeof(struct mdl_cma_priv), (gctPOINTER *)&mdl_priv));
    mdl_priv->kvaddr = gcvNULL;

#if defined(CONFIG_ZONE_DMA32) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    if (Flags & gcvALLOC_FLAG_4GB_ADDR)
    {
        gfp |= __GFP_DMA32;
    }
#endif

#if gcdENABLE_BUFFERABLE_VIDEO_MEMORY
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
    mdl_priv->kvaddr = dma_alloc_wc(&os->device->platform->device->dev,
#else
    mdl_priv->kvaddr = dma_alloc_writecombine(&os->device->platform->device->dev,
#endif
#else
    mdl_priv->kvaddr = dma_alloc_coherent(&os->device->platform->device->dev,
#endif
            NumPages * PAGE_SIZE,
            &mdl_priv->physical,
            gfp);

    if (mdl_priv->kvaddr == gcvNULL)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    Mdl->priv = mdl_priv;
    Mdl->dmaHandle = mdl_priv->physical;
    atomic_add(NumPages, &priv->cmasize);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (mdl_priv)
    {
        gckOS_Free(os, mdl_priv);
    }
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_CMAFSLGetSGT(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *SGT
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
    struct page ** pages = gcvNULL;
    struct page * page = gcvNULL;
    struct sg_table *sgt = NULL;
    struct mdl_cma_priv *mdl_priv = (struct mdl_cma_priv*)Mdl->priv;

    gceSTATUS status = gcvSTATUS_OK;
    gctSIZE_T offset = Offset & ~PAGE_MASK; /* Offset to the first page */
    gctINT skipPages = Offset >> PAGE_SHIFT;     /* skipped pages */
    gctINT numPages = (PAGE_ALIGN(Offset + Bytes) >> PAGE_SHIFT) - skipPages;
    gctINT i;

    gcmkASSERT(Offset + Bytes <= Mdl->numPages << PAGE_SHIFT);

    sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL | gcdNOWARN);
    if (!sgt)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    pages = kmalloc(sizeof(struct page*) * numPages, GFP_KERNEL | gcdNOWARN);
    if (!pages)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

#if !defined(phys_to_page)
    page = virt_to_page(mdlPriv->kvaddr);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
    page = phys_to_page(mdlPriv->physical);
#else
    page = phys_to_page(dma_to_phys(&Allocator->os->device->platform->device->dev, mdl_priv->physical));
#endif

    for (i = 0; i < numPages; ++i)
    {
        pages[i] = nth_page(page, i + skipPages);
    }

    if (sg_alloc_table_from_pages(sgt, pages, numPages, offset, Bytes, GFP_KERNEL) < 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    *SGT = (gctPOINTER)sgt;

OnError:
    if (pages)
    {
        kfree(pages);
    }

    if (gcmIS_ERROR(status) && sgt)
    {
        kfree(sgt);
    }

    return status;
#else
    return gcvSTATUS_NOT_SUPPORTED;
#endif
}

static void
_CMAFSLFree(
    IN gckALLOCATOR Allocator,
    IN OUT PLINUX_MDL Mdl
    )
{
    gckOS os = Allocator->os;
    struct mdl_cma_priv *mdlPriv=(struct mdl_cma_priv *)Mdl->priv;
    gcsCMA_PRIV_PTR priv = (gcsCMA_PRIV_PTR)Allocator->privateData;

#if gcdENABLE_BUFFERABLE_VIDEO_MEMORY
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
    dma_free_wc(&os->device->platform->device->dev,
#else
    dma_free_writecombine(&os->device->platform->device->dev,
#endif
#else
    dma_free_coherent(&os->device->platform->device->dev,
#endif
            Mdl->numPages * PAGE_SIZE,
            mdlPriv->kvaddr,
            mdlPriv->physical);

    gckOS_Free(os, mdlPriv);
    atomic_sub(Mdl->numPages, &priv->cmasize);
}

static gceSTATUS
_CMAFSLMmap(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctBOOL Cacheable,
    IN gctSIZE_T skipPages,
    IN gctSIZE_T numPages,
    IN struct vm_area_struct *vma
    )
{
    gckOS os = Allocator->os;
    struct mdl_cma_priv *mdlPriv = (struct mdl_cma_priv*)Mdl->priv;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p vma=%p", Allocator, Mdl, vma);

    gcmkASSERT(skipPages + numPages <= Mdl->numPages);

    /* Now map all the vmalloc pages to this user address. */
    if (Mdl->contiguous)
    {
#if gcdENABLE_BUFFERABLE_VIDEO_MEMORY
        /* map kernel memory to user space.. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,6,0)
        if (dma_mmap_wc(&os->device->platform->device->dev,
#else
        if (dma_mmap_writecombine(&os->device->platform->device->dev,
#endif
#else
        if (dma_mmap_coherent(&os->device->platform->device->dev,
#endif
            vma,
            (gctINT8_PTR)mdlPriv->kvaddr + (skipPages << PAGE_SHIFT),
            mdlPriv->physical + (skipPages << PAGE_SHIFT),
            numPages << PAGE_SHIFT) < 0)

        {
            gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): dma_mmap_attrs error",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }
    else
    {
        gcmkFATAL("%s(%d): unexpected noncontiguous mdl\n");
        gcmkONERROR(gcvSTATUS_HEAP_CORRUPTED);
    }

OnError:
    gcmkFOOTER();
    return status;
}

static void
_CMAFSLUnmapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctUINT32 Size
    )
{
    if (unlikely(current->mm == gcvNULL))
    {
        /* Do nothing if process is exiting. */
        return;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
    if (vm_munmap((unsigned long)MdlMap->vmaAddr, Size) < 0)
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): vm_munmap failed",
                __FUNCTION__, __LINE__
                );
    }
#else
    down_write(&current_mm_mmap_sem);
    if (do_munmap(current->mm, (unsigned long)MdlMap->vmaAddr, Size) < 0)
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): do_munmap failed",
                __FUNCTION__, __LINE__
                );
    }
    up_write(&current_mm_mmap_sem);
#endif
}

static gceSTATUS
_CMAFSLMapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctBOOL Cacheable
    )
{
    gctPOINTER userLogical = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p Cacheable=%d", Allocator, Mdl, Cacheable);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
    userLogical = (gctPOINTER)vm_mmap(gcvNULL,
                    0L,
                    Mdl->numPages * PAGE_SIZE,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_NORESERVE,
                    0);
#else
    down_write(&current_mm_mmap_sem);
    userLogical = (gctPOINTER)do_mmap_pgoff(gcvNULL,
                    0L,
                    Mdl->numPages * PAGE_SIZE,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED,
                    0);
    up_write(&current_mm_mmap_sem);
#endif

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_OS,
        "%s(%d): vmaAddr->%p for phys_addr->%p",
        __FUNCTION__, __LINE__, userLogical, Mdl
        );

    if (IS_ERR(userLogical))
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "%s(%d): do_mmap_pgoff error",
            __FUNCTION__, __LINE__
            );

        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    down_write(&current_mm_mmap_sem);

    do
    {
        struct vm_area_struct *vma = find_vma(current->mm, (unsigned long)userLogical);
        if (vma == gcvNULL)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_INFO, gcvZONE_OS,
                "%s(%d): find_vma error",
                __FUNCTION__, __LINE__
                );

            gcmkERR_BREAK(gcvSTATUS_OUT_OF_RESOURCES);
        }

        gcmkERR_BREAK(_CMAFSLMmap(Allocator, Mdl, Cacheable, 0, Mdl->numPages, vma));
        MdlMap->vmaAddr = userLogical;
        MdlMap->cacheable = Cacheable;
        MdlMap->vma = vma;
    }
    while (gcvFALSE);

    up_write(&current_mm_mmap_sem);

OnError:
    if (gcmIS_ERROR(status) && userLogical && !IS_ERR(userLogical))
    {
        _CMAFSLUnmapUser(Allocator, Mdl, userLogical, Mdl->numPages * PAGE_SIZE);
    }
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_CMAMapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *Logical
    )
{
    struct mdl_cma_priv *mdl_priv=(struct mdl_cma_priv *)Mdl->priv;
    *Logical = (uint8_t *)mdl_priv->kvaddr + Offset;
    return gcvSTATUS_OK;
}

static gceSTATUS
_CMAUnmapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{
    return gcvSTATUS_OK;
}

static gceSTATUS
_CMACache(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctPOINTER Logical,
    IN gctSIZE_T Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    switch (Operation)
    {
    case gcvCACHE_CLEAN:
    case gcvCACHE_FLUSH:
        _MemoryBarrier();
        break;
    case gcvCACHE_INVALIDATE:
        break;
    default:
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_CMAPhysical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    struct mdl_cma_priv *mdl_priv=(struct mdl_cma_priv *)Mdl->priv;

    *Physical = mdl_priv->physical + Offset;

    return gcvSTATUS_OK;
}

static void
_CMAAllocatorDestructor(
    gcsALLOCATOR *Allocator
    )
{
    _CMAAllocatorDebugfsCleanup(Allocator);

    if (Allocator->privateData)
    {
        kfree(Allocator->privateData);
    }

    kfree(Allocator);
}

/* Default allocator operations. */
static gcsALLOCATOR_OPERATIONS CMAFSLAllocatorOperations =
{
    .Alloc              = _CMAFSLAlloc,
    .Free               = _CMAFSLFree,
    .Mmap               = _CMAFSLMmap,
    .MapUser            = _CMAFSLMapUser,
    .UnmapUser          = _CMAFSLUnmapUser,
    .MapKernel          = _CMAMapKernel,
    .UnmapKernel        = _CMAUnmapKernel,
    .Cache              = _CMACache,
    .Physical           = _CMAPhysical,
    .GetSGT             = _CMAFSLGetSGT,
};

/* Default allocator entry. */
gceSTATUS
_CMAFSLAlloctorInit(
    IN gckOS Os,
    IN gcsDEBUGFS_DIR *Parent,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator = gcvNULL;
    gcsCMA_PRIV_PTR priv = gcvNULL;

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &CMAFSLAllocatorOperations, &allocator));

    priv = kzalloc(gcmSIZEOF(gcsCMA_PRIV), GFP_KERNEL | gcdNOWARN);

    if (!priv)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    atomic_set(&priv->cmasize, 0);

    /* Register private data. */
    allocator->privateData = priv;
    allocator->destructor = _CMAAllocatorDestructor;

    _CMAAllocatorDebugfsInit(allocator, Parent);

    allocator->capability = gcvALLOC_FLAG_CONTIGUOUS
                          | gcvALLOC_FLAG_DMABUF_EXPORTABLE
#if defined(CONFIG_ZONE_DMA32) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                          | gcvALLOC_FLAG_4GB_ADDR
#endif
                          ;
#if defined(CONFIG_ARM64)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,0,0)
    Os->allocatorLimitMarker = (Os->device->baseAddress + totalram_pages() * PAGE_SIZE) > 0x100000000;
#else
    Os->allocatorLimitMarker = (Os->device->baseAddress + totalram_pages * PAGE_SIZE) > 0x100000000;
#endif
#else
    Os->allocatorLimitMarker = gcvFALSE;
#endif
    priv->cmaLimitRequest = gcvFALSE;

    if (Os->allocatorLimitMarker)
    {
        allocator->capability |= gcvALLOC_FLAG_CMA_LIMIT;
    }

    allocator->capability |= gcvALLOC_FLAG_CMA_PREEMPT;

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    if (allocator)
    {
        kfree(allocator);
    }
    return status;
}

