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


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_allocator.h"
#include <linux/pagemap.h>
#include <linux/seq_file.h>
#include <linux/mman.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "gc_hal_kernel_platform.h"

#define _GC_OBJ_ZONE    gcvZONE_OS

#define gcdDISCRETE_PAGES 0

struct gfp_alloc
{
    atomic_t low;
    atomic_t high;
};

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,24)
struct sg_table
{
    struct scatterlist *sgl;
    unsigned int nents;
    unsigned int orig_nents;
};
#endif

struct gfp_mdl_priv
{
    int contiguous;

    union
    {
        /* Pointer to a array of pages. */
        struct
        {
            struct page *contiguousPages;
            dma_addr_t dma_addr;
            int exact;
        };

        struct
        {
            /* Pointer to a array of pointers to page. */
            struct page **nonContiguousPages;
            struct sg_table sgt;
        };
    };

    gcsPLATFORM * platform;
};

/******************************************************************************\
************************** GFP Allocator Debugfs ***************************
\******************************************************************************/

static int gc_usage_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckALLOCATOR Allocator = node->device;
    struct gfp_alloc *priv = Allocator->privateData;
    long long low  = (long long)atomic_read(&priv->low);
    long long high = (long long)atomic_read(&priv->high);

    seq_printf(m, "type        n pages        bytes\n");
    seq_printf(m, "normal   %10llu %12llu\n", low, low * PAGE_SIZE);
    seq_printf(m, "HighMem  %10llu %12llu\n", high, high * PAGE_SIZE);

    return 0;
}

static gcsINFO InfoList[] =
{
    {"usage", gc_usage_show},
};

static void
_GFPAllocatorDebugfsInit(
    IN gckALLOCATOR Allocator,
    IN gckDEBUGFS_DIR Root
    )
{
    gcmkVERIFY_OK(
        gckDEBUGFS_DIR_Init(&Allocator->debugfsDir, Root->root, "gfp"));

    gcmkVERIFY_OK(gckDEBUGFS_DIR_CreateFiles(
        &Allocator->debugfsDir,
        InfoList,
        gcmCOUNTOF(InfoList),
        Allocator
        ));
}

static void
_GFPAllocatorDebugfsCleanup(
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

static void
_NonContiguousFree(
    IN struct page ** Pages,
    IN gctUINT32 NumPages
    )
{
    gctINT i;

    gcmkHEADER_ARG("Pages=%p, NumPages=%u", Pages, NumPages);

    gcmkASSERT(Pages != gcvNULL);

    for (i = 0; i < NumPages; i++)
    {
        __free_page(Pages[i]);
    }

    if (is_vmalloc_addr(Pages))
    {
        vfree(Pages);
    }
    else
    {
        kfree(Pages);
    }

    gcmkFOOTER_NO();
}

static struct page **
_NonContiguousAlloc(
    IN gctUINT32 NumPages,
    IN gctUINT32 Gfp
    )
{
    struct page ** pages;
    struct page *p;
    gctINT i, size;

    gcmkHEADER_ARG("NumPages=%u", NumPages);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
    if (NumPages > totalram_pages)
#else
    if (NumPages > num_physpages)
#endif
    {
        gcmkFOOTER_NO();
        return gcvNULL;
    }

    size = NumPages * sizeof(struct page *);

    pages = kmalloc(size, GFP_KERNEL | gcdNOWARN);

    if (!pages)
    {
        pages = vmalloc(size);

        if (!pages)
        {
            gcmkFOOTER_NO();
            return gcvNULL;
        }
    }

    for (i = 0; i < NumPages; i++)
    {
        p = alloc_page(Gfp);

        if (!p)
        {
            _NonContiguousFree(pages, i);
            gcmkFOOTER_NO();
            return gcvNULL;
        }

#if gcdDISCRETE_PAGES
        if (i != 0)
        {
            if (page_to_pfn(pages[i-1]) == page_to_pfn(p)-1)
            {
                /* Replaced page. */
                struct page *l = p;

                /* Allocate a page which is not contiguous to previous one. */
                p = alloc_page(Gfp);

                /* Give replaced page back. */
                __free_page(l);

                if (!p)
                {
                    _NonContiguousFree(pages, i);
                    gcmkFOOTER_NO();
                    return gcvNULL;
                }
            }
        }
#endif

        pages[i] = p;
    }

    gcmkFOOTER_ARG("pages=0x%X", pages);
    return pages;
}

/***************************************************************************\
************************ GFP Allocator **********************************
\***************************************************************************/
static gceSTATUS
_GFPAlloc(
    IN gckALLOCATOR Allocator,
    INOUT PLINUX_MDL Mdl,
    IN gctSIZE_T NumPages,
    IN gctUINT32 Flags
    )
{
    gceSTATUS status;
    gctUINT i;
    u32 gfp = GFP_KERNEL | __GFP_HIGHMEM | gcdNOWARN;
    gctBOOL contiguous = Flags & gcvALLOC_FLAG_CONTIGUOUS;

    struct gfp_alloc *priv = (struct gfp_alloc *)Allocator->privateData;
    struct gfp_mdl_priv *mdlPriv = gcvNULL;
    int result;
    int low = 0;
    int high = 0;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p NumPages=%zu Flags=0x%x", Allocator, Mdl, NumPages, Flags);

#ifdef gcdSYS_FREE_MEMORY_LIMIT
    if (Flags & gcvALLOC_FLAG_MEMLIMIT)
    {
        struct sysinfo temsysinfo;
        si_meminfo(&temsysinfo);

        if ((temsysinfo.freeram < NumPages) || ((temsysinfo.freeram-NumPages) < gcdSYS_FREE_MEMORY_LIMIT))
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }
#endif

    mdlPriv = kzalloc(sizeof(struct gfp_mdl_priv), GFP_KERNEL | __GFP_NORETRY);

    if (!mdlPriv)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

#if defined(CONFIG_ZONE_DMA32) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
    if ((Flags & gcvALLOC_FLAG_4GB_ADDR) || (Allocator->os->device->platform->flagBits & gcvPLATFORM_FLAG_LIMIT_4G_ADDRESS))
    {
        /* remove __GFP_HIGHMEM bit, add __GFP_DMA32 bit */
        gfp &= ~__GFP_HIGHMEM;
        gfp |= __GFP_DMA32;
    }
#else
    if (Flags & gcvALLOC_FLAG_4GB_ADDR || (Allocator->os->device->platform->flagBits & gcvPLATFORM_FLAG_LIMIT_4G_ADDRESS))
    {
        /* remove __GFP_HIGHMEM bit, add __GFP_DMA bit */
        gfp &= ~__GFP_HIGHMEM;
        gfp |= __GFP_DMA;
    }

#endif

    if (contiguous)
    {
        size_t bytes = NumPages << PAGE_SHIFT;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
        void *addr = NULL;

        addr = alloc_pages_exact(bytes, (gfp & ~__GFP_HIGHMEM) | __GFP_NORETRY);

        mdlPriv->contiguousPages = addr ? virt_to_page(addr) : gcvNULL;

        if (mdlPriv->contiguousPages)
        {
            mdlPriv->exact = gcvTRUE;
        }
#endif

        if (mdlPriv->contiguousPages == gcvNULL)
        {
            int order = get_order(bytes);

            if (order >= MAX_ORDER)
            {
                status = gcvSTATUS_OUT_OF_MEMORY;
                goto OnError;
            }

            mdlPriv->contiguousPages = alloc_pages(gfp, order);

        }

        if (mdlPriv->contiguousPages == gcvNULL)
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

        mdlPriv->dma_addr = dma_map_page(galcore_device,
                mdlPriv->contiguousPages, 0, NumPages * PAGE_SIZE,
                DMA_TO_DEVICE);

        if (!mdlPriv->dma_addr)
        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
            if (mdlPriv->exact)
            {
                free_pages_exact(page_address(mdlPriv->contiguousPages), bytes);
            }
            else
#endif
            {
                __free_pages(mdlPriv->contiguousPages, get_order(bytes));
            }

            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

#if defined(CONFIG_X86)
        if (!PageHighMem(mdlPriv->contiguousPages))
        {
            if (set_memory_wc((unsigned long)page_address(mdlPriv->contiguousPages), NumPages) != 0)
            {
                printk("%s(%d): failed to set_memory_wc\n", __func__, __LINE__);
            }
        }
#endif
    }
    else
    {
        mdlPriv->nonContiguousPages = _NonContiguousAlloc(NumPages, gfp);

        if (mdlPriv->nonContiguousPages == gcvNULL)
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0) \
    && (defined(ARCH_HAS_SG_CHAIN) || defined(CONFIG_ARCH_HAS_SG_CHAIN))
        result = sg_alloc_table_from_pages(&mdlPriv->sgt,
                    mdlPriv->nonContiguousPages, NumPages, 0,
                    NumPages << PAGE_SHIFT, GFP_KERNEL);

#else
        result = alloc_sg_list_from_pages(&mdlPriv->sgt.sgl,
                    mdlPriv->nonContiguousPages, NumPages, 0,
                    NumPages << PAGE_SHIFT, &mdlPriv->sgt.nents);

        mdlPriv->sgt.orig_nents = mdlPriv->sgt.nents;
#endif
        if (result < 0)
        {
            _NonContiguousFree(mdlPriv->nonContiguousPages, NumPages);
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

        result = dma_map_sg(galcore_device,
                    mdlPriv->sgt.sgl, mdlPriv->sgt.nents, DMA_TO_DEVICE);

        if (result != mdlPriv->sgt.nents)
        {
            _NonContiguousFree(mdlPriv->nonContiguousPages, NumPages);

#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0) \
    && (defined (ARCH_HAS_SG_CHAIN) || defined (CONFIG_ARCH_HAS_SG_CHAIN))
            sg_free_table(&mdlPriv->sgt);
#else
            kfree(mdlPriv->sgt.sgl);
#endif
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

#if defined(CONFIG_X86)
        if (set_pages_array_wc(mdlPriv->nonContiguousPages, NumPages))
        {
            printk("%s(%d): failed to set_pages_array_wc\n", __func__, __LINE__);
        }
#endif
    }

    for (i = 0; i < NumPages; i++)
    {
        struct page *page;
        gctPHYS_ADDR_T phys = 0U;

        if (contiguous)
        {
            page = nth_page(mdlPriv->contiguousPages, i);
        }
        else
        {
            page = mdlPriv->nonContiguousPages[i];
        }

        SetPageReserved(page);

        phys = page_to_phys(page);

        BUG_ON(!phys);

        if (PageHighMem(page))
        {
            high++;
        }
        else
        {
            low++;
        }
    }

    mdlPriv->platform = Allocator->os->device->platform;
    mdlPriv->contiguous = contiguous;
    atomic_add(low, &priv->low);
    atomic_add(high, &priv->high);

    Mdl->priv = mdlPriv;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (mdlPriv)
    {
        kfree(mdlPriv);
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_GFPGetSGT(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctSIZE_T Offset,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *SGT
    )
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
    struct page ** pages = gcvNULL;
    struct page ** tmpPages = gcvNULL;
    struct sg_table *sgt = NULL;
    struct gfp_mdl_priv *mdlPriv = (struct gfp_mdl_priv*)Mdl->priv;

    gceSTATUS status = gcvSTATUS_OK;
    gctSIZE_T offset = Offset & ~PAGE_MASK; /* Offset to the first page */
    gctINT skipPages = Offset >> PAGE_SHIFT;     /* skipped pages */
    gctINT numPages = (PAGE_ALIGN(Offset + Bytes) >> PAGE_SHIFT) - skipPages;
    gctINT i;

    gcmkASSERT(Offset + Bytes <= Mdl->numPages << PAGE_SHIFT);

    if (Mdl->contiguous)
    {
        pages = tmpPages = kmalloc(sizeof(struct page*) * numPages, GFP_KERNEL | gcdNOWARN);
        if (!pages)
        {
            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }

        for (i = 0; i < numPages; ++i)
        {
            pages[i] = nth_page(mdlPriv->contiguousPages, i + skipPages);
        }
    }
    else
    {
        pages = &mdlPriv->nonContiguousPages[skipPages];
    }

    sgt = kmalloc(sizeof(struct sg_table), GFP_KERNEL | gcdNOWARN);
    if (!sgt)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    if (sg_alloc_table_from_pages(sgt, pages, numPages, offset, Bytes, GFP_KERNEL) < 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    *SGT = (gctPOINTER)sgt;

OnError:
    if (tmpPages)
    {
        kfree(tmpPages);
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
_GFPFree(
    IN gckALLOCATOR Allocator,
    IN OUT PLINUX_MDL Mdl
    )
{
    gctINT i;
    struct page * page;
    struct gfp_alloc *priv = (struct gfp_alloc *)Allocator->privateData;
    struct gfp_mdl_priv *mdlPriv = Mdl->priv;
    int low  = 0;
    int high = 0;

    if (Mdl->contiguous)
    {
        dma_unmap_page(galcore_device, mdlPriv->dma_addr,
                Mdl->numPages << PAGE_SHIFT, DMA_TO_DEVICE);
    }
    else
    {
        dma_unmap_sg(galcore_device, mdlPriv->sgt.sgl, mdlPriv->sgt.nents,
                DMA_TO_DEVICE);

#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0) \
    && (defined (ARCH_HAS_SG_CHAIN) || defined (CONFIG_ARCH_HAS_SG_CHAIN))
        sg_free_table(&mdlPriv->sgt);
#else
        kfree(mdlPriv->sgt.sgl);
#endif
    }

    for (i = 0; i < Mdl->numPages; i++)
    {
        if (Mdl->contiguous)
        {
            page = nth_page(mdlPriv->contiguousPages, i);
        }
        else
        {
            page = mdlPriv->nonContiguousPages[i];
        }

        ClearPageReserved(page);

        if (PageHighMem(page))
        {
            high++;
        }
        else
        {
            low++;
        }
    }

    atomic_sub(low, &priv->low);
    atomic_sub(high, &priv->high);

    if (Mdl->contiguous)
    {
#if defined(CONFIG_X86)
        if (!PageHighMem(mdlPriv->contiguousPages))
        {
            set_memory_wb((unsigned long)page_address(mdlPriv->contiguousPages), Mdl->numPages);
        }
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
        if (mdlPriv->exact == gcvTRUE)
        {
            free_pages_exact(page_address(mdlPriv->contiguousPages), Mdl->numPages * PAGE_SIZE);
        }
        else
#endif
        {
            __free_pages(mdlPriv->contiguousPages, get_order(Mdl->numPages * PAGE_SIZE));
        }
    }
    else
    {
#if defined(CONFIG_X86)
        set_pages_array_wb(mdlPriv->nonContiguousPages, Mdl->numPages);
#endif

        _NonContiguousFree(mdlPriv->nonContiguousPages, Mdl->numPages);
    }

    kfree(Mdl->priv);
}

static gceSTATUS
_GFPMmap(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctBOOL Cacheable,
    IN gctSIZE_T skipPages,
    IN gctSIZE_T numPages,
    IN struct vm_area_struct *vma
    )
{
    struct gfp_mdl_priv *mdlPriv = (struct gfp_mdl_priv*)Mdl->priv;
    gcsPLATFORM *platform = mdlPriv->platform;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p vma=%p", Allocator, Mdl, vma);

    vma->vm_flags |= gcdVM_FLAGS;

    if (Cacheable == gcvFALSE)
    {
        vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
    }

    if (platform && platform->ops->adjustProt)
    {
        platform->ops->adjustProt(vma);
    }

    gcmkASSERT(skipPages + numPages <= Mdl->numPages);

    /* Now map all the vmalloc pages to this user address. */
    if (mdlPriv->contiguous)
    {
        /* map kernel memory to user space.. */
        if (remap_pfn_range(vma,
                            vma->vm_start,
                            page_to_pfn(mdlPriv->contiguousPages) + skipPages,
                            numPages << PAGE_SHIFT,
                            vma->vm_page_prot) < 0)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_INFO, gcvZONE_OS,
                "%s(%d): remap_pfn_range error.",
                __FUNCTION__, __LINE__
                );

            gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
        }
    }
    else
    {
        gctUINT i;
        unsigned long start = vma->vm_start;

        for (i = 0; i < numPages; ++i)
        {
            unsigned long pfn = page_to_pfn(mdlPriv->nonContiguousPages[i + skipPages]);

            if (remap_pfn_range(vma,
                                start,
                                pfn,
                                PAGE_SIZE,
                                vma->vm_page_prot) < 0)
            {
                gcmkTRACE(
                    gcvLEVEL_ERROR,
                    "%s(%d): remap_pfn_range error.",
                    __FUNCTION__, __LINE__
                    );

                gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
            }

            start += PAGE_SIZE;
        }
    }

OnError:
    gcmkFOOTER();
    return status;
}

static void
_GFPUnmapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctUINT32 Size
    )
{
    MdlMap->cacheable = gcvFALSE;

    if (unlikely(current->mm == gcvNULL))
    {
        /* Do nothing if process is exiting. */
        return;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,5,0)
    if (vm_munmap((unsigned long)MdlMap->vmaAddr, Size) < 0)
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): vm_munmap failed",
                __FUNCTION__, __LINE__
                );
    }
#else
    down_write(&current->mm->mmap_sem);
    if (do_munmap(current->mm, (unsigned long)MdlMap->vmaAddr, Size) < 0)
    {
        gcmkTRACE_ZONE(
                gcvLEVEL_WARNING, gcvZONE_OS,
                "%s(%d): do_munmap failed",
                __FUNCTION__, __LINE__
                );
    }
    up_write(&current->mm->mmap_sem);
#endif

    MdlMap->vma = NULL;
}

static gceSTATUS
_GFPMapUser(
    gckALLOCATOR Allocator,
    PLINUX_MDL Mdl,
    PLINUX_MDL_MAP MdlMap,
    gctBOOL Cacheable
    )
{
    gctPOINTER userLogical = gcvNULL;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Allocator=%p Mdl=%p Cacheable=%d", Allocator, Mdl, Cacheable);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
    userLogical = (gctPOINTER)vm_mmap(NULL,
                    0L,
                    Mdl->numPages * PAGE_SIZE,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED | MAP_NORESERVE,
                    0);
#else
    down_write(&current->mm->mmap_sem);
    userLogical = (gctPOINTER)do_mmap_pgoff(NULL,
                    0L,
                    Mdl->numPages * PAGE_SIZE,
                    PROT_READ | PROT_WRITE,
                    MAP_SHARED,
                    0);
    up_write(&current->mm->mmap_sem);
#endif

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, gcvZONE_OS,
        "%s(%d): vmaAddr->%p for phys_addr->%p",
        __FUNCTION__, __LINE__,
        userLogical,
        Mdl
        );

    if (IS_ERR(userLogical))
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_INFO, gcvZONE_OS,
            "%s(%d): do_mmap_pgoff error",
            __FUNCTION__, __LINE__
            );

        userLogical = gcvNULL;

        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    down_write(&current->mm->mmap_sem);
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

        gcmkERR_BREAK(_GFPMmap(Allocator, Mdl, Cacheable, 0, Mdl->numPages, vma));
        MdlMap->vma = vma;
    }
    while (gcvFALSE);
    up_write(&current->mm->mmap_sem);

    if (gcmIS_SUCCESS(status))
    {
        MdlMap->vmaAddr = userLogical;
        MdlMap->cacheable = Cacheable;
    }

OnError:
    if (gcmIS_ERROR(status) && userLogical)
    {
        _GFPUnmapUser(Allocator, Mdl, userLogical, Mdl->numPages * PAGE_SIZE);
    }
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_GFPMapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    OUT gctPOINTER *Logical
    )
{
    void *addr = 0;
    gctINT numPages = Mdl->numPages;
    struct gfp_mdl_priv *mdlPriv = Mdl->priv;

#if gcdNONPAGED_MEMORY_CACHEABLE
    if (Mdl->contiguous)
    {
        addr = page_address(mdlPriv->contiguousPages);
    }
    else
    {
        addr = vmap(mdlPriv->nonContiguousPages,
                    numPages,
                    0,
                    PAGE_KERNEL);

        /* Trigger a page fault. */
        memset(addr, 0, numPages * PAGE_SIZE);
    }
#else
    struct page ** pages;
    gctBOOL free = gcvFALSE;
    gctINT i;

    if (Mdl->contiguous)
    {
        pages = kmalloc(sizeof(struct page *) * numPages, GFP_KERNEL | gcdNOWARN);

        if (!pages)
        {
            return gcvSTATUS_OUT_OF_MEMORY;
        }

        for (i = 0; i < numPages; i++)
        {
            pages[i] = nth_page(mdlPriv->contiguousPages, i);
        }

        free = gcvTRUE;
    }
    else
    {
        pages = mdlPriv->nonContiguousPages;
    }

    addr = vmap(pages, numPages, 0, pgprot_writecombine(PAGE_KERNEL));

    if (free)
    {
        kfree(pages);
    }
#endif

    if (addr)
    {
        *Logical = addr;
        return gcvSTATUS_OK;
    }
    else
    {
        return gcvSTATUS_OUT_OF_MEMORY;
    }
}

static gceSTATUS
_GFPUnmapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{

#if !gcdNONPAGED_MEMORY_CACHEABLE
    vunmap(Logical);
#endif

    return gcvSTATUS_OK;
}

static gceSTATUS
_GFPCache(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical,
    IN gctUINT32 Physical,
    IN gctUINT32 Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    struct gfp_mdl_priv *mdlPriv = Mdl->priv;
    enum dma_data_direction dir;

    switch (Operation)
    {
    case gcvCACHE_CLEAN:
        dir = DMA_TO_DEVICE;

        if (mdlPriv->contiguous)
        {
            dma_sync_single_for_device(galcore_device,
                    mdlPriv->dma_addr, Mdl->numPages << PAGE_SHIFT, dir);
        }
        else
        {
            dma_sync_sg_for_device(galcore_device,
                    mdlPriv->sgt.sgl, mdlPriv->sgt.nents, dir);
        }

        break;
    case gcvCACHE_FLUSH:
        dir = DMA_BIDIRECTIONAL;

        if (mdlPriv->contiguous)
        {
            dma_sync_single_for_device(galcore_device,
                    mdlPriv->dma_addr, Mdl->numPages << PAGE_SHIFT, dir);
        }
        else
        {
            dma_sync_sg_for_device(galcore_device,
                    mdlPriv->sgt.sgl, mdlPriv->sgt.nents, dir);
        }

        break;
    case gcvCACHE_INVALIDATE:
        dir = DMA_FROM_DEVICE;

        if (mdlPriv->contiguous)
        {
            dma_sync_single_for_cpu(galcore_device,
                    mdlPriv->dma_addr, Mdl->numPages << PAGE_SHIFT, dir);
        }
        else
        {
            dma_sync_sg_for_cpu(galcore_device,
                    mdlPriv->sgt.sgl, mdlPriv->sgt.nents, dir);
        }

        break;
    default:
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    return gcvSTATUS_OK;
}

static gceSTATUS
_GFPPhysical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    struct gfp_mdl_priv *mdlPriv = Mdl->priv;
    gctUINT32 offsetInPage = Offset & ~PAGE_MASK;
    gctUINT32 index = Offset / PAGE_SIZE;

    if (Mdl->contiguous)
    {
        *Physical = page_to_phys(nth_page(mdlPriv->contiguousPages, index));
    }
    else
    {
        *Physical = page_to_phys(mdlPriv->nonContiguousPages[index]);
    }

    *Physical += offsetInPage;

    return gcvSTATUS_OK;
}

static void
_GFPAllocatorDestructor(
    gcsALLOCATOR *Allocator
    )
{
    _GFPAllocatorDebugfsCleanup(Allocator);

    if (Allocator->privateData)
    {
        kfree(Allocator->privateData);
    }

    kfree(Allocator);
}

/* GFP allocator operations. */
static gcsALLOCATOR_OPERATIONS GFPAllocatorOperations = {
    .Alloc              = _GFPAlloc,
    .Free               = _GFPFree,
    .Mmap               = _GFPMmap,
    .MapUser            = _GFPMapUser,
    .UnmapUser          = _GFPUnmapUser,
    .MapKernel          = _GFPMapKernel,
    .UnmapKernel        = _GFPUnmapKernel,
    .Cache              = _GFPCache,
    .Physical           = _GFPPhysical,
    .GetSGT             = _GFPGetSGT,
};

/* GFP allocator entry. */
gceSTATUS
_GFPAlloctorInit(
    IN gckOS Os,
    IN gcsDEBUGFS_DIR *Parent,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator = gcvNULL;
    struct gfp_alloc *priv = gcvNULL;

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &GFPAllocatorOperations, &allocator));

    priv = kzalloc(sizeof(struct gfp_alloc), GFP_KERNEL | gcdNOWARN);

    if (!priv)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    atomic_set(&priv->low,  0);
    atomic_set(&priv->high, 0);

    /* Register private data. */
    allocator->privateData = priv;
    allocator->destructor = _GFPAllocatorDestructor;

    _GFPAllocatorDebugfsInit(allocator, Parent);

    allocator->capability = gcvALLOC_FLAG_CONTIGUOUS
                          | gcvALLOC_FLAG_NON_CONTIGUOUS
                          | gcvALLOC_FLAG_CACHEABLE
                          | gcvALLOC_FLAG_MEMLIMIT
                          | gcvALLOC_FLAG_ALLOC_ON_FAULT
                          | gcvALLOC_FLAG_DMABUF_EXPORTABLE
#if defined(CONFIG_ZONE_DMA32) && LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
                          | gcvALLOC_FLAG_4GB_ADDR
#endif
                          ;

#if defined(gcdEMULATE_SECURE_ALLOCATOR)
    allocator->capability |= gcvALLOC_FLAG_SECURITY;
#endif

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    if (allocator)
    {
        kfree(allocator);
    }
    return status;
}

