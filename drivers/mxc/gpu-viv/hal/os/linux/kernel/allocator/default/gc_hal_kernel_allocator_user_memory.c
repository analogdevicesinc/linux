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
#include <linux/scatterlist.h>

#include <linux/slab.h>
#include <linux/pagemap.h>

#define _GC_OBJ_ZONE gcvZONE_ALLOCATOR

enum um_desc_type
{
    UM_PHYSICAL_MAP,
    UM_PAGE_MAP,
    UM_PFN_MAP,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION (2,6,24)
struct sg_table
{
    struct scatterlist *sgl;
    unsigned int nents;
    unsigned int orig_nents;
};
#endif

/* Descriptor of a user memory imported. */
struct um_desc
{
    int type;

    union
    {
        /* UM_PHYSICAL_MAP. */
        unsigned long physical;

        /* UM_PAGE_MAP. */
        struct
        {
            struct page **pages;
            struct sg_table sgt;
        };

        /* UM_PFN_MAP. */
        struct
        {
            unsigned long *pfns;
            int *refs;
        };
    };

    /* contiguous chunks, does not include padding pages. */
    int chunk_count;

    unsigned long vm_flags;
    unsigned long user_vaddr;
    size_t size;
    unsigned long offset;

    size_t pageCount;
    size_t extraPage;
};

static int import_physical_map(struct um_desc *um, unsigned long phys)
{
    um->type = UM_PHYSICAL_MAP;
    um->physical = phys & PAGE_MASK;
    um->chunk_count = 1;
    return 0;
}

static int import_page_map(struct um_desc *um,
                unsigned long addr, size_t page_count, size_t size)
{
    int i;
    int result;
    struct page **pages;

    pages = kzalloc(page_count * sizeof(void *), GFP_KERNEL | gcdNOWARN);
    if (!pages)
        return -ENOMEM;

    down_read(&current->mm->mmap_sem);

    result = get_user_pages(
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
            current,
            current->mm,
#endif
            addr & PAGE_MASK,
            page_count,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
            FOLL_WRITE,
#else
            1,
            0,
#endif
            pages,
            NULL);

    up_read(&current->mm->mmap_sem);

    if (result < page_count)
    {
        for (i = 0; i < result; i++)
        {
            if (pages[i])
            {
                put_page(pages[i]);
            }
        }

        kfree(pages);
        return -ENODEV;
    }

    um->chunk_count = 1;
    for (i = 1; i < page_count; i++)
    {
        if (page_to_pfn(pages[i]) != page_to_pfn(pages[i - 1]) + 1)
        {
            ++um->chunk_count;
        }
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0) \
    && (defined(ARCH_HAS_SG_CHAIN) || defined(CONFIG_ARCH_HAS_SG_CHAIN))
    result = sg_alloc_table_from_pages(&um->sgt, pages, page_count,
                    addr & ~PAGE_MASK, size, GFP_KERNEL | gcdNOWARN);

#else
    result = alloc_sg_list_from_pages(&um->sgt.sgl, pages, page_count,
                    addr & ~PAGE_MASK, size, &um->sgt.nents);

    um->sgt.orig_nents = um->sgt.nents;
#endif
    if (unlikely(result < 0))
    {
        printk("[galcore]: %s: sg_alloc_table_from_pages failed\n", __FUNCTION__);
        goto error;
    }

    result = dma_map_sg(galcore_device, um->sgt.sgl, um->sgt.nents, DMA_TO_DEVICE);
    if (unlikely(result != um->sgt.nents))
    {
        printk("[galcore]: %s: dma_map_sg failed\n", __FUNCTION__);
        goto error;
    }

    um->type = UM_PAGE_MAP;
    um->pages = pages;

    return 0;

error:
#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0) \
    && (defined(ARCH_HAS_SG_CHAIN) || defined(CONFIG_ARCH_HAS_SG_CHAIN))
    sg_free_table(&um->sgt);
#else
    kfree(um->sgt.sgl);
#endif

    if (um->pages)
    {
        kfree(um->pages);
    }
    return result;
}


static int import_pfn_map(struct um_desc *um,
                unsigned long addr, size_t pfn_count)
{
    int i;
    struct vm_area_struct *vma;
    unsigned long *pfns;
    int *refs;

    if (!current->mm)
        return -ENOTTY;

    down_read(&current->mm->mmap_sem);
    vma = find_vma(current->mm, addr);
    up_read(&current->mm->mmap_sem);

    if (!vma)
        return -ENOTTY;

    pfns = kzalloc(pfn_count * sizeof(unsigned long), GFP_KERNEL | gcdNOWARN);

    if (!pfns)
        return -ENOMEM;

    refs = kzalloc(pfn_count * sizeof(int), GFP_KERNEL | gcdNOWARN);

    if (!refs)
    {
        kfree(pfns);
        return -ENOMEM;
    }

    for (i = 0; i < pfn_count; i++)
    {
        spinlock_t *ptl;
        pgd_t *pgd;
        pud_t *pud;
        pmd_t *pmd;
        pte_t *pte;

        pgd = pgd_offset(current->mm, addr);
        if (pgd_none(*pgd) || pgd_bad(*pgd))
            goto err;

        pud = pud_offset(pgd, addr);
        if (pud_none(*pud) || pud_bad(*pud))
            goto err;

        pmd = pmd_offset(pud, addr);
        if (pmd_none(*pmd) || pmd_bad(*pmd))
            goto err;

        pte = pte_offset_map_lock(current->mm, pmd, addr, &ptl);
        if (!pte)
        {
            spin_unlock(ptl);
            goto err;
        }

        if (!pte_present(*pte))
        {
            pte_unmap_unlock(pte, ptl);
            goto err;
        }

        pfns[i] = pte_pfn(*pte);
        pte_unmap_unlock(pte, ptl);

        /* Advance to next. */
        addr += PAGE_SIZE;
    }

    for (i = 0; i < pfn_count; i++)
    {
        if (pfn_valid(pfns[i]))
        {
            struct page *page = pfn_to_page(pfns[i]);
            refs[i] = get_page_unless_zero(page);
        }
    }

    um->chunk_count = 1;
    for (i = 1; i < pfn_count; i++)
    {
        if (pfns[i] != pfns[i - 1] + 1)
        {
            ++um->chunk_count;
        }
    }

    um->type = UM_PFN_MAP;
    um->pfns = pfns;
    um->refs = refs;
    return 0;

err:
    if (pfns)
        kfree(pfns);

    if (refs)
        kfree(refs);

    return -ENOTTY;
}

static gceSTATUS
_Import(
    IN gckOS Os,
    IN gctPOINTER Memory,
    IN gctUINT32 Physical,
    IN gctSIZE_T Size,
    IN struct um_desc * UserMemory
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    unsigned long vm_flags = 0;
    struct vm_area_struct *vma = NULL;
    unsigned long start, end, memory;
    int result = 0;

    gctSIZE_T extraPage;
    gctSIZE_T pageCount, i;

    gcmkHEADER_ARG("Os=0x%p Memory=%p Physical=0x%x Size=%lu", Os, Memory, Physical, Size);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Memory != gcvNULL || Physical != ~0U);
    gcmkVERIFY_ARGUMENT(Size > 0);

    memory = (unsigned long)Memory;

    /* Get the number of required pages. */
    end = (memory + Size + PAGE_SIZE - 1) >> PAGE_SHIFT;
    start = memory >> PAGE_SHIFT;
    pageCount = end - start;

    /* Allocate extra page to avoid cache overflow */
#if gcdENABLE_2D
    extraPage = 2;
#else
    extraPage = (((memory + gcmALIGN(Size + 64, 64) + PAGE_SIZE - 1) >> PAGE_SHIFT) > end) ? 1 : 0;
#endif

    gcmkTRACE_ZONE(
        gcvLEVEL_INFO, _GC_OBJ_ZONE,
        "%s(%d): pageCount: %d. extraPage: %d",
        __FUNCTION__, __LINE__,
        pageCount, extraPage
        );

    /* Overflow. */
    if ((memory + Size) < memory)
    {
        gcmkFOOTER_ARG("status=%d", gcvSTATUS_INVALID_ARGUMENT);
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    if (memory)
    {
        unsigned long vaddr = memory;

        for (i = 0; i < pageCount; i++)
        {
            u32 data;

            get_user(data, (u32 *)vaddr);
            put_user(data, (u32 *)vaddr);
            vaddr += PAGE_SIZE;

            /* Fix QM crash with test_buffers */
            if (vaddr > memory + Size - 4)
            {
                vaddr = memory + Size - 4;
            }
        }

        vma = find_vma(current->mm, memory);

        if (!vma)
        {
            /* No such memory, or across vmas. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

#ifdef CONFIG_ARM
        /* coherent cache in case vivt or vipt-aliasing cache. */
        __cpuc_flush_user_range(memory, memory + Size, vma->vm_flags);
#endif

        vm_flags = vma->vm_flags;
        vaddr = vma->vm_end;

        while (vaddr < memory + Size)
        {
            vma = find_vma(current->mm, vaddr);

            if (!vma)
            {
                /* No such memory. */
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
            }

            if ((vma->vm_flags & VM_PFNMAP) != (vm_flags & VM_PFNMAP))
            {
                /* Can not support different map type: both PFN and PAGE detected. */
                gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
            }

            vaddr = vma->vm_end;
        }
    }

    if (Physical != gcvINVALID_PHYSICAL_ADDRESS)
    {
        result = import_physical_map(UserMemory, Physical);
    }
    else
    {
        if (vm_flags & VM_PFNMAP)
        {
            result = import_pfn_map(UserMemory, memory, pageCount);
        }
        else
        {
            result = import_page_map(UserMemory, memory, pageCount, Size);
        }
    }

    if (result == -EINVAL)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }
    else if (result == -ENOMEM)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }
    else if (result < 0)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    UserMemory->vm_flags = vm_flags;
    UserMemory->user_vaddr = (unsigned long)Memory;
    UserMemory->size  = Size;
    UserMemory->offset = (Physical != gcvINVALID_PHYSICAL_ADDRESS)
                       ? (Physical & ~PAGE_MASK)
                       : (memory & ~PAGE_MASK);

    UserMemory->pageCount = pageCount;
    UserMemory->extraPage = extraPage;

    if (extraPage && UserMemory->type == UM_PAGE_MAP)
    {
        /*Add the padding pages */
        UserMemory->chunk_count++;
    }

    /* Success. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_UserMemoryAttach(
    IN gckALLOCATOR Allocator,
    IN gcsATTACH_DESC_PTR Desc,
    IN PLINUX_MDL Mdl
    )
{
    gceSTATUS status;
    struct um_desc * userMemory = gcvNULL;

    gckOS os = Allocator->os;

    gcmkHEADER();

    /* Handle is meangless for this importer. */
    gcmkVERIFY_ARGUMENT(Desc != gcvNULL);

    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(struct um_desc), (gctPOINTER *)&userMemory));

    gckOS_ZeroMemory(userMemory, gcmSIZEOF(struct um_desc));

    gcmkONERROR(_Import(os, Desc->userMem.memory, Desc->userMem.physical, Desc->userMem.size, userMemory));

    Mdl->priv = userMemory;
    Mdl->numPages = userMemory->pageCount + userMemory->extraPage;
    Mdl->contiguous = (userMemory->chunk_count == 1);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (userMemory != gcvNULL)
    {
        gckOS_Free(os,(gctPOINTER)userMemory);
    }
    gcmkFOOTER();
    return status;
}

static void release_physical_map(struct um_desc *um)
{
}

static void release_page_map(struct um_desc *um)
{
    int i;

    dma_unmap_sg(galcore_device, um->sgt.sgl, um->sgt.nents, DMA_TO_DEVICE);

#if LINUX_VERSION_CODE >= KERNEL_VERSION (3,6,0) \
    && (defined(ARCH_HAS_SG_CHAIN) || defined(CONFIG_ARCH_HAS_SG_CHAIN))
    sg_free_table(&um->sgt);
#else
    kfree(um->sgt.sgl);
#endif

    for (i = 0; i < um->pageCount; i++)
    {
        if (!PageReserved(um->pages[i]))
        {
             SetPageDirty(um->pages[i]);
        }

        put_page(um->pages[i]);
    }

    kfree(um->pages);
}

static void release_pfn_map(struct um_desc *um)
{

    int i;

    for (i = 0; i < um->pageCount; i++)
    {
        if (pfn_valid(um->pfns[i]))
        {
            struct page *page = pfn_to_page(um->pfns[i]);
            if (!PageReserved(page))
            {
                SetPageDirty(page);
            }

            if (um->refs[i])
            {
                put_page(page);
            }
        }
    }

    kfree(um->pfns);
    kfree(um->refs);
}

static void
_UserMemoryFree(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl
    )
{
    gckOS os = Allocator->os;
    struct um_desc *userMemory = Mdl->priv;

    gcmkHEADER();

    if (userMemory)
    {
        switch (userMemory->type)
        {
        case UM_PHYSICAL_MAP:
            release_physical_map(userMemory);
            break;
        case UM_PAGE_MAP:
            release_page_map(userMemory);
            break;
        case UM_PFN_MAP:
            release_pfn_map(userMemory);
            break;
        }

        gcmkOS_SAFE_FREE(os, userMemory);
    }

    gcmkFOOTER_NO();
}

static gceSTATUS
_UserMemoryMapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctBOOL Cacheable
    )
{
    struct um_desc *userMemory = Mdl->priv;

    MdlMap->vmaAddr = (gctPOINTER)userMemory->user_vaddr;
    MdlMap->cacheable = gcvTRUE;

    return gcvSTATUS_OK;
}

static void
_UserMemoryUnmapUser(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN PLINUX_MDL_MAP MdlMap,
    IN gctUINT32 Size
    )
{
    return;
}

static gceSTATUS
_UserMemoryMapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    OUT gctPOINTER *Logical
    )
{
    /* Kernel doesn't acess video memory. */
    return gcvSTATUS_NOT_SUPPORTED;
}

static gceSTATUS
_UserMemoryUnmapKernel(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical
    )
{
    /* Kernel doesn't acess video memory. */
    return gcvSTATUS_NOT_SUPPORTED;
}

static gceSTATUS
_UserMemoryCache(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctPOINTER Logical,
    IN gctUINT32 Physical,
    IN gctUINT32 Bytes,
    IN gceCACHEOPERATION Operation
    )
{
    struct um_desc *um = Mdl->priv;
    enum dma_data_direction dir;

    if (um->type != UM_PAGE_MAP)
    {
        _MemoryBarrier();
        return gcvSTATUS_OK;
    }

#ifdef CONFIG_ARM
    /* coherent cache in case vivt or vipt-aliasing cache. */
    __cpuc_flush_user_range(um->user_vaddr,
                            um->user_vaddr + um->size, um->vm_flags);
#endif

    switch (Operation)
    {
    case gcvCACHE_CLEAN:
        dir = DMA_TO_DEVICE;
        dma_sync_sg_for_device(galcore_device, um->sgt.sgl, um->sgt.nents, dir);
        break;
    case gcvCACHE_FLUSH:
        dir = DMA_BIDIRECTIONAL;
        dma_sync_sg_for_device(galcore_device, um->sgt.sgl, um->sgt.nents, dir);
        break;
    case gcvCACHE_INVALIDATE:
        dir = DMA_FROM_DEVICE;
        dma_sync_sg_for_cpu(galcore_device, um->sgt.sgl, um->sgt.nents, dir);
        break;
    default:
        return gcvSTATUS_INVALID_ARGUMENT;
    }


    return gcvSTATUS_OK;
}

static gceSTATUS
_UserMemoryPhysical(
    IN gckALLOCATOR Allocator,
    IN PLINUX_MDL Mdl,
    IN gctUINT32 Offset,
    OUT gctPHYS_ADDR_T * Physical
    )
{
    gckOS os = Allocator->os;
    struct um_desc *userMemory = Mdl->priv;
    unsigned long offset = Offset + userMemory->offset;
    gctUINT32 offsetInPage = offset & ~PAGE_MASK;
    gctUINT32 index = offset / PAGE_SIZE;

    if (index >= userMemory->pageCount)
    {
        if (index < userMemory->pageCount + userMemory->extraPage)
        {
            *Physical = page_to_phys(os->paddingPage);
        }
        else
        {
            return gcvSTATUS_INVALID_ARGUMENT;
        }
    }
    else
    {
        switch (userMemory->type)
        {
        case UM_PHYSICAL_MAP:
            *Physical = userMemory->physical + index * PAGE_SIZE;
            break;
        case UM_PAGE_MAP:
            *Physical = page_to_phys(userMemory->pages[index]);
            break;
        case UM_PFN_MAP:
            *Physical = userMemory->pfns[index] << PAGE_SHIFT;
            break;
        }
    }

    *Physical += offsetInPage;

    return gcvSTATUS_OK;
}

static void
_UserMemoryAllocatorDestructor(
    gcsALLOCATOR *Allocator
    )
{
    if (Allocator->privateData)
    {
        kfree(Allocator->privateData);
    }

    kfree(Allocator);
}

/* User memory allocator (importer) operations. */
static gcsALLOCATOR_OPERATIONS UserMemoryAllocatorOperations =
{
    .Attach             = _UserMemoryAttach,
    .Free               = _UserMemoryFree,
    .MapUser            = _UserMemoryMapUser,
    .UnmapUser          = _UserMemoryUnmapUser,
    .MapKernel          = _UserMemoryMapKernel,
    .UnmapKernel        = _UserMemoryUnmapKernel,
    .Cache              = _UserMemoryCache,
    .Physical           = _UserMemoryPhysical,
};

/* Default allocator entry. */
gceSTATUS
_UserMemoryAlloctorInit(
    IN gckOS Os,
    IN gcsDEBUGFS_DIR *Parent,
    OUT gckALLOCATOR * Allocator
    )
{
    gceSTATUS status;
    gckALLOCATOR allocator;

    gcmkONERROR(
        gckALLOCATOR_Construct(Os, &UserMemoryAllocatorOperations, &allocator));

    allocator->destructor  = _UserMemoryAllocatorDestructor;

    allocator->capability = gcvALLOC_FLAG_USERMEMORY;

    *Allocator = allocator;

    return gcvSTATUS_OK;

OnError:
    return status;
}

