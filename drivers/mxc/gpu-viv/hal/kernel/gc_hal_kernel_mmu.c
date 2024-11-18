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


#include "gc_hal_kernel_precomp.h"

#define _GC_OBJ_ZONE gcvZONE_MMU

static gceSTATUS
_FillPageTable(IN gctUINT32_PTR PageTable, IN gctUINT32 PageCount, IN gctUINT32 EntryValue)
{
    gctUINT i;

    for (i = 0; i < PageCount; i++)
        _WritePageEntry(PageTable + i, EntryValue);

    return gcvSTATUS_OK;
}

static gceSTATUS
_FillMap(IN gctUINT32_PTR Map, IN gctUINT32 PageCount, IN gctUINT32 EntryValue)
{
    gctUINT i;

    for (i = 0; i < PageCount; i++)
        Map[i] = EntryValue;

    return gcvSTATUS_OK;
}

static gceSTATUS
_Link(IN gcsADDRESS_AREA_PTR Area, IN gctUINT32 Index, IN gctUINT32 Next)
{
    if (Index >= Area->stlbEntries) {
        /* Just move heap pointer. */
        Area->heapList = Next;
    } else {
        /* Address page table. */
        gctUINT32_PTR map = Area->mapLogical;

        /* Dispatch on node type. */
        switch (gcmENTRY_TYPE(map[Index])) {
        case gcvMMU_SINGLE:
            /* Set single index. */
            map[Index] = (Next << 8) | gcvMMU_SINGLE;
            break;

        case gcvMMU_FREE:
            /* Set index. */
            map[Index + 1] = Next;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", Index);
            return gcvSTATUS_HEAP_CORRUPTED;
        }
    }

    /* Success. */
    return gcvSTATUS_OK;
}

static gceSTATUS
_AddFree(IN gcsADDRESS_AREA_PTR Area, IN gctUINT32 Index, IN gctUINT32 Node, IN gctUINT32 Count)
{
    gctUINT32_PTR map = Area->mapLogical;

    if (Count == 1) {
        /* Initialize a single page node. */
        map[Node] = (~((1U << 8) - 1)) | gcvMMU_SINGLE;
    } else {
        /* Initialize the node. */
        map[Node + 0] = (Count << 8) | gcvMMU_FREE;
        map[Node + 1] = ~0U;
    }

    /* Append the node. */
    return _Link(Area, Index, Node);
}

static gceSTATUS
_Collect(IN gcsADDRESS_AREA_PTR Area)
{
    gctUINT32_PTR map = Area->mapLogical;
    gceSTATUS     status;
    gctUINT32     i, previous, start = 0, count = 0;

    previous       = ~0U;
    Area->heapList = ~0U;
    Area->freeNodes           = gcvFALSE;

    /* Walk the entire page table. */
    for (i = 0; i < Area->stlbEntries; ++i) {
        /* Dispatch based on type of page. */
        switch (gcmENTRY_TYPE(map[i])) {
        case gcvMMU_USED:
            /* Used page, so close any open node. */
            if (count > 0) {
                /* Add the node. */
                gcmkONERROR(_AddFree(Area, previous, start, count));

                /* Reset the node. */
                previous = start;
                count    = 0;
            }
            break;

        case gcvMMU_SINGLE:
            /* Single free node. */
            if (count++ == 0) {
                /* Start a new node. */
                start = i;
            }
            break;

        case gcvMMU_FREE:
            /* A free node. */
            if (count == 0) {
                /* Start a new node. */
                start = i;
            }

            /* Advance the count. */
            count += map[i] >> 8;

            /* Advance the index into the page table. */
            i += (map[i] >> 8) - 1;
            break;

        default:
            gcmkFATAL("MMU page table correcupted at index %u!", i);
            return gcvSTATUS_HEAP_CORRUPTED;
        }
    }

    /* See if we have an open node left. */
    if (count > 0) {
        /* Add the node to the list. */
        gcmkONERROR(_AddFree(Area, previous, start, count));
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_MMU, "Performed a garbage collection of the MMU heap.");

    /* Success. */
    return gcvSTATUS_OK;

OnError:
    /* Return the staus. */
    return status;
}

static gctUINT32
_SetPage(gctUINT32 PageAddress, gctUINT32 PageAddressExt, gctBOOL Writable)
{
    gctUINT32 entry = PageAddress
                      /* AddressExt */
                      | (PageAddressExt << 4)
                      /* Ignore exception */
                      | (0 << 1)
                      /* Present */
                      | (1 << 0);

    if (Writable) {
        /* writable */
        entry |= (1 << 2);
#if gcdUSE_MMU_EXCEPTION
    } else {
        /* If this page is read only, set exception bit to make exception happens
         * when writing to it.
         */
        entry |= gcdMMU_STLB_EXCEPTION;
    }
#else
    }
#endif

    return entry;
}

static gctUINT32
_MtlbOffset(gctADDRESS Address)
{
    return (gctUINT32)((Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT);
}

gctUINT32
_AddressToIndex(IN gcsADDRESS_AREA_PTR Area, IN gctADDRESS Address)
{
    gctUINT32 stlbShift = (Area->areaType == gcvAREA_TYPE_1M) ?
                          gcdMMU_STLB_1M_SHIFT :
                          gcdMMU_STLB_4K_SHIFT;

    gctUINT32 stlbMask = (Area->areaType == gcvAREA_TYPE_1M) ?
                         gcdMMU_STLB_1M_MASK :
                         gcdMMU_STLB_4K_MASK;

    gctUINT32 stlbEntryNum = (Area->areaType == gcvAREA_TYPE_1M) ?
                             gcdMMU_STLB_1M_ENTRY_NUM :
                             gcdMMU_STLB_4K_ENTRY_NUM;

    gctUINT32 mtlbOffset = (gctUINT32)((Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT);

    gctUINT32 stlbOffset = (gctUINT32)((Address & stlbMask) >> stlbShift);

    return (mtlbOffset - Area->mappingStart) * stlbEntryNum + stlbOffset;
}

static gctUINT32_PTR
_StlbEntry(gcsADDRESS_AREA_PTR Area, gctADDRESS Address)
{
    gctUINT32 index = _AddressToIndex(Area, Address);

    return &Area->stlbLogical[index];
}

static gctBOOL
_IsRangeInsected(gctUINT64 baseAddress1, gctSIZE_T size1,
                 gctUINT64 baseAddress2, gctSIZE_T size2)
{
    gctUINT64 endAddress1 = baseAddress1 + size1 - 1;
    gctUINT64 endAddress2 = baseAddress2 + size2 - 1;

    if (!size1 || !size2)
        return gcvFALSE;

    return (((baseAddress2 <= endAddress1) && (endAddress2 >= baseAddress1)) ||
            ((baseAddress1 <= endAddress2) && (endAddress1 >= baseAddress2)));
}

static gceSTATUS
_FillFlatMappingInMap(gcsADDRESS_AREA_PTR Area, gctUINT32 Index, gctUINT32 NumPages)
{
    gceSTATUS     status;
    gctUINT32     i;
    gctBOOL       gotIt    = gcvFALSE;
    gctUINT32     index    = Index;
    gctUINT32_PTR map      = Area->mapLogical;
    gctUINT32     previous = ~0U;

    /* Find node which contains index. */
    for (i = 0; !gotIt && (i < Area->stlbEntries);) {
        gctUINT32 numPages;

        switch (gcmENTRY_TYPE(map[i])) {
        case gcvMMU_SINGLE:
            if (i == index) {
                gotIt = gcvTRUE;
            } else {
                previous = i;
                i        = map[i] >> 8;
            }
            break;

        case gcvMMU_FREE:
            numPages = map[i] >> 8;
            if (index >= i && index + NumPages - 1 < i + numPages) {
                gotIt = gcvTRUE;
            } else {
                previous = i;
                i        = map[i + 1];
            }
            break;

        case gcvMMU_USED:
            i++;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", index);
            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }
    }

    switch (gcmENTRY_TYPE(map[i])) {
    case gcvMMU_SINGLE:
        /* Unlink single node from free list. */
        gcmkONERROR(_Link(Area, previous, map[i] >> 8));
        break;

    case gcvMMU_FREE:
        /* Split the node. */
        {
            gctUINT32 start;
            gctUINT32 next       = map[i + 1];
            gctUINT32 total      = map[i] >> 8;
            gctUINT32 countLeft  = index - i;
            gctUINT32 countRight = total - countLeft - NumPages;

            if (countLeft) {
                start = i;
                _AddFree(Area, previous, start, countLeft);
                previous = start;
            }

            if (countRight) {
                start = index + NumPages;
                _AddFree(Area, previous, start, countRight);
                previous = start;
            }

            _Link(Area, previous, next);
        }
        break;
    }

    _FillMap(&map[index], NumPages, gcvMMU_USED);

    return gcvSTATUS_OK;
OnError:
    return status;
}

static gceSTATUS
_CollectFreeSpace(IN gckMMU Mmu, OUT gcsFreeSpaceNode_PTR *Array, OUT gctINT *Size)
{
    gceSTATUS            status  = gcvSTATUS_OK;
    gctPOINTER           pointer = gcvNULL;
    gcsFreeSpaceNode_PTR array   = gcvNULL;
    gcsFreeSpaceNode_PTR node    = gcvNULL;
    gctINT               size    = 0;
    gctINT               i       = 0;

    for (i = 0; i < gcdMMU_MTLB_ENTRY_NUM; i++) {
        if (!Mmu->mtlbLogical[i]) {
            if (!node) {
                /* This is the first entry of the free space. */
                node += 1;
                size++;
            }
        } else if (node) {
            /* Reset the start. */
            node = gcvNULL;
        }
    }

    /* Allocate memory for the array. */
    gcmkONERROR(gckOS_Allocate(Mmu->os, gcmSIZEOF(*array) * size, &pointer));

    array = (gcsFreeSpaceNode_PTR)pointer;
    node  = gcvNULL;

    for (i = 0, size = 0; i < gcdMMU_MTLB_ENTRY_NUM; i++) {
        if (!Mmu->mtlbLogical[i]) {
            if (!node) {
                /* This is the first entry of the free space. */
                node = &array[size++];

                node->start   = i;
                node->entries = 0;
            }

            node->entries++;
        } else if (node) {
            /* Reset the start. */
            node = gcvNULL;
        }
    }

#if gcdMMU_TABLE_DUMP
    for (i = 0; i < size; i++) {
        gckOS_Print("%s(%d): [%d]: start=%d, entries=%d.\n",
                    __FUNCTION__, __LINE__, i,
                    array[i].start, array[i].entries);
    }
#endif

    *Array = array;
    *Size  = size;

    return gcvSTATUS_OK;

OnError:
    if (pointer != gcvNULL)
        gckOS_Free(Mmu->os, pointer);

    return status;
}

gceSTATUS
_GetMtlbFreeSpace(IN gckMMU Mmu, IN gctUINT32 NumEntries,
                  OUT gctUINT32 *MtlbStart, OUT gctUINT32 *MtlbEnd)
{
    gceSTATUS            status     = gcvSTATUS_OK;
    gcsFreeSpaceNode_PTR nodeArray  = gcvNULL;
    gctINT               i, nodeArraySize = 0;
    gctUINT              numEntries = gcdMMU_MTLB_ENTRY_NUM;
    gctINT32             mStart     = -1;
    gctINT32             mEnd       = -1;

    gcmkONERROR(_CollectFreeSpace(Mmu, &nodeArray, &nodeArraySize));

    /* Find the smallest space for NumEntries */
    for (i = 0; i < nodeArraySize; i++) {
        if (nodeArray[i].entries <= numEntries && NumEntries <= nodeArray[i].entries) {
            numEntries = nodeArray[i].entries;

            mStart = nodeArray[i].start;
            mEnd   = nodeArray[i].start + NumEntries - 1;
        }
    }

    if (mStart == -1 && mEnd == -1)
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);

    *MtlbStart = (gctUINT32)mStart;
    *MtlbEnd   = (gctUINT32)mEnd;

OnError:
    if (nodeArray)
        gckOS_Free(Mmu->os, (gctPOINTER)nodeArray);

    return status;
}

static gcePOOL
_GetPageTablePool(IN gckOS Os)
{
    gcePOOL   pool = gcvPOOL_DEFAULT;
    gctUINT64 data = 0;
    gceSTATUS status;

    status = gckOS_QueryOption(Os, "mmuPageTablePool", &data);

    if (status == gcvSTATUS_OK && data == 1) {
#if USE_LINUX_PCIE
        gctUINT64 externalSize = 0;

        status = gckOS_QueryOption(Os, "externalSize", &externalSize);
        if (status == gcvSTATUS_OK && externalSize != 0)
            pool = gcvPOOL_LOCAL_EXTERNAL;
        else
            pool = gcvPOOL_VIRTUAL;
#else
        pool = gcvPOOL_VIRTUAL;
#endif
    }

    return pool;
}

static gceSTATUS
_GetCurStlbChunk(IN gckMMU Mmu, IN gctUINT32 MtlbIndex,
                 OUT gcsMMU_STLB_CHUNK_PTR *StlbChunk)
{
    gcsMMU_STLB_CHUNK_PTR curStlbChunk = (gcsMMU_STLB_CHUNK_PTR)Mmu->staticSTLB;

    while (curStlbChunk) {
        if (MtlbIndex >= curStlbChunk->mtlbIndex &&
            (MtlbIndex < (curStlbChunk->mtlbIndex + curStlbChunk->mtlbEntryNum))) {
            break;
        }
        curStlbChunk = curStlbChunk->next;
    }

    *StlbChunk = curStlbChunk;

    return gcvSTATUS_OK;
}

static gceSTATUS
gckMMU_FillFlatMappingWithPage16M(IN gckMMU Mmu, IN gctUINT64 PhysBase, IN gctSIZE_T flatSize,
                                  IN gctBOOL reserved, IN gctBOOL needShiftMapping,
                                  IN gctBOOL specificFlatMapping, IN gctADDRESS reqVirtualBase,
                                  OUT gctADDRESS *GpuBaseAddress)
{
    gceSTATUS status;
    gckKERNEL kernel      = Mmu->hardware->kernel;
    gctBOOL   mutex       = gcvFALSE;
    gctUINT32 physBaseExt = (gctUINT32)(PhysBase >> 32);
#if gcdENABLE_40BIT_VA
    gctUINT64 start       = PhysBase & ~gcdMMU_PAGE_16M_MASK;
    gctUINT64 end         = (PhysBase + flatSize - 1) & ~gcdMMU_PAGE_16M_MASK;
    gctUINT32 sEnd   = (gctUINT32)((end & gcdMMU_STLB_16M_MASK) >> gcdMMU_STLB_16M_SHIFT);
#else
    gctUINT32 physBase    = (gctUINT32)PhysBase;
    gctUINT64 start       = physBase & ~gcdMMU_PAGE_16M_MASK;
    gctUINT64 end         = (physBase + flatSize - 1) & ~gcdMMU_PAGE_16M_MASK;
#endif
    gctUINT32      mStart = (gctUINT32)(start >> gcdMMU_MTLB_SHIFT);
    gctUINT32      mEnd   = (gctUINT32)(end >> gcdMMU_MTLB_SHIFT);
    gctUINT32      sStart = (gctUINT32)((start & gcdMMU_STLB_16M_MASK) >> gcdMMU_STLB_16M_SHIFT);
    gctPHYS_ADDR_T physical;
    gcsMMU_STLB_CHUNK_PTR newStlbChunk    = gcvNULL;
    gctUINT32             stlbIndex       = 0;
    gctUINT32             totalNewStlbs   = 0;
    gctINT32              firstMtlbEntry  = -1;
    gctUINT32             mtlbCurEntry;
    gctADDRESS            flatVirtualBase = 0;
    gcsMMU_STLB_CHUNK_PTR curStlbChunk    = gcvNULL;
    enum {
        COLOR_NONE = 0,
        COLOR_RED  = 1, /* occupied entry */
        COLOR_BLUE = 2, /* empty entry */
        COLOR_MAX  = COLOR_BLUE,
    } lastColor           = COLOR_NONE;
    gctUINT32 colorNumber = 0;
#if !gcdENABLE_40BIT_VA
    gctUINT32 mCursor;
#endif

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    if (needShiftMapping) {
        gctUINT32 mEntries;
        gctUINT32 sEntries;

#if gcdENABLE_40BIT_VA
        if (!PhysBase) {
            mEntries =
                (gctUINT32)((flatSize + Mmu->reserveRangeSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                            (1ULL << gcdMMU_MTLB_SHIFT));
            sStart = (gctUINT32)(Mmu->reserveRangeSize / gcdMMU_PAGE_16M_SIZE);
        } else {
            mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                                   (1ULL << gcdMMU_MTLB_SHIFT));
            sStart   = 0;
        }

        sEntries = (gctUINT32)((flatSize + gcdMMU_PAGE_16M_SIZE - 1) / gcdMMU_PAGE_16M_SIZE);

        gcmkONERROR(_GetMtlbFreeSpace(Mmu, mEntries, &mStart, &mEnd));
        sEnd = (sStart + sEntries - 1) % gcdMMU_STLB_16M_ENTRY_NUM;
#else
        mEntries = (gctUINT32)(((physBase + flatSize + gcdMMU_PAGE_16M_SIZE - 1) >> gcdMMU_STLB_16M_SHIFT) -
                               (physBase >> gcdMMU_STLB_16M_SHIFT));

        gcmkONERROR(_GetMtlbFreeSpace(Mmu, mEntries, &mStart, &mEnd));

        sStart = mStart % gcdMMU_STLB_16M_ENTRY_NUM;
        sEntries = mEntries;
#endif
    }

    if (specificFlatMapping) {
        gctUINT64 reqStart = reqVirtualBase & ~gcdMMU_PAGE_16M_MASK;
        gctUINT64 reqEnd   = (reqVirtualBase + flatSize - 1) & ~gcdMMU_PAGE_16M_MASK;

        mStart             = (gctUINT32)(reqStart >> gcdMMU_MTLB_SHIFT);
        mEnd               = (gctUINT32)(reqEnd >> gcdMMU_MTLB_SHIFT);
        sStart             = (gctUINT32)((reqStart & gcdMMU_STLB_16M_MASK) >> gcdMMU_STLB_16M_SHIFT);
#if gcdENABLE_40BIT_VA
        sEnd               = (gctUINT32)((reqEnd & gcdMMU_STLB_16M_MASK) >> gcdMMU_STLB_16M_SHIFT);
#endif
    }

    /* No matter direct mapping or shift mapping or specific mapping, store gpu virtual ranges */
    flatVirtualBase = ((gctADDRESS)mStart << gcdMMU_MTLB_SHIFT) |
                      (sStart << gcdMMU_STLB_16M_SHIFT) |
                      (PhysBase & gcdMMU_PAGE_16M_MASK);

    /* Return GPU virtual base address if necessary */
    if (GpuBaseAddress)
        *GpuBaseAddress = flatVirtualBase;

    mtlbCurEntry = mStart;

    /* find all new stlbs, part of new flat mapping range may already have stlbs*/
    while (mtlbCurEntry <= mEnd) {
        if (*(Mmu->mtlbLogical + mtlbCurEntry) == 0) {
            if (lastColor != COLOR_BLUE) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_BLUE;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }

#if gcdENABLE_40BIT_VA
            totalNewStlbs++;
            if (-1 == firstMtlbEntry)
                firstMtlbEntry = mtlbCurEntry;
#else
            _GetCurStlbChunk(Mmu, mtlbCurEntry, &curStlbChunk);

            if (!curStlbChunk) {
                gctUINT32 stlbNum = mtlbCurEntry >> 4;

                if (Mmu->stlbAllocated[stlbNum] == gcvFALSE) {
                    Mmu->stlbAllocated[stlbNum] = gcvTRUE;
                    totalNewStlbs++;
                }
            }

            if (-1 == firstMtlbEntry)
                firstMtlbEntry = mtlbCurEntry & (~((1 << 4) - 1));
#endif
        } else {
            if (lastColor != COLOR_RED) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_RED;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }
        }
        mtlbCurEntry++;
    }

    /* Need allocate a new chunk of stlbs */
    if (totalNewStlbs) {
        gcePOOL   pool      = Mmu->pool;
        gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

        gcmkONERROR(gckOS_Allocate(Mmu->os, sizeof(struct _gcsMMU_STLB_CHUNK),
                                   (gctPOINTER *)&newStlbChunk));

#if gcdENABLE_40BIT_VA
        newStlbChunk->mtlbEntryNum = totalNewStlbs;
        newStlbChunk->size         = gcdMMU_STLB_16M_SIZE * newStlbChunk->mtlbEntryNum;
#else
        newStlbChunk->mtlbEntryNum = totalNewStlbs * 16;
        newStlbChunk->size         = gcdMMU_STLB_16M_SIZE * totalNewStlbs;
#endif
        newStlbChunk->next         = gcvNULL;
        newStlbChunk->videoMem     = gcvNULL;
        newStlbChunk->logical      = gcvNULL;
        newStlbChunk->pageCount    = 0;
        newStlbChunk->mtlbIndex    = firstMtlbEntry;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

#if !gcdENABLE_40BIT_VA
        if (!Mmu->pageTableOver4G)
            allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

        gcmkONERROR(gckKERNEL_AllocateVideoMemory(kernel, 64,
                                                  gcvVIDMEM_TYPE_COMMAND,
                                                  allocFlag | gcvALLOC_FLAG_4K_PAGES,
                                                  &newStlbChunk->size, &pool,
                                                  &newStlbChunk->videoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(kernel, newStlbChunk->videoMem,
                                           gcvFALSE, gcvFALSE,
                                           (gctPOINTER *)&newStlbChunk->logical));

        gcmkONERROR(gckOS_ZeroMemory(newStlbChunk->logical, newStlbChunk->size));

        /* Get GPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(kernel, newStlbChunk->videoMem,
                                                  0, &physical));

        newStlbChunk->physBase = physical;
    }

#if gcdENABLE_40BIT_VA
    while (mStart <= mEnd) {
        gctUINT32      last = (mStart == mEnd) ? sEnd : (gcdMMU_STLB_16M_ENTRY_NUM - 1);
        gctPHYS_ADDR_T stlbPhyBase;
        gctUINT32_PTR  stlbLogical;

        gcmkASSERT(mStart < gcdMMU_MTLB_ENTRY_NUM);

        if (*(Mmu->mtlbLogical + mStart) == 0) {
            gctUINT32 mtlbEntry;

            curStlbChunk = newStlbChunk;
            stlbPhyBase  = curStlbChunk->physBase + (stlbIndex * gcdMMU_STLB_16M_SIZE);
            stlbLogical  = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbIndex * gcdMMU_STLB_16M_SIZE));

            physical = stlbPhyBase
                       /* 16MB page size */
                       | (0x3 << 2)
                       /* Ignore exception */
                       | (0 << 1)
                       /* Present */
                       | (1 << 0);

            gcmkSAFECASTPHYSADDRT(mtlbEntry, physical);

            _WritePageEntry(Mmu->mtlbLogical + mStart, mtlbEntry);

#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__, mStart,
                        _ReadPageEntry(Mmu->mtlbLogical + mStart));

            gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n",
                        __FUNCTION__, __LINE__, stlbLogical, stlbPhyBase);
#    endif

            gcmkDUMP(Mmu->os, "#[mmu-mtlb: flat-mapping, slot: %d]", mStart);

            gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)Mmu->mtlbPhysical + mStart * 4,
                     Mmu->mtlbLogical[mStart], 4);

            ++stlbIndex;
        } else {
            gctUINT32 mtlbEntry = _ReadPageEntry(Mmu->mtlbLogical + mStart);
            gctUINT   stlbOffset;

            _GetCurStlbChunk(Mmu, mStart, &curStlbChunk);

            if (!curStlbChunk)
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

            stlbOffset  = mStart - curStlbChunk->mtlbIndex;

            stlbPhyBase = curStlbChunk->physBase + (stlbOffset * gcdMMU_STLB_16M_SIZE);
            stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbOffset * gcdMMU_STLB_16M_SIZE));

            if (stlbPhyBase != (mtlbEntry & gcdMMU_MTLB_ENTRY_STLB_MASK))
                gcmkASSERT(0);
        }

#if gcdDUMP_IN_KERNEL
        gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping: 0x%llX - 0x%llX]",
                 start, start + (last - sStart) * gcdMMU_PAGE_16M_SIZE - 1);
#    endif

        while (sStart <= last) {
            gcmkASSERT(!(start & gcdMMU_PAGE_16M_MASK));
            if (reserved) {
                /* program NOT_PRESENT | EXCEPTION  for reserved entries */
                _WritePageEntry(stlbLogical + sStart, 1 << 1);
            } else {
                _WritePageEntry(stlbLogical + sStart,
                                _SetPage((gctUINT32)start, physBaseExt, gcvTRUE));
            }
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert STLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__, sStart,
                        _ReadPageEntry(stlbLogical + sStart));
#    endif
            /* next page. */
            start += gcdMMU_PAGE_16M_SIZE;
            if ((start & 0xFFFFFFFF) == 0)
                physBaseExt++;

            sStart++;
            curStlbChunk->pageCount++;
        }

#if gcdDUMP_IN_KERNEL
        {
            gctUINT32 i    = sStart;
            gctUINT32 data = stlbLogical[i] & ~0xF;
            gctUINT32 step = (last > i) ? (stlbLogical[i + 1] - stlbLogical[i]) : 0;
            gctUINT32 mask = stlbLogical[i] & 0xF;

            gcmkDUMP(Mmu->os, "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                     (unsigned long long)stlbPhyBase + i * 4, data, (last - i) * 4, step, mask);
        }
#    endif

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(kernel, curStlbChunk->videoMem, 0,
                                              curStlbChunk->logical, curStlbChunk->size));

        sStart = 0;
        ++mStart;
    }

    gcmkASSERT(totalNewStlbs == stlbIndex);
#else

    mCursor = mStart;

    while (mCursor <= mEnd) {
        gctPHYS_ADDR_T stlbPhyBase;
        gctUINT32_PTR stlbLogical;

        gcmkASSERT(mCursor < gcdMMU_MTLB_ENTRY_NUM);

        if (*(Mmu->mtlbLogical + mCursor) == 0) {
            gctUINT32 mtlbEntry;

            _GetCurStlbChunk(Mmu, mCursor, &curStlbChunk);
            if (!curStlbChunk) {
                if (totalNewStlbs)
                    curStlbChunk = newStlbChunk;
            }

            stlbIndex   = (mCursor - curStlbChunk->mtlbIndex) >> 4;
            stlbPhyBase = curStlbChunk->physBase + (stlbIndex * gcdMMU_STLB_16M_SIZE);
            stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbIndex * gcdMMU_STLB_16M_SIZE));

            physical = stlbPhyBase
                       /* 16MB page size */
                       | (0x3 << 2)
                       /* Ignore exception */
                       | (0 << 1)
                       /* Present */
                       | (1 << 0);

            gcmkSAFECASTPHYSADDRT(mtlbEntry, physical);

            _WritePageEntry(Mmu->mtlbLogical + mCursor, mtlbEntry);

#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__, mStart,
                        _ReadPageEntry(Mmu->mtlbLogical + mCursor));

            gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n",
                        __FUNCTION__, __LINE__, stlbLogical, stlbPhyBase);
#    endif

            gcmkDUMP(Mmu->os, "#[mmu-mtlb: flat-mapping, slot: %d]", mCursor);

            gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)Mmu->mtlbPhysical + mCursor * 4,
                     Mmu->mtlbLogical[mCursor], 4);

#if gcdDUMP_IN_KERNEL
            gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping: 0x%llX - 0x%llX]",
                     start, start + gcdMMU_PAGE_16M_SIZE - 1);
#    endif

            if (*(stlbLogical + sStart) == 0) {
                gcmkASSERT(!(start & gcdMMU_PAGE_16M_MASK));

                if (reserved) {
                    /* program NOT_PRESENT | EXCEPTION  for reserved entries */
                    _WritePageEntry(stlbLogical + sStart, 1 << 1);
                } else {
                    _WritePageEntry(stlbLogical + sStart,
                                    _SetPage((gctUINT32)start, physBaseExt, gcvTRUE));
                }
#if gcdMMU_TABLE_DUMP
                gckOS_Print("%s(%d): insert STLB[%d]: %08x\n",
                            __FUNCTION__, __LINE__,
                            sStart, _ReadPageEntry(stlbLogical + sStart));
#    endif
            }

            gcmkONERROR(gckVIDMEM_NODE_CleanCache(kernel, curStlbChunk->videoMem, 0,
                                                  curStlbChunk->logical, curStlbChunk->size));

            curStlbChunk->pageCount++;
        }

        /* next page. */
        start += gcdMMU_PAGE_16M_SIZE;
        if ((start & 0xFFFFFFFF) == 0)
            physBaseExt++;

        if (++sStart == gcdMMU_STLB_16M_ENTRY_NUM)
            sStart = 0;

        ++mCursor;
    }
#endif

    if (newStlbChunk) {
        /* Insert the stlbChunk into staticSTLB. */
        if (Mmu->staticSTLB == gcvNULL) {
            Mmu->staticSTLB = newStlbChunk;
        } else {
            gcmkASSERT(newStlbChunk != gcvNULL);
            gcmkASSERT(newStlbChunk->next == gcvNULL);
            newStlbChunk->next = Mmu->staticSTLB;
            Mmu->staticSTLB    = newStlbChunk;
        }
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;
OnError:
    /* Roll back the allocation.
     * We don't need roll back mtlb programming as gckmONERROR
     * is only used during allocation time.
     */
    if (newStlbChunk) {
        if (newStlbChunk->videoMem)
            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, newStlbChunk->videoMem));

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, newStlbChunk));
    }
    if (mutex) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }
    return status;
}

static gceSTATUS
gckMMU_FillFlatMappingWithPage1M(IN gckMMU Mmu, IN gctUINT64 PhysBase, IN gctSIZE_T flatSize,
                                 IN gctBOOL reserved, IN gctBOOL needShiftMapping,
                                 IN gctBOOL specificFlatMapping, IN gctADDRESS reqVirtualBase,
                                 OUT gctADDRESS *GpuBaseAddress)
{
    gceSTATUS status;
    gckKERNEL kernel      = Mmu->hardware->kernel;
    gctBOOL   mutex       = gcvFALSE;
    gctUINT32 physBaseExt = (gctUINT32)(PhysBase >> 32);
#if gcdENABLE_40BIT_VA
    gctUINT64 start       = PhysBase & ~gcdMMU_PAGE_1M_MASK;
    gctUINT64 end         = (PhysBase + flatSize - 1) & ~gcdMMU_PAGE_1M_MASK;
#else
    gctUINT32 physBase    = (gctUINT32)PhysBase;
    gctUINT64 start       = physBase & ~gcdMMU_PAGE_1M_MASK;
    gctUINT64 end         = (physBase + flatSize - 1) & ~gcdMMU_PAGE_1M_MASK;
#endif
    gctUINT32      mStart = (gctUINT32)(start >> gcdMMU_MTLB_SHIFT);
    gctUINT32      mEnd   = (gctUINT32)(end >> gcdMMU_MTLB_SHIFT);
    gctUINT32      sStart = (gctUINT32)((start & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT);
    gctUINT32      sEnd   = (gctUINT32)((end & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT);
    gctPHYS_ADDR_T physical;
    gcsMMU_STLB_CHUNK_PTR newStlbChunk    = gcvNULL;
    gctUINT32             stlbIndex       = 0;
    gctUINT32             totalNewStlbs   = 0;
    gctINT32              firstMtlbEntry  = -1;
    gctUINT32             mtlbCurEntry;
    gctADDRESS            flatVirtualBase = 0;
    gcsMMU_STLB_CHUNK_PTR curStlbChunk    = gcvNULL;
    enum {
        COLOR_NONE = 0,
        COLOR_RED  = 1, /* occupied entry */
        COLOR_BLUE = 2, /* empty entry */
        COLOR_MAX  = COLOR_BLUE,
    } lastColor = COLOR_NONE;
    gctUINT32 colorNumber = 0;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    if (needShiftMapping) {
        gctUINT32 mEntries;
        gctUINT32 sEntries;

#if gcdENABLE_40BIT_VA
        if (!PhysBase) {
            mEntries =
                (gctUINT32)((flatSize + Mmu->reserveRangeSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                            (1ULL << gcdMMU_MTLB_SHIFT));
            sStart = (gctUINT32)(Mmu->reserveRangeSize / gcdMMU_PAGE_1M_SIZE);
        } else {
            mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                                   (1ULL << gcdMMU_MTLB_SHIFT));
            sStart   = 0;
        }
#else
        mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                               (1ULL << gcdMMU_MTLB_SHIFT));
        sStart   = 0;
#endif

        gcmkONERROR(_GetMtlbFreeSpace(Mmu, mEntries, &mStart, &mEnd));

        sEntries = (gctUINT32)((flatSize + gcdMMU_PAGE_1M_SIZE - 1) / gcdMMU_PAGE_1M_SIZE);
        sEnd     = (sStart + sEntries - 1) % gcdMMU_STLB_1M_ENTRY_NUM;
    }

    if (specificFlatMapping) {
        gctUINT64 reqStart = reqVirtualBase & ~gcdMMU_PAGE_1M_MASK;
        gctUINT64 reqEnd   = (reqVirtualBase + flatSize - 1) & ~gcdMMU_PAGE_1M_MASK;

        mStart             = (gctUINT32)(reqStart >> gcdMMU_MTLB_SHIFT);
        mEnd               = (gctUINT32)(reqEnd >> gcdMMU_MTLB_SHIFT);
        sStart             = (gctUINT32)((reqStart & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT);
        sEnd               = (gctUINT32)((reqEnd & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT);
    }

    /* No matter direct mapping or shift mapping or specific mapping, store gpu virtual ranges */
    flatVirtualBase = ((gctADDRESS)mStart << gcdMMU_MTLB_SHIFT) |
                      (sStart << gcdMMU_STLB_1M_SHIFT) |
                      (PhysBase & gcdMMU_PAGE_1M_MASK);

    /* Return GPU virtual base address if necessary */
    if (GpuBaseAddress)
        *GpuBaseAddress = flatVirtualBase;

    mtlbCurEntry = mStart;

    /* find all new stlbs, part of new flat mapping range may already have stlbs*/
    while (mtlbCurEntry <= mEnd) {
        if (*(Mmu->mtlbLogical + mtlbCurEntry) == 0) {
            if (lastColor != COLOR_BLUE) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_BLUE;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }

            totalNewStlbs++;
            if (-1 == firstMtlbEntry)
                firstMtlbEntry = mtlbCurEntry;
        } else {
            if (lastColor != COLOR_RED) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_RED;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }
        }
        mtlbCurEntry++;
    }

    /* Need allocate a new chunk of stlbs */
    if (totalNewStlbs) {
        gcePOOL   pool      = Mmu->pool;
        gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

        gcmkONERROR(gckOS_Allocate(Mmu->os, sizeof(struct _gcsMMU_STLB_CHUNK),
                                   (gctPOINTER *)&newStlbChunk));

        newStlbChunk->mtlbEntryNum = totalNewStlbs;
        newStlbChunk->next         = gcvNULL;
        newStlbChunk->videoMem     = gcvNULL;
        newStlbChunk->logical      = gcvNULL;
        newStlbChunk->size         = gcdMMU_STLB_1M_SIZE * newStlbChunk->mtlbEntryNum;
        newStlbChunk->pageCount    = 0;
        newStlbChunk->mtlbIndex    = firstMtlbEntry;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

#if !gcdENABLE_40BIT_VA
        if (!Mmu->pageTableOver4G)
            allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

        gcmkONERROR(gckKERNEL_AllocateVideoMemory(kernel, 64,
                                                  gcvVIDMEM_TYPE_COMMAND,
                                                  allocFlag | gcvALLOC_FLAG_4K_PAGES,
                                                  &newStlbChunk->size, &pool,
                                                  &newStlbChunk->videoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(kernel, newStlbChunk->videoMem,
                                           gcvFALSE, gcvFALSE,
                                           (gctPOINTER *)&newStlbChunk->logical));

        gcmkONERROR(gckOS_ZeroMemory(newStlbChunk->logical, newStlbChunk->size));

        /* Get GPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(kernel, newStlbChunk->videoMem, 0, &physical));

        newStlbChunk->physBase = physical;
    }

    while (mStart <= mEnd) {
        gctUINT32      last = (mStart == mEnd) ? sEnd : (gcdMMU_STLB_1M_ENTRY_NUM - 1);
        gctPHYS_ADDR_T stlbPhyBase;
        gctUINT32_PTR  stlbLogical;

        gcmkASSERT(mStart < gcdMMU_MTLB_ENTRY_NUM);

        if (*(Mmu->mtlbLogical + mStart) == 0) {
            gctUINT32 mtlbEntry;

            curStlbChunk = newStlbChunk;
            stlbPhyBase  = curStlbChunk->physBase + (stlbIndex * gcdMMU_STLB_1M_SIZE);
            stlbLogical  = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbIndex * gcdMMU_STLB_1M_SIZE));

            physical = stlbPhyBase
                       /* 1MB page size */
                       | (1 << 3)
                       /* Ignore exception */
                       | (0 << 1)
                       /* Present */
                       | (1 << 0);

            gcmkSAFECASTPHYSADDRT(mtlbEntry, physical);

            _WritePageEntry(Mmu->mtlbLogical + mStart, mtlbEntry);

#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__,
                        mStart, _ReadPageEntry(Mmu->mtlbLogical + mStart));

            gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n"
                         __FUNCTION__, __LINE__,
                        stlbLogical, stlbPhyBase);
#endif

            gcmkDUMP(Mmu->os, "#[mmu-mtlb: flat-mapping, slot: %d]", mStart);

            gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)Mmu->mtlbPhysical + mStart * 4,
                     Mmu->mtlbLogical[mStart], 4);

            ++stlbIndex;
        } else {
            gctUINT32 mtlbEntry = _ReadPageEntry(Mmu->mtlbLogical + mStart);
            gctUINT   stlbOffset;

            _GetCurStlbChunk(Mmu, mStart, &curStlbChunk);

            if (!curStlbChunk)
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

            stlbOffset = mStart - curStlbChunk->mtlbIndex;

            stlbPhyBase = curStlbChunk->physBase + (stlbOffset * gcdMMU_STLB_1M_SIZE);
            stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbOffset * gcdMMU_STLB_1M_SIZE));

            if (stlbPhyBase != (mtlbEntry & gcdMMU_MTLB_ENTRY_STLB_MASK))
                gcmkASSERT(0);
        }

#if gcdDUMP_IN_KERNEL
        gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping: 0x%08X - 0x%08X]",
                 start, start + (last - sStart) * gcdMMU_PAGE_1M_SIZE - 1);
#endif

        while (sStart <= last) {
            gcmkASSERT(!(start & gcdMMU_PAGE_1M_MASK));
            if (reserved) {
                /* program NOT_PRESENT | EXCEPTION  for reserved entries */
                _WritePageEntry(stlbLogical + sStart, 1 << 1);
            } else {
                _WritePageEntry(stlbLogical + sStart,
                                _SetPage((gctUINT32)start, physBaseExt, gcvTRUE));
            }
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert STLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__,
                        sStart, _ReadPageEntry(stlbLogical + sStart));
#endif
            /* next page. */
            start += gcdMMU_PAGE_1M_SIZE;
            if ((start & 0xFFFFFFFF) == 0)
                physBaseExt++;

            sStart++;
            curStlbChunk->pageCount++;
        }

#if gcdDUMP_IN_KERNEL
        {
            gctUINT32 i    = sStart;
            gctUINT32 data = stlbLogical[i] & ~0xF;
            gctUINT32 step = (last > i) ? (stlbLogical[i + 1] - stlbLogical[i]) : 0;
            gctUINT32 mask = stlbLogical[i] & 0xF;

            gcmkDUMP(Mmu->os, "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                     (unsigned long long)stlbPhyBase + i * 4, data, (last - i) * 4, step, mask);
        }
#endif

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(kernel, curStlbChunk->videoMem, 0,
                                              curStlbChunk->logical, curStlbChunk->size));

        sStart = 0;
        ++mStart;
    }

    gcmkASSERT(totalNewStlbs == stlbIndex);

    if (newStlbChunk) {
        /* Insert the stlbChunk into staticSTLB. */
        if (Mmu->staticSTLB == gcvNULL) {
            Mmu->staticSTLB = newStlbChunk;
        } else {
            gcmkASSERT(newStlbChunk != gcvNULL);
            gcmkASSERT(newStlbChunk->next == gcvNULL);
            newStlbChunk->next = Mmu->staticSTLB;
            Mmu->staticSTLB    = newStlbChunk;
        }
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;
OnError:
    /* Roll back the allocation.
     * We don't need roll back mtlb programming as gckmONERROR
     * is only used during allocation time.
     */
    if (newStlbChunk) {
        if (newStlbChunk->videoMem)
            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, newStlbChunk->videoMem));

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, newStlbChunk));
    }
    if (mutex) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }
    return status;
}

static gceSTATUS
gckMMU_FillFlatMappingWithPage64K(IN gckMMU Mmu, IN gctUINT64 PhysBase, IN gctSIZE_T flatSize,
                                  IN gctBOOL reserved, IN gctBOOL needShiftMapping,
                                  IN gctBOOL specificFlatMapping, IN gctADDRESS reqVirtualBase,
                                  OUT gctADDRESS *GpuBaseAddress)
{
    gceSTATUS status;
    gckKERNEL kernel      = Mmu->hardware->kernel;
    gctBOOL   mutex       = gcvFALSE;
    gctUINT32 physBaseExt = (gctUINT32)(PhysBase >> 32);
#if gcdENABLE_40BIT_VA
    gctUINT64 start       = PhysBase & ~gcdMMU_PAGE_64K_MASK;
    gctUINT64 end         = (PhysBase + flatSize - 1) & ~gcdMMU_PAGE_64K_MASK;
#else
    gctUINT32 physBase    = (gctUINT32)PhysBase;
    gctUINT64 start       = physBase & ~gcdMMU_PAGE_64K_MASK;
    gctUINT64 end         = (physBase + flatSize - 1) & ~gcdMMU_PAGE_64K_MASK;
#endif
    gctUINT32      mStart = (gctUINT32)(start >> gcdMMU_MTLB_SHIFT);
    gctUINT32      mEnd   = (gctUINT32)(end >> gcdMMU_MTLB_SHIFT);
    gctUINT32      sStart = (gctUINT32)((start & gcdMMU_STLB_64K_MASK) >> gcdMMU_STLB_64K_SHIFT);
    gctUINT32      sEnd   = (gctUINT32)((end & gcdMMU_STLB_64K_MASK) >> gcdMMU_STLB_64K_SHIFT);
    gctPHYS_ADDR_T physical;
    gcsMMU_STLB_CHUNK_PTR newStlbChunk    = gcvNULL;
    gctUINT32             stlbIndex       = 0;
    gctUINT32             totalNewStlbs   = 0;
    gctINT32              firstMtlbEntry  = -1;
    gctUINT32             mtlbCurEntry;
    gctADDRESS            flatVirtualBase = 0;
    gcsMMU_STLB_CHUNK_PTR curStlbChunk    = gcvNULL;
    enum {
        COLOR_NONE = 0,
        COLOR_RED  = 1, /* occupied entry */
        COLOR_BLUE = 2, /* empty entry */
        COLOR_MAX  = COLOR_BLUE,
    } lastColor = COLOR_NONE;
    gctUINT32 colorNumber = 0;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    if (needShiftMapping) {
        gctUINT32 mEntries;
        gctUINT32 sEntries;

#if gcdENABLE_40BIT_VA
        if (!PhysBase) {
            mEntries =
                (gctUINT32)((flatSize + Mmu->reserveRangeSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                            (1ULL << gcdMMU_MTLB_SHIFT));
            sStart = (gctUINT32)(Mmu->reserveRangeSize / gcdMMU_PAGE_64K_SIZE);
        } else {
            mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                                   (1ULL << gcdMMU_MTLB_SHIFT));
            sStart   = 0;
        }
#else
        mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                               (1ULL << gcdMMU_MTLB_SHIFT));
        sStart   = 0;
#endif

        gcmkONERROR(_GetMtlbFreeSpace(Mmu, mEntries, &mStart, &mEnd));

        sEntries = (gctUINT32)((flatSize + gcdMMU_PAGE_64K_SIZE - 1) / gcdMMU_PAGE_64K_SIZE);
        sEnd     = (sStart + sEntries - 1) % gcdMMU_STLB_64K_ENTRY_NUM;
    }

    if (specificFlatMapping) {
        gctUINT64 reqStart = reqVirtualBase & ~gcdMMU_PAGE_64K_MASK;
        gctUINT64 reqEnd   = (reqVirtualBase + flatSize - 1) & ~gcdMMU_PAGE_64K_MASK;

        mStart             = (gctUINT32)(reqStart >> gcdMMU_MTLB_SHIFT);
        mEnd               = (gctUINT32)(reqEnd >> gcdMMU_MTLB_SHIFT);
        sStart = (gctUINT32)((reqStart & gcdMMU_STLB_64K_MASK) >> gcdMMU_STLB_64K_SHIFT);
        sEnd   = (gctUINT32)((reqEnd & gcdMMU_STLB_64K_MASK) >> gcdMMU_STLB_64K_SHIFT);
    }

    /* No matter direct mapping or shift mapping or specific mapping, store gpu virtual ranges */
    flatVirtualBase = ((gctADDRESS)mStart << gcdMMU_MTLB_SHIFT) |
                      (sStart << gcdMMU_STLB_64K_SHIFT) |
                      (PhysBase & gcdMMU_PAGE_64K_MASK);

    /* Return GPU virtual base address if necessary */
    if (GpuBaseAddress)
        *GpuBaseAddress = flatVirtualBase;

    mtlbCurEntry = mStart;

    /* find all new stlbs, part of new flat mapping range may already have stlbs*/
    while (mtlbCurEntry <= mEnd) {
        if (*(Mmu->mtlbLogical + mtlbCurEntry) == 0) {
            if (lastColor != COLOR_BLUE) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_BLUE;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }

            totalNewStlbs++;
            if (-1 == firstMtlbEntry)
                firstMtlbEntry = mtlbCurEntry;
        } else {
            if (lastColor != COLOR_RED) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_RED;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }
        }
        mtlbCurEntry++;
    }

    /* Need allocate a new chunk of stlbs */
    if (totalNewStlbs) {
        gcePOOL   pool      = Mmu->pool;
        gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

        gcmkONERROR(gckOS_Allocate(Mmu->os, sizeof(struct _gcsMMU_STLB_CHUNK),
                                   (gctPOINTER *)&newStlbChunk));

        newStlbChunk->mtlbEntryNum = totalNewStlbs;
        newStlbChunk->next         = gcvNULL;
        newStlbChunk->videoMem     = gcvNULL;
        newStlbChunk->logical      = gcvNULL;
        newStlbChunk->size         = gcdMMU_STLB_64K_SIZE * newStlbChunk->mtlbEntryNum;
        newStlbChunk->pageCount    = 0;
        newStlbChunk->mtlbIndex    = firstMtlbEntry;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

#if !gcdENABLE_40BIT_VA
        if (!Mmu->pageTableOver4G)
            allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

        gcmkONERROR(gckKERNEL_AllocateVideoMemory(kernel, 64,
                                                  gcvVIDMEM_TYPE_COMMAND,
                                                  allocFlag | gcvALLOC_FLAG_4K_PAGES,
                                                  &newStlbChunk->size, &pool,
                                                  &newStlbChunk->videoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(kernel, newStlbChunk->videoMem,
                                           gcvFALSE, gcvFALSE,
                                           (gctPOINTER *)&newStlbChunk->logical));

        gcmkONERROR(gckOS_ZeroMemory(newStlbChunk->logical, newStlbChunk->size));

        /* Get GPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(kernel, newStlbChunk->videoMem, 0, &physical));

        newStlbChunk->physBase = physical;
    }

    while (mStart <= mEnd) {
        gctUINT32      last = (mStart == mEnd) ? sEnd : (gcdMMU_STLB_64K_ENTRY_NUM - 1);
        gctPHYS_ADDR_T stlbPhyBase;
        gctUINT32_PTR  stlbLogical;

        gcmkASSERT(mStart < gcdMMU_MTLB_ENTRY_NUM);

        if (*(Mmu->mtlbLogical + mStart) == 0) {
            gctUINT32 mtlbEntry;

            curStlbChunk = newStlbChunk;
            stlbPhyBase  = curStlbChunk->physBase + (stlbIndex * gcdMMU_STLB_64K_SIZE);
            stlbLogical  = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbIndex * gcdMMU_STLB_64K_SIZE));

            physical = stlbPhyBase
                       /* 64KB page size */
                       | (1 << 2)
                       /* Ignore exception */
                       | (0 << 1)
                       /* Present */
                       | (1 << 0);

            gcmkSAFECASTPHYSADDRT(mtlbEntry, physical);

            _WritePageEntry(Mmu->mtlbLogical + mStart, mtlbEntry);

#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__, mStart,
                        _ReadPageEntry(Mmu->mtlbLogical + mStart));

            gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n",
                        __FUNCTION__, __LINE__,
                        stlbLogical, stlbPhyBase);
#endif

            gcmkDUMP(Mmu->os, "#[mmu-mtlb: flat-mapping, slot: %d]", mStart);

            gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)Mmu->mtlbPhysical + mStart * 4,
                     Mmu->mtlbLogical[mStart], 4);

            ++stlbIndex;
        } else {
            gctUINT32 mtlbEntry = _ReadPageEntry(Mmu->mtlbLogical + mStart);
            gctUINT   stlbOffset;

            _GetCurStlbChunk(Mmu, mStart, &curStlbChunk);

            if (!curStlbChunk)
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

            stlbOffset = mStart - curStlbChunk->mtlbIndex;

            stlbPhyBase = curStlbChunk->physBase + (stlbOffset * gcdMMU_STLB_64K_SIZE);
            stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbOffset * gcdMMU_STLB_64K_SIZE));

            if (stlbPhyBase != (mtlbEntry & gcdMMU_MTLB_ENTRY_STLB_MASK))
                gcmkASSERT(0);
        }

#if gcdDUMP_IN_KERNEL
        gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping: 0x%08X - 0x%08X]",
                 start, start + (last - sStart) * gcdMMU_PAGE_64K_SIZE - 1);
#endif

        while (sStart <= last) {
            gcmkASSERT(!(start & gcdMMU_PAGE_64K_MASK));
            if (reserved) {
                /* program NOT_PRESENT | EXCEPTION  for reserved entries */
                _WritePageEntry(stlbLogical + sStart, 1 << 1);
            } else {
                _WritePageEntry(stlbLogical + sStart,
                                _SetPage((gctUINT32)start, physBaseExt, gcvTRUE));
            }
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert STLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__,
                        sStart, _ReadPageEntry(stlbLogical + sStart));
#endif
            /* next page. */
            start += gcdMMU_PAGE_64K_SIZE;
            if ((start & 0xFFFFFFFF) == 0)
                physBaseExt++;

            sStart++;
            curStlbChunk->pageCount++;
        }

#if gcdDUMP_IN_KERNEL
        {
            gctUINT32 i    = sStart;
            gctUINT32 data = stlbLogical[i] & ~0xF;
            gctUINT32 step = (last > i) ? (stlbLogical[i + 1] - stlbLogical[i]) : 0;
            gctUINT32 mask = stlbLogical[i] & 0xF;

            gcmkDUMP(Mmu->os, "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                     (unsigned long long)stlbPhyBase + i * 4, data, (last - i) * 4, step, mask);
        }
#endif

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(kernel, curStlbChunk->videoMem, 0,
                                              curStlbChunk->logical, curStlbChunk->size));

        sStart = 0;
        ++mStart;
    }

    gcmkASSERT(totalNewStlbs == stlbIndex);

    if (newStlbChunk) {
        /* Insert the stlbChunk into staticSTLB. */
        if (Mmu->staticSTLB == gcvNULL) {
            Mmu->staticSTLB = newStlbChunk;
        } else {
            gcmkASSERT(newStlbChunk != gcvNULL);
            gcmkASSERT(newStlbChunk->next == gcvNULL);
            newStlbChunk->next = Mmu->staticSTLB;
            Mmu->staticSTLB    = newStlbChunk;
        }
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;
OnError:
    /* Roll back the allocation.
     * We don't need roll back mtlb programming as gckmONERROR
     * is only used during allocation time.
     */
    if (newStlbChunk) {
        if (newStlbChunk->videoMem)
            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, newStlbChunk->videoMem));

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, newStlbChunk));
    }
    if (mutex) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }
    return status;
}

static gceSTATUS
gckMMU_FillFlatMappingWithPage4K(IN gckMMU Mmu, IN gctUINT64 PhysBase, IN gctSIZE_T flatSize,
                                 IN gctBOOL reserved, IN gctBOOL needShiftMapping,
                                 IN gctBOOL specificFlatMapping, IN gctADDRESS reqVirtualBase,
                                 OUT gctADDRESS *GpuBaseAddress)
{
    gceSTATUS status;
    gckKERNEL kernel      = Mmu->hardware->kernel;
    gctBOOL   mutex       = gcvFALSE;
    gctUINT32 physBaseExt = (gctUINT32)(PhysBase >> 32);
#if gcdENABLE_40BIT_VA
    gctUINT64 start       = PhysBase & ~gcdMMU_PAGE_4K_MASK;
    gctUINT64 end         = (PhysBase + flatSize - 1) & ~gcdMMU_PAGE_4K_MASK;
#else
    gctUINT32 physBase = (gctUINT32)PhysBase;
    gctUINT64 start = physBase & ~gcdMMU_PAGE_4K_MASK;
    gctUINT64 end = (physBase + flatSize - 1) & ~gcdMMU_PAGE_4K_MASK;
#endif
    gctUINT32      mStart = (gctUINT32)(start >> gcdMMU_MTLB_SHIFT);
    gctUINT32      mEnd   = (gctUINT32)(end >> gcdMMU_MTLB_SHIFT);
    gctUINT32      sStart = (gctUINT32)((start & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT);
    gctUINT32      sEnd   = (gctUINT32)((end & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT);
    gctPHYS_ADDR_T physical;
    gcsMMU_STLB_CHUNK_PTR newStlbChunk    = gcvNULL;
    gctUINT32             stlbIndex       = 0;
    gctUINT32             totalNewStlbs   = 0;
    gctINT32              firstMtlbEntry  = -1;
    gctUINT32             mtlbCurEntry;
    gctADDRESS            flatVirtualBase = 0;
    gcsMMU_STLB_CHUNK_PTR curStlbChunk    = gcvNULL;
    enum {
        COLOR_NONE = 0,
        COLOR_RED  = 1, /* occupied entry */
        COLOR_BLUE = 2, /* empty entry */
        COLOR_MAX  = COLOR_BLUE,
    } lastColor = COLOR_NONE;
    gctUINT32 colorNumber = 0;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    if (needShiftMapping) {
        gctUINT32 mEntries;
        gctUINT32 sEntries;

#if gcdENABLE_40BIT_VA
        if (!PhysBase) {
            mEntries =
                (gctUINT32)((flatSize + Mmu->reserveRangeSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                            (1ULL << gcdMMU_MTLB_SHIFT));
            sStart = (gctUINT32)(Mmu->reserveRangeSize / gcdMMU_PAGE_4K_SIZE);
        } else {
            mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                                   (1ULL << gcdMMU_MTLB_SHIFT));
            sStart   = 0;
        }
#else
        mEntries = (gctUINT32)((flatSize + (1ULL << gcdMMU_MTLB_SHIFT) - 1) /
                               (1ULL << gcdMMU_MTLB_SHIFT));
        sStart   = 0;
#endif

        gcmkONERROR(_GetMtlbFreeSpace(Mmu, mEntries, &mStart, &mEnd));

        sEntries = (gctUINT32)((flatSize + gcdMMU_PAGE_4K_SIZE - 1) / gcdMMU_PAGE_4K_SIZE);
        sEnd     = (sStart + sEntries - 1) % gcdMMU_STLB_4K_ENTRY_NUM;
    }

    if (specificFlatMapping) {
        gctUINT64 reqStart = reqVirtualBase & ~gcdMMU_PAGE_4K_MASK;
        gctUINT64 reqEnd   = (reqVirtualBase + flatSize - 1) & ~gcdMMU_PAGE_4K_MASK;

        mStart             = (gctUINT32)(reqStart >> gcdMMU_MTLB_SHIFT);
        mEnd               = (gctUINT32)(reqEnd >> gcdMMU_MTLB_SHIFT);
        sStart             = (gctUINT32)((reqStart & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT);
        sEnd               = (gctUINT32)((reqEnd & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT);
    }

    /* No matter direct mapping or shift mapping or specific mapping, store gpu virtual ranges */
    flatVirtualBase = ((gctADDRESS)mStart << gcdMMU_MTLB_SHIFT) |
                      (sStart << gcdMMU_STLB_4K_SHIFT) |
                      (PhysBase & gcdMMU_PAGE_4K_MASK);

    /* Return GPU virtual base address if necessary */
    if (GpuBaseAddress)
        *GpuBaseAddress = flatVirtualBase;

    mtlbCurEntry = mStart;

    /* find all new stlbs, part of new flat mapping range may already have stlbs*/
    while (mtlbCurEntry <= mEnd) {
        if (*(Mmu->mtlbLogical + mtlbCurEntry) == 0) {
            if (lastColor != COLOR_BLUE) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_BLUE;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }

            totalNewStlbs++;
            if (-1 == firstMtlbEntry)
                firstMtlbEntry = mtlbCurEntry;
        } else {
            if (lastColor != COLOR_RED) {
                if (colorNumber < COLOR_MAX) {
                    lastColor = COLOR_RED;
                    colorNumber++;
                } else {
                    gcmkPRINT("There is a hole in new flat mapping range, which is not correct");
                }
            }
        }
        mtlbCurEntry++;
    }

    /* Need allocate a new chunk of stlbs */
    if (totalNewStlbs) {
        gcePOOL   pool      = Mmu->pool;
        gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;

        gcmkONERROR(gckOS_Allocate(Mmu->os, sizeof(struct _gcsMMU_STLB_CHUNK),
                                   (gctPOINTER *)&newStlbChunk));

        newStlbChunk->mtlbEntryNum = totalNewStlbs;
        newStlbChunk->next         = gcvNULL;
        newStlbChunk->videoMem     = gcvNULL;
        newStlbChunk->logical      = gcvNULL;
        newStlbChunk->size         = gcdMMU_STLB_4K_SIZE * newStlbChunk->mtlbEntryNum;
        newStlbChunk->pageCount    = 0;
        newStlbChunk->mtlbIndex    = firstMtlbEntry;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

#if !gcdENABLE_40BIT_VA
        if (!Mmu->pageTableOver4G)
            allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

        gcmkONERROR(gckKERNEL_AllocateVideoMemory(kernel, 64,
                                                  gcvVIDMEM_TYPE_COMMAND,
                                                  allocFlag | gcvALLOC_FLAG_4K_PAGES,
                                                  &newStlbChunk->size, &pool,
                                                  &newStlbChunk->videoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(kernel, newStlbChunk->videoMem,
                                           gcvFALSE, gcvFALSE,
                                           (gctPOINTER *)&newStlbChunk->logical));

        gcmkONERROR(gckOS_ZeroMemory(newStlbChunk->logical, newStlbChunk->size));

        /* Get GPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(kernel, newStlbChunk->videoMem, 0, &physical));

        newStlbChunk->physBase = physical;
    }

    while (mStart <= mEnd) {
        gctUINT32      last = (mStart == mEnd) ? sEnd : (gcdMMU_STLB_4K_ENTRY_NUM - 1);
        gctPHYS_ADDR_T stlbPhyBase;
        gctUINT32_PTR  stlbLogical;

        gcmkASSERT(mStart < gcdMMU_MTLB_ENTRY_NUM);

        if (*(Mmu->mtlbLogical + mStart) == 0) {
            gctUINT32 mtlbEntry;

            curStlbChunk = newStlbChunk;
            stlbPhyBase  = curStlbChunk->physBase + (stlbIndex * gcdMMU_STLB_4K_SIZE);
            stlbLogical  = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbIndex * gcdMMU_STLB_4K_SIZE));

            physical = stlbPhyBase
                       /* 4KB page size */
                       | (0 << 3)
                       /* Ignore exception */
                       | (0 << 1)
                       /* Present */
                       | (1 << 0);

            gcmkSAFECASTPHYSADDRT(mtlbEntry, physical);

            _WritePageEntry(Mmu->mtlbLogical + mStart, mtlbEntry);

#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__, mStart,
                        _ReadPageEntry(Mmu->mtlbLogical + mStart));

            gckOS_Print("%s(%d): STLB: logical:%08x -> physical:%08x\n",
                        __FUNCTION__, __LINE__,
                        stlbLogical, stlbPhyBase);
#endif

            gcmkDUMP(Mmu->os, "#[mmu-mtlb: flat-mapping, slot: %d]", mStart);

            gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)Mmu->mtlbPhysical + mStart * 4,
                     Mmu->mtlbLogical[mStart], 4);

            ++stlbIndex;
        } else {
            gctUINT32 mtlbEntry = _ReadPageEntry(Mmu->mtlbLogical + mStart);
            gctUINT   stlbOffset;

            curStlbChunk = (gcsMMU_STLB_CHUNK_PTR)Mmu->staticSTLB;

            _GetCurStlbChunk(Mmu, mStart, &curStlbChunk);

            if (!curStlbChunk)
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

            stlbOffset = mStart - curStlbChunk->mtlbIndex;

            stlbPhyBase = curStlbChunk->physBase + (stlbOffset * gcdMMU_STLB_4K_SIZE);
            stlbLogical = (gctUINT32_PTR)((gctUINT8_PTR)curStlbChunk->logical +
                                          (stlbOffset * gcdMMU_STLB_4K_SIZE));

            if (stlbPhyBase != (mtlbEntry & gcdMMU_MTLB_ENTRY_STLB_MASK))
                gcmkASSERT(0);
        }

#if gcdDUMP_IN_KERNEL
        gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping: 0x%08X - 0x%08X]", start,
                 start + (last - sStart) * gcdMMU_PAGE_4K_SIZE - 1);
#endif

        while (sStart <= last) {
            gcmkASSERT(!(start & gcdMMU_PAGE_4K_MASK));
            if (reserved) {
                /* program NOT_PRESENT | EXCEPTION  for reserved entries */
                _WritePageEntry(stlbLogical + sStart, 1 << 1);
            } else {
                _WritePageEntry(stlbLogical + sStart,
                                _SetPage((gctUINT32)start, physBaseExt, gcvTRUE));
            }
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert STLB[%d]: %08x\n", __FUNCTION__, __LINE__, sStart,
                        _ReadPageEntry(stlbLogical + sStart));
#endif
            /* next page. */
            start += gcdMMU_PAGE_4K_SIZE;
            if ((start & 0xFFFFFFFF) == 0)
                physBaseExt++;

            sStart++;
            curStlbChunk->pageCount++;
        }

#if gcdDUMP_IN_KERNEL
        {
            gctUINT32 i    = sStart;
            gctUINT32 data = stlbLogical[i] & ~0xF;
            gctUINT32 step = (last > i) ? (stlbLogical[i + 1] - stlbLogical[i]) : 0;
            gctUINT32 mask = stlbLogical[i] & 0xF;

            gcmkDUMP(Mmu->os, "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                     (unsigned long long)stlbPhyBase + i * 4, data, (last - i) * 4, step, mask);
        }
#endif

        gcmkONERROR(gckVIDMEM_NODE_CleanCache(kernel, curStlbChunk->videoMem, 0,
                                              curStlbChunk->logical, curStlbChunk->size));

        sStart = 0;
        ++mStart;
    }

    gcmkASSERT(totalNewStlbs == stlbIndex);

    if (newStlbChunk) {
        /* Insert the stlbChunk into staticSTLB. */
        if (Mmu->staticSTLB == gcvNULL) {
            Mmu->staticSTLB = newStlbChunk;
        } else {
            gcmkASSERT(newStlbChunk != gcvNULL);
            gcmkASSERT(newStlbChunk->next == gcvNULL);
            newStlbChunk->next = Mmu->staticSTLB;
            Mmu->staticSTLB    = newStlbChunk;
        }
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    return gcvSTATUS_OK;
OnError:
    /* Roll back the allocation.
     * We don't need roll back mtlb programming as gckmONERROR
     * is only used during allocation time.
     */
    if (newStlbChunk) {
        if (newStlbChunk->videoMem)
            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, newStlbChunk->videoMem));

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, newStlbChunk));
    }
    if (mutex) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }
    return status;
}

static gceSTATUS
_SetupAddressArea(IN gckMMU Mmu, IN gcsADDRESS_AREA_PTR Area,
                  IN gctUINT32 NumMTLBEntries, IN gctBOOL Reserve)
{
    gceSTATUS     status;
    gctUINT32_PTR map;
    gctUINT32     stlbSize = (Area->areaType == gcvAREA_TYPE_1M) ?
                             gcdMMU_STLB_1M_SIZE :
                             gcdMMU_STLB_4K_SIZE;

    gcmkHEADER();
    Area->stlbSize = NumMTLBEntries * stlbSize;

    gcmkSAFECASTSIZET(Area->stlbEntries, Area->stlbSize / gcmSIZEOF(gctUINT32));

    gcmkONERROR(gckOS_Allocate(Mmu->os, Area->stlbSize, (void **)&Area->mapLogical));

    /* Initialization. */
    map             = Area->mapLogical;
    map[0]          = (Area->stlbEntries << 8) | gcvMMU_FREE;
    map[1]          = ~0U;
    Area->heapList  = 0;
    Area->freeNodes = gcvFALSE;

#if gcdENABLE_40BIT_VA
    if (Area->mappingStart == 0 && Reserve) {
        gctUINT32 num = (gctUINT32)(Mmu->reserveRangeSize / gcdMMU_PAGE_4K_SIZE);

        _FillMap(&map[0], num, gcvMMU_USED);
        map[num]       = ((Area->stlbEntries - num) << 8) | gcvMMU_FREE;
        map[num + 1]   = ~0U;
        Area->heapList = num;
    }
#endif
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_ConstructDynamicStlb(IN gckMMU Mmu, IN gcsADDRESS_AREA_PTR Area,
                      IN gctUINT32 NumEntries)
{
    gceSTATUS status    = gcvSTATUS_OK;
    gctBOOL   acquired  = gcvFALSE;
    gckKERNEL kernel    = Mmu->hardware->kernel;
    gctUINT32 allocFlag = gcvALLOC_FLAG_CONTIGUOUS;
    gcePOOL   pool      = Mmu->pool;
    gctUINT32 address;
    gctUINT32 mtlbEntry;
    gctUINT32 i;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
    allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

#if !gcdENABLE_40BIT_VA
    if (!Mmu->pageTableOver4G)
        allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

    /* Construct Slave TLB. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(kernel, 64, gcvVIDMEM_TYPE_COMMAND,
                                              allocFlag | gcvALLOC_FLAG_4K_PAGES,
                                              &Area->stlbSize,
                                              &pool, &Area->stlbVideoMem));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(kernel, Area->stlbVideoMem,
                                       gcvFALSE, gcvFALSE,
                                       (gctPOINTER *)&Area->stlbLogical));

#if gcdUSE_MMU_EXCEPTION
    gcmkONERROR(_FillPageTable(Area->stlbLogical, Area->stlbEntries,
                               /* Enable exception */
                               1 << 1));
#else
    /* Invalidate all entries. */
    gcmkONERROR(gckOS_ZeroMemory(Area->stlbLogical, Area->stlbSize));
#endif

    /* Get stlb table physical. */
    gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(kernel, Area->stlbVideoMem,
                                              0, &Area->stlbPhysical));

    if (Area->areaType == gcvAREA_TYPE_1M) {
        gcmkDUMP(Mmu->os, "#[mmu: 1M page size dynamic space: 0x%llx - 0x%llx]",
                 (Area->mappingStart << gcdMMU_MTLB_SHIFT),
                 (Area->mappingEnd << gcdMMU_MTLB_SHIFT) - 1);
    } else {
        gcmkDUMP(Mmu->os, "#[mmu: 4K page size dynamic space: 0x%llxX - 0x%llx]",
                 (Area->mappingStart << gcdMMU_MTLB_SHIFT),
                 (Area->mappingEnd << gcdMMU_MTLB_SHIFT) - 1);
    }

    gcmkDUMP(Mmu->os, "#[mmu-stlb]");

    gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08lX]",
             (unsigned long long)Area->stlbPhysical,
             Area->stlbLogical[0],
             (unsigned long)Area->stlbSize);

    gcmkSAFECASTPHYSADDRT(address, Area->stlbPhysical);

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Map to Master TLB. */
    for (i = Area->mappingStart; i < Area->mappingStart + NumEntries; i++) {
        if (Area->areaType == gcvAREA_TYPE_1M) {
            mtlbEntry = address
                        /* 1M page size */
                        | (1 << 3)
                        /*Ignore exception */
                        | (0 << 1)
                        /* Present */
                        | (1 << 0);

            address += gcdMMU_STLB_1M_SIZE;
        } else {
            mtlbEntry = address
                        /* 4KB page size */
                        | (0 << 2)
                        /*Ignore exception */
                        | (0 << 1)
                        /* Present */
                        | (1 << 0);

            address += gcdMMU_STLB_4K_SIZE;
        }

        _WritePageEntry(Mmu->mtlbLogical + i, mtlbEntry);

#if gcdMMU_TABLE_DUMP
        gckOS_Print("%s(%d): insert MTLB[%d]: %08x\n",
                    __FUNCTION__, __LINE__,
                    i, _ReadPageEntry(Mmu->mtlbLogical + i));
#endif
    }

    gcmkDUMP(Mmu->os, "#[mmu-mtlb: slot: %d - %d]",
             Area->mappingStart, Area->mappingEnd - 1);

#if gcdDUMP_IN_KERNEL
    {
        gctUINT32 data = Mmu->mtlbLogical[Area->mappingStart] & ~0x3F;
        gctUINT32 step = 0;
        gctUINT32 mask = Mmu->mtlbLogical[Area->mappingStart] & 0x3F;

        if (NumEntries > 1)
            step = Mmu->mtlbLogical[Area->mappingStart + 1] - Mmu->mtlbLogical[Area->mappingStart];

        gcmkDUMP(Mmu->os, "@[physical.step 0x%010llX 0x%08X 0x%08X 0x%08X 0x%08X]",
                 (unsigned long long)(Mmu->mtlbPhysical + Area->mappingStart * 4),
                 data, NumEntries * 4, step, mask);
    }
#endif

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    return status;
}

gceSTATUS
gckMMU_SetupDynamicSpace(IN gckMMU Mmu)
{
    gceSTATUS            status;
    gcsFreeSpaceNode_PTR nodeArray        = gcvNULL;
    gctINT               i, nodeArraySize = 0;
    gctUINT              area4KEntries    = 0;
    gckKERNEL            kernel           = Mmu->hardware->kernel;
    gcsADDRESS_AREA_PTR  area4K           = &Mmu->dynamicArea4K;
#if gcdENABLE_40BIT_VA
    gcsADDRESS_AREA_PTR lowArea4K         = &Mmu->dynamicLowArea4K;
    gctUINT             lowArea4KEntries  = 0;
    gctUINT             indexOf4G;
#if gcdENABLE_GPU_1M_PAGE
    gcsADDRESS_AREA_PTR lowArea1M         = &Mmu->dynamicLowArea1M;
    gctUINT             lowArea1MEntries;
#    endif
#endif
#if gcdENABLE_GPU_1M_PAGE
    gcsADDRESS_AREA_PTR area1M            = &Mmu->dynamicArea1M;
    gctUINT             area1MEntres;
#endif

    /* Find all the free address space. */
    gcmkONERROR(_CollectFreeSpace(Mmu, &nodeArray, &nodeArraySize));

    for (i = 0; i < nodeArraySize; i++) {
        if (nodeArray[i].entries > area4KEntries) {
            area4K->mappingStart = nodeArray[i].start;
            area4KEntries        = nodeArray[i].entries;
            area4K->mappingEnd   = area4K->mappingStart + area4KEntries - 1;
        }
    }

#if gcdENABLE_TRUST_APPLICATION
    if (Mmu->hardware->options.secureMode == gcvSECURE_IN_TA) {
        /* Setup secure address area when needed. */
        gctUINT32           secureAreaSize = gcdMMU_SECURE_AREA_SIZE;
        gcsADDRESS_AREA_PTR secureArea     = &Mmu->secureArea;

        gcmkASSERT(area4KEntries > (gctINT)secureAreaSize);

        secureArea->mappingStart = area4K->mappingStart + (area4KEntries - secureAreaSize);

        gcmkONERROR(_SetupAddressArea(Mmu, secureArea, secureAreaSize, gcvFALSE));

        area4KEntries -= secureAreaSize;
        area4K->mappingEnd -= secureAreaSize;
    }
#endif

#if gcdENABLE_40BIT_VA
    indexOf4G = gcd4G_SIZE / (1ULL << gcdMMU_MTLB_SHIFT);
    if (area4K->mappingStart < indexOf4G && area4K->mappingEnd >= indexOf4G) {
        lowArea4K->mappingStart = area4K->mappingStart;
        lowArea4K->areaType     = gcvAREA_TYPE_4K;
        lowArea4KEntries        = indexOf4G - lowArea4K->mappingStart;
        lowArea4K->mappingEnd   = lowArea4K->mappingStart + lowArea4KEntries - 1;
        area4K->mappingStart    = indexOf4G;
        area4KEntries -= lowArea4KEntries;
        area4K->mappingEnd = area4K->mappingStart + area4KEntries - 1;
    } else if (area4K->mappingEnd < indexOf4G) {
        lowArea4K->mappingStart = area4K->mappingStart;
        lowArea4K->areaType     = gcvAREA_TYPE_4K;
        lowArea4K->mappingEnd   = area4K->mappingEnd;
        lowArea4KEntries        = area4KEntries;
        area4KEntries           = 0;
    } else if (area4K->mappingStart >= indexOf4G) {
        for (i = 0; i < nodeArraySize; i++) {
            if (nodeArray[i].start + nodeArray[i].entries - 1 < indexOf4G) {
                lowArea4K->mappingStart = nodeArray[i].start;
                lowArea4KEntries = nodeArray[i].entries;
                lowArea4K->mappingEnd = area4K->mappingStart + area4KEntries - 1;
                break;
            }
        }
    }

#if gcdENABLE_GPU_1M_PAGE
    if (lowArea4KEntries > 1) {
        lowArea1MEntries        = lowArea4KEntries >> 1;
        lowArea1M->mappingStart = lowArea4K->mappingStart + (lowArea4KEntries - lowArea1MEntries);
        lowArea1M->mappingEnd   = lowArea1M->mappingStart + lowArea1MEntries - 1;
        lowArea1M->areaType     = gcvAREA_TYPE_1M;
        lowArea4KEntries -= lowArea1MEntries;
        lowArea4K->mappingEnd -= lowArea1MEntries;

        gcmkONERROR(_SetupAddressArea(Mmu, lowArea1M, lowArea1MEntries, gcvFALSE));

        gcmkONERROR(_ConstructDynamicStlb(Mmu, lowArea1M, lowArea1MEntries));
    }
#    endif

    if (lowArea4KEntries > 0) {
        gcmkONERROR(_SetupAddressArea(Mmu, lowArea4K, lowArea4KEntries, gcvTRUE));

        gcmkONERROR(_ConstructDynamicStlb(Mmu, lowArea4K, lowArea4KEntries));
    }
#endif

    if (area4KEntries > 0) {
#if gcdENABLE_GPU_1M_PAGE
        area1MEntres         = area4KEntries >> 1;
        area1M->mappingStart = area4K->mappingStart + (area4KEntries - area1MEntres);
        area1M->mappingEnd   = area1M->mappingStart + area1MEntres - 1;
        area1M->areaType     = gcvAREA_TYPE_1M;
        area4KEntries -= area1MEntres;
        area4K->mappingEnd -= area1MEntres;

        gcmkONERROR(_SetupAddressArea(Mmu, area1M, area1MEntres, gcvFALSE));

        gcmkONERROR(_ConstructDynamicStlb(Mmu, area1M, area1MEntres));
#endif

        area4K->areaType = gcvAREA_TYPE_4K;

        gcmkONERROR(_SetupAddressArea(Mmu, area4K, area4KEntries, gcvFALSE));

        gcmkONERROR(_ConstructDynamicStlb(Mmu, area4K, area4KEntries));
    }

    gcmkVERIFY_OK(gckOS_Free(Mmu->os, (gctPOINTER)nodeArray));

    return gcvSTATUS_OK;

OnError:
#if gcdENABLE_GPU_1M_PAGE
    if (area1M->mapLogical)
        gcmkVERIFY_OK(gckOS_Free(Mmu->os, (gctPOINTER)area1M->mapLogical));

    if (area1M->stlbVideoMem)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, area1M->stlbVideoMem));
#endif

    if (area4K->mapLogical)
        gcmkVERIFY_OK(gckOS_Free(Mmu->os, (gctPOINTER)area4K->mapLogical));

    if (area4K->stlbVideoMem)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, area4K->stlbVideoMem));

    if (nodeArray)
        gcmkVERIFY_OK(gckOS_Free(Mmu->os, (gctPOINTER)nodeArray));

    return status;
}

gctUINT32
_GetPageCountOfUsedNode(gctUINT32_PTR Node)
{
    gctUINT32 count;

    count = gcmENTRY_COUNT(*Node);

    if ((count << 8) == (~((1U << 8) - 1)))
        count = 1;

    return count;
}

static gcsADDRESS_AREA_PTR
_GetProcessArea(IN gckMMU Mmu, IN gcePAGE_TYPE PageType,
                IN gctBOOL LowVA, IN gctBOOL Secure)
{
#if gcdENABLE_TRUST_APPLICATION
    if (Secure == gcvTRUE)
        return &Mmu->secureArea;
#endif

    if (PageType == gcvPAGE_TYPE_1M) {
#if gcdENABLE_40BIT_VA
        if (LowVA)
            return &Mmu->dynamicLowArea1M;
        else
#endif
            return &Mmu->dynamicArea1M;
    } else {
#if gcdENABLE_40BIT_VA
        if (LowVA)
            return &Mmu->dynamicLowArea4K;
        else
#endif
            return &Mmu->dynamicArea4K;
    }
}

/*******************************************************************************
 **
 **  _Construct
 **
 **  Construct a new gckMMU object.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gctSIZE_T MmuSize
 **          Number of bytes for the page table.
 **
 **  OUTPUT:
 **
 **      gckMMU *Mmu
 **          Pointer to a variable that receives the gckMMU object pointer.
 */
gceSTATUS
_Construct(IN gckKERNEL Kernel, IN gctSIZE_T MmuSize, OUT gckMMU *Mmu)
{
    gckOS               os;
    gckHARDWARE         hardware;
    gceSTATUS           status;
    gckMMU              mmu                   = gcvNULL;
    gctUINT32_PTR       map;
    gctPOINTER          pointer               = gcvNULL;
    gctPHYS_ADDR_T      physBase;
    gctSIZE_T           physSize;
    gctPHYS_ADDR_T      contiguousBase;
    gctADDRESS          contiguousBaseAddress = 0;
    gctSIZE_T           contiguousSize        = 0;
    gctADDRESS          gpuAddress            = 0;
    gctPHYS_ADDR_T      gpuPhysical;
    gcsADDRESS_AREA_PTR area                  = gcvNULL;
    gcePOOL             pool;
    gctUINT64           data;
    gctUINT32           allocFlag             = gcvALLOC_FLAG_CONTIGUOUS;
    gctUINT64           mmuEnabled;
    gckDEVICE           device;

    gcmkHEADER_ARG("Kernel=%p MmuSize=%lu", Kernel, MmuSize);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(MmuSize > 0);
    gcmkVERIFY_ARGUMENT(Mmu != gcvNULL);

    /* Extract the gckOS object pointer. */
    os = Kernel->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Extract the gckHARDWARE object pointer. */
    hardware = Kernel->hardware;
    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    device = Kernel->device;

    /* Allocate memory for the gckMMU object. */
    gcmkONERROR(gckOS_Allocate(os, sizeof(struct _gckMMU), &pointer));

    gckOS_ZeroMemory(pointer, sizeof(struct _gckMMU));

    mmu = pointer;

    /* Initialize the gckMMU object. */
    mmu->object.type     = gcvOBJ_MMU;
    mmu->os              = os;
    mmu->hardware        = hardware;
    mmu->pageTableMutex  = gcvNULL;
    mmu->mtlbLogical     = gcvNULL;
    mmu->staticSTLB      = gcvNULL;
    mmu->enabled         = gcvFALSE;
    mmu->initMode        = gcvMMU_INIT_FROM_CMD;
    mmu->pageTableOver4G = gcvFALSE;

    mmu->dynamicAreaSetuped = gcvFALSE;
    mmu->pool               = _GetPageTablePool(mmu->os);
    gcsLIST_Init(&mmu->hardwareList);

    /* Use 4K page size for MMU version 0. */
    area              = &mmu->dynamicArea4K;
    area->mapLogical  = gcvNULL;
    area->stlbLogical = gcvNULL;

    /* Create the page table mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &mmu->pageTableMutex));

    gcmkONERROR(gckOS_QueryOption(os, "mmu", &mmuEnabled));

    mmu->flatMappingMode = gcdFLAT_MAPPING_MODE;

    if (hardware->mmuVersion == 0) {
        area->stlbSize = MmuSize;

        /* Construct address space management table. */
        gcmkONERROR(gckOS_Allocate(mmu->os, area->stlbSize, &pointer));

        area->mapLogical = pointer;

        pool = mmu->pool;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

        /* Construct page table read by GPU. */
        gcmkONERROR(gckKERNEL_AllocateVideoMemory(Kernel, 4096, gcvVIDMEM_TYPE_COMMAND, allocFlag,
                                                  &area->stlbSize, &pool, &area->stlbVideoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(Kernel, area->stlbVideoMem,
                                           gcvFALSE, gcvFALSE, &pointer));

        area->stlbLogical = pointer;

        /* Get GPU physical address. */
        gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(Kernel,
                                                  area->stlbVideoMem, 0,
                                                  &area->stlbPhysical));

        /* Compute number of entries in page table. */
        gcmkSAFECASTSIZET(area->stlbEntries, area->stlbSize / sizeof(gctUINT32));

        /* Mark all pages as free. */
        map = area->mapLogical;

        _FillPageTable(area->stlbLogical, area->stlbEntries, (gctUINT32)mmu->safeAddress);

        gcmkDUMP(mmu->os, "#[mmu0: fill with safe address]");

        gcmkDUMP(mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08lX]",
                 (unsigned long long)area->stlbPhysical,
                 area->stlbLogical[0],
                 (unsigned long)area->stlbSize);

        map[0]          = (area->stlbEntries << 8) | gcvMMU_FREE;
        map[1]          = ~0U;
        area->heapList  = 0;
        area->freeNodes = gcvFALSE;

        contiguousBase = device->contiguousBases[0];
        contiguousSize = device->contiguousSizes[0];

        if (!contiguousSize) {
            status = gckOS_QueryOption(mmu->os, "contiguousBase", &contiguousBase);

            if (gcmIS_SUCCESS(status)) {
                status = gckOS_QueryOption(mmu->os, "contiguousSize", &data);
                contiguousSize = (gctSIZE_T)data;
            }
        }

        if (contiguousSize) {
            /* Convert to GPU address. */
            mmu->contiguousBaseAddresses[0] =
                (gctUINT32)(contiguousBase - Kernel->hardware->baseAddress);
        }
    } else {
        gctPHYS_ADDR_T gpuContiguousBase = gcvINVALID_PHYSICAL_ADDRESS;
        gctPHYS_ADDR_T gpuExternalBase   = gcvINVALID_PHYSICAL_ADDRESS;
        int i = 0;

        mmu->mtlbSize = gcdMMU_MTLB_SIZE;

        if (device->externalSize)
            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os,
                                                       device->externalBase,
                                                       &gpuExternalBase));

        contiguousBase = device->contiguousBases[0];
        contiguousSize = device->contiguousSizes[0];

        if (!contiguousSize) {
            status = gckOS_QueryOption(mmu->os, "contiguousBase", &contiguousBase);

            if (gcmIS_SUCCESS(status)) {
                status = gckOS_QueryOption(mmu->os, "contiguousSize", &data);
                contiguousSize = (gctSIZE_T)data;
            }
        }

        if (!device->lowContiguousSize) {
            data = 0;

            gckOS_QueryOption(mmu->os, "lowContiguousSize", &data);
            if (data) {
                contiguousSize += (gctSIZE_T)data;

                status = gckOS_QueryOption(mmu->os, "lowContiguousBase", &data);
                if (gcmIS_SUCCESS(status))
                    contiguousBase = (gctADDRESS)data;
            }
        } else {
            contiguousSize += device->lowContiguousSize;
            contiguousBase = device->lowContiguousBase;
        }

        if (contiguousSize)
            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os, contiguousBase,
                                                       &gpuContiguousBase));

        pool = mmu->pool;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag |= gcvALLOC_FLAG_CACHEABLE;
#endif

#if !gcdENABLE_40BIT_VA
        if (!mmu->pageTableOver4G)
            allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

        /*
         * mtlb address requires 256 alignment in 4K mode and 1024 alignment in 1K mode (Descriptor only use bit31 ~ bit10 for 1K mode).
         * stlb address is always 64 byte alignment (mtlb entry uses bit31 ~ bit6 for aligned stlb address).
         */
        gcmkONERROR(gckKERNEL_AllocateVideoMemory(Kernel, 1024, gcvVIDMEM_TYPE_COMMAND,
                                                  allocFlag | gcvALLOC_FLAG_4K_PAGES,
                                                  &mmu->mtlbSize, &pool, &mmu->mtlbVideoMem));

        /* Lock for kernel side CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(Kernel, mmu->mtlbVideoMem,
                                           gcvFALSE, gcvFALSE, &pointer));

        mmu->mtlbLogical = pointer;

        mmu->dynamicArea4K.mappingStart    = gcvINVALID_VALUE;
        mmu->dynamicArea1M.mappingStart    = gcvINVALID_VALUE;
        mmu->dynamicLowArea4K.mappingStart = gcvINVALID_VALUE;
        mmu->dynamicLowArea1M.mappingStart = gcvINVALID_VALUE;

        /* Get mtlb table physical. */
        gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(Kernel, mmu->mtlbVideoMem,
                                                  0, &mmu->mtlbPhysical));

        /* Invalid all the entries. */
        gcmkONERROR(gckOS_ZeroMemory(pointer, mmu->mtlbSize));

        gcmkONERROR(gckOS_QueryOption(mmu->os, "physBase", &physBase));

        gcmkONERROR(gckOS_QueryOption(mmu->os, "physSize", &data));

        physSize = (gctSIZE_T)data;

        if (physSize) {
#ifdef MSDX
            gctUINT64 wddmMode = 0;
#endif
            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os, physBase, &gpuPhysical));

#if gcdENABLE_40BIT_VA
            gpuAddress = (gctADDRESS)gpuPhysical;
#else
            gcmkSAFECASTPHYSADDRT(gpuAddress, gpuPhysical);
#endif

            if (gpuAddress < mmu->reserveRangeSize) {

                if (gpuAddress + physSize <= mmu->reserveRangeSize)
                    gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

                gcmkPRINT("Galcore warning: pre-flat mapping base address can't be lower than 0x1000000, adjust it to 0x1000000. ");

                physSize = (gctUINT32)gpuAddress + physSize - mmu->reserveRangeSize;

                gpuAddress = (gctADDRESS)mmu->reserveRangeSize;
            }

#ifdef MSDX
            if ((gcvSTATUS_OK != gckOS_QueryOption(mmu->os, "wddmMode", &wddmMode)) || !wddmMode)
#endif
                /* Setup user specified flat mapping. */
                gcmkONERROR(gckMMU_FillFlatMapping(mmu, gpuAddress, physSize, gcvFALSE, gcvFALSE, gcvNULL));
        }

#if !(0 || gcdCAPTURE_ONLY_MODE)
        if (!_ReadPageEntry(mmu->mtlbLogical + 0)) {
#if !gcdENABLE_40BIT_VA
            gctUINT32 mtlbEntry;
            /*
             * Reserved the first mtlb.
             * 1MB page size, Ignore exception, Not Present.
             */
            mtlbEntry = (1 << 3) | (0 << 1) | (0 << 0);

            _WritePageEntry(mmu->mtlbLogical + 0, mtlbEntry);

            gcmkDUMP(mmu->os, "#[mmu-mtlb: reserved 16M space, slot: 0]");
            gcmkDUMP(mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)mmu->mtlbPhysical, mmu->mtlbLogical[0], 4);
#    endif

            /* Store the gpu virtual ranges */
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].start = 0;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].end   = gcdVA_RESERVED_SIZE - 1;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].size  = gcdVA_RESERVED_SIZE;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].flag  = gcvFLATMAP_DIRECT;
            mmu->gpuAddressRangeCount++;
            mmu->reserveRangeSize = gcdVA_RESERVED_SIZE;
        }
#elif defined(VSIMULATOR_DEBUG)
        if (!_ReadPageEntry(mmu->mtlbLogical + 0)) {
#if !gcdENABLE_40BIT_VA
            gctUINT32 mtlbEntry;
            /*
             * Reserved the first mtlb.
             * 1MB page size, Ignore exception, Not Present.
             */
            mtlbEntry = (1 << 3) | (0 << 1) | (0 << 0);

            _WritePageEntry(mmu->mtlbLogical + 0, mtlbEntry);

            gcmkDUMP(mmu->os, "#[mmu-mtlb: reserved 16M space, slot: 0]");
            gcmkDUMP(mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
                     (unsigned long long)mmu->mtlbPhysical, mmu->mtlbLogical[0], 4);
#    endif

            /* Store the gpu virtual ranges */
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].start = 0;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].end   = gcdVA_RESERVED_SIZE - 1;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].size  = gcdVA_RESERVED_SIZE;
            mmu->gpuAddressRanges[mmu->gpuAddressRangeCount].flag  = gcvFLATMAP_DIRECT;
            mmu->gpuAddressRangeCount++;
            mmu->reserveRangeSize = gcdVA_RESERVED_SIZE;
        }
#endif

        gcmkONERROR(gckMMU_SetupSRAM(mmu, mmu->hardware, Kernel->device));

        for (i = 0; i < gcdSYSTEM_RESERVE_COUNT; i++) {
            if (i) {
                contiguousSize = device->contiguousSizes[i];

                if (contiguousSize)
                    gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os,
                                                               device->contiguousBases[i],
                                                               &gpuContiguousBase));
            } else {
#if gcdENABLE_40BIT_VA
                if (contiguousBase > gcd4G_SIZE - gcd4G_VA_FM_SIZE)
                    contiguousBaseAddress = gcd4G_SIZE - gcd4G_VA_FM_SIZE;
#endif
            }

            if (contiguousSize && gpuContiguousBase != gcvINVALID_PHYSICAL_ADDRESS) {
                /* Setup flat mapping for reserved memory (VIDMEM). */
                gcmkONERROR(gckMMU_FillFlatMapping(mmu, gpuContiguousBase, contiguousSize,
                                                   gcvFALSE, gcvTRUE, &contiguousBaseAddress));

                if (mmuEnabled)
                    mmu->contiguousBaseAddresses[i] = contiguousBaseAddress;
                else
                    gcmkSAFECASTPHYSADDRT(mmu->contiguousBaseAddresses[i], gpuContiguousBase);

                if (device->showMemInfo)
                    gcmkPRINT("[Galcore]: system reserved pool%d CPU physical=0x%llx GPU physical=0x%llx virtual=0x%llx size=0x%llx",
                              i, contiguousBase, gpuContiguousBase, mmu->contiguousBaseAddresses[i], (gctUINT64)contiguousSize);

            }
        }

        if (Kernel->device->externalSize && gpuExternalBase != gcvINVALID_PHYSICAL_ADDRESS) {
            gctADDRESS externalBaseAddress = 0;

            /* Setup flat mapping for external memory. */
            gcmkONERROR(gckMMU_FillFlatMapping(mmu, gpuExternalBase,
                                               Kernel->device->externalSize,
                                               gcvFALSE, gcvTRUE, &externalBaseAddress));

            mmu->externalBaseAddress = externalBaseAddress;

            if (device->showMemInfo)
                gcmkPRINT("[Galcore]: device%d external pool CPU physical=0x%llx GPU physical=0x%llx virtual=0x%llx size=0x%zx",
                           device->id, device->externalBase, gpuExternalBase, externalBaseAddress, device->externalSize);
        }

        if (Kernel->device->exclusiveSize) {
            gctUINT64  gpuExclusiveBase;
            gctADDRESS exclusiveBaseAddress = 0;

            gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(mmu->os,
                                                       Kernel->device->exclusiveBase,
                                                       &gpuExclusiveBase));

            /* Setup flat mapping for external memory. */
            gcmkONERROR(gckMMU_FillFlatMapping(mmu, gpuExclusiveBase,
                                               Kernel->device->exclusiveSize,
                                               gcvFALSE, gcvTRUE, &exclusiveBaseAddress));

            mmu->exclusiveBaseAddress = exclusiveBaseAddress;

            if (device->showMemInfo)
                gcmkPRINT("[Galcore]: device%d exclusive pool CPU physical=0x%llx GPU physical=0x%llx virtual=0x%llx size=0x%zx",
                           device->id, device->exclusiveBase, gpuExclusiveBase, exclusiveBaseAddress, device->exclusiveSize);
        }

        device->showMemInfo = gcvFALSE;
    }

    /* A 64 byte for safe address, we use 256 here. */
    mmu->safePageSize = 256;

    pool = mmu->pool;

    allocFlag = gcvALLOC_FLAG_CONTIGUOUS | gcvALLOC_FLAG_4K_PAGES;

#if !gcdENABLE_40BIT_VA
    if (!mmu->pageTableOver4G)
        allocFlag |= gcvALLOC_FLAG_4GB_ADDR;
#endif

    /* Allocate safe page from video memory. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(Kernel, 256, gcvVIDMEM_TYPE_COMMAND, allocFlag,
                                              &mmu->safePageSize, &pool, &mmu->safePageVideoMem));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(Kernel, mmu->safePageVideoMem,
                                       gcvFALSE, gcvFALSE,
                                       &mmu->safePageLogical));

    /* Get GPU physical address. */
    gcmkONERROR(gckVIDMEM_NODE_GetGPUPhysical(Kernel, mmu->safePageVideoMem,
                                              0, &mmu->safePagePhysical));

    gcmkSAFECASTPHYSADDRT(mmu->safeAddress, mmu->safePagePhysical);

    gckOS_ZeroMemory(mmu->safePageLogical, mmu->safePageSize);

    gcmkDUMP(mmu->os, "#[safe page]");
    gcmkDUMP(mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08lX]",
             (unsigned long long)mmu->safePagePhysical, 0, (unsigned long)mmu->safePageSize);

    gcmkDUMP_BUFFER(mmu->os, gcvDUMP_BUFFER_KERNEL_COMMAND,
                    mmu->safePageLogical, mmu->safeAddress, mmu->safePageSize);

    gcmkONERROR(gckQUEUE_Allocate(os, &mmu->recentFreedAddresses, 16));

    mmu->sRAMMapped = gcvFALSE;

    /* Return the gckMMU object pointer. */
    *Mmu = mmu;

    /* Success. */
    gcmkFOOTER_ARG("*Mmu=%p", *Mmu);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (mmu != gcvNULL) {
        if (area != gcvNULL && area->mapLogical != gcvNULL) {
            gcmkVERIFY_OK(gckOS_Free(os, (gctPOINTER)area->mapLogical));

            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, area->stlbVideoMem));
        }

        if (mmu->mtlbLogical != gcvNULL)
            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, mmu->mtlbVideoMem));

        if (mmu->pageTableMutex != gcvNULL) {
            /* Delete the mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(os, mmu->pageTableMutex));
        }

        gcmkVERIFY_OK(gckQUEUE_Free(os, &mmu->recentFreedAddresses));

        /* Mark the gckMMU object as unknown. */
        mmu->object.type = gcvOBJ_UNKNOWN;

        /* Free the allocates memory. */
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, mmu));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_FreeAddressArea(gckKERNEL Kernel, gcsADDRESS_AREA *Area)
{
    gceSTATUS status = gcvSTATUS_OK;

    if (Area->mapLogical != gcvNULL)
        gcmkVERIFY_OK(gckOS_Free(Kernel->os, (gctPOINTER)Area->mapLogical));

    if (Area->stlbLogical != gcvNULL) {
        /* Free page table. */
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, Area->stlbVideoMem));
    }

    return status;
}

/*******************************************************************************
 **
 **  _Destroy
 **
 **  Destroy a gckMMU object.
 **
 **  INPUT:
 **
 **      gckMMU Mmu
 **          Pointer to an gckMMU object.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
_Destroy(IN gckMMU Mmu)
{
    gckKERNEL kernel = Mmu->hardware->kernel;

    gcmkHEADER_ARG("Mmu=%p", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    while (Mmu->staticSTLB != gcvNULL) {
        gcsMMU_STLB_CHUNK_PTR pre = Mmu->staticSTLB;

        Mmu->staticSTLB           = pre->next;

        if (pre->videoMem)
            gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, pre->videoMem));

        if (pre->mtlbEntryNum != 0) {
            gctUINT i;

            for (i = 0; i < pre->mtlbEntryNum; ++i) {
                _WritePageEntry(Mmu->mtlbLogical + pre->mtlbIndex + i, 0);
#if gcdMMU_TABLE_DUMP
                gckOS_Print("%s(%d): clean MTLB[%d]\n",
                            __FUNCTION__, __LINE__, pre->mtlbIndex + i);
#endif
            }

            gcmkDUMP(Mmu->os, "#[mmu-mtlb: clean up slot: %d - %d]", pre->mtlbIndex,
                     pre->mtlbIndex + pre->mtlbEntryNum - 1);

            gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08lX]",
                     (unsigned long long)(Mmu->mtlbPhysical + pre->mtlbIndex * 4),
                     Mmu->mtlbLogical[pre->mtlbIndex],
                     (unsigned long)(pre->mtlbEntryNum * 4));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, pre));
    }

    if (Mmu->hardware->mmuVersion != 0)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, Mmu->mtlbVideoMem));

    /* Free address area. */
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->dynamicArea4K));
#if gcdENABLE_GPU_1M_PAGE
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->dynamicArea1M));
#endif
#if gcdENABLE_40BIT_VA
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->dynamicLowArea4K));
#if gcdENABLE_GPU_1M_PAGE
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->dynamicLowArea1M));
#    endif
#endif
    gcmkVERIFY_OK(_FreeAddressArea(kernel, &Mmu->secureArea));

    /* Delete the page table mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Mmu->os, Mmu->pageTableMutex));

    if (Mmu->safePageLogical != gcvNULL)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(kernel, Mmu->safePageVideoMem));

    gcmkVERIFY_OK(gckQUEUE_Free(Mmu->os, &Mmu->recentFreedAddresses));

    /* Mark the gckMMU object as unknown. */
    Mmu->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckMMU object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Mmu->os, Mmu));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
 ** _AdjstIndex
 **
 **  Adjust the index from which we search for a usable node to make sure
 **  index allocated is greater than Start.
 */
gceSTATUS
_AdjustIndex(IN gckMMU Mmu, IN gctUINT32 Index, IN gctUINT32 PageCount,
             IN gctUINT32 Start, OUT gctUINT32 *IndexAdjusted)
{
    gceSTATUS           status;
    gctUINT32           index = Index;
    gcsADDRESS_AREA_PTR area  = &Mmu->dynamicArea4K;
    gctUINT32_PTR       map   = area->mapLogical;

    gcmkHEADER();

    for (; index < area->stlbEntries;) {
        gctUINT32 result   = 0;
        gctUINT32 nodeSize = 0;

        if (index >= Start)
            break;

        switch (gcmENTRY_TYPE(map[index])) {
        case gcvMMU_SINGLE:
            nodeSize = 1;
            break;

        case gcvMMU_FREE:
            nodeSize = map[index] >> 8;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", index);
            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }

        if (nodeSize > PageCount) {
            result = index + (nodeSize - PageCount);

            if (result >= Start)
                break;
        }

        switch (gcmENTRY_TYPE(map[index])) {
        case gcvMMU_SINGLE:
            index = map[index] >> 8;
            break;

        case gcvMMU_FREE:
            index = map[index + 1];
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", index);
            gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
        }
    }

    *IndexAdjusted = index;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_Construct(IN gckKERNEL Kernel, IN gctSIZE_T MmuSize, OUT gckMMU *Mmu)
{
    return _Construct(Kernel, MmuSize, Mmu);
}

gceSTATUS
gckMMU_Destroy(IN gckMMU Mmu)
{
    return _Destroy(Mmu);
}

/*******************************************************************************
 **
 **  gckMMU_AllocatePages
 **
 **  Allocate pages inside the page table.
 **
 **  INPUT:
 **
 **      gckMMU Mmu
 **          Pointer to an gckMMU object.
 **
 **      gctSIZE_T PageCount
 **          Number of pages to allocate.
 **
 **  OUTPUT:
 **
 **      gctPOINTER *PageTable
 **          Pointer to a variable that receives the base address of the page
 **          table.
 **
 **      gctADDRESS *Address
 **          Pointer to a variable that receives the hardware specific address.
 */
gceSTATUS
_AllocatePages(IN gckMMU Mmu, IN gctSIZE_T PageCount, IN gceVIDMEM_TYPE Type,
               IN gcePAGE_TYPE PageType, IN gctBOOL LowVA, IN gctBOOL Secure,
               OUT gctPOINTER *PageTable, OUT gctADDRESS *Address)
{
    gceSTATUS           status;
    gctBOOL             mutex = gcvFALSE;
    gctUINT32           index = 0, previous = ~0U, left;
    gctUINT32_PTR       map;
    gctBOOL             gotIt;
    gctADDRESS          address;
    gctUINT32           pageCount;
    gcsADDRESS_AREA_PTR area = _GetProcessArea(Mmu, PageType, LowVA, Secure);

    gcmkHEADER_ARG("Mmu=%p PageCount=%lu", Mmu, PageCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageCount > 0);
    gcmkVERIFY_ARGUMENT(PageTable != gcvNULL);

    if (PageCount > area->stlbEntries) {
        /* Not enough pages available. */
        gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
    }

    gcmkSAFECASTSIZET(pageCount, PageCount);

#if gcdBOUNDARY_CHECK
    /* Extra pages as bounary. */
    pageCount += gcdBOUNDARY_CHECK * 2;
#endif

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    mutex = gcvTRUE;

    /* Cast pointer to page table. */
    for (map = area->mapLogical, gotIt = gcvFALSE; !gotIt;) {
        index = area->heapList;

        if (Mmu->hardware->mmuVersion == 0 &&
            Type == gcvVIDMEM_TYPE_VERTEX_BUFFER) {
            gcmkONERROR(_AdjustIndex(Mmu, index, pageCount,
                                     gcdVERTEX_START / gcmSIZEOF(gctUINT32),
                                     &index));
        }

        /* Walk the heap list. */
        for (; !gotIt && (index < area->stlbEntries);) {
            /* Check the node type. */
            switch (gcmENTRY_TYPE(map[index])) {
            case gcvMMU_SINGLE:
                /* Single odes are valid if we only need 1 page. */
                if (pageCount == 1) {
                    gotIt = gcvTRUE;
                } else {
                    /* Move to next node. */
                    previous = index;
                    index    = map[index] >> 8;
                }
                break;

            case gcvMMU_FREE:
                /* Test if the node has enough space. */
                if (pageCount <= (map[index] >> 8)) {
                    gotIt = gcvTRUE;
                } else {
                    /* Move to next node. */
                    previous = index;
                    index    = map[index + 1];
                }
                break;

            default:
                gcmkFATAL("MMU table correcupted at index %u!", index);
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }

        /* Test if we are out of memory. */
        if (index >= area->stlbEntries) {
            if (area->freeNodes) {
                /* Time to move out the trash! */
                gcmkONERROR(_Collect(area));

                /* We are going to search from start, so reset previous to start. */
                previous = ~0U;
            } else {
                /* Out of resources. */
                gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
            }
        }
    }

    switch (gcmENTRY_TYPE(map[index])) {
    case gcvMMU_SINGLE:
        /* Unlink single node from free list. */
        gcmkONERROR(_Link(area, previous, map[index] >> 8));
        break;

    case gcvMMU_FREE:
        /* Check how many pages will be left. */
        left = (map[index] >> 8) - pageCount;
        switch (left) {
        case 0:
            /* The entire node is consumed, just unlink it. */
            gcmkONERROR(_Link(area, previous, map[index + 1]));
            break;

        case 1:
            /*
             * One page will remain.  Convert the node to a single node and
             * advance the index.
             */
            map[index] = (map[index + 1] << 8) | gcvMMU_SINGLE;
            index++;
            break;

        default:
            /*
             * Enough pages remain for a new node.  However, we will just adjust
             * the size of the current node and advance the index.
             */
            map[index] = (left << 8) | gcvMMU_FREE;
            index += left;
            break;
        }
        break;
    }

    /* Mark node as used. */
    gcmkONERROR(_FillMap(&map[index], pageCount, gcvMMU_USED));

#if gcdBOUNDARY_CHECK
    index += gcdBOUNDARY_CHECK;
#endif

    /* Record pageCount of allocated node at the beginning of node. */
    if (pageCount == 1)
        map[index] = (~((1U << 8) - 1)) | gcvMMU_USED;
    else
        map[index] = (pageCount << 8) | gcvMMU_USED;

    if (area->stlbLogical != gcvNULL) {
        /* Return pointer to page table. */
        *PageTable = &area->stlbLogical[index];
    } else {
        /* Page table for secure area is handled in trust application. */
        *PageTable = gcvNULL;
    }

    /* Build virtual address. */
    if (Mmu->hardware->mmuVersion == 0) {
        gctUINT32 low;

        gcmkONERROR(gckHARDWARE_BuildVirtualAddress(Mmu->hardware, index, 0, &low));

        address = low;
    } else {
        gctUINT32 num   = (PageType == gcvPAGE_TYPE_1M) ?
                          gcdMMU_STLB_1M_ENTRY_NUM :
                          gcdMMU_STLB_4K_ENTRY_NUM;
        gctUINT32 shift = (PageType == gcvPAGE_TYPE_1M) ?
                          gcdMMU_STLB_1M_SHIFT :
                          gcdMMU_STLB_4K_SHIFT;
        gctUINT32 masterOffset = index / num + area->mappingStart;
        gctUINT32 slaveOffset  = index % num;

        address = ((gctADDRESS)masterOffset << gcdMMU_MTLB_SHIFT) | (slaveOffset << shift);
    }

    if (Address != gcvNULL)
        *Address = address;

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    /* Success. */
    gcmkFOOTER_ARG("*PageTable=%p *Address=%08x", *PageTable, gcmOPT_VALUE(Address));
    return gcvSTATUS_OK;

OnError:

    if (mutex) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckMMU_FreePages
 **
 **  Free pages inside the page table.
 **
 **  INPUT:
 **
 **      gckMMU Mmu
 **          Pointer to an gckMMU object.
 **
 **      gctPOINTER PageTable
 **          Base address of the page table to free.
 **
 **      gctSIZE_T PageCount
 **          Number of pages to free.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
_FreePages(IN gckMMU Mmu, IN gctBOOL Secure, IN gcePAGE_TYPE PageType,
           IN gctBOOL LowVA, IN gctADDRESS Address,
           IN gctPOINTER PageTable, IN gctSIZE_T PageCount)
{
    gctUINT32           index;
    gctUINT32_PTR       node;
    gceSTATUS           status;
    gctBOOL             acquired = gcvFALSE;
    gctUINT32           pageCount;
    gcuQUEUEDATA        data;
    gcsADDRESS_AREA_PTR area = _GetProcessArea(Mmu, PageType, LowVA, gcvFALSE);
    gctUINT32 pageSize = (PageType == gcvPAGE_TYPE_1M) ?
                         gcdMMU_PAGE_1M_SIZE : gcdMMU_PAGE_4K_SIZE;

    gcmkHEADER_ARG("Mmu=%p PageTable=%p PageCount=%lu",
                   Mmu, PageTable, PageCount);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageCount > 0);

    gcmkSAFECASTSIZET(pageCount, PageCount);

#if gcdBOUNDARY_CHECK
    pageCount += gcdBOUNDARY_CHECK * 2;
#endif

    /* Get the node by index. */
    index = (gctUINT32)((gctUINT32_PTR)PageTable - area->stlbLogical);

    node = area->mapLogical + index;

    if (pageCount != _GetPageCountOfUsedNode(node))
        gcmkONERROR(gcvSTATUS_INVALID_REQUEST);

#if gcdBOUNDARY_CHECK
    node -= gcdBOUNDARY_CHECK;
#endif

    gcmkONERROR(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (Mmu->hardware->mmuVersion == 0)
        _FillPageTable(PageTable, pageCount, (gctUINT32)Mmu->safeAddress);

    if (pageCount == 1) {
        /* Single page node. */
        node[0] = (~((1U << 8) - 1)) | gcvMMU_SINGLE;

        if (PageTable != gcvNULL) {
#if gcdUSE_MMU_EXCEPTION
            /* Enable exception */
            _WritePageEntry(PageTable, (1 << 1));
#else
            _WritePageEntry(PageTable, 0);
#endif
        }
    } else {
        /* Mark the node as free. */
        node[0] = (pageCount << 8) | gcvMMU_FREE;
        node[1] = ~0U;

        if (PageTable != gcvNULL) {
#if gcdUSE_MMU_EXCEPTION
            /* Enable exception */
            gcmkVERIFY_OK(_FillPageTable(PageTable, (gctUINT32)PageCount, 1 << 1));
#else
            gcmkVERIFY_OK(_FillPageTable(PageTable, (gctUINT32)PageCount, 0));
#endif
        }
    }

    gcmkDUMP(Mmu->os, "#[mmu-stlb: free 0x%llx - 0x%llx]",
             Address, Address + pageCount * pageSize - 1);

    gcmkDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 0x%08X]",
             (unsigned long long)(area->stlbPhysical + index * 4),
             *(gctUINT32_PTR)PageTable, pageCount * 4);

    /* We have free nodes. */
    area->freeNodes = gcvTRUE;

    /* Record freed address range. */
    data.addressData.start = Address;
    data.addressData.end   = Address + (gctUINT32)PageCount * pageSize;
    gckQUEUE_Enqueue(&Mmu->recentFreedAddresses, &data);

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
    acquired = gcvFALSE;

#if gcdENABLE_TRUST_APPLICATION
    if (Mmu->hardware->options.secureMode == gcvSECURE_IN_TA) {
        gckKERNEL_SecurityUnmapMemory(Mmu->hardware->kernel,
                                      Address, (gctUINT32)PageCount);
    }
#endif

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_AllocatePages(IN gckMMU Mmu, IN gctSIZE_T PageCount,
                     IN gcePAGE_TYPE PageType,
                     OUT gctPOINTER *PageTable, OUT gctADDRESS *Address)
{
    return gckMMU_AllocatePagesEx(Mmu, PageCount, gcvVIDMEM_TYPE_GENERIC,
                                  PageType, gcvFALSE,
                                  gcvFALSE, PageTable, Address);
}

gceSTATUS
gckMMU_AllocatePagesEx(IN gckMMU Mmu, IN gctSIZE_T PageCount,
                       IN gceVIDMEM_TYPE Type, IN gcePAGE_TYPE PageType,
                       IN gctBOOL LowVA, IN gctBOOL Secure,
                       OUT gctPOINTER *PageTable, OUT gctADDRESS *Address)
{
#if gcdDISABLE_GPU_VIRTUAL_ADDRESS
    gcmkPRINT("GPU virtual address is disabled.");
    return gcvSTATUS_NOT_SUPPORTED;
#else
    return _AllocatePages(Mmu, PageCount, Type, PageType,
                          LowVA, Secure, PageTable, Address);
#endif
}

gceSTATUS
gckMMU_FreePages(IN gckMMU Mmu, IN gctBOOL Secure,
                 IN gcePAGE_TYPE PageType, IN gctBOOL LowVA,
                 IN gctADDRESS Address,
                 IN gctPOINTER PageTable, IN gctSIZE_T PageCount)
{
    return _FreePages(Mmu, Secure, PageType, LowVA,
                      Address, PageTable, PageCount);
}

gceSTATUS
gckMMU_SetPage(IN gckMMU Mmu, IN gctPHYS_ADDR_T PageAddress,
               IN gcePAGE_TYPE PageType, IN gctBOOL LowVA,
               IN gctBOOL Writable, IN gctUINT32 *PageEntry)
{
    gctUINT32 addressExt;
    gctUINT32 address;

    gcmkHEADER_ARG("Mmu=%p", Mmu);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);
    gcmkVERIFY_ARGUMENT(PageEntry != gcvNULL);

    if (PageType == gcvPAGE_TYPE_1M)
        gcmkVERIFY_ARGUMENT(!(PageAddress & 0xFFFFF));
    else
        gcmkVERIFY_ARGUMENT(!(PageAddress & 0xFFF));

    /* [31:0]. */
    address = (gctUINT32)(PageAddress & 0xFFFFFFFF);
    /* [39:32]. */
    addressExt = (gctUINT32)((PageAddress >> 32) & 0xFF);

    if (Mmu->hardware->mmuVersion == 0)
        _WritePageEntry(PageEntry, address);
    else
        _WritePageEntry(PageEntry, _SetPage(address, addressExt, gcvTRUE));

#ifdef DUMP_IN_KERNEL
    {
        gctUINT32          *stlbLogical;
        gctPHYS_ADDR_T      stlbPhysical;
        gctPHYS_ADDR_T      physical;
        gcsADDRESS_AREA_PTR area;

#if gcdENABLE_40BIT_VA
        if (Flag & lowVA) {
            area = (PageType == gcvPAGE_TYPE_1M) ?
                   &Mmu->dynamicLowArea1M : &Mmu->dynamicLowArea4K;
        } else {
            area = (PageType == gcvPAGE_TYPE_1M) ?
                   &Mmu->dynamicArea1M : &Mmu->dynamicArea4K;
        }
#    else
        area = (PageType == gcvPAGE_TYPE_1M) ?
               &Mmu->dynamicArea1M : &Mmu->dynamicArea4K;
#    endif

        stlbLogical  = area->stlbLogical;
        stlbPhysical = area->stlbPhysical;

        physical = stlbPhysical + (stlbLogical - PageEntry) * 4;

        gckDUMP(Mmu->os, "@[physical.fill 0x%010llX 0x%08X 4",
                physical, *PageEntry);
    }
#endif

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_Flush(IN gckMMU Mmu, IN gceVIDMEM_TYPE Type)
{
    gckHARDWARE     hardware;
    gctUINT32       mask;
    gctINT          i;
    gcsLISTHEAD_PTR hardwareHead;

    if (Type == gcvVIDMEM_TYPE_VERTEX_BUFFER ||
        Type == gcvVIDMEM_TYPE_INDEX_BUFFER ||
        Type == gcvVIDMEM_TYPE_COMMAND) {
        mask = gcvPAGE_TABLE_DIRTY_BIT_FE;
    } else {
        mask = gcvPAGE_TABLE_DIRTY_BIT_OTHER;
    }

    hardware = Mmu->hardware;

    for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++)
        gcmkVERIFY_OK(gckOS_AtomSetMask(hardware->pageTableDirty[i], mask));

    gcmkLIST_FOR_EACH(hardwareHead, &Mmu->hardwareList) {
        hardware = gcmCONTAINEROF(hardwareHead, struct _gckHARDWARE, mmuHead);

        if (hardware != Mmu->hardware) {
            for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++) {
                gcmkVERIFY_OK(gckOS_AtomSetMask(hardware->pageTableDirty[i],
                                                mask));
            }
        }
    }

    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_DumpPageTableEntry(IN gckMMU Mmu, IN gceAREA_TYPE AreaType, IN gctADDRESS Address)
{
    gctUINT32_PTR       pageTable;
    gctUINT32           index;
    gctUINT32           mtlb, stlb;
    gcsADDRESS_AREA_PTR area = (AreaType == gcvAREA_TYPE_4K) ?
                               &Mmu->dynamicArea4K : &Mmu->dynamicArea1M;

    gctUINT32 stlbShift = (AreaType == gcvAREA_TYPE_4K) ?
                          gcdMMU_STLB_4K_SHIFT : gcdMMU_STLB_1M_SHIFT;

    gctUINT32 stlbMask = (AreaType == gcvAREA_TYPE_4K) ?
                         gcdMMU_STLB_4K_MASK : gcdMMU_STLB_1M_MASK;

    gctUINT32 stlbEntryNum = (AreaType == gcvAREA_TYPE_4K) ?
                             gcdMMU_STLB_4K_ENTRY_NUM : gcdMMU_STLB_1M_ENTRY_NUM;

    gcmkHEADER_ARG("Mmu=0x%08X Address=0x%llx", Mmu, Address);
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkASSERT(Mmu->hardware->mmuVersion > 0);

#if gcdENABLE_40BIT_VA
    if (Address < gcd4G_SIZE) {
        area = (AreaType == gcvAREA_TYPE_4K) ?
               &Mmu->dynamicLowArea4K : &Mmu->dynamicLowArea1M;
    }
#endif

    mtlb = (gctUINT32)((Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT);

    if (AreaType != gcvAREA_TYPE_FLATMAP) {
        stlb = (Address & stlbMask) >> stlbShift;

        pageTable = area->stlbLogical;

        index = (mtlb - area->mappingStart) * stlbEntryNum + stlb;

        gcmkPRINT("    Page table entry = 0x%08X",
                  _ReadPageEntry(pageTable + index));
    } else {
        gcsMMU_STLB_CHUNK_PTR stlbChunkObj = Mmu->staticSTLB;
        gctUINT32             entry        = Mmu->mtlbLogical[mtlb];

        stlb = (Address & gcdMMU_STLB_1M_MASK) >> gcdMMU_STLB_1M_SHIFT;

        entry &= 0xFFFFFFF0;

        while (stlbChunkObj) {
            gctUINT i;
            gctBOOL found = gcvFALSE;

            for (i = 0; i < stlbChunkObj->mtlbEntryNum; ++i) {
                gctPHYS_ADDR_T stlbPhysBase = stlbChunkObj->physBase + (i * gcdMMU_STLB_1M_SIZE);
                gctUINT32_PTR  stlbLogical  = (gctUINT32_PTR)((gctUINT8_PTR)stlbChunkObj->logical +
                                                              (i * gcdMMU_STLB_1M_SIZE));
                if (entry == stlbPhysBase) {
                    gcmkPRINT("    Page table entry = 0x%08X", stlbLogical[stlb]);
                    found = gcvTRUE;
                    break;
                }
            }
            if (found)
                break;
            stlbChunkObj = stlbChunkObj->next;
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

void
gckMMU_CheckSaftPage(IN gckMMU Mmu)
{
    gctUINT8_PTR safeLogical = Mmu->safePageLogical;
    gctUINT32    offsets[]   = { 0, 64, 128, 256, 2560, 4000 };

    gctUINT32 i = 0;

    while (i < gcmCOUNTOF(offsets)) {
        if (safeLogical[offsets[i]] != 0) {
            gcmkPRINT("%s(%d) safe page is over written [%d] = %x",
                      __FUNCTION__, __LINE__, i, safeLogical[offsets[i]]);
        }
    }
}

void
gckMMU_DumpAddressSpace(IN gckMMU Mmu)
{
    gctUINT i;
    gctUINT next;
    gcsADDRESS_AREA_PTR area = &Mmu->dynamicArea4K;
    gctUINT32_PTR       map  = area->mapLogical;
    gctBOOL             used = gcvFALSE;
    gctUINT32           numPages;

    /* Grab the mutex. */
    gcmkVERIFY_OK(gckOS_AcquireMutex(Mmu->os, Mmu->pageTableMutex, gcvINFINITE));

    /* Find node which contains index. */
    for (i = 0; i < area->stlbEntries; i = next) {
        switch (gcmENTRY_TYPE(map[i])) {
        case gcvMMU_SINGLE:
            numPages = 1;
            next     = i + numPages;
            used     = gcvFALSE;
            break;

        case gcvMMU_FREE:
            numPages = map[i] >> 8;
            next     = i + numPages;
            used     = gcvFALSE;
            break;

        case gcvMMU_USED:
            numPages = 1;
            next     = i + numPages;
            used     = gcvTRUE;
            break;

        default:
            gcmkFATAL("MMU table correcupted at index %u!", i);
            return;
        }

        if (!used)
            gcmkPRINT("Available Range [%d - %d)", i, i + numPages);
    }

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Mmu->os, Mmu->pageTableMutex));
}

void
gckMMU_DumpRecentFreedAddress(IN gckMMU Mmu)
{
    gckQUEUE      queue = &Mmu->recentFreedAddresses;
    gctUINT32     i;
    gcuQUEUEDATA *data;

    if (queue->count) {
        gcmkPRINT("    Recent %d freed GPU address ranges:", queue->count);

        for (i = 0; i < queue->count; i++) {
            gckQUEUE_GetData(queue, i, &data);

            gcmkPRINT("      [%llx - %llx]",
                      data->addressData.start, data->addressData.end);
        }
    }
}

gceSTATUS
gckMMU_FillFlatMapping(IN gckMMU Mmu, IN gctUINT64 PhysBase,
                       IN gctSIZE_T Size, IN gctBOOL Reserved,
                       IN gctBOOL AbleToShift, OUT gctADDRESS *GpuBaseAddress)
{
    gceSTATUS           status;
    gckHARDWARE         hardware = Mmu->hardware;
    gctUINT32           mtlb;
    gctUINT32           physBase;
    gcsADDRESS_AREA_PTR area                    = &Mmu->dynamicArea4K;
    gctBOOL             physicalRangeOverlapped = gcvFALSE;
    gctBOOL             virtualRangeOverlapped  = gcvFALSE;
    gctBOOL             specificFlatMapping     = gcvFALSE;
    gctBOOL             needShiftMapping        = gcvFALSE;
    gctUINT64           flatBase                = PhysBase;
    gctSIZE_T           flatSize                = Size;
    gctUINT64           base                    = flatBase;
    gctUINT64           end                     = base + flatSize;
    gctADDRESS          reqVirtualBase          = 0;
    gctADDRESS          flatVirtualBase         = 0;
    gctADDRESS          baseAddress             = 0;
    gceFLATMAP_FLAG     mapFlag                 = gcvFLATMAP_DIRECT;
    gctUINT32           i;

    if (!hardware->mmuVersion)
        return gcvSTATUS_OK;

    /************************ Get flat mapping type and range. ************************/
    {
        for (i = 0; i < Mmu->gpuPhysicalRangeCount; i++) {
            if (base < Mmu->gpuPhysicalRanges[i].start) {
                if (end > Mmu->gpuPhysicalRanges[i].start) {
                    physicalRangeOverlapped = gcvTRUE;
                    if (Mmu->gpuPhysicalRanges[i].flag == gcvFLATMAP_DIRECT) {
                        /* Overlapped part is direct mapping, continue direct mapping */
                        end = Mmu->gpuPhysicalRanges[i].start;
                    } else {
                        /* Overlapped part is shift mapping, do entire shift mapping */
                        needShiftMapping = gcvTRUE;
                    }
                }

                flatSize = (gctSIZE_T)(end - base);
            } else if (end > Mmu->gpuPhysicalRanges[i].end) {
                if (base < Mmu->gpuPhysicalRanges[i].end) {
                    physicalRangeOverlapped = gcvTRUE;
                    if (Mmu->gpuPhysicalRanges[i].flag == gcvFLATMAP_DIRECT) {
                        /* Overlapped part is direct mapping, continue direct mapping */
                        base = Mmu->gpuPhysicalRanges[i].end + 1;
                    } else {
                        /* Overlapped part is shift mapping, do entire shift mapping */
                        needShiftMapping = gcvTRUE;
                    }
                }

                flatBase = base;
                flatSize = (gctSIZE_T)(end - base);
            } else {
                /* it is already inside existing flat mapping ranges. */
                flatSize = 0;
            }

            if (flatSize == 0) {
                if (GpuBaseAddress)
                    *GpuBaseAddress = (gctADDRESS)PhysBase;

                return gcvSTATUS_OK;
            }
        }
    }

    /* overwrite the original parameters */
    PhysBase = flatBase;
    physBase = (gctUINT32)flatBase;

    mtlb     = _MtlbOffset(physBase);

    if (GpuBaseAddress)
        reqVirtualBase = *GpuBaseAddress;

    /*
     * if no partcial physical range overlap to request entire shift mapping,
     * it is specific shift mapping or directly mapping by default.
     */
    if (!needShiftMapping) {
        flatVirtualBase = reqVirtualBase ?
                          reqVirtualBase : (gctADDRESS)flatBase;
    }

    for (i = 0; i < Mmu->gpuAddressRangeCount; i++) {
        if (_IsRangeInsected(flatVirtualBase, flatSize,
                             Mmu->gpuAddressRanges[i].start,
                             (gctSIZE_T)Mmu->gpuAddressRanges[i].size)) {
            virtualRangeOverlapped = gcvTRUE;
        }
    }

    /* If gpu virtual range overlapped or gpu physical over 4G, still need entire shift mapping */
    if ((!physicalRangeOverlapped && virtualRangeOverlapped) ||
        PhysBase + flatSize - 1 > ((1ULL << gcdMMU_VA_BITS) - 1)) {
        needShiftMapping = gcvTRUE;
    }

    if (needShiftMapping && !AbleToShift) {
        /*
         * Return without mapping any address.
         * By now, only physBase physSize could run here.
         */
        return gcvSTATUS_OK;
    }

    if (needShiftMapping || specificFlatMapping)
        mapFlag = gcvFLATMAP_SHIFT;

    specificFlatMapping = (reqVirtualBase && !virtualRangeOverlapped && !physicalRangeOverlapped);

    /************************ Setup flat mapping in dynamic range. ****************/
    if (area->mappingStart != gcvINVALID_VALUE &&
        mtlb >= area->mappingStart &&
        mtlb < area->mappingEnd &&
        !needShiftMapping) {
        /* This path is useless now, keep it 4K page size */

        gctUINT32_PTR stlbEntry;

        stlbEntry = _StlbEntry(area, physBase);

        /* Must be aligned to page. */
        gcmkASSERT((flatSize & 0xFFF) == 0);

        for (i = 0; i < (flatSize / gcdMMU_PAGE_4K_SIZE); i++) {
            /* Flat mapping in page table. */
            _WritePageEntry(stlbEntry, _SetPage(physBase + i * gcdMMU_PAGE_4K_SIZE, 0, gcvTRUE));
#if gcdMMU_TABLE_DUMP
            gckOS_Print("%s(%d): insert MTLB[%d] STLB[%d]: %08x\n",
                        __FUNCTION__, __LINE__,
                        (physBase & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT,
                        ((physBase & gcdMMU_STLB_4K_MASK) >> gcdMMU_STLB_4K_SHIFT) + i,
                        _ReadPageEntry(stlbEntry));
#endif
            stlbEntry++;
        }

#if gcdDUMP_IN_KERNEL
        {
            gctPHYS_ADDR_T physical;
            gctUINT32      data = _SetPage(physBase, 0, gcvTRUE) & ~0xF;
            gctUINT32      step = (_SetPage(physBase + gcdMMU_PAGE_4K_SIZE, 0, gcvTRUE) & ~0xF) - data;
            gctUINT32      mask = _SetPage(physBase, 0, gcvTRUE) & 0xF;

            physical = area->stlbPhysical + 4 * _AddressToIndex(area, physBase);

            gcmkDUMP(Mmu->os, "#[mmu-stlb: flat-mapping in dynamic: 0x%08X - 0x%08X]",
                     physBase, physBase - 1 + flatSize);

            gcmkDUMP(Mmu->os, "@[physical.step 0x%010llX 0x%08X 0x%08lX 0x%08X 0x%08X",
                     (unsigned long long)physical, data,
                     (unsigned long)(flatSize / gcdMMU_PAGE_4K_SIZE * sizeof(gctUINT32)),
                     step, mask);
        }
#endif

        /* Flat mapping in map. */
        _FillFlatMappingInMap(area, _AddressToIndex(area, physBase),
                              (gctUINT32)flatSize / gcdMMU_PAGE_4K_SIZE);

        return gcvSTATUS_OK;
    }

    /************************ Setup flat mapping in non dynamic range. **************/
    switch (Mmu->flatMappingMode) {
    case gcvPAGE_TYPE_16M:
        if (flatSize >= gcdMMU_PAGE_16M_SIZE) {
            gcmkONERROR(gckMMU_FillFlatMappingWithPage16M(Mmu, PhysBase, flatSize, Reserved,
                                                          needShiftMapping, specificFlatMapping,
                                                          reqVirtualBase, &baseAddress));

            break;
        }
    /* FALLTHRU */
    case gcvPAGE_TYPE_1M:
        gcmkONERROR(gckMMU_FillFlatMappingWithPage1M(Mmu, PhysBase, flatSize, Reserved,
                                                     needShiftMapping, specificFlatMapping,
                                                     reqVirtualBase, &baseAddress));

        break;
    case gcvPAGE_TYPE_64K:
        gcmkONERROR(gckMMU_FillFlatMappingWithPage64K(Mmu, PhysBase, flatSize, Reserved,
                                                      needShiftMapping, specificFlatMapping,
                                                      reqVirtualBase, &baseAddress));

        break;
    case gcvPAGE_TYPE_4K:
        gcmkONERROR(gckMMU_FillFlatMappingWithPage4K(Mmu, PhysBase, flatSize, Reserved,
                                                     needShiftMapping, specificFlatMapping,
                                                     reqVirtualBase, &baseAddress));

        break;
    }

    if (GpuBaseAddress)
        *GpuBaseAddress = baseAddress;

    /* Store the gpu virtual ranges */
    Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].start = baseAddress;
    Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].end   = baseAddress + flatSize - 1;
    Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].size  = flatSize;
    Mmu->gpuAddressRanges[Mmu->gpuAddressRangeCount].flag  = mapFlag;
    Mmu->gpuAddressRangeCount++;

    gcmkASSERT(Mmu->gpuAddressRangeCount <= gcdMAX_FLAT_MAPPING_COUNT);

    /* Store the gpu physical ranges */
    Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].start  = flatBase;
    Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].end    = flatBase + flatSize - 1;
    Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].size   = flatSize;
    Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].flag   = mapFlag;
    Mmu->gpuPhysicalRanges[Mmu->gpuPhysicalRangeCount].vStart = baseAddress;
    Mmu->gpuPhysicalRangeCount++;

    gcmkASSERT(Mmu->gpuPhysicalRangeCount <= gcdMAX_FLAT_MAPPING_COUNT);

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckMMU_IsFlatMapped(IN gckMMU Mmu, IN gctUINT64 Physical,
                    IN gctSIZE_T Bytes, OUT gctBOOL *In,
                    INOUT gctADDRESS *Address)
{
    gceSTATUS status;
    gctUINT32 i;
    gctBOOL   inFlatmapping = gcvFALSE;

    gcmkHEADER();

    gcmkVERIFY_ARGUMENT(In != gcvNULL);

    if (gckHARDWARE_IsFeatureAvailable(Mmu->hardware, gcvFEATURE_MMU) == gcvFALSE) {
        /* gcvSTATUS_NOT_SUPPORTED */
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    if (Address && *Address != gcvINVALID_ADDRESS) {
        for (i = 0; i < Mmu->gpuAddressRangeCount; i++) {
            if ((*Address >= Mmu->gpuAddressRanges[i].start) &&
                (*Address + Bytes - 1 <= Mmu->gpuAddressRanges[i].end)) {
                inFlatmapping = gcvTRUE;
                goto Found;
            }
        }
    }

    if (Physical != gcvINVALID_PHYSICAL_ADDRESS) {
        for (i = 0; i < Mmu->gpuPhysicalRangeCount; i++) {
            if (Physical >= Mmu->gpuPhysicalRanges[i].start &&
                (Physical + Bytes - 1 <= Mmu->gpuPhysicalRanges[i].end)) {
                inFlatmapping = gcvTRUE;

                if (Address) {
                    *Address = Mmu->gpuPhysicalRanges[i].vStart +
                               (gctADDRESS)(Physical - Mmu->gpuPhysicalRanges[i].start);
                }

                break;
            }
        }
    }

Found:
    *In = inFlatmapping;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_SetupSRAM(IN gckMMU Mmu, IN gckHARDWARE Hardware, IN gckDEVICE Device)
{
    gctBOOL        needMapInternalSRAM = gcvFALSE;
    gctPHYS_ADDR_T reservedBase        = gcvINVALID_PHYSICAL_ADDRESS;
    gctUINT32      reservedSize        = 0;
    gctINT         i                   = 0;
    gctINT         j                   = 0;
    gceSTATUS      status              = gcvSTATUS_OK;

    gcmkHEADER_ARG("Mmu=%p Hardware=%p", Mmu, Hardware);

    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);

    if (Hardware->mmuVersion == 0) {
        gcmkFOOTER();
        return status;
    }

    if (!Mmu->sRAMMapped) {
        gctADDRESS address = gcvINVALID_ADDRESS;
        gctUINT32  size    = 0;
        gctINT32   cursor  = -1;

        /* Map all the SRAMs in MMU table. */
        for (i = 0; i < gcvCORE_COUNT; i++) {
            for (j = gcvSRAM_INTERNAL0; j < gcvSRAM_INTER_COUNT; j++) {
                reservedBase = Device->sRAMBases[i][j];
                reservedSize = Device->sRAMSizes[i][j];

                Device->sRAMBaseAddresses[i][j] = 0;

                needMapInternalSRAM = reservedSize && (reservedBase != gcvINVALID_PHYSICAL_ADDRESS);

                /* Map the internal SRAM. */
                if (needMapInternalSRAM) {
                    if (Device->showSRAMMapInfo) {
                        gcmkPRINT("Galcore Info: MMU mapped core%d SRAM base=0x%llx size=0x%x",
                                  i, reservedBase, reservedSize);
                    }

                    /*
                     * Default gpu virtual base = 0.
                     * It can be specified if not conflict with existing mapping.
                     */
                    gcmkONERROR(gckMMU_FillFlatMapping(Mmu, reservedBase, reservedSize, gcvTRUE,
                                                       gcvTRUE, &Device->sRAMBaseAddresses[i][j]));

                    Device->sRAMBases[i][j] = reservedBase;
                } else if (reservedSize && reservedBase == gcvINVALID_PHYSICAL_ADDRESS) {
                    /*
                     * Reserve the internal SRAM range in first MMU mtlb and
                     * set base to gcdRESERVE_START. If internal SRAM range is
                     * not specified, which means it is reserve usage.
                     */
                    if (cursor == -1)
                        cursor = i;

                    Device->sRAMBaseAddresses[i][j] = (i > cursor) ? Device->sRAMBaseAddresses[cursor][j] :
                                                      (address == gcvINVALID_ADDRESS) ? gcdRESERVE_START :
                                                      address + gcmALIGN(size, gcdRESERVE_ALIGN);

                    Device->sRAMBases[i][j] = Device->sRAMBaseAddresses[i][j];
                    address = Device->sRAMBaseAddresses[i][j];

                    size = Device->sRAMSizes[i][j];

                    Device->sRAMPhysFaked[i][j] = gcvFALSE;
                }

#if gcdCAPTURE_ONLY_MODE
                Device->sRAMPhysFaked[i][j] = gcvTRUE;
#endif
            }
        }

        address = gcdVA_RESERVED_SIZE;

        /* Map all the external SRAMs in MMU table. */
        for (i = 0; i < gcvSRAM_EXT_COUNT; i++) {
            if (Device->extSRAMSizes[i] &&
               (Device->extSRAMBases[i] != gcvINVALID_PHYSICAL_ADDRESS)) {
                gcmkONERROR(gckOS_CPUPhysicalToGPUPhysical(Mmu->os,
                                                           Device->extSRAMBases[i],
                                                           &Device->extSRAMGPUBases[i]));

                gcmkONERROR(gckMMU_FillFlatMapping(Mmu,
                                                   Device->extSRAMGPUBases[i],
                                                   Device->extSRAMSizes[i],
                                                   gcvFALSE,
                                                   gcvTRUE,
                                                   &address));

                Device->extSRAMBaseAddresses[i] = address;

                address += Device->extSRAMSizes[i];

                Device->extSRAMGPUPhysNames[i] =
                    gckKERNEL_AllocateNameFromPointer(Hardware->kernel, Device->extSRAMPhysicals[i]);
            }
        }

        /* The value of non-reserve range start address is equal with reserve range size. */
        Mmu->reserveRangeSize = (gctSIZE_T)address;
        Mmu->sRAMMapped       = gcvTRUE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the error. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckMMU_GetPageEntry(IN gckMMU Mmu, IN gcePAGE_TYPE PageType, IN gctBOOL LowVA,
                    IN gctADDRESS Address, IN gctUINT32_PTR *PageTable)
{
    gctUINT32_PTR       pageTable;
    gctUINT32           index;
    gctUINT32           mtlbOffset, stlbOffset;
    gcsADDRESS_AREA_PTR area = _GetProcessArea(Mmu, PageType, LowVA, gcvFALSE);

    gcmkHEADER_ARG("Mmu=0x%08X Address=0x%llx", Mmu, Address);
    gcmkVERIFY_OBJECT(Mmu, gcvOBJ_MMU);

    gcmkASSERT(Mmu->hardware->mmuVersion > 0);

    mtlbOffset = (gctUINT32)((Address & gcdMMU_MTLB_MASK) >> gcdMMU_MTLB_SHIFT);

    if (mtlbOffset >= area->mappingStart) {
        gctUINT32 stlbShift = (PageType == gcvPAGE_TYPE_1M) ?
                              gcdMMU_STLB_1M_SHIFT :
                              gcdMMU_STLB_4K_SHIFT;

        gctUINT32 stlbMask = (PageType == gcvPAGE_TYPE_1M) ?
                             gcdMMU_STLB_1M_MASK :
                             gcdMMU_STLB_4K_MASK;

        gctUINT32 stlbEntryNum = (PageType == gcvPAGE_TYPE_1M) ?
                                 gcdMMU_STLB_1M_ENTRY_NUM :
                                 gcdMMU_STLB_4K_ENTRY_NUM;

        stlbOffset = (Address & stlbMask) >> stlbShift;

        pageTable = area->stlbLogical;

        index = (mtlbOffset - area->mappingStart) * stlbEntryNum + stlbOffset;

        *PageTable = pageTable + index;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckMMU_GetAreaType(IN gckMMU Mmu, IN gctADDRESS Address,
                   OUT gceAREA_TYPE *AreaType)
{
    gctUINT32           mtlbIndex;
    gctBOOL             flatMapped;
    gceSTATUS           status = gcvSTATUS_OK;
    gcsADDRESS_AREA_PTR area4K = &Mmu->dynamicArea4K;
#if gcdENABLE_GPU_1M_PAGE
    gcsADDRESS_AREA_PTR area1M = &Mmu->dynamicArea1M;
#endif
#if gcdENABLE_40BIT_VA
    gcsADDRESS_AREA_PTR lowArea4K = &Mmu->dynamicLowArea4K;
#if gcdENABLE_GPU_1M_PAGE
    gcsADDRESS_AREA_PTR lowArea1M = &Mmu->dynamicLowArea1M;
#    endif
#endif

    mtlbIndex = _MtlbOffset(Address);

    gcmkONERROR(gckMMU_IsFlatMapped(Mmu, gcvINVALID_PHYSICAL_ADDRESS,
                                    1, &flatMapped, &Address));

    if (flatMapped) {
        /* FlatMapping */
        *AreaType = gcvAREA_TYPE_FLATMAP;
    }
#if gcdENABLE_GPU_1M_PAGE
    else if (mtlbIndex >= area1M->mappingStart &&
             mtlbIndex <= area1M->mappingEnd) {
        *AreaType = gcvAREA_TYPE_1M;
    }
#endif
    else if (mtlbIndex >= area4K->mappingStart &&
             mtlbIndex <= area4K->mappingEnd) {
        *AreaType = gcvAREA_TYPE_4K;
    }
#if gcdENABLE_40BIT_VA
    else if (mtlbIndex >= lowArea4K->mappingStart &&
             mtlbIndex <= lowArea4K->mappingEnd) {
        *AreaType = gcvAREA_TYPE_4K;
    }
#if gcdENABLE_GPU_1M_PAGE
    else if (mtlbIndex >= lowArea1M->mappingStart &&
             mtlbIndex <= lowArea1M->mappingEnd) {
        *AreaType = gcvAREA_TYPE_1M;
    }
#    endif
#endif
    else {
        /* unknown type */
        *AreaType = gcvAREA_TYPE_UNKNOWN;
    }

OnError:
    return status;
}
