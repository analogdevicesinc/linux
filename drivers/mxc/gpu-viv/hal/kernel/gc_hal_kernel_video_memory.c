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

#if defined(__QNXNTO__)
#include <stdint.h>
#include <stdlib.h>
#include <sys/slogcodes.h>
#include <time.h>

extern unsigned int slogUsageInterval;
extern const uint64_t slogLowWaterFPC;
#endif

#define _GC_OBJ_ZONE    gcvZONE_VIDMEM

/******************************************************************************
 ******************************* Private Functions ****************************
 ******************************************************************************/

/*******************************************************************************
 **
 **  _Split
 **
 **  Split a node on the required byte boundary.
 **
 **  INPUT:
 **
 **      gckOS Os
 **          Pointer to an gckOS object.
 **
 **      gcuVIDMEM_NODE_PTR Node
 **          Pointer to the node to split.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes to keep in the node.
 **
 **  OUTPUT:
 **
 **      Nothing.
 **
 **  RETURNS:
 **
 **      gctBOOL
 **          gcvTRUE if the node was split successfully, or gcvFALSE if there is an
 **          error.
 **
 */
static gctBOOL
_Split(IN gckOS Os, IN gcuVIDMEM_NODE_PTR Node, IN gctSIZE_T Bytes)
{
    gcuVIDMEM_NODE_PTR node;
    gctPOINTER         pointer = gcvNULL;

    /* Make sure the byte boundary makes sense. */
    if (Bytes <= 0 || Bytes > Node->VidMem.bytes) {
        /* Error. */
        return gcvFALSE;
    }

    /* Allocate a new gcuVIDMEM_NODE object. */
    if (gcmIS_ERROR(gckOS_Allocate(Os, gcmSIZEOF(gcuVIDMEM_NODE), &pointer))) {
        /* Error. */
        return gcvFALSE;
    }

    node = pointer;

    /* Initialize gcuVIDMEM_NODE structure. */
    node->VidMem.offset    = Node->VidMem.offset + Bytes;
    node->VidMem.bytes     = Node->VidMem.bytes - Bytes;
    node->VidMem.alignment = 0;
    node->VidMem.locked    = 0;
    node->VidMem.parent    = Node->VidMem.parent;
    node->VidMem.pool      = Node->VidMem.pool;
    node->VidMem.processID = 0;
    node->VidMem.logical   = gcvNULL;
    node->VidMem.kvaddr    = gcvNULL;

    /* Insert node behind specified node. */
    node->VidMem.next = Node->VidMem.next;
    node->VidMem.prev = Node;
    Node->VidMem.next = node;
    node->VidMem.next->VidMem.prev = node;

    /* Insert free node behind specified node. */
    node->VidMem.nextFree = Node->VidMem.nextFree;
    node->VidMem.prevFree = Node;
    Node->VidMem.nextFree = node;
    node->VidMem.nextFree->VidMem.prevFree = node;

    /* Adjust size of specified node. */
    Node->VidMem.bytes = Bytes;

    /* Success. */
    return gcvTRUE;
}

/*******************************************************************************
 **
 **  _Merge
 **
 **  Merge two adjacent nodes together.
 **
 **  INPUT:
 **
 **      gckOS Os
 **          Pointer to an gckOS object.
 **
 **      gcuVIDMEM_NODE_PTR Node
 **          Pointer to the first of the two nodes to merge.
 **
 **  OUTPUT:
 **
 **      Nothing.
 **
 */
static gceSTATUS
_Merge(IN gckOS Os, IN gcuVIDMEM_NODE_PTR Node)
{
    gcuVIDMEM_NODE_PTR node;
    gceSTATUS          status;

    /* Save pointer to next node. */
    node = Node->VidMem.next;

    /* This is a good time to make sure the heap is not corrupted. */
    if (Node->VidMem.offset + Node->VidMem.bytes != node->VidMem.offset) {
        /* Corrupted heap. */
        gcmkASSERT(Node->VidMem.offset + Node->VidMem.bytes == node->VidMem.offset);
        return gcvSTATUS_HEAP_CORRUPTED;
    }

    /* Adjust byte count. */
    Node->VidMem.bytes += node->VidMem.bytes;

    /* Unlink next node from linked list. */
    Node->VidMem.next     = node->VidMem.next;
    Node->VidMem.nextFree = node->VidMem.nextFree;

    Node->VidMem.next->VidMem.prev         = Node;
    Node->VidMem.nextFree->VidMem.prevFree = Node;

    /* Free next node. */
    status = gcmkOS_SAFE_FREE(Os, node);
    return status;
}

#if gcdENABLE_VIDEO_MEMORY_TRACE
static gceSTATUS
_TraceGpuMem(IN gckOS Os, IN gctINT32 ProcessID, IN gcuVIDMEM_NODE_PTR Node)
{
    gctUINT64 Delta = 0;
    gckVIDMEM_BLOCK vidMemBlock = Node->VirtualChunk.parent;

    if (Node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
        Delta = Node->VidMem.bytes;
    else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
        Delta = Node->VirtualChunk.bytes;
    else
        Delta = Node->Virtual.bytes;

    if (ProcessID < 0)
        gckOS_TraceGpuMemory(Os, -ProcessID, -Delta);
    else
        gckOS_TraceGpuMemory(Os, ProcessID, Delta);

    return gcvSTATUS_OK;
}
#endif

#define PIDINFO_LIST_NODE_Const 0x01
#define PIDINFO_LIST_NODE_Ref 0x02
#define PIDINFO_LIST_NODE_Deref 0x03
#define PIDINFO_LIST_NODE_Clean 0x04

static gceSTATUS
_UpdatePIDInfoListNode(gckOS Os, gckVIDMEM_NODE Node,
                       gcuVIDMEM_NODE_PTR VideoNode,
                       gctUINT32 ProcessID, gctUINT32 Behavier)
{
#if gcdENABLE_VIDEO_MEMORY_TRACE
    gckVIDMEM_PIDINFO pid_info;
    gctUINT32 pid = 0;
    gctUINT32 oldValue = 0;
    gckVIDMEM_PIDINFO pre_pid_info;

    switch (Behavier) {
    case PIDINFO_LIST_NODE_Const:
        /* Construct pid info struct */
        gcmkVERIFY_OK(gckOS_Allocate(Os, gcmSIZEOF(gcsVIDMEM_PIDINFO), (gctPOINTER *)&pid_info));
        gcmkVERIFY_OK(gckOS_ZeroMemory((gctPOINTER)pid_info, gcmSIZEOF(gcsVIDMEM_PIDINFO)));

        Node->pidInfo = pid_info;

        gckOS_GetProcessID(&Node->pidInfo->pid);

        gcmkVERIFY_OK(gckOS_AtomConstruct(Os, &Node->pidInfo->ref));
        gckOS_AtomSet(Os, Node->pidInfo->ref, 1);
        gckOS_CreateMutex(Os, &Node->infoMutex);

        Node->pidInfo->next = gcvNULL;

        _TraceGpuMem(Os, Node->pidInfo->pid, VideoNode);
        break;
    case PIDINFO_LIST_NODE_Ref:
        gcmkVERIFY_OK(gckOS_AcquireMutex(Os, Node->infoMutex, gcvINFINITE));

        pid_info = Node->pidInfo;

        gckOS_GetProcessID(&pid);

        /* find the current pid from pid info node. */
        while (gcvTRUE) {
            if (!pid_info)
                break;
            if (pid_info->pid == pid) {
                gckOS_AtomIncrement(Os, pid_info->ref, &oldValue);
                break;
            }
            pid_info = pid_info->next;
        }

        /* Doesn't find! Create a new node. */
        if (!pid_info) {
            gckVIDMEM_PIDINFO t_info;
            gctPOINTER t_pointer;

            gckOS_Allocate(Os, gcmSIZEOF(gcsVIDMEM_PIDINFO), &t_pointer);
            gcmkVERIFY_OK(gckOS_ZeroMemory(t_pointer, gcmSIZEOF(gcsVIDMEM_PIDINFO)));
            t_info = t_pointer;
            t_info->pid = pid;
            gckOS_AtomConstruct(Os, &t_info->ref);
            gckOS_AtomSet(Os, t_info->ref, 1);

            t_info->next = Node->pidInfo;
            Node->pidInfo = t_info;
            _TraceGpuMem(Os, pid, VideoNode);
        }
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Os, Node->infoMutex));
        break;
    case PIDINFO_LIST_NODE_Deref:
        if (ProcessID == 0)
            gckOS_GetProcessID(&pid);
        else
            pid = ProcessID;

        gcmkVERIFY_OK(gckOS_AcquireMutex(Os, Node->infoMutex, gcvINFINITE));
        pid_info = Node->pidInfo;
        pre_pid_info = Node->pidInfo;

        /* find the matched info node. */
        while (gcvTRUE) {
            if (!pid_info)
                break;
            if (pid_info->pid == pid) {
                gctINT t_oldValue = 0;

                if (pid_info->ref)
                    gcmkVERIFY_OK(gckOS_AtomDecrement(Os, pid_info->ref, &t_oldValue));

                if (t_oldValue == 1) {
                    /* release unused pid info node */
                    _TraceGpuMem(Os, -(gctINT32)pid, VideoNode);
                    if (pid_info == Node->pidInfo)
                        Node->pidInfo = pid_info->next;
                    else
                        pre_pid_info->next = pid_info->next;
                    gcmkOS_SAFE_FREE(Os, pid_info->ref);
                    gcmkOS_SAFE_FREE(Os, pid_info);
                }
                break;
            }
            pre_pid_info = pid_info;
            pid_info = pid_info->next;
        }

        gcmkVERIFY_OK(gckOS_ReleaseMutex(Os, Node->infoMutex));
        break;
    case PIDINFO_LIST_NODE_Clean:
        while (Node->pidInfo) {
            gctINT32 ref = 0;

            pid_info = Node->pidInfo;
            gckOS_AtomGet(Os, pid_info->ref, &ref);
            if (ref)
                _TraceGpuMem(Os, -(gctINT32)pid_info->pid, VideoNode);
            Node->pidInfo = pid_info->next;
            gcmkOS_SAFE_FREE(Os, pid_info->ref);
            gcmkOS_SAFE_FREE(Os, pid_info);
        }
        gckOS_DeleteMutex(Os, Node->infoMutex);
        break;
    }
#endif

    return gcvSTATUS_OK;
}

/******************************************************************************
 ******************************* gckVIDMEM API Code ***************************
 ******************************************************************************/

/*******************************************************************************
 **
 **  gckVIDMEM_Construct
 **
 **  Construct a new gckVIDMEM object.
 **
 **  INPUT:
 **
 **      gckOS Os
 **          Pointer to an gckOS object.
 **
 **      gctPHYS_ADDR_T PhysicalBase
 **          Base physical address for the video memory heap.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes in the video memory heap.
 **
 **      gctSIZE_T Threshold
 **          Minimum number of bytes beyond am allocation before the node is
 **          split.  Can be used as a minimum alignment requirement.
 **
 **      gctSIZE_T BankSize
 **          Number of bytes per physical memory bank.  Used by bank
 **          optimization.
 **
 **  OUTPUT:
 **
 **      gckVIDMEM *Memory
 **          Pointer to a variable that will hold the pointer to the gckVIDMEM
 **          object.
 */
gceSTATUS
gckVIDMEM_Construct(IN gckOS Os, IN gctPHYS_ADDR_T PhysicalBase,
                    IN gctSIZE_T Bytes, IN gctSIZE_T Threshold,
                    IN gctSIZE_T BankSize, OUT gckVIDMEM *Memory)
{
    gckVIDMEM          memory = gcvNULL;
    gceSTATUS          status;
    gcuVIDMEM_NODE_PTR node;
    gctINT             i, banks = 0;
    gctPOINTER         pointer = gcvNULL;
    gctSIZE_T          heapBytes;
    gctSIZE_T          bankSize;
    gctSIZE_T          base = 0;

    gcmkHEADER_ARG("Os=0x%x PhysicalBase=0x%llx Bytes=%lu Threshold=%lu BankSize=%lu",
                   Os, PhysicalBase, Bytes, Threshold, BankSize);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Memory != gcvNULL);

    heapBytes = Bytes;
    bankSize  = BankSize;

    /* Allocate the gckVIDMEM object. */
    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(struct _gckVIDMEM), &pointer));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(struct _gckVIDMEM));

    memory = pointer;

    /* Initialize the gckVIDMEM object. */
    memory->object.type = gcvOBJ_VIDMEM;
    memory->os          = Os;

    /* Set video memory heap information. */
    memory->physicalBase = PhysicalBase;
    memory->bytes        = heapBytes;
    memory->freeBytes    = heapBytes;
    memory->minFreeBytes = heapBytes;
    memory->capability   = ~0u;
    memory->threshold    = Threshold;
    memory->mutex        = gcvNULL;

    /* Walk all possible banks. */
    for (i = 0; i < gcmCOUNTOF(memory->sentinel); ++i) {
        gctSIZE_T bytes;

        if (BankSize == 0) {
            bytes = heapBytes;
        } else {
            /* Compute number of bytes for this bank. */
            bytes = gcmALIGN(base + 1, bankSize) - base;

            if (bytes > heapBytes) {
                /* Make sure we don't exceed the total number of bytes. */
                bytes = heapBytes;
            }
        }

        if (bytes == 0) {
            /* Mark heap is not used. */
            memory->sentinel[i].VidMem.next     = gcvNULL;
            memory->sentinel[i].VidMem.prev     = gcvNULL;
            memory->sentinel[i].VidMem.nextFree = gcvNULL;
            memory->sentinel[i].VidMem.prevFree = gcvNULL;
            continue;
        }

        /* Allocate one gcuVIDMEM_NODE union. */
        gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcuVIDMEM_NODE), &pointer));

        node = pointer;

        /* Initialize gcuVIDMEM_NODE union. */
        node->VidMem.parent    = memory;

        node->VidMem.next      =
        node->VidMem.prev      =
        node->VidMem.nextFree  =
        node->VidMem.prevFree  = &memory->sentinel[i];

        node->VidMem.offset    = base;
        node->VidMem.bytes     = bytes;
        node->VidMem.alignment = 0;
        node->VidMem.pool      = gcvPOOL_UNKNOWN;

        node->VidMem.locked    = 0;

        node->VidMem.processID = 0;
        node->VidMem.logical   = gcvNULL;

#if gcdENABLE_VG
        node->VidMem.kernelVirtual = gcvNULL;
#endif
        node->VidMem.kvaddr        = gcvNULL;

        /* Initialize the linked list of nodes. */
        memory->sentinel[i].VidMem.next     = node;
        memory->sentinel[i].VidMem.prev     = node;
        memory->sentinel[i].VidMem.nextFree = node;
        memory->sentinel[i].VidMem.prevFree = node;

        /* Mark sentinel. */
        memory->sentinel[i].VidMem.bytes = 0;

        /* Adjust address for next bank. */
        base += bytes;
        heapBytes -= bytes;
        banks++;
    }

    /* Assign all the bank mappings. */
    memory->mapping[gcvVIDMEM_TYPE_COLOR_BUFFER]    = banks - 1;
    memory->mapping[gcvVIDMEM_TYPE_BITMAP]          = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_DEPTH_BUFFER]    = banks - 1;
    memory->mapping[gcvVIDMEM_TYPE_HZ_BUFFER]       = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_TEXTURE]         = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_VERTEX_BUFFER]   = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_INDEX_BUFFER]    = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_TILE_STATUS]     = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_COMMAND]         = banks - 1;

    if (banks > 1)
        --banks;
    memory->mapping[gcvVIDMEM_TYPE_GENERIC]         = 0;

#if gcdENABLE_VG
    memory->mapping[gcvVIDMEM_TYPE_IMAGE]   = 0;
    memory->mapping[gcvVIDMEM_TYPE_MASK]    = 0;
    memory->mapping[gcvVIDMEM_TYPE_SCISSOR] = 0;
#endif
    memory->mapping[gcvVIDMEM_TYPE_ICACHE]    = 0;
    memory->mapping[gcvVIDMEM_TYPE_TXDESC]    = 0;
    memory->mapping[gcvVIDMEM_TYPE_FENCE]     = 0;
    memory->mapping[gcvVIDMEM_TYPE_TFBHEADER] = 0;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "[GALCORE] INDEX:         bank %d",
                   memory->mapping[gcvVIDMEM_TYPE_INDEX_BUFFER]);
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "[GALCORE] VERTEX:        bank %d",
                   memory->mapping[gcvVIDMEM_TYPE_VERTEX_BUFFER]);
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "[GALCORE] TEXTURE:       bank %d",
                   memory->mapping[gcvVIDMEM_TYPE_TEXTURE]);
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "[GALCORE] RENDER_TARGET: bank %d",
                   memory->mapping[gcvVIDMEM_TYPE_COLOR_BUFFER]);
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "[GALCORE] DEPTH:         bank %d",
                   memory->mapping[gcvVIDMEM_TYPE_DEPTH_BUFFER]);
    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "[GALCORE] TILE_STATUS:   bank %d",
                   memory->mapping[gcvVIDMEM_TYPE_TILE_STATUS]);

    /* Allocate the mutex. */
    gcmkONERROR(gckOS_CreateMutex(Os, &memory->mutex));

    /* Return pointer to the gckVIDMEM object. */
    *Memory = memory;

    /* Success. */
    gcmkFOOTER_ARG("*Memory=%p", *Memory);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (memory != gcvNULL) {
        if (memory->mutex != gcvNULL) {
            /* Delete the mutex. */
            gcmkVERIFY_OK(gckOS_DeleteMutex(Os, memory->mutex));
        }

        for (i = 0; i < banks; ++i) {
            /* Free the heap. */
            gcmkASSERT(memory->sentinel[i].VidMem.next != gcvNULL);
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Os, memory->sentinel[i].VidMem.next));
        }

        /* Free the object. */
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Os, memory));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_Destroy
 **
 **  Destroy an gckVIDMEM object.
 **
 **  INPUT:
 **
 **      gckVIDMEM Memory
 **          Pointer to an gckVIDMEM object to destroy.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gckVIDMEM_Destroy(IN gckVIDMEM Memory)
{
    gcuVIDMEM_NODE_PTR node, next;
    gctINT             i;

    gcmkHEADER_ARG("Memory=%p", Memory);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Memory, gcvOBJ_VIDMEM);

    /* Walk all sentinels. */
    for (i = 0; i < gcmCOUNTOF(Memory->sentinel); ++i) {
        /* Bail out of the heap is not used. */
        if (Memory->sentinel[i].VidMem.next == gcvNULL)
            break;

        /* Walk all the nodes until we reach the sentinel. */
        for (node = Memory->sentinel[i].VidMem.next;
             node->VidMem.bytes != 0;
             node = next) {
            /* Save pointer to the next node. */
            next = node->VidMem.next;

            /* Free the node. */
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Memory->os, node));
        }
    }

    /* Free the mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Memory->os, Memory->mutex));

    /* Mark the object as unknown. */
    Memory->object.type = gcvOBJ_UNKNOWN;

    /* Free the gckVIDMEM object. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Memory->os, Memory));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

#if gcdENABLE_BANK_ALIGNMENT

#if !gcdBANK_BIT_START
#        error gcdBANK_BIT_START not defined.
#    endif

#if !gcdBANK_BIT_END
#        error gcdBANK_BIT_END not defined.
#    endif
/*******************************************************************************
 **  _GetSurfaceBankAlignment
 **
 **  Return the required offset alignment required to the make BaseAddress
 **  aligned properly.
 **
 **  INPUT:
 **
 **      gckOS Os
 **          Pointer to gcoOS object.
 **
 **      gceVIDMEM_TYPE Type
 **          Type of allocation.
 **
 **      gctUINT32 BaseAddress
 **          Base address of current video memory node.
 **
 **  OUTPUT:
 **
 **      gctUINT32_PTR AlignmentOffset
 **          Pointer to a variable that will hold the number of bytes to skip in
 **          the current video memory node in order to make the alignment bank
 **          aligned.
 */
static gceSTATUS
_GetSurfaceBankAlignment(IN gckKERNEL Kernel, IN gceVIDMEM_TYPE Type,
                         IN gctUINT32 BaseAddress,
                         OUT gctUINT32_PTR AlignmentOffset)
{
    gctUINT32 bank;
    /* To retrieve the bank. */
    static const gctUINT32 bankMask = (0xFFFFFFFF << gcdBANK_BIT_START)
                                    ^ (0xFFFFFFFF << (gcdBANK_BIT_END + 1));

    /* To retrieve the bank and all the lower bytes. */
    static const gctUINT32 byteMask = ~(0xFFFFFFFF << (gcdBANK_BIT_END + 1));

    gcmkHEADER_ARG("Type=%d BaseAddress=0x%x ", Type, BaseAddress);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(AlignmentOffset != gcvNULL);

    switch (Type) {
    case gcvVIDMEM_TYPE_COLOR_BUFFER:
    case gcvVIDMEM_TYPE_TEXTURE:
    case gcvVIDMEM_TYPE_BITMAP:
        bank = (BaseAddress & bankMask) >> (gcdBANK_BIT_START);

        /* Align to the first bank. */
        *AlignmentOffset = (bank == 0) ? 0 :
            ((1 << (gcdBANK_BIT_END + 1)) + 0) - (BaseAddress & byteMask);
        break;

    case gcvVIDMEM_TYPE_DEPTH_BUFFER:
        bank = (BaseAddress & bankMask) >> (gcdBANK_BIT_START);

        /* Align to the third bank. */
        *AlignmentOffset = (bank == 2) ? 0 :
            ((1 << (gcdBANK_BIT_END + 1)) + (2 << gcdBANK_BIT_START))
                                          - (BaseAddress & byteMask);

        /* Minimum 256 byte alignment needed for fast_msaa. */
        if (gcdBANK_CHANNEL_BIT > 7 ||
            ((gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_FAST_MSAA) != gcvSTATUS_TRUE) &&
             (gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_SMALL_MSAA) != gcvSTATUS_TRUE))) {
            /* Add a channel offset at the channel bit. */
            *AlignmentOffset += (1 << gcdBANK_CHANNEL_BIT);
        }
        break;

    default:
        /* no alignment needed. */
        *AlignmentOffset = 0;
    }

    /* Return the status. */
    gcmkFOOTER_ARG("*AlignmentOffset=%u", *AlignmentOffset);
    return gcvSTATUS_OK;
}
#endif

static gcuVIDMEM_NODE_PTR
_FindNode(IN gckKERNEL Kernel, IN gckVIDMEM Memory,
          IN gctINT Bank, IN gctSIZE_T Bytes,
          IN gceVIDMEM_TYPE Type, IN OUT gctUINT32_PTR Alignment)
{
    gcuVIDMEM_NODE_PTR node;
    gctUINT32          alignment;
    gctUINT32          bankAlignment;

    if (Memory->sentinel[Bank].VidMem.nextFree == gcvNULL) {
        /* No free nodes left. */
        return gcvNULL;
    }

    /*
     * Walk all free nodes until we have one that is big enough or we have
     * reached the sentinel.
     */
    for (node = Memory->sentinel[Bank].VidMem.nextFree;
         node->VidMem.bytes != 0;
         node = node->VidMem.nextFree) {
        gctUINT32 offset = (gctUINT32)(node->VidMem.parent->physicalBase + node->VidMem.offset);

        if (node->VidMem.bytes < Bytes)
            continue;

#if gcdENABLE_BANK_ALIGNMENT
        if (gcmIS_ERROR(_GetSurfaceBankAlignment(Kernel, Type, offset, &bankAlignment)))
            return gcvNULL;

        /*bankAlignment = gcmALIGN(bankAlignment, *Alignment);*/
#else
        bankAlignment = 0;
#endif

        /* Compute number of bytes to skip for alignment. */
        alignment = (*Alignment == 0) ?
                        0 : (*Alignment - ((offset + bankAlignment) & (*Alignment - 1)));

        if (alignment == *Alignment) {
            /* Node is already aligned. */
            alignment = 0;
        }

        if (node->VidMem.bytes >= Bytes + alignment + bankAlignment) {
            /* This node is big enough. */
            *Alignment = alignment + bankAlignment;
            return node;
        }
    }

    /* Not enough memory. */
    return gcvNULL;
}

/*******************************************************************************
 **
 **  gckVIDMEM_AllocateLinear
 **
 **  Allocate linear memory from the gckVIDMEM object.
 **
 **  INPUT:
 **
 **      gckVIDMEM Memory
 **          Pointer to an gckVIDMEM object.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes to allocate.
 **
 **      gctUINT32 Alignment
 **          Byte alignment for allocation.
 **
 **      gceVIDMEM_TYPE Type
 **          Type of surface to allocate (use by bank optimization).
 **
 **      gctUINT32 Flag
 **          Flag of allocatetion.
 **
 **      gctBOOL Specified
 **          If user must use this pool, it should set Specified to gcvTRUE,
 **          otherwise allocator may reserve some memory for other usage, such
 **          as small block size allocation request.
 **
 **  OUTPUT:
 **
 **      gcuVIDMEM_NODE_PTR * Node
 **          Pointer to a variable that will hold the allocated memory node.
 */
static gceSTATUS
gckVIDMEM_AllocateLinear(IN gckKERNEL Kernel, IN gckVIDMEM Memory,
                         IN gctSIZE_T Bytes, IN gctUINT32 Alignment,
                         IN gceVIDMEM_TYPE Type, IN gctUINT32 Flag,
                         IN gctBOOL Specified, OUT gcuVIDMEM_NODE_PTR *Node)
{
    gceSTATUS          status;
    gcuVIDMEM_NODE_PTR node;
    gctUINT32          alignment;
    gctINT             bank, i;
    gctBOOL            acquired     = gcvFALSE;
    gctUINT64          mappingInOne = 1;

    gcmkHEADER_ARG("Memory=%p Bytes=%lu Alignment=%u Type=%d",
                   Memory, Bytes, Alignment, Type);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Memory, gcvOBJ_VIDMEM);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Node != gcvNULL);
    gcmkVERIFY_ARGUMENT(Type < gcvVIDMEM_TYPE_COUNT);

    if (Alignment && (Alignment & (Alignment - 1)))
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

    /* Acquire the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(Memory->os, Memory->mutex, gcvINFINITE));

    acquired = gcvTRUE;

#if defined(__QNXNTO__)
    if (Flag & gcvALLOC_FLAG_CMA_LIMIT)
    {
        if ((Memory->physicalBase > UINT32_MAX) ||
            ((Memory->physicalBase + Memory->bytes) > UINT32_MAX))
        {
            /* Pool is allocated above 4G limit */
            status = gcvSTATUS_OUT_OF_MEMORY;
            goto OnError;
        }
    }

    if (slogUsageInterval > 0) {
        static gctSIZE_T lowwaterFPC = ~0;
        static time_t last_slog_time;
        int do_slog_now = 0;
        time_t this_slog_time = time(NULL);
        gctUINT32 pid = 0;

        gcmkVERIFY_OK(gckOS_GetProcessID(&pid));

        if (Memory->freeBytes < lowwaterFPC) {
            if (Memory->freeBytes < slogLowWaterFPC) {
                do_slog_now = 1;
            }
            lowwaterFPC = Memory->freeBytes;
        }

        if (abs(this_slog_time - last_slog_time) > slogUsageInterval) {
            do_slog_now = 1;
        }

        if (do_slog_now) {
            last_slog_time = this_slog_time;
            slogf(_SLOGC_GRAPHICS_GL, _SLOG_INFO, "%s: Memory->freeBytes = %u, lowest Memory->freeBytes = %u. "
                    "Handling message from pid: %u. Requested bytes: %zu",
                    __FUNCTION__, (unsigned) Memory->freeBytes, (unsigned) lowwaterFPC, pid, Bytes);
        }
    }
#endif
    if (Bytes > Memory->freeBytes) {
        /* Not enough memory. */
        status = gcvSTATUS_OUT_OF_MEMORY;
        goto OnError;
    }

#if gcdSMALL_BLOCK_SIZE
    if ((Memory->freeBytes < (Memory->bytes / gcdRATIO_FOR_SMALL_MEMORY)) &&
        Bytes >= gcdSMALL_BLOCK_SIZE &&
        Specified == gcvFALSE) {
        /* The left memory is for small memory.*/
        status = gcvSTATUS_OUT_OF_MEMORY;
        goto OnError;
    }
#endif

    /* Find the default bank for this surface type. */
    gcmkASSERT((gctINT)Type < gcmCOUNTOF(Memory->mapping));
    bank      = Memory->mapping[Type];
    alignment = Alignment;

    /* Find a free node in the default bank. */
    node = _FindNode(Kernel, Memory, bank, Bytes, Type, &alignment);

    /* Out of memory? */
    if (node == gcvNULL) {
        /* Walk all lower banks. */
        for (i = bank - 1; i >= 0; --i) {
            /* Find a free node inside the current bank. */
            node = _FindNode(Kernel, Memory, i, Bytes, Type, &alignment);
            if (node != gcvNULL)
                break;
        }
    }

    if (node == gcvNULL) {
        /* Walk all upper banks. */
        for (i = bank + 1; i < gcmCOUNTOF(Memory->sentinel); ++i) {
            if (Memory->sentinel[i].VidMem.nextFree == gcvNULL) {
                /* Abort when we reach unused banks. */
                break;
            }

            /* Find a free node inside the current bank. */
            node = _FindNode(Kernel, Memory, i, Bytes, Type, &alignment);
            if (node != gcvNULL)
                break;
        }
    }

    if (node == gcvNULL) {
        /* Out of memory. */
        status = gcvSTATUS_OUT_OF_MEMORY;
        goto OnError;
    }

    /* Do we have an alignment? */
    if (alignment > 0) {
        /* Split the node so it is aligned. */
        if (_Split(Memory->os, node, alignment)) {
            /* Successful split, move to aligned node. */
            node = node->VidMem.next;

            /* Remove alignment. */
            alignment = 0;
        }
    }

    /* Do we have enough memory after the allocation to split it? */
    if (node->VidMem.bytes - Bytes > Memory->threshold) {
        /* Adjust the node size. */
        _Split(Memory->os, node, Bytes);
    }

    /* Remove the node from the free list. */
    node->VidMem.prevFree->VidMem.nextFree = node->VidMem.nextFree;
    node->VidMem.nextFree->VidMem.prevFree = node->VidMem.prevFree;
    node->VidMem.nextFree                  = gcvNULL;
    node->VidMem.prevFree                  = gcvNULL;

    /* Fill in the information. */
    node->VidMem.alignment = alignment;
    node->VidMem.parent    = Memory;
    node->VidMem.logical   = gcvNULL;
    gcmkONERROR(gckOS_GetProcessID(&node->VidMem.processID));

    /* Adjust the number of free bytes. */
    Memory->freeBytes -= node->VidMem.bytes;

    if (Memory->freeBytes < Memory->minFreeBytes)
        Memory->minFreeBytes = Memory->freeBytes;

#if gcdENABLE_VG
    node->VidMem.kernelVirtual = gcvNULL;
#endif

    /* Release the mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(Memory->os, Memory->mutex));

    gckOS_QueryOption(Memory->os, "allMapInOne", &mappingInOne);
    if (!mappingInOne) {
        gcmkONERROR(gckOS_RequestReservedMemoryArea(Memory->os,
                                                    Memory->physical,
                                                    node->VidMem.offset,
                                                    node->VidMem.bytes,
                                                    &node->VidMem.physical));
    }

    /* Return the pointer to the node. */
    *Node = node;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Allocated %u bytes @ %p [0x%08X]",
                   node->VidMem.bytes, node, node->VidMem.offset);

    /* Success. */
    gcmkFOOTER_ARG("*Node=%p", *Node);
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Memory->os, Memory->mutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_AllocateVirtual
 **
 **  Construct a new gcuVIDMEM_NODE union for virtual memory.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gctSIZE_T Bytes
 **          Number of byte to allocate.
 **
 **  OUTPUT:
 **
 **      gcuVIDMEM_NODE_PTR *Node
 **          Pointer to a variable that receives the gcuVIDMEM_NODE union pointer.
 */
static gceSTATUS
gckVIDMEM_AllocateVirtual(IN gckKERNEL Kernel,
                          IN gctUINT32 Flag,
                          IN gctSIZE_T Bytes,
                          OUT gcuVIDMEM_NODE_PTR *Node)
{
    gckOS              os;
    gceSTATUS          status;
    gcuVIDMEM_NODE_PTR node    = gcvNULL;
    gctPOINTER         pointer = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p Flag=%x Bytes=%zu",
                   Kernel, Flag, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Node != gcvNULL);

    /* Extract the gckOS object pointer. */
    os = Kernel->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Allocate an gcuVIDMEM_NODE union. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(gcuVIDMEM_NODE), &pointer));

    gcmkVERIFY_OK(gckOS_ZeroMemory(pointer, gcmSIZEOF(gcuVIDMEM_NODE)));

    node = pointer;

    /* Initialize gcuVIDMEM_NODE union for virtual memory. */
    node->Virtual.kernel        = Kernel;
    node->Virtual.contiguous    = Flag & gcvALLOC_FLAG_CONTIGUOUS;
    node->Virtual.logical       = gcvNULL;
    node->Virtual.kvaddr        = gcvNULL;
    node->Virtual.bytes         = Bytes;
#if gcdENABLE_VG
    node->Virtual.kernelVirtual = gcvNULL;
#endif
    node->Virtual.secure        = (Flag & gcvALLOC_FLAG_SECURITY) != 0;
    node->Virtual.onFault       = (Flag & gcvALLOC_FLAG_ALLOC_ON_FAULT) != 0;
    node->Virtual.lowVA         = (Flag & (gcvALLOC_FLAG_32BIT_VA | gcvALLOC_FLAG_PRIOR_32BIT_VA)) != 0;

    /* Allocate the virtual memory. */
    if (Flag & gcvALLOC_FLAG_CONTIGUOUS) {
        gcmkONERROR_EX(gckOS_AllocatePagedMemory(os, Kernel, Flag,
                                                 &node->Virtual.bytes,
                                                 &node->Virtual.gid,
                                                 &node->Virtual.physical),
                       gcvSTATUS_OUT_OF_MEMORY);
    } else {
        gcmkONERROR(gckOS_AllocatePagedMemory(os, Kernel, Flag,
                                              &node->Virtual.bytes,
                                              &node->Virtual.gid,
                                              &node->Virtual.physical));
    }

    gckOS_GetProcessID(&node->Virtual.processID);

    /* Calculate required GPU page (4096) count. */
    /* Assume start address is 4096 aligned. */
    node->Virtual.pageCount =
        (gctSIZE_T)(((gctUINT64)node->Virtual.bytes + (4096 - 1)) >> 12);

    /* Return pointer to the gcuVIDMEM_NODE union. */
    *Node = node;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Created virtual node %p for %zu bytes @ %p",
                   node, Bytes, node->Virtual.physical);

    /* Success. */
    gcmkFOOTER_ARG("*Node=%p", *Node);
    return gcvSTATUS_OK;

OnError:
    /* Roll back. */
    if (node != gcvNULL) {
        /* Free the structure. */
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, node));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_AddToBlockList(IN gckKERNEL Kernel, IN gckVIDMEM_BLOCK VidMemBlock)
{
    VidMemBlock->next   = Kernel->vidMemBlock;
    Kernel->vidMemBlock = VidMemBlock;

    return gcvSTATUS_OK;
}

static gceSTATUS
_RemoveFromBlockList(IN gckKERNEL Kernel, IN gckVIDMEM_BLOCK VidMemBlock)
{
    gckVIDMEM_BLOCK vidMemBlock;
    gckVIDMEM_BLOCK previous = gcvNULL;
    gctUINT32       index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;

    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));
    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    for (vidMemBlock = Kernel->vidMemBlock;
         vidMemBlock != gcvNULL;
         vidMemBlock = vidMemBlock->next) {
        if (vidMemBlock->addresses[index] == VidMemBlock->addresses[index]) {
            if (previous)
                previous->next = vidMemBlock->next;
            else
                Kernel->vidMemBlock = vidMemBlock->next;

            vidMemBlock->next = gcvNULL;

            break;
        }
        previous = vidMemBlock;
    }

    return gcvSTATUS_OK;
}

static gckVIDMEM_BLOCK
_FindFreeBlock(IN gckKERNEL Kernel, IN gctSIZE_T Bytes, IN gctUINT32 Flag)
{
    gckVIDMEM_BLOCK vidMemBlock = gcvNULL;
    gctBOOL cacheable = (Flag & gcvALLOC_FLAG_CACHEABLE) != 0;

    for (vidMemBlock = Kernel->vidMemBlock;
         vidMemBlock != gcvNULL;
         vidMemBlock = vidMemBlock->next) {
        if (vidMemBlock->freeBytes >= Bytes && vidMemBlock->cacheable == cacheable) {
            /* Found the block */
            break;
        }
    }

    return vidMemBlock;
}

static gceSTATUS
_SplitVirtualChunk(IN gckOS Os, IN gcuVIDMEM_NODE_PTR Node, IN gctSIZE_T Bytes)
{
    gceSTATUS          status = gcvSTATUS_OK;
    gcuVIDMEM_NODE_PTR node   = gcvNULL;
    gctPOINTER         pointer;

    if (Bytes <= 0 || Bytes > Node->VirtualChunk.bytes)
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

    /* Allocate a new gcuVIDMEM_NODE object. */
    gcmkONERROR(gckOS_Allocate(Os, gcmSIZEOF(gcuVIDMEM_NODE), &pointer));

    gcmkVERIFY_OK(gckOS_ZeroMemory(pointer, gcmSIZEOF(gcuVIDMEM_NODE)));

    node = pointer;

    /* Initialize the new gcuVIDMEM_NODE. */
    node->VirtualChunk.offset = Node->VirtualChunk.offset + Bytes;
    node->VirtualChunk.bytes  = Node->VirtualChunk.bytes - Bytes;
    node->VirtualChunk.parent = Node->VirtualChunk.parent;
    node->VirtualChunk.kernel = Node->VirtualChunk.kernel;
    node->VirtualChunk.kvaddr = gcvNULL;

    /* Insert chunk behind specified chunk. */
    node->VirtualChunk.next = Node->VirtualChunk.next;
    node->VirtualChunk.prev = Node;
    Node->VirtualChunk.next = node;
    node->VirtualChunk.next->VirtualChunk.prev = node;

    /* Insert free chunk behind specified chunk. */
    node->VirtualChunk.nextFree = Node->VirtualChunk.nextFree;
    node->VirtualChunk.prevFree = Node;
    Node->VirtualChunk.nextFree = node;
    node->VirtualChunk.nextFree->VirtualChunk.prevFree = node;

    /* Adjust size of specified chunk. */
    Node->VirtualChunk.bytes = Bytes;

OnError:
    return status;
}

static gceSTATUS
_MergeVirtualChunk(IN gckOS Os, IN gcuVIDMEM_NODE_PTR Node)
{
    gcuVIDMEM_NODE_PTR node;
    gceSTATUS          status = gcvSTATUS_OK;

    node = Node->VirtualChunk.next;

    if (Node->VirtualChunk.offset + Node->VirtualChunk.bytes !=
        node->VirtualChunk.offset) {
        /* Corrupted heap. */
        gcmkASSERT(Node->VirtualChunk.offset + Node->VirtualChunk.bytes ==
                   node->VirtualChunk.offset);

        return gcvSTATUS_HEAP_CORRUPTED;
    }

    /* Merge. */
    Node->VirtualChunk.bytes += node->VirtualChunk.bytes;

    /* Unlink next node from linked list. */
    Node->VirtualChunk.next     = node->VirtualChunk.next;
    Node->VirtualChunk.nextFree = node->VirtualChunk.nextFree;

    Node->VirtualChunk.next->VirtualChunk.prev         = Node;
    Node->VirtualChunk.nextFree->VirtualChunk.prevFree = Node;

    /* Free the next node. */
    status = gcmkOS_SAFE_FREE(Os, node);
    return status;
}

static gctBOOL
_IsVidMemBlockFree(IN gckVIDMEM_BLOCK VidMemBlock)
{
    return (VidMemBlock->freeBytes == VidMemBlock->bytes);
}

static gcuVIDMEM_NODE_PTR
_FindVirtualChunkNode(IN gckKERNEL Kernel,
                      IN gckVIDMEM_BLOCK VidMemBlock,
                      IN gctSIZE_T Bytes)
{
    gcuVIDMEM_NODE_PTR node;

    if (VidMemBlock->node.VirtualChunk.nextFree == gcvNULL) {
        /* No free chunk left. */
        return gcvNULL;
    }

    for (node = VidMemBlock->node.VirtualChunk.nextFree;
         node->VirtualChunk.bytes != 0;
         node = node->VirtualChunk.nextFree) {
        if (node->VirtualChunk.bytes >= Bytes) {
            /* Got it. */
            return node;
        }
    }

    return gcvNULL;
}

/*******************************************************************************
 **
 ** _ConvertPhysical
 **
 **  Convert CPU physical to GPU address for video node.
 **
 **  INPUT:
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gcuVIDMEM_NODE_PTR Node
 **          Pointer to a gcuVIDMEM_NODE union.
 **
 **      gceCORE Core
 **          Id of current GPU.
 **
 **      gctPHYS_ADDR_T PhysicalAddress
 **          CPU physical address
 **
 **  OUTPUT:
 **      gctADDRESS *Address
 **          A pointer hold the GPU address.
 */
static gceSTATUS
_ConvertPhysical(IN gckKERNEL Kernel,
                 IN gceCORE Core,
                 IN gcuVIDMEM_NODE_PTR Node,
                 IN gckVIDMEM_BLOCK VidMemBlock,
                 IN gctPHYS_ADDR_T PhysicalAddress,
                 OUT gctADDRESS *Address)
{
    gceSTATUS status;
    gctUINT64 physical = 0;

    gcmkHEADER_ARG("Node=%p", Node);

    if ((Node && !Node->Virtual.contiguous) ||
        (VidMemBlock && !VidMemBlock->contiguous)) {
        /* non-contiguous, mapping is required. */
        status = gcvSTATUS_NOT_SUPPORTED;
        goto OnError;
    }

    if ((Node && Node->Virtual.secure) ||
        (VidMemBlock && VidMemBlock->secure)) {
        /* Secure, mapping is forced. */
        status = gcvSTATUS_NOT_SUPPORTED;
        goto OnError;
    }

    /* Convert to GPU physical address. */
    gckOS_CPUPhysicalToGPUPhysical(Kernel->os, PhysicalAddress, &physical);

#if gcdENABLE_VG
    if (Core == gcvCORE_VG) {
        /* VG MMU can always support contiguous. */
        *Address = physical;
        gcmkFOOTER_ARG("*Address=0x%llx", *Address);
        return gcvSTATUS_OK;
    }
#endif

    if (physical > gcvMAXUINT32 ||
        (Node && (physical + Node->Virtual.bytes - 1 > gcvMAXUINT32)) ||
        (VidMemBlock && (physical + VidMemBlock->bytes - 1 > gcvMAXUINT32))) {
        /* Above 4G (32bit), mapping is required currently. */
        status = gcvSTATUS_NOT_SUPPORTED;
        goto OnError;
    }

    if (!gckHARDWARE_IsFeatureAvailable(Kernel->hardware, gcvFEATURE_MMU)) {
        if (physical < Kernel->hardware->baseAddress)
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

        /* Subtract baseAddress to get a GPU address used for programming. */
        physical -= Kernel->hardware->baseAddress;

        /* 2G upper is virtual space, better to move to gckHARDWARE section. */
        if (Node && (physical + Node->Virtual.bytes > 0x80000000U)) {
            /* End is above 2G, ie virtual space. */
            status = gcvSTATUS_NOT_SUPPORTED;
            goto OnError;
        }

        *Address = (gctADDRESS)physical;

        gcmkFOOTER_ARG("*Address=0x%llx", *Address);
        return gcvSTATUS_OK;
    } else {
        gctBOOL    flatMapped;
        gctSIZE_T  bytes   = 1;
        gctADDRESS address = gcvINVALID_ADDRESS;

        if (Node)
            bytes = Node->Virtual.bytes;
        else if (VidMemBlock)
            bytes = VidMemBlock->bytes;
        else
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

        gcmkONERROR(gckMMU_IsFlatMapped(Kernel->mmu, physical,
                                        bytes, &flatMapped, &address));

        if (!flatMapped) {
            status = gcvSTATUS_NOT_SUPPORTED;
            goto OnError;
        }

        *Address = address;

        gcmkFOOTER_ARG("*Address=0x%llx", *Address);
        return gcvSTATUS_OK;
    }

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_MapVidMemBlock(IN gckKERNEL Kernel, IN gckVIDMEM_BLOCK VidMemBlock)
{
    gceSTATUS        status;
    gckOS            os = Kernel->os;
    gctPHYS_ADDR_T   physAddr;
    gctUINT32        index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;
#endif

    gcmkHEADER_ARG("Kernel=%p VidMemBlock=%p", Kernel, VidMemBlock);

#if gcdSHARED_PAGETABLE
    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));
    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    gcmkVERIFY_ARGUMENT(VidMemBlock != gcvNULL);
    gcmkASSERT(VidMemBlock->pageCount > 0);

    gcmkONERROR(gckOS_GetPhysicalFromHandle(os, VidMemBlock->physical, 0, &physAddr));

    status = _ConvertPhysical(Kernel, Kernel->core,
                              gcvNULL, VidMemBlock, physAddr,
                              &VidMemBlock->addresses[index]);
    if (gcmIS_ERROR(status)) {
        gctSIZE_T pageCount = VidMemBlock->pageCount;

        /* If physical address is not aligned. */
        if (physAddr & (gcd1M_PAGE_SIZE - 1)) {
            if (VidMemBlock->contiguous)
                pageCount++;
            else
                gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        }

        VidMemBlock->fixedPageCount = (gctUINT32)pageCount;

        /* Allocate pages inside the MMU. */
        gcmkONERROR(gckMMU_AllocatePagesEx(Kernel->mmu,
                                           pageCount,
                                           VidMemBlock->type,
                                           gcvPAGE_TYPE_1M,
                                           VidMemBlock->lowVA,
                                           VidMemBlock->secure,
                                           &VidMemBlock->pageTables[index],
                                           &VidMemBlock->addresses[index]));

        if (VidMemBlock->onFault != gcvTRUE) {
            /* Map the pages. */
            gcmkONERROR(gckOS_Map1MPages(os, Kernel,
                                         VidMemBlock->physical,
                                         pageCount,
                                         VidMemBlock->addresses[index],
                                         VidMemBlock->pageTables[index],
                                         gcvTRUE,
                                         VidMemBlock->type));
        }

        gcmkONERROR(gckMMU_Flush(Kernel->mmu, VidMemBlock->type));

        /* Calculate the GPU virtual address. */
        VidMemBlock->addresses[index] |= (physAddr & ((1 << 20) - 1));
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Mapped video memory block %p to 0x%08X",
                   VidMemBlock, VidMemBlock->addresses[index]);

    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (VidMemBlock->pageTables[index] != gcvNULL) {
        /* Free the pages from the MMU. */
        gcmkVERIFY_OK(gckMMU_FreePages(Kernel->mmu,
                                       VidMemBlock->secure,
                                       gcvPAGE_TYPE_1M,
                                       VidMemBlock->lowVA,
                                       VidMemBlock->addresses[index],
                                       VidMemBlock->pageTables[index],
                                       VidMemBlock->fixedPageCount));

        VidMemBlock->pageTables[index] = gcvNULL;
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_UnmapVidMemBlock(IN gckKERNEL Kernel, IN gckVIDMEM_BLOCK VidMemBlock)
{
    gceSTATUS status;
    gctUINT32 index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;
#endif

    gcmkHEADER_ARG("Kernel=%p VidMemBlock=%p", Kernel, VidMemBlock);

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);
    gcmkVERIFY_ARGUMENT(VidMemBlock != gcvNULL);

#if gcdSHARED_PAGETABLE
    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));

    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    if (VidMemBlock->pageTables[index] != gcvNULL) {
        /* Free the pages from the MMU. */
        gcmkONERROR(gckMMU_FreePages(Kernel->mmu, VidMemBlock->secure,
                                     gcvPAGE_TYPE_1M,
                                     VidMemBlock->lowVA,
                                     VidMemBlock->addresses[index],
                                     VidMemBlock->pageTables[index],
                                     VidMemBlock->fixedPageCount));

        VidMemBlock->pageTables[index] = gcvNULL;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_BLOCK_Construct(IN gckKERNEL Kernel,
                          IN gctSIZE_T BlockSize,
                          IN gceVIDMEM_TYPE Type,
                          IN gctUINT32 Flag,
                          OUT gckVIDMEM_BLOCK *VidMemBlock)
{
    gceSTATUS          status;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gckOS              os          = Kernel->os;
    gctPOINTER         pointer;

    gcmkHEADER_ARG("Kernel=%p BlockSize=%zu Type=%d Flag=0x%x",
                   Kernel, BlockSize, Type, Flag);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(BlockSize > 0);
    gcmkVERIFY_ARGUMENT(VidMemBlock != gcvNULL);

    /* Allocate an gckVIDMEM_BLOCK object. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(gcsVIDMEM_BLOCK), &pointer));

    gcmkVERIFY_OK(gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsVIDMEM_BLOCK)));

    vidMemBlock = pointer;

    /* Initialize the gckVIDMEM_BLOCK object. */
    vidMemBlock->object.type = gcvOBJ_VIDMEM_BLOCK;
    vidMemBlock->os          = os;
    vidMemBlock->bytes       = BlockSize;
    vidMemBlock->freeBytes   = BlockSize;
    /* 1M page count. */
    vidMemBlock->pageCount   = (gctUINT32)(BlockSize >> 20);
    vidMemBlock->type        = Type;
    vidMemBlock->contiguous  = Flag & gcvALLOC_FLAG_CONTIGUOUS;
    vidMemBlock->secure      = (Flag & gcvALLOC_FLAG_SECURITY) != 0;
    vidMemBlock->onFault     = (Flag & gcvALLOC_FLAG_ALLOC_ON_FAULT) != 0;
    vidMemBlock->lowVA       = (Flag & (gcvALLOC_FLAG_32BIT_VA | gcvALLOC_FLAG_PRIOR_32BIT_VA)) != 0;
    vidMemBlock->cacheable   = (Flag & gcvALLOC_FLAG_CACHEABLE) != 0;
    vidMemBlock->mutex       = gcvNULL;
    vidMemBlock->physical    = gcvNULL;

    /* Allocate the mutex. */
    gcmkONERROR(gckOS_CreateMutex(os, &vidMemBlock->mutex));

    /* Allocate one gcuVIDMEM_NODE union. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(gcuVIDMEM_NODE), &pointer));

    gcmkVERIFY_OK(gckOS_ZeroMemory(pointer, gcmSIZEOF(gcuVIDMEM_NODE)));

    node = pointer;

    if (!vidMemBlock->contiguous)
        Flag |= gcvALLOC_FLAG_1M_PAGES;

    /* Alloc 1M page size aligned memory block. */
    gcmkONERROR_EX(gckOS_AllocatePagedMemory(os, Kernel, Flag,
                                             &BlockSize,
                                             &vidMemBlock->gid,
                                             &vidMemBlock->physical),
                   gcvSTATUS_OUT_OF_MEMORY);

    /* Map current hardware mmu table with 1M pages for this video memory block. */
    gcmkONERROR(gckVIDMEM_MapVidMemBlock(Kernel, vidMemBlock));

    /* Initialize gcuVIDMEM_NODE union for virtual memory. */
    node->VirtualChunk.kernel  = Kernel;
    node->VirtualChunk.offset  = 0;
    node->VirtualChunk.bytes   = BlockSize;
    node->VirtualChunk.kvaddr  = gcvNULL;
    node->VirtualChunk.logical = gcvNULL;
    node->VirtualChunk.parent  = vidMemBlock;

    /* Initialize the virtual chunk linked-list. */
    node->VirtualChunk.next     =
    node->VirtualChunk.prev     =
    node->VirtualChunk.nextFree =
    node->VirtualChunk.prevFree = &vidMemBlock->node;

    vidMemBlock->node.VirtualChunk.next         =
    vidMemBlock->node.VirtualChunk.prev         =
    vidMemBlock->node.VirtualChunk.nextFree     =
    vidMemBlock->node.VirtualChunk.prevFree     = node;

    vidMemBlock->node.VirtualChunk.bytes = 0;

    *VidMemBlock = vidMemBlock;

    gcmkFOOTER_ARG("*VidMemBlock=%p", *VidMemBlock);

    return gcvSTATUS_OK;

OnError:
    if (vidMemBlock != gcvNULL) {
        if (vidMemBlock->mutex)
            gcmkVERIFY_OK(gckOS_DeleteMutex(os, vidMemBlock->mutex));

        if (vidMemBlock->physical) {
            gcmkVERIFY_OK(gckOS_FreePagedMemory(os, vidMemBlock->physical,
                                                vidMemBlock->bytes));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, vidMemBlock));
    }

    if (node != gcvNULL)
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, node));

    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_BLOCK_Destroy(IN gckKERNEL Kernel, IN gckVIDMEM_BLOCK VidMemBlock)
{
    gckKERNEL ker = gcvNULL;
    gctINT    i   = 0;

    gcmkHEADER_ARG("Kernel=%p VidMemBlock=%p", Kernel, VidMemBlock);

    /* Verify the arguments. */
    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);
    gcmkVERIFY_ARGUMENT(VidMemBlock != gcvNULL);

    if (VidMemBlock->physical) {
        gcmkVERIFY_OK(gckOS_FreePagedMemory(Kernel->os,
                                            VidMemBlock->physical,
                                            VidMemBlock->bytes));
    }

    for (i = 0; i < gcvCORE_COUNT; i++) {
        gcmkVERIFY_OK(gckOS_QueryKernel(Kernel, i, &ker));

        if (ker)
            gcmkVERIFY_OK(_UnmapVidMemBlock(ker, VidMemBlock));
    }

    /* Free the mutex. */
    gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, VidMemBlock->mutex));

    /* Free the virtual chunk. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, VidMemBlock->node.VirtualChunk.next));

    /* Free the video memory block. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, VidMemBlock));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

static gceSTATUS
_AllocateVirtualChunk(IN gckKERNEL Kernel,
                      IN gckVIDMEM_BLOCK VidMemBlock,
                      IN gceVIDMEM_TYPE Type,
                      INOUT gctSIZE_T *Bytes,
                      OUT gcuVIDMEM_NODE_PTR *Node)
{
    gceSTATUS          status   = gcvSTATUS_OK;
    gctBOOL            acquired = gcvFALSE;
    gcuVIDMEM_NODE_PTR node;
    gctSIZE_T          bytes;

    gcmkHEADER_ARG("Kernel=%p VidMemBlock=%p Type=%d Bytes=%zx",
                   Kernel, VidMemBlock, Type, *Bytes);

    gcmkVERIFY_ARGUMENT(Node != gcvNULL);
    gcmkVERIFY_ARGUMENT(VidMemBlock != gcvNULL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Type < gcvVIDMEM_TYPE_COUNT);

    gcmkONERROR(gckOS_AcquireMutex(Kernel->os, VidMemBlock->mutex, gcvINFINITE));

    acquired = gcvTRUE;

    bytes = gcmALIGN(*Bytes, 4096);

    if (bytes > VidMemBlock->freeBytes) {
        /* No enough memory. */
        status = gcvSTATUS_OUT_OF_MEMORY;
        goto OnError;
    }

    node = _FindVirtualChunkNode(Kernel, VidMemBlock, bytes);
    if (node == gcvNULL) {
        status = gcvSTATUS_OUT_OF_MEMORY;
        goto OnError;
    }

    if (node->VirtualChunk.bytes > bytes) {
        /* Split the chunk. */
        _SplitVirtualChunk(Kernel->os, node, bytes);
    }

    /* Remove the chunk from the free list. */
    node->VirtualChunk.prevFree->VirtualChunk.nextFree = node->VirtualChunk.nextFree;
    node->VirtualChunk.nextFree->VirtualChunk.prevFree = node->VirtualChunk.prevFree;
    node->VirtualChunk.nextFree                        = gcvNULL;
    node->VirtualChunk.prevFree                        = gcvNULL;

    /* Fill in the information. */
    node->VirtualChunk.parent = VidMemBlock;
    gcmkVERIFY_OK(gckOS_GetProcessID(&node->VirtualChunk.processID));
    VidMemBlock->freeBytes -= node->VirtualChunk.bytes;

    *Bytes = bytes;
    *Node  = node;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, VidMemBlock->mutex));
    }

    return status;
}

static gceSTATUS
gckVIDMEM_AllocateVirtualChunk(IN gckKERNEL Kernel, IN gceVIDMEM_TYPE Type,
                               IN gctUINT32 Flag, IN gctSIZE_T Bytes,
                               OUT gcuVIDMEM_NODE_PTR *Node)
{
    gckOS              os;
    gceSTATUS          status;
    gcuVIDMEM_NODE_PTR node;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gctSIZE_T          blockSize;
    gctBOOL            acquired = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p Flag=%x Bytes=%zu", Kernel, Flag, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Bytes > 0);
    gcmkVERIFY_ARGUMENT(Node != gcvNULL);

    /* Extract the gckOS object pointer. */
    os = Kernel->os;
    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Acquire the vidMem block mutex */
    gcmkONERROR(gckOS_AcquireMutex(os, Kernel->vidMemBlockMutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Find the free vidmem block. */
    vidMemBlock = _FindFreeBlock(Kernel, Bytes, Flag);
    if (!vidMemBlock) {
        /* Not found, construct new block. */
        blockSize = gcmALIGN(Bytes, gcd1M_PAGE_SIZE);

        gcmkONERROR_EX(gckVIDMEM_BLOCK_Construct(Kernel, blockSize, Type,
                                                 Flag, &vidMemBlock),
                       gcvSTATUS_OUT_OF_MEMORY);

        gcmkONERROR(_AddToBlockList(Kernel, vidMemBlock));
    }

    /* Allocate virtual chunk node in the found block. */
    gcmkONERROR_EX(_AllocateVirtualChunk(Kernel, vidMemBlock,
                                         Type, &Bytes, &node),
                   gcvSTATUS_OUT_OF_MEMORY);

    /* Return pointer to the gcuVIDMEM_NODE union. */
    *Node = node;

    /* Release the vidMem block mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, Kernel->vidMemBlockMutex));

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Created virtual node %p for %zu bytes @ %p",
                   node, Bytes, node->Virtual.physical);

    /* Success. */
    gcmkFOOTER_ARG("*Node=%p", *Node);
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the vidMem block mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, Kernel->vidMemBlockMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_Free
 **
 **  Free an allocated video memory node.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gcuVIDMEM_NODE_PTR Node
 **          Pointer to a gcuVIDMEM_NODE object.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
static gceSTATUS
gckVIDMEM_Free(IN gckKERNEL Kernel, IN gcuVIDMEM_NODE_PTR Node)
{
    gceSTATUS          status;
    gckKERNEL          kernel      = gcvNULL;
    gckVIDMEM          memory      = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gcuVIDMEM_NODE_PTR node;
    gctBOOL            mutexAcquired       = gcvFALSE;
    gctBOOL            vbMutexAcquired     = gcvFALSE;
    gctBOOL            vbListMutexAcquired = gcvFALSE;
    gctUINT64          mappingInOne        = 1;

    gcmkHEADER_ARG("Node=%p", Node);

    /* Verify the arguments. */
    if (Node == gcvNULL || Node->VidMem.parent == gcvNULL) {
        /* Invalid object. */
        gcmkONERROR(gcvSTATUS_INVALID_OBJECT);
    }

    vidMemBlock = Node->VirtualChunk.parent;

    /**************************** Video Memory ********************************/

    if (Node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        /* Extract pointer to gckVIDMEM object owning the node. */
        memory = Node->VidMem.parent;

        /* Acquire the mutex. */
        gcmkONERROR(gckOS_AcquireMutex(memory->os, memory->mutex, gcvINFINITE));

        mutexAcquired = gcvTRUE;

        if (Node->VidMem.kvaddr) {
#if gcdCAPTURE_ONLY_MODE
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, Node->VidMem.kvaddr));
#else
            gcmkONERROR(gckOS_DestroyKernelMapping(Kernel->os,
                                                   Node->VidMem.parent->physical,
                                                   Node->VidMem.kvaddr));
#endif

            Node->VidMem.kvaddr = gcvNULL;
        }

#ifdef __QNXNTO__
        /* Unmap the video memory. */
        if (Node->VidMem.logical != gcvNULL) {
            gckKERNEL_UnmapVideoMemory(Kernel, Node->VidMem.pool,
                                       Node->VidMem.physical,
                                       Node->VidMem.logical,
                                       Node->VidMem.processID,
                                       Node->VidMem.bytes);

            Node->VidMem.logical = gcvNULL;
        }

        /* Reset. */
        Node->VidMem.processID = 0;

        /* Don't try to re-free an already freed node. */
        if (Node->VidMem.nextFree == gcvNULL &&
            Node->VidMem.prevFree == gcvNULL)
#endif
        {
#if gcdENABLE_VG
            if (Node->VidMem.kernelVirtual) {
                gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                               "%s(%d) Unmap %p from kernel space.",
                               __FUNCTION__, __LINE__,
                               Node->VidMem.kernelVirtual);

                gcmkVERIFY_OK(gckOS_UnmapPhysical(memory->os,
                                                  Node->VidMem.kernelVirtual,
                                                  Node->VidMem.bytes));

                Node->VidMem.kernelVirtual = gcvNULL;
            }
#endif

            /* Check if Node is already freed. */
            if (Node->VidMem.nextFree) {
                /* Node is alread freed. */
                gcmkONERROR(gcvSTATUS_INVALID_DATA);
            }

            gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne);
            if (!mappingInOne) {
                gckOS_ReleaseReservedMemoryArea(Node->VidMem.physical);
                Node->VidMem.physical = gcvNULL;
            }

            /* Update the number of free bytes. */
            memory->freeBytes += Node->VidMem.bytes;

            /* Find the next free node. */
            for (node = Node->VidMem.next;
                 node != gcvNULL && node->VidMem.nextFree == gcvNULL;
                 node = node->VidMem.next)
                ;

            if (node == gcvNULL)
                gcmkONERROR(gcvSTATUS_INVALID_DATA);

            /* Insert this node in the free list. */
            Node->VidMem.nextFree = node;
            Node->VidMem.prevFree = node->VidMem.prevFree;

            Node->VidMem.prevFree->VidMem.nextFree = Node;
            node->VidMem.prevFree                  = Node;

            /* Is the next node a free node and not the sentinel? */
            if (Node->VidMem.next == Node->VidMem.nextFree &&
                Node->VidMem.next->VidMem.bytes != 0) {
                /* Merge this node with the next node. */
                gcmkONERROR(_Merge(memory->os, node = Node));
                gcmkASSERT(node->VidMem.nextFree != node);
                gcmkASSERT(node->VidMem.prevFree != node);
            }

            /* Is the previous node a free node and not the sentinel? */
            if (Node->VidMem.prev == Node->VidMem.prevFree &&
                Node->VidMem.prev->VidMem.bytes != 0) {
                /* Merge this node with the previous node. */
                gcmkONERROR(_Merge(memory->os, node = Node->VidMem.prev));
                gcmkASSERT(node->VidMem.nextFree != node);
                gcmkASSERT(node->VidMem.prevFree != node);
            }
        }

        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(memory->os, memory->mutex));

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM, "Node %p is freed.", Node);

        /* Success. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        /* Acquire the vidMem block mutex */
        gcmkONERROR(gckOS_AcquireMutex(Kernel->os,
                                       Kernel->vidMemBlockMutex,
                                       gcvINFINITE));
        vbListMutexAcquired = gcvTRUE;

        if (vidMemBlock) {
            gckOS os = vidMemBlock->os;

            gcmkONERROR(gckOS_AcquireMutex(os, vidMemBlock->mutex, gcvINFINITE));
            vbMutexAcquired = gcvTRUE;
            kernel          = Node->VirtualChunk.kernel;

            if (Kernel != kernel) {
                gcmkFATAL("ERROR: You allocate vidMemBLock on core[%d], but try to free it on core[%d]",
                          kernel->core, Kernel->core);
            }

            if (Node->VirtualChunk.kvaddr) {
                gcmkONERROR(gckOS_DestroyKernelMapping(kernel->os,
                                                       vidMemBlock->physical,
                                                       Node->VirtualChunk.kvaddr));

                Node->VirtualChunk.kvaddr = gcvNULL;
            }

            /* Handle the free chunk in the linked-list */
            {
                /* Check if chunk is in free list. */
                if (Node->VirtualChunk.nextFree) {
                    /* Chunk is already freed. */
                    gcmkONERROR(gcvSTATUS_INVALID_DATA);
                }

                vidMemBlock->freeBytes += Node->VirtualChunk.bytes;

                /* Find the next free chunk. */
                for (node = Node->VirtualChunk.next;
                     node != gcvNULL && node->VirtualChunk.nextFree == gcvNULL;
                     node = node->VirtualChunk.next)
                    ;

                if (node == gcvNULL)
                    gcmkONERROR(gcvSTATUS_INVALID_DATA);

                /* Insert this chunk in the free list. */
                Node->VirtualChunk.nextFree = node;
                Node->VirtualChunk.prevFree = node->VirtualChunk.prevFree;

                Node->VirtualChunk.prevFree->VirtualChunk.nextFree =
                node->VirtualChunk.prevFree = Node;

                /* Is the next chunk a free chunk. */
                if (Node->VirtualChunk.next == Node->VirtualChunk.nextFree &&
                    Node->VirtualChunk.next->VirtualChunk.bytes != 0) {
                    /* Merge this chunk with the next chunk. */
                    gcmkONERROR(_MergeVirtualChunk(os, node = Node));
                    gcmkASSERT(node->VirtualChunk.nextFree != node);
                    gcmkASSERT(node->VirtualChunk.prevFree != node);
                }

                /* Is the previous chunk a free chunk. */
                if (Node->VirtualChunk.prev == Node->VirtualChunk.prevFree &&
                    Node->VirtualChunk.prev->VirtualChunk.bytes != 0) {
                    /* Merge this chunk with the previous chunk. */
                    gcmkONERROR(_MergeVirtualChunk(os, node = Node->VirtualChunk.prev));
                    gcmkASSERT(node->VirtualChunk.nextFree != node);
                    gcmkASSERT(node->VirtualChunk.prevFree != node);
                }
            }

            /* Release the mutex. */
            gcmkVERIFY_OK(gckOS_ReleaseMutex(os, vidMemBlock->mutex));

            /* Only free the vidmem block when all the chunks are freed. */
            if (_IsVidMemBlockFree(vidMemBlock)) {
                gcmkONERROR(_RemoveFromBlockList(kernel, vidMemBlock));

                gcmkONERROR(gckVIDMEM_BLOCK_Destroy(kernel, vidMemBlock));
            }
        }

        /* Release the vidMem block mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->vidMemBlockMutex));

        /* Success. */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    /*************************** Virtual Memory *******************************/

    /* Get gckKERNEL object. */
    kernel = Node->Virtual.kernel;

    /* Verify the gckKERNEL object pointer. */
    gcmkVERIFY_OBJECT(kernel, gcvOBJ_KERNEL);

#if gcdENABLE_VG
    if (Node->Virtual.kernelVirtual) {
        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                       "%s(%d) Unmap %p from kernel space.",
                       __FUNCTION__, __LINE__,
                       Node->Virtual.kernelVirtual);

        gcmkVERIFY_OK(gckOS_UnmapPhysical(kernel->os,
                                          Node->Virtual.kernelVirtual,
                                          Node->Virtual.bytes));

        Node->Virtual.kernelVirtual = gcvNULL;
    }
#endif

    if (Node->Virtual.kvaddr) {
        gcmkVERIFY_OK(gckOS_DestroyKernelMapping(kernel->os,
                                                 Node->Virtual.physical,
                                                 Node->Virtual.kvaddr));
    }

    /* Free the virtual memory. */
    gcmkVERIFY_OK(gckOS_FreePagedMemory(kernel->os,
                                        Node->Virtual.physical,
                                        Node->Virtual.bytes));

    /* Delete the gcuVIDMEM_NODE union. */
    gcmkVERIFY_OK(gcmkOS_SAFE_FREE(kernel->os, Node));

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (mutexAcquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(memory->os, memory->mutex));
    }

    if (vbMutexAcquired) {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(vidMemBlock->os,
                                         vidMemBlock->mutex));
    }

    if (vbListMutexAcquired) {
        gcmkVERIFY_OK(gckOS_ReleaseMutex(vidMemBlock->os,
                                         Kernel->vidMemBlockMutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_Lock
 **
 **  Lock a video memory node and return its hardware specific address.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gcuVIDMEM_NODE_PTR Node
 **          Pointer to a gcuVIDMEM_NODE union.
 **
 **  OUTPUT:
 **
 **      gctADDRESS *Address
 **          Pointer to a variable that will hold the hardware specific address.
 **
 */
static gceSTATUS
gckVIDMEM_Lock(IN gckKERNEL Kernel,
               IN gcuVIDMEM_NODE_PTR Node,
               OUT gctADDRESS *Address)
{
    gcmkHEADER_ARG("Kernel=%p Node=%p", Kernel, Node);

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);
    gcmkVERIFY_ARGUMENT(Kernel->device != gcvNULL);

    /* Increment the lock count. */
    if (Node->VidMem.locked++ == 0) {
        gctADDRESS address;
        gctADDRESS offset = (gctADDRESS)Node->VidMem.offset;

        switch (Node->VidMem.pool) {
        case gcvPOOL_LOCAL_EXCLUSIVE:
            address = Kernel->exclusiveBaseAddress + offset;
            break;
        case gcvPOOL_LOCAL_EXTERNAL:
            address = Kernel->externalBaseAddress + offset;
            break;
        case gcvPOOL_LOCAL_INTERNAL:
            address = Kernel->internalBaseAddress + offset;
            break;
        case gcvPOOL_INTERNAL_SRAM:
            address = Kernel->sRAMBaseAddresses[Kernel->sRAMIndex] + offset;
            break;
        case gcvPOOL_EXTERNAL_SRAM:
            address = Kernel->extSRAMBaseAddresses[Kernel->device->extSRAMIndex] + offset;
            break;
        case gcvPOOL_SYSTEM_32BIT_VA:
            address = Kernel->lowContiguousBaseAddress + offset;
            break;

        default:
            gcmkASSERT(Node->VidMem.pool == gcvPOOL_SYSTEM);
            /* FALLTHRU */
            gcmkFALLTHRU;
        case gcvPOOL_SYSTEM:
            address = Kernel->contiguousBaseAddresses[Kernel->device->memIndex] + offset;
            break;
        }

        /* Save address. */
        Node->VidMem.address = address;
    }

    *Address = Node->VidMem.address;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Locked node 0x%x (%d) @ 0x%llx", Node,
                   Node->VidMem.locked, *Address);

    gcmkFOOTER_ARG("*Address=0x%llx", *Address);
    return gcvSTATUS_OK;
}

static gceSTATUS
gckVIDMEM_LockVirtual(IN gckKERNEL Kernel, IN gcuVIDMEM_NODE_PTR Node,
                      OUT gctADDRESS *Address)
{
    gceSTATUS      status;
    gctPHYS_ADDR_T physicalAddress;
    gctBOOL        locked = gcvFALSE;
    gckOS          os     = Kernel->os;
    gctUINT32      index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;
#endif

    gcmkHEADER_ARG("Kernel=%p Node=%p", Kernel, Node);

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

#if gcdSHARED_PAGETABLE
    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));
    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    gcmkONERROR(gckOS_GetPhysicalFromHandle(os, Node->Virtual.physical,
                                            0, &physicalAddress));

#if !gcdENABLE_VG
    gcmkVERIFY_ARGUMENT(Kernel->hardware != gcvNULL);

    if (!Kernel->hardware->options.enableMMU) {
        if (physicalAddress >= ((gctUINT64)1 << 32)) {
            gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        } else {
            Node->Virtual.addresses[index] = (gctADDRESS)physicalAddress;
            *Address                       = Node->Virtual.addresses[index];

            gcmkFOOTER_ARG("*Address=0x%llx", *Address);
            return gcvSTATUS_OK;
        }
    }
#else
    Node->Virtual.physicalAddress = physicalAddress;
#endif

    /* Increment the lock count. */
    if (Node->Virtual.lockeds[index]++ == 0) {
        locked = gcvTRUE;

        status = _ConvertPhysical(Kernel, Kernel->core, Node,
                                  gcvNULL, physicalAddress,
                                  &Node->Virtual.addresses[index]);

        if (gcmIS_ERROR(status)) {
            /* Do GPU address mapping. */
#if gcdSECURITY
            gctPHYS_ADDR physicalArrayPhysical;
            gctPOINTER   physicalArrayLogical;

            gcmkONERROR(gckOS_AllocatePageArray(os, Kernel,
                                                Node->Virtual.physical,
                                                Node->Virtual.pageCount,
                                                &physicalArrayLogical,
                                                &physicalArrayPhysical));

            gcmkONERROR(gckKERNEL_SecurityMapMemory(Kernel, physicalArrayLogical,
                                                    Node->Virtual.pageCount,
                                                    &Node->Virtual.addresses[index]));

            gcmkONERROR(gckOS_FreeNonPagedMemory(os, physicalArrayPhysical,
                                                 physicalArrayLogical, 1));
#else
#if gcdENABLE_VG
            if (Kernel->vg != gcvNULL) {
                gctUINT32 vgAddr;

                /* Allocate pages inside the MMU. */
                gcmkONERROR(gckVGMMU_AllocatePages(Kernel->vg->mmu,
                                                   Node->Virtual.pageCount,
                                                   &Node->Virtual.pageTables[index],
                                                   &vgAddr));

                Node->Virtual.addresses[index] = vgAddr;
            } else {
#    endif
                /* Allocate pages inside the MMU. */
                gcmkONERROR(gckMMU_AllocatePagesEx(Kernel->mmu, Node->Virtual.pageCount,
                                                   Node->Virtual.type,
                                                   gcvPAGE_TYPE_4K,
                                                   Node->Virtual.lowVA, Node->Virtual.secure,
                                                   &Node->Virtual.pageTables[index],
                                                   &Node->Virtual.addresses[index]));
#if gcdENABLE_VG
            }
#    endif

            if (Node->Virtual.onFault != gcvTRUE) {
#if gcdENABLE_TRUST_APPLICATION
#if gcdENABLE_VG
                if (Kernel->core != gcvCORE_VG &&
                    Kernel->hardware->options.secureMode == gcvSECURE_IN_TA)
#        else
                if (Kernel->hardware->options.secureMode == gcvSECURE_IN_TA)
#        endif
                {
                    gcmkONERROR(gckKERNEL_MapInTrustApplicaiton(Kernel,
                                                                Node->Virtual.logical,
                                                                Node->Virtual.physical,
                                                                Node->Virtual.addresses[index],
                                                                Node->Virtual.pageCount));
                } else {
#    endif
                    gcmkDUMP(os, "#[mmu: dynamic mapping: address=0x%08X pageCount=%lu]",
                             Node->Virtual.addresses[index],
                             (unsigned long)Node->Virtual.pageCount);

                    /* Map the pages. */
                    gcmkONERROR(gckOS_MapPagesEx(os, Kernel,
                                                 Node->Virtual.physical,
                                                 Node->Virtual.pageCount,
                                                 Node->Virtual.addresses[index],
                                                 Node->Virtual.pageTables[index],
                                                 gcvTRUE,
                                                 Node->Virtual.type));
#if gcdENABLE_TRUST_APPLICATION
                }
#    endif
            }

#if gcdENABLE_VG
            if (gcvNULL != Kernel->vg && Kernel->core == gcvCORE_VG)
                gcmkONERROR(gckVGMMU_Flush(Kernel->vg->mmu));
            else
#    endif
                gcmkONERROR(gckMMU_Flush(Kernel->mmu, Node->Virtual.type));
#endif

            /* GPU MMU page size is fixed at 4096 now. */
            Node->Virtual.addresses[index] |= physicalAddress & (4096 - 1);
        }

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                       "Mapped virtual node %p to 0x%08X", Node,
                       Node->Virtual.addresses[index]);
    }

    /* Return hardware address. */
    *Address = Node->Virtual.addresses[index];

    gcmkFOOTER_ARG("*Address=0x%llx", *Address);
    return gcvSTATUS_OK;

OnError:
    if (locked) {
        if (Node->Virtual.pageTables[index] != gcvNULL) {
#if gcdENABLE_VG
            if (Kernel->vg != gcvNULL) {
                /* Free the pages from the MMU. */
                gcmkVERIFY_OK(gckVGMMU_FreePages(Kernel->vg->mmu,
                                                 Node->Virtual.pageTables[index],
                                                 Node->Virtual.pageCount));
            } else {
#endif
                /* Free the pages from the MMU. */
                gcmkVERIFY_OK(gckMMU_FreePages(Kernel->mmu,
                                               Node->Virtual.secure,
                                               gcvPAGE_TYPE_4K,
                                               Node->Virtual.lowVA,
                                               Node->Virtual.addresses[index],
                                               Node->Virtual.pageTables[index],
                                               Node->Virtual.pageCount));
#if gcdENABLE_VG
            }
#endif

            Node->Virtual.pageTables[index] = gcvNULL;
        }

        Node->Virtual.lockeds[index]--;
    }

    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_LockVirtualChunk(IN gckKERNEL Kernel,
                           IN gcuVIDMEM_NODE_PTR Node,
                           OUT gctADDRESS *Address)
{
    gceSTATUS       status      = gcvSTATUS_OK;
    gckVIDMEM_BLOCK vidMemBlock = Node->VirtualChunk.parent;
    gctUINT32       index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;
#endif

    gcmkHEADER_ARG("Kernel=%p Node=%p", Kernel, Node);

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

#if gcdSHARED_PAGETABLE
    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));
    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    gcmkASSERT(vidMemBlock != gcvNULL);

#if !gcdENABLE_VG
    gcmkVERIFY_ARGUMENT(Kernel->hardware != gcvNULL);

    if (!Kernel->hardware->options.enableMMU) {
        gctPHYS_ADDR_T physAddr;

        gcmkONERROR(gckOS_GetPhysicalFromHandle(Kernel->os,
                                                vidMemBlock->physical,
                                                0, &physAddr));

        if (physAddr >= ((gctUINT64)1 << 32)) {
            gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        } else {
            vidMemBlock->addresses[index] = (gctADDRESS)physAddr;

            Node->VirtualChunk.addresses[index] =
                vidMemBlock->addresses[index] + (gctADDRESS)Node->VirtualChunk.offset;
        }
    }
#endif

    /* Increment the lock count. */
    if (Node->VirtualChunk.lockeds[index]++ == 0) {
        if (!vidMemBlock->pageTables[index]) {
            /* Map current hardware mmu table with 1M pages for this video memory block. */
            gcmkONERROR(gckVIDMEM_MapVidMemBlock(Kernel, vidMemBlock));
        }

        Node->VirtualChunk.addresses[index] =
            vidMemBlock->addresses[index] + (gctADDRESS)Node->VirtualChunk.offset;
    }

    /* Return hardware address. */
    *Address = Node->VirtualChunk.addresses[index];

    gcmkFOOTER_ARG("*Address=0x%llx", *Address);

    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_Unlock
 **
 **  Unlock a video memory node.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gcuVIDMEM_NODE_PTR Node
 **          Pointer to a locked gcuVIDMEM_NODE union.
 **
 **      gctBOOL *Asynchroneous
 **          Pointer to a variable specifying whether the surface should be
 **          unlocked asynchroneously or not.
 **
 **  OUTPUT:
 **
 **      gctBOOL *Asynchroneous
 **          Pointer to a variable receiving the number of bytes used in the
 **          command buffer specified by 'Commands'.  If gcvNULL, there is no
 **          command buffer.
 */
static gceSTATUS
gckVIDMEM_Unlock(IN gckKERNEL Kernel,
                 IN gcuVIDMEM_NODE_PTR Node,
                 IN OUT gctBOOL *Asynchroneous)
{
    gceSTATUS status;

    gcmkHEADER_ARG("Node=%p *Asynchroneous=%d",
                   Node, gcmOPT_VALUE(Asynchroneous));

    if (Node->VidMem.locked <= 0) {
        /* The surface was not locked. */
        gcmkONERROR(gcvSTATUS_MEMORY_UNLOCKED);
    }

    if (Asynchroneous != gcvNULL) {
        /* Schedule an event to sync with GPU. */
        *Asynchroneous = gcvTRUE;
    } else {
        /* Decrement the lock count. */
        Node->VidMem.locked--;
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Unlocked node %p (%d)",
                   Node, Node->VidMem.locked);

    /* Success. */
    gcmkFOOTER_ARG("*Asynchroneous=%d", gcmOPT_VALUE(Asynchroneous));
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_UnlockVirtual(IN gckKERNEL Kernel,
                        IN gcuVIDMEM_NODE_PTR Node,
                        IN OUT gctBOOL *Asynchroneous)
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 index;

    gcmkHEADER_ARG("Node=%p *Asynchroneous=%d",
                   Node, gcmOPT_VALUE(Asynchroneous));

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

#if !gcdENABLE_VG
    gcmkVERIFY_ARGUMENT(Kernel->hardware != gcvNULL);

    if (!Kernel->hardware->options.enableMMU) {
        gcmkFOOTER();
        return status;
    }
#endif

    if (Asynchroneous != gcvNULL) {
        /* Schedule the surface to be unlocked. */
        *Asynchroneous = gcvTRUE;
    } else {
#if gcdSHARED_PAGETABLE
        gceHARDWARE_TYPE hwType;

        gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));

        index = (gctUINT32)hwType;
#else
        index = (gctUINT32)Kernel->core;
#endif

        if (Node->Virtual.lockeds[index] == 0)
            gcmkONERROR(gcvSTATUS_MEMORY_UNLOCKED);

        /* Decrement lock count. */
        --Node->Virtual.lockeds[index];

        /* See if we can unlock the resources. */
        if (Node->Virtual.lockeds[index] == 0) {
            gctADDRESS address;

            /* Adjust address to page aligned for underlying functions. */
            address = Node->Virtual.addresses[index] & ~(4096 - 1);

#if gcdSECURITY
            if (Node->Virtual.addresses[index] > 0x80000000U) {
                gcmkONERROR(gckKERNEL_SecurityUnmapMemory(Kernel, address,
                                                          Node->Virtual.pageCount));
            }
#else
            /* Free the page table. */
            if (Node->Virtual.pageTables[index] != gcvNULL) {
#if gcdENABLE_VG
                if (Kernel->vg != gcvNULL) {
                    gcmkONERROR(gckVGMMU_FreePages(Kernel->vg->mmu,
                                                   Node->Virtual.pageTables[index],
                                                   Node->Virtual.pageCount));
                } else {
#    endif
                    gcmkONERROR(gckMMU_FreePages(Kernel->mmu,
                                                 Node->Virtual.secure,
                                                 gcvPAGE_TYPE_4K,
                                                 Node->Virtual.lowVA,
                                                 address,
                                                 Node->Virtual.pageTables[index],
                                                 Node->Virtual.pageCount));
#if gcdENABLE_VG
                }
#    endif

                gcmkONERROR(gckOS_UnmapPages(Kernel->os,
                                             Node->Virtual.pageCount,
                                             address));

                /* Mark page table as freed. */
                Node->Virtual.pageTables[index] = gcvNULL;
            }
#endif
        }

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                       "Unmapped virtual node %p from 0x%08X", Node,
                       Node->Virtual.addresses[index]);
    }

    /* Success. */
    gcmkFOOTER_ARG("*Asynchroneous=%d", gcmOPT_VALUE(Asynchroneous));
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_UnlockVirtualChunk(IN gckKERNEL Kernel,
                             IN gcuVIDMEM_NODE_PTR Node,
                             IN OUT gctBOOL *Asynchroneous)
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;
#endif

    gcmkHEADER_ARG("Node=%p *Asynchroneous=%d",
                   Node, gcmOPT_VALUE(Asynchroneous));

    gcmkVERIFY_ARGUMENT(Kernel != gcvNULL);

#if !gcdENABLE_VG
    gcmkVERIFY_ARGUMENT(Kernel->hardware != gcvNULL);

    if (!Kernel->hardware->options.enableMMU) {
        gcmkFOOTER();
        return status;
    }
#endif

#if gcdSHARED_PAGETABLE
    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));
    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    if (Asynchroneous != gcvNULL) {
        /* Schedule an event to sync with GPU. */
        *Asynchroneous = gcvTRUE;
    } else {
        if (Node->VirtualChunk.lockeds[index] == 0) {
            /* The surface was not locked. */
            gcmkONERROR(gcvSTATUS_MEMORY_UNLOCKED);
        }

        /* Unmap and free pages when video memory free. */

        /* Decrement the lock count. */
        --Node->VirtualChunk.lockeds[index];
    }

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_VIDMEM,
                   "Unlocked node %p (%d)",
                   Node, Node->VirtualChunk.lockeds[index]);

    /* Success. */
    gcmkFOOTER_ARG("*Asynchroneous=%d", gcmOPT_VALUE(Asynchroneous));
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_HANDLE_Allocate
 **
 **  Allocate a handle for a gckVIDMEM_NODE object.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gckVIDMEM_NODE Node
 **          Pointer to a gckVIDMEM_NODE object.
 **
 **  OUTPUT:
 **
 **      gctUINT32 *Handle
 **          Pointer to a variable receiving a handle represent this
 **          gckVIDMEM_NODE in userspace.
 */
gceSTATUS
gckVIDMEM_HANDLE_Allocate(IN gckKERNEL Kernel, IN gckVIDMEM_NODE Node,
                          OUT gctUINT32 *Handle)
{
    gceSTATUS        status;
    gctUINT32        processID      = 0;
    gctPOINTER       pointer        = gcvNULL;
    gctPOINTER       handleDatabase = gcvNULL;
    gctPOINTER       mutex          = gcvNULL;
    gctUINT32        handle         = 0;
    gckVIDMEM_HANDLE handleObject   = gcvNULL;
    gckOS            os             = Kernel->os;

    gcmkHEADER_ARG("Kernel=%p, Node=%p", Kernel, Node);

    gcmkVERIFY_OBJECT(os, gcvOBJ_OS);

    /* Allocate a gckVIDMEM_HANDLE object. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(gcsVIDMEM_HANDLE), &pointer));

    gcmkVERIFY_OK(gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsVIDMEM_HANDLE)));

    handleObject = pointer;

    gcmkONERROR(gckOS_AtomConstruct(os, &handleObject->reference));

    /* Set default reference count to 1. */
    gckOS_AtomSet(os, handleObject->reference, 1);

    gcmkVERIFY_OK(gckOS_GetProcessID(&processID));

    gcmkONERROR(gckKERNEL_FindHandleDatbase(Kernel, processID,
                                            &handleDatabase, &mutex));

    /* Allocate a handle for this object. */
    gcmkONERROR(gckKERNEL_AllocateIntegerId(handleDatabase, handleObject, &handle));

    handleObject->node   = Node;
    handleObject->handle = handle;

    *Handle = handle;

    gcmkFOOTER_ARG("*Handle=%d", *Handle);
    return gcvSTATUS_OK;

OnError:
    if (handleObject != gcvNULL) {
        if (handleObject->reference != gcvNULL) {
            gcmkVERIFY_OK(gckOS_AtomDestroy(os,
                                            handleObject->reference));
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, handleObject));
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_HANDLE_Reference(IN gckKERNEL Kernel,
                           IN gctUINT32 ProcessID,
                           IN gctUINT32 Handle)
{
    gceSTATUS        status;
    gckVIDMEM_HANDLE handleObject = gcvNULL;
    gctPOINTER       database     = gcvNULL;
    gctPOINTER       mutex        = gcvNULL;
    gctINT32         oldValue     = 0;
    gctBOOL          acquired     = gcvFALSE;

    gcmkHEADER_ARG("Handle=%d PrcoessID=%d", Handle, ProcessID);

    gcmkONERROR(gckKERNEL_FindHandleDatbase(Kernel, ProcessID,
                                            &database, &mutex));

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Translate handle to gckVIDMEM_HANDLE object. */
    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Handle,
                                         (gctPOINTER *)&handleObject));

    /* Increase the reference count. */
    gckOS_AtomIncrement(Kernel->os, handleObject->reference, &oldValue);

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));
    acquired = gcvFALSE;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_HANDLE_Dereference(IN gckKERNEL Kernel,
                             IN gctUINT32 ProcessID,
                             IN gctUINT32 Handle)
{
    gceSTATUS        status;
    gctPOINTER       handleDatabase = gcvNULL;
    gctPOINTER       mutex          = gcvNULL;
    gctINT32         oldValue       = 0;
    gckVIDMEM_HANDLE handleObject   = gcvNULL;
    gctBOOL          acquired       = gcvFALSE;

    gcmkHEADER_ARG("Handle=%d PrcoessID=%d", Handle, ProcessID);

    gcmkONERROR(gckKERNEL_FindHandleDatbase(Kernel, ProcessID,
                                            &handleDatabase, &mutex));

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Translate handle to gckVIDMEM_HANDLE. */
    gcmkONERROR(gckKERNEL_QueryIntegerId(handleDatabase, Handle,
                                         (gctPOINTER *)&handleObject));

    gckOS_AtomDecrement(Kernel->os, handleObject->reference, &oldValue);

    if (oldValue == 1) {
        /* Remove handle from database if this is the last reference. */
        gcmkVERIFY_OK(gckKERNEL_FreeIntegerId(handleDatabase, Handle));
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));
    acquired = gcvFALSE;

    if (oldValue == 1) {
        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os,
                                        handleObject->reference));
        gcmkOS_SAFE_FREE(Kernel->os, handleObject);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_HANDLE_Lookup(IN gckKERNEL Kernel, IN gctUINT32 ProcessID,
                        IN gctUINT32 Handle, OUT gckVIDMEM_NODE *Node)
{
    gceSTATUS        status;
    gckVIDMEM_HANDLE handleObject = gcvNULL;
    gckVIDMEM_NODE   node         = gcvNULL;
    gctPOINTER       database     = gcvNULL;
    gctPOINTER       mutex        = gcvNULL;
    gctBOOL          acquired     = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p ProcessID=%d Handle=%d",
                   Kernel, ProcessID, Handle);

    gcmkONERROR(gckKERNEL_FindHandleDatbase(Kernel, ProcessID,
                                            &database, &mutex));

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));
    acquired = gcvTRUE;

    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Handle,
                                         (gctPOINTER *)&handleObject));

    node = handleObject->node;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));
    acquired = gcvFALSE;

    *Node = node;

    gcmkFOOTER_ARG("*Node=%p", *Node);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_HANDLE_Lookup2(IN gckKERNEL Kernel, IN gcsDATABASE_PTR Database,
                         IN gctUINT32 Handle, OUT gckVIDMEM_NODE *Node)
{
    gceSTATUS        status;
    gckVIDMEM_HANDLE handleObject = gcvNULL;
    gckVIDMEM_NODE   node         = gcvNULL;
    gctPOINTER       database     = gcvNULL;
    gctPOINTER       mutex        = gcvNULL;
    gctBOOL          acquired     = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p Database=%p Handle=%d",
                   Kernel, Database, Handle);

    database = Database->handleDatabase;
    mutex    = Database->handleDatabaseMutex;

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));
    acquired = gcvTRUE;

    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Handle,
                                         (gctPOINTER *)&handleObject));

    node = handleObject->node;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));
    acquired = gcvFALSE;

    *Node = node;

    gcmkFOOTER_ARG("*Node=%p", *Node);
    return gcvSTATUS_OK;

OnError:
    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    gcmkFOOTER();
    return status;
}

static gceSTATUS
gckVIDMEM_NODE_Construct(IN gckKERNEL Kernel, IN gcuVIDMEM_NODE_PTR VideoNode,
                         IN gceVIDMEM_TYPE Type, IN gcePOOL Pool,
                         IN gctUINT32 Flag, OUT gckVIDMEM_NODE *NodeObject)
{
    gceSTATUS      status;
    gckVIDMEM_NODE node    = gcvNULL;
    gctPOINTER     pointer = gcvNULL;
    gckOS          os      = Kernel->os;
    gctUINT        i;

    /* Construct a node. */
    gcmkONERROR(gckOS_Allocate(os, gcmSIZEOF(gcsVIDMEM_NODE), &pointer));

    gcmkVERIFY_OK(gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsVIDMEM_NODE)));

    node = pointer;

    node->metadata.magic = VIV_VIDMEM_METADATA_MAGIC;
    node->metadata.ts_fd = -1;

    node->node   = VideoNode;
    node->kernel = Kernel;
    node->type   = Type;
    node->pool   = Pool;
    node->flag   = Flag;
    node->fd     = -1;
#if gcdENABLE_VIDEO_MEMORY_MIRROR
    node->mirror.mirrorNode = gcvNULL;
    node->mirror.type       = gcvMIRROR_TYPE_NONE;
#endif

    _UpdatePIDInfoListNode(os, node, VideoNode, 0, PIDINFO_LIST_NODE_Const);

    gcmkONERROR(gckOS_AtomConstruct(os, &node->reference));

    gcmkONERROR(gckOS_CreateMutex(os, &node->mutex));

    for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++) {
        gcmkONERROR(gckOS_CreateSignal(os, gcvFALSE,
                                       &node->sync[i].signal));
    }

    /* Reference is 1 by default . */
    gckOS_AtomSet(os, node->reference, 1);

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os,
                                     Kernel->db->videoMemListMutex,
                                     gcvINFINITE));

    /* Add into video memory node list. */
    gcsLIST_Add(&node->link, &Kernel->db->videoMemList);

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os,
                                     Kernel->db->videoMemListMutex));

    *NodeObject = node;

    return gcvSTATUS_OK;

OnError:
    if (node != gcvNULL) {
        if (node->mutex)
            gcmkVERIFY_OK(gckOS_DeleteMutex(os, node->mutex));

        if (node->reference != gcvNULL)
            gcmkVERIFY_OK(gckOS_AtomDestroy(os, node->reference));

        for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++) {
            if (node->sync[i].signal != gcvNULL) {
                gcmkVERIFY_OK(gckOS_DestroySignal(os,
                                                  node->sync[i].signal));
            }
        }

        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, node));
    }

    return status;
}

#if gcdENABLE_VIDEO_MEMORY_MIRROR
gceSTATUS
_AllocateVideoMemoryMirror(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                           IN gceMIRROR_TYPE MirrorType)
{
    gceSTATUS          status     = gcvSTATUS_OK;
    gckVIDMEM_NODE     mirrorNode = gcvNULL;
    gcuVIDMEM_NODE_PTR node       = gcvNULL;
    gcePOOL            mirrorPool = gcvPOOL_UNKNOWN;
    gctSIZE_T          bytes;
    gceVIDMEM_TYPE     type;
    gctUINT32          flag       = gcvALLOC_FLAG_CONTIGUOUS;

    gcmkONERROR(gckVIDMEM_NODE_GetSize(Kernel, NodeObject, &bytes));
    gcmkONERROR(gckVIDMEM_NODE_GetType(Kernel, NodeObject, &type, gcvNULL));

    if (MirrorType == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
        status = gckVIDMEM_AllocateVirtual(Kernel, flag, bytes, &node);
        if (gcmIS_ERROR(status)) {
            flag = gcvALLOC_FLAG_NON_CONTIGUOUS;
            gcmkONERROR(gckVIDMEM_AllocateVirtual(Kernel,
                                                  gcvALLOC_FLAG_NON_CONTIGUOUS,
                                                  bytes, &node));
        }

        mirrorPool = gcvPOOL_VIRTUAL;
    } else if (MirrorType == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
        gckVIDMEM videoMemory = gcvNULL;

        /* Get pointer to gckVIDMEM object for pool. */
        gcmkONERROR(gckKERNEL_GetVideoMemoryPool(Kernel,
                                                 gcvPOOL_LOCAL_EXCLUSIVE,
                                                 &videoMemory));

        gcmkONERROR(gckVIDMEM_AllocateLinear(Kernel, videoMemory, bytes, 4096,
                                             type, flag, gcvFALSE, &node));

        mirrorPool = gcvPOOL_LOCAL_EXCLUSIVE;
    } else {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    /* Construct a node. */
    gcmkONERROR(gckVIDMEM_NODE_Construct(Kernel, node, type,
                                         mirrorPool, flag, &mirrorNode));

    NodeObject->mirror.mirrorNode = mirrorNode;

    return gcvSTATUS_OK;
OnError:
    if (node)
        gcmkVERIFY_OK(gckVIDMEM_Free(Kernel, node));

    return status;
}

gceSTATUS
_FreeVideoMemoryMirror(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject)
{
    gceSTATUS status = gcvSTATUS_OK;

    if (NodeObject->mirror.mirrorNode) {
        status = gckVIDMEM_NODE_Dereference(Kernel, NodeObject->mirror.mirrorNode);
        NodeObject->mirror.mirrorNode = gcvNULL;
        NodeObject->mirror.type       = gcvMIRROR_TYPE_NONE;
    }

    return status;
}

#endif

gceSTATUS
gckVIDMEM_NODE_AllocateLinear(IN gckKERNEL Kernel, IN gckVIDMEM VideoMemory,
                              IN gcePOOL Pool, IN gceVIDMEM_TYPE Type,
                              IN gctUINT32 Flag, IN gctUINT32 Alignment,
                              IN gctBOOL Specified, IN OUT gctSIZE_T *Bytes,
                              OUT gckVIDMEM_NODE *NodeObject)
{
    gceSTATUS          status;
    gctSIZE_T          bytes      = *Bytes;
    gcuVIDMEM_NODE_PTR node       = gcvNULL;
    gckVIDMEM_NODE     nodeObject = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p VideoMemory=%p Pool=%d Alignment=%d Type=%d *Bytes=%zu",
                   Kernel, VideoMemory, Pool, Alignment, Type, bytes);

    /* Update the flags */
    switch (Pool) {
    case gcvPOOL_LOCAL_EXCLUSIVE:
        Flag &= ~gcvALLOC_FLAG_CPU_ACCESS;
        Flag |= gcvALLOC_FLAG_NON_CPU_ACCESS;
        break;
    default:
        Flag &= ~gcvALLOC_FLAG_NON_CPU_ACCESS;
        break;
    }

    gcmkONERROR_EX(gckVIDMEM_AllocateLinear(Kernel, VideoMemory, bytes, Alignment,
                                            Type, Flag, Specified, &node),
                   gcvSTATUS_OUT_OF_MEMORY);

    /* Update pool. */
    node->VidMem.pool = Pool;
    bytes             = node->VidMem.bytes;

    /* Construct a node. */
    gcmkONERROR(gckVIDMEM_NODE_Construct(Kernel, node, Type,
                                         Pool, Flag, &nodeObject));

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    /* Just update the mirror buffer type here */
    if (Flag & gcvALLOC_FLAG_WITH_MIRROR) {
        nodeObject->mirror.type = gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR;
#if gcdSTATIC_VIDEO_MEMORY_MIRROR
        gcmkONERROR(_AllocateVideoMemoryMirror(Kernel, nodeObject,
                                               gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR));
#    else
        nodeObject->mirror.refCount = 0;
#    endif
    }
#endif

    *Bytes      = bytes;
    *NodeObject = nodeObject;

    gcmkFOOTER_ARG("*Bytes=%u *NodeObject=%p", bytes, nodeObject);
    return gcvSTATUS_OK;

OnError:
    if (nodeObject)
        gckVIDMEM_NODE_Dereference(Kernel, nodeObject);

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_AllocateVirtual(IN gckKERNEL Kernel, IN gcePOOL Pool,
                               IN gceVIDMEM_TYPE Type, IN gctUINT32 Flag,
                               IN OUT gctSIZE_T *Bytes,
                               OUT gckVIDMEM_NODE *NodeObject)
{
    gceSTATUS          status;
    gctSIZE_T          bytes      = *Bytes;
    gcuVIDMEM_NODE_PTR node       = gcvNULL;
    gckVIDMEM_NODE     nodeObject = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p Pool=%d Type=%d Flag=%x *Bytes=%u",
                   Kernel, Pool, Type, Flag, bytes);

    if (Flag & gcvALLOC_FLAG_CONTIGUOUS) {
        gcmkONERROR_EX(gckVIDMEM_AllocateVirtual(Kernel, Flag, bytes, &node),
                       gcvSTATUS_OUT_OF_MEMORY);
    } else {
        gcmkONERROR(gckVIDMEM_AllocateVirtual(Kernel, Flag, bytes, &node));
    }

    /* Update type. */
    node->Virtual.type = Type;
    bytes              = node->Virtual.bytes;

    /* Construct a node. */
    gcmkONERROR(gckVIDMEM_NODE_Construct(Kernel, node, Type,
                                         Pool, Flag, &nodeObject));

    *Bytes      = bytes;
    *NodeObject = nodeObject;

    gcmkFOOTER_ARG("*Bytes=%u *NodeObject=%p", bytes, nodeObject);
    return gcvSTATUS_OK;

OnError:
    if (node)
        gcmkVERIFY_OK(gckVIDMEM_Free(Kernel, node));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_AllocateVirtualChunk(IN gckKERNEL Kernel, IN gcePOOL Pool,
                                    IN gceVIDMEM_TYPE Type, IN gctUINT32 Flag,
                                    IN OUT gctSIZE_T *Bytes,
                                    OUT gckVIDMEM_NODE *NodeObject)
{
    gceSTATUS          status;
    gctSIZE_T          bytes      = *Bytes;
    gcuVIDMEM_NODE_PTR node       = gcvNULL;
    gckVIDMEM_NODE     nodeObject = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p Pool=%d Type=%d Flag=%x *Bytes=%u",
                   Kernel, Pool, Type, Flag, bytes);

    gcmkONERROR_EX(gckVIDMEM_AllocateVirtualChunk(Kernel, Type, Flag, bytes, &node),
                   gcvSTATUS_OUT_OF_MEMORY);

    bytes = node->VirtualChunk.bytes;

    /* Construct a node. */
    gcmkONERROR(gckVIDMEM_NODE_Construct(Kernel, node, Type,
                                         Pool, Flag, &nodeObject));

    *Bytes      = bytes;
    *NodeObject = nodeObject;

    gcmkFOOTER_ARG("*Bytes=%u *NodeObject=%p", bytes, nodeObject);
    return gcvSTATUS_OK;

OnError:
    if (node)
        gcmkVERIFY_OK(gckVIDMEM_Free(Kernel, node));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_Reference(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject)
{
    gctINT32 oldValue;

    gcmkHEADER_ARG("Kernel=%p NodeObject=%p", Kernel, NodeObject);

    gcmkVERIFY_ARGUMENT(NodeObject != gcvNULL);

    gckOS_AtomIncrement(Kernel->os, NodeObject->reference, &oldValue);

    _UpdatePIDInfoListNode(Kernel->os, NodeObject, NodeObject->node,
                           0, PIDINFO_LIST_NODE_Ref);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_DereferenceEx(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                             IN gctUINT32 ProcessID)
{
    gctINT32   oldValue = 0;
    gctPOINTER database = Kernel->db->nameDatabase;
    gctPOINTER mutex    = Kernel->db->nameDatabaseMutex;
    gctUINT    i;

    gcmkHEADER_ARG("Kernel=%p NodeObject=%p", Kernel, NodeObject);
    gcmkVERIFY_ARGUMENT(NodeObject != gcvNULL);

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));

    gcmkVERIFY_OK(gckOS_AtomDecrement(Kernel->os,
                                      NodeObject->reference,
                                      &oldValue));

    if (oldValue == 1 && NodeObject->name) {
        /* Free name if exists. */
        gcmkVERIFY_OK(gckKERNEL_FreeIntegerId(database, NodeObject->name));
    }

    _UpdatePIDInfoListNode(Kernel->os, NodeObject, NodeObject->node,
                           ProcessID, PIDINFO_LIST_NODE_Deref);

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    if (oldValue == 1) {
        gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os,
                                         Kernel->db->videoMemListMutex,
                                         gcvINFINITE));

        /* Remove from video memory node list. */
        gcsLIST_Del(&NodeObject->link);

        _UpdatePIDInfoListNode(Kernel->os, NodeObject, NodeObject->node,
                               ProcessID, PIDINFO_LIST_NODE_Clean);

        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os,
                                         Kernel->db->videoMemListMutex));

        /* Free gcuVIDMEM_NODE. */
        if (NodeObject->node)
            gcmkVERIFY_OK(gckVIDMEM_Free(Kernel, NodeObject->node));

#if gcdENABLE_VIDEO_MEMORY_MIRROR
#if gcdSTATIC_VIDEO_MEMORY_MIRROR
        gcmkVERIFY_OK(_FreeVideoMemoryMirror(Kernel, NodeObject));
#    else
        gcmkASSERT(NodeObject->mirror.mirrorNode == gcvNULL);
#    endif
#endif

        gcmkVERIFY_OK(gckOS_AtomDestroy(Kernel->os, NodeObject->reference));

        gcmkVERIFY_OK(gckOS_DeleteMutex(Kernel->os, NodeObject->mutex));

        for (i = 0; i < gcvENGINE_GPU_ENGINE_COUNT; i++) {
            if (NodeObject->sync[i].signal != gcvNULL) {
                gcmkVERIFY_OK(gckOS_DestroySignal(Kernel->os,
                                                  NodeObject->sync[i].signal));
            }
        }

        /* Should not cause recursive call since tsNode->tsNode should be NULL */
        if (NodeObject->tsNode) {
            gcmkASSERT(!NodeObject->tsNode->tsNode);
            gckVIDMEM_NODE_Dereference(Kernel, NodeObject->tsNode);
        }

        gcmkOS_SAFE_FREE(Kernel->os, NodeObject);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_Dereference(gckKERNEL Kernel, gckVIDMEM_NODE NodeObject)
{
    gcmkHEADER_ARG("Kernel=%p NodeObject=%p", Kernel, NodeObject);
    gcmkVERIFY_ARGUMENT(NodeObject != gcvNULL);

    gckVIDMEM_NODE_DereferenceEx(Kernel, NodeObject, 0);

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_GetReference(IN gckKERNEL Kernel,
                            IN gckVIDMEM_NODE NodeObject,
                            OUT gctINT32 *ReferenceCount)
{
    gctINT32 value;

    gckOS_AtomGet(Kernel->os, NodeObject->reference, &value);

    *ReferenceCount = value;
    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_Lock(IN gckKERNEL Kernel,
                    IN gckVIDMEM_NODE NodeObject,
                    OUT gctADDRESS *Address)
{
    gceSTATUS          status;
    gckOS              os       = Kernel->os;
    gctBOOL            acquired = gcvFALSE;
    gcuVIDMEM_NODE_PTR node;
    gckVIDMEM_BLOCK    vidMemBlock;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

    if (gcvNULL == NodeObject)
        gcmkONERROR(gcvSTATUS_INVALID_OBJECT);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    /* allocate virtual mem for dma */
    if (NodeObject->mirror.type == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
#if !gcdSTATIC_VIDEO_MEMORY_MIRROR
        if (NodeObject->mirror.refCount == 0) {
            gcmkONERROR(_AllocateVideoMemoryMirror(Kernel, NodeObject,
                                                   gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR));
        }

        NodeObject->mirror.refCount++;
#    endif
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif
    vidMemBlock = node->VirtualChunk.parent;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(os, NodeObject->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
        gcmkONERROR(gckVIDMEM_Lock(Kernel, node, Address));
    else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
        gcmkONERROR(gckVIDMEM_LockVirtualChunk(Kernel, node, Address));
    else
        gcmkONERROR(gckVIDMEM_LockVirtual(Kernel, node, Address));

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));

    gcmkFOOTER_ARG("*Address=0x%llx", *Address);
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_Unlock(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                      IN gctUINT32 ProcessID, IN OUT gctBOOL *Asynchroneous)
{
    gceSTATUS          status;
    gckOS              os          = Kernel->os;
    gctBOOL            acquired    = gcvFALSE;
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;

    gcmkHEADER_ARG("NodeObject=%p Asynchroneous=%p", NodeObject, Asynchroneous);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    /* allocate virtual mem for dma */
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif
    vidMemBlock = node->VirtualChunk.parent;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(os, NodeObject->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        gcmkONERROR(gckVIDMEM_Unlock(Kernel, node,
                                     Asynchroneous));
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        gcmkONERROR(gckVIDMEM_UnlockVirtualChunk(Kernel, node,
                                                 Asynchroneous));
    } else {
        gcmkONERROR(gckVIDMEM_UnlockVirtual(Kernel, node,
                                            Asynchroneous));
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));

#if gcdENABLE_VIDEO_MEMORY_MIRROR && !gcdSTATIC_VIDEO_MEMORY_MIRROR
    /* allocate virtual mem for dma */
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
        if (Asynchroneous == gcvNULL) {
            if (NodeObject->mirror.refCount <= 0)
                gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

            NodeObject->mirror.refCount--;
        }

        if (NodeObject->mirror.refCount == 0)
            gcmkONERROR(_FreeVideoMemoryMirror(Kernel, NodeObject));
    }
#endif

    gcmkFOOTER_ARG("*Asynchroneous=0x%08X", gcmOPT_VALUE(Asynchroneous));
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_CleanCache(IN gckKERNEL Kernel,
                          IN gckVIDMEM_NODE NodeObject, IN gctSIZE_T Offset,
                          IN gctPOINTER Logical, IN gctSIZE_T Bytes)
{
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gctPHYS_ADDR       physHandle  = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gceSTATUS          status;

    gcmkHEADER_ARG("Kernel=%p NodeObject=%p Offset=0x%zx Logical=%p Bytes=0x%zx",
                   Kernel, NodeObject, Offset, Logical, Bytes);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif
    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        gcmkONERROR(gckOS_MemoryBarrier(Kernel->os, Logical));
#if gcdENABLE_VIDEO_MEMORY_MIRROR
        goto OnSync;
#else
        /* Reserved pool can't be cacheable */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
#endif
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        physHandle = vidMemBlock->physical;
    } else {
        physHandle = node->Virtual.physical;
    }

    gcmkONERROR(gckOS_CacheClean(Kernel->os, 0, physHandle,
                                 Offset, Logical, Bytes));

#if gcdENABLE_VIDEO_MEMORY_MIRROR
OnSync:
    if (NodeObject->mirror.mirrorNode) {
        gcmkONERROR(gckKERNEL_SyncVideoMemoryMirror(Kernel, NodeObject, Offset, Bytes,
                                                    gcvSYNC_MEMORY_DIRECTION_SYSTEM_TO_LOCAL));
    }
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_InvalidateCache(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                               IN gctSIZE_T Offset, IN gctPOINTER Logical, IN gctSIZE_T Bytes)
{
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gctPHYS_ADDR       physHandle  = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gceSTATUS          status;

    gcmkHEADER_ARG("Kernel=%p NodeObject=%p Offset=0x%zx Logical=%p Bytes=0x%zx",
                   Kernel, NodeObject, Offset, Logical, Bytes);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif
    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
#if gcdENABLE_VIDEO_MEMORY_MIRROR
        goto OnSync;
#else
        /* Reserved pool can't be cacheable */
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
#endif
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        physHandle = vidMemBlock->physical;
    } else {
        physHandle = node->Virtual.physical;
    }

    gcmkONERROR(gckOS_CacheInvalidate(Kernel->os, 0, physHandle,
                                      Offset, Logical, Bytes));

#if gcdENABLE_VIDEO_MEMORY_MIRROR
OnSync:
    if (NodeObject->mirror.mirrorNode) {
        gcmkONERROR(gckKERNEL_SyncVideoMemoryMirror(Kernel, NodeObject, Offset, Bytes,
                                                    gcvSYNC_MEMORY_DIRECTION_LOCAL_TO_SYSTEM));
    }
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_GetLockCount(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                            OUT gctINT32 *LockCount)
{
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gctINT32           lockCount   = 0;
    gctINT             i           = 0;
    gctINT             count;

#if gcdSHARED_PAGETABLE
    count = gcvHARDWARE_NUM_TYPES;
#else
    count = gcvCORE_COUNT;
#endif

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif
    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        lockCount = node->VidMem.locked;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        for (; i < count; i++)
            lockCount += node->VirtualChunk.lockeds[i];
    } else {
        for (; i < count; i++)
            lockCount += node->Virtual.lockeds[i];
    }

    *LockCount = lockCount;

    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_LockCPU(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                       IN gctBOOL Cacheable, IN gctBOOL FromUser,
                       OUT gctPOINTER *Logical)
{
    gceSTATUS          status;
    gckOS              os       = Kernel->os;
    gctBOOL            acquired = gcvFALSE;
    gcuVIDMEM_NODE_PTR node;
    gckVIDMEM_BLOCK    vidMemBlock;
    gctPOINTER         logical = gcvNULL;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    /* allocate virtual mem for dma */
    if (NodeObject->mirror.type == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
#if !gcdSTATIC_VIDEO_MEMORY_MIRROR
        if (NodeObject->mirror.refCount == 0) {
            gcmkONERROR(_AllocateVideoMemoryMirror(Kernel, NodeObject,
                                                   gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR));
        }
        NodeObject->mirror.refCount++;
#    endif
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif

    vidMemBlock = node->VirtualChunk.parent;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(os, NodeObject->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
#if !USE_LINUX_PCIE
        /* Do not care it has cacheable flag if it is PCIE */
        if (Cacheable == gcvTRUE)
            gcmkONERROR(gcvSTATUS_INVALID_REQUEST);
#endif

        if (FromUser) {
#if gcdCAPTURE_ONLY_MODE
            node->VidMem.logical = NodeObject->captureLogical;
#else
            /* Map video memory pool to user space. */
            gcmkONERROR(gckKERNEL_MapVideoMemory(Kernel, gcvTRUE, node->VidMem.pool,
                                                 node->VidMem.physical,
                                                 node->VidMem.offset,
                                                 node->VidMem.bytes,
                                                 &node->VidMem.logical));
#endif

            logical = node->VidMem.logical;
        } else {
            /* Map video memory pool to kernel space. */
            if (!node->VidMem.kvaddr) {
#if gcdCAPTURE_ONLY_MODE
                gcmkONERROR(gckOS_Allocate(os, node->VidMem.bytes, &node->VidMem.kvaddr));
#else
                gcmkONERROR(gckOS_CreateKernelMapping(os, node->VidMem.parent->physical,
                                                      node->VidMem.offset,
                                                      node->VidMem.bytes,
                                                      &node->VidMem.kvaddr));
#endif
            }

            logical = node->VidMem.kvaddr;
        }
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        if (FromUser) {
            /* Lock the entire vidmem block. */
            gcmkONERROR(gckOS_LockPages(os, vidMemBlock->physical,
                                        vidMemBlock->bytes,
                                        Cacheable, &logical));

            /* Get the logical with offset in block. */
            logical                    = (uint8_t *)logical + node->VirtualChunk.offset;
            node->VirtualChunk.logical = logical;
        } else {
            /* Map once and will cancel map when free. */
            if (!node->VirtualChunk.kvaddr) {
                gcmkONERROR(gckOS_CreateKernelMapping(os, vidMemBlock->physical,
                                                      node->VirtualChunk.offset,
                                                      node->VirtualChunk.bytes,
                                                      &node->VirtualChunk.kvaddr));
            }

            logical = node->VirtualChunk.kvaddr;
        }
    } else {
        if (FromUser) {
            gcmkONERROR(gckOS_LockPages(os, node->Virtual.physical,
                                        node->Virtual.bytes,
                                        Cacheable, &logical));

            node->Virtual.logical = logical;
        } else {
            /* Map once and will cancel map when free. */
            if (!node->Virtual.kvaddr) {
                gcmkONERROR(gckOS_CreateKernelMapping(os, node->Virtual.physical, 0,
                                                      node->Virtual.bytes,
                                                      &node->Virtual.kvaddr));
            }

            logical = node->Virtual.kvaddr;
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));

    *Logical = logical;

    gcmkFOOTER_ARG("*Logical=%p", logical);
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_UnlockCPU(IN gckKERNEL Kernel,
                         IN gckVIDMEM_NODE NodeObject,
                         IN gctUINT32 ProcessID,
                         IN gctBOOL FromUser, IN gctBOOL Defer)
{
    gceSTATUS          status;
    gckOS              os       = Kernel->os;
    gctBOOL            acquired = gcvFALSE;
    gcuVIDMEM_NODE_PTR node;
    gckVIDMEM_BLOCK    vidMemBlock;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
        node = NodeObject->mirror.mirrorNode->node;
    } else {
        node = NodeObject->node;
    }
#else
    node = NodeObject->node;
#endif

    vidMemBlock = node->VirtualChunk.parent;

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(os, NodeObject->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        if (FromUser) {
#if gcdCAPTURE_ONLY_MODE || defined __QNXNTO__
            /* Do nothing here. */
#else
            if (!Defer) {
                /* Unmap the video memory. */
                if (node->VidMem.logical != gcvNULL) {
                    gckKERNEL_UnmapVideoMemory(Kernel, node->VidMem.pool,
                                               node->VidMem.physical,
                                               node->VidMem.logical,
                                               node->VidMem.processID,
                                               node->VidMem.bytes);

                    node->VidMem.logical = gcvNULL;
                }
            }
#endif
        } else {
            /*
             * Kernel side may lock for CPU access for multiple times. Since
             * we don't have lock counts currently, we don't cancel CPU
             * mapping here, and will cancel at 'free' instead.
             */
            /*
             *gcmkONERROR(
             *    gckOS_DestroyKernelMapping(os,
             *                               node->VidMem.parent->physical,
             *                               node->VidMem.kvaddr));
             *
             * node->VidMem.kvaddr = gcvNULL;
             */
        }
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        if (FromUser) {
            gcmkONERROR(gckOS_UnlockPages(os, vidMemBlock->physical,
                                          vidMemBlock->bytes,
                                          node->VirtualChunk.logical));
        } else {
            /* Nothing to do. */
        }
    } else {
        if (FromUser) {
            gcmkONERROR(gckOS_UnlockPages(os, node->Virtual.physical,
                                          node->Virtual.bytes,
                                          node->Virtual.logical));
        } else {
            /*
             * Kernel side may lock for CPU access for multiple times. Since
             * we don't have lock counts currently, we don't cancel CPU
             * mapping here, and will cancel at 'free' instead.
             */
            /*
             *gcmkONERROR(
             *    gckOS_DestroyKernelMapping(os,
             *                               node->Virtual.physical,
             *                               node->Virtual.kvaddr));
             *
             *node->Virtual.kvaddr = gcvNULL;
             */
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));

#if gcdENABLE_VIDEO_MEMORY_MIRROR && !gcdSTATIC_VIDEO_MEMORY_MIRROR
    /* allocate virtual mem for dma */
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
        if (NodeObject->mirror.refCount <= 0)
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);

        if (--NodeObject->mirror.refCount == 0)
            gcmkONERROR(_FreeVideoMemoryMirror(Kernel, NodeObject));
    }
#endif

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_GetCPUPhysical(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                              IN gctSIZE_T Offset, OUT gctPHYS_ADDR_T *PhysicalAddress)
{
    gceSTATUS          status;
    gckOS              os          = Kernel->os;
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

    node = NodeObject->node;
    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        if (Offset >= node->VidMem.bytes) {
            /* Exceeds node size. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        *PhysicalAddress = node->VidMem.parent->physicalBase +
                           node->VidMem.offset + Offset;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        if (Offset >= node->VirtualChunk.bytes) {
            /* Exceeds node size. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        gcmkONERROR(gckOS_GetPhysicalFromHandle(os, vidMemBlock->physical,
                                                node->VirtualChunk.offset + Offset,
                                                PhysicalAddress));
    } else {
        if (Offset >= node->Virtual.bytes) {
            /* Exceeds node size. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        gcmkONERROR(gckOS_GetPhysicalFromHandle(os, node->Virtual.physical,
                                                Offset, PhysicalAddress));
    }

    gcmkFOOTER_ARG("*PhysicalAddress=0x%llx", *PhysicalAddress);
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_GetGPUPhysical(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                              IN gctUINT32 Offset, OUT gctPHYS_ADDR_T *PhysicalAddress)
{
    gceSTATUS      status = gcvSTATUS_OK;
    gckOS          os     = Kernel->os;
    gctPHYS_ADDR_T physical;
    gckVIDMEM_NODE node   = NodeObject;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

#if gcdENABLE_VIDEO_MEMORY_MIRROR
    if (NodeObject->mirror.mirrorNode &&
        NodeObject->mirror.type == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
        node = NodeObject->mirror.mirrorNode;
    }
#endif

    /* Get CPU physical address. */
    gcmkONERROR(gckVIDMEM_NODE_GetCPUPhysical(Kernel, node, Offset, &physical));

    gcmkVERIFY_OK(gckOS_CPUPhysicalToGPUPhysical(os, physical, PhysicalAddress));

    gcmkFOOTER_ARG("*PhysicalAddress=0x%llx", *PhysicalAddress);
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_GetGid(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                      OUT gctUINT32 *Gid)
{
    gcuVIDMEM_NODE_PTR node        = NodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
        *Gid = 0;
    else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
        *Gid = vidMemBlock->gid;
    else
        *Gid = node->Virtual.gid;

    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_GetSize(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                       OUT gctSIZE_T *Size)
{
    gcuVIDMEM_NODE_PTR node        = NodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
        *Size = node->VidMem.bytes;
    else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
        *Size = node->VirtualChunk.bytes;
    else
        *Size = node->Virtual.bytes;

    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_GetType(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                       OUT gceVIDMEM_TYPE *Type, OUT gcePOOL *Pool)
{
    if (Type)
        *Type = NodeObject->type;

    if (Pool)
        *Pool = NodeObject->pool;

    return gcvSTATUS_OK;
}

#if defined(CONFIG_DMA_SHARED_BUFFER)

/*******************************************************************************
 **
 **
 ** Code for dma_buf ops
 **
 **
 ******************************************************************************/

#    include <linux/slab.h>
#    include <linux/mm_types.h>
#    include <linux/dma-buf.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0)
#include <linux/module.h>
MODULE_IMPORT_NS(DMA_BUF);
#endif

static struct sg_table *
_dmabuf_map(struct dma_buf_attachment *attachment, enum dma_data_direction direction)
{
    struct sg_table *sgt        = gcvNULL;
    struct dma_buf  *dmabuf     = attachment->dmabuf;
    gckVIDMEM_NODE   nodeObject = dmabuf->priv;
    gceSTATUS        status     = gcvSTATUS_OK;

    do {
        gcuVIDMEM_NODE_PTR node        = nodeObject->node;
        gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;
        gctPHYS_ADDR       physical    = gcvNULL;
        gctSIZE_T          offset      = 0;
        gctSIZE_T          bytes       = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
        DEFINE_DMA_ATTRS(attrs);
#    else
        unsigned long attrs = 0;
#    endif

        if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
            physical = node->VidMem.parent->physical;
            offset   = node->VidMem.offset;
            bytes    = node->VidMem.bytes;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
            dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
#    else
            attrs |= DMA_ATTR_SKIP_CPU_SYNC;
#    endif
        } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
            physical = vidMemBlock->physical;
            offset   = node->VirtualChunk.offset;
            bytes    = node->VirtualChunk.bytes;
        } else {
            physical = node->Virtual.physical;
            offset   = 0;
            bytes    = node->Virtual.bytes;
        }

        gcmkERR_BREAK(gckOS_MemoryGetSGT(nodeObject->kernel->os, physical,
                                         offset, bytes, (gctPOINTER *)&sgt));

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
        if (dma_map_sg_attrs(attachment->dev, sgt->sgl, sgt->nents, direction, &attrs) == 0)
#    else
        if (dma_map_sg_attrs(attachment->dev, sgt->sgl, sgt->nents, direction, attrs) == 0)
#    endif
        {
            sg_free_table(sgt);
            kfree(sgt);
            sgt = gcvNULL;
            gcmkERR_BREAK(gcvSTATUS_GENERIC_IO);
        }
    } while (gcvFALSE);

    return sgt;
}

static void
_dmabuf_unmap(struct dma_buf_attachment *attachment,
              struct sg_table *sgt, enum dma_data_direction direction)
{
    struct dma_buf    *dmabuf     = attachment->dmabuf;
    gckVIDMEM_NODE     nodeObject = dmabuf->priv;
    gcuVIDMEM_NODE_PTR node       = nodeObject->node;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
    DEFINE_DMA_ATTRS(attrs);
#    else
    unsigned long attrs = 0;
#    endif

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
        dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, &attrs);
#    else
        attrs |= DMA_ATTR_SKIP_CPU_SYNC;
#    endif
    }

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 8, 0)
    dma_unmap_sg_attrs(attachment->dev, sgt->sgl, sgt->nents, direction, &attrs);
#    else
    dma_unmap_sg_attrs(attachment->dev, sgt->sgl, sgt->nents, direction, attrs);
#    endif

    sg_free_table(sgt);
    kfree(sgt);
}

static int
_dmabuf_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
    gckVIDMEM_NODE     nodeObject  = dmabuf->priv;
    gcuVIDMEM_NODE_PTR node        = nodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;
    gctPHYS_ADDR       physical    = gcvNULL;
    gctSIZE_T          skipPages   = vma->vm_pgoff;
    gctSIZE_T          numPages    = PAGE_ALIGN(vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
    gceSTATUS          status      = gcvSTATUS_OK;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        physical   = node->VidMem.parent->physical;
        skipPages += (node->VidMem.offset >> PAGE_SHIFT);
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        physical   = vidMemBlock->physical;
        skipPages += (node->VirtualChunk.offset >> PAGE_SHIFT);
    } else {
        physical = node->Virtual.physical;
    }

    gcmkONERROR(gckOS_MemoryMmap(nodeObject->kernel->os,
                                 physical, skipPages, numPages, vma));

OnError:
    return gcmIS_ERROR(status) ? -EINVAL : 0;
}

static void
_dmabuf_release(struct dma_buf *dmabuf)
{
    gckVIDMEM_NODE nodeObject = dmabuf->priv;

    if (nodeObject->metadata.ts_dma_buf) {
        dma_buf_put(nodeObject->metadata.ts_dma_buf);
        nodeObject->metadata.ts_dma_buf = NULL;
    }

    gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(nodeObject->kernel, nodeObject));
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 5, 7)
static void *
_dmabuf_kmap(struct dma_buf *dmabuf, unsigned long offset)
{
    gckVIDMEM_NODE     nodeObject  = dmabuf->priv;
    gcuVIDMEM_NODE_PTR node        = nodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;
    gctINT8_PTR        kvaddr      = gcvNULL;
    gctPHYS_ADDR       physical    = gcvNULL;
    gctSIZE_T          bytes       = 0;

    offset = (offset << PAGE_SHIFT);
    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        physical = node->VidMem.parent->physical;
        offset  += node->VidMem.offset;
        bytes    = node->VidMem.bytes;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        physical = vidMemBlock->physical;
        offset  += node->VirtualChunk.offset;
        bytes    = node->VirtualChunk.bytes;
    } else {
        physical = node->Virtual.physical;
        bytes    = node->Virtual.bytes;
    }

    if (gcmIS_SUCCESS(gckOS_CreateKernelMapping(nodeObject->kernel->os,
                                                physical, 0, bytes,
                                                (gctPOINTER *)&kvaddr))) {
        kvaddr += offset;
    }

    return (gctPOINTER)kvaddr;
}

static void
_dmabuf_kunmap(struct dma_buf *dmabuf, unsigned long offset, void *ptr)
{
    gckVIDMEM_NODE     nodeObject  = dmabuf->priv;
    gcuVIDMEM_NODE_PTR node        = nodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;
    gctINT8_PTR        kvaddr      = (gctINT8_PTR)ptr - (offset << PAGE_SHIFT);
    gctPHYS_ADDR       physical    = gcvNULL;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        physical = node->VidMem.parent->physical;
        kvaddr  -= node->VidMem.offset;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        physical = vidMemBlock->physical;
        kvaddr  -= node->VirtualChunk.offset;
    } else {
        physical = node->Virtual.physical;
    }

    gcmkVERIFY_OK(gckOS_DestroyKernelMapping(nodeObject->kernel->os, physical,
                                             (gctPOINTER *)&kvaddr));
}
#    endif

static struct dma_buf_ops _dmabuf_ops = {
    .map_dma_buf   = _dmabuf_map,
    .unmap_dma_buf = _dmabuf_unmap,
    .mmap          = _dmabuf_mmap,
    .release       = _dmabuf_release,
#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 5, 7)
#    elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0)
    .map           = _dmabuf_kmap,
    .unmap = _dmabuf_kunmap,
#    elif LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
    .map_atomic    = _dmabuf_kmap,
    .unmap_atomic  = _dmabuf_kunmap,
    .map           = _dmabuf_kmap,
    .unmap         = _dmabuf_kunmap,
#    else
    .kmap_atomic   = _dmabuf_kmap,
    .kunmap_atomic = _dmabuf_kunmap,
    .kmap          = _dmabuf_kmap,
    .kunmap        = _dmabuf_kunmap,
#    endif
};
#endif

gceSTATUS
gckVIDMEM_NODE_Export(IN gckKERNEL Kernel,
                      IN gckVIDMEM_NODE NodeObject,
                      IN gctINT32 Flags,
                      OUT gctPOINTER *DmaBuf,
                      OUT gctINT32 *FD)
{
#if defined(CONFIG_DMA_SHARED_BUFFER)
    gceSTATUS       status = gcvSTATUS_OK;
    struct dma_buf *dmabuf = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p NodeObject=%p", Kernel, NodeObject);

    dmabuf = NodeObject->dmabuf;
    if (!dmabuf) {
        gctSIZE_T          bytes       = 0;
        gctPHYS_ADDR       physical    = gcvNULL;
        gcuVIDMEM_NODE_PTR node        = NodeObject->node;
        gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;

        if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
            physical = node->VidMem.parent->physical;
            bytes    = node->VidMem.bytes;
            /*
             * Align export size. when allocate memory from VIDMEM,
             * the actual node size may not same with aligned size.
             */
            bytes = bytes & ~(PAGE_SIZE - 1);
        } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
            physical = vidMemBlock->physical;
            bytes    = node->VirtualChunk.bytes;
        } else {
            physical = node->Virtual.physical;
            bytes    = node->Virtual.bytes;
        }

        /* Donot really get SGT, just check if the allocator support GetSGT. */
        gcmkONERROR(gckOS_MemoryGetSGT(Kernel->os, physical, 0, 0, NULL));

        {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
            DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

            exp_info.ops   = &_dmabuf_ops;
            exp_info.size  = bytes;
            exp_info.flags = Flags;
            exp_info.priv  = NodeObject;
            dmabuf         = dma_buf_export(&exp_info);
#    elif LINUX_VERSION_CODE >= KERNEL_VERSION(3, 17, 0)
            dmabuf = dma_buf_export(NodeObject, &_dmabuf_ops, bytes, Flags, NULL);
#    else
            dmabuf = dma_buf_export(NodeObject, &_dmabuf_ops, bytes, Flags);
#    endif
        }

        if (IS_ERR(dmabuf))
            gcmkONERROR(gcvSTATUS_GENERIC_IO);

        /* Reference this gckVIDMEM_NODE object. */
        gckVIDMEM_NODE_Reference(Kernel, NodeObject);
        NodeObject->dmabuf = dmabuf;
    }

    if (DmaBuf)
        *DmaBuf = NodeObject->dmabuf;

    if (FD) {
        gctINT fd = dma_buf_fd(dmabuf, Flags);

        if (fd < 0)
            gcmkONERROR(gcvSTATUS_GENERIC_IO);

        NodeObject->fd = fd;

        *FD = fd;
    }

OnError:
    gcmkFOOTER_ARG("*DmaBuf=%p *FD=0x%x", gcmOPT_POINTER(DmaBuf), gcmOPT_VALUE(FD));
    return status;
#else
    gcmkFATAL("The kernel did NOT support CONFIG_DMA_SHARED_BUFFER");
    return gcvSTATUS_NOT_SUPPORTED;
#endif
}

gceSTATUS
gckVIDMEM_NODE_Name(IN gckKERNEL Kernel,
                    IN gckVIDMEM_NODE NodeObject,
                    OUT gctUINT32 *Name)
{
    gceSTATUS  status;
    gctUINT32  name       = 0;
    gctPOINTER database   = Kernel->db->nameDatabase;
    gctPOINTER mutex      = Kernel->db->nameDatabaseMutex;
    gctBOOL    acquired   = gcvFALSE;
    gctBOOL    referenced = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p NodeObject=%p", Kernel, NodeObject);

    gcmkVERIFY_ARGUMENT(Name != gcvNULL);

    gcmkONERROR(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));
    acquired = gcvTRUE;

    gcmkONERROR(gckVIDMEM_NODE_Reference(Kernel, NodeObject));
    referenced = gcvTRUE;

    if (NodeObject->name == 0) {
        /* Name this node. */
        gcmkONERROR(gckKERNEL_AllocateIntegerId(database, NodeObject, &name));
        NodeObject->name = name;
    } else {
        name = NodeObject->name;
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));
    acquired = gcvFALSE;

    gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, NodeObject));

    *Name = name;

    gcmkFOOTER_ARG("*Name=%d", *Name);
    return gcvSTATUS_OK;

OnError:
    if (referenced)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, NodeObject));

    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_Import(IN gckKERNEL Kernel, IN gctUINT32 Name, OUT gckVIDMEM_NODE *NodeObject)
{
    gceSTATUS      status;
    gckVIDMEM_NODE node       = gcvNULL;
    gctPOINTER     database   = Kernel->db->nameDatabase;
    gctPOINTER     mutex      = Kernel->db->nameDatabaseMutex;
    gctBOOL        acquired   = gcvFALSE;
    gctBOOL        referenced = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p Name=%d", Kernel, Name);

    gcmkONERROR(gckOS_AcquireMutex(Kernel->os, mutex, gcvINFINITE));
    acquired = gcvTRUE;

    /* Lookup in database to get the node. */
    gcmkONERROR(gckKERNEL_QueryIntegerId(database, Name, (gctPOINTER *)&node));

    /* Reference the node. */
    gcmkONERROR(gckVIDMEM_NODE_Reference(Kernel, node));
    referenced = gcvTRUE;

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));
    acquired = gcvFALSE;

    *NodeObject = node;
    gcmkFOOTER_ARG("*NodeObject=%p", node);
    return gcvSTATUS_OK;

OnError:
    if (referenced)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, node));

    if (acquired)
        gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, mutex));

    gcmkFOOTER();
    return status;
}

typedef struct _gcsVIDMEM_NODE_FDPRIVATE {
    gcsFDPRIVATE   base;
    gckKERNEL      kernel;
    gckVIDMEM_NODE node;
} gcsVIDMEM_NODE_FDPRIVATE;

static gctINT
_ReleaseFdPrivate(gcsFDPRIVATE_PTR FdPrivate)
{
    /* Cast private info. */
    gcsVIDMEM_NODE_FDPRIVATE *private = (gcsVIDMEM_NODE_FDPRIVATE *)FdPrivate;

    gckVIDMEM_NODE_Dereference(private->kernel, private->node);
    gckOS_Free(private->kernel->os, private);

    return 0;
}

/*******************************************************************************
 **
 **  gckVIDMEM_NODE_GetFd
 **
 **  Attach a gckVIDMEM_NODE object to a native fd.
 **
 **  OUTPUT:
 **
 **      gctUINT32 *Fd
 **          Pointer to a variable receiving a native fd from os.
 */
gceSTATUS
gckVIDMEM_NODE_GetFd(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                     IN gctBOOL Exported, OUT gctINT *Fd)
{
    gceSTATUS                 status;
    gctBOOL                   referenced = gcvFALSE;
    gcsVIDMEM_NODE_FDPRIVATE *fdPrivate  = gcvNULL;

    gcmkHEADER_ARG("Kernel = 0x%X NodeObject = %d", Kernel, NodeObject);

    if (Exported) {
        *Fd = NodeObject->fd;

        gcmkFOOTER_ARG("*Fd = %d", *Fd);
        return gcvSTATUS_OK;
    }

    /* Reference node object. */
    gcmkVERIFY_OK(gckVIDMEM_NODE_Reference(Kernel, NodeObject));
    referenced = gcvTRUE;

    /* Allocated fd owns a reference. */
    gcmkONERROR(gckOS_Allocate(Kernel->os,
                               gcmSIZEOF(gcsVIDMEM_NODE_FDPRIVATE),
                               (gctPOINTER *)&fdPrivate));

    fdPrivate->base.release = _ReleaseFdPrivate;
    fdPrivate->kernel       = Kernel;
    fdPrivate->node         = NodeObject;

    /* Allocated fd owns a reference. */
    gcmkONERROR(gckOS_GetFd("vidmem", &fdPrivate->base, Fd));

    gcmkFOOTER_ARG("*Fd = %d", *Fd);
    return gcvSTATUS_OK;

OnError:
    if (referenced)
        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel, NodeObject));

    if (fdPrivate)
        gcmkVERIFY_OK(gcmkOS_SAFE_FREE(Kernel->os, fdPrivate));

    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_WrapUserMemory(IN gckKERNEL Kernel,
                              IN gcsUSER_MEMORY_DESC_PTR Desc,
                              IN gceVIDMEM_TYPE Type,
                              OUT gckVIDMEM_NODE *NodeObject,
                              OUT gctUINT64 *Bytes)
{
    gceSTATUS      status     = gcvSTATUS_OK;
    gckVIDMEM_NODE nodeObject = gcvNULL;
    gctBOOL        found      = gcvFALSE;

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    gcmkVERIFY_ARGUMENT(Desc != gcvNULL);

#if defined(CONFIG_DMA_SHARED_BUFFER)
    if (Desc->flag & gcvALLOC_FLAG_DMABUF) {
        struct dma_buf *dmabuf;
        int fd = (int)Desc->handle;

        if (fd >= 0) {
            /* Import dma buf handle. */
            dmabuf = dma_buf_get(fd);

            if (IS_ERR(dmabuf))
                return PTR_ERR(dmabuf);

            Desc->handle = -1;
            Desc->dmabuf = gcmPTR_TO_UINT64(dmabuf);

            dma_buf_put(dmabuf);
        } else if (fd == -1) {
            /* It is called by our kernel drm driver. */

            if (IS_ERR(gcmUINT64_TO_PTR(Desc->dmabuf))) {
                gcmkPRINT("Wrap memory: invalid dmabuf from kernel.\n");

                gcmkFOOTER();
                return gcvSTATUS_INVALID_ARGUMENT;
            }

            dmabuf = gcmUINT64_TO_PTR(Desc->dmabuf);
        } else {
            gcmkPRINT("Wrap memory: invalid dmabuf fd.\n");

            gcmkFOOTER();
            return gcvSTATUS_INVALID_ARGUMENT;
        }

        if (dmabuf->ops == &_dmabuf_ops) {
            gctBOOL referenced = gcvFALSE;

            nodeObject = dmabuf->priv;
            do {
                /* Reference the node. */
                gcmkERR_BREAK(gckVIDMEM_NODE_Reference(Kernel, nodeObject));
                referenced  = gcvTRUE;
                found       = gcvTRUE;

                *NodeObject = nodeObject;
                *Bytes      = (gctUINT64)dmabuf->size;
            } while (gcvFALSE);

            if (gcmIS_ERROR(status) && referenced) {
                gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(Kernel,
                                                         nodeObject));
            }
        }
    }
#endif

    if (!found) {
        gckOS              os   = Kernel->os;
        gcuVIDMEM_NODE_PTR node = gcvNULL;

        do {
            gctSIZE_T      pageCountCpu    = 0;
            gctSIZE_T      pageSizeCpu     = 0;
            gctPHYS_ADDR_T physicalAddress = 0;

            gcmkVERIFY_OK(gckOS_GetPageSize(os, &pageSizeCpu));

            /* Allocate an gcuVIDMEM_NODE union. */
            gcmkERR_BREAK(gckOS_Allocate(os, gcmSIZEOF(gcuVIDMEM_NODE), (gctPOINTER *)&node));
            gckOS_ZeroMemory(node, gcmSIZEOF(gcuVIDMEM_NODE));

            /* Initialize gcuVIDMEM_NODE union for virtual memory. */
            node->Virtual.kernel = Kernel;

            node->Virtual.lowVA = (Desc->flag & gcvALLOC_FLAG_32BIT_VA);

            /* Wrap Memory. */
            gcmkERR_BREAK(gckOS_WrapMemory(os, Kernel, Desc,
                                           &node->Virtual.bytes,
                                           &node->Virtual.physical,
                                           &node->Virtual.contiguous,
                                           &pageCountCpu));

            /* Get base physical address. */
            gcmkERR_BREAK(gckOS_GetPhysicalFromHandle(os, node->Virtual.physical,
                                                      0, &physicalAddress));

            /* Allocate handle for this video memory. */
            gcmkERR_BREAK(gckVIDMEM_NODE_Construct(Kernel, node, Type,
                                                   gcvPOOL_VIRTUAL,
                                                   Desc->flag, &nodeObject));

            node->Virtual.pageCount = (pageCountCpu * pageSizeCpu -
                    (physicalAddress & (pageSizeCpu - 1) & ~(4096 - 1))) >> 12;

#if defined(gcdWRAP_USER_MEMORY_MIRROR) && gcdENABLE_VIDEO_MEMORY_MIRROR
            /* Just update mirror buffer type here */
            nodeObject->mirror.type = gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR;
#if gcdSTATIC_VIDEO_MEMORY_MIRROR
            gcmkERR_BREAK(_AllocateVideoMemoryMirror(Kernel, nodeObject,
                                                     gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR));
#    else
            nodeObject->mirror.refCount = 0;
#    endif
#endif

            *NodeObject = nodeObject;
            *Bytes      = (gctUINT64)node->Virtual.bytes;
        } while (gcvFALSE);

        if (gcmIS_ERROR(status) && node) {
            /* Free the structure. */
            gcmkVERIFY_OK(gcmkOS_SAFE_FREE(os, node));
        }
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckVIDMEM_NODE_SetCommitStamp(IN gckKERNEL Kernel, IN gceENGINE Engine,
                              IN gckVIDMEM_NODE NodeObject, IN gctUINT64 CommitStamp)
{
    NodeObject->sync[Engine].commitStamp = CommitStamp;
    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_GetCommitStamp(IN gckKERNEL Kernel, IN gceENGINE Engine,
                              IN gckVIDMEM_NODE NodeObject,
                              OUT gctUINT64_PTR CommitStamp)
{
    *CommitStamp = NodeObject->sync[Engine].commitStamp;
    return gcvSTATUS_OK;
}

/*******************************************************************************
 **
 **  gckVIDMEM_NODE_Find
 **
 **  Find gckVIDMEM_NODE object according to GPU address of specified core.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Kernel object, specifies core.
 **
 **      gctADDRESS Address
 **          GPU address to search.
 **
 **  OUTPUT:
 **
 **      gckVIDMEM_NODE *NodeObject
 **          Pointer to a variable hold found video memory node.
 **
 **      gctSIZE_T *Offset
 **          The offset of specified GPU address in found video memory node.
 */
gceSTATUS
gckVIDMEM_NODE_Find(IN gckKERNEL Kernel, IN gctADDRESS Address,
                    OUT gckVIDMEM_NODE *NodeObject,
                    OUT gctSIZE_T *Offset)
{
    gceSTATUS          status      = gcvSTATUS_NOT_FOUND;
    gckVIDMEM_NODE     nodeObject  = gcvNULL;
    gcuVIDMEM_NODE_PTR node        = gcvNULL;
    gckVIDMEM_BLOCK    vidMemBlock = gcvNULL;
    gcsLISTHEAD_PTR    pos;
    gctUINT32          index;
#if gcdSHARED_PAGETABLE
    gceHARDWARE_TYPE hwType;

    gcmkVERIFY_OK(gckKERNEL_GetHardwareType(Kernel, &hwType));
    index = (gctUINT32)hwType;
#else
    index = (gctUINT32)Kernel->core;
#endif

    gcmkVERIFY_OK(gckOS_AcquireMutex(Kernel->os,
                                     Kernel->db->videoMemListMutex,
                                     gcvINFINITE));

    gcmkLIST_FOR_EACH(pos, &Kernel->db->videoMemList)
    {
        nodeObject  = (gckVIDMEM_NODE)gcmCONTAINEROF(pos, struct _gcsVIDMEM_NODE, link);
        node        = nodeObject->node;
        vidMemBlock = node->VirtualChunk.parent;

        if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
            if (!node->VidMem.locked) {
                /* Don't check against unlocked node. */
                continue;
            }

            if (Address >= node->VidMem.address &&
                Address <= node->VidMem.address + node->VidMem.bytes - 1) {
                *NodeObject = nodeObject;

                if (Offset)
                    *Offset = (gctSIZE_T)(Address - node->VidMem.address);

                status = gcvSTATUS_OK;
                break;
            }
        } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
            if (!node->VirtualChunk.lockeds[index]) {
                /* Don't check against unlocked node. */
                continue;
            }

            if (Address >= node->VirtualChunk.addresses[index] &&
                (Address <= node->VirtualChunk.addresses[index] +
                            node->VirtualChunk.bytes - 1)) {
                *NodeObject = nodeObject;

                if (Offset)
                    *Offset = (gctSIZE_T)(Address - node->VirtualChunk.addresses[index]);

                status = gcvSTATUS_OK;
                break;
            }
        } else {
            if (!node->Virtual.lockeds[index]) {
                /* Don't check against unlocked node. */
                continue;
            }

            if (Address >= node->Virtual.addresses[index] &&
                (Address <= node->Virtual.addresses[index] +
                            node->Virtual.bytes - 1)) {
                *NodeObject = nodeObject;

                if (Offset)
                    *Offset = (gctSIZE_T)(Address - node->Virtual.addresses[index]);

                status = gcvSTATUS_OK;
                break;
            }
        }
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(Kernel->os, Kernel->db->videoMemListMutex));

    return status;
}

gceSTATUS
gckVIDMEM_NODE_IsContiguous(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                            OUT gctBOOL *Contiguous)
{
    gceSTATUS          status;
    gckOS              os       = Kernel->os;
    gctBOOL            acquired = gcvFALSE;
    gcuVIDMEM_NODE_PTR node;
    gckVIDMEM_BLOCK    vidMemBlock;

    gcmkHEADER();

    /* Grab the mutex. */
    gcmkONERROR(gckOS_AcquireMutex(os, NodeObject->mutex, gcvINFINITE));
    acquired = gcvTRUE;

    node        = NodeObject->node;
    vidMemBlock = node->VirtualChunk.parent;

    *Contiguous = gcvFALSE;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        *Contiguous = gcvTRUE;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        if (vidMemBlock->contiguous)
            *Contiguous = gcvTRUE;
    } else {
        if (node->Virtual.contiguous)
            *Contiguous = gcvTRUE;
    }

    gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));

    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    if (acquired) {
        /* Release the mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(os, NodeObject->mutex));
    }

    /* Return the status. */
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
 **
 **  gckVIDMEM_NODE_GetMemoryHandle
 **
 **  Get the @NodeObject memory descripter handle.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Kernel object, specifies core.
 **
 **      gckVIDMEM_NODE NodeObject
 **          Pointer to gckVIDMEM_NODE object.
 **
 **      gctPOINTER *MemoryHandle
 **          Pointer to Pointer.
 **  OUTPUT:
 **
 **      gctPOINTER *MemoryHandle
 **          Store the mdl handle.
 */
gceSTATUS
gckVIDMEM_NODE_GetMemoryHandle(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                               OUT gctPOINTER *MemoryHandle)
{
    gcuVIDMEM_NODE_PTR node         = NodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock  = node->VirtualChunk.parent;
    gctUINT64          mappingInOne = 1;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne);

        if (mappingInOne)
            *MemoryHandle = node->VidMem.parent->physical;
        else
            *MemoryHandle = node->VidMem.physical;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        *MemoryHandle = vidMemBlock->physical;
    } else {
        *MemoryHandle = node->Virtual.physical;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

gceSTATUS
gckVIDMEM_NODE_GetMapKernel(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                            OUT gctPOINTER *KernelMap)
{
    gcuVIDMEM_NODE_PTR node        = NodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;

    if (node->VidMem.parent->object.type == gcvOBJ_VIDMEM)
        *KernelMap = node->VidMem.kvaddr;
    else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK)
        *KernelMap = node->VirtualChunk.kvaddr;
    else
        *KernelMap = node->Virtual.kvaddr;

    return gcvSTATUS_OK;
}

/*******************************************************************************
 **
 **  gckVIDMEM_NODE_GetOffset
 **
 **  Get the @NodeObject physical offset based on its memory handle(MDL).
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Kernel object, specifies core.
 **
 **      gckVIDMEM_NODE NodeObject
 **          Pointer to gckVIDMEM_NODE object.
 **
 **      gctSIZE_T *Offset
 **          Pointer to offset.
 **  OUTPUT:
 */
gceSTATUS
gckVIDMEM_NODE_GetOffset(IN gckKERNEL Kernel, IN gckVIDMEM_NODE NodeObject,
                         OUT gctSIZE_T *Offset)
{
    gcuVIDMEM_NODE_PTR node        = NodeObject->node;
    gckVIDMEM_BLOCK    vidMemBlock = node->VirtualChunk.parent;

    gcmkHEADER_ARG("NodeObject=%p", NodeObject);

    if (node->VidMem.parent && node->VidMem.parent->object.type == gcvOBJ_VIDMEM) {
        gctUINT64 mappingInOne = 1;

        gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne);

        if (mappingInOne)
            *Offset = node->VidMem.offset;
        else
            *Offset = 0;
    } else if (vidMemBlock && vidMemBlock->object.type == gcvOBJ_VIDMEM_BLOCK) {
        *Offset = node->VirtualChunk.offset;
    } else {
        *Offset = 0;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}
