/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2023 Vivante Corporation
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
*    Copyright (C) 2014 - 2023 Vivante Corporation
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

#define _GC_OBJ_ZONE gcvZONE_KERNEL

/******************************************************************************
 ******************************* gckKERNEL API Code ***************************
 *****************************************************************************/

/******************************************************************************
 **
 **  gckKERNEL_QueryVideoMemory
 **
 **  Query the amount of video memory.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **  OUTPUT:
 **
 **      gcsHAL_INTERFACE *Interface
 **          Pointer to an gcsHAL_INTERFACE structure that will be filled in
 **          with the memory information.
 */
gceSTATUS
gckKERNEL_QueryVideoMemory(IN gckKERNEL Kernel, OUT gcsHAL_INTERFACE *Interface)
{
    gckGALDEVICE galDevice = gcvNULL;
    gckDEVICE    device    = gcvNULL;

    gcmkHEADER_ARG("Kernel=%p", Kernel);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Interface);

    /* Extract the pointer to the gckGALDEVICE class. */
    galDevice = (gckGALDEVICE)Kernel->context;
    device    = Kernel->device;

    /* Get internal memory size and physical address. */
    Interface->u.QueryVideoMemory.internalSize     = galDevice->internalSize;
    Interface->u.QueryVideoMemory.internalPhysName = galDevice->internalPhysName;

    /* Get external memory size and physical address. */
    Interface->u.QueryVideoMemory.externalSize     = device->externalSize;
    Interface->u.QueryVideoMemory.externalPhysName = device->externalPhysName;

    /* Get contiguous memory size and physical address. */
    Interface->u.QueryVideoMemory.contiguousSize     = device->contiguousSizes[device->memIndex];
    Interface->u.QueryVideoMemory.contiguousPhysName = device->contiguousPhysNames[device->memIndex];

    /* Get exclusive memory size and physical address. */
    Interface->u.QueryVideoMemory.exclusiveSize     = device->exclusiveSize;
    Interface->u.QueryVideoMemory.exclusivePhysName = device->exclusivePhysName;

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/******************************************************************************
 **
 **  gckKERNEL_GetVideoMemoryPool
 **
 **  Get the gckVIDMEM object belonging to the specified pool.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gcePOOL Pool
 **          Pool to query gckVIDMEM object for.
 **
 **  OUTPUT:
 **
 **      gckVIDMEM *VideoMemory
 **          Pointer to a variable that will hold the pointer to the gckVIDMEM
 **          object belonging to the requested pool.
 */
gceSTATUS
gckKERNEL_GetVideoMemoryPool(IN gckKERNEL  Kernel,
                             IN gcePOOL    Pool,
                             OUT gckVIDMEM *VideoMemory)
{
    gckGALDEVICE galDevice = gcvNULL;
    gckDEVICE    device    = gcvNULL;
    gckVIDMEM    videoMemory;

    gcmkHEADER_ARG("Kernel=%p Pool=%d", Kernel, Pool);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(VideoMemory);

    /* Extract the pointer to the gckGALDEVICE class. */
    galDevice = (gckGALDEVICE)Kernel->context;
    device    = Kernel->device;

    /* Dispatch on pool. */
    switch (Pool) {
    case gcvPOOL_LOCAL_INTERNAL:
        /* Internal memory. */
        videoMemory = galDevice->internalVidMem;
        break;

    case gcvPOOL_LOCAL_EXTERNAL:
        /* External memory. */
        videoMemory = device->externalVidMem;
        break;

    case gcvPOOL_SYSTEM:
        /* System memory. */
        videoMemory = device->contiguousVidMems[device->memIndex];
        break;

    case gcvPOOL_LOCAL_EXCLUSIVE:
        /* gpu exclusive memory. */
        videoMemory = device->exclusiveVidMem;
        break;

    case gcvPOOL_INTERNAL_SRAM:
        /* Internal SRAM memory. */
        videoMemory = Kernel->sRAMVidMem[Kernel->sRAMIndex];
        break;

    case gcvPOOL_EXTERNAL_SRAM:
        /* External SRAM memory. */
        videoMemory = device->extSRAMVidMems[device->extSRAMIndex];
        break;


#if gcdENABLE_40BIT_VA
    case gcvPOOL_SYSTEM_32BIT_VA:
        /* System memory. */
        videoMemory = galDevice->lowContiguousVidMem;
        break;
#endif

    default:
        /* Unknown pool. */
        videoMemory = NULL;
    }

    /* Return pointer to the gckVIDMEM object. */
    *VideoMemory = videoMemory;

    /* Return status. */
    gcmkFOOTER_ARG("*VideoMemory=%p", *VideoMemory);
    return (!videoMemory) ? gcvSTATUS_OUT_OF_MEMORY : gcvSTATUS_OK;
}

/******************************************************************************
 **
 **  gckKERNEL_MapMemory
 **
 **  Map video memory into the current process space.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gctPHYS_ADDR Physical
 **          Physical address of video memory to map.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes to map.
 **
 **  OUTPUT:
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will hold the base address of the
 **          mapped memory region.
 */
gceSTATUS
gckKERNEL_MapMemory(IN gckKERNEL    Kernel,
                    IN gctPHYS_ADDR Physical,
                    IN gctSIZE_T    Bytes,
                    OUT gctPOINTER  *Logical)
{
    gckKERNEL    kernel   = Kernel;
    gctPHYS_ADDR physical = gcmNAME_TO_PTR(Physical);

    return gckOS_MapMemory(Kernel->os, physical, Bytes, Logical);
}

/******************************************************************************
 **
 **  gckKERNEL_UnmapMemory
 **
 **  Unmap video memory from the current process space.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gctPHYS_ADDR Physical
 **          Physical address of video memory to map.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes to map.
 **
 **      gctPOINTER Logical
 **          Base address of the mapped memory region.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gckKERNEL_UnmapMemory(IN gckKERNEL    Kernel,
                      IN gctPHYS_ADDR Physical,
                      IN gctSIZE_T    Bytes,
                      IN gctPOINTER   Logical,
                      IN gctUINT32    ProcessID)
{
    gckKERNEL    kernel   = Kernel;
    gctPHYS_ADDR physical = gcmNAME_TO_PTR(Physical);

    return gckOS_UnmapMemoryEx(Kernel->os, physical, Bytes, Logical, ProcessID);
}

/******************************************************************************
 **
 **  gckKERNEL_DestroyProcessReservedUserMap
 **
 **  Destroy process reserved memory
 **
 **  INPUT:
 **
 **      gctPHYS_ADDR Physical
 **          Physical address of video memory to map.
 **
 **      gctUINT32 Pid
 **          Process ID.
 */
gceSTATUS
gckKERNEL_DestroyProcessReservedUserMap(IN gckKERNEL Kernel, IN gctUINT32 Pid)
{
    gceSTATUS    status    = gcvSTATUS_OK;
    gckGALDEVICE galDevice = gcvNULL;
    gckDEVICE    device    = gcvNULL;
    gctSIZE_T    bytes     = 0;
    gctPHYS_ADDR physHandle = gcvNULL;
    /* when unmap reserved memory, we don't need real logical*/
    gctPOINTER   Logical    = (gctPOINTER)0xFFFFFFFF;
    gctINT       i;

    gcmkHEADER_ARG("Logical=%p pid=%u", Logical, Pid);
    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Extract the pointer to the gckGALDEVICE class. */
    galDevice = (gckGALDEVICE)Kernel->context;
    device = Kernel->device;

    physHandle = (PLINUX_MDL)galDevice->internalPhysical;
    bytes = galDevice->internalSize;

    if (bytes) {
        gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
    }

    physHandle = (PLINUX_MDL)device->externalPhysical;
    bytes = device->externalSize;

    if (bytes) {
        gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
    }

#if !gcdCAPTURE_ONLY_MODE
    /* System memory. */
    for (i = 0; i < gcdSYSTEM_RESERVE_COUNT; i++) {
        physHandle = (PLINUX_MDL)device->contiguousPhysicals[i];
        bytes = device->contiguousSizes[i];

        if (bytes) {
            gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
        }
    }
#endif

    /* External shared SRAM memory. */
    for (i = 0; i < gcvSRAM_EXT_COUNT; i++) {
        physHandle = (PLINUX_MDL)device->extSRAMPhysicals[i];
        bytes = device->extSRAMSizes[i];
        if (bytes) {
            gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
        }
    }

    /* Per core SRAM reserved usage. */
    for (i = 0; i < gcvSRAM_INTER_COUNT; i++) {
        if (!Kernel->sRAMPhysFaked[i]) {
            physHandle = (PLINUX_MDL)Kernel->sRAMPhysical[i];
            bytes      = Kernel->sRAMSizes[i];
            if (bytes) {
                gckOS_UnmapMemoryEx(Kernel->os, physHandle, bytes, Logical, Pid);
            }
        }
    }

    /* Retunn the status. */
    gcmkFOOTER_NO();
    return status;
}

/******************************************************************************
 **
 **  gckKERNEL_MapVideoMemory
 **
 **  Get the logical address for a hardware specific memory address for the
 **  current process.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gctBOOL InUserSpace
 **          gcvTRUE to map the memory into the user space.
 **
 **      gcePOOL Pool
 **          Specify pool type.
 **
 **      gctUINT32 Offset
 **          Offset to pool start.
 **
 **      gctUINT32 Bytes
 **          Number of bytes to map.
 **
 **  OUTPUT:
 **
 **      gctPOINTER *Logical
 **          Pointer to a variable that will hold the logical address of the
 **          specified memory address.
 */
gceSTATUS
gckKERNEL_MapVideoMemory(IN gckKERNEL    Kernel,
                         IN gctBOOL      InUserSpace,
                         IN gcePOOL      Pool,
                         IN gctPHYS_ADDR Physical,
                         IN gctSIZE_T    Offset,
                         IN gctSIZE_T    Bytes,
                         OUT gctPOINTER  *Logical)
{
    gckGALDEVICE galDevice    = gcvNULL;
    gckDEVICE    device       = gcvNULL;
    gctSIZE_T    bytes        = 0;
    gctPHYS_ADDR physHandle   = gcvNULL;
    gceSTATUS    status       = gcvSTATUS_OK;
    gctPOINTER   logical      = gcvNULL;
    gctUINT64    mappingInOne = 1;

    gcmkHEADER_ARG("Kernel=%p InUserSpace=%d Pool=%d Offset=%X Bytes=%X",
                   Kernel, InUserSpace, Pool, Offset, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);
    gcmkVERIFY_ARGUMENT(Logical);

    if (Physical)
        gcmkONERROR(gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne));

    if (mappingInOne) {
        /* Extract the pointer to the gckGALDEVICE class. */
        galDevice = (gckGALDEVICE)Kernel->context;
        device = Kernel->device;

        /* Dispatch on pool. */
        switch (Pool) {
        case gcvPOOL_LOCAL_INTERNAL:
            physHandle = (PLINUX_MDL)galDevice->internalPhysical;
            bytes      = galDevice->internalSize;
            break;

        case gcvPOOL_LOCAL_EXTERNAL:
            physHandle = (PLINUX_MDL)device->externalPhysical;
            bytes      = device->externalSize;
            break;

        case gcvPOOL_SYSTEM:
            /* System memory. */
            physHandle = (PLINUX_MDL)device->contiguousPhysicals[device->memIndex];
            bytes      = device->contiguousSizes[device->memIndex];
            break;

#if gcdENABLE_40BIT_VA
        case gcvPOOL_SYSTEM_32BIT_VA:
            /* System memory. */
            physHandle = (PLINUX_MDL)galDevice->lowContiguousPhysical;
            bytes      = galDevice->lowContiguousSize;
            break;
#endif

        case gcvPOOL_EXTERNAL_SRAM:
            /* External shared SRAM memory. */
            physHandle = (PLINUX_MDL)device->extSRAMPhysicals[device->extSRAMIndex];
            bytes      = device->extSRAMSizes[device->extSRAMIndex];
            break;

        case gcvPOOL_INTERNAL_SRAM:
            /* Per core SRAM reserved usage. */
            if (Kernel->sRAMPhysFaked[Kernel->sRAMIndex]) {
                *Logical = gcvNULL;

                gcmkFOOTER_NO();
                return gcvSTATUS_OK;
            }
            /* Per core SRAM memory block. */
            physHandle = (PLINUX_MDL)Kernel->sRAMPhysical[Kernel->sRAMIndex];
            bytes      = Kernel->sRAMSizes[Kernel->sRAMIndex];
            break;

        default:
            /* Invalid memory pool. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

    } else {
        physHandle = (PLINUX_MDL)Physical;
        bytes      = Bytes;
        Offset     = 0;
    }

    gcmkONERROR(gckOS_LockPages(Kernel->os, physHandle, bytes, gcvFALSE, &logical));
    /* Build logical address of specified address. */
    *Logical = (gctPOINTER)((gctUINT8_PTR)logical + Offset);
OnError:
    /* Retunn the status. */
    gcmkFOOTER_ARG("*Logical=%p", gcmOPT_POINTER(Logical));
    return status;
}

/******************************************************************************
 **
 **  gckKERNEL_UnmapVideoMemory
 **
 **  Unmap video memory for the current process.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gcePOOL Pool
 **          Specify pool type.

 **      gctPHYS_ADDR Physical
 **          Node ptr.
 **
 **      gctUINT32 Pid
 **          Process ID of the current process.
 **
 **      gctSIZE_T Bytes
 **          Number of bytes to map.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gckKERNEL_UnmapVideoMemory(IN gckKERNEL    Kernel,
                           IN gcePOOL      Pool,
                           IN gctPHYS_ADDR Physical,
                           IN gctPOINTER   Logical,
                           IN gctUINT32    Pid,
                           IN gctSIZE_T    Bytes)
{
    gceSTATUS    status       = gcvSTATUS_OK;
    gckGALDEVICE galDevice    = gcvNULL;
    gckDEVICE    device       = gcvNULL;
    gctSIZE_T    bytes        = 0;
    gctPHYS_ADDR physHandle   = gcvNULL;
    gctUINT64    mappingInOne = 1;

    gcmkHEADER_ARG("Logical=%p pid=%u Bytes=%zu", Logical, Pid, Bytes);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (Logical == gcvNULL)
        return gcvSTATUS_OK;

    if (Physical)
        gcmkONERROR(gckOS_QueryOption(Kernel->os, "allMapInOne", &mappingInOne));

    if (mappingInOne) {
        /* Extract the pointer to the gckGALDEVICE class. */
        galDevice = (gckGALDEVICE)Kernel->context;
        device = Kernel->device;

        /* Dispatch on pool. */
        switch (Pool) {
        case gcvPOOL_LOCAL_INTERNAL:
            physHandle = (PLINUX_MDL)galDevice->internalPhysical;
            bytes = galDevice->internalSize;
            break;

        case gcvPOOL_LOCAL_EXTERNAL:
            physHandle = (PLINUX_MDL)device->externalPhysical;
            bytes = device->externalSize;
            break;

        case gcvPOOL_SYSTEM:
            /* System memory. */
            physHandle = (PLINUX_MDL)device->contiguousPhysicals[device->memIndex];
            bytes = device->contiguousSizes[device->memIndex];
            break;

#if gcdENABLE_40BIT_VA
        case gcvPOOL_SYSTEM_32BIT_VA:
            /* System memory. */
            physHandle = (PLINUX_MDL)galDevice->lowContiguousPhysical;
            bytes = galDevice->lowContiguousSize;
            break;
#endif

        case gcvPOOL_EXTERNAL_SRAM:
            /* External shared SRAM memory. */
            physHandle = (PLINUX_MDL)device->extSRAMPhysicals[device->extSRAMIndex];
            bytes = device->extSRAMSizes[device->extSRAMIndex];
            break;

        case gcvPOOL_INTERNAL_SRAM:
            /* Per core SRAM reserved usage. */
            if (Kernel->sRAMPhysFaked[Kernel->sRAMIndex]) {
                gcmkFOOTER_NO();
                return gcvSTATUS_OK;
            }
            /* Per core SRAM memory block. */
            physHandle = (PLINUX_MDL)Kernel->sRAMPhysical[Kernel->sRAMIndex];
            bytes = Kernel->sRAMSizes[Kernel->sRAMIndex];
            break;

        default:
            /* Invalid memory pool. */
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
    } else {
        physHandle = (PLINUX_MDL)Physical;
        bytes = Bytes;
    }

    gcmkONERROR(gckOS_UnlockPages(Kernel->os, physHandle, bytes, Logical));

OnError:
    /* Retunn the status. */
    gcmkFOOTER_NO();
    return status;
}

/******************************************************************************
 **
 **  gckKERNEL_Notify
 **
 **  This function iscalled by clients to notify the gckKERNRL object of an
 **  event.
 **
 **  INPUT:
 **
 **      gckKERNEL Kernel
 **          Pointer to an gckKERNEL object.
 **
 **      gceNOTIFY Notification
 **          Notification event.
 **
 **  OUTPUT:
 **
 **      Nothing.
 */
gceSTATUS
gckKERNEL_Notify(IN gckKERNEL Kernel, IN gceNOTIFY Notification)
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Kernel=%p Notification=%d", Kernel, Notification);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    /* Dispatch on notifcation. */
    switch (Notification) {
    case gcvNOTIFY_INTERRUPT:
        /* Process the interrupt. */
#if COMMAND_PROCESSOR_VERSION > 1
        status = gckINTERRUPT_Notify(Kernel->interrupt, 0);
#else
        status = gckHARDWARE_Notify(Kernel->hardware);
#endif
        break;

    default:
        break;
    }

    /* Success. */
    gcmkFOOTER();
    return status;
}

#if gcdENABLE_VIDEO_MEMORY_MIRROR
gceSTATUS
gckKERNEL_SyncVideoMemoryMirror(gckKERNEL Kernel,
                                gckVIDMEM_NODE Node,
                                gctSIZE_T Offset,
                                gctSIZE_T Bytes,
                                gctUINT32 Reason)
{
    gceSTATUS    status = gcvSTATUS_NOT_SUPPORTED;
    gcsPLATFORM *platform;

    gcmkHEADER();

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Kernel, gcvOBJ_KERNEL);

    if (!Node->mirror.mirrorNode) {
        gcmkFOOTER();
        return gcvSTATUS_INVALID_ARGUMENT;
    }

    platform = Kernel->os->device->platform;

    if (platform && platform->ops->dmaCopy) {
        gckVIDMEM_NODE srcNode = gcvNULL, dstNode = gcvNULL;
        gcsDMA_TRANS_INFO info;

        /* Mirror node is local memory */
        if (Node->mirror.type == gcvMIRROR_TYPE_LOCAL_MEMORY_MIRROR) {
            if (Reason == gcvSYNC_MEMORY_DIRECTION_LOCAL_TO_SYSTEM) {
                /* local -> system */
                srcNode = Node->mirror.mirrorNode;
                dstNode = Node;
            } else if (Reason == gcvSYNC_MEMORY_DIRECTION_SYSTEM_TO_LOCAL) {
                /* system -> local */
                srcNode = Node;
                dstNode = Node->mirror.mirrorNode;
            }
            /* Mirror node is system memory */
        } else if (Node->mirror.type == gcvMIRROR_TYPE_SYSTEM_MEMORY_MIRROR) {
            if (Reason == gcvSYNC_MEMORY_DIRECTION_LOCAL_TO_SYSTEM) {
                /* local -> system */
                srcNode = Node;
                dstNode = Node->mirror.mirrorNode;
            } else if (Reason == gcvSYNC_MEMORY_DIRECTION_SYSTEM_TO_LOCAL) {
                /* system -> local */
                srcNode = Node->mirror.mirrorNode;
                dstNode = Node;
            }
        } else {
            gcmkFOOTER();
            return gcvSTATUS_INVALID_ARGUMENT;
        }

        info.src_node = srcNode;
        info.dst_node = dstNode;
        info.offset = Offset;
        info.bytes = Bytes;
        info.reason = Reason;

        status = platform->ops->dmaCopy(Kernel, &info);
    }

    gcmkFOOTER();
    return status;
}
#endif
