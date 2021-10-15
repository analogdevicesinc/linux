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
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/sched.h>

#define _GC_OBJ_ZONE    gcvZONE_DEVICE

static gckGALDEVICE galDevice;

extern gcTA globalTA[16];

/******************************************************************************\
******************************** Debugfs Support *******************************
\******************************************************************************/

/******************************************************************************\
***************************** DEBUG SHOW FUNCTIONS *****************************
\******************************************************************************/

int gc_info_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    int i = 0;
    gceCHIPMODEL chipModel = 0;
    gctUINT32 chipRevision = 0;
    gctUINT32 productID = 0;
    gctUINT32 ecoID = 0;

    if (!device)
        return -ENXIO;

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (device->kernels[i])
        {
            if (i == gcvCORE_VG)
            {
#if gcdENABLE_VG
                chipModel = device->kernels[i]->vg->hardware->chipModel;
                chipRevision = device->kernels[i]->vg->hardware->chipRevision;
#endif
            }
            else
            {
                chipModel = device->kernels[i]->hardware->identity.chipModel;
                chipRevision = device->kernels[i]->hardware->identity.chipRevision;
                productID = device->kernels[i]->hardware->identity.productID;
                ecoID = device->kernels[i]->hardware->identity.ecoID;
            }

            seq_printf(m, "gpu      : %d\n", i);
            seq_printf(m, "model    : %4x\n", chipModel);
            seq_printf(m, "revision : %4x\n", chipRevision);
            seq_printf(m, "product  : %4x\n", productID);
            seq_printf(m, "eco      : %4x\n", ecoID);
            seq_printf(m, "\n");
        }
    }

    return 0;
}

int gc_clients_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;

    gckKERNEL kernel = _GetValidKernel(device);

    gcsDATABASE_PTR database;
    gctINT i, pid;
    char name[24];

    if (!kernel)
        return -ENXIO;

    seq_printf(m, "%-8s%s\n", "PID", "NAME");
    seq_printf(m, "------------------------\n");

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    /* Walk the databases. */
    for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
    {
        for (database = kernel->db->db[i];
             database != gcvNULL;
             database = database->next)
        {
            pid = database->processID;

            gcmkVERIFY_OK(gckOS_GetProcessNameByPid(pid, gcmSIZEOF(name), name));

            seq_printf(m, "%-8d%s\n", pid, name);
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    /* Success. */
    return 0;
}

int gc_meminfo_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gckKERNEL kernel = _GetValidKernel(device);
    gckVIDMEM memory;
    gceSTATUS status;
    gcsDATABASE_PTR database;
    gctUINT32 i;

    gctUINT32 free = 0, used = 0, total = 0, minFree = 0, maxUsed = 0;

    gcsDATABASE_COUNTERS virtualCounter = {0, 0, 0};
    gcsDATABASE_COUNTERS nonPagedCounter = {0, 0, 0};

    if (!kernel)
        return -ENXIO;

    status = gckKERNEL_GetVideoMemoryPool(kernel, gcvPOOL_SYSTEM, &memory);

    if (gcmIS_SUCCESS(status))
    {
        gcmkVERIFY_OK(
            gckOS_AcquireMutex(memory->os, memory->mutex, gcvINFINITE));

        free    = memory->freeBytes;
        minFree = memory->minFreeBytes;
        used    = memory->bytes - memory->freeBytes;
        maxUsed = memory->bytes - memory->minFreeBytes;
        total   = memory->bytes;

        gcmkVERIFY_OK(gckOS_ReleaseMutex(memory->os, memory->mutex));
    }

    seq_printf(m, "VIDEO MEMORY:\n");
    seq_printf(m, "  POOL SYSTEM:\n");
    seq_printf(m, "    Free :    %10u B\n", free);
    seq_printf(m, "    Used :    %10u B\n", used);
    seq_printf(m, "    MinFree : %10u B\n", minFree);
    seq_printf(m, "    MaxUsed : %10u B\n", maxUsed);
    seq_printf(m, "    Total :   %10u B\n", total);

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    /* Walk the databases. */
    for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
    {
        for (database = kernel->db->db[i];
             database != gcvNULL;
             database = database->next)
        {
            gcsDATABASE_COUNTERS * counter;
            counter = &database->vidMemPool[gcvPOOL_VIRTUAL];
            virtualCounter.bytes += counter->bytes;
            virtualCounter.maxBytes += counter->maxBytes;

            counter = &database->nonPaged;
            nonPagedCounter.bytes += counter->bytes;
            nonPagedCounter.bytes += counter->maxBytes;
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    seq_printf(m, "  POOL VIRTUAL:\n");
    seq_printf(m, "    Used :    %10llu B\n", virtualCounter.bytes);
    seq_printf(m, "    MaxUsed : %10llu B\n", virtualCounter.bytes);

    return 0;
}

static const char * vidmemTypeStr[gcvVIDMEM_TYPE_COUNT] =
{
    "Generic",
    "Index",
    "Vertex",
    "Texture",
    "RenderTarget",
    "Depth",
    "Bitmap",
    "TileStatus",
    "Image",
    "Mask",
    "Scissor",
    "HZ",
    "ICache",
    "TxDesc",
    "Fence",
    "TFBHeader",
    "Command",
};

static const char * poolStr[gcvPOOL_NUMBER_OF_POOLS] =
{
    "Unknown",
    "Default",
    "Local",
    "Internal",
    "External",
    "Unified",
    "System",
    "Sram",
    "Virtual",
    "User",
    "Insram",
    "Exsram",
};

static void
_ShowDummyRecord(
    IN struct seq_file *File,
    IN gcsDATABASE_PTR Database
    )
{
}

static void
_ShowVideoMemoryRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;
    gctUINT32 handle;
    gckVIDMEM_NODE nodeObject;
    gctPHYS_ADDR_T physical;
    gctINT32 refCount = 0;
    gctINT32 lockCount = 0;
    gceSTATUS status;

    seq_printf(m, "Video Memory Node:\n");
    seq_printf(m, "  handle         nodeObject       size         type     pool     physical  ref lock\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_VIDEO_MEMORY)
            {
                continue;
            }

            handle = gcmPTR2INT32(record->data);

            status = gckVIDMEM_HANDLE_Lookup2(
                record->kernel,
                Database,
                handle,
                &nodeObject
                );

            if (gcmIS_ERROR(status))
            {
                seq_printf(m, "%6u Invalid Node\n", handle);
                continue;
            }

            gcmkONERROR(gckVIDMEM_NODE_GetPhysical(record->kernel, nodeObject, 0, &physical));
            gcmkONERROR(gckVIDMEM_NODE_GetReference(record->kernel, nodeObject, &refCount));
            gcmkONERROR(gckVIDMEM_NODE_GetLockCount(record->kernel, nodeObject, &lockCount));

            seq_printf(m, "%#8x %#18lx %10lu %12s %8s %#12llx %4d %4d\n",
                handle,
                (unsigned long)nodeObject,
                (unsigned long)record->bytes,
                vidmemTypeStr[nodeObject->type],
                poolStr[nodeObject->pool],
                physical,
                refCount,
                lockCount
                );
        }
    }

OnError:
    return;
}

static void
_ShowCommandBufferRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    return;
}

static void
_ShowNonPagedRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;

    seq_printf(m, "NonPaged Memory:\n");
    seq_printf(m, "  name              vaddr       size\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_NON_PAGED)
            {
                continue;
            }

            seq_printf(m, "%6u %#18lx %10lu\n",
                gcmPTR2INT32(record->physical),
                (unsigned long)record->data,
                (unsigned long)record->bytes
                );
        }
    }
}

static void
_ShowContiguousRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    return;
}

static void
_ShowSignalRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;

    seq_printf(m, "User signal:\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_SIGNAL)
            {
                continue;
            }

            seq_printf(m, "%#10x\n", gcmPTR2INT32(record->data));
        }
    }
}

static void
_ShowLockRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;
    gceSTATUS status;
    gctUINT32 handle;
    gckVIDMEM_NODE nodeObject;

    seq_printf(m, "Video Memory Lock:\n");
    seq_printf(m, "  handle         nodeObject              vaddr\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_VIDEO_MEMORY_LOCKED)
            {
                continue;
            }

            handle = gcmPTR2INT32(record->data);

            status = gckVIDMEM_HANDLE_Lookup2(
                record->kernel,
                Database,
                handle,
                &nodeObject
                );

            if (gcmIS_ERROR(status))
            {
                nodeObject = gcvNULL;
            }

            seq_printf(m, "%#8x %#18lx %#18lx\n",
                handle,
                (unsigned long)nodeObject,
                (unsigned long)record->physical
                );
        }
    }
}

static void
_ShowContextRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;

    seq_printf(m, "Context:\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_CONTEXT)
            {
                continue;
            }

            seq_printf(m, "%6u\n", gcmPTR2INT32(record->data));
        }
    }
}

static void
_ShowMapMemoryRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;

    seq_printf(m, "Map Memory:\n");
    seq_printf(m, "  name              vaddr       size\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_MAP_MEMORY)
            {
                continue;
            }

            seq_printf(m, "%#6lx %#18lx %10lu\n",
                (unsigned long)record->physical,
                (unsigned long)record->data,
                (unsigned long)record->bytes
                );
        }
    }
}

static void
_ShowMapUserMemoryRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    return;
}

static void
_ShowShbufRecord(
    IN struct seq_file *m,
    IN gcsDATABASE_PTR Database
    )
{
    gctUINT i;

    seq_printf(m, "ShBuf:\n");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR r = Database->list[i];

        while (r != NULL)
        {
            gcsDATABASE_RECORD_PTR record = r;
            r = r->next;

            if (record->type != gcvDB_SHBUF)
            {
                continue;
            }

            seq_printf(m, "%#8x\n", gcmPTR2INT32(record->data));
        }
    }
}

static void
_ShowCounters(
    struct seq_file *File,
    gcsDATABASE_PTR Database
    )
{
    gctUINT i = 0;

    static const char * otherCounterNames[] = {
        "AllocNonPaged",
        "AllocContiguous",
        "MapUserMemory",
        "MapMemory",
    };

    gcsDATABASE_COUNTERS * otherCounters[] = {
        &Database->nonPaged,
        &Database->contiguous,
        &Database->mapUserMemory,
        &Database->mapMemory,
    };

    seq_printf(File, "%-16s %16s %16s %16s\n", "", "Current", "Maximum", "Total");

    /* Print surface type counters. */
    seq_printf(File, "%-16s %16lld %16lld %16lld\n",
               "All-Types",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvVIDMEM_TYPE_COUNT; i++)
    {
        seq_printf(File, "%-16s %16lld %16lld %16lld\n",
                   vidmemTypeStr[i],
                   Database->vidMemType[i].bytes,
                   Database->vidMemType[i].maxBytes,
                   Database->vidMemType[i].totalBytes);
    }
    seq_puts(File, "\n");

    /* Print surface pool counters. */
    seq_printf(File, "%-16s %16lld %16lld %16lld\n",
               "All-Pools",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvPOOL_NUMBER_OF_POOLS; i++)
    {
        seq_printf(File, "%-16s %16lld %16lld %16lld\n",
                   poolStr[i],
                   Database->vidMemPool[i].bytes,
                   Database->vidMemPool[i].maxBytes,
                   Database->vidMemPool[i].totalBytes);
    }
    seq_puts(File, "\n");

    /* Print other counters. */
    for (i = 0; i < gcmCOUNTOF(otherCounterNames); i++)
    {
        seq_printf(File, "%-16s %16lld %16lld %16lld\n",
                   otherCounterNames[i],
                   otherCounters[i]->bytes,
                   otherCounters[i]->maxBytes,
                   otherCounters[i]->totalBytes);
    }
    seq_puts(File, "\n");
}

static int
_ShowRecord(
    IN struct seq_file *File,
    IN gcsDATABASE_PTR Database,
    IN gcsDATABASE_RECORD_PTR Record
    )
{
    gctUINT32 handle;
    gckVIDMEM_NODE nodeObject;
    gctPHYS_ADDR_T physical;
    gceSTATUS status = gcvSTATUS_OK;

    static const char * recordTypes[gcvDB_NUM_TYPES] = {
        "Unknown",
        "VideoMemory",
        "CommandBuffer",
        "NonPaged",
        "Contiguous",
        "Signal",
        "VidMemLock",
        "Context",
        "Idel",
        "MapMemory",
        "MapUserMemory",
        "ShBuf",
    };

    handle = gcmPTR2INT32(Record->data);

    if (Record->type == gcvDB_VIDEO_MEMORY || Record->type == gcvDB_VIDEO_MEMORY_LOCKED)
    {
        status = gckVIDMEM_HANDLE_Lookup2(
            Record->kernel,
            Database,
            handle,
            &nodeObject
        );

        if (gcmIS_ERROR(status))
        {
            seq_printf(File, "%6u Invalid Node\n", handle);
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        gcmkONERROR(gckVIDMEM_NODE_GetPhysical(Record->kernel, nodeObject, 0, &physical));
    }
    else
    {
        physical = (gctUINT64)(gctUINTPTR_T)Record->physical;
    }

    seq_printf(File, "%-14s %3d %16x %16zx %16zu\n",
        recordTypes[Record->type],
        Record->kernel->core,
        gcmPTR2INT32(Record->data),
        (size_t) physical,
        Record->bytes
        );

OnError:
    return status;
}

static void
_ShowDataBaseOldFormat(
    IN struct seq_file *File,
    IN gcsDATABASE_PTR Database
    )
{
    gctINT pid;
    gctUINT i;
    char name[24];

    /* Process ID and name */
    pid = Database->processID;
    gcmkVERIFY_OK(gckOS_GetProcessNameByPid(pid, gcmSIZEOF(name), name));

    seq_printf(File, "--------------------------------------------------------------------------------\n");
    seq_printf(File, "Process: %-8d %s\n", pid, name);

    seq_printf(File, "Records:\n");

    seq_printf(File, "%14s %3s %16s %16s %16s\n",
               "Type", "GPU", "Data/Node", "Physical/Node", "Bytes");

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR record = Database->list[i];

        while (record != NULL)
        {
            _ShowRecord(File, Database, record);
            record = record->next;
        }
    }

    seq_printf(File, "Counters:\n");

    _ShowCounters(File, Database);
}

static void
_ShowDatabase(
    IN struct seq_file *File,
    IN gcsDATABASE_PTR Database
    )
{
    gctINT pid;
    gctUINT i;
    char name[24];
    gctBOOL hasType[gcvDB_NUM_TYPES] = {0,};
    void (* showFuncs[])(struct seq_file *, gcsDATABASE_PTR) =
    {
        _ShowDummyRecord,
        _ShowVideoMemoryRecord,
        _ShowCommandBufferRecord,
        _ShowNonPagedRecord,
        _ShowContiguousRecord,
        _ShowSignalRecord,
        _ShowLockRecord,
        _ShowContextRecord,
        _ShowDummyRecord,
        _ShowMapMemoryRecord,
        _ShowMapUserMemoryRecord,
        _ShowShbufRecord,
    };

    gcmSTATIC_ASSERT(gcmCOUNTOF(showFuncs) == gcvDB_NUM_TYPES,
                     "DB type mismatch");

    /* Process ID and name */
    pid = Database->processID;
    gcmkVERIFY_OK(gckOS_GetProcessNameByPid(pid, gcmSIZEOF(name), name));

    seq_printf(File, "--------------------------------------------------------------------------------\n");
    seq_printf(File, "Process: %-8d %s\n", pid, name);

    for (i = 0; i < gcmCOUNTOF(Database->list); i++)
    {
        gcsDATABASE_RECORD_PTR record = Database->list[i];

        while (record != NULL)
        {
            hasType[record->type] = gcvTRUE;
            record = record->next;
        }
    }

    for (i = 0; i < gcvDB_NUM_TYPES; i++)
    {
        if (hasType[i])
        {
            showFuncs[i](File, Database);
        }
    }
}

static int
gc_db_show_old(struct seq_file *m, void *data)
{
    gcsDATABASE_PTR database;
    gctINT i;
    static gctUINT64 idleTime = 0;
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gckKERNEL kernel = _GetValidKernel(device);

    if (!kernel)
        return -ENXIO;

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    if (kernel->db->idleTime)
    {
        /* Record idle time if DB upated. */
        idleTime = kernel->db->idleTime;
        kernel->db->idleTime = 0;
    }

    /* Idle time since last call */
    seq_printf(m, "GPU Idle: %llu ns\n",  idleTime);

    /* Walk the databases. */
    for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
    {
        for (database = kernel->db->db[i];
             database != gcvNULL;
             database = database->next)
        {
            _ShowDataBaseOldFormat(m, database);
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    return 0 ;
}

static int
gc_db_show(struct seq_file *m, void *data)
{
    gcsDATABASE_PTR database;
    gctINT i;
    static gctUINT64 idleTime = 0;
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gckKERNEL kernel = _GetValidKernel(device);

    if (!kernel)
        return -ENXIO;

    /* Acquire the database mutex. */
    gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

    if (kernel->db->idleTime)
    {
        /* Record idle time if DB upated. */
        idleTime = kernel->db->idleTime;
        kernel->db->idleTime = 0;
    }

    /* Idle time since last call */
    seq_printf(m, "GPU Idle: %llu ns\n",  idleTime);

    /* Walk the databases. */
    for (i = 0; i < gcmCOUNTOF(kernel->db->db); ++i)
    {
        for (database = kernel->db->db[i];
             database != gcvNULL;
             database = database->next)
        {
            _ShowDatabase(m, database);
        }
    }

    /* Release the database mutex. */
    gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));

    return 0 ;
}

static int
gc_version_show(struct seq_file *m, void *data)
{
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gcsPLATFORM * platform = gcvNULL;

    if (!device)
        return -ENXIO;

    platform = device->platform;
    if (!platform)
        return -ENXIO;

    seq_printf(m, "%s built at %s\n",  gcvVERSION_STRING, HOST);

    if (platform->name)
    {
        seq_printf(m, "Platform path: %s\n", platform->name);
    }
    else
    {
        seq_printf(m, "Code path: %s\n", __FILE__);
    }

    return 0 ;
}

static void print_ull(char dest[32], unsigned long long u)
{
    unsigned t[7];
    int i;

    if (u < 1000)
    {
        sprintf(dest, "%27llu", u);
        return;
    }

    for (i = 0; i < 7 && u; i++)
    {
        t[i] = do_div(u, 1000);
    }

    dest += sprintf(dest, "%*s", (7 - i) * 4, "");
    dest += sprintf(dest, "%3u", t[--i]);

    for (i--; i >= 0; i--)
    {
        dest += sprintf(dest, ",%03u", t[i]);
    }
}

/*******************************************************************************
**
** Show PM state timer.
**
** Entry is called as 'idle' for compatible reason, it shows more information
** than idle actually.
**
**  Start: Start time of this counting period.
**  End: End time of this counting peroid.
**  On: Time GPU stays in gcvPOWER_0N.
**  Off: Time GPU stays in gcvPOWER_0FF.
**  Idle: Time GPU stays in gcvPOWER_IDLE.
**  Suspend: Time GPU stays in gcvPOWER_SUSPEND.
*/
static int
gc_idle_show(struct seq_file *m, void *data)
{
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gckKERNEL kernel = _GetValidKernel(device);
    char str[32];

    gctUINT64 on;
    gctUINT64 off;
    gctUINT64 idle;
    gctUINT64 suspend;

    if (!kernel)
        return -ENXIO;

    gckHARDWARE_QueryStateTimer(kernel->hardware, &on, &off, &idle, &suspend);

    /* Idle time since last call */
    print_ull(str, on);
    seq_printf(m, "On:      %s ns\n",  str);
    print_ull(str, off);
    seq_printf(m, "Off:     %s ns\n",  str);
    print_ull(str, idle);
    seq_printf(m, "Idle:    %s ns\n",  str);
    print_ull(str, suspend);
    seq_printf(m, "Suspend: %s ns\n",  str);

    return 0 ;
}

extern void
_DumpState(
    IN gckKERNEL Kernel
    );

/*******************************************************************************
**
** Show PM state timer.
**
** Entry is called as 'idle' for compatible reason, it shows more information
** than idle actually.
**
**  Start: Start time of this counting period.
**  End: End time of this counting peroid.
**  On: Time GPU stays in gcvPOWER_0N.
**  Off: Time GPU stays in gcvPOWER_0FF.
**  Idle: Time GPU stays in gcvPOWER_IDLE.
**  Suspend: Time GPU stays in gcvPOWER_SUSPEND.
*/
static int dumpCore = 0;
static gctBOOL dumpAllCore = gcvFALSE;

static int
gc_dump_trigger_show(struct seq_file *m, void *data)
{
#if gcdENABLE_3D || gcdENABLE_2D
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gckKERNEL kernel = gcvNULL;
    gckHARDWARE Hardware = gcvNULL;
    gctBOOL powerManagement = gcvFALSE;
    gceSTATUS status = gcvSTATUS_OK;
    gceCHIPPOWERSTATE statesStored, state;

    if (((dumpCore < gcvCORE_MAJOR) || (dumpCore >= gcvCORE_COUNT)) && (!dumpAllCore))
    {
        return -ENXIO;
    }

    seq_printf(m, "Dump one core: For example, dump core 0: echo 0 > /sys/kernel/debug/gc/dump_trigger; cat /sys/kernel/debug/gc/dump_trigger\n");
    seq_printf(m, "Dump all cores: echo all > /sys/kernel/debug/gc/dump_trigger; cat /sys/kernel/debug/gc/dump_trigger\n");
    seq_printf(m, "The dump will be in [dmesg].\n");

    if (dumpAllCore)
    {
        gctINT8 i = 0;

        for (i = 0; i < gcvCORE_COUNT; ++i)
        {
            if (!device->kernels[i])
            {
                continue;
            }

            kernel = device->kernels[i];
            Hardware = kernel->hardware;
            powerManagement = Hardware->options.powerManagement;

            if (powerManagement)
            {
                gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                    Hardware, gcvFALSE
                    ));
            }

            gcmkONERROR(gckHARDWARE_QueryPowerState(
                Hardware, &statesStored
                ));

            gcmkONERROR(gckHARDWARE_SetPowerState(
                Hardware, gcvPOWER_ON_AUTO
                ));

            _DumpState(kernel);

            switch(statesStored)
            {
            case gcvPOWER_OFF:
                state = gcvPOWER_OFF_BROADCAST;
                break;
            case gcvPOWER_IDLE:
                state = gcvPOWER_IDLE_BROADCAST;
                break;
            case gcvPOWER_SUSPEND:
                state = gcvPOWER_SUSPEND_BROADCAST;
                break;
            case gcvPOWER_ON:
                state = gcvPOWER_ON_AUTO;
                break;
            default:
                state = statesStored;
                break;
            }

            if (powerManagement)
            {
                gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                    Hardware, gcvTRUE
                    ));
            }

            gcmkONERROR(gckHARDWARE_SetPowerState(
                Hardware, state
                ));

        }
    }
    else
    {
        if (device->kernels[dumpCore])
        {
            kernel = device->kernels[dumpCore];
        }
        else
        {
            seq_printf(m, "Dump core from invalid coreid.\n");
            goto OnError;
        }

        Hardware = kernel->hardware;
        powerManagement = Hardware->options.powerManagement;

        if (powerManagement)
        {
            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                Hardware, gcvFALSE
                ));
        }

        gcmkONERROR(gckHARDWARE_QueryPowerState(
            Hardware, &statesStored
            ));

        gcmkONERROR(gckHARDWARE_SetPowerState(
            Hardware, gcvPOWER_ON_AUTO
            ));

        _DumpState(kernel);

        switch(statesStored)
        {
        case gcvPOWER_OFF:
            state = gcvPOWER_OFF_BROADCAST;
            break;
        case gcvPOWER_IDLE:
            state = gcvPOWER_IDLE_BROADCAST;
            break;
        case gcvPOWER_SUSPEND:
            state = gcvPOWER_SUSPEND_BROADCAST;
            break;
        case gcvPOWER_ON:
            state = gcvPOWER_ON_AUTO;
            break;
        default:
            state = statesStored;
            break;
        }

        if (powerManagement)
        {
            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                Hardware, gcvTRUE
                ));
        }

        gcmkONERROR(gckHARDWARE_SetPowerState(
            Hardware, state
            ));
    }

OnError:
#endif
    return 0;
}

static int dumpProcess = 0;

static void
_ShowVideoMemoryOldFormat(
    struct seq_file *File,
    gcsDATABASE_PTR Database
    )
{
    gctUINT i = 0;

    static const char * otherCounterNames[] = {
        "AllocNonPaged",
        "AllocContiguous",
        "MapUserMemory",
        "MapMemory",
    };

    gcsDATABASE_COUNTERS * otherCounters[] = {
        &Database->nonPaged,
        &Database->contiguous,
        &Database->mapUserMemory,
        &Database->mapMemory,
    };

    seq_printf(File, "%-16s %16s %16s %16s\n", "", "Current", "Maximum", "Total");

    /* Print surface type counters. */
    seq_printf(File, "%-16s %16llu %16llu %16llu\n",
               "All-Types",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvVIDMEM_TYPE_COUNT; i++)
    {
        seq_printf(File, "%-16s %16llu %16llu %16llu\n",
                   vidmemTypeStr[i],
                   Database->vidMemType[i].bytes,
                   Database->vidMemType[i].maxBytes,
                   Database->vidMemType[i].totalBytes);
    }
    seq_puts(File, "\n");

    /* Print surface pool counters. */
    seq_printf(File, "%-16s %16llu %16llu %16llu\n",
               "All-Pools",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvPOOL_NUMBER_OF_POOLS; i++)
    {
        seq_printf(File, "%-16s %16llu %16llu %16llu\n",
                   poolStr[i],
                   Database->vidMemPool[i].bytes,
                   Database->vidMemPool[i].maxBytes,
                   Database->vidMemPool[i].totalBytes);
    }
    seq_puts(File, "\n");

    /* Print other counters. */
    for (i = 0; i < gcmCOUNTOF(otherCounterNames); i++)
    {
        seq_printf(File, "%-16s %16llu %16llu %16llu\n",
                   otherCounterNames[i],
                   otherCounters[i]->bytes,
                   otherCounters[i]->maxBytes,
                   otherCounters[i]->totalBytes);
    }
    seq_puts(File, "\n");
}

static void
_ShowVideoMemory(
    struct seq_file *File,
    gcsDATABASE_PTR Database
    )
{
    gctUINT i = 0;

    static const char * otherCounterNames[] = {
        "AllocNonPaged",
        "MapMemory",
    };

    gcsDATABASE_COUNTERS * otherCounters[] = {
        &Database->nonPaged,
        &Database->mapMemory,
    };

    seq_printf(File, "%-16s %16s %16s %16s\n", "", "Current", "Maximum", "Total");

    /* Print surface type counters. */
    seq_printf(File, "%-16s %16llu %16llu %16llu\n",
               "All-Types",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvVIDMEM_TYPE_COUNT; i++)
    {
        seq_printf(File, "%-16s %16llu %16llu %16llu\n",
                   vidmemTypeStr[i],
                   Database->vidMemType[i].bytes,
                   Database->vidMemType[i].maxBytes,
                   Database->vidMemType[i].totalBytes);
    }
    seq_puts(File, "\n");

    /* Print surface pool counters. */
    seq_printf(File, "%-16s %16llu %16llu %16llu\n",
               "All-Pools",
               Database->vidMem.bytes,
               Database->vidMem.maxBytes,
               Database->vidMem.totalBytes);

    for (i = 1; i < gcvPOOL_NUMBER_OF_POOLS; i++)
    {
        seq_printf(File, "%-16s %16llu %16llu %16llu\n",
                   poolStr[i],
                   Database->vidMemPool[i].bytes,
                   Database->vidMemPool[i].maxBytes,
                   Database->vidMemPool[i].totalBytes);
    }
    seq_puts(File, "\n");

    /* Print other counters. */
    for (i = 0; i < gcmCOUNTOF(otherCounterNames); i++)
    {
        seq_printf(File, "%-16s %16llu %16llu %16llu\n",
                   otherCounterNames[i],
                   otherCounters[i]->bytes,
                   otherCounters[i]->maxBytes,
                   otherCounters[i]->totalBytes);
    }
    seq_puts(File, "\n");
}

static int gc_vidmem_show_old(struct seq_file *m, void *unused)
{
    gceSTATUS status;
    gcsDATABASE_PTR database;
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    char name[64];
    int i;

    gckKERNEL kernel = _GetValidKernel(device);

    if (!kernel)
        return -ENXIO;

    if (dumpProcess == 0)
    {
        /* Acquire the database mutex. */
        gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

        for (i = 0; i < gcmCOUNTOF(kernel->db->db); i++)
        {
            for (database = kernel->db->db[i];
                 database != gcvNULL;
                 database = database->next)
            {
                gckOS_GetProcessNameByPid(database->processID, gcmSIZEOF(name), name);
                seq_printf(m, "VidMem Usage (Process %u: %s):\n", database->processID, name);
                _ShowVideoMemoryOldFormat(m, database);
                seq_puts(m, "\n");
            }
        }

        /* Release the database mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));
    }
    else
    {
        /* Find the database. */
        status = gckKERNEL_FindDatabase(kernel, dumpProcess, gcvFALSE, &database);

        if (gcmIS_ERROR(status))
        {
            seq_printf(m, "ERROR: process %d not found\n", dumpProcess);
            return 0;
        }

        gckOS_GetProcessNameByPid(dumpProcess, gcmSIZEOF(name), name);
        seq_printf(m, "VidMem Usage (Process %d: %s):\n", dumpProcess, name);
        _ShowVideoMemoryOldFormat(m, database);
    }

    return 0;
}

static int gc_vidmem_show(struct seq_file *m, void *unused)
{
    gceSTATUS status;
    gcsDATABASE_PTR database;
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    char name[64];
    int i;

    gckKERNEL kernel = _GetValidKernel(device);

    if (!kernel)
        return -ENXIO;

    if (dumpProcess == 0)
    {
        /* Acquire the database mutex. */
        gcmkVERIFY_OK(
        gckOS_AcquireMutex(kernel->os, kernel->db->dbMutex, gcvINFINITE));

        for (i = 0; i < gcmCOUNTOF(kernel->db->db); i++)
        {
            for (database = kernel->db->db[i];
                 database != gcvNULL;
                 database = database->next)
            {
                gckOS_GetProcessNameByPid(database->processID, gcmSIZEOF(name), name);
                seq_printf(m, "VidMem Usage (Process %u: %s):\n", database->processID, name);
                _ShowVideoMemory(m, database);
                seq_puts(m, "\n");
            }
        }

        /* Release the database mutex. */
        gcmkVERIFY_OK(gckOS_ReleaseMutex(kernel->os, kernel->db->dbMutex));
    }
    else
    {
        /* Find the database. */
        status = gckKERNEL_FindDatabase(kernel, dumpProcess, gcvFALSE, &database);

        if (gcmIS_ERROR(status))
        {
            seq_printf(m, "ERROR: process %d not found\n", dumpProcess);
            return 0;
        }

        gckOS_GetProcessNameByPid(dumpProcess, gcmSIZEOF(name), name);
        seq_printf(m, "VidMem Usage (Process %d: %s):\n", dumpProcess, name);
        _ShowVideoMemory(m, database);
    }

    return 0;
}

static inline int strtoint_from_user(const char __user *s,
                        size_t count, int *res)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)
    int ret = kstrtoint_from_user(s, count, 10, res);

    return ret < 0 ? ret : count;
#else
    /* sign, base 2 representation, newline, terminator */
    char buf[1 + sizeof(long) * 8 + 1 + 1];

    size_t len = min(count, sizeof(buf) - 1);

    if (copy_from_user(buf, s, len))
        return -EFAULT;
    buf[len] = '\0';

    *res = (int) simple_strtol(buf, NULL, 0);

    return count;
#endif
}

static int gc_vidmem_write(const char __user *buf, size_t count, void* data)
{
    return strtoint_from_user(buf, count, &dumpProcess);
}

static int gc_dump_trigger_write(const char __user *buf, size_t count, void* data)
{
    char str[1 + sizeof(long) * 8 + 1 + 1];

    size_t len = min(count, sizeof(str) - 1);

    if (copy_from_user(str, buf, len))
        return -EFAULT;

    str[len] = '\0';

    if (str[0] == 'a' && str[1] == 'l' && str[2] == 'l')
    {
        dumpAllCore = gcvTRUE;
        return count;
    }
    else
    {
        dumpAllCore = gcvFALSE;
        return strtoint_from_user(buf, count, &dumpCore);
    }
}

static int gc_clk_show(struct seq_file* m, void* data)
{
    gcsINFO_NODE *node = m->private;
    gckGALDEVICE device = node->device;
    gctUINT i;
    gceSTATUS status;

    if (!device)
        return -ENXIO;

    for (i = gcvCORE_MAJOR; i < gcvCORE_COUNT; i++)
    {
        if (device->kernels[i])
        {
            gckHARDWARE hardware = device->kernels[i]->hardware;

            if (i == gcvCORE_VG)
            {
                continue;
            }

            status = gckHARDWARE_QueryFrequency(hardware);
            if (gcmIS_ERROR(status))
            {
                seq_printf(m, "query gpu%d clock fail.\n", i);
                continue;
            }

            if (hardware->mcClk)
            {
                seq_printf(m, "gpu%d mc clock: %d HZ.\n", i, hardware->mcClk);
            }

            if (hardware->shClk)
            {
                seq_printf(m, "gpu%d sh clock: %d HZ.\n", i, hardware->shClk);
            }
        }
    }

    return 0;
}

static gctUINT32 clkScale[2] = {0, 0};

static int _set_clk(const char* buf)
{
    gckHARDWARE hardware;
    gckGALDEVICE device = galDevice;
    gctINT n, j, k;
    gctBOOL isSpace = gcvFALSE;
    char data[20];

    memset(data, 0, 20);
    n = j = k = 0;

    while (gcvTRUE)
    {
        if ((buf[k] >= '0') && (buf[k] <= '9'))
        {
            if (isSpace)
            {
                data[n++] = ' ';
                isSpace = gcvFALSE;
            }
            data[n++] = buf[k];
        }
        else if (buf[k] == ' ')
        {
            if (n > 0)
            {
                isSpace = gcvTRUE;
            }
        }
        else if (buf[k] == '\n')
        {
            break;
        }
        else
        {
            printk("Error: command format must be this: echo \"0 32 32\" > /sys/kernel/debug/gc/clk\n");
            return 0;
        }

        k++;

        if (k >= 20)
        {
            break;
        }
    }

    if (3 == sscanf(data, "%d %d %d", &dumpCore, &clkScale[0], &clkScale[1])) {
        printk("Change core:%d MC scale:%d SH scale:%d\n",
                dumpCore, clkScale[0], clkScale[1]);
    } else {
        printk("usage: echo \"0 32 32\" > clk\n");
        return 0;
    }

    if (device->kernels[dumpCore])
    {
        hardware = device->kernels[dumpCore]->hardware;

        gckHARDWARE_SetClock(hardware, dumpCore, clkScale[0], clkScale[1]);
    }
    else
    {
        printk("Error: invalid core\n");
    }

    return 0;
}

static int gc_clk_write(const char __user *buf, size_t count, void* data)
{
    size_t ret;
    char _buf[100];

    count = min_t(size_t, count, (sizeof(_buf)-1));
    ret = copy_from_user(_buf, buf, count);
    if (ret != 0)
    {
        printk("Error: lost data: %d\n", (int)ret);
        return -EFAULT;
    }
    _buf[count] = 0;

    _set_clk(_buf);

    return count;
}

static gcsINFO InfoList[] =
{
    {"info", gc_info_show},
    {"clients", gc_clients_show},
    {"meminfo", gc_meminfo_show},
    {"idle", gc_idle_show},
    {"database", gc_db_show_old},
    {"database64x", gc_db_show},
    {"version", gc_version_show},
    {"vidmem", gc_vidmem_show_old, gc_vidmem_write},
    {"vidmem64x", gc_vidmem_show, gc_vidmem_write},
    {"dump_trigger", gc_dump_trigger_show, gc_dump_trigger_write},
    {"clk", gc_clk_show, gc_clk_write},
};

static gceSTATUS
_DebugfsInit(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckDEBUGFS_DIR dir = &Device->debugfsDir;

    gcmkONERROR(gckDEBUGFS_DIR_Init(dir, gcvNULL, "gc"));
    gcmkONERROR(gckDEBUGFS_DIR_CreateFiles(dir, InfoList, gcmCOUNTOF(InfoList), Device));

OnError:
    return status;
}

static void
_DebugfsCleanup(
    IN gckGALDEVICE Device
    )
{
    gckDEBUGFS_DIR dir = &Device->debugfsDir;

    if (Device->debugfsDir.root)
    {
        gcmkVERIFY_OK(gckDEBUGFS_DIR_RemoveFiles(dir, InfoList, gcmCOUNTOF(InfoList)));

        gckDEBUGFS_DIR_Deinit(dir);
    }
}


/******************************************************************************\
*************************** Memory Allocation Wrappers *************************
\******************************************************************************/

static gceSTATUS
_AllocateMemory(
    IN gckGALDEVICE Device,
    IN gctSIZE_T Bytes,
    OUT gctPOINTER *Logical,
    OUT gctPHYS_ADDR *Physical,
    OUT gctUINT64 *PhysAddr
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctPHYS_ADDR_T physAddr;

    gcmkHEADER_ARG("Device=%p Bytes=0x%zx", Device, Bytes);

    gcmkVERIFY_ARGUMENT(Device != NULL);
    gcmkVERIFY_ARGUMENT(Logical != NULL);
    gcmkVERIFY_ARGUMENT(Physical != NULL);
    gcmkVERIFY_ARGUMENT(PhysAddr != NULL);

    gcmkONERROR(gckOS_AllocateNonPagedMemory(
        Device->os, gcvFALSE, gcvALLOC_FLAG_CONTIGUOUS, &Bytes, Physical, Logical
        ));

    gcmkONERROR(gckOS_GetPhysicalFromHandle(
        Device->os, *Physical, 0, &physAddr
        ));

    *PhysAddr = physAddr;

OnError:
    gcmkFOOTER_ARG(
        "*Logical=%p *Physical=%p *PhysAddr=0x%llx",
        gcmOPT_POINTER(Logical), gcmOPT_POINTER(Physical), gcmOPT_VALUE(PhysAddr)
        );

    return status;
}

static gceSTATUS
_FreeMemory(
    IN gckGALDEVICE Device,
    IN gctPOINTER Logical,
    IN gctPHYS_ADDR Physical
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Device=%p Logical=%p Physical=%p",
                   Device, Logical, Physical);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    status = gckOS_FreeNonPagedMemory(
        Device->os, Physical, Logical,
        ((PLINUX_MDL) Physical)->numPages * PAGE_SIZE
        );

    gcmkFOOTER();
    return status;
}

static gceSTATUS
_SetupContiguousVidMem(
    IN gckGALDEVICE Device,
    IN const gcsMODULE_PARAMETERS * Args
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT64 physAddr = ~0ULL;
    gckGALDEVICE device = Device;

    gcmkHEADER_ARG("Device=%p Args=%p", Device, Args);

    /* set up the contiguous memory */
    device->contiguousBase = Args->contiguousBase;
    device->contiguousSize = Args->contiguousSize;

    if (Args->contiguousSize == 0)
    {
        gcmkFOOTER_NO();
        return gcvSTATUS_OK;
    }

    if (Args->contiguousBase == 0)
    {
        while (device->contiguousSize > 0)
        {
            /* Allocate contiguous memory. */
            status = _AllocateMemory(
                device,
                device->contiguousSize,
                &device->contiguousLogical,
                &device->contiguousPhysical,
                &physAddr
                );

            if (gcmIS_SUCCESS(status))
            {
                status = gckVIDMEM_Construct(
                    device->os,
                    physAddr,
                    device->contiguousSize,
                    64,
                    Args->bankSize,
                    &device->contiguousVidMem
                    );

                if (gcmIS_SUCCESS(status))
                {
                    gckALLOCATOR allocator = ((PLINUX_MDL)device->contiguousPhysical)->allocator;
                    device->contiguousVidMem->capability = allocator->capability | gcvALLOC_FLAG_MEMLIMIT;
                    device->contiguousVidMem->physical = device->contiguousPhysical;
                    device->contiguousBase = physAddr;
                    if (device->contiguousBase > 0xFFFFFFFFULL)
                    {
                        device->contiguousVidMem->capability &= ~gcvALLOC_FLAG_4GB_ADDR;
                    }
                    break;
                }

                gcmkONERROR(_FreeMemory(
                    device,
                    device->contiguousLogical,
                    device->contiguousPhysical
                    ));

                device->contiguousLogical  = gcvNULL;
                device->contiguousPhysical = gcvNULL;
            }

            if (device->contiguousSize <= (4 << 20))
            {
                device->contiguousSize = 0;
            }
            else
            {
                device->contiguousSize -= (4 << 20);
            }
        }
    }
    else
    {
        /* Create the contiguous memory heap. */
        status = gckVIDMEM_Construct(
            device->os,
            Args->contiguousBase,
            Args->contiguousSize,
            64,
            Args->bankSize,
            &device->contiguousVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable contiguous memory pool. */
            device->contiguousVidMem = gcvNULL;
            device->contiguousSize   = 0;
        }
        else
        {
            gckALLOCATOR allocator;
            gctBOOL contiguousRequested = Args->contiguousRequested;

#if gcdCAPTURE_ONLY_MODE
            contiguousRequested = gcvTRUE;
#endif

            gcmkONERROR(gckOS_RequestReservedMemory(
                device->os, Args->contiguousBase, Args->contiguousSize,
                "galcore contiguous memory",
                contiguousRequested,
                &device->contiguousPhysical
                ));

            allocator = ((PLINUX_MDL)device->contiguousPhysical)->allocator;

            device->contiguousVidMem->capability = allocator->capability | gcvALLOC_FLAG_MEMLIMIT;
            device->contiguousVidMem->physical = device->contiguousPhysical;
            device->requestedContiguousBase = Args->contiguousBase;
            device->requestedContiguousSize = Args->contiguousSize;

            device->contiguousPhysName = 0;
            device->contiguousSize = Args->contiguousSize;
        }
    }

    if (Args->showArgs)
    {
        gcmkPRINT("Galcore Info: ContiguousBase=0x%llx ContiguousSize=0x%zx\n", device->contiguousBase, device->contiguousSize);
    }

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_SetupExternalSRAMVidMem(
    IN gckGALDEVICE Device
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE device = Device;
    gctINT32 i, j = 0;

    gcmkHEADER_ARG("Device=%p", Device);

    /* Setup external SRAM memory region. */
    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        if (!device->extSRAMSizes[i])
        {
            /* Keep this path for internal test, read from feature database. */
            device->extSRAMSizes[i] = device->device->extSRAMSizes[i];
        }

        if (device->extSRAMSizes[i] > 0)
        {
            /* create the external SRAM memory heap */
            status = gckVIDMEM_Construct(
                device->os,
                device->extSRAMBases[i],
                device->extSRAMSizes[i],
                64,
                0,
                &device->extSRAMVidMem[i]
                );

            if (gcmIS_ERROR(status))
            {
                /* Error, disable external SRAM heap. */
                device->extSRAMSizes[i] = 0;
            }
            else
            {
                char sRAMName[40];
                snprintf(sRAMName, gcmSIZEOF(sRAMName) - 1, "Galcore external sram%d", i);

#if gcdCAPTURE_ONLY_MODE
                device->args.sRAMRequested = gcvTRUE;
#endif
                /* Map external SRAM memory. */
                gcmkONERROR(gckOS_RequestReservedMemory(
                        device->os,
                        device->extSRAMBases[i], device->extSRAMSizes[i],
                        sRAMName,
                        device->args.sRAMRequested,
                        &device->extSRAMPhysical[i]
                        ));

                device->extSRAMVidMem[i]->physical = device->extSRAMPhysical[i];
                device->device->extSRAMPhysical[i] = device->extSRAMPhysical[i];

                for (j = 0; j < gcdMAX_GPU_COUNT; j++)
                {
                    if (device->irqLines[j] != -1 && device->kernels[j])
                    {
                        device->kernels[j]->hardware->options.extSRAMGPUPhysNames[i] = gckKERNEL_AllocateNameFromPointer(device->kernels[j], device->extSRAMPhysical[i]);
                    }
                }
            }
        }
    }

OnError:
    gcmkFOOTER();
    return status;
}

/******************************************************************************\
******************************* Interrupt Handler ******************************
\******************************************************************************/
irqreturn_t threadRoutine(int irq, void *ctxt)
{
    gckGALDEVICE device = galDevice;
    gceCORE core = (gceCORE)gcmPTR2INT32(ctxt) - 1;

    gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_DRIVER,
                   "Starting isr Thread with extension=%p",
                   device);

    gckKERNEL_Notify(device->kernels[core], gcvNOTIFY_INTERRUPT);
    return IRQ_HANDLED;
}

irqreturn_t isrRoutine(int irq, void *ctxt)
{
    gceSTATUS status;
    gckGALDEVICE device;
    gceCORE core = (gceCORE)gcmPTR2INT32(ctxt) - 1;

    device = galDevice;

    /* Call kernel interrupt notification. */
    status = gckHARDWARE_Interrupt(device->kernels[core]->hardware);

    if (gcmIS_SUCCESS(status))
    {
        return IRQ_WAKE_THREAD;
    }

    return IRQ_NONE;
}

static irqreturn_t isrRoutineVG(int irq, void *ctxt)
{
#if gcdENABLE_VG
    gceSTATUS status;
    gckGALDEVICE device;

    device = galDevice;

    /* Serve the interrupt. */
    status = gckVGINTERRUPT_Enque(device->kernels[gcvCORE_VG]->vg->interrupt);

    /* Determine the return value. */
    return (status == gcvSTATUS_NOT_OUR_INTERRUPT)
        ? IRQ_RETVAL(0)
        : IRQ_RETVAL(1);
#else
    return IRQ_NONE;
#endif
}

static const char *isrNames[] =
{
    "galcore:0",
    "galcore:3d-1",
    "galcore:3d-2",
    "galcore:3d-3",
    "galcore:3d-4",
    "galcore:3d-5",
    "galcore:3d-6",
    "galcore:3d-7",
    "galcore:2d",
    "galcore:vg",
#if gcdDEC_ENABLE_AHB
    "galcore:dec"
#endif
};

static gceSTATUS
_SetupIsr(
    IN gceCORE Core
    )
{
    gctINT ret = 0;
    gceSTATUS status = gcvSTATUS_OK;
    gckGALDEVICE Device = galDevice;

    gcmkHEADER_ARG("Device=%p Core=%d", Device, Core);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    if (Device->irqLines[Core] < 0)
    {
        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    gcmSTATIC_ASSERT(gcvCORE_COUNT == gcmCOUNTOF(isrNames),
                     "isrNames array does not match core types");

    /*
     * Hook up the isr based on the irq line.
     * For shared irq, device-id can not be 0, but CORE_MAJOR value is.
     * Add by 1 here and subtract by 1 in isr to fix the issue.
     */
    if (gcvCORE_VG == Core) {
        ret = request_irq(
            Device->irqLines[Core], isrRoutineVG, gcdIRQF_FLAG,
            isrNames[Core], (void *)(uintptr_t)(Core + 1)
            );
    }
    else
    {
        ret = request_threaded_irq(
            Device->irqLines[Core], isrRoutine, threadRoutine, gcdIRQF_FLAG,
            isrNames[Core], (void *)(uintptr_t)(Core + 1)
            );
    }

    if (ret != 0)
    {
        gcmkTRACE_ZONE(
            gcvLEVEL_ERROR, gcvZONE_DRIVER,
            "%s(%d): Could not register irq line %d (error=%d)\n",
            __FUNCTION__, __LINE__,
            Device->irqLines[Core], ret
            );

        gcmkONERROR(gcvSTATUS_GENERIC_IO);
    }

    /* Mark ISR as initialized. */
    Device->isrInitializeds[Core] = gcvTRUE;

OnError:
    gcmkFOOTER();
    return status;
}

static gceSTATUS
_ReleaseIsr(
    IN gceCORE Core
    )
{
    gckGALDEVICE Device = galDevice;

    gcmkHEADER_ARG("Device=%p Core=%d", Device, Core);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    /* release the irq */
    if (Device->isrInitializeds[Core])
    {
        free_irq(Device->irqLines[Core], (void *)(uintptr_t)(Core + 1));
        Device->isrInitializeds[Core] = gcvFALSE;
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckGALDEVICE_Construct
**
**  Constructor.
**
**  INPUT:
**
**  OUTPUT:
**
**      gckGALDEVICE * Device
**          Pointer to a variable receiving the gckGALDEVICE object pointer on
**          success.
*/
gceSTATUS
gckGALDEVICE_Construct(
    IN gcsPLATFORM * Platform,
    IN const gcsMODULE_PARAMETERS * Args,
    OUT gckGALDEVICE *Device
    )
{
    gckKERNEL kernel = gcvNULL;
    gckGALDEVICE device;
    gctINT32 i;

#if !gcdCAPTURE_ONLY_MODE
    gceHARDWARE_TYPE type;
#endif

    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Platform=%p Args=%p", Platform, Args);

    /* Allocate device structure. */
    device = kmalloc(sizeof(struct _gckGALDEVICE), GFP_KERNEL | __GFP_NOWARN);

    if (!device)
    {
        gcmkONERROR(gcvSTATUS_OUT_OF_MEMORY);
    }

    memset(device, 0, sizeof(struct _gckGALDEVICE));

    device->platform = Platform;
    device->platform->dev = gcvNULL;

    device->args = *Args;

    /* Clear irq lines. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        device->irqLines[i] = -1;
#if USE_LINUX_PCIE
        device->bars[i] = -1;
#endif
    }

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        device->irqLines[i]                  = Args->irqs[i];
        device->requestedRegisterMemBases[i] = Args->registerBases[i];
        device->requestedRegisterMemSizes[i] = Args->registerSizes[i];
#if USE_LINUX_PCIE
        device->bars[i]                      = Args->bars[i];
#endif
        gcmkTRACE_ZONE(gcvLEVEL_INFO, _GC_OBJ_ZONE,
                       "Get register base %llx of core %d",
                       Args->registerBases[i], i);
    }

    device->requestedContiguousBase  = 0;
    device->requestedContiguousSize  = 0;

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        unsigned long physical;
        physical = (unsigned long)device->requestedRegisterMemBases[i];

        /* Set up register memory region. */
        if (physical != 0)
        {
            if (Args->registerBasesMapped[i])
            {
                device->registerBases[i] = Args->registerBasesMapped[i];
                device->requestedRegisterMemBases[i] = 0;
            }
            else
            {
#if USE_LINUX_PCIE
                gcmkPRINT("register should be mapped in platform layer");
#endif
                if (!request_mem_region(physical,
                        device->requestedRegisterMemSizes[i],
                        "galcore register region"))
                {
                    gcmkTRACE_ZONE(
                            gcvLEVEL_ERROR, gcvZONE_DRIVER,
                            "%s(%d): Failed to claim %lu bytes @ 0x%lx\n",
                            __FUNCTION__, __LINE__,
                            device->requestedRegisterMemSizes[i], physical
                            );

                    gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
                }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
                device->registerBases[i] = (gctPOINTER)ioremap(physical, device->requestedRegisterMemSizes[i]);
#else
                device->registerBases[i] = (gctPOINTER)ioremap_nocache(physical, device->requestedRegisterMemSizes[i]);
#endif

                if (device->registerBases[i] == gcvNULL)
                {
                    gcmkTRACE_ZONE(
                            gcvLEVEL_ERROR, gcvZONE_DRIVER,
                            "%s(%d): Unable to map %ld bytes @ 0x%zx\n",
                            __FUNCTION__, __LINE__,
                            physical, device->requestedRegisterMemSizes[i]
                            );

                    gcmkONERROR(gcvSTATUS_OUT_OF_RESOURCES);
                }
            }
        }
    }

    /* Set the base address */
    device->baseAddress = device->physBase = Args->baseAddress;
    device->physSize = Args->physSize;

    /* Set the external base address */
    device->externalBase = Args->externalBase;
    device->externalSize = Args->externalSize;

    for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
    {
        device->extSRAMBases[i] = Args->extSRAMBases[i];
        device->extSRAMSizes[i] = Args->extSRAMSizes[i];
    }

    /* Construct the gckOS object. */
    gcmkONERROR(gckOS_Construct(device, &device->os));


    if (device->externalSize > 0)
    {
        /* create the external memory heap */
        status = gckVIDMEM_Construct(
            device->os,
            device->externalBase,
            device->externalSize,
            64,
            0,
            &device->externalVidMem
            );

        if (gcmIS_ERROR(status))
        {
            /* Error, disable external heap. */
            device->externalSize = 0;
        }
        else
        {
            /* Map external memory. */
            gcmkONERROR(gckOS_RequestReservedMemory(
                    device->os,
                    device->externalBase, device->externalSize,
                    "galcore external memory",
                    gcvTRUE,
                    &device->externalPhysical
                    ));

            device->externalVidMem->physical = device->externalPhysical;
        }
    }

    /* Construct the gckDEVICE object for os independent core management. */
    gcmkONERROR(gckDEVICE_Construct(device->os, &device->device));

    device->device->showSRAMMapInfo = Args->showArgs;

    device->platform->dev = device->device;

    if (device->irqLines[gcvCORE_MAJOR] != -1)
    {
        gcmkONERROR(gctaOS_ConstructOS(device->os, &device->taos));
    }

    /* Setup contiguous video memory pool. */
    gcmkONERROR(_SetupContiguousVidMem(device, Args));

#if gcdEXTERNAL_SRAM_DEFAULT_POOL
    /* Setup external SRAM video memory pool. */
    gcmkONERROR(_SetupExternalSRAMVidMem(device));
#endif

    /* Add core for all available major cores. */
    for (i = gcvCORE_MAJOR; i <= gcvCORE_3D_MAX; i++)
    {
        if (device->irqLines[i] != -1)
        {
            gcmkONERROR(gcTA_Construct(
                device->taos,
                (gceCORE)i,
                &globalTA[i]
                ));

            gcmkONERROR(gckDEVICE_AddCore(
                device->device,
                (gceCORE)i,
                Args->chipIDs[i],
                device,
                &device->kernels[i]
                ));

            gcmkONERROR(gckHARDWARE_SetFastClear(
                device->kernels[i]->hardware,
                Args->fastClear,
                Args->compression
                ));

            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                device->kernels[i]->hardware,
                Args->powerManagement
                ));

#if gcdENABLE_FSCALE_VAL_ADJUST
            gcmkONERROR(gckHARDWARE_SetMinFscaleValue(
                device->kernels[i]->hardware,
                Args->gpu3DMinClock
                ));
#endif
        }
        else
        {
            device->kernels[i] = gcvNULL;
        }
    }

#if !gcdCAPTURE_ONLY_MODE
    if (device->irqLines[gcvCORE_2D] != -1)
    {
        gcmkONERROR(gckDEVICE_AddCore(
            device->device,
            gcvCORE_2D,
            gcvCHIP_ID_DEFAULT,
            device,
            &device->kernels[gcvCORE_2D]
            ));

        /* Verify the hardware type */
        gcmkONERROR(gckHARDWARE_GetType(
            device->kernels[gcvCORE_2D]->hardware,
            &type
            ));

        if (type != gcvHARDWARE_2D)
        {
            gcmkTRACE_ZONE(
                gcvLEVEL_ERROR, gcvZONE_DRIVER,
                "%s(%d): Unexpected hardware type: %d\n",
                __FUNCTION__, __LINE__,
                type
                );

            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        gcmkONERROR(gckHARDWARE_EnablePowerManagement(
            device->kernels[gcvCORE_2D]->hardware,
            Args->powerManagement
            ));

#if gcdENABLE_FSCALE_VAL_ADJUST
        gcmkONERROR(gckHARDWARE_SetMinFscaleValue(
            device->kernels[gcvCORE_2D]->hardware, 1
            ));
#endif
    }
    else
    {
        device->kernels[gcvCORE_2D] = gcvNULL;
    }

    if (device->irqLines[gcvCORE_VG] != -1)
    {
#if gcdENABLE_VG
        gcmkONERROR(gckDEVICE_AddCore(
            device->device,
            gcvCORE_VG,
            gcvCHIP_ID_DEFAULT,
            device,
            &device->kernels[gcvCORE_VG]
            ));

        gcmkONERROR(gckVGHARDWARE_EnablePowerManagement(
            device->kernels[gcvCORE_VG]->vg->hardware,
            Args->powerManagement
            ));
#endif
    }
    else
    {
        device->kernels[gcvCORE_VG] = gcvNULL;
    }
#else
    device->kernels[gcvCORE_2D] = gcvNULL;

    device->kernels[gcvCORE_VG] = gcvNULL;
#endif

#if !gcdEXTERNAL_SRAM_DEFAULT_POOL
    /* Setup external SRAM video memory pool. */
    gcmkONERROR(_SetupExternalSRAMVidMem(device));
#endif

    /* Create the suspend semaphore. */
    gcmkONERROR(gckOS_CreateSemaphore(device->os, &device->suspendSemaphore));

    /* Grab the first valid kernel. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (device->kernels[i] != gcvNULL)
        {
            kernel = device->kernels[i];
            break;
        }
    }

    if (!kernel)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (device->internalPhysical)
    {
        device->internalPhysName = gcmPTR_TO_NAME(device->internalPhysical);
    }

    if (device->externalPhysical)
    {
        device->externalPhysName = gcmPTR_TO_NAME(device->externalPhysical);
    }

    if (device->contiguousPhysical)
    {
        device->contiguousPhysName = gcmPTR_TO_NAME(device->contiguousPhysical);
    }

    gcmkONERROR(_DebugfsInit(device));

    /* Return pointer to the device. */
    *Device = galDevice = device;

OnError:
    if (gcmIS_ERROR(status))
    {
        /* Roll back. */
        gcmkVERIFY_OK(gckGALDEVICE_Destroy(device));
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Destroy
**
**  Class destructor.
**
**  INPUT:
**
**      Nothing.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Destroy(
    gckGALDEVICE Device)
{
    gctINT i, j = 0;
    gckKERNEL kernel = gcvNULL;

    gcmkHEADER_ARG("Device=%p", Device);

    if (Device != gcvNULL)
    {
        /* Grab the first available kernel */
        for (i = 0; i < gcdMAX_GPU_COUNT; i++)
        {
            if (Device->kernels[i])
            {
                kernel = Device->kernels[i];
                break;
            }
        }

        if (kernel)
        {
            if (Device->internalPhysName != 0)
            {
                gcmRELEASE_NAME(Device->internalPhysName);
                Device->internalPhysName = 0;
            }
            if (Device->externalPhysName != 0)
            {
                gcmRELEASE_NAME(Device->externalPhysName);
                Device->externalPhysName = 0;
            }
            if (Device->contiguousPhysName != 0)
            {
                gcmRELEASE_NAME(Device->contiguousPhysName);
                Device->contiguousPhysName = 0;
            }
        }

        /* Destroy per-core SRAM heap. */
        for (i = 0; i < gcvCORE_COUNT; i++)
        {
            if (Device->kernels[i])
            {
                kernel = Device->kernels[i];

                for (j = gcvSRAM_INTERNAL0; j < gcvSRAM_INTER_COUNT; j++)
                {
                    if (kernel->sRAMPhysical[j] != gcvNULL)
                    {
                        /* Release reserved SRAM memory. */
                        gckOS_ReleaseReservedMemory(
                            Device->os,
                            kernel->sRAMPhysical[j]
                            );

                        kernel->sRAMPhysical[j] = gcvNULL;
                    }

                    if (kernel->sRAMVidMem[j] != gcvNULL)
                    {
                        /* Destroy the SRAM contiguous heap. */
                        gcmkVERIFY_OK(gckVIDMEM_Destroy(kernel->sRAMVidMem[j]));
                        kernel->sRAMVidMem[j] = gcvNULL;
                    }
                }
            }
        }

        if (Device->device)
        {
            gcmkVERIFY_OK(gckDEVICE_Destroy(Device->os, Device->device));

            for (i = 0; i < gcdMAX_GPU_COUNT; i++)
            {
                if (globalTA[i])
                {
                    gcTA_Destroy(globalTA[i]);
                    globalTA[i] = gcvNULL;
                }
            }

            Device->device = gcvNULL;
        }

        for (i = 0; i < gcdMAX_GPU_COUNT; i++)
        {
            if (Device->kernels[i] != gcvNULL)
            {
                Device->kernels[i] = gcvNULL;
            }
        }

        if (Device->internalLogical != gcvNULL)
        {
            /* Unmap the internal memory. */
            iounmap(Device->internalLogical);
            Device->internalLogical = gcvNULL;
        }

        if (Device->internalVidMem != gcvNULL)
        {
            /* Destroy the internal heap. */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->internalVidMem));
            Device->internalVidMem = gcvNULL;
        }

        for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
        {
            if (Device->extSRAMPhysical[i] != gcvNULL)
            {
                gckOS_ReleaseReservedMemory(
                    Device->os,
                    Device->extSRAMPhysical[i]
                    );
                Device->extSRAMPhysical[i] = gcvNULL;
            }

            if (Device->extSRAMVidMem[i] != gcvNULL)
            {
                gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->extSRAMVidMem[i]));
                Device->extSRAMVidMem[i] = gcvNULL;
            }
        }

        if (Device->externalPhysical != gcvNULL)
        {
            gckOS_ReleaseReservedMemory(
                Device->os,
                Device->externalPhysical
                );
            Device->externalPhysical = gcvNULL;
        }

        if (Device->externalLogical != gcvNULL)
        {
            Device->externalLogical = gcvNULL;
        }

        if (Device->externalVidMem != gcvNULL)
        {
            /* destroy the external heap */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->externalVidMem));
            Device->externalVidMem = gcvNULL;
        }

        /*
         * Destroy contiguous memory pool after gckDEVICE destroyed. gckDEVICE
         * may allocates GPU memory types from SYSTEM pool.
         */
        if (Device->contiguousPhysical != gcvNULL)
        {
            if (Device->requestedContiguousBase == 0)
            {
                gcmkVERIFY_OK(_FreeMemory(
                    Device,
                    Device->contiguousLogical,
                    Device->contiguousPhysical
                    ));
            }
            else
            {
                gckOS_ReleaseReservedMemory(
                    Device->os,
                    Device->contiguousPhysical
                    );
                Device->contiguousPhysical = gcvNULL;
                Device->requestedContiguousBase = 0;
                Device->requestedContiguousSize = 0;
            }

            Device->contiguousLogical  = gcvNULL;
            Device->contiguousPhysical = gcvNULL;
        }

        if (Device->contiguousVidMem != gcvNULL)
        {
            /* Destroy the contiguous heap. */
            gcmkVERIFY_OK(gckVIDMEM_Destroy(Device->contiguousVidMem));
            Device->contiguousVidMem = gcvNULL;
        }

        for (i = 0; i < gcdMAX_GPU_COUNT; i++)
        {
            if (Device->registerBases[i])
            {
                /* Unmap register memory. */
                if (Device->requestedRegisterMemBases[i] != 0)
                {
                    iounmap(Device->registerBases[i]);

                    release_mem_region(Device->requestedRegisterMemBases[i],
                            Device->requestedRegisterMemSizes[i]);
                }

                Device->registerBases[i] = gcvNULL;
                Device->requestedRegisterMemBases[i] = 0;
                Device->requestedRegisterMemSizes[i] = 0;
            }
        }

        /* Destroy the suspend semaphore. */
        if (Device->suspendSemaphore)
        {
            gcmkVERIFY_OK(gckOS_DestroySemaphore(Device->os, Device->suspendSemaphore));
        }

        if (Device->taos)
        {
            gcmkVERIFY_OK(gctaOS_DestroyOS(Device->taos));
            Device->taos = gcvNULL;
        }

        /* Destroy the gckOS object. */
        if (Device->os != gcvNULL)
        {
            gcmkVERIFY_OK(gckOS_Destroy(Device->os));
            Device->os = gcvNULL;
        }

        _DebugfsCleanup(Device);

        /* Free the device. */
        kfree(Device);
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}

/*******************************************************************************
**
**  gckGALDEVICE_Start
**
**  Start the gal device, including the following actions: setup the isr routine
**  and start the daemoni thread.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Start successfully.
*/
gceSTATUS
gckGALDEVICE_Start(
    IN gckGALDEVICE Device
    )
{
    gctUINT i;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Device=%p", Device);

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

        /* Setup the ISR routine. */
        gcmkONERROR(_SetupIsr(i));

        if (i == gcvCORE_VG)
        {
#if gcdENABLE_VG
            /* Switch to SUSPEND power state. */
            gcmkONERROR(gckVGHARDWARE_SetPowerState(
                Device->kernels[gcvCORE_VG]->vg->hardware, gcvPOWER_OFF_BROADCAST
                ));
#endif
        }
        else
        {
            /* Switch to SUSPEND power state. */
            gcmkONERROR(gckHARDWARE_SetPowerState(
                Device->kernels[i]->hardware, gcvPOWER_OFF_BROADCAST
                ));

            gcmkONERROR(gckHARDWARE_StartTimerReset(Device->kernels[i]->hardware));
        }
    }

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Stop
**
**  Stop the gal device, including the following actions: stop the daemon
**  thread, release the irq.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      Nothing.
*/
gceSTATUS
gckGALDEVICE_Stop(
    gckGALDEVICE Device
    )
{
    gctUINT i;
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Device=%p", Device);

    gcmkVERIFY_ARGUMENT(Device != NULL);

    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

        if (Device->isrInitializeds[i] == gcvFALSE)
        {
            continue;
        }

        if (i == gcvCORE_VG)
        {
#if gcdENABLE_VG
            /* Switch to OFF power state. */
            gcmkONERROR(gckVGHARDWARE_SetPowerState(
                Device->kernels[i]->vg->hardware, gcvPOWER_OFF
                ));
#endif
        }
        else
        {
            gcmkONERROR(gckHARDWARE_EnablePowerManagement(
                Device->kernels[i]->hardware, gcvTRUE
                ));

            /* Switch to OFF power state. */
            gcmkONERROR(gckHARDWARE_SetPowerState(
                Device->kernels[i]->hardware, gcvPOWER_OFF
                ));

            gckHARDWARE_StartTimerReset(Device->kernels[i]->hardware);
        }

        synchronize_irq(Device->irqLines[i]);

        /* Stop the ISR routine. */
        gcmkONERROR(_ReleaseIsr(i));

    }

OnError:
    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Suspend
**
**  Suspend the gal device to specific state.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**      gceCHIPPOWERSTATE State
**          State to suspend.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Suspend successfully.
*/
gceSTATUS
gckGALDEVICE_Suspend(
    IN gckGALDEVICE Device,
    IN gceCHIPPOWERSTATE State
    )
{
    gctUINT i;
    gceSTATUS status = gcvSTATUS_OK;
    gckHARDWARE hardware;
#if gcdENABLE_VG
    gckVGHARDWARE vgHardware;
#endif
    gceCHIPPOWERSTATE currentState = gcvPOWER_INVALID;

    gcmkHEADER_ARG("Device=%p", Device);

    /* Acquire the suspend semaphore. */
    gcmkONERROR(gckOS_AcquireSemaphore(Device->os, Device->suspendSemaphore));
    Device->suspendSemaphoreAcquired = gcvTRUE;

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }
        synchronize_irq(Device->irqLines[i]);
        Device->statesStored[i] = gcvPOWER_INVALID;
    }

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            vgHardware = Device->kernels[i]->vg->hardware;
        }
        else
#endif
        {
            hardware = Device->kernels[i]->hardware;
        }

        /* Query state. */
#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            gcmkONERROR(gckVGHARDWARE_QueryPowerManagementState(vgHardware,
                    &currentState));
        }
        else
#endif
        {
            gcmkONERROR(gckHARDWARE_QueryPowerState(hardware, &currentState));
        }

        /* Store state. */
        Device->statesStored[i] = currentState;

#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            gcmkONERROR(gckVGHARDWARE_SetPowerState(vgHardware, State));
        }
        else
#endif
        {
            gcmkONERROR(gckHARDWARE_SetPowerState(hardware, State));
        }
    }

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    /* Roll back the state for touched cores. */
    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

        if (Device->statesStored[i] == gcvPOWER_INVALID)
        {
            continue;
        }

        /* Reset stored state. */
        Device->statesStored[i] = gcvPOWER_INVALID;
    }

    /* Release the suspend semaphore. */
    if (Device->suspendSemaphoreAcquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(Device->os,
                Device->suspendSemaphore));
        Device->suspendSemaphoreAcquired = gcvFALSE;
    }

    gcmkFOOTER();
    return status;
}

/*******************************************************************************
**
**  gckGALDEVICE_Resume
**
**  Resume the gal device.
**
**  INPUT:
**
**      gckGALDEVICE Device
**          Pointer to an gckGALDEVICE object.
**
**  OUTPUT:
**
**      Nothing.
**
**  RETURNS:
**
**      gcvSTATUS_OK
**          Resume successfully.
*/
gceSTATUS
gckGALDEVICE_Resume(
    IN gckGALDEVICE Device
    )
{
    gctUINT i;
    gceSTATUS status = gcvSTATUS_OK;
    gckHARDWARE hardware;
#if gcdENABLE_VG
    gckVGHARDWARE vgHardware;
#endif
    gceCHIPPOWERSTATE state;

    gcmkHEADER_ARG("Device=%p", Device);

    for (i = 0; i < gcdMAX_GPU_COUNT; i++)
    {
        if (Device->kernels[i] == gcvNULL)
        {
            continue;
        }

        if (Device->statesStored[i] == gcvPOWER_INVALID)
        {
            continue;
        }

#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            vgHardware = Device->kernels[i]->vg->hardware;
        }
        else
#endif
        {
            hardware = Device->kernels[i]->hardware;
        }

#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            gcmkONERROR(gckVGHARDWARE_SetPowerState(vgHardware, gcvPOWER_ON));
        }
        else
#endif
        {
            gcmkONERROR(gckHARDWARE_SetPowerState(hardware, gcvPOWER_ON));
        }

        /* Convert global state to corresponding internal state. */
        switch (Device->statesStored[i])
        {
        case gcvPOWER_ON:
            state = gcvPOWER_ON_AUTO;
            break;
        case gcvPOWER_IDLE:
            state = gcvPOWER_IDLE_BROADCAST;
            break;
        case gcvPOWER_SUSPEND:
            state = gcvPOWER_SUSPEND_BROADCAST;
            break;
        case gcvPOWER_OFF:
            state = gcvPOWER_OFF_BROADCAST;
            break;
        default:
            state = Device->statesStored[i];
            break;
        }

        /* Restore state. */
#if gcdENABLE_VG
        if (i == gcvCORE_VG)
        {
            vgHardware = Device->kernels[i]->vg->hardware;

            gcmkONERROR(gckVGHARDWARE_SetPowerState(vgHardware, state));
        }
        else
#endif
        {
            hardware = Device->kernels[i]->hardware;

            gcmkONERROR(gckHARDWARE_SetPowerState(hardware, state));
        }

        /* Reset stored state. */
        Device->statesStored[i] = gcvPOWER_INVALID;
    }

OnError:
    /* Release the suspend semaphore. */
    if (Device->suspendSemaphoreAcquired)
    {
        gcmkVERIFY_OK(gckOS_ReleaseSemaphore(Device->os,
                Device->suspendSemaphore));
        Device->suspendSemaphoreAcquired = gcvFALSE;
    }

    gcmkFOOTER();
    return status;
}
