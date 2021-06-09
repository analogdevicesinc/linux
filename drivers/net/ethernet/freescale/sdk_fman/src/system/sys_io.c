/*
 * Copyright 2008-2012 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/io.h>

#include "std_ext.h"
#include "error_ext.h"
#include "string_ext.h"
#include "list_ext.h"
#include "sys_io_ext.h"


#define __ERR_MODULE__      MODULE_UNKNOWN


typedef struct {
    uint64_t    virtAddr;
    uint64_t    physAddr;
    uint32_t    size;
    t_List      node;
} t_IoMap;
#define IOMAP_OBJECT(ptr)  LIST_OBJECT(ptr, t_IoMap, node)

LIST(mapsList);


static void EnqueueIoMap(t_IoMap *p_IoMap)
{
    uint32_t   intFlags;

    intFlags = XX_DisableAllIntr();
    LIST_AddToTail(&p_IoMap->node, &mapsList);
    XX_RestoreAllIntr(intFlags);
}

static t_IoMap * FindIoMapByVirtAddr(uint64_t addr)
{
    t_IoMap     *p_IoMap;
    t_List      *p_Pos;

    LIST_FOR_EACH(p_Pos, &mapsList)
    {
        p_IoMap = IOMAP_OBJECT(p_Pos);
        if ((addr >= p_IoMap->virtAddr) && (addr < p_IoMap->virtAddr+p_IoMap->size))
            return p_IoMap;
    }

    return NULL;
}

static t_IoMap * FindIoMapByPhysAddr(uint64_t addr)
{
    t_IoMap     *p_IoMap;
    t_List      *p_Pos;

    LIST_FOR_EACH(p_Pos, &mapsList)
    {
        p_IoMap = IOMAP_OBJECT(p_Pos);
        if ((addr >= p_IoMap->physAddr) && (addr < p_IoMap->physAddr+p_IoMap->size))
            return p_IoMap;
    }

    return NULL;
}

t_Error SYS_RegisterIoMap (uint64_t virtAddr, uint64_t physAddr, uint32_t size)
{
    t_IoMap *p_IoMap;

    p_IoMap = (t_IoMap*)XX_Malloc(sizeof(t_IoMap));
    if (!p_IoMap)
        RETURN_ERROR(MINOR, E_NO_MEMORY, ("message handler object!!!"));
    memset(p_IoMap, 0, sizeof(t_IoMap));

    p_IoMap->virtAddr = virtAddr;
    p_IoMap->physAddr = physAddr;
    p_IoMap->size     = size;

    INIT_LIST(&p_IoMap->node);
    EnqueueIoMap(p_IoMap);

    return E_OK;
}

t_Error SYS_UnregisterIoMap  (uint64_t virtAddr)
{
    t_IoMap *p_IoMap = FindIoMapByVirtAddr(virtAddr);
    if (!p_IoMap)
        RETURN_ERROR(MINOR, E_NO_DEVICE, ("message handler not found in list!!!"));

    LIST_Del(&p_IoMap->node);
    XX_Free(p_IoMap);

    return E_OK;
}

uint64_t SYS_PhysToVirt(uint64_t addr)
{
    t_IoMap *p_IoMap = FindIoMapByPhysAddr(addr);
    if (p_IoMap)
    {
        /* This is optimization - put the latest in the list-head - like a cache */
        if (mapsList.p_Next != &p_IoMap->node)
        {
            uint32_t intFlags = XX_DisableAllIntr();
            LIST_DelAndInit(&p_IoMap->node);
            LIST_Add(&p_IoMap->node, &mapsList);
            XX_RestoreAllIntr(intFlags);
        }
        return (uint64_t)(addr - p_IoMap->physAddr + p_IoMap->virtAddr);
    }
    return PTR_TO_UINT(phys_to_virt((unsigned long)addr));
}

uint64_t SYS_VirtToPhys(uint64_t addr)
{
    t_IoMap *p_IoMap;

    if (addr == 0)
        return 0;

    p_IoMap = FindIoMapByVirtAddr(addr);
    if (p_IoMap)
        return (uint64_t)(addr - p_IoMap->virtAddr + p_IoMap->physAddr);
    return (uint64_t)virt_to_phys(UINT_TO_PTR(addr));
}
