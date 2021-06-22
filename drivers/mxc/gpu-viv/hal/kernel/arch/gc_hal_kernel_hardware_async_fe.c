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


#include "gc_hal.h"
#include "gc_hal_kernel.h"
#include "AQ.h"
#include "gc_hal_kernel_context.h"

#define _GC_OBJ_ZONE    gcvZONE_HARDWARE

struct _gckASYNC_FE
{
    /* Number of free descriptors. */
    gctPOINTER                  freeDscriptors;
};

gceSTATUS
gckASYNC_FE_Construct(
    IN gckHARDWARE Hardware,
    OUT gckASYNC_FE * FE
    )
{
    gceSTATUS status;
    gctUINT32 data;
    gckASYNC_FE fe;

    gcmkHEADER();

    gcmkONERROR(gckOS_Allocate(Hardware->os,
                               gcmSIZEOF(struct _gckASYNC_FE),
                               (gctPOINTER *)&fe));
    gckOS_ZeroMemory(fe, gcmSIZEOF(struct _gckASYNC_FE));

    gcmkVERIFY_OK(gckOS_ReadRegisterEx(
        Hardware->os,
        Hardware->core,
        GCREG_FE_ASYNC_STATUS_Address,
       &data
        ));

    gcmkONERROR(gckOS_AtomConstruct(Hardware->os, &fe->freeDscriptors));

    data = gcmGETFIELD(data, GCREG_FE_ASYNC_STATUS, FREE_DESCRIPTOR);

    gcmkTRACE_ZONE(gcvLEVEL_INFO, _GC_OBJ_ZONE, "free descriptor=%d", data);

    gcmkONERROR(gckOS_AtomSet(Hardware->os, fe->freeDscriptors, data));

    /* Enable interrupts. */
    gcmkONERROR(gckOS_WriteRegisterEx(Hardware->os, Hardware->core, AQ_INTR_ENBL_EX_Address, ~0U));

    *FE = fe;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    if (fe)
    {
        if (fe->freeDscriptors)
        {
            gckOS_AtomDestroy(Hardware->os, fe->freeDscriptors);
        }
        gcmkOS_SAFE_FREE(Hardware->os, fe);
    }

    gcmkFOOTER();
    return status;
}

void
gckASYNC_FE_Destroy(
    IN gckHARDWARE Hardware,
    IN gckASYNC_FE FE
    )
{
    if (FE->freeDscriptors)
    {
        gcmkOS_SAFE_FREE(Hardware->os, FE->freeDscriptors);
    }

    gcmkOS_SAFE_FREE(Hardware->os, FE);
}

gceSTATUS
gckASYNC_FE_Initialize(
    IN gckHARDWARE Hardware,
    IN gckASYNC_FE FE
    )
{
    return gcvSTATUS_OK;
}

gceSTATUS
gckASYNC_FE_Nop(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN OUT gctSIZE_T * Bytes
    )
{
    gctUINT32_PTR logical = (gctUINT32_PTR) Logical;
    gceSTATUS status;

    gcmkHEADER_ARG("Hardware=0x%x Logical=0x%x *Bytes=%lu",
                   Hardware, Logical, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT((Logical == gcvNULL) || (Bytes != gcvNULL));

    if (Logical != gcvNULL)
    {
        if (*Bytes < 8)
        {
            /* Command queue too small. */
            gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
        }

        /* Append NOP. */
        logical[0] = gcmSETFIELDVALUE(0, AQ_COMMAND_NOP_COMMAND, OPCODE, NOP);

        gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE, "0x%x: NOP", Logical);
    }

    if (Bytes != gcvNULL)
    {
        /* Return number of bytes required by the NOP command. */
        *Bytes = 8;
    }

    /* Success. */
    gcmkFOOTER_ARG("*Bytes=%lu", gcmOPT_VALUE(Bytes));
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
gckASYNC_FE_Event(
    IN gckHARDWARE Hardware,
    IN gctPOINTER Logical,
    IN gctUINT8 Event,
    IN gceKERNEL_WHERE FromWhere,
    IN OUT gctUINT32 * Bytes
    )
{
    gctUINT size;
    gctUINT32 destination = 0;
    gctUINT32_PTR logical = (gctUINT32_PTR) Logical;
    gceSTATUS status;
    gctBOOL blt;
    gctBOOL extraEventStates;
    gctBOOL multiCluster;

    gcmkHEADER_ARG("Hardware=0x%x Logical=0x%x Event=%u FromWhere=%d *Bytes=%lu",
                   Hardware, Logical, Event, FromWhere, gcmOPT_VALUE(Bytes));

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Hardware, gcvOBJ_HARDWARE);
    gcmkVERIFY_ARGUMENT((Logical == gcvNULL) || (Bytes != gcvNULL));
    gcmkVERIFY_ARGUMENT(Event < 32);

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_BLT_ENGINE))
    {
        /* Send all event from blt. */
        if (FromWhere == gcvKERNEL_PIXEL)
        {
            FromWhere = gcvKERNEL_BLT;
        }
    }

    blt = FromWhere == gcvKERNEL_BLT ? gcvTRUE : gcvFALSE;

    multiCluster = gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_MULTI_CLUSTER);

    /* Determine the size of the command. */

    extraEventStates = Hardware->extraEventStates && (FromWhere == gcvKERNEL_PIXEL);

    size = extraEventStates
         ? gcmALIGN(8 + (1 + 5) * 4, 8) /* EVENT + 5 STATES */
         : 8;

    if (blt)
    {
        size += 16;
        if (multiCluster)
            size += 8;
    }

    if (Logical != gcvNULL)
    {
        if (*Bytes < size)
        {
            /* Command queue too small. */
            gcmkONERROR(gcvSTATUS_BUFFER_TOO_SMALL);
        }

        switch (FromWhere)
        {
        case gcvKERNEL_COMMAND:
            /* From command processor. */
            destination = gcmSETFIELDVALUE(0, AQ_EVENT, FE_SRC, ENABLE);
            break;

        case gcvKERNEL_PIXEL:
            /* From pixel engine. */
            destination = gcmSETFIELDVALUE(0, AQ_EVENT, PE_SRC, ENABLE);
            break;

        case gcvKERNEL_BLT:
            destination = gcmSETFIELDVALUE(0, AQ_EVENT, BLT_SRC, ENABLE);
            break;

        default:
            gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        if (blt)
        {
            *logical++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

            *logical++
                = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

            if (multiCluster)
            {
                *logical++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltClusterControlRegAddrs);

                *logical++
                    = gcmSETFIELD(0, GCREG_BLT_CLUSTER_CONTROL, CLUSTER_ENABLE,
                                  Hardware->identity.clusterAvailMask & Hardware->options.userClusterMask);
            }
        }

        /* Append EVENT(Event, destination). */
        *logical++
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQEventRegAddrs)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);

        *logical++
            = gcmSETFIELD(destination, AQ_EVENT, EVENT_ID, Event);

        if (blt)
        {
            *logical++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

            *logical++
                = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
        }


        /* Make sure the event ID gets written out before GPU can access it. */
        gcmkONERROR(
            gckOS_MemoryBarrier(Hardware->os, logical + 1));

#if gcmIS_DEBUG(gcdDEBUG_TRACE)
        {
            gctPHYS_ADDR_T phys;
            gckOS_GetPhysicalAddress(Hardware->os, Logical, &phys);
            gckOS_CPUPhysicalToGPUPhysical(Hardware->os, phys, &phys);
            gcmkTRACE_ZONE(gcvLEVEL_INFO, gcvZONE_HARDWARE,
                           "0x%08x: EVENT %d", phys, Event);
        }
#endif

        /* Append the extra states. These are needed for the chips that do not
        ** support back-to-back events due to the async interface. The extra
        ** states add the necessary delay to ensure that event IDs do not
        ** collide. */
        if (extraEventStates)
        {
            *logical++ = gcmSETFIELDVALUE(0,
                                          AQ_COMMAND_LOAD_STATE_COMMAND,
                                          OPCODE,
                                          LOAD_STATE)
                       | gcmSETFIELD(0,
                                     AQ_COMMAND_LOAD_STATE_COMMAND,
                                     ADDRESS,
                                     AQMemoryFePageTableRegAddrs)
                       | gcmSETFIELD(0,
                                     AQ_COMMAND_LOAD_STATE_COMMAND,
                                     COUNT,
                                     5);
            *logical++ = 0;
            *logical++ = 0;
            *logical++ = 0;
            *logical++ = 0;
            *logical++ = 0;
        }

#if gcdINTERRUPT_STATISTIC
        if (Event < gcmCOUNTOF(Hardware->kernel->eventObj->queues))
        {
            gckOS_AtomSetMask(Hardware->pendingEvent, 1 << Event);
        }
#endif
    }

    if (Bytes != gcvNULL)
    {
        /* Return number of bytes required by the EVENT command. */
        *Bytes = size;
    }

    /* Success. */
    gcmkFOOTER_ARG("*Bytes=%lu", gcmOPT_VALUE(Bytes));
    return gcvSTATUS_OK;

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}


void
gckASYNC_FE_UpdateAvaiable(
    IN gckHARDWARE Hardware
    )
{
    gceSTATUS status;
    gctUINT32 data;
    gctINT32 oldValue;
    gckASYNC_FE fe = Hardware->asyncFE;

    status = gckOS_ReadRegisterEx(
        Hardware->os,
        Hardware->core,
        GCREG_FE_ASYNC_STATUS_Address,
        &data
        );

    if (gcmIS_SUCCESS(status))
    {
        data = gcmGETFIELD(data, GCREG_FE_ASYNC_STATUS, FREE_DESCRIPTOR);

        while (data--)
        {
            gckOS_AtomIncrement(Hardware->os, fe->freeDscriptors, &oldValue);
        }
    }
}

gceSTATUS
gckASYNC_FE_ReserveSlot(
    IN gckHARDWARE Hardware,
    OUT gctBOOL * Available
    )
{
    gctINT32 oldValue;
    gckASYNC_FE fe = Hardware->asyncFE;

    gckOS_AtomDecrement(Hardware->os, fe->freeDscriptors, &oldValue);

    if (oldValue > 0)
    {
        /* Get one slot. */
        *Available = gcvTRUE;
    }
    else
    {
        /* No available slot, restore decreased one.*/
        gckOS_AtomIncrement(Hardware->os, fe->freeDscriptors, &oldValue);
        *Available = gcvFALSE;
    }

    return gcvSTATUS_OK;
}

gceSTATUS
gckASYNC_FE_Execute(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Address,
    IN gctUINT32 Bytes
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Hardware=%p Address=%x Bytes=%lu", Hardware, Address, Bytes);

    gcmkONERROR(gckOS_WriteRegisterEx(
        Hardware->os,
        Hardware->core,
        GCREG_FE_DESCRIPTOR_START_ADDRESS_Address,
        Address
        ));

    gcmkONERROR(gckOS_MemoryBarrier(
        Hardware->os,
        gcvNULL
        ));

    gcmkONERROR(gckOS_WriteRegisterEx(
        Hardware->os,
        Hardware->core,
        GCREG_FE_DESCRIPTOR_END_ADDRESS_Address,
        Address + Bytes
        ));

OnError:
    gcmkFOOTER();
    return status;
}
