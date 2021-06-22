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
#include "gc_hal_kernel_context.h"
#include "gc_hal_kernel_buffer.h"
#include "AQ.h"

/******************************************************************************\
******************************** Debugging Macro *******************************
\******************************************************************************/

/* Zone used for header/footer. */
#define _GC_OBJ_ZONE    gcvZONE_HARDWARE


/******************************************************************************\
************************** Context State Buffer Helpers ************************
\******************************************************************************/

#define _STATE(reg)                                                            \
    _State(                                                                    \
        Context, index,                                                        \
        reg ## _Address >> 2,                                                  \
        reg ## _ResetValue,                                                    \
        reg ## _Count,                                                         \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_COUNT(reg, count)                                               \
    _State(                                                                    \
        Context, index,                                                        \
        reg ## _Address >> 2,                                                  \
        reg ## _ResetValue,                                                    \
        count,                                                                 \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_COUNT_OFFSET(reg, offset, count)                                \
    _State(                                                                    \
        Context, index,                                                        \
        (reg ## _Address >> 2) + offset,                                       \
        reg ## _ResetValue,                                                    \
        count,                                                                 \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_MIRROR_COUNT(reg, mirror, count)                                \
    _StateMirror(                                                              \
        Context,                                                               \
        reg ## _Address >> 2,                                                  \
        count,                                                                 \
        mirror ## _Address >> 2                                                \
        )

#define _STATE_HINT(reg)                                                       \
    _State(                                                                    \
        Context, index,                                                        \
        reg ## _Address >> 2,                                                  \
        reg ## _ResetValue,                                                    \
        reg ## _Count,                                                         \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_HINT_BLOCK(reg, block, count)                                   \
    _State(                                                                    \
        Context, index,                                                        \
        (reg ## _Address >> 2) + (block << reg ## _BLK),                       \
        reg ## _ResetValue,                                                    \
        count,                                                                 \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_COUNT_OFFSET_HINT(reg, offset, count)                           \
    _State(                                                                    \
        Context, index,                                                        \
        (reg ## _Address >> 2) + offset,                                       \
        reg ## _ResetValue,                                                    \
        count,                                                                 \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_X(reg)                                                          \
    _State(                                                                    \
        Context, index,                                                        \
        reg ## _Address >> 2,                                                  \
        reg ## _ResetValue,                                                    \
        reg ## _Count,                                                         \
        gcvTRUE, gcvFALSE                                                      \
        )

#define _STATE_INIT_VALUE(reg, value)                                          \
    _State(                                                                    \
        Context, index,                                                        \
        reg ## _Address >> 2,                                                  \
        value,                                                                 \
        reg ## _Count,                                                         \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_INIT_VALUE_BLOCK(reg, value, block, count)                      \
    _State(                                                                    \
        Context, index,                                                        \
        (reg ## _Address >> 2) + (block << reg ## _BLK),                       \
        value,                                                                 \
        count,                                                                 \
        gcvFALSE, gcvFALSE                                                     \
        )


#define _CLOSE_RANGE()                                                         \
    _TerminateStateBlock(Context, index)

#define _ENABLE(reg, field)                                                    \
    do                                                                         \
    {                                                                          \
        if (gcmVERIFYFIELDVALUE(data, reg, MASK_ ## field, ENABLED))           \
        {                                                                      \
            enable |= gcmFIELDMASK(reg, field);                                \
        }                                                                      \
    }                                                                          \
    while (gcvFALSE)

#define _BLOCK_COUNT(reg)                                                      \
    ((reg ## _Count) >> (reg ## _BLK))


/******************************************************************************\
*********************** Support Functions and Definitions **********************
\******************************************************************************/

#define gcdSTATE_MASK \
    (gcmSETFIELDVALUE(0, AQ_COMMAND_NOP_COMMAND, OPCODE, NOP) | 0xC0FFEE)

#if gcdENABLE_3D
static gctUINT32
_TerminateStateBlock(
    IN gckCONTEXT Context,
    IN gctUINT32 Index
    )
{
    gctUINT32_PTR buffer;
    gctUINT32 align;

    /* Determine if we need alignment. */
    align = (Index & 1) ? 1 : 0;

    /* Address correct index. */
    buffer = (Context->buffer == gcvNULL)
        ? gcvNULL
        : Context->buffer->logical;

    /* Flush the current state block; make sure no pairing with the states
       to follow happens. */
    if (align && (buffer != gcvNULL))
    {
        buffer[Index] = 0xDEADDEAD;
    }

    /* Reset last address. */
    Context->lastAddress = ~0U;

    /* Return alignment requirement. */
    return align;
}
#endif


#if gcdENABLE_3D
static gctUINT32
_FlushPipe(
    IN gckCONTEXT Context,
    IN gctUINT32 Index,
    IN gcePIPE_SELECT Pipe
    )
{
    gctUINT32 flushSlots;
    gctBOOL txCacheFix;
    gctBOOL fcFlushStall;
    gctBOOL iCacheInvalidate;
    gctBOOL halti5;
    gctBOOL snapPages;
    gctBOOL hwTFB;
    gctBOOL blt;
    gctBOOL peTSFlush;
    gctBOOL multiCluster;

    txCacheFix
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_TEX_CACHE_FLUSH_FIX);

    fcFlushStall
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_FC_FLUSH_STALL);

    iCacheInvalidate
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SHADER_HAS_INSTRUCTION_CACHE);

    halti5
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI5);

    snapPages
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SNAPPAGE_CMD_FIX) &&
          gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SNAPPAGE_CMD);

    hwTFB
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HW_TFB);

    blt
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_BLT_ENGINE);
    multiCluster
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_MULTI_CLUSTER);

    peTSFlush
        = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_PE_TILE_CACHE_FLUSH_FIX);

    flushSlots = blt ? 10 : 6;

    if (Pipe == gcvPIPE_3D)
    {
        if (!txCacheFix)
        {
            /* Semaphore stall */
            flushSlots += blt ? 8 : 4;
        }

        /* VST cache */
        flushSlots += 2;
    }

    if (fcFlushStall)
    {
        /* Flush tile status cache. */
        flushSlots += blt ? ((!peTSFlush) ? 14 :10) : 6;
    }

    if (iCacheInvalidate && !halti5)
    {
        flushSlots += blt ? 16 : 12;
    }

    if (hwTFB)
    {
        flushSlots += 2;
    }

    /* Snap pages */
    if (snapPages)
    {
        flushSlots += 2;
    }

    if (Context->buffer != gcvNULL)
    {
        gctUINT32_PTR buffer;

        /* Address correct index. */
        buffer = Context->buffer->logical + Index;

        if (Pipe == gcvPIPE_3D && !txCacheFix)
        {
            if (blt)
            {
                /* Semaphore from FE to BLT. */
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, BLT_ENGINE);

                /* Stall from FE to BLT. */
                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, BLT_ENGINE);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
            }
            else
            {
                /* Semaphore from FE to PE. */
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

                /* Stall from FE to PE. */
                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);
            }
        }

        /* Flush the current pipe. */
        *buffer++
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
            | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
            | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs);

        *buffer++
            = (Pipe == gcvPIPE_2D)
                ?   gcmSETFIELDVALUE(0, AQ_FLUSH, PE2D_CACHE, ENABLE)
                :   gcmSETFIELDVALUE(0, AQ_FLUSH,     ZCACHE, ENABLE)
                  | gcmSETFIELDVALUE(0, AQ_FLUSH,     CCACHE, ENABLE)
                  | (multiCluster ? 0 : gcmSETFIELDVALUE(0, AQ_FLUSH, TCACHE, ENABLE))
                  | gcmSETFIELDVALUE(0, AQ_FLUSH,     SHL1_CACHE, ENABLE)
                  | gcmSETFIELDVALUE(0, AQ_FLUSH,     VSSHL1_CACHE, ENABLE)
                  | gcmSETFIELDVALUE(0, AQ_FLUSH,     PSSHL1_CACHE, ENABLE);

        if (hwTFB)
        {
             *buffer++
                 = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                 | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTFBInvalidateRegAddrs)
                 | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);

             *buffer++
                 = GCREG_TFB_INVALIDATE_INVALIDATE_TRUE;
        }

        /* Flush VST in separate cmd. */
        if (Pipe == gcvPIPE_3D)
        {
            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs);

            *buffer++ = gcmSETFIELDVALUE(0, AQ_FLUSH, VST_CACHE, ENABLE);
        }

        /* Semaphore from FE to PE. */
        if (blt)
        {
            /* Semaphore from FE to BLT. */
            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, BLT_ENGINE);

            /* Stall from FE to BLT. */
            *buffer++
                = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

            *buffer++
                = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, BLT_ENGINE);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
        }
        else
        {
            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

            /* Stall from FE to PE. */
            *buffer++
                = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

            *buffer++
                = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);
        }

        if (fcFlushStall)
        {
            if (!peTSFlush && blt)
            {
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                    | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltCacheFlushRegAddrs)
                    | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_CACHE_FLUSH, TILE_CACHE, ENABLE);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
            }
            else
            {
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                    | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcTileCacheFlushRegAddrs)
                    | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);

                *buffer++
                    = gcmSETFIELDVALUE(0, GC_TILE_CACHE_FLUSH, FLUSH, ENABLE);
            }

            /* Semaphore from FE to PE. */
            if (blt)
            {
                /* Semaphore from FE to BLT. */
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, BLT_ENGINE);

                /* Stall from FE to BLT. */
                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, BLT_ENGINE);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
            }
            else
            {
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

                /* Stall from FE to PE. */
                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);
            }
        }

        if (iCacheInvalidate && !halti5)
        {
            /* Invalidate I$ after pipe is stalled */
            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregShaderConfigRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0,  GCREG_SHADER_CONFIG, INST_DECODE_SELECT, VS);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregSHCacheControlRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, GCREG_SH_CACHE_CONTROL, MODE, MEMORY)
                | gcmSETFIELD(0, GCREG_SH_CACHE_CONTROL, INVALIDATE_VS, 1);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregShaderConfigRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0,  GCREG_SHADER_CONFIG, INST_DECODE_SELECT, PS);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregSHCacheControlRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, GCREG_SH_CACHE_CONTROL, MODE, MEMORY)
                | gcmSETFIELD(0, GCREG_SH_CACHE_CONTROL, INVALIDATE_PS, 1);

            /* Semaphore from FE to PE. */
            if (blt)
            {
                /* Semaphore from FE to BLT. */
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, BLT_ENGINE);

                /* Stall from FE to BLT. */
                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, BLT_ENGINE);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
            }
            else
            {
                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

                *buffer++
                    = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

                /* Stall from FE to PE. */
                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

                *buffer++
                    = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                    | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);
            }
        }

        if (snapPages)
        {
            *buffer++
                = gcmSETFIELDVALUE(0, GCCMD_SNAP_TO_PAGE_COMMAND, OPCODE, SNAP_TO_PAGE)
                | gcmSETFIELDVALUE(0, GCCMD_SNAP_TO_PAGE_COMMAND, CLIENT, VS)
                | gcmSETFIELDVALUE(0, GCCMD_SNAP_TO_PAGE_COMMAND, CLIENT, TCS)
                | gcmSETFIELDVALUE(0, GCCMD_SNAP_TO_PAGE_COMMAND, CLIENT, TES)
                | gcmSETFIELDVALUE(0, GCCMD_SNAP_TO_PAGE_COMMAND, CLIENT, GS)
                | gcmSETFIELDVALUE(0, GCCMD_SNAP_TO_PAGE_COMMAND, CLIENT, PS);

            *buffer++
                = 0;
        }
    }

    /* Number of slots taken by flushing pipe. */
    return flushSlots;
}
#endif

#if gcdENABLE_3D
static gctUINT32
_SemaphoreStall(
    IN gckCONTEXT Context,
    IN gctUINT32 Index
    )
{
    gctBOOL blt = gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_BLT_ENGINE);
    if (Context->buffer != gcvNULL)
    {
        gctUINT32_PTR buffer;

        /* Address correct index. */
        buffer = Context->buffer->logical + Index;

        if (blt)
        {
            /* Semaphore from FE to BLT. */
            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, LOCK);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, BLT_ENGINE);

            /* Stall from FE to BLT. */
            *buffer++
                = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

            *buffer++
                = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, BLT_ENGINE);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregBltGeneralControlRegAddrs);

            *buffer
                = gcmSETFIELDVALUE(0, GCREG_BLT_GENERAL_CONTROL, STREAM_CONTROL, UNLOCK);
        }
        else
        {
            /* Semaphore from FE to PE. */
            *buffer++
                = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs);

            *buffer++
                = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

            /* Stall from FE to PE. */
            *buffer++
                = gcmSETFIELDVALUE(0, STALL_COMMAND, OPCODE, STALL);

            *buffer
                = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE,      FRONT_END)
                | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);
        }
    }

    /* Semaphore/stall takes 4 slots. */
    return (blt ? 8 : 4);
}
#endif

#if (gcdENABLE_3D || gcdENABLE_2D)
static gctUINT32
_SwitchPipe(
    IN gckCONTEXT Context,
    IN gctUINT32 Index,
    IN gcePIPE_SELECT Pipe
    )
{
    gctUINT32 slots = 2;

    if (Context->buffer != gcvNULL)
    {
        gctUINT32_PTR buffer;

        /* Address correct index. */
        buffer = Context->buffer->logical + Index;

        /* LoadState(AQPipeSelect, 1), pipe. */
        *buffer++
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
            | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQPipeSelectRegAddrs)
            | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1);

        *buffer
            = (Pipe == gcvPIPE_2D)
                ? AQ_PIPE_SELECT_PIPE_PIPE2D
                : AQ_PIPE_SELECT_PIPE_PIPE3D;
    }

    Context->pipeSelectBytes = slots * gcmSIZEOF(gctUINT32);

    return slots;
}
#endif

#if gcdENABLE_3D
static gctUINT32
_State(
    IN gckCONTEXT Context,
    IN gctUINT32 Index,
    IN gctUINT32 Address,
    IN gctUINT32 Value,
    IN gctUINT32 Size,
    IN gctBOOL FixedPoint,
    IN gctBOOL Hinted
    )
{
    gctUINT32_PTR buffer;
    gctUINT32 align;
    gctUINT32 i;

    /* Determine if we need alignment. */
    align = (Index & 1) ? 1 : 0;

    /* Address correct index. */
    buffer = (Context->buffer == gcvNULL)
        ? gcvNULL
        : Context->buffer->logical;

    if ((buffer == gcvNULL) && (Address + Size > Context->maxState))
    {
        /* Determine maximum state. */
        Context->maxState = Address + Size;
    }

    if (buffer == gcvNULL)
    {
        /* Update number of states. */
        Context->numStates += Size;
    }

    /* Do we need a new entry? */
    if ((Address != Context->lastAddress) || (FixedPoint != Context->lastFixed))
    {
        if (buffer != gcvNULL)
        {
            if (align)
            {
                /* Add filler. */
                buffer[Index++] = 0xDEADDEAD;
            }

            /* LoadState(Address, Count). */
            gcmkASSERT((Index & 1) == 0);

            if (FixedPoint)
            {
                buffer[Index]
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE,  LOAD_STATE)
                    | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT,   FIXED16_DOT16)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   Size)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, Address);
            }
            else
            {
                buffer[Index]
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE , LOAD_STATE)
                    | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT,   NORMAL)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   Size)
                    | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, Address);
            }

            /* Walk all the states. */
            for (i = 0; i < (gctUINT32)Size; i += 1)
            {
                /* Set state to uninitialized value. */
                buffer[Index + 1 + i] = Value;

                /* Set index in state mapping table. */
                Context->map[Address + i].index = (gctUINT)Index + 1 + i;
            }
        }

        /* Save information for this LoadState. */
        Context->lastIndex   = (gctUINT)Index;
        Context->lastAddress = Address + (gctUINT32)Size;
        Context->lastSize    = Size;
        Context->lastFixed   = FixedPoint;

        /* Return size for load state. */
        return align + 1 + Size;
    }

    /* Append this state to the previous one. */
    if (buffer != gcvNULL)
    {
        /* Update last load state. */
        buffer[Context->lastIndex] =
            gcmSETFIELD(buffer[Context->lastIndex],
                        AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,
                        Context->lastSize + Size);

        /* Walk all the states. */
        for (i = 0; i < (gctUINT32)Size; i += 1)
        {
            /* Set state to uninitialized value. */
            buffer[Index + i] = Value;

            /* Set index in state mapping table. */
            Context->map[Address + i].index = (gctUINT)Index + i;
        }
    }

    /* Update last address and size. */
    Context->lastAddress += (gctUINT32)Size;
    Context->lastSize    += Size;

    /* Return number of slots required. */
    return Size;
}

static gctUINT32
_StateMirror(
    IN gckCONTEXT Context,
    IN gctUINT32 Address,
    IN gctUINT32 Size,
    IN gctUINT32 AddressMirror
    )
{
    gctUINT32 i;

    /* Process when buffer is set. */
    if (Context->buffer != gcvNULL)
    {
        /* Walk all states. */
        for (i = 0; i < Size; i++)
        {
            /* Copy the mapping address. */
            Context->map[Address + i].index =
                Context->map[AddressMirror + i].index;
        }
    }

    /* Return the number of required maps. */
    return Size;
}

static void
_UpdateUnifiedReg(
    IN gckCONTEXT Context,
    IN gctUINT32 Address,
    IN gctUINT32 Size,
    IN gctUINT32 Count
    )
{
    gctUINT base;
    gctUINT nopCount;
    gctUINT32_PTR nop;
    gcsCONTEXT_PTR buffer;
    gcsSTATE_MAP_PTR map;
    gctUINT i;

    /* Get the current context buffer. */
    buffer = Context->buffer;

    /* Get the state map. */
    map = Context->map;

    base = map[Address].index;

    if (Count > 1024)
    {
        buffer->logical[base - 1]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
            | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT, NORMAL)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1024)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, Address);

        buffer->logical[base + 1024 + 1]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
            | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT, NORMAL)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, Count - 1024)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, Address + 1024);

        /* Determine the number of NOP commands. */
        nopCount = (Size / 2) - (Count / 2);
        /* Determine the location of the first NOP. */
        nop = &buffer->logical[base + (Count | 1) + 2];

        /* Fill the unused space with NOPs. */
        for (i = 0; i < nopCount; i += 1)
        {
            if (nop >= buffer->logical + Context->totalSize)
            {
                break;
            }

            /* Generate a NOP command. */
            *nop = gcmSETFIELDVALUE(0, AQ_COMMAND_NOP_COMMAND, OPCODE, NOP);

            /* Advance. */
            nop += 2;
        }
    }
    else
    {
        buffer->logical[base - 1]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
            | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT, NORMAL)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, Count)
            | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, Address);

        /* Determine the number of NOP commands. */
        nopCount = (Size / 2) - (Count / 2) + Size / 1024;

        /* Determine the location of the first NOP. */
        nop = &buffer->logical[base + (Count | 1)];

        /* Fill the unused space with NOPs. */
        for (i = 0; i < nopCount; i += 1)
        {
            if (nop >= buffer->logical + Context->totalSize)
            {
                break;
            }

            /* Generate a NOP command. */
            *nop = gcmSETFIELDVALUE(0, AQ_COMMAND_NOP_COMMAND, OPCODE, NOP);

            /* Advance. */
            nop += 2;
        }
    }
}
#endif

#if (gcdENABLE_3D || gcdENABLE_2D)

static gceSTATUS
_InitializeNoShaderAndPixelEngine(
    IN gckCONTEXT Context
    )
{
    gctUINT32_PTR buffer;
    gctUINT32 index;

#if gcdENABLE_3D
    gctBOOL halti5;
    gctBOOL hasSecurity;
    gctBOOL hasRobustness;
    gctBOOL multiCluster;
    gctUINT clusterAliveMask;
#endif

    gckHARDWARE hardware;

    gcmkHEADER();

    hardware = Context->hardware;

    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    /* Reset the buffer index. */
    index = 0;

    /* Reset the last state address. */
    Context->lastAddress = ~0U;

    /* Get the buffer pointer. */
    buffer = (Context->buffer == gcvNULL)
        ? gcvNULL
        : Context->buffer->logical;


    /**************************************************************************/
    /* Build 2D states. *******************************************************/


#if gcdENABLE_3D
    /**************************************************************************/
    /* Build 3D states. *******************************************************/

    halti5 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI5);
    hasSecurity = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SECURITY);
    hasRobustness = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_ROBUSTNESS);
    multiCluster = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MULTI_CLUSTER);
    clusterAliveMask = hardware->identity.clusterAvailMask & hardware->options.userClusterMask;

    /* Store the 3D entry index. */
    Context->entryOffset3D = (gctUINT)index * gcmSIZEOF(gctUINT32);

    /* Switch to 3D pipe. */
    index += _SwitchPipe(Context, index, gcvPIPE_3D);

    if (multiCluster)
    {
        index += _STATE_INIT_VALUE_BLOCK(GCREG_CLUSTER_CONTROL,
                                         gcmSETFIELD(0, GCREG_CLUSTER_CONTROL,
                                                     CLUSTER_ALIVE_ENABLE, clusterAliveMask),
                                         0, 4);
        index += _STATE_INIT_VALUE(GCREG_CLUSTER_CHECKER_BOARD,
                                   gcmSETFIELDVALUE(0, GCREG_CLUSTER_CHECKER_BOARD,
                                                    SIZE, BLOCK32X32));
    }

    /* Current context pointer. */
#if gcdDEBUG && defined(gcregContextRegAddrs)
    index += _STATE(GCREG_CONTEXT);
#endif

    index += _FlushPipe(Context, index, gcvPIPE_3D);

    /* Global states. */
    if (hasSecurity)
    {
        index += _STATE(GCREG_APP_CONTEXT_ID_LOW);
        index += _CLOSE_RANGE();
        index += _STATE(GCREG_APP_CONTEXT_ID_HIGH);
    }

    if (halti5)
    {
        gctUINT32 uscControl = gcmSETFIELD(0, GCREG_USC_CONTROL, MEMORY_CONTROLLER_SELECT, 2);
        index += _STATE(GCREG_SEMANTIC_LOCATION);
        index += _STATE(GCREG_SEMANTICS);

        uscControl |= gcmSETFIELD(0, GCREG_USC_CONTROL, CACHE_PAGES, hardware->options.uscL1CacheRatio);
        if (multiCluster)
        {
            uscControl |= gcmSETFIELD(0, GCREG_USC_CONTROL, ATTRIB_CACHE_PAGES, hardware->options.uscAttribCacheRatio);
        }
        index += _STATE_INIT_VALUE(GCREG_USC_CONTROL, uscControl);
    }
    else
    {
        index += _STATE(GCREG_VARYING_PACKING);
        index += _STATE(GCREG_COMPONENT_TYPE_LOW);
        index += _STATE(GCREG_COMPONENT_TYPE_HIGH);
        index += _STATE(GCREG_VARYING_PACKING_EX);
        index += _STATE(GCREG_COMPONENT_TYPE_EX);
        index += _STATE(GCREG_COMPONENT_TYPE_EX2);
    }

    index += _CLOSE_RANGE();

    /* Memory Controller */
    index += _STATE(GC_MEMORY_CONFIG);

    if (hasSecurity || hasRobustness)
    {
        index += _STATE_INIT_VALUE(GCREG_MMU_CONFIG,
                    gcmSETFIELDVALUE(0, GCREG_MMU_CONFIG, MASK_PAGE_TABLE_ID, MASKED));
    }

    /* Semaphore/stall. */
    index += _SemaphoreStall(Context, index);
#endif

    /**************************************************************************/
    /* Link to another address. ***********************************************/

    Context->linkIndex3D = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LINK_COMMAND, OPCODE,   LINK)
            | gcmSETFIELD     (0, AQ_COMMAND_LINK_COMMAND, PREFETCH, 0);

        buffer[index + 1]
            = 0;
    }

    index += 2;

    /* Store the end of the context buffer. */
    Context->bufferSize = index * gcmSIZEOF(gctUINT32);


    /**************************************************************************/
    /* Pipe switch for the case where neither 2D nor 3D are used. *************/

    /* Store the 3D entry index. */
    Context->entryOffsetXDFrom2D = (gctUINT)index * gcmSIZEOF(gctUINT32);

    /* Switch to 3D pipe. */
    index += _SwitchPipe(Context, index, gcvPIPE_3D);

    /* Store the location of the link. */
    Context->linkIndexXD = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LINK_COMMAND, OPCODE,   LINK)
            | gcmSETFIELD     (0, AQ_COMMAND_LINK_COMMAND, PREFETCH, 0);

        buffer[index + 1]
            = 0;
    }

    index += 2;


    /**************************************************************************/
    /* Save size for buffer. **************************************************/

    Context->totalSize = index * gcmSIZEOF(gctUINT32);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

}

static gceSTATUS
_InitializeContextBuffer(
    IN gckCONTEXT Context
    )
{
    gctUINT32_PTR buffer = gcvNULL;
    gctUINT32 index;

#if gcdENABLE_3D
    gctBOOL halti0, halti1, halti2, halti3, halti4, halti5;
    gctUINT i;
    gctUINT vertexUniforms, fragmentUniforms;
    gctBOOL unifiedUniform;
    gctBOOL hasGS, hasTS;
    gctBOOL genericAttrib;
    gctBOOL hasICache;
    gctBOOL hasICachePrefetch;
    gctUINT numRT = 0;
    gctUINT numSamplers = 32;
    gctBOOL hasTXdesc;
    gctBOOL hasSecurity;
    gctBOOL hasRobustness;
    gctBOOL multiCluster;
    gctBOOL smallBatch;
    gctBOOL multiCoreBlockSetCfg2;
    gctUINT clusterAliveMask;
    gctBOOL hasPSCSThrottle;
    gctBOOL hasMsaaFragOperation;
    gctBOOL newGPipe;
#endif

    gckHARDWARE hardware;

    gcmkHEADER();

    hardware = Context->hardware;

    gcmkVERIFY_OBJECT(hardware, gcvOBJ_HARDWARE);

    if (!hardware->options.hasShader)
    {
        return _InitializeNoShaderAndPixelEngine(Context);
    }

    /* Reset the buffer index. */
    index = 0;

    /* Reset the last state address. */
    Context->lastAddress = ~0U;

    /* Get the buffer pointer. */
    buffer = (Context->buffer == gcvNULL)
        ? gcvNULL
        : Context->buffer->logical;


    /**************************************************************************/
    /* Build 2D states. *******************************************************/


#if gcdENABLE_3D
    /**************************************************************************/
    /* Build 3D states. *******************************************************/

    halti0 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI0);
    halti1 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI1);
    halti2 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI2);
    halti3 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI3);
    halti4 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI4);
    halti5 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALTI5);
    hasGS  = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_GEOMETRY_SHADER);
    hasTS  = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TESSELLATION);
    genericAttrib = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_GENERIC_ATTRIB);
    hasICache = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SHADER_HAS_INSTRUCTION_CACHE);
    hasTXdesc = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_DESCRIPTOR);
    hasSecurity = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SECURITY);
    hasRobustness = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_ROBUSTNESS);
    hasICachePrefetch = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SH_INSTRUCTION_PREFETCH);
    multiCluster = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MULTI_CLUSTER);
    smallBatch = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SMALL_BATCH) && hardware->options.smallBatch;
    multiCoreBlockSetCfg2 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MULTI_CORE_BLOCK_SET_CONFIG2);
    clusterAliveMask = hardware->identity.clusterAvailMask & hardware->options.userClusterMask;
    hasPSCSThrottle = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_PSCS_THROTTLE);
    hasMsaaFragOperation = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MSAA_FRAGMENT_OPERATION);
    newGPipe = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_NEW_GPIPE);

    /* Multi render target. */
    if (Context->hardware->identity.chipModel == gcv880 &&
        Context->hardware->identity.chipRevision == 0x5124 &&
        Context->hardware->identity.customerID == 0x103)
    {
        numRT = 16;
    }
    else if (halti2 ||
            ((Context->hardware->identity.chipModel == gcv900) &&
            (Context->hardware->identity.chipRevision == 0x5250)))
    {
        numRT = 8;
    }
    else if (halti0)
    {
        numRT = 4;
    }
    else
    {
        numRT = 1;
    }

    if (hasGS && hasTS)
    {
        numSamplers = 80;
    }

    /* Query how many uniforms can support. */
    gcmCONFIGUREUNIFORMS2(Context->hardware->identity.chipModel,
                         Context->hardware->identity.chipRevision,
                         halti5,
                         smallBatch,
                         Context->hardware->identity.numConstants,
                         unifiedUniform,
                         vertexUniforms,
                         fragmentUniforms);

#if !gcdENABLE_UNIFIED_CONSTANT
    if (Context->hardware->identity.numConstants > 256)
    {
        unifiedUniform = gcvTRUE;
    }
    else
    {
        unifiedUniform = gcvFALSE;
    }
#endif

    /* Store the 3D entry index. */
    Context->entryOffset3D = (gctUINT)index * gcmSIZEOF(gctUINT32);

    /* Switch to 3D pipe. */
    index += _SwitchPipe(Context, index, gcvPIPE_3D);

    if (multiCluster)
    {
        index += _STATE_INIT_VALUE_BLOCK(GCREG_CLUSTER_CONTROL,
                                         gcmSETFIELD(0, GCREG_CLUSTER_CONTROL,
                                                     CLUSTER_ALIVE_ENABLE, clusterAliveMask),
                                         0, 4);
        index += _STATE_INIT_VALUE(GCREG_CLUSTER_CHECKER_BOARD,
                                   gcmSETFIELDVALUE(0, GCREG_CLUSTER_CHECKER_BOARD,
                                                    SIZE, BLOCK32X32));
    }

    /* Current context pointer. */
#if gcdDEBUG && defined(gcregContextRegAddrs)
    index += _STATE(GCREG_CONTEXT);
#endif

    index += _FlushPipe(Context, index, gcvPIPE_3D);

    /* Global states. */
    if (hasSecurity)
    {
        index += _STATE(GCREG_APP_CONTEXT_ID_LOW);
        index += _CLOSE_RANGE();
        index += _STATE(GCREG_APP_CONTEXT_ID_HIGH);
    }

    index += _STATE(AQ_VERTEX_ELEMENT_CONFIG);
    index += _CLOSE_RANGE();
    index += _STATE(GCREG_MULTI_SAMPLE_CONFIG);
    index += _STATE(GCREG_VARYINGS);

    if (halti5)
    {
        gctUINT32 uscControl = gcmSETFIELD(0, GCREG_USC_CONTROL, MEMORY_CONTROLLER_SELECT, 2);
        index += _STATE(GCREG_SEMANTIC_LOCATION);
        index += _STATE(GCREG_SEMANTICS);

        uscControl |= gcmSETFIELD(0, GCREG_USC_CONTROL, CACHE_PAGES, hardware->options.uscL1CacheRatio);
        if (multiCluster)
        {
            uscControl |= gcmSETFIELD(0, GCREG_USC_CONTROL, ATTRIB_CACHE_PAGES, hardware->options.uscAttribCacheRatio);
        }
        index += _STATE_INIT_VALUE(GCREG_USC_CONTROL, uscControl);
    }
    else
    {
        index += _STATE(GCREG_VARYING_PACKING);
        index += _STATE(GCREG_COMPONENT_TYPE_LOW);
        index += _STATE(GCREG_COMPONENT_TYPE_HIGH);
        index += _STATE(GCREG_VARYING_PACKING_EX);
        index += _STATE(GCREG_COMPONENT_TYPE_EX);
        index += _STATE(GCREG_COMPONENT_TYPE_EX2);
    }

    if (hasGS)
    {
        index += _STATE(GCREG_DEPTH_SLICE);
    }

    index += _STATE(AQ_MODE);

    /* Front End states. */
    if (halti5)
    {
        index += _STATE(GCREG_FE_ATTRIBUTE);
        index += _CLOSE_RANGE();
        index += _STATE(GCREG_FE_ID);
        index += _STATE(GCREG_FE_PACKING);
        index += _STATE(GCREG_FE_CONTROL);
        index += _STATE(GCREG_FE_FETCH);
        if (genericAttrib || newGPipe)
        {
            index += _STATE(GCREG_FE_GENERIC_X);
            index += _STATE(GCREG_FE_GENERIC_Y);
            index += _STATE(GCREG_FE_GENERIC_Z);
            index += _STATE(GCREG_FE_GENERIC_W);
        }
    }
    else
    {
        index += _STATE_COUNT(AQ_VERTEX_ELEMENT_CTRL, (halti0 ? 16 : 12));
        index += _CLOSE_RANGE();
        if (genericAttrib)
        {
            index += _STATE(GCREG_FE_GENERICS_X);
            index += _STATE(GCREG_FE_GENERICS_Y);
            index += _STATE(GCREG_FE_GENERICS_Z);
            index += _STATE(GCREG_FE_GENERICS_W);
        }
    }

    if (halti2 || (Context->hardware->identity.streamCount > 8))
    {
        index += _STATE(GCREG_FE_STREAM_ADDRESS);
        index += _STATE(GCREG_FE_STREAM_STRIDE);
        index += _STATE(GCREG_FE_STREAM_DIVISOR);
    }
    else if (Context->hardware->identity.streamCount > 1)
    {
        index += _STATE_HINT(GCREG_FE_VERTEX_ADDRESS);
        index += _STATE(GCREG_FE_VERTEX_STRIDE);
    }
    else
    {
        index += _STATE_HINT(AQ_VERTEX_STREAM_BASE_ADDR);
        index += _STATE(AQ_VERTEX_STREAM_CTRL);

    }
    index += _STATE_HINT(AQ_INDEX_STREAM_BASE_ADDR);
    index += _STATE(AQ_INDEX_STREAM_CTRL);
    index += _STATE(GCREG_FE_PRIMITIVE_RESTART);
    if (halti1)
    {
        index += _STATE(GCREG_FE_INDEX_MIN);
        index += _STATE(GCREG_FE_INDEX_MAX);
    }
    index += _CLOSE_RANGE();

    if (hasRobustness)
    {
        index += _STATE_HINT(GCREG_FE_STREAM_END_ADDRESS);
        index += _CLOSE_RANGE();
        index += _STATE_HINT(GCREG_FE_INDEX_END_ADDRESS);
        index += _CLOSE_RANGE();
    }

    /* WD */
    if (multiCluster)
    {
        index += _STATE_INIT_VALUE(GCREG_WD_STATIC_CONFIG,
                                   gcmSETFIELDVALUE(0, GCREG_WD_STATIC_CONFIG, ASSIGNMENT_MODE, ROUND_ROBIN));
    }

    if (halti5)
    {
        index += _STATE_INIT_VALUE(GCREG_SHADER_STATIC,
                                   gcmSETFIELD(0,
                                               GCREG_SHADER_STATIC,
                                               GPIPE_SMALL_BATCH,
                                               smallBatch ?
                                               GCREG_SHADER_STATIC_GPIPE_SMALL_BATCH_ENABLE
                                             : GCREG_SHADER_STATIC_GPIPE_SMALL_BATCH_DISABLE)
                                 | gcmSETFIELD(0,
                                               GCREG_SHADER_STATIC,
                                               PIXEL_SMALL_BATCH,
                                               smallBatch ?
                                               GCREG_SHADER_STATIC_PIXEL_SMALL_BATCH_ENABLE
                                             : GCREG_SHADER_STATIC_PIXEL_SMALL_BATCH_DISABLE)
                                 | gcmSETFIELDVALUE(0,
                                                    GCREG_SHADER_STATIC,
                                                    DUAL_ISSUE,
                                                    ENABLE)
                                 | gcmSETFIELDVALUE(0,
                                                    GCREG_SHADER_STATIC,
                                                    TWO_GROUP_FAST_REISSUE,
                                                    ENABLE));

        index += _STATE(GCREG_SHADER_MISC_CONFIG);
    }
    else
    {
        /* This register is programed by all chips, which program all DECODE_SELECT as VS
        ** except SAMPLER_DECODE_SELECT.
        */
        index += _STATE(GCREG_SHADER_CONFIG);
    }

    if (hasICache)
    {
        /* I-Cache states. */
        index += _STATE(GCREG_SH_CACHE_CONTROL);
        index += _STATE(GCREG_VS_INSTRUCTION);
        index += _STATE(GCREG_CE_INSTRUCTION);
        index += _STATE(GCREG_PS_INSTRUCTION);
        index += _CLOSE_RANGE();

        if (hasICachePrefetch)
        {
            if (halti5)
            {
                index += _STATE_HINT(GCREG_VS_INSTRUCTION_PREFETCH_RELATIVE_END);
                index += _STATE_HINT(GCREG_PS_INSTRUCTION_PREFETCH_RELATIVE_END);
            }
            else
            {
                index += _STATE_HINT(GCREG_VS_INSTRUCTION_PREFETCH_END);
                index += _STATE_HINT(GCREG_PS_INSTRUCTION_PREFETCH_END);
            }
            index += _CLOSE_RANGE();
        }
    }

    if (multiCluster)
    {
        index += _STATE(GCREG_GPIPE_LOAD_BALANCING);
    }

    /* Vertex Shader states. */
    index += _STATE(AQ_VERTEX_SHADER_OUTPUT_CONTROL);
    index += _STATE_INIT_VALUE(AQ_VERTEX_SHADER_INPUT_CONTROL,
                     gcmSETFIELD(0, AQ_VERTEX_SHADER_INPUT_CONTROL, COUNT, 1));
    index += _STATE(AQ_VERTEX_SHADER_TEMPORARY_REGISTER_CONTROL);
    index += _STATE(AQ_VERTEX_SHADER_LOAD_BALANCING);

    if (halti5)
    {
        index += _STATE(GCREG_VS_UNPACK);
        index += _STATE(GCREG_VS_THROTTLE);
        index += _STATE(GCREG_VS_ATTRIBUTE);
        index += _STATE(GCREG_VS_SAMPLER_BASE);
        index += _STATE(GCREG_VS_INPUT);
        index += _STATE(GCREG_VS_OUTPUT);
    }
    else
    {
        index += _STATE(AQ_VERTEX_SHADER_OUTPUT);
        index += _STATE(AQ_VERTEX_SHADER_INPUT);
    }

    if (multiCluster)
    {
        index += _STATE(GCREG_FA_CONTROL);
        index += _STATE(GCREG_VS_THROTTLE_EX);
    }

    index += _CLOSE_RANGE();

    /* GS */
    if (hasGS)
    {
        index += _STATE(GCREG_GS_CONTROL);
        index += _STATE(GCREG_GS_TYPE);
        index += _STATE(GCREG_GS_COUNTERS);
        index += _STATE(GCREG_GS_START);
        index += _STATE(GCREG_GS_RELATIVE_END);
        index += _STATE_HINT(GCREG_GS_INSTRUCTION);
        index += _STATE(GCREG_GS_INSTRUCTION_PREFETCH_RELATIVE_END);
        index += _STATE(GCREG_GS_THROTTLE);
        index += _STATE(GCREG_GS_ATTRIBUTE);
        index += _STATE(GCREG_GS_ATTRIBUTE_EX);
        index += _STATE(GCREG_GS_UNIFORM_BASE);
        index += _STATE(GCREG_GS_SAMPLER_BASE);
        index += _STATE(GCREG_GS_OUTPUT);
        index += _CLOSE_RANGE();
    }

    /* TCS & TES */
    if (hasTS)
    {
        index += _STATE(GCREG_FE_PATCH_CONTROL);

        index += _STATE(GCREG_TCS_CONTROL);
        index += _STATE(GCREG_TCS_THROTTLE);
        index += _STATE(GCREG_TCS_ATTRIBUTE);
        index += _STATE(GCREG_TCS_ATTRIBUTE_EX);
        index += _STATE(GCREG_TCS_START);
        index += _STATE(GCREG_TCS_RELATIVE_END);
        index += _STATE_HINT(GCREG_TCS_INSTRUCTION);
        index += _STATE(GCREG_TCS_INSTRUCTION_PREFETCH_RELATIVE_END);
        index += _STATE(GCREG_TCS_OUTPUT);
        index += _STATE(GCREG_TCS_UNIFORM_BASE);
        index += _STATE(GCREG_TCS_SAMPLER_BASE);

        index += _CLOSE_RANGE();

        index += _STATE(GCREG_TES_CONTROL);
        index += _STATE(GCREG_TES_THROTTLE);
        index += _STATE(GCREG_TES_ATTRIBUTE);
        index += _STATE(GCREG_TES_START);
        index += _STATE(GCREG_TES_RELATIVE_END);
        index += _STATE_HINT(GCREG_TES_INSTRUCTION);
        index += _STATE(GCREG_TES_INSTRUCTION_PREFETCH_RELATIVE_END);
        index += _STATE(GCREG_TES_OUTPUT);
        index += _STATE(GCREG_TES_UNIFORM_BASE);
        index += _STATE(GCREG_TES_SAMPLER_BASE);
        index += _STATE(GCREG_TES_THROTTLE_EX);

        index += _STATE(GCREG_TPG_CONTROL);

    }

    index += _CLOSE_RANGE();

    /* TFB */
    if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HW_TFB))
    {
        index += _STATE(GCREG_TFB_CONTROL);
        index += _STATE_HINT(GCREG_TFB_HEADER);
        index += _STATE_HINT_BLOCK(GCREG_TFB_BUFFER_ADDRESS, 0, 4);
        index += _STATE_COUNT(GCREG_TFB_BUFFER_SIZE, 4);
        index += _STATE_COUNT(GCREG_TFB_BUFFER_STRIDE, 4);
        index += _STATE_COUNT(GCREG_TFB_DESCRIPTOR_COUNT, 4);
        index += _STATE_COUNT(GCREG_TFB_DESCRIPTOR, 128*4);

        index += _STATE_HINT(GCREG_TFB_QUERY_HEADER);
        index += _CLOSE_RANGE();

    }

    /* Primitive Assembly states. */
    index += _STATE_X(AQ_VIEWPORT_XSCALE);
    index += _STATE_X(AQ_VIEWPORT_YSCALE);
    index += _STATE(AQ_VIEWPORT_ZSCALE);
    index += _STATE_X(AQ_VIEWPORT_XOFFSET);
    index += _STATE_X(AQ_VIEWPORT_YOFFSET);
    index += _STATE(AQ_VIEWPORT_ZOFFSET);
    index += _STATE(AQPA_CLIP_AA_LINE_SCALE);
    index += _STATE_INIT_VALUE(AQPA_CLIP_POINT_SIZE_DIV_BY2, 0x3F000000);
    index += _STATE(AQ_SYSTEM);
    index += _STATE(AQPA_CLIP_WPLANE_LIMIT);
    index += _STATE(AQ_PACLIPVT30);
    index += _STATE(AQPA_MASKED);
    index += _STATE(GCREG_PA_LINE_ADJUST_SUB);
    index += _STATE(GCREG_PA_LINE_ADJUST_ADD);
    index += _STATE(GCREG_PA_CLIP_W);
    index += _STATE_X(GCREG_PA_MAX_POINT_SIZE);
    index += _STATE(GCREG_PA_SMALL_W);
    index += _STATE(GCREG_PA_CLIP_CONTROL);

    if (halti5)
    {
        index += _STATE(GCREG_PA_CONTROL);
        index += _STATE(GCREG_PA_PACK);
    }
    else
    {
        index += _STATE_COUNT(AQPA_CLIP_FLAT_COLOR_TEX, Context->hardware->identity.varyingsCount);
    }

    if (multiCluster)
    {
        index += _STATE(GCREG_FINAL_PA_CONTROL);
    }

    index += _STATE(GCREG_MULTI_CHIP_CONTROL);
    index += _STATE(GCREG_MULTI_CHIP_TOP_LEFT);
    index += _STATE(GCREG_MULTI_CHIP_BOTTOM_RIGHT);

    if (multiCoreBlockSetCfg2)
    {
        index += _STATE(GCREG_MULTI_CHIP_SET_MAPPING);
        index += _STATE(GCREG_MULTI_CHIP_SET_MAPPING_EX);
    }

    /* Setup states. */
    index += _STATE_X(AQ_SETUP_CLIP_LEFT);
    index += _STATE_X(AQ_SETUP_CLIP_TOP);
    index += _STATE_X(AQ_SETUP_CLIP_RIGHT);
    index += _STATE_X(AQ_SETUP_CLIP_BOTTOM);
    index += _STATE(AQ_SETUP_SLOPE_SCALED_DEPTH_BIAS);
    index += _STATE(AQ_SETUP_DEPTH_BIAS);
    index += _STATE(AQ_SETUP_CONFIG);
    index += _STATE(GCREG_SE_AREA_THRESHOLD);
    index += _STATE_X(GCREG_SE_CLIP_COMPARE_RIGHT);
    index += _STATE_X(GCREG_SE_CLIP_COMPARE_BOTTOM);

    /* Raster states. */
    index += _STATE(AQ_RASTER_CONTROL);
    index += _STATE(GCREG_MULTI_SAMPLE_COORDS);
    index += _STATE(GCREG_MULTI_SAMPLE_JITTER);
    index += _STATE(GCREG_CENTROIDS);
    index += _STATE_INIT_VALUE(GCREG_RA_CONTROL, gcmSETFIELD(GCREG_RA_CONTROL_ResetValue,
                                                             GCREG_RA_CONTROL,
                                                             SMALL_BATCH,
                                                             smallBatch ?
                                                             GCREG_RA_CONTROL_SMALL_BATCH_ENABLED
                                                           : GCREG_RA_CONTROL_SMALL_BATCH_DISABLED));
    index += _STATE(GCREG_HZ_BASE_ADDRESS1);
    index += _STATE(GCREG_RA_CONTROL_HZ);

    if (halti2)
    {
        index += _STATE(GCREG_RA_MEDIUMP_MASK);
    }

    if (halti5)
    {
        index += _STATE(GCREG_RA_CONTROL_EX);
    }


    /* Pixel Shader states. */
    index += _STATE(AQ_PIXEL_SHADER_COLOR_OUT);
    index += _STATE(AQ_PIXEL_SHADER_INPUT_CONTROL);
    index += _STATE(AQ_PIXEL_SHADER_TEMPORARY_REGISTER_CONTROL);
    index += _STATE(AQ_PIXEL_SHADER_CONTROL);
    index += _STATE(GCREG_PS_OUTPUT_MODE);
    index += _STATE(GCREG_PS_OUTPUT_PRECISION);

    if (halti2)
    {
        index += _STATE(GCREG_PS_INPUT_TYPE);
    }

    if (numRT == 16)
    {
        index += _STATE(GCREG_PS_OUTPUT_CONTROL);
        index += _STATE(GCREG_PS_OUTPUT_CONTROL1);
        index += _STATE(GCREG_PS_OUTPUT_CONTROL2);
    }
    else if (numRT == 8)
    {
        index += _STATE(GCREG_PS_OUTPUT_CONTROL);
        index += _STATE(GCREG_PS_OUTPUT_PRECISION_EX);
    }

    if (hasMsaaFragOperation)
    {
        index += _STATE(GCREG_PS_SAMPLE_COVERAGE_CTRL);
        index += _STATE_COUNT(GCREG_PS_SAMPLE_DITHER_TABLE, 8);
    }

    if (halti5)
    {
        index += _STATE(GCREG_PS_UNPACK);
        index += _STATE(GCREG_PS_SAMPLER_BASE);
        index += _STATE(GCREG_PS_MAX);
    }

    if (hasPSCSThrottle)
    {
        index += _STATE(GCREG_PSCS_THROTTLE);
    }

    index += _CLOSE_RANGE();

    /* Texture states. */
    if (hasTXdesc)
    {
        /* Texture descriptor states */
        index += _STATE(GCREG_TX_GLOBAL_CONFIG);

        if (smallBatch)
        {
            index += _STATE_INIT_VALUE(GCREG_SH_ALLOC_SAMPLERS, numSamplers);
            index += _STATE_INIT_VALUE(GCREG_SH_ALLOC_TEXTURES, numSamplers);

            index += _STATE_COUNT(GCREG_TX_GPIPE_SAMPLER_CTRL0, numSamplers);
            index += _STATE_COUNT(GCREG_TX_GPIPE_SAMPLER_CTRL1, numSamplers);
            index += _STATE_COUNT(GCREG_TX_GPIPE_SAMPLER_LOD_MAX_MIN, numSamplers);
            index += _STATE_COUNT(GCREG_TX_GPIPE_SAMPLER_LOD_BIAS, numSamplers);
            index += _STATE_COUNT(GCREG_TX_GPIPE_SAMPLER_ANISO_CTRL, numSamplers);

            index += _STATE_HINT_BLOCK(GCREG_TX_GPIPE_TEXTURE_DESCRIPTOR_ADDR, 0, numSamplers);
            index += _STATE_COUNT(GCREG_TX_GPIPE_TEXTURE_CTRL, numSamplers);

            index += _CLOSE_RANGE();
        }
        else
        {
            index += _STATE_COUNT(GCREG_TX_PIXEL_SAMPLER_CTRL0, numSamplers);
            index += _STATE_COUNT(GCREG_TX_PIXEL_SAMPLER_CTRL1, numSamplers);
            index += _STATE_COUNT(GCREG_TX_PIXEL_SAMPLER_LOD_MAX_MIN, numSamplers);
            index += _STATE_COUNT(GCREG_TX_PIXEL_SAMPLER_LOD_BIAS, numSamplers);
            index += _STATE_COUNT(GCREG_TX_PIXEL_SAMPLER_ANISO_CTRL, numSamplers);

            index += _STATE_HINT_BLOCK(GCREG_TX_PIXEL_TEXTURE_DESCRIPTOR_ADDR, 0, numSamplers);
            index += _STATE_COUNT(GCREG_TX_PIXEL_TEXTURE_CTRL, numSamplers);

            index += _CLOSE_RANGE();

            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_SAMPLER_CTRL0, GCREG_TX_PIXEL_SAMPLER_CTRL0, numSamplers);
            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_SAMPLER_CTRL1, GCREG_TX_PIXEL_SAMPLER_CTRL1, numSamplers);
            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_SAMPLER_LOD_MAX_MIN, GCREG_TX_PIXEL_SAMPLER_LOD_MAX_MIN, numSamplers);
            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_SAMPLER_LOD_BIAS, GCREG_TX_PIXEL_SAMPLER_LOD_BIAS, numSamplers);
            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_SAMPLER_ANISO_CTRL, GCREG_TX_PIXEL_SAMPLER_ANISO_CTRL, numSamplers);
            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_TEXTURE_DESCRIPTOR_ADDR, GCREG_TX_PIXEL_TEXTURE_DESCRIPTOR_ADDR, numSamplers);
            _STATE_MIRROR_COUNT(GCREG_TX_GPIPE_TEXTURE_CTRL, GCREG_TX_PIXEL_TEXTURE_CTRL, numSamplers);
        }
    }
    else
    {
        index += _STATE_COUNT(AQ_TEXTURE_SAMPLE_MODE, 12);
        index += _STATE_COUNT(AQ_TEXTURE_SAMPLE_WH, 12);
        index += _STATE_COUNT(AQ_TEXTURE_SAMPLE_LOG_WH, 12);
        index += _STATE_COUNT(AQ_TEXTURE_SAMPLE_LOD, 12);
        index += _STATE_COUNT(AQ_TEXTURE_SAMPLE_BORDER_COLOR, 12);
        index += _STATE_COUNT(GCREG_TX_STRIDE, 12);
        index += _STATE_COUNT(GCREG_TX_VOLUME, 12);
        index += _STATE_COUNT(GCREG_TX_MODE, 12);
        index += _STATE_COUNT(GCREG_TX_CONTROL_YUV, 12);
        index += _STATE_COUNT(GCREG_TX_STRIDE_YUV, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD00_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD01_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD02_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD03_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD04_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD05_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD06_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD07_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD08_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD09_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD10_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD11_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD12_ADDRESS, 0, 12);
        index += _STATE_HINT_BLOCK(AQ_TEXTURE_SAMPLE_LOD13_ADDRESS, 0, 12);
        index += _CLOSE_RANGE();

        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TEXTURE_LINEAR))
        {
            /*
             * Linear stride LODn will overwrite LOD0 on GC880,GC2000.
             * And only LOD0 is valid for this register.
             */
            gctUINT count = halti1 ? 14 : 1;

            for (i = 0; i < 12; i += 1)
            {
                index += _STATE_COUNT_OFFSET(GCREG_TX_SAMPLER_STRIDE, i * 16, count);
            }
        }

        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SUPPORT_GCREGTX))
        {
            gctUINT texBlockCount;
            gctUINT gcregTXLogSizeResetValue;

            /* Enable the integer filter pipe for all texture samplers
               so that the floating point filter clock will shut off until
               we start using the floating point filter.
            */
            gcregTXLogSizeResetValue = gcmSETFIELDVALUE(
                GCREG_TX_LOG_SIZE_ResetValue,
                GCREG_TX_LOG_SIZE,
                INTEGER_FILTER_CONTROL,
                INTEGER);

            /* New texture block. */
            index += _STATE(GCREG_TX_CONFIG);
            index += _STATE(GCREG_TX_SIZE);
            index += _STATE_INIT_VALUE(GCREG_TX_LOG_SIZE, gcregTXLogSizeResetValue);
            index += _STATE(GCREG_TX_LOD);
            index += _STATE(GCREG_TX_BORDER_COLOR);
            index += _STATE(GCREG_TX_LINEAR_STRIDE);
            index += _STATE(GCREG_TX3_D);
            index += _STATE(GCREG_TX_EXT_CONFIG);
            index += _STATE(GCREG_TX_CONTROL_YUV_EX);
            index += _STATE(GCREG_TX_STRIDE_YUV_EX);

            if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_FILTER))
            {
                index += _STATE(GCREG_TX_HORIZONTAL_KERNEL);
                index += _STATE(GCREG_TX_VERTICAL_KERNEL);
            }

            texBlockCount = _BLOCK_COUNT(GCREG_TX_ADDRESS);

            for (i = 0; i < texBlockCount; i += 1)
            {
                index += _STATE_HINT_BLOCK(GCREG_TX_ADDRESS, i, 14);
            }
        }

        if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_TEX_BASELOD))
        {
            index += _STATE(GCREG_TX_BASE_LOD);
        }

        if (halti3 ||
            gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_SUPPORT_DEC))
        {
            index += _STATE(GCREG_TX_CONFIG2);
        }

        if (halti4)
        {
            index += _STATE_COUNT(GCREG_TX_SLICE, 32);
            index += _STATE_COUNT(GCREG_TX_CONFIG3, 32);
        }

        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_FRAC_PRECISION_6BIT))
        {
            index += _STATE_COUNT(GCREG_TX_SIZE_EXT, 32);
            index += _STATE_COUNT(GCREG_TX_VOLUME_EXT, 32);
            index += _STATE_COUNT(GCREG_TX_LOD_EXT, 32);
            index += _STATE_COUNT(GCREG_TX_LOD_BIAS_EXT, 32);
            index += _STATE_COUNT(GCREG_TX_ANISO_CTRL, 32);
        }

        /* ASTC */
        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TEXTURE_ASTC))
        {
            index += _STATE(GCREG_TX_ASTC0_EX);
            index += _STATE(GCREG_TX_ASTC1_EX);
            index += _STATE(GCREG_TX_ASTC2_EX);
            index += _STATE(GCREG_TX_ASTC3_EX);
        }
    }

    if (halti3)
    {
        index += _STATE(GCREG_TX_GUARDBAND);
    }

    /* Thread walker states. */
    index += _STATE(GCREG_TW_CONFIG);
    index += _STATE(GCREG_TW_INFO_X);
    index += _STATE(GCREG_TW_INFO_Y);
    index += _STATE(GCREG_TW_INFO_Z);
    index += _STATE(GCREG_TW_WORK_GROUP_X);
    index += _STATE(GCREG_TW_WORK_GROUP_Y);
    index += _STATE(GCREG_TW_WORK_GROUP_Z);
    index += _STATE(GCREG_TW_SHADER_INFO2);
    index += _STATE(GCREG_TW_SHADER_INFO);

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SHADER_ENHANCEMENTS2))
    {
        index += _STATE(GCREG_TW_WORK_GROUP_COUNT_X);
        index += _STATE(GCREG_TW_WORK_GROUP_COUNT_Y);
        index += _STATE(GCREG_TW_WORK_GROUP_COUNT_Z);
        index += _STATE(GCREG_TW_WORKGROUP_SIZE_X);
        index += _STATE(GCREG_TW_WORKGROUP_SIZE_Y);
        index += _STATE(GCREG_TW_WORKGROUP_SIZE_Z);
    }

    if (halti5)
    {
        index += _STATE(GCREG_TW_INFO_GLOBAL_SCALE_X);
        index += _STATE(GCREG_TW_INFO_GLOBAL_SCALE_Y);
        index += _STATE(GCREG_TW_INFO_GLOBAL_SCALE_Z);
    }

    index += _CLOSE_RANGE();

    /* VS/PS Start/End PC register */
    if (halti5)
    {
        index += _STATE(GCREG_VS_START_PC);
        index += _STATE(GCREG_VS_RELATIVE_END);
        index += _STATE(GCREG_PS_START_PC);
        index += _STATE(GCREG_PS_RELATIVE_END);
        index += _CLOSE_RANGE();
    }
    else if (hasICache)
    {
        /* New Shader instruction PC registers(20bit). */
        index += _STATE(GCREG_VS_START_PC);
        index += _STATE(GCREG_VS_END_PC);
        index += _STATE(GCREG_PS_START_PC);
        index += _STATE(GCREG_PS_END_PC);
        index += _CLOSE_RANGE();
    }
    else
    {
        if (Context->hardware->identity.instructionCount <= 256)
        {
            /* old shader instruction PC registers (12bit)*/
            index += _STATE(AQ_VERTEX_SHADER_END_PC);
            index += _STATE(AQ_VERTEX_SHADER_START_PC);
            index += _CLOSE_RANGE();

            index += _STATE(AQ_PIXEL_SHADER_END_PC);
            index += _STATE(AQ_PIXEL_SHADER_START_PC);
            index += _CLOSE_RANGE();
        }
        else
        {
            /* New Shader instruction PC registers (16bit) */
            index += _STATE(GCREG_VS_PROGRAM);
            index += _STATE(GCREG_PS_PROGRAM);
            index += _CLOSE_RANGE();
        }
    }


    if (!hasICachePrefetch)
    {
        /* This unified one need SELECT bit to steer */
        if (Context->hardware->identity.instructionCount > 1024)
        {
            for (i = 0;
                 i < Context->hardware->identity.instructionCount << 2;
                 i += 256 << 2
                 )
            {
                index += _STATE_COUNT_OFFSET(GCREG_SH_INSTRUCTIONS, i, 256 << 2);
                index += _CLOSE_RANGE();
            }
        }
        /* This unified one is steered by base adddress, it's automatical. */
        else if (Context->hardware->identity.instructionCount > 256)
        {
            /* VS instruction memory. */
            for (i = 0;
                 i < Context->hardware->identity.instructionCount << 2;
                 i += 256 << 2
                 )
            {
                index += _STATE_COUNT_OFFSET(GCREG_VS_SHADER_CODE, i, 256 << 2);
                index += _CLOSE_RANGE();
            }

            _STATE_MIRROR_COUNT(GCREG_PS_SHADER_CODE,
                                GCREG_VS_SHADER_CODE,
                                Context->hardware->identity.instructionCount << 2);
        }
        /* if (Context->hardware->identity.instructionCount <= 256). This is non-unified one. */
        else
        {
            index += _STATE(AQ_VERTEX_SHADER_INST);
            index += _CLOSE_RANGE();
            index += _STATE(AQ_PIXEL_SHADER_INST);
            index += _CLOSE_RANGE();
        }
    }

    if (unifiedUniform)
    {
        gctINT numConstants = Context->hardware->identity.numConstants;

        /* Base Offset register */
        index += _STATE(GCREG_PIXEL_SHADER_CONST);
        index += _STATE(GCREG_VERTEX_SHADER_CONST);
        index += _CLOSE_RANGE();

        if (smallBatch)
        {
            index += _STATE_INIT_VALUE(GCREG_SH_ALLOC_UNIFORMS, numConstants);
        }

        for (i = 0;
             numConstants > 0;
             i += 256 << 2,
             numConstants -= 256
             )
        {
            if (halti5)
            {
                if (numConstants >= 256)
                {
                    if (smallBatch)
                    {
                        index += _STATE_COUNT_OFFSET(GCREG_GPIPE_UNIFORMS, i, 256 << 2);
                    }
                    else
                    {
                        index += _STATE_COUNT_OFFSET(GCREG_PIXEL_UNIFORMS, i, 256 << 2);
                    }
                }
                else
                {
                    if (smallBatch)
                    {
                        index += _STATE_COUNT_OFFSET(GCREG_GPIPE_UNIFORMS, i, numConstants << 2);
                    }
                    else
                    {
                        index += _STATE_COUNT_OFFSET(GCREG_PIXEL_UNIFORMS, i, numConstants << 2);
                    }
                }
                index += _CLOSE_RANGE();
            }
            else
            {
                if (numConstants >= 256)
                {
                    index += _STATE_COUNT_OFFSET(GCREG_SH_UNIFORMS, i, 256 << 2);
                }
                else
                {
                    index += _STATE_COUNT_OFFSET(GCREG_SH_UNIFORMS, i, numConstants << 2);
                }

                index += _CLOSE_RANGE();
            }
        }

        if (halti5 && !smallBatch)
        {
            _STATE_MIRROR_COUNT(GCREG_GPIPE_UNIFORMS,
                                GCREG_PIXEL_UNIFORMS,
                                Context->hardware->identity.numConstants << 2);
        }
    }
#if gcdENABLE_UNIFIED_CONSTANT
    else
#endif
    {
        index += _STATE_COUNT(AQ_VERTEX_SHADER_CONST, vertexUniforms * 4);
        index += _STATE_COUNT(AQ_PIXEL_SHADER_CONST, fragmentUniforms * 4);
    }

    if (halti1)
    {
        index += _STATE(GCREG_SH_SAMPLER_ARBITRATION);
    }

    if (halti5)
    {
        index += _STATE(GCREG_SH_ICACHE_INVALIDATE);
    }

    /* Store the index of the "XD" entry. */
    Context->entryOffsetXDFrom3D = (gctUINT)index * gcmSIZEOF(gctUINT32);


    /* Pixel Engine states. */
    index += _STATE(AQ_DEPTH_CONFIG);
    index += _STATE(AQ_DEPTH_NEAR);
    index += _STATE(AQ_DEPTH_FAR);
    index += _STATE(AQ_DEPTH_NORMALIZE);
    index += _STATE(AQ_DEPTH_STRIDE);
    index += _STATE(AQ_STENCIL_OPERATION);
    index += _STATE(AQ_STENCIL_CONFIG);
    index += _STATE(AQ_ALPHA_TEST);
    index += _STATE(AQ_ALPHA_BLEND_FACTOR);
    index += _STATE(AQ_ALPHA_BLEND);
    index += _STATE(AQ_PIXEL_CONFIG);
    index += _STATE(AQ_PIXEL_STRIDE);
    index += _STATE(GCREG_PE_HIERARCHICAL_CONTROL);
    index += _STATE_HINT(GCREG_PE_HIERARCHICAL_ADDRESS);
    index += _STATE(GCREG_PE_EXTRA_REFERENCE);
    index += _STATE(GCREG_PE_DITHER_LOW);
    index += _STATE(GCREG_PE_DITHER_HIGH);

    if(gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALF_FLOAT_PIPE) )
    {
        index += _STATE(GCREG_PE_BLEND_FACTOR_LOW);
        index += _STATE(GCREG_PE_BLEND_FACTOR_HIGH);
    }
    index += _STATE(GCREG_PE_CONFIG);
    index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_CONFIG, 3);
    index += _STATE(GCREG_PE_BACKFACE_STENCIL);

    if (halti3)
    {
        index += _STATE(GCREG_PS_OUTPUT_MAP);
    }

    index += _STATE_HINT_BLOCK(GCREG_PE_COLOR_BASE_ADDRESS, 0,
                               Context->hardware->identity.pixelPipes);

    if (Context->hardware->identity.pixelPipes == 1)
    {
        index += _STATE_HINT(AQ_PIXEL_ADDRESS);
        index += _STATE_HINT(AQ_DEPTH_ADDRESS);
    }

    if (Context->hardware->identity.pixelPipes > 1 || halti0)
    {
        index += _STATE_HINT_BLOCK(GCREG_PE_DEPTH_BASE_ADDRESS, 0,
                                   Context->hardware->identity.pixelPipes);
    }

    for (i = 0; i < 3; i++)
    {
        index += _STATE_HINT_BLOCK(GCREG_PE_RENDER_TARGET_ADDRESS,
                                   i,
                                   Context->hardware->identity.pixelPipes);
    }

    if (numRT == 16)
    {
        for (i = 0; i < 15; i++)
        {
            index += _STATE_HINT_BLOCK(GCREG_PE_RENDER_TARGET_ADDRESS_EX2,
                                       i,
                                       Context->hardware->identity.pixelPipes);
        }
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_CONFIG_EX2, 15);
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_EXTRA_CONFIG_EX2, 15);
    }
    else if (numRT == 8)
    {
        for (i = 0; i < 7; i++)
        {
          index += _STATE_HINT_BLOCK(GCREG_PE_RENDER_TARGET_ADDRESS_EX,
                                     i,
                                     Context->hardware->identity.pixelPipes);
        }
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_CONFIG_EX, 7);
    }


    if (halti3)
    {
        index += _STATE(GCREG_PE_CONFIG_EX);
    }

    if (halti4)
    {
        index += _STATE(GCREG_PE_ALPHA_CONFIG);
    }

    if (hasGS)
    {
        index += _STATE(GCREG_COLOR_SLICE);
    }

    if (halti5)
    {
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_EXTRA_CONFIG_EX, 7);
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_ALPHA_TEST, 7);
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_ALPHA_CONTROL, 7);
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_BLEND_COLOR_LOW, 7);
        index += _STATE_COUNT(GCREG_PE_RENDER_TARGET_BLEND_COLOR_HIGH, 7);
    }

    if (hasRobustness)
    {
        index += _STATE_HINT(GCREG_PE_RENDER_TARGET_END_ADDRES_EX);
        index += _STATE_HINT(GCREG_DEPTH_BUFFER_END_ADDRESS);
    }

    /* Memory Controller */
    index += _STATE(GC_MEMORY_CONFIG);

    index += _CLOSE_RANGE();
    index += _STATE_HINT(GC_COLOR_CACHE);
    index += _STATE_HINT(GC_COLOR_BASE);
    index += _STATE(GC_COLOR_CLEAR_VALUE);
    index += _STATE_HINT(GC_DEPTH_CACHE);
    index += _STATE_HINT(GC_DEPTH_BASE);
    index += _STATE(GC_DEPTH_CLEAR_VALUE);
    index += _STATE(GC_TILE_STATUS_COUNTER_Z);
    index += _STATE(GC_TILE_STATUS_COUNTER_C);
    index += _STATE_HINT(GCREG_MC_CACHE_HZ);
    index += _STATE(GCREG_MC_COUNTER_HZ);
    index += _STATE(GCREG_MC_CLEAR_VALUE_HZ);
    index += _STATE(GCREG_MC_TEXTURE_CONFIG);
    index += _STATE_HINT(GCREG_MC_TEXTURE_BUFFER);
    index += _STATE(GCREG_MC_TEXTURE_CLEAR);


    if (halti2)
    {
        index += _STATE(GCREG_MC_TEXTURE_CLEAR64);
        index += _STATE(GCREG_MC_COLOR_CLEAR64);
        index += _STATE_COUNT_OFFSET(GCREG_MRT_MEMORY_CONFIG, 1, 7);
        index += _STATE_COUNT_OFFSET_HINT(GCREG_MRT_COLOR_CACHE, 1, 7);
        index += _STATE_COUNT_OFFSET_HINT(GCREG_MRT_COLOR_BASE, 1, 7);
        index += _STATE_COUNT_OFFSET(GCREG_MRT_COLOR_CLEAR_VALUE, 1, 7);
        index += _STATE_COUNT_OFFSET(GCREG_MRTMC_COLOR_CLEAR64, 1, 7);
        index += _STATE_COUNT_OFFSET(GCREG_MRT_TILE_STATUS_COUNTER_C, 1, 7);
    }

    index += _CLOSE_RANGE();

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_BUG_FIXES18))
    {
        index += _STATE_INIT_VALUE(GCREG_OCCLUSION_CONTROL, GCREG_OCCLUSION_CONTROL_MODE_RENDER);
        index += _CLOSE_RANGE();
    }

    if (halti3)
    {
        index += _STATE_HINT(GCREG_MC_TEXTURE_BASE_BUFFER);
        index += _CLOSE_RANGE();
    }

    if (hasSecurity || hasRobustness)
    {
        index += _STATE_INIT_VALUE(GCREG_MMU_CONFIG,
                    gcmSETFIELDVALUE(0, GCREG_MMU_CONFIG, MASK_PAGE_TABLE_ID, MASKED));
    }

    /* Semaphore/stall. */
    index += _SemaphoreStall(Context, index);
#endif

    /**************************************************************************/
    /* Link to another address. ***********************************************/

    Context->linkIndex3D = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LINK_COMMAND, OPCODE,   LINK)
            | gcmSETFIELD     (0, AQ_COMMAND_LINK_COMMAND, PREFETCH, 0);

        buffer[index + 1]
            = 0;
    }

    index += 2;

    /* Store the end of the context buffer. */
    Context->bufferSize = index * gcmSIZEOF(gctUINT32);


    /**************************************************************************/
    /* Pipe switch for the case where neither 2D nor 3D are used. *************/

    /* Store the 3D entry index. */
    Context->entryOffsetXDFrom2D = (gctUINT)index * gcmSIZEOF(gctUINT32);

    /* Switch to 3D pipe. */
    index += _SwitchPipe(Context, index, gcvPIPE_3D);

    /* Store the location of the link. */
    Context->linkIndexXD = (gctUINT)index;

    if (buffer != gcvNULL)
    {
        buffer[index + 0]
            = gcmSETFIELDVALUE(0, AQ_COMMAND_LINK_COMMAND, OPCODE,   LINK)
            | gcmSETFIELD     (0, AQ_COMMAND_LINK_COMMAND, PREFETCH, 0);

        buffer[index + 1]
            = 0;
    }

    index += 2;


    /**************************************************************************/
    /* Save size for buffer. **************************************************/

    Context->totalSize = index * gcmSIZEOF(gctUINT32);

    /* Success. */
    gcmkFOOTER_NO();
    return gcvSTATUS_OK;
}
#endif

static gceSTATUS
_DestroyContext(
    IN gckCONTEXT Context
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (Context != gcvNULL)
    {
        gcsCONTEXT_PTR bufferHead;

        /* Free context buffers. */
        for (bufferHead = Context->buffer; Context->buffer != gcvNULL;)
        {
            /* Get a shortcut to the current buffer. */
            gcsCONTEXT_PTR buffer = Context->buffer;

            /* Get the next buffer. */
            gcsCONTEXT_PTR next = buffer->next;

            /* Last item? */
            if (next == bufferHead)
            {
                next = gcvNULL;
            }

            /* Destroy the signal. */
            if (buffer->signal != gcvNULL)
            {
                /* Wait until the context buffer becomes available;
                   this avoid GPU hang due to context command corruption */
                gcmkONERROR(gckOS_WaitSignal(
                    Context->os, buffer->signal, gcvFALSE, gcvINFINITE
                ));

                gcmkONERROR(gckOS_DestroySignal(
                    Context->os, buffer->signal
                    ));

                buffer->signal = gcvNULL;
            }

            /* Free state delta map. */
            if (buffer->logical != gcvNULL)
            {
                gckKERNEL kernel = Context->hardware->kernel;

#if gcdCAPTURE_ONLY_MODE
                gceDATABASE_TYPE dbType;
                gctUINT32 processID;
#endif

                /* End cpu access. */
                gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                    kernel,
                    buffer->videoMem,
                    0,
                    gcvFALSE,
                    gcvFALSE
                    ));

                /* Synchronized unlock. */
                gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
                    kernel,
                    buffer->videoMem,
                    0,
                    gcvNULL
                    ));

#if gcdCAPTURE_ONLY_MODE
                /* Encode surface type and pool to database type. */
                dbType = gcvDB_VIDEO_MEMORY
                       | (gcvVIDMEM_TYPE_GENERIC << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
                       | (buffer->videoMem->pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

                gcmkONERROR(gckOS_GetProcessID(&processID));

                gcmkONERROR(
                    gckKERNEL_RemoveProcessDB(kernel,
                        processID,
                        dbType,
                        buffer->videoMem));
#endif

                /* Free video memory. */
                gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
                    kernel,
                    buffer->videoMem
                    ));

                buffer->logical = gcvNULL;
            }

            /* Free context buffer. */
            gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, buffer));

            /* Remove from the list. */
            Context->buffer = next;
        }

        /* Mark the gckCONTEXT object as unknown. */
        Context->object.type = gcvOBJ_UNKNOWN;

        /* Free the gckCONTEXT object. */
        gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, Context));
    }

OnError:
    return status;
}

#if (gcdENABLE_3D || gcdENABLE_2D)
static gceSTATUS
_AllocateContextBuffer(
    IN gckCONTEXT Context,
    IN gcsCONTEXT_PTR Buffer
    )
{
    gceSTATUS status;
    gckKERNEL kernel = Context->hardware->kernel;
    gcePOOL pool = gcvPOOL_DEFAULT;
    gctSIZE_T totalSize = Context->totalSize;
    gctUINT32 allocFlag = 0;

#if gcdCAPTURE_ONLY_MODE
    gceDATABASE_TYPE dbType;
    gctUINT32 processID;
#endif

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
    allocFlag = gcvALLOC_FLAG_CACHEABLE;
#endif

    /* Allocate video memory node for command buffers. */
    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
        kernel,
        64,
        gcvVIDMEM_TYPE_COMMAND,
        allocFlag,
        &totalSize,
        &pool,
        &Buffer->videoMem
        ));

#if gcdCAPTURE_ONLY_MODE
    gcmkONERROR(gckVIDMEM_HANDLE_Allocate(kernel, Buffer->videoMem, &Context->buffer->handle));

    /* Encode surface type and pool to database type. */
    dbType = gcvDB_VIDEO_MEMORY
           | (gcvVIDMEM_TYPE_GENERIC << gcdDB_VIDEO_MEMORY_TYPE_SHIFT)
           | (pool << gcdDB_VIDEO_MEMORY_POOL_SHIFT);

    gcmkONERROR(gckOS_GetProcessID(&processID));

    /* Record in process db. */
    gcmkONERROR(
            gckKERNEL_AddProcessDB(kernel,
                                   processID,
                                   dbType,
                                   Buffer->videoMem ,
                                   gcvNULL,
                                   totalSize));
#endif

    /* Lock for GPU access. */
    gcmkONERROR(gckVIDMEM_NODE_Lock(
        kernel,
        Buffer->videoMem,
        &Buffer->address
        ));

    /* Lock for kernel side CPU access. */
    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        kernel,
        Buffer->videoMem,
        gcvFALSE,
        gcvFALSE,
        (gctPOINTER *)&Buffer->logical
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}
#endif

/******************************************************************************\
**************************** Context Management API ****************************
\******************************************************************************/

/******************************************************************************\
**
**  gckCONTEXT_Construct
**
**  Construct a new gckCONTEXT object.
**
**  INPUT:
**
**      gckOS Os
**          Pointer to gckOS object.
**
**      gctUINT32 ProcessID
**          Current process ID.
**
**      gckHARDWARE Hardware
**          Pointer to gckHARDWARE object.
**
**  OUTPUT:
**
**      gckCONTEXT * Context
**          Pointer to a variable thet will receive the gckCONTEXT object
**          pointer.
*/
#if (gcdENABLE_3D || gcdENABLE_2D)
gceSTATUS
gckCONTEXT_Construct(
    IN gckOS Os,
    IN gckHARDWARE Hardware,
    IN gctUINT32 ProcessID,
    OUT gckCONTEXT * Context
    )
{
    gceSTATUS status;
    gckCONTEXT context = gcvNULL;
    gctUINT32 allocationSize;
    gctUINT i;
    gctPOINTER pointer = gcvNULL;

    gcmkHEADER_ARG("Os=%p Hardware=%p", Os, Hardware);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Os, gcvOBJ_OS);
    gcmkVERIFY_ARGUMENT(Context != gcvNULL);


    /**************************************************************************/
    /* Allocate and initialize basic fields of gckCONTEXT. ********************/

    /* The context object size. */
    allocationSize = gcmSIZEOF(struct _gckCONTEXT);

    /* Allocate the object. */
    gcmkONERROR(gckOS_Allocate(
        Os, allocationSize, &pointer
        ));

    context = pointer;

    /* Reset the entire object. */
    gcmkONERROR(gckOS_ZeroMemory(context, allocationSize));

    /* Initialize the gckCONTEXT object. */
    context->object.type = gcvOBJ_CONTEXT;
    context->os          = Os;
    context->hardware    = Hardware;


#if !gcdENABLE_3D
    context->entryPipe = gcvPIPE_2D;
    context->exitPipe  = gcvPIPE_2D;
#elif gcdCMD_NO_2D_CONTEXT
    context->entryPipe = gcvPIPE_3D;
    context->exitPipe  = gcvPIPE_3D;
#else
    context->entryPipe
        = gcmGETFIELD(context->hardware->identity.chipFeatures, GC_FEATURES, PIPE_2D)
            ? gcvPIPE_2D
            : gcvPIPE_3D;
    context->exitPipe = gcvPIPE_3D;
#endif

    /* Get the command buffer requirements. */
    gcmkONERROR(gckHARDWARE_QueryCommandBuffer(
        Hardware,
        gcvENGINE_RENDER,
        &context->alignment,
        &context->reservedHead,
        gcvNULL
        ));

    /**************************************************************************/
    /* Get the size of the context buffer. ************************************/

    gcmkONERROR(_InitializeContextBuffer(context));

    if (context->maxState > 0)
    {
        /**************************************************************************/
        /* Allocate and reset the state mapping table. ****************************/
        if (context->hardware->kernel->command->stateMap == gcvNULL)
        {
            /* Allocate the state mapping table. */
            gcmkONERROR(gckOS_Allocate(
                Os,
                gcmSIZEOF(gcsSTATE_MAP) * context->maxState,
                &pointer
                ));

            context->map = pointer;

            /* Zero the state mapping table. */
            gcmkONERROR(gckOS_ZeroMemory(
                context->map, gcmSIZEOF(gcsSTATE_MAP) * context->maxState
                ));

            context->hardware->kernel->command->stateMap = pointer;
        }
        else
        {
            context->map = context->hardware->kernel->command->stateMap;
        }
    }

    /**************************************************************************/
    /* Allocate the context and state delta buffers. **************************/

    for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; i += 1)
    {
        /* Allocate a context buffer. */
        gcsCONTEXT_PTR buffer;

        /* Allocate the context buffer structure. */
        gcmkONERROR(gckOS_Allocate(
            Os,
            gcmSIZEOF(gcsCONTEXT),
            &pointer
            ));

        buffer = pointer;

        /* Reset the context buffer structure. */
        gcmkVERIFY_OK(gckOS_ZeroMemory(
            buffer, gcmSIZEOF(gcsCONTEXT)
            ));

        /* Append to the list. */
        if (context->buffer == gcvNULL)
        {
            buffer->next    = buffer;
            context->buffer = buffer;
        }
        else
        {
            buffer->next          = context->buffer->next;
            context->buffer->next = buffer;
        }

        /* Set the number of delta in the order of creation. */
#if gcmIS_DEBUG(gcdDEBUG_CODE)
        buffer->num = i;
#endif

        /* Create the busy signal. */
        gcmkONERROR(gckOS_CreateSignal(
            Os, gcvFALSE, &buffer->signal
            ));

        /* Set the signal, buffer is currently not busy. */
        gcmkONERROR(gckOS_Signal(
            Os, buffer->signal, gcvTRUE
            ));

        /* Create a new physical context buffer. */
        gcmkONERROR(_AllocateContextBuffer(
            context, buffer
            ));

        /* Set gckEVENT object pointer. */
        buffer->eventObj = Hardware->kernel->eventObj;

        /* Set the pointers to the LINK commands. */
        if (context->linkIndex2D != 0)
        {
            buffer->link2D = &buffer->logical[context->linkIndex2D];
        }

        if (context->linkIndex3D != 0)
        {
            buffer->link3D = &buffer->logical[context->linkIndex3D];
        }

        if (context->linkIndexXD != 0)
        {
            gctPOINTER xdLink;
            gctUINT32 xdEntryAddress;
            gctUINT32 xdEntrySize;
            gctUINT32 linkBytes;

            /* Determine LINK parameters. */
            xdLink
                = &buffer->logical[context->linkIndexXD];

            xdEntryAddress
                = buffer->address
                + context->entryOffsetXDFrom3D;

            xdEntrySize
                = context->bufferSize
                - context->entryOffsetXDFrom3D;

            /* Query LINK size. */
            gcmkONERROR(gckWLFE_Link(
                Hardware, gcvNULL, 0, 0, &linkBytes, gcvNULL, gcvNULL
                ));

            /* Generate a LINK. */
            gcmkONERROR(gckWLFE_Link(
                Hardware,
                xdLink,
                xdEntryAddress,
                xdEntrySize,
                &linkBytes,
                gcvNULL,
                gcvNULL
                ));
        }
    }


    /**************************************************************************/
    /* Initialize the context buffers. ****************************************/

    /* Initialize the current context buffer. */
    gcmkONERROR(_InitializeContextBuffer(context));

    /* Make all created contexts equal. */
    {
        gcsCONTEXT_PTR currContext, tempContext;

        /* Set the current context buffer. */
        currContext = context->buffer;

        /* Get the next context buffer. */
        tempContext = currContext->next;

        /* Loop through all buffers. */
        while (tempContext != currContext)
        {
            if (tempContext == gcvNULL)
            {
                gcmkONERROR(gcvSTATUS_NOT_FOUND);
            }

            /* Copy the current context. */
            gckOS_MemCopy(
                tempContext->logical,
                currContext->logical,
                context->totalSize
                );

            /* Get the next context buffer. */
            tempContext = tempContext->next;
        }
    }

    /* Return pointer to the gckCONTEXT object. */
    *Context = context;

    /* Success. */
    gcmkFOOTER_ARG("*Context=0x%08X", *Context);
    return gcvSTATUS_OK;

OnError:
    /* Roll back on error. */
    gcmkVERIFY_OK(_DestroyContext(context));

    /* Return the status. */
    gcmkFOOTER();
    return status;
}
#endif

/******************************************************************************\
**
**  gckCONTEXT_Destroy
**
**  Destroy a gckCONTEXT object.
**
**  INPUT:
**
**      gckCONTEXT Context
**          Pointer to an gckCONTEXT object.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCONTEXT_Destroy(
    IN gckCONTEXT Context
    )
{
    gceSTATUS status;

    gcmkHEADER_ARG("Context=%p", Context);

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);

    /* Destroy the context and all related objects. */
    status = _DestroyContext(Context);

    /* Success. */
    gcmkFOOTER_NO();
    return status;
}

/******************************************************************************\
**
**  gckCONTEXT_Update
**
**  Merge all pending state delta buffers into the current context buffer.
**
**  INPUT:
**
**      gckCONTEXT Context
**          Pointer to an gckCONTEXT object.
**
**      gctUINT32 ProcessID
**          Current process ID.
**
**      gcsSTATE_DELTA_PTR StateDelta
**          Pointer to the state delta.
**
**  OUTPUT:
**
**      Nothing.
*/
gceSTATUS
gckCONTEXT_Update(
    IN gckCONTEXT Context,
    IN gctUINT32 ProcessID,
    IN gcsSTATE_DELTA_PTR StateDelta
    )
{
#if gcdENABLE_3D
    gceSTATUS status = gcvSTATUS_OK;
    gcsSTATE_DELTA _stateDelta;
    gckKERNEL kernel;
    gcsCONTEXT_PTR buffer;
    gcsSTATE_MAP_PTR map;
    gctBOOL needCopy = gcvFALSE;
    gcsSTATE_DELTA_PTR uDelta = gcvNULL;
    gcsSTATE_DELTA_PTR kDelta = gcvNULL;
    gcsSTATE_DELTA_RECORD_PTR record;
    gcsSTATE_DELTA_RECORD_PTR recordArray = gcvNULL;
    gctUINT elementCount;
    gctUINT address;
    gctUINT32 mask;
    gctUINT32 data;
    gctUINT index;
    gctUINT i, j;
    gctUINT32 dirtyRecordArraySize = 0;

    gcmkHEADER_ARG(
        "Context=%p ProcessID=%d StateDelta=%p",
        Context, ProcessID, StateDelta
        );

    /* Verify the arguments. */
    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);

    /* Get a shortcut to the kernel object. */
    kernel = Context->hardware->kernel;

    /* Check wehther we need to copy the structures or not. */
    gcmkONERROR(gckOS_QueryNeedCopy(Context->os, ProcessID, &needCopy));

    /* Get the current context buffer. */
    buffer = Context->buffer;

    /* Wait until the context buffer becomes available; this will
       also reset the signal and mark the buffer as busy. */
    gcmkONERROR(gckOS_WaitSignal(
        Context->os, buffer->signal, gcvFALSE, gcvINFINITE
        ));

#if gcmIS_DEBUG(gcdDEBUG_CODE) && defined(gcregContextRegAddrs) && gcdENABLE_3D
    /* Update current context token. */
    buffer->logical[Context->map[gcregContextRegAddrs].index]
        = (gctUINT32)gcmPTR2INT32(Context);
#endif

    if (StateDelta != gcvNULL)
    {
        /* Get the state map. */
        map = Context->map;

        /* Get the first delta item. */
        uDelta = StateDelta;

        /* Reset the vertex stream count. */
        elementCount = 0;

        /* Merge all pending deltas. */
        {
            /* Get access to the state delta. */
            gcmkONERROR(gckKERNEL_OpenUserData(
                kernel, needCopy,
                &_stateDelta,
                uDelta, gcmSIZEOF(gcsSTATE_DELTA),
                (gctPOINTER *) &kDelta
                ));

            dirtyRecordArraySize
                = gcmSIZEOF(gcsSTATE_DELTA_RECORD) * kDelta->recordCount;

            if (dirtyRecordArraySize)
            {
                /* Get access to the state records. */
                gcmkONERROR(gckOS_MapUserPointer(
                    kernel->os,
                    gcmUINT64_TO_PTR(kDelta->recordArray),
                    dirtyRecordArraySize,
                    (gctPOINTER *) &recordArray
                    ));

                if (recordArray == gcvNULL)
                {
                    gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
                }

                /* Merge all pending states. */
                for (j = 0; j < kDelta->recordCount; j += 1)
                {
                    if (j >= Context->numStates)
                    {
                        break;
                    }

                    /* Get the current state record. */
                    record = &recordArray[j];

                    /* Get the state address. */
                    gcmkONERROR(gckOS_ReadMappedPointer(kernel->os, &record->address, &address));

                    /* Make sure the state is a part of the mapping table. */
                    if (address >= Context->maxState)
                    {
                        gcmkTRACE(
                            gcvLEVEL_ERROR,
                            "%s(%d): State 0x%04X (0x%04X) is not mapped.\n",
                            __FUNCTION__, __LINE__,
                            address, address << 2
                            );

                        continue;
                    }

                    /* Get the state index. */
                    index = map[address].index;

                    /* Skip the state if not mapped. */
                    if (index == 0)
                    {
                        continue;
                    }

                    /* Get the data mask. */
                    gcmkONERROR(gckOS_ReadMappedPointer(kernel->os, &record->mask, &mask));

                    /* Get the new data value. */
                    gcmkONERROR(gckOS_ReadMappedPointer(kernel->os, &record->data, &data));

                    /* Masked states that are being completly reset or regular states. */
                    if ((mask == 0) || (mask == ~0U))
                    {
                        /* Process special states. */
                        if (address == gcMemoryConfigRegAddrs)
                        {
                            /* Force auto-disable to be disabled. */
                            data = gcmSETFIELDVALUE(data, GC_MEMORY_CONFIG, AUTO_DISABLE_C,             DISABLE);
                            data = gcmSETFIELDVALUE(data, GC_MEMORY_CONFIG, AUTO_DISABLE_Z,             DISABLE);
                            data = gcmSETFIELDVALUE(data, GC_MEMORY_CONFIG, AUTO_DISABLE_HZ_FAST_CLEAR, DISABLE);
                        }

                        /* Set new data. */
                        buffer->logical[index] = data;
                    }

                    /* Masked states that are being set partially. */
                    else
                    {
                        buffer->logical[index]
                            = (~mask & buffer->logical[index])
                            | ( mask & data);
                    }
                }
            }

            /* Get the element count. */
            if (kDelta->elementCount != 0)
            {
                elementCount = kDelta->elementCount;
            }

            if (dirtyRecordArraySize)
            {
                /* Get access to the state records. */
                gcmkONERROR(gckOS_UnmapUserPointer(
                    kernel->os,
                    gcmUINT64_TO_PTR(kDelta->recordArray),
                    dirtyRecordArraySize,
                    recordArray
                    ));

                recordArray = gcvNULL;
            }

            /* Close access to the current state delta. */
            gcmkONERROR(gckKERNEL_CloseUserData(
                kernel, needCopy,
                gcvTRUE,
                uDelta, gcmSIZEOF(gcsSTATE_DELTA),
                (gctPOINTER *) &kDelta
                ));
        }

        /* Hardware disables all input attribute when the attribute 0 is programmed,
           it then reenables those attributes that were explicitely programmed by
           the software. Because of this we cannot program the entire array of
           values, otherwise we'll get all attributes reenabled, but rather program
           only those that are actully needed by the software.
           elementCount = attribCount + 1 to make sure 0 is a flag to indicate if UMD
           touches it.
        */
        if (elementCount != 0)
        {
            gctUINT base;
            gctUINT nopCount;
            gctUINT32_PTR nop;
            gctUINT fe2vsCount;
            gctUINT attribCount = elementCount -1;
            gctUINT32 feAttributeStatgeAddr = AQVertexElementCtrlRegAddrs;
            if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI5))
            {
                fe2vsCount = 32;
                base = map[gcregFEAttributeRegAddrs].index;
                feAttributeStatgeAddr = gcregFEAttributeRegAddrs;
            }
            else if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI0))
            {
                fe2vsCount = 16;
                base = map[AQVertexElementCtrlRegAddrs].index;
            }
            else
            {
                fe2vsCount = 12;
                base = map[AQVertexElementCtrlRegAddrs].index;
            }

            /* Set the proper state count. */
            if (attribCount == 0)
            {
                gcmkASSERT(gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_ZERO_ATTRIB_SUPPORT));

                buffer->logical[base - 1]
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE , LOAD_STATE)
                         | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT,   NORMAL)
                         | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                         | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, feAttributeStatgeAddr);

                /* Set the proper state count. */
                buffer->logical[base + 1] =
                        gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE , LOAD_STATE)
                        | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT,   NORMAL)
                        | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   1)
                        | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregFEResetAttributesRegAddrs);
                buffer->logical[base + 2] = 0x1;
                attribCount = 3;
            }
            else
            {
                buffer->logical[base - 1]
                    = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE , LOAD_STATE)
                         | gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, FLOAT,   NORMAL)
                         | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT,   attribCount)
                         | gcmSETFIELD     (0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, feAttributeStatgeAddr);
            }

            /* Determine the number of NOP commands. */
            nopCount = (fe2vsCount / 2) - (attribCount / 2);
            /* Determine the location of the first NOP. */
            nop = &buffer->logical[base + (attribCount | 1)];

            /* Fill the unused space with NOPs. */
            for (i = 0; i < nopCount; i += 1)
            {
                if (nop >= buffer->logical + Context->totalSize)
                {
                    break;
                }

                /* Generate a NOP command. */
                *nop = gcmSETFIELDVALUE(0, AQ_COMMAND_NOP_COMMAND, OPCODE, NOP);

                /* Advance. */
                nop += 2;
            }
        }

        if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SMALL_BATCH) &&
            Context->hardware->options.smallBatch)
        {
            gctUINT numConstant = (gctUINT)Context->hardware->identity.numConstants;
            gctUINT32 constCount = 0;

            /* Get the const number after merge. */
            index = map[gcregSHAllocUniformsRegAddrs].index;
            data = buffer->logical[index];
            constCount = gcmGETFIELD(data, GCREG_SH_ALLOC_UNIFORMS, SIZE);

            _UpdateUnifiedReg(Context, gcregGpipeUniformsRegAddrs, numConstant << 2, constCount << 2);
        }

        if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_SMALL_BATCH) &&
            Context->hardware->options.smallBatch)
        {
            gctUINT numSamplers = 80;
            gctUINT32 samplerCount = 0;

            /* Get the sampler number after merge. */
            index = map[gcregSHAllocSamplersRegAddrs].index;
            data = buffer->logical[index];
            samplerCount = gcmGETFIELD(data, GCREG_SH_ALLOC_SAMPLERS, SIZE);

            _UpdateUnifiedReg(Context, gcregTXGpipeSamplerCtrl0RegAddrs, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, gcregTXGpipeSamplerCtrl1RegAddrs, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, gcregTXGpipeSamplerLodMaxMinRegAddrs, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, gcregTXGpipeSamplerLodBiasRegAddrs, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, gcregTXGpipeSamplerAnisoCtrlRegAddrs, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, gcregTXGpipeTextureDescriptorAddrRegAddrs, numSamplers, samplerCount);
            _UpdateUnifiedReg(Context, gcregTXGpipeTextureCtrlRegAddrs, numSamplers, samplerCount);
        }
    }

    /* Schedule an event to mark the context buffer as available. */
    gcmkONERROR(gckEVENT_Signal(
        buffer->eventObj, buffer->signal, gcvKERNEL_PIXEL
        ));

    /* Advance to the next context buffer. */
    Context->buffer = buffer->next;

    /* Return the status. */
    gcmkFOOTER();
    return gcvSTATUS_OK;

OnError:
    /* Get access to the state records. */
    if (kDelta != gcvNULL && recordArray != gcvNULL)
    {
        gcmkVERIFY_OK(gckOS_UnmapUserPointer(
            kernel->os,
            gcmUINT64_TO_PTR(kDelta->recordArray),
            dirtyRecordArraySize,
            (gctPOINTER *) &recordArray
            ));
    }

    /* Close access to the current state delta. */
    gcmkVERIFY_OK(gckKERNEL_CloseUserData(
        kernel, needCopy,
        gcvTRUE,
        uDelta, gcmSIZEOF(gcsSTATE_DELTA),
        (gctPOINTER *) &kDelta
        ));

    /* Return the status. */
    gcmkFOOTER();
    return status;
#else
    return gcvSTATUS_OK;
#endif
}

gceSTATUS
gckCONTEXT_MapBuffer(
    IN gckCONTEXT Context,
    OUT gctUINT64 *Logicals,
    OUT gctUINT32 *Bytes
    )
{
    gceSTATUS status;
    int i = 0;
    gckKERNEL kernel = Context->hardware->kernel;
    gctPOINTER logical;
    gcsCONTEXT_PTR buffer;

    gcmkHEADER_ARG("Context=%p", Context);

    buffer = Context->buffer;

    for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; i++)
    {
        /* Lock for userspace CPU access. */
        gcmkONERROR(gckVIDMEM_NODE_LockCPU(
            kernel,
            buffer->videoMem,
            gcvFALSE,
            gcvTRUE,
            &logical
            ));

        Logicals[i] = gcmPTR_TO_UINT64(logical);
        buffer = buffer->next;
    }

    *Bytes = (gctUINT)Context->totalSize;

    gcmkFOOTER_NO();
    return gcvSTATUS_OK;

OnError:
    gcmkFOOTER();
    return status;
}
