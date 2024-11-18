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


#include "gc_hal.h"
#include "gc_hal_kernel.h"
#include "gc_hal_kernel_context.h"
#include "gc_hal_kernel_buffer.h"

/******************************************************************************\
******************************** Debugging Macro *******************************
\******************************************************************************/

/* Zone used for header/footer. */
#define _GC_OBJ_ZONE    gcvZONE_HARDWARE


/******************************************************************************\
************************** Context State Buffer Helpers ************************
\******************************************************************************/

#define _STATE(reg)                                                            \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        reg ## _Count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_COUNT(reg, count)                                               \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_COUNT_OFFSET(reg, offset, count)                                \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + offset, \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvFALSE                                                     \
        )

#define _STATE_MIRROR_COUNT(reg, mirror, count)                                \
    _StateMirror(\
        Context, \
        reg ## _Address >> 2, \
        count, \
        mirror ## _Address >> 2                                                \
        )

#define _STATE_HINT(reg)                                                       \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        reg ## _Count, \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_HINT_BLOCK(reg, block, count)                                   \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + (block << reg ## _BLK), \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_COUNT_OFFSET_HINT(reg, offset, count)                           \
    _State(\
        Context, index, \
        (reg ## _Address >> 2) + offset, \
        reg ## _ResetValue, \
        count, \
        gcvFALSE, gcvTRUE                                                      \
        )

#define _STATE_X(reg)                                                          \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        reg ## _ResetValue, \
        reg ## _Count, \
        gcvTRUE, gcvFALSE                                                      \
        )

#define _STATE_INIT_VALUE(reg, value)                                          \
    _State(\
        Context, index, \
        reg ## _Address >> 2, \
        value, \
        reg ## _Count, \
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
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to BLT. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }
            else
            {
                /* Semaphore from FE to PE. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to PE. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }
        }

        /* Flush the current pipe. */
        *buffer++
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E03) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

        *buffer++
            = (Pipe == gcvPIPE_2D)
                ?
   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 3:3) - (0 ?
 3:3) + 1))))))) << (0 ?
 3:3))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 3:3) - (0 ?
 3:3) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 3:3) - (0 ? 3:3) + 1))))))) << (0 ? 3:3)))
                :   ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 1:1) - (0 ?
 1:1) + 1))))))) << (0 ?
 1:1))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 1:1) - (0 ?
 1:1) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 1:1) - (0 ? 1:1) + 1))))))) << (0 ? 1:1)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:2) - (0 ?
 2:2) + 1))))))) << (0 ?
 2:2))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 2:2) - (0 ?
 2:2) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 2:2) - (0 ? 2:2) + 1))))))) << (0 ? 2:2)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 10:10) - (0 ?
 10:10) + 1))))))) << (0 ?
 10:10))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 10:10) - (0 ?
 10:10) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 10:10) - (0 ? 10:10) + 1))))))) << (0 ? 10:10)))
                  | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 11:11) - (0 ?
 11:11) + 1))))))) << (0 ?
 11:11))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 11:11) - (0 ?
 11:11) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 11:11) - (0 ? 11:11) + 1))))))) << (0 ? 11:11)));

        if (hwTFB)
        {
             *buffer++
                 = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                 | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x7003) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                 | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

             *buffer++
                 = 0x12345678;
        }

        /* Flush VST in separate cmd. */
        if (Pipe == gcvPIPE_3D)
        {
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E03) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++ = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
        }

        /* Semaphore from FE to PE. */
        if (blt)
        {
            /* Semaphore from FE to BLT. */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

            /* Stall from FE to BLT. */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
        }
        else
        {
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

            /* Stall from FE to PE. */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
        }

        if (fcFlushStall)
        {
            if (!peTSFlush && blt)
            {
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502B) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }
            else
            {
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0594) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }

            /* Semaphore from FE to PE. */
            if (blt)
            {
                /* Semaphore from FE to BLT. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to BLT. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }
            else
            {
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to PE. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }
        }

        if (iCacheInvalidate && !halti5)
        {
            /* Invalidate I$ after pipe is stalled */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0218) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x021A) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0218) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x021A) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));

            /* Semaphore from FE to PE. */
            if (blt)
            {
                /* Semaphore from FE to BLT. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to BLT. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
            }
            else
            {
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

                /* Stall from FE to PE. */
                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                *buffer++
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
            }
        }

        if (snapPages)
        {
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x13 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x02 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x04 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)));

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
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

            /* Stall from FE to BLT. */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x10 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x502E) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 0:0) - (0 ?
 0:0) + 1))))))) << (0 ?
 0:0))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 0:0) - (0 ?
 0:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 0:0) - (0 ? 0:0) + 1))))))) << (0 ? 0:0)));
        }
        else
        {
            /* Semaphore from FE to PE. */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E02) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));

            /* Stall from FE to PE. */
            *buffer++
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x09 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

            *buffer
                = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:0) - (0 ?
 4:0) + 1))))))) << (0 ?
 4:0))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 4:0) - (0 ?
 4:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:0) - (0 ? 4:0) + 1))))))) << (0 ? 4:0)))
                | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 12:8) - (0 ?
 12:8) + 1))))))) << (0 ?
 12:8))) | (((gctUINT32) (0x07 & ((gctUINT32) ((((1 ?
 12:8) - (0 ?
 12:8) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 12:8) - (0 ? 12:8) + 1))))))) << (0 ? 12:8)));
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
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x0E00) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

        *buffer
            = (Pipe == gcvPIPE_2D)
                ? 0x1
                : 0x0;
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
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (Size) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (Address) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));
            }
            else
            {
                buffer[Index]
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (Size) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                    | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (Address) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));
            }

            /* Walk all the states. */
            for (i = 0; i < (gctUINT32)Size; i += 1)
            {
                /* Set state to uninitialized value. */
                buffer[Index + 1 + i] = Value;

                /* Set index in state mapping table. */
                Context->map[Address + i].index = (gctUINT)Index + 1 + i;

#if gcdSECURE_USER
                /* Save hint. */
                if (Context->hint != gcvNULL)
                {
                    Context->hint[Address + i] = Hinted;
                }
#endif
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
            ((((gctUINT32) (buffer[Context->lastIndex])) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (Context->lastSize + Size) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)));

        /* Walk all the states. */
        for (i = 0; i < (gctUINT32)Size; i += 1)
        {
            /* Set state to uninitialized value. */
            buffer[Index + i] = Value;

            /* Set index in state mapping table. */
            Context->map[Address + i].index = (gctUINT)Index + i;

#if gcdSECURE_USER
            /* Save hint. */
            if (Context->hint != gcvNULL)
            {
                Context->hint[Address + i] = Hinted;
            }
#endif
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

#if gcdSECURE_USER
            Context->hint[Address + i] =
                Context->hint[AddressMirror + i];
#endif
        }
    }

    /* Return the number of required maps. */
    return Size;
}
#endif

#if (gcdENABLE_3D || gcdENABLE_2D)
static gceSTATUS
_InitializeContextBuffer(
    IN gckCONTEXT Context
    )
{
    gctUINT32_PTR buffer;
    gctUINT32 index;

#if gcdENABLE_3D
    gctBOOL halti0, halti1, halti2, halti3, halti4, halti5;
    gctUINT i;
    gctUINT vertexUniforms, fragmentUniforms, vsConstBase, psConstBase, constMax;
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
    gctBOOL multiCoreBlockSetCfg2;
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
    multiCoreBlockSetCfg2 = gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_MULTI_CORE_BLOCK_SET_CONFIG2);

    /* Multi render target. */
    if (halti2 ||
        (Context->hardware->identity.chipModel == gcv900 && Context->hardware->identity.chipRevision == 0x5250)
       )
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
    {if (Context->hardware->identity.numConstants > 256){    unifiedUniform = gcvTRUE;
if (halti5){    vsConstBase  = 0xD000;
    psConstBase  = 0xD800;
}else{    vsConstBase  = 0xC000;
    psConstBase  = 0xC000;
}if ((Context->hardware->identity.chipModel == gcv880) && ((Context->hardware->identity.chipRevision & 0xfff0) == 0x5120)){    vertexUniforms   = 512;
    fragmentUniforms   = 64;
    constMax     = 576;
}else{    vertexUniforms   = gcmMIN(512, Context->hardware->identity.numConstants - 64);
    fragmentUniforms   = gcmMIN(512, Context->hardware->identity.numConstants - 64);
    constMax     = Context->hardware->identity.numConstants;
}}else if (Context->hardware->identity.numConstants == 256){    if (Context->hardware->identity.chipModel == gcv2000 && (Context->hardware->identity.chipRevision == 0x5118 || Context->hardware->identity.chipRevision == 0x5140))    {        unifiedUniform = gcvFALSE;
        vsConstBase  = 0x1400;
        psConstBase  = 0x1C00;
        vertexUniforms   = 256;
        fragmentUniforms   = 64;
        constMax     = 320;
    }    else    {        unifiedUniform = gcvFALSE;
        vsConstBase  = 0x1400;
        psConstBase  = 0x1C00;
        vertexUniforms   = 256;
        fragmentUniforms   = 256;
        constMax     = 512;
    }}else{    unifiedUniform = gcvFALSE;
    vsConstBase  = 0x1400;
    psConstBase  = 0x1C00;
    vertexUniforms   = 168;
    fragmentUniforms   = 64;
    constMax     = 232;
}};


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

    /* Current context pointer. */
#if gcdDEBUG
    index += _State(Context, index, 0x03850 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
#endif

    index += _FlushPipe(Context, index, gcvPIPE_3D);

    /* Global states. */
    if (hasSecurity)
    {
        index += _State(Context, index, 0x03900 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
        index += _State(Context, index, 0x03904 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    index += _State(Context, index, 0x03814 >> 2, 0x00000001, 1, gcvFALSE, gcvFALSE);
    index += _CLOSE_RANGE();
    index += _State(Context, index, 0x03818 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0381C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (halti5)
    {
        index += _State(Context, index, 0x03888 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x038C0 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x03884 >> 2, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | (((gctUINT32) ((gctUINT32) (hardware->options.uscL1CacheRatio) & ((gctUINT32) ((((1 ?
 2:0) - (0 ?
 2:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 2:0) - (0 ?
 2:0) + 1))))))) << (0 ?
 2:0))) | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 20:16) - (0 ?
 20:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:16) - (0 ?
 20:16) + 1))))))) << (0 ?
 20:16))) | (((gctUINT32) ((gctUINT32) (2) & ((gctUINT32) ((((1 ?
 20:16) - (0 ?
 20:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 20:16) - (0 ? 20:16) + 1))))))) << (0 ? 20:16))), 1, gcvFALSE, gcvFALSE);
    }
    else
    {
        index += _State(Context, index, 0x03820 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x03828 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0382C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x03834 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x03838 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x03854 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (hasGS)
    {
        index += _State(Context, index, 0x0388C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    index += _State(Context, index, 0x0384C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    /* Front End states. */
    if (halti5)
    {
        index += _State(Context, index, 0x17800 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
        index += _State(Context, index, 0x007C4 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x17880 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x17900 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x17980 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x17A00 >> 2, 0x3F800000, 32, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x007D0 >> 2, 0x00000000, 2, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x007D8 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x17A80 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
    }
    else
    {
        index += _State(Context, index, 0x00600 >> 2, 0x00000000, (halti0 ? 16 : 12), gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
        if (genericAttrib)
        {
            index += _State(Context, index, 0x006C0 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x00700 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x00740 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x00780 >> 2, 0x3F800000, 16, gcvFALSE, gcvFALSE);
        }
    }

    if (halti2 || (Context->hardware->identity.streamCount > 8))
    {
        index += _State(Context, index, 0x14600 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14640 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14680 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
    }
    else if (Context->hardware->identity.streamCount > 1)
    {
        index += _State(Context, index, 0x00680 >> 2, 0x00000000, 8, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x006A0 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
    }
    else
    {
        index += _State(Context, index, 0x0064C >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x00650 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    }
    index += _State(Context, index, 0x00644 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x00648 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00674 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00678 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0067C >> 2, 0xFFFFFFFF, 1, gcvFALSE, gcvFALSE);
    index += _CLOSE_RANGE();

    if (hasRobustness)
    {
        index += _State(Context, index, 0x146C0 >> 2, 0x00000000, 16, gcvFALSE, gcvTRUE);
        index += _CLOSE_RANGE();
        index += _State(Context, index, 0x007F8 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _CLOSE_RANGE();
    }

    if (halti5)
    {
        index += _State(Context, index, 0x008B8 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x15600 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }
    else
    {
        /* This register is programed by all chips, which program all DECODE_SELECT as VS
        ** except SAMPLER_DECODE_SELECT.
        */
        index += _State(Context, index, 0x00860 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (hasICache)
    {
        /* I-Cache states. */
        index += _State(Context, index, 0x00868 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0086C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0304C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01028 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();

        if (hasICachePrefetch)
        {
            if (halti5)
            {
                index += _State(Context, index, 0x15604 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
                index += _State(Context, index, 0x01094 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
            }
            else
            {
                index += _State(Context, index, 0x00890 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
                index += _State(Context, index, 0x0104C >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
            }
            index += _CLOSE_RANGE();
        }
    }

    /* Vertex Shader states. */
    index += _State(Context, index, 0x00804 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00808 >> 2, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ?
 5:0) + 1))))))) << (0 ?
 5:0))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 5:0) - (0 ?
 5:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:0) - (0 ? 5:0) + 1))))))) << (0 ? 5:0))), 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0080C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00830 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (halti5)
    {
        index += _State(Context, index, 0x00898 >> 2, 0x00000000, 2, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x008A0 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00870 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x008A8 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x008C0 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x008E0 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
    }
    else
    {
        index += _State(Context, index, 0x00810 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00820 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
    }

    index += _CLOSE_RANGE();

    /* GS */
    if (hasGS)
    {
        index += _State(Context, index, 0x01100 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01104 >> 2, 0x00000001, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01108 >> 2, 0x01000001, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0110C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01110 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01114 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x0111C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01140 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01144 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01148 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0114C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01154 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01120 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
    }

    /* TCS & TES */

    if (hasTS)
    {
        index += _State(Context, index, 0x007C0 >> 2, 0x00000003, 1, gcvFALSE, gcvFALSE);

        index += _State(Context, index, 0x14A14 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A18 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A1C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A40 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A00 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A04 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A08 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x14A10 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A20 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A44 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14A4C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

        index += _CLOSE_RANGE();

        index += _State(Context, index, 0x14B18 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B1C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B20 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B04 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B08 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B0C >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x14B14 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B40 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B24 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B2C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14B34 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

        index += _State(Context, index, 0x14B00 >> 2, 0x00040000, 1, gcvFALSE, gcvFALSE);

    }

    index += _CLOSE_RANGE();

    /* TFB */
    if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HW_TFB))
    {
        index += _State(Context, index, 0x1C000 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x1C008 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x1C040 >> 2) + (0 << 4), 0x00000000, 4, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x1C080 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x1C0C0 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x1C100 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x1C800 >> 2, 0x00000000, 128*4, gcvFALSE, gcvFALSE);

        index += _State(Context, index, 0x1C014 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _CLOSE_RANGE();

    }

    /* Primitive Assembly states. */
    index += _State(Context, index, 0x00A00 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00A04 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00A08 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A0C >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00A10 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00A14 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A18 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A1C >> 2, 0x3F000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A28 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A2C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A30 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A34 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A38 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A3C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A80 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A84 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00A8C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00A88 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (halti5)
    {
        index += _State(Context, index, 0x00AA8 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00A90 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
    }
    else
    {
        index += _State(Context, index, 0x00A40 >> 2, 0x00000000, Context->hardware->identity.varyingsCount, gcvFALSE, gcvFALSE);
    }

    index += _State(Context, index, 0x03A00 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x03A04 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x03A08 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (multiCoreBlockSetCfg2)
    {
        index += _State(Context, index, 0x03A0C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x03A10 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    /* Setup states. */
    index += _State(Context, index, 0x00C00 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00C04 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00C08 >> 2, 0x45000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00C0C >> 2, 0x45000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00C10 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00C14 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00C18 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00C1C >> 2, 0x42000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00C20 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);
    index += _State(Context, index, 0x00C24 >> 2, 0x00000000, 1, gcvTRUE, gcvFALSE);

    /* Raster states. */
    index += _State(Context, index, 0x00E00 >> 2, 0x000000F1, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00E10 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00E04 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00E40 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00E08 >> 2, 0x17000031, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00E24 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00E20 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (halti2)
    {
        index += _State(Context, index, 0x00E0C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (halti5)
    {
        index += _State(Context, index, 0x00E34 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }


    /* Pixel Shader states. */
    index += _State(Context, index, 0x01004 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01008 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0100C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01010 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01030 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01034 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (halti2)
    {
        index += _State(Context, index, 0x01040 >> 2, 0x00000000, 2, gcvFALSE, gcvFALSE);
    }

    if (numRT == 8)
    {
        index += _State(Context, index, 0x0102C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01038 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (halti4)
    {
        index += _State(Context, index, 0x01054 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (halti5)
    {
        index += _State(Context, index, 0x01080 >> 2, 0x00000000, 4, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01058 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01098 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }


    index += _CLOSE_RANGE();

    /* Texture states. */
    if (hasTXdesc)
    {
        /* Texture descriptor states */
        index += _State(Context, index, 0x14C40 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

        index += _State(Context, index, 0x16C00 >> 2, 0x00000000, numSamplers, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x16E00 >> 2, 0x00000000, numSamplers, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x17000 >> 2, 0x00000000, numSamplers, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x17200 >> 2, 0x00000000, numSamplers, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x17400 >> 2, 0x00000000, numSamplers, gcvFALSE, gcvFALSE);

        index += _State(Context, index, (0x15C00 >> 2) + (0 << 0), 0x00000000, numSamplers, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x15E00 >> 2, 0x00000000, numSamplers, gcvFALSE, gcvFALSE);

        index += _CLOSE_RANGE();

        _StateMirror(Context, 0x16000 >> 2, numSamplers , 0x16C00 >> 2);
        _StateMirror(Context, 0x16200 >> 2, numSamplers , 0x16E00 >> 2);
        _StateMirror(Context, 0x16400 >> 2, numSamplers , 0x17000 >> 2);
        _StateMirror(Context, 0x16600 >> 2, numSamplers , 0x17200 >> 2);
        _StateMirror(Context, 0x16800 >> 2, numSamplers , 0x17400 >> 2);
        _StateMirror(Context, 0x15800 >> 2, numSamplers , 0x15C00 >> 2);
        _StateMirror(Context, 0x15A00 >> 2, numSamplers , 0x15E00 >> 2);
    }
    else
    {
        index += _State(Context, index, 0x02000 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02040 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02080 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x020C0 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02100 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02140 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02180 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x021C0 >> 2, 0x00321000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02200 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x02240 >> 2, 0x00000000, 12, gcvFALSE, gcvFALSE);
        index += _State(Context, index, (0x02400 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02440 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02480 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x024C0 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02500 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02540 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02580 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x025C0 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02600 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02640 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02680 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x026C0 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02700 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x02740 >> 2) + (0 << 4), 0x00000000, 12, gcvFALSE, gcvTRUE);
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
                index += _State(Context, index, (0x02C00 >> 2) + i * 16, 0x00000000, count, gcvFALSE, gcvFALSE);
            }
        }

        if (halti1)
        {
            gctUINT texBlockCount;
            gctUINT gcregTXLogSizeResetValue;

            /* Enable the integer filter pipe for all texture samplers
               so that the floating point filter clock will shut off until
               we start using the floating point filter.
            */
            gcregTXLogSizeResetValue = ((((gctUINT32) (0x00000000)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 29:29) - (0 ?
 29:29) + 1))))))) << (0 ?
 29:29))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 29:29) - (0 ?
 29:29) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 29:29) - (0 ? 29:29) + 1))))))) << (0 ? 29:29)));

            /* New texture block. */
            index += _State(Context, index, 0x10000 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10080 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10100 >> 2, gcregTXLogSizeResetValue, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10180 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10200 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10280 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10300 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10380 >> 2, 0x00321000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10400 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10480 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);

            if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_FILTER))
            {
                index += _State(Context, index, 0x12000 >> 2, 0x00000000, 256, gcvFALSE, gcvFALSE);
                index += _State(Context, index, 0x12400 >> 2, 0x00000000, 256, gcvFALSE, gcvFALSE);
            }

            texBlockCount = ((512) >> (4));

            for (i = 0; i < texBlockCount; i += 1)
            {
                index += _State(Context, index, (0x10800 >> 2) + (i << 4), 0x00000000, 14, gcvFALSE, gcvTRUE);
            }
        }

        if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_TEX_BASELOD))
        {
            index += _State(Context, index, 0x10700 >> 2, 0x00000F00, 32, gcvFALSE, gcvFALSE);
        }

        if (halti3 ||
            gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_SUPPORT_DEC))
        {
            index += _State(Context, index, 0x10780 >> 2, 0x00030000, 32, gcvFALSE, gcvFALSE);
        }

        if (halti4)
        {
            index += _State(Context, index, 0x11200 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x11280 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
        }

        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TX_FRAC_PRECISION_6BIT))
        {
            index += _State(Context, index, 0x11000 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x11080 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x11100 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x11180 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x11300 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
        }

        /* ASTC */
        if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_TEXTURE_ASTC))
        {
            index += _State(Context, index, 0x10500 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10580 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10600 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x10680 >> 2, 0x00000000, 32, gcvFALSE, gcvFALSE);
        }
    }

    if (halti3)
    {
        index += _State(Context, index, 0x14C00 >> 2, 0x00000000, 16, gcvFALSE, gcvFALSE);
    }

    /* Thread walker states. */
    index += _State(Context, index, 0x00900 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00904 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00908 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0090C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00910 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00914 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00918 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x00924 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0091C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_SHADER_ENHANCEMENTS2))
    {
        index += _State(Context, index, 0x00940 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00944 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00948 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0094C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00950 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00954 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (halti5)
    {
        index += _State(Context, index, 0x00958 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0095C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00960 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    index += _CLOSE_RANGE();

    /* VS/PS Start/End PC register */
    if (halti5)
    {
        index += _State(Context, index, 0x00874 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x008BC >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0087C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x01090 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
    }
    else if (hasICache)
    {
        /* New Shader instruction PC registers(20bit). */
        index += _State(Context, index, 0x00874 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00878 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x0087C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00880 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
    }
    else
    {
        if (Context->hardware->identity.instructionCount <= 256)
        {
            /* old shader instruction PC registers (12bit)*/
            index += _State(Context, index, 0x00800 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x00838 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
            index += _CLOSE_RANGE();

            index += _State(Context, index, 0x01000 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x01018 >> 2, 0x01000000, 1, gcvFALSE, gcvFALSE);
            index += _CLOSE_RANGE();
        }
        else
        {
            /* New Shader instruction PC registers (16bit) */
            index += _State(Context, index, 0x0085C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
            index += _State(Context, index, 0x0101C >> 2, 0x00000100, 1, gcvFALSE, gcvFALSE);
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
                index += _State(Context, index, (0x20000 >> 2) + i, 0x00000000, 256 << 2, gcvFALSE, gcvFALSE);
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
                index += _State(Context, index, (0x0C000 >> 2) + i, 0x00000000, 256 << 2, gcvFALSE, gcvFALSE);
                index += _CLOSE_RANGE();
            }

            _StateMirror(Context, 0x08000 >> 2, Context->hardware->identity.instructionCount << 2 , 0x0C000 >> 2);
        }
        /* if (Context->hardware->identity.instructionCount <= 256). This is non-unified one. */
        else
        {
            index += _State(Context, index, 0x04000 >> 2, 0x00000000, 1024, gcvFALSE, gcvFALSE);
            index += _CLOSE_RANGE();
            index += _State(Context, index, 0x06000 >> 2, 0x00000000, 1024, gcvFALSE, gcvFALSE);
            index += _CLOSE_RANGE();
        }
    }

    if (unifiedUniform)
    {
        gctINT numConstants = Context->hardware->identity.numConstants;

        /* Base Offset register */
        index += _State(Context, index, 0x01024 >> 2, 0x00000100, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x00864 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();

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
                    index += _State(Context, index, (0x36000 >> 2) + i, 0x00000000, 256 << 2, gcvFALSE, gcvFALSE);
                }
                else
                {
                    index += _State(Context, index, (0x36000 >> 2) + i, 0x00000000, numConstants << 2, gcvFALSE, gcvFALSE);
                }
                index += _CLOSE_RANGE();
            }
            else
            {
                if (numConstants >= 256)
                {
                    index += _State(Context, index, (0x30000 >> 2) + i, 0x00000000, 256 << 2, gcvFALSE, gcvFALSE);
                }
                else
                {
                    index += _State(Context, index, (0x30000 >> 2) + i, 0x00000000, numConstants << 2, gcvFALSE, gcvFALSE);
                }

                index += _CLOSE_RANGE();
            }
        }

        if (halti5)
        {
            _StateMirror(Context, 0x34000 >> 2, Context->hardware->identity.numConstants << 2 , 0x36000 >> 2);
        }
    }
#if gcdENABLE_UNIFIED_CONSTANT
    else
#endif
    {
        index += _State(Context, index, 0x05000 >> 2, 0x00000000, vertexUniforms * 4, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x07000 >> 2, 0x00000000, fragmentUniforms * 4, gcvFALSE, gcvFALSE);
    }

    if (halti1)
    {
        index += _State(Context, index, 0x00884 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (halti5)
    {
        index += _State(Context, index, 0x008B0 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    /* Store the index of the "XD" entry. */
    Context->entryOffsetXDFrom3D = (gctUINT)index * gcmSIZEOF(gctUINT32);


    /* Pixel Engine states. */
    index += _State(Context, index, 0x01400 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01404 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01408 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0140C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01414 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01418 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0141C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01420 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01424 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01428 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x0142C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01434 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01454 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01458 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x014A0 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x014A8 >> 2, 0xFFFFFFFF, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x014AC >> 2, 0xFFFFFFFF, 1, gcvFALSE, gcvFALSE);

    if(gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_HALF_FLOAT_PIPE) )
    {
        index += _State(Context, index, 0x014B0 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x014B4 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }
    index += _State(Context, index, 0x014A4 >> 2, 0x000E400C, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01580 >> 2, 0x00000000, 3, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x014B8 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);

    if (halti3)
    {
        index += _State(Context, index, 0x0103C >> 2, 0x76543210, 1, gcvFALSE, gcvFALSE);
    }

    index += _State(Context, index, (0x01460 >> 2) + (0 << 3), 0x00000000, Context->hardware->identity.pixelPipes, gcvFALSE, gcvTRUE);

    if (Context->hardware->identity.pixelPipes == 1)
    {
        index += _State(Context, index, 0x01430 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x01410 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    }

    if (Context->hardware->identity.pixelPipes > 1 || halti0)
    {
        index += _State(Context, index, (0x01480 >> 2) + (0 << 3), 0x00000000, Context->hardware->identity.pixelPipes, gcvFALSE, gcvTRUE);
    }

    for (i = 0; i < 3; i++)
    {
        index += _State(Context, index, (0x01500 >> 2) + (i << 3), 0x00000000, Context->hardware->identity.pixelPipes, gcvFALSE, gcvTRUE);
    }

    if (numRT == 8)
    {
        for (i = 0; i < 7; i++)
        {
          index += _State(Context, index, (0x14800 >> 2) + (i << 3), 0x00000000, Context->hardware->identity.pixelPipes, gcvFALSE, gcvTRUE);
        }
        index += _State(Context, index, 0x14900 >> 2, 0x00000000, 7, gcvFALSE, gcvFALSE);
    }


    if (halti3)
    {
        index += _State(Context, index, 0x014BC >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (halti4)
    {
        index += _State(Context, index, 0x014C0 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    }

    if (hasGS)
    {
        index += _State(Context, index, 0x038A0 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
    }

    if (halti5)
    {
        index += _State(Context, index, 0x14920 >> 2, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14940 >> 2, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14960 >> 2, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x14980 >> 2, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x149A0 >> 2, 0x00000000, 7, gcvFALSE, gcvFALSE);
    }

    if (hasRobustness)
    {
        index += _State(Context, index, 0x149C0 >> 2, 0x00000000, 8, gcvFALSE, gcvTRUE);
        index += _State(Context, index, 0x014C4 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    }

    /* Memory Controller */
    index += _State(Context, index, 0x01654 >> 2, 0x00200000, 1, gcvFALSE, gcvFALSE);

    index += _CLOSE_RANGE();
    index += _State(Context, index, 0x01658 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x0165C >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x01660 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01664 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x01668 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x0166C >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01670 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01674 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x016A4 >> 2, 0x00000000, 1, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x016AC >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x016A8 >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01720 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
    index += _State(Context, index, 0x01740 >> 2, 0x00000000, 8, gcvFALSE, gcvTRUE);
    index += _State(Context, index, 0x01760 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);


    if (halti2)
    {
        index += _State(Context, index, 0x01780 >> 2, 0x00000000, 8, gcvFALSE, gcvFALSE);
        index += _State(Context, index, 0x016BC >> 2, 0x00000000, 1, gcvFALSE, gcvFALSE);
        index += _State(Context, index, (0x017A0 >> 2) + 1, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, (0x017C0 >> 2) + 1, 0x00000000, 7, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x017E0 >> 2) + 1, 0x00000000, 7, gcvFALSE, gcvTRUE);
        index += _State(Context, index, (0x01A00 >> 2) + 1, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, (0x01A20 >> 2) + 1, 0x00000000, 7, gcvFALSE, gcvFALSE);
        index += _State(Context, index, (0x01A40 >> 2) + 1, 0x00000000, 7, gcvFALSE, gcvFALSE);
    }

    index += _CLOSE_RANGE();

    if (gckHARDWARE_IsFeatureAvailable(hardware, gcvFEATURE_BUG_FIXES18))
    {
        index += _State(Context, index, 0x03860 >> 2, 0x6, 1, gcvFALSE, gcvFALSE);
        index += _CLOSE_RANGE();
    }

    if (halti3)
    {
        index += _State(Context, index, 0x01A80 >> 2, 0x00000000, 8, gcvFALSE, gcvTRUE);
        index += _CLOSE_RANGE();
    }

    if (hasSecurity || hasRobustness)
    {
        index += _State(Context, index, 0x001AC >> 2, ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ?
 16:16) + 1))))))) << (0 ?
 16:16))) | (((gctUINT32) (0x1 & ((gctUINT32) ((((1 ?
 16:16) - (0 ?
 16:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 16:16) - (0 ? 16:16) + 1))))))) << (0 ? 16:16))), 1, gcvFALSE, gcvFALSE);
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
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

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
            = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x08 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
            | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

        buffer[index + 1]
            = 0;
    }

    index += 2;


    /**************************************************************************/
    /* Save size for buffer. **************************************************/

    Context->totalSize = index * gcmSIZEOF(gctUINT32);

#if gcdENABLE_3D
    psConstBase = psConstBase;
    vsConstBase = vsConstBase;
    constMax = constMax;
#endif

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
                gcmkONERROR(gckOS_DestroySignal(
                    Context->os, buffer->signal
                    ));

                buffer->signal = gcvNULL;
            }

            /* Free state delta map. */
            if (buffer->logical != gcvNULL)
            {
                if (Context->hardware->kernel->virtualCommandBuffer)
                {
                    gcmkONERROR(gckEVENT_DestroyVirtualCommandBuffer(
                        Context->hardware->kernel->eventObj,
                        Context->totalSize,
                        buffer->physical,
                        buffer->logical,
                        gcvKERNEL_PIXEL
                        ));
                }
                else
                {
                    gcmkONERROR(gckEVENT_FreeContiguousMemory(
                        Context->hardware->kernel->eventObj,
                        Context->totalSize,
                        buffer->physical,
                        buffer->logical,
                        gcvKERNEL_PIXEL
                        ));
                }

                buffer->logical = gcvNULL;
            }

            /* Free context buffer. */
            gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, buffer));

            /* Remove from the list. */
            Context->buffer = next;
        }

#if gcdSECURE_USER
        /* Free the hint array. */
        if (Context->hint != gcvNULL)
        {
            gcmkONERROR(gcmkOS_SAFE_FREE(Context->os, Context->hint));
        }
#endif

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
    gctPOINTER pointer;
    gctUINT32 address;
    gctSIZE_T totalSize = Context->totalSize;

    if (Context->hardware->kernel->virtualCommandBuffer)
    {
        gcmkONERROR(gckKERNEL_AllocateVirtualCommandBuffer(
            Context->hardware->kernel,
            gcvFALSE,
            &totalSize,
            &Buffer->physical,
            &pointer
            ));

        gcmkONERROR(gckKERNEL_GetGPUAddress(
            Context->hardware->kernel,
            pointer,
            gcvFALSE,
            Buffer->physical,
            &address
            ));
    }
    else
    {
        gctUINT32 allocFlag;

#if gcdENABLE_CACHEABLE_COMMAND_BUFFER
        allocFlag = gcvALLOC_FLAG_CACHEABLE | gcvALLOC_FLAG_CONTIGUOUS;
#else
        allocFlag = gcvALLOC_FLAG_CONTIGUOUS;
#endif
        gcmkONERROR(gckOS_AllocateNonPagedMemory(
            Context->os,
            gcvFALSE,
            allocFlag,
            &totalSize,
            &Buffer->physical,
            &pointer
            ));

        gcmkONERROR(gckHARDWARE_ConvertLogical(
            Context->hardware,
            pointer,
            gcvFALSE,
            &address
            ));
    }

    Buffer->logical = pointer;
    Buffer->address = address;

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

    gcmkHEADER_ARG("Os=0x%08X Hardware=0x%08X", Os, Hardware);

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
        = (((((gctUINT32) (context->hardware->identity.chipFeatures)) >> (0 ? 9:9)) & ((gctUINT32) ((((1 ? 9:9) - (0 ? 9:9) + 1) == 32) ? ~0U : (~(~0U << ((1 ? 9:9) - (0 ? 9:9) + 1)))))) )
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

        /**************************************************************************/
        /* Allocate the hint array. ***********************************************/

#if gcdSECURE_USER
        /* Allocate hints. */
        gcmkONERROR(gckOS_Allocate(
            Os,
            gcmSIZEOF(gctBOOL) * context->maxState,
            &pointer
            ));

        context->hint = pointer;
#endif
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
            gcmkONERROR(gckHARDWARE_Link(
                Hardware, gcvNULL, 0, 0, &linkBytes, gcvNULL, gcvNULL
                ));

            /* Generate a LINK. */
            gcmkONERROR(gckHARDWARE_Link(
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

    gcmkHEADER_ARG("Context=0x%08X", Context);

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

#if gcdSECURE_USER
    gcskSECURE_CACHE_PTR cache;
#endif

    gcmkHEADER_ARG(
        "Context=0x%08X ProcessID=%d StateDelta=0x%08X",
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

#if gcdSECURE_USER
    /* Get the cache form the database. */
    gcmkONERROR(gckKERNEL_GetProcessDBCache(kernel, ProcessID, &cache));
#endif

#if gcmIS_DEBUG(gcdDEBUG_CODE) && 1 && gcdENABLE_3D
    /* Update current context token. */
    buffer->logical[Context->map[0x0E14].index]
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
                    if (address == 0x0595)
                    {
                        /* Force auto-disable to be disabled. */
                        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 5:5) - (0 ?
 5:5) + 1))))))) << (0 ?
 5:5))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 5:5) - (0 ?
 5:5) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 5:5) - (0 ? 5:5) + 1))))))) << (0 ? 5:5)));
                        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 4:4) - (0 ?
 4:4) + 1))))))) << (0 ?
 4:4))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 4:4) - (0 ?
 4:4) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 4:4) - (0 ? 4:4) + 1))))))) << (0 ? 4:4)));
                        data = ((((gctUINT32) (data)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 13:13) - (0 ?
 13:13) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 13:13) - (0 ?
 13:13) + 1))))))) << (0 ?
 13:13))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 13:13) - (0 ?
 13:13) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 13:13) - (0 ? 13:13) + 1))))))) << (0 ? 13:13)));
                    }

#if gcdSECURE_USER
                    /* Do we need to convert the logical address? */
                    if (Context->hint[address])
                    {
                        /* Map handle into physical address. */
                        gcmkONERROR(gckKERNEL_MapLogicalToPhysical(
                            kernel, cache, (gctPOINTER) &data
                            ));
                    }
#endif

                    /* Set new data. */
                    buffer->logical[index] = data;
                }

                /* Masked states that are being set partially. */
                else
                {
                    buffer->logical[index]
                        = (~mask & buffer->logical[index])
                        | (mask & data);
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
            gctUINT32 feAttributeStatgeAddr = 0x0180;
            if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI5))
            {
                fe2vsCount = 32;
                base = map[0x5E00].index;
                feAttributeStatgeAddr = 0x5E00;
            }
            else if (gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_HALTI0))
            {
                fe2vsCount = 16;
                base = map[0x0180].index;
            }
            else
            {
                fe2vsCount = 12;
                base = map[0x0180].index;
            }

            /* Set the proper state count. */
            if (attribCount == 0)
            {
                gcmkASSERT(gckHARDWARE_IsFeatureAvailable(Context->hardware, gcvFEATURE_ZERO_ATTRIB_SUPPORT));

                buffer->logical[base - 1]
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (feAttributeStatgeAddr) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));

                /* Set the proper state count. */
                buffer->logical[base + 1] =
                        ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (1) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                        | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (0x01F2) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));
                buffer->logical[base + 2] = 0x1;
                attribCount = 3;
            }
            else
            {
                buffer->logical[base - 1]
                    = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x01 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 26:26) - (0 ?
 26:26) + 1))))))) << (0 ?
 26:26))) | (((gctUINT32) (0x0 & ((gctUINT32) ((((1 ?
 26:26) - (0 ?
 26:26) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 26:26) - (0 ? 26:26) + 1))))))) << (0 ? 26:26)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 25:16) - (0 ?
 25:16) + 1))))))) << (0 ?
 25:16))) | (((gctUINT32) ((gctUINT32) (attribCount) & ((gctUINT32) ((((1 ?
 25:16) - (0 ?
 25:16) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 25:16) - (0 ? 25:16) + 1))))))) << (0 ? 25:16)))
                         | ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 15:0) - (0 ?
 15:0) + 1))))))) << (0 ?
 15:0))) | (((gctUINT32) ((gctUINT32) (feAttributeStatgeAddr) & ((gctUINT32) ((((1 ?
 15:0) - (0 ?
 15:0) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 15:0) - (0 ? 15:0) + 1))))))) << (0 ? 15:0)));
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
                *nop = ((((gctUINT32) (0)) & ~(((gctUINT32) (((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ?
 31:27) - (0 ?
 31:27) + 1))))))) << (0 ?
 31:27))) | (((gctUINT32) (0x03 & ((gctUINT32) ((((1 ?
 31:27) - (0 ?
 31:27) + 1) == 32) ?
 ~0U : (~(~0U << ((1 ? 31:27) - (0 ? 31:27) + 1))))))) << (0 ? 31:27)));

                /* Advance. */
                nop += 2;
            }
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
    OUT gctUINT32 *Physicals,
    OUT gctUINT64 *Logicals,
    OUT gctUINT32 *Bytes
    )
{
    gceSTATUS status;
    int i = 0;
    gctSIZE_T pageCount;
    gckVIRTUAL_COMMAND_BUFFER_PTR commandBuffer;
    gckKERNEL kernel = Context->hardware->kernel;
    gctPOINTER logical;
    gctPHYS_ADDR physical;

    gcsCONTEXT_PTR buffer;

    gcmkHEADER();

    gcmkVERIFY_OBJECT(Context, gcvOBJ_CONTEXT);

    buffer = Context->buffer;

    for (i = 0; i < gcdCONTEXT_BUFFER_COUNT; i++)
    {
        if (kernel->virtualCommandBuffer)
        {
            commandBuffer = (gckVIRTUAL_COMMAND_BUFFER_PTR)buffer->physical;
            physical = commandBuffer->virtualBuffer.physical;

            gcmkONERROR(gckOS_CreateUserVirtualMapping(
                kernel->os,
                physical,
                Context->totalSize,
                &logical,
                &pageCount));
        }
        else
        {
            physical = buffer->physical;

            gcmkONERROR(gckOS_MapMemory(
                kernel->os,
                physical,
                Context->totalSize,
                &logical));
        }

        Physicals[i] = gcmPTR_TO_NAME(physical);

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

