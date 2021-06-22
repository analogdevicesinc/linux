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


#include <gc_hal.h>
#include <gc_feature_database.h>

#include "gc_hal_kernel_hardware_func_flop_reset.h"

/*
 * Flop reset.
 *
 * The flops can be reset with PPU, NN and TP programs.
 * PPU:
 *   Requirements:
 *   1. DP inst with all bins enabled.
 *   2. Load inst which has at least two shader group,
 *      and every thread should load from different 64-byte address.
 *   3. Stroe inst which has at least 6 threads, whose addresses are
 *      from different 64-byte address and flush.
 *   Case:
 *   * InImage: 64x6 = {1}, unsigned int8
 *   * OutImage: 64x6, unsigned int8
 *   * OutImage = InImage + InImage
 * NN:
 *   Requirements:
 *   1. A XYDP6 case.
 *   2. NN cmd that uses only 1 core and make othere core's kernel size
 *      to be 0.
 *   Case:
 *   * Input: 3x2x1 = {1}
 *   * Kernel: 2x2x1 = {1}
 *   * Output: 2x1x1
 * TP:
 *   Requirements:
 *   1. Run TP fc on all TP cores.
 *   Case:
 *   * Input: 1x1x2 = {1}
 *   * Kernel: 1x1x2x64 = {1}
 *   * Output: 1x64
 */

/*
 * PPU.
 */
#define PPU_IMAGE_XSIZE 64
#define PPU_IMAGE_YSIZE 6
#define PPU_IMAGE_DATA 0x01010101
#define MAX_PPU_INSTRUCTION_COUNT 16
#define MAX_PPU_COMMAND_NUM 128

#define GCREG_SH_INSTRUCTION_TYPE_INVALID (~0U)

typedef enum _gceFLOP_RESET_PPU_DATA {
    gcvFLOP_RESET_PPU_INSTRUCTION = 0,
    gcvFLOP_RESET_PPU_INPUT       = 1,
    gcvFLOP_RESET_PPU_OUTPUT      = 2,
    gcvFLOP_RESET_PPU_DATA_NUM
}
gceFLOP_RESET_PPU_DATA;

/*
 * NN convolution.
 */
#define MAX_NN_COMMAND_NUM 64

#define NN_KERNEL_XSIZE 2
#define NN_KERNEL_YSIZE 2
#define NN_KERNEL_ZSIZE 1

#define NN_INPUT_XSIZE 3
#define NN_INPUT_YSIZE 2
#define NN_INPUT_ZSIZE 1

#define NN_OUTPUT_XSIZE 2
#define NN_OUTPUT_YSIZE 1
#define NN_OUTPUT_ZSIZE 1

typedef enum _gceVIP_ARCH_TYPE {
    gcvVIP_ARCH_TYPE_V6,
    gcvVIP_ARCH_TYPE_V7,
    gcvVIP_ARCH_TYPE_V8
}
gceVIP_ARCH_TYPE;

typedef enum _gceFLOP_RESET_NN_DATA {
    gcvFLOP_RESET_NN_INSTRUCTION = 0,
    gcvFLOP_RESET_NN_INPUT       = 1,
    gcvFLOP_RESET_NN_OUTPUT      = 2,
    gcvFLOP_RESET_NN_KERNEL      = 3,
    gcvFLOP_RESET_NN_DATA_NUM
}
gceFLOP_RESET_NN_DATA;

#define TP_KERNEL_XSIZE 1
#define TP_KERNEL_YSIZE 1
#define TP_KERNEL_ZSIZE 2
#define TP_KENREL_UNITS 64

#define TP_INPUT_XSIZE 1
#define TP_INPUT_YSIZE 1
#define TP_INPUT_ZSIZE 2

#define TP_OUTPUT_XSIZE 1
#define TP_OUTPUT_YSIZE 64
#define TP_OUTPUT_ZSIZE 1

typedef enum _gceFLOP_RESET_TP_DATA {
    gcvFLOP_RESET_TP_INSTRUCTION = 0,
    gcvFLOP_RESET_TP_INPUT       = 1,
    gcvFLOP_RESET_TP_OUTPUT      = 2,
    gcvFLOP_RESET_TP_KERNEL      = 3,
    gcvFLOP_RESET_TP_DATA_NUM
}
gceFLOP_RESET_TP_DATA;

static gceSTATUS
_AllocateVideoMemory(
    IN gckKERNEL Kernel,
    IN gceVIDMEM_TYPE Type,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    IN OUT gctSIZE_T *Bytes,
    OUT gckVIDMEM_NODE *Node,
    OUT gctPOINTER *Logical,
    OUT gctUINT32 *Address
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes = 0;
    gcePOOL pool = gcvPOOL_DEFAULT;

    if (!Bytes || *Bytes == 0)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (Pool)
    {
        pool = *Pool;
    }

    bufferBytes = *Bytes;

    gcmkONERROR(gckKERNEL_AllocateVideoMemory(
        Kernel,
        64,
        Type,
        AllocFlag,
        &bufferBytes,
        &pool,
        &bufferNode
        ));

    gcmkONERROR(gckVIDMEM_NODE_LockCPU(
        Kernel,
        bufferNode,
        gcvFALSE,
        gcvFALSE,
        &bufferLogical
        ));

    gcmkONERROR(gckVIDMEM_NODE_Lock(
        Kernel,
        bufferNode,
        &bufferAddress
        ));

    gcmkONERROR(gckOS_ZeroMemory(bufferLogical, bufferBytes));

    *Bytes = bufferBytes;

    if (Pool)
    {
        *Pool = pool;
    }

    if (Node)
    {
        *Node = bufferNode;
    }

    if (Logical)
    {
        *Logical = bufferLogical;
    }

    if (Address)
    {
        *Address = bufferAddress;
    }

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        if (bufferAddress)
        {
            gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
                Kernel,
                bufferNode,
                0,
                gcvNULL
                ));
        }

        if (bufferLogical)
        {
            gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
                Kernel,
                bufferNode,
                0,
                gcvFALSE,
                gcvFALSE
                ));
        }

        gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
            Kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_FreeVideoMemory(
    IN gckKERNEL Kernel,
    IN gckVIDMEM_NODE Node
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Node)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkVERIFY_OK(gckVIDMEM_NODE_Unlock(
        Kernel,
        Node,
        0,
        gcvNULL
        ));

    gcmkVERIFY_OK(gckVIDMEM_NODE_UnlockCPU(
        Kernel,
        Node,
        0,
        gcvFALSE,
        gcvFALSE
        ));

    gcmkVERIFY_OK(gckVIDMEM_NODE_Dereference(
        Kernel,
        Node
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_BitValue(
    IN gctUINT8_PTR *Base,
    IN gctUINT32 Value,
    IN gctUINT32_PTR Offset,
    IN gctUINT Length
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32_PTR msb = (gctUINT32_PTR)(*Base) + 1, lsb = (gctUINT32_PTR)(*Base);

    gcmkASSERT(*Offset <= 32 && Length <= 32);

    if ((*Offset) < 32)
    {
        gctUINT32 end = (*Offset) + Length, data = *lsb;

        if (end < 32)
        {
            /************************************************************************
             *       offset    32           64                                      *
             *     _________________________                                        *
             *    |_____|////|_|____________|                                       *
             *              end                                                     *
             ************************************************************************/
            data  = (*lsb & ((1 << *Offset) - 1));
            data |= (*lsb & ~((1 << end) - 1));
            data |= (Value << *Offset);

            *lsb = data;
            *Offset = end;
        }
        else if (end < 64)
        {
            /************************************************************************
             *       offset    32           64                                      *
             *     _________________________                                        *
             *    |_____|//////|//|_________|                                       *
             *                   end                                                *
             ************************************************************************/
            gctUINT32 length_m = end - 32;
            gctUINT32 data_l = (*lsb & ((1 << *Offset) - 1));
            gctUINT32 data_m = (*msb & ~((1 << length_m) - 1));

            data_l |= (Value << *Offset);
            data_m |= (Value >> (32 - *Offset));

            *lsb = data_l;

            if (end > 32)
                *msb = data_m;

            *Offset = length_m;

            *Base = (gctUINT8_PTR)msb;
        }

    }

    return status;
}

static gceSTATUS
_GetVIPCoreInfo(
    IN gckHARDWARE Hardware,
    OUT gceVIP_ARCH_TYPE *ArchType,
    OUT gctUINT8 *DataType,
    OUT gctUINT32 *CoreCount,
    OUT gctUINT32 *Zdp,
    OUT gctUINT32 *KernelBurstSize
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcsFEATURE_DATABASE *database = (gcsFEATURE_DATABASE *)(Hardware->featureDatabase);
    gceVIP_ARCH_TYPE archType;
    gctUINT8 dataType = GCREG_NN_DATA_TYPE_UINT8;
    gctUINT32 coreCount = 0;
    gctUINT32 zdp = 1;
    gctUINT32 kernelBurstSize;

    gcmkASSERT(database);

    /* Choose one supported format. */
    if (database->NNCoreCount_INT8 > 0)
    {
        dataType = GCREG_NN_DATA_TYPE_UINT8;
        coreCount = database->NNCoreCount_INT8;
    }
    else if (database->NNCoreCount_INT16 > 0)
    {
        dataType = GCREG_NN_DATA_TYPE_INT16;
        coreCount = database->NNCoreCount_INT16;
    }
    else if (database->NNCoreCount_FLOAT16 > 0)
    {
        dataType = GCREG_NN_DATA_TYPE_FP16;
        coreCount = database->NNCoreCount_FLOAT16;
    }
    else if (database->NNCoreCount_BFLOAT > 0)
    {
        dataType = GCREG_NN_DATA_TYPE_BFP16;
        coreCount = database->NNCoreCount_BFLOAT;
    }
    else
    {
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    if (database->NN_XYDP0)
    {
        archType = gcvVIP_ARCH_TYPE_V8;
    }
    else if (database->VIP_V7)
    {
        archType = gcvVIP_ARCH_TYPE_V7;
    }
    else
    {
        archType = gcvVIP_ARCH_TYPE_V6;
    }

    zdp = database->NN_ZDP3 ? 3 : 1;

    kernelBurstSize = database->DDR_KERNEL_BURST_SIZE;

    if (ArchType)
    {
        *ArchType = archType;
    }

    if (DataType)
    {
        *DataType = dataType;
    }

    if (CoreCount)
    {
        *CoreCount = coreCount;
    }

    if (Zdp)
    {
        *Zdp = zdp;
    }

    if (KernelBurstSize)
    {
        *KernelBurstSize = kernelBurstSize;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_GetMapIndex(
    gctUINT8 DataType,
    gctUINT32_PTR Index
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Index)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (DataType)
    {
    case GCREG_NN_DATA_TYPE_FP16:
        *Index = 1;
        break;

    case GCREG_NN_DATA_TYPE_BFP16:
        *Index = 2;
        break;

    default:
        *Index = 0;
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
_GetNNDataSize(
   IN gctUINT8 DataType,
   OUT gctUINT32_PTR DataSize
   )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!DataSize)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (DataType)
    {
    case GCREG_NN_DATA_TYPE_INT8:
    case GCREG_NN_DATA_TYPE_UINT8:
        *DataSize = 1;
        break;

    case GCREG_NN_DATA_TYPE_INT16:
    case GCREG_NN_DATA_TYPE_FP16:
    case GCREG_NN_DATA_TYPE_BFP16:
        *DataSize = 2;
        break;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

/*
 * PPU.
 */
static gceSTATUS
_ProgramPPUInput(
    IN gckHARDWARE Hardware,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 1; /* U8 format. */
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT32 *buffer = gcvNULL;
    gctUINT32 i;

    bufferBytes = bytes = InImageXSize * InImageYSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT32_PTR)bufferLogical;

    /* Fill the data. */
    for (i = 0; i < bytes / 4; i++)
    {
        buffer[i] = PPU_IMAGE_DATA;
    }

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: ppu input]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramPPUOutput(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Width,
    IN gctUINT32 Height,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 1;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    bufferBytes = bytes = Width * Height * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gctUINT32
_SETBITS(
    IN gctUINT32 Data,
    IN gctUINT32 Start,
    IN gctUINT32 End,
    IN gctUINT32 Value
    )
{
    gctUINT32 data = Data;
    gctUINT32 mask;

    if (End >= Start)
    {
        mask =  ((~0ULL >> (63 - End + Start)) << Start);
        data &= ~mask;
        data |= ((Value) << Start) & mask;
        return data;
    }
    else
    {
        mask =  ((~0ULL >> (63 - Start + End)) << End);
        data &= ~mask;
        data |= ((Value) << End) & mask;
        return data;
    }
}

static gctUINT32
_SETBIT(
    IN gctUINT32 Data,
    IN gctUINT32 Position,
    IN gctUINT32 Value
    )
{
    gctUINT32 data;

    data = _SETBITS(Data, Position, Position, Value);

    return data;
}

static gctUINT32
_GETBITS(
    IN gctUINT32 Data,
    IN gctUINT32 Start,
    IN gctUINT32 End
    )
{
    gctUINT32 data = Data;
    gctUINT32 mask;

    if (End >= Start)
    {
        mask = (~0ULL >> (63 - (End - Start)));
        return (data >> Start) & mask;
    }
    else
    {
        mask = (~0ULL >> (63 - (Start - End)));
        return (data >> End) & mask;;
    }
}

static gctUINT32
_GETBIT(
    IN gctUINT32 Data,
    IN gctUINT32 Position
    )
{
    gctUINT32 data;

    data = _GETBITS(Data, Position, Position);

    return data;
}

static gceSTATUS
gckPPU_SetImmediate(
    IN gctUINT32 Where,
    IN gctUINT32 Value,
    IN gctUINT32 Type,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (Where)
    {
    case 0:
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_ADR, _GETBITS(Value, 8, 0));
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_SWIZZLE, _GETBITS(Value, 16, 9));
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_MODIFIER_NEG, _GETBIT(Value, 17));
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_MODIFIER_ABS, _GETBIT(Value, 18));
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC0_REL_ADR, _GETBIT(Value, 19) | (Type << 1));
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC0_TYPE, AQ_SHADER_SRC_REG_TYPE_IMMEDIATE);
        break;

    case 1:
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_ADR, _GETBITS(Value, 8, 0));
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_SWIZZLE, _GETBITS(Value, 16, 9));
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_MODIFIER_NEG, _GETBIT(Value, 17));
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_MODIFIER_ABS, _GETBIT(Value, 18));
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_REL_ADR, _GETBIT(Value, 19) | (Type << 1));
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC1_TYPE, AQ_SHADER_SRC_REG_TYPE_IMMEDIATE);
        break;

    case 2:
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_ADR, _GETBITS(Value, 8, 0));
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_SWIZZLE, _GETBITS(Value, 16, 9));
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_MODIFIER_NEG, _GETBIT(Value, 17));
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_MODIFIER_ABS, _GETBIT(Value, 18));
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_REL_ADR, _GETBIT(Value, 19) | (Type << 1));
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_TYPE, AQ_SHADER_SRC_REG_TYPE_IMMEDIATE);
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_SetInstructionType(
    IN gctUINT32 Type,
    OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, INST_TYPE_0, _GETBIT(Type, 0));
    Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, INST_TYPE_1, _GETBITS(Type, 2, 1));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_IsEndOfBB(
    IN gckHARDWARE Hardware,
    IN gctUINT32 OpCode,
    OUT gctUINT32_PTR Inst
)
{
    gceSTATUS status = gcvSTATUS_OK;

    gcsFEATURE_DATABASE *database = (gcsFEATURE_DATABASE *)Hardware->featureDatabase;
    gctUINT32 bits = 0;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    if (!database->SH_END_OF_BB)
    {
        return gcvSTATUS_OK;
    }

    switch (OpCode)
    {
    case AQ_INST_OP_CODE_MOV:
    case AQ_INST_OP_CODE_MOVAI:
    case AQ_INST_OP_CODE_MOVAR:
    case AQ_INST_OP_CODE_MOVAF:
    case AQ_INST_OP_CODE_SELECT:
    case AQ_INST_OP_CODE_CMP:
    case AQ_INST_OP_CODE_SET:
        bits = _SETBITS(Inst[1], GCREG_SH_END_OF_BASIC_BLOCK_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_CC_VER_End, 1);
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SAMPLER_SWIZZLE, bits);
        break;

    case AQ_INST_OP_CODE_ATOM_ADD:
    case AQ_INST_OP_CODE_ATOM_XCHG:
    case AQ_INST_OP_CODE_ATOM_CMP_XCHG:
    case AQ_INST_OP_CODE_ATOM_MIN:
    case AQ_INST_OP_CODE_ATOM_MAX:
    case AQ_INST_OP_CODE_ATOM_OR:
    case AQ_INST_OP_CODE_ATOM_AND:
    case AQ_INST_OP_CODE_ATOM_XOR:
    case AQ_INST_OP_CODE_IMG_ATOM:
        bits = _SETBITS(Inst[0], GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_End, 1);
        Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, CONDITION_CODE, bits);
        break;

    case AQ_INST_OP_CODE_LOAD:
    case AQ_INST_OP_CODE_LOADP:
    case AQ_INST_OP_CODE_STORE:
    case AQ_INST_OP_CODE_STOREP:
    case AQ_INST_OP_CODE_IMG_LOAD:
    case AQ_INST_OP_CODE_IMG_LOAD_3D:
    case AQ_INST_OP_CODE_IMG_STORE:
    case AQ_INST_OP_CODE_IMG_STORE_3D:
        bits = _SETBITS(Inst[0], GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_End, 1);
        Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, CONDITION_CODE, bits);
        break;

    default:
        if (OpCode != AQ_INST_OP_CODE_BRANCH &&
            OpCode != AQ_INST_OP_CODE_BRANCH_ANY &&
            OpCode != AQ_INST_OP_CODE_CALL &&
            OpCode != AQ_INST_OP_CODE_RET &&
            OpCode != AQ_INST_OP_CODE_TEXKILL)
        {
            bits = _SETBITS(Inst[0], GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_End, 1);
            Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, CONDITION_CODE, bits);
        }
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_AddOpCode(
    IN gckHARDWARE Hardware,
    IN gctUINT32 OpCode,
    IN gctUINT32 Extended,
    IN gctUINT32 Type,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, OP_CODE, _GETBITS(OpCode, 5, 0));
    Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, OP_CODE_MSB6, _GETBIT(OpCode, 6));

    switch (OpCode)
    {
    case AQ_INST_OP_CODE_EXTENDED:
        gcmkONERROR(gckPPU_SetImmediate(2, Extended, GCREG_SH_IMMEDIATE_TYPE_U20, Inst));
        break;

    case AQ_INST_OP_CODE_EVIS:
        Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, DEST_REL_ADR, _GETBITS(Extended, 2, 0));
        Inst[0] = _SETBIT(Inst[0], 31, _GETBIT(Extended, 3));
        Inst[1] = _SETBITS(Inst[1], 1, 0, _GETBITS(Extended, 5, 4));
        break;

    case AQ_INST_OP_CODE_CMP:
    case AQ_INST_OP_CODE_MOV:
    case AQ_INST_OP_CODE_SELECT:
        Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, CONDITION_CODE, _GETBITS(Extended, 4, 0));
        break;

    default:
        break;
    }

    if (Type != GCREG_SH_INSTRUCTION_TYPE_INVALID)
    {
        gcmkONERROR(gckPPU_SetInstructionType(Type, Inst));
    }

    gcmkONERROR(gckPPU_IsEndOfBB(Hardware, OpCode, Inst));

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_SetDestination(
    IN gctUINT32 Address,
    IN gctUINT32 WriteEnable,
    IN gctUINT32 Saturate,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, DEST_VALID, 1);
    Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, DEST_ADR, Address);
    Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, DEST_WRITE_ENABLE, WriteEnable);
    Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, SATURATE, Saturate);

    return gcvSTATUS_OK;

OnError:
    return status;
}

#define gcdVX_ENABLE ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
#define gcdVX_ENABLE4(X, Y, Z, W) ((1 << (X)) | (1 << (Y)) | (1 << (Z)) | (1 << (W)))
#define gcdVX_ENABLE1(X) (1 << (X))
#define gcdVX_ENABLE2(X, Y) ((1 << (X)) | (1 << (Y)))
#define gcdVX_ENABLE3(X, Y, Z) ((1 << (X)) | (1 << (Y)) | (1 << (Z)))
#define gcdVX_SWIZZLE (0 | (1 << 2) | (2 << 4) | (3 << 6))
#define gcdVX_SWIZZLE1(X) ((X) | ((X) << 2) | ((X) << 4) | ((X) << 6))
#define gcdVX_SWIZZLE2(X, Y) ((X) | ((Y) << 2) | ((Y) << 4) | ((Y) << 6))
#define gcdVX_SWIZZLE4(X, Y, Z, W) ((X) | ((Y) << 2) | ((Z) << 4) | ((W) << 6))

static gctUINT32
gckPPU_GetPixel(
    IN gctUINT32 Format
    )
{
    gctUINT32 pixel = 0;

    switch(Format)
    {
    case GCREG_SH_INSTRUCTION_TYPE_UNSIGNED8:
        pixel = 15;
        break;

    case GCREG_SH_INSTRUCTION_TYPE_SIGNED16:
    case GCREG_SH_INSTRUCTION_TYPE_UNSIGNED16:
        pixel = 7;
        break;

    default:
        pixel = 15;
        break;
    }

    return pixel;
}

gceSTATUS
gckPPU_SetEVIS(
    IN gctUINT32 Start,
    IN gctUINT32 End,
    IN gctUINT32 Evis,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    Inst[0] = gcmSETFIELD(Inst[0], AQ_INST, DEST_WRITE_ENABLE, Start);
    Inst[0] = _SETBITS(Inst[0], 30, 27, End);
    Inst[1] = _SETBITS(Inst[1], 10, 2, Evis);

    return gcvSTATUS_OK;

OnError:
    return status;
}

static gceSTATUS
gckPPU_SetSource(
    IN gctUINT32 Where,
    IN gctUINT32 Address,
    IN gctUINT32 Swizzle,
    IN gctUINT32 Type,
    IN gctBOOL Negate,
    IN gctBOOL Absolute,
    IN gctUINT32 Relative,
    IN OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    switch (Where)
    {
    case 0:
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_VALID, 1);
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_ADR, Address);
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_SWIZZLE, Swizzle);
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_MODIFIER_NEG, Negate);
        Inst[1] = gcmSETFIELD(Inst[1], AQ_INST, SRC0_MODIFIER_ABS, Absolute);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC0_REL_ADR, Relative);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC0_TYPE, Type);
        break;

    case 1:
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_VALID, 1);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_ADR, Address);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_SWIZZLE, Swizzle);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_MODIFIER_NEG, Negate);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_MODIFIER_ABS, Absolute);
        Inst[2] = gcmSETFIELD(Inst[2], AQ_INST, SRC1_REL_ADR, Relative);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC1_TYPE, Type);
        break;

    case 2:
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_VALID, 1);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_ADR, Address);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_SWIZZLE, Swizzle);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_MODIFIER_NEG, Negate);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_MODIFIER_ABS, Absolute);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_REL_ADR, Relative);
        Inst[3] = gcmSETFIELD(Inst[3], AQ_INST, SRC2_TYPE, Type);
        break;

    default:
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
        break;
    }

    return gcvSTATUS_OK;

OnError:
    return status;
}

static const gctUINT32 NEGATE_FLAG   = 1 << 0;
static const gctUINT32 ABSOLUTE_FLAG = 1 << 1;

static gceSTATUS
gckPPU_SetUniform(
    IN gctUINT32 Where,
    IN gctUINT32 Address,
    IN gctUINT32 Swizzle,
    IN gctUINT32 Modifiers,
    OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate = (Modifiers & NEGATE_FLAG) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(gckPPU_SetSource(
        Where,
        Address,
        Swizzle,
        AQ_SHADER_SRC_REG_TYPE_UNBOUNDED_CONST,
        negate,
        absolute,
        0,
        Inst
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}

gceSTATUS
gckPPU_SetTempReg(
    IN gctUINT32 Where,
    IN gctUINT32 Address,
    IN gctUINT32 Swizzle,
    IN gctUINT32 Modifiers,
    OUT gctUINT32_PTR Inst
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate = (Modifiers & NEGATE_FLAG) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;

    if (!Inst)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(gckPPU_SetSource(
        Where,
        Address,
        Swizzle,
        AQ_SHADER_SRC_REG_TYPE_TEMP,
        negate,
        absolute,
        0,
        Inst
        ));

    return gcvSTATUS_OK;

OnError:
    return status;
}


static gceSTATUS
_ProgramPPUInstruction(
    IN gckHARDWARE Hardware,
    IN gctUINT32 DataType,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gctUINT32 *InstCount,
    OUT gctUINT32 *RegCount,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    gctUINT32 instCount = 0;
    gctUINT32_PTR inst = gcvNULL;

    gctUINT32 inImage1DataType = DataType;
    gctUINT32 inImage2DataType = DataType;
    gctUINT32 outImageDataType = DataType;

    if (!Data || !InstCount || !RegCount)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    bufferBytes = bytes = gcmSIZEOF(gctUINT32) * MAX_PPU_INSTRUCTION_COUNT;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    inst = (gctUINT32_PTR)bufferLogical;

    /* img_load.u8 r1, c0, r0.xy */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, AQ_INST_OP_CODE_IMG_LOAD, 0, inImage1DataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, gckPPU_GetPixel(inImage1DataType), 1, &inst[instCount]));
    gcmkONERROR(gckPPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &inst[instCount]));
    instCount += 4;

    /*img_load.u8 r2, c0, r0.xy */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, AQ_INST_OP_CODE_IMG_LOAD, 0, inImage2DataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetDestination(2, gcdVX_ENABLE, gcvFALSE, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, gckPPU_GetPixel(inImage2DataType), 1, &inst[instCount]));
    gcmkONERROR(gckPPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &inst[instCount]));
    instCount += 4;

    /* dp2x8 r1, r1, r2, c3_512 */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, AQ_INST_OP_CODE_EVIS, GCREG_SH_VISION_OPCODE_DP2X8, outImageDataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, 7, (inImage1DataType | (inImage2DataType << 3)), &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(0, 1, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 2, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetSource (2, 2, gcdVX_SWIZZLE, AQ_SHADER_SRC_REG_TYPE_UNIFORM512, gcvFALSE, gcvFALSE, 0, &inst[instCount]));
    instCount += 4;

    /* img_store.u8 r1, c2, r0.xy, r1 */
    gcmkONERROR(gckPPU_AddOpCode(Hardware, AQ_INST_OP_CODE_IMG_STORE, 0, outImageDataType, &inst[instCount]));
    gcmkONERROR(gckPPU_SetEVIS(0, gckPPU_GetPixel(outImageDataType), 1, &inst[instCount]));
    gcmkONERROR(gckPPU_SetUniform(0, 1, gcdVX_SWIZZLE, 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &inst[instCount]));
    gcmkONERROR(gckPPU_SetTempReg(2, 1, gcdVX_SWIZZLE, 0, &inst[instCount]));
    instCount += 4;

    bytes = gcmSIZEOF(gctUINT32) * instCount;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: ppu instruction]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    *InstCount = instCount;
    *RegCount = 0x3;

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramPPUCommand(
    IN gckHARDWARE Hardware,
    IN gctUINT32 Stride,
    IN gctUINT32 Width,
    IN gctUINT32 Height,
    IN gctUINT32 WorkDim,
    IN gctUINT32 ValueOrder,
    IN gctUINT32 GroupSizeX,
    IN gctUINT32 GroupSizeY,
    IN gctUINT32 GroupSizeZ,
    IN gctUINT32 GlobalScaleX,
    IN gctUINT32 GlobalScaleY,
    IN gctUINT32 GlobalScaleZ,
    IN gctUINT32 GlobalOffsetX,
    IN gctUINT32 GlobalOffsetY,
    IN gctUINT32 GlobalOffsetZ,
    IN gctUINT32 ThreadAllocation,
    IN gctUINT32 InImageAddress,
    IN gctUINT32 OutImageAddress,
    IN gctUINT32 InstAddress,
    IN gctUINT32 InstCount,
    IN gctUINT32 RegCount,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes = 0;
    gctUINT32 bytes = 0;
    gctUINT8_PTR endLogical;
    gctUINT32 endAddress;
    gctUINT32 endBytes = 0;
    gctUINT32_PTR commands = gcvNULL;
    gctUINT32 index = 0;
    gctUINT32 groupCountX = (Width +  GlobalScaleX - 1) / GlobalScaleX;
    gctUINT32 groupCountY = (Height +  GlobalScaleY - 1) / GlobalScaleY;
    gctUINT32 groupCountZ = 0;

    if (!Command)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    bufferBytes = gcmSIZEOF(gctUINT32) * MAX_PPU_COMMAND_NUM;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    commands = (gctUINT32_PTR)bufferLogical;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQModeRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_MODE, API_MODE, OpenCL);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE, FRONT_END)
                      | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, STALL);
    commands[index++] = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE, FRONT_END)
                      | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPixelUniformsRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 4);
    commands[index++] = InImageAddress;
    commands[index++] = Stride;
    commands[index++] = Height << 16 | Width;
    commands[index++] = gcmSETFIELD(0, GCREG_SH_IMAGE, SHIFT, 0)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, MULTIPLY, GCREG_SH_IMAGE_MULTIPLY_ONE)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, ADDRESSING, GCREG_SH_IMAGE_ADDRESSING_CLAMP)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, CONVERSION, GCREG_SH_IMAGE_CONVERSION_U8)
                      | gcmSETFIELDVALUE(0, GCREG_SH_IMAGE, TILING, LINEAR)
                      | gcmSETFIELDVALUE(0, GCREG_SH_IMAGE, TYPE, 2D)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, COMPONENT_COUNT, GCREG_SH_IMAGE_COMPONENT_COUNT_ONE_COMPONENT)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_R, GCREG_SH_IMAGE_SWIZZLE_R_X)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_G, GCREG_SH_IMAGE_SWIZZLE_G_ZERO)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_B, GCREG_SH_IMAGE_SWIZZLE_B_ZERO)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_A, GCREG_SH_IMAGE_SWIZZLE_A_ZERO);

    commands[index++] = 0xFFFFFFFF;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPixelUniformsRegAddrs + 0x04)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 4);
    commands[index++] = OutImageAddress;
    commands[index++] = Stride;
    commands[index++] = Height << 16 | Width;
    commands[index++] = gcmSETFIELD(0, GCREG_SH_IMAGE, SHIFT, 0)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, MULTIPLY, GCREG_SH_IMAGE_MULTIPLY_ONE)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, ADDRESSING, GCREG_SH_IMAGE_ADDRESSING_CLAMP)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, CONVERSION, GCREG_SH_IMAGE_CONVERSION_U8)
                      | gcmSETFIELDVALUE(0, GCREG_SH_IMAGE, TILING, LINEAR)
                      | gcmSETFIELDVALUE(0, GCREG_SH_IMAGE, TYPE, 2D)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, COMPONENT_COUNT, GCREG_SH_IMAGE_COMPONENT_COUNT_ONE_COMPONENT)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_R, GCREG_SH_IMAGE_SWIZZLE_R_X)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_G, GCREG_SH_IMAGE_SWIZZLE_G_ZERO)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_B, GCREG_SH_IMAGE_SWIZZLE_B_ZERO)
                      | gcmSETFIELD(0, GCREG_SH_IMAGE, SWIZZLE_A, GCREG_SH_IMAGE_SWIZZLE_A_ZERO);

    commands[index++] = 0xFFFFFFFF;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPixelUniformsRegAddrs + 0x08)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 16);
    commands[index++] = 0x55555555;
    commands[index++] = 0x00000000; /* TCfg. */
    commands[index++] = 0x01234567;
    commands[index++] = 0x89abcdef;
    commands[index++] = 0x55555555;
    commands[index++] = 0x01234567;
    commands[index++] = 0x89abcdef; /* BinSelect. */
    commands[index++] = 0x00000000; /* AccumType, ConstantType, and PostShift. */
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000;
    commands[index++] = 0x00000000; /* Constant. */

    commands[index++] = 0xFFFFFFFF;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWConfigRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_TW_CONFIG, WORK_DIM, GCREG_TW_CONFIG_WORK_DIM_TWO)
                      | gcmSETFIELDVALUE(0, GCREG_TW_CONFIG, TRAVERSE_ORDER, XYZ)
                      | gcmSETFIELD(0, GCREG_TW_CONFIG, VALUE_ORDER, GCREG_TW_CONFIG_VALUE_ORDER_LGW);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregSHIcacheInvalidateRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_SH_ICACHE_INVALIDATE, VS,  1)
                      | gcmSETFIELD(0, GCREG_SH_ICACHE_INVALIDATE, TCS, 1)
                      | gcmSETFIELD(0, GCREG_SH_ICACHE_INVALIDATE, TES, 1)
                      | gcmSETFIELD(0, GCREG_SH_ICACHE_INVALIDATE, GS,  1)
                      | gcmSETFIELD(0, GCREG_SH_ICACHE_INVALIDATE, PS,  1);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSUnpackRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_PS_UNPACK, VARYING0, 0);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQPixelShaderTemporaryRegisterControlRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = RegCount;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSSamplerBaseRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPixelShaderConstRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSStartPCRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSRelativeEndRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = InstCount / 4;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSInstructionRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = InstAddress;

    if (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_HALTI5))
    {
        commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregShaderMiscConfigRegAddrs)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
        commands[index++] = gcmSETFIELDVALUE(0, GCREG_SHADER_MISC_CONFIG, RTNE_ROUNDING, ENABLE);
    }
    else
    {
        commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregShaderConfigRegAddrs)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
        commands[index++] = gcmSETFIELDVALUE(0, GCREG_SHADER_CONFIG, RTNE_ROUNDING, ENABLE);
    }

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregSHCacheControlRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, GCREG_SH_CACHE_CONTROL, MODE, MEMORY);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSInstructionPrefetchRelativeEndRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = InstCount / 4 - 1;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQPixelShaderInputControlRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, AQ_PIXEL_SHADER_INPUT_CONTROL, COUNT,  1)
                      | gcmSETFIELD(0, AQ_PIXEL_SHADER_INPUT_CONTROL, TIMEOUT, ~0);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregVSThrottleRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPAControlRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregVaryingsRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSOutputModeRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQVertexShaderOutputControlRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, AQ_VERTEX_SHADER_OUTPUT_CONTROL, COUNT, 1);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregSemanticLocationRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSInstructionPrefetchRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWConfigRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_TW_CONFIG, WORK_DIM, GCREG_TW_CONFIG_WORK_DIM_TWO)
                      | gcmSETFIELDVALUE(0, GCREG_TW_CONFIG, TRAVERSE_ORDER, XYZ)
                      | gcmSETFIELD(0, GCREG_TW_CONFIG, VALUE_ORDER, GCREG_TW_CONFIG_VALUE_ORDER_WGL);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWShaderInfo2RegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWShaderInfoRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = ThreadAllocation;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWInfoGlobalOffsetXRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = GlobalOffsetX;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWInfoGlobalOffsetYRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = GlobalOffsetY;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWInfoGlobalOffsetZRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = GlobalOffsetZ;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWInfoGlobalScaleXRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = GlobalScaleX;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWInfoGlobalScaleYRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = GlobalScaleY;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWInfoGlobalScaleZRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = GlobalScaleZ;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWWorkGroupCountXRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 6);
    commands[index++] = groupCountX - 1;
    commands[index++] = groupCountY - 1;
    commands[index++] = groupCountZ - 1;
    commands[index++] = gcmSETFIELD(0, GCREG_TW_WORKGROUP_SIZE_X, SIZE, GroupSizeX - 1);
    commands[index++] = gcmSETFIELD(0, GCREG_TW_WORKGROUP_SIZE_Y, SIZE, GroupSizeY - 1);
    commands[index++] = gcmSETFIELD(0, GCREG_TW_WORKGROUP_SIZE_Z, SIZE, GroupSizeZ - 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregTWTriggerRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0xBADABEEB;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, SHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, VSSHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, PSSHL1_CACHE, ENABLE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQSemaphoreRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_SEMAPHORE, SOURCE, FRONT_END)
                      | gcmSETFIELDVALUE(0, AQ_SEMAPHORE, DESTINATION, PIXEL_ENGINE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, STALL);

    commands[index++] = gcmSETFIELDVALUE(0, STALL_STALL, SOURCE, FRONT_END)
                      | gcmSETFIELDVALUE(0, STALL_STALL, DESTINATION, PIXEL_ENGINE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, ZCACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, CCACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, SHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, VSSHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, PSSHL1_CACHE, ENABLE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, ZCACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, CCACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, SHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, VSSHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, PSSHL1_CACHE, ENABLE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcTileCacheFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, GC_TILE, CACHE_FLUSH_FLUSH, ENABLE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, ZCACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, CCACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, SHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, VSSHL1_CACHE, ENABLE)
                      | gcmSETFIELDVALUE(0, AQ_FLUSH, PSSHL1_CACHE, ENABLE);

    bytes = gcmSIZEOF(gctUINT32) * index;

    endLogical = (gctUINT8_PTR)bufferLogical + bytes;
    endAddress = bufferAddress + bytes;

    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));
        gcmkONERROR(gckWLFE_End(Hardware, endLogical, endAddress, &endBytes));
    }

    bytes += endBytes;

    gcmkASSERT(bytes <= bufferBytes);

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Command->funcVidMem = bufferNode;
    Command->funcVidMemBytes = bufferBytes;
    Command->logical = bufferLogical;
    Command->address = bufferAddress;
    Command->bytes = bytes;
    Command->endAddress = endAddress;
    Command->endLogical = endLogical;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

/*
**  gckHARDWARE_ResetFlopWithPPU
**
**  Generate the command to do PPU program as follows.
**      InImage: 64x6 = {1}, unsigned int8
**      OutImage: 64x6, unsigned int8
**      OutImage = InImage + InImage
**
**  INPUT:
**
**      gckHARDWARE Hardware
**
**      gctUINT32 AllocFlag
**
**      gcePOOL *Pool
**
**  OUTPUT:
**
**      gcePOOL *Pool
**
**      gcsFUNCTION_COMMAND *Command
*/
gceSTATUS
gckHARDWARE_ResetFlopWithPPU(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND *Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 dataType = GCREG_SH_INSTRUCTION_TYPE_UNSIGNED8;
    gcsFEATURE_DATABASE *database = gcvNULL;
    gctUINT32 numShaderCores;
    gctUINT32 stride, width, height;
    gctUINT32 workDim;
    gctUINT32 valueOrder;
    gctUINT32 groupSizeX, groupSizeY, groupSizeZ;
    gctUINT32 globalScaleX, globalScaleY, globalScaleZ;
    gctUINT32 globalOffsetX, globalOffsetY, globalOffsetZ;
    gctUINT32 threadAllocation;
    gctUINT32 inImageAddress = 0, outImageAddress = 0, instAddress = 0;
    gctUINT32 instCount = 0, regCount = 0;
    gctUINT32 dataCount;
    gctPOINTER pointer = gcvNULL;
    gcsFUNCTION_EXECUTION_DATA *data = gcvNULL;
    gctUINT32 i;

    /* Exectution data. */
    dataCount = gcvFLOP_RESET_PPU_DATA_NUM;
    gcmkASSERT(dataCount > 0);

    gcmkONERROR(gckOS_Allocate(
        Hardware->os,
        gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount,
        &pointer
        ));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount);
    data = (gcsFUNCTION_EXECUTION_DATA_PTR)pointer;

    database = (gcsFEATURE_DATABASE *)Hardware->featureDatabase;

    numShaderCores = database->NumShaderCores;

    stride = PPU_IMAGE_XSIZE * 1;
    width = PPU_IMAGE_XSIZE;
    height = PPU_IMAGE_YSIZE;

    gcmkONERROR(_ProgramPPUInput(
        Hardware,
        width,
        height,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_PPU_INPUT]
        ));

    gcmkONERROR(_ProgramPPUOutput(
        Hardware,
        width,
        height,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_PPU_OUTPUT]
        ));

    gcmkONERROR(_ProgramPPUInstruction(
        Hardware,
        dataType,
        AllocFlag,
        Pool,
        &instCount,
        &regCount,
        &data[gcvFLOP_RESET_PPU_INSTRUCTION]
        ));

    workDim = GCREG_TW_CONFIG_WORK_DIM_TWO;
    valueOrder = GCREG_TW_CONFIG_VALUE_ORDER_LGW;
    groupSizeX = 1;
    groupSizeY = 1;
    groupSizeZ = 0;
    globalScaleX = 4;
    globalScaleY = 1;
    globalScaleZ = 0;
    globalOffsetX = 0;
    globalOffsetY = 0;
    globalOffsetZ = 0;
    threadAllocation = (groupSizeX * groupSizeY + numShaderCores * 4 - 1) / (numShaderCores * 4);
    inImageAddress = data[gcvFLOP_RESET_PPU_INPUT].address;
    outImageAddress = data[gcvFLOP_RESET_PPU_OUTPUT].address;
    instAddress = data[gcvFLOP_RESET_PPU_INSTRUCTION].address;

    gcmkONERROR(_ProgramPPUCommand(
        Hardware,
        stride,
        width,
        height,
        workDim,
        valueOrder,
        groupSizeX,
        groupSizeY,
        groupSizeZ,
        globalScaleX,
        globalScaleY,
        globalScaleZ,
        globalOffsetX,
        globalOffsetY,
        globalOffsetZ,
        threadAllocation,
        inImageAddress,
        outImageAddress,
        instAddress,
        instCount,
        regCount,
        AllocFlag,
        Pool,
        Command
        ));

    Command->data = data;
    Command->dataCount = dataCount;

    return gcvSTATUS_OK;

OnError:
    if (Command->funcVidMem)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            Command->funcVidMem
            ));
        Command->funcVidMem = gcvNULL;
    }

    if (data)
    {
        for (i = 0; i < dataCount; i++)
        {
            if (data[i].bufVidMem)
            {
                gcmkVERIFY_OK(_FreeVideoMemory(
                    Hardware->kernel,
                    data[i].bufVidMem
                    ));
            }
        }

        gcmkVERIFY_OK(gckOS_Free(Hardware->os, data));
    }

    return status;
}

/*
 * NN
 */
static gceSTATUS
_ProgramNNKernel(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT32 CoreCount,
    IN gctUINT32 Zdp,
    IN gctUINT8 DataType,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 filterBytes = 0;
    gctUINT8_PTR kernels = gcvNULL;
    gctUINT32 offset = 0;
    gctUINT8_PTR kernelStreamSizePtr = gcvNULL;
    gctUINT32 filterTotalCount = 1;
    gctUINT32 itemBytes = 1;
    gctUINT32 biasBytes = 4;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    filterBytes = KernelXSize * KernelYSize * KernelZSize * itemBytes;

    /* Kernel buffer. */
    if (gcvVIP_ARCH_TYPE_V8 == ArchType)
    {
        /* Head (align to 64) + body (align to 64) + tail (align to 64). */
        gcmkASSERT(Zdp == 1 || Zdp == 3);
        bufferBytes = 64 + gcmALIGN_NP2((gcmALIGN_NP2(filterBytes, Zdp * 3) * (Zdp * 3) * filterTotalCount) + (1 * (16 / itemBytes)), 64) + 64;
    }
    else
    {
        /* Head (align to 64) + body ( align to 64). */
        bufferBytes = 64 + gcmALIGN_NP2(((filterBytes + biasBytes +  3) * filterTotalCount + 3), 64);
    }

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    kernels = (gctUINT8_PTR)bufferLogical;

    /* V8 huffman encoder. */
    if (gcvVIP_ARCH_TYPE_V8 == ArchType)
    {
        gctUINT32 i = 0;
        gctUINT8 rlt[][18] = {
            {0},            /* uint8 */
            {1, 1, 0, 1},   /* fp16 */
            {7, 1},         /* int8 */
            {0},            /* uint16 */
            {3, 1, 0, 1},   /* int16 */
            {0},            /* uint4 */
            {0},            /* int4 */
            {1, 1, 0, 1}    /* bf16 */
        };
        gctUINT8 map[][9] = {
            {1, 8, 7, 0, 4, 5, 6, 2, 3},
            {1, 5, 0, 7, 8, 2, 6, 3, 4},
            {1, 0, 7, 8, 4, 5, 6, 2, 3},
        };
        gctBOOL bit16 = DataType == GCREG_NN_DATA_TYPE_INT16 ||
                        DataType == GCREG_NN_DATA_TYPE_FP16  ||
                        DataType == GCREG_NN_DATA_TYPE_BFP16;
        gctBOOL fp16 = DataType == GCREG_NN_DATA_TYPE_FP16;
        gctUINT32 index = 0;

        if (Hardware->identity.customerID == 0x9f)
        {
            rlt[0][0] = 3;
            rlt[0][1] = 1;
            rlt[0][3] = 1;
        }

        gcmkONERROR(_GetMapIndex(DataType, &index));

        gcmkONERROR(_BitValue(&kernels, 0, &offset, 1));        /* precode */
        gcmkONERROR(_BitValue(&kernels, bit16, &offset, 1));    /* bit16 */
        gcmkONERROR(_BitValue(&kernels, fp16, &offset, 1));     /* fp16 */
        gcmkONERROR(_BitValue(&kernels, 0, &offset, 1));        /* reserved */
        gcmkONERROR(_BitValue(&kernels, 1, &offset, 4));        /* version, 1 */
        gcmkONERROR(_BitValue(&kernels, 4, &offset, 8));        /* zero run length size */

        for (i = 0; i < 18; i++)
        {
            /* Zero run length x 18. */
            gcmkONERROR(_BitValue(&kernels, rlt[DataType][i], &offset, 8));
        }

        for (i = 0; i < 4; i++)
        {
            /* Map x 4. */
            gcmkONERROR(_BitValue(&kernels, (map[index][2 * i + 1] << 4) + map[index][2 * i], &offset, 8));
        }

        /* Avg bias */
        gcmkONERROR(_BitValue(&kernels, 0, &offset, 16));

        /* Reserved, must zero. */
        gcmkONERROR(_BitValue(&kernels, 0, &offset, 16));

        kernelStreamSizePtr = kernels;

        for (i = 0; i < CoreCount; i ++)
        {
            /* Stream size. */
            gcmkONERROR(_BitValue(&kernels, 0, &offset, 32));
        }

        kernels = (gctUINT8_PTR)bufferLogical + gcmALIGN_NP2((gctUINT32)((gctUINT8_PTR)kernels - (gctUINT8_PTR)bufferLogical), 64);

        switch (DataType)
        {
        case GCREG_NN_DATA_TYPE_INT16:
            /* Huffman data: 00000018 00924600 */
            gcmkONERROR(_BitValue(&kernels, 0x04058000, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x640101fc, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x00001200, &offset, 32));

            /* Only on core, stream size. */
            gcmkONERROR(_BitValue(&kernelStreamSizePtr, 0x0000006d, &offset, 32));

            break;

        case GCREG_NN_DATA_TYPE_UINT8:
        case GCREG_NN_DATA_TYPE_INT8:
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0xec000038, &offset, 32));

            /* Only on core, stream size. */
            gcmkONERROR(_BitValue(&kernelStreamSizePtr, 0x35, &offset, 32));

            break;

        case GCREG_NN_DATA_TYPE_FP16:
            /* Huffman data: 0009db68 000006c0 000001f0 00000900 00024000. */
            gcmkONERROR(_BitValue(&kernels, 0x0009db68, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x000006c0, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x000001f0, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x00000900, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x00024000, &offset, 32));

            /* Only on core, stream size. */
            gcmkONERROR(_BitValue(&kernelStreamSizePtr, 0x000000a3, &offset, 32));

            break;

        case GCREG_NN_DATA_TYPE_BFP16:
            /* Huffman data: 0007fff8 7f00fdfc c0397f00 0900001f 40000000 00000002. */
            gcmkONERROR(_BitValue(&kernels, 0x0007fff8, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x7f00fdfc, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0xc0397f00, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x0900001f, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x40000000, &offset, 32));
            /* Huffman data. */
            gcmkONERROR(_BitValue(&kernels, 0x00000002, &offset, 32));

            /* Only on core, stream size. */
            gcmkONERROR(_BitValue(&kernelStreamSizePtr, 0x000000b2, &offset, 32));

            break;

        default:
            gcmkFATAL("Huffman encode not support this format! Please check!");
            break;
        }
    }
    else
    {
        gctBOOL zeroAll = gcvFALSE;
        gctUINT8 zrl = 0;
        gctUINT16 vzNum = 1;
        gctUINT32 bias = 0;
        gctUINT32 totalSize = gcmALIGN_NP2((filterTotalCount * (filterBytes + biasBytes +  3) + 3), 64);

        gckOS_ZeroMemory(kernels, totalSize + 64);

        *((gctUINT32_PTR)kernels) = totalSize;
        kernels += totalSize;
        if (zeroAll)
        {
            /*
             * Zrl & coreFilterCount, both compressed weight and bias are zero,
             * the size (1 * 1 * 2 * 2 + 4 ) < 64, aligned to 64.
             */
            *((gctUINT32_PTR)kernels) = (vzNum << (8 * itemBytes));
        }
        else
        {
            gctINT16 value = (DataType == GCREG_NN_DATA_TYPE_FP16) ? 0x3c00 /*1.0f*/ : 1;
            gctUINT32 i = 0;

            _BitValue(&kernels, zrl, &offset, 8);
            _BitValue(&kernels, vzNum, &offset, 16);
            _BitValue(&kernels, value, &offset, 8 * itemBytes);
            _BitValue(&kernels, bias, &offset, 32);

            if (DataType == GCREG_NN_DATA_TYPE_UINT16 ||
                DataType == GCREG_NN_DATA_TYPE_INT16)
            {
                _BitValue(&kernels, 0, &offset, 16);
            }

            for (i = 1; i < filterBytes / itemBytes; i++)
            {
                _BitValue(&kernels, value, &offset, 8 * itemBytes);
            }
        }
    }

    bytes = kernels + (offset + 7) / 8 - (gctUINT8_PTR)bufferLogical;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn kernel]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramNNInput(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 InImageZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 inputSize = InImageXSize * InImageYSize * InImageZSize;
    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT8_PTR buffer = gcvNULL;

    gctUINT32 i = 0;
    gctUINT32 offset = 0;
    gctUINT32 value[] = {
        1,      /* uint8 */
        0x3c00, /* fp16 */
        1,      /* int8 */
        1,      /* uint16 */
        1,      /* int16 */
        1,      /* uint4 */
        1,      /* int4 */
        0x3f80  /* bf16 */
    };

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = inputSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    if (gcvVIP_ARCH_TYPE_V8 == ArchType)
    {
        value[GCREG_NN_DATA_TYPE_INT16] = 0x81;
    }

    buffer = (gctUINT8_PTR)bufferLogical;

    for (i = 0; i < inputSize; i++)
    {
        _BitValue(&buffer, value[DataType], &offset, itemBytes * 8);
    }

    bytes = buffer + (offset + 7) / 8 - (gctUINT8_PTR)bufferLogical;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn input]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramNNOutput(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 OutputXSize,
    IN gctUINT32 OutputYSize,
    IN gctUINT32 OutputZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    if (!Data)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = bytes = OutputXSize * OutputYSize * OutputZSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramNNInstruction(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 OutImageXSize,
    IN gctUINT32 OutImageYSize,
    IN gctUINT32 OutImageZSize,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN gctUINT32 InImageAddress,
    IN gctUINT32 OutImageAddress,
    IN gctUINT32 KernelAddress,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckOS os = Hardware->os;

    gctUINT32 itemBytes;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT32 *command = gcvNULL;

    gctUINT8 kernelDataType;
    gctUINT8 inImageDataType;
    gctUINT8 outImageDataType;

    gctUINT32 kernelsPerCore = 1;

    gctUINT32 nnLayerFlush = 1;
    gctUINT32 noZOffset = 0;
    gctUINT32 imageEndAddress = 2048;
    gctUINT32 postShift = 0;
    gctUINT32 postShiftBit56 = 0;
    gctUINT8 coefZP = 0;
    gctUINT8 outputZP = 0;

    bufferBytes = bytes = gcmSIZEOF(gctUINT32) * ((ArchType == gcvVIP_ARCH_TYPE_V6) ? 16 : 32);

    /* Allocate buffer. */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    command = (gctUINT32_PTR)bufferLogical;

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    kernelDataType   =
    inImageDataType  =
    outImageDataType = DataType;

    switch (ArchType)
    {
    case gcvVIP_ARCH_TYPE_V8:
        noZOffset = 1;
        outputZP = 0;
        postShift = (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NN_FLOAT_POST_MULT)) ? 0x1f : 0;
        postShiftBit56 = (gckHARDWARE_IsFeatureAvailable(Hardware, gcvFEATURE_NN_FLOAT_POST_MULT)) ? 3 : 0;
        break;

    case gcvVIP_ARCH_TYPE_V7:
    case gcvVIP_ARCH_TYPE_V6:
        postShift = (DataType == GCREG_NN_DATA_TYPE_INT8) ? 15 : 0;
        break;

    default:
        gcmkONERROR(gcvSTATUS_NOT_SUPPORTED);
        break;
    }

    /* gcregNNInstWord0 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD0, LAYER_TYPE,          0) /* 0: conv, 1: fc */
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, NO_ZLOCATION_OFFSET, noZOffset)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, KERNEL_XYSIZE,       KernelXSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, KERNEL_ZSIZE,        (KernelZSize & 0x3FFF))
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, KERNELS_PER_CORE,    kernelsPerCore)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, POOLING,             0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, POOLING_XYSIZE,      0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, PRELU,               0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD0, LAST_LAYER,          nnLayerFlush)
        );

    /* gcregNNInstWord1 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD1, INIMAGE_XSIZE,             InImageXSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, INIMAGE_YSIZE,             InImageYSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, KERNEL_DATA_TYPE,          kernelDataType >> 1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, INIMAGE_DATA_TYPE,         inImageDataType >> 1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, OUTIMAGE_DATA_TYPE,        outImageDataType >> 1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, KERNEL_DATA_SIZE_MINUS1,   kernelDataType & 0x1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, INIMAGE_DATA_SIZE_MINUS1,  inImageDataType & 0x1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD1, OUTIMAGE_DATA_SIZE_MINUS1, outImageDataType & 0x1)
        );

    /* gcregNNInstWord2 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD2, RELU,                0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, ACTIVATION_FUNCTION, 0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, INIMAGE_XOFFSET,     0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, INIMAGE_YOFFSET,     0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, POST_MULTIPLIER,     0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, BRICK_MODE,          0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, BRICK_DISTANCE,      0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD2, POST_SHIFT,          postShift)
        );

    /* gcregNNInstWord3 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD3, OUTIMAGE_XSIZE, OutImageXSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD3, OUTIMAGE_YSIZE, OutImageYSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD3, NO_BIAS,        0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD3, NO_FLUSH,       0)
        );

    /* gcregNNInstWord4 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD4, OUTIMAGE_ZSIZE,        OutImageZSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD4, ROUNDING_MODE,         GCREG_NN_INST_WORD4_ROUNDING_MODE_SIMPLE_ROUNDING)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD4, OUTIMAGE_TILE_XSIZE,   1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD4, OUTIMAGE_TILE_YSIZE,   1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD4, INIMAGE_XOFFSET_BIT3,  (0 >> 3) & 0x1)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD4, INIMAGE_YOFFSET_BIT3,  (0 >> 3) & 0x1)
        );

    /* gcregNNInstWord5 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD5, KERNEL_ZSIZE1,       ((KernelZSize >> 14) & 0x3F))
      | gcmSETFIELD (0, GCREG_NN_INST_WORD5, KERNEL_BASE_ADDRESS, (KernelAddress >> 6))
        );

    /* gcregNNInstWord6 */
    gcmkWRITE_MEMORY(command, InImageAddress);

    /* gcregNNInstWord7 */
    gcmkWRITE_MEMORY(command, OutImageAddress);

    /* gcregNNInstWord8 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD8, KERNEL_YSIZE,      KernelYSize)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD8, OUT_IMAGE_YSTRIDE, OutImageYSize)
        );

    /* gcregNNInstWord9 */
    gcmkWRITE_MEMORY(command, 0);

    /* gcregNNInstWord10 */
    gcmkWRITE_MEMORY(command, 0);

    /* gcregNNInstWord11 */
    gcmkWRITE_MEMORY(command, 0);

    /* gcregNNInstWord12 */
    gcmkWRITE_MEMORY(command, 0);

    /* gcregNNInstWord13 */
    gcmkWRITE_MEMORY(command, 0);

    /* gcregNNInstWord14 */
    gcmkWRITE_MEMORY(command, imageEndAddress);

    /* gcregNNInstWord15 */
    gcmkWRITE_MEMORY(
        command,
        gcmSETFIELD (0, GCREG_NN_INST_WORD15, IN_IMAGE_BORDER_MODE,     0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD15, IN_IMAGE_BORDER_CONSTANT, 0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD15, KERNEL_DATA_TYPE_MSB,     kernelDataType >> 2)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD15, INIMAGE_DATA_TYPE_MSB,    inImageDataType >> 2)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD15, OUTIMAGE_DATA_TYPE_MSB,   outImageDataType >> 2)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD15, POST_MULTIPLIER_BIT6TO1,  0)
      | gcmSETFIELD (0, GCREG_NN_INST_WORD15, POST_SHIFT_BIT6TO5,       postShiftBit56)
        );

    /* V7 or V8 */
    if (ArchType == gcvVIP_ARCH_TYPE_V7 ||
        ArchType == gcvVIP_ARCH_TYPE_V8)
    {
        /* gcregNNInstWord16 */
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0,GCREG_NN_INST_WORD16, IN_IMAGE_XSTRIDE, InImageXSize * itemBytes)
          | gcmSETFIELD (0,GCREG_NN_INST_WORD16, IN_IMAGE_YSTRIDE, InImageYSize)
            );

        /* gcregNNInstWord17 */
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0,GCREG_NN_INST_WORD17, OUT_IMAGE_XSTRIDE, OutImageXSize * itemBytes)
          | gcmSETFIELD (0,GCREG_NN_INST_WORD17, POST_MULTIPLIER_BIT14TO7, 0)
            );

        /* gcregNNInstWord18 */
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0, GCREG_NN_INST_WORD18, OUT_IMAGE_CIRCULAR_BUF_SIZE, 0 >> 6)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD18, PER_CH_POST_MULT,            0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD18, IN_IMAGE_XOFFSET_BIT4,       (0 >> 4) & 0x1)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD18, IN_IMAGE_YOFFSET_BIT4,       (0 >> 4) & 0x1)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD18, SLOW_OUTPUT,                 0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD18, KERNEL_DATA_TYPE_BIT3,       kernelDataType >> 3)
            );

        /* GCREG_NN_INST_WORD19_OUT_IMAGE_CIRCULAR_BUF_END_ADDR_PLUS1 */
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0,GCREG_NN_INST_WORD19, OUT_IMAGE_CIRCULAR_BUF_END_ADDR_PLUS1, 0xFFFFFFFF >> 6)
          | gcmSETFIELD (0,GCREG_NN_INST_WORD19, B_FLOAT16_MODE,                        0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD19, IN_IMAGE_DATA_TYPE_BIT3,              inImageDataType >> 3)
            );

        /* GCREG_NN_INST_WORD20 */
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0,GCREG_NN_INST_WORD20, IN_IMAGE_CIRCULAR_BUF_SIZE, 0 >> 6)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD20, OUT_IMAGE_DATA_TYPE_BIT3,  outImageDataType >> 3)
            );

        /* GCREG_NN_INST_WORD21_IN_IMAGE_CIRCULAR_BUF_END_ADDR_PLUS1 */
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0,GCREG_NN_INST_WORD21, IN_IMAGE_CIRCULAR_BUF_END_ADDR_PLUS1, 0xFFFFFFFF >> 6)
            );

        /*GCREG_NN_INST_WORD22*/
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0, GCREG_NN_INST_WORD22, COEF_ZERO_POINT,                    coefZP)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD22, OUT_IMAGE_ZERO_POINT,               outputZP)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD22, KERNEL_DIRECT_STREAM_FROM_VIP_SRAM, 0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD22, DEPTH_WISE,                         0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD22, POST_MULTIPLIER_BIT22TO15,          0)
            );

        /*GCREG_NN_INST_WORD23*/
        gcmkWRITE_MEMORY(command, 0);

        /*GCREG_NN_INST_WORD24*/
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0, GCREG_NN_INST_WORD24, IN_IMAGE_TRANSPOSE_CH_MINUS_ONE,    0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD24, OUT_IMAGE_TRANSPOSE_BUF_START_ADDR, 0 >> 4)
            );

        /*GCREG_NN_INST_WORD25*/
        gcmkWRITE_MEMORY(
            command,
            gcmSETFIELD (0, GCREG_NN_INST_WORD25, OUT_IMAGE_TRANSPOSE_CH_MINUS_ONE,          0)
          | gcmSETFIELD (0, GCREG_NN_INST_WORD25, OUT_IMAGE_TRANSPOSE_BUF_END_ADDR_PLUS_ONE, 0 >> 4)
            );
    }

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: nn instruction]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramNNCommand(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT32 KernelBurstSize,
    IN gctUINT32 InstAddress,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes;
    gctUINT32 bytes;
    gctUINT8_PTR endLogical;
    gctUINT32 endAddress;
    gctUINT32 endBytes = 0;
    gcsFEATURE_DATABASE *database = (gcsFEATURE_DATABASE *)(Hardware->featureDatabase);
    gctUINT32 index = 0;
    gctINT32 disableZDPN = 1, disableSWTiling = 1;
    gctBOOL enableNNStride = gcvFALSE;
    gctUINT32 smallBatch;
    gctUINT32 ddrBurstSize;
    gctUINT32 *commands = gcvNULL;

    bufferBytes = gcmSIZEOF(gctUINT32) * MAX_NN_COMMAND_NUM;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    commands = (gctUINT32 *)bufferLogical;

    disableZDPN = (database->NN_ZDP3 || database->NN_ZDP6) ? 0 : 1;

    enableNNStride = database->NN_STRIDE_SUPPORT;
    disableSWTiling = enableNNStride ? 0 : 1;

    if (Hardware->identity.chipModel == 0x8000 &&
        Hardware->identity.chipRevision == 0x7120 &&
        (Hardware->identity.customerID == 0x80 ||
         Hardware->identity.customerID == 0x92))
    {
        smallBatch = GCREG_CONFIG_NN_SMALL_BATCH_ENABLE;
    }
    else
    {
        smallBatch = (database->NN_SMALLBATCH_PHASE1 && database->NN_COMMAND_KERNEL_REQUEST_CONFICT_FIX)
                     ? GCREG_CONFIG_NN_SMALL_BATCH_ENABLE : GCREG_CONFIG_NN_SMALL_BATCH_DISABLE;
    }

    switch(KernelBurstSize)
    {
    case 256:
        ddrBurstSize = GCREG_CONFIG_NN_DDR_BURST_SIZE_SIZE256_B;
        break;

    case 64:
    default:
        ddrBurstSize = GCREG_CONFIG_NN_DDR_BURST_SIZE_SIZE64_B;
        break;
    }

    commands = (gctUINT32_PTR)bufferLogical;

    if (gcvVIP_ARCH_TYPE_V6 == ArchType)
    {
        commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregMMUConfigRegAddrs)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
        commands[index++] = 0;

        commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                          | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
        commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, L2_CACHE, ENABLE);
    }

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregSramRemapStartAddressRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = Hardware->options.sRAMGPUVirtAddrs[0];
    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregOnChipBufferRemapStartAddressRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregOnChipBufferRemapEndAddressRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0x00000000;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, L2_CACHE, ENABLE);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregConfigNNRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_CONFIG_NN, ZDPN, disableZDPN)
                      | gcmSETFIELD(0, GCREG_CONFIG_NN, SW_TILING, disableSWTiling)
                      | gcmSETFIELD(0, GCREG_CONFIG_NN, SMALL_BATCH, smallBatch)
                      | gcmSETFIELD(0, GCREG_CONFIG_NN, DDR_BURST_SIZE, ddrBurstSize)
                      | gcmSETFIELD(0, GCREG_CONFIG_NN, COMMAND_SIZE, GCREG_CONFIG_NN_COMMAND_SIZE_SIZE128_B);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregVipFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_VIP_FLUSH, NN_FLUSH_CLIENT_ID, 0)
                      | gcmSETFIELD(0, GCREG_VIP_FLUSH, NN_NO_FLUSH, 0);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSTriggerNNRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELD(0, GCREG_PS_TRIGGER_NN, COMMAND_BUFFER_ADDR, (InstAddress >> 6))
                      | gcmSETFIELD(0, GCREG_PS_TRIGGER_NN, COMMAND_EVENT_ID, 0);

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, gcregPSWaitForEventRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = 0;

    commands[index++] = gcmSETFIELDVALUE(0, AQ_COMMAND_LOAD_STATE_COMMAND, OPCODE, LOAD_STATE)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, ADDRESS, AQFlushRegAddrs)
                      | gcmSETFIELD(0, AQ_COMMAND_LOAD_STATE_COMMAND, COUNT, 1);
    commands[index++] = gcmSETFIELDVALUE(0, AQ_FLUSH, SHL1_CACHE, ENABLE);

    bytes = gcmSIZEOF(gctUINT32) * index;

    endLogical = (gctUINT8_PTR)bufferLogical + bytes;
    endAddress = bufferAddress + bytes;

    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));
        gcmkONERROR(gckWLFE_End(Hardware, endLogical, endAddress, &endBytes));
    }

    bytes += endBytes;

    gcmkASSERT(bytes <= bufferBytes);

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Command->funcVidMem = bufferNode;
    Command->funcVidMemBytes = bufferBytes;
    Command->logical = bufferLogical;
    Command->address = bufferAddress;
    Command->bytes = bytes;
    Command->endAddress = endAddress;
    Command->endLogical = endLogical;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkONERROR(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

gceSTATUS
gckHARDWARE_ResetFlopWithNN(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND *Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 kernelXSize = NN_KERNEL_XSIZE;
    gctUINT32 kernelYSize = NN_KERNEL_YSIZE;
    gctUINT32 kernelZSize = NN_KERNEL_ZSIZE;

    gctUINT32 inImageXSize = NN_INPUT_XSIZE;
    gctUINT32 inImageYSize = NN_INPUT_YSIZE;
    gctUINT32 inImageZSize = NN_INPUT_ZSIZE;

    gctUINT32 outImageXSize = NN_OUTPUT_XSIZE;
    gctUINT32 outImageYSize = NN_OUTPUT_YSIZE;
    gctUINT32 outImageZSize = NN_OUTPUT_ZSIZE;

    gctUINT32 i;
    gctPOINTER pointer = gcvNULL;

    gceVIP_ARCH_TYPE archType;
    gctUINT8 dataType;
    gctUINT32 coreCount = 0;
    gctUINT32 itemBytes = 0;
    gctUINT32 zdp = 1;
    gctUINT32 kernelBurstSize;
    gcsFUNCTION_EXECUTION_DATA_PTR data = gcvNULL;
    gctUINT32 dataCount = 0;

    if (!Command)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(_GetVIPCoreInfo(
        Hardware,
        &archType,
        &dataType,
        &coreCount,
        &zdp,
        &kernelBurstSize
        ));

    gcmkONERROR(_GetNNDataSize(dataType, &itemBytes));

    /* Exectution data. */
    dataCount = gcvFLOP_RESET_NN_DATA_NUM;
    gcmkASSERT(dataCount > 0);

    gcmkONERROR(gckOS_Allocate(
        Hardware->os,
        gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount,
        &pointer
        ));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount);
    data = (gcsFUNCTION_EXECUTION_DATA *)pointer;

    /* Kernel. */
    gcmkONERROR(_ProgramNNKernel(
        Hardware,
        archType,
        coreCount,
        zdp,
        dataType,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_KERNEL]
        ));

    /* Input. */
    gcmkONERROR(_ProgramNNInput(
        Hardware,
        archType,
        dataType,
        inImageXSize,
        inImageYSize,
        inImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_INPUT]
        ));

    /* Output. */
    gcmkONERROR(_ProgramNNOutput(
        Hardware,
        dataType,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_OUTPUT]
        ));

    /* Commands. */
    gcmkONERROR(_ProgramNNInstruction(
        Hardware,
        archType,
        dataType,
        inImageXSize,
        inImageYSize,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        data[gcvFLOP_RESET_NN_INPUT].address,
        data[gcvFLOP_RESET_NN_OUTPUT].address,
        data[gcvFLOP_RESET_NN_KERNEL].address,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_NN_INSTRUCTION]
        ));

    gcmkONERROR(_ProgramNNCommand(
        Hardware,
        archType,
        kernelBurstSize,
        data[gcvFLOP_RESET_NN_INSTRUCTION].address,
        AllocFlag,
        Pool,
        Command
        ));

    Command->data = data;
    Command->dataCount = dataCount;

    return gcvSTATUS_OK;

OnError:
    if (Command && Command->funcVidMem)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            Command->funcVidMem
            ));
        Command->funcVidMem = gcvNULL;
    }

    if (data)
    {
        for (i = 0; i < dataCount; i++)
        {
            if (data[i].bufVidMem)
            {
                gcmkVERIFY_OK(_FreeVideoMemory(
                    Hardware->kernel,
                    data[i].bufVidMem
                    ));
            }
        }

        gcmkVERIFY_OK(gckOS_Free(Hardware->os, data));
    }

    return status;
}

static gceSTATUS
_ProgramTPKernel(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT32 CoreCount,
    IN gctUINT32 Zdp,
    IN gctUINT8 DataType,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress =0;
    gctSIZE_T bufferBytes = 0x3C0;
    gctUINT32 *buffer = gcvNULL;
    gctUINT32 i;


    /* hardcode */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    buffer = (gctUINT32_PTR)bufferLogical;

    /* Fill the data. */
    for (i = 0; i < bufferBytes / 4; i++)
    {
        buffer[i] = 0;
    }

    buffer[0] = 0x01150410;
    buffer[1] = buffer[81] = buffer[161] = 0x00000100;
    buffer[5] = buffer[85] = buffer[165] = 0x26543780;
    buffer[6] = buffer[86] = buffer[166] = 0x000000ff;
    buffer[7] = buffer[87] = buffer[167] = 0x0006801a;
    buffer[48] = buffer[128] = buffer[208] = 0x00024938;
    buffer[64] = buffer[144] = buffer[224] = 0x00024938;
    buffer[80] = buffer[160] = 0x01140410;


    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bufferBytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP kernel]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bufferBytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bufferBytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramTPInput(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 InImageZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 inputSize = InImageXSize * InImageYSize * InImageZSize;
    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT8_PTR buffer = gcvNULL;

    gctUINT32 i = 0;
    gctUINT32 offset = 0;
    gctUINT32 value[] = {
        0xff,   /* uint8 , the case set scale = 0.003921569*/
        0x3c00, /* fp16 */
        1,      /* int8 */
        1,      /* uint16 */
        1,      /* int16 */
        1,      /* uint4 */
        1,      /* int4 */
        0x3f80  /* bf16 */
    };
    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = inputSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    if (gcvVIP_ARCH_TYPE_V8 == ArchType)
    {
        value[GCREG_NN_DATA_TYPE_INT16] = 0x81;
    }

    buffer = (gctUINT8_PTR)bufferLogical;

    for (i = 0; i < inputSize; i++)
    {
        _BitValue(&buffer, value[DataType], &offset, itemBytes * 8);
    }

    bytes = buffer + (offset + 7) / 8 - (gctUINT8_PTR)bufferLogical;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP input]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramTPOutput(
    IN gckHARDWARE Hardware,
    IN gctUINT8 DataType,
    IN gctUINT32 OutputXSize,
    IN gctUINT32 OutputYSize,
    IN gctUINT32 OutputZSize,
    IN gctUINT32 AllocFlag,
    IN OUT gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA *Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 itemBytes = 0;
    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;

    if (!Data)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    gcmkONERROR(_GetNNDataSize(DataType, &itemBytes));

    bufferBytes = bytes = OutputXSize * OutputYSize * OutputZSize * itemBytes;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_BITMAP,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    gcmkVERIFY_OK(_FreeVideoMemory(
        Hardware->kernel,
        bufferNode
        ));

    return status;
}

static gceSTATUS
_ProgramTPInstruction(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT8 DataType,
    IN gctUINT32 InImageXSize,
    IN gctUINT32 InImageYSize,
    IN gctUINT32 OutImageXSize,
    IN gctUINT32 OutImageYSize,
    IN gctUINT32 OutImageZSize,
    IN gctUINT32 KernelXSize,
    IN gctUINT32 KernelYSize,
    IN gctUINT32 KernelZSize,
    IN gctUINT32 InImageAddress,
    IN gctUINT32 OutImageAddress,
    IN gctUINT32 KernelAddress,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_EXECUTION_DATA_PTR Data
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes, bytes;
    gctUINT32 *command = gcvNULL;
    gctUINT32 i;

    bufferBytes = bytes = 0x180;

    /* Allocate buffer. */
    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    command = (gctUINT32_PTR)bufferLogical;

    /* Fill the data. */
    for (i = 0; i < bufferBytes / 4; i++)
    {
        command[i] = 0;
    }

    for ( i = 0; i < 3; i++)
    {
        command[0] = command[2] = command[3] = command[20] = 0x00000001;
        command[1] = 0x00020001;
        command[8] = command[9] = command[16] = command[19] = 0x00010001;
        command[10] = InImageAddress;
        command[24] = 0x0000240a;
        command[26] = command[28] = 0x03ffffff;
        command[30] = 0x00008100;
        command = command + 32;
    }

    command = (gctUINT32_PTR)bufferLogical;

    command[6] = 0xa0002a1b;
    command[38] = command[70] = 0xa000281b;
    command[12] = command[44] = 0xc0000002;
    command[76] = 0x80000002;
    command[22] = 0x00010016;
    command[54] = command[86] = 0x00010015;
    command[11] = KernelAddress;
    command[43] = KernelAddress + 0x140;
    command[75] = KernelAddress + 0x280;
    command[13] = OutImageAddress;
    command[45] = OutImageAddress + 0x16;
    command[77] = OutImageAddress + 0x2b;

    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

#if gcdDUMP_IN_KERNEL
    gcmkDUMP(Hardware->os, "#[flop reset: TP instruction]");
    gcmkDUMP_BUFFER(
        Hardware->os,
        gcvDUMP_BUFFER_KERNEL_COMMAND,
        bufferLogical,
        bufferAddress,
        bytes
        );
#endif

    Data->bufVidMem = bufferNode;
    Data->bufVidMemBytes = bufferBytes;
    Data->address = bufferAddress;
    Data->logical = bufferLogical;
    Data->bytes = bytes;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

static gceSTATUS
_ProgramTPCommand(
    IN gckHARDWARE Hardware,
    IN gceVIP_ARCH_TYPE ArchType,
    IN gctUINT32 InstAddress,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gckVIDMEM_NODE bufferNode = gcvNULL;
    gctPOINTER bufferLogical = gcvNULL;
    gctUINT32 bufferAddress = 0;
    gctSIZE_T bufferBytes;
    gctUINT32 bytes;
    gctUINT32 *commands;

    gctUINT8_PTR endLogical;
    gctUINT32 endAddress;
    gctUINT32 endBytes = 0;
    gctUINT32 i = 0;
    gctUINT32 k;

    bufferBytes = gcmSIZEOF(gctUINT32) * 64;

    gcmkONERROR(_AllocateVideoMemory(
        Hardware->kernel,
        gcvVIDMEM_TYPE_COMMAND,
        AllocFlag,
        Pool,
        &bufferBytes,
        &bufferNode,
        &bufferLogical,
        &bufferAddress
        ));

    commands = (gctUINT32_PTR)bufferLogical;
    for (i = 0; i < 3; i++)
    {
        k = 14 * i;
        commands[0 + k] = 0x08010e4e;
        commands[1 + k] = 0x00400000;
        commands[2 + k] = 0x08010e4f;
        commands[3 + k] = 0x00000000;
        commands[4 + k] = 0x08010e50;
        commands[5 + k] = 0x00000000;
        commands[6 + k] = 0x08010e53;
        commands[7 + k] = 0x00000000;
        commands[8 + k] = 0x08010e54;
        commands[9 + k] = 0x00000008;
        commands[10 + k] = 0x08010e27;
        commands[11 + k] = 0x00000000;
        commands[12 + k] = 0x0801042e;
    }

    commands[13] = (InstAddress & 0xffffffC0) | (0x1);
    commands[27] = ((InstAddress + 0x80) & 0xffffffC0) | (0x1);
    commands[37] = 0x00000000;
    commands[41] = ((InstAddress + 0x100) & 0xffffffC0);
    commands[42] = 0x08010429;
    commands[43] = 0;
    commands[44] = 0x08010E03;
    commands[45] = 0x20;

    bytes = 46 * 4;

    endLogical = (gctUINT8_PTR)bufferLogical + bytes;
    endAddress = bufferAddress + bytes;

    if (Hardware->wlFE)
    {
        gcmkONERROR(gckWLFE_End(Hardware, gcvNULL, ~0U, &endBytes));
        gcmkONERROR(gckWLFE_End(Hardware, endLogical, endAddress, &endBytes));
    }

    bytes += endBytes;

    gcmkASSERT(bytes <= bufferBytes);


    gcmkONERROR(gckVIDMEM_NODE_CleanCache(
        Hardware->kernel,
        bufferNode,
        0,
        bufferLogical,
        bytes
        ));

    Command->funcVidMem = bufferNode;
    Command->funcVidMemBytes = bufferBytes;
    Command->logical = bufferLogical;
    Command->address = bufferAddress;
    Command->bytes = bytes;
    Command->endAddress = endAddress;
    Command->endLogical = endLogical;

    return gcvSTATUS_OK;

OnError:
    if (bufferNode)
    {
        gcmkONERROR(_FreeVideoMemory(
            Hardware->kernel,
            bufferNode
            ));
    }

    return status;
}

/*
 * TP.
 */
gceSTATUS
gckHARDWARE_ResetFlopWithTP(
    IN gckHARDWARE Hardware,
    IN gctUINT32 AllocFlag,
    IN gcePOOL *Pool,
    OUT gcsFUNCTION_COMMAND_PTR Command
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctUINT32 kernelXSize = TP_KERNEL_XSIZE;
    gctUINT32 kernelYSize = TP_KERNEL_YSIZE;
    gctUINT32 kernelZSize = TP_KERNEL_ZSIZE;

    gctUINT32 inImageXSize = TP_INPUT_XSIZE;
    gctUINT32 inImageYSize = TP_INPUT_YSIZE;
    gctUINT32 inImageZSize = TP_INPUT_ZSIZE;

    gctUINT32 outImageXSize = TP_OUTPUT_XSIZE;
    gctUINT32 outImageYSize = TP_OUTPUT_YSIZE;
    gctUINT32 outImageZSize = TP_OUTPUT_ZSIZE;

    gctUINT32 i;
    gctPOINTER pointer = gcvNULL;

    gceVIP_ARCH_TYPE archType = gcvVIP_ARCH_TYPE_V8;
    gctUINT8 dataType;
    gctUINT32 coreCount = 0;
    gctUINT32 zdp = 1;
    gctUINT32 itemBytes = 0;
    gcsFUNCTION_EXECUTION_DATA_PTR data = gcvNULL;
    gctUINT32 dataCount = 0;

    if (!Command)
    {
        gcmkONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }


    dataType = GCREG_NN_DATA_TYPE_UINT8;

    gcmkONERROR(_GetNNDataSize(dataType, &itemBytes));

    /* Exectution data. */
    dataCount = gcvFLOP_RESET_TP_DATA_NUM;
    gcmkASSERT(dataCount > 0);

    gcmkONERROR(gckOS_Allocate(
        Hardware->os,
        gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount,
        &pointer
        ));
    gckOS_ZeroMemory(pointer, gcmSIZEOF(gcsFUNCTION_EXECUTION_DATA) * dataCount);
    data = (gcsFUNCTION_EXECUTION_DATA *)pointer;

    /* Kernel. */
    gcmkONERROR(_ProgramTPKernel(
        Hardware,
        archType,
        coreCount,
        zdp,
        dataType,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_KERNEL]
        ));

    /* Input. */
    gcmkONERROR(_ProgramTPInput(
        Hardware,
        archType,
        dataType,
        inImageXSize,
        inImageYSize,
        inImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_INPUT]
        ));

    /* Output. */
    gcmkONERROR(_ProgramTPOutput(
        Hardware,
        dataType,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_OUTPUT]
        ));

    /* Commands. */
    gcmkONERROR(_ProgramTPInstruction(
        Hardware,
        archType,
        dataType,
        inImageXSize,
        inImageYSize,
        outImageXSize,
        outImageYSize,
        outImageZSize,
        kernelXSize,
        kernelYSize,
        kernelZSize,
        data[gcvFLOP_RESET_TP_INPUT].address,
        data[gcvFLOP_RESET_TP_OUTPUT].address,
        data[gcvFLOP_RESET_TP_KERNEL].address,
        AllocFlag,
        Pool,
        &data[gcvFLOP_RESET_TP_INSTRUCTION]
        ));

    gcmkONERROR(_ProgramTPCommand(
        Hardware,
        archType,
        data[gcvFLOP_RESET_TP_INSTRUCTION].address,
        AllocFlag,
        Pool,
        Command
        ));

    Command->data = data;
    Command->dataCount = dataCount;

    return gcvSTATUS_OK;

OnError:
    if (Command && Command->funcVidMem)
    {
        gcmkVERIFY_OK(_FreeVideoMemory(
            Hardware->kernel,
            Command->funcVidMem
            ));
        Command->funcVidMem = gcvNULL;
    }

    if (data)
    {
        for (i = 0; i < dataCount; i++)
        {
            if (data[i].bufVidMem)
            {
                gcmkVERIFY_OK(_FreeVideoMemory(
                    Hardware->kernel,
                    data[i].bufVidMem
                    ));
            }
        }

        gcmkVERIFY_OK(gckOS_Free(Hardware->os, data));
    }

    return status;
}
