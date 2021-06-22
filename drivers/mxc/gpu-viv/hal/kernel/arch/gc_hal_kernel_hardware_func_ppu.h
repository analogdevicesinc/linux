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


//#include "gc_hal_user_hardware_precomp.h"
//#include "gc_hal_user.h"
/*
**
*/
#if gcdINITIALIZE_PPU

#define gcdRESET_PPU_SH         1


#define INPUT_PPU_IDX            0
#define OUTPUT_PPU_IDX            1
#define INST_PPU_IDX            2

#include "gc_feature_database.h"

#define GCREG_SH_INSTRUCTION_TYPE_INVALID ~0U

#define gcdVX_ENABLE ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))
#define gcdVX_ENABLE4(X, Y, Z, W) ((1 << (X)) | (1 << (Y)) | (1 << (Z)) | (1 << (W)))
#define gcdVX_ENABLE1(X) (1 << (X))
#define gcdVX_ENABLE2(X, Y) ((1 << (X)) | (1 << (Y)))
#define gcdVX_ENABLE3(X, Y, Z) ((1 << (X)) | (1 << (Y)) | (1 << (Z)))
#define gcdVX_SWIZZLE (0 | (1 << 2) | (2 << 4) | (3 << 6))
#define gcdVX_SWIZZLE1(X) ((X) | ((X) << 2) | ((X) << 4) | ((X) << 6))
#define gcdVX_SWIZZLE2(X, Y) ((X) | ((Y) << 2) | ((Y) << 4) | ((Y) << 6))
#define gcdVX_SWIZZLE4(X, Y, Z, W) ((X) | ((Y) << 2) | ((Z) << 4) | ((W) << 6))


#define GETBIT(data, position) (((data) >> (position)) & 0x1)

#define SETBIT(data, position, value)   (\
            ((data) & (~((1ULL) << position)))   \
             |  \
             (((value) << position) & ((1ULL) << position))   \
             )

#define _START(reg_field)       (0 ? reg_field)
#define _END(reg_field)         (1 ? reg_field)
#define _GETSIZE(reg_field)     (_END(reg_field) - _START(reg_field) + 1)
#define _ALIGN(data, reg_field) ((gctUINT32)(data) << _START(reg_field))
#define _MASK(reg_field)        ((_GETSIZE(reg_field) == 32) \
                                    ?  ~0 \
                                    : (gctUINT32)(~((gctUINT64)(~0) << _GETSIZE(reg_field))))

#define AQSETFIELD(data, reg, field, value) \
( \
    ((gctUINT32)(data) & ~_ALIGN(_MASK(reg##_##field), reg##_##field)) \
        | \
    _ALIGN((gctUINT32)(value) & _MASK(reg##_##field), reg##_##field) \
)

#define AQSETFIELDVALUE(data, reg, field, value) \
( \
    ((gctUINT32)(data) & ~_ALIGN(_MASK(reg##_##field), reg##_##field)) \
        | \
    _ALIGN(reg##_##field##_##value & _MASK(reg##_##field), reg##_##field) \
)

gctUINT32 HwFunc_SETBITS(
    IN gctUINT32 Data,
    IN const unsigned int Start,
    IN const unsigned int End,
    IN const gctUINT32 Value
    )
{
    gctUINT32 data = Data;
    if (End >= Start)
    {
        gctUINT32 _Mask =  ((~0ULL >> (63 - End + Start)) << Start);
        data &= ~_Mask;
        data |= ((Value) << Start) & _Mask;
        return data;
    }
    else
    {
        gctUINT32 _Mask =  ((~0ULL >> (63 - Start + End)) << End);
        data &= ~_Mask;
        data |= ((Value) << End) & _Mask;
        return data;
    }
}

gctUINT32 HwFunc_GETBITS(
    IN gctUINT32 Data,
    IN const unsigned int Start,
    IN const unsigned int End
    )
{
    gctUINT32 data = Data;
    if (End >= Start)
    {
        gctUINT32 _Mask = (~0ULL >> (63 - (End - Start)));
        return (data >> Start) & _Mask;
    }
    else
    {
        gctUINT32 _Mask = (~0ULL >> (63 - (Start - End)));
        return (data >> End) & _Mask;;
    }
}

gceSTATUS
_InitializePPU_SetImmediate(
    IN gctUINT32                            Where,
    IN gctUINT32                            Value,
    IN gctUINT32                            Type,
    IN OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Where=0x%x", Where);

    switch (Where)
    {
        case 0:
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_ADR, HwFunc_GETBITS(Value, 8, 0));
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_SWIZZLE, HwFunc_GETBITS(Value, 16, 9));
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_MODIFIER_NEG, GETBIT(Value, 17));
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_MODIFIER_ABS, GETBIT(Value, 18));
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC0_REL_ADR, GETBIT(Value, 19) | (Type << 1));
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC0_TYPE, AQ_SHADER_SRC_REG_TYPE_IMMEDIATE);
            break;

        case 1:
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_ADR, HwFunc_GETBITS(Value, 8, 0));
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_SWIZZLE, HwFunc_GETBITS(Value, 16, 9));
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_MODIFIER_NEG, GETBIT(Value, 17));
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_MODIFIER_ABS, GETBIT(Value, 18));
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_REL_ADR, GETBIT(Value, 19) | (Type << 1));
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC1_TYPE, AQ_SHADER_SRC_REG_TYPE_IMMEDIATE);
            break;

        case 2:
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_ADR, HwFunc_GETBITS(Value, 8, 0));
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_SWIZZLE, HwFunc_GETBITS(Value, 16, 9));
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_MODIFIER_NEG, GETBIT(Value, 17));
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_MODIFIER_ABS, GETBIT(Value, 18));
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_REL_ADR, GETBIT(Value, 19) | (Type << 1));
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_TYPE, AQ_SHADER_SRC_REG_TYPE_IMMEDIATE);
            break;
    }

    gcmkFOOTER();
    return status;
}

gceSTATUS
_InitializePPU_SetInstructionType(
    IN gctUINT32                            Type,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Type=0x%x", Type);

    binary[1] = AQSETFIELD(binary[1], AQ_INST, INST_TYPE_0, GETBIT(Type, 0));
    binary[2] = AQSETFIELD(binary[2], AQ_INST, INST_TYPE_1, HwFunc_GETBITS(Type, 2, 1));

    gcmkFOOTER();
    return status;
}


gceSTATUS
_InitializePPU_IsEndOfBB(
    IN gckHARDWARE                            Hardware,
    IN gctUINT32                            Opcode,
    OUT gctUINT32_PTR                        binary
)
{
    gceSTATUS status = gcvSTATUS_OK;
    gcmkHEADER_ARG("Opcode=0x%x", Opcode);

    if (binary != NULL)
    {
        if (((gcsFEATURE_DATABASE *)Hardware->featureDatabase)->SH_END_OF_BB)
        {
            switch (Opcode)
            {
            case AQ_INST_OP_CODE_MOV:
            case AQ_INST_OP_CODE_MOVAI:
            case AQ_INST_OP_CODE_MOVAR:
            case AQ_INST_OP_CODE_MOVAF:
            case AQ_INST_OP_CODE_SELECT:
            case AQ_INST_OP_CODE_CMP:
            case AQ_INST_OP_CODE_SET:
                binary[1] = AQSETFIELD(binary[1], AQ_INST, SAMPLER_SWIZZLE,
                    HwFunc_SETBITS(binary[1], GCREG_SH_END_OF_BASIC_BLOCK_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_CC_VER_End, 1));
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
                binary[0] = AQSETFIELD(binary[0], AQ_INST, CONDITION_CODE,
                    HwFunc_SETBITS(binary[0], GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_End, 1));
                break;
            case AQ_INST_OP_CODE_LOAD:
            case AQ_INST_OP_CODE_LOADP:
            case AQ_INST_OP_CODE_STORE:
            case AQ_INST_OP_CODE_STOREP:
            case AQ_INST_OP_CODE_IMG_LOAD:
            case AQ_INST_OP_CODE_IMG_LOAD_3D:
            case AQ_INST_OP_CODE_IMG_STORE:
            case AQ_INST_OP_CODE_IMG_STORE_3D:
                binary[0] = AQSETFIELD(binary[0], AQ_INST, CONDITION_CODE,
                    HwFunc_SETBITS(binary[0], GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_End, 1));
                break;
            default:
                if (Opcode != AQ_INST_OP_CODE_BRANCH &&
                    Opcode != AQ_INST_OP_CODE_BRANCH_ANY &&
                    Opcode != AQ_INST_OP_CODE_CALL &&
                    Opcode != AQ_INST_OP_CODE_RET &&
                    Opcode != AQ_INST_OP_CODE_TEXKILL)
                    binary[0] = AQSETFIELD(binary[0], AQ_INST, CONDITION_CODE,
                        HwFunc_SETBITS(binary[0], GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_Start, GCREG_SH_END_OF_BASIC_BLOCK_NO_CC_VER_End, 1));
                break;
            }
        }
    }

//OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}
gceSTATUS
_InitializePPU_AddOpcode(
    IN gckHARDWARE                            Hardware,
    IN gctUINT32                            Opcode,
    IN gctUINT32                            Extended,
    IN gctINT32                             Type,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Opcode=0x%x", Opcode);

    binary[0] = AQSETFIELD(binary[0], AQ_INST, OP_CODE, HwFunc_GETBITS(Opcode, 5, 0));
    binary[2] = AQSETFIELD(binary[2], AQ_INST, OP_CODE_MSB6, GETBIT(Opcode, 6));
    if (Opcode == AQ_INST_OP_CODE_EXTENDED)
    {
        gcmkONERROR(_InitializePPU_SetImmediate(2, Extended, GCREG_SH_IMMEDIATE_TYPE_U20, binary));
    }
    else if (Opcode == AQ_INST_OP_CODE_EVIS)
    {
        binary[0] = AQSETFIELD(binary[0], AQ_INST, DEST_REL_ADR, HwFunc_GETBITS(Extended, 2, 0));
        binary[0] = SETBIT(binary[0], 31, GETBIT(Extended, 3));
        binary[1] = HwFunc_SETBITS(binary[1], 1, 0, HwFunc_GETBITS(Extended, 5, 4));
    }
    else if (Opcode == AQ_INST_OP_CODE_CMP || Opcode == AQ_INST_OP_CODE_MOV || Opcode == AQ_INST_OP_CODE_SELECT)
    {
        binary[0] = AQSETFIELD(binary[0], AQ_INST, CONDITION_CODE, HwFunc_GETBITS(Extended, 4, 0));
    }

    if((gctUINT32)Type != GCREG_SH_INSTRUCTION_TYPE_INVALID)
        gcmkONERROR(_InitializePPU_SetInstructionType(Type, binary));

    gcmkONERROR(_InitializePPU_IsEndOfBB(Hardware, Opcode, binary));

OnError:
    /* Return the status. */
    gcmkFOOTER();
    return status;
}

gceSTATUS
_InitializePPU_SetDestination(
    IN gctUINT32                            Address,
    IN gctUINT32                            Enable,
    IN gctBOOL                              Saturate,
    IN OUT gctUINT32_PTR                    binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Address=0x%x", Address);

    binary[0] = AQSETFIELD(binary[0], AQ_INST, DEST_VALID, 1);
    binary[0] = AQSETFIELD(binary[0], AQ_INST, DEST_ADR, Address);
    binary[0] = AQSETFIELD(binary[0], AQ_INST, DEST_WRITE_ENABLE, Enable);
    binary[0] = AQSETFIELD(binary[0], AQ_INST, SATURATE, Saturate);

    gcmkFOOTER();
    return status;
}

#define gcdVX_ENABLE ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))

static gctUINT32 _InitializePPU_GetPixel(gctUINT32 format)
{
    gctUINT32 pixel = 0;
    switch(format)
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
_InitializePPU_SetEVIS(
    IN gctUINT32                            Start,
    IN gctUINT32                            End,
    IN gctUINT32                            Evis,
    IN OUT gctUINT32_PTR                    binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Evis=0x%x", Evis);

    binary[0] = AQSETFIELD(binary[0], AQ_INST, DEST_WRITE_ENABLE, Start);
    binary[0] = HwFunc_SETBITS(binary[0], 30, 27, End);
    binary[1] = HwFunc_SETBITS(binary[1], 10, 2, Evis);

    gcmkFOOTER();
    return status;
}

#define HwFunc_SETSOURCE(Where, Common_I, REL_ADR_I, TYPE_I, Address, Swizzle, Type, Negate, Absolute, Relative, Binary) \
    binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRCWhere_VALID, 1); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_ADR, Address); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_SWIZZLE, Swizzle); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_MODIFIER_NEG, Negate); \
            binary[Common_I] = AQSETFIELD(binary[Common_I], AQ_INST, SRC0_MODIFIER_ABS, Absolute); \
            binary[REL_ADR_I] = AQSETFIELD(binary[REL_ADR_I], AQ_INST, SRC0_REL_ADR, Relative); \
            binary[TYPE_I] = AQSETFIELD(binary[TYPE_I], AQ_INST, SRC0_TYPE, Type);

gceSTATUS
_InitializePPU_SetSource(
    IN gctUINT32                            Where,
    IN gctUINT32                            Address,
    IN gctUINT32                            Swizzle,
    IN gctUINT32                            Type,
    IN gctBOOL                              Negate,
    IN gctBOOL                              Absolute,
    IN gctUINT32                            Relative,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("Where=0x%x", Where);
    switch (Where)
    {
        case 0:
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_VALID, 1);
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_ADR, Address);
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_SWIZZLE, Swizzle);
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_MODIFIER_NEG, Negate);
            binary[1] = AQSETFIELD(binary[1], AQ_INST, SRC0_MODIFIER_ABS, Absolute);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC0_REL_ADR, Relative);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC0_TYPE, Type);
            break;

        case 1:
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_VALID, 1);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_ADR, Address);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_SWIZZLE, Swizzle);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_MODIFIER_NEG, Negate);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_MODIFIER_ABS, Absolute);
            binary[2] = AQSETFIELD(binary[2], AQ_INST, SRC1_REL_ADR, Relative);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC1_TYPE, Type);
            break;

        case 2:
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_VALID, 1);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_ADR, Address);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_SWIZZLE, Swizzle);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_MODIFIER_NEG, Negate);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_MODIFIER_ABS, Absolute);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_REL_ADR, Relative);
            binary[3] = AQSETFIELD(binary[3], AQ_INST, SRC2_TYPE, Type);
            break;
    }
    gcmkFOOTER();
    return status;
}

static const gctUINT32 NEGATE_FLAG   = 1 << 0;
static const gctUINT32 ABSOLUTE_FLAG = 1 << 1;

gceSTATUS
_InitializePPU_SetUniform(
    IN gctUINT32                            Where,
    IN gctUINT32                            Address,
    IN gctUINT32                            Swizzle,
    IN gctUINT32                            Modifiers,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate   = (Modifiers & NEGATE_FLAG  ) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;
    gcmkHEADER_ARG("Where=0x%x", Where);

    gcmkONERROR(_InitializePPU_SetSource(Where, Address, Swizzle, AQ_SHADER_SRC_REG_TYPE_UNBOUNDED_CONST, negate, absolute, 0, binary));

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS
_InitializePPU_SetTempReg(
    IN gctUINT32                            Where,
    IN gctUINT32                            Address,
    IN gctUINT32                            Swizzle,
    IN gctUINT32                            Modifiers,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gctBOOL negate   = (Modifiers & NEGATE_FLAG  ) ? gcvTRUE : gcvFALSE;
    gctBOOL absolute = (Modifiers & ABSOLUTE_FLAG) ? gcvTRUE : gcvFALSE;
    gcmkHEADER_ARG("Where=0x%x", Where);

    gcmkONERROR(_InitializePPU_SetSource(Where, Address, Swizzle, AQ_SHADER_SRC_REG_TYPE_TEMP, negate, absolute, 0, binary));

OnError:
    gcmkFOOTER();
    return status;
}


gceSTATUS
_InitializePPU_SetSourceBin(
    IN gctUINT32                            SourceBin,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;

    gcmkHEADER_ARG("SourceBin=0x%x", SourceBin);

    binary[1] = HwFunc_SETBITS(binary[1], 25, 22, SourceBin);

    gcmkFOOTER();
    return status;
}


gceSTATUS
_InitializePPU_SetImmediateValue(
    IN gctUINT32                            Where,
    IN gctUINT32                            Value,
    OUT gctUINT32_PTR                        binary
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctINT32 raw = AQSETFIELD(0, GCREG_SH_IMMEDIATE, VALUE, Value) | AQSETFIELDVALUE(0, GCREG_SH_IMMEDIATE, TYPE, U20);

    gcmkHEADER_ARG("Where=0x%x", Where);

    gcmkONERROR(_InitializePPU_SetSource(Where, HwFunc_GETBITS(raw, 8, 0), HwFunc_GETBITS(raw, 16, 9), AQ_SHADER_SRC_REG_TYPE_IMMEDIATE, GETBIT(raw, 17), GETBIT(raw, 18), HwFunc_GETBITS(raw, 21, 19), binary));

OnError:
    gcmkFOOTER();
    return status;
}

gceSTATUS gcoHwFunc_SH_CMD(
    IN gckHARDWARE            Hardware,
    IN gctUINT32            Data_type,
    IN OUT gctUINT32_PTR    binarys,
    OUT  gctUINT32_PTR        command_count,
    OUT  gctUINT32_PTR        reg_count
    )
{
    gceSTATUS status = gcvSTATUS_OK;
    gctUINT32 count = 0;
    gctUINT32 Input1 = Data_type;
    gctUINT32 Input2 = Data_type;
    gctUINT32 Output = Data_type;

    gcmkHEADER_ARG("binarys=0x%x", binarys);

    /* a. DP instruction with all bin */
    /* b. Store to 6 line which size are 64 bytes and flush out */

    /*img_load.u8 r1, c0, r0.xy*/
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, AQ_INST_OP_CODE_IMG_LOAD, 0, Input1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, _InitializePPU_GetPixel(Input1), 1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &binarys[count]));
    count += 4;

    /*img_load.u8 r2, c0, r0.xy */
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, AQ_INST_OP_CODE_IMG_LOAD, 0, Input2, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetDestination(2, gcdVX_ENABLE, gcvFALSE, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, _InitializePPU_GetPixel(Input2), 1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetUniform(0, 0, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &binarys[count]));
    count += 4;

    /* dp2x8 r1, r1, r2, c3_512 */
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, AQ_INST_OP_CODE_EVIS, GCREG_SH_VISION_OPCODE_DP2X8, Output, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetDestination(1, gcdVX_ENABLE, gcvFALSE, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, 7, (Input1 | (Input2 << 3)), &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(0, 1, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 2, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetSource (2, 2, gcdVX_SWIZZLE, AQ_SHADER_SRC_REG_TYPE_UNIFORM512, gcvFALSE, gcvFALSE, 0, &binarys[count]));
    count += 4;


    /*img_store.u8 r1, c2, r0.xy, r1*/
    gcmkONERROR(_InitializePPU_AddOpcode(Hardware, AQ_INST_OP_CODE_IMG_STORE, 0, Output, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetEVIS(0, _InitializePPU_GetPixel(Output), 1, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetUniform(0, 1, gcdVX_SWIZZLE, 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(1, 0, gcdVX_SWIZZLE2(0, 1), 0, &binarys[count]));
    gcmkONERROR(_InitializePPU_SetTempReg(2, 1, gcdVX_SWIZZLE, 0, &binarys[count]));
    count += 4;

    *command_count = count;
    *reg_count = 0x3;
OnError:
    gcmkFOOTER();
    return status;
}

#endif /*gcdINITIALIZE_PPU*/

