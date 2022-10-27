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


#ifndef __gc_hal_kernel_hardware_func_flop_reset_config_h_
#define __gc_hal_kernel_hardware_func_flop_reset_config_h_

#include "gc_hal.h"

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
 * Commment: 0x85 WAR does not follow the settings.
 */

#define PPU_IMAGE_XSIZE             64
#define PPU_IMAGE_YSIZE             6
#define PPU_IMAGE_DATA              0x01010101
#define MAX_PPU_INSTRUCTION_COUNT   16
#define MAX_PPU_COMMAND_NUM         128
#define NN_INSTRUCTION_LEN          128
#define NN_INSTRUCTION_LEN_EXT      192
#define TP_INSTRUCTION_LEN          128

#define GCREG_SH_INSTRUCTION_TYPE_INVALID (~0U)

typedef enum _gceFLOP_RESET_PPU_DATA {
    gcvFLOP_RESET_PPU_INSTRUCTION = 0,
    gcvFLOP_RESET_PPU_INPUT       = 1,
    gcvFLOP_RESET_PPU_OUTPUT      = 2,
    gcvFLOP_RESET_PPU_DATA_NUM
} gceFLOP_RESET_PPU_DATA;

/*
 * NN convolution.
 */
#define MAX_NN_COMMAND_NUM          192

#define NN_KERNEL_XSIZE             2
#define NN_KERNEL_YSIZE             2
#define NN_KERNEL_ZSIZE             1

#define NN_INPUT_XSIZE              3
#define NN_INPUT_YSIZE              2
#define NN_INPUT_ZSIZE              1

#define NN_OUTPUT_XSIZE             2
#define NN_OUTPUT_YSIZE             1
#define NN_OUTPUT_ZSIZE             1

typedef enum _gceVIP_ARCH_TYPE {
    gcvVIP_ARCH_TYPE_V6,
    gcvVIP_ARCH_TYPE_V7,
    gcvVIP_ARCH_TYPE_V8
} gceVIP_ARCH_TYPE;

typedef enum _gceFLOP_RESET_NN_DATA {
    gcvFLOP_RESET_NN_INSTRUCTION =  0,
    gcvFLOP_RESET_NN_INPUT       =  1,
    gcvFLOP_RESET_NN_OUTPUT      =  2,
    gcvFLOP_RESET_NN_KERNEL      =  3,
    gcvFLOP_RESET_NN_DATA_NUM
} gceFLOP_RESET_NN_DATA;

#define TP_KERNEL_XSIZE             1
#define TP_KERNEL_YSIZE             1
#define TP_KERNEL_ZSIZE             2
#define TP_KENREL_UNITS             64

#define TP_INPUT_XSIZE              1
#define TP_INPUT_YSIZE              1
#define TP_INPUT_ZSIZE              2

#define TP_OUTPUT_XSIZE             1
#define TP_OUTPUT_YSIZE             64
#define TP_OUTPUT_ZSIZE             1

typedef enum _gceFLOP_RESET_TP_DATA {
    gcvFLOP_RESET_TP_INSTRUCTION =  0,
    gcvFLOP_RESET_TP_INPUT       =  1,
    gcvFLOP_RESET_TP_OUTPUT      =  2,
    gcvFLOP_RESET_TP_KERNEL      =  3,
    gcvFLOP_RESET_TP_DATA_NUM
} gceFLOP_RESET_TP_DATA;

static gctUINT32 flopResetInputs[] = {
        0x33,      /* uint8 */
        0x3266,    /* fp16 */
        0x33,      /* int8 */
        1,         /* uint16 */
        0x33,      /* int16 */
        1,         /* uint4 */
        1,         /* int4 */
        0x3f80     /* bf16 */
    };

typedef struct{
    gctUINT32 ChipID;
    gctUINT32 ChipVersion;
    gctUINT32 ProductID;
    gctUINT32 EcoID;
    gctUINT32 customerID;
    gctUINT32 formalRelease;
    gctUINT8  InputDataType;
    gctUINT32 NNkerLen;
    gctUINT32 NNCmdLen;
    gctUINT32 NNCmdOffset;
    gctUINT32 TPCoreCount;
    gctUINT32 TPkerLen;
    gctUINT32 TPCmdLen;
    gctUINT32 TPCmdOffset[16];
    gctUINT32 NNIns[48];
    gctUINT32 NNKer[256];
    gctUINT32 NNCmd[192];
    gctUINT32 TPIns[256];
    gctUINT32 TPKer[512];
    gctUINT32 TPCmd[280];
} chipCmdData;

static chipCmdData chipcmd[] = {
    {
        0x8000,    /* ChipID */
        0x8002,    /* ChipRevision */
        0x5080009, /* ProductID */
        0x6000000, /* EcoID */
        0x9f,      /* CustomerID */
        0x0,       /*formalRelease*/
        0,         /*InputDataType*/
        0xC0,      /*NNkerLen*/
        88,        /*NNCmdLen*/
        15,        /*NNCmdOffset*/
        3,         /*TPCoreCount*/
        0x3C0,     /*TPkerLen*/
        0x000000C8,
        {
            15, 29, 43
        },
        {
            0xA010004A, 0x001000C0, 0x54000000, 0x00080080,
            0x02084001, 0x037FD200, 0xDFF6F000, 0xDFF6E000,
            0x00012004, 0x00000000, 0x00000000, 0x00000800,
            0x00000880, 0x00000000, 0x00000800, 0x00000000,
            0x00020003, 0x01000002, 0x10000000, 0x03FFFFFF,
            0x00000000, 0x03FFFFFF, 0x00040000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000
        },
        {
            0x01030410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26540781, 0x00000000, 0x00000035,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0xEC024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x0001FE00, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000
        },
        {
            0x08010E4E, 0x00400000, 0x08010E4E, 0x00400000,
            0x08010E4F, 0x00000000, 0x08010E50, 0x00000000,
            0x08010E4C, 0x00000040, 0x08010E54, 0x00000000,
            0x08010E27, 0x00000000, 0x08010428, 0xDFF47000,
            0x08010429, 0x00000000, 0x08010E03, 0x00000C23,
            0x08010E03, 0x00000C23
        },
        {
            0x00000001, 0x00020001, 0x00000001, 0x00000001,
            0x00000000, 0x00000000, 0xA0002A1B, 0x00000000,
            0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62000,
            0xC0000002, 0xDFF6E000, 0x00000000, 0x00000000,
            0x00010001, 0x00000000, 0x00000000, 0x00010001,
            0x00000001, 0x00000000, 0x00010016, 0x00000000,
            0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
            0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
            0x00000001, 0x00020001, 0x00000001, 0x00000001,
            0x00000000, 0x00000000, 0xA000281B, 0x00000000,
            0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62140,
            0xC0000002, 0xDFF6E016, 0x00000000, 0x00000000,
            0x00010001, 0x00000000, 0x00000000, 0x00010001,
            0x00000001, 0x00000000, 0x00010015, 0x00000000,
            0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
            0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
            0x00000001, 0x00020001, 0x00000001, 0x00000001,
            0x00000000, 0x00000000, 0xA000281B, 0x00000000,
            0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62280,
            0x80000002, 0xDFF6E02B, 0x00000000, 0x00000000,
            0x00010001, 0x00000000, 0x00000000, 0x00010001,
            0x00000001, 0x00000000, 0x00010015, 0x00000000,
            0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
            0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000
        },
        {
            0x01150410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26543780, 0x000000FF, 0x0006801A,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x01140410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26543780, 0x000000FF, 0x0006801A,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x01140410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26543780, 0x000000FF, 0x0006801A,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
        },
        {
            0x08010E4E, 0x00400000, 0x08010E4E, 0x00400000,
            0x08010E4F, 0x00000000, 0x08010E50, 0x00000000,
            0x08010E53, 0x00000000, 0x08010E54, 0x00000008,
            0x08010E27, 0x00000000, 0x0801042E, 0xDFF61001,
            0x08010E4E, 0x00400000, 0x08010E4F, 0x00000000,
            0x08010E50, 0x00000000, 0x08010E53, 0x00000000,
            0x08010E54, 0x00000008, 0x08010E27, 0x00000000,
            0x0801042E, 0xDFF61081, 0x08010E4E, 0x00400000,
            0x08010E4F, 0x00000000, 0x08010E50, 0x00000000,
            0x08010E53, 0x00000000, 0x08010E54, 0x00000000,
            0x08010E27, 0x00000000, 0x0801042E, 0xDFF61100,
            0x08010429, 0x00000000, 0x08010E03, 0x00000C23,
            0x08010E03, 0x00000C23
        }
    },
    {
        0x8000,     /* ChipID */
        0x8002,     /* ChipRevision */
        0x5080009,  /* ProductID */
        0x6000000,  /* EcoID */
        0xa0,       /* CustomerID */
        0x0,
        0,          /*InputDataType*/
        0x000000C0,
        0x00000058,
        15,
        3,          /*TPCoreCount*/
        0x000003C0,
        0x000000C8,
        {15, 29, 43},
        {
            0xA010004A, 0x001000C0, 0x54000000, 0x00080080,
            0x02084001, 0x037FD200, 0xDFF6F000, 0xDFF6E000,
            0x00012004, 0x00000000, 0x00000000, 0x00000800,
            0x00000880, 0x00000000, 0x00000800, 0x00000000,
            0x00020003, 0x01000002, 0x10000000, 0x03FFFFFF,
            0x00000000, 0x03FFFFFF, 0x00040000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000
        },
        {
            0x01030410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26540781, 0x00000000, 0x00000035,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0xEC024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x0001FE00, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000
        },
        {
            0x08010E4E, 0x00400000, 0x08010E4E, 0x00400000,
            0x08010E4F, 0x00000000, 0x08010E50, 0x00000000,
            0x08010E4C, 0x00000040, 0x08010E54, 0x00000000,
            0x08010E27, 0x00000000, 0x08010428, 0xDFF47000,
            0x08010429, 0x00000000, 0x08010E03, 0x00000C23,
            0x08010E03, 0x00000C23
        },
        {
            0x00000001, 0x00020001, 0x00000001, 0x00000001,
            0x00000000, 0x00000000, 0xA0002A1B, 0x00000000,
            0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62000,
            0xC0000002, 0xDFF6E000, 0x00000000, 0x00000000,
            0x00010001, 0x00000000, 0x00000000, 0x00010001,
            0x00000001, 0x00000000, 0x00010016, 0x00000000,
            0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
            0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
            0x00000001, 0x00020001, 0x00000001, 0x00000001,
            0x00000000, 0x00000000, 0xA000281B, 0x00000000,
            0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62140,
            0xC0000002, 0xDFF6E016, 0x00000000, 0x00000000,
            0x00010001, 0x00000000, 0x00000000, 0x00010001,
            0x00000001, 0x00000000, 0x00010015, 0x00000000,
            0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
            0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000,
            0x00000001, 0x00020001, 0x00000001, 0x00000001,
            0x00000000, 0x00000000, 0xA000281B, 0x00000000,
            0x00010001, 0x00010001, 0xDFF6F000, 0xDFF62280,
            0x80000002, 0xDFF6E02B, 0x00000000, 0x00000000,
            0x00010001, 0x00000000, 0x00000000, 0x00010001,
            0x00000001, 0x00000000, 0x00010015, 0x00000000,
            0x0000240A, 0x00000000, 0x03FFFFFF, 0x00000000,
            0x03FFFFFF, 0x00000000, 0x00008100, 0x00000000
        },
        {
            0x01150410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26543780, 0x000000FF, 0x0006801A,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x01140410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26543780, 0x000000FF, 0x0006801A,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x01140410, 0x00000100, 0x00000000, 0x00000000,
            0x00000000, 0x26543780, 0x000000FF, 0x0006801A,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00024938, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
            0x00000000, 0x00000000, 0x00000000, 0x00000000,
        },
        {
            0x08010E4E, 0x00400000, 0x08010E4E, 0x00400000,
            0x08010E4F, 0x00000000, 0x08010E50, 0x00000000,
            0x08010E53, 0x00000000, 0x08010E54, 0x00000008,
            0x08010E27, 0x00000000, 0x0801042E, 0xDFF61001,
            0x08010E4E, 0x00400000, 0x08010E4F, 0x00000000,
            0x08010E50, 0x00000000, 0x08010E53, 0x00000000,
            0x08010E54, 0x00000008, 0x08010E27, 0x00000000,
            0x0801042E, 0xDFF61081, 0x08010E4E, 0x00400000,
            0x08010E4F, 0x00000000, 0x08010E50, 0x00000000,
            0x08010E53, 0x00000000, 0x08010E54, 0x00000000,
            0x08010E27, 0x00000000, 0x0801042E, 0xDFF61100,
            0x08010429, 0x00000000, 0x08010E03, 0x00000C23,
            0x08010E03, 0x00000C23
        }
    }
};

static chipCmdData*
gcQuerychipCmdDB(gctUINT32 CustomerID,
                 gctUINT32 ChipID,
                 gctUINT32 ChipVersion,
                 gctUINT32 ProductID,
                 gctUINT32 EcoID)
{
    gctINT entryNum = sizeof(chipcmd) / sizeof(chipcmd[0]);
    gctINT i;

    /* check formal release entries first */
    for (i = 0; i < entryNum; ++i) {
        if (chipcmd[i].customerID == CustomerID &&
            chipcmd[i].ChipID == ChipID && /*chipmodel*/
            chipcmd[i].ChipVersion == ChipVersion &&
            chipcmd[i].ProductID == ProductID &&
            chipcmd[i].EcoID == EcoID &&
            chipcmd[i].formalRelease)
            return &chipcmd[i];
    }

    for (i = 0; i < entryNum; ++i) {
        if (chipcmd[i].customerID == CustomerID &&
            chipcmd[i].ChipID == ChipID &&
            (chipcmd[i].ChipVersion & 0xFFF0) == (ChipVersion & 0xFFF0) &&
            chipcmd[i].ProductID == ProductID &&
            chipcmd[i].EcoID == EcoID &&
            !chipcmd[i].formalRelease)
            return &chipcmd[i];
    }
    return gcvNULL;
}

#endif
