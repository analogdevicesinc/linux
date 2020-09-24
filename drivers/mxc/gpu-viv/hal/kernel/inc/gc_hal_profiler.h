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


#ifndef __gc_hal_profiler_h_
#define __gc_hal_profiler_h_

#include "shared/gc_hal_profiler_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

#define GLVERTEX_OBJECT 10
#define GLVERTEX_OBJECT_BYTES 11

#define GLINDEX_OBJECT 20
#define GLINDEX_OBJECT_BYTES 21

#define GLTEXTURE_OBJECT 30
#define GLTEXTURE_OBJECT_BYTES 31

#define GLBUFOBJ_OBJECT 40
#define GLBUFOBJ_OBJECT_BYTES 41

#define    ES11_CALLS              151
#define    ES11_DRAWCALLS          (ES11_CALLS             + 1)
#define    ES11_STATECHANGECALLS   (ES11_DRAWCALLS         + 1)
#define    ES11_POINTCOUNT         (ES11_STATECHANGECALLS  + 1)
#define    ES11_LINECOUNT          (ES11_POINTCOUNT        + 1)
#define    ES11_TRIANGLECOUNT      (ES11_LINECOUNT         + 1)

#define    ES30_CALLS              159
#define    ES30_DRAWCALLS          (ES30_CALLS             + 1)
#define    ES30_STATECHANGECALLS   (ES30_DRAWCALLS         + 1)
#define    ES30_POINTCOUNT         (ES30_STATECHANGECALLS  + 1)
#define    ES30_LINECOUNT          (ES30_POINTCOUNT        + 1)
#define    ES30_TRIANGLECOUNT      (ES30_LINECOUNT         + 1)

#define    VG11_CALLS              88
#define    VG11_DRAWCALLS          (VG11_CALLS              + 1)
#define    VG11_STATECHANGECALLS   (VG11_DRAWCALLS          + 1)
#define    VG11_FILLCOUNT          (VG11_STATECHANGECALLS   + 1)
#define    VG11_STROKECOUNT        (VG11_FILLCOUNT          + 1)
/* End of Driver API ID Definitions. */

/* HAL & MISC IDs. */
#define HAL_VERTBUFNEWBYTEALLOC    1
#define HAL_VERTBUFTOTALBYTEALLOC  (HAL_VERTBUFNEWBYTEALLOC     + 1)
#define HAL_VERTBUFNEWOBJALLOC     (HAL_VERTBUFTOTALBYTEALLOC   + 1)
#define HAL_VERTBUFTOTALOBJALLOC   (HAL_VERTBUFNEWOBJALLOC      + 1)
#define HAL_INDBUFNEWBYTEALLOC     (HAL_VERTBUFTOTALOBJALLOC    + 1)
#define HAL_INDBUFTOTALBYTEALLOC   (HAL_INDBUFNEWBYTEALLOC      + 1)
#define HAL_INDBUFNEWOBJALLOC      (HAL_INDBUFTOTALBYTEALLOC    + 1)
#define HAL_INDBUFTOTALOBJALLOC    (HAL_INDBUFNEWOBJALLOC       + 1)
#define HAL_TEXBUFNEWBYTEALLOC     (HAL_INDBUFTOTALOBJALLOC     + 1)
#define HAL_TEXBUFTOTALBYTEALLOC   (HAL_TEXBUFNEWBYTEALLOC      + 1)
#define HAL_TEXBUFNEWOBJALLOC      (HAL_TEXBUFTOTALBYTEALLOC    + 1)
#define HAL_TEXBUFTOTALOBJALLOC    (HAL_TEXBUFNEWOBJALLOC       + 1)

#define GPU_CYCLES           1
#define GPU_READ64BYTE       (GPU_CYCLES         + 1)
#define GPU_WRITE64BYTE      (GPU_READ64BYTE     + 1)
#define GPU_TOTALCYCLES      (GPU_WRITE64BYTE    + 1)
#define GPU_IDLECYCLES       (GPU_TOTALCYCLES    + 1)

#define VS_INSTCOUNT          1
#define VS_BRANCHINSTCOUNT    (VS_INSTCOUNT          + 1)
#define VS_TEXLDINSTCOUNT     (VS_BRANCHINSTCOUNT    + 1)
#define VS_RENDEREDVERTCOUNT  (VS_TEXLDINSTCOUNT     + 1)
#define VS_SOURCE             (VS_RENDEREDVERTCOUNT  + 1)
#define VS_NONIDLESTARVECOUNT (VS_SOURCE             + 1)
#define VS_STARVELCOUNT       (VS_NONIDLESTARVECOUNT + 1)
#define VS_STALLCOUNT         (VS_STARVELCOUNT       + 1)
#define VS_PROCESSCOUNT       (VS_STALLCOUNT         + 1)

#define PS_INSTCOUNT          1
#define PS_BRANCHINSTCOUNT    (PS_INSTCOUNT          + 1)
#define PS_TEXLDINSTCOUNT     (PS_BRANCHINSTCOUNT    + 1)
#define PS_RENDEREDPIXCOUNT   (PS_TEXLDINSTCOUNT     + 1)
#define PS_SOURCE             (PS_RENDEREDPIXCOUNT   + 1)
#define PS_NONIDLESTARVECOUNT (PS_SOURCE             + 1)
#define PS_STARVELCOUNT       (PS_NONIDLESTARVECOUNT + 1)
#define PS_STALLCOUNT         (PS_STARVELCOUNT       + 1)
#define PS_PROCESSCOUNT       (PS_STALLCOUNT         + 1)
#define PS_SHADERCYCLECOUNT   (PS_PROCESSCOUNT       + 1)

#define PA_INVERTCOUNT        1
#define PA_INPRIMCOUNT        (PA_INVERTCOUNT      + 1)
#define PA_OUTPRIMCOUNT       (PA_INPRIMCOUNT      + 1)
#define PA_DEPTHCLIPCOUNT     (PA_OUTPRIMCOUNT     + 1)
#define PA_TRIVIALREJCOUNT    (PA_DEPTHCLIPCOUNT   + 1)
#define PA_CULLCOUNT          (PA_TRIVIALREJCOUNT  + 1)
#define PA_NONIDLESTARVECOUNT (PA_CULLCOUNT        + 1)
#define PA_STARVELCOUNT       (PA_NONIDLESTARVECOUNT + 1)
#define PA_STALLCOUNT         (PA_STARVELCOUNT     + 1)
#define PA_PROCESSCOUNT       (PA_STALLCOUNT       + 1)

#define SE_TRIANGLECOUNT        1
#define SE_LINECOUNT            (SE_TRIANGLECOUNT        + 1)
#define SE_STARVECOUNT          (SE_LINECOUNT            + 1)
#define SE_STALLCOUNT           (SE_STARVECOUNT          + 1)
#define SE_RECEIVETRIANGLECOUNT (SE_STALLCOUNT           + 1)
#define SE_SENDTRIANGLECOUNT    (SE_RECEIVETRIANGLECOUNT + 1)
#define SE_RECEIVELINESCOUNT    (SE_SENDTRIANGLECOUNT    + 1)
#define SE_SENDLINESCOUNT       (SE_RECEIVELINESCOUNT    + 1)
#define SE_NONIDLESTARVECOUNT   (SE_SENDLINESCOUNT       + 1)
#define SE_PROCESSCOUNT         (SE_NONIDLESTARVECOUNT   + 1)

#define RA_VALIDPIXCOUNT      1
#define RA_TOTALQUADCOUNT     (RA_VALIDPIXCOUNT      + 1)
#define RA_VALIDQUADCOUNTEZ   (RA_TOTALQUADCOUNT     + 1)
#define RA_TOTALPRIMCOUNT     (RA_VALIDQUADCOUNTEZ   + 1)
#define RA_PIPECACHEMISSCOUNT (RA_TOTALPRIMCOUNT     + 1)
#define RA_PREFCACHEMISSCOUNT (RA_PIPECACHEMISSCOUNT + 1)
#define RA_EEZCULLCOUNT       (RA_PREFCACHEMISSCOUNT + 1)
#define RA_NONIDLESTARVECOUNT (RA_EEZCULLCOUNT       + 1)
#define RA_STARVELCOUNT       (RA_NONIDLESTARVECOUNT + 1)
#define RA_STALLCOUNT         (RA_STARVELCOUNT       + 1)
#define RA_PROCESSCOUNT       (RA_STALLCOUNT         + 1)

#define TX_TOTBILINEARREQ     1
#define TX_TOTTRILINEARREQ    (TX_TOTBILINEARREQ      + 1)
#define TX_TOTDISCARDTEXREQ   (TX_TOTTRILINEARREQ     + 1)
#define TX_TOTTEXREQ          (TX_TOTDISCARDTEXREQ    + 1)
#define TX_MEMREADCOUNT       (TX_TOTTEXREQ           + 1)
#define TX_MEMREADIN8BCOUNT   (TX_MEMREADCOUNT        + 1)
#define TX_CACHEMISSCOUNT     (TX_MEMREADIN8BCOUNT    + 1)
#define TX_CACHEHITTEXELCOUNT (TX_CACHEMISSCOUNT      + 1)
#define TX_CACHEMISSTEXELCOUNT (TX_CACHEHITTEXELCOUNT + 1)
#define TX_NONIDLESTARVECOUNT  (TX_CACHEMISSTEXELCOUNT+ 1)
#define TX_STARVELCOUNT        (TX_NONIDLESTARVECOUNT + 1)
#define TX_STALLCOUNT          (TX_STARVELCOUNT       + 1)
#define TX_PROCESSCOUNT        (TX_STALLCOUNT         + 1)

#define PE_KILLEDBYCOLOR      1
#define PE_KILLEDBYDEPTH      (PE_KILLEDBYCOLOR    + 1)
#define PE_DRAWNBYCOLOR       (PE_KILLEDBYDEPTH    + 1)
#define PE_DRAWNBYDEPTH       (PE_DRAWNBYCOLOR     + 1)

#define MC_READREQ8BPIPE      1
#define MC_READREQ8BIP        (MC_READREQ8BPIPE    + 1)
#define MC_WRITEREQ8BPIPE     (MC_READREQ8BIP      + 1)
#define MC_AXIMINLATENCY      (MC_WRITEREQ8BPIPE   + 1)
#define MC_AXIMAXLATENCY      (MC_AXIMINLATENCY    + 1)
#define MC_AXITOTALLATENCY    (MC_AXIMAXLATENCY    + 1)
#define MC_AXISAMPLECOUNT     (MC_AXITOTALLATENCY  + 1)

#define AXI_READREQSTALLED    1
#define AXI_WRITEREQSTALLED   (AXI_READREQSTALLED  + 1)
#define AXI_WRITEDATASTALLED  (AXI_WRITEREQSTALLED + 1)

#define FE_DRAWCOUNT    1
#define FE_OUTVERTEXCOUNT     (FE_DRAWCOUNT  + 1)
#define FE_STALLCOUNT         (FE_OUTVERTEXCOUNT + 1)
#define FE_STARVECOUNT        (FE_STALLCOUNT + 1)

#define PVS_INSTRCOUNT        1
#define PVS_ALUINSTRCOUNT     (PVS_INSTRCOUNT      + 1)
#define PVS_TEXINSTRCOUNT     (PVS_ALUINSTRCOUNT   + 1)
#define PVS_ATTRIBCOUNT       (PVS_TEXINSTRCOUNT   + 1)
#define PVS_UNIFORMCOUNT      (PVS_ATTRIBCOUNT     + 1)
#define PVS_FUNCTIONCOUNT     (PVS_UNIFORMCOUNT    + 1)
#define PVS_SOURCE            (PVS_FUNCTIONCOUNT   + 1)

#define PPS_INSTRCOUNT       1
#define PPS_ALUINSTRCOUNT    (PPS_INSTRCOUNT       + 1)
#define PPS_TEXINSTRCOUNT    (PPS_ALUINSTRCOUNT    + 1)
#define PPS_ATTRIBCOUNT      (PPS_TEXINSTRCOUNT    + 1)
#define PPS_UNIFORMCOUNT     (PPS_ATTRIBCOUNT      + 1)
#define PPS_FUNCTIONCOUNT    (PPS_UNIFORMCOUNT     + 1)
#define PPS_SOURCE           (PPS_FUNCTIONCOUNT    + 1)
/* End of MISC Counter IDs. */


/* Category Constants. */
#define VPHEADER        0x010000
#define VPG_INFO        0x020000
#define VPG_TIME        0x030000
#define VPG_MEM         0x040000
#define VPG_ES11        0x050000
#define VPG_ES30        0x060000
#define VPG_VG11        0x070000
#define VPG_HAL         0x080000
#define VPG_HW          0x090000
#define VPG_GPU         0x0a0000
#define VPG_VS          0x0b0000
#define VPG_PS          0x0c0000
#define VPG_PA          0x0d0000
#define VPG_SETUP       0x0e0000
#define VPG_RA          0x0f0000
#define VPG_TX          0x100000
#define VPG_PE          0x110000
#define VPG_MC          0x120000
#define VPG_AXI         0x130000
#define VPG_PROG        0x140000
#define VPG_PVS         0x150000
#define VPG_PPS         0x160000
#define VPG_ES11_TIME   0x170000
#define VPG_ES30_TIME   0x180000
#define VPG_FRAME       0x190000
#define VPG_ES11_DRAW   0x200000
#define VPG_ES30_DRAW   0x210000
#define VPG_VG11_TIME   0x220000
#define VPG_FE          0x230000
#define VPG_MULTI_GPU   0x240000
#define VPNG_FE         0x250000
#define VPNG_VS         0x260000
#define VPNG_PS         0x270000
#define VPNG_PA         0x280000
#define VPNG_SETUP      0x290000
#define VPNG_RA         0x2a0000
#define VPNG_TX         0x2b0000
#define VPNG_PE         0x2c0000
#define VPNG_MCC        0x2d0000
#define VPNG_MCZ        0x2e0000
#define VPNG_HI         0x2f0000
#define VPNG_L2         0x300000
#define VPG_FINISH      0x310000
#define VPG_END         0xff0000

/* Info. */
#define VPC_INFOCOMPANY         (VPG_INFO + 1)
#define VPC_INFOVERSION         (VPC_INFOCOMPANY + 1)
#define VPC_INFORENDERER        (VPC_INFOVERSION + 1)
#define VPC_INFOREVISION        (VPC_INFORENDERER + 1)
#define VPC_INFODRIVER          (VPC_INFOREVISION + 1)
#define VPC_INFODRIVERMODE      (VPC_INFODRIVER + 1)
#define VPC_INFOSCREENSIZE      (VPC_INFODRIVERMODE + 1)

/* Counter Constants. */
#define VPC_ELAPSETIME          (VPG_TIME + 1)
#define VPC_CPUTIME             (VPC_ELAPSETIME + 1)

#define VPC_MEMMAXRES           (VPG_MEM + 1)
#define VPC_MEMSHARED           (VPC_MEMMAXRES + 1)
#define VPC_MEMUNSHAREDDATA     (VPC_MEMSHARED + 1)
#define VPC_MEMUNSHAREDSTACK    (VPC_MEMUNSHAREDDATA + 1)

/* OpenGL ES11 Statics Counter IDs. */
#define    VPC_ES11CALLS            (VPG_ES11 +    ES11_CALLS)
#define    VPC_ES11DRAWCALLS        (VPG_ES11 +    ES11_DRAWCALLS)
#define    VPC_ES11STATECHANGECALLS (VPG_ES11 +    ES11_STATECHANGECALLS)
#define    VPC_ES11POINTCOUNT       (VPG_ES11 +    ES11_POINTCOUNT)
#define    VPC_ES11LINECOUNT        (VPG_ES11 +    ES11_LINECOUNT)
#define    VPC_ES11TRIANGLECOUNT    (VPG_ES11 +    ES11_TRIANGLECOUNT)

/* OpenGL ES30 Statistics Counter IDs. */
#define    VPC_ES30CALLS            (VPG_ES30 +    ES30_CALLS)
#define    VPC_ES30DRAWCALLS        (VPG_ES30 +    ES30_DRAWCALLS)
#define    VPC_ES30STATECHANGECALLS (VPG_ES30 +    ES30_STATECHANGECALLS)
#define    VPC_ES30POINTCOUNT       (VPG_ES30 +    ES30_POINTCOUNT)
#define    VPC_ES30LINECOUNT        (VPG_ES30 +    ES30_LINECOUNT)
#define    VPC_ES30TRIANGLECOUNT    (VPG_ES30 +    ES30_TRIANGLECOUNT)

/* OpenVG Statistics Counter IDs. */
#define    VPC_VG11CALLS            (VPG_VG11 +    VG11_CALLS)
#define    VPC_VG11DRAWCALLS        (VPG_VG11 +    VG11_DRAWCALLS)
#define    VPC_VG11STATECHANGECALLS (VPG_VG11 +    VG11_STATECHANGECALLS)
#define    VPC_VG11FILLCOUNT        (VPG_VG11 +    VG11_FILLCOUNT)
#define    VPC_VG11STROKECOUNT      (VPG_VG11 +    VG11_STROKECOUNT)

/* HAL Counters. */
#define VPC_HALVERTBUFNEWBYTEALLOC      (VPG_HAL + HAL_VERTBUFNEWBYTEALLOC)
#define VPC_HALVERTBUFTOTALBYTEALLOC    (VPG_HAL + HAL_VERTBUFTOTALBYTEALLOC)
#define VPC_HALVERTBUFNEWOBJALLOC       (VPG_HAL + HAL_VERTBUFNEWOBJALLOC)
#define VPC_HALVERTBUFTOTALOBJALLOC     (VPG_HAL + HAL_VERTBUFTOTALOBJALLOC)
#define VPC_HALINDBUFNEWBYTEALLOC       (VPG_HAL + HAL_INDBUFNEWBYTEALLOC)
#define VPC_HALINDBUFTOTALBYTEALLOC     (VPG_HAL + HAL_INDBUFTOTALBYTEALLOC)
#define VPC_HALINDBUFNEWOBJALLOC        (VPG_HAL + HAL_INDBUFNEWOBJALLOC)
#define VPC_HALINDBUFTOTALOBJALLOC      (VPG_HAL + HAL_INDBUFTOTALOBJALLOC)
#define VPC_HALTEXBUFNEWBYTEALLOC       (VPG_HAL + HAL_TEXBUFNEWBYTEALLOC)
#define VPC_HALTEXBUFTOTALBYTEALLOC     (VPG_HAL + HAL_TEXBUFTOTALBYTEALLOC)
#define VPC_HALTEXBUFNEWOBJALLOC        (VPG_HAL + HAL_TEXBUFNEWOBJALLOC)
#define VPC_HALTEXBUFTOTALOBJALLOC      (VPG_HAL + HAL_TEXBUFTOTALOBJALLOC)

/* HW: GPU Counters. */
#define VPC_GPUCYCLES                   (VPG_GPU + GPU_CYCLES)
#define VPC_GPUREAD64BYTE               (VPG_GPU + GPU_READ64BYTE)
#define VPC_GPUWRITE64BYTE              (VPG_GPU + GPU_WRITE64BYTE)
#define VPC_GPUTOTALCYCLES              (VPG_GPU + GPU_TOTALCYCLES)
#define VPC_GPUIDLECYCLES               (VPG_GPU + GPU_IDLECYCLES)

/* HW: Shader Counters. */
#define VPC_VSINSTCOUNT                 (VPG_VS + VS_INSTCOUNT)
#define VPC_VSBRANCHINSTCOUNT           (VPG_VS + VS_BRANCHINSTCOUNT)
#define VPC_VSTEXLDINSTCOUNT            (VPG_VS + VS_TEXLDINSTCOUNT)
#define VPC_VSRENDEREDVERTCOUNT         (VPG_VS + VS_RENDEREDVERTCOUNT)
#define VPC_VSNONIDLESTARVECOUNT        (VPG_VS + VS_NONIDLESTARVECOUNT)
#define VPC_VSSTARVELCOUNT              (VPG_VS + VS_STARVELCOUNT)
#define VPC_VSSTALLCOUNT                (VPG_VS + VS_STALLCOUNT)
#define VPC_VSPROCESSCOUNT              (VPG_VS + VS_PROCESSCOUNT)
/* HW: PS Count. */
#define VPC_PSINSTCOUNT                 (VPG_PS + PS_INSTCOUNT)
#define VPC_PSBRANCHINSTCOUNT           (VPG_PS + PS_BRANCHINSTCOUNT)
#define VPC_PSTEXLDINSTCOUNT            (VPG_PS + PS_TEXLDINSTCOUNT)
#define VPC_PSRENDEREDPIXCOUNT          (VPG_PS + PS_RENDEREDPIXCOUNT)
#define VPC_PSNONIDLESTARVECOUNT        (VPG_PS + PS_NONIDLESTARVECOUNT)
#define VPC_PSSTARVELCOUNT              (VPG_PS + PS_STARVELCOUNT)
#define VPC_PSSTALLCOUNT                (VPG_PS + PS_STALLCOUNT)
#define VPC_PSPROCESSCOUNT              (VPG_PS + PS_PROCESSCOUNT)
#define VPC_PSSHADERCYCLECOUNT          (VPG_PS + PS_SHADERCYCLECOUNT)

/* HW: PA Counters. */
#define VPC_PAINVERTCOUNT               (VPG_PA + PA_INVERTCOUNT)
#define VPC_PAINPRIMCOUNT               (VPG_PA + PA_INPRIMCOUNT)
#define VPC_PAOUTPRIMCOUNT              (VPG_PA + PA_OUTPRIMCOUNT)
#define VPC_PADEPTHCLIPCOUNT            (VPG_PA + PA_DEPTHCLIPCOUNT)
#define VPC_PATRIVIALREJCOUNT           (VPG_PA + PA_TRIVIALREJCOUNT)
#define VPC_PACULLCOUNT                 (VPG_PA + PA_CULLCOUNT)
#define VPC_PANONIDLESTARVECOUNT        (VPG_PA + PA_NONIDLESTARVECOUNT)
#define VPC_PASTARVELCOUNT              (VPG_PA + PA_STARVELCOUNT)
#define VPC_PASTALLCOUNT                (VPG_PA + PA_STALLCOUNT)
#define VPC_PAPROCESSCOUNT              (VPG_PA + PA_PROCESSCOUNT)

/* HW: Setup Counters. */
#define VPC_SETRIANGLECOUNT             (VPG_SETUP + SE_TRIANGLECOUNT)
#define VPC_SELINECOUNT                 (VPG_SETUP + SE_LINECOUNT)
#define VPC_SESTARVECOUNT               (VPG_SETUP + SE_STARVECOUNT)
#define VPC_SESTALLCOUNT                (VPG_SETUP + SE_STALLCOUNT)
#define VPC_SERECEIVETRIANGLECOUNT      (VPG_SETUP + SE_RECEIVETRIANGLECOUNT)
#define VPC_SESENDTRIANGLECOUNT         (VPG_SETUP + SE_SENDTRIANGLECOUNT)
#define VPC_SERECEIVELINESCOUNT         (VPG_SETUP + SE_RECEIVELINESCOUNT)
#define VPC_SESENDLINESCOUNT            (VPG_SETUP + SE_SENDLINESCOUNT)
#define VPC_SENONIDLESTARVECOUNT        (VPG_SETUP + SE_NONIDLESTARVECOUNT)
#define VPC_SEPROCESSCOUNT              (VPG_SETUP + SE_PROCESSCOUNT)

/* HW: RA Counters. */
#define VPC_RAVALIDPIXCOUNT             (VPG_RA + RA_VALIDPIXCOUNT)
#define VPC_RATOTALQUADCOUNT            (VPG_RA + RA_TOTALQUADCOUNT)
#define VPC_RAVALIDQUADCOUNTEZ          (VPG_RA + RA_VALIDQUADCOUNTEZ)
#define VPC_RATOTALPRIMCOUNT            (VPG_RA + RA_TOTALPRIMCOUNT)
#define VPC_RAPIPECACHEMISSCOUNT        (VPG_RA + RA_PIPECACHEMISSCOUNT)
#define VPC_RAPREFCACHEMISSCOUNT        (VPG_RA + RA_PREFCACHEMISSCOUNT)
#define VPC_RAEEZCULLCOUNT              (VPG_RA + RA_EEZCULLCOUNT)
#define VPC_RANONIDLESTARVECOUNT        (VPG_RA + RA_NONIDLESTARVECOUNT)
#define VPC_RASTARVELCOUNT              (VPG_RA + RA_STARVELCOUNT)
#define VPC_RASTALLCOUNT                (VPG_RA + RA_STALLCOUNT)
#define VPC_RAPROCESSCOUNT              (VPG_RA + RA_PROCESSCOUNT)

/* HW: TEX Counters. */
#define VPC_TXTOTBILINEARREQ            (VPG_TX + TX_TOTBILINEARREQ)
#define VPC_TXTOTTRILINEARREQ           (VPG_TX + TX_TOTTRILINEARREQ)
#define VPC_TXTOTDISCARDTEXREQ          (VPG_TX + TX_TOTDISCARDTEXREQ)
#define VPC_TXTOTTEXREQ                 (VPG_TX + TX_TOTTEXREQ)
#define VPC_TXMEMREADCOUNT              (VPG_TX + TX_MEMREADCOUNT)
#define VPC_TXMEMREADIN8BCOUNT          (VPG_TX + TX_MEMREADIN8BCOUNT)
#define VPC_TXCACHEMISSCOUNT            (VPG_TX + TX_CACHEMISSCOUNT)
#define VPC_TXCACHEHITTEXELCOUNT        (VPG_TX + TX_CACHEHITTEXELCOUNT)
#define VPC_TXCACHEMISSTEXELCOUNT       (VPG_TX + TX_CACHEMISSTEXELCOUNT)
#define VPC_TXNONIDLESTARVECOUNT        (VPG_TX + TX_NONIDLESTARVECOUNT)
#define VPC_TXSTARVELCOUNT              (VPG_TX + TX_STARVELCOUNT)
#define VPC_TXSTALLCOUNT                (VPG_TX + TX_STALLCOUNT)
#define VPC_TXPROCESSCOUNT              (VPG_TX + TX_PROCESSCOUNT)

/* HW: PE Counters. */
#define VPC_PEKILLEDBYCOLOR             (VPG_PE + PE_KILLEDBYCOLOR)
#define VPC_PEKILLEDBYDEPTH             (VPG_PE + PE_KILLEDBYDEPTH)
#define VPC_PEDRAWNBYCOLOR              (VPG_PE + PE_DRAWNBYCOLOR)
#define VPC_PEDRAWNBYDEPTH              (VPG_PE + PE_DRAWNBYDEPTH)

/* HW: MC Counters. */
#define VPC_MCREADREQ8BPIPE             (VPG_MC + MC_READREQ8BPIPE)
#define VPC_MCREADREQ8BIP               (VPG_MC + MC_READREQ8BIP)
#define VPC_MCWRITEREQ8BPIPE            (VPG_MC + MC_WRITEREQ8BPIPE)
#define VPC_MCAXIMINLATENCY             (VPG_MC + MC_AXIMINLATENCY)
#define VPC_MCAXIMAXLATENCY             (VPG_MC + MC_AXIMAXLATENCY)
#define VPC_MCAXITOTALLATENCY           (VPG_MC + MC_AXITOTALLATENCY)
#define VPC_MCAXISAMPLECOUNT            (VPG_MC + MC_AXISAMPLECOUNT)

/* HW: AXI Counters. */
#define VPC_AXIREADREQSTALLED           (VPG_AXI + AXI_READREQSTALLED)
#define VPC_AXIWRITEREQSTALLED          (VPG_AXI + AXI_WRITEREQSTALLED)
#define VPC_AXIWRITEDATASTALLED         (VPG_AXI + AXI_WRITEDATASTALLED)

/* HW: FE Counters. */
#define VPC_FEDRAWCOUNT                 (VPG_FE + FE_DRAWCOUNT)
#define VPC_FEOUTVERTEXCOUNT            (VPG_FE + FE_OUTVERTEXCOUNT)
#define VPC_FESTALLCOUNT                (VPG_FE + FE_STALLCOUNT)
#define VPC_FESTARVECOUNT               (VPG_FE + FE_STARVECOUNT)

/* HW: Shader Counters. */
#define VPNC_VSINSTCOUNT                 (VPNG_VS + 1)
#define VPNC_VSBRANCHINSTCOUNT           (VPNG_VS + 2)
#define VPNC_VSTEXLDINSTCOUNT            (VPNG_VS + 3)
#define VPNC_VSRENDEREDVERTCOUNT         (VPNG_VS + 4)
#define VPNC_VSNONIDLESTARVECOUNT        (VPNG_VS + 5)
#define VPNC_VSSTARVELCOUNT              (VPNG_VS + 6)
#define VPNC_VSSTALLCOUNT                (VPNG_VS + 7)
#define VPNC_VSPROCESSCOUNT              (VPNG_VS + 8)
#define VPNC_VSSHADERCYCLECOUNT          (VPNG_VS + 9)
#define VPNC_VS_COUNT                    VPNC_VSSHADERCYCLECOUNT - VPNG_VS

/* HW: PS Count. */
#define VPNC_PSINSTCOUNT                 (VPNG_PS + 1)
#define VPNC_PSBRANCHINSTCOUNT           (VPNG_PS + 2)
#define VPNC_PSTEXLDINSTCOUNT            (VPNG_PS + 3)
#define VPNC_PSRENDEREDPIXCOUNT          (VPNG_PS + 4)
#define VPNC_PSNONIDLESTARVECOUNT        (VPNG_PS + 5)
#define VPNC_PSSTARVELCOUNT              (VPNG_PS + 6)
#define VPNC_PSSTALLCOUNT                (VPNG_PS + 7)
#define VPNC_PSPROCESSCOUNT              (VPNG_PS + 8)
#define VPNC_PSSHADERCYCLECOUNT          (VPNG_PS + 9)
#define VPNC_PS_COUNT                    VPNC_PSSHADERCYCLECOUNT - VPNG_PS

/* HW: PA Counters. */
#define VPNC_PAINVERTCOUNT               (VPNG_PA + 1)
#define VPNC_PAINPRIMCOUNT               (VPNG_PA + 2)
#define VPNC_PAOUTPRIMCOUNT              (VPNG_PA + 3)
#define VPNC_PADEPTHCLIPCOUNT            (VPNG_PA + 4)
#define VPNC_PATRIVIALREJCOUNT           (VPNG_PA + 5)
#define VPNC_PACULLPRIMCOUNT             (VPNG_PA + 6)
#define VPNC_PADROPPRIMCOUNT             (VPNG_PA + 7)
#define VPNC_PAFRCLIPPRIMCOUNT           (VPNG_PA + 8)
#define VPNC_PAFRCLIPDROPPRIMCOUNT       (VPNG_PA + 9)
#define VPNC_PANONIDLESTARVECOUNT        (VPNG_PA + 10)
#define VPNC_PASTARVELCOUNT              (VPNG_PA + 11)
#define VPNC_PASTALLCOUNT                (VPNG_PA + 12)
#define VPNC_PAPROCESSCOUNT              (VPNG_PA + 13)
#define VPNC_PA_COUNT                    VPNC_PAPROCESSCOUNT - VPNG_PA

/* HW: Setup Counters. */
#define VPNC_SECULLTRIANGLECOUNT         (VPNG_SETUP + 1)
#define VPNC_SECULLLINECOUNT             (VPNG_SETUP + 2)
#define VPNC_SECLIPTRIANGLECOUNT         (VPNG_SETUP + 3)
#define VPNC_SECLIPLINECOUNT             (VPNG_SETUP + 4)
#define VPNC_SESTARVECOUNT               (VPNG_SETUP + 5)
#define VPNC_SESTALLCOUNT                (VPNG_SETUP + 6)
#define VPNC_SERECEIVETRIANGLECOUNT      (VPNG_SETUP + 7)
#define VPNC_SESENDTRIANGLECOUNT         (VPNG_SETUP + 8)
#define VPNC_SERECEIVELINESCOUNT         (VPNG_SETUP + 9)
#define VPNC_SESENDLINESCOUNT            (VPNG_SETUP + 10)
#define VPNC_SENONIDLESTARVECOUNT        (VPNG_SETUP + 11)
#define VPNC_SETRIVIALREJLINECOUNT       (VPNG_SETUP + 12)
#define VPNC_SEPROCESSCOUNT              (VPNG_SETUP + 13)
#define VPNC_SE_COUNT                    VPNC_SEPROCESSCOUNT - VPNG_SETUP

/* HW: RA Counters. */
#define VPNC_RAVALIDPIXCOUNT             (VPNG_RA + 1)
#define VPNC_RATOTALQUADCOUNT            (VPNG_RA + 2)
#define VPNC_RAVALIDQUADCOUNTEZ          (VPNG_RA + 3)
#define VPNC_RAINPUTPRIMCOUNT            (VPNG_RA + 4)
#define VPNC_RAPIPECACHEMISSCOUNT        (VPNG_RA + 5)
#define VPNC_RAPREFCACHEMISSCOUNT        (VPNG_RA + 6)
#define VPNC_RAPIPEHZCACHEMISSCOUNT      (VPNG_RA + 7)
#define VPNC_RAPREFHZCACHEMISSCOUNT      (VPNG_RA + 8)
#define VPNC_RAOUTPUTQUADCOUNT           (VPNG_RA + 9)
#define VPNC_RAOUTPUTPIXELCOUNT          (VPNG_RA + 10)
#define VPNC_RAEEZCULLCOUNT              (VPNG_RA + 11)
#define VPNC_RANONIDLESTARVECOUNT        (VPNG_RA + 12)
#define VPNC_RASTARVELCOUNT              (VPNG_RA + 13)
#define VPNC_RASTALLCOUNT                (VPNG_RA + 14)
#define VPNC_RAPROCESSCOUNT              (VPNG_RA + 15)
#define VPNC_RA_COUNT                    VPNC_RAPROCESSCOUNT - VPNG_RA

/* HW: TEX Counters. */
#define VPNC_TXTOTBILINEARREQ            (VPNG_TX + 1)
#define VPNC_TXTOTTRILINEARREQ           (VPNG_TX + 2)
#define VPNC_TXTOTDISCARDTEXREQ          (VPNG_TX + 3)
#define VPNC_TXTOTTEXREQ                 (VPNG_TX + 4)
#define VPNC_TXMC0MISSCOUNT              (VPNG_TX + 5)
#define VPNC_TXMC0REQCOUNT               (VPNG_TX + 6)
#define VPNC_TXMC1MISSCOUNT              (VPNG_TX + 7)
#define VPNC_TXMC1REQCOUNT               (VPNG_TX + 8)
#define VPNC_TX_COUNT                    VPNC_TXMC1REQCOUNT - VPNG_TX

/* HW: PE Counters. */
#define VPNC_PE0KILLEDBYCOLOR             (VPNG_PE + 1)
#define VPNC_PE0KILLEDBYDEPTH             (VPNG_PE + 2)
#define VPNC_PE0DRAWNBYCOLOR              (VPNG_PE + 3)
#define VPNC_PE0DRAWNBYDEPTH              (VPNG_PE + 4)
#define VPNC_PE1KILLEDBYCOLOR             (VPNG_PE + 5)
#define VPNC_PE1KILLEDBYDEPTH             (VPNG_PE + 6)
#define VPNC_PE1DRAWNBYCOLOR              (VPNG_PE + 7)
#define VPNC_PE1DRAWNBYDEPTH              (VPNG_PE + 8)
#define VPNC_PE_COUNT                     VPNC_PE1DRAWNBYDEPTH - VPNG_PE

/* HW: MCC Counters. */
#define VPNC_MCCREADREQ8BCOLORPIPE        (VPNG_MCC + 1)
#define VPNC_MCCREADREQ8BSOCOLORPIPE      (VPNG_MCC + 2)
#define VPNC_MCCWRITEREQ8BCOLORPIPE       (VPNG_MCC + 3)
#define VPNC_MCCREADREQSOCOLORPIPE        (VPNG_MCC + 4)
#define VPNC_MCCWRITEREQCOLORPIPE         (VPNG_MCC + 5)
#define VPNC_MCCREADREQ8BDEPTHPIPE        (VPNG_MCC + 6)
#define VPNC_MCCREADREQ8BSFDEPTHPIPE      (VPNG_MCC + 7)
#define VPNC_MCCWRITEREQ8BDEPTHPIPE       (VPNG_MCC + 8)
#define VPNC_MCCREADREQSFDEPTHPIPE        (VPNG_MCC + 9)
#define VPNC_MCCWRITEREQDEPTHPIPE         (VPNG_MCC + 10)
#define VPNC_MCCREADREQ8BOTHERPIPE        (VPNG_MCC + 11)
#define VPNC_MCCWRITEREQ8BOTHERPIPE       (VPNG_MCC + 12)
#define VPNC_MCCREADREQOTHERPIPE          (VPNG_MCC + 13)
#define VPNC_MCCWRITEREQOTHERPIPE         (VPNG_MCC + 14)
#define VPNC_MCCAXIMINLATENCY             (VPNG_MCC + 15)
#define VPNC_MCCAXIMAXLATENCY             (VPNG_MCC + 16)
#define VPNC_MCCAXITOTALLATENCY           (VPNG_MCC + 17)
#define VPNC_MCCAXISAMPLECOUNT            (VPNG_MCC + 18)
#define VPNC_MCCFEREADBANDWIDTH           (VPNG_MCC + 19)
#define VPNC_MCCMMUREADBANDWIDTH          (VPNG_MCC + 20)
#define VPNC_MCCBLTREADBANDWIDTH          (VPNG_MCC + 21)
#define VPNC_MCCSH0READBANDWIDTH          (VPNG_MCC + 22)
#define VPNC_MCCSH1READBANDWIDTH          (VPNG_MCC + 23)
#define VPNC_MCCPEWRITEBANDWIDTH          (VPNG_MCC + 24)
#define VPNC_MCCBLTWRITEBANDWIDTH         (VPNG_MCC + 25)
#define VPNC_MCCSH0WRITEBANDWIDTH         (VPNG_MCC + 26)
#define VPNC_MCCSH1WRITEBANDWIDTH         (VPNG_MCC + 27)
#define VPNC_MCC_COUNT                    VPNC_MCCSH1WRITEBANDWIDTH - VPNG_MCC

/* HW: MCZ Counters. */
#define VPNC_MCZREADREQ8BCOLORPIPE        (VPNG_MCZ + 1)
#define VPNC_MCZREADREQ8BSOCOLORPIPE      (VPNG_MCZ + 2)
#define VPNC_MCZWRITEREQ8BCOLORPIPE       (VPNG_MCZ + 3)
#define VPNC_MCZREADREQSOCOLORPIPE        (VPNG_MCZ + 4)
#define VPNC_MCZWRITEREQCOLORPIPE         (VPNG_MCZ + 5)
#define VPNC_MCZREADREQ8BDEPTHPIPE        (VPNG_MCZ + 6)
#define VPNC_MCZREADREQ8BSFDEPTHPIPE      (VPNG_MCZ + 7)
#define VPNC_MCZWRITEREQ8BDEPTHPIPE       (VPNG_MCZ + 8)
#define VPNC_MCZREADREQSFDEPTHPIPE        (VPNG_MCZ + 9)
#define VPNC_MCZWRITEREQDEPTHPIPE         (VPNG_MCZ + 10)
#define VPNC_MCZREADREQ8BOTHERPIPE        (VPNG_MCZ + 11)
#define VPNC_MCZWRITEREQ8BOTHERPIPE       (VPNG_MCZ + 12)
#define VPNC_MCZREADREQOTHERPIPE          (VPNG_MCZ + 13)
#define VPNC_MCZWRITEREQOTHERPIPE         (VPNG_MCZ + 14)
#define VPNC_MCZAXIMINLATENCY             (VPNG_MCZ + 15)
#define VPNC_MCZAXIMAXLATENCY             (VPNG_MCZ + 16)
#define VPNC_MCZAXITOTALLATENCY           (VPNG_MCZ + 17)
#define VPNC_MCZAXISAMPLECOUNT            (VPNG_MCZ + 18)
#define VPNC_MCZ_COUNT                    VPNC_MCZAXISAMPLECOUNT - VPNG_MCZ

/* HW: HI Counters. */
#define VPNC_HI0READ8BYTE                (VPNG_HI + 1)
#define VPNC_HI0WRITE8BYTE               (VPNG_HI + 2)
#define VPNC_HI0READREQ                  (VPNG_HI + 3)
#define VPNC_HI0WRITEREQ                 (VPNG_HI + 4)
#define VPNC_HI0AXIREADREQSTALL          (VPNG_HI + 5)
#define VPNC_HI0AXIWRITEREQSTALL         (VPNG_HI + 6)
#define VPNC_HI0AXIWRITEDATASTALL        (VPNG_HI + 7)
#define VPNC_HI1READ8BYTE                (VPNG_HI + 8)
#define VPNC_HI1WRITE8BYTE               (VPNG_HI + 9)
#define VPNC_HI1READREQ                  (VPNG_HI + 10)
#define VPNC_HI1WRITEREQ                 (VPNG_HI + 11)
#define VPNC_HI1AXIREADREQSTALL          (VPNG_HI + 12)
#define VPNC_HI1AXIWRITEREQSTALL         (VPNG_HI + 13)
#define VPNC_HI1AXIWRITEDATASTALL        (VPNG_HI + 14)
#define VPNC_HITOTALCYCLES               (VPNG_HI + 15)
#define VPNC_HIIDLECYCLES                (VPNG_HI + 16)
#define VPNC_HIREAD8BYTE                 (VPNG_HI + 17)
#define VPNC_HIWRITE8BYTE                (VPNG_HI + 18)
#define VPNC_HIOCBREAD16BYTE             (VPNG_HI + 19)
#define VPNC_HIOCBWRITE16BYTE            (VPNG_HI + 20)
#define VPNC_HI_COUNT                    VPNC_HIOCBWRITE16BYTE - VPNG_HI

/* HW: L2 Counters. */
#define VPNC_L2AXI0READREQCOUNT          (VPNG_L2 + 1)
#define VPNC_L2AXI1READREQCOUNT          (VPNG_L2 + 2)
#define VPNC_L2AXI0WRITEREQCOUNT         (VPNG_L2 + 3)
#define VPNC_L2AXI1WRITEREQCOUNT         (VPNG_L2 + 4)
#define VPNC_L2READTRANSREQBYAXI0        (VPNG_L2 + 5)
#define VPNC_L2READTRANSREQBYAXI1        (VPNG_L2 + 6)
#define VPNC_L2WRITETRANSREQBYAXI0       (VPNG_L2 + 7)
#define VPNC_L2WRITETRANSREQBYAXI1       (VPNG_L2 + 8)
#define VPNC_L2AXI0MINLATENCY            (VPNG_L2 + 9)
#define VPNC_L2AXI0MAXLATENCY            (VPNG_L2 + 10)
#define VPNC_L2AXI0TOTLATENCY            (VPNG_L2 + 11)
#define VPNC_L2AXI0TOTREQCOUNT           (VPNG_L2 + 12)
#define VPNC_L2AXI1MINLATENCY            (VPNG_L2 + 13)
#define VPNC_L2AXI1MAXLATENCY            (VPNG_L2 + 14)
#define VPNC_L2AXI1TOTLATENCY            (VPNG_L2 + 15)
#define VPNC_L2AXI1TOTREQCOUNT           (VPNG_L2 + 16)
#define VPNC_L2_COUNT                    VPNC_L2AXI1TOTREQCOUNT - VPNG_L2

/* HW: FE Counters. */
#define VPNC_FEDRAWCOUNT                 (VPNG_FE + 1)
#define VPNC_FEOUTVERTEXCOUNT            (VPNG_FE + 2)
#define VPNC_FECACHEMISSCOUNT            (VPNG_FE + 3)
#define VPNC_FECACHELKCOUNT              (VPNG_FE + 4)
#define VPNC_FESTALLCOUNT                (VPNG_FE + 5)
#define VPNC_FESTARVECOUNT               (VPNG_FE + 6)
#define VPNC_FEPROCESSCOUNT              (VPNG_FE + 7)
#define VPNC_FE_COUNT                    VPNC_FEPROCESSCOUNT - VPNG_FE

#define TOTAL_COUNTER_NUMBER             (VPNC_FE_COUNT + VPNC_VS_COUNT + VPNC_PA_COUNT + VPNC_SE_COUNT + VPNC_RA_COUNT \
                                          + VPNC_PS_COUNT + VPNC_TX_COUNT + VPNC_PE_COUNT + VPNC_MCC_COUNT + VPNC_MCZ_COUNT \
                                          + VPNC_HI_COUNT + VPNC_L2_COUNT)

#define TOTAL_MODULE_NUMBER              12

/* PROGRAM: Shader program counters. */
#define VPC_PVSINSTRCOUNT           (VPG_PVS + PVS_INSTRCOUNT)
#define VPC_PVSALUINSTRCOUNT        (VPG_PVS + PVS_ALUINSTRCOUNT)
#define VPC_PVSTEXINSTRCOUNT        (VPG_PVS + PVS_TEXINSTRCOUNT)
#define VPC_PVSATTRIBCOUNT          (VPG_PVS + PVS_ATTRIBCOUNT)
#define VPC_PVSUNIFORMCOUNT         (VPG_PVS + PVS_UNIFORMCOUNT)
#define VPC_PVSFUNCTIONCOUNT        (VPG_PVS + PVS_FUNCTIONCOUNT)
#define VPC_PVSSOURCE               (VPG_PVS + PVS_SOURCE)

#define VPC_PPSINSTRCOUNT           (VPG_PPS + PPS_INSTRCOUNT)
#define VPC_PPSALUINSTRCOUNT        (VPG_PPS + PPS_ALUINSTRCOUNT)
#define VPC_PPSTEXINSTRCOUNT        (VPG_PPS + PPS_TEXINSTRCOUNT)
#define VPC_PPSATTRIBCOUNT          (VPG_PPS + PPS_ATTRIBCOUNT)
#define VPC_PPSUNIFORMCOUNT         (VPG_PPS + PPS_UNIFORMCOUNT)
#define VPC_PPSFUNCTIONCOUNT        (VPG_PPS + PPS_FUNCTIONCOUNT)
#define VPC_PPSSOURCE               (VPG_PPS + PPS_SOURCE)

#define VPC_PROGRAMHANDLE           (VPG_PROG + 1)

#define VPC_ES30_DRAW_NO            (VPG_ES30_DRAW + 1)
#define VPC_ES11_DRAW_NO            (VPG_ES11_DRAW + 1)
#define VPC_ES30_GPU_NO             (VPG_MULTI_GPU + 1)


#define   MODULE_FRONT_END_COUNTER_NUM                    0x5
#define   MODULE_VERTEX_SHADER_COUNTER_NUM                0x9
#define   MODULE_PRIMITIVE_ASSEMBLY_COUNTER_NUM           0xC
#define   MODULE_SETUP_COUNTER_NUM                        0xD
#define   MODULE_RASTERIZER_COUNTER_NUM                   0xE
#define   MODULE_PIXEL_SHADER_COUNTER_NUM                 0x9
#define   MODULE_TEXTURE_COUNTER_NUM                      0x8
#define   MODULE_PIXEL_ENGINE_COUNTER_NUM                 0x8
#define   MODULE_MEMORY_CONTROLLER_COLOR_COUNTER_NUM      0xC
#define   MODULE_MEMORY_CONTROLLER_DEPTH_COUNTER_NUM      0xC
#define   MODULE_HOST_INTERFACE0_COUNTER_NUM              0x9
#define   MODULE_HOST_INTERFACE1_COUNTER_NUM              0x7
#define   MODULE_GPUL2_CACHE_COUNTER_NUM                  0xE
#define   TOTAL_PROBE_NUMBER (MODULE_FRONT_END_COUNTER_NUM + MODULE_VERTEX_SHADER_COUNTER_NUM + MODULE_PRIMITIVE_ASSEMBLY_COUNTER_NUM \
                              + MODULE_SETUP_COUNTER_NUM + MODULE_RASTERIZER_COUNTER_NUM + MODULE_PIXEL_SHADER_COUNTER_NUM \
                              + MODULE_TEXTURE_COUNTER_NUM + MODULE_PIXEL_ENGINE_COUNTER_NUM + MODULE_MEMORY_CONTROLLER_COLOR_COUNTER_NUM \
                              + MODULE_MEMORY_CONTROLLER_DEPTH_COUNTER_NUM + MODULE_HOST_INTERFACE0_COUNTER_NUM + MODULE_HOST_INTERFACE1_COUNTER_NUM \
                              + MODULE_GPUL2_CACHE_COUNTER_NUM)


#ifdef ANDROID
#define DEFAULT_PROFILE_FILE_NAME   "/sdcard/vprofiler.vpd"
#else
#define DEFAULT_PROFILE_FILE_NAME   "vprofiler.vpd"
#endif

#define VPHEADER_VERSION "VP20"

#define VPFILETYPE_GL "10"

#define VPFILETYPE_CL "00"

#if gcdENDIAN_BIG
#define BIG_ENDIAN_TRANS_INT(x) ((gctUINT32)(\
        (((gctUINT32)(x) & (gctUINT32)0x000000FFUL) << 24) | \
        (((gctUINT32)(x) & (gctUINT32)0x0000FF00UL) << 8)  | \
        (((gctUINT32)(x) & (gctUINT32)0x00FF0000UL) >> 8)  | \
        (((gctUINT32)(x) & (gctUINT32)0xFF000000UL) >> 24)))
#else
#define BIG_ENDIAN_TRANS_INT(x) x
#endif

/* Write a data value. */
#define gcmWRITE_VALUE(IntData) \
    do \
    { \
        gceSTATUS status; \
        gctINT32 value = IntData; \
        value = BIG_ENDIAN_TRANS_INT(value); \
        gcmERR_BREAK(gcoPROFILER_Write(Profiler, gcmSIZEOF(value), &value)); \
    } \
    while (gcvFALSE)

#define gcmWRITE_CONST(Const) \
    do \
    { \
        gceSTATUS status; \
        gctINT32 data = Const; \
        data = BIG_ENDIAN_TRANS_INT(data); \
        gcmERR_BREAK(gcoPROFILER_Write(Profiler, gcmSIZEOF(data), &data)); \
    } \
    while (gcvFALSE)

#define gcmWRITE_COUNTER(Counter, Value) \
    gcmWRITE_CONST(Counter); \
    gcmWRITE_VALUE(Value)

/* Write a data value. */
#define gcmRECORD_VALUE(IntData) \
    do \
    { \
        gctINT32 value = IntData; \
        value = BIG_ENDIAN_TRANS_INT(value); \
        counterData[counterIndex++] = value; \
    } \
    while (gcvFALSE)

#define gcmRECORD_CONST(Const) \
    do \
    { \
        gctINT32 data = Const; \
        data = BIG_ENDIAN_TRANS_INT(data); \
        counterData[counterIndex++] = data; \
    } \
    while (gcvFALSE)

#define gcmRECORD_COUNTER(Counter, Value) \
    gcmRECORD_CONST(Counter); \
    gcmRECORD_VALUE(Value)

/* Write a string value (char*). */
#define gcmWRITE_STRING(String) \
    do \
    { \
        gceSTATUS status; \
        gctINT32 length; \
        length = (gctINT32) gcoOS_StrLen((gctSTRING)String, gcvNULL); \
        length = BIG_ENDIAN_TRANS_INT(length); \
        gcmERR_BREAK(gcoPROFILER_Write(Profiler, gcmSIZEOF(length), &length)); \
        gcmERR_BREAK(gcoPROFILER_Write(Profiler, length, String)); \
    } \
    while (gcvFALSE)

#define gcmWRITE_BUFFER(Size, Buffer) \
    do \
    { \
        gceSTATUS status; \
        gcmERR_BREAK(gcoPROFILER_Write(Profiler, Size, Buffer)); \
    } \
    while (gcvFALSE)


#define gcmGET_COUNTER(counter, counterId) \
    do \
    { \
        if (*(memory + (counterId + offset) * (1 << clusterIDWidth)) == 0xdeaddead) \
        { \
            counter = 0xdeaddead; \
        } \
        else \
        { \
            gctUINT32 i; \
            gctUINT32_PTR Memory = memory; \
            counter = 0; \
            Memory = memory + TOTAL_PROBE_NUMBER * CoreId * (1 << clusterIDWidth); \
            for (i = 0; i < (gctUINT32)(1 << clusterIDWidth); i++) \
            { \
                counter += *(Memory + (counterId + offset) * (1 << clusterIDWidth) + i); \
            } \
        } \
    } \
    while (gcvFALSE)

#define gcmGET_LATENCY_COUNTER(minLatency, maxLatency, counterId) \
    do \
    { \
        if (*(memory + (counterId + offset) * (1 << clusterIDWidth)) == 0xdeaddead) \
        { \
            minLatency = maxLatency = 0xdeaddead; \
        } \
        else \
        { \
            gctUINT32 i; \
            gctUINT32_PTR Memory = memory; \
            Memory = memory + TOTAL_PROBE_NUMBER * CoreId * (1 << clusterIDWidth); \
            for (i = 0; i < (gctUINT32)(1 << clusterIDWidth); i++) \
            { \
                maxLatency += ((*(Memory + (counterId + offset) * (1 << clusterIDWidth) + i) & 0xfff000) >> 12); \
                minLatency += (*(Memory + (counterId + offset) * (1 << clusterIDWidth) + i) & 0x000fff); \
                if (minLatency == 4095) \
                    minLatency = 0; \
            } \
        } \
    } \
    while (gcvFALSE)

#define NumOfPerFrameBuf        16
#define NumOfPerDrawBuf         128

typedef struct gcsCounterBuffer * gcsCounterBuffer_PTR;

struct gcsCounterBuffer
{
    gcsPROFILER_COUNTERS        *counters;
    gctHANDLE                   couterBufobj;
    gctUINT32                   probeAddress;
    gctPOINTER                  logicalAddress;
    gceCOUNTER_OPTYPE           opType;
    gctUINT32                   opID;
    gctUINT32                   startPos;
    gctUINT32                   endPos;
    gctUINT32                   dataSize;
    gctBOOL                     available;
    gctBOOL                     needDump;
    gcsCounterBuffer_PTR        next;
    gcsCounterBuffer_PTR        prev;
};

typedef struct _gcoPROFILER *        gcoPROFILER;

struct _gcoPROFILER
{
    gctBOOL                     enable;
    gctBOOL                     enablePrint;
    gctBOOL                     disableProbe;
    gctBOOL                     probeMode;

    gctFILE                     file;
    gctCHAR*                    fileName;

    gcsCounterBuffer_PTR        counterBuf;
    gctUINT32                   bufferCount;

    gctBOOL                     perDrawMode;
    gctBOOL                     needDump;
    gctBOOL                     counterEnable;

    gceProfilerClient           profilerClient;

    /*query some features from hw*/
    gctUINT32                   coreCount;
    gctUINT32                   shaderCoreCount;
    gctBOOL                     bHalti4;
    gctBOOL                     psRenderPixelFix;
    gctBOOL                     axiBus128bits;
};

typedef struct _gcsPROBESTATES
{
    gceProbeStatus              status;
    gctUINT32                   probeAddress;
}gcsPROBESTATES;

/* Construct a Profiler object per context. */
gceSTATUS
gcoPROFILER_Construct(
    OUT gcoPROFILER * Profiler
    );

gceSTATUS
gcoPROFILER_Destroy(
    IN gcoPROFILER Profiler
    );

gceSTATUS
gcoPROFILER_Initialize(
    IN gcoPROFILER Profiler
    );

gceSTATUS
gcoPROFILER_Disable(
    void
    );

gceSTATUS
gcoPROFILER_EnableCounters(
    IN gcoPROFILER Profiler,
    IN gceCOUNTER_OPTYPE operationType
    );

gceSTATUS
gcoPROFILER_End(
    IN gcoPROFILER Profiler,
    IN gceCOUNTER_OPTYPE operationType,
    IN gctUINT32 OpID
    );

gceSTATUS
gcoPROFILER_Write(
    IN gcoPROFILER Profiler,
    IN gctSIZE_T ByteCount,
    IN gctCONST_POINTER Data
    );

gceSTATUS
gcoPROFILER_Flush(
    IN gcoPROFILER Profiler
    );
#ifdef __cplusplus
}
#endif

#endif /* __gc_hal_profiler_h_ */


