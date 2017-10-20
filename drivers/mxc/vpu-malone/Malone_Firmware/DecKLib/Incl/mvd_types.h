/***************************************************
  Copyright (c) 2015 Amphion Semiconductor Ltd 
                All rights reserved.               
 ***************************************************
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 ****************************************************

  Author    : MediaIP FW Team
  File name : mvd_types.h
  Function  : Used as a home for generic MVD base decoder
              types

 ***************************************************/

#ifndef _MVD_TYPES_H_
#define _MVD_TYPES_H_

#include "basetype.h"

///////////////////////////////////////////////////////
// tAuxCRCData

typedef struct
{
  u_int32 uDpbCRC2;
  u_int32 uBSCRC;
  u_int32 uCRC0;
  u_int32 uCRC1;
  u_int32 uCRC2;
  u_int32 uCRC3;
  u_int32 uCRC4;
  u_int32 uCRC5;
  u_int32 uCRC6;
  u_int32 uCRC7;
  
} tAuxCRCData, *ptAuxCRCData;

///////////////////////////////////////////////////////
// tAuxCRC2Data

typedef struct
{
  u_int32 uCRC8;
  u_int32 uCRC9;
  u_int32 uCRC10;
  u_int32 uCRC11;
  u_int32 uCRC12;
  u_int32 uCRC13;
  u_int32 uCRC14;
  
} tAuxCRC2Data, *ptAuxCRC2Data;

#define MVD_TRUE       0x1UL
#define MVD_FALSE      0x0UL

typedef float          MvdFloat;           // fVariableName,   *pfPointerName
typedef double         MvdDouble;          // dVariableName,   *pdPointerName

typedef volatile u_int32  MvdHwReg;  // rVariableName,   *prPointerName
typedef volatile u_int32 *MvdHwAddr;

/* ************** */
/*  Debug arrays  */
/* ************** */
typedef struct
{              
  int32 index;
  int32 array[512];
} DBG_ARRAY_512;  

typedef struct
{              
  int32 index;
  int32 array[1024];
} DBG_ARRAY_1024;  

typedef struct
{              
  int32 index;
  int32 array[2048];
} DBG_ARRAY_2048;  

typedef struct
{              
  int32 index;
  int32 array[4096];
} DBG_ARRAY_4096;

///////////////////////////////////////////////////////
// Metadata structs

///////////////////////////////////////////////////////
// TimeStamp Metadata

typedef struct
{              
  u_int32   uPTS;
  u_int32   uDTS;
  u_int32   uPESFlags;
  bool      bValid;

} tMVD_METADATA_TS, *ptMVD_METADATA_TS;

///////////////////////////////////////////////////////
// Pic Struct Metadata

typedef struct
{              
  u_int32 uDummy;
  bool    bValid;
  
} tMVD_METADATA_PIC_STRUCT, *ptMVD_METADATA_PIC_STRUCT;

///////////////////////////////////////////////////////
// UData Metadata

typedef struct
{
  bool     bValid;
  void *   pUDataMemChunk;
  u_int32  uWrPtr;
  
} tMVD_METADATA_UDATA, *ptMVD_METADATA_UDATA;

#endif /* _MVD_TYPES_H_ */

/* End of file */
