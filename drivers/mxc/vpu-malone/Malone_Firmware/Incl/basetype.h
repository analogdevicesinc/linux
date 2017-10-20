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
  Filename:        basetype.h
  Description:     Public header file for use in all FW
  Author:          Media IP FW team (Belfast)
 
 ************************************************/

#ifndef _BASETYPE_H_
#define _BASETYPE_H_
#ifndef VPU_KERNEL_BUILD
#include <stdint.h>
#include <stdio.h>
#else
#include <linux/types.h>
#include <linux/kernel.h>
#endif

#ifdef WIN32
#if _MSC_VER
typedef unsigned __int64    u_int64;
#else
typedef unsigned long long  u_int64;
#endif
#else
typedef unsigned long long  u_int64;
#endif

#ifdef MALONE_64BIT_ADDR
typedef uint64_t       uint_addr;
#else
typedef uint32_t       uint_addr;
#endif

typedef uint32_t       u_int32;
typedef uint16_t       u_int16;
typedef uint8_t        u_int8;

//Alignment Macro
#define ALIGN_SIZE(x)    (((x)+sizeof(u_int32) - 1)/(sizeof(u_int32)))

#ifdef WIN32
#if _MSC_VER
typedef signed __int64      int64;
#else
typedef signed long long    int64;
#endif
#else
typedef signed long long    int64;
#endif

typedef int32_t         int32;
typedef int16_t         int16;
typedef int8_t          int8;
typedef unsigned char       BYTE;

// XXX:TAS  No idea what all this ifdefing is about
//          should just need the C++ ifdef
#ifndef HOST_BUILD
#ifndef COREPLAY_API 
#ifndef _MSC_VER
#ifndef VPU_KERNEL_BUILD
typedef unsigned int        bool;
#endif
#endif
#else
#ifndef _MSC_VER
typedef unsigned int        bool;
#endif
#endif
#else
  #ifndef __cplusplus
    // bool is defined in cpp
    typedef unsigned int        bool;
  #endif /* __cplusplus */
#endif

typedef unsigned char       BOOLEAN;

#define VOID_PARAMS         void

#define INT32               int32

#define UINT8              u_int8
#define UINT16             u_int16
#define UINT32             u_int32
#define UINT64             u_int64
#ifdef _MSC_VER
#define BOOL               int
#else
#define BOOL               bool
#endif
#define SINT8              int8
#define SINT16             int16
#define SINT32             int32
#ifdef WIN32
#define SINT64             int64
#define SINT64_MIN         (-9223372036854775807i64 - 1i64)
#else
#define SINT64             int64
#define SINT64_MIN         (-9223372036854775807LL - 1LL)
#endif

#ifndef VPU_KERNEL_BUILD
#define SUCCESS 0
#define FAILURE -1 
#endif

#ifdef __cplusplus
#ifndef FALSE
  #define FALSE 0
#endif  
#ifndef TRUE
  #define TRUE  1
#endif  
#else  // 'C'
#ifndef false
  #define false 0
#endif  
#ifndef true
  #define true  1
#endif  

#ifndef FALSE
  #define FALSE 0
#endif  
#ifndef TRUE
  #define TRUE  1
#endif  
#endif

#ifndef NULL
#define NULL 0
#endif

#ifndef OKAY
#define	OKAY 0
#endif

#ifndef ZERO
#define	ZERO 0
#endif

// Define more error types as used - 1st is generic
#define MEDIAIP_ERROR_FLAG        1
#define MEDIAIP_END_ES            2
#define MEDIAIP_DECODER_EXIT      3
#define MEDIAIP_ENCODER_EXIT      4
#define MEDIAIP_VC1D_NOFREEFRAMES 5
#define MEDIAIP_DECODER_SKIP      6

/* Alignment macros - align address to a specific byte alignment */
#define MEDIAIP_ALIGN_16(addr)  addr = (addr + (( 0x1 << 0x4 )  - 0x1 ) ) & ~((( 0x1 << 0x4 )  - 0x1 ))
#define MEDIAIP_ALIGN_32(addr)  addr = (addr + (( 0x1 << 0x5 )  - 0x1 ) ) & ~((( 0x1 << 0x5 )  - 0x1 ))
#define MEDIAIP_ALIGN_64(addr)  addr = (addr + (( 0x1 << 0x6 )  - 0x1 ) ) & ~((( 0x1 << 0x6 )  - 0x1 ))
#define MEDIAIP_ALIGN_128(addr)  addr = (addr + (( 0x1 << 0x7 )  - 0x1 ) ) & ~((( 0x1 << 0x7 )  - 0x1 ))
#define MEDIAIP_ALIGN_256(addr)  addr = (addr + (( 0x1 << 0x8 )  - 0x1 ) ) & ~((( 0x1 << 0x8 )  - 0x1 ))
#define MEDIAIP_ALIGN_512(addr)  addr = (addr + (( 0x1 << 0x9 )  - 0x1 ) ) & ~((( 0x1 << 0x9 )  - 0x1 ))

/* used when declaring single bitfields
*/
#define FLAG                      1

#ifndef WIN32
 #if CPU == MIPS
  #define DOUBLE_ALIGN __attribute__((aligned(8)))
  #define SECTION(sec) __attribute__ ((section (#sec)))
  #define ALIGNED(ali) __attribute__ ((aligned (ali)))
 #else
   #define DOUBLE_ALIGN
 #endif
#endif // win32
/* We need to stop defining ARM and use only CPU == ARM !!! */
/* Lose all #ifdef ARM from code                            */

/* - Only for RCVCT */
#ifdef __CC_ARM
  #define ALIGN_8_u_int8      __align(8) u_int8
  #define ALIGN_8_u_int16     __align(8) u_int16
  #define ALIGN_8_u_int32     __align(8) u_int32
  #define ALIGN_256_u_int16   __align(256) u_int16
  #define ALIGN_256_u_int32   __align(256) u_int32
  #define ALIGN_1024_u_int32  __align(1024) u_int32
  #define ALIGN_1024_u_int64  __align(1024) u_int64
  #define ALIGN_1024          __align(1024)
#else
/* All others? */
  #define ALIGN_8_u_int8      u_int8 __attribute__((aligned(8)))
  #define ALIGN_8_u_int16     u_int16 __attribute__((aligned(8)))
  #define ALIGN_8_u_int32     u_int32 __attribute__((aligned(8)))  
  #define ALIGN_256_u_int16   u_int16 __attribute__((aligned(256)))
  #define ALIGN_256_u_int32   u_int32 __attribute__((aligned(256)))
  #define ALIGN_1024_u_int32  u_int32 __attribute__((aligned(1024)))
  #define ALIGN_1024_u_int64  u_int64 __attribute__((aligned(1024)))
  #define ALIGN_1024          __attribute__((aligned(1024)))
#endif

#ifndef COREPLAY_API
#if RTOS == UCOS

/* Additions for uCOS */

typedef unsigned char  INT8U;                    /* Unsigned  8 bit quantity                           */
typedef signed   char  INT8S;                    /* Signed    8 bit quantity                           */
typedef unsigned int   INT16U;                   /* Unsigned 16 bit quantity                           */
typedef signed   int   INT16S;                   /* Signed   16 bit quantity                           */
typedef unsigned long  INT32U;                   /* Unsigned 32 bit quantity                           */
typedef signed   long  INT32S;                   /* Signed   32 bit quantity                           */
typedef float          FP32;                     /* Single precision floating point                    */
typedef double         FP64;                     /* Double precision floating point                    */

typedef unsigned int   OS_STK;                   /* Each stack entry is 16-bit wide                    */
typedef unsigned int   OS_CPU_SR;                /* Define size of CPU status register (PSR = 32 bits) */

#define UBYTE          INT8U                     /* ... to uC/OS V1.xx.  Not actually needed for ...   */
#define WORD           INT16S                    /* ... uC/OS-II.                                      */
#define UWORD          INT16U
#define LONG           INT32S
#define ULONG          INT32U

#endif

typedef signed short int   SHORT;   /* Signed   16 bit quantity  */
typedef unsigned short     USHORT;

#endif /* COREPLAY_API */

/****************************************************************************/
/*                                                                          */
/* Hardware register access macros                                          */
/*                                                                          */
/****************************************************************************/

#define RMO(y) \
      ( ((y) & 0x00000001) ?  0 : \
      ( ((y) & 0x00000002) ?  1 : \
      ( ((y) & 0x00000004) ?  2 : \
      ( ((y) & 0x00000008) ?  3 : \
      ( ((y) & 0x00000010) ?  4 : \
      ( ((y) & 0x00000020) ?  5 : \
      ( ((y) & 0x00000040) ?  6 : \
      ( ((y) & 0x00000080) ?  7 : \
      ( ((y) & 0x00000100) ?  8 : \
      ( ((y) & 0x00000200) ?  9 : \
      ( ((y) & 0x00000400) ? 10 : \
      ( ((y) & 0x00000800) ? 11 : \
      ( ((y) & 0x00001000) ? 12 : \
      ( ((y) & 0x00002000) ? 13 : \
      ( ((y) & 0x00004000) ? 14 : \
      ( ((y) & 0x00008000) ? 15 : \
      ( ((y) & 0x00010000) ? 16 : \
      ( ((y) & 0x00020000) ? 17 : \
      ( ((y) & 0x00040000) ? 18 : \
      ( ((y) & 0x00080000) ? 19 : \
      ( ((y) & 0x00100000) ? 20 : \
      ( ((y) & 0x00200000) ? 21 : \
      ( ((y) & 0x00400000) ? 22 : \
      ( ((y) & 0x00800000) ? 23 : \
      ( ((y) & 0x01000000) ? 24 : \
      ( ((y) & 0x02000000) ? 25 : \
      ( ((y) & 0x04000000) ? 26 : \
      ( ((y) & 0x08000000) ? 27 : \
      ( ((y) & 0x10000000) ? 28 : \
      ( ((y) & 0x20000000) ? 29 : \
      ( ((y) & 0x40000000) ? 30 : \
      ( ((y) & 0x80000000) ? 31 : 0 ))))))))))))))))))))))))))))))))

/*
 * Access macros used to get, set/clear bits within a hardware register.
 * These macros *do not* perform any automatic shifting of bits and are
 * meant to be used with bit definitions which include their encoded bit
 * position within the register definition (e.g. an enable bit).
 */

#ifdef VSIM_ENV
  UINT32  MEDIAIP_GET(UINT32,UINT32);
  void    MEDIAIP_SET(UINT32,UINT32,UINT32);
  #define MEDIAIP_PUT(reg,mask,val)	{ \
								VSimAPI_WriteRegister((UINT32) reg, val); \
          }
  UINT32  MEDIAIP_SM_GET(volatile UINT32 *);
  void    MEDIAIP_SM_SET(volatile UINT32 *, UINT32, UINT32 );
  void    MEDIAIP_SM_PUT(volatile UINT32 *, UINT32 );

#else

  #define MEDIAIP_GET(reg,mask)      (*(LPREG)(reg) & (mask))
  #define MEDIAIP_SET(reg,mask,val)  (*(LPREG)(reg)) = ((*(LPREG)(reg) & ~(mask)) | ((val) & (mask)))
  #define MEDIAIP_PUT(reg,val)       (*(LPREG)(reg)) = (val)
  #define MEDIAIP_SM_GET(x)          (*(x))
  #define MEDIAIP_SM_SET(a,b,c)      (*(a)) = (((*(a)) & ~(b)) | ((c) & (b)))
  #define MEDIAIP_SM_PUT(a,b)        (*(a)) = (b)

// IDJ: Consider using this?
// No- there are only a few instances where this would make a difference.
//efine MEDIAIP_SET(reg,mask,val)  (*(LPREG)(reg)) = ((~(mask) ? 0 : (*(LPREG)(reg) & ~(mask))) | ((val) & (mask)))

#endif

/*
 * Access macros used to get & set a numerical value within a hardware
 * register.  These macros perform automatic shifting (based on the mask)
 * of the numerical value used.  These macros are useful for setting a
 * numerical value into a multi-bit contiguous field within a register.
 */
#define MEDIAIP_GET_VAL(reg,mask)           ((*(LPREG)(reg) & (mask)) >> RMO(mask))
#define MEDIAIP_SET_VAL(reg,mask,val)       (*(LPREG)(reg)) =                     \
                   ((*(LPREG)(reg) & ~(mask)) | (((unsigned long)(val) << RMO(mask)) & (mask)))

#define MEDIAIP_REGWRITE(reg,val)    *((LPREG)(reg)) = (val)
#define MEDIAIP_REGREAD(reg)         *((LPREG)(reg))


#define MEDIAIP_SET_REG(reg,mask,val)    reg = ((reg & ~(mask)) | ((val) & (mask)))
#define MEDIAIP_GET_REG(reg,mask)        reg & mask

typedef unsigned long   HW_DWORD;      /* was u_int32; */
typedef unsigned short  HW_WORD;       /* was u_int16; */
typedef unsigned char   HW_BYTE;       /* was u_int8; */
typedef unsigned int    HW_BOOL;       /* was bool; */
typedef void            HW_VOID;

typedef volatile u_int32 *LPREG;

#ifdef __CC_ARM
#define FUNC_INLINE	__inline
#else
#define FUNC_INLINE	inline
#endif

#endif /* _BASETYPE_H_ */

/* End of File */

