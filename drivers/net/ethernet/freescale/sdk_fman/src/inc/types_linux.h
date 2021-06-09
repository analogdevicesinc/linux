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

#ifndef __TYPES_LINUX_H__
#define __TYPES_LINUX_H__

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/delay.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
    #error "This kernel is probably not supported!!!"
#elif   (!((LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19)) || \
           (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,27)) || \
           (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,30))))
    #warning "This kernel is probably not supported!!! You may need to add some fixes."
#endif    /* LINUX_VERSION_CODE */


typedef float               float_t;    /* Single precision floating point  */
typedef double              double_t;   /* Double precision floating point  */


#define _Packed
#define _PackedType __attribute__ ((packed))

typedef  phys_addr_t physAddress_t;

#define UINT8_MAX   0xFF
#define UINT8_MIN   0
#define UINT16_MAX  0xFFFF
#define UINT16_MIN  0
#define UINT32_MAX  0xFFFFFFFF
#define UINT32_MIN  0
#define UINT64_MAX  0xFFFFFFFFFFFFFFFFLL
#define UINT64_MIN  0
#define INT8_MAX    0x7F
#define INT8_MIN    0x80
#define INT16_MAX   0x7FFF
#define INT16_MIN   0x8000
#define INT32_MAX   0x7FFFFFFF
#define INT32_MIN   0x80000000
#define INT64_MAX   0x7FFFFFFFFFFFFFFFLL
#define INT64_MIN   0x8000000000000000LL

#define ON          1
#define OFF         0

#define FALSE       false
#define TRUE        true


/************************/
/* memory access macros */
/************************/
#ifdef CONFIG_FMAN_ARM
#define in_be16(a)		__be16_to_cpu(__raw_readw(a))
#define in_be32(a)		__be32_to_cpu(__raw_readl(a))
#define out_be16(a, v)		__raw_writew(__cpu_to_be16(v), a)
#define out_be32(a, v)		__raw_writel(__cpu_to_be32(v), a)
#endif

#define GET_UINT8(arg)              *(volatile uint8_t *)(&(arg))
#define GET_UINT16(arg)             in_be16(&(arg))//*(volatile uint16_t*)(&(arg))
#define GET_UINT32(arg)             in_be32(&(arg))//*(volatile uint32_t*)(&(arg))
#define GET_UINT64(arg)             *(volatile uint64_t*)(&(arg))

#ifdef VERBOSE_WRITE
void    XX_Print(char *str, ...);
#define WRITE_UINT8(arg, data)  \
    do { XX_Print("ADDR: 0x%08x, VAL: 0x%02x\r\n",    (uint32_t)&(arg), (data)); *(volatile uint8_t *)(&(arg)) = (data); } while (0)
#define WRITE_UINT16(arg, data) \
    do { XX_Print("ADDR: 0x%08x, VAL: 0x%04x\r\n",    (uint32_t)&(arg), (data)); out_be16(&(arg), data); /* *(volatile uint16_t*)(&(arg)) = (data);*/ } while (0)
#define WRITE_UINT32(arg, data) \
    do { XX_Print("ADDR: 0x%08x, VAL: 0x%08x\r\n",    (uint32_t)&(arg), (data)); out_be32(&(arg), data); /* *(volatile uint32_t*)(&(arg)) = (data);*/ } while (0)
#define WRITE_UINT64(arg, data) \
    do { XX_Print("ADDR: 0x%08x, VAL: 0x%016llx\r\n", (uint32_t)&(arg), (data)); *(volatile uint64_t*)(&(arg)) = (data); } while (0)

#else  /* not VERBOSE_WRITE */
#define WRITE_UINT8(arg, data)      *(volatile uint8_t *)(&(arg)) = (data)
#define WRITE_UINT16(arg, data)     out_be16(&(arg), data)//*(volatile uint16_t*)(&(arg)) = (data)
#define WRITE_UINT32(arg, data)     out_be32(&(arg), data)//*(volatile unsigned int *)(&(arg)) = (data)
#define WRITE_UINT64(arg, data)     *(volatile uint64_t*)(&(arg)) = (data)
#endif /* not VERBOSE_WRITE */


/*****************************************************************************/
/*                      General stuff                                        */
/*****************************************************************************/
#ifdef ARRAY_SIZE
#undef ARRAY_SIZE
#endif /* ARRAY_SIZE */

#ifdef MAJOR
#undef MAJOR
#endif /* MAJOR */

#ifdef MINOR
#undef MINOR
#endif /* MINOR */

#ifdef QE_SIZEOF_BD
#undef QE_SIZEOF_BD
#endif /* QE_SIZEOF_BD */

#ifdef BD_BUFFER_CLEAR
#undef BD_BUFFER_CLEAR
#endif /* BD_BUFFER_CLEAR */

#ifdef BD_BUFFER
#undef BD_BUFFER
#endif /* BD_BUFFER */

#ifdef BD_STATUS_AND_LENGTH_SET
#undef BD_STATUS_AND_LENGTH_SET
#endif /* BD_STATUS_AND_LENGTH_SET */

#ifdef BD_STATUS_AND_LENGTH
#undef BD_STATUS_AND_LENGTH
#endif /* BD_STATUS_AND_LENGTH */

#ifdef BD_BUFFER_ARG
#undef BD_BUFFER_ARG
#endif /* BD_BUFFER_ARG */

#ifdef BD_GET_NEXT
#undef BD_GET_NEXT
#endif /* BD_GET_NEXT */

#ifdef QE_SDEBCR_BA_MASK
#undef QE_SDEBCR_BA_MASK
#endif /* QE_SDEBCR_BA_MASK */

#ifdef BD_BUFFER_SET
#undef BD_BUFFER_SET
#endif /* BD_BUFFER_SET */

#ifdef UPGCR_PROTOCOL
#undef UPGCR_PROTOCOL
#endif /* UPGCR_PROTOCOL */

#ifdef UPGCR_TMS
#undef UPGCR_TMS
#endif /* UPGCR_TMS */

#ifdef UPGCR_RMS
#undef UPGCR_RMS
#endif /* UPGCR_RMS */

#ifdef UPGCR_ADDR
#undef UPGCR_ADDR
#endif /* UPGCR_ADDR */

#ifdef UPGCR_DIAG
#undef UPGCR_DIAG
#endif /* UPGCR_DIAG */

#ifdef NCSW_PARAMS
#undef NCSW_PARAMS
#endif /* NCSW_PARAMS */

#ifdef NO_IRQ
#undef NO_IRQ
#endif /* NO_IRQ */

#define PRINT_LINE   XX_Print("%s:\n %s [%d]\n",__FILE__,__FUNCTION__,__LINE__);


#endif /* __TYPES_LINUX_H__ */
