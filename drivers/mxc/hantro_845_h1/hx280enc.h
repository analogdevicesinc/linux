 /*****************************************************************************
 * Encoder device driver (kernel module header)
 *
 * Copyright (C) 2012 Google Finland Oy.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
--------------------------------------------------------------------------------
--
--  Abstract : 6280/7280/8270/8290/H1 Encoder device driver (kernel module)
--
*****************************************************************************/
#ifndef _UAPI_HX280ENC_H_
#define _UAPI_HX280ENC_H_
#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */

/*
 * Macros to help debugging
 */

#undef PDEBUG   /* undef it, just in case */
#ifdef HX280ENC_DEBUG
#  ifdef __KERNEL__
    /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk(KERN_INFO "hmp4e: " fmt, ## args)
#  else
    /* This one for user space */
#    define PDEBUG(fmt, args...) printf(__FILE__ ":%d: " fmt, __LINE__, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...)  /* not debugging: nothing */
#endif

/*
 * Ioctl definitions
 */
struct mirror_regs {
	unsigned int regs[512]; /* pointer to user registers */
};

/* Use 'k' as magic number */
#define HX280ENC_IOC_MAGIC  'k'
/*
 * S means "Set" through a ptr,
 * T means "Tell" directly with the argument value
 * G means "Get": reply by setting through a pointer
 * Q means "Query": response is on the return value
 * X means "eXchange": G and S atomically
 * H means "sHift": T and Q atomically
 */
 /*
  * #define HX280ENC_IOCGBUFBUSADDRESS _IOR(HX280ENC_IOC_MAGIC,  1, unsigned long *)
  * #define HX280ENC_IOCGBUFSIZE       _IOR(HX280ENC_IOC_MAGIC,  2, unsigned int *)
  */
#define HX280ENC_IOCGHWOFFSET      _IOR(HX280ENC_IOC_MAGIC,  3, u32 *)
#define HX280ENC_IOCGHWIOSIZE      _IOR(HX280ENC_IOC_MAGIC,  4, u32 *)

#define HX280ENC_IOCH_ENC_RESERVE   _IOR(HX280ENC_IOC_MAGIC, 11, u32 *)
#define HX280ENC_IOCH_ENC_RELEASE   _IOR(HX280ENC_IOC_MAGIC, 12, u32 *)
#define HX280ENC_IOCG_CORE_WAIT     _IOR(HX280ENC_IOC_MAGIC, 13, u32 *)
#define HX280ENC_IOC_MAXNR 30

#endif /* !_UAPI_HX280ENC_H_ */
