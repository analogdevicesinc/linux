/*
 *  H2 Encoder device driver (kernel module)
 *
 *  COPYRIGHT(C) 2014 VERISILICON
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef _HX280ENC_H_
#define _HX280ENC_H_
#include <linux/ioctl.h>    /* needed for the _IOW etc stuff used later */
#ifdef HANTROMMU_SUPPORT
#include "hantrommu.h"
#endif

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

#define ENC_HW_ID1                  0x48320100
#define ENC_HW_ID2                  0x80006000
#define CORE_INFO_MODE_OFFSET       31
#define CORE_INFO_AMOUNT_OFFSET     28

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
#define HX280ENC_IOCGHWOFFSET      _IOR(HX280ENC_IOC_MAGIC,  3, unsigned long *)
#define HX280ENC_IOCGHWIOSIZE      _IOR(HX280ENC_IOC_MAGIC,  4, unsigned int *)
#define HX280ENC_IOC_CLI           _IO(HX280ENC_IOC_MAGIC,  5)
#define HX280ENC_IOC_STI           _IO(HX280ENC_IOC_MAGIC,  6)
#define HX280ENC_IOCXVIRT2BUS      _IOWR(HX280ENC_IOC_MAGIC,  7, unsigned long *)

#define HX280ENC_IOCHARDRESET      _IO(HX280ENC_IOC_MAGIC, 8)   /* debugging tool */

#define HX280ENC_IOCGSRAMOFFSET    _IOR(HX280ENC_IOC_MAGIC,  9, unsigned long *)
#define HX280ENC_IOCGSRAMEIOSIZE    _IOR(HX280ENC_IOC_MAGIC,  10, unsigned int *)
#define HX280ENC_IOCH_ENC_RESERVE   _IOR(HX280ENC_IOC_MAGIC, 11, unsigned int *)
#define HX280ENC_IOCH_ENC_RELEASE   _IOR(HX280ENC_IOC_MAGIC, 12, unsigned int *)
#define HX280ENC_IOCG_CORE_NUM      _IOR(HX280ENC_IOC_MAGIC, 13, unsigned int *)
#define HX280ENC_IOCG_EN_CORE      _IOR(HX280ENC_IOC_MAGIC, 14, unsigned int *)

#define HX280ENC_IOCG_CORE_WAIT     _IOR(HX280ENC_IOC_MAGIC, 19, unsigned int *)

#define HX280ENC_IOC_WRITE_REGS     _IOW(HX280ENC_IOC_MAGIC, 20, struct enc_regs_buffer *)
#define HX280ENC_IOC_READ_REGS      _IOR(HX280ENC_IOC_MAGIC, 21, struct enc_regs_buffer *)

#define HX280ENC_IOC_MAXNR 30

typedef struct {
  unsigned long base_addr;
  u32 iosize;
  int irq;
  u32 resouce_shared; //indicate the core share resources with other cores or not.If 1, means cores can not work at the same time.
} CORE_CONFIG;

struct enc_regs_buffer {
	u32 core_id;
	u32 *regs;
	u32 offset;
	u32 size;
	u32 *reserved;
};

#endif /* !_HX280ENC_H_ */
