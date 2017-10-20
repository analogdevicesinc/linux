/*
 * Copyright 2017 NXP
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file vpu_debug.h
 *
 * @brief VPU debug definition
 *
 * @ingroup VPU
 */

#ifndef __VPU_DEBUG_H
#define __VPU_DEBUG_H



#define LVL_NOPRINT 0
#define LVL_ISR 2
#define LVL_CRI 3
#define LVL_FUNC 5
#define LVL_PRINTALL 10



#ifdef VPU_KERNEL_BUILD

#include <linux/io.h>

//#define vpu_lib_dbg_level LVL_PRINTALL

#define vpu_lib_dbg_level LVL_NOPRINT

#define err_msg(fmt, arg...) do { if (vpu_lib_dbg_level > LVL_NOPRINT) \
	printk("[ERR]\t%s:%d " fmt,  __FILE__, __LINE__, ## arg); else \
	printk("[ERR]\t" fmt, ## arg); \
	} while (0)
#define info_msg(fmt, arg...) do { if (vpu_lib_dbg_level > LVL_NOPRINT) \
	printk("[INFO]\t%s:%d " fmt,  __FILE__, __LINE__, ## arg); else \
	printk("[INFO]\t" fmt, ## arg); \
	} while (0)
#define warn_msg(fmt, arg...) do { if (vpu_lib_dbg_level > LVL_NOPRINT) \
	printk("[WARN]\t%s:%d " fmt,  __FILE__, __LINE__, ## arg); else \
	printk("[WARN]\t" fmt, ## arg); \
	} while (0)

#define dprintf(level, fmt, arg...) do {if (level <= vpu_lib_dbg_level) printk("[DEBUG]\t%s " fmt, __FUNCTION__, ## arg);} while(0)



#define ENTER_FUNC() dprintf(LVL_FUNC, "enter %s()\n", __func__)
#define EXIT_FUNC() dprintf(LVL_FUNC, "exit %s()\n", __func__)

#else

#include <stdio.h>

#include "VPU_lib.h"


//#define vpu_lib_dbg_level LVL_PRINTALL

#define vpu_lib_dbg_level LVL_CRI

#define err_msg(fmt, arg...) do { if (vpu_lib_dbg_level > LVL_NOPRINT) \
	printf("[ERR]\t%s:%d " fmt,  __FILE__, __LINE__, ## arg); else \
	printf("[ERR]\t" fmt, ## arg); \
	} while (0)
#define info_msg(fmt, arg...) do { if (vpu_lib_dbg_level > LVL_NOPRINT) \
	printf("[INFO]\t%s:%d " fmt,  __FILE__, __LINE__, ## arg); else \
	printf("[INFO]\t" fmt, ## arg); \
	} while (0)
#define warn_msg(fmt, arg...) do { if (vpu_lib_dbg_level > LVL_NOPRINT) \
	printf("[WARN]\t%s:%d " fmt,  __FILE__, __LINE__, ## arg); else \
	printf("[WARN]\t" fmt, ## arg); \
	} while (0)

//#define dprintf(level, fmt, arg...) do {if (level <= vpu_lib_dbg_level) printf("[DEBUG]\t%s " fmt, __FUNCTION__, ## arg);} while(0)
#define dprintf(level, fmt, arg...) do { if (vpu_lib_dbg_level >= level) printf("[DEBUG]\t%s:%d " fmt, __FILE__, __LINE__, ## arg); } while (0)

#define ENTER_FUNC() dprintf(LVL_FUNC, "enter %s()\n", __func__)
#define EXIT_FUNC() dprintf(LVL_FUNC, "exit %s()\n", __func__)

#endif

#endif
