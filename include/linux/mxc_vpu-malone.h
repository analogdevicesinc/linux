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
 * @defgroup VPU Video Processor Unit Driver
 */

/*!
 * @file linux/mxc_vpu-malone.h
 *
 * @brief VPU system initialization and file operation definition
 *
 * @ingroup VPU
 */

#ifndef __LINUX_MXC_VPU_MALONE_H__
#define __LINUX_MXC_VPU_MALONE_H__

#include <linux/fs.h>

struct vpu_mem_desc {
	u32 size;
	dma_addr_t phy_addr;
	void *cpu_addr;		/* cpu address to free the dma mem */
	u64 virt_uaddr;		/* virtual user space address */
};

#define VPU_IOC_MAGIC  'V'

#define VPU_IOC_PHYMEM_ALLOC	_IO(VPU_IOC_MAGIC, 0)
#define VPU_IOC_PHYMEM_FREE	_IO(VPU_IOC_MAGIC, 1)
#define VPU_IOC_WAIT4INT	_IO(VPU_IOC_MAGIC, 2)
#define VPU_IOC_CLKGATE_SETTING	_IO(VPU_IOC_MAGIC, 3)
#define VPU_IOC_REQ_VSHARE_MEM	_IO(VPU_IOC_MAGIC, 4)
#define VPU_IOC_SYS_SW_RESET	_IO(VPU_IOC_MAGIC, 5)
#define VPU_IOC_GET_SHARE_MEM   _IO(VPU_IOC_MAGIC, 6)
#define VPU_IOC_LOCK_DEV	_IO(VPU_IOC_MAGIC, 7)

#endif
