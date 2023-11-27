/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef _VDI_H_
#define _VDI_H_

#include "wave6-vpuconfig.h"
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/device.h>

/* system register write */
#define vpu_write_reg(VPU_INST, ADDR, DATA) wave6_vdi_writel(VPU_INST, ADDR, DATA)
/* system register read */
#define vpu_read_reg(CORE, ADDR) wave6_vdi_readl(CORE, ADDR)

struct vpu_buf {
	size_t size;
	dma_addr_t daddr;
	void *vaddr;
};

struct vpu_dma_buf {
	size_t size;
	dma_addr_t dma_addr;
	void *vaddr;
	phys_addr_t phys_addr;
};

enum endian_mode {
	VDI_LITTLE_ENDIAN = 0, /* 64bit LE */
	VDI_BIG_ENDIAN, /* 64bit BE */
	VDI_32BIT_LITTLE_ENDIAN,
	VDI_32BIT_BIG_ENDIAN,
	/* WAVE PRODUCTS */
	VDI_128BIT_LITTLE_ENDIAN = 16,
	VDI_128BIT_LE_BYTE_SWAP,
	VDI_128BIT_LE_WORD_SWAP,
	VDI_128BIT_LE_WORD_BYTE_SWAP,
	VDI_128BIT_LE_DWORD_SWAP,
	VDI_128BIT_LE_DWORD_BYTE_SWAP,
	VDI_128BIT_LE_DWORD_WORD_SWAP,
	VDI_128BIT_LE_DWORD_WORD_BYTE_SWAP,
	VDI_128BIT_BE_DWORD_WORD_BYTE_SWAP,
	VDI_128BIT_BE_DWORD_WORD_SWAP,
	VDI_128BIT_BE_DWORD_BYTE_SWAP,
	VDI_128BIT_BE_DWORD_SWAP,
	VDI_128BIT_BE_WORD_BYTE_SWAP,
	VDI_128BIT_BE_WORD_SWAP,
	VDI_128BIT_BE_BYTE_SWAP,
	VDI_128BIT_BIG_ENDIAN = 31,
	VDI_ENDIAN_MAX
};

#define VDI_128BIT_ENDIAN_MASK 0xf
/**
 * @brief make clock stable before changing clock frequency
 * @detail before invoking vdi_set_clock_freg caller MUST invoke vdi_ready_change_clock
 *		function.
 * after changing clock frequency caller also invoke wave6_vdi_done_change_clock() function.
 * @return 0 failure
 * 1 success
 */
int wave6_vdi_ready_change_clock(unsigned long core_idx);
int wave6_vdi_set_change_clock(unsigned long core_idx, unsigned long clock_mask);
int wave6_vdi_done_change_clock(unsigned long core_idx);
int wave6_vdi_buffer_sync(struct device *dev, struct vpu_buf *vb, int dir);

#endif //#ifndef _VDI_H_
