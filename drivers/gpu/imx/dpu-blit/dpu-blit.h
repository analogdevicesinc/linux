/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef __DPU_BLIT_H__
#define __DPU_BLIT_H__

#define COMMAND_BUFFER_SIZE	65536 /* up to 64k bytes */
#define CMDSEQ_FIFO_SPACE_THRESHOLD   192
#define WORD_SIZE   4

struct dpu_bliteng {
	struct device		*dev;
	void __iomem *base;
	s32 id;
	struct mutex mutex;
	bool inuse;
	s32 irq_store9_shdload;
	s32 irq_store9_framecomplete;
	s32 irq_store9_seqcomplete;

	void *buffer_addr_virt;
	u32 buffer_addr_phy;

	u32 *cmd_list;

	struct dpu_soc *dpu;
};

#endif
