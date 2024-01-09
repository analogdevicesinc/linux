// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - low level access functions
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include <linux/bug.h>
#include "wave6-vdi.h"
#include "wave6-vpu.h"
#include "wave6-regdefine.h"
#include <linux/delay.h>

#define VDI_SYSTEM_ENDIAN VDI_LITTLE_ENDIAN
#define VDI_128BIT_BUS_SYSTEM_ENDIAN VDI_128BIT_LITTLE_ENDIAN

void wave6_vdi_writel(struct vpu_device *vpu_dev, unsigned int addr, unsigned int data)
{
	writel(data, vpu_dev->reg_base + addr);
}

unsigned int wave6_vdi_readl(struct vpu_device *vpu_dev, u32 addr)
{
	return readl(vpu_dev->reg_base + addr);
}

int wave6_vdi_clear_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	if (!vb || !vb->vaddr) {
		dev_err(vpu_dev->dev, "%s(): unable to clear unmapped buffer\n", __func__);
		return -EINVAL;
	}

	memset(vb->vaddr, 0, vb->size);
	return vb->size;
}

int wave6_vdi_write_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb, size_t offset,
			   u8 *data, int len, int endian)
{
	if (!vb || !vb->vaddr) {
		dev_err(vpu_dev->dev, "%s(): unable to write to unmapped buffer\n", __func__);
		return -EINVAL;
	}

	if (offset > vb->size || len > vb->size || offset + len > vb->size) {
		dev_err(vpu_dev->dev, "%s(): buffer too small\n", __func__);
		return -ENOSPC;
	}

	wave6_swap_endian(vpu_dev->product_code, data, len, endian);
	memcpy(vb->vaddr + offset, data, len);
	return len;
}

int wave6_vdi_allocate_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	void *vaddr;
	dma_addr_t daddr;

	if (!vb->size) {
		dev_err(vpu_dev->dev, "%s(): requested size==0\n", __func__);
		return -EINVAL;
	}
	vaddr = dma_alloc_coherent(vpu_dev->dev, vb->size, &daddr, GFP_KERNEL);
	if (!vaddr)
		return -ENOMEM;
	vb->vaddr = vaddr;
	vb->daddr = daddr;

	return 0;
}

void wave6_vdi_free_dma_memory(struct vpu_device *vpu_dev, struct vpu_buf *vb)
{
	if (vb->size == 0)
		return;

	if (!vb->vaddr)
		dev_err(vpu_dev->dev, "%s(): requested free of unmapped buffer\n", __func__);
	else
		dma_free_coherent(vpu_dev->dev, vb->size, vb->vaddr, vb->daddr);

	memset(vb, 0, sizeof(*vb));
}
