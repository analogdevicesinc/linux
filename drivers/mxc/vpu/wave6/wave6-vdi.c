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
