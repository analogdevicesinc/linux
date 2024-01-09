/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * Wave6 series multi-standard codec IP - vpu control interface
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#ifndef __WAVE6_VPU_CTRL_H__
#define __WAVE6_VPU_CTRL_H__

#include <linux/device.h>

enum {
	WAVE6_VPU_STATE_OFF,
	WAVE6_VPU_STATE_PREPARE,
	WAVE6_VPU_STATE_ON,
	WAVE6_VPU_STATE_SLEEP,
};

struct wave6_vpu_entity {
	struct list_head list;
	struct device *dev;
	u32 (*read_reg)(struct device *dev, u32 addr);
	void (*write_reg)(struct device *dev, u32 addr, u32 data);
	void (*on_boot)(struct device *dev);
	bool booted;
};

int wave6_convert_endian(unsigned int endian);
void wave6_swap_endian(u32 product_code, u8 *data, int len, int endian);
int wave6_vpu_ctrl_resume_and_get(struct device *dev, struct wave6_vpu_entity *entity);
void wave6_vpu_ctrl_put_sync(struct device *dev, struct wave6_vpu_entity *entity);
int wave6_vpu_ctrl_get_state(struct device *dev);
int wave6_vpu_ctrl_wait_done(struct device *dev, struct wave6_vpu_entity *entity);
void *wave6_vpu_ctrl_get_sram(struct device *dev, dma_addr_t *dma_addr, u32 *size);
#endif
