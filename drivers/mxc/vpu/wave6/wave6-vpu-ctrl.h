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
	void (*pause)(struct device *dev, int resume);
	bool booted;
};

int wave6_convert_endian(unsigned int endian);
void wave6_swap_endian(u8 *data, int len, int endian);
int wave6_alloc_dma(struct device *dev, struct vpu_buf *vb);
int wave6_write_dma(struct vpu_buf *vb, size_t offset, u8 *data, int len, int endian);
void wave6_free_dma(struct vpu_buf *vb);
int wave6_vpu_ctrl_resume_and_get(struct device *dev, struct wave6_vpu_entity *entity);
void wave6_vpu_ctrl_put_sync(struct device *dev, struct wave6_vpu_entity *entity);
int wave6_vpu_ctrl_get_state(struct device *dev);
int wave6_vpu_ctrl_wait_done(struct device *dev);
int wave6_vpu_ctrl_require_buffer(struct device *dev, struct wave6_vpu_entity *entity);
bool wave6_vpu_ctrl_support_follower(struct device *dev);
#endif
