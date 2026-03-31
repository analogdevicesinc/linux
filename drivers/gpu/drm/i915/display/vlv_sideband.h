/* SPDX-License-Identifier: MIT */
/* Copyright © 2025 Intel Corporation */

#ifndef _VLV_SIDEBAND_H_
#define _VLV_SIDEBAND_H_

#include <linux/types.h>

#include "vlv_iosf_sb.h"
#include "vlv_iosf_sb_reg.h"

enum dpio_phy;
struct drm_device;

void vlv_bunit_get(struct drm_device *drm);
u32 vlv_bunit_read(struct drm_device *drm, u32 reg);
void vlv_bunit_write(struct drm_device *drm, u32 reg, u32 val);
void vlv_bunit_put(struct drm_device *drm);

void vlv_cck_get(struct drm_device *drm);
u32 vlv_cck_read(struct drm_device *drm, u32 reg);
void vlv_cck_write(struct drm_device *drm, u32 reg, u32 val);
void vlv_cck_put(struct drm_device *drm);

void vlv_ccu_get(struct drm_device *drm);
u32 vlv_ccu_read(struct drm_device *drm, u32 reg);
void vlv_ccu_write(struct drm_device *drm, u32 reg, u32 val);
void vlv_ccu_put(struct drm_device *drm);

void vlv_dpio_get(struct drm_device *drm);
u32 vlv_dpio_read(struct drm_device *drm, enum dpio_phy phy, int reg);
void vlv_dpio_write(struct drm_device *drm, enum dpio_phy phy, int reg, u32 val);
void vlv_dpio_put(struct drm_device *drm);

void vlv_flisdsi_get(struct drm_device *drm);
u32 vlv_flisdsi_read(struct drm_device *drm, u32 reg);
void vlv_flisdsi_write(struct drm_device *drm, u32 reg, u32 val);
void vlv_flisdsi_put(struct drm_device *drm);

void vlv_nc_get(struct drm_device *drm);
u32 vlv_nc_read(struct drm_device *drm, u8 addr);
void vlv_nc_put(struct drm_device *drm);

void vlv_punit_get(struct drm_device *drm);
u32 vlv_punit_read(struct drm_device *drm, u32 addr);
int vlv_punit_write(struct drm_device *drm, u32 addr, u32 val);
void vlv_punit_put(struct drm_device *drm);

#endif /* _VLV_SIDEBAND_H_ */
