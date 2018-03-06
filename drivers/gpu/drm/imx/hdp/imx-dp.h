/*
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _IMX_DP_H_
#define _IMX_DP_H_

void dp_fw_load(state_struct *state);
int dp_fw_init(state_struct *state);
void dp_mode_set(state_struct *state, struct drm_display_mode *mode, int format, int color_depth, int max_link_rate);
int dp_phy_init(state_struct *state, struct drm_display_mode *mode, int format, int color_depth);
int dp_get_edid_block(void *data, u8 *buf, u32 block, size_t len);
int dp_get_hpd_state(state_struct *state, u8 *hpd);

#endif
