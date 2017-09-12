/*
 * Copyright 2018 NXP
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

#ifndef _IMX8_PIXEL_COMBINER_H_
#define _IMX8_PIXEL_COMBINER_H_

enum {
	PC_BYPASS,
	PC_COMBINE,
	PC_CONVERSION,
	PC_SPLIT_RGB,
};

struct pc;

void pc_enable(struct pc *pc);
void pc_disable(struct pc *pc);
void pc_configure(struct pc *pc, unsigned int di, unsigned int frame_width,
		u32 mode, u32 format);
struct pc *pc_lookup_by_phandle(struct device *dev, const char *name);

#endif
