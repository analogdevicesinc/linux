/*
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __NWL_DSI__
#define __NWL_DSI__

#include <drm/drmP.h>
#include <drm/drm_mipi_dsi.h>
#include <linux/phy/phy.h>

/*
 * RGB bit distribution within the 24-bit data bus,
 * as specified by the DPI specification
 */
enum dpi_interface_color_coding {
	DPI_16_BIT_565_PACKED, /* 0x0 cfg1 */
	DPI_16_BIT_565_ALIGNED, /* 0x1 cfg 2 */
	DPI_16_BIT_565_SHIFTED, /* 0x2 cfg 3 */
	DPI_18_BIT_PACKED, /* 0x3 cfg1 */
	DPI_18_BIT_ALIGNED, /* 0x4* cfg2 */
	DPI_24_BIT /* 0x5 */
};

/* DSI packet type of pixels, as specified by the DPI specification */
enum dpi_pixel_format {
	DPI_FMT_16_BIT, /* 0x0 */
	DPI_FMT_18_BIT, /* 0x1 */
	DPI_FMT_18_BIT_LOOSELY_PACKED, /* 0x2 */
	DPI_FMT_24_BIT /* 0x3 */
};

unsigned long nwl_dsi_get_bit_clock(struct drm_bridge *bridge,
	unsigned long pixclock);

#endif /* __NWL_DSI_H__ */
