/*
 * Copyright (C) 2017 Intel Corporation.
 *
 * Intel Video and Image Processing(VIP) Frame Buffer II driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _ALTVIPFB2_H
#define _ALTVIPFB2_H
#include <linux/io.h>
#include <linux/fb.h>

#define DRIVER_NAME	"altvipfb2"
#define PALETTE_SIZE	256
#define BYTES_PER_PIXEL		4

/* control registers */
#define ALTVIPFB2_CONTROL		0
#define ALTVIPFB2_STATUS		0x4
#define ALTVIPFB2_INTERRUPT		0x8
#define ALTVIPFB2_FRAME_COUNTER		0xC
#define ALTVIPFB2_FRAME_DROP		0x10
#define ALTVIPFB2_FRAME_INFO		0x14
#define ALTVIPFB2_FRAME_START		0x18
#define ALTVIPFB2_FRAME_READER		0x1C

int altvipfb2_probe(struct device *dev, void __iomem *base);
int altvipfb2_remove(struct device *dev);

struct altvipfb2_priv {
	struct	fb_info info;
	void	__iomem *base;
	int		irq_base;
	u32	pseudo_palette[PALETTE_SIZE];
};

#endif /* _ALTVIPFB2_H */
