/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
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

#include <linux/io.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <video/dpu.h>
#include "dpu-prv.h"

#define SSQCNTS			0
#define SSQCYCLE		0x8
#define SWRESET			0xC
#define TCON_CTRL		0x10
#define BYPASS			BIT(3)
#define RSDSINVCTRL		0x14
#define MAPBIT3_0		0x18
#define MAPBIT7_4		0x1C
#define MAPBIT11_8		0x20
#define MAPBIT15_12		0x24
#define MAPBIT19_16		0x28
#define MAPBIT23_20		0x2C
#define MAPBIT27_24		0x30
#define MAPBIT31_28		0x34
#define MAPBIT34_32		0x38
#define MAPBIT3_0_DUAL		0x3C
#define MAPBIT7_4_DUAL		0x40
#define MAPBIT11_8_DUAL		0x44
#define MAPBIT15_12_DUAL	0x48
#define MAPBIT19_16_DUAL	0x4C
#define MAPBIT23_20_DUAL	0x50
#define MAPBIT27_24_DUAL	0x54
#define MAPBIT31_28_DUAL	0x58
#define MAPBIT34_32_DUAL	0x5C
#define SPGPOSON(n)		(0x60 + (n) * 16)
#define X(n)			(((n) & 0x7FFF) << 16)
#define Y(n)			((n) & 0x7FFF)
#define SPGMASKON(n)		(0x64 + (n) * 16)
#define SPGPOSOFF(n)		(0x68 + (n) * 16)
#define SPGMASKOFF(n)		(0x6C + (n) * 16)
#define SMXSIGS(n)		(0x120 + (n) * 8)
#define SMXFCTTABLE(n)		(0x124 + (n) * 8)
#define RESET_OVER_UNFERFLOW	0x180
#define DUAL_DEBUG		0x184

struct dpu_tcon {
	void __iomem *base;
	struct mutex mutex;
	int id;
	bool inuse;
	struct dpu_soc *dpu;
};

static inline u32 dpu_tcon_read(struct dpu_tcon *tcon, unsigned int offset)
{
	return readl(tcon->base + offset);
}

static inline void dpu_tcon_write(struct dpu_tcon *tcon,
				  unsigned int offset, u32 value)
{
	writel(value, tcon->base + offset);
}

int tcon_set_fmt(struct dpu_tcon *tcon, u32 bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB888_1X24:
		dpu_tcon_write(tcon, MAPBIT3_0,   0x19181716);
		dpu_tcon_write(tcon, MAPBIT7_4,   0x1d1c1b1a);
		dpu_tcon_write(tcon, MAPBIT11_8,  0x0f0e0d0c);
		dpu_tcon_write(tcon, MAPBIT15_12, 0x13121110);
		dpu_tcon_write(tcon, MAPBIT19_16, 0x05040302);
		dpu_tcon_write(tcon, MAPBIT23_20, 0x09080706);
		break;
	case MEDIA_BUS_FMT_RGB101010_1X30:
	case MEDIA_BUS_FMT_RGB888_1X30_PADLO:
	case MEDIA_BUS_FMT_RGB666_1X30_PADLO:
		dpu_tcon_write(tcon, MAPBIT3_0,   0x17161514);
		dpu_tcon_write(tcon, MAPBIT7_4,   0x1b1a1918);
		dpu_tcon_write(tcon, MAPBIT11_8,  0x0b0a1d1c);
		dpu_tcon_write(tcon, MAPBIT15_12, 0x0f0e0d0c);
		dpu_tcon_write(tcon, MAPBIT19_16, 0x13121110);
		dpu_tcon_write(tcon, MAPBIT23_20, 0x03020100);
		dpu_tcon_write(tcon, MAPBIT27_24, 0x07060504);
		dpu_tcon_write(tcon, MAPBIT31_28, 0x00000908);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tcon_set_fmt);

void tcon_cfg_videomode(struct dpu_tcon *tcon, struct drm_display_mode *m)
{
	u32 val;
	int y;

	val = dpu_tcon_read(tcon, TCON_CTRL);
	val &= ~BYPASS;
	dpu_tcon_write(tcon, TCON_CTRL, val);

	/* dsp_control[0]: hsync */
	dpu_tcon_write(tcon, SPGPOSON(0), X(m->hsync_start));
	dpu_tcon_write(tcon, SPGMASKON(0), 0xffff);

	dpu_tcon_write(tcon, SPGPOSOFF(0), X(m->hsync_end));
	dpu_tcon_write(tcon, SPGMASKOFF(0), 0xffff);

	dpu_tcon_write(tcon, SMXSIGS(0), 0x2);
	dpu_tcon_write(tcon, SMXFCTTABLE(0), 0x1);

	/* dsp_control[1]: vsync */
	dpu_tcon_write(tcon, SPGPOSON(1),
				X(m->hsync_start) | Y(m->vsync_start - 1));
	dpu_tcon_write(tcon, SPGMASKON(1), 0x0);

	dpu_tcon_write(tcon, SPGPOSOFF(1),
				X(m->hsync_start) | Y(m->vsync_end - 1));
	dpu_tcon_write(tcon, SPGMASKOFF(1), 0x0);

	dpu_tcon_write(tcon, SMXSIGS(1), 0x3);
	dpu_tcon_write(tcon, SMXFCTTABLE(1), 0x1);

	/* dsp_control[2]: data enable */
	/* horizontal */
	dpu_tcon_write(tcon, SPGPOSON(2), 0x0);
	dpu_tcon_write(tcon, SPGMASKON(2), 0xffff);

	dpu_tcon_write(tcon, SPGPOSOFF(2), X(m->hdisplay));
	dpu_tcon_write(tcon, SPGMASKOFF(2), 0xffff);

	/* vertical */
	dpu_tcon_write(tcon, SPGPOSON(3), 0x0);
	dpu_tcon_write(tcon, SPGMASKON(3), 0x7fff0000);

	dpu_tcon_write(tcon, SPGPOSOFF(3), Y(m->vdisplay));
	dpu_tcon_write(tcon, SPGMASKOFF(3), 0x7fff0000);

	dpu_tcon_write(tcon, SMXSIGS(2), 0x2c);
	dpu_tcon_write(tcon, SMXFCTTABLE(2), 0x8);

	/* dsp_control[3]: kachuck */
	y = m->vdisplay + 1;

	dpu_tcon_write(tcon, SPGPOSON(4), X(0x0) | Y(y));
	dpu_tcon_write(tcon, SPGMASKON(4), 0x0);

	dpu_tcon_write(tcon, SPGPOSOFF(4), X(0x20) | Y(y));
	dpu_tcon_write(tcon, SPGMASKOFF(4), 0x0);

	dpu_tcon_write(tcon, SMXSIGS(3), 0x6);
	dpu_tcon_write(tcon, SMXFCTTABLE(3), 0x2);
}
EXPORT_SYMBOL_GPL(tcon_cfg_videomode);

struct dpu_tcon *dpu_tcon_get(struct dpu_soc *dpu, int id)
{
	struct dpu_tcon *tcon;
	int i;

	for (i = 0; i < ARRAY_SIZE(tcon_ids); i++)
		if (tcon_ids[i] == id)
			break;

	if (i == ARRAY_SIZE(tcon_ids))
		return ERR_PTR(-EINVAL);

	tcon = dpu->tcon_priv[i];

	mutex_lock(&tcon->mutex);

	if (tcon->inuse) {
		mutex_unlock(&tcon->mutex);
		return ERR_PTR(-EBUSY);
	}

	tcon->inuse = true;

	mutex_unlock(&tcon->mutex);

	return tcon;
}
EXPORT_SYMBOL_GPL(dpu_tcon_get);

void dpu_tcon_put(struct dpu_tcon *tcon)
{
	mutex_lock(&tcon->mutex);

	tcon->inuse = false;

	mutex_unlock(&tcon->mutex);
}
EXPORT_SYMBOL_GPL(dpu_tcon_put);

void _dpu_tcon_init(struct dpu_soc *dpu, unsigned int id)
{
}

int dpu_tcon_init(struct dpu_soc *dpu, unsigned int id,
			unsigned long unused, unsigned long base)
{
	struct dpu_tcon *tcon;

	tcon = devm_kzalloc(dpu->dev, sizeof(*tcon), GFP_KERNEL);
	if (!tcon)
		return -ENOMEM;

	dpu->tcon_priv[id] = tcon;

	tcon->base = devm_ioremap(dpu->dev, base, SZ_512);
	if (!tcon->base)
		return -ENOMEM;

	tcon->dpu = dpu;
	mutex_init(&tcon->mutex);

	return 0;
}
