/*
 * Copyright (C) 2017 NXP
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

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <drm/drm_fourcc.h>

#include <video/imx-dcss.h>
#include <video/viv-metadata.h>
#include "dcss-prv.h"

#define USE_CTXLD			1

/* DEC400D registers offsets */
#define DEC400D_READCONFIG_BASE		0x800
#define DEC400D_READCONFIG(i)		(DEC400D_READCONFIG_BASE + ((i) << 2))
#define DEC400D_READCONFIG_BASE     0x800
#define DEC400D_READBUFFERBASE0     0x900
#define DEC400D_READCACHEBASE0      0x980
#define DEC400D_CONTROL             0xB00
#define DEC400D_CLEAR               0xB80
#define   COMPRESSION_ENABLE_BIT	0
#define   COMPRESSION_FORMAT_BIT	3
#define   COMPRESSION_ALIGN_MODE_BIT	16
#define   TILE_ALIGN_MODE_BIT		22
#define   TILE_MODE_BIT			25
#define DEC400D_READBUFFERBASE0		0x900
#define DEC400D_READCACHEBASE0		0x980
#define DEC400D_CONTROL			0xB00
#define   DISABLE_COMPRESSION_BIT	1
#define   SHADOW_TRIGGER_BIT		29
#define DEC400_CFMT_ARGB8           0x0
#define DEC400_CFMT_XRGB8           0x1
#define DEC400_CFMT_AYUV            0x2
#define DEC400_CFMT_UYVY            0x3
#define DEC400_CFMT_YUY2            0x4
#define DEC400_CFMT_YUV_ONLY        0x5
#define DEC400_CFMT_UV_MIX          0x6
#define DEC400_CFMT_ARGB4           0x7
#define DEC400_CFMT_XRGB4           0x8
#define DEC400_CFMT_A1R5G5B5        0x9
#define DEC400_CFMT_X1R5G5B5        0xA
#define DEC400_CFMT_R5G6B5          0xB
#define DEC400_CFMT_Z24S8           0xC
#define DEC400_CFMT_Z24             0xD
#define DEC400_CFMT_Z16             0xE
#define DEC400_CFMT_A2R10G10B10     0xF
#define DEC400_CFMT_BAYER           0x10
#define DEC400_CFMT_SIGNED_BAYER    0x11

struct dcss_dec400d_priv {
	struct dcss_soc *dcss;
	void __iomem *dec400d_reg;
	uint32_t dec400d_reg_base;
	uint64_t modifier[4];
	uint32_t pixel_format;
	uint32_t ctx_id;
	bool bypass;		/* bypass or decompress */
};

static void dcss_dec400d_write(struct dcss_dec400d_priv *dec400d,
			       uint32_t value,
			       uint32_t offset)
{
#if !USE_CTXLD
	dcss_writel(value, dec400d->dec400d_reg + offset);
#else
	dcss_ctxld_write(dec400d->dcss, dec400d->ctx_id,
			 value, dec400d->dec400d_reg_base + offset);
#endif
}

int dcss_dec400d_init(struct dcss_soc *dcss, unsigned long dec400d_base)
{
	struct dcss_dec400d_priv *dec400d;

	dec400d = devm_kzalloc(dcss->dev, sizeof(*dec400d), GFP_KERNEL);
	if (!dec400d)
		return -ENOMEM;

	dcss->dec400d_priv = dec400d;
	dec400d->dcss = dcss;

	dec400d->dec400d_reg = devm_ioremap(dcss->dev, dec400d_base, SZ_4K);
	if (!dec400d->dec400d_reg) {
		dev_err(dcss->dev, "dec400d: unable to remap dec400d base\n");
		return -ENOMEM;
	}

	dec400d->dec400d_reg_base = dec400d_base;

#if USE_CTXLD
	dec400d->ctx_id = CTX_SB_HP;
#endif

	return 0;
}

void dcss_dec400d_exit(struct dcss_soc *dcss)
{
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	if (!IS_ERR(dec400d)) {
		devm_kfree(dcss->dev, dec400d);
		dcss->dec400d_priv = NULL;
	}
}

void dcss_dec400d_set_format_mod(struct dcss_soc *dcss,
				 uint32_t fourcc,
				 uint32_t mod_idx,
				 uint64_t modifier)
{
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	if (mod_idx > 3) {
		WARN_ON(1);
		return;
	}

	if (mod_idx == 0)
		dec400d->pixel_format = fourcc;

	dec400d->modifier[mod_idx] = modifier;
}
EXPORT_SYMBOL(dcss_dec400d_set_format_mod);

void dcss_dec400d_bypass(struct dcss_soc *dcss)
{
	uint32_t control;
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	dcss_dec400d_read_config(dcss, 0, false, 0);

	control = dcss_readl(dec400d->dec400d_reg + DEC400D_CONTROL);
	pr_debug("%s: dec400d control = %#x\n", __func__, control);

	control |= 0x1 << DISABLE_COMPRESSION_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);

	/* Trigger shadow registers */
	control |= 0x1 << SHADOW_TRIGGER_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);

	dec400d->bypass = true;
}
EXPORT_SYMBOL(dcss_dec400d_bypass);

void dcss_dec400d_shadow_trig(struct dcss_soc *dcss)
{
	uint32_t control;
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	/* do nothing */
	if (dec400d->bypass == true)
		return;

	control = dcss_readl(dec400d->dec400d_reg + DEC400D_CONTROL);

	/* Trigger shadow registers */
	control |= 0x1 << SHADOW_TRIGGER_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);
}
EXPORT_SYMBOL(dcss_dec400d_shadow_trig);

void dcss_dec400d_addr_set(struct dcss_soc *dcss,
			   uint32_t baddr,
			   uint32_t caddr)
{
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	/* set frame buffer base addr */
	dcss_dec400d_write(dec400d, baddr, DEC400D_READBUFFERBASE0);

	/* set tile status cache addr */
	dcss_dec400d_write(dec400d, caddr, DEC400D_READCACHEBASE0);

	dec400d->bypass = false;
}
EXPORT_SYMBOL(dcss_dec400d_addr_set);

void dcss_dec400d_read_config(struct dcss_soc *dcss,
			      uint32_t read_id,
			      bool compress_en,
			      uint32_t compress_format)
{
	uint32_t cformat = 0;
	uint32_t read_config = 0x0;
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	/* TODO: using 'read_id' 0 by default */
	if (read_id) {
		WARN_ON(1);
		return;
	}

	if (compress_en == false)
		goto config;

	switch (compress_format) {
	    case _VIV_CFMT_ARGB8:
	        cformat = DEC400_CFMT_ARGB8;
	        break;
	    case _VIV_CFMT_XRGB8:
	        cformat = DEC400_CFMT_XRGB8;
	        break;
	    case _VIV_CFMT_AYUV:
	        cformat = DEC400_CFMT_AYUV;
	        break;
	    case _VIV_CFMT_UYVY:
	        cformat = DEC400_CFMT_UYVY;
	        break;
	    case _VIV_CFMT_YUY2:
	        cformat = DEC400_CFMT_YUY2;
	        break;
	    case _VIV_CFMT_YUV_ONLY:
	        cformat = DEC400_CFMT_YUV_ONLY;
	        break;
	    case _VIV_CFMT_UV_MIX:
	        cformat = DEC400_CFMT_UV_MIX;
	        break;
	    case _VIV_CFMT_ARGB4:
	        cformat = DEC400_CFMT_ARGB4;
	        break;
	    case _VIV_CFMT_XRGB4:
	        cformat = DEC400_CFMT_XRGB4;
	        break;
	    case _VIV_CFMT_A1R5G5B5:
	        cformat = DEC400_CFMT_A1R5G5B5;
	        break;
	    case _VIV_CFMT_X1R5G5B5:
	        cformat = DEC400_CFMT_X1R5G5B5;
	        break;
	    case _VIV_CFMT_R5G6B5:
	        cformat = DEC400_CFMT_R5G6B5;
	        break;
	    case _VIV_CFMT_Z24S8:
	        cformat = DEC400_CFMT_Z24S8;
	        break;
	    case _VIV_CFMT_Z24:
	        cformat = DEC400_CFMT_Z24;
	        break;
	    case _VIV_CFMT_Z16:
	        cformat = DEC400_CFMT_Z16;
	        break;
	    case _VIV_CFMT_A2R10G10B10:
	        cformat = DEC400_CFMT_A2R10G10B10;
	        break;
	    case _VIV_CFMT_BAYER:
	        cformat = DEC400_CFMT_BAYER;
	        break;
	    case _VIV_CFMT_SIGNED_BAYER:
	        cformat = DEC400_CFMT_SIGNED_BAYER;
	        break;
	default:
		/* TODO: not support yet */
		WARN_ON(1);
		return;
	}

	/* Dec compress format */
	read_config |= cformat << COMPRESSION_FORMAT_BIT;

	/* ALIGN32_BYTE */
	read_config |= 0x2 << COMPRESSION_ALIGN_MODE_BIT;

	/* TILE1_ALIGN */
	read_config |= 0x0 << TILE_ALIGN_MODE_BIT;

	/* TILE8x4 */
	read_config |= 0x3 << TILE_MODE_BIT;

	/* Compression Enable */
	read_config |= 0x1 << COMPRESSION_ENABLE_BIT;

config:
	dcss_dec400d_write(dec400d, read_config, DEC400D_READCONFIG(read_id));
}
EXPORT_SYMBOL(dcss_dec400d_read_config);

void dcss_dec400d_fast_clear_config(struct dcss_soc *dcss,
				    uint32_t fc_value,
				    bool enable)
{
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	dcss_dec400d_write(dec400d, fc_value, DEC400D_CLEAR);
}
EXPORT_SYMBOL(dcss_dec400d_fast_clear_config);

void dcss_dec400d_enable(struct dcss_soc *dcss)
{
	uint32_t control;
	struct dcss_dec400d_priv *dec400d = dcss->dec400d_priv;

	if (dec400d->bypass)
		return;

	control = dcss_readl(dec400d->dec400d_reg + DEC400D_CONTROL);

	/* enable compression */
	control &= ~(0x1 << DISABLE_COMPRESSION_BIT);
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);

	/* Trigger shadow registers */
	control |= 0x1 << SHADOW_TRIGGER_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);
}
EXPORT_SYMBOL(dcss_dec400d_enable);
