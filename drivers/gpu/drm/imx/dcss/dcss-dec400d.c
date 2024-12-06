// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <drm/drm_fourcc.h>

#include "dcss-dev.h"

/* DEC400D registers offsets */
#define DEC400D_READCONFIG_BASE		0x800
#define DEC400D_READCONFIG(i)		(DEC400D_READCONFIG_BASE + ((i) << 2))
#define   COMPRESSION_ENABLE_BIT	BIT(0)
#define   COMPRESSION_FORMAT_POS	3
#define   COMPRESSION_ALIGN_MODE_POS	16
#define   TILE_ALIGN_MODE_POS		22
#define   TILE_MODE_POS			25
#define DEC400D_READBUFFERBASE0		0x900
#define DEC400D_READCACHEBASE0		0x980
#define DEC400D_CONTROL			0xB00
#define DEC400D_CLEAR			0xB80
#define DEC400D_READBUFFERBASE0		0x900
#define DEC400D_READCACHEBASE0		0x980
#define DEC400D_CONTROL			0xB00
#define   DISABLE_COMPRESSION_BIT	BIT(1)
#define   SHADOW_TRIGGER_BIT		BIT(29)
#define DEC400_CFMT_ARGB8		0x0
#define DEC400_CFMT_XRGB8		0x1
#define DEC400_CFMT_AYUV		0x2
#define DEC400_CFMT_UYVY		0x3
#define DEC400_CFMT_YUY2		0x4
#define DEC400_CFMT_YUV_ONLY		0x5
#define DEC400_CFMT_UV_MIX		0x6
#define DEC400_CFMT_ARGB4		0x7
#define DEC400_CFMT_XRGB4		0x8
#define DEC400_CFMT_A1R5G5B5		0x9
#define DEC400_CFMT_X1R5G5B5		0xA
#define DEC400_CFMT_R5G6B5		0xB
#define DEC400_CFMT_Z24S8		0xC
#define DEC400_CFMT_Z24			0xD
#define DEC400_CFMT_Z16			0xE
#define DEC400_CFMT_A2R10G10B10		0xF
#define DEC400_CFMT_BAYER		0x10
#define DEC400_CFMT_SIGNED_BAYER	0x11

struct dcss_dec400d {
	struct device *dev;
	void __iomem *base_reg;
	u32 base_ofs;
	struct dcss_ctxld *ctxld;
	u32 ctx_id;
	bool bypass;		/* bypass or decompress */
};

static void dcss_dec400d_write(struct dcss_dec400d *dec400d,
			       u32 value,
			       u32 offset)
{
	dcss_ctxld_write(dec400d->ctxld, dec400d->ctx_id,
			 value, dec400d->base_ofs + offset);
}

int dcss_dec400d_init(struct dcss_dev *dcss, unsigned long dec400d_base)
{
	struct dcss_dec400d *dec400d;
	int ret;

	dec400d = kzalloc(sizeof(*dec400d), GFP_KERNEL);
	if (!dec400d)
		return -ENOMEM;

	dcss->dec400d = dec400d;
	dec400d->dev = dcss->dev;
	dec400d->ctxld = dcss->ctxld;

	dec400d->base_reg = ioremap(dec400d_base, SZ_4K);
	if (!dec400d->base_reg) {
		dev_err(dcss->dev, "dec400d: unable to remap dec400d base\n");
		ret = -ENOMEM;
		goto free_mem;
	}

	dec400d->base_ofs = dec400d_base;

	dec400d->ctx_id = CTX_SB_HP;

	return 0;

free_mem:
	kfree(dcss->dec400d);
	return ret;
}

void dcss_dec400d_exit(struct dcss_dec400d *dec400d)
{
	if (dec400d->base_reg)
		iounmap(dec400d->base_reg);

	kfree(dec400d);
}

void dcss_dec400d_read_config(struct dcss_dec400d *dec400d,
			      u32 read_id,
			      bool compress_en,
			      u32 compress_format)
{
	u32 cformat = 0;
	u32 read_config = 0x0;

	/* TODO: using 'read_id' 0 by default */
	if (read_id) {
		WARN_ON(1);
		return;
	}

	if (!compress_en)
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
	read_config |= cformat << COMPRESSION_FORMAT_POS;

	/* ALIGN32_BYTE */
	read_config |= 0x2 << COMPRESSION_ALIGN_MODE_POS;

	/* TILE1_ALIGN */
	read_config |= 0x0 << TILE_ALIGN_MODE_POS;

	/* TILE8x4 */
	read_config |= 0x3 << TILE_MODE_POS;

	/* Compression Enable */
	read_config |= COMPRESSION_ENABLE_BIT;

config:
	dcss_dec400d_write(dec400d, read_config, DEC400D_READCONFIG(read_id));
}

void dcss_dec400d_bypass(struct dcss_dec400d *dec400d)
{
	u32 control;

	dcss_dec400d_read_config(dec400d, 0, false, 0);

	control = dcss_readl(dec400d->base_reg + DEC400D_CONTROL);
	dev_dbg(dec400d->dev, "%s: dec400d control = %#x\n", __func__, control);

	control |= DISABLE_COMPRESSION_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);

	/* Trigger shadow registers */
	control |= SHADOW_TRIGGER_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);

	dec400d->bypass = true;
}

void dcss_dec400d_shadow_trig(struct dcss_dec400d *dec400d)
{
	u32 control;

	/* do nothing */
	if (dec400d->bypass)
		return;

	control = dcss_readl(dec400d->base_reg + DEC400D_CONTROL);

	/* Trigger shadow registers */
	control |= SHADOW_TRIGGER_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);
}

void dcss_dec400d_addr_set(struct dcss_dec400d *dec400d, u32 baddr, u32 caddr)
{
	/* set frame buffer base addr */
	dcss_dec400d_write(dec400d, baddr, DEC400D_READBUFFERBASE0);

	/* set tile status cache addr */
	dcss_dec400d_write(dec400d, caddr, DEC400D_READCACHEBASE0);

	dec400d->bypass = false;
}

void dcss_dec400d_fast_clear_config(struct dcss_dec400d *dec400d,
				    u32 fc_value,
				    bool enable)
{
	dcss_dec400d_write(dec400d, fc_value, DEC400D_CLEAR);
}

void dcss_dec400d_enable(struct dcss_dec400d *dec400d)
{
	u32 control;

	if (dec400d->bypass)
		return;

	control = dcss_readl(dec400d->base_reg + DEC400D_CONTROL);

	/* enable compression */
	control &= ~(DISABLE_COMPRESSION_BIT);
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);

	/* Trigger shadow registers */
	control |= SHADOW_TRIGGER_BIT;
	dcss_dec400d_write(dec400d, control, DEC400D_CONTROL);
}
