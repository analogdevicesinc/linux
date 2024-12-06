// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/bsearch.h>
#include <linux/io.h>
#include <drm/drm_fourcc.h>

#include "dcss-dev.h"
#include "dcss-hdr10-tables.h"

#define DCSS_HDR10_A0_LUT		0x0000
#define DCSS_HDR10_A1_LUT		0x1000
#define DCSS_HDR10_A2_LUT		0x2000
/* one CSCA and CSCB for each channel(pipe) */
#define DCSS_HDR10_CSCA_BASE		0x3000
#define DCSS_HDR10_CSCB_BASE		0x3800

/* one CSCO for all channels(pipes) */
#define DCSS_HDR10_CSCO_BASE		0x3000

#define DCSS_HDR10_LUT_CONTROL		(DCSS_HDR10_CSCA_BASE + 0x80)
#define   LUT_ENABLE			BIT(0)
#define   LUT_EN_FOR_ALL_PELS		BIT(1)
#define   LUT_BYPASS			BIT(15)
#define DCSS_HDR10_FL2FX		(DCSS_HDR10_CSCB_BASE + 0x74)
#define DCSS_HDR10_LTNL			(DCSS_HDR10_CSCO_BASE + 0x74)
#define   LTNL_PASS_THRU		BIT(0)
#define   FIX2FLT_DISABLE		BIT(1)
#define   LTNL_EN_FOR_ALL_PELS		BIT(2)
#define   FIX2FLT_EN_FOR_ALL_PELS	BIT(3)

/* following offsets are relative to CSC(A|B|O)_BASE */
#define DCSS_HDR10_CSC_CONTROL		0x00
#define   CSC_EN			BIT(0)
#define   CSC_ALL_PIX_EN		BIT(1)
#define   CSC_BYPASS			BIT(15)
#define DCSS_HDR10_CSC_H00		0x04
#define DCSS_HDR10_CSC_H10		0x08
#define DCSS_HDR10_CSC_H20		0x0C
#define DCSS_HDR10_CSC_H01		0x10
#define DCSS_HDR10_CSC_H11		0x14
#define DCSS_HDR10_CSC_H21		0x18
#define DCSS_HDR10_CSC_H02		0x1C
#define DCSS_HDR10_CSC_H12		0x20
#define DCSS_HDR10_CSC_H22		0x24
#define   H_COEF_MASK			GENMASK(15, 0)
#define DCSS_HDR10_CSC_IO0		0x28
#define DCSS_HDR10_CSC_IO1		0x2C
#define DCSS_HDR10_CSC_IO2		0x30
#define   PRE_OFFSET_MASK		GENMASK(9, 0)
#define DCSS_HDR10_CSC_IO_MIN0		0x34
#define DCSS_HDR10_CSC_IO_MIN1		0x38
#define DCSS_HDR10_CSC_IO_MIN2		0x3C
#define DCSS_HDR10_CSC_IO_MAX0		0x40
#define DCSS_HDR10_CSC_IO_MAX1		0x44
#define DCSS_HDR10_CSC_IO_MAX2		0x48
#define   IO_CLIP_MASK			GENMASK(9, 0)
#define DCSS_HDR10_CSC_NORM		0x4C
#define   NORM_MASK			GENMASK(4, 0)
#define DCSS_HDR10_CSC_OO0		0x50
#define DCSS_HDR10_CSC_OO1		0x54
#define DCSS_HDR10_CSC_OO2		0x58
#define   POST_OFFSET_MASK		GENMASK(27, 0)
#define DCSS_HDR10_CSC_OMIN0		0x5C
#define DCSS_HDR10_CSC_OMIN1		0x60
#define DCSS_HDR10_CSC_OMIN2		0x64
#define DCSS_HDR10_CSC_OMAX0		0x68
#define DCSS_HDR10_CSC_OMAX1		0x6C
#define DCSS_HDR10_CSC_OMAX2		0x70
#define   POST_CLIP_MASK		GENMASK(9, 0)

#define HDR10_IPIPE_LUT_MAX_ENTRIES	1024
#define HDR10_OPIPE_LUT_MAX_ENTRIES	1023
#define HDR10_CSC_MAX_REGS		29

#define OPIPE_CH_NO			3

/* Pipe config descriptor */

/* bits per component */
#define HDR10_BPC_POS			0
#define HDR10_BPC_MASK			GENMASK(1, 0)
/* colorspace */
#define HDR10_CS_POS			2
#define HDR10_CS_MASK			GENMASK(3, 2)
/* nonlinearity type */
#define HDR10_NL_POS			4
#define HDR10_NL_MASK			GENMASK(8, 4)
/* pixel range */
#define HDR10_PR_POS			9
#define HDR10_PR_MASK			GENMASK(10, 9)
/* gamut type */
#define HDR10_G_POS			11
#define HDR10_G_MASK			GENMASK(15, 11)

/* FW Table Type Descriptor */
#define HDR10_TT_LUT			BIT(0)
#define HDR10_TT_CSCA			BIT(1)
#define HDR10_TT_CSCB			BIT(2)
/* Pipe type */
#define HDR10_PT_OUTPUT			BIT(3)
/* Output pipe config descriptor */
#define HDR10_IPIPE_DESC_POS		4
#define HDR10_IPIPE_DESC_MASK		GENMASK(19, 4)
/* Input pipe config descriptor */
#define HDR10_OPIPE_DESC_POS		20
#define HDR10_OPIPE_DESC_MASK		GENMASK(35, 20)

/* config invalid */
#define HDR10_DESC_INVALID		BIT(63)

enum dcss_hdr10_csc {
	HDR10_CSCA,
	HDR10_CSCB,
};

struct dcss_hdr10_ch {
	struct dcss_hdr10 *hdr10;
	void __iomem *base_reg;
	u32 base_ofs;

	u64 old_cfg_desc;

	u32 id;
};

struct dcss_hdr10 {
	struct device *dev;
	struct dcss_ctxld *ctxld;

	u32 ctx_id;

	struct dcss_hdr10_ch ch[4]; /* 4th channel is, actually, OPIPE */
};

static void dcss_hdr10_write(struct dcss_hdr10_ch *ch, u32 val, u32 ofs)
{
	struct dcss_hdr10 *hdr10 = ch->hdr10;

	dcss_ctxld_write(hdr10->ctxld, hdr10->ctx_id, val, ch->base_ofs + ofs);
}

static void dcss_hdr10_csc_fill(struct dcss_hdr10_ch *ch,
				enum dcss_hdr10_csc csc_to_use,
				const u32 *map)
{
	int i;
	u32 csc_base_ofs[] = {
		DCSS_HDR10_CSCA_BASE + DCSS_HDR10_CSC_CONTROL,
		DCSS_HDR10_CSCB_BASE + DCSS_HDR10_CSC_CONTROL,
	};

	for (i = 0; i < HDR10_CSC_MAX_REGS; i++) {
		u32 reg_ofs = csc_base_ofs[csc_to_use] + i * sizeof(u32);

		dcss_hdr10_write(ch, map[i], reg_ofs);
	}
}

static void dcss_hdr10_lut_fill(struct dcss_hdr10_ch *ch, const u16 *map)
{
	int i, comp;
	u32 lut_base_ofs, ctrl_ofs, lut_entries;

	if (ch->id == OPIPE_CH_NO) {
		ctrl_ofs = DCSS_HDR10_LTNL;
		lut_entries = HDR10_OPIPE_LUT_MAX_ENTRIES;
	} else {
		ctrl_ofs = DCSS_HDR10_LUT_CONTROL;
		lut_entries = HDR10_IPIPE_LUT_MAX_ENTRIES;
	}

	if (ch->id != OPIPE_CH_NO)
		dcss_hdr10_write(ch, *map++, ctrl_ofs);

	for (comp = 0; comp < 3; comp++) {
		lut_base_ofs = DCSS_HDR10_A0_LUT + comp * 0x1000;

		if (ch->id == OPIPE_CH_NO) {
			dcss_hdr10_write(ch, map[0], lut_base_ofs);
			lut_base_ofs += 4;
		}

		for (i = 0; i < lut_entries; i++) {
			u32 reg_ofs = lut_base_ofs + i * sizeof(u32);

			dcss_hdr10_write(ch, map[i], reg_ofs);
		}
	}

	map += lut_entries;

	if (ch->id != OPIPE_CH_NO)
		dcss_hdr10_write(ch, *map, DCSS_HDR10_FL2FX);
	else
		dcss_hdr10_write(ch, *map, ctrl_ofs);
}

static int dcss_hdr10_ch_init_all(struct dcss_hdr10 *hdr10,
				  unsigned long hdr10_base)
{
	struct dcss_hdr10_ch *ch;
	int i;

	for (i = 0; i < 4; i++) {
		ch = &hdr10->ch[i];

		ch->base_ofs = hdr10_base + i * 0x4000;

		ch->base_reg = ioremap(ch->base_ofs, SZ_16K);
		if (!ch->base_reg) {
			dev_err(hdr10->dev, "hdr10: unable to remap ch base\n");
			return -ENOMEM;
		}

		ch->old_cfg_desc = HDR10_DESC_INVALID;

		ch->id = i;
		ch->hdr10 = hdr10;
	}

	return 0;
}

static int dcss_hdr10_id_compare(const void *a, const void *b)
{
	const u32 id = *(const u32 *)a;
	const u32 tbl_id = *(const u32 *)b;

	if (id == tbl_id)
		return 0;

	if (id > tbl_id)
		return 1;

	return -1;
}

static struct dcss_pipe_cfg *dcss_hdr10_get_pipe_cfg(struct dcss_hdr10 *hdr10,
						     u32 desc)
{
	struct dcss_pipe_cfg *res;

	res = bsearch(&desc, dcss_cfg_table, ARRAY_SIZE(dcss_cfg_table),
		      sizeof(dcss_cfg_table[0]), dcss_hdr10_id_compare);
	if (!res)
		dev_dbg(hdr10->dev,
			"hdr10 cfg table doesn't support desc(0x%08x)\n", desc);

	return res;
}

static int dcss_hdr10_get_tbls(struct dcss_hdr10 *hdr10, u32 desc,
			       const u16 **ilut, const u32 **csca,
			       const u32 **cscb, const u16 **olut,
			       const u32 **csco)
{
	struct dcss_pipe_cfg *pipe_cfg;

	pipe_cfg = dcss_hdr10_get_pipe_cfg(hdr10, desc);
	if (!pipe_cfg) {
		dev_err(hdr10->dev, "failed to get hdr10 pipe configurations\n");
		return -EINVAL;
	}

	dev_dbg(hdr10->dev, "found tbl_id = 0x%08x: (%d, %d, %d, %d, %d)",
			pipe_cfg->id, pipe_cfg->idx[0], pipe_cfg->idx[1],
			pipe_cfg->idx[2], pipe_cfg->idx[3], pipe_cfg->idx[4]);

	*csca = dcss_cscas[pipe_cfg->idx[0]];
	*ilut = dcss_iluts[pipe_cfg->idx[1]];
	*cscb = dcss_cscbs[pipe_cfg->idx[2]];
	*olut = dcss_oluts[pipe_cfg->idx[3]];
	*csco = dcss_cscos[pipe_cfg->idx[4]];

	return 0;
}

static void dcss_hdr10_write_pipe_tbls(struct dcss_hdr10_ch *ch,
				       const u16 *lut, const u32 *csca,
				       const u32 *cscb)
{
	if (csca)
		dcss_hdr10_csc_fill(ch, HDR10_CSCA, csca);

	if (ch->id != OPIPE_CH_NO && cscb)
		dcss_hdr10_csc_fill(ch, HDR10_CSCB, cscb);

	if (lut)
		dcss_hdr10_lut_fill(ch, lut);
}

int dcss_hdr10_init(struct dcss_dev *dcss, unsigned long hdr10_base)
{
	int ret;
	struct dcss_hdr10 *hdr10;

	hdr10 = kzalloc(sizeof(*hdr10), GFP_KERNEL);
	if (!hdr10)
		return -ENOMEM;

	dcss->hdr10 = hdr10;
	hdr10->dev = dcss->dev;
	hdr10->ctx_id = CTX_SB_HP;
	hdr10->ctxld = dcss->ctxld;

	ret = dcss_hdr10_ch_init_all(hdr10, hdr10_base);
	if (ret) {
		int i;

		for (i = 0; i < 4; i++) {
			if (hdr10->ch[i].base_reg)
				iounmap(hdr10->ch[i].base_reg);
		}

		goto cleanup;
	}

	return 0;

cleanup:
	kfree(hdr10);

	return ret;
}

void dcss_hdr10_exit(struct dcss_hdr10 *hdr10)
{
	int i;

	for (i = 0; i < 4; i++) {
		if (hdr10->ch[i].base_reg)
			iounmap(hdr10->ch[i].base_reg);
	}

	kfree(hdr10);
}

static u32 dcss_hdr10_pipe_desc(struct dcss_hdr10_pipe_cfg *pipe_cfg)
{
	u32 desc;

	desc = 2 << HDR10_BPC_POS;
	desc |= pipe_cfg->is_yuv ? 2 << HDR10_CS_POS : 1 << HDR10_CS_POS;
	desc |= ((1 << pipe_cfg->nl) << HDR10_NL_POS) & HDR10_NL_MASK;
	desc |= ((1 << pipe_cfg->pr) << HDR10_PR_POS) & HDR10_PR_MASK;
	desc |= ((1 << pipe_cfg->g) << HDR10_G_POS) & HDR10_G_MASK;

	return desc;
}

static u64 dcss_hdr10_get_desc(struct dcss_hdr10_pipe_cfg *ipipe_cfg,
			       struct dcss_hdr10_pipe_cfg *opipe_cfg)
{
	u32 ipipe_desc, opipe_desc;

	ipipe_desc = dcss_hdr10_pipe_desc(ipipe_cfg);
	opipe_desc = dcss_hdr10_pipe_desc(opipe_cfg);

	return (ipipe_desc & 0xFFFF) | ((opipe_desc & 0xFFFF) << 16);
}

bool dcss_hdr10_pipe_cfg_is_supported(struct dcss_hdr10 *hdr10,
				      struct dcss_hdr10_pipe_cfg *ipipe_cfg,
				      struct dcss_hdr10_pipe_cfg *opipe_cfg)
{
	u32 desc = dcss_hdr10_get_desc(ipipe_cfg, opipe_cfg);

	return !!dcss_hdr10_get_pipe_cfg(hdr10, desc);
}

void dcss_hdr10_setup(struct dcss_hdr10 *hdr10, int ch_num,
		      struct dcss_hdr10_pipe_cfg *ipipe_cfg,
		      struct dcss_hdr10_pipe_cfg *opipe_cfg)
{
	const u16 *ilut, *olut;
	const u32 *csca, *cscb, *csco;
	u32 desc = dcss_hdr10_get_desc(ipipe_cfg, opipe_cfg);

	if (hdr10->ch[ch_num].old_cfg_desc == desc)
		return;

	if (dcss_hdr10_get_tbls(hdr10, desc, &ilut, &csca, &cscb, &olut, &csco))
		return;

	dcss_hdr10_write_pipe_tbls(&hdr10->ch[ch_num], ilut, csca, cscb);

	hdr10->ch[ch_num].old_cfg_desc = desc;

	dcss_hdr10_write_pipe_tbls(&hdr10->ch[OPIPE_CH_NO], olut, csco, NULL);
}
