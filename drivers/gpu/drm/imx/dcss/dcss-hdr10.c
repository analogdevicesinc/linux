// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019 NXP.
 */

#include <linux/device.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/seq_file.h>
#include <linux/firmware.h>
#include <drm/drm_fourcc.h>

#include "dcss-dev.h"

#define USE_TBL_HEADER

#ifdef USE_TBL_HEADER
#include "dcss-hdr10-tables.h"
#endif

#define USE_CTXLD

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

struct dcss_hdr10_tbl_node {
	struct list_head node;

	u64 tbl_descriptor;
	u32 *tbl_data;
};

struct dcss_hdr10_opipe_tbls {
	struct list_head lut;
	struct list_head csc;
};

struct dcss_hdr10_ipipe_tbls {
	struct list_head lut;
	struct list_head csca;
	struct list_head cscb;
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

	struct dcss_hdr10_ipipe_tbls *ipipe_tbls;
	struct dcss_hdr10_opipe_tbls *opipe_tbls;

	u8 *fw_data;
	u32 fw_size;
};

static void dcss_hdr10_write(struct dcss_hdr10_ch *ch, u32 val, u32 ofs)
{
	struct dcss_hdr10 *hdr10 = ch->hdr10;

	dcss_ctxld_write(hdr10->ctxld, hdr10->ctx_id, val, ch->base_ofs + ofs);
}

static void dcss_hdr10_csc_fill(struct dcss_hdr10_ch *ch,
				enum dcss_hdr10_csc csc_to_use,
				u32 *map)
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

static void dcss_hdr10_lut_fill(struct dcss_hdr10_ch *ch, u32 *map)
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

static u32 *dcss_hdr10_find_tbl(u64 desc, struct list_head *head)
{
	struct list_head *node;
	struct dcss_hdr10_tbl_node *tbl_node;

	list_for_each(node, head) {
		tbl_node = container_of(node, struct dcss_hdr10_tbl_node, node);

		if ((tbl_node->tbl_descriptor & desc) == desc)
			return tbl_node->tbl_data;
	}

	return NULL;
}

static int dcss_hdr10_get_tbls(struct dcss_hdr10 *hdr10, bool input,
			       u64 desc, u32 **lut, u32 **csca, u32 **cscb)
{
	struct list_head *lut_list, *csca_list, *cscb_list;

	lut_list = input ? &hdr10->ipipe_tbls->lut : &hdr10->opipe_tbls->lut;
	csca_list = input ? &hdr10->ipipe_tbls->csca : &hdr10->opipe_tbls->csc;
	cscb_list = input ? &hdr10->ipipe_tbls->cscb : NULL;

	*lut = dcss_hdr10_find_tbl(desc, lut_list);
	*csca = dcss_hdr10_find_tbl(desc, csca_list);

	*cscb = NULL;
	if (cscb_list)
		*cscb = dcss_hdr10_find_tbl(desc, cscb_list);

	return 0;
}

static void dcss_hdr10_write_pipe_tbls(struct dcss_hdr10_ch *ch,
				       u32 *lut, u32 *csca, u32 *cscb)
{
	if (csca)
		dcss_hdr10_csc_fill(ch, HDR10_CSCA, csca);

	if (ch->id != OPIPE_CH_NO && cscb)
		dcss_hdr10_csc_fill(ch, HDR10_CSCB, cscb);

	if (lut)
		dcss_hdr10_lut_fill(ch, lut);
}

static int dcss_hdr10_tbl_add(struct dcss_hdr10 *hdr10, u64 desc, u32 sz,
			      u32 *data)
{
	struct dcss_hdr10_tbl_node *node;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	/* we don't need to store the table type and pipe type */
	node->tbl_descriptor = desc >> 4;
	node->tbl_data = data;

	if (!(desc & HDR10_PT_OUTPUT)) {
		if (desc & HDR10_TT_LUT)
			list_add(&node->node, &hdr10->ipipe_tbls->lut);
		else if (desc & HDR10_TT_CSCA)
			list_add(&node->node, &hdr10->ipipe_tbls->csca);
		else if (desc & HDR10_TT_CSCB)
			list_add(&node->node, &hdr10->ipipe_tbls->cscb);

		return 0;
	}

	if (desc & HDR10_TT_LUT)
		list_add(&node->node, &hdr10->opipe_tbls->lut);
	else if (desc & HDR10_TT_CSCA)
		list_add(&node->node, &hdr10->opipe_tbls->csc);

	return 0;
}

static int dcss_hdr10_parse_fw_data(struct dcss_hdr10 *hdr10)
{
	u32 *data = (u32 *)hdr10->fw_data;
	u32 remaining = hdr10->fw_size / sizeof(u32);
	u64 tbl_desc;
	u32 tbl_size;
	int ret;

	while (remaining) {
		tbl_desc = *((u64 *)data);
		data += 2;
		tbl_size = *data++;

		ret = dcss_hdr10_tbl_add(hdr10, tbl_desc, tbl_size, data);
		if (ret)
			return ret;

		data += tbl_size;
		remaining -= tbl_size + 3;
	}

	return 0;
}

static void dcss_hdr10_cleanup_tbls(struct dcss_hdr10 *hdr10)
{
	int i;
	struct dcss_hdr10_tbl_node *tbl_node, *next;
	struct list_head *tbls[] = {
		&hdr10->ipipe_tbls->lut,
		&hdr10->ipipe_tbls->csca,
		&hdr10->ipipe_tbls->cscb,
		&hdr10->opipe_tbls->lut,
		&hdr10->opipe_tbls->csc,
	};

	for (i = 0; i < 5; i++) {
		list_for_each_entry_safe(tbl_node, next, tbls[i], node) {
			list_del(&tbl_node->node);
			kfree(tbl_node);
		}
	}

	kfree(hdr10->opipe_tbls);
	kfree(hdr10->ipipe_tbls);
}

#ifndef USE_TBL_HEADER
static void dcss_hdr10_fw_handler(const struct firmware *fw, void *context)
{
	struct dcss_hdr10 *hdr10 = context;
	int i;

	if (!fw) {
		dev_err(hdr10->dev, "hdr10: DCSS FW load failed.\n");
		return;
	}

	/* we need to keep the tables for the entire life of the driver */
	hdr10->fw_data = kzalloc(fw->size, GFP_KERNEL);
	if (!hdr10->fw_data)
		return;

	memcpy(hdr10->fw_data, fw->data, fw->size);
	hdr10->fw_size = fw->size;

	release_firmware(fw);

	if (dcss_hdr10_parse_fw_data(hdr10)) {
		dcss_hdr10_cleanup_tbls(hdr10);
		return;
	}

	for (i = 0; i < 4; i++) {
		u32 *lut, *csca, *cscb;
		struct dcss_hdr10_ch *ch = &hdr10->ch[i];
		bool is_input_pipe = i != OPIPE_CH_NO ? true : false;

		if (ch->old_cfg_desc != HDR10_DESC_INVALID) {
			dcss_hdr10_get_tbls(hdr10, is_input_pipe,
					    ch->old_cfg_desc, &lut,
					    &csca, &cscb);
			dcss_hdr10_write_pipe_tbls(ch, lut, csca, cscb);
		}
	}

	dev_info(hdr10->dev, "hdr10: DCSS FW loaded successfully\n");
}
#endif

static int dcss_hdr10_tbls_init(struct dcss_hdr10 *hdr10)
{
	hdr10->ipipe_tbls = kzalloc(sizeof(*hdr10->ipipe_tbls), GFP_KERNEL);
	if (!hdr10->ipipe_tbls)
		return -ENOMEM;

	INIT_LIST_HEAD(&hdr10->ipipe_tbls->lut);
	INIT_LIST_HEAD(&hdr10->ipipe_tbls->csca);
	INIT_LIST_HEAD(&hdr10->ipipe_tbls->cscb);

	hdr10->opipe_tbls = kzalloc(sizeof(*hdr10->opipe_tbls), GFP_KERNEL);
	if (!hdr10->opipe_tbls) {
		kfree(hdr10->ipipe_tbls);
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&hdr10->opipe_tbls->lut);
	INIT_LIST_HEAD(&hdr10->opipe_tbls->csc);

	return 0;
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

	ret = dcss_hdr10_tbls_init(hdr10);
	if (ret < 0) {
		dev_err(dcss->dev, "hdr10: Cannot init table lists.\n");
		goto cleanup;
	}

#ifndef USE_TBL_HEADER
	ret = request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG, "dcss.fw",
				      dcss->dev, GFP_KERNEL, hdr10,
				      dcss_hdr10_fw_handler);
	if (ret < 0) {
		dev_err(dcss->dev, "hdr10: Cannot async load DCSS FW.\n");
		goto cleanup_tbls;
	}
#else
	hdr10->fw_data = (u8 *)dcss_hdr10_tables;
	hdr10->fw_size = sizeof(dcss_hdr10_tables);

	ret = dcss_hdr10_parse_fw_data(hdr10);
	if (ret)
		goto cleanup_tbls;
#endif

	ret = dcss_hdr10_ch_init_all(hdr10, hdr10_base);
	if (ret) {
		int i;

		for (i = 0; i < 4; i++) {
			if (hdr10->ch[i].base_reg)
				iounmap(hdr10->ch[i].base_reg);
		}

		goto cleanup_tbls;
	}

	return 0;

cleanup_tbls:
	dcss_hdr10_cleanup_tbls(hdr10);

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

	dcss_hdr10_cleanup_tbls(hdr10);

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

static void dcss_hdr10_pipe_setup(struct dcss_hdr10_ch *ch, u64 desc)
{
	bool pipe_cfg_chgd;
	u32 *csca, *cscb, *lut;

	pipe_cfg_chgd = ch->old_cfg_desc != desc;

	if (!pipe_cfg_chgd)
		return;

	dcss_hdr10_get_tbls(ch->hdr10, ch->id != OPIPE_CH_NO,
			    desc, &lut, &csca, &cscb);
	dcss_hdr10_write_pipe_tbls(ch, lut, csca, cscb);

	ch->old_cfg_desc = desc;
}

void dcss_hdr10_setup(struct dcss_hdr10 *hdr10, int ch_num,
		      struct dcss_hdr10_pipe_cfg *ipipe_cfg,
		      struct dcss_hdr10_pipe_cfg *opipe_cfg)
{
	u64 desc = dcss_hdr10_get_desc(ipipe_cfg, opipe_cfg);

	dcss_hdr10_pipe_setup(&hdr10->ch[ch_num], desc);

	/*
	 * Input pipe configuration doesn't matter for configuring the output
	 * pipe. So, will just mask off the input part of the descriptor.
	 */
	dcss_hdr10_pipe_setup(&hdr10->ch[OPIPE_CH_NO], desc & ~0xffff);
}
