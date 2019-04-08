/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020 NXP
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
#include <linux/clk.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <video/dpu.h>
#include <video/imx8-pc.h>
#include <video/imx8-prefetch.h>
#include "dpu-prv.h"

#define IMX_DPU_BLITENG_NAME "imx-drm-dpu-bliteng"

static bool display_plane_video_proc = true;
module_param(display_plane_video_proc, bool, 0444);
MODULE_PARM_DESC(display_plane_video_proc,
		 "Enable video processing for display [default=true]");

#define DPU_CM_REG_DEFINE1(name1, name2)		\
static inline u32 name1(const struct cm_reg_ofs *ofs)	\
{							\
	return ofs->name2;				\
}

#define DPU_CM_REG_DEFINE2(name1, name2)		\
static inline u32 name1(const struct cm_reg_ofs *ofs,	\
			unsigned int n)			\
{							\
	return ofs->name2 + (4 * n);			\
}

DPU_CM_REG_DEFINE1(LOCKUNLOCK, lockunlock);
DPU_CM_REG_DEFINE1(LOCKSTATUS, lockstatus);
DPU_CM_REG_DEFINE2(USERINTERRUPTMASK, userinterruptmask);
DPU_CM_REG_DEFINE2(INTERRUPTENABLE, interruptenable);
DPU_CM_REG_DEFINE2(INTERRUPTPRESET, interruptpreset);
DPU_CM_REG_DEFINE2(INTERRUPTCLEAR, interruptclear);
DPU_CM_REG_DEFINE2(INTERRUPTSTATUS, interruptstatus);
DPU_CM_REG_DEFINE2(USERINTERRUPTENABLE, userinterruptenable);
DPU_CM_REG_DEFINE2(USERINTERRUPTPRESET, userinterruptpreset);
DPU_CM_REG_DEFINE2(USERINTERRUPTCLEAR, userinterruptclear);
DPU_CM_REG_DEFINE2(USERINTERRUPTSTATUS, userinterruptstatus);
DPU_CM_REG_DEFINE1(GENERALPURPOSE, generalpurpose);

static inline u32 dpu_cm_read(struct dpu_soc *dpu, unsigned int offset)
{
	return readl(dpu->cm_reg + offset);
}

static inline void dpu_cm_write(struct dpu_soc *dpu,
				unsigned int offset, u32 value)
{
	writel(value, dpu->cm_reg + offset);
}

/* Constant Frame Unit */
static const unsigned long cf_ofss[] = {0x4400, 0x5400, 0x4c00, 0x5c00};
static const unsigned long cf_pec_ofss[] = {0x960, 0x9e0, 0x9a0, 0xa20};

/* Display Engine Configuration Unit */
static const unsigned long dec_ofss[] = {0xb400, 0xb420};

/* External Destination Unit */
static const unsigned long ed_ofss[] = {0x4800, 0x5800, 0x5000, 0x6000};
static const unsigned long ed_pec_ofss[] = {0x980, 0xa00, 0x9c0, 0xa40};

/* Fetch Decode Unit */
static const unsigned long fd_ofss[] = {0x6c00, 0x7800};
static const unsigned long fd_pec_ofss[] = {0xa80, 0xaa0};

/* Fetch ECO Unit */
static const unsigned long fe_ofss[] = {0x7400, 0x8000, 0x6800, 0x1c00};
static const unsigned long fe_pec_ofss[] = {0xa90, 0xab0, 0xa70, 0x850};

/* Frame Generator Unit */
static const unsigned long fg_ofss[] = {0xb800, 0xd400};

/* Fetch Layer Unit */
static const unsigned long fl_ofss[] = {0x8400};
static const unsigned long fl_pec_ofss[] = {0xac0};

/* Fetch Warp Unit */
static const unsigned long fw_ofss[] = {0x6400};
static const unsigned long fw_pec_ofss[] = {0xa60};

/* Horizontal Scaler Unit */
static const unsigned long hs_ofss[] = {0x9000, 0x9c00, 0x3000};
static const unsigned long hs_pec_ofss[] = {0xb00, 0xb60, 0x8c0};

/* Layer Blend Unit */
static const unsigned long lb_ofss[] = {0xa400, 0xa800, 0xac00, 0xb000};
static const unsigned long lb_pec_ofss[] = {0xba0, 0xbc0, 0xbe0, 0xc00};

/* Signature Unit */
static const unsigned long sig_ofss[] = {0xd000, 0xec00};

/* Store Unit */
static const unsigned long st_ofss[] = {0x4000};
static const unsigned long st_pec_ofss[] = {0x940};

/* Timing Controller Unit */
static const unsigned long tcon_ofss[] = {0xcc00, 0xe800};

/* Vertical Scaler Unit */
static const unsigned long vs_ofss[] = {0x9400, 0xa000, 0x3400};
static const unsigned long vs_pec_ofss[] = {0xb20, 0xb80, 0x8e0};

static const struct dpu_unit _cfs = {
	.name = "ConstFrame",
	.num = ARRAY_SIZE(cf_ids),
	.ids = cf_ids,
	.pec_ofss = cf_pec_ofss,
	.ofss = cf_ofss,
};

static const struct dpu_unit _decs = {
	.name = "DisEngCfg",
	.num = ARRAY_SIZE(dec_ids),
	.ids = dec_ids,
	.pec_ofss = NULL,
	.ofss = dec_ofss,
};

static const struct dpu_unit _eds = {
	.name = "ExtDst",
	.num = ARRAY_SIZE(ed_ids),
	.ids = ed_ids,
	.pec_ofss = ed_pec_ofss,
	.ofss = ed_ofss,
};

static const struct dpu_unit _fds = {
	.name = "FetchDecode",
	.num = ARRAY_SIZE(fd_ids),
	.ids = fd_ids,
	.pec_ofss = fd_pec_ofss,
	.ofss = fd_ofss,
	.dprc_ids = fd_dprc_ids,
};

static const struct dpu_unit _fes = {
	.name = "FetchECO",
	.num = ARRAY_SIZE(fe_ids),
	.ids = fe_ids,
	.pec_ofss = fe_pec_ofss,
	.ofss = fe_ofss,
};

static const struct dpu_unit _fgs = {
	.name = "FrameGen",
	.num = ARRAY_SIZE(fg_ids),
	.ids = fg_ids,
	.pec_ofss = NULL,
	.ofss = fg_ofss,
};

static const struct dpu_unit _fls = {
	.name = "FetchLayer",
	.num = ARRAY_SIZE(fl_ids),
	.ids = fl_ids,
	.pec_ofss = fl_pec_ofss,
	.ofss = fl_ofss,
	.dprc_ids = fl_dprc_ids,
};

static const struct dpu_unit _fws = {
	.name = "FetchWarp",
	.num = ARRAY_SIZE(fw_ids),
	.ids = fw_ids,
	.pec_ofss = fw_pec_ofss,
	.ofss = fw_ofss,
	.dprc_ids = fw_dprc_ids,
};

static const struct dpu_unit _hss = {
	.name = "HScaler",
	.num = ARRAY_SIZE(hs_ids),
	.ids = hs_ids,
	.pec_ofss = hs_pec_ofss,
	.ofss = hs_ofss,
};

static const struct dpu_unit _lbs = {
	.name = "LayerBlend",
	.num = ARRAY_SIZE(lb_ids),
	.ids = lb_ids,
	.pec_ofss = lb_pec_ofss,
	.ofss = lb_ofss,
};

static const struct dpu_unit _sigs = {
	.name = "Signature",
	.num = ARRAY_SIZE(sig_ids),
	.ids = sig_ids,
	.pec_ofss = NULL,
	.ofss = sig_ofss,
};

static const struct dpu_unit _sts = {
	.name = "Store",
	.num = ARRAY_SIZE(st_ids),
	.ids = st_ids,
	.pec_ofss = st_pec_ofss,
	.ofss = st_ofss,
};

static const struct dpu_unit _tcons = {
	.name = "TCon",
	.num = ARRAY_SIZE(tcon_ids),
	.ids = tcon_ids,
	.pec_ofss = NULL,
	.ofss = tcon_ofss,
};

static const struct dpu_unit _vss = {
	.name = "VScaler",
	.num = ARRAY_SIZE(vs_ids),
	.ids = vs_ids,
	.pec_ofss = vs_pec_ofss,
	.ofss = vs_ofss,
};

static const struct cm_reg_ofs _cm_reg_ofs = {
	.ipidentifier = 0,
	.lockunlock = 0x40,
	.lockstatus = 0x44,
	.userinterruptmask = 0x48,
	.interruptenable = 0x50,
	.interruptpreset = 0x58,
	.interruptclear = 0x60,
	.interruptstatus = 0x68,
	.userinterruptenable = 0x80,
	.userinterruptpreset = 0x88,
	.userinterruptclear = 0x90,
	.userinterruptstatus = 0x98,
	.generalpurpose = 0x100,
};

static const unsigned long unused_irq[] = {0x00000000, 0xfffe0008};

static const struct dpu_data dpu_data_qxp = {
	.cm_ofs = 0x0,
	.cfs = &_cfs,
	.decs = &_decs,
	.eds = &_eds,
	.fds = &_fds,
	.fes = &_fes,
	.fgs = &_fgs,
	.fls = &_fls,
	.fws = &_fws,
	.hss = &_hss,
	.lbs = &_lbs,
	.sigs = &_sigs,
	.sts = &_sts,
	.tcons = &_tcons,
	.vss = &_vss,
	.cm_reg_ofs = &_cm_reg_ofs,
	.unused_irq = unused_irq,
	.plane_src_mask = DPU_PLANE_SRC_FL0_ID | DPU_PLANE_SRC_FW2_ID |
			  DPU_PLANE_SRC_FD0_ID | DPU_PLANE_SRC_FD1_ID,
	.has_dual_ldb = true,
	.syncmode_min_prate = UINT_MAX,	/* pc is unused */
	.singlemode_max_width = UINT_MAX, 	/* pc is unused */
};

static const struct dpu_data dpu_data_qm = {
	.cm_ofs = 0x0,
	.cfs = &_cfs,
	.decs = &_decs,
	.eds = &_eds,
	.fds = &_fds,
	.fes = &_fes,
	.fgs = &_fgs,
	.fls = &_fls,
	.fws = &_fws,
	.hss = &_hss,
	.lbs = &_lbs,
	.sigs = &_sigs,
	.sts = &_sts,
	.tcons = &_tcons,
	.vss = &_vss,
	.cm_reg_ofs = &_cm_reg_ofs,
	.unused_irq = unused_irq,
	.plane_src_mask = DPU_PLANE_SRC_FL0_ID | DPU_PLANE_SRC_FW2_ID |
			  DPU_PLANE_SRC_FD0_ID | DPU_PLANE_SRC_FD1_ID,
	.has_dual_ldb = false,
	.syncmode_min_prate = 300000,
	.singlemode_max_width = 2560,
	.master_stream_id = 1,
};

static const struct of_device_id dpu_dt_ids[] = {
	{
		.compatible = "fsl,imx8qxp-dpu",
		.data = &dpu_data_qxp,
	}, {
		.compatible = "fsl,imx8qm-dpu",
		.data = &dpu_data_qm,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, dpu_dt_ids);

unsigned int dpu_get_syncmode_min_prate(struct dpu_soc *dpu)
{
	return dpu->data->syncmode_min_prate;
}
EXPORT_SYMBOL_GPL(dpu_get_syncmode_min_prate);

unsigned int dpu_get_singlemode_max_width(struct dpu_soc *dpu)
{
	return dpu->data->singlemode_max_width;
}
EXPORT_SYMBOL_GPL(dpu_get_singlemode_max_width);

unsigned int dpu_get_master_stream_id(struct dpu_soc *dpu)
{
	return dpu->data->master_stream_id;
}
EXPORT_SYMBOL_GPL(dpu_get_master_stream_id);

bool dpu_vproc_has_fetcheco_cap(u32 cap_mask)
{
	return !!(cap_mask & DPU_VPROC_CAP_FETCHECO);
}
EXPORT_SYMBOL_GPL(dpu_vproc_has_fetcheco_cap);

bool dpu_vproc_has_hscale_cap(u32 cap_mask)
{
	return !!(cap_mask & DPU_VPROC_CAP_HSCALE);
}
EXPORT_SYMBOL_GPL(dpu_vproc_has_hscale_cap);

bool dpu_vproc_has_vscale_cap(u32 cap_mask)
{
	return !!(cap_mask & DPU_VPROC_CAP_VSCALE);
}
EXPORT_SYMBOL_GPL(dpu_vproc_has_vscale_cap);

u32 dpu_vproc_get_fetcheco_cap(u32 cap_mask)
{
	return cap_mask & DPU_VPROC_CAP_FETCHECO;
}
EXPORT_SYMBOL_GPL(dpu_vproc_get_fetcheco_cap);

u32 dpu_vproc_get_hscale_cap(u32 cap_mask)
{
	return cap_mask & DPU_VPROC_CAP_HSCALE;
}
EXPORT_SYMBOL_GPL(dpu_vproc_get_hscale_cap);

u32 dpu_vproc_get_vscale_cap(u32 cap_mask)
{
	return cap_mask & DPU_VPROC_CAP_VSCALE;
}
EXPORT_SYMBOL_GPL(dpu_vproc_get_vscale_cap);

int dpu_format_horz_chroma_subsampling(u32 format)
{
	switch (format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
		return 2;
	default:
		return 1;
	}
}

int dpu_format_vert_chroma_subsampling(u32 format)
{
	switch (format) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		return 2;
	default:
		return 1;
	}
}

int dpu_format_num_planes(u32 format)
{
	switch (format) {
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
	case DRM_FORMAT_NV16:
	case DRM_FORMAT_NV61:
	case DRM_FORMAT_NV24:
	case DRM_FORMAT_NV42:
		return 2;
	default:
		return 1;
	}
}

int dpu_format_plane_width(int width, u32 format, int plane)
{
	if (plane >= dpu_format_num_planes(format))
		return 0;

	if (plane == 0)
		return width;

	return width / dpu_format_horz_chroma_subsampling(format);
}

int dpu_format_plane_height(int height, u32 format, int plane)
{
	if (plane >= dpu_format_num_planes(format))
		return 0;

	if (plane == 0)
		return height;

	return height / dpu_format_vert_chroma_subsampling(format);
}

static void dpu_detach_pm_domains(struct dpu_soc *dpu)
{
	if (dpu->pd_pll1_link && !IS_ERR(dpu->pd_pll1_link))
		device_link_del(dpu->pd_pll1_link);
	if (dpu->pd_pll1_dev && !IS_ERR(dpu->pd_pll1_dev))
		dev_pm_domain_detach(dpu->pd_pll1_dev, true);

	if (dpu->pd_pll0_link && !IS_ERR(dpu->pd_pll0_link))
		device_link_del(dpu->pd_pll0_link);
	if (dpu->pd_pll0_dev && !IS_ERR(dpu->pd_pll0_dev))
		dev_pm_domain_detach(dpu->pd_pll0_dev, true);

	if (dpu->pd_dc_link && !IS_ERR(dpu->pd_dc_link))
		device_link_del(dpu->pd_dc_link);
	if (dpu->pd_dc_dev && !IS_ERR(dpu->pd_dc_dev))
		dev_pm_domain_detach(dpu->pd_dc_dev, true);

	dpu->pd_dc_dev = NULL;
	dpu->pd_dc_link = NULL;
	dpu->pd_pll0_dev = NULL;
	dpu->pd_pll0_link = NULL;
	dpu->pd_pll1_dev = NULL;
	dpu->pd_pll1_link = NULL;
}

static int dpu_attach_pm_domains(struct dpu_soc *dpu)
{
	struct device *dev = dpu->dev;
	u32 flags = DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE;
	int ret = 0;

	dpu->pd_dc_dev = dev_pm_domain_attach_by_name(dev, "dc");
	if (IS_ERR(dpu->pd_dc_dev)) {
		ret = PTR_ERR(dpu->pd_dc_dev);
		dev_err(dev, "Failed to attach dc pd dev: %d\n", ret);
		goto fail;
	}
	dpu->pd_dc_link = device_link_add(dev, dpu->pd_dc_dev, flags);
	if (IS_ERR(dpu->pd_dc_link)) {
		ret = PTR_ERR(dpu->pd_dc_link);
		dev_err(dev, "Failed to add device link to dc pd dev: %d\n",
			ret);
		goto fail;
	}

	dpu->pd_pll0_dev = dev_pm_domain_attach_by_name(dev, "pll0");
	if (IS_ERR(dpu->pd_pll0_dev)) {
		ret = PTR_ERR(dpu->pd_pll0_dev);
		dev_err(dev, "Failed to attach pll0 pd dev: %d\n", ret);
		goto fail;
	}
	dpu->pd_pll0_link = device_link_add(dev, dpu->pd_pll0_dev, flags);
	if (IS_ERR(dpu->pd_pll0_link)) {
		ret = PTR_ERR(dpu->pd_pll0_link);
		dev_err(dev, "Failed to add device link to pll0 pd dev: %d\n",
			ret);
		goto fail;
	}

	dpu->pd_pll1_dev = dev_pm_domain_attach_by_name(dev, "pll1");
	if (IS_ERR(dpu->pd_pll1_dev)) {
		ret = PTR_ERR(dpu->pd_pll1_dev);
		dev_err(dev, "Failed to attach pll0 pd dev: %d\n", ret);
		goto fail;
	}
	dpu->pd_pll1_link = device_link_add(dev, dpu->pd_pll1_dev, flags);
	if (IS_ERR(dpu->pd_pll1_link)) {
		ret = PTR_ERR(dpu->pd_pll1_link);
		dev_err(dev, "Failed to add device link to pll1 pd dev: %d\n",
			ret);
		goto fail;
	}

	return ret;
fail:
	dpu_detach_pm_domains(dpu);
	return ret;
}

#define DPU_UNITS_ADDR_DBG(unit)					\
{									\
	const struct dpu_unit *us = data->unit##s;			\
	int i;								\
	for (i = 0; i < us->num; i++) {					\
		if (us->pec_ofss) {					\
			dev_dbg(&pdev->dev, "%s%d: pixengcfg @ 0x%08lx,"\
				" unit @ 0x%08lx\n", us->name,		\
				us->ids[i],				\
				dpu_base + us->pec_ofss[i],		\
				dpu_base + us->ofss[i]);		\
		} else {						\
			dev_dbg(&pdev->dev,				\
				"%s%d: unit @ 0x%08lx\n", us->name,	\
				us->ids[i], dpu_base + us->ofss[i]);	\
		}							\
	}								\
}

static void dpu_units_addr_dbg(struct dpu_soc *dpu,
			struct platform_device *pdev, unsigned long dpu_base)
{
	const struct dpu_data *data = dpu->data;

	dev_dbg(dpu->dev, "Common: 0x%08lx\n", dpu_base + data->cm_ofs);
	DPU_UNITS_ADDR_DBG(cf);
	DPU_UNITS_ADDR_DBG(dec);
	DPU_UNITS_ADDR_DBG(ed);
	DPU_UNITS_ADDR_DBG(fd);
	DPU_UNITS_ADDR_DBG(fe);
	DPU_UNITS_ADDR_DBG(fg);
	DPU_UNITS_ADDR_DBG(fl);
	DPU_UNITS_ADDR_DBG(fw);
	DPU_UNITS_ADDR_DBG(hs);
	DPU_UNITS_ADDR_DBG(lb);
	DPU_UNITS_ADDR_DBG(sig);
	DPU_UNITS_ADDR_DBG(st);
	DPU_UNITS_ADDR_DBG(tcon);
	DPU_UNITS_ADDR_DBG(vs);
}

static int dpu_get_irq(struct platform_device *pdev, struct dpu_soc *dpu)
{
#define DPU_GET_IRQ(name)						\
{									\
	dpu->irq_##name = platform_get_irq_byname(pdev, "" #name "");	\
	dev_dbg(dpu->dev, "irq_" #name ": %d\n", dpu->irq_##name);	\
	if (dpu->irq_##name < 0) {					\
		dev_err(dpu->dev, "failed to get irq " #name "\n");	\
		return dpu->irq_##name;					\
	}								\
}

	DPU_GET_IRQ(extdst0_shdload);
	DPU_GET_IRQ(extdst4_shdload);
	DPU_GET_IRQ(extdst1_shdload);
	DPU_GET_IRQ(extdst5_shdload);
	DPU_GET_IRQ(disengcfg_shdload0);
	DPU_GET_IRQ(disengcfg_framecomplete0);
	DPU_GET_IRQ(sig0_shdload);
	DPU_GET_IRQ(sig0_valid);
	DPU_GET_IRQ(disengcfg_shdload1);
	DPU_GET_IRQ(disengcfg_framecomplete1);
	DPU_GET_IRQ(sig1_shdload);
	DPU_GET_IRQ(sig1_valid);

	return 0;
}

static void dpu_irq_handle(struct irq_desc *desc, enum dpu_irq irq)
{
	struct dpu_soc *dpu = irq_desc_get_handler_data(desc);
	const struct dpu_data *data = dpu->data;
	const struct cm_reg_ofs *ofs = data->cm_reg_ofs;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu_cm_read(dpu, USERINTERRUPTSTATUS(ofs, irq / 32));
	status &= dpu_cm_read(dpu, USERINTERRUPTENABLE(ofs, irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_linear_revmap(dpu->domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

#define DPU_IRQ_HANDLER_DEFINE(name1, name2)			\
static void dpu_##name1##_irq_handler(struct irq_desc *desc)	\
{								\
	dpu_irq_handle(desc, IRQ_##name2);			\
}

DPU_IRQ_HANDLER_DEFINE(extdst0_shdload, EXTDST0_SHDLOAD)
DPU_IRQ_HANDLER_DEFINE(extdst4_shdload, EXTDST4_SHDLOAD)
DPU_IRQ_HANDLER_DEFINE(extdst1_shdload, EXTDST1_SHDLOAD)
DPU_IRQ_HANDLER_DEFINE(extdst5_shdload, EXTDST5_SHDLOAD)
DPU_IRQ_HANDLER_DEFINE(disengcfg_shdload0, DISENGCFG_SHDLOAD0)
DPU_IRQ_HANDLER_DEFINE(disengcfg_framecomplete0, DISENGCFG_FRAMECOMPLETE0)
DPU_IRQ_HANDLER_DEFINE(sig0_shdload, SIG0_SHDLOAD);
DPU_IRQ_HANDLER_DEFINE(sig0_valid, SIG0_VALID);
DPU_IRQ_HANDLER_DEFINE(disengcfg_shdload1, DISENGCFG_SHDLOAD1)
DPU_IRQ_HANDLER_DEFINE(disengcfg_framecomplete1, DISENGCFG_FRAMECOMPLETE1)
DPU_IRQ_HANDLER_DEFINE(sig1_shdload, SIG1_SHDLOAD);
DPU_IRQ_HANDLER_DEFINE(sig1_valid, SIG1_VALID);

int dpu_map_irq(struct dpu_soc *dpu, int irq)
{
	int virq = irq_linear_revmap(dpu->domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->domain, irq);

	return virq;
}
EXPORT_SYMBOL_GPL(dpu_map_irq);

static int dpu_irq_init(struct dpu_soc *dpu)
{
	const struct dpu_data *data = dpu->data;
	const struct cm_reg_ofs *ofs = data->cm_reg_ofs;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int ret, i;

	dpu->domain = irq_domain_add_linear(dpu->dev->of_node,
					    dpu->irq_line_num,
					    &irq_generic_chip_ops, dpu);
	if (!dpu->domain) {
		dev_err(dpu->dev, "failed to add irq domain\n");
		return -ENODEV;
	}

	ret = irq_alloc_domain_generic_chips(dpu->domain, 32, 1, "DPU",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dpu->dev, "failed to alloc generic irq chips\n");
		irq_domain_remove(dpu->domain);
		return ret;
	}

	for (i = 0; i < dpu->irq_line_num; i += 32) {
		/* Mask and clear all interrupts */
		dpu_cm_write(dpu, USERINTERRUPTENABLE(ofs, i / 32), 0);
		dpu_cm_write(dpu, USERINTERRUPTCLEAR(ofs, i / 32),
					~data->unused_irq[i / 32]);
		dpu_cm_write(dpu, INTERRUPTENABLE(ofs, i / 32), 0);
		dpu_cm_write(dpu, INTERRUPTCLEAR(ofs, i / 32),
					~data->unused_irq[i / 32]);

		/* Set all interrupts to user mode */
		dpu_cm_write(dpu, USERINTERRUPTMASK(ofs, i / 32),
					~data->unused_irq[i / 32]);

		gc = irq_get_domain_generic_chip(dpu->domain, i);
		gc->reg_base = dpu->cm_reg;
		gc->unused = data->unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = USERINTERRUPTCLEAR(ofs, i / 32);
		ct->regs.mask = USERINTERRUPTENABLE(ofs, i / 32);
	}

#define DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(name)	\
irq_set_chained_handler_and_data(dpu->irq_##name, dpu_##name##_irq_handler, dpu)

	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(extdst0_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(extdst4_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(extdst1_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(extdst5_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(disengcfg_shdload0);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(disengcfg_framecomplete0);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(sig0_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(sig0_valid);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(disengcfg_shdload1);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(disengcfg_framecomplete1);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(sig1_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA1(sig1_valid);

#define DPU_IRQ_CHIP_PM_GET(name)					\
{									\
	ret = irq_chip_pm_get(irq_get_irq_data(dpu->irq_##name));	\
	if (ret < 0) {							\
		dev_err(dpu->dev,					\
			"failed to get irq chip PM for irq%d %d\n",	\
						dpu->irq_##name, ret);	\
		goto pm_get_rollback;					\
	}								\
	dpu->irq_chip_pm_get_##name = true;				\
}

#define DPU_IRQ_CHIP_PM_PUT_CHECK(name)					\
{									\
	if (dpu->irq_chip_pm_get_##name) {				\
		irq_chip_pm_put(irq_get_irq_data(dpu->irq_##name));	\
		dpu->irq_chip_pm_get_##name = false;			\
	}								\
}

	DPU_IRQ_CHIP_PM_GET(extdst0_shdload);
	DPU_IRQ_CHIP_PM_GET(extdst4_shdload);
	DPU_IRQ_CHIP_PM_GET(extdst1_shdload);
	DPU_IRQ_CHIP_PM_GET(extdst5_shdload);
	DPU_IRQ_CHIP_PM_GET(disengcfg_shdload0);
	DPU_IRQ_CHIP_PM_GET(disengcfg_framecomplete0);
	DPU_IRQ_CHIP_PM_GET(sig0_shdload);
	DPU_IRQ_CHIP_PM_GET(sig0_valid);
	DPU_IRQ_CHIP_PM_GET(disengcfg_shdload1);
	DPU_IRQ_CHIP_PM_GET(disengcfg_framecomplete1);
	DPU_IRQ_CHIP_PM_GET(sig1_shdload);
	DPU_IRQ_CHIP_PM_GET(sig1_valid);

	return 0;

pm_get_rollback:
	DPU_IRQ_CHIP_PM_PUT_CHECK(extdst0_shdload);
	DPU_IRQ_CHIP_PM_PUT_CHECK(extdst4_shdload);
	DPU_IRQ_CHIP_PM_PUT_CHECK(extdst1_shdload);
	DPU_IRQ_CHIP_PM_PUT_CHECK(extdst5_shdload);
	DPU_IRQ_CHIP_PM_PUT_CHECK(disengcfg_shdload0);
	DPU_IRQ_CHIP_PM_PUT_CHECK(disengcfg_framecomplete0);
	DPU_IRQ_CHIP_PM_PUT_CHECK(sig0_shdload);
	DPU_IRQ_CHIP_PM_PUT_CHECK(sig0_valid);
	DPU_IRQ_CHIP_PM_PUT_CHECK(disengcfg_shdload1);
	DPU_IRQ_CHIP_PM_PUT_CHECK(disengcfg_framecomplete1);
	DPU_IRQ_CHIP_PM_PUT_CHECK(sig1_shdload);
	DPU_IRQ_CHIP_PM_PUT_CHECK(sig1_valid);

	return ret;
}

static void dpu_irq_exit(struct dpu_soc *dpu)
{
	unsigned int i, irq;

#define DPU_IRQ_CHIP_PM_PUT(name)				\
{								\
	irq_chip_pm_put(irq_get_irq_data(dpu->irq_##name));	\
	dpu->irq_chip_pm_get_##name = false;			\
}

	DPU_IRQ_CHIP_PM_PUT(extdst0_shdload);
	DPU_IRQ_CHIP_PM_PUT(extdst4_shdload);
	DPU_IRQ_CHIP_PM_PUT(extdst1_shdload);
	DPU_IRQ_CHIP_PM_PUT(extdst5_shdload);
	DPU_IRQ_CHIP_PM_PUT(disengcfg_shdload0);
	DPU_IRQ_CHIP_PM_PUT(disengcfg_framecomplete0);
	DPU_IRQ_CHIP_PM_PUT(sig0_shdload);
	DPU_IRQ_CHIP_PM_PUT(sig0_valid);
	DPU_IRQ_CHIP_PM_PUT(disengcfg_shdload1);
	DPU_IRQ_CHIP_PM_PUT(disengcfg_framecomplete1);
	DPU_IRQ_CHIP_PM_PUT(sig1_shdload);
	DPU_IRQ_CHIP_PM_PUT(sig1_valid);

#define DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(name)	\
irq_set_chained_handler_and_data(dpu->irq_##name, NULL, NULL)

	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(extdst0_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(extdst4_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(extdst1_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(extdst5_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(disengcfg_shdload0);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(disengcfg_framecomplete0);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(sig0_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(sig0_valid);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(disengcfg_shdload1);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(disengcfg_framecomplete1);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(sig1_shdload);
	DPU_IRQ_SET_CHAINED_HANDLER_AND_DATA2(sig1_valid);

	for (i = 0; i < dpu->irq_line_num; i++) {
		irq = irq_linear_revmap(dpu->domain, i);
		if (irq)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(dpu->domain);
}

#define _DPU_UNITS_INIT(unit)						\
{									\
	const struct dpu_unit *us = data->unit##s;			\
	int i;								\
									\
	/* software check */						\
	if (WARN_ON(us->num > ARRAY_SIZE(unit##_ids)))			\
		return -EINVAL;						\
									\
	for (i = 0; i < us->num; i++)					\
		_dpu_##unit##_init(dpu, us->ids[i]);			\
}

static int
_dpu_submodules_init(struct dpu_soc *dpu, struct platform_device *pdev)
{
	const struct dpu_data *data = dpu->data;

	_DPU_UNITS_INIT(cf);
	_DPU_UNITS_INIT(dec);
	_DPU_UNITS_INIT(ed);
	_DPU_UNITS_INIT(fd);
	_DPU_UNITS_INIT(fe);
	_DPU_UNITS_INIT(fg);
	_DPU_UNITS_INIT(fl);
	_DPU_UNITS_INIT(fw);
	_DPU_UNITS_INIT(hs);
	_DPU_UNITS_INIT(lb);
	_DPU_UNITS_INIT(sig);
	_DPU_UNITS_INIT(st);
	_DPU_UNITS_INIT(tcon);
	_DPU_UNITS_INIT(vs);

	return 0;
}

#define DPU_UNIT_INIT(dpu, base, unit, name, id, pec_ofs, ofs)		\
{									\
	int ret;							\
	ret = dpu_##unit##_init((dpu),	(id),				\
				(pec_ofs) ? (base) + (pec_ofs) : 0,	\
				(base) + (ofs));			\
	if (ret) {							\
		dev_err((dpu)->dev, "init %s%d failed with %d\n",	\
						(name), (id), ret);	\
		return ret;						\
	}								\
}

#define DPU_UNITS_INIT(unit)						\
{									\
	const struct dpu_unit *us = data->unit##s;			\
	int i;								\
									\
	/* software check */						\
	if (WARN_ON(us->num > ARRAY_SIZE(unit##_ids)))			\
		return -EINVAL;						\
									\
	for (i = 0; i < us->num; i++)					\
		DPU_UNIT_INIT(dpu, dpu_base, unit, us->name,		\
			      us->ids[i],				\
			      us->pec_ofss ? us->pec_ofss[i] : 0,	\
			      us->ofss[i]);				\
}

static int dpu_submodules_init(struct dpu_soc *dpu,
		struct platform_device *pdev, unsigned long dpu_base)
{
	const struct dpu_data *data = dpu->data;
	const struct dpu_unit *fds = data->fds;
	const struct dpu_unit *fls = data->fls;
	const struct dpu_unit *fws = data->fws;
	const struct dpu_unit *tcons = data->tcons;
	struct dpu_fetchunit *fu;
	struct dprc *dprc;
	struct dpu_tcon *tcon;
	struct pc *pc;
	int i;

	DPU_UNITS_INIT(cf);
	DPU_UNITS_INIT(dec);
	DPU_UNITS_INIT(ed);
	DPU_UNITS_INIT(fd);
	DPU_UNITS_INIT(fe);
	DPU_UNITS_INIT(fg);
	DPU_UNITS_INIT(fl);
	DPU_UNITS_INIT(fw);
	DPU_UNITS_INIT(hs);
	DPU_UNITS_INIT(lb);
	DPU_UNITS_INIT(sig);
	DPU_UNITS_INIT(st);
	DPU_UNITS_INIT(tcon);
	DPU_UNITS_INIT(vs);

	for (i = 0; i < fds->num; i++) {
		dprc = dprc_lookup_by_phandle(dpu->dev, "fsl,dpr-channels",
					      fds->dprc_ids[i]);
		if (!dprc)
			return -EPROBE_DEFER;

		fu = dpu_fd_get(dpu, i);
		fetchunit_get_dprc(fu, dprc);
		dpu_fd_put(fu);
	}

	for (i = 0; i < fls->num; i++) {
		dprc = dprc_lookup_by_phandle(dpu->dev, "fsl,dpr-channels",
					      fls->dprc_ids[i]);
		if (!dprc)
			return -EPROBE_DEFER;

		fu = dpu_fl_get(dpu, i);
		fetchunit_get_dprc(fu, dprc);
		dpu_fl_put(fu);
	}

	for (i = 0; i < fws->num; i++) {
		dprc = dprc_lookup_by_phandle(dpu->dev, "fsl,dpr-channels",
					      fws->dprc_ids[i]);
		if (!dprc)
			return -EPROBE_DEFER;

		fu = dpu_fw_get(dpu, fw_ids[i]);
		fetchunit_get_dprc(fu, dprc);
		dpu_fw_put(fu);
	}

	pc = pc_lookup_by_phandle(dpu->dev, "fsl,pixel-combiner");
	if (!pc)
		return -EPROBE_DEFER;

	for (i = 0; i < tcons->num; i++) {
		tcon = dpu_tcon_get(dpu, i);
		tcon_get_pc(tcon, pc);
		dpu_tcon_put(tcon);
	}

	return 0;
}

static int platform_remove_devices_fn(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static void platform_device_unregister_children(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, platform_remove_devices_fn);
}

struct dpu_platform_reg {
	struct dpu_client_platformdata pdata;
	const char *name;
};

static struct dpu_platform_reg client_reg[] = {
	{
		.pdata = {
			.stream_id = 0,
		},
		.name = "imx-dpu-crtc",
	}, {
		.pdata = {
			.stream_id = 1,
		},
		.name = "imx-dpu-crtc",
	}, {
		.pdata = { },
		.name = IMX_DPU_BLITENG_NAME,
	}
};

static DEFINE_MUTEX(dpu_client_id_mutex);
static int dpu_client_id;

static int dpu_get_plane_resource(struct dpu_soc *dpu,
				  struct dpu_plane_res *res)
{
	const struct dpu_unit *fds = dpu->data->fds;
	const struct dpu_unit *fls = dpu->data->fls;
	const struct dpu_unit *fws = dpu->data->fws;
	const struct dpu_unit *lbs = dpu->data->lbs;
	struct dpu_plane_grp *grp = plane_res_to_grp(res);
	int i;

	for (i = 0; i < ARRAY_SIZE(res->ed); i++) {
		res->ed[i] = dpu_ed_get(dpu, i);
		if (IS_ERR(res->ed[i]))
			return PTR_ERR(res->ed[i]);
	}
	for (i = 0; i < fds->num; i++) {
		res->fd[i] = dpu_fd_get(dpu, i);
		if (IS_ERR(res->fd[i]))
			return PTR_ERR(res->fd[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fe); i++) {
		res->fe[i] = dpu_fe_get(dpu, i);
		if (IS_ERR(res->fe[i]))
			return PTR_ERR(res->fe[i]);
		grp->hw_plane_fetcheco_num = ARRAY_SIZE(res->fe);
	}
	for (i = 0; i < fls->num; i++) {
		res->fl[i] = dpu_fl_get(dpu, i);
		if (IS_ERR(res->fl[i]))
			return PTR_ERR(res->fl[i]);
	}
	for (i = 0; i < fws->num; i++) {
		res->fw[i] = dpu_fw_get(dpu, fw_ids[i]);
		if (IS_ERR(res->fw[i]))
			return PTR_ERR(res->fw[i]);
	}
	/* HScaler could be shared with capture. */
	if (display_plane_video_proc) {
		for (i = 0; i < ARRAY_SIZE(res->hs); i++) {
			res->hs[i] = dpu_hs_get(dpu, hs_ids[i]);
			if (IS_ERR(res->hs[i]))
				return PTR_ERR(res->hs[i]);
		}
		grp->hw_plane_hscaler_num = ARRAY_SIZE(res->hs);
	}
	for (i = 0; i < lbs->num; i++) {
		res->lb[i] = dpu_lb_get(dpu, i);
		if (IS_ERR(res->lb[i]))
			return PTR_ERR(res->lb[i]);
	}
	/* VScaler could be shared with capture. */
	if (display_plane_video_proc) {
		for (i = 0; i < ARRAY_SIZE(res->vs); i++) {
			res->vs[i] = dpu_vs_get(dpu, vs_ids[i]);
			if (IS_ERR(res->vs[i]))
				return PTR_ERR(res->vs[i]);
		}
		grp->hw_plane_vscaler_num = ARRAY_SIZE(res->vs);
	}

	grp->hw_plane_num = fds->num + fls->num + fws->num;

	return 0;
}

static void dpu_put_plane_resource(struct dpu_plane_res *res)
{
	struct dpu_plane_grp *grp = plane_res_to_grp(res);
	int i;

	for (i = 0; i < ARRAY_SIZE(res->ed); i++) {
		if (!IS_ERR_OR_NULL(res->ed[i]))
			dpu_ed_put(res->ed[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fd); i++) {
		if (!IS_ERR_OR_NULL(res->fd[i]))
			dpu_fd_put(res->fd[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fe); i++) {
		if (!IS_ERR_OR_NULL(res->fe[i]))
			dpu_fe_put(res->fe[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fl); i++) {
		if (!IS_ERR_OR_NULL(res->fl[i]))
			dpu_fl_put(res->fl[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->fw); i++) {
		if (!IS_ERR_OR_NULL(res->fw[i]))
			dpu_fw_put(res->fw[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->hs); i++) {
		if (!IS_ERR_OR_NULL(res->hs[i]))
			dpu_hs_put(res->hs[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->lb); i++) {
		if (!IS_ERR_OR_NULL(res->lb[i]))
			dpu_lb_put(res->lb[i]);
	}
	for (i = 0; i < ARRAY_SIZE(res->vs); i++) {
		if (!IS_ERR_OR_NULL(res->vs[i]))
			dpu_vs_put(res->vs[i]);
	}

	grp->hw_plane_num = 0;
}

static int dpu_add_client_devices(struct dpu_soc *dpu)
{
	const struct dpu_data *data = dpu->data;
	struct device *dev = dpu->dev;
	struct dpu_platform_reg *reg;
	struct dpu_plane_grp *plane_grp;
	struct dpu_store *st9 = NULL;
	size_t client_num, reg_size;
	int i, id, ret;

	client_num = ARRAY_SIZE(client_reg);

	reg = devm_kcalloc(dev, client_num, sizeof(*reg), GFP_KERNEL);
	if (!reg)
		return -ENODEV;

	plane_grp = devm_kzalloc(dev, sizeof(*plane_grp), GFP_KERNEL);
	if (!plane_grp)
		return -ENODEV;

	mutex_init(&plane_grp->mutex);

	mutex_lock(&dpu_client_id_mutex);
	id = dpu_client_id;
	dpu_client_id += client_num;
	mutex_unlock(&dpu_client_id_mutex);

	reg_size = client_num * sizeof(struct dpu_platform_reg);
	memcpy(reg, &client_reg[0], reg_size);

	plane_grp->src_mask = data->plane_src_mask;
	plane_grp->id = id / client_num;
	plane_grp->has_vproc = display_plane_video_proc;

	ret = dpu_get_plane_resource(dpu, &plane_grp->res);
	if (ret)
		goto err_get_plane_res;

	st9 = dpu_st_get(dpu, 9);
	if (IS_ERR(st9)) {
		ret = PTR_ERR(st9);
		goto err_get_plane_res;
	}

	for (i = 0; i < client_num; i++) {
		struct platform_device *pdev;
		struct device_node *of_node = NULL;

		if (!strcmp(reg[i].name, IMX_DPU_BLITENG_NAME)) {
			/* As bliteng has no of_node, so to use dpu's. */
			of_node = dev->of_node;
		} else {
			/* Associate subdevice with the corresponding port node. */
			of_node = of_graph_get_port_by_id(dev->of_node, i);
			if (!of_node) {
				dev_info(dev,
					"no port@%d node in %s, not using DISP%d\n",
					i, dev->of_node->full_name, i);
				continue;
			}
		}

		reg[i].pdata.plane_grp = plane_grp;
		reg[i].pdata.di_grp_id = plane_grp->id;
		reg[i].pdata.st9 = st9;

		pdev = platform_device_alloc(reg[i].name, id++);
		if (!pdev) {
			ret = -ENOMEM;
			goto err_register;
		}

		pdev->dev.parent = dev;

		reg[i].pdata.of_node = of_node;
		ret = platform_device_add_data(pdev, &reg[i].pdata,
					       sizeof(reg[i].pdata));
		if (!ret)
			ret = platform_device_add(pdev);
		if (ret) {
			platform_device_put(pdev);
			goto err_register;
		}
	}

	return 0;

err_register:
	platform_device_unregister_children(to_platform_device(dev));
	dpu_st_put(st9);
err_get_plane_res:
	dpu_put_plane_resource(&plane_grp->res);

	return ret;
}

static int dpu_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id;
	struct device_node *np = pdev->dev.of_node;
	struct dpu_soc *dpu;
	struct resource *res;
	unsigned long dpu_base;
	const struct dpu_data *data;
	int ret;

	of_id = of_match_device(dpu_dt_ids, &pdev->dev);
	if (!of_id)
		return -ENODEV;

	data = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dpu_base = res->start;

	dpu = devm_kzalloc(&pdev->dev, sizeof(*dpu), GFP_KERNEL);
	if (!dpu)
		return -ENODEV;

	dpu->dev = &pdev->dev;
	dpu->data = data;
	dpu->id = of_alias_get_id(np, "dpu");
	dpu->irq_line_num = platform_irq_count(pdev);
	if (dpu->irq_line_num < 0)
		return dpu->irq_line_num;

	dpu_units_addr_dbg(dpu, pdev, dpu_base);

	ret = dpu_get_irq(pdev, dpu);
	if (ret < 0)
		return ret;

	ret = dpu_sc_misc_get_handle(dpu);
	if (ret < 0)
		return ret;

	spin_lock_init(&dpu->lock);

	dpu->cm_reg = devm_ioremap(dpu->dev, dpu_base + data->cm_ofs, SZ_1K);
	if (!dpu->cm_reg)
		return -ENOMEM;

	ret = dpu_attach_pm_domains(dpu);
	if (ret)
		return ret;

	ret = dpu_irq_init(dpu);
	if (ret)
		goto failed_irq;

	ret = dpu_submodules_init(dpu, pdev, dpu_base);
	if (ret)
		goto failed_submodules_init;

	ret = dpu_sc_misc_init(dpu);
	if (ret < 0) {
		dev_err(dpu->dev,
			"failed to initialize pixel link %d\n", ret);
		goto failed_sc_misc_init;
	}

	platform_set_drvdata(pdev, dpu);

	ret = dpu_add_client_devices(dpu);
	if (ret) {
		dev_err(dpu->dev,
			"adding client devices failed with %d\n", ret);
		goto failed_add_clients;
	}

	dev_info(dpu->dev, "driver probed\n");

	return 0;

failed_add_clients:
	platform_set_drvdata(pdev, NULL);
failed_sc_misc_init:
failed_submodules_init:
	dpu_irq_exit(dpu);
failed_irq:
	dpu_detach_pm_domains(dpu);
	return ret;
}

static int dpu_remove(struct platform_device *pdev)
{
	struct dpu_soc *dpu = platform_get_drvdata(pdev);

	platform_device_unregister_children(pdev);

	dpu_irq_exit(dpu);
	dpu_detach_pm_domains(dpu);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dpu_suspend(struct device *dev)
{
	/*
	 * The dpu core driver currently depends on the client drivers
	 * to do suspend operations to leave dpu a cleaned up state
	 * machine status before the system enters sleep mode.
	 */
	return 0;
}

static int dpu_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dpu_soc *dpu = platform_get_drvdata(pdev);

	dpu_sc_misc_init(dpu);

	_dpu_submodules_init(dpu, pdev);

	return 0;
}
#endif

static const struct dev_pm_ops dpu_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(dpu_suspend, dpu_resume)
};

static struct platform_driver dpu_driver = {
	.driver = {
		.pm = &dpu_pm_ops,
		.name = "dpu-core",
		.of_match_table = dpu_dt_ids,
	},
	.probe = dpu_probe,
	.remove = dpu_remove,
};

module_platform_driver(dpu_driver);

MODULE_DESCRIPTION("i.MX DPU driver");
MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_LICENSE("GPL");
