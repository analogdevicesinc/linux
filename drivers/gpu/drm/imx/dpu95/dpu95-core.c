// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2020,2022,2023 NXP
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include "dpu95.h"
#include "dpu95-drv.h"

/* register in blk-ctrl */
#define QOS_SETTING			0x1c
#define  DISPLAY_PANIC_QOS_MASK		0x70
#define  DISPLAY_PANIC_QOS(n)		(((n) & 0x7) << 4)
#define  DISPLAY_ARQOS_MASK		0x7
#define  DISPLAY_ARQOS(n)		((n) & 0x7)

static inline u32 dpu95_comctrl_irq_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->comctrl_irq_reg + offset);
}

static inline void dpu95_comctrl_irq_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->comctrl_irq_reg + offset);
}

static inline u32 dpu95_disp_irq0_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->disp_irq0_reg + offset);
}

static inline void dpu95_disp_irq0_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->disp_irq0_reg + offset);
}

static inline u32 dpu95_disp_irq2_read(struct dpu95_soc *dpu, unsigned int offset)
{
	return readl(dpu->disp_irq2_reg + offset);
}

static inline void dpu95_disp_irq2_write(struct dpu95_soc *dpu,
					 unsigned int offset, u32 value)
{
	writel(value, dpu->disp_irq2_reg + offset);
}

static inline void dpu95_dm_mask_write(struct dpu95_soc *dpu,
				       unsigned int offset, u32 value)
{
	writel(value, dpu->dm_mask_reg + offset);
}

/* Constant Frame */
static const unsigned int cf_ids[] = {0, 1, 4, 5};
static const enum dpu95_unit_type cf_types[] = {DPU95_DISP, DPU95_DISP,
						DPU95_DISP, DPU95_DISP};
static const unsigned long cf_ofss[] = {0xf0000, 0x130000, 0x100000, 0x140000};
static const unsigned long cf_aux_ofss[] = {0xf1000, 0x131000, 0x101000, 0x141000};

static const struct dpu95_units dpu_cfs = {
	.ids = cf_ids,
	.types = cf_types,
	.ofss = cf_ofss,
	.aux_ofss = cf_aux_ofss,
	.cnt = ARRAY_SIZE(cf_ids),
	.name = "ConstFrame",
	.init = dpu95_cf_init,
	.hw_init = dpu95_cf_hw_init,
};

/* Domain Blend */
static const unsigned int db_ids[] = {0, 1};
static const enum dpu95_unit_type db_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long db_ofss[] = {0x2a0000, 0x320000};

static const struct dpu95_units dpu_dbs = {
	.ids = db_ids,
	.types = db_types,
	.ofss = db_ofss,
	.cnt = ARRAY_SIZE(db_ids),
	.name = "DomainBlend",
	.init = dpu95_db_init,
	.hw_init = dpu95_db_hw_init,
};

/* Dither */
static const unsigned int dt_ids[] = {0, 1};
static const enum dpu95_unit_type dt_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long dt_ofss[] = {0x310000, 0x370000};
static const unsigned long dt_aux_ofss[] = {0x311000, 0x371020};

static const struct dpu95_units dpu_dts = {
	.ids = dt_ids,
	.types = dt_types,
	.ofss = dt_ofss,
	.aux_ofss = dt_aux_ofss,
	.cnt = ARRAY_SIZE(dt_ids),
	.name = "Dither",
	.init = dpu95_dt_init,
	.hw_init = dpu95_dt_hw_init,
};

/* External Destination */
static const unsigned int ed_ids[] = {0, 1, 4, 5};
static const enum dpu95_unit_type ed_types[] = {DPU95_DISP, DPU95_DISP,
						DPU95_DISP, DPU95_DISP};
static const unsigned long ed_ofss[] = {0x110000, 0x150000, 0x120000, 0x160000};
static const unsigned long ed_aux_ofss[] = {0x111000, 0x151000, 0x121000, 0x161000};

static const struct dpu95_units dpu_eds = {
	.ids = ed_ids,
	.types = ed_types,
	.ofss = ed_ofss,
	.aux_ofss = ed_aux_ofss,
	.cnt = ARRAY_SIZE(ed_ids),
	.name = "ExtDst",
	.init = dpu95_ed_init,
	.hw_init = dpu95_ed_hw_init,
};

/* Fetch ECO */
static const unsigned int fe_ids[] = {0, 1, 2, 9};
static const enum dpu95_unit_type fe_types[] = {DPU95_DISP, DPU95_DISP,
						DPU95_DISP, DPU95_DISP};
static const unsigned long fe_ofss[] = {0x210000, 0x230000, 0x250000, 0xa0000};
static const unsigned long fe_aux_ofss[] = {0x211000, 0x231000, 0x251000, 0xa1000};

static const struct dpu95_units dpu_fes = {
	.ids = fe_ids,
	.types = fe_types,
	.ofss = fe_ofss,
	.aux_ofss = fe_aux_ofss,
	.cnt = ARRAY_SIZE(fe_ids),
	.name = "FetchEco",
	.init = dpu95_fe_init,
	.hw_init = dpu95_fe_hw_init,
};

/* Frame Generator */
static const unsigned int fg_ids[] = {0, 1};
static const enum dpu95_unit_type fg_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long fg_ofss[] = {0x2b0000, 0x330000};

static const struct dpu95_units dpu_fgs = {
	.ids = fg_ids,
	.types = fg_types,
	.ofss = fg_ofss,
	.cnt = ARRAY_SIZE(fg_ids),
	.name = "FrameGen",
	.init = dpu95_fg_init,
	.hw_init = dpu95_fg_hw_init,
};

/* Fetch Layer */
static const unsigned int fl_ids[] = {0, 1};
static const enum dpu95_unit_type fl_types[] = {DPU95_DISP, DPU95_DISP};
static const unsigned long fl_ofss[] = {0x1d0000, 0x1e0000};
static const unsigned long fl_aux_ofss[] = {0x1d1000, 0x1e1000};

static const struct dpu95_units dpu_fls = {
	.ids = fl_ids,
	.types = fl_types,
	.ofss = fl_ofss,
	.aux_ofss = fl_aux_ofss,
	.cnt = ARRAY_SIZE(fl_ids),
	.name = "FetchLayer",
	.init = dpu95_fl_init,
	.hw_init = dpu95_fl_hw_init,
};

/* Fetch YUV */
static const unsigned int fy_ids[] = {0, 1, 2, 3};
static const enum dpu95_unit_type fy_types[] = {DPU95_DISP, DPU95_DISP,
						DPU95_DISP, DPU95_DISP};
static const unsigned long fy_ofss[] = {0x200000, 0x220000, 0x240000, 0x1f0000};
static const unsigned long fy_aux_ofss[] = {0x201000, 0x221000, 0x241000, 0x1f1000};

static const struct dpu95_units dpu_fys = {
	.ids = fy_ids,
	.types = fy_types,
	.ofss = fy_ofss,
	.aux_ofss = fy_aux_ofss,
	.cnt = ARRAY_SIZE(fy_ids),
	.name = "FetchYUV",
	.init = dpu95_fy_init,
	.hw_init = dpu95_fy_hw_init,
};

/* Horizontal Scaler */
static const unsigned int hs_ids[] = {4, 9};
static const enum dpu95_unit_type hs_types[] = {DPU95_DISP, DPU95_BLIT};
static const unsigned long hs_ofss[] = {0x270000, 0xb0000};
static const unsigned long hs_aux_ofss[] = {0x271000, 0xb1000};

static const struct dpu95_units dpu_hss = {
	.ids = hs_ids,
	.types = hs_types,
	.ofss = hs_ofss,
	.aux_ofss = hs_aux_ofss,
	.cnt = ARRAY_SIZE(hs_ids),
	.name = "HScaler",
	.init = dpu95_hs_init,
	.hw_init = dpu95_hs_hw_init,
};

/* Layer Blend */
static const unsigned int lb_ids[] = {1, 2, 3, 4, 5, 6};
static const enum dpu95_unit_type lb_types[] = {DPU95_DISP, DPU95_DISP,
						DPU95_DISP, DPU95_DISP,
						DPU95_DISP, DPU95_DISP};
static const unsigned long lb_ofss[] = {0x170000, 0x180000, 0x190000,
					0x1a0000, 0x1b0000, 0x1c0000};
static const unsigned long lb_aux_ofss[] = {0x171000, 0x181000, 0x191000,
					    0x1a1000, 0x1b1000, 0x1c1000};

static const struct dpu95_units dpu_lbs = {
	.ids = lb_ids,
	.types = lb_types,
	.ofss = lb_ofss,
	.aux_ofss = lb_aux_ofss,
	.cnt = ARRAY_SIZE(lb_ids),
	.name = "LayerBlend",
	.init = dpu95_lb_init,
	.hw_init = dpu95_lb_hw_init,
};

static const struct dpu95_units *dpu_all_units[] = {
	&dpu_cfs,
	&dpu_dbs,
	&dpu_dts,
	&dpu_eds,
	&dpu_fes,
	&dpu_fgs,
	&dpu_fls,
	&dpu_fys,
	&dpu_hss,
	&dpu_lbs,
};

static void dpu95_dm_extdst0_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST0_DM, ALLOW_ALL);
}

static void dpu95_dm_extdst1_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST1_DM, ALLOW_ALL);
}

static void dpu95_dm_extdst4_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST4_DM, ALLOW_ALL);
}

static void dpu95_dm_extdst5_dm_allow_all(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST5_DM, ALLOW_ALL);
}

void dpu95_enable_display_pipeline_sync(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, DISPLAY_STATIC, PIPELINE_SYNC);
}

void dpu95_disable_display_pipeline_sync(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, DISPLAY_STATIC, 0);
}

static void dpu95_dm_extdst0_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST0_STATIC, MASTER);
}

static void dpu95_dm_extdst1_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST1_STATIC, MASTER);
}

static void dpu95_dm_extdst4_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST4_STATIC, MASTER);
}

static void dpu95_dm_extdst5_master(struct dpu95_soc *dpu)
{
	dpu95_dm_mask_write(dpu, EXTDST5_STATIC, MASTER);
}

static void dpu95_comctrl_irq_handle(struct irq_desc *desc, enum dpu95_irq irq)
{
	struct dpu95_soc *dpu = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu95_comctrl_irq_read(dpu, INTERRUPTSTATUS(irq / 32));
	status &= dpu95_comctrl_irq_read(dpu, INTERRUPTENABLE(irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_linear_revmap(dpu->comctrl_irq_domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

static void dpu95_disp_irq0_handle(struct irq_desc *desc, enum dpu95_irq irq)
{
	struct dpu95_soc *dpu = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu95_disp_irq0_read(dpu, INTERRUPTSTATUS(irq / 32));
	status &= dpu95_disp_irq0_read(dpu, INTERRUPTENABLE(irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_linear_revmap(dpu->disp_irq0_domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

static void dpu95_disp_irq2_handle(struct irq_desc *desc, enum dpu95_irq irq)
{
	struct dpu95_soc *dpu = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int virq;
	u32 status;

	chained_irq_enter(chip, desc);

	status = dpu95_disp_irq2_read(dpu, INTERRUPTSTATUS(irq / 32));
	status &= dpu95_disp_irq2_read(dpu, INTERRUPTENABLE(irq / 32));

	if (status & BIT(irq % 32)) {
		virq = irq_linear_revmap(dpu->disp_irq2_domain, irq);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

static void dpu95_comctrl_sw0_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW0);
}

static void dpu95_comctrl_sw1_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW1);
}

static void dpu95_comctrl_sw2_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW2);
}

static void dpu95_comctrl_sw3_irq_handler(struct irq_desc *desc)
{
	dpu95_comctrl_irq_handle(desc, DPU95_IRQ_COMCTRL_SW3);
}

static void dpu95_dec_framecomplete0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0);
}

static void dpu95_dec_framecomplete1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1);
}

static void dpu95_dec_seqcomplete0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DISENGCFG_SEQCOMPLETE0);
}

static void dpu95_dec_seqcomplete1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DISENGCFG_SEQCOMPLETE1);
}

static void dpu95_dec_shdload0_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DISENGCFG_SHDLOAD0);
}

static void dpu95_dec_shdload1_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DISENGCFG_SHDLOAD1);
}

static void dpu95_ed0_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_EXTDST0_SHDLOAD);
}

static void dpu95_ed1_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_EXTDST1_SHDLOAD);
}

static void dpu95_db0_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq0_handle(desc, DPU95_IRQ_DOMAINBLEND0_SHDLOAD);
}

static void dpu95_db1_shdload_irq_handler(struct irq_desc *desc)
{
	dpu95_disp_irq2_handle(desc, DPU95_IRQ_DOMAINBLEND1_SHDLOAD);
}

static void (* const dpu95_comctrl_irq_handler[DPU95_IRQ_COUNT])(struct irq_desc *desc) = {
	[DPU95_IRQ_COMCTRL_SW0]              = dpu95_comctrl_sw0_irq_handler,
	[DPU95_IRQ_COMCTRL_SW1]              = dpu95_comctrl_sw1_irq_handler,
	[DPU95_IRQ_COMCTRL_SW2]              = dpu95_comctrl_sw2_irq_handler,
	[DPU95_IRQ_COMCTRL_SW3]              = dpu95_comctrl_sw3_irq_handler,
};

static void (* const dpu95_display_irq0_handler[DPU95_IRQ_COUNT])(struct irq_desc *desc) = {
	[DPU95_IRQ_DOMAINBLEND0_SHDLOAD]     = dpu95_db0_shdload_irq_handler,
	[DPU95_IRQ_EXTDST0_SHDLOAD]          = dpu95_ed0_shdload_irq_handler,
	[DPU95_IRQ_DISENGCFG_SHDLOAD0]       = dpu95_dec_shdload0_irq_handler,
	[DPU95_IRQ_DISENGCFG_FRAMECOMPLETE0] = dpu95_dec_framecomplete0_irq_handler,
	[DPU95_IRQ_DISENGCFG_SEQCOMPLETE0]   = dpu95_dec_seqcomplete0_irq_handler,
};

static void (* const dpu95_display_irq2_handler[DPU95_IRQ_COUNT])(struct irq_desc *desc) = {
	[DPU95_IRQ_DOMAINBLEND1_SHDLOAD]     = dpu95_db1_shdload_irq_handler,
	[DPU95_IRQ_EXTDST1_SHDLOAD]          = dpu95_ed1_shdload_irq_handler,
	[DPU95_IRQ_DISENGCFG_SHDLOAD1]       = dpu95_dec_shdload1_irq_handler,
	[DPU95_IRQ_DISENGCFG_FRAMECOMPLETE1] = dpu95_dec_framecomplete1_irq_handler,
	[DPU95_IRQ_DISENGCFG_SEQCOMPLETE1]   = dpu95_dec_seqcomplete1_irq_handler,
};

int dpu95_map_comctrl_irq(struct dpu95_soc *dpu, int irq)
{
	int virq = irq_linear_revmap(dpu->comctrl_irq_domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->comctrl_irq_domain, irq);

	return virq;
}

int dpu95_map_disp_irq0(struct dpu95_soc *dpu, int irq)
{
	int virq = irq_linear_revmap(dpu->disp_irq0_domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->disp_irq0_domain, irq);

	return virq;
}

int dpu95_map_disp_irq2(struct dpu95_soc *dpu, int irq)
{
	int virq = irq_linear_revmap(dpu->disp_irq2_domain, irq);

	if (!virq)
		virq = irq_create_mapping(dpu->disp_irq2_domain, irq);

	return virq;
}

static const unsigned long unused_irq[] = {0x00000000, 0x00000000, 0xffc00000};

void dpu95_irq_hw_init(struct dpu95_soc *dpu)
{
	int i;

	for (i = 0; i < DPU95_IRQ_COUNT; i += 32) {
		/* mask and clear all interrupts */
		dpu95_comctrl_irq_write(dpu, INTERRUPTENABLE(i / 32), 0);
		dpu95_comctrl_irq_write(dpu, INTERRUPTCLEAR(i / 32), ~unused_irq[i / 32]);
		dpu95_disp_irq0_write(dpu, INTERRUPTENABLE(i / 32), 0);
		dpu95_disp_irq0_write(dpu, INTERRUPTCLEAR(i / 32), ~unused_irq[i / 32]);
		dpu95_disp_irq2_write(dpu, INTERRUPTENABLE(i / 32), 0);
		dpu95_disp_irq2_write(dpu, INTERRUPTCLEAR(i / 32), ~unused_irq[i / 32]);
	}
}

static struct irq_domain *dpu95_find_parent_irq_domain(struct device *dev)
{
	struct device_node *parent;
	struct irq_domain *domain;

	parent = of_irq_find_parent(dev->of_node);
	if (!parent) {
		dev_err(dev, "failed to find parent irq node\n");
		return NULL;
	}

	domain = irq_find_host(parent);
	of_node_put(parent);
	if (!domain) {
		dev_err(dev, "failed to find parent irq domain\n");
		return NULL;
	}

	return domain;
}

static int dpu95_irq_init(struct platform_device *pdev, struct dpu95_soc *dpu)
{
	struct irq_domain *parent_domain;
	struct device *dev = &pdev->dev;
	struct irq_chip_generic *gc;
	struct irq_chip_type *ct;
	int ret, i, j;

	parent_domain = dpu95_find_parent_irq_domain(dev);
	if (!parent_domain)
		return -ENODEV;

	for (i = 0; i < DPU95_COMCTRL_IRQ_IRQS; i++) {
		dpu->comctrl_irq[i] = platform_get_irq(pdev, dpu_comctrl_irq[i]);
		if (dpu->comctrl_irq[i] < 0)
			return dev_err_probe(dev, dpu->comctrl_irq[i],
					     "failed to get comctrl irq[%d]\n",
					     dpu_comctrl_irq[i]);
	}

	for (i = 0; i < DPU95_DISPLAY_IRQ0_IRQS; i++) {
		dpu->disp_irq0[i] = platform_get_irq(pdev, dpu_display_irq0[i]);
		if (dpu->disp_irq0[i] < 0)
			return dev_err_probe(dev, dpu->disp_irq0[i],
					     "failed to get display irq0[%d]\n",
					     dpu_display_irq0[i]);
	}

	for (i = 0; i < DPU95_DISPLAY_IRQ2_IRQS; i++) {
		dpu->disp_irq2[i] = platform_get_irq(pdev, dpu_display_irq2[i]);
		if (dpu->disp_irq2[i] < 0)
			return dev_err_probe(dev, dpu->disp_irq2[i],
					     "failed to get display irq2[%d]\n",
					     dpu_display_irq2[i]);
	}

	dpu->comctrl_irq_domain = irq_domain_add_linear(dev->of_node,
						      DPU95_IRQ_COUNT,
						      &irq_generic_chip_ops,
						      dpu);
	if (!dpu->comctrl_irq_domain) {
		dev_err(dev, "failed to add comctrl irq domain\n");
		return -ENODEV;
	}

	dpu->disp_irq0_domain = irq_domain_add_linear(dev->of_node,
						      DPU95_IRQ_COUNT,
						      &irq_generic_chip_ops,
						      dpu);
	if (!dpu->disp_irq0_domain) {
		dev_err(dev, "failed to add display irq0 domain\n");
		ret = -ENODEV;
		goto err0;
	}

	dpu->disp_irq2_domain = irq_domain_add_linear(dev->of_node,
						      DPU95_IRQ_COUNT,
						      &irq_generic_chip_ops,
						      dpu);
	if (!dpu->disp_irq2_domain) {
		dev_err(dev, "failed to add display irq2 domain\n");
		ret = -ENODEV;
		goto err1;
	}

	ret = irq_alloc_domain_generic_chips(dpu->comctrl_irq_domain, 32, 1,
					     "DPU COMCTRL IRQ",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dev, "failed to alloc generic irq chips for comctrl irq: %d\n",
			ret);
		goto err2;
	}

	ret = irq_alloc_domain_generic_chips(dpu->disp_irq0_domain, 32, 1,
					     "DPU DISP IRQ0",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dev, "failed to alloc generic irq chips for display irq0: %d\n",
			ret);
		goto err2;
	}

	ret = irq_alloc_domain_generic_chips(dpu->disp_irq2_domain, 32, 1,
					     "DPU DISP IRQ2",
					     handle_level_irq, 0, 0, 0);
	if (ret < 0) {
		dev_err(dev, "failed to alloc generic irq chips for display irq2: %d\n",
			ret);
		goto err2;
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i += 32) {
		gc = irq_get_domain_generic_chip(dpu->comctrl_irq_domain, i);
		gc->reg_base = dpu->comctrl_irq_reg;
		gc->unused = unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = INTERRUPTCLEAR(i / 32);
		ct->regs.mask = INTERRUPTENABLE(i / 32);
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i += 32) {
		gc = irq_get_domain_generic_chip(dpu->disp_irq0_domain, i);
		gc->reg_base = dpu->disp_irq0_reg;
		gc->unused = unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = INTERRUPTCLEAR(i / 32);
		ct->regs.mask = INTERRUPTENABLE(i / 32);
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i += 32) {
		gc = irq_get_domain_generic_chip(dpu->disp_irq2_domain, i);
		gc->reg_base = dpu->disp_irq2_reg;
		gc->unused = unused_irq[i / 32];
		ct = gc->chip_types;
		ct->chip.irq_ack = irq_gc_ack_set_bit;
		ct->chip.irq_mask = irq_gc_mask_clr_bit;
		ct->chip.irq_unmask = irq_gc_mask_set_bit;
		ct->regs.ack = INTERRUPTCLEAR(i / 32);
		ct->regs.mask = INTERRUPTENABLE(i / 32);
	}

	ret = pm_runtime_resume_and_get(parent_domain->pm_dev);
	if (ret < 0) {
		dev_err(dev, "failed to get parent irq domain RPM: %d\n", ret);
		goto err2;
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i++) {
		if (!dpu95_comctrl_irq_handler[i])
			continue;

		for (j = 0; j < DPU95_COMCTRL_IRQ_IRQS; j++) {
			if (dpu_comctrl_irq[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->comctrl_irq[j],
							 dpu95_comctrl_irq_handler[i],
							 dpu);
			break;
		}
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i++) {
		if (!dpu95_display_irq0_handler[i])
			continue;

		for (j = 0; j < DPU95_DISPLAY_IRQ0_IRQS; j++) {
			if (dpu_display_irq0[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq0[j],
							 dpu95_display_irq0_handler[i],
							 dpu);
			break;
		}
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i++) {
		if (!dpu95_display_irq2_handler[i])
			continue;

		for (j = 0; j < DPU95_DISPLAY_IRQ2_IRQS; j++) {
			if (dpu_display_irq2[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq2[j],
							 dpu95_display_irq2_handler[i],
							 dpu);
			break;
		}
	}

	pm_runtime_put(parent_domain->pm_dev);

	return 0;

err2:
	irq_domain_remove(dpu->disp_irq2_domain);
err1:
	irq_domain_remove(dpu->disp_irq0_domain);
err0:
	irq_domain_remove(dpu->comctrl_irq_domain);
	return ret;
}

static void devm_dpu95_irq_exit(void *data)
{
	struct irq_domain *parent_domain;
	struct dpu95_soc *dpu = data;
	unsigned int irq;
	int ret, i, j;

	parent_domain = dpu95_find_parent_irq_domain(dpu->dev);
	if (!parent_domain)
		return;

	ret = pm_runtime_resume_and_get(parent_domain->pm_dev);
	if (ret < 0) {
		dev_err(dpu->dev, "failed to get parent irq domain RPM: %d\n",
			ret);
		return;
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i++) {
		if (!dpu95_comctrl_irq_handler[i])
			continue;

		for (j = 0; j < DPU95_COMCTRL_IRQ_IRQS; j++) {
			if (dpu_comctrl_irq[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->comctrl_irq[j],
							 NULL, NULL);
			break;
		}
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i++) {
		if (!dpu95_display_irq0_handler[i])
			continue;

		for (j = 0; j < DPU95_DISPLAY_IRQ0_IRQS; j++) {
			if (dpu_display_irq0[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq0[j],
							 NULL, NULL);
			break;
		}
	}

	for (i = 0; i < DPU95_IRQ_COUNT; i++) {
		if (!dpu95_display_irq2_handler[i])
			continue;

		for (j = 0; j < DPU95_DISPLAY_IRQ2_IRQS; j++) {
			if (dpu_display_irq2[j] != i)
				continue;

			irq_set_chained_handler_and_data(dpu->disp_irq2[j],
							 NULL, NULL);
			break;
		}
	}

	pm_runtime_put(parent_domain->pm_dev);

	for (i = 0; i < DPU95_COMCTRL_IRQ_IRQS; i++) {
		irq = irq_linear_revmap(dpu->comctrl_irq_domain,
					dpu_comctrl_irq[i]);
		if (irq)
			irq_dispose_mapping(irq);
	}

	for (i = 0; i < DPU95_DISPLAY_IRQ0_IRQS; i++) {
		irq = irq_linear_revmap(dpu->disp_irq0_domain,
					dpu_display_irq0[i]);
		if (irq)
			irq_dispose_mapping(irq);
	}

	for (i = 0; i < DPU95_DISPLAY_IRQ2_IRQS; i++) {
		irq = irq_linear_revmap(dpu->disp_irq2_domain,
					dpu_display_irq2[i]);
		if (irq)
			irq_dispose_mapping(irq);
	}

	irq_domain_remove(dpu->comctrl_irq_domain);
	irq_domain_remove(dpu->disp_irq0_domain);
	irq_domain_remove(dpu->disp_irq2_domain);
}

void dpu95_submodules_hw_init(struct dpu95_soc *dpu)
{
	const struct dpu95_units *us;
	int i, j;

	dpu95_dm_extdst0_dm_allow_all(dpu);
	dpu95_dm_extdst1_dm_allow_all(dpu);
	dpu95_dm_extdst4_dm_allow_all(dpu);
	dpu95_dm_extdst5_dm_allow_all(dpu);

	dpu95_dm_extdst0_master(dpu);
	dpu95_dm_extdst1_master(dpu);
	dpu95_dm_extdst4_master(dpu);
	dpu95_dm_extdst5_master(dpu);

	for (i = 0; i < ARRAY_SIZE(dpu_all_units); i++) {
		us = dpu_all_units[i];

		for (j = 0; j < us->cnt; j++)
			us->hw_init(dpu, j);
	}
}

int dpu95_set_qos(struct dpu95_soc *dpu)
{
	int ret;

	ret = regmap_update_bits(dpu->regmap, QOS_SETTING,
				 DISPLAY_PANIC_QOS_MASK | DISPLAY_ARQOS_MASK,
				 DISPLAY_PANIC_QOS(0x3) | DISPLAY_ARQOS(0x3));
	if (ret < 0) {
		dev_err(dpu->dev, "failed to set QoS: %d\n", ret);
		return ret;
	}

	return 0;
}

static int dpu95_submodules_init(struct dpu95_soc *dpu, unsigned long dpu_base)
{
	const struct dpu95_units *us;
	unsigned long aux_ofs;
	int i, j, ret;

	for (i = 0; i < ARRAY_SIZE(dpu_all_units); i++) {
		us = dpu_all_units[i];

		for (j = 0; j < us->cnt; j++) {
			aux_ofs = us->aux_ofss ? dpu_base + us->aux_ofss[j] : 0;

			ret = us->init(dpu, j, us->ids[j], us->types[j],
				       aux_ofs, dpu_base + us->ofss[j]);
			if (ret) {
				dev_err(dpu->dev,
					"failed to initialize %s%d: %d\n",
					us->name, us->ids[j], ret);
				return ret;
			}
		}
	}

	return 0;
}

static int dpu95_get_layerblends_for_plane_grp(struct dpu95_soc *dpu,
					       struct dpu95_plane_res *res)
{
	int i, ret;

	for (i = 0; i < dpu_lbs.cnt; i++) {
		res->lb[i] = dpu95_lb_get(dpu, dpu_lbs.ids[i]);
		if (IS_ERR(res->lb[i])) {
			ret = PTR_ERR(res->lb[i]);
			dev_err(dpu->dev, "failed to get %s%d: %d\n",
				dpu_lbs.name, dpu_lbs.ids[i], ret);
			return ret;
		}
	}

	return 0;
}

static int dpu95_get_plane_grp_res(struct dpu95_soc *dpu,
				   struct dpu95_plane_grp *grp)
{
	struct dpu95_plane_res *res = &grp->res;
	int i, ret;

	ret = dpu95_get_layerblends_for_plane_grp(dpu, res);
	if (ret)
		return ret;

	for (i = 0; i < dpu_fys.cnt; i++) {
		res->fy[i] = dpu95_fy_get(dpu, dpu_fys.ids[i]);
		if (IS_ERR(res->fy[i])) {
			ret = PTR_ERR(res->fy[i]);
			dev_err(dpu->dev, "failed to get %s%d: %d\n",
				dpu_fys.name, dpu_fys.ids[i], ret);
			return ret;
		}
	}

	for (i = 0; i < dpu_fls.cnt; i++) {
		res->fl[i] = dpu95_fl_get(dpu, dpu_fls.ids[i]);
		if (IS_ERR(res->fl[i])) {
			ret = PTR_ERR(res->fl[i]);
			dev_err(dpu->dev, "failed to get %s%d: %d\n",
				dpu_fls.name, dpu_fls.ids[i], ret);
			return ret;
		}
	}

	INIT_LIST_HEAD(&grp->fu_list);

	for (i = dpu_fys.cnt - 1; i >= 0; i--)
		dpu95_fu_add_to_list(res->fy[i], &grp->fu_list);

	for (i = dpu_fls.cnt - 1; i >= 0; i--)
		dpu95_fu_add_to_list(res->fl[i], &grp->fu_list);

	return 0;
}

int dpu95_core_init(struct dpu95_drm_device *dpu_drm)
{
	struct drm_device *drm = &dpu_drm->base;
	struct device *dev = drm->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct dpu95_soc *dpu = &dpu_drm->dpu_soc;
	struct device_node *np = dev->of_node;
	unsigned long dpu_base;
	struct resource *res;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dpu_base = res->start;

	dpu->dev = dev;

	dpu->comctrl_irq_reg = devm_ioremap(dev, dpu_base + 0x1000, SZ_64);
	if (!dpu->comctrl_irq_reg)
		return -ENOMEM;

	dpu->dm_mask_reg = devm_ioremap(dev, dpu_base + 0x2000, SZ_64);
	if (!dpu->dm_mask_reg)
		return -ENOMEM;

	dpu->disp_irq0_reg = devm_ioremap(dev, dpu_base + 0x381000, SZ_64);
	if (!dpu->disp_irq0_reg)
		return -ENOMEM;

	dpu->disp_irq2_reg = devm_ioremap(dev, dpu_base + 0x3a1000, SZ_64);
	if (!dpu->disp_irq2_reg)
		return -ENOMEM;

	dpu->regmap = syscon_regmap_lookup_by_phandle(np, "nxp,blk-ctrl");
	if (IS_ERR(dpu->regmap)) {
		ret = PTR_ERR(dpu->regmap);
		dev_err_probe(dev, ret, "failed to get blk-ctrl regmap\n");
		return ret;
	}

	dpu->clk_axi = devm_clk_get(dev, "axi");
	if (IS_ERR(dpu->clk_axi))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_axi),
				     "failed to get AXI clock\n");

	dpu->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(dpu->clk_apb))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_apb),
				     "failed to get APB clock\n");

	dpu->clk_pix = devm_clk_get(dev, "pix");
	if (IS_ERR(dpu->clk_pix))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_pix),
				     "failed to get pixel clock\n");

	dpu->clk_ocram = devm_clk_get(dev, "ocram");
	if (IS_ERR(dpu->clk_ocram))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_ocram),
				     "failed to get ocram clock\n");

	dpu->clk_ldb = devm_clk_get(dev, "ldb");
	if (IS_ERR(dpu->clk_ldb))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_ldb),
				     "failed to get ldb clock\n");

	dpu->clk_ldb_vco = devm_clk_get(dev, "ldb_vco");
	if (IS_ERR(dpu->clk_ldb_vco))
		return dev_err_probe(dev, PTR_ERR(dpu->clk_ldb_vco),
				     "failed to get ldb_vco clock\n");

	ret = dpu95_submodules_init(dpu, dpu_base);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	ret = dpu95_get_plane_grp_res(dpu, &dpu_drm->dpu_plane_grp);
	if (ret)
		return ret;

	ret = dpu95_irq_init(pdev, dpu);
	if (ret)
		return ret;

	return devm_add_action_or_reset(dev, devm_dpu95_irq_exit, dpu);
}
