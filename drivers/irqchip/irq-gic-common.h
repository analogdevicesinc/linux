/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2002 ARM Limited, All Rights Reserved.
 */

#ifndef _IRQ_GIC_COMMON_H
#define _IRQ_GIC_COMMON_H

#include <linux/of.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>
#include <linux/irqchip/arm-gic-common.h>

struct gic_quirk {
	const char *desc;
	const char *compatible;
	const char *property;
	bool (*init)(void *data);
	u32 iidr;
	u32 mask;
};

int gic_configure_irq(unsigned int irq, unsigned int type,
                       void __iomem *base);
void gic_dist_config(void __iomem *base, int gic_irqs, u8 priority);
void gic_cpu_config(void __iomem *base, int nr, u8 priority);
void gic_enable_quirks(u32 iidr, const struct gic_quirk *quirks,
		void *data);
void gic_enable_of_quirks(const struct device_node *np,
			  const struct gic_quirk *quirks, void *data);

extern const struct msi_parent_ops gic_v3_its_msi_parent_ops;

#define RDIST_FLAGS_PROPBASE_NEEDS_FLUSHING    (1 << 0)
#define RDIST_FLAGS_RD_TABLES_PREALLOCATED     (1 << 1)
#define RDIST_FLAGS_FORCE_NON_SHAREABLE        (1 << 2)

#endif /* _IRQ_GIC_COMMON_H */
