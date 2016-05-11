/*
 *  Copyright (C) 2014 Altera Corporation
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
#include <asm/cacheflush.h>

#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/edac.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>

#include "altera_edac.h"
#include "edac_core.h"
#include "edac_module.h"

/* MPU L2 Register Defines */
#define ALTR_MPUL2_CONTROL_OFFSET	0x100
#define ALTR_MPUL2_CTL_CACHE_EN_MASK	0x00000001

/* L2 ECC Management Group Defines */
#define ALTR_MAN_GRP_L2_ECC_OFFSET	0x00
#define ALTR_L2_ECC_EN_MASK		0x00000001
#define ALTR_L2_ECC_INJS_MASK		0x00000002
#define ALTR_L2_ECC_INJD_MASK		0x00000004

/* Arria 10 L2 ECC Management Group Defines */
#define ALTR_A10_L2_ECC_CTL_OFFSET	0x10
#define ALTR_A10_L2_ECC_EN_CTL_MASK	0x00000001

#define ALTR_A10_L2_ECC_STAT_OFFSET	0xa4
#define ALTR_A10_L2_ECC_CE_STAT_MASK	0x00008000
#define ALTR_A10_L2_ECC_UE_STAT_MASK	0x80000000

#define ALTR_A10_L2_ECC_CLR_OFFSET	0xa8
#define ALTR_A10_L2_ECC_CE_CLR_MASK	0x00008000
#define ALTR_A10_L2_ECC_UE_CLR_MASK	0x80000000

#define ALTR_A10_L2_ECC_INJ_OFFSET	0x10
#define ALTR_A10_L2_ECC_CE_INJ_MASK	0x00000101
#define ALTR_A10_L2_ECC_UE_INJ_MASK	0x00010101

#ifdef CONFIG_EDAC_DEBUG
static void *l2_init_mem(size_t size, void **other)
{
	struct device *dev = *other;
	void *ptemp = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!ptemp)
		return NULL;

	wmb();
	flush_cache_all();

	return ptemp;
}

static void l2_free_mem(void *p, size_t size, void *other)
{
	struct device *dev = other;
	if (dev && p)
		devm_kfree(dev, p);
}

static struct edac_dev_sysfs_attribute altr_l2_sysfs_attributes[] = {
	{
		.attr = { .name = "altr_l2_trigger",
			  .mode = (S_IRUGO | S_IWUSR) },
		.show = NULL,
		.store = altr_ecc_mgr_trig
	},
	{
		.attr = {.name = NULL }
	}
};
#endif	/* #ifdef CONFIG_EDAC_DEBUG */

/*
 * altr_l2_dependencies()
 *	Test for L2 cache ECC dependencies upon entry because
 *	the preloader/UBoot should have initialized the L2
 *	memory and enabled the ECC.
 *	Can't turn on ECC here because accessing un-initialized
 *	memory will cause CE/UE errors possibly causing an ABORT.
 *	Bail if ECC is not on.
 *	Test For 1) L2 ECC is enabled and 2) L2 Cache is enabled.
 */
static int altr_l2_dependencies(struct platform_device *pdev,
				void __iomem *base)
{
	u32 control;
	struct device_node *np;
	void __iomem *l2_vbase;
	void __iomem *en_addr;
	u32 en_mask;

	if (of_machine_is_compatible("altr,socfpga-arria10")) {
		en_addr = (void __iomem *)((uintptr_t)base +
					   ALTR_A10_L2_ECC_CTL_OFFSET);
		en_mask = ALTR_A10_L2_ECC_EN_CTL_MASK;
	} else {
		en_addr = (void __iomem *)((uintptr_t)base +
					   ALTR_MAN_GRP_L2_ECC_OFFSET);
		en_mask = ALTR_L2_ECC_EN_MASK;
	}

	control = readl(en_addr) & en_mask;
	if (!control) {
		dev_err(&pdev->dev, "L2: No ECC present, or ECC disabled\n");
		return -ENODEV;
	}

	np = of_find_compatible_node(NULL, NULL, "arm,pl310-cache");
	if (!np) {
		dev_err(&pdev->dev,
			"L2 ECC: Unable to find arm,pl310-cache in dtb\n");
		return -ENODEV;
	}

	l2_vbase = of_iomap(np, 0);
	if (!l2_vbase) {
		dev_err(&pdev->dev,
			"L2 ECC: Unable to find arm,pl310-cache mapping in dtb\n");
		return -ENODEV;
	}

	control = readl(l2_vbase + ALTR_MPUL2_CONTROL_OFFSET);
	if (!(control & ALTR_MPUL2_CTL_CACHE_EN_MASK)) {
		dev_err(&pdev->dev, "L2: Cache disabled\n");
		return -ENODEV;
	}

	return 0;
}

const struct ecc_mgr_prv_data l2ecc_data = {
	.setup = altr_l2_dependencies,
	.ce_clear_mask = 0,
	.ue_clear_mask = 0,
#ifdef CONFIG_EDAC_DEBUG
	.eccmgr_sysfs_attr = altr_l2_sysfs_attributes,
	.init_mem = l2_init_mem,
	.free_mem = l2_free_mem,
	.ecc_enable_mask = ALTR_L2_ECC_EN_MASK,
	.ce_set_mask = (ALTR_L2_ECC_EN_MASK | ALTR_L2_ECC_INJS_MASK),
	.ue_set_mask = (ALTR_L2_ECC_EN_MASK | ALTR_L2_ECC_INJD_MASK),
	.trig_alloc_sz = 5000,
#endif
};

const struct ecc_mgr_prv_data a10_l2ecc_data = {
	.setup = altr_l2_dependencies,
	.ce_clear_mask = ALTR_A10_L2_ECC_CE_CLR_MASK,
	.ue_clear_mask = ALTR_A10_L2_ECC_UE_CLR_MASK,
	.clear_mask_offs = ALTR_A10_L2_ECC_CLR_OFFSET,
	.ce_status_mask = ALTR_A10_L2_ECC_CE_STAT_MASK,
	.ue_status_mask = ALTR_A10_L2_ECC_UE_STAT_MASK,
	.status_mask_offs = ALTR_A10_L2_ECC_STAT_OFFSET,
#ifdef CONFIG_EDAC_DEBUG
	.eccmgr_sysfs_attr = altr_l2_sysfs_attributes,
	.init_mem = l2_init_mem,
	.free_mem = l2_free_mem,
	.ecc_enable_mask = ALTR_A10_L2_ECC_EN_CTL_MASK,
	.enable_mask_offs = ALTR_A10_L2_ECC_CTL_OFFSET,
	.ce_set_mask = ALTR_A10_L2_ECC_CE_INJ_MASK,
	.ue_set_mask = ALTR_A10_L2_ECC_UE_INJ_MASK,
	.set_mask_offs = ALTR_A10_L2_ECC_INJ_OFFSET,
	.trig_alloc_sz = 5000,
#endif
};
