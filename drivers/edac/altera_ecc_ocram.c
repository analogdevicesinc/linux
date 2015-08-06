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

#include <linux/ctype.h>
#include <linux/edac.h>
#include <linux/genalloc.h>
#include <linux/of.h>
#include "altera_edac.h"

/* OCRAM ECC Management Group Defines */
#define ALTR_MAN_GRP_OCRAM_ECC_OFFSET	0x00
#define ALTR_OCR_ECC_EN_MASK		0x00000001
#define ALTR_OCR_ECC_INJS_MASK		0x00000002
#define ALTR_OCR_ECC_INJD_MASK		0x00000004
#define ALTR_OCR_ECC_SERR_MASK		0x00000008
#define ALTR_OCR_ECC_DERR_MASK		0x00000010

/* Arria 10 OCRAM ECC Management Group Defines */
#define ALTR_A10_OCR_ECC_CTL_OFFSET	0x08
#define ALTR_A10_OCR_ECC_EN_CTL_MASK	0x00000001

#define ALTR_A10_OCR_ECC_STAT_OFFSET	0x20
#define ALTR_A10_OCR_ECC_CE_STAT_MASK	0x00000001
#define ALTR_A10_OCR_ECC_UE_STAT_MASK	0x00000100

#define ALTR_A10_OCR_ECC_CLR_OFFSET	0x20
#define ALTR_A10_OCR_ECC_CE_CLR_MASK	0x00000001
#define ALTR_A10_OCR_ECC_UE_CLR_MASK	0x00000100

#define ALTR_A10_OCR_ECC_INJ_OFFSET	0x24
#define ALTR_A10_OCR_ECC_CE_INJ_MASK	0x00000001
#define ALTR_A10_OCR_ECC_UE_INJ_MASK	0x00000100

#ifdef CONFIG_EDAC_DEBUG
static void *ocram_init_mem(size_t size, void **other)
{
	struct device_node *np;
	struct gen_pool *gp;
	void *sram_addr;
	const char *compat = "altr,ocram-edac";

	if (of_machine_is_compatible("altr,socfpga-arria10"))
		compat = "altr,a10-ocram-edac";

	np = of_find_compatible_node(NULL, NULL, compat);
	if (!np)
		return NULL;

	gp = of_get_named_gen_pool(np, "iram", 0);
	if (!gp)
		return NULL;
	*other = gp;

	sram_addr = (void *)gen_pool_alloc(gp, size);
	if (!sram_addr)
		return NULL;

	memset(sram_addr, 0, size);
	wmb();	/* Ensure data is written out */

	return sram_addr;
}

static void ocram_free_mem(void *p, size_t size, void *other)
{
	gen_pool_free((struct gen_pool *)other, (u32)p, size);
}

static struct edac_dev_sysfs_attribute altr_ocr_sysfs_attributes[] = {
	{
		.attr = { .name = "altr_ocram_trigger",
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
 * altr_ocram_dependencies()
 *	Test for OCRAM ECC dependencies upon entry because
 *	the kernel startup code should have initialized the OCRAM
 *	memory and enabled the ECC.
 *	Fail if ECC is not on.
 *	Test for OCRAM ECC is enabled.
 */
static int altr_ocram_dependencies(struct platform_device *pdev,
				   void __iomem *base)
{
	u32 control;
	void __iomem *en_addr;
	u32 en_mask;

	if (of_machine_is_compatible("altr,socfpga-arria10")) {
		en_addr = (void __iomem *)((uintptr_t)base +
					   ALTR_A10_OCR_ECC_CTL_OFFSET);
		en_mask = ALTR_A10_OCR_ECC_EN_CTL_MASK;
	} else {
		en_addr = (void __iomem *)((uintptr_t)base +
					   ALTR_MAN_GRP_OCRAM_ECC_OFFSET);
		en_mask = ALTR_OCR_ECC_EN_MASK;
	}

	control = readl(en_addr) & en_mask;
	if (!control) {
		dev_err(&pdev->dev, "OCRAM: No ECC present, or ECC disabled\n");
		return -ENODEV;
	}

	return 0;
}

const struct ecc_mgr_prv_data ocramecc_data = {
	.ce_clear_mask = (ALTR_OCR_ECC_EN_MASK | ALTR_OCR_ECC_SERR_MASK),
	.ue_clear_mask = (ALTR_OCR_ECC_EN_MASK | ALTR_OCR_ECC_DERR_MASK),
#ifdef CONFIG_EDAC_DEBUG
	.eccmgr_sysfs_attr = altr_ocr_sysfs_attributes,
	.init_mem = ocram_init_mem,
	.free_mem = ocram_free_mem,
	.ecc_enable_mask = ALTR_OCR_ECC_EN_MASK,
	.ce_set_mask = (ALTR_OCR_ECC_EN_MASK | ALTR_OCR_ECC_INJS_MASK),
	.ue_set_mask = (ALTR_OCR_ECC_EN_MASK | ALTR_OCR_ECC_INJD_MASK),
	.trig_alloc_sz = (32 * sizeof(u32)),
#endif
};

const struct ecc_mgr_prv_data a10_ocramecc_data = {
	.setup = altr_ocram_dependencies,
	.ce_clear_mask = ALTR_A10_OCR_ECC_CE_CLR_MASK,
	.ue_clear_mask = ALTR_A10_OCR_ECC_UE_CLR_MASK,
	.clear_mask_offs = ALTR_A10_OCR_ECC_CLR_OFFSET,
	.ce_status_mask = ALTR_A10_OCR_ECC_CE_STAT_MASK,
	.ue_status_mask = ALTR_A10_OCR_ECC_UE_STAT_MASK,
	.status_mask_offs = ALTR_A10_OCR_ECC_STAT_OFFSET,
#ifdef CONFIG_EDAC_DEBUG
	.eccmgr_sysfs_attr = altr_ocr_sysfs_attributes,
	.init_mem = ocram_init_mem,
	.free_mem = ocram_free_mem,
	.ecc_enable_mask = ALTR_A10_OCR_ECC_EN_CTL_MASK,
	.enable_mask_offs = ALTR_A10_OCR_ECC_CTL_OFFSET,
	.ce_set_mask = ALTR_A10_OCR_ECC_CE_INJ_MASK,
	.ue_set_mask = ALTR_A10_OCR_ECC_UE_INJ_MASK,
	.set_mask_offs = ALTR_A10_OCR_ECC_INJ_OFFSET,
	.trig_alloc_sz = (32 * sizeof(u32)),
#endif
};
