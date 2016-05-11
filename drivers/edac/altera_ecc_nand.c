/*
 * Copyright (C) 2015 Altera Corporation
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

#include "altera_edac.h"

#ifdef CONFIG_EDAC_DEBUG
static struct edac_dev_sysfs_attribute altr_a10_nand_sysfs_attributes[] = {
	{
		.attr = { .name = "altr_nand_trigger",
			  .mode = (S_IRUGO | S_IWUSR) },
		.show = NULL,
		.store = altr_a10_ecc_mgr_trig
	},
	{
		.attr = { .name = NULL }
	}
};
#endif	/* #ifdef CONFIG_EDAC_DEBUG */

const struct ecc_mgr_prv_data a10_nandecc_data = {
	.setup = altr_a10_ecc_dependencies,
	.ce_clear_mask = ALTR_A10_ECC_CEA_INTSTAT_MASK,
	.ue_clear_mask = ALTR_A10_ECC_UEA_INTSTAT_MASK,
	.clear_mask_offs = ALTR_A10_ECC_INTSTAT_OFFSET,
	.ce_status_mask = ALTR_A10_ECC_CEA_INTSTAT_MASK,
	.ue_status_mask = ALTR_A10_ECC_UEA_INTSTAT_MASK,
	.status_mask_offs = ALTR_A10_ECC_INTSTAT_OFFSET,
#ifdef CONFIG_EDAC_DEBUG
	.eccmgr_sysfs_attr = altr_a10_nand_sysfs_attributes,
	.ecc_enable_mask = ALTR_A10_ECC_EN_CTL_MASK,
	.enable_mask_offs = ALTR_A10_ECC_CTL_OFFSET,
	.ce_set_mask = ALTR_A10_ECC_CEA_INTTEST_MASK,
	.ue_set_mask = ALTR_A10_ECC_UEA_INTTEST_MASK,
	.set_mask_offs = ALTR_A10_ECC_INTTEST_OFFSET,
#endif
};
