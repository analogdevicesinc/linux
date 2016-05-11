/*
 * Copyright (C) 2015 Altera Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms and conditions of the GNU General Public License,
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

#include <linux/of.h>

#include "ecc.h"

#define SOCFPGA_A10_NAND_ECC_INTMASK	BIT(11)
#define SOCFPGA_A10_NAND_RD_ECC_INTMASK	BIT(13)
#define SOCFPGA_A10_NAND_WR_ECC_INTMASK	BIT(12)

static int socfpga_init_a10_nand_ecc(const char *compat, u32 intmask)
{
	struct device_node *np;
	int ret;

	np = of_find_compatible_node(NULL, NULL, compat);
	if (!np) {
		pr_err("SOCFPGA: Unable to find %s in dtb\n", compat);
		ret = -ENODEV;
		goto out;
	}

	ret = socfpga_init_a10_ecc(np, intmask, 0);

out:
	of_node_put(np);
	return ret;
}

void socfpga_init_nand_ecc(void)
{
	if (of_machine_is_compatible("altr,socfpga-arria10")) {
		if (socfpga_init_a10_nand_ecc("altr,a10-nand-buf-edac",
					      SOCFPGA_A10_NAND_ECC_INTMASK))
			return;
		if (socfpga_init_a10_nand_ecc("altr,a10-nand-rd-edac",
					      SOCFPGA_A10_NAND_RD_ECC_INTMASK))
			return;
		if (socfpga_init_a10_nand_ecc("altr,a10-nand-wr-edac",
					      SOCFPGA_A10_NAND_WR_ECC_INTMASK))
			return;
		goto success;
	}

	return;

success:
	pr_alert("SOCFPGA: Success Initializing NAND ECC\n");
}
