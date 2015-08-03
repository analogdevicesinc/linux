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

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include "core.h"

/* A10 Error Checking and Correction (ECC) Controller registers offset */
#define SOCFPGA_A10_ECC_CTRL_OFFS	0x08
#define SOCFPGA_A10_ECC_INITSTAT_OFFS	0x0c
#define SOCFPGA_A10_ECC_ERRINTEN_OFFS	0x10
#define SOCFPGA_A10_ECC_INTSTAT_OFFS	0x20

/* A10 ECC Controller memory initialization timeout */
#define SOCFPGA_A10_ECC_INIT_WATCHDOG	10000

/* A10 ECC Controller control bits */
#define SOCFPGA_A10_ECC_EN		BIT(0)
#define SOCFPGA_A10_ECC_SERRINTEN	BIT(0)

/* A10 ECC PORTA memory control bits */
#define SOCFPGA_A10_ECC_INITA		BIT(16)
#define SOCFPGA_A10_ECC_INITCOMPLETEA	BIT(0)
#define SOCFPGA_A10_ECC_SERRPENA	BIT(0)
#define SOCFPGA_A10_ECC_DERRPENA	BIT(8)
#define SOCFPGA_A10_ECC_ERRPENA_MASK	(SOCFPGA_A10_ECC_SERRPENA |	\
					 SOCFPGA_A10_ECC_DERRPENA)

/* A10 ECC PORTB memory control bits */
#define SOCFPGA_A10_ECC_INITB		BIT(24)
#define SOCFPGA_A10_ECC_INITCOMPLETEB	BIT(8)
#define SOCFPGA_A10_ECC_SERRPENB	BIT(16)
#define SOCFPGA_A10_ECC_DERRPENB	BIT(24)
#define SOCFPGA_A10_ECC_ERRPENB_MASK	(SOCFPGA_A10_ECC_SERRPENB |	\
					 SOCFPGA_A10_ECC_DERRPENB)

static inline u32 ecc_read(void __iomem *dev, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)dev + offs);

	return readl(paddr);
}

static inline void ecc_write(u32 val, void __iomem *dev, size_t offs)
{
	void __iomem *paddr = (void __iomem *)((uintptr_t)dev + offs);

	writel(val, paddr);
}

static void ecc_set_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = ecc_read(ioaddr, offs);

	value |= bit_mask;
	ecc_write(value, ioaddr, offs);
}

static void ecc_clear_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = ecc_read(ioaddr, offs);

	value &= ~bit_mask;
	ecc_write(value, ioaddr, offs);
}

static int ecc_test_bit(void __iomem *ioaddr, size_t offs, u32 bit_mask)
{
	u32 value = ecc_read(ioaddr, offs);

	return (value & bit_mask) ? 1 : 0;
}

/*
 * This function uses the memory initialization block in the A10 ECC controller
 * to initialize the entire memory data and ECC bits.
 * The initialization block clears the memory data.
 */
static int init_memory_port(void __iomem *ioaddr, int port)
{
	int limit = SOCFPGA_A10_ECC_INIT_WATCHDOG;
	u32 init_mask = SOCFPGA_A10_ECC_INITA;
	u32 stat_mask = SOCFPGA_A10_ECC_INITCOMPLETEA;
	u32 clear_mask = SOCFPGA_A10_ECC_ERRPENA_MASK;
	int ret = 0;

	if (port) {
		init_mask = SOCFPGA_A10_ECC_INITB;
		stat_mask = SOCFPGA_A10_ECC_INITCOMPLETEB;
		clear_mask = SOCFPGA_A10_ECC_ERRPENB_MASK;
	}

	ecc_set_bit(ioaddr, SOCFPGA_A10_ECC_CTRL_OFFS, init_mask);
	while (limit--) {
		if (ecc_test_bit(ioaddr, SOCFPGA_A10_ECC_INITSTAT_OFFS,
				 stat_mask))
			break;
		udelay(1);
	}
	if (limit < 0)
		ret = -EBUSY;

	/* Clear any pending ECC interrupts */
	ecc_write(clear_mask, ioaddr, SOCFPGA_A10_ECC_INTSTAT_OFFS);

	return ret;
}

int socfpga_init_a10_ecc(struct device_node *np, u32 intmask, int dual_port)
{
	void __iomem *ecc_ioaddr;
	int ret = 0;

	if (!sys_manager_base_addr) {
		pr_err("SOCFPGA: sys-mgr is not initialized\n");
		ret = -EINVAL;
		goto out;
	}

	ecc_ioaddr = of_iomap(np, 0);
	if (!ecc_ioaddr) {
		pr_err("ECC: Unable to find %s mapping in dtb\n",
		       np->full_name);
		ret = -EINVAL;
		goto out;
	}

	/* Disable ECC */
	writel(intmask, sys_manager_base_addr +
	       SOCFPGA_A10_SYSMGR_ECC_INTMASK_SET);
	ecc_clear_bit(ecc_ioaddr, SOCFPGA_A10_ECC_ERRINTEN_OFFS,
		      SOCFPGA_A10_ECC_SERRINTEN);
	ecc_clear_bit(ecc_ioaddr, SOCFPGA_A10_ECC_CTRL_OFFS,
		      SOCFPGA_A10_ECC_EN);

	/* Use HW initialization block to clear memory */
	ret = init_memory_port(ecc_ioaddr, 0);
	if (ret) {
		pr_err("ECC: cannot init %s PORTA memory\n", np->full_name);
		goto out;
	}

	if (dual_port) {
		ret = init_memory_port(ecc_ioaddr, 1);
		if (ret) {
			pr_err("ECC: cannot init %s PORTB memory\n",
			       np->full_name);
			goto out;
		}
	}

	/* Enable ECC */
	ecc_set_bit(ecc_ioaddr, SOCFPGA_A10_ECC_CTRL_OFFS, SOCFPGA_A10_ECC_EN);
	ecc_set_bit(ecc_ioaddr, SOCFPGA_A10_ECC_ERRINTEN_OFFS,
		    SOCFPGA_A10_ECC_SERRINTEN);
	writel(intmask, sys_manager_base_addr +
	       SOCFPGA_A10_SYSMGR_ECC_INTMASK_CLR);

	/* Ensure data is written out */
	wmb();

out:
	return ret;
}
