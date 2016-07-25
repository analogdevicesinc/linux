/*
 * AXI clk div driver
 *
 * Copyright 2016 Analog Devices Inc.
 * Author: Michael Hennerich <michael.hennerich@analog.com>
 *
 * Licensed under the GPL-2.
 *
 */

#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>

#define ADI_REG_VERSION		0x0000				/*Version and Scratch Registers */
#define ADI_VERSION(x)		(((x) & 0xffffffff) << 0)	/* RO, Version number. */
#define VERSION_IS(x,y,z)	((x) << 16 | (y) << 8 | (z))
#define ADI_REG_ID		0x0004			 	/*Version and Scratch Registers */
#define ADI_ID(x)		(((x) & 0xffffffff) << 0)   	/* RO, Instance identifier number. */
#define ADI_REG_SCRATCH		0x0008			 	/*Version and Scratch Registers */
#define ADI_SCRATCH(x)		(((x) & 0xffffffff) << 0)	/* RW, Scratch register. */

#define ADI_REG_RSTN		0x0040
#define ADI_RSTN		(1 << 0)

#define ADI_REG_DIV		0x0044
#define ADI_REG_WIDTH		0x0048

static void __init axi_div_clk_setup(struct device_node *node)
{
	int num_parents;
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name;
	void __iomem *reg = NULL;
	u32 tmp;

	num_parents = of_clk_get_parent_count(node);
	if (num_parents < 1) {
		pr_err("%s: no parent found", clk_name);
		return;
	}

	reg = of_iomap(node, 0);
	if (reg == NULL) {
		pr_err("%s: failed to map divide register", clk_name);
		goto error;
	}

	parent_name = of_clk_get_parent_name(node, 0);

	of_property_read_u32(node, "adi,pulse-width", &tmp);
	clk_writel(tmp, reg + ADI_REG_WIDTH);
	
	of_property_read_u32(node, "adi,divider", &tmp);
	clk_writel(tmp, reg + ADI_REG_DIV);
	clk_writel(ADI_RSTN, reg + ADI_REG_RSTN);

	clk = clk_register_divider(NULL, clk_name, parent_name,
				   0, reg + ADI_REG_DIV, 0, 32,
			    CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_ONE_BASED , NULL);
	if (!IS_ERR(clk)) {
		of_clk_add_provider(node, of_clk_src_simple_get, clk);
		return;
	}
	pr_err("%s: failed to register %s div clock (%ld)\n",
	       __func__, clk_name, PTR_ERR(clk));
error:
	if (reg)
		iounmap(reg);
}

CLK_OF_DECLARE(axi_div_clk, "adi,axi-clkdiv-1.00.a", axi_div_clk_setup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Driver for the Analog Devices' AXI clkgen pcore clock generator");
