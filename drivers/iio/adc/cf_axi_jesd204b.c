/*
 * ADI AXI-JESD204B Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include "cf_axi_jesd204b.h"

struct jesd204b_state {
	struct device 			*dev;
	void __iomem			*regs;
};

/*
 * IO accessors
 */

static inline void jesd204b_write(struct jesd204b_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int jesd204b_read(struct jesd204b_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}


static int __devinit jesd204b_of_probe(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct jesd204b_state *st;
	struct resource r_mem; /* IO mem resources */
	resource_size_t remap_size, phys_addr;
	unsigned frmcnt, bytecnt;
	int ret;

	dev_info(dev, "Device Tree Probing \'%s\'\n",
		 op->dev.of_node->name);

	st = devm_kzalloc(&op->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	st->dev = dev;
	dev_set_drvdata(dev, st);

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		return ret;
	}

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}

	jesd204b_write(st, AXI_JESD204B_REG_TEST_MODE,
		       AXI_JESD204B_REG_TEST_MODE_JESD_RESET |
		       AXI_JESD204B_REG_TEST_MODE_GTX_RESET);

	ret = of_property_read_u32(op->dev.of_node,
				   "jesd,frames-per-multiframe", &frmcnt);
	if (ret)
		goto failed2;

	ret = of_property_read_u32(op->dev.of_node,
				   "jesd,bytes-per-frame", &bytecnt);
	if (ret)
		goto failed2;

	jesd204b_write(st, AXI_JESD204B_REG_CTRL,
		       (of_property_read_bool(op->dev.of_node, "jesd,scramble_en") ?
		       AXI_JESD204B_CTRL_SCR_EN : 0) |
		       (of_property_read_bool(op->dev.of_node, "jesd,lanesync_en") ?
		       AXI_JESD204B_CTRL_LANESYNC_EN : 0));

	jesd204b_write(st, AXI_JESD204B_REG_FRMCTRL,
		       AXI_JESD204B_FRMCTRL_FRMCNT(frmcnt - 1) |
		       AXI_JESD204B_FRMCTRL_BYTECNT(bytecnt - 1));

	dev_info(dev, "AXI-JESD204B (0x%X) at 0x%08llX mapped to 0x%p,",
		 jesd204b_read(st, AXI_JESD204B_REG_VERSION),
		 (unsigned long long)phys_addr, st->regs);

	return 0;

failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	dev_set_drvdata(dev, NULL);

	return ret;
}

/**
 * jesd204b_of_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int __devexit jesd204b_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct jesd204b_state *st = dev_get_drvdata(dev);


	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	dev_set_drvdata(dev, NULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id jesd204b_of_match[] __devinitconst = {
	{ .compatible = "xlnx,axi-jesd204b-rx2-1.00.a", },
	{ .compatible = "xlnx,axi-jesd204b-rx4-1.00.a", },
{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, jesd204b_of_match);

static struct platform_driver jesd204b_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = jesd204b_of_match,
	},
	.probe		= jesd204b_of_probe,
	.remove		= __devexit_p(jesd204b_of_remove),
};

module_platform_driver(jesd204b_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices CF FFT");
MODULE_LICENSE("GPL v2");
