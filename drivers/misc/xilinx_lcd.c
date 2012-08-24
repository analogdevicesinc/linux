/*
 * Xilinx LCD Module Driver
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
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#define INIT_DELAY 10
#define INST_DELAY 1
#define DATA_DELAY 1
#define DEF_DELAY  1

#define SIGNAL_RW	0x10
#define SIGNAL_RS	0x20
#define SIGNAL_ENABLE 	0x40

/* Register Offset Definitions */
#define XGPIO_DATA_OFFSET	0x0	/* Data register  */
#define XGPIO_TRI_OFFSET	0x4	/* I/O direction register  */

/*
 * Usage Example:
 * ip addr show eth0 | sed -n '3p' | awk '{ print $2 }' | cut -f 1 -d "/" > line1
 * (/sys/bus/platform/devices/40000000.gpio/line1)
 */

struct xlnx_lcd_state {
	struct mutex lock;
	void __iomem *regs;
};

struct xlnx_lcd_state *st;

/*
 * IO accessors
 */

static inline void xlnx_lcd_write(struct xlnx_lcd_state *st, unsigned reg,
				  unsigned val)
{
      iowrite32(val, st->regs + reg);
}

static void xlnx_lcd_setup(void)
{
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, 0x03);
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, SIGNAL_ENABLE | 0x3);
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, 0x03);
	msleep(INIT_DELAY);
}

static void xlnx_lcd_write_instr(unsigned inst)
{
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET,
		       SIGNAL_ENABLE | ((inst >> 4) & 0x0F));
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, ((inst >> 4) & 0x0F));
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, SIGNAL_ENABLE | (inst & 0x0F));
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, (inst & 0x0F));
	msleep(INST_DELAY);
}

static void xlnx_lcd_write_char(char data)
{
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET,
		       SIGNAL_RS | SIGNAL_ENABLE | ((data >> 4) & 0x0F));
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET,
		       SIGNAL_RS | ((data >> 4) & 0x0F));
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET,
		       SIGNAL_RS | SIGNAL_ENABLE | (data & 0x0F));
	udelay(DEF_DELAY);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, SIGNAL_RS | (data & 0x0F));
	msleep(DATA_DELAY);
}

static void xlnx_lcd_init(void)
{
	/* init GPIO */
	xlnx_lcd_write(st, XGPIO_TRI_OFFSET, 0x00);
	xlnx_lcd_write(st, XGPIO_DATA_OFFSET, 0x00);

	/* init LCD */
	msleep(15);
	xlnx_lcd_setup();
	msleep(5);
	xlnx_lcd_setup();
	udelay(100);
	xlnx_lcd_setup();
	xlnx_lcd_write_instr(0x22);
	xlnx_lcd_write_instr(0x08);
	xlnx_lcd_write_instr(0x01);
	xlnx_lcd_write_instr(0x06);
	xlnx_lcd_write_instr(0x0E);
}

static void xlnx_lcd_print_str(const char *str)
{
	int i, eol = 0;

	for (i = 0; i < 16; i++) {

		if (eol) {
			xlnx_lcd_write_char(' ');
			continue;
		}

		switch (str[i]) {
		case 0:
		case '\n':
			eol = 1;
			xlnx_lcd_write_char(' ');
			break;
		default:
			xlnx_lcd_write_char(str[i]);
		}
	}
}

static ssize_t xlnx_lcd_line1_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	mutex_lock(&st->lock);
	xlnx_lcd_write_instr(0x02); /* HOME */
	xlnx_lcd_print_str(buf);
	mutex_unlock(&st->lock);
	return count;
}

static ssize_t xlnx_lcd_line2_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	mutex_lock(&st->lock);
	xlnx_lcd_write_instr(0x80 | 40); /* Second Line */
	xlnx_lcd_print_str(buf);
	mutex_unlock(&st->lock);
	return count;
}

static DEVICE_ATTR(line1, S_IWUSR, NULL, xlnx_lcd_line1_store);
static DEVICE_ATTR(line2, S_IWUSR, NULL, xlnx_lcd_line2_store);

static struct attribute *xlnx_lcd_attributes[] = {
	&dev_attr_line1.attr,
	&dev_attr_line2.attr,
	NULL
};

static const struct attribute_group xlnx_lcd_attr_group = {
	.attrs = xlnx_lcd_attributes,
};

static int __devinit xlnx_lcd_of_probe(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem;	/* IO mem resources */
	resource_size_t remap_size, phys_addr;
	int ret;

	dev_info(dev, "Device Tree Probing \'%s\'\n", op->dev.of_node->name);

	st = devm_kzalloc(&op->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	dev_set_drvdata(dev, st);
	mutex_init(&st->lock);

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

	xlnx_lcd_init();

	ret = sysfs_create_group(&dev->kobj, &xlnx_lcd_attr_group);

	dev_info(dev, "LCD 0x%08llX mapped to 0x%p\n",
		 (unsigned long long)phys_addr, st->regs);

	return 0;

failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	dev_set_drvdata(dev, NULL);

	return ret;
}

static int __devexit xlnx_lcd_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem;	/* IO mem resources */
	struct xlnx_lcd_state *st = dev_get_drvdata(dev);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	sysfs_remove_group(&dev->kobj, &xlnx_lcd_attr_group);

	dev_set_drvdata(dev, NULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id xlnx_lcd_of_match[] __devinitconst = {
	{.compatible = "xlnx,axi-xlnx-lcd-1.00.a",},
	{ /* end of list */ },
};

MODULE_DEVICE_TABLE(of, xlnx_lcd_of_match);

static struct platform_driver xlnx_lcd_of_driver = {
	.driver = {
		   .name = KBUILD_MODNAME,
		   .owner = THIS_MODULE,
		   .of_match_table = xlnx_lcd_of_match,
		   },
	.probe = xlnx_lcd_of_probe,
	.remove = __devexit_p(xlnx_lcd_of_remove),
};

module_platform_driver(xlnx_lcd_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Xilinx LCD Module Driver");
MODULE_LICENSE("GPL v2");
