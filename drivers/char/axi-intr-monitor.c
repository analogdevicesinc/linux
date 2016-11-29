/*
 * AXI Interrupt Monitor driver
 *
 * Copyright 2016 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/bitmap.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/mod_devicetable.h>
#include <linux/fs.h>

#define AXI_IRQ_MON_REG_VERSION		0x00
#define AXI_IRQ_MON_REG_CONTROL		0x08
#define AXI_IRQ_MON_REG_IRQ		0x0C
#define AXI_IRQ_MON_REG_COUNT_TO	0x10
#define AXI_IRQ_MON_REG_COUNT_FROM	0x14

struct axi_intr_mon {
	int irq;
	void __iomem *mem;
	unsigned long in_use;

	bool active;
	struct mutex user_lock;
	spinlock_t lock;

	DECLARE_KFIFO(ts_in, uint32_t, 128);
	DECLARE_KFIFO(ts_out, uint32_t, 128);
};

/* Miscdev does not allow multiple instance and we don't really need them */
static struct axi_intr_mon *g_axi_intr_mon;

static int axi_intr_mon_open(struct inode *inode, struct file *file)
{
	struct axi_intr_mon *axi_intr_mon = g_axi_intr_mon;

	/* Make sure only one open at a time */
	if (test_and_set_bit(0, &axi_intr_mon->in_use))
		return -EBUSY;

	writel(1, axi_intr_mon->mem + AXI_IRQ_MON_REG_CONTROL);

	/* Make sure the IRQ handler is not using the FIFO */
	synchronize_irq(axi_intr_mon->irq);
	kfifo_reset(&axi_intr_mon->ts_in);
	kfifo_reset(&axi_intr_mon->ts_out);
	axi_intr_mon->active = false;

	/* Signal the IRQ handler that it is safe to access the FIFOs */
	set_bit(1, &axi_intr_mon->in_use);

	return 0;
}

static int axi_intr_mon_release(struct inode *inode, struct file *file)
{
	struct axi_intr_mon *axi_intr_mon = g_axi_intr_mon;

	clear_bit(1, &axi_intr_mon->in_use);
	writel(0, axi_intr_mon->mem + AXI_IRQ_MON_REG_CONTROL);
	clear_bit(0, &axi_intr_mon->in_use);

	return 0;
}

static ssize_t axi_intr_mon_read(struct file *file, char __user *buf,
	size_t count, loff_t *ppos)
{
	struct axi_intr_mon *axi_intr_mon = g_axi_intr_mon;
	unsigned int copied;
	int ret;

	mutex_lock(&axi_intr_mon->user_lock);
	ret = kfifo_to_user(&axi_intr_mon->ts_out, buf, count, &copied);
	mutex_unlock(&axi_intr_mon->user_lock);

	return ret ? ret : copied;
}

static ssize_t axi_intr_mon_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
	struct axi_intr_mon *axi_intr_mon = g_axi_intr_mon;
	unsigned long flags;
	unsigned int copied;
	int ret, ret2;
	uint32_t ts;

	mutex_lock(&axi_intr_mon->user_lock);
	ret = kfifo_from_user(&axi_intr_mon->ts_in, buf, count, &copied);
	spin_lock_irqsave(&axi_intr_mon->lock, flags);
	if (!axi_intr_mon->active) {
		ret2 = kfifo_get(&axi_intr_mon->ts_in, &ts);
		if (ret2 > 0) {
			writel(ts, axi_intr_mon->mem + AXI_IRQ_MON_REG_COUNT_TO);
			readl(axi_intr_mon->mem + AXI_IRQ_MON_REG_COUNT_TO);
			axi_intr_mon->active = true;
		}
	}
	spin_unlock_irqrestore(&axi_intr_mon->lock, flags);

	mutex_unlock(&axi_intr_mon->user_lock);

	return ret ? ret : copied;
}

static const struct file_operations axi_intr_mon_fops = {
	.owner = THIS_MODULE,
	.write = axi_intr_mon_write,
	.read = axi_intr_mon_read,
	.open = axi_intr_mon_open,
	.release = axi_intr_mon_release,
};

static irqreturn_t axi_intr_mon_irq(int irq, void *devid)
{
	struct axi_intr_mon *axi_intr_mon = devid;
	uint32_t ts, irq_val;
	int ret;

	irq_val = readl(axi_intr_mon->mem + AXI_IRQ_MON_REG_IRQ);
	if (!irq_val) /* No IRQ */
		return IRQ_NONE;

	if (!test_bit(1, &axi_intr_mon->in_use))
		return IRQ_HANDLED;

	ts = readl(axi_intr_mon->mem + AXI_IRQ_MON_REG_COUNT_FROM);
	kfifo_put(&axi_intr_mon->ts_out, ts);

	spin_lock(&axi_intr_mon->lock);
	ret = kfifo_get(&axi_intr_mon->ts_in, &ts);
	if (ret > 0)
		writel(ts, axi_intr_mon->mem + AXI_IRQ_MON_REG_COUNT_TO);
	else
		axi_intr_mon->active = false;
	readl(axi_intr_mon->mem + AXI_IRQ_MON_REG_COUNT_TO);
	spin_unlock(&axi_intr_mon->lock);

	writel(irq_val, axi_intr_mon->mem + AXI_IRQ_MON_REG_IRQ);

	return IRQ_HANDLED;
}

static struct miscdevice axi_intr_mon_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "axi_intr_monitor",
	.fops = &axi_intr_mon_fops,
};

static int axi_intr_mon_probe(struct platform_device *pdev)
{
	struct axi_intr_mon *axi_intr_mon;
	struct resource *res;
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -EINVAL;

	axi_intr_mon = devm_kzalloc(&pdev->dev, sizeof(*axi_intr_mon),
		GFP_KERNEL);
	if (!axi_intr_mon)
		return -ENOMEM;

	INIT_KFIFO(axi_intr_mon->ts_in);
	INIT_KFIFO(axi_intr_mon->ts_out);
	mutex_init(&axi_intr_mon->user_lock);
	spin_lock_init(&axi_intr_mon->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	axi_intr_mon->mem = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(axi_intr_mon->mem))
		return PTR_ERR(axi_intr_mon->mem);

	ret = devm_request_irq(&pdev->dev, irq, axi_intr_mon_irq, 0,
		dev_name(&pdev->dev), axi_intr_mon);
	if (ret)
		return ret;

	g_axi_intr_mon = axi_intr_mon;

	ret = misc_register(&axi_intr_mon_miscdev);
	if (ret) {
		g_axi_intr_mon = NULL;
		return ret;
	}

	return 0;
}

static int axi_intr_mon_remove(struct platform_device *pdev)
{
	misc_deregister(&axi_intr_mon_miscdev);
	g_axi_intr_mon = NULL;

	return 0;
}

static struct of_device_id axi_intr_mon_of_match[] = {
	{ .compatible = "adi,axi-intr-monitor-1.00.a", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, xdevcfg_of_match);

static struct platform_driver axi_intr_mon_driver = {
	.probe = axi_intr_mon_probe,
	.remove = axi_intr_mon_remove,
	.driver = {
		.name = "axi-intr-monitor",
		.of_match_table = axi_intr_mon_of_match,
	},
};
module_platform_driver(axi_intr_mon_driver);
