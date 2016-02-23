/*
 * Altera XCVR Configuration Driver
 *
 * Copyright 2015-2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <linux/clk.h>

/* Registers Description */

#define ALTERA_XCVR_REG_VERSION		0x0000
#define ALTERA_XCVR_REG_ID			0x0004
#define ALTERA_XCVR_REG_SCRATCH		0x0008

#define ALTERA_XCVR_REG_RESETN		0x000C
#define ALTERA_XCVR_RESETN			(1 << 0)

#define ALTERA_XCVR_REG_RX_SYSREF	0x0040
#define ALTERA_XCVR_RX_SYSREF_SEL	(1 << 1)
#define ALTERA_XCVR_RX_SYSREF		(1 << 0)

#define ALTERA_XCVR_REG_RX_SYNC		0x0044
#define ALTERA_XCVR_RX_SYNC			(1 << 0)

#define ALTERA_XCVR_REG_RX_STATUS	0x0048

#define ALTERA_XCVR_REG_RX_RESETN	0x004C
#define ALTERA_XCVR_RX_RESETN		(1 << 0)

#define ALTERA_XCVR_REG_TX_SYSREF	0x0080
#define ALTERA_XCVR_TX_SYSREF_SEL	(1 << 1)
#define ALTERA_XCVR_TX_SYSREF		(1 << 0)

#define ALTERA_XCVR_REG_TX_SYNC		0x0084
#define ALTERA_XCVR_TX_SYNC			(1 << 0)

#define ALTERA_XCVR_REG_TX_STATUS	0x0088

#define ALTERA_XCVR_REG_TX_RESETN	0x008C
#define ALTERA_XCVR_TX_RESETN		(1 << 0)

#define ALTERA_XCVR_REG_DEVICE_TYPE	0x00C0


struct altera_xcvr_state {
	struct device 		*dev;
	void __iomem		*regs;
	bool				ext_sysref_en;
	bool				rx_en;
	bool				tx_en;
	struct delayed_work	delayed_work;
};

static inline void altera_xcvr_write(struct altera_xcvr_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int altera_xcvr_read(struct altera_xcvr_state *st,
	unsigned reg)
{
	return ioread32(st->regs + reg);
}

static ssize_t altera_xcvr_sysfs_show(struct device *dev,
									  struct device_attribute *attr,
									  char *buf);
static ssize_t altera_xcvr_sysfs_store(struct device *dev,
									   struct device_attribute *attr,
									   const char *buf, size_t count);

static DEVICE_ATTR(altera_xcvr_version, S_IRUGO,
				   altera_xcvr_sysfs_show, NULL);
static DEVICE_ATTR(altera_xcvr_reset, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_reset, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_sysref, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_sync, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_rx_status, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_reset, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_sysref, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_sync, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);
static DEVICE_ATTR(altera_xcvr_tx_status, S_IRUGO | S_IWUSR,
				   altera_xcvr_sysfs_show, altera_xcvr_sysfs_store);

static struct attribute *altera_xcvr_sysfs_attrs[] = {
	&dev_attr_altera_xcvr_version.attr,
	&dev_attr_altera_xcvr_reset.attr,
	&dev_attr_altera_xcvr_rx_reset.attr,
	&dev_attr_altera_xcvr_rx_sysref.attr,
	&dev_attr_altera_xcvr_rx_sync.attr,
	&dev_attr_altera_xcvr_rx_status.attr,
	&dev_attr_altera_xcvr_tx_reset.attr,
	&dev_attr_altera_xcvr_tx_sysref.attr,
	&dev_attr_altera_xcvr_tx_sync.attr,
	&dev_attr_altera_xcvr_tx_status.attr,
	NULL
};

static const struct attribute_group altera_xcvr_sysfs_group = {
	.attrs = altera_xcvr_sysfs_attrs,
};

static ssize_t altera_xcvr_sysfs_show(struct device *dev,
									  struct device_attribute *attr,
									  char *buf)
{
	struct altera_xcvr_state *st = dev_get_drvdata(dev);
	signed reg = -1;
	unsigned val = 0;

	if (attr == &dev_attr_altera_xcvr_version)
		reg = ALTERA_XCVR_REG_VERSION;
	if (attr == &dev_attr_altera_xcvr_reset)
		reg = ALTERA_XCVR_REG_RESETN;
	if (attr == &dev_attr_altera_xcvr_rx_reset)
		reg = ALTERA_XCVR_REG_RX_RESETN;
	if (attr == &dev_attr_altera_xcvr_rx_sysref)
		reg = ALTERA_XCVR_REG_RX_SYSREF;
	if (attr == &dev_attr_altera_xcvr_rx_sync)
		reg = ALTERA_XCVR_REG_RX_SYNC;
	if (attr == &dev_attr_altera_xcvr_rx_status)
		reg = ALTERA_XCVR_REG_RX_STATUS;
	if (attr == &dev_attr_altera_xcvr_tx_reset)
		reg = ALTERA_XCVR_REG_TX_RESETN;
	if (attr == &dev_attr_altera_xcvr_tx_sysref)
		reg = ALTERA_XCVR_REG_TX_SYSREF;
	if (attr == &dev_attr_altera_xcvr_tx_sync)
		reg = ALTERA_XCVR_REG_TX_SYNC;
	if (attr == &dev_attr_altera_xcvr_tx_status)
		reg = ALTERA_XCVR_REG_TX_STATUS;
	if (reg == -1)
		return -EINVAL;

	val = altera_xcvr_read(st, reg);

	return sprintf(buf, "0x%x\n", val);
}

static ssize_t altera_xcvr_sysfs_store(struct device *dev,
									   struct device_attribute *attr,
									   const char *buf, size_t count)
{
	struct altera_xcvr_state *st = dev_get_drvdata(dev);
	signed reg = -1;
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 0, &val);
	if (ret)
		return ret;

	if (attr == &dev_attr_altera_xcvr_reset)
		reg = ALTERA_XCVR_REG_RESETN;
	else if (attr == &dev_attr_altera_xcvr_rx_reset)
		reg = ALTERA_XCVR_REG_RX_RESETN;
	else if (attr == &dev_attr_altera_xcvr_rx_sysref)
		reg = ALTERA_XCVR_REG_RX_SYSREF;
	else if (attr == &dev_attr_altera_xcvr_rx_sync)
		reg = ALTERA_XCVR_REG_RX_SYNC;
	else if (attr == &dev_attr_altera_xcvr_rx_status)
		reg = ALTERA_XCVR_REG_RX_STATUS;
	else if (attr == &dev_attr_altera_xcvr_tx_reset)
		reg = ALTERA_XCVR_REG_TX_RESETN;
	else if (attr == &dev_attr_altera_xcvr_tx_sysref)
		reg = ALTERA_XCVR_REG_TX_SYSREF;
	else if (attr == &dev_attr_altera_xcvr_tx_sync)
		reg = ALTERA_XCVR_REG_TX_SYNC;
	else if (attr == &dev_attr_altera_xcvr_tx_status)
		reg = ALTERA_XCVR_REG_TX_STATUS;
	if (reg == -1)
		return -EINVAL;

	altera_xcvr_write(st, reg, val);

	return count;
}

static void altera_xcvr_work_func(struct work_struct *work)
{
	struct altera_xcvr_state *st =
			container_of(work, struct altera_xcvr_state, delayed_work.work);
	unsigned status;

	altera_xcvr_write(st, ALTERA_XCVR_REG_RESETN, 0);
	mdelay(10);
	altera_xcvr_write(st, ALTERA_XCVR_REG_RESETN, ALTERA_XCVR_RESETN);
	mdelay(10);

	/* RX */
	if (st->rx_en) {
		altera_xcvr_write(st, ALTERA_XCVR_REG_RX_SYSREF,
					(st->ext_sysref_en ? ALTERA_XCVR_RX_SYSREF_SEL : 0));
		mdelay(10);
		altera_xcvr_write(st, ALTERA_XCVR_REG_RX_SYSREF,
					(st->ext_sysref_en ? ALTERA_XCVR_RX_SYSREF_SEL : ALTERA_XCVR_RX_SYSREF));
		mdelay(100);

		status = altera_xcvr_read(st, ALTERA_XCVR_REG_RX_STATUS);
		if ((status & 0x1ff) != 0x1ff)
			dev_err(st->dev, "RX transceiver NOT ready [%04x]!!\n", status);
	}

	/* TX */
	if (st->tx_en) {
		altera_xcvr_write(st, ALTERA_XCVR_REG_TX_SYSREF,
					(st->ext_sysref_en ? ALTERA_XCVR_TX_SYSREF_SEL : 0));
		mdelay(10);
		altera_xcvr_write(st, ALTERA_XCVR_REG_TX_SYSREF,
					(st->ext_sysref_en ? ALTERA_XCVR_TX_SYSREF_SEL : ALTERA_XCVR_TX_SYSREF));
		mdelay(100);

		status = altera_xcvr_read(st, ALTERA_XCVR_REG_TX_STATUS);
		if ((status & 0x1ff) != 0x1ff)
			dev_err(st->dev, "TX transceiver NOT ready [%04x]!!\n", status);
	}
}

static int altera_xcvr_probe(struct platform_device *pdev)
{
	struct altera_xcvr_state *st;
	struct resource *mem;
	int ret;
	struct device_node *np = pdev->dev.of_node;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pdev->dev, "Not enough memory for device\n");
		return -ENOMEM;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->ext_sysref_en = of_property_read_bool(np, "adi,external-sysref-enable");
	st->rx_en = of_property_read_bool(np, "adi,rx-enable");
	st->tx_en = of_property_read_bool(np, "adi,tx-enable");

	st->dev = &pdev->dev;
	platform_set_drvdata(pdev, st);

	altera_xcvr_write(st, ALTERA_XCVR_REG_RESETN, ALTERA_XCVR_RESETN);
	mdelay(10);

	if (st->rx_en) {
		altera_xcvr_write(st, ALTERA_XCVR_REG_RX_RESETN, ALTERA_XCVR_RX_RESETN);
		mdelay(10);

		altera_xcvr_write(st, ALTERA_XCVR_REG_RX_SYNC, ALTERA_XCVR_RX_SYNC);
		altera_xcvr_write(st, ALTERA_XCVR_REG_RX_SYSREF,
					(st->ext_sysref_en ? ALTERA_XCVR_RX_SYSREF_SEL : 0));
	}

	if (st->tx_en) {
		altera_xcvr_write(st, ALTERA_XCVR_REG_TX_RESETN, ALTERA_XCVR_TX_RESETN);
		mdelay(10);

		altera_xcvr_write(st, ALTERA_XCVR_REG_TX_SYNC, ALTERA_XCVR_TX_SYNC);
		altera_xcvr_write(st, ALTERA_XCVR_REG_TX_SYSREF,
					(st->ext_sysref_en ? ALTERA_XCVR_TX_SYSREF_SEL : 0));
	}

	INIT_DELAYED_WORK(&st->delayed_work, altera_xcvr_work_func);
	schedule_delayed_work(&st->delayed_work, HZ * 5);

	ret = sysfs_create_group(&pdev->dev.kobj, &altera_xcvr_sysfs_group);
	if (ret)
		dev_err(&pdev->dev, "Can't create the sysfs group\n");

	dev_info(&pdev->dev, "Altera XCVR probed\n");

	return 0;
}

static int altera_xcvr_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &altera_xcvr_sysfs_group);

	return 0;
}

static const struct of_device_id altera_xcvr_of_match[] = {
	{ .compatible = "adi,altera-xcvr-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, altera_xcvr_of_match);

static struct platform_driver altera_xcvr_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = altera_xcvr_of_match,
	},
	.probe	= altera_xcvr_probe,
	.remove	= altera_xcvr_remove,
};

module_platform_driver(altera_xcvr_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Altera XCVR Configuration Driver");
MODULE_LICENSE("GPL v2");
