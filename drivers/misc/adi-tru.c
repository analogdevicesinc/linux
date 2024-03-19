// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analog Devices Trigger Routing Unit (TRU) driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/adi-tru.h>


#define TRU_SSR0                        0x0
#define TRU_MTR                         0x7e0
#define TRU_ERRADDR                     0x7e8
#define TRU_STAT                        0x7ec
#define TRU_REVID                       0x7f0
#define TRU_GCTL                        0x7f4

#define TRU_GCTL_EN                     0x1
#define TRU_GCTL_RESET                  0x2

#define TRU_STAT_LWERR                  0x1
#define TRU_STAT_ADDRERR                0x2

#define TRU_SSR_LOCK                    0x80000000


static LIST_HEAD(tru_list);
static DEFINE_MUTEX(tru_list_lock);


/* Get TRU device by its alias ID */
struct adi_tru *adi_tru_get(u32 alias_id)
{
	struct adi_tru *tru;

	mutex_lock(&tru_list_lock);
	list_for_each_entry(tru, &tru_list, node) {
		if (tru->alias_id == alias_id) {
			mutex_unlock(&tru_list_lock);
			return tru;
		}
	}
	mutex_unlock(&tru_list_lock);

	return NULL;
}
EXPORT_SYMBOL_GPL(adi_tru_get);

int adi_tru_enable(struct adi_tru *tru)
{
	u32 reg;

	mutex_lock(&tru->lock);

	reg = readl(tru->base + TRU_GCTL);
	reg |= TRU_GCTL_EN;
	writel(reg, tru->base + TRU_GCTL);

	mutex_unlock(&tru->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(adi_tru_enable);

int adi_tru_disable(struct adi_tru *tru)
{
	u32 reg;

	mutex_lock(&tru->lock);

	reg = readl(tru->base + TRU_GCTL);
	reg &= ~TRU_GCTL_EN;
	writel(reg, tru->base + TRU_GCTL);

	mutex_unlock(&tru->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(adi_tru_disable);

int adi_tru_soft_reset(struct adi_tru *tru)
{
	u32 reg;

	mutex_lock(&tru->lock);

	reg = readl(tru->base + TRU_GCTL);
	reg |= TRU_GCTL_RESET;
	writel(reg, tru->base + TRU_GCTL);

	mutex_unlock(&tru->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(adi_tru_soft_reset);

int adi_tru_trigger(struct adi_tru *tru, int n, ...)
{
	va_list ap;
	u32 reg, source;
	int i;

	if (n < 0 || n > 4) {
		dev_err(tru->dev, "Invalid number of arguments");
		return -EINVAL;
	}

	va_start(ap, n);

	reg = 0;
	for (i = 0; i < n; i++) {
		source = va_arg(ap, u32);
		reg |= source << (i * 8);
	}

	writel(reg, tru->base + TRU_MTR);

	return 0;
}
EXPORT_SYMBOL_GPL(adi_tru_trigger);

int adi_tru_connect_source_to_target(struct adi_tru *tru,
				     u32 source, u32 target, bool locked)
{
	u32 reg;

	if (source > tru->last_source_id) {
		dev_err(tru->dev, "Invalid TRU source: %d", source);
		return -EINVAL;
	}

	if (target > tru->last_target_id) {
		dev_err(tru->dev, "Invalid TRU target: %d", target);
		return -EINVAL;
	}

	mutex_lock(&tru->lock);

	reg = readl(tru->base + target * 4);
	if (reg & TRU_SSR_LOCK) {
		mutex_unlock(&tru->lock);
		return -EINVAL;
	}

	if (locked)
		source |= TRU_SSR_LOCK;

	writel(source, tru->base + target * 4);

	mutex_unlock(&tru->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(adi_tru_connect_source_to_target);

/*
 * sysfs interface for TRU
 *
 * ssr \
 *      + ssr0 (RW)
 *      + ssr1 (RW)
 *      + ssr2 (RW)
 *      ...
 * mtr    (RW)
 * enable (RW)
 * status (RW)
 * reset  (WO)
 */
static ssize_t mtr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	u32 mtr = readl(tru->base + TRU_MTR);

	/* TODO a better way is to output four trigger source IDs */
	return scnprintf(buf, PAGE_SIZE, "0x%08x\n", mtr);
}

static ssize_t mtr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	unsigned long value;
	int err;

	/* TODO a better way is to input four trigger source IDs */
	err = kstrtoul(buf, 0, &value);
	if (err)
		return err;

	writel(value, tru->base + TRU_MTR);

	return count;
}

static DEVICE_ATTR_RW(mtr);

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	u32 gctl = readl(tru->base + TRU_GCTL);

	return scnprintf(buf, PAGE_SIZE, "%d\n", (gctl & TRU_GCTL_EN) ? 1 : 0);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	unsigned long value;
	int err;

	err = kstrtoul(buf, 0, &value);
	if (err)
		return err;

	if (value == 0)
		err = adi_tru_disable(tru);
	else if (value == 1)
		err = adi_tru_enable(tru);
	else
		return -EINVAL;

	return err ? err : count;
}

static DEVICE_ATTR_RW(enable);

static ssize_t reset_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	unsigned long value;
	int err;

	err = kstrtoul(buf, 0, &value);
	if (err)
		return err;

	if (value == 0)
		err = 0; // do nothing
	else if (value == 1)
		err = adi_tru_soft_reset(tru);
	else
		return -EINVAL;

	return err ? err : count;
}

static DEVICE_ATTR_WO(reset);

static ssize_t status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	u32 stat;
	bool addrerr, lwerr;

	stat = readl(tru->base + TRU_STAT);
	addrerr = ((stat & TRU_STAT_ADDRERR) != 0);
	lwerr = ((stat & TRU_STAT_LWERR) != 0);

	return scnprintf(buf, PAGE_SIZE,
			 "%d: %saddress error, %slock write error\n",
			 stat,
			 addrerr ? "" : "no ", lwerr ? "" : "no ");
}

static ssize_t status_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	unsigned long value;
	int err;

	err = kstrtoul(buf, 0, &value);
	if (err)
		return err;

	writel(value, tru->base + TRU_STAT);

	return count;
}

static DEVICE_ATTR_RW(status);

static struct attribute *tru_attrs[] = {
	&dev_attr_mtr.attr,
	&dev_attr_enable.attr,
	&dev_attr_status.attr,
	&dev_attr_reset.attr,
	NULL,
};

static struct attribute_group tru_attribute_group = {
	.attrs	= tru_attrs,
};

static struct device_attribute *ssr_dev_attrs;
static struct attribute **ssr_attrs;
static struct attribute_group ssr_attribute_group = {
	.name	= "ssr",
};

static ssize_t ssr_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	int ssr, source;
	u32 reg;
	bool locked;

	if (sscanf(attr->attr.name, "ssr%d", &ssr) != 1)
		return -EINVAL;

	reg = readl(tru->base + ssr * 4);
	source = reg & 0xff;
	locked = ((reg & TRU_SSR_LOCK) != 0);

	return scnprintf(buf, PAGE_SIZE, "%d%s\n", source,
			 locked ? "(locked)" : "");
}

static ssize_t ssr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct adi_tru *tru = dev_get_drvdata(dev);
	int value;
	u32 source, target;
	int err;
	bool locked = false;

	if (sscanf(attr->attr.name, "ssr%d", &value) != 1)
		return -EINVAL;
	target = value;

	if (sscanf(buf, "%d", &value) != 1)
		return -EINVAL;
	if (value < 0)
		return -EINVAL;
	source = value;

	if (strstr(buf, "locked") != NULL)
		locked = true;

	err = adi_tru_connect_source_to_target(tru, source, target, locked);
	if (err)
		return err;

	return count;
}

static int adi_tru_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct adi_tru *tru;
	struct device *dev;
	struct resource *res;
	int ret, n, i, id;

	tru = devm_kzalloc(&pdev->dev, sizeof(*tru), GFP_KERNEL);
	if (!tru)
		return -ENOMEM;

	mutex_init(&tru->lock);

	tru->dev = &pdev->dev;

	dev = tru->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tru->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(tru->base))
		return PTR_ERR(tru->base);

	platform_set_drvdata(pdev, tru);

	id = of_alias_get_id(np, "tru");
	if (id < 0) {
		dev_err(tru->dev,
			"The alias of tru node should be truN");
		return id;
	}
	tru->alias_id = id;

	ret = of_property_read_u32(np, "adi,tru-last-source-id",
				   &tru->last_source_id);
	if (ret) {
		dev_err(tru->dev,
			"Can't find adi,tru-max-source-id in tru node");
		return ret;
	}

	ret = of_property_read_u32(np, "adi,tru-last-target-id",
				   &tru->last_target_id);
	if (ret) {
		dev_err(tru->dev,
			"Can't find adi,tru-last-target-id in tru node");
		return ret;
	}

	/* Soft reset all TRU registers */
	writel(TRU_GCTL_RESET, tru->base + TRU_GCTL);

	tru->preset_locked =
		of_property_read_bool(np, "adi,tru-connections-preset-locked");

	/* Configure TRU with the static settings in device tree */
	n = of_property_count_elems_of_size(np, "adi,tru-connections-preset",
					    sizeof(u32));
	if (n < 0 || n % 2 != 0)
		return -EINVAL;

	for (i = 0; i < n / 2; i++) {
		u32 source, target;

		ret = of_property_read_u32_index(np,
						 "adi,tru-connections-preset",
						 i * 2, &source);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(np,
						 "adi,tru-connections-preset",
						 i * 2 + 1, &target);
		if (ret)
			return ret;

		ret = adi_tru_connect_source_to_target(tru, source, target,
						       tru->preset_locked);
		if (ret)
			return ret;
	}

	/* Enable TRU */
	writel(TRU_GCTL_EN, tru->base + TRU_GCTL);

	/* Add to tru to tru_list */
	mutex_lock(&tru_list_lock);
	list_add_tail(&tru->node, &tru_list);
	mutex_unlock(&tru_list_lock);


	/* Create sysfs interface files */

	ret = sysfs_create_group(&dev->kobj, &tru_attribute_group);
	if (ret)
		return ret;

	ssr_dev_attrs =
		devm_kcalloc(dev, tru->last_target_id + 1,
			     sizeof(struct device_attribute), GFP_KERNEL);
	if (ssr_dev_attrs == NULL)
		return -ENOMEM;

	ssr_attrs =
		devm_kcalloc(dev, tru->last_target_id + 2,
			     sizeof(struct attribute *), GFP_KERNEL);
	if (ssr_attrs == NULL)
		return -ENOMEM;

	for (i = 0; i <= tru->last_target_id; i++) {
		char *name;

		name = devm_kasprintf(dev, GFP_KERNEL, "ssr%d", i);
		if (name == NULL)
			return -ENOMEM;

		sysfs_attr_init(&ssr_dev_attrs[i].attr);
		ssr_dev_attrs[i].attr.name = name;
		ssr_dev_attrs[i].attr.mode = S_IRUGO | S_IWUSR;
		ssr_dev_attrs[i].show = ssr_show;
		ssr_dev_attrs[i].store = ssr_store;

		ssr_attrs[i] = &ssr_dev_attrs[i].attr;
	}

	ssr_attribute_group.attrs = ssr_attrs;

	ret = sysfs_create_group(&dev->kobj, &ssr_attribute_group);
	if (ret)
		return ret;

	return 0;
}

static int adi_tru_remove(struct platform_device *pdev)
{
	struct adi_tru *tru = platform_get_drvdata(pdev);

	/* Disable TRU */
	writel(0, tru->base + TRU_GCTL);

	/* Remove tru from tru_list */
	mutex_lock(&tru_list_lock);
	list_del(&tru->node);
	mutex_unlock(&tru_list_lock);

	return 0;
}

static const struct of_device_id adi_tru_match_table[] = {
	{ .compatible = "adi,tru", },
	{}
};

MODULE_DEVICE_TABLE(of, adi_tru_match_table);

static struct platform_driver adi_tru_driver = {
	.driver			= {
		.name		= "adi-tru",
		.of_match_table = adi_tru_match_table,
	},
	.probe			= adi_tru_probe,
	.remove			= adi_tru_remove,
};

module_platform_driver(adi_tru_driver);

MODULE_DESCRIPTION("Analog Devices Trigger Routing Unit (TRU) driver");
MODULE_LICENSE("GPL v2");
