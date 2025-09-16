// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Author: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/radix-tree.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/soc/adi/system_config.h>
#include <linux/types.h>

struct system_context {
	/* underlying regmap_mmio */
	struct regmap *regmap;
	/* tree of register definitions by index */
	struct radix_tree_root tree;
	/* configuration we were created with */
	struct system_config *config;
};

static int regmap_system_read(void *context, unsigned int reg, unsigned int *val)
{
	struct system_context *ctx = context;
	struct system_register *sreg = radix_tree_lookup(&ctx->tree, reg);
	int ret;

	if (!sreg)
		return -EIO;

	if (sreg->is_bits) {
		u32 tmp;

		ret = regmap_read(ctx->regmap, sreg->offset, &tmp);
		if (ret)
			return ret;

		tmp = (tmp & sreg->mask) >> sreg->shift;
		*val = tmp;
		return 0;
	}

	return regmap_read(ctx->regmap, sreg->offset, val);
}

static int regmap_system_write(void *context, unsigned int reg, unsigned int val)
{
	struct system_context *ctx = context;
	struct system_register *sreg = radix_tree_lookup(&ctx->tree, reg);

	if (!sreg)
		return -EIO;

	if (sreg->is_bits) {
		return regmap_update_bits(ctx->regmap, sreg->offset, sreg->mask,
			(val << sreg->shift) & sreg->mask);
	}

	return regmap_write(ctx->regmap, sreg->offset, val);
}

static struct system_context *create_context(struct system_config *config)
{
	struct regmap *regmap = config->mmio_regmap;
	struct system_context *ctx;
	size_t i;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return ERR_PTR(-ENOMEM);

	ctx->regmap = regmap;
	INIT_RADIX_TREE(&ctx->tree, GFP_KERNEL);

	for (i = 0; i < config->len; ++i) {
		struct system_register *sreg = &config->registers[i];

		ret = radix_tree_insert(&ctx->tree, sreg->id, sreg);
		if (ret)
			return ERR_PTR(ret);
	}

	config->config.max_register = config->max_register;
	config->config.reg_bits = 8 * sizeof(u32);
	config->config.val_bits = 8 * sizeof(u32);
	config->config.reg_stride = 1;

	return ctx;
}

static void regmap_system_free_context(void *context)
{
	struct system_context *ctx = context;
	unsigned int i;

	for (i = 0; i < ctx->config->len; ++i)
		radix_tree_delete(&ctx->tree, ctx->config->registers[i].id);

	WARN_ON(!radix_tree_empty(&ctx->tree));

	kfree(ctx);
}

static const struct regmap_bus regmap_system_bus = {
	.fast_io = true,
	.reg_write = regmap_system_write,
	.reg_read = regmap_system_read,
	.free_context = regmap_system_free_context,
	.val_format_endian_default = REGMAP_ENDIAN_LITTLE,
};

struct regmap *__regmap_init_system_config(struct device *dev,
	struct system_config *config,
	struct lock_class_key *lock_key, const char *lock_name)
{
	struct system_context *ctx = create_context(config);

	if (IS_ERR(ctx))
		return ERR_CAST(ctx);

	return __regmap_init(dev, &regmap_system_bus, ctx, &config->config,
		lock_key, lock_name);
}

struct regmap *__devm_regmap_init_system_config(struct device *dev,
	struct system_config *config,
	struct lock_class_key *lock_key, const char *lock_name)
{
	struct system_context *ctx = create_context(config);

	if (IS_ERR(ctx))
		return ERR_PTR(PTR_ERR(ctx));

	return __devm_regmap_init(dev, &regmap_system_bus, ctx, &config->config,
		lock_key, lock_name);
}

static DEFINE_SPINLOCK(system_config_lock);
static LIST_HEAD(system_config_list);

struct regmap *system_config_regmap_lookup_by_phandle(struct device_node *np,
	const char *property)
{
	struct system_config *config = NULL;
	struct system_config *entry;
	struct device_node *config_np;
	unsigned long flags;

	config_np = of_parse_phandle(np, property, 0);
	if (!config_np)
		return ERR_PTR(-ENODEV);

	spin_lock_irqsave(&system_config_lock, flags);
	list_for_each_entry(entry, &system_config_list, list) {
		if (entry->np == config_np) {
			config = entry;
			break;
		}
	}
	spin_unlock_irqrestore(&system_config_lock, flags);

	of_node_put(config_np);

	if (!config)
		return ERR_PTR(-EPROBE_DEFER);

	return config->system_regmap;
}
EXPORT_SYMBOL_GPL(system_config_regmap_lookup_by_phandle);

int system_config_probe(struct platform_device *pdev, struct system_config *config)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regmap *regmap_mmio;
	struct regmap *regmap_system;
	struct resource *res;
	void __iomem *base;
	unsigned long flags;

	struct regmap_config mmio_config = {
		.reg_bits = 8 * sizeof(u32),
		.val_bits = 8 * sizeof(u32),
		.reg_stride = sizeof(u32),
	};

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(base))
		return PTR_ERR(base);

	mmio_config.name = of_node_full_name(np);
	mmio_config.max_register = resource_size(res) - sizeof(u32);
	mmio_config.cache_type = REGCACHE_NONE;

	regmap_mmio = devm_regmap_init_mmio(dev, base, &mmio_config);
	if (IS_ERR(regmap_mmio)) {
		dev_err(dev, "mmio regmap initialization failed\n");
		return PTR_ERR(regmap_mmio);
	}

	config->mmio_regmap = regmap_mmio;
	regmap_system = devm_regmap_init_system_config(dev, config);
	if (IS_ERR(regmap_system)) {
		dev_err(dev, "system config regmap initialization failed\n");
		return PTR_ERR(regmap_system);
	}

	config->np = np;
	config->system_regmap = regmap_system;
	platform_set_drvdata(pdev, config);

	spin_lock_irqsave(&system_config_lock, flags);
	list_add_tail(&config->list, &system_config_list);
	spin_unlock_irqrestore(&system_config_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(system_config_probe);

int system_config_remove(struct platform_device *pdev)
{
	struct system_config *config = platform_get_drvdata(pdev);
	unsigned long flags;

	spin_lock_irqsave(&system_config_lock, flags);
	list_del(&config->list);
	spin_unlock_irqrestore(&system_config_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(system_config_remove);
