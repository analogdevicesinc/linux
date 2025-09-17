/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Implementation of system_config, potential replacement for syscon that generalizes
 * it to support arbitrary regmap registration and requires the driver to be initialized
 * first
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef SOC_ADI_SYSTEM_CONFIG_H
#define SOC_ADI_SYSTEM_CONFIG_H

#include <linux/list.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/radix-tree.h>
#include <linux/regmap.h>
#include <linux/types.h>

struct system_register {
	u32 id;
	u32 offset;
	u32 mask;
	u8 shift;
	bool is_bits;
};

struct system_config {
	/* User configured */
	struct system_register *registers;
	unsigned int max_register;
	size_t len;

	/* Internal data populated during usage */
	struct regmap_config config;
	struct regmap *mmio_regmap;
	struct device_node *np;
	struct list_head list;
	struct regmap *system_regmap;
};

struct regmap *__regmap_init_system_config(struct device *dev,
					   struct system_config *config,
					   struct lock_class_key *lock_key,
					   const char *lock_name);

struct regmap *__devm_regmap_init_system_config(struct device *dev,
						struct system_config
						*config,
						struct lock_class_key
						*lock_key,
						const char *lock_name);

#define regmap_init_system_config(dev, config) \
	__regmap_lockdep_wrapper(__regmap_init_system_config, #config, dev, config)

#define devm_regmap_init_system_config(dev, config) \
	__regmap_lockdep_wrapper(__devm_regmap_init_system_config, #config, \
		dev, config)

struct regmap *system_config_regmap_lookup_by_phandle(struct device_node
						      *np,
						      const char
						      *property);

int system_config_probe(struct platform_device *pdev,
			struct system_config *config);
int system_config_remove(struct platform_device *pdev);

#endif
