// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2022, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/device.h>

#include "../core.h"
#include "../pinconf.h"
#include "../pinmux.h"
#include "pinctrl-adi.h"

static inline const struct group_desc *adi_pinctrl_find_group_by_name(
	struct pinctrl_dev *pctldev,
	const char *name)
{
	const struct group_desc *grp = NULL;
	int i;

	for (i = 0; i < pctldev->num_groups; i++) {
		grp = pinctrl_generic_get_group(pctldev, i);
		if (grp && !strcmp(grp->grp.name, name))
			break;
	}

	return grp;
}

static void adi_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			     unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int adi_dt_node_to_map(struct pinctrl_dev *pctldev,
			      struct device_node *np,
			      struct pinctrl_map **map, unsigned int *num_maps)
{
	const struct group_desc *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	struct adi_pin *pin;
	int map_num = 1;
	int i;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = adi_pinctrl_find_group_by_name(pctldev, np->name);
	if (!grp)
		return -EINVAL;

	for (i = 0; i < grp->grp.npins; i++) {
		pin = &((struct adi_pin *)(grp->data))[i];
		map_num++;
	}

	new_map = kmalloc_array(map_num, sizeof(struct pinctrl_map),
				GFP_KERNEL);
	if (!new_map)
		return -ENOMEM;

	*map = new_map;
	*num_maps = map_num;
	/* create mux map */
	parent = of_get_parent(np);
	if (!parent) {
		kfree(new_map);
		return -EINVAL;
	}
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np->name;
	of_node_put(parent);

	/* create config map */
	new_map++;
	for (i = 0; i < grp->grp.npins; i++) {
		pin = &((struct adi_pin *)(grp->data))[i];

		new_map[i].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[i].data.configs.group_or_pin =
			pin_get_name(pctldev, pin->pin);

		new_map[i].data.configs.configs = (long *)&pin->conf.mio;
		new_map[i].data.configs.num_configs = 1;
	}

	return 0;
}

static void adi_dt_free_map(struct pinctrl_dev *pctldev,
			    struct pinctrl_map *map, unsigned int num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops adi_pctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.pin_dbg_show		= adi_pin_dbg_show,
	.dt_node_to_map		= adi_dt_node_to_map,
	.dt_free_map		= adi_dt_free_map,
};

static int adi_pmx_set(struct pinctrl_dev *pctldev, unsigned int selector,
		       unsigned int group)
{
	struct function_desc *func;
	struct group_desc *grp;
	unsigned int npins;

	/*
	 * Configure the mux mode for each pin in the group for a specific
	 * function.
	 */
	grp = pinctrl_generic_get_group(pctldev, group);
	if (!grp)
		return -EINVAL;

	func = pinmux_generic_get_function(pctldev, selector);
	if (!func)
		return -EINVAL;

	npins = grp->grp.npins;
	return 0;
}

const struct pinmux_ops adi_pmx_ops = {
	.get_functions_count	= pinmux_generic_get_function_count,
	.get_function_name	= pinmux_generic_get_function_name,
	.get_function_groups	= pinmux_generic_get_function_groups,
	.set_mux		= adi_pmx_set,
};

static void adi_pinctrl_parse_pin(struct adi_pinctrl *ipctl,
				  unsigned int *pin_id, struct adi_pin *pin,
				  const __be32 **list_p,
				  struct device_node *np)
{
	struct adi_pin_mio *pin_mio = &pin->conf.mio;
	struct adi_pin_reg *pin_reg;
	const __be32 *list = *list_p;
	uint32_t pin_num, mux_reg, conf_reg;

	pin_num = be32_to_cpu(*list++);
	mux_reg = be32_to_cpu(*list++);
	conf_reg = be32_to_cpu(*list++);
	pin_mio->input_pin = pin_num;
	pin_mio->mux_sel = mux_reg;
	pin_mio->config = (unsigned long)conf_reg;
	*pin_id = pin_num;
	pin_reg = &ipctl->pin_regs[*pin_id];
	pin->pin = *pin_id;
	pin_reg->pin_num = pin_num;
	pin_reg->mux_reg = mux_reg;
	pin_reg->conf_reg = conf_reg;
	*list_p = list;
}

static int adi_pinconf_get(struct pinctrl_dev *pctldev,
			   unsigned int pin_id, unsigned long *config)
{
	struct adi_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct adi_pinctrl_soc_info *info = ipctl->info;

	if (!info->adi_pinconf_get)
		return -EINVAL;

	/*
	 * Call the registered function to get the pinconf
	 */
	return info->adi_pinconf_get(pctldev, pin_id, config);
}


static int adi_pinconf_set(struct pinctrl_dev *pctldev,
			   unsigned int pin_id, unsigned long *configs,
			   unsigned int num_configs)
{
	struct adi_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct adi_pinctrl_soc_info *info = ipctl->info;

	if (!info->adi_pinconf_set)
		return -EINVAL;

	/*
	 * Call the registered function to set the pinconf
	 */
	return info->adi_pinconf_set(pctldev, pin_id,
				     configs, num_configs);
}

static void adi_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				 struct seq_file *s, unsigned int pin_id)
{
	struct adi_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct adi_pinctrl_soc_info *info = ipctl->info;
	unsigned long config;
	int ret;

	ret = info->adi_pinconf_get(pctldev, pin_id, &config);
	if (ret) {
		seq_puts(s, "N/A");
		return;
	}

	seq_printf(s, "0x%lx", config);
}

static void adi_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
				       struct seq_file *s, unsigned int group)
{
	struct group_desc *grp;
	unsigned long config;
	const char *name;
	int i, ret;

	if (group >= pctldev->num_groups)
		return;

	seq_puts(s, "\n");
	grp = pinctrl_generic_get_group(pctldev, group);
	if (!grp)
		return;

	for (i = 0; i < grp->grp.npins; i++) {
		struct adi_pin *pin = &((struct adi_pin *)(grp->data))[i];

		name = pin_get_name(pctldev, pin->pin);
		ret = adi_pinconf_get(pctldev, pin->pin, &config);
		if (ret)
			return;
		seq_printf(s, "  %s: 0x%lx\n", name, config);
	}
}

static const struct pinconf_ops adi_pinconf_ops = {
	.pin_config_get			= adi_pinconf_get,
	.pin_config_set			= adi_pinconf_set,
	.pin_config_dbg_show		= adi_pinconf_dbg_show,
	.pin_config_group_dbg_show	= adi_pinconf_group_dbg_show,
};

/*
 * Each pin represented in adi,pins consists of a number of u32 PIN_FUNC_ID
 * and 1 u32 CONFIG, the total size is PIN_FUNC_ID + CONFIG for each pin.
 *
 * PIN_FUNC_ID format:
 * Default:
 *     <Pin# src_mux_sel config_word>
 */
#define ADI_PIN_SIZE 12

static int adi_pinctrl_parse_groups(struct device_node *np,
				    struct group_desc *grp,
				    struct adi_pinctrl *ipctl,
				    u32 index)
{
	const struct adi_pinctrl_soc_info *info = ipctl->info;
	struct adi_pin *pin;
	unsigned int *pins;
	int size, pin_size;
	const __be32 *list;
	int i;

	pin_size = ADI_PIN_SIZE;
	grp->grp.name = np->name;

	/*
	 * the binding format is adi,pins = <PIN_FUNC_ID CONFIG ...>,
	 * do sanity check and calculate pins number
	 */
	list = of_get_property(np, "adi,pins", &size);
	if (!list)
		return -EINVAL;

	/* we do not check return since it's safe node passed down */
	if (!size || size % pin_size)
		return -EINVAL;

	grp->grp.npins = size / pin_size;
	grp->data = devm_kcalloc(ipctl->dev,
				 grp->grp.npins, sizeof(struct adi_pin),
				 GFP_KERNEL);
	pins = devm_kcalloc(ipctl->dev,
			    grp->grp.npins, sizeof(unsigned int),
			    GFP_KERNEL);
	grp->grp.pins = pins;

	if (!grp->grp.pins || !grp->data)
		return -ENOMEM;

	for (i = 0; i < grp->grp.npins; i++) {
		pin = &((struct adi_pin *)(grp->data))[i];
		if (info->adi_pinctrl_parse_pin)
			info->adi_pinctrl_parse_pin(ipctl, &pins[i],
						    pin, &list);
		else
			adi_pinctrl_parse_pin(ipctl, &pins[i],
					      pin, &list, np);
	}

	return 0;
}

static int adi_pinctrl_parse_functions(struct device_node *np,
				       struct adi_pinctrl *ipctl,
				       u32 index)
{
	struct pinctrl_dev *pctl = ipctl->pctl;
	struct device_node *child;
	struct function_desc *func;
	struct group_desc *grp;
	const char **group_names;
	u32 i = 0;

	func = pinmux_generic_get_function(pctl, index);
	if (!func)
		return -EINVAL;

	func->func.name = np->name;
	func->func.ngroups = of_get_child_count(np);
	if (func->func.ngroups == 0)
		return -EINVAL;

	group_names = devm_kcalloc(ipctl->dev, func->func.ngroups,
				   sizeof(char *), GFP_KERNEL);
	if (!group_names)
		return -ENOMEM;

	func->func.groups = group_names;

	for_each_child_of_node(np, child) {
		group_names[i] = child->name;
		grp = devm_kzalloc(ipctl->dev, sizeof(struct group_desc),
				   GFP_KERNEL);
		if (!grp) {
			of_node_put(child);
			return -ENOMEM;
		}

		mutex_lock(&ipctl->mutex);
		radix_tree_insert(&pctl->pin_group_tree,
				  ipctl->group_index++, grp);
		mutex_unlock(&ipctl->mutex);
		adi_pinctrl_parse_groups(child, grp, ipctl, i++);
	}

	return 0;
}

/*
 * Check if the DT contains pins in the direct child nodes. This indicates the
 * newer DT format to store pins. This function returns true if the first found
 * adi,pins property is in a child of np. Otherwise false is returned.
 */
static bool adi_pinctrl_dt_is_flat_functions(struct device_node *np)
{
	struct device_node *function_np;
	struct device_node *pinctrl_np;

	for_each_child_of_node(np, function_np) {
		if (of_property_read_bool(function_np, "adi,pins")) {
			of_node_put(function_np);
			return true;
		}

		for_each_child_of_node(function_np, pinctrl_np) {
			if (of_property_read_bool(pinctrl_np, "adi,pins")) {
				of_node_put(pinctrl_np);
				of_node_put(function_np);
				return false;
			}
		}
	}

	return true;
}

static int adi_pinctrl_probe_dt(struct platform_device *pdev,
				struct adi_pinctrl *ipctl)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	struct pinctrl_dev *pctl = ipctl->pctl;
	u32 nfuncs = 0;
	u32 i = 0;
	bool flat_funcs;

	if (!np)
		return -ENODEV;

	flat_funcs = adi_pinctrl_dt_is_flat_functions(np);
	if (flat_funcs) {
		nfuncs = 1;
	} else {
		nfuncs = of_get_child_count(np);
		if (nfuncs == 0)
			return -EINVAL;
	}

	for (i = 0; i < nfuncs; i++) {
		struct function_desc *function;

		function = devm_kzalloc(&pdev->dev, sizeof(*function),
					GFP_KERNEL);
		if (!function)
			return -ENOMEM;

		mutex_lock(&ipctl->mutex);
		radix_tree_insert(&pctl->pin_function_tree, i, function);
		mutex_unlock(&ipctl->mutex);
	}

	pctl->num_functions = nfuncs;
	ipctl->group_index = 0;
	if (flat_funcs) {
		pctl->num_groups = of_get_child_count(np);
	} else {
		pctl->num_groups = 0;
		for_each_child_of_node(np, child)
		pctl->num_groups += of_get_child_count(child);
	}

	if (flat_funcs) {
		adi_pinctrl_parse_functions(np, ipctl, 0);
	} else {
		i = 0;
		for_each_child_of_node(np, child)
		adi_pinctrl_parse_functions(child, ipctl, i++);
	}

	return 0;
}

int adi_pinctrl_probe(struct platform_device *pdev,
		      const struct adi_pinctrl_soc_info *info)
{
	struct pinctrl_desc *adi_pinctrl_desc;
	struct adi_pinctrl *ipctl;
	int ret, i;
	const __be32 *prop;

	if (!info || !info->pins || !info->npins)
		return -EINVAL;

	/* Create state holders etc for this driver */
	ipctl = devm_kzalloc(&pdev->dev, sizeof(*ipctl), GFP_KERNEL);
	if (!ipctl)
		return -ENOMEM;
	ipctl->pin_regs = devm_kmalloc_array(&pdev->dev, info->npins,
					     sizeof(*ipctl->pin_regs),
					     GFP_KERNEL);
	if (!ipctl->pin_regs)
		return -ENOMEM;

	for (i = 0; i < info->npins; i++) {
		ipctl->pin_regs[i].mux_reg = -1;
		ipctl->pin_regs[i].conf_reg = -1;
	}

	adi_pinctrl_desc = devm_kzalloc(&pdev->dev, sizeof(*adi_pinctrl_desc),
					GFP_KERNEL);
	if (!adi_pinctrl_desc)
		return -ENOMEM;
	adi_pinctrl_desc->name = dev_name(&pdev->dev);
	adi_pinctrl_desc->pins = info->pins;
	adi_pinctrl_desc->npins = info->npins;
	adi_pinctrl_desc->pctlops = &adi_pctrl_ops;
	adi_pinctrl_desc->pmxops = &adi_pmx_ops;
	adi_pinctrl_desc->confops = &adi_pinconf_ops;
	adi_pinctrl_desc->owner = THIS_MODULE;
	mutex_init(&ipctl->mutex);
	ipctl->info = info;
	ipctl->dev = &pdev->dev;

	prop = of_get_property(pdev->dev.of_node, "reg", NULL);
	if (!prop)
		return -EINVAL;
	ipctl->phys_addr = of_read_number(prop, 1);

	platform_set_drvdata(pdev, ipctl);
	ret = devm_pinctrl_register_and_init(&pdev->dev,
					     adi_pinctrl_desc, ipctl,
					     &ipctl->pctl);
	if (ret)
		return ret;

	ret = adi_pinctrl_probe_dt(pdev, ipctl);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "initialized ADI pinctrl driver\n");
	return pinctrl_enable(ipctl->pctl);
}
EXPORT_SYMBOL_GPL(adi_pinctrl_probe);

static int __maybe_unused adi_pinctrl_suspend(struct device *dev)
{
	struct adi_pinctrl *ipctl = dev_get_drvdata(dev);

	return pinctrl_force_sleep(ipctl->pctl);
}

static int __maybe_unused adi_pinctrl_resume(struct device *dev)
{
	struct adi_pinctrl *ipctl = dev_get_drvdata(dev);

	return pinctrl_force_default(ipctl->pctl);
}

const struct dev_pm_ops adi_pinctrl_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(adi_pinctrl_suspend,
				     adi_pinctrl_resume)
};
EXPORT_SYMBOL_GPL(adi_pinctrl_pm_ops);

MODULE_AUTHOR("Howard Massey <Howard.Massey@analog.com>");
MODULE_DESCRIPTION("ADI pinctrl driver");
MODULE_LICENSE("GPL v2");
