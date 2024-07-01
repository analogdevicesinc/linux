// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Core driver for the S32V pin controller
 *
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 * Copyright (C) 2017 NXP
 *
 * Based on pinctrl-imx.c:
 *	Author: Dong Aisheng <dong.aisheng@linaro.org>
 *	Copyright (C) 2012 Freescale Semiconductor, Inc.
 *	Copyright (C) 2012 Linaro Ltd.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "../core.h"
#include "pinctrl-s32v.h"

/**
 * @dev: a pointer back to containing device
 * @base: the offset to the controller in virtual memory
 */
struct s32v_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctl;
	void __iomem *base;
	const struct s32v_pinctrl_soc_info *info;
};

static const char *pin_get_name_from_info(struct s32v_pinctrl_soc_info *info,
					  const unsigned int pin_id)
{
	int i;

	for (i = 0; i < info->npins; i++) {
		if (info->pins[i].number == pin_id)
			return info->pins[i].name;
	}

	return NULL;
}

static inline const struct s32v_pin_group *s32v_pinctrl_find_group_by_name(
				const struct s32v_pinctrl_soc_info *info,
				const char *name)
{
	const struct s32v_pin_group *grp = NULL;
	unsigned int i;

	for (i = 0; i < info->ngroups; i++) {
		if (!strcmp(info->groups[i].name, name)) {
			grp = &info->groups[i];
			break;
		}
	}

	return grp;
}

static int s32v_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;

	return info->ngroups;
}

static const char *s32v_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned int selector)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;

	return info->groups[selector].name;
}

static int s32v_get_group_pins(struct pinctrl_dev *pctldev,
			       unsigned int selector, const unsigned int **pins,
			       unsigned int *npins)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;

	if (selector >= info->ngroups)
		return -EINVAL;

	*pins = info->groups[selector].pin_ids;
	*npins = info->groups[selector].npins;

	return 0;
}

static void s32v_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			      unsigned int offset)
{
	seq_printf(s, "%s", dev_name(pctldev->dev));
}

static int s32v_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np,
			       struct pinctrl_map **map, unsigned int *num_maps)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;
	const struct s32v_pin_group *grp;
	struct pinctrl_map *new_map;
	struct device_node *parent;
	int map_num = 1;
	int i, j;

	/*
	 * first find the group of this node and check if we need create
	 * config maps for pins
	 */
	grp = s32v_pinctrl_find_group_by_name(info, np->name);
	if (!grp) {
		dev_err(info->dev, "unable to find group for node %s\n",
			np->name);
		return -EINVAL;
	}

	for (i = 0; i < grp->npins; i++)
		map_num++;

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
	for (i = j = 0; i < grp->npins; i++) {
		new_map[j].type = PIN_MAP_TYPE_CONFIGS_PIN;
		new_map[j].data.configs.group_or_pin =
			pin_get_name(pctldev, grp->pins[i].pin_id);
		new_map[j].data.configs.configs = &grp->pins[i].config;
		new_map[j].data.configs.num_configs = 1;
		j++;
	}

	dev_dbg(pctldev->dev, "maps: function %s group %s num %d\n",
		(*map)->data.mux.function, (*map)->data.mux.group, map_num);

	return 0;
}

static void s32v_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned int num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops s32v_pctrl_ops = {
	.get_groups_count = s32v_get_groups_count,
	.get_group_name = s32v_get_group_name,
	.get_group_pins = s32v_get_group_pins,
	.pin_dbg_show = s32v_pin_dbg_show,
	.dt_node_to_map = s32v_dt_node_to_map,
	.dt_free_map = s32v_dt_free_map,

};

static int s32v_pmx_set(struct pinctrl_dev *pctldev, unsigned int selector,
			unsigned int group)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;
	unsigned int npins, pin_id;
	int i;
	struct s32v_pin_group *grp;

	/*
	 * Configure the mux mode for each pin in the group for a specific
	 * function.
	 */
	grp = &info->groups[group];
	npins = grp->npins;

	dev_dbg(ipctl->dev, "enable function %s group %s\n",
		info->functions[selector].name, grp->name);

	for (i = 0; i < npins; i++) {
		struct s32v_pin *pin = &grp->pins[i];

		pin_id = pin->pin_id;

		writel(pin->config, ipctl->base + S32V_PAD_CONFIG(pin_id));
		dev_dbg(ipctl->dev, "write: offset 0x%x val %lu\n",
			S32V_PAD_CONFIG(pin_id), pin->config);
	}

	return 0;
}

static int s32v_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;

	return info->nfunctions;
}

static const char *s32v_pmx_get_func_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;

	return info->functions[selector].name;
}

static int s32v_pmx_get_groups(struct pinctrl_dev *pctldev,
			       unsigned int selector,
			       const char * const **groups,
			       unsigned int * const num_groups)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;

	*groups = info->functions[selector].groups;
	*num_groups = info->functions[selector].num_groups;

	return 0;
}

static const struct pinmux_ops s32v_pmx_ops = {
	.get_functions_count = s32v_pmx_get_funcs_count,
	.get_function_name = s32v_pmx_get_func_name,
	.get_function_groups = s32v_pmx_get_groups,
	.set_mux = s32v_pmx_set,
};

static int s32v_pinconf_get(struct pinctrl_dev *pctldev,
			    unsigned int pin_id, unsigned long *config)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);

	*config = readl(ipctl->base + S32V_PAD_CONFIG(pin_id));

	return 0;
}

static int s32v_pinconf_set(struct pinctrl_dev *pctldev,
			    unsigned int pin_id, unsigned long *configs,
			    unsigned int num_configs)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	int i;

	dev_dbg(ipctl->dev, "pinconf set pin %s\n",
		pin_get_name(pctldev, pin_id));

	for (i = 0; i < num_configs; i++) {
		writel(configs[i], ipctl->base + S32V_PAD_CONFIG(pin_id));
		dev_dbg(ipctl->dev, "write: offset 0x%x val 0x%lx\n",
			S32V_PAD_CONFIG(pin_id), configs[i]);
	} /* for each config */

	return 0;
}

static void s32v_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				  struct seq_file *s, unsigned int pin_id)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned long config;

	config = readl(ipctl->base + S32V_PAD_CONFIG(pin_id));
	seq_printf(s, "0x%lx", config);
}

static void s32v_pinconf_group_dbg_show(struct pinctrl_dev *pctldev,
					struct seq_file *s, unsigned int group)
{
	struct s32v_pinctrl *ipctl = pinctrl_dev_get_drvdata(pctldev);
	const struct s32v_pinctrl_soc_info *info = ipctl->info;
	struct s32v_pin_group *grp;
	unsigned long config;
	const char *name;
	int i, ret;

	if (group > info->ngroups)
		return;

	seq_puts(s, "\n");
	grp = &info->groups[group];
	for (i = 0; i < grp->npins; i++) {
		struct s32v_pin *pin = &grp->pins[i];

		name = pin_get_name(pctldev, pin->pin_id);
		ret = s32v_pinconf_get(pctldev, pin->pin_id, &config);
		if (ret)
			return;
		seq_printf(s, "%s: 0x%lx", name, config);
	}
}

static const struct pinconf_ops s32v_pinconf_ops = {
	.pin_config_get = s32v_pinconf_get,
	.pin_config_set = s32v_pinconf_set,
	.pin_config_dbg_show = s32v_pinconf_dbg_show,
	.pin_config_group_dbg_show = s32v_pinconf_group_dbg_show,
};

static struct pinctrl_desc s32v_pinctrl_desc = {
	.pctlops = &s32v_pctrl_ops,
	.pmxops = &s32v_pmx_ops,
	.confops = &s32v_pinconf_ops,
	.owner = THIS_MODULE,
};

/*
 * Each pin represented in fsl,pins consists of 5 u32 PIN_FUNC_ID and
 * 1 u32 CONFIG, so 24 types in total for each pin.
 */
#define FSL_PIN_SIZE 24
#define SHARE_FSL_PIN_SIZE 20

static int s32v_pinctrl_parse_groups(struct device_node *np,
				     struct s32v_pin_group *grp,
				     struct s32v_pinctrl_soc_info *info,
				     u32 index)
{
	int size, i;
	const __be32 *list;

	dev_dbg(info->dev, "group(%d): %s\n", index, np->name);

	/* Initialise group */
	grp->name = np->name;

	/*
	 * the binding format is fsl,pins = <PIN CONFIG>,
	 * do sanity check and calculate pins number
	 */
	list = of_get_property(np, "fsl,pins", &size);
	if (!list) {
		dev_err(info->dev, "no fsl,pins property in node %s\n",
			np->full_name);
		return -EINVAL;
	}

	/* we do not check return since it's safe node passed down */
	if (!size || size % S32V_PIN_SIZE) {
		dev_err(info->dev, "Invalid fsl,pins property in node %s\n",
			np->full_name);
		return -EINVAL;
	}

	grp->npins = size / S32V_PIN_SIZE;
	grp->pins = devm_kzalloc(info->dev,
				 grp->npins * sizeof(struct s32v_pin),
				 GFP_KERNEL);
	grp->pin_ids = devm_kzalloc(info->dev,
				    grp->npins * sizeof(unsigned int),
				    GFP_KERNEL);
	if (!grp->pins || !grp->pin_ids)
		return -ENOMEM;

	for (i = 0; i < grp->npins; i++) {
		struct s32v_pin *pin = &grp->pins[i];

		pin->pin_id = be32_to_cpu(*list++);
		pin->config = be32_to_cpu(*list++);
		grp->pin_ids[i] = grp->pins[i].pin_id;

		dev_dbg(info->dev, "%s: 0x%08lx",
			pin_get_name_from_info(info, pin->pin_id), pin->config);
	}

	return 0;
}

static int s32v_pinctrl_parse_functions(struct device_node *np,
					struct s32v_pinctrl_soc_info *info,
					u32 index)
{
	struct device_node *child;
	struct s32v_pmx_func *func;
	struct s32v_pin_group *grp;
	static u32 grp_index;
	u32 i = 0;

	dev_dbg(info->dev, "parse function(%d): %s\n", index, np->name);

	func = &info->functions[index];

	/* Initialise function */
	func->name = np->name;
	func->num_groups = of_get_child_count(np);
	if (func->num_groups == 0) {
		dev_err(info->dev, "no groups defined in %s\n", np->full_name);
		return -EINVAL;
	}
	func->groups = devm_kzalloc(info->dev,
				    func->num_groups * sizeof(char *),
				    GFP_KERNEL);

	for_each_child_of_node(np, child) {
		func->groups[i] = child->name;
		grp = &info->groups[grp_index++];
		s32v_pinctrl_parse_groups(child, grp, info, i++);
	}

	return 0;
}

static int s32v_pinctrl_probe_dt(struct platform_device *pdev,
				 struct s32v_pinctrl_soc_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *child;
	u32 nfuncs = 0;
	u32 i = 0;

	if (!np)
		return -ENODEV;

	nfuncs = of_get_child_count(np);
	if (nfuncs <= 0) {
		dev_err(&pdev->dev, "no functions defined\n");
		return -EINVAL;
	}

	info->nfunctions = nfuncs;
	info->functions = devm_kzalloc(&pdev->dev,
				       nfuncs * sizeof(struct s32v_pmx_func),
				       GFP_KERNEL);
	if (!info->functions)
		return -ENOMEM;

	info->ngroups = 0;
	for_each_child_of_node(np, child)
		info->ngroups += of_get_child_count(child);
	info->groups = devm_kzalloc(&pdev->dev, info->ngroups *
						sizeof(struct s32v_pin_group),
				    GFP_KERNEL);
	if (!info->groups)
		return -ENOMEM;

	for_each_child_of_node(np, child)
		s32v_pinctrl_parse_functions(child, info, i++);

	return 0;
}

int s32v_pinctrl_probe(struct platform_device *pdev,
		       struct s32v_pinctrl_soc_info *info)
{
	struct s32v_pinctrl *ipctl;
	struct resource *res;
	int ret;

	if (!info || !info->pins || !info->npins) {
		dev_err(&pdev->dev, "wrong pinctrl info\n");
		return -EINVAL;
	}
	info->dev = &pdev->dev;

	/* Create state holders etc for this driver */
	ipctl = devm_kzalloc(&pdev->dev, sizeof(*ipctl), GFP_KERNEL);
	if (!ipctl)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ipctl->base = devm_ioremap_resource(&pdev->dev, res);

	if (IS_ERR(ipctl->base))
		return PTR_ERR(ipctl->base);

	s32v_pinctrl_desc.name = dev_name(&pdev->dev);
	s32v_pinctrl_desc.pins = info->pins;
	s32v_pinctrl_desc.npins = info->npins;

	ret = s32v_pinctrl_probe_dt(pdev, info);
	if (ret) {
		dev_err(&pdev->dev, "fail to probe dt properties\n");
		return ret;
	}

	ipctl->info = info;
	ipctl->dev = info->dev;
	platform_set_drvdata(pdev, ipctl);
	ipctl->pctl = pinctrl_register(&s32v_pinctrl_desc, &pdev->dev, ipctl);
	if (!ipctl->pctl) {
		dev_err(&pdev->dev, "could not register s32 pinctrl driver\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "initialized s32 pinctrl driver\n");

	return 0;
}

void s32v_pinctrl_remove(struct platform_device *pdev)
{
	struct s32v_pinctrl *ipctl = platform_get_drvdata(pdev);

	pinctrl_unregister(ipctl->pctl);
}
