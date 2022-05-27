// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/of.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

// TODO: Remove
int a2b_node_init_extra_regmap(struct a2b_node *node)
{
	struct device_node *np = node->dev.of_node;
	struct device *dev = &node->dev;
	struct property *prop;
	const __be32 *p = NULL;
	u32 val, reg;
	int num_regs;
	int ret;
	int i;

	/* Get number of reg/value pairs in map */
	prop = of_find_property(np, "adi,reg-init", &i);
	if (!prop)
		return 0;

	i = i / sizeof(u32);
	if (i & 1) {
		dev_err(dev, "%pOF: 'node-regs' contains odd number of entries",
			np);
		return -EINVAL;
	}
	num_regs = i / 2;

	for (i = 0; i < num_regs; i++) {
		p = of_prop_next_u32(prop, p, &reg);
		if (!p)
			return -EINVAL;
		if (reg > A2B_MAX_REG)
			return -EINVAL;
		p = of_prop_next_u32(prop, p, &val);
		if (!p)
			return -EINVAL;
		if (val > 0xFF)
			return -EINVAL;
		ret = a2b_node_write(node, reg, val);
		if (ret < 0)
			return ret;
	}

	return 0;
}
