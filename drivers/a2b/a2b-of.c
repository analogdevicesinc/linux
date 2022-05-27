// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/export.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

#define A2B_MAX_TDM_SLOTS 32

static int a2b_get_slot_mask(const struct device_node *np, const char *propname,
			     u32 *mask)
{
	u32 slots[A2B_MAX_TDM_SLOTS];
	unsigned int i, num;
	int ret, proplen;

	if (!of_get_property(np, propname, &proplen))
		return 0;

	num = proplen / sizeof(u32);

	if (num > ARRAY_SIZE(slots))
		return -EINVAL;

	ret = of_property_read_u32_array(np, propname, slots, num);
	if (ret < 0)
		return ret;

	*mask = 0;

	for (i = 0; i < num; i++) {
		if (slots[i] >= A2B_MAX_TDM_SLOTS)
			return -EINVAL;

		*mask |= BIT(slots[i]);
	}

	return 0;
}

static int a2b_of_read_slot_config(struct device *dev, struct device_node *np,
				   struct a2b_slot_config *config)
{
	struct device_node *dn_np, *up_np;
	int ret = 0;

	memset(config, 0, sizeof(*config));

	dn_np = of_get_child_by_name(np, "downstream");
	if (dn_np) {
		ret = a2b_get_slot_mask(dn_np, "rx-slots",
					&config->dn_rx_slots);
		if (ret < 0) {
			dev_err(dev, "invalid downstream rx-slots property\n");
			goto err_put_dn_node;
		}

		of_property_read_u32(dn_np, "#tx-slots",
				     &config->dn_n_tx_slots);
		of_property_read_u32(dn_np, "#forward-slots",
				     &config->dn_n_forward_slots);
		if (config->dn_n_tx_slots + config->dn_n_forward_slots >= 32) {
			dev_err(dev, "invalid downstream tx-slots property\n");
			goto err_put_dn_node;
		}
	}

	up_np = of_get_child_by_name(np, "upstream");
	if (up_np) {
		ret = a2b_get_slot_mask(up_np, "rx-slots",
					&config->up_rx_slots);
		if (ret < 0) {
			dev_err(dev, "invalid upstream rx-slots property\n");
			goto err_put_up_node;
		}

		of_property_read_u32(up_np, "#tx-slots",
				     &config->up_n_tx_slots);
		of_property_read_u32(up_np, "#forward-slots",
				     &config->up_n_forward_slots);
		if (config->up_n_tx_slots + config->up_n_forward_slots >= 32) {
			dev_err(dev, "invalid upstream tx-slots property\n");
			goto err_put_up_node;
		}
	}

err_put_up_node:
	of_node_put(up_np);
err_put_dn_node:
	of_node_put(dn_np);

	return ret;
}

static int a2b_of_subnode_read_config(struct a2b_subnode *subnode)
{
	struct device *dev = &subnode->node.dev;
	struct device_node *np = dev->of_node;
	int ret;

	of_property_read_string(np, "label", &subnode->node.label);

	ret = a2b_of_read_slot_config(dev, np, &subnode->slot_config);
	if (ret < 0) {
		dev_err(dev, "Slot config is invalid: %d\n", ret);
		return ret;
	}

	ret = a2b_of_read_transceiver_config(&subnode->node);
	if (ret)
		return ret;

	ret = a2b_of_read_pin_config(&subnode->node);
	if (ret)
		return ret;

	ret = a2b_of_read_pll_config(&subnode->node);
	if (ret)
		return ret;

	return a2b_of_read_tdm_config(&subnode->node);
}

/**
 * a2b_of_bus_enumerate - Enumerate the sub nodes of A2B bus from devicetree
 */
int a2b_of_bus_enumerate(struct a2b_mainnode *mainnode)
{
	struct device_node *sub_nodes[A2B_MAX_NODES] = {};
	struct device *dev = &mainnode->node.dev;
	struct device_node *nodes_np;
	struct device_node *sub_np;
	struct a2b_subnode *subnode;
	int n = 0;
	u32 val;
	int ret;
	int i;

	nodes_np = of_get_child_by_name(dev->of_node, "nodes");
	if (!nodes_np) {
		dev_err(dev, "no 'nodes' property given\n");
		return -EINVAL;
	}

	/*
	 * Nodes must be initialized in bus order, which may not match FDT
	 * order.
	 */
	for_each_available_child_of_node (nodes_np, sub_np) {
		ret = of_property_read_u32(sub_np, "reg", &val);
		if (ret) {
			dev_err(dev, "%pOF: Missing 'reg' proprety\n", sub_np);
			ret = -EINVAL;
			goto err_put_nodes;
		}

		if (val >= A2B_MAX_NODES) {
			dev_err(dev, "%pOF: Invalid sub-node address: %d\n",
				sub_np, val);
			ret = -EINVAL;
			goto err_put_nodes;
		}

		if (sub_nodes[val]) {
			dev_err(dev, "%pOF: Duplicate sub-node address: %d\n",
				sub_np, val);
			ret = -EINVAL;
			goto err_put_nodes;
		}

		sub_nodes[val] = sub_np;
		n++;
	}

	if (n == 0) {
		dev_err(dev, "No sub-nodes specified\n");
		ret = -EINVAL;
		goto err_put_nodes;
	}

	for (i = 0; i < n; i++) {
		sub_np = sub_nodes[i];
		if (!sub_np) {
			dev_err(dev, "Sub-node %d is missing\n", i);
			ret = -EINVAL;
			goto err_free_subnodes;
		}

		subnode = a2b_subnode_alloc(mainnode, i);
		if (IS_ERR(subnode)) {
			ret = PTR_ERR(subnode);
			goto err_free_subnodes;
		}
		subnode->node.dev.of_node = sub_np;
		mainnode->subnodes[i] = subnode;

		ret = a2b_of_subnode_read_config(subnode);
		if (ret)
			goto err_free_subnodes;
	}
	mainnode->num_subnodes = n;

	of_node_put(nodes_np);
	return 0;

err_free_subnodes:
	for (i = 0; i < n; i++) {
		if (mainnode->subnodes[i]) {
			a2b_subnode_free(mainnode->subnodes[i]);
			mainnode->subnodes[i] = NULL;
		}
	}

err_put_nodes:
	of_node_put(nodes_np);

	return ret;
}
EXPORT_SYMBOL_GPL(a2b_of_bus_enumerate);

int a2b_of_mainnode_read_config(struct a2b_mainnode *mainnode)
{
	struct device *dev = &mainnode->node.dev;
	struct device_node *np = dev->of_node;
	u32 val;
	int ret;

	of_property_read_string(np, "label", &mainnode->node.label);

	/* Slot format setup */

	val = 24;
	of_property_read_u32(np, "adi,upstream-slot-size", &val);
	if (val < 8 || val > 32 || (val % 4 != 0)) {
		dev_err(dev, "invalid upstream-slot-size %d\n", val);
		return -EINVAL;
	}
	mainnode->bus_config.up_slot_size = val;

	val = 24;
	of_property_read_u32(np, "adi,downstream-slot-size", &val);
	if (val < 8 || val > 32 || (val % 4 != 0)) {
		dev_err(dev, "invalid downstream-slot-size %d\n", val);
		return -EINVAL;
	}
	mainnode->bus_config.dn_slot_size = val;

	mainnode->bus_config.dn_slot_alt_fmt = of_property_read_bool(
		np, "adi,alternate-downstream-slot-format");
	mainnode->bus_config.up_slot_alt_fmt =
		of_property_read_bool(np, "adi,alternate-upstream-slot-format");

	ret = a2b_of_read_transceiver_config(&mainnode->node);
	if (ret)
		return ret;

	ret = a2b_of_read_pin_config(&mainnode->node);
	if (ret)
		return ret;

	ret = a2b_of_read_pll_config(&mainnode->node);
	if (ret)
		return ret;

	return a2b_of_read_tdm_config(&mainnode->node);
}

static int a2b_parse_lvds_power(struct a2b_node *node, const char *prop_name)
{
	struct device_node *np = node->dev.of_node;
	const char *power;
	int ret;

	ret = of_property_read_string(np, prop_name, &power);
	if (ret == -EINVAL)
		return 0;
	else if (ret < 0)
		return ret;

	ret = A2B_TXCTL_OVERRIDE;
	if (!strcmp(power, "low")) {
		ret |= A2B_TXCTL_LEVEL_LOW;
	} else if (!strcmp(power, "medium")) {
		ret |= A2B_TXCTL_LEVEL_MEDIUM;
	} else if (!strcmp(power, "high")) {
		ret |= A2B_TXCTL_LEVEL_HIGH;
	} else {
		dev_err(&node->dev, "%pOF: Invalid value '%s' for '%s'\n", np,
			power, prop_name);
		return -EINVAL;
	}

	return ret;
}

int a2b_of_read_transceiver_config(struct a2b_node *node)
{
	int val;

	val = a2b_parse_lvds_power(node, "adi,lvds-a-tx-power");
	if (val < 0)
		return val;
	node->transceiver_config.a_power = val;

	val = a2b_parse_lvds_power(node, "adi,lvds-b-tx-power");
	if (val < 0)
		return val;
	node->transceiver_config.b_power = val;

	node->transceiver_config.invert =
		of_property_read_bool(node->dev.of_node, "adi,invert-xcvr-b");

	return 0;
}
EXPORT_SYMBOL_GPL(a2b_of_read_transceiver_config);

int a2b_of_read_pin_config(struct a2b_node *node)
{
	struct device_node *np = node->dev.of_node;
	const char *drive;
	int ret;

	/* Power-on reset is 'high', lets use that as the default */
	node->pin_config = A2B_PIN_DRIVE_STRENGTH_HIGH;

	ret = of_property_read_string(np, "adi,drive-strength", &drive);
	if (ret == -EINVAL)
		return 0;
	else if (ret < 0)
		return ret;

	if (!strcmp(drive, "low")) {
		node->pin_config = A2B_PIN_DRIVE_STRENGTH_LOW;
	} else if (!strcmp(drive, "high")) {
		node->pin_config = A2B_PIN_DRIVE_STRENGTH_HIGH;
	} else {
		dev_err(&node->dev,
			"%pOF: Invalid value '%s' for 'drive-strength'\n", np,
			drive);
		return -EINVAL;
	}

	return 0;
}

int a2b_of_read_pll_config(struct a2b_node *node)
{
	struct device_node *np = node->dev.of_node;
	struct a2b_pll_config *cfg = &node->pll_config;
	u32 val;

	if (of_property_read_bool(np, "adi,spread-a2b-clock"))
		cfg->spread_spectrum_mode = A2B_SPREAD_SPECTRUM_A2B;
	else if (of_property_read_bool(np, "adi,spread-a2b-i2s-clock"))
		cfg->spread_spectrum_mode = A2B_SPREAD_SPECTRUM_A2B_I2S;

	cfg->spread_spectrum_high =
		of_property_read_bool(np, "adi,spread-spectrum-high");

	val = 4;
	of_property_read_u32(np, "adi,spread-specturum-frequency", &val);
	if (val < 4 || val > 7)
		return -EINVAL;

	cfg->spread_spectrum_frequency = val;

	return 0;
}

int a2b_of_read_tdm_config(struct a2b_node *node)
{
	struct a2b_tdm_config *cfg = &node->tdm_config;
	struct device *dev = &node->dev;
	struct device_node *np = dev->of_node;

	cfg->tdm_channels = 2;
	of_property_read_u32(np, "adi,tdm-mode", &cfg->tdm_channels);

	cfg->tdm_slot_size = 32;
	of_property_read_u32(np, "adi,tdm-slot-size", &cfg->tdm_slot_size);

	cfg->alt_sync = of_property_read_bool(np, "adi,alternating-sync");
	cfg->early_sync = of_property_read_bool(np, "adi,early-sync");
	cfg->invert_sync = of_property_read_bool(np, "adi,invert-sync");

	return 0;
}
