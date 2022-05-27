// SPDX-License-Identifier: GPL-2.0-only

#include <linux/regmap.h>
#include <linux/i2c.h>

#include <linux/a2b/a2b.h>
#include <linux/a2b/a2b-regs.h>

static int a2b_subnode_read(void *context, unsigned int reg, unsigned int *val)
{
	struct a2b_subnode *node = context;
	struct a2b_mainnode *mainnode = node->mainnode;
	int ret;

	if (reg > 0xff)
		return -EINVAL;

	mutex_lock(&mainnode->bus_lock);
	ret = regmap_write(mainnode->node.regmap, A2B_NODEADR, node->node.id);
	if (ret == 0)
		ret = i2c_smbus_read_byte_data(mainnode->bus_client, reg);
	mutex_unlock(&mainnode->bus_lock);

	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

static int a2b_subnode_write(void *context, unsigned int reg, unsigned int val)
{
	struct a2b_subnode *node = context;
	struct a2b_mainnode *mainnode = node->mainnode;
	int ret;

	if (reg > 0xff || val > 0xff)
		return -EINVAL;

	mutex_lock(&mainnode->bus_lock);
	ret = regmap_write(mainnode->node.regmap, A2B_NODEADR, node->node.id);
	if (ret == 0)
		ret = i2c_smbus_write_byte_data(mainnode->bus_client, reg, val);
	mutex_unlock(&mainnode->bus_lock);

	return ret;
}

static const struct regmap_bus a2b_subnode_regmap_bus = {
	.reg_read = a2b_subnode_read,
	.reg_write = a2b_subnode_write,
};

struct regmap *a2b_subnode_regmap_init(struct a2b_subnode *subnode,
				       const struct regmap_config *config)
{
	return regmap_init(&subnode->node.dev, &a2b_subnode_regmap_bus, subnode,
			   config);
}

/*
 * Main-node always as a regmap, subnode only after probe
 * These functions are used pre-probe when enumerating the bus
 */
int a2b_node_read(struct a2b_node *node, uint8_t reg)
{
	unsigned int val;
	int ret;

	if (node->id == A2B_MAIN_NODE_ID)
		ret = regmap_read(node->regmap, reg, &val);
	else
		ret = a2b_subnode_read(a2b_node_to_subnode(node), reg, &val);
	if (ret)
		return ret;

	return val;
}

int a2b_node_write(struct a2b_node *node, uint8_t reg, uint8_t val)
{
	if (node->id == A2B_MAIN_NODE_ID)
		return regmap_write(node->regmap, reg, val);
	else
		return a2b_subnode_write(a2b_node_to_subnode(node), reg, val);
}
