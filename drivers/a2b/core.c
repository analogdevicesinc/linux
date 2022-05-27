// SPDX-License-Identifier: GPL-2.0-only

#include <linux/device.h>
#include <linux/export.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

static DEFINE_IDA(a2b_main_node_ida);

static struct bus_type a2b_bus = {
	.name = "a2b",
	.match = a2b_match,
};

#define A2B_MAIN_NODE_ID 0xff

struct a2b_node {
	unsigned int node_id;
	struct regmap *regmap;
};

struct a2b_main_node {
	struct a2b_node node;
	unsigned int bus_id;

	struct i2c_client *bus_client;
};

struct a2b_sub_node {
	struct a2b_node node;

	struct a2b_main_node *main_node;
};

static struct a2b_sub_node *a2b_sub_node_alloc(struct a2b_main_node *main_node,
	unsigned int id)
{
	struct a2b_sub_node *sub_node;

	sub_node = kzalloc(sizeof(*sub_node), GFP_KERNEL);
	if (!sub_node)
		return NULL;

	sub_node->main_node = main_node;
	sub_node->node.id = id;
	sub_node->node.dev.parent = &main_node->dev;
	sub_node->node.dev.bus = &a2b_bus;
	sub_node->node.dev.type = &a2b_sub_node_type;

	dev_set_name(&sub_node->node.dev, "a2b%d.%d", main_node->bus_id, id);

	device_initialize(&sub_node->node.dev);

	return sub_node;
}

static void a2b_main_node_release(struct device *dev)
{
	struct a2b_main_node *main_node = dev_to_a2b_main_node(dev);

	ida_simple_remove(&a2b_main_node_ida, main_node->bus_id);
	kfree(main_node);
}

static struct device_type a2b_main_node_type = {
	.release = a2b_main_node_release,
};

static void a2b_sub_node_release(struct device *dev)
{
	struct a2b_sub_node *sub_node = dev_to_a2b_sub_node(dev);

	kfree(sub_node);
}

static struct device_type a2b_sub_node_type = {
	.release = a2b_sub_node_release,
};

struct a2b_main_node *a2b_main_node_register(struct device *parent,
	int irq)
{
	struct a2b_main_node *main_node;
	int id;

	main_node = kzalloc(sizeof(*main_node), GFP_KERNEL);
	if (!main_node)
		return ERR_PTR(-ENOMEM);

	id = ida_simple_get(&a2b_main_node_ida, 0, 0, GFP_KERNEL);
	if (id < 0)
		return ERR_PTR(id);

	main_node->bus_id = id;

	main_node->node.dev.parent = parent;
	main_node->node.dev.bus = &a2b_bus;
	main_node->node.dev.type = &a2b_main_node_type;
	main_node->node.dev.of_node = parent->of_node;
	dev_set_name(main_node->node.dev, "a2b%d", main_node->bus_id);

	ret = device_register(&main_node->node.dev);
	if (ret) {
		put_device(&main_node->node.dev);
		return ERR_PTR(ret);
	

	return main_node;
}

static int a2b_unregister_sub_nodes(struct device *dev, void *data)
{
	if (dev->type == &a2b_sub_node_type)
		device_unregister(dev);

	return 0;
}

void a2b_main_node_unregister(struct a2b_main_node *main_node)
{
	device_for_each_child(&main_node.node->dev, NULL,
		a2b_unregister_sub_nodes);

	device_unregister(main_node->node.dev);
}

static int __init a2b_init(void)
{
	return bus_register(&a2b_bus);
}
subsys_initcal(a2b_init);

static void __exit a2b_exit(void)
{
	bus_unregister(&a2b_bus);
	ida_destroy(&a2b_ida);
}
module_exit(a2b_exit);

MODULE_LICENSE("GPL v2");
