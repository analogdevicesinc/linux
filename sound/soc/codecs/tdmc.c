/*
 * TDM-C bus support
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/err.h>

#include "tdmc.h"

static DEFINE_IDA(tdmc_master_ida);

static struct tdmc_master *dev_to_tdmc_master(struct device *dev)
{
	return container_of(dev, struct tdmc_master, dev);
}

static struct tdmc_slave *dev_to_tdmc_slave(struct device *dev)
{
	return container_of(dev, struct tdmc_slave, dev);
}

static struct tdmc_slave_driver *driver_to_tdmc_slave_driver(
	struct device_driver *driver)
{
	return container_of(driver, struct tdmc_slave_driver, driver);
}

static void tdmc_master_release(struct device *dev)
{
	struct tdmc_master *master = dev_to_tdmc_master(dev);

	ida_simple_remove(&tdmc_master_ida, master->id);
	mutex_destroy(&master->lock);
	kfree(master);
}

static struct device_type tdmc_master_type = {
	.release = tdmc_master_release,
};

static void tdmc_slave_release(struct device *dev)
{
	struct tdmc_slave *slave = dev_to_tdmc_slave(dev);
	kfree(slave);
}

static struct device_type tdmc_slave_type = {
	.release = tdmc_slave_release,
};

static int tdmc_device_probe(struct device *dev)
{
	struct tdmc_slave_driver *driver;
	struct tdmc_slave *slave;
	int ret;

	if (dev->type != &tdmc_slave_type)
		return 0;

	slave = dev_to_tdmc_slave(dev);
	driver = driver_to_tdmc_slave_driver(dev->driver);

	if (slave->master->ops->attach_slave) {
		ret = slave->master->ops->attach_slave(slave->master, slave);
		if (ret)
			return ret;
	}

	ret = driver->probe(slave);
	if (ret)
		goto err;

	return 0;
err:
	if (slave->master->ops->detach_slave)
		slave->master->ops->detach_slave(slave->master, slave);
	return ret;
}

static int tdmc_device_remove(struct device *dev)
{
	struct tdmc_slave_driver *driver;
	struct tdmc_slave *slave;

	if (dev->type != &tdmc_slave_type)
		return 0;

	slave = dev_to_tdmc_slave(dev);
	driver = driver_to_tdmc_slave_driver(dev->driver);

	if (driver->remove)
		driver->remove(slave);

	if (slave->master->ops->detach_slave)
		slave->master->ops->detach_slave(slave->master, slave);

	return 0;
}

static int tdmc_device_match(struct device *dev, struct device_driver *driver)
{
	return of_driver_match_device(dev, driver);
}

static struct bus_type tdmc_bus = {
	.name = "tdmc",
	.match = tdmc_device_match,
	.probe = tdmc_device_probe,
	.remove = tdmc_device_remove,
};

static struct tdmc_slave *tdmc_slave_alloc(struct tdmc_master *master,
	unsigned int port)
{
	struct tdmc_slave *slave;

	slave = kzalloc(sizeof(*slave), GFP_KERNEL);
	if (!slave)
		return NULL;

	slave->master = master;
	slave->port = port;
	slave->dev.parent = &master->dev;
	slave->dev.bus = &tdmc_bus;
	slave->dev.type = &tdmc_slave_type;

	dev_set_name(&slave->dev, "tdmc%d.%d", master->id, slave->port);

	device_initialize(&slave->dev);

	return slave;
}

static void tdmc_of_register_devices(struct tdmc_master *master)
{
	struct tdmc_slave *slave;
	struct device_node *np;
	u32 port;
	int ret;

	for_each_child_of_node(master->dev.of_node, np) {
		ret = of_property_read_u32(np, "reg", &port);
		if (ret) {
			dev_err(&master->dev,
				"Missing or invalid 'reg' property for child node %s: %d\n",
				np->full_name, ret);
			continue;
		}

		slave = tdmc_slave_alloc(master, port);
		if (!slave) {
			dev_err(&master->dev,
				"Failed to allocate slave for node %s\n",
				np->full_name);
			continue;
		}

		slave->dev.of_node = np;

		ret = device_add(&slave->dev);
		if (ret) {
			put_device(&slave->dev);
			dev_err(&master->dev,
				"Failed to register slave for node %s\n",
				np->full_name);
			continue;
		}
	}
}

static struct tdmc_master *tdmc_master_register(struct device *parent,
	const struct tdmc_ops *ops, void *drvdata)
{
	struct tdmc_master *master;
	int ret;
	int id;

	master = kzalloc(sizeof(*master), GFP_KERNEL);
	if (!master)
		return ERR_PTR(-ENOMEM);

	id = ida_simple_get(&tdmc_master_ida, 0, 0, GFP_KERNEL);
	if (id < 0) {
		kfree(master);
		return ERR_PTR(id);
	}

	mutex_init(&master->lock);

	master->id = id;
	master->ops = ops;
	master->dev.parent = parent;
	master->dev.bus = &tdmc_bus;
	master->dev.type = &tdmc_master_type;
	master->dev.of_node = parent->of_node;
	dev_set_name(&master->dev, "tdmc%d", master->id);
	dev_set_drvdata(&master->dev, drvdata);

	ret = device_register(&master->dev);
	if (ret) {
		put_device(&master->dev);
		return ERR_PTR(ret);
	}

	if (master->dev.of_node)
		tdmc_of_register_devices(master);

	return master;
}

static int tdmc_master_unregister_slave(struct device *dev, void *data)
{
	if (dev->type == &tdmc_slave_type)
		device_unregister(dev);

	return 0;
}

static void tdmc_master_unregister(struct tdmc_master *master)
{
	device_for_each_child(&master->dev, NULL, tdmc_master_unregister_slave);
	mutex_lock(&master->lock);
	master->ops = NULL;
	mutex_unlock(&master->lock);
	device_unregister(&master->dev);
}

static void devm_tdmc_master_release(struct device *dev, void *res)
{
	struct tdmc_master **master = res;

	tdmc_master_unregister(*master);
}

/**
 * devm_tdmc_master_register() - Register a new TDM-C master device
 * @parent: The parent device for the TDM-C master
 * @ops: TDM-C master implementation specific callbacks.
 * @drvdata: TDM-C master specific driver data that can be used in the
 *  callbacks.
 *
 * The master will automatically be unregistered if the parent device is
 * removed.
 *
 * Returns: 0 when successful, otherwise a negative error code.
 */
int devm_tdmc_master_register(struct device *parent,
	const struct tdmc_ops *ops, void *drvdata)
{
	struct tdmc_master **master;
	int ret;

	master = devres_alloc(devm_tdmc_master_release, sizeof(*master),
		GFP_KERNEL);

	*master = tdmc_master_register(parent, ops, drvdata);
	if (IS_ERR(*master)) {
		ret = PTR_ERR(*master);
		devres_free(master);
		return ret;
	}

	devres_add(parent, master);

	return 0;
}
EXPORT_SYMBOL_GPL(devm_tdmc_master_register);

/**
 * tdmc_slave_driver_register() - Register a TDM-C slave driver
 * @owner: Module where the driver is defined.
 * @driver: The TDM-C slave driver to register.
 *
 * Do not call this directly, use the module_tdmc_driver() macro instead.
 *
 * Returns: 0 when successful, otherwise a negative error code.
 */
int tdmc_slave_driver_register(struct module *owner,
	struct tdmc_slave_driver *driver)
{
	driver->driver.bus = &tdmc_bus;
	driver->driver.owner = owner;

	return driver_register(&driver->driver);
}
EXPORT_SYMBOL_GPL(tdmc_slave_driver_register);

/**
 * tdmc_slave_driver_unregister() - Unregister a TDM-C slave driver
 * @driver: The TDM-C slave driver to unregister
 *
 * Do not call this directly, use the module_tdmc_driver() macro instead.
 */
void tdmc_slave_driver_unregister(struct tdmc_slave_driver *driver)
{
	driver_unregister(&driver->driver);
}
EXPORT_SYMBOL_GPL(tdmc_slave_driver_unregister);

static int tdmc_regmap_write(struct device *dev, const void *data, size_t count)
{
	struct tdmc_slave *slave = dev_to_tdmc_slave(dev);
	struct tdmc_master *master = slave->master;
	unsigned int reg, val;
	int ret;

	if (WARN_ON(count != 8))
		return -EINVAL;

	reg = be32_to_cpu(((__be32 *)data)[0]);
	val = be32_to_cpu(((__be32 *)data)[1]);

	mutex_lock(&master->lock);
	if (master->ops)
		ret = master->ops->write(master, slave, reg, val);
	else
		ret = -ENODEV;
	mutex_unlock(&master->lock);

	return ret;
}

static int tdmc_regmap_read(struct device *dev, const void *reg_buf,
	size_t reg_size, void *val_buf, size_t val_size)
{
	struct tdmc_slave *slave = dev_to_tdmc_slave(dev);
	struct tdmc_master *master = slave->master;
	unsigned int reg;
	unsigned int val;
	int ret;

	if (WARN_ON(reg_size != 4 || val_size != 4))
		return -EINVAL;
	
	reg = be32_to_cpu(((__be32 *)reg_buf)[0]);

	mutex_lock(&master->lock);
	if (master->ops) {
		if (master->ops->read) {
			ret = master->ops->read(master, slave, reg, &val);
			((__be32 *)val_buf)[0] = cpu_to_be32(val);
		} else
			ret = -ENOSYS;
	} else {
		ret = -ENODEV;
	}
	mutex_unlock(&master->lock);

	return ret;
}

static const struct regmap_bus tdmc_regmap_bus = {
	.write = tdmc_regmap_write,
	.read = tdmc_regmap_read,
};

/**
 * devm_tdmc_regmap_init() - Create managed regmap struct for a TDM-C slave
 * @slave: The slave for which to create a regmap struct
 * @config: Register map configuration for the slave
 *
 * The regmap struct will be freed automatically when the slave is unbound.
 *
 * Returns: A valid pointer to a regmap struct on success, or a ERR_PTR() on
 * error.
 */
struct regmap *devm_tdmc_regmap_init(struct tdmc_slave *slave,
	const struct regmap_config *config)
{
	return devm_regmap_init(&slave->dev, &tdmc_regmap_bus, config);
}
EXPORT_SYMBOL_GPL(devm_tdmc_regmap_init);

static int tdmc_init(void)
{
	return bus_register(&tdmc_bus);
}
postcore_initcall(tdmc_init);

static void tdmc_exit(void)
{
	bus_unregister(&tdmc_bus);
}
module_exit(tdmc_exit);
