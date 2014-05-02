/*
 * TDM-C bus support
 *
 * Copyright 2014 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */
#ifndef __SOUND_SOC_CODECS_tdmc_H__
#define __SOUND_SOC_CODECS_tdmc_H__

#include <linux/device.h>
#include <linux/mutex.h>

struct tdmc_master;
struct tdmc_slave;

/**
 * struct tdmc_ops - TDM-C master implementation specific callbacks
 * @write: Is called when a slave attached to the master writes a register.
 * @read: Is called when the a slave attached to the master reads a register.
 * @attach_slave: Is called when a new slave driver is registered for the
 *  master, If the master can not support the slave this should return a
 *  negative error value.
 * @detach_slave: Is called when a slave driver is unregistered from the master.
 */
struct tdmc_ops {
	int (*write)(struct tdmc_master *master, struct tdmc_slave *slave, 
		unsigned int reg, unsigned int val);
	int (*read)(struct tdmc_master *master, struct tdmc_slave *slave, 
		unsigned int reg, unsigned int *val);
	int (*attach_slave)(struct tdmc_master *master,
		struct tdmc_slave *slave);
	void (*detach_slave)(struct tdmc_master *master,
		struct tdmc_slave *slave);
};

/**
 * struct tdmc_master - TDM-C master instance
 * @dev: Device driver model for the master.
 * @id: Unique id of the master.
 * @ops: Master specific callbacks
 * @lock: Lock protecting concurrent access to the master
 */
struct tdmc_master {
	struct device dev;
	unsigned int id;
	const struct tdmc_ops *ops;
	struct mutex lock;
};

/**
 * struct tdmc_slave - TDM-C slave instance
 * @dev: Device driver model device for the slave
 * @master: The tdmc master the slave is attached to
 * @port: The port number of the master the slave is attached to
 */
struct tdmc_slave {
	struct device dev;
	struct tdmc_master *master;
	unsigned int port;
};

/**
 * tdmc_slave_driver - TDM-C slave driver
 * @driver: Device driver model driver for the slave driver
 * @probe: Is called when a device is bound to the driver
 * @remove:  Is called when a device is unbound from the driver
 */
struct tdmc_slave_driver {
	struct device_driver driver;
	int (*probe)(struct tdmc_slave *dev);
	void (*remove)(struct tdmc_slave *dev);
};

/**
 * tdmc_master_get_drvdata() - Get driver data of a TDM-C master
 * @master: The TDM-C master for which to get the driver data.
 *
 * This function can be used to get the drvdata that was passed to
 * devm_tdmc_master_register(). It is typically used inside the tdmc_ops
 * callbacks.
 */
static inline void *tdmc_master_get_drvdata(struct tdmc_master *master)
{
	return dev_get_drvdata(&master->dev);
}

int devm_tdmc_master_register(struct device *paranet,
	const struct tdmc_ops *ops, void *drvdata);

int tdmc_slave_driver_register(struct module *, struct tdmc_slave_driver *);
void tdmc_slave_driver_unregister(struct tdmc_slave_driver *);

#define __tdmc_slave_driver_register(driver) \
	tdmc_slave_driver_register(THIS_MODULE, driver)

/**
 * module_tdmc_slave_driver() - Helper macro for registering a TDM-C slave driver
 * @__driver: A TDM-C slave driver struct
 *
 * This macro creates the boilerplate functions for registering and
 * unregistering a TDM-C slave driver.  It should be used instead of manually
 * calling tdmc_slave_driver_register()/tdmc_slave_driver_unregister().
 */
#define module_tdmc_slave_driver(__driver) \
	module_driver(__driver, __tdmc_slave_driver_register, \
		tdmc_slave_driver_unregister)

struct regmap_config;
struct regmap;

struct regmap *devm_tdmc_regmap_init(struct tdmc_slave *slave,
	const struct regmap_config *config);

#endif
