/**
 * The JESD204 subsystem
 *
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Based on elements of the IIO & CoreSight subsystems.
 */
#ifndef _JESD204_H_
#define _JESD204_H_

#include <linux/device.h>

#define JESD204_ALIGN	L1_CACHE_BYTES

/**
 * struct jesd204_dev - JESD204 device
 * @dev:		device structure, should be assigned a parent and owner
 */
struct jesd204_dev {
	struct device			dev;
};

struct jesd204_dev *jesd204_dev_alloc(int sizeof_priv);
void jesd204_dev_free(struct jesd204_dev *jdev);

struct jesd204_dev *devm_jesd204_dev_alloc(struct device *dev, int sizeof_priv);
void devm_jesd204_dev_free(struct device *dev, struct jesd204_dev *jdev);

/**
 * jesd204_dev_register() - register a device with the JESD204 subsystem
 * @jdev:		Device structure filled by the device driver
 **/
#define jesd204_dev_register(jdev) \
	__jesd204_dev_register((jdev), THIS_MODULE)
int __jesd204_dev_register(struct jesd204_dev *jdev, struct module *this_mod);
void jesd204_dev_unregister(struct jesd204_dev *jdev);

/**
 * devm_jesd204_dev_register - Resource-managed jesd204_dev_register()
 * @dev:	Device to allocate jesd204_dev for
 * @jdev:	Device structure filled by the device driver
 *
 * Managed jesd204_dev_register.  The JESD204 device registered with this
 * function is automatically unregistered on driver detach. This function
 * calls jesd204_dev_register() internally. Refer to that function for more
 * information.
 *
 * If an jesd204_dev registered with this function needs to be unregistered
 * separately, devm_jesd204_dev_unregister() must be used.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
#define devm_jesd204_dev_register(dev, jdev) \
	__devm_jesd204_dev_register((dev), (jdev), THIS_MODULE)
int __devm_jesd204_dev_register(struct device *dev, struct jesd204_dev *jdev,
				struct module *this_mod);
void devm_jesd204_dev_unregister(struct device *dev, struct jesd204_dev *jdev);

/**
 * dev_to_jesd204_dev() - Get jesd204_dev struct from a device struct
 * @dev:		The device embedded in jesd204_dev
 *
 * Note: The device must be a jesd204_dev, otherwise the result is undefined.
 */
static inline struct jesd204_dev *dev_to_jesd204_dev(struct device *dev)
{
	return container_of(dev, struct jesd204_dev, dev);
}

#endif
