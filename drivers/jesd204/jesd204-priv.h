/**
 * The JESD204 subsystem
 *
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _JESD204_PRIV_H_
#define _JESD204_PRIV_H_

#include <linux/module.h>
#include <linux/jesd204/jesd204.h>

/**
 * struct jesd204_dev_priv - JESD204 device private framework data
 * @id:			used to identify device internally
 * @driver_module:		used to make it harder to undercut users
 */
struct jesd204_dev_priv {
	struct jesd204_dev		jesd204_dev;

	/**
	 * Don't let these definitions go outside of the framework
	 * unless there's a good reason for them
	 */

	int				id;
	struct module			*driver_module;
};

static inline struct jesd204_dev_priv *jesd204_dev_to_priv(
		struct jesd204_dev *jdev)
{
	return container_of(jdev, struct jesd204_dev_priv, jesd204_dev);
}

#endif /* _JESD204_PRIV_H_ */
