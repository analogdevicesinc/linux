/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#ifndef _JESD204_PRIV_H_
#define _JESD204_PRIV_H_

#include <linux/jesd204/jesd204.h>

struct jesd204_dev;

/**
 * struct jesd204_dev - JESD204 device
 * @list		list entry for the framework to keep a list of devices
 * @np			reference in the device-tree for this JESD204 device
 * @ref			ref count for this JESD204 device
 */
struct jesd204_dev {
	struct list_head		list;

	struct device_node		*np;
	struct kref			ref;
};

#endif /* _JESD204_PRIV_H_ */
