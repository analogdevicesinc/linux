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
struct jesd204_dev_top;

/**
 * struct jesd204_dev - JESD204 device
 * @list		list entry for the framework to keep a list of devices
 * @is_top		true if this device is a top device in a topology of
 *			devices that make up a JESD204 link (typically the
 *			device that is the ADC, DAC, or transceiver)
 * @dev			device that registers itself as a JESD204 device
 * @np			reference in the device-tree for this JESD204 device
 * @ref			ref count for this JESD204 device
 */
struct jesd204_dev {
	struct list_head		list;

	bool				is_top;

	struct device			*dev;
	struct device_node		*np;
	struct kref			ref;
};

/**
 * struct jesd204_dev_top - JESD204 top device (in a JESD204 topology)
 * @list		list entry for the framework to keep a list of top
 *			devices (and implicitly topologies)
 * @jdev		JESD204 device data
 */
struct jesd204_dev_top {
	struct list_head		list;

	struct jesd204_dev		jdev;
};

static inline struct jesd204_dev_top *jesd204_dev_top_dev(
		struct jesd204_dev *jdev)
{
	if (!jdev || !jdev->is_top)
		return NULL;
	return container_of(jdev, struct jesd204_dev_top, jdev);
}

#endif /* _JESD204_PRIV_H_ */
