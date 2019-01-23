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
 * struct jesd204_dev_list_entry - Entry for a JESD204 device in a list
 * @entry		list entry for a device to keep a list of devices
 * @jdev		pointer to JESD204 device for this list entry
 */
struct jesd204_dev_list_entry {
	struct list_head		entry;
	struct jesd204_dev		*jdev;
};

/**
 * struct jesd204_dev_con_out - Output connection of a JESD204 device
 * @entry		list entry for a device to keep a list of connections
 * @owner		pointer to JESD204 device to which this connection
 *			belongs to
 * @jdev_top		pointer to JESD204 top device, to which this connection
 *			belongs to
 * @dests		list of JESD204 devices this connection is connected
 *			as input
 * @dests_count		number of connected JESD204 devices to this output
 * @of			device-tree reference and arguments for this connection
 */
struct jesd204_dev_con_out {
	struct list_head		entry;
	struct jesd204_dev		*owner;
	struct jesd204_dev_top		*jdev_top;
	struct list_head		dests;
	unsigned int			dests_count;
	struct of_phandle_args		of;
};

/**
 * struct jesd204_dev - JESD204 device
 * @entry		list entry for the framework to keep a list of devices
 * @is_top		true if this device is a top device in a topology of
 *			devices that make up a JESD204 link (typically the
 *			device that is the ADC, DAC, or transceiver)
 * @dev			device that registers itself as a JESD204 device
 * @np			reference in the device-tree for this JESD204 device
 * @ref			ref count for this JESD204 device
 * @inputs		array of pointers to output connections from other
 *			devices
 * @inputs_count	number of @inputs in the array
 * @outputs		list of output connections that take input from this
 *			device
 * @outputs_count	number of @outputs in the list
 */
struct jesd204_dev {
	struct list_head		entry;

	bool				is_top;

	struct device			*dev;
	struct device_node		*np;
	struct kref			ref;

	struct jesd204_dev_con_out	**inputs;
	unsigned int			inputs_count;
	struct list_head		outputs;
	unsigned int			outputs_count;
};

/**
 * struct jesd204_dev_top - JESD204 top device (in a JESD204 topology)
 * @entry		list entry for the framework to keep a list of top
 *			devices (and implicitly topologies)
 * @jdev		JESD204 device data
 */
struct jesd204_dev_top {
	struct list_head		entry;

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
