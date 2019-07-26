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

typedef int (*jesd204_cb_priv)(struct jesd204_dev *jdev, void *data);

enum jesd204_dev_state {
	JESD204_STATE_ERROR = -1,
	JESD204_STATE_UNINIT = 0,
	JESD204_STATE_INITIALIZED,
	JESD204_STATE_PROBED,
	JESD204_STATE_LINK_INIT,
};

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
 * @error		error code for this connection
 * @state		current state of this connection
 */
struct jesd204_dev_con_out {
	struct list_head		entry;
	struct jesd204_dev		*owner;
	struct jesd204_dev_top		*jdev_top;
	struct list_head		dests;
	unsigned int			dests_count;
	struct of_phandle_args		of;
	int				error;
	enum jesd204_dev_state		state;
};

/**
 * struct jesd204_dev - JESD204 device
 * @entry		list entry for the framework to keep a list of devices
 * @is_top		true if this device is a top device in a topology of
 *			devices that make up a JESD204 link (typically the
 *			device that is the ADC, DAC, or transceiver)
 * @dev			device that registers itself as a JESD204 device
 * @link_ops		JESD204 operations for JESD204 link management
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
	const jesd204_link_cb		*link_ops;
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
 * @fsm_complete_cb	callback that gets called after a topology has finished
 *			it's state transition, meaning that all JESD204 devices
 *			have moved to the desired @nxt_state
 * @cb_ref		kref which all JESD204 devices will increment when they
 *			need to defer their state transition; an equivalent
 *			notification must be called by the device to decrement
 *			this and finally call the @fsm_complete_cb
 *			callback (if provided)
 * @cb_data		pointer to private data used during a state transition
 * @nxt_state		next state this topology has to transition to
 * @cur_state		current state of this topology
 * @error		error code for this topology after a state has failed
 *			to transition
 * @init_links		initial settings passed from the driver
 * @active_links	active JESD204 link settings
 * @num_links		number of links
 */
struct jesd204_dev_top {
	struct list_head		entry;

	struct jesd204_dev		jdev;

	jesd204_cb_priv			fsm_complete_cb;
	struct kref			cb_ref;
	void				*cb_data;
	enum jesd204_dev_state		nxt_state;
	enum jesd204_dev_state		cur_state;
	int				error;

	const struct jesd204_link	*init_links;
	struct jesd204_link		*active_links;
	unsigned int			num_links;
};

struct list_head *jesd204_topologies_get(void);

static inline struct jesd204_dev_top *jesd204_dev_top_dev(
		struct jesd204_dev *jdev)
{
	if (!jdev || !jdev->is_top)
		return NULL;
	return container_of(jdev, struct jesd204_dev_top, jdev);
}

const char *jesd204_state_str(enum jesd204_dev_state state);

int jesd204_dev_init_link_data(struct jesd204_dev *jdev);

int jesd204_init_topology(struct jesd204_dev_top *jdev_top);

int jesd204_fsm_probe(struct jesd204_dev *jdev);

int jesd204_fsm_init_links(struct jesd204_dev *jdev,
			   enum jesd204_dev_state init_state);
#endif /* _JESD204_PRIV_H_ */
