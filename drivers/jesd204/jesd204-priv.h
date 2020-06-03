/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * The JESD204 framework
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#ifndef _JESD204_PRIV_H_
#define _JESD204_PRIV_H_

#include <linux/jesd204/jesd204.h>

#define JESD204_MAX_LINKS		16

struct jesd204_dev;
struct jesd204_dev_top;
struct jesd204_link_opaque;
struct jesd204_dev_con_out;
struct jesd204_fsm_data;

enum jesd204_dev_state {
	JESD204_STATE_ERROR = -1,
	JESD204_STATE_UNINIT = 0,
	JESD204_STATE_INITIALIZED,
	JESD204_STATE_PROBED,
	JESD204_STATE_LINK_INIT,
	JESD204_STATE_LINK_SUPPORTED,
	JESD204_STATE_LINK_SETUP,
	JESD204_STATE_CLOCKS_ENABLE,
	JESD204_STATE_LINK_ENABLE,
	JESD204_STATE_LINK_RUNNING,
	JESD204_STATE_LINK_DISABLE,
	JESD204_STATE_CLOCKS_DISABLE,
	JESD204_STATE_LINK_UNINIT,
	JESD204_STATE_DONT_CARE = 999,
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
 * @topo_id		topology ID, that this connection belongs to
 *			(must match JESD204 top device)
 * @link_id		JESD204 link ID, that this connection belongs to
 *			(obtained from DT, must match a link ID in the top device)
 * @link_idx		JESD204 link index in the list of link IDs of the top device
 *			(computed at initialize)
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
	unsigned int			topo_id;
	unsigned int			link_id;
	unsigned int			link_idx;
	struct list_head		dests;
	unsigned int			dests_count;
	struct of_phandle_args		of;
	int				error;
	enum jesd204_dev_state		state;
};

/**
 * struct jesd204_dev - JESD204 device
 * @dev			underlying device object
 * @id			unique device id
 * @entry		list entry for the framework to keep a list of devices
 * @priv		private data to be returned to the driver
 * @fsm_started		true if the FSM has been started for this device
 * @is_top		true if this device is a top device in a topology of
 *			devices that make up a JESD204 link (typically the
 *			device that is the ADC, DAC, or transceiver)
 * @error		error code for this device if something happened
 * @parent		parent device that registers itself as a JESD204 device
 * @state_ops		ops for each state transition of type @struct jesd204_state_ops
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
	struct device			dev;
	int				id;
	struct list_head		entry;
	void				*priv;

	bool				fsm_started;
	bool				is_top;

	int				error;
	struct device			*parent;
	const struct jesd204_state_ops	*state_ops;
	struct device_node		*np;
	struct kref			ref;

	struct jesd204_dev_con_out	**inputs;
	unsigned int			inputs_count;
	struct list_head		outputs;
	unsigned int			outputs_count;
};

/**
 * struct jesd204_link_opaque - JESD204 link information (opaque part)
 * @link		public link information
 * @jdev_top		JESD204 top level this links belongs to
 * @link_idx		Index in the array of JESD204 links in @jdev_top
 * @cb_ref		kref which for each JESD204 link will increment when it
 *			needs to defer a state transition; an equivalent
 *			notification must be called by the device to decrement
 *			this and finally call the @fsm_complete_cbs
 *			callback (if provided)
 * @cur_state		current state of the JESD204 link
 * @fsm_data		reference to state-transition information
 * @flags		internal flags set by the framework
 * @error		error codes for the JESD204 link
 */
struct jesd204_link_opaque {
	struct jesd204_link		link;
	struct jesd204_dev_top		*jdev_top;
	unsigned int			link_idx;

	struct kref			cb_ref;
	enum jesd204_dev_state		state;
	struct jesd204_fsm_data		*fsm_data;
	unsigned long			flags;
	int				error;
};

/**
 * struct jesd204_dev_top - JESD204 top device (in a JESD204 topology)
 * @jdev		JESD204 device data
 * @entry		list entry for the framework to keep a list of top
 *			devices (and implicitly topologies)
 * @initialized		true the topoology connections have been initialized
 * @fsm_data		ref to JESD204 FSM data for JESD204_LNK_FSM_PARALLEL
 * @cb_ref		kref which for each JESD204 link will increment when it
 *			needs to defer a state transition; this is similar to the
 *			cb_ref on the jesd204_link_opaque struct, but each link
 *			increments/decrements it, to group transitions of multiple
 *			JESD204 links
 * @topo_id		topology ID for this device (and top-level device)
 *			(connections should match against this)
 * @link_ids		JESD204 link IDs for this top-level device
 *			(connections should match against this)
 * @init_links		initial settings passed from the driver
 * @active_links	active JESD204 link settings
 * @staged_links	JESD204 link settings staged to be committed as active
 * @num_links		number of links
 */
struct jesd204_dev_top {
	struct jesd204_dev		jdev;
	struct list_head		entry;
	bool				initialized;

	struct jesd204_fsm_data		*fsm_data;
	struct kref			cb_ref;

	int				topo_id;
	unsigned int			link_ids[JESD204_MAX_LINKS];
	unsigned int			num_links;

	const struct jesd204_link	*init_links;
	struct jesd204_link_opaque	*active_links;
	struct jesd204_link_opaque	*staged_links;
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

int jesd204_dev_init_link_data(struct jesd204_dev_top *jdev_top,
			       unsigned int link_idx);

int jesd204_init_topology(struct jesd204_dev_top *jdev_top);

int jesd204_fsm_probe(struct jesd204_dev *jdev);

#endif /* _JESD204_PRIV_H_ */
