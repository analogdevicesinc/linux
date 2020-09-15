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

#define JESD204_STATE_FSM_OFFSET	100

#define JESD204_STATE_ENUM(x)	\
	JESD204_STATE_##x = (JESD204_FSM_STATE_##x + JESD204_STATE_FSM_OFFSET)

enum jesd204_dev_state {
	JESD204_STATE_UNINIT = 0,
	JESD204_STATE_INITIALIZED,
	JESD204_STATE_PROBED,
	JESD204_STATE_IDLE,
	JESD204_STATE_ENUM(DEVICE_INIT),
	JESD204_STATE_ENUM(LINK_INIT),
	JESD204_STATE_ENUM(LINK_SUPPORTED),
	JESD204_STATE_ENUM(LINK_PRE_SETUP),
	JESD204_STATE_ENUM(CLK_SYNC_STAGE1),
	JESD204_STATE_ENUM(CLK_SYNC_STAGE2),
	JESD204_STATE_ENUM(CLK_SYNC_STAGE3),
	JESD204_STATE_ENUM(LINK_SETUP),
	JESD204_STATE_ENUM(OPT_SETUP_STAGE1),
	JESD204_STATE_ENUM(OPT_SETUP_STAGE2),
	JESD204_STATE_ENUM(OPT_SETUP_STAGE3),
	JESD204_STATE_ENUM(OPT_SETUP_STAGE4),
	JESD204_STATE_ENUM(OPT_SETUP_STAGE5),
	JESD204_STATE_ENUM(CLOCKS_ENABLE),
	JESD204_STATE_ENUM(LINK_ENABLE),
	JESD204_STATE_ENUM(LINK_RUNNING),
	JESD204_STATE_ENUM(OPT_POST_RUNNING_STAGE),
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
 * @id			unique ID for this connection
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
	unsigned int			id;
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
 * @dev_data		ref to data provided by the driver registering with the framework
 * @id			unique device id
 * @entry		list entry for the framework to keep a list of devices
 * @priv		private data to be returned to the driver
 * @fsm_inited		true if the FSM has been initialized for this device
 * @is_top		true if this device is a top device in a topology of
 *			devices that make up a JESD204 link (typically the
 *			device that is the ADC, DAC, or transceiver)
 * @is_sysref_provider	true if this device wants to be a SYSREF provider
 * @is_sec_sysref_provider true if this device wants to be the secondary SYSREF provider
 * @stop_states		array of states on which to stop, after finishing transition
 * @sysfs_attr_group	attribute group for the sysfs files of this JESD204 device
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
	const struct jesd204_dev_data	*dev_data;
	int				id;
	struct list_head		entry;
	void				*priv;

	bool				fsm_inited;
	bool				is_top;

	bool				is_sysref_provider;
	bool				is_sec_sysref_provider;
	bool				stop_states[JESD204_FSM_STATES_NUM + 1];

	struct device_node		*np;

	struct attribute_group		sysfs_attr_group;

	struct jesd204_dev_con_out	**inputs;
	unsigned int			inputs_count;
	struct list_head		outputs;
	unsigned int			outputs_count;
};

#define dev_to_jesd204_dev(dev)		\
	container_of(dev, struct jesd204_dev, dev)

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
 * @state		current state of the JESD204 link
 * @fsm_paused		true if the FSM has paused during a transition (set)
 * @fsm_data		reference to state-transition information
 * @flags		internal flags set by the framework
 */
struct jesd204_link_opaque {
	struct jesd204_link		link;
	struct jesd204_dev_top		*jdev_top;
	unsigned int			link_idx;

	struct kref			cb_ref;
	enum jesd204_dev_state		state;
	bool				fsm_paused;
	struct jesd204_fsm_data		*fsm_data;
	unsigned long			flags;
};

/**
 * struct jesd204_dev_top - JESD204 top device (in a JESD204 topology)
 * @jdev		JESD204 device data
 * @entry		list entry for the framework to keep a list of top
 *			devices (and implicitly topologies)
 * @initialized		true the topoology connections have been initialized
 * @num_retries		number of retries if an error occurs
 * @jdev_sysref		reference to the object that is the SYSREF provider for this topology
 * @jdev_sysref_sec	reference to the object that is the secondary SYSREF provider
 * 			for this topology, in case the primary jdev_sysref doesn't exist
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
 * @num_links		number of links
 * @init_links		initial settings passed from the driver
 * @active_links	active JESD204 link settings
 * @staged_links	JESD204 link settings staged to be committed as active
 */
struct jesd204_dev_top {
	struct jesd204_dev		jdev;
	struct list_head		entry;
	bool				initialized;
	unsigned int			num_retries;

	struct jesd204_dev		*jdev_sysref;
	struct jesd204_dev		*jdev_sysref_sec;

	struct jesd204_fsm_data		*fsm_data;
	struct kref			cb_ref;

	int				topo_id;
	unsigned int			link_ids[JESD204_MAX_LINKS];
	unsigned int			num_links;

	const struct jesd204_link	*init_links;
	struct jesd204_link_opaque	*active_links;
	struct jesd204_link_opaque	*staged_links;
};

int jesd204_device_count_get(void);

struct jesd204_dev_top *jesd204_dev_get_topology_top_dev(struct jesd204_dev *jdev);

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

int jesd204_dev_create_sysfs(struct jesd204_dev *jdev);
void jesd204_dev_destroy_sysfs(struct jesd204_dev *jdev);

#endif /* _JESD204_PRIV_H_ */
