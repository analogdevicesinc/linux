// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * net/switchdev/switchdev.c - Switch device API
 * Copyright (c) 2014-2015 Jiri Pirko <jiri@resnulli.us>
 * Copyright (c) 2014-2015 Scott Feldman <sfeldma@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/list.h>
#include <linux/workqueue.h>
#include <linux/if_vlan.h>
#include <linux/rtnetlink.h>
#include <net/switchdev.h>

static bool switchdev_obj_eq(const struct switchdev_obj *a,
			     const struct switchdev_obj *b)
{
	const struct switchdev_obj_port_vlan *va, *vb;
	const struct switchdev_obj_port_mdb *ma, *mb;

	if (a->id != b->id || a->orig_dev != b->orig_dev)
		return false;

	switch (a->id) {
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		va = SWITCHDEV_OBJ_PORT_VLAN(a);
		vb = SWITCHDEV_OBJ_PORT_VLAN(b);
		return va->flags == vb->flags &&
			va->vid == vb->vid &&
			va->changed == vb->changed;
	case SWITCHDEV_OBJ_ID_PORT_MDB:
	case SWITCHDEV_OBJ_ID_HOST_MDB:
		ma = SWITCHDEV_OBJ_PORT_MDB(a);
		mb = SWITCHDEV_OBJ_PORT_MDB(b);
		return ma->vid == mb->vid &&
			ether_addr_equal(ma->addr, mb->addr);
	default:
		break;
	}

	BUG();
}

static LIST_HEAD(deferred);
static DEFINE_SPINLOCK(deferred_lock);

typedef void switchdev_deferred_func_t(struct net_device *dev,
				       const void *data);

struct switchdev_deferred_item {
	struct list_head list;
	struct net_device *dev;
	netdevice_tracker dev_tracker;
	switchdev_deferred_func_t *func;
	unsigned long data[];
};

static struct switchdev_deferred_item *switchdev_deferred_dequeue(void)
{
	struct switchdev_deferred_item *dfitem;

	spin_lock_bh(&deferred_lock);
	if (list_empty(&deferred)) {
		dfitem = NULL;
		goto unlock;
	}
	dfitem = list_first_entry(&deferred,
				  struct switchdev_deferred_item, list);
	list_del(&dfitem->list);
unlock:
	spin_unlock_bh(&deferred_lock);
	return dfitem;
}

/**
 *	switchdev_deferred_process - Process ops in deferred queue
 *
 *	Called to flush the ops currently queued in deferred ops queue.
 *	rtnl_lock must be held.
 */
void switchdev_deferred_process(void)
{
	struct switchdev_deferred_item *dfitem;

	ASSERT_RTNL();

	while ((dfitem = switchdev_deferred_dequeue())) {
		dfitem->func(dfitem->dev, dfitem->data);
		netdev_put(dfitem->dev, &dfitem->dev_tracker);
		kfree(dfitem);
	}
}
EXPORT_SYMBOL_GPL(switchdev_deferred_process);

static void switchdev_deferred_process_work(struct work_struct *work)
{
	rtnl_lock();
	switchdev_deferred_process();
	rtnl_unlock();
}

static DECLARE_WORK(deferred_process_work, switchdev_deferred_process_work);

static int switchdev_deferred_enqueue(struct net_device *dev,
				      const void *data, size_t data_len,
				      switchdev_deferred_func_t *func)
{
	struct switchdev_deferred_item *dfitem;

	dfitem = kmalloc(struct_size(dfitem, data, data_len), GFP_ATOMIC);
	if (!dfitem)
		return -ENOMEM;
	dfitem->dev = dev;
	dfitem->func = func;
	memcpy(dfitem->data, data, data_len);
	netdev_hold(dev, &dfitem->dev_tracker, GFP_ATOMIC);
	spin_lock_bh(&deferred_lock);
	list_add_tail(&dfitem->list, &deferred);
	spin_unlock_bh(&deferred_lock);
	schedule_work(&deferred_process_work);
	return 0;
}

static int switchdev_port_attr_notify(enum switchdev_notifier_type nt,
				      struct net_device *dev,
				      const struct switchdev_attr *attr,
				      struct netlink_ext_ack *extack)
{
	int err;
	int rc;

	struct switchdev_notifier_port_attr_info attr_info = {
		.attr = attr,
		.handled = false,
	};

	rc = call_switchdev_blocking_notifiers(nt, dev,
					       &attr_info.info, extack);
	err = notifier_to_errno(rc);
	if (err) {
		WARN_ON(!attr_info.handled);
		return err;
	}

	if (!attr_info.handled)
		return -EOPNOTSUPP;

	return 0;
}

static int switchdev_port_attr_set_now(struct net_device *dev,
				       const struct switchdev_attr *attr,
				       struct netlink_ext_ack *extack)
{
	return switchdev_port_attr_notify(SWITCHDEV_PORT_ATTR_SET, dev, attr,
					  extack);
}

static void switchdev_port_attr_set_deferred(struct net_device *dev,
					     const void *data)
{
	const struct switchdev_attr *attr = data;
	int err;

	err = switchdev_port_attr_set_now(dev, attr, NULL);
	if (err && err != -EOPNOTSUPP)
		netdev_err(dev, "failed (err=%d) to set attribute (id=%d)\n",
			   err, attr->id);
	if (attr->complete)
		attr->complete(dev, err, attr->complete_priv);
}

static int switchdev_port_attr_set_defer(struct net_device *dev,
					 const struct switchdev_attr *attr)
{
	return switchdev_deferred_enqueue(dev, attr, sizeof(*attr),
					  switchdev_port_attr_set_deferred);
}

/**
 *	switchdev_port_attr_set - Set port attribute
 *
 *	@dev: port device
 *	@attr: attribute to set
 *	@extack: netlink extended ack, for error message propagation
 *
 *	rtnl_lock must be held and must not be in atomic section,
 *	in case SWITCHDEV_F_DEFER flag is not set.
 */
int switchdev_port_attr_set(struct net_device *dev,
			    const struct switchdev_attr *attr,
			    struct netlink_ext_ack *extack)
{
	if (attr->flags & SWITCHDEV_F_DEFER)
		return switchdev_port_attr_set_defer(dev, attr);
	ASSERT_RTNL();
	return switchdev_port_attr_set_now(dev, attr, extack);
}
EXPORT_SYMBOL_GPL(switchdev_port_attr_set);

static size_t switchdev_obj_size(const struct switchdev_obj *obj)
{
	switch (obj->id) {
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		return sizeof(struct switchdev_obj_port_vlan);
	case SWITCHDEV_OBJ_ID_PORT_MDB:
		return sizeof(struct switchdev_obj_port_mdb);
	case SWITCHDEV_OBJ_ID_HOST_MDB:
		return sizeof(struct switchdev_obj_port_mdb);
	default:
		BUG();
	}
	return 0;
}

static int switchdev_port_obj_notify(enum switchdev_notifier_type nt,
				     struct net_device *dev,
				     const struct switchdev_obj *obj,
				     struct netlink_ext_ack *extack)
{
	int rc;
	int err;

	struct switchdev_notifier_port_obj_info obj_info = {
		.obj = obj,
		.handled = false,
	};

	rc = call_switchdev_blocking_notifiers(nt, dev, &obj_info.info, extack);
	err = notifier_to_errno(rc);
	if (err) {
		WARN_ON(!obj_info.handled);
		return err;
	}
	if (!obj_info.handled)
		return -EOPNOTSUPP;
	return 0;
}

static void switchdev_obj_id_to_helpful_msg(struct net_device *dev,
					    enum switchdev_obj_id obj_id,
					    int err, bool add)
{
	const char *action = add ? "add" : "del";
	const char *reason = "";
	const char *problem;
	const char *obj_str;

	switch (obj_id) {
	case SWITCHDEV_OBJ_ID_UNDEFINED:
		obj_str = "Undefined object";
		problem = "Attempted operation is undefined, indicating a possible programming\n"
			  "error.\n";
		break;
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		obj_str = "VLAN entry";
		problem = "Failure in VLAN settings on this port might disrupt network\n"
			  "segmentation or traffic isolation, affecting network partitioning.\n";
		break;
	case SWITCHDEV_OBJ_ID_PORT_MDB:
		obj_str = "Port Multicast Database entry";
		problem = "Failure in updating the port's Multicast Database could lead to\n"
			  "multicast forwarding issues.\n";
		break;
	case SWITCHDEV_OBJ_ID_HOST_MDB:
		obj_str = "Host Multicast Database entry";
		problem = "Failure in updating the host's Multicast Database may impact multicast\n"
			  "group memberships or traffic delivery, affecting multicast\n"
			  "communication.\n";
		break;
	case SWITCHDEV_OBJ_ID_MRP:
		obj_str = "Media Redundancy Protocol configuration for port";
		problem = "Failure to set MRP ring ID on this port prevents communication with\n"
			  "the specified redundancy ring, resulting in an inability to engage\n"
			  "in MRP-based network operations.\n";
		break;
	case SWITCHDEV_OBJ_ID_RING_TEST_MRP:
		obj_str = "MRP Test Frame Operations for port";
		problem = "Failure to generate/monitor MRP test frames may lead to inability to\n"
			  "assess the ring's operational integrity and fault response, hindering\n"
			  "proactive network management.\n";
		break;
	case SWITCHDEV_OBJ_ID_RING_ROLE_MRP:
		obj_str = "MRP Ring Role Configuration";
		problem = "Improper MRP ring role configuration may create conflicts in the ring,\n"
			  "disrupting communication for all participants, or isolate the local\n"
			  "system from the ring, hindering its ability to communicate with other\n"
			  "participants.\n";
		break;
	case SWITCHDEV_OBJ_ID_RING_STATE_MRP:
		obj_str = "MRP Ring State Configuration";
		problem = "Failure to correctly set the MRP ring state can result in network\n"
			  "loops or leave segments without communication. In a Closed state,\n"
			  "it maintains loop prevention by blocking one MRM port, while an Open\n"
			  "state activates in response to failures, changing port states to\n"
			  "preserve network connectivity.\n";
		break;
	case SWITCHDEV_OBJ_ID_IN_TEST_MRP:
		obj_str = "MRP_InTest Frame Generation Configuration";
		problem = "Failure in managing MRP_InTest frame generation can misjudge the\n"
			  "interconnection ring's state, leading to incorrect blocking or\n"
			  "unblocking of the I/C port. This misconfiguration might result\n"
			  "in unintended network loops or isolate critical network segments,\n"
			  "compromising network integrity and reliability.\n";
		break;
	case SWITCHDEV_OBJ_ID_IN_ROLE_MRP:
		obj_str = "Interconnection Ring Role Configuration";
		problem = "Failure in incorrect assignment of interconnection ring roles\n"
			  "(MIM/MIC) can impair the formation of the interconnection rings.\n";
		break;
	case SWITCHDEV_OBJ_ID_IN_STATE_MRP:
		obj_str = "Interconnection Ring State Configuration";
		problem = "Failure in updating the interconnection ring state can lead in\n"
			  "case of Open state to incorrect blocking or unblocking of the\n"
			  "I/C port, resulting in unintended network loops or isolation\n"
			  "of critical network\n";
		break;
	default:
		obj_str = "Unknown object";
		problem	= "Indicating a possible programming error.\n";
	}

	switch (err) {
	case -ENOSPC:
		reason = "Current HW/SW setup lacks sufficient resources.\n";
		break;
	}

	netdev_err(dev, "Failed to %s %s (object id=%d) with error: %pe (%d).\n%s%s\n",
		   action, obj_str, obj_id, ERR_PTR(err), err, problem, reason);
}

static void switchdev_port_obj_add_deferred(struct net_device *dev,
					    const void *data)
{
	const struct switchdev_obj *obj = data;
	int err;

	ASSERT_RTNL();
	err = switchdev_port_obj_notify(SWITCHDEV_PORT_OBJ_ADD,
					dev, obj, NULL);
	if (err && err != -EOPNOTSUPP)
		switchdev_obj_id_to_helpful_msg(dev, obj->id, err, true);
	if (obj->complete)
		obj->complete(dev, err, obj->complete_priv);
}

static int switchdev_port_obj_add_defer(struct net_device *dev,
					const struct switchdev_obj *obj)
{
	return switchdev_deferred_enqueue(dev, obj, switchdev_obj_size(obj),
					  switchdev_port_obj_add_deferred);
}

/**
 *	switchdev_port_obj_add - Add port object
 *
 *	@dev: port device
 *	@obj: object to add
 *	@extack: netlink extended ack
 *
 *	rtnl_lock must be held and must not be in atomic section,
 *	in case SWITCHDEV_F_DEFER flag is not set.
 */
int switchdev_port_obj_add(struct net_device *dev,
			   const struct switchdev_obj *obj,
			   struct netlink_ext_ack *extack)
{
	if (obj->flags & SWITCHDEV_F_DEFER)
		return switchdev_port_obj_add_defer(dev, obj);
	ASSERT_RTNL();
	return switchdev_port_obj_notify(SWITCHDEV_PORT_OBJ_ADD,
					 dev, obj, extack);
}
EXPORT_SYMBOL_GPL(switchdev_port_obj_add);

static int switchdev_port_obj_del_now(struct net_device *dev,
				      const struct switchdev_obj *obj)
{
	return switchdev_port_obj_notify(SWITCHDEV_PORT_OBJ_DEL,
					 dev, obj, NULL);
}

static void switchdev_port_obj_del_deferred(struct net_device *dev,
					    const void *data)
{
	const struct switchdev_obj *obj = data;
	int err;

	err = switchdev_port_obj_del_now(dev, obj);
	if (err && err != -EOPNOTSUPP)
		switchdev_obj_id_to_helpful_msg(dev, obj->id, err, false);
	if (obj->complete)
		obj->complete(dev, err, obj->complete_priv);
}

static int switchdev_port_obj_del_defer(struct net_device *dev,
					const struct switchdev_obj *obj)
{
	return switchdev_deferred_enqueue(dev, obj, switchdev_obj_size(obj),
					  switchdev_port_obj_del_deferred);
}

/**
 *	switchdev_port_obj_del - Delete port object
 *
 *	@dev: port device
 *	@obj: object to delete
 *
 *	rtnl_lock must be held and must not be in atomic section,
 *	in case SWITCHDEV_F_DEFER flag is not set.
 */
int switchdev_port_obj_del(struct net_device *dev,
			   const struct switchdev_obj *obj)
{
	if (obj->flags & SWITCHDEV_F_DEFER)
		return switchdev_port_obj_del_defer(dev, obj);
	ASSERT_RTNL();
	return switchdev_port_obj_del_now(dev, obj);
}
EXPORT_SYMBOL_GPL(switchdev_port_obj_del);

/**
 *	switchdev_port_obj_act_is_deferred - Is object action pending?
 *
 *	@dev: port device
 *	@nt: type of action; add or delete
 *	@obj: object to test
 *
 *	Returns true if a deferred item is pending, which is
 *	equivalent to the action @nt on an object @obj.
 *
 *	rtnl_lock must be held.
 */
bool switchdev_port_obj_act_is_deferred(struct net_device *dev,
					enum switchdev_notifier_type nt,
					const struct switchdev_obj *obj)
{
	struct switchdev_deferred_item *dfitem;
	bool found = false;

	ASSERT_RTNL();

	spin_lock_bh(&deferred_lock);

	list_for_each_entry(dfitem, &deferred, list) {
		if (dfitem->dev != dev)
			continue;

		if ((dfitem->func == switchdev_port_obj_add_deferred &&
		     nt == SWITCHDEV_PORT_OBJ_ADD) ||
		    (dfitem->func == switchdev_port_obj_del_deferred &&
		     nt == SWITCHDEV_PORT_OBJ_DEL)) {
			if (switchdev_obj_eq((const void *)dfitem->data, obj)) {
				found = true;
				break;
			}
		}
	}

	spin_unlock_bh(&deferred_lock);

	return found;
}
EXPORT_SYMBOL_GPL(switchdev_port_obj_act_is_deferred);

static ATOMIC_NOTIFIER_HEAD(switchdev_notif_chain);
static BLOCKING_NOTIFIER_HEAD(switchdev_blocking_notif_chain);

/**
 *	register_switchdev_notifier - Register notifier
 *	@nb: notifier_block
 *
 *	Register switch device notifier.
 */
int register_switchdev_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&switchdev_notif_chain, nb);
}
EXPORT_SYMBOL_GPL(register_switchdev_notifier);

/**
 *	unregister_switchdev_notifier - Unregister notifier
 *	@nb: notifier_block
 *
 *	Unregister switch device notifier.
 */
int unregister_switchdev_notifier(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&switchdev_notif_chain, nb);
}
EXPORT_SYMBOL_GPL(unregister_switchdev_notifier);

/**
 *	call_switchdev_notifiers - Call notifiers
 *	@val: value passed unmodified to notifier function
 *	@dev: port device
 *	@info: notifier information data
 *	@extack: netlink extended ack
 *	Call all network notifier blocks.
 */
int call_switchdev_notifiers(unsigned long val, struct net_device *dev,
			     struct switchdev_notifier_info *info,
			     struct netlink_ext_ack *extack)
{
	info->dev = dev;
	info->extack = extack;
	return atomic_notifier_call_chain(&switchdev_notif_chain, val, info);
}
EXPORT_SYMBOL_GPL(call_switchdev_notifiers);

int register_switchdev_blocking_notifier(struct notifier_block *nb)
{
	struct blocking_notifier_head *chain = &switchdev_blocking_notif_chain;

	return blocking_notifier_chain_register(chain, nb);
}
EXPORT_SYMBOL_GPL(register_switchdev_blocking_notifier);

int unregister_switchdev_blocking_notifier(struct notifier_block *nb)
{
	struct blocking_notifier_head *chain = &switchdev_blocking_notif_chain;

	return blocking_notifier_chain_unregister(chain, nb);
}
EXPORT_SYMBOL_GPL(unregister_switchdev_blocking_notifier);

int call_switchdev_blocking_notifiers(unsigned long val, struct net_device *dev,
				      struct switchdev_notifier_info *info,
				      struct netlink_ext_ack *extack)
{
	info->dev = dev;
	info->extack = extack;
	return blocking_notifier_call_chain(&switchdev_blocking_notif_chain,
					    val, info);
}
EXPORT_SYMBOL_GPL(call_switchdev_blocking_notifiers);

struct switchdev_nested_priv {
	bool (*check_cb)(const struct net_device *dev);
	bool (*foreign_dev_check_cb)(const struct net_device *dev,
				     const struct net_device *foreign_dev);
	const struct net_device *dev;
	struct net_device *lower_dev;
};

static int switchdev_lower_dev_walk(struct net_device *lower_dev,
				    struct netdev_nested_priv *priv)
{
	struct switchdev_nested_priv *switchdev_priv = priv->data;
	bool (*foreign_dev_check_cb)(const struct net_device *dev,
				     const struct net_device *foreign_dev);
	bool (*check_cb)(const struct net_device *dev);
	const struct net_device *dev;

	check_cb = switchdev_priv->check_cb;
	foreign_dev_check_cb = switchdev_priv->foreign_dev_check_cb;
	dev = switchdev_priv->dev;

	if (check_cb(lower_dev) && !foreign_dev_check_cb(lower_dev, dev)) {
		switchdev_priv->lower_dev = lower_dev;
		return 1;
	}

	return 0;
}

static struct net_device *
switchdev_lower_dev_find_rcu(struct net_device *dev,
			     bool (*check_cb)(const struct net_device *dev),
			     bool (*foreign_dev_check_cb)(const struct net_device *dev,
							  const struct net_device *foreign_dev))
{
	struct switchdev_nested_priv switchdev_priv = {
		.check_cb = check_cb,
		.foreign_dev_check_cb = foreign_dev_check_cb,
		.dev = dev,
		.lower_dev = NULL,
	};
	struct netdev_nested_priv priv = {
		.data = &switchdev_priv,
	};

	netdev_walk_all_lower_dev_rcu(dev, switchdev_lower_dev_walk, &priv);

	return switchdev_priv.lower_dev;
}

static struct net_device *
switchdev_lower_dev_find(struct net_device *dev,
			 bool (*check_cb)(const struct net_device *dev),
			 bool (*foreign_dev_check_cb)(const struct net_device *dev,
						      const struct net_device *foreign_dev))
{
	struct switchdev_nested_priv switchdev_priv = {
		.check_cb = check_cb,
		.foreign_dev_check_cb = foreign_dev_check_cb,
		.dev = dev,
		.lower_dev = NULL,
	};
	struct netdev_nested_priv priv = {
		.data = &switchdev_priv,
	};

	netdev_walk_all_lower_dev(dev, switchdev_lower_dev_walk, &priv);

	return switchdev_priv.lower_dev;
}

static int __switchdev_handle_fdb_event_to_device(struct net_device *dev,
		struct net_device *orig_dev, unsigned long event,
		const struct switchdev_notifier_fdb_info *fdb_info,
		bool (*check_cb)(const struct net_device *dev),
		bool (*foreign_dev_check_cb)(const struct net_device *dev,
					     const struct net_device *foreign_dev),
		int (*mod_cb)(struct net_device *dev, struct net_device *orig_dev,
			      unsigned long event, const void *ctx,
			      const struct switchdev_notifier_fdb_info *fdb_info))
{
	const struct switchdev_notifier_info *info = &fdb_info->info;
	struct net_device *br, *lower_dev, *switchdev;
	struct list_head *iter;
	int err = -EOPNOTSUPP;

	if (check_cb(dev))
		return mod_cb(dev, orig_dev, event, info->ctx, fdb_info);

	/* Recurse through lower interfaces in case the FDB entry is pointing
	 * towards a bridge or a LAG device.
	 */
	netdev_for_each_lower_dev(dev, lower_dev, iter) {
		/* Do not propagate FDB entries across bridges */
		if (netif_is_bridge_master(lower_dev))
			continue;

		/* Bridge ports might be either us, or LAG interfaces
		 * that we offload.
		 */
		if (!check_cb(lower_dev) &&
		    !switchdev_lower_dev_find_rcu(lower_dev, check_cb,
						  foreign_dev_check_cb))
			continue;

		err = __switchdev_handle_fdb_event_to_device(lower_dev, orig_dev,
							     event, fdb_info, check_cb,
							     foreign_dev_check_cb,
							     mod_cb);
		if (err && err != -EOPNOTSUPP)
			return err;
	}

	/* Event is neither on a bridge nor a LAG. Check whether it is on an
	 * interface that is in a bridge with us.
	 */
	br = netdev_master_upper_dev_get_rcu(dev);
	if (!br || !netif_is_bridge_master(br))
		return 0;

	switchdev = switchdev_lower_dev_find_rcu(br, check_cb, foreign_dev_check_cb);
	if (!switchdev)
		return 0;

	if (!foreign_dev_check_cb(switchdev, dev))
		return err;

	return __switchdev_handle_fdb_event_to_device(br, orig_dev, event, fdb_info,
						      check_cb, foreign_dev_check_cb,
						      mod_cb);
}

int switchdev_handle_fdb_event_to_device(struct net_device *dev, unsigned long event,
		const struct switchdev_notifier_fdb_info *fdb_info,
		bool (*check_cb)(const struct net_device *dev),
		bool (*foreign_dev_check_cb)(const struct net_device *dev,
					     const struct net_device *foreign_dev),
		int (*mod_cb)(struct net_device *dev, struct net_device *orig_dev,
			      unsigned long event, const void *ctx,
			      const struct switchdev_notifier_fdb_info *fdb_info))
{
	int err;

	err = __switchdev_handle_fdb_event_to_device(dev, dev, event, fdb_info,
						     check_cb, foreign_dev_check_cb,
						     mod_cb);
	if (err == -EOPNOTSUPP)
		err = 0;

	return err;
}
EXPORT_SYMBOL_GPL(switchdev_handle_fdb_event_to_device);

static int __switchdev_handle_port_obj_add(struct net_device *dev,
			struct switchdev_notifier_port_obj_info *port_obj_info,
			bool (*check_cb)(const struct net_device *dev),
			bool (*foreign_dev_check_cb)(const struct net_device *dev,
						     const struct net_device *foreign_dev),
			int (*add_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_obj *obj,
				      struct netlink_ext_ack *extack))
{
	struct switchdev_notifier_info *info = &port_obj_info->info;
	struct net_device *br, *lower_dev, *switchdev;
	struct netlink_ext_ack *extack;
	struct list_head *iter;
	int err = -EOPNOTSUPP;

	extack = switchdev_notifier_info_to_extack(info);

	if (check_cb(dev)) {
		err = add_cb(dev, info->ctx, port_obj_info->obj, extack);
		if (err != -EOPNOTSUPP)
			port_obj_info->handled = true;
		return err;
	}

	/* Switch ports might be stacked under e.g. a LAG. Ignore the
	 * unsupported devices, another driver might be able to handle them. But
	 * propagate to the callers any hard errors.
	 *
	 * If the driver does its own bookkeeping of stacked ports, it's not
	 * necessary to go through this helper.
	 */
	netdev_for_each_lower_dev(dev, lower_dev, iter) {
		if (netif_is_bridge_master(lower_dev))
			continue;

		/* When searching for switchdev interfaces that are neighbors
		 * of foreign ones, and @dev is a bridge, do not recurse on the
		 * foreign interface again, it was already visited.
		 */
		if (foreign_dev_check_cb && !check_cb(lower_dev) &&
		    !switchdev_lower_dev_find(lower_dev, check_cb, foreign_dev_check_cb))
			continue;

		err = __switchdev_handle_port_obj_add(lower_dev, port_obj_info,
						      check_cb, foreign_dev_check_cb,
						      add_cb);
		if (err && err != -EOPNOTSUPP)
			return err;
	}

	/* Event is neither on a bridge nor a LAG. Check whether it is on an
	 * interface that is in a bridge with us.
	 */
	if (!foreign_dev_check_cb)
		return err;

	br = netdev_master_upper_dev_get(dev);
	if (!br || !netif_is_bridge_master(br))
		return err;

	switchdev = switchdev_lower_dev_find(br, check_cb, foreign_dev_check_cb);
	if (!switchdev)
		return err;

	if (!foreign_dev_check_cb(switchdev, dev))
		return err;

	return __switchdev_handle_port_obj_add(br, port_obj_info, check_cb,
					       foreign_dev_check_cb, add_cb);
}

/* Pass through a port object addition, if @dev passes @check_cb, or replicate
 * it towards all lower interfaces of @dev that pass @check_cb, if @dev is a
 * bridge or a LAG.
 */
int switchdev_handle_port_obj_add(struct net_device *dev,
			struct switchdev_notifier_port_obj_info *port_obj_info,
			bool (*check_cb)(const struct net_device *dev),
			int (*add_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_obj *obj,
				      struct netlink_ext_ack *extack))
{
	int err;

	err = __switchdev_handle_port_obj_add(dev, port_obj_info, check_cb,
					      NULL, add_cb);
	if (err == -EOPNOTSUPP)
		err = 0;
	return err;
}
EXPORT_SYMBOL_GPL(switchdev_handle_port_obj_add);

/* Same as switchdev_handle_port_obj_add(), except if object is notified on a
 * @dev that passes @foreign_dev_check_cb, it is replicated towards all devices
 * that pass @check_cb and are in the same bridge as @dev.
 */
int switchdev_handle_port_obj_add_foreign(struct net_device *dev,
			struct switchdev_notifier_port_obj_info *port_obj_info,
			bool (*check_cb)(const struct net_device *dev),
			bool (*foreign_dev_check_cb)(const struct net_device *dev,
						     const struct net_device *foreign_dev),
			int (*add_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_obj *obj,
				      struct netlink_ext_ack *extack))
{
	int err;

	err = __switchdev_handle_port_obj_add(dev, port_obj_info, check_cb,
					      foreign_dev_check_cb, add_cb);
	if (err == -EOPNOTSUPP)
		err = 0;
	return err;
}
EXPORT_SYMBOL_GPL(switchdev_handle_port_obj_add_foreign);

static int __switchdev_handle_port_obj_del(struct net_device *dev,
			struct switchdev_notifier_port_obj_info *port_obj_info,
			bool (*check_cb)(const struct net_device *dev),
			bool (*foreign_dev_check_cb)(const struct net_device *dev,
						     const struct net_device *foreign_dev),
			int (*del_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_obj *obj))
{
	struct switchdev_notifier_info *info = &port_obj_info->info;
	struct net_device *br, *lower_dev, *switchdev;
	struct list_head *iter;
	int err = -EOPNOTSUPP;

	if (check_cb(dev)) {
		err = del_cb(dev, info->ctx, port_obj_info->obj);
		if (err != -EOPNOTSUPP)
			port_obj_info->handled = true;
		return err;
	}

	/* Switch ports might be stacked under e.g. a LAG. Ignore the
	 * unsupported devices, another driver might be able to handle them. But
	 * propagate to the callers any hard errors.
	 *
	 * If the driver does its own bookkeeping of stacked ports, it's not
	 * necessary to go through this helper.
	 */
	netdev_for_each_lower_dev(dev, lower_dev, iter) {
		if (netif_is_bridge_master(lower_dev))
			continue;

		/* When searching for switchdev interfaces that are neighbors
		 * of foreign ones, and @dev is a bridge, do not recurse on the
		 * foreign interface again, it was already visited.
		 */
		if (foreign_dev_check_cb && !check_cb(lower_dev) &&
		    !switchdev_lower_dev_find(lower_dev, check_cb, foreign_dev_check_cb))
			continue;

		err = __switchdev_handle_port_obj_del(lower_dev, port_obj_info,
						      check_cb, foreign_dev_check_cb,
						      del_cb);
		if (err && err != -EOPNOTSUPP)
			return err;
	}

	/* Event is neither on a bridge nor a LAG. Check whether it is on an
	 * interface that is in a bridge with us.
	 */
	if (!foreign_dev_check_cb)
		return err;

	br = netdev_master_upper_dev_get(dev);
	if (!br || !netif_is_bridge_master(br))
		return err;

	switchdev = switchdev_lower_dev_find(br, check_cb, foreign_dev_check_cb);
	if (!switchdev)
		return err;

	if (!foreign_dev_check_cb(switchdev, dev))
		return err;

	return __switchdev_handle_port_obj_del(br, port_obj_info, check_cb,
					       foreign_dev_check_cb, del_cb);
}

/* Pass through a port object deletion, if @dev passes @check_cb, or replicate
 * it towards all lower interfaces of @dev that pass @check_cb, if @dev is a
 * bridge or a LAG.
 */
int switchdev_handle_port_obj_del(struct net_device *dev,
			struct switchdev_notifier_port_obj_info *port_obj_info,
			bool (*check_cb)(const struct net_device *dev),
			int (*del_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_obj *obj))
{
	int err;

	err = __switchdev_handle_port_obj_del(dev, port_obj_info, check_cb,
					      NULL, del_cb);
	if (err == -EOPNOTSUPP)
		err = 0;
	return err;
}
EXPORT_SYMBOL_GPL(switchdev_handle_port_obj_del);

/* Same as switchdev_handle_port_obj_del(), except if object is notified on a
 * @dev that passes @foreign_dev_check_cb, it is replicated towards all devices
 * that pass @check_cb and are in the same bridge as @dev.
 */
int switchdev_handle_port_obj_del_foreign(struct net_device *dev,
			struct switchdev_notifier_port_obj_info *port_obj_info,
			bool (*check_cb)(const struct net_device *dev),
			bool (*foreign_dev_check_cb)(const struct net_device *dev,
						     const struct net_device *foreign_dev),
			int (*del_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_obj *obj))
{
	int err;

	err = __switchdev_handle_port_obj_del(dev, port_obj_info, check_cb,
					      foreign_dev_check_cb, del_cb);
	if (err == -EOPNOTSUPP)
		err = 0;
	return err;
}
EXPORT_SYMBOL_GPL(switchdev_handle_port_obj_del_foreign);

static int __switchdev_handle_port_attr_set(struct net_device *dev,
			struct switchdev_notifier_port_attr_info *port_attr_info,
			bool (*check_cb)(const struct net_device *dev),
			int (*set_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_attr *attr,
				      struct netlink_ext_ack *extack))
{
	struct switchdev_notifier_info *info = &port_attr_info->info;
	struct netlink_ext_ack *extack;
	struct net_device *lower_dev;
	struct list_head *iter;
	int err = -EOPNOTSUPP;

	extack = switchdev_notifier_info_to_extack(info);

	if (check_cb(dev)) {
		err = set_cb(dev, info->ctx, port_attr_info->attr, extack);
		if (err != -EOPNOTSUPP)
			port_attr_info->handled = true;
		return err;
	}

	/* Switch ports might be stacked under e.g. a LAG. Ignore the
	 * unsupported devices, another driver might be able to handle them. But
	 * propagate to the callers any hard errors.
	 *
	 * If the driver does its own bookkeeping of stacked ports, it's not
	 * necessary to go through this helper.
	 */
	netdev_for_each_lower_dev(dev, lower_dev, iter) {
		if (netif_is_bridge_master(lower_dev))
			continue;

		err = __switchdev_handle_port_attr_set(lower_dev, port_attr_info,
						       check_cb, set_cb);
		if (err && err != -EOPNOTSUPP)
			return err;
	}

	return err;
}

int switchdev_handle_port_attr_set(struct net_device *dev,
			struct switchdev_notifier_port_attr_info *port_attr_info,
			bool (*check_cb)(const struct net_device *dev),
			int (*set_cb)(struct net_device *dev, const void *ctx,
				      const struct switchdev_attr *attr,
				      struct netlink_ext_ack *extack))
{
	int err;

	err = __switchdev_handle_port_attr_set(dev, port_attr_info, check_cb,
					       set_cb);
	if (err == -EOPNOTSUPP)
		err = 0;
	return err;
}
EXPORT_SYMBOL_GPL(switchdev_handle_port_attr_set);

int switchdev_bridge_port_offload(struct net_device *brport_dev,
				  struct net_device *dev, const void *ctx,
				  struct notifier_block *atomic_nb,
				  struct notifier_block *blocking_nb,
				  bool tx_fwd_offload,
				  struct netlink_ext_ack *extack)
{
	struct switchdev_notifier_brport_info brport_info = {
		.brport = {
			.dev = dev,
			.ctx = ctx,
			.atomic_nb = atomic_nb,
			.blocking_nb = blocking_nb,
			.tx_fwd_offload = tx_fwd_offload,
		},
	};
	int err;

	ASSERT_RTNL();

	err = call_switchdev_blocking_notifiers(SWITCHDEV_BRPORT_OFFLOADED,
						brport_dev, &brport_info.info,
						extack);
	return notifier_to_errno(err);
}
EXPORT_SYMBOL_GPL(switchdev_bridge_port_offload);

void switchdev_bridge_port_unoffload(struct net_device *brport_dev,
				     const void *ctx,
				     struct notifier_block *atomic_nb,
				     struct notifier_block *blocking_nb)
{
	struct switchdev_notifier_brport_info brport_info = {
		.brport = {
			.ctx = ctx,
			.atomic_nb = atomic_nb,
			.blocking_nb = blocking_nb,
		},
	};

	ASSERT_RTNL();

	call_switchdev_blocking_notifiers(SWITCHDEV_BRPORT_UNOFFLOADED,
					  brport_dev, &brport_info.info,
					  NULL);
}
EXPORT_SYMBOL_GPL(switchdev_bridge_port_unoffload);

int switchdev_bridge_port_replay(struct net_device *brport_dev,
				 struct net_device *dev, const void *ctx,
				 struct notifier_block *atomic_nb,
				 struct notifier_block *blocking_nb,
				 struct netlink_ext_ack *extack)
{
	struct switchdev_notifier_brport_info brport_info = {
		.brport = {
			.dev = dev,
			.ctx = ctx,
			.atomic_nb = atomic_nb,
			.blocking_nb = blocking_nb,
		},
	};
	int err;

	ASSERT_RTNL();

	err = call_switchdev_blocking_notifiers(SWITCHDEV_BRPORT_REPLAY,
						brport_dev, &brport_info.info,
						extack);
	return notifier_to_errno(err);
}
EXPORT_SYMBOL_GPL(switchdev_bridge_port_replay);
