// SPDX-License-Identifier: GPL-2.0+
/**
 * The JESD204 framework - finite state machine logic
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>

#include "jesd204-priv.h"

typedef int (*jesd204_fsm_cb)(struct jesd204_dev *jdev,
			      struct jesd204_dev_con_out *con,
			      struct jesd204_fsm_data *data);

typedef int (*jesd204_fsm_done_cb)(struct jesd204_dev *jdev,
				   struct jesd204_fsm_data *data);
/**
 * struct jesd204_fsm_data - JESD204 device state change data
 * @jdev_top		top JESD204 for which this state change
 * @link_idx		JESD204 link index for this transition
 * @fsm_change_cb	callback to propagate during a state transition
 * @fsm_complete_cb	callback to run after a state transition has completed
 * @cb_data		callback data for @fsm_change_cb
 * @cur_state		current state from which this FSM is transitioning
 * @nxt_state		next state to which this FSM is transitioning
 * @inputs		true if this is running on the inputs
 */
struct jesd204_fsm_data {
	struct jesd204_dev_top		*jdev_top;
	int				link_idx;
	jesd204_fsm_cb			fsm_change_cb;
	jesd204_fsm_done_cb		fsm_complete_cb;
	void				*cb_data;
	enum jesd204_dev_state		cur_state;
	enum jesd204_dev_state		nxt_state;
	bool				inputs;
};

typedef int (*jesd204_propagated_cb)(struct jesd204_dev *jdev,
				     struct jesd204_dev_con_out *con,
				     struct jesd204_fsm_data *data);

/**
 * struct jesd204_fsm_table_entry - JESD204 link states table entry
 * @state		target JESD204 state
 * @op			callback ID associated with transitioning to @state
 * @last		marker for the last state in the transition series
 */
struct jesd204_fsm_table_entry {
	enum jesd204_dev_state	state;
	enum jesd204_dev_op	op;
	bool			last;
};

struct jesd204_fsm_table_entry_iter {
	const struct jesd204_fsm_table_entry	*table;
};

#define _JESD204_STATE_OP(x, _last)	\
{					\
	.state = JESD204_STATE_##x,	\
	.op = JESD204_OP_##x,		\
	.last = _last			\
}
#define JESD204_STATE_OP(x)		_JESD204_STATE_OP(x, false)
#define JESD204_STATE_OP_LAST(x)	_JESD204_STATE_OP(x, true)

static int jesd204_fsm_table(struct jesd204_dev *jdev,
			     enum jesd204_dev_state init_state,
			     const struct jesd204_fsm_table_entry *table);

/* States to transition to initialize a JESD204 link */
static const struct jesd204_fsm_table_entry jesd204_init_links_states[] = {
	JESD204_STATE_OP_LAST(LINK_INIT),
};

/* States to transition to start a JESD204 link */
static const struct jesd204_fsm_table_entry jesd204_start_links_states[] = {
	JESD204_STATE_OP(LINK_SUPPORTED),
	JESD204_STATE_OP(LINK_SETUP),
	JESD204_STATE_OP(CLOCKS_ENABLE),
	JESD204_STATE_OP(LINK_ENABLE),
	JESD204_STATE_OP_LAST(LINK_RUNNING),
};

/* States to transition when un-initializing a device */
static const struct jesd204_fsm_table_entry jesd204_uninit_dev_states[] = {
	JESD204_STATE_OP(LINK_DISABLE),
	JESD204_STATE_OP(CLOCKS_DISABLE),
	JESD204_STATE_OP_LAST(LINK_UNINIT),
};

const char *jesd204_state_str(enum jesd204_dev_state state)
{
	switch (state) {
	case JESD204_STATE_ERROR:
		return "error";
	case JESD204_STATE_UNINIT:
		return "uninitialized";
	case JESD204_STATE_INITIALIZED:
		return "initialized";
	case JESD204_STATE_PROBED:
		return "probed";
	case JESD204_STATE_LINK_INIT:
		return "link_init";
	case JESD204_STATE_LINK_SUPPORTED:
		return "link_supported";
	case JESD204_STATE_LINK_SETUP:
		return "link_setup";
	case JESD204_STATE_CLOCKS_ENABLE:
		return "clocks_enable";
	case JESD204_STATE_CLOCKS_DISABLE:
		return "clocks_disable";
	case JESD204_STATE_LINK_ENABLE:
		return "link_enable";
	case JESD204_STATE_LINK_DISABLE:
		return "link_disable";
	case JESD204_STATE_LINK_RUNNING:
		return "link_running";
	case JESD204_STATE_LINK_UNINIT:
		return "link_uninit";
	case JESD204_STATE_DONT_CARE:
		return "dont_care";
	default:
		return "<unknown>";
	}
}

static int jesd204_dev_set_error(struct jesd204_dev *jdev,
				 struct jesd204_dev_con_out *con,
				 int err)
{
	struct jesd204_dev_top *jdev_top;

	if (err == 0)
		return 0;

	if (con)
		con->error = err;

	jdev_top = jesd204_dev_top_dev(jdev);
	if (jdev_top)
		jdev_top->error = err;

	return err;
}

static int jesd204_dev_propagate_cb_inputs(struct jesd204_dev *jdev,
					   jesd204_propagated_cb propagated_cb,
					   struct jesd204_fsm_data *data)
{
	struct jesd204_dev_con_out *con = NULL;
	unsigned int i;
	int ret = 0;

	for (i = 0; i < jdev->inputs_count; i++) {
		con = jdev->inputs[i];

		ret = jesd204_dev_propagate_cb_inputs(con->owner,
						      propagated_cb, data);
		if (ret)
			break;
		ret = propagated_cb(con->owner, con, data);
		if (ret)
			break;
	}

	return ret;
}

static int jesd204_dev_propagate_cb_outputs(struct jesd204_dev *jdev,
					    jesd204_propagated_cb propagated_cb,
					    struct jesd204_fsm_data *data)
{
	struct jesd204_dev_con_out *con = NULL;
	struct jesd204_dev_list_entry *e;
	int ret = 0;

	list_for_each_entry(con, &jdev->outputs, entry) {
		list_for_each_entry(e, &con->dests, entry) {
			ret = propagated_cb(e->jdev, con, data);
			if (ret)
				goto done;
			ret = jesd204_dev_propagate_cb_outputs(e->jdev,
							       propagated_cb,
							       data);
			if (ret)
				goto done;
		}
	}

done:
	return ret;
}

static int jesd204_dev_propagate_cb(struct jesd204_dev *jdev,
				    jesd204_propagated_cb propagated_cb,
				    struct jesd204_fsm_data *data)
{
	int ret;

	data->inputs = true;
	ret = jesd204_dev_propagate_cb_inputs(jdev, propagated_cb, data);
	if (ret)
		goto out;

	data->inputs = false;
	ret = jesd204_dev_propagate_cb_outputs(jdev, propagated_cb, data);
	if (ret)
		goto out;

	ret = propagated_cb(jdev, NULL, data);
	/* FIXME: error message here? */
out:
	return ret;
}

static void __jesd204_dev_top_fsm_change_cb(struct kref *ref)
{
	struct jesd204_dev_top *jdev_top =
		container_of(ref, typeof(*jdev_top), cb_ref);
	struct jesd204_dev *jdev = &jdev_top->jdev;
	struct jesd204_fsm_data *fsm_data = jdev_top->fsm_data;
	int ret;

	if (!fsm_data->fsm_complete_cb)
		goto out;

	ret = fsm_data->fsm_complete_cb(jdev, fsm_data->cb_data);
	if (jesd204_dev_set_error(jdev, NULL, ret)) {
		dev_err(jdev->parent,
			"error from completion cb %d, state %s\n",
			ret, jesd204_state_str(fsm_data->cur_state));
		jdev_top->error = ret;
	}
out:
	jdev_top->fsm_data = NULL;
}

static void __jesd204_fsm_kref_link_put_get(struct jesd204_dev_top *jdev_top,
					    bool put)
{
	struct kref *kref;

	kref = &jdev_top->cb_ref;

	if (put)
		kref_put(kref, __jesd204_dev_top_fsm_change_cb);
	else
		kref_get(kref);
}

static void jesd204_fsm_kref_link_get(struct jesd204_dev_top *jdev_top)
{
	__jesd204_fsm_kref_link_put_get(jdev_top, false);
}

static void jesd204_fsm_kref_link_put(struct jesd204_dev_top *jdev_top)
{
	__jesd204_fsm_kref_link_put_get(jdev_top, true);
}

static int jesd204_con_validate_cur_state(struct jesd204_dev *jdev,
					  struct jesd204_dev_con_out *c,
					  enum jesd204_dev_state cur_state,
					  enum jesd204_dev_state nxt_state)
{
	if (cur_state == JESD204_STATE_DONT_CARE)
		return 0;

	if (c && c->state == nxt_state)
		return 0;

	if (c && cur_state != c->state) {
		dev_warn(jdev->parent,
			 "invalid connection state: %s, exp: %s, nxt: %s\n",
			 jesd204_state_str(c->state),
			 jesd204_state_str(cur_state),
			 jesd204_state_str(nxt_state));
		return jesd204_dev_set_error(jdev, c, -EINVAL);
	}

	return 0;
}

static int __jesd204_fsm(struct jesd204_dev *jdev,
			 struct jesd204_dev_top *jdev_top,
			 enum jesd204_dev_state cur_state,
			 enum jesd204_dev_state nxt_state,
			 jesd204_fsm_cb fsm_change_cb,
			 void *cb_data,
			 jesd204_fsm_done_cb fsm_complete_cb)
{
	struct jesd204_fsm_data data;
	int ret;

	ret = jesd204_con_validate_cur_state(jdev, NULL, cur_state, nxt_state);
	if (ret)
		return ret;

	kref_init(&jdev_top->cb_ref);

	memset(&data, 0, sizeof(data));
	data.jdev_top = jdev_top;
	data.cur_state = cur_state;
	data.nxt_state = nxt_state;
	data.fsm_change_cb = fsm_change_cb;
	data.fsm_complete_cb = fsm_complete_cb;
	data.cb_data = cb_data;

	ret = jesd204_dev_propagate_cb(jdev, fsm_change_cb, &data);

	jesd204_fsm_kref_link_put(jdev_top);

	return ret;
}

static bool jesd204_dev_has_con_in_topology(struct jesd204_dev *jdev,
					    struct jesd204_dev_top *jdev_top)
{
	struct jesd204_dev_con_out *c;
	int i;

	list_for_each_entry(c, &jdev->outputs, entry) {
		if (c->jdev_top == jdev_top)
			return true;
	}

	for (i = 0; i < jdev->inputs_count; i++) {
		c = jdev->inputs[i];
		if (c->jdev_top == jdev_top)
			return true;
	}

	return false;
}

static int jesd204_fsm(struct jesd204_dev *jdev,
		       enum jesd204_dev_state cur_state,
		       enum jesd204_dev_state nxt_state,
		       jesd204_fsm_cb fsm_change_cb,
		       void *cb_data,
		       jesd204_fsm_done_cb fsm_complete_cb)
{
	struct list_head *jesd204_topologies = jesd204_topologies_get();
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	int ret;

	if (jdev_top)
		return __jesd204_fsm(jdev, jdev_top,
				     cur_state, nxt_state, fsm_change_cb,
				     cb_data, fsm_complete_cb);

	list_for_each_entry(jdev_top, jesd204_topologies, entry) {
		if (!jesd204_dev_has_con_in_topology(jdev, jdev_top))
			continue;

		ret = __jesd204_fsm(jdev, jdev_top,
				    cur_state, nxt_state, fsm_change_cb,
				    cb_data, fsm_complete_cb);
		if (ret)
			return ret;
	}

	return 0;
}

static int jesd204_dev_initialize_cb(struct jesd204_dev *jdev,
				     struct jesd204_dev_con_out *con,
				     struct jesd204_fsm_data *fsm_data)
{
	if (con)
		con->jdev_top = fsm_data->jdev_top;

	return JESD204_STATE_CHANGE_DONE;
}

int jesd204_init_topology(struct jesd204_dev_top *jdev_top)
{
	if (!jdev_top)
		return -EINVAL;

	return jesd204_fsm(&jdev_top->jdev,
			   JESD204_STATE_UNINIT, JESD204_STATE_INITIALIZED,
			   jesd204_dev_initialize_cb, jdev_top, NULL);
}

static int jesd204_fsm_init_links(struct jesd204_dev *jdev,
				  enum jesd204_dev_state init_state)
{
	int ret;

	ret = jesd204_fsm_table(jdev, init_state, jesd204_init_links_states);
	if (ret)
		return ret;

	return jesd204_dev_init_link_data(jdev);
}

static int jesd204_fsm_start_links(struct jesd204_dev *jdev,
				   enum jesd204_dev_state init_state)
{
	return jesd204_fsm_table(jdev, init_state, jesd204_start_links_states);
}

static int jesd204_fsm_probed_cb(struct jesd204_dev *jdev,
				 struct jesd204_dev_con_out *con,
				 struct jesd204_fsm_data *fsm_data)
{
	if (!jdev->parent)
		return JESD204_STATE_CHANGE_DEFER;
	return JESD204_STATE_CHANGE_DONE;
}

static int jesd204_fsm_probe_done(struct jesd204_dev *jdev,
				  struct jesd204_fsm_data *data)
{
	int ret;

	ret = jesd204_fsm_init_links(jdev, JESD204_STATE_PROBED);
	if (ret)
		return ret;

	return jesd204_fsm_start_links(jdev, JESD204_STATE_LINK_INIT);
}

int jesd204_fsm_probe(struct jesd204_dev *jdev)
{
	return jesd204_fsm(jdev,
			   JESD204_STATE_INITIALIZED, JESD204_STATE_PROBED,
			   jesd204_fsm_probed_cb, NULL, jesd204_fsm_probe_done);
}

static int jesd204_fsm_table_entry_cb(struct jesd204_dev *jdev,
				      struct jesd204_dev_con_out *con,
				      struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	struct jesd204_dev_top *jdev_top;
	jesd204_link_cb link_op;
	int link_idx, ret;

	if (!jdev->link_ops)
		return JESD204_STATE_CHANGE_DONE;

	link_op = jdev->link_ops[it->table[0].op];
	if (!link_op)
		return JESD204_STATE_CHANGE_DONE;

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ret = link_op(jdev, link_idx,
			      &jdev_top->active_links[link_idx]);
		if (ret != JESD204_STATE_CHANGE_DONE)
			return ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

static int jesd204_fsm_table_entry_done(struct jesd204_dev *jdev,
					struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	const struct jesd204_fsm_table_entry *table = it->table;

	if (table[0].last)
		return 0;

	return jesd204_fsm_table(jdev, table[0].state, &table[1]);
}

static int jesd204_fsm_table(struct jesd204_dev *jdev,
			     enum jesd204_dev_state init_state,
			     const struct jesd204_fsm_table_entry *table)
{
	struct jesd204_fsm_table_entry_iter it;

	it.table = table;

	return jesd204_fsm(jdev,
			   init_state, table[0].state,
			   jesd204_fsm_table_entry_cb,
			   &it,
			   jesd204_fsm_table_entry_done);
}

void jesd204_fsm_uninit_device(struct jesd204_dev *jdev)
{
	jesd204_fsm_table(jdev, JESD204_STATE_DONT_CARE,
			  jesd204_uninit_dev_states);
}
