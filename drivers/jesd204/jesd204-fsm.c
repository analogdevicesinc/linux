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

#define JESD204_FSM_BUSY	BIT(0)

typedef int (*jesd204_fsm_cb)(struct jesd204_dev *jdev,
			      struct jesd204_link_opaque *ol,
			      struct jesd204_dev_con_out *con,
			      struct jesd204_fsm_data *data);

typedef int (*jesd204_fsm_done_cb)(struct jesd204_dev *jdev,
				   struct jesd204_link_opaque *ol,
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
			     unsigned int link_idx,
			     enum jesd204_dev_state init_state,
			     const struct jesd204_fsm_table_entry *table,
			     bool handle_busy_flags);

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

static int jesd204_con_link_idx_in_jdev_top(struct jesd204_dev_con_out *con,
					    struct jesd204_dev_top *jdev_top)
{
	int i;

	if (con->topo_id != jdev_top->topo_id)
		return -EINVAL;

	for (i = 0; i < jdev_top->num_links; i++) {
		if (jdev_top->link_ids[i] == con->link_id)
			return i;
	}

	return -EINVAL;
}

static int jesd204_dev_set_error(struct jesd204_dev *jdev,
				 struct jesd204_link_opaque *ol,
				 struct jesd204_dev_con_out *con,
				 int err)
{
	/* FIXME: should we exit here? */
	if (err == 0)
		return 0;

	jdev->error = err;

	if (con) {
		con->error = err;
		con->state = JESD204_STATE_ERROR;
	}

	if (ol) {
		ol->error = err;
		ol->state = JESD204_STATE_ERROR;
	}

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

		if (data->link_idx != JESD204_LINKS_ALL &&
		    data->link_idx != con->link_idx)
			continue;

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
			if (data->link_idx != JESD204_LINKS_ALL &&
			    data->link_idx != con->link_idx)
				continue;

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

static int __jesd204_link_fsm_update_state(struct jesd204_dev *jdev,
					   struct jesd204_link_opaque *ol,
					   struct jesd204_fsm_data *fsm_data)
{
	ol->fsm_data = NULL;

	if (ol->error) {
		dev_err(jdev->parent, "jesd got error from topology %d\n",
			ol->error);
		return ol->error;
	}

	dev_info(jdev->parent, "JESD204 link[%u] transition %s -> %s\n",
		 ol->link_idx,
		 jesd204_state_str(fsm_data->cur_state),
		 jesd204_state_str(fsm_data->nxt_state));
	ol->state = fsm_data->nxt_state;

	return 0;
}

static void __jesd204_link_fsm_done_cb(struct kref *ref)
{
	struct jesd204_link_opaque *ol =
		container_of(ref, typeof(*ol), cb_ref);
	struct jesd204_dev *jdev = &ol->jdev_top->jdev;
	struct jesd204_fsm_data *fsm_data = ol->fsm_data;
	int ret;

	ret = __jesd204_link_fsm_update_state(jdev, ol, fsm_data);
	if (ret)
		goto out;

	if (!fsm_data->fsm_complete_cb)
		goto out;

	ret = fsm_data->fsm_complete_cb(jdev, ol, fsm_data);
	if (jesd204_dev_set_error(jdev, ol, NULL, ret)) {
		dev_err(jdev->parent,
			"error from completion cb %d, state %s\n",
			ret,
			jesd204_state_str(ol->state));
	}

out:
	ol->fsm_data = NULL;
}

static void __jesd204_all_links_fsm_done_cb(struct kref *ref)
{
	struct jesd204_dev_top *jdev_top =
		container_of(ref, typeof(*jdev_top), cb_ref);
	struct jesd204_dev *jdev = &jdev_top->jdev;
	struct jesd204_fsm_data *fsm_data = jdev_top->fsm_data;
	struct jesd204_link_opaque *ol;
	unsigned int link_idx;
	int ret;

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		ret = __jesd204_link_fsm_update_state(jdev, ol, fsm_data);
		if (ret)
			goto out;
	}

	if (!fsm_data->fsm_complete_cb)
		goto out;

	ret = fsm_data->fsm_complete_cb(jdev, NULL, fsm_data);
	if (ret == 0)
		goto out;

	dev_err(jdev->parent,
		"error from completion cb %d, state %s\n",
		ret, jesd204_state_str(fsm_data->cur_state));

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		jesd204_dev_set_error(jdev, ol, NULL, ret);
	}
out:
	jdev_top->fsm_data = NULL;
}

static void __jesd204_fsm_kref_link_put_get(struct jesd204_dev_top *jdev_top,
					    unsigned int link_idx,
					    bool put)
{
	struct kref *kref;

	if (link_idx != JESD204_LINKS_ALL) {
		kref = &jdev_top->active_links[link_idx].cb_ref;
		if (put)
			kref_put(kref, __jesd204_link_fsm_done_cb);
		else
			kref_get(kref);
		return;
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		kref = &jdev_top->cb_ref;
		for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
			if (put)
				kref_put(kref,
					 __jesd204_all_links_fsm_done_cb);
			else
				kref_get(kref);
		}
	}
}

static void jesd204_fsm_kref_link_get(struct jesd204_dev_top *jdev_top,
				      unsigned int link_idx)
{
	__jesd204_fsm_kref_link_put_get(jdev_top, link_idx, false);
}

static void jesd204_fsm_kref_link_put(struct jesd204_dev_top *jdev_top,
				      unsigned int link_idx)
{
	__jesd204_fsm_kref_link_put_get(jdev_top, link_idx, true);
}

static int jesd204_con_validate_cur_state(struct jesd204_dev *jdev,
					  struct jesd204_link_opaque *ol,
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
			 "JESD204 link[%d] invalid connection state: %s, exp: %s, nxt: %s\n",
			 c->link_idx,
			 jesd204_state_str(c->state),
			 jesd204_state_str(cur_state),
			 jesd204_state_str(nxt_state));
		return jesd204_dev_set_error(jdev, ol, c, -EINVAL);
	}

	return 0;
}

static int jesd204_fsm_propagate_cb(struct jesd204_dev *jdev,
					     struct jesd204_link_opaque *ol,
					     struct jesd204_dev_con_out *con,
					     struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_dev_top *jdev_top = fsm_data->jdev_top;
	int ret;

	jesd204_fsm_kref_link_get(jdev_top, fsm_data->link_idx);

	ret = jesd204_con_validate_cur_state(jdev, ol, con,
					     fsm_data->cur_state,
					     fsm_data->nxt_state);
	if (ret)
		return ret;

	ret = fsm_data->fsm_change_cb(jdev, ol, con, fsm_data);
	if (ret < 0) {
		dev_err(jdev->parent,
			"JESD204 link[%u] got error from cb: %d\n",
			ol->link_idx, ret);
		return ret;
	}

	if (ret != JESD204_STATE_CHANGE_DONE)
		return ret;

	if (con)
		con->state = fsm_data->nxt_state;

	jesd204_fsm_kref_link_put(jdev_top, fsm_data->link_idx);
	return 0;
}

static int jesd204_fsm_propagated_cb(struct jesd204_dev *jdev,
				     struct jesd204_dev_con_out *con,
				     struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_dev_top *jdev_top;
	struct jesd204_link_opaque *ol;
	unsigned int link_idx;
	int ret;

	if (!fsm_data->fsm_change_cb)
		return 0;

	/* if this transitioned already, we're done */
	if (con && con->state == fsm_data->nxt_state)
		return 0;

	jdev_top = fsm_data->jdev_top;
	link_idx = fsm_data->link_idx;

	if (con && jdev_top->initialized && con->link_idx == JESD204_LINKS_ALL) {
		dev_err(jdev->parent, "Uninitialized connection in topology\n");
		return -EINVAL;
	}

	if (con && con->link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[con->link_idx];
		return jesd204_fsm_propagate_cb(jdev, ol, con, fsm_data);
	}

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		return jesd204_fsm_propagate_cb(jdev, ol, con, fsm_data);
	}

	/* FIXME: this implies top-level device ; see about making this better logic; same as Point1 */
	if (!con) {
		return jesd204_fsm_propagate_cb(jdev, NULL, NULL, fsm_data);
	}

	if (jdev_top->initialized)
		return 0;

	/* FIXME: make this better; same as Point1 */
	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		ret = jesd204_fsm_propagate_cb(jdev, ol, con, fsm_data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int jesd204_fsm_link_init(struct jesd204_dev_top *jdev_top,
				 struct jesd204_fsm_data *fsm_data,
				 unsigned int link_idx)
{
	struct jesd204_link_opaque *ol;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		ol->fsm_data = fsm_data;
		kref_init(&ol->cb_ref);
		return 0;
	}

	/* sequentional all links */
	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		kref_init(&jdev_top->cb_ref);
		for (link_idx = 1; link_idx < jdev_top->num_links; link_idx++)
			kref_get(&jdev_top->cb_ref);
		jdev_top->fsm_data = fsm_data;
	}

	return 0;
}

static int jesd204_fsm_test_and_set_busy(struct jesd204_dev_top *jdev_top,
					 enum jesd204_dev_state cur_state,
					 unsigned int link_idx)
{
	struct jesd204_dev *jdev = &jdev_top->jdev;
	struct jesd204_link_opaque *ol;

	/* ignore if the transition is busy */
	if (cur_state == JESD204_STATE_DONT_CARE)
		return 0;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		if (test_and_set_bit(JESD204_FSM_BUSY, &ol->flags)) {
			dev_err(jdev->parent, "JESD204 link [%u]: FSM is busy\n",
				ol->link_idx);
			return -EBUSY;
		}
		return 0;
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		if (test_and_set_bit(JESD204_FSM_BUSY, &ol->flags)) {
			dev_err(jdev->parent, "JESD204 link [%u]: FSM is busy\n",
				ol->link_idx);
			goto err_unwind_busy;
		}
	}

	return 0;

err_unwind_busy:
	for (; link_idx != JESD204_LINKS_ALL; link_idx--) {
		ol = &jdev_top->active_links[link_idx];
		clear_bit(JESD204_FSM_BUSY, &ol->flags);
	}

	return -EBUSY;
}

static void jesd204_fsm_clear_busy(struct jesd204_dev_top *jdev_top,
				   int link_idx)
{
	struct jesd204_link_opaque *ol;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		clear_bit(JESD204_FSM_BUSY, &ol->flags);
		return;
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		clear_bit(JESD204_FSM_BUSY, &ol->flags);
	}
}

static int jesd204_validate_lnk_state(struct jesd204_dev *jdev,
				      struct jesd204_link_opaque *ol,
				      struct jesd204_fsm_data *fsm_data)
{
	enum jesd204_dev_state cur_state = fsm_data->cur_state;
	enum jesd204_dev_state nxt_state = fsm_data->nxt_state;

	if (cur_state == JESD204_STATE_DONT_CARE)
		return 0;

	if (cur_state != ol->state) {
		dev_warn(jdev->parent,
			 "JESD204 link[%d] invalid link state: %s, exp: %s, nxt: %s\n",
			 ol->link_idx,
			 jesd204_state_str(ol->state),
			 jesd204_state_str(cur_state),
			 jesd204_state_str(nxt_state));
		return jesd204_dev_set_error(jdev, ol, NULL, -EINVAL);
	}

	return 0;
}

static int jesd204_validate_fsm_data(struct jesd204_dev *jdev,
				     struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_dev_top *jdev_top = fsm_data->jdev_top;
	unsigned int link_idx = fsm_data->link_idx;
	struct jesd204_link_opaque *ol;
	int ret;

	if (!jdev_top) {
		dev_err(jdev->parent, "Null top-level device\n");
		return -EINVAL;
	}

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		return jesd204_validate_lnk_state(jdev, ol, fsm_data);
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		ret = jesd204_validate_lnk_state(jdev, ol, fsm_data);
		if (ret)
			return ret;
	}

	return 0;
}

static int __jesd204_fsm(struct jesd204_dev *jdev,
			 struct jesd204_dev_top *jdev_top,
			 unsigned int link_idx,
			 enum jesd204_dev_state cur_state,
			 enum jesd204_dev_state nxt_state,
			 jesd204_fsm_cb fsm_change_cb,
			 void *cb_data,
			 jesd204_fsm_done_cb fsm_complete_cb,
			 bool handle_busy_flags)
{
	struct jesd204_fsm_data data;
	int ret;

	if (handle_busy_flags) {
		ret = jesd204_fsm_test_and_set_busy(jdev_top, cur_state,
						    link_idx);
		if (ret)
			return ret;
	}

	memset(&data, 0, sizeof(data));
	data.jdev_top = jdev_top;
	data.cur_state = cur_state;
	data.nxt_state = nxt_state;
	data.fsm_change_cb = fsm_change_cb;
	data.fsm_complete_cb = fsm_complete_cb;
	data.cb_data = cb_data;
	data.link_idx = link_idx;

	ret = jesd204_validate_fsm_data(jdev, &data);
	if (ret)
		goto out_clear_busy;

	ret = jesd204_fsm_link_init(jdev_top, &data, link_idx);
	if (ret)
		goto out_clear_busy;

	ret = jesd204_dev_propagate_cb(jdev,
				       jesd204_fsm_propagated_cb,
				       &data);

	jesd204_fsm_kref_link_put(jdev_top, link_idx);

out_clear_busy:
	if (handle_busy_flags)
		jesd204_fsm_clear_busy(jdev_top, link_idx);

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
		       unsigned int link_idx,
		       enum jesd204_dev_state cur_state,
		       enum jesd204_dev_state nxt_state,
		       jesd204_fsm_cb fsm_change_cb,
		       void *cb_data,
		       jesd204_fsm_done_cb fsm_complete_cb,
		       bool handle_busy_flags)
{
	struct list_head *jesd204_topologies = jesd204_topologies_get();
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	int ret;

	if (jdev_top)
		return __jesd204_fsm(jdev, jdev_top, link_idx,
				     cur_state, nxt_state, fsm_change_cb,
				     cb_data, fsm_complete_cb,
				     handle_busy_flags);

	list_for_each_entry(jdev_top, jesd204_topologies, entry) {
		if (!jesd204_dev_has_con_in_topology(jdev, jdev_top))
			continue;

		ret = __jesd204_fsm(jdev, jdev_top, link_idx,
				    cur_state, nxt_state, fsm_change_cb,
				    cb_data, fsm_complete_cb,
				    handle_busy_flags);
		if (ret)
			return ret;
	}

	return 0;
}

static int jesd204_dev_initialize_cb(struct jesd204_dev *jdev,
				     struct jesd204_link_opaque *ol,
				     struct jesd204_dev_con_out *con,
				     struct jesd204_fsm_data *fsm_data)
{
	int ret;

	if (fsm_data->jdev_top->initialized) {
		dev_err(jdev->parent,
			"top-level device already initialized\n");
		return -EINVAL;
	}

	if (!con)
		return JESD204_STATE_CHANGE_DONE;

	ret = jesd204_con_link_idx_in_jdev_top(con, fsm_data->jdev_top);
	if (ret >= 0) {
		con->jdev_top = fsm_data->jdev_top;
		con->link_idx = ret;
	}

	return JESD204_STATE_CHANGE_DONE;
}

int jesd204_init_topology(struct jesd204_dev_top *jdev_top)
{
	int ret;

	if (!jdev_top)
		return -EINVAL;

	ret = jesd204_fsm(&jdev_top->jdev, JESD204_LINKS_ALL,
			  JESD204_STATE_UNINIT, JESD204_STATE_INITIALIZED,
			  jesd204_dev_initialize_cb, jdev_top, NULL, true);
	if (ret)
		return ret;

	jdev_top->initialized = true;

	return 0;
}

static int jesd204_fsm_init_link(struct jesd204_dev *jdev,
				 struct jesd204_dev_top *jdev_top,
				 unsigned int link_idx,
				 enum jesd204_dev_state init_state)
{
	int ret;

	ret = jesd204_fsm_table(jdev, link_idx,
				init_state, jesd204_init_links_states, false);
	if (ret)
		return ret;

	return jesd204_dev_init_link_data(jdev_top, link_idx);
}

static int jesd204_fsm_start_link(struct jesd204_dev *jdev,
				  unsigned int link_idx,
				  enum jesd204_dev_state init_state)
{
	return jesd204_fsm_table(jdev, link_idx,
				 init_state, jesd204_start_links_states,
				 false);
}

static int jesd204_fsm_probed_cb(struct jesd204_dev *jdev,
				 struct jesd204_link_opaque *ol,
				 struct jesd204_dev_con_out *con,
				 struct jesd204_fsm_data *fsm_data)
{
	if (!jdev->fsm_started)
		return JESD204_STATE_CHANGE_DEFER;
	return JESD204_STATE_CHANGE_DONE;
}

static int jesd204_fsm_probe_done(struct jesd204_dev *jdev,
				  struct jesd204_link_opaque *ol,
				  struct jesd204_fsm_data *fsm_data)
{
	int ret;

	ret = jesd204_fsm_init_link(jdev, fsm_data->jdev_top,
				    fsm_data->link_idx,
				    JESD204_STATE_PROBED);
	if (ret)
		return ret;

	return jesd204_fsm_start_link(jdev, fsm_data->link_idx,
				      JESD204_STATE_LINK_INIT);
}

int jesd204_fsm_start(struct jesd204_dev *jdev, unsigned int link_idx)
{
	int ret;

	if (!jdev)
		return 0;

	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	jdev->fsm_started = true;

	ret = jesd204_fsm(jdev, link_idx,
			  JESD204_STATE_INITIALIZED, JESD204_STATE_PROBED,
			  jesd204_fsm_probed_cb, NULL,
			  jesd204_fsm_probe_done, true);

	if (ret)
		jdev->fsm_started = false;

	return ret;
}
EXPORT_SYMBOL_GPL(jesd204_fsm_start);

static int jesd204_fsm_table_entry_cb(struct jesd204_dev *jdev,
				      struct jesd204_link_opaque *ol,
				      struct jesd204_dev_con_out *con,
				      struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	struct jesd204_dev_top *jdev_top;
	jesd204_link_cb link_op;
	unsigned int link_idx;
	int ret, ret1;

	if (!jdev->state_ops)
		return JESD204_STATE_CHANGE_DONE;

	link_op = jdev->state_ops[it->table[0].op].per_link;
	if (!link_op)
		return JESD204_STATE_CHANGE_DONE;

	if (con)
		return link_op(jdev, con->link_idx, &ol->link);

	/* From here-on it's assumed that this is called for the top-level device */

	jdev_top = fsm_data->jdev_top;
	link_idx = fsm_data->link_idx;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		return link_op(jdev, link_idx, &ol->link);
	}

	ret1 = JESD204_STATE_CHANGE_DONE;
	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		ret = link_op(jdev, link_idx, &ol->link);
		if (ret < 0)
			return ret;
		if (ret == JESD204_STATE_CHANGE_DEFER)
			ret1 = JESD204_STATE_CHANGE_DEFER;
	}

	return ret1;
}

static int jesd204_fsm_table_entry_done(struct jesd204_dev *jdev,
					struct jesd204_link_opaque *ol,
					struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	const struct jesd204_fsm_table_entry *table = it->table;
	jesd204_dev_cb op;
	int ret;

	if (jdev->state_ops &&
	    jdev->state_ops[table[0].op].post_transition) {
		op = jdev->state_ops[table[0].op].post_transition;
		ret = op(jdev, fsm_data->link_idx);
		if (ret < 0)
			return jesd204_dev_set_error(jdev, NULL, NULL, ret);
	}

	if (table[0].last)
		return 0;

	return jesd204_fsm_table(jdev, fsm_data->link_idx,
				 table[0].state, &table[1], false);
}

static int jesd204_fsm_table(struct jesd204_dev *jdev,
			     unsigned int link_idx,
			     enum jesd204_dev_state init_state,
			     const struct jesd204_fsm_table_entry *table,
			     bool handle_busy_flags)
{
	struct jesd204_fsm_table_entry_iter it;
	jesd204_dev_cb op;
	int ret;

	it.table = table;

	if (jdev->state_ops &&
	    jdev->state_ops[table[0].op].pre_transition) {
		op = jdev->state_ops[table[0].op].pre_transition;
		ret = op(jdev, link_idx);
		if (ret < 0)
			return jesd204_dev_set_error(jdev, NULL, NULL, ret);
	}

	return jesd204_fsm(jdev, link_idx,
			   init_state, table[0].state,
			   jesd204_fsm_table_entry_cb,
			   &it,
			   jesd204_fsm_table_entry_done,
			   handle_busy_flags);
}

void jesd204_fsm_uninit_device(struct jesd204_dev *jdev)
{
	if (!jdev->fsm_started)
		return;

	jesd204_fsm_table(jdev, JESD204_LINKS_ALL,
			  JESD204_STATE_DONT_CARE, jesd204_uninit_dev_states,
			  false);
}
