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

typedef int (*jesd204_cb_con_priv)(struct jesd204_dev *jdev,
				   struct jesd204_dev_con_out *con,
				   void *data);

/**
 * struct jesd204_fsm_data - JESD204 device state change data
 * @jdev_top			top JESD204 for which this state change
 * @propagated_cb	callback to propagate to trigger state change
 */
struct jesd204_fsm_data {
	struct jesd204_dev_top	*jdev_top;
	jesd204_cb_con_priv	fsm_change_cb;
	bool			inputs;
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
			     struct jesd204_fsm_table_entry *table);

/* States to transition to initialize a JESD204 link */
static struct jesd204_fsm_table_entry jesd204_init_links_states[] = {
	JESD204_STATE_OP_LAST(LINK_INIT),
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

	return jesd204_dev_set_error(jdev, con, ret);
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
	return jesd204_dev_set_error(jdev, con, ret);
}

static inline int jesd204_dev_propagate_cb(struct jesd204_dev *jdev,
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
out:
	return jesd204_dev_set_error(jdev, NULL, ret);
}

static void __jesd204_dev_top_fsm_change_cb(struct kref *ref)
{
	struct jesd204_dev_top *jdev_top;
	struct jesd204_dev *jdev;
	int ret;

	jdev_top = container_of(ref, typeof(*jdev_top), cb_ref);
	jdev = &jdev_top->jdev;

	jdev_top->cur_state = jdev_top->nxt_state;
	if (jdev_top->error) {
		dev_err(jdev->dev, "jesd got error from topology %d\n",
			jdev_top->error);
		jdev_top->cur_state = JESD204_STATE_ERROR;
		goto out;
	}

	if (jdev_top->fsm_complete_cb) {
		ret = jdev_top->fsm_complete_cb(jdev, jdev_top->cb_data);
		jesd204_dev_set_error(jdev, NULL, ret);
		if (ret) {
			dev_err(jdev->dev, "error from completion cb %d, state %s\n",
				ret, jesd204_state_str(jdev_top->cur_state));
			jdev_top->cur_state = JESD204_STATE_ERROR;
			goto out;
		}
	}

out:
	/**
	 * Reset nxt_state ; so that other devices won't run another
	 * state change
	 */
	jdev_top->nxt_state = JESD204_STATE_UNINIT;
	jdev_top->cb_data = NULL;
}

static int jesd204_dev_validate_cur_state(struct jesd204_dev_top *jdev_top,
					  struct jesd204_dev *jdev,
					  struct jesd204_dev_con_out *c,
					  enum jesd204_dev_state state)
{
	if (state != jdev_top->cur_state) {
		dev_warn(jdev->dev,
			 "invalid jesd state: %s, exp: %s, nxt: %s\n",
			 jesd204_state_str(state),
			 jesd204_state_str(jdev_top->cur_state),
			 jesd204_state_str(jdev_top->nxt_state));
		return jesd204_dev_set_error(jdev, c, -EINVAL);
	}

	return 0;
}

static int jesd204_dev_update_con_state(struct jesd204_dev *jdev,
					struct jesd204_dev_top *jdev_top,
					struct jesd204_dev_con_out *c)
{
	int ret;

	if (!c || c->state == jdev_top->nxt_state)
		return 0;

	if (c->jdev_top) {
		if (c->jdev_top->nxt_state == JESD204_STATE_UNINIT)
			return 0;
		if (c->jdev_top != jdev_top)
			return 0;
		kref_get(&c->jdev_top->cb_ref);
	}

	ret = jesd204_dev_validate_cur_state(jdev_top, jdev, c, c->state);
	if (ret)
		return ret;

	c->state = jdev_top->nxt_state;
	if (c->jdev_top)
		kref_put(&c->jdev_top->cb_ref,
			 __jesd204_dev_top_fsm_change_cb);

	return 0;
}

static int jesd204_fsm_cb(struct jesd204_dev *jdev,
			  struct jesd204_dev_con_out *con,
			  struct jesd204_fsm_data *s)
{
	struct jesd204_dev_top *jdev_top = s->jdev_top;
	int ret;

	kref_get(&jdev_top->cb_ref);

	if (s->fsm_change_cb) {
		ret = s->fsm_change_cb(jdev, con, jdev_top->cb_data);
		if (ret < 0)
			return ret;
	} else {
		ret = JESD204_STATE_CHANGE_DONE;
	}

	if (ret == JESD204_STATE_CHANGE_DONE) {
		ret = jesd204_dev_update_con_state(jdev, jdev_top, con);
		kref_put(&jdev_top->cb_ref,  __jesd204_dev_top_fsm_change_cb);
	} else {
		ret = 0;
	}

	return ret;
}

static int __jesd204_fsm(struct jesd204_dev *jdev,
			 struct jesd204_dev_top *jdev_top,
			 enum jesd204_dev_state cur_state,
			 enum jesd204_dev_state nxt_state,
			 jesd204_cb_con_priv fsm_change_cb,
			 void *cb_data,
			 jesd204_cb_priv fsm_complete_cb)
{
	struct jesd204_fsm_data data;
	int ret;

	ret = jesd204_dev_validate_cur_state(jdev_top, jdev, NULL, cur_state);
	if (ret)
		return ret;

	kref_init(&jdev_top->cb_ref);

	memset(&data, 0, sizeof(data));
	data.fsm_change_cb = fsm_change_cb;
	data.jdev_top = jdev_top;

	jdev_top->fsm_complete_cb = fsm_complete_cb;
	jdev_top->nxt_state = nxt_state;
	jdev_top->cb_data = cb_data;

	ret = jesd204_dev_propagate_cb(jdev,
				       jesd204_fsm_cb,
				       &data);

	kref_put(&jdev_top->cb_ref, __jesd204_dev_top_fsm_change_cb);

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
		       jesd204_cb_con_priv fsm_change_cb,
		       void *cb_data,
		       jesd204_cb_priv fsm_complete_cb)
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
				     void *data)
{
	if (con)
		con->jdev_top = data;

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

static int jesd204_fsm_probed_cb(struct jesd204_dev *jdev,
				 struct jesd204_dev_con_out *con,
				 void *data)
{
	if (!jdev->dev)
		return JESD204_STATE_CHANGE_DEFER;
	return JESD204_STATE_CHANGE_DONE;
}

static int jesd204_fsm_probe_done(struct jesd204_dev *jdev, void *data)
{
	return jesd204_fsm_init_links(jdev, JESD204_STATE_PROBED);
}

int jesd204_fsm_probe(struct jesd204_dev *jdev)
{
	return jesd204_fsm(jdev,
			   JESD204_STATE_INITIALIZED, JESD204_STATE_PROBED,
			   jesd204_fsm_probed_cb, NULL, jesd204_fsm_probe_done);
}

static int jesd204_fsm_table_entry_cb(struct jesd204_dev *jdev,
				      struct jesd204_dev_con_out *con,
				      void *data)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);
	struct jesd204_fsm_table_entry *table = data;
	jesd204_link_cb link_op;
	int link_idx, ret;

	if (!jdev_top) {
		if (!con || !con->jdev_top)
			return -EFAULT;
		jdev_top = con->jdev_top;
	}

	if (!jdev->link_ops)
		return JESD204_STATE_CHANGE_DONE;

	link_op = jdev->link_ops[table[0].op];
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
					void *data)
{
	struct jesd204_fsm_table_entry *table = data;

	if (table[0].last)
		return 0;

	return jesd204_fsm_table(jdev, table[0].state, &table[1]);
}

static int jesd204_fsm_table(struct jesd204_dev *jdev,
			     enum jesd204_dev_state init_state,
			     struct jesd204_fsm_table_entry *table)
{
	return jesd204_fsm(jdev,
			   init_state, table[0].state,
			   jesd204_fsm_table_entry_cb,
			   table,
			   jesd204_fsm_table_entry_done);
}

int jesd204_fsm_init_links(struct jesd204_dev *jdev,
			   enum jesd204_dev_state init_state)
{
	int ret;

	ret = jesd204_fsm_table(jdev, init_state, jesd204_init_links_states);
	if (ret)
		return ret;

	return jesd204_dev_init_link_data(jdev);
}
