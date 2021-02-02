// SPDX-License-Identifier: GPL-2.0+
/**
 * The JESD204 framework - finite state machine logic
 *
 * Copyright (c) 2019 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "jesd204-priv.h"

#define JESD204_FSM_BUSY	BIT(0)

#define EINVALID_STATE		9000

typedef int (*jesd204_fsm_cb)(struct jesd204_dev *jdev,
			      struct jesd204_dev_con_out *con,
			      unsigned int link_idx,
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
 * @rollback		this is a rollback, so we're not stopping for anything
 * @resuming		true if this FSM is resuming from a deferred state
 * @completed		true if the state change has completed
 * @has_paused		true if the FSM should stop after this transition completes
 * @mode		mode to run the callback for this FSM (see enum jesd204_state_op_mode)
 * @per_device_ran	list of JESD204 device IDs to mark when a device's
 *			callback was ran, when running ops per device/not per-link
 */
struct jesd204_fsm_data {
	struct jesd204_dev_top		*jdev_top;
	int				link_idx;
	jesd204_fsm_cb			fsm_change_cb;
	jesd204_fsm_done_cb		fsm_complete_cb;
	void				*cb_data;
	enum jesd204_dev_state		cur_state;
	enum jesd204_dev_state		nxt_state;
	bool				rollback;
	bool				resuming;
	bool				completed;
	bool				has_paused;
	enum jesd204_state_op_mode	mode;
	bool				*per_device_ran;
};

static int jesd204_fsm_handle_con_cb(struct jesd204_dev *jdev,
				     struct jesd204_dev_con_out *con,
				     unsigned int link_idx,
				     struct jesd204_fsm_data *fsm_data);

static int jesd204_fsm_handle_con(struct jesd204_dev *jdev,
				  struct jesd204_dev_con_out *con,
				  struct jesd204_fsm_data *fsm_data);

/**
 * struct jesd204_fsm_table_entry - JESD204 link states table entry
 * @state		target JESD204 state
 * @op			callback ID associated with transitioning to @state
 * @first		marker for the first state in the transition series (when iterating backward)
 * @last		marker for the last state in the transition series (when iterating forward)
 * @post_hook		hook to be run for this state, in the done_cb part
 */
struct jesd204_fsm_table_entry {
	enum jesd204_dev_state	state;
	enum jesd204_dev_op	op;
	bool			first;
	bool			last;
	jesd204_fsm_done_cb	post_hook;
};

/**
 * struct jesd204_fsm_table_entry_iter - JESD204 table state iterator
 * @table		current entry in a state table
 */
struct jesd204_fsm_table_entry_iter {
	const struct jesd204_fsm_table_entry	*table;
};

#define JESD204_STATE_NOP(x)					\
{								\
	.state = JESD204_STATE_##x,				\
	.first = true,						\
}

#define _JESD204_STATE_OP(x, _last, _post_hook)			\
{								\
	.state = JESD204_STATE_##x,				\
	.op = JESD204_OP_##x,					\
	.last = _last,						\
	.post_hook = _post_hook,				\
}
#define JESD204_STATE_OP(x)				_JESD204_STATE_OP(x, false, NULL)
#define JESD204_STATE_OP_WITH_POST_HOOK(x, _hook)	_JESD204_STATE_OP(x, false, _hook)
#define JESD204_STATE_OP_LAST(x)			_JESD204_STATE_OP(x, true, NULL)

static int jesd204_fsm_table(struct jesd204_dev *jdev,
			     unsigned int link_idx,
			     enum jesd204_dev_state init_state,
			     const struct jesd204_fsm_table_entry *table,
			     bool rollback,
			     bool resuming,
			     bool handle_busy_flags);

static int jesd204_fsm_init_link(struct jesd204_dev *jdev,
				 struct jesd204_fsm_data *fsm_data);

static void __jesd204_fsm_clear_errors(struct jesd204_dev *jdev,
				       unsigned int link_idx,
				       bool handle_busy_flags);

/* States to transition to start a JESD204 link */
static const struct jesd204_fsm_table_entry jesd204_start_links_states[] = {
	JESD204_STATE_NOP(IDLE),
	JESD204_STATE_OP(DEVICE_INIT),
	JESD204_STATE_OP_WITH_POST_HOOK(LINK_INIT, jesd204_fsm_init_link),
	JESD204_STATE_OP(LINK_SUPPORTED),
	JESD204_STATE_OP(LINK_PRE_SETUP),
	JESD204_STATE_OP(CLK_SYNC_STAGE1),
	JESD204_STATE_OP(CLK_SYNC_STAGE2),
	JESD204_STATE_OP(CLK_SYNC_STAGE3),
	JESD204_STATE_OP(LINK_SETUP),
	JESD204_STATE_OP(OPT_SETUP_STAGE1),
	JESD204_STATE_OP(OPT_SETUP_STAGE2),
	JESD204_STATE_OP(OPT_SETUP_STAGE3),
	JESD204_STATE_OP(OPT_SETUP_STAGE4),
	JESD204_STATE_OP(OPT_SETUP_STAGE5),
	JESD204_STATE_OP(CLOCKS_ENABLE),
	JESD204_STATE_OP(LINK_ENABLE),
	JESD204_STATE_OP(LINK_RUNNING),
	JESD204_STATE_OP_LAST(OPT_POST_RUNNING_STAGE),
};

const char *jesd204_state_str(enum jesd204_dev_state state)
{
	switch (state) {
	case JESD204_STATE_UNINIT:
		return "uninitialized";
	case JESD204_STATE_INITIALIZED:
		return "initialized";
	case JESD204_STATE_PROBED:
		return "probed";
	case JESD204_STATE_IDLE:
		return "idle";
	case JESD204_STATE_DEVICE_INIT:
		return "device_init";
	case JESD204_STATE_LINK_INIT:
		return "link_init";
	case JESD204_STATE_LINK_SUPPORTED:
		return "link_supported";
	case JESD204_STATE_LINK_PRE_SETUP:
		return "link_pre_setup";
	case JESD204_STATE_CLK_SYNC_STAGE1:
		return "clk_sync_stage1";
	case JESD204_STATE_CLK_SYNC_STAGE2:
		return "clk_sync_stage2";
	case JESD204_STATE_CLK_SYNC_STAGE3:
		return "clk_sync_stage3";
	case JESD204_STATE_LINK_SETUP:
		return "link_setup";
	case JESD204_STATE_CLOCKS_ENABLE:
		return "clocks_enable";
	case JESD204_STATE_LINK_ENABLE:
		return "link_enable";
	case JESD204_STATE_LINK_RUNNING:
		return "link_running";
	case JESD204_STATE_OPT_SETUP_STAGE1:
		return "opt_setup_stage1";
	case JESD204_STATE_OPT_SETUP_STAGE2:
		return "opt_setup_stage2";
	case JESD204_STATE_OPT_SETUP_STAGE3:
		return "opt_setup_stage3";
	case JESD204_STATE_OPT_SETUP_STAGE4:
		return "opt_setup_stage4";
	case JESD204_STATE_OPT_SETUP_STAGE5:
		return "opt_setup_stage5";
	case JESD204_STATE_OPT_POST_RUNNING_STAGE:
		return "opt_post_running_stage";
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

	if (con)
		con->error = err;

	ol->link.error = err;

	return err;
}

static int jesd204_fsm_propagate_cb_inputs(struct jesd204_dev *jdev_it,
					   struct jesd204_fsm_data *data)
{
	struct jesd204_dev_con_out *con;
	int i, ret = 0;

	for (i = 0; i < jdev_it->inputs_count; i++) {
		con = jdev_it->inputs[i];

		ret = jesd204_fsm_propagate_cb_inputs(con->owner, data);
		if (ret)
			break;
		ret = jesd204_fsm_handle_con(con->owner, con, data);
		if (ret)
			break;
	}

	return ret;
}

static int jesd204_fsm_propagate_rollback_cb_inputs(struct jesd204_dev *jdev_it,
						    struct jesd204_fsm_data *data)
{
	struct jesd204_dev_con_out *con;
	int i;

	if (jdev_it->inputs_count == 0)
		return 0;

	for (i = jdev_it->inputs_count - 1; i >= 0; i--) {
		con = jdev_it->inputs[i];

		jesd204_fsm_handle_con(con->owner, con, data);
		jesd204_fsm_propagate_rollback_cb_inputs(con->owner, data);
	}

	return 0;
}

static int jesd204_fsm_propagate_cb_outputs(struct jesd204_dev *jdev_it,
					    struct jesd204_fsm_data *data)
{
	struct jesd204_dev_con_out *con;
	struct jesd204_dev_list_entry *e;
	int ret = 0;

	list_for_each_entry(con, &jdev_it->outputs, entry) {
		list_for_each_entry(e, &con->dests, entry) {
			ret = jesd204_fsm_handle_con(e->jdev, con, data);
			if (ret)
				goto done;
			ret = jesd204_fsm_propagate_cb_outputs(e->jdev, data);
			if (ret)
				goto done;
		}
	}

done:
	return ret;
}

static int jesd204_fsm_propagate_rollback_cb_outputs(struct jesd204_dev *jdev_it,
						     struct jesd204_fsm_data *data)
{
	struct jesd204_dev_con_out *con;
	struct jesd204_dev_list_entry *e;

	list_for_each_entry_reverse(con, &jdev_it->outputs, entry) {
		list_for_each_entry_reverse(e, &con->dests, entry) {
			jesd204_fsm_propagate_rollback_cb_outputs(e->jdev, data);
			jesd204_fsm_handle_con(e->jdev, con, data);
		}
	}

	return 0;
}

static int jesd204_fsm_propagate_cb_top_level(struct jesd204_dev *jdev_it,
					      struct jesd204_fsm_data *fsm_data)
{
	int i, ret;

	if (fsm_data->link_idx != JESD204_LINKS_ALL)
		return jesd204_fsm_handle_con_cb(jdev_it, NULL,
						 fsm_data->link_idx,
						 fsm_data);

	for (i = 0; i < fsm_data->jdev_top->num_links; i++) {
		ret = jesd204_fsm_handle_con_cb(jdev_it, NULL, i, fsm_data);
		if (ret)
			break;
	}
	/* FIXME: error message here? */

	return ret;
}

static int jesd204_fsm_propagate_rollback_cb_top_level(struct jesd204_dev *jdev_it,
						       struct jesd204_fsm_data *fsm_data)
{
	int i;

	if (fsm_data->link_idx != JESD204_LINKS_ALL) {
		jesd204_fsm_handle_con_cb(jdev_it, NULL, fsm_data->link_idx,
					   fsm_data);
		return 0;
	}

	for (i = fsm_data->jdev_top->num_links - 1; i >= 0; i--)
		jesd204_fsm_handle_con_cb(jdev_it, NULL, i, fsm_data);

	return 0;
}

static int __jesd204_fsm_propagate_cb(struct jesd204_dev *jdev,
				      struct jesd204_fsm_data *data)
{
	int ret;

	ret = jesd204_fsm_propagate_cb_inputs(jdev, data);
	if (ret)
		return ret;

	ret = jesd204_fsm_propagate_cb_outputs(jdev, data);
	if (ret)
		return ret;

	return jesd204_fsm_propagate_cb_top_level(jdev, data);
}

static int __jesd204_fsm_propagate_rollback_cb(struct jesd204_dev *jdev,
					       struct jesd204_fsm_data *data)
{
	jesd204_fsm_propagate_rollback_cb_top_level(jdev, data);
	jesd204_fsm_propagate_rollback_cb_outputs(jdev, data);
	jesd204_fsm_propagate_rollback_cb_inputs(jdev, data);

	return 0;
}

static int jesd204_fsm_propagate_cb(struct jesd204_dev *jdev,
				    struct jesd204_fsm_data *data)
{
	int cnt = jesd204_device_count_get();
	int ret;

	/**
	 * Always alocate this, we may never know which devices want
	 * to run per_device & which per_link
	 */
	data->per_device_ran = kcalloc(cnt, sizeof(bool), GFP_KERNEL);
	if (!data->per_device_ran)
		return -ENOMEM;

	if (data->rollback)
		ret = __jesd204_fsm_propagate_rollback_cb(jdev, data);
	else
		ret = __jesd204_fsm_propagate_cb(jdev, data);

	kfree(data->per_device_ran);
	data->per_device_ran = NULL;
	return ret;
}

static int __jesd204_link_fsm_update_state(struct jesd204_dev *jdev,
					   struct jesd204_link_opaque *ol,
					   struct jesd204_fsm_data *fsm_data)
{
	bool exit_on_error = true;

	ol->fsm_data = NULL;

	if (fsm_data->cur_state == JESD204_STATE_DONT_CARE)
		exit_on_error = false;

	if (fsm_data->rollback)
		exit_on_error = false;

	if (exit_on_error && ol->link.error) {
		jesd204_err(jdev, "jesd got error from topology %d\n",
			    ol->link.error);
		return ol->link.error;
	}

	if (fsm_data->cur_state != JESD204_STATE_DONT_CARE &&
	    fsm_data->nxt_state != JESD204_STATE_DONT_CARE)
		jesd204_info(jdev, "JESD204[%u] transition %s -> %s\n",
			     ol->link.link_id,
			     jesd204_state_str(fsm_data->cur_state),
			     jesd204_state_str(fsm_data->nxt_state));

	if (fsm_data->nxt_state != JESD204_STATE_DONT_CARE)
		ol->state = fsm_data->nxt_state;

	fsm_data->completed = true;

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

	ret = fsm_data->fsm_complete_cb(jdev, fsm_data);
	if (ret == 0)
		goto out;

	if (fsm_data->rollback)
		goto out;

	jesd204_dev_set_error(jdev, ol, NULL, ret);

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
	int link_idx;
	int ret;

	if (fsm_data->rollback) {
		for (link_idx = jdev_top->num_links - 1; link_idx >= 0; link_idx--) {
			ol = &jdev_top->active_links[link_idx];
			__jesd204_link_fsm_update_state(jdev, ol, fsm_data);
		}
	} else {
		for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
			ol = &jdev_top->active_links[link_idx];
			ret = __jesd204_link_fsm_update_state(jdev, ol, fsm_data);
			if (ret)
				goto out;
		}
	}

	if (!fsm_data->fsm_complete_cb)
		goto out;

	ret = fsm_data->fsm_complete_cb(jdev, fsm_data);
	if (ret == 0)
		goto out;

	if (fsm_data->rollback)
		goto out;

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

	kref = &jdev_top->cb_ref;
	if (put)
		kref_put(kref, __jesd204_all_links_fsm_done_cb);
	else
		kref_get(kref);
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
					  struct jesd204_dev_con_out *c,
					  struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_link_opaque *ol;

	if (fsm_data->cur_state == JESD204_STATE_DONT_CARE)
		return 0;

	if (fsm_data->rollback)
		return 0;

	if (c->state == fsm_data->nxt_state)
		return 0;

	if (c->link_idx == JESD204_LINKS_ALL) {
		if (!fsm_data->jdev_top->initialized)
			return 0;
		jesd204_err(jdev, "Uninitialized connection in topology\n");
		return -EINVAL;
	}

	if (fsm_data->cur_state != c->state) {
		ol = &fsm_data->jdev_top->active_links[c->link_idx];
		jesd204_warn(jdev,
			 "JESD204[%d] invalid con[%u] state: %s, exp: %s, nxt: %s\n",
			 c->link_id,
			 c->id,
			 jesd204_state_str(c->state),
			 jesd204_state_str(fsm_data->cur_state),
			 jesd204_state_str(fsm_data->nxt_state));
		return jesd204_dev_set_error(jdev, ol, c, -EINVAL);
	}

	return 0;
}

static int jesd204_fsm_handle_con_cb(struct jesd204_dev *jdev_it,
				     struct jesd204_dev_con_out *con,
				     unsigned int link_idx,
				     struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_dev_top *jdev_top = fsm_data->jdev_top;
	struct jesd204_link_opaque *ol;
	int ret;

	jesd204_fsm_kref_link_get(jdev_top, fsm_data->link_idx);

	if (fsm_data->mode == JESD204_STATE_OP_MODE_PER_DEVICE) {
		if (fsm_data->per_device_ran[jdev_it->id])
			goto out;

		fsm_data->per_device_ran[jdev_it->id] = true;
	}

	ret = fsm_data->fsm_change_cb(jdev_it, con, link_idx, fsm_data);

	/**
	 * Rollbacks trump everything, we don't return any errors
	 */
	if (fsm_data->rollback)
		goto out;

	if (ret < 0) {
		ol = &jdev_top->active_links[link_idx];
		jesd204_err(jdev_it,
			    "JESD204[%u] got error from cb: %d\n",
			    ol->link.link_id, ret);
		return jesd204_dev_set_error(jdev_it, ol, con, ret);
	}

	if (ret != JESD204_STATE_CHANGE_DONE)
		return ret;

out:
	if (con && fsm_data->nxt_state != JESD204_STATE_DONT_CARE)
		con->state = fsm_data->nxt_state;

	jesd204_fsm_kref_link_put(jdev_top, fsm_data->link_idx);
	return 0;
}

static int jesd204_fsm_handle_con(struct jesd204_dev *jdev_it,
				  struct jesd204_dev_con_out *con,
				  struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_dev_top *jdev_top;
	unsigned int link_idx;
	int ret;

	if (!fsm_data->fsm_change_cb)
		return 0;

	/* if this transitioned already, we're done */
	if (con->state == fsm_data->nxt_state)
		return 0;

	ret = jesd204_con_validate_cur_state(jdev_it, con, fsm_data);
	if (ret)
		return ret;

	if (fsm_data->link_idx != JESD204_LINKS_ALL &&
	    fsm_data->link_idx != con->link_idx)
		return 0;

	if (con->link_idx != JESD204_LINKS_ALL)
		return jesd204_fsm_handle_con_cb(jdev_it, con, con->link_idx,
						 fsm_data);

	jdev_top = fsm_data->jdev_top;

	if (jdev_top->initialized)
		return 0;

	/**
	 * At this point, the connections don't have initialized link indexes
	 * so they don't know to which JESD204 link they belong to.
	 * So, we initialize then here; this is a special case,
	 * we are running jesd204_dev_initialize_cb()
	 */
	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ret = jesd204_fsm_handle_con_cb(jdev_it, con, link_idx, fsm_data);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static void jesd204_fsm_link_init(struct jesd204_dev_top *jdev_top,
				  struct jesd204_fsm_data *fsm_data)
{
	unsigned int link_idx = fsm_data->link_idx;
	struct jesd204_link_opaque *ol;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		ol->fsm_data = fsm_data;
		kref_init(&ol->cb_ref);
		return;
	}

	kref_init(&jdev_top->cb_ref);
	jdev_top->fsm_data = fsm_data;
}

static int jesd204_fsm_test_and_set_busy(struct jesd204_dev_top *jdev_top,
					 enum jesd204_dev_state cur_state,
					 unsigned int link_idx)
{
	struct jesd204_dev *jdev = &jdev_top->jdev;
	struct jesd204_link_opaque *ol;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];
		if (test_and_set_bit(JESD204_FSM_BUSY, &ol->flags)) {
			jesd204_err(jdev, "JESD204[%u]: FSM is busy\n",
				    ol->link.link_id);
			return -EBUSY;
		}
		return 0;
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];
		if (test_and_set_bit(JESD204_FSM_BUSY, &ol->flags)) {
			jesd204_err(jdev, "JESD204[%u]: FSM is busy\n",
				    ol->link.link_id);
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

	if (cur_state == ol->state)
		return 0;

	/* When rolling back we need to validate this, to not skip the first state (in a rollback) */
	if (fsm_data->rollback && nxt_state == ol->state)
		return 0;

	if (fsm_data->rollback || fsm_data->resuming)
		return -EINVALID_STATE;

	jesd204_warn(jdev,
		 "JESD204[%u] invalid link state: %s, exp: %s, nxt: %s\n",
		 ol->link.link_id,
		 jesd204_state_str(ol->state),
		 jesd204_state_str(cur_state),
		 jesd204_state_str(nxt_state));

	return jesd204_dev_set_error(jdev, ol, NULL, -EINVAL);
}

static int jesd204_validate_resuming_state(struct jesd204_dev *jdev,
					   struct jesd204_link_opaque *ol,
					   struct jesd204_fsm_data *fsm_data)
{
	enum jesd204_dev_state cur_state = fsm_data->cur_state;
	bool resuming = fsm_data->resuming;

	if (cur_state == JESD204_STATE_DONT_CARE)
		return 0;

	if (ol->fsm_paused && !resuming) {
		jesd204_warn(jdev, "JESD204[%u] FSM is paused; a resume is required\n",
			     ol->link.link_id);
		return -EINVAL;
	}

	if (!ol->fsm_paused && resuming) {
		jesd204_warn(jdev, "JESD204[%u] FSM is NOT paused; a transition is required\n",
			     ol->link.link_id);
		return -EINVAL;
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

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];

		ret = jesd204_validate_resuming_state(jdev, ol, fsm_data);
		if (ret)
			return ret;

		return jesd204_validate_lnk_state(jdev, ol, fsm_data);
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];

		ret = jesd204_validate_resuming_state(jdev, ol, fsm_data);
		if (ret)
			return ret;

		ret = jesd204_validate_lnk_state(jdev, ol, fsm_data);
		if (ret)
			return ret;
	}

	return 0;
}

static int __jesd204_fsm(struct jesd204_dev *jdev,
			 struct jesd204_dev_top *jdev_top,
			 struct jesd204_fsm_data *data,
			 bool handle_busy_flags)
{
	int ret;

	data->jdev_top = jdev_top;

	if (handle_busy_flags) {
		ret = jesd204_fsm_test_and_set_busy(jdev_top, data->cur_state,
						    data->link_idx);
		if (ret)
			return ret;
	}

	ret = jesd204_validate_fsm_data(jdev, data);
	if (ret)
		goto out_clear_busy;

	jesd204_fsm_link_init(jdev_top, data);

	/**
	 * Always propagate from the top-level device, otherwise if
	 * if we propagate from a device that is somewhere in a topology
	 * and belongs to a certain JESD204 link, we may miss certain
	 * devices when propagating changes for all JESD204 links
	 */
	ret = jesd204_fsm_propagate_cb(&jdev_top->jdev, data);
	if (ret)
		goto out_clear_busy;

	jesd204_fsm_kref_link_put(data->jdev_top, data->link_idx);

out_clear_busy:
	if (handle_busy_flags)
		jesd204_fsm_clear_busy(jdev_top, data->link_idx);

	return ret;
}

static int jesd204_fsm(struct jesd204_dev *jdev,
		       struct jesd204_fsm_data *data,
		       bool handle_busy_flags)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_get_topology_top_dev(jdev);

	if (!jdev_top)
		return -EFAULT;

	return __jesd204_fsm(jdev, jdev_top, data, handle_busy_flags);
}

static int jesd204_dev_initialize_cb(struct jesd204_dev *jdev,
				     struct jesd204_dev_con_out *con,
				     unsigned int link_idx,
				     struct jesd204_fsm_data *fsm_data)
{
	int ret;

	if (fsm_data->jdev_top->initialized) {
		jesd204_err(jdev, "top-level device already initialized\n");
		return -EINVAL;
	}

	if (!con)
		return JESD204_STATE_CHANGE_DONE;

	ret = jesd204_con_link_idx_in_jdev_top(con, fsm_data->jdev_top);
	if (ret < 0)
		return JESD204_STATE_CHANGE_DEFER;

	con->jdev_top = fsm_data->jdev_top;
	con->link_idx = ret;

	return JESD204_STATE_CHANGE_DONE;
}

int jesd204_init_topology(struct jesd204_dev_top *jdev_top)
{
	struct jesd204_fsm_data data;
	int ret;

	if (!jdev_top)
		return -EINVAL;

	memset(&data, 0, sizeof(data));
	data.cur_state = JESD204_STATE_UNINIT;
	data.nxt_state = JESD204_STATE_INITIALIZED;
	data.fsm_change_cb = jesd204_dev_initialize_cb;
	data.cb_data = jdev_top;
	data.link_idx = JESD204_LINKS_ALL;

	ret = jesd204_fsm(&jdev_top->jdev, &data, true);
	if (ret)
		return ret;

	jdev_top->initialized = true;

	return 0;
}

static int jesd204_fsm_init_link(struct jesd204_dev *jdev,
				 struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_top_dev(jdev);

	if (!jdev_top)
		return 0;

	return jesd204_dev_init_link_data(jdev_top, fsm_data->link_idx);
}

static int jesd204_fsm_start_link(struct jesd204_dev *jdev,
				  unsigned int link_idx,
				  enum jesd204_dev_state init_state,
				  bool resuming,
				  bool handle_busy_flags)
{
	return jesd204_fsm_table(jdev, link_idx, init_state,
				 jesd204_start_links_states,
				 false, resuming, handle_busy_flags);
}

static int jesd204_init_secondary_sysref_cb(struct jesd204_dev *jdev,
				  struct jesd204_dev_top *jdev_top)
{
	if (!jdev->is_sec_sysref_provider)
		return 0;

	if (!jdev->dev_data->sysref_cb) {
		jesd204_err(jdev, "Configured as SYSREF, but no SYSREF cb\n");
		return -EINVAL;
	}

	/**
	 * For multi-link devices, we get here the number of links.
	 * FIXME: would it make sense to re-use the per_device_ran[] logic?
	 */
	if (jdev_top->jdev_sysref_sec && (jdev_top->jdev_sysref_sec != jdev)) {
		jesd204_err(jdev, "Duplicate SYSREF for topology %pOF\n",
			    jdev_top->jdev.np);
		jesd204_err(jdev, "Previously set SYSREF is %pOF\n",
			    jdev_top->jdev_sysref_sec->np);
		return -EEXIST;
	}

	if (jdev_top->jdev_sysref_sec)
		return 0;

	jesd204_info(jdev, "Using as sec SYSREF provider\n");
	jdev_top->jdev_sysref_sec = jdev;

	return 0;
}

static int jesd204_init_sysref_cb(struct jesd204_dev *jdev,
				  struct jesd204_dev_top *jdev_top)
{
	if (!jdev->is_sysref_provider)
		return jesd204_init_secondary_sysref_cb(jdev, jdev_top);

	if (!jdev->dev_data->sysref_cb) {
		jesd204_err(jdev, "Configured as SYSREF, but no SYSREF cb\n");
		return -EINVAL;
	}

	/**
	 * For multi-link devices, we get here the number of links.
	 * FIXME: would it make sense to re-use the per_device_ran[] logic?
	 */
	if (jdev_top->jdev_sysref && (jdev_top->jdev_sysref != jdev)) {
		jesd204_err(jdev, "Duplicate SYSREF for topology %pOF\n",
			    jdev_top->jdev.np);
		jesd204_err(jdev, "Previously set SYSREF is %pOF\n",
			    jdev_top->jdev_sysref->np);
		return -EEXIST;
	}

	if (jdev_top->jdev_sysref)
		return 0;

	jesd204_info(jdev, "Using as SYSREF provider\n");
	jdev_top->jdev_sysref = jdev;

	return 0;
}

static int jesd204_fsm_probed_cb(struct jesd204_dev *jdev,
				 struct jesd204_dev_con_out *con,
				 unsigned int link_idx,
				 struct jesd204_fsm_data *fsm_data)
{
	int ret;

	if (!jdev->fsm_inited)
		return JESD204_STATE_CHANGE_DEFER;

	ret = jesd204_init_sysref_cb(jdev, fsm_data->jdev_top);
	if (ret)
		return ret;

	return JESD204_STATE_CHANGE_DONE;
}

static int jesd204_fsm_probe_done(struct jesd204_dev *jdev,
				  struct jesd204_fsm_data *fsm_data)
{
	return jesd204_fsm_start_link(jdev, fsm_data->link_idx,
				      JESD204_STATE_PROBED, false, false);
}

static int jesd204_fsm_start_from_probe(struct jesd204_dev *jdev,
					unsigned int link_idx)
{
	struct jesd204_fsm_data data;
	int ret;

	jdev->fsm_inited = true;

	memset(&data, 0, sizeof(data));
	data.cur_state = JESD204_STATE_INITIALIZED;
	data.nxt_state = JESD204_STATE_PROBED;
	data.fsm_change_cb = jesd204_fsm_probed_cb;
	data.fsm_complete_cb = jesd204_fsm_probe_done;
	data.link_idx = link_idx;

	ret = jesd204_fsm(jdev, &data, true);
	if (ret)
		jdev->fsm_inited = false;

	return ret;
}

static int __jesd204_fsm_start(struct jesd204_dev *jdev, unsigned int link_idx,
			       bool resuming)
{
	if (!jdev)
		return 0;

	if (IS_ERR(jdev))
		return PTR_ERR(jdev);

	if (!jdev->fsm_inited)
		return jesd204_fsm_start_from_probe(jdev, link_idx);

	return jesd204_fsm_start_link(jdev, link_idx,
				      JESD204_STATE_IDLE, resuming, true);
}

int jesd204_fsm_start(struct jesd204_dev *jdev, unsigned int link_idx)
{
	return __jesd204_fsm_start(jdev, link_idx, false);
}
EXPORT_SYMBOL_GPL(jesd204_fsm_start);

int jesd204_fsm_resume(struct jesd204_dev *jdev, unsigned int link_idx)
{
	return __jesd204_fsm_start(jdev, link_idx, true);
}
EXPORT_SYMBOL_GPL(jesd204_fsm_resume);

static void jesd204_fsm_handle_stop_state(struct jesd204_dev *jdev,
					  unsigned int link_idx,
					  struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	int state_idx;

	if (fsm_data->rollback)
		return;

	/* FSM states from DT start at offset 100 */
	state_idx = it->table[0].state - JESD204_STATE_FSM_OFFSET;
	if (state_idx < 0 || state_idx >= JESD204_FSM_STATES_NUM)
		return;

	/* FIXME: maybe add support for stopping per-link? */
	if (!jdev->stop_states[state_idx])
		return;

	fsm_data->has_paused = true;
}

static int jesd204_fsm_table_entry_cb(struct jesd204_dev *jdev,
				      struct jesd204_dev_con_out *con,
				      unsigned int link_idx,
				      struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	const struct jesd204_state_op *state_op;
	enum jesd204_state_op_reason reason;
	struct jesd204_link_opaque *ol;

	jesd204_fsm_handle_stop_state(jdev, link_idx, fsm_data);

	if (!jdev->dev_data->state_ops)
		return JESD204_STATE_CHANGE_DONE;

	state_op = &jdev->dev_data->state_ops[it->table[0].op];

	if (fsm_data->rollback)
		reason = JESD204_STATE_OP_REASON_UNINIT;
	else
		reason = JESD204_STATE_OP_REASON_INIT;

	switch (state_op->mode) {
	case JESD204_STATE_OP_MODE_PER_DEVICE:
		if (!state_op->per_device)
			return JESD204_STATE_CHANGE_DONE;
		if (fsm_data->per_device_ran[jdev->id])
			return JESD204_STATE_CHANGE_DONE;
		fsm_data->per_device_ran[jdev->id] = true;
		return state_op->per_device(jdev, reason);
	case JESD204_STATE_OP_MODE_PER_LINK:
		if (!state_op->per_link)
			return JESD204_STATE_CHANGE_DONE;
		ol = &fsm_data->jdev_top->active_links[link_idx];
		return state_op->per_link(jdev, reason, &ol->link);
	default:
		jesd204_err(jdev, "Invalid state_op mode %d\n", state_op->mode);
		return -EINVAL;
	}
}

static int jesd204_fsm_table_entry_done(struct jesd204_dev *jdev,
					struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_fsm_table_entry_iter *it = fsm_data->cb_data;
	const struct jesd204_fsm_table_entry *table = it->table;
	const struct jesd204_state_op *state_op;
	int ret;

	if (table[0].post_hook) {
		ret = table[0].post_hook(jdev, fsm_data);
		if (ret)
			return ret;
	}

	if (!fsm_data->rollback) {
		state_op = &jdev->dev_data->state_ops[it->table[0].op];
		if (state_op->post_state_sysref && jesd204_dev_is_top(jdev))
			jesd204_sysref_async(jdev);
	}

	return 0;
}

static bool jesd204_fsm_table_end(const struct jesd204_fsm_table_entry *entry,
				  bool rollback)
{
	if (rollback)
		return entry->first;
	return entry->last;
}

static void jesd204_fsm_set_paused_state(struct jesd204_dev *jdev,
					 struct jesd204_dev_top *jdev_top,
					 unsigned int link_idx,
					 const struct jesd204_fsm_table_entry *table,
					 bool paused)
{
	struct jesd204_link_opaque *ol;

	/* this is for displaying the correct state when resuming */
	if (!paused && !table[0].first)
		table--;

	if (link_idx != JESD204_LINKS_ALL) {
		ol = &jdev_top->active_links[link_idx];

		if (ol->fsm_paused == paused)
			return;

		jesd204_notice(jdev, "JESD204[%u] %s state %s\n",
			       link_idx,
			       paused ? "paused at" : "resuming from",
			       jesd204_state_str(table[0].state));

		ol->fsm_paused = paused;
		return;
	}

	for (link_idx = 0; link_idx < jdev_top->num_links; link_idx++) {
		ol = &jdev_top->active_links[link_idx];

		if (ol->fsm_paused == paused)
			continue;

		jesd204_notice(jdev, "JESD204[%u] %s state %s\n",
			       link_idx,
			       paused ? "paused at" : "resuming from",
			       jesd204_state_str(table[0].state));

		ol->fsm_paused = paused;
	}
}

static int jesd204_fsm_table_single(struct jesd204_dev *jdev,
				    struct jesd204_dev_top *jdev_top,
				    struct jesd204_fsm_data *data,
				    enum jesd204_dev_state init_state,
				    const struct jesd204_fsm_table_entry *table,
				    bool rollback,
				    bool handle_busy_flags)
{
	struct jesd204_fsm_table_entry_iter *it = data->cb_data;
	const struct jesd204_state_op *state_op;
	int ret, ret1, cnt;

	cnt = jesd204_device_count_get();

	it->table = table;

	ret1 = 0;
	ret = 0;
	/**
	 * FIXME: the handle_busy_flags logic needs re-visit, we should lock
	 * here and unlock after the loop is done
	 */
	while (!jesd204_fsm_table_end(&it->table[0], rollback)) {
		it->table = table;

		state_op = &jdev->dev_data->state_ops[table[0].op];

		data->completed = false;
		data->cur_state = init_state;
		data->nxt_state = table[0].state;
		data->rollback = rollback;

		ret = __jesd204_fsm(jdev, jdev_top, data, handle_busy_flags);

		if (ret == -EINVALID_STATE && data->resuming)
			goto next_state;

		if (ret && !rollback) {
			const struct jesd204_fsm_table_entry *t = table;
			ret1 = ret;
			if (!t[0].first)
				t--;
			jesd204_err(jdev, "Rolling back from '%s', got error %d\n",
				    jesd204_state_str(t[0].state), ret);
			rollback = true;
			continue;
		}

		jesd204_fsm_set_paused_state(jdev, jdev_top, data->link_idx,
					     table, data->has_paused);

		if (data->has_paused)
			break;

		if (!data->completed && !rollback)
			break;

		data->resuming = false;
next_state:
		init_state = table[0].state;

		if (rollback)
			table--;
		else
			table++;
	}

	return ret1;
}

static int jesd204_fsm_run_finished_cb_cb(struct jesd204_dev *jdev,
					  struct jesd204_dev_con_out *con,
					  unsigned int link_idx,
					  struct jesd204_fsm_data *fsm_data)
{
	const struct jesd204_link * const *links = fsm_data->cb_data;

	if (!jdev->dev_data->fsm_finished_cb)
		return JESD204_STATE_CHANGE_DONE;

	jdev->dev_data->fsm_finished_cb(jdev, links,
					fsm_data->jdev_top->num_links);

	return JESD204_STATE_CHANGE_DONE;
}

static void jesd204_fsm_run_finished_cb(struct jesd204_dev *jdev,
					struct jesd204_dev_top *jdev_top,
					unsigned int link_idx,
					bool handle_busy_flags)
{
	struct jesd204_fsm_data data;
	struct jesd204_link **links;
	unsigned int i;

	links = kcalloc(jdev_top->num_links, sizeof(*links), GFP_KERNEL);
	if (!links)
		return;

	for (i = 0; i < jdev_top->num_links; i++)
		links[i] = &jdev_top->active_links[i].link;

	memset(&data, 0, sizeof(data));
	data.fsm_change_cb = jesd204_fsm_run_finished_cb_cb;
	data.cur_state = JESD204_STATE_DONT_CARE;
	data.nxt_state = JESD204_STATE_DONT_CARE;
	data.link_idx = link_idx;
	data.mode = JESD204_STATE_OP_MODE_PER_DEVICE;
	data.cb_data = links;

	jesd204_fsm(jdev, &data, handle_busy_flags);

	kfree(links);
}

static int jesd204_fsm_table(struct jesd204_dev *jdev,
			     unsigned int link_idx,
			     enum jesd204_dev_state init_state,
			     const struct jesd204_fsm_table_entry *table,
			     bool rollback,
			     bool resuming,
			     bool handle_busy_flags)
{
	struct jesd204_dev_top *jdev_top = jesd204_dev_get_topology_top_dev(jdev);
	struct jesd204_fsm_table_entry_iter it;
	struct jesd204_fsm_data data;
	unsigned int num_retries;
	int ret;

	if (!jdev_top)
		return -EFAULT;

	memset(&data, 0, sizeof(data));
	data.fsm_change_cb = jesd204_fsm_table_entry_cb;
	data.fsm_complete_cb = jesd204_fsm_table_entry_done;
	data.resuming = resuming;
	data.cb_data = &it;
	data.link_idx = link_idx;

	num_retries = jdev_top->num_retries;

	do {
		ret = jesd204_fsm_table_single(jdev, jdev_top, &data,
					       init_state, table, rollback,
					       handle_busy_flags);
		/**
		 * If we got an error, we rolled-back, we should be in IDLE
		 * for the next retry, and clear errors on JESD204 link objects
		 */
		if (ret && num_retries) {
			init_state = JESD204_STATE_IDLE;
			__jesd204_fsm_clear_errors(jdev, link_idx,
						   handle_busy_flags);
		}
	} while (ret && num_retries--);

	if (ret)
		jesd204_err(jdev, "FSM completed with error %d\n", ret);

	jesd204_fsm_run_finished_cb(jdev, jdev_top, link_idx, handle_busy_flags);

	return ret;
}

void jesd204_fsm_stop(struct jesd204_dev *jdev, unsigned int link_idx)
{
	const struct jesd204_fsm_table_entry *start;

	if (!jdev->fsm_inited)
		return;

	start = &jesd204_start_links_states[ARRAY_SIZE(jesd204_start_links_states) - 1];

	jesd204_fsm_table(jdev, link_idx, start->state, start, true, false, true);
}
EXPORT_SYMBOL_GPL(jesd204_fsm_stop);

static int jesd204_fsm_clr_errors_cb(struct jesd204_dev *jdev,
				     struct jesd204_dev_con_out *con,
				     unsigned int link_idx,
				     struct jesd204_fsm_data *fsm_data)
{
	struct jesd204_link_opaque *ol;

	if (con) {
		con->error = 0;
		return JESD204_STATE_CHANGE_DONE;
	}

	ol = &fsm_data->jdev_top->active_links[link_idx];
	ol->link.error = 0;

	return JESD204_STATE_CHANGE_DONE;
}

static void __jesd204_fsm_clear_errors(struct jesd204_dev *jdev,
				       unsigned int link_idx,
				       bool handle_busy_flags)
{
	struct jesd204_fsm_data data;

	memset(&data, 0, sizeof(data));
	data.fsm_change_cb = jesd204_fsm_clr_errors_cb;
	data.cur_state = JESD204_STATE_DONT_CARE;
	data.nxt_state = JESD204_STATE_DONT_CARE;
	data.link_idx = link_idx;

	jesd204_fsm(jdev, &data, handle_busy_flags);
}

void jesd204_fsm_clear_errors(struct jesd204_dev *jdev, unsigned int link_idx)
{
	__jesd204_fsm_clear_errors(jdev, link_idx, true);
}
EXPORT_SYMBOL_GPL(jesd204_fsm_clear_errors);
