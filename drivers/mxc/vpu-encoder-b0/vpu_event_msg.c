/*
 * Copyright(c) 2018 NXP. All rights reserved.
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * vpu_event_msg.c
 *
 * Author Ming Qian<ming.qian@nxp.com>
 */
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/vmalloc.h>

#include "vpu_encoder_b0.h"
#include "vpu_event_msg.h"

static atomic64_t total_ext_data = ATOMIC64_INIT(0);

static struct vpu_event_msg *alloc_event_msg(void)
{
	struct vpu_event_msg *msg = NULL;

	msg = vzalloc(sizeof(*msg));

	return msg;
}

static void free_event_msg(struct vpu_event_msg *msg)
{
	if (!msg)
		return;

	free_msg_ext_buffer(msg);
	vfree(msg);
}

static void set_msg_count(struct vpu_ctx *ctx, unsigned long count)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	if (attr)
		attr->msg_count = count;
}

static void inc_msg_count(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	if (attr)
		attr->msg_count++;
}

static void dec_msg_count(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	if (attr)
		attr->msg_count--;
}

static bool is_msg_count_full(struct vpu_ctx *ctx)
{
	struct vpu_attr *attr = get_vpu_ctx_attr(ctx);

	if (!attr)
		return false;
	if (attr->msg_count > MSG_COUNT_THD)
		return true;
	return false;
}

void cleanup_ctx_msg_queue(struct vpu_ctx *ctx)
{
	struct vpu_event_msg *msg;
	struct vpu_event_msg *tmp;

	WARN_ON(!ctx);

	mutex_lock(&ctx->instance_mutex);
	list_for_each_entry_safe(msg, tmp, &ctx->msg_q, list) {
		list_del_init(&msg->list);
		vpu_dbg(LVL_WARN, "drop core[%d] ctx[%d] msg:[%d]\n",
				ctx->core_dev->id, ctx->str_index, msg->msgid);
		free_event_msg(msg);
		dec_msg_count(ctx);
	}

	list_for_each_entry_safe(msg, tmp, &ctx->idle_q, list) {
		list_del_init(&msg->list);
		free_event_msg(msg);
		dec_msg_count(ctx);
	}
	mutex_unlock(&ctx->instance_mutex);
}

static int increase_idle_msg(struct vpu_ctx *ctx, u32 count)
{
	int i;

	for (i = 0; i < count; i++) {
		struct vpu_event_msg *msg = alloc_event_msg();

		if (!msg)
			continue;
		list_add_tail(&msg->list, &ctx->idle_q);
		inc_msg_count(ctx);
	}

	return 0;
}

int init_ctx_msg_queue(struct vpu_ctx *ctx)
{
	WARN_ON(!ctx);
	if (!ctx)
		return -EINVAL;

	mutex_lock(&ctx->instance_mutex);

	set_msg_count(ctx, 0);
	INIT_LIST_HEAD(&ctx->msg_q);
	INIT_LIST_HEAD(&ctx->idle_q);
	increase_idle_msg(ctx, MSG_DEFAULT_COUNT);

	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

struct vpu_event_msg *get_idle_msg(struct vpu_ctx *ctx)
{
	struct vpu_event_msg *msg = NULL;

	WARN_ON(!ctx);

	mutex_lock(&ctx->instance_mutex);
	if (list_empty(&ctx->idle_q))
		increase_idle_msg(ctx, 1);

	msg = list_first_entry(&ctx->idle_q, struct vpu_event_msg, list);
	if (msg)
		list_del_init(&msg->list);

	mutex_unlock(&ctx->instance_mutex);

	return msg;
}

void put_idle_msg(struct vpu_ctx *ctx, struct vpu_event_msg *msg)
{
	WARN_ON(!ctx);

	if (!ctx || !msg)
		return;

	free_msg_ext_buffer(msg);

	mutex_lock(&ctx->instance_mutex);
	if (is_msg_count_full(ctx)) {
		free_event_msg(msg);
		dec_msg_count(ctx);
	} else {
		list_add_tail(&msg->list, &ctx->idle_q);
	}
	mutex_unlock(&ctx->instance_mutex);
}

struct vpu_event_msg *pop_event_msg(struct vpu_ctx *ctx)
{
	struct vpu_event_msg *msg = NULL;

	WARN_ON(!ctx);

	mutex_lock(&ctx->instance_mutex);
	if (list_empty(&ctx->msg_q))
		goto exit;

	msg = list_first_entry(&ctx->msg_q, struct vpu_event_msg, list);
	if (msg)
		list_del_init(&msg->list);

exit:
	mutex_unlock(&ctx->instance_mutex);
	return msg;
}

void push_back_event_msg(struct vpu_ctx *ctx, struct vpu_event_msg *msg)
{
	WARN_ON(!ctx);

	if (!ctx || !msg)
		return;

	mutex_lock(&ctx->instance_mutex);
	list_add_tail(&msg->list, &ctx->msg_q);
	mutex_unlock(&ctx->instance_mutex);
}

int alloc_msg_ext_buffer(struct vpu_event_msg *msg, u32 number)
{
	WARN_ON(!msg);

	if (!msg || !number)
		return -EINVAL;

	msg->ext_data = vzalloc(number * sizeof(u32));
	if (!msg->ext_data)
		return -ENOMEM;
	msg->number = number;

	atomic64_add(number, &total_ext_data);
	vpu_dbg(LVL_DEBUG, "++++alloc %d msg ext data: %lld\n",
			number, get_total_ext_data_number());

	return 0;
}

void free_msg_ext_buffer(struct vpu_event_msg *msg)
{
	WARN_ON(!msg);

	if (!msg || !msg->ext_data)
		return;

	atomic64_sub(msg->number, &total_ext_data);
	vfree(msg->ext_data);
	msg->ext_data = NULL;
	vpu_dbg(LVL_DEBUG, "----free %d msg ext data: %lld\n",
			msg->number, get_total_ext_data_number());
}

long long get_total_ext_data_number(void)
{
	return atomic64_read(&total_ext_data);
}
