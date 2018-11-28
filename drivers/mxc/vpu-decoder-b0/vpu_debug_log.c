/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file vpu-debug_log.c
 *
 * copyright here may be changed later
 *
 *
 */
#include "vpu_debug_log.h"

int init_log_info_queue(struct vpu_ctx *ctx)
{
	if (!ctx)
		return -EINVAL;

	mutex_lock(&ctx->instance_mutex);
	INIT_LIST_HEAD(&ctx->log_q);
	mutex_unlock(&ctx->instance_mutex);
	return 0;
}

int create_log_info_queue(struct vpu_ctx *ctx, u_int32 vpu_log_depth)
{
	struct vpu_log_info *vpu_info = NULL;
	u_int32 i;

	if (!ctx)
		return -EINVAL;

	for (i = 0; i < vpu_log_depth; i++) {
		vpu_info = kzalloc(sizeof(*vpu_info), GFP_KERNEL);
		if (!vpu_info)
			continue;

		list_add_tail(&vpu_info->list, &ctx->log_q);
	}

	return 0;
}

int destroy_log_info_queue(struct vpu_ctx *ctx)
{
	struct vpu_log_info *vpu_info, *temp_info;
	u_int32 ret = 0;

	if (!ctx)
		return -EINVAL;

	mutex_lock(&ctx->instance_mutex);
	if (list_empty(&ctx->log_q)) {
		ret = -EINVAL;
		goto exit;
	}
	list_for_each_entry_safe(vpu_info, temp_info, &ctx->log_q, list)
		if (!vpu_info) {
			list_del_init(&vpu_info->list);
			kfree(vpu_info);
		}

exit:
	mutex_unlock(&ctx->instance_mutex);

	return ret;
}

int put_log_info(struct vpu_ctx *ctx, struct vpu_log_info *vpu_info)
{
	if (!ctx || !vpu_info)
		return -EINVAL;

	mutex_lock(&ctx->instance_mutex);
	list_add_tail(&vpu_info->list, &ctx->log_q);
	mutex_unlock(&ctx->instance_mutex);

	return 0;
}

struct vpu_log_info *pop_log_info(struct vpu_ctx *ctx)
{
	struct vpu_log_info *vpu_info = NULL;

	if (!ctx)
		return NULL;

	mutex_lock(&ctx->instance_mutex);
	if (list_empty(&ctx->log_q))
		vpu_info = NULL;
	vpu_info = list_first_entry(&ctx->log_q, struct vpu_log_info, list);
	if (vpu_info)
		list_del_init(&vpu_info->list);
	mutex_unlock(&ctx->instance_mutex);
	return vpu_info;
}

int set_log_info(struct vpu_log_info *vpu_info, enum ACTION_TYPE type, u_int32 info, u_int32 info_data)
{
	if (!vpu_info)
		return -EINVAL;
	if (type >= LOG_RESERVED)
		return -EINVAL;

	vpu_info->type = type;
	vpu_info->log_info[type] = info;
	vpu_info->data = info_data;
	return 0;
}

int record_log_info(struct vpu_ctx *ctx, enum ACTION_TYPE type, u_int32 info, u_int32 info_data)
{
	struct vpu_log_info *vpu_info = NULL;

	if (!ctx)
		return -EINVAL;

	vpu_info = pop_log_info(ctx);
	if (!vpu_info)
		return -EINVAL;
	set_log_info(vpu_info, type, info, info_data);
	put_log_info(ctx, vpu_info);

	return 0;
}

