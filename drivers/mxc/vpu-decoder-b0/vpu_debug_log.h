/*
 * Copyright 2018 NXP
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*!
 * @file vpu_debug_log.h
 *
 * @brief VPU debug definition
 *
 */
#ifndef _VPU_DEBUG_LOG_H_
#define _VPU_DEBUG_LOG_H_
#include "vpu_b0.h"
enum ACTION_TYPE {
	LOG_NULL = 0,
	LOG_EVENT,
	LOG_COMMAND,
	LOG_PADDING,
	LOG_EOS,
	LOG_UPDATE_STREAM,
	LOG_RESERVED,
};

struct vpu_log_info {
	struct list_head list;
	enum ACTION_TYPE type;
	u_int32 log_info[LOG_RESERVED];
	u_int32 data;
};
int init_log_info_queue(struct vpu_ctx *ctx);
int create_log_info_queue(struct vpu_ctx *ctx, u_int32 vpu_log_depth);
int destroy_log_info_queue(struct vpu_ctx *ctx);
int put_log_info(struct vpu_ctx *ctx, struct vpu_log_info *vpu_info);
struct vpu_log_info *pop_log_info(struct vpu_ctx *ctx);
int set_log_info(struct vpu_log_info *vpu_info, enum ACTION_TYPE type, u_int32 info, u_int32 info_data);
int record_log_info(struct vpu_ctx *ctx, enum ACTION_TYPE type, u_int32 info, u_int32 info_data);

#endif

