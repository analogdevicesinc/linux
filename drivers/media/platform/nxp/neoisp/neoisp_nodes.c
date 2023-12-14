// SPDX-License-Identifier: GPL-2.0+
/*
 * NEOISP nodes description
 *
 * Copyright 2023-2024 NXP
 */

#include <linux/videodev2.h>
#include "neoisp.h"

const struct neoisp_node_desc_s node_desc[NEOISP_NODES_COUNT] = {
	/* NEOISP_INPUT0_NODE */
	{
		.ent_name = NEOISP_NAME "-input0",
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.caps = V4L2_CAP_VIDEO_OUTPUT_MPLANE,
		.link_flags = MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED,
	},
	/* NEOISP_INPUT1_NODE */
	{
		.ent_name = NEOISP_NAME "-input1",
		.buf_type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.caps = V4L2_CAP_VIDEO_OUTPUT_MPLANE,
		.link_flags = 0u,
	},
	/* NEOISP_PARAMS_NODE */
	{
		.ent_name = NEOISP_NAME "-params",
		.buf_type = V4L2_BUF_TYPE_META_OUTPUT,
		.caps = V4L2_CAP_META_OUTPUT,
		.link_flags = MEDIA_LNK_FL_ENABLED,
	},
	/* NEOISP_FRAME_NODE */
	{
		.ent_name = NEOISP_NAME "-frame",
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE,
		.link_flags = MEDIA_LNK_FL_ENABLED,
	},
	/* NEOISP_IR_NODE */
	{
		.ent_name = NEOISP_NAME "-ir",
		.buf_type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE,
		.link_flags = 0u,
	},
	/* NEOISP_STATS_NODE */
	{
		.ent_name = NEOISP_NAME "-stats",
		.buf_type = V4L2_BUF_TYPE_META_CAPTURE,
		.caps = V4L2_CAP_META_CAPTURE,
		.link_flags = MEDIA_LNK_FL_ENABLED,
	}
};
