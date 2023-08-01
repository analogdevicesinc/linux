/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Analog Devices Inc.
 */

#ifndef MAX_SERDES_H
#define MAX_SERDES_H

#include <linux/types.h>

#include <media/v4l2-subdev.h>

#define MAX_SERDES_PHYS_MAX		4
#define MAX_SERDES_STREAMS_NUM		4
#define MAX_SERDES_VC_ID_NUM		4

enum max_gmsl_version {
	MAX_GMSL_MIN,
	MAX_GMSL_2_3Gbps = MAX_GMSL_MIN,
	MAX_GMSL_2_6Gbps,
	MAX_GMSL_3,
	MAX_GMSL_MAX = MAX_GMSL_3,
};

struct max_phys_config {
	unsigned int lanes[MAX_SERDES_PHYS_MAX];
	unsigned int clock_lane[MAX_SERDES_PHYS_MAX];
};

struct max_phys_configs {
	const struct max_phys_config *configs;
	unsigned int num_configs;
};

struct max_i2c_xlate {
	u8 src;
	u8 dst;
	bool en;
};

struct max_mipi_format {
	u8 dt;
	u8 bpp;
};

struct max_source {
	struct v4l2_subdev *sd;
	u16 pad;
	struct fwnode_handle *ep_fwnode;

	unsigned int index;
};

struct max_asc {
	struct v4l2_async_connection base;
	struct max_source *source;
};

static inline struct max_asc *asc_to_max(struct v4l2_async_connection *asc)
{
	return container_of(asc, struct max_asc, base);
}

const struct max_mipi_format *max_mipi_format_by_dt(u8 dt);

int max_get_fd_stream_entry(struct v4l2_subdev *sd, u32 pad, u32 stream,
			    struct v4l2_mbus_frame_desc_entry *entry);

int max_get_bpps(struct max_source *sources, u32 source_sink_pad_offset,
		 u32 *bpps, const struct v4l2_subdev_krouting *routing,
		 u32 pad, u64 streams_mask);
int max_process_bpps(struct device *dev, u32 bpps, u32 allowed_double_bpps,
		     unsigned int *doubled_bpp);

int max_xlate_enable_disable_streams(struct max_source *sources,
				     u32 source_sink_pad_offset,
				     const struct v4l2_subdev_state *state,
				     u32 pad, u64 updated_streams_mask,
				     u32 sink_pad_start, u32 num_sink_pads,
				     bool enable);

int max_get_streams_masks(struct device *dev,
			  const struct v4l2_subdev_state *state,
			  u32 pad, u64 updated_streams_mask,
			  u32 num_pads, u32 sink_pad_start,
			  u32 num_sink_pads, u64 *old_streams_masks,
			  u64 **new_streams_masks, bool enable);

#endif // MAX_SERDES_H
