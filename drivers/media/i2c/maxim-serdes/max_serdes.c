// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stringify.h>

#include <media/mipi-csi2.h>

#include <uapi/linux/media-bus-format.h>

#include "max_serdes.h"

#define MAX_MIPI_FMT(_dt, _bpp)	\
{				\
	.dt = (_dt),		\
	.bpp = (_bpp),		\
}

static const struct max_mipi_format max_mipi_formats[] = {
	MAX_MIPI_FMT(MIPI_CSI2_DT_EMBEDDED_8B, 8),
	MAX_MIPI_FMT(MIPI_CSI2_DT_YUV422_8B, 16),
	MAX_MIPI_FMT(MIPI_CSI2_DT_YUV422_10B, 20),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RGB565, 16),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RGB666, 18),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RGB888, 24),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RAW8, 8),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RAW10, 10),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RAW12, 12),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RAW14, 14),
	MAX_MIPI_FMT(MIPI_CSI2_DT_RAW16, 16),
};

const struct max_mipi_format *max_mipi_format_by_dt(u8 dt)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(max_mipi_formats); i++)
		if (max_mipi_formats[i].dt == dt)
			return &max_mipi_formats[i];

	return NULL;
}
EXPORT_SYMBOL_GPL(max_mipi_format_by_dt);

int max_get_fd_stream_entry(struct v4l2_subdev *sd, u32 pad, u32 stream,
			    struct v4l2_mbus_frame_desc_entry *entry)
{
	struct v4l2_mbus_frame_desc fd;
	unsigned int i;
	int ret;

	ret = v4l2_subdev_call(sd, pad, get_frame_desc, pad, &fd);
	if (ret)
		return ret;

	if (fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2)
		return -EOPNOTSUPP;

	for (i = 0; i < fd.num_entries; i++) {
		if (fd.entry[i].stream == stream) {
			*entry = fd.entry[i];
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL(max_get_fd_stream_entry);

int max_get_bpps(struct max_source *sources, u32 source_sink_pad_offset,
		 u32 *bpps, const struct v4l2_subdev_krouting *routing,
		 u32 pad, u64 streams_mask)
{
	struct v4l2_subdev_route *route;
	int ret;

	*bpps = 0;

	for_each_active_route(routing, route) {
		struct v4l2_mbus_frame_desc_entry entry;
		const struct max_mipi_format *format;
		struct max_source *source;

		if (route->sink_pad == pad) {
			if (!(BIT_ULL(route->sink_stream) & streams_mask))
				continue;
		} else if (route->source_pad == pad) {
			if (!(BIT_ULL(route->source_stream) & streams_mask))
				continue;
		} else {
			continue;
		}

		source = &sources[route->sink_pad + source_sink_pad_offset];
		if (!source)
			continue;

		ret = max_get_fd_stream_entry(source->sd, source->pad,
					      route->sink_stream, &entry);
		if (ret)
			return ret;

		format = max_mipi_format_by_dt(entry.bus.csi2.dt);
		if (!format)
			continue;

		*bpps |= BIT(format->bpp);
	}

	return 0;
}
EXPORT_SYMBOL(max_get_bpps);

int max_process_bpps(struct device *dev, u32 bpps, u32 allowed_double_bpps,
		     unsigned int *doubled_bpp)
{
	unsigned int min_bpp;
	unsigned int max_bpp;
	bool doubled = false;

	if (!bpps)
		return 0;

	*doubled_bpp = 0;

	/*
	 * Hardware can double bpps 8, 10, 12, and it can pad bpps < 16
	 * to another bpp <= 16:
	 * Hardware can only stream a single constant bpp up to 24.
	 *
	 * From these features and limitations, the following rules
	 * can be deduced:
	 *
	 * A bpp of 8 can always be doubled if present.
	 * A bpp of 10 can be doubled only if there are no other bpps or the
	 * only other bpp is 20.
	 * A bpp of 12 can be doubled only if there are no other bpps or the
	 * only other bpp is 24.
	 * Bpps <= 16 cannot coexist with bpps > 16.
	 * Bpps <= 16 need to be padded to the biggest bpp.
	 */

	min_bpp = __ffs(bpps);
	max_bpp = __fls(bpps);

	if (min_bpp == 8) {
		doubled = true;
	} else if (min_bpp == 10 || min_bpp == 12) {
		u32 bpp_or_double = BIT(min_bpp) | BIT(min_bpp * 2);
		u32 other_bpps = bpps & ~bpp_or_double;

		if (!other_bpps)
			doubled = true;
	}

	if (doubled && (allowed_double_bpps & BIT(min_bpp))) {
		*doubled_bpp = min_bpp;
		bpps &= ~BIT(min_bpp);
		bpps |= BIT(min_bpp * 2);
	}

	min_bpp = __ffs(bpps);
	max_bpp = __fls(bpps);

	if (max_bpp > 24) {
		dev_err(dev, "Cannot stream bpps > 24\n");
		return -EINVAL;
	}

	if (min_bpp <= 16 && max_bpp > 16) {
		dev_err(dev, "Cannot stream bpps <= 16 with bpps > 16\n");
		return -EINVAL;
	}

	if (max_bpp > 16 && min_bpp != max_bpp) {
		dev_err(dev, "Cannot stream multiple bpps when one is > 16\n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(max_process_bpps);

int max_xlate_enable_disable_streams(struct max_source *sources,
				     u32 source_sink_pad_offset,
				     const struct v4l2_subdev_state *state,
				     u32 pad, u64 updated_streams_mask,
				     u32 sink_pad_start, u32 num_sink_pads,
				     bool enable)
{
	u32 failed_sink_pad;
	int ret;
	u32 i;

	for (i = sink_pad_start; i < sink_pad_start + num_sink_pads; i++) {
		u64 matched_streams_mask = updated_streams_mask;
		u64 updated_sink_streams_mask;
		struct max_source *source;

		updated_sink_streams_mask =
			v4l2_subdev_state_xlate_streams(state, pad, i,
							&matched_streams_mask);
		if (!updated_sink_streams_mask)
			continue;

		source = &sources[i + source_sink_pad_offset];
		if (!source)
			continue;

		if (enable)
			ret = v4l2_subdev_enable_streams(source->sd, source->pad,
							 updated_sink_streams_mask);
		else
			ret = v4l2_subdev_disable_streams(source->sd, source->pad,
							  updated_sink_streams_mask);
		if (ret) {
			failed_sink_pad = i;
			goto err;
		}
	}

	return 0;

err:
	for (i = sink_pad_start; i < failed_sink_pad; i++) {
		u64 matched_streams_mask = updated_streams_mask;
		u64 updated_sink_streams_mask;
		struct max_source *source;

		updated_sink_streams_mask =
			v4l2_subdev_state_xlate_streams(state, pad, i,
							&matched_streams_mask);
		if (!updated_sink_streams_mask)
			continue;

		source = &sources[i + source_sink_pad_offset];
		if (!source)
			continue;

		if (!enable)
			v4l2_subdev_enable_streams(source->sd, source->pad,
						   updated_sink_streams_mask);
		else
			v4l2_subdev_disable_streams(source->sd, source->pad,
						    updated_sink_streams_mask);
	}

	return ret;
}
EXPORT_SYMBOL(max_xlate_enable_disable_streams);

int max_get_streams_masks(struct device *dev,
			  const struct v4l2_subdev_state *state,
			  u32 pad, u64 updated_streams_mask,
			  u32 num_pads, u32 sink_pad_start,
			  u32 num_sink_pads, u64 *old_streams_masks,
			  u64 **new_streams_masks, bool enable)
{
	u64 *streams_masks;
	unsigned int i;

	streams_masks = devm_kcalloc(dev, num_pads, sizeof(*streams_masks), GFP_KERNEL);
	if (!streams_masks)
		return -ENOMEM;

	for (i = 0; i < num_pads; i++)
		streams_masks[i] = old_streams_masks[i];

	for (i = sink_pad_start; i < sink_pad_start + num_sink_pads; i++) {
		u64 matched_streams_mask = updated_streams_mask;
		u64 updated_sink_streams_mask;

		updated_sink_streams_mask =
			v4l2_subdev_state_xlate_streams(state, pad, i,
							&matched_streams_mask);
		if (!updated_sink_streams_mask)
			continue;

		if (enable)
			streams_masks[i] |= updated_sink_streams_mask;
		else
			streams_masks[i] &= ~updated_sink_streams_mask;
	}

	if (enable)
		streams_masks[pad] |= updated_streams_mask;
	else
		streams_masks[pad] &= ~updated_streams_mask;

	*new_streams_masks = streams_masks;

	return 0;
}
EXPORT_SYMBOL(max_get_streams_masks);

MODULE_DESCRIPTION("Maxim GMSL2 Serializer/Deserializer Driver");
MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_LICENSE("GPL");
