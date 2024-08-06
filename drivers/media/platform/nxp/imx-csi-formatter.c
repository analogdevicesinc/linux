// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2024 NXP
 *
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

/* CSI Pixel Formaterr Registers Define */

#define CSI_VCx_INTERLACED_LINE_CNT(x)		(0x00 + (x) * 0x04)
#define INTERLACED_ODD_LINE_CNT_SET(x)		FIELD_PREP(GENMASK(13, 0), (x))
#define INTERLACED_EVEN_LINE_CNT_SET(x)		FIELD_PREP(GENMASK(29, 16), (x))

#define CSI_VC_INTERLACED_CTRL			0x20
#define CSI_VCx_INTERLACED_MODE(vc, mode)	FIELD_PREP(GENMASK(2 * (vc) + 1, 2 * (vc)), (mode))

#define CSI_VC_INTERLACED_ERR			0x24
#define CSI_VC_ERR_MASK				GENMASK(7, 0)
#define CSI_VC_ERR(vc)				BIT((vc))

#define CSI_VC_YUV420_FIRST_LINE_EVEN		0x28
#define YUV420_FIRST_LINE_EVEN(vc)		BIT((vc))

#define CSI_RAW32_CTRL				0x30
#define CSI_VCX_RAW32_MODE(vc)			BIT((vc))
#define CSI_VCX_RAW32_SWAP_MODE(vc)		BIT((vc) + 8)

#define STREAM_FENCING_CTRL			0x34
#define CSI_VCx_STREAM_FENCING(vc)		BIT((vc))
#define CSI_VCx_STREAM_FENCING_RST(vc)		BIT((vc) + 8)

#define STREAM_FENCING_STS			0x38
#define STREAM_FENCING_STS_MASK			GENMASK(7, 0)

#define CSI_VCX_NON_PIXEL_DATA_TYPE(vc)		(0x40 + (vc) * 0x04)

#define CSI_VCX_PIXEL_DATA_CTRL(vc)		(0x60 + (vc) * 0x04)
#define NEW_VC(x)				FIELD_PREP(GENMASK(3, 1), x)
#define REROUTE_VC_ENABLE			BIT(0)

#define CSI_VCx_ROUTE_PIXEL_DATA_TYPE(vc)	(0x80 + (vc) * 0x04)

#define CSI_VCx_NON_PIXEL_DATA_CTRL(vc)		(0xa0 + (vc) * 0x04)

#define CSI_VCx_PIXEL_DATA_TYPE(vc)		(0xc0 + (vc) * 0x04)

#define CSI_VCx_PIXEL_DATA_TYPE_ERR(vc)		(0xe0 + (vc) * 0x04)

#define CSI_FORMATTER_PAD_SINK			0
#define CSI_FORMATTER_PAD_SOURCE		1
#define CSI_FORMATTER_PAD_NUM			2

#define CSI_FORMATTER_DEF_MBUS_CODE		MEDIA_BUS_FMT_UYVY8_1X16
#define CSI_FORMATTER_DEF_PIX_WIDTH		1920U
#define CSI_FORMATTER_DEF_PIX_HEIGHT		1080U
#define CSI_FORMATTER_MAX_PIX_WIDTH		0xffff
#define CSI_FORMATTER_MAX_PIX_HEIGHT		0xffff

#define CSI_FORMATTER_DRV_NAME			"csi-pixel-formatter"
#define CSI_FORMATTER_VC_MAX			8

struct formatter_pix_format {
	u32 code;
	u32 data_type;
};

struct csi_formatter {
	struct device *dev;
	struct regmap *regs;
	struct clk *clk;

	struct v4l2_subdev sd;
	struct v4l2_subdev *csi_sd;
	struct v4l2_async_notifier notifier;
	struct media_pad pads[CSI_FORMATTER_PAD_NUM];
	struct v4l2_mbus_framefmt format[CSI_FORMATTER_PAD_NUM];
	const struct formatter_pix_format *fmt;
	u16 remote_pad;
	u64 enabled_streams;

	u32 reg_offset;
};

struct dt_index {
	u8 dtype;
	u8 index;
};

static const struct dt_index formatter_dt_to_index_map[] = {
	{ .dtype = MIPI_CSI2_DT_YUV420_8B,        .index = 0 },
	{ .dtype = MIPI_CSI2_DT_YUV420_8B_LEGACY, .index = 2 },
	{ .dtype = MIPI_CSI2_DT_YUV422_8B,        .index = 6 },
	{ .dtype = MIPI_CSI2_DT_RGB444,		  .index = 8 },
	{ .dtype = MIPI_CSI2_DT_RGB555,           .index = 9 },
	{ .dtype = MIPI_CSI2_DT_RGB565,           .index = 10 },
	{ .dtype = MIPI_CSI2_DT_RGB666,           .index = 11 },
	{ .dtype = MIPI_CSI2_DT_RGB888,           .index = 12 },
	{ .dtype = MIPI_CSI2_DT_RAW6,             .index = 16 },
	{ .dtype = MIPI_CSI2_DT_RAW7,             .index = 17 },
	{ .dtype = MIPI_CSI2_DT_RAW8,             .index = 18 },
	{ .dtype = MIPI_CSI2_DT_RAW10,            .index = 19 },
	{ .dtype = MIPI_CSI2_DT_RAW12,            .index = 20 },
	{ .dtype = MIPI_CSI2_DT_RAW14,            .index = 21 },
	{ .dtype = MIPI_CSI2_DT_RAW16,            .index = 22 },
};

static const struct formatter_pix_format formats[] = {
	/* YUV formats */
	{
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.data_type = MIPI_CSI2_DT_YUV422_8B,
	},
	/* RGB formats */
	{
		.code = MEDIA_BUS_FMT_RGB565_1X16,
		.data_type = MIPI_CSI2_DT_RGB565,
	}, {
		.code = MEDIA_BUS_FMT_BGR888_1X24,
		.data_type = MIPI_CSI2_DT_RGB888,
	},
	/* RAW (Bayer and greyscale) formats. */
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_Y8_1X8,
		.data_type = MIPI_CSI2_DT_RAW8,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.data_type = MIPI_CSI2_DT_RAW10,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_Y12_1X12,
		.data_type = MIPI_CSI2_DT_RAW12,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.data_type = MIPI_CSI2_DT_RAW14,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.data_type = MIPI_CSI2_DT_RAW16,
	}
};

static const struct v4l2_mbus_framefmt formatter_default_fmt = {
	.code = CSI_FORMATTER_DEF_MBUS_CODE,
	.width = CSI_FORMATTER_DEF_PIX_WIDTH,
	.height = CSI_FORMATTER_DEF_PIX_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SMPTE170M,
	.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SMPTE170M),
	.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SMPTE170M),
	.quantization = V4L2_QUANTIZATION_LIM_RANGE,
};

static const struct formatter_pix_format *find_csi_format(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (code == formats[i].code)
			return &formats[i];
	return &formats[0];
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static inline struct csi_formatter *sd_to_formatter(struct v4l2_subdev *sdev)
{
	return container_of(sdev, struct csi_formatter, sd);
}

static int __formatter_subdev_set_routing(struct v4l2_subdev *sd,
					  struct v4l2_subdev_state *state,
					  struct v4l2_subdev_krouting *routing)
{
	int ret;

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing_with_fmt(sd, state, routing,
						&formatter_default_fmt);
}
static int formatter_subdev_init_state(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = CSI_FORMATTER_PAD_SINK,
			.sink_stream = 0,
			.source_pad = CSI_FORMATTER_PAD_SOURCE,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return __formatter_subdev_set_routing(sd, sd_state, &routing);;
}

static int formatter_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *sd_state,
					   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad == CSI_FORMATTER_PAD_SOURCE) {
		struct v4l2_mbus_framefmt *fmt;

		if (code->index > 0)
			return -EINVAL;

		fmt = v4l2_subdev_state_get_format(sd_state, code->pad,
						   code->stream);
		code->code = fmt->code;
		return 0;
	}

	if (code->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	code->code = formats[code->index].code;

	return 0;
}

static int formatter_subdev_set_fmt(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *sdformat)
{
	struct csi_formatter *formatter = sd_to_formatter(sd);
	struct formatter_pix_format const *format;
	struct v4l2_mbus_framefmt *fmt;

	if (sdformat->pad == CSI_FORMATTER_PAD_SOURCE)
		return v4l2_subdev_get_fmt(sd, sd_state, sdformat);

	/*
	 * Validate the media bus code and clamp and align the size.
	 *
	 * The total number of bits per line must be a multiple of 8. We thus
	 * need to align the width for formats that are not multiples of 8
	 * bits.
	 */
	format = find_csi_format(sdformat->format.code);

	v4l_bound_align_image(&sdformat->format.width, 1,
			      CSI_FORMATTER_MAX_PIX_WIDTH, 2,
			      &sdformat->format.height, 1,
			      CSI_FORMATTER_MAX_PIX_HEIGHT, 0, 0);

	fmt = v4l2_subdev_state_get_format(sd_state, sdformat->pad,
					   sdformat->stream);
	*fmt = sdformat->format;

	/* Set default code if user set an invalid value */
	fmt->code = format->code;

	/* Propagate the format from sink stream to source stream */
	fmt = v4l2_subdev_state_get_opposite_stream_format(sd_state, sdformat->pad,
							   sdformat->stream);
	if (!fmt)
		return -EINVAL;

	*fmt = sdformat->format;

	/* Store the CSIS format descriptor for active formats. */
	if (sdformat->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		formatter->fmt = format;

	return 0;
}

static int formatter_subdev_get_frame_desc(struct v4l2_subdev *sd,
					   unsigned int pad,
					   struct v4l2_mbus_frame_desc *fd)
{
	struct csi_formatter *formatter = sd_to_formatter(sd);
	struct v4l2_mbus_frame_desc csi_fd;
	struct v4l2_subdev_route *route;
	struct v4l2_subdev_state *state;
	int ret;

	if (pad != CSI_FORMATTER_PAD_SOURCE)
		return -EINVAL;

	ret = v4l2_subdev_call(formatter->csi_sd, pad, get_frame_desc,
			       formatter->remote_pad, &csi_fd);
	if (ret)
		return ret;

	if (csi_fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2) {
		dev_err(formatter->dev,
			"Frame descriptor does not describe CSI-2 link\n");
		return -EINVAL;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry *entry = NULL;
		unsigned int i;

		if (route->source_pad != pad)
			continue;

		for (i = 0; i < csi_fd.num_entries; ++i) {
			if (csi_fd.entry[i].stream == route->sink_stream) {
				entry = &csi_fd.entry[i];
				break;
			}
		}

		if (!entry) {
			dev_err(formatter->dev,
				"Failed to find stream from source frames desc\n");
			ret = -EPIPE;
			goto out_unlock;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;
		fd->entry[fd->num_entries].flags = entry->flags;
		fd->entry[fd->num_entries].length = entry->length;
		fd->entry[fd->num_entries].pixelcode = entry->pixelcode;
		fd->entry[fd->num_entries].bus.csi2.vc = entry->bus.csi2.vc;
		fd->entry[fd->num_entries].bus.csi2.dt = entry->bus.csi2.dt;

		fd->num_entries++;
	}

out_unlock:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int formatter_subdev_set_routing(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					enum v4l2_subdev_format_whence which,
					struct v4l2_subdev_krouting *routing)
{
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    media_entity_is_streaming(&sd->entity))
		return -EBUSY;

	return __formatter_subdev_set_routing(sd, state, routing);
}

static inline void formatter_write(struct csi_formatter *formatter,
				   unsigned int reg, unsigned int value)
{
	u32 offset = formatter->reg_offset;

	regmap_write(formatter->regs, reg + offset, value);
}

static u8 get_index_by_dt(u8 data_type)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formatter_dt_to_index_map); ++i)
		if (data_type == formatter_dt_to_index_map[i].dtype)
			break;

	if (i == ARRAY_SIZE(formatter_dt_to_index_map)) {
		return formatter_dt_to_index_map[0].index;
	}

	return formatter_dt_to_index_map[i].index;
}

static u8 get_vc(struct csi_formatter *formatter, unsigned int stream)
{
	struct v4l2_mbus_frame_desc source_fd;
	struct v4l2_mbus_frame_desc_entry *entry = NULL;
	unsigned int i;
	int ret;

	/*
	 * Return virtual channel 0 as default value when remote subdev
	 * don't implement .get_frame_desc subdev callback
	 */
	ret = v4l2_subdev_call(formatter->csi_sd, pad, get_frame_desc,
			       formatter->remote_pad, &source_fd);
	if (ret < 0)
		return (ret == -ENOIOCTLCMD) ? 0 : ret;

	for (i = 0; i < source_fd.num_entries; ++i) {
		if (source_fd.entry[i].stream == stream) {
			entry = &source_fd.entry[i];
			break;
		}
	}

	if (!entry) {
		dev_err(formatter->dev,
			"Can't find valid frame desc corresponding to stream %d\n", stream);
		return -EPIPE;
	}

	return entry->bus.csi2.vc;
}

static int csi_formatter_start_stream(struct csi_formatter *formatter,
				      u64 stream_mask)
{
	const struct formatter_pix_format *fmt = formatter->fmt;
	unsigned int i;
	u32 val;
	u8 vc;

	for (i = 0; i < V4L2_FRAME_DESC_ENTRY_MAX; ++i) {
		if (stream_mask & BIT(i))
			break;
	}

	if (i == V4L2_FRAME_DESC_ENTRY_MAX) {
		dev_err(formatter->dev, "Stream ID out of range\n");
		return -EINVAL;
	}

	val = BIT(get_index_by_dt(fmt->data_type));
	vc = get_vc(formatter, i);

	if (vc < 0 || vc > CSI_FORMATTER_VC_MAX) {
		dev_err(formatter->dev, "Invalid virtual channel(%d)\n", vc);
		return -EINVAL;
	}

	formatter_write(formatter, CSI_VCx_PIXEL_DATA_TYPE(vc), val);

	return 0;
}

static int formatter_subdev_enable_streams(struct v4l2_subdev *sd,
					   struct v4l2_subdev_state *state,
					   u32 pad, u64 streams_mask)
{
	struct csi_formatter *formatter = sd_to_formatter(sd);
	struct device *dev = formatter->dev;
	u64 sink_streams;
	int ret;

	if (!formatter->csi_sd) {
		dev_err(dev, "CSI controller don't link with formatter\n");
		return -EPIPE;
	}

	if (!formatter->enabled_streams) {
		ret = pm_runtime_resume_and_get(formatter->dev);
		if (ret < 0) {
			dev_err(dev, "Formatter runtime get fail\n");
			return ret;
		}
	}

	ret = csi_formatter_start_stream(formatter, streams_mask);
	if (ret)
		goto runtime_put;

	sink_streams = v4l2_subdev_state_xlate_streams(state,
						       CSI_FORMATTER_PAD_SOURCE,
						       CSI_FORMATTER_PAD_SINK,
						       &streams_mask);

	dev_dbg(dev, "remote sd: %s pad: %u, sink_stream:0x%llx\n",
		formatter->csi_sd->name, formatter->remote_pad, sink_streams);

	ret = v4l2_subdev_enable_streams(formatter->csi_sd, formatter->remote_pad,
					 sink_streams);
	if (ret)
		goto runtime_put;

	formatter->enabled_streams |= streams_mask;

	return 0;

runtime_put:
	pm_runtime_put(formatter->dev);
	return ret;
}

static int csi_formatter_stop_stream(struct csi_formatter *formatter,
				     u64 stream_mask)
{
	unsigned int i;
	u8 vc;

	for (i = 0; i < V4L2_FRAME_DESC_ENTRY_MAX; ++i) {
		if (stream_mask & BIT(i))
			break;
	}

	if (i == V4L2_FRAME_DESC_ENTRY_MAX) {
		dev_err(formatter->dev, "Stream ID out of range\n");
		return -EINVAL;
	}

	vc = get_vc(formatter, i);

	if (vc < 0 || vc > CSI_FORMATTER_VC_MAX) {
		dev_err(formatter->dev, "Invalid virtual channel(%d)\n", vc);
		return -EINVAL;
	}

	formatter_write(formatter, CSI_VCx_PIXEL_DATA_TYPE(vc), 0);

	return 0;
}

static int formatter_subdev_disable_streams(struct v4l2_subdev *sd,
					    struct v4l2_subdev_state *state,
					    u32 pad, u64 streams_mask)
{
	struct csi_formatter *formatter = sd_to_formatter(sd);
	u64 sink_streams;
	int ret;

	sink_streams = v4l2_subdev_state_xlate_streams(state,
						       CSI_FORMATTER_PAD_SOURCE,
						       CSI_FORMATTER_PAD_SINK,
						       &streams_mask);

	ret = v4l2_subdev_disable_streams(formatter->csi_sd, formatter->remote_pad,
					  sink_streams);
	if (ret)
		return ret;

	csi_formatter_stop_stream(formatter, streams_mask);

	formatter->enabled_streams &= ~streams_mask;

	if (!formatter->enabled_streams)
		pm_runtime_put(formatter->dev);

	return 0;
}

static const struct v4l2_subdev_pad_ops formatter_subdev_pad_ops = {
	.enum_mbus_code	= formatter_subdev_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = formatter_subdev_set_fmt,
	.get_frame_desc = formatter_subdev_get_frame_desc,
	.set_routing = formatter_subdev_set_routing,
	.enable_streams = formatter_subdev_enable_streams,
	.disable_streams = formatter_subdev_disable_streams,
};

static const struct v4l2_subdev_ops formatter_subdev_ops = {
	.pad   = &formatter_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops formatter_internal_ops = {
	.init_state = formatter_subdev_init_state,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

static const struct media_entity_operations formatter_entity_ops = {
	.link_validate	= v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

static int csi_formatter_subdev_init(struct csi_formatter *formatter)
{
	struct v4l2_subdev *sd = &formatter->sd;
	int ret;

	v4l2_subdev_init(sd, &formatter_subdev_ops);

	snprintf(sd->name, sizeof(sd->name), "%s", dev_name(formatter->dev));
	sd->internal_ops = &formatter_internal_ops;

	sd->owner = THIS_MODULE;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_STREAMS;
	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	sd->entity.ops = &formatter_entity_ops;
	sd->dev = formatter->dev;

	formatter->pads[CSI_FORMATTER_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	formatter->pads[CSI_FORMATTER_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, CSI_FORMATTER_PAD_NUM,
				     formatter->pads);
	if (ret) {
		dev_err(formatter->dev, "Failed to init pads\n");
		return ret;
	}

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		media_entity_cleanup(&sd->entity);

	return ret;
}

static inline struct csi_formatter *
notifier_to_formatter(struct v4l2_async_notifier *n)
{
	return container_of(n, struct csi_formatter, notifier);
}

static int csi_formatter_notify_bound(struct v4l2_async_notifier *notifier,
				      struct v4l2_subdev *sd,
				      struct v4l2_async_connection *asd)
{
	const unsigned int link_flags = MEDIA_LNK_FL_IMMUTABLE
				      | MEDIA_LNK_FL_ENABLED;
	struct csi_formatter *formatter = notifier_to_formatter(notifier);
	struct v4l2_subdev *sdev = &formatter->sd;
	struct media_pad *sink = &sdev->entity.pads[CSI_FORMATTER_PAD_SINK];
	struct media_pad *remote_pad;
	int ret;

	formatter->csi_sd = sd;


	dev_dbg(formatter->dev, "Bound subdev: %s pad\n", sd->name);

	ret = v4l2_create_fwnode_links_to_pad(sd, sink, link_flags);
	if (ret < 0)
		return ret;

	remote_pad = media_pad_remote_pad_first(sink);
	if (!remote_pad) {
		dev_err(formatter->dev, "Pipe not setup correctly\n");
		return -EPIPE;
	}
	formatter->remote_pad = remote_pad->index;

	return 0;
}

static const struct v4l2_async_notifier_operations formatter_notify_ops = {
	.bound = csi_formatter_notify_bound,
};

static int csi_formatter_async_register(struct csi_formatter *formatter)
{
	struct device *dev = formatter->dev;
	struct v4l2_async_connection *asd;
	struct fwnode_handle *ep;
	int ret;

	v4l2_async_subdev_nf_init(&formatter->notifier, &formatter->sd);

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0,
					     FWNODE_GRAPH_ENDPOINT_NEXT);
	if (!ep)
		return -ENOTCONN;

	asd = v4l2_async_nf_add_fwnode_remote(&formatter->notifier, ep,
					      struct v4l2_async_connection);
	if (IS_ERR(asd)) {
		fwnode_handle_put(ep);
		return PTR_ERR(asd);
	}

	fwnode_handle_put(ep);

	formatter->notifier.ops = &formatter_notify_ops;

	ret = v4l2_async_nf_register(&formatter->notifier);
	if (ret)
		return ret;

	return v4l2_async_register_subdev(&formatter->sd);
}

/* -----------------------------------------------------------------------------
 * Suspend/resume
 */

static int csi_formatter_system_suspend(struct device *dev)
{
	return pm_runtime_force_suspend(dev);
}

static int csi_formatter_system_resume(struct device *dev)
{
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0) {
		dev_err(dev, "force resume %s failed!\n", dev_name(dev));
		return ret;
	}

	return 0;
}

static int csi_formatter_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct csi_formatter *formatter = sd_to_formatter(sd);

	clk_disable_unprepare(formatter->clk);

	return 0;
}

static int csi_formatter_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct csi_formatter *formatter = sd_to_formatter(sd);

	return clk_prepare_enable(formatter->clk);
}

static const struct dev_pm_ops csi_formatter_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(csi_formatter_system_suspend,
				csi_formatter_system_resume)
	SET_RUNTIME_PM_OPS(csi_formatter_runtime_suspend,
			   csi_formatter_runtime_resume,
			   NULL)
};

static int csi_formatter_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct csi_formatter *formatter;
	u32 val;
	int ret;

	formatter = devm_kzalloc(dev, sizeof(*formatter), GFP_KERNEL);
	if (!formatter)
		return -ENOMEM;

	formatter->dev = dev;

	formatter->regs = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(formatter->regs)) {
		dev_err(dev, "Failed to get csi formatter regmap\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(dev->of_node, "reg", &val);
	if (ret < 0) {
		dev_err(dev, "Failed to get csi formatter reg property\n");
		return ret;
	}
	formatter->reg_offset = val;

	formatter->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(formatter->clk)) {
		dev_err(dev, "Failed to get pixel clock\n");
		return PTR_ERR(formatter->clk);
	}

	ret = csi_formatter_subdev_init(formatter);
	if (ret < 0) {
		dev_err(dev, "formatter subdev init fail\n");
		return ret;
	}

	/* Initialize formatter pixel format */
	formatter->fmt = find_csi_format(formatter_default_fmt.code);

	ret = csi_formatter_async_register(formatter);
	if (ret < 0) {
		v4l2_subdev_cleanup(&formatter->sd);
		dev_err(dev, "Async register failed\n");
		return ret;
	}

	platform_set_drvdata(pdev, &formatter->sd);

	/* Enable runtime PM. */
	pm_runtime_enable(dev);

	return 0;
}

static void csi_formatter_remove(struct platform_device *pdev)
{
	struct v4l2_subdev *sd = platform_get_drvdata(pdev);
	struct csi_formatter *formatter = sd_to_formatter(sd);

	v4l2_async_nf_unregister(&formatter->notifier);
	v4l2_async_nf_cleanup(&formatter->notifier);
	v4l2_async_unregister_subdev(&formatter->sd);

	pm_runtime_disable(&pdev->dev);

	media_entity_cleanup(&formatter->sd.entity);
	fwnode_handle_put(formatter->sd.fwnode);

	pm_runtime_set_suspended(&pdev->dev);
}

static const struct of_device_id csi_formatter_of_match[] = {
	{ .compatible = "fsl,imx95-csi-formatter" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, csi_formatter_of_match);

static struct platform_driver dwc_csi_device_driver = {
	.driver = {
		.owner          = THIS_MODULE,
		.name           = CSI_FORMATTER_DRV_NAME,
		.of_match_table = csi_formatter_of_match,
		.pm             = &csi_formatter_pm_ops,
	},
	.probe  = csi_formatter_probe,
	.remove = csi_formatter_remove,
};

module_platform_driver(dwc_csi_device_driver);

MODULE_DESCRIPTION("NXP CSI Pixel Formatter driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" CSI_FORMATTER_DRV_NAME);
MODULE_AUTHOR("NXP Semiconductor, Inc.");
