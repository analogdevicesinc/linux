// SPDX-License-Identifier: GPL-2.0
/*
 * Generic Camera Driver
 *
 * A stub V4L2 sensor driver that registers a sub-device in the media
 * pipeline without programming any real sensor hardware.
 *
 * The device is instantiated from the device tree as an I2C client. No
 * transfer is ever issued on the bus: the I2C binding exists so that the
 * sub-device is enumerated at the right place in the topology (typically
 * behind a GMSL serializer's I2C address translator) and so that it is
 * bound and unbound along with the rest of the link.
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/property.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define CAM_DEFAULT_DATAFMT	MEDIA_BUS_FMT_UYVY8_1X16
#define CAM_DEFAULT_WIDTH	1920
#define CAM_DEFAULT_HEIGHT	1080
#define CAM_DEFAULT_LANES	4

#define CAM_MAX_CSI2_DT		0x3f
#define CAM_MAX_STREAMS		V4L2_MBUS_CSI2_MAX_VC_IDS

/*
 * Pad 0 is the CSI-2 source carrying every stream. Routing needs a sink
 * for each route, so each stream also gets an internal sink pad: internal
 * pads are not connectable from userspace and exist only to give the
 * stream an origin inside the entity.
 */
#define CAM_PAD_SOURCE		0
#define CAM_PAD_SINK(n)		(1 + (n))
#define CAM_NUM_PADS(n)		(1 + (n))

#define CAM_MIN_DIM		1
#define CAM_MAX_DIM		65535

struct cam_resolution {
	u32 width;
	u32 height;
};

static const struct cam_resolution cam_res[] = {
	{640,	480},
	{1024,	768},
	{1280,	720},
	{1920,	1080},
	{1920,	1280}
};

struct cam_info {
	u32 code;
	u32 width;
	u32 height;
};

struct cam {
	struct v4l2_subdev sd;
	struct media_pad pads[CAM_NUM_PADS(CAM_MAX_STREAMS)];
	struct v4l2_ctrl_handler ctrl_handler;
	const struct cam_info *info;
	u32 num_lanes;
	s64 link_freq;

	u32 width;
	u32 height;

	struct cam_resolution res[ARRAY_SIZE(cam_res) + 1];
	unsigned int num_res;

	u8 dt;
	bool dt_override;


	u8 vcs[CAM_MAX_STREAMS];
	unsigned int num_streams;

	u64 enabled_streams;

	struct gpio_desc *reset_gpio;
	struct clk *clk;
};

static inline struct cam *sd_to_cam(struct v4l2_subdev *sd)
{
	return container_of(sd, struct cam, sd);
}

static const struct {
	u32 code;
	u8 dt;
} cam_mbus_formats[] = {
	{ MEDIA_BUS_FMT_SRGGB12_1X12,	MIPI_CSI2_DT_RAW12 },
	{ MEDIA_BUS_FMT_SGRBG12_1X12,	MIPI_CSI2_DT_RAW12 },
	{ MEDIA_BUS_FMT_SGBRG12_1X12,	MIPI_CSI2_DT_RAW12 },
	{ MEDIA_BUS_FMT_SBGGR12_1X12,	MIPI_CSI2_DT_RAW12 },
	{ MEDIA_BUS_FMT_SRGGB10_1X10,	MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SGRBG10_1X10,	MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SGBRG10_1X10,	MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SBGGR10_1X10,	MIPI_CSI2_DT_RAW10 },
	{ MEDIA_BUS_FMT_SRGGB8_1X8,	MIPI_CSI2_DT_RAW8 },
	{ MEDIA_BUS_FMT_YUYV8_1X16,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_YVYU8_1X16,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_UYVY8_1X16,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_VYUY8_1X16,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_RGB888_1X24,	MIPI_CSI2_DT_RGB888 },
	{ MEDIA_BUS_FMT_YUYV8_2X8,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_YVYU8_2X8,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_UYVY8_2X8,	MIPI_CSI2_DT_YUV422_8B },
	{ MEDIA_BUS_FMT_VYUY8_2X8,	MIPI_CSI2_DT_YUV422_8B },
};

static u8 cam_code_to_dt(struct cam *priv, u32 code)
{
	unsigned int i;

	if (priv->dt_override)
		return priv->dt;

	for (i = 0; i < ARRAY_SIZE(cam_mbus_formats); i++)
		if (cam_mbus_formats[i].code == code)
			return cam_mbus_formats[i].dt;

	return MIPI_CSI2_DT_YUV422_8B;
}

static int cam_power_on(struct cam *priv)
{
	int ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret)
		return ret;

	gpiod_set_value_cansleep(priv->reset_gpio, 0);

	fsleep(1000);

	return 0;
}

static void cam_power_off(struct cam *priv)
{
	gpiod_set_value_cansleep(priv->reset_gpio, 1);
	clk_disable_unprepare(priv->clk);
}

static int cam_enable_streams(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state, u32 pad,
			u64 streams_mask)
{
	struct cam *priv = sd_to_cam(sd);
	int ret;

	dev_dbg(sd->dev, "%s: pad %u streams 0x%llx\n", __func__, pad,
		streams_mask);

	if (!priv->enabled_streams) {
		ret = cam_power_on(priv);
		if (ret)
			return ret;
	}

	priv->enabled_streams |= streams_mask;

	return 0;
}

static int cam_disable_streams(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state, u32 pad,
			u64 streams_mask)
{
	struct cam *priv = sd_to_cam(sd);

	dev_dbg(sd->dev, "%s: pad %u streams 0x%llx\n", __func__, pad,
		streams_mask);

	priv->enabled_streams &= ~streams_mask;

	if (!priv->enabled_streams)
		cam_power_off(priv);

	return 0;
}

static int cam_set_fmt(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_format *fmt)
{
	struct cam *priv = sd_to_cam(sd);
	struct v4l2_mbus_framefmt *format;
	u32 i = 0, sum = U32_MAX, index = 0;

	format = v4l2_subdev_state_get_format(state, fmt->pad, fmt->stream);
	if (!format)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(cam_mbus_formats); ++i) {

		if (fmt->format.code == cam_mbus_formats[i].code)
			break;
	}

	if (i == ARRAY_SIZE(cam_mbus_formats))
		fmt->format.code = CAM_DEFAULT_DATAFMT;

	for (i = 0; i < priv->num_res; ++i) {

		u32 diff = abs_diff(fmt->format.width, priv->res[i].width) +
			   abs_diff(fmt->format.height, priv->res[i].height);

		if (diff < sum) {
			sum = diff;
			index = i;
		}
	}

	fmt->format.width = priv->res[index].width;
	fmt->format.height = priv->res[index].height;

	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	*format = fmt->format;
	format = v4l2_subdev_state_get_opposite_stream_format(state, fmt->pad,
							      fmt->stream);
	if (!format)
		return -EINVAL;

	*format = fmt->format;

	return 0;
}

static int cam_enum_mbus_code(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(cam_mbus_formats))
		return -EINVAL;

	code->code = cam_mbus_formats[code->index].code;

	return 0;
}

static int cam_enum_frame_size(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			struct v4l2_subdev_frame_size_enum *fse)
{
	struct cam *priv = sd_to_cam(sd);

	if (fse->index >= priv->num_res)
		return -EINVAL;

	fse->min_width = priv->res[fse->index].width;
	fse->min_height = priv->res[fse->index].height;
	fse->max_width = priv->res[fse->index].width;
	fse->max_height = priv->res[fse->index].height;

	return 0;
}

static int cam_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
			struct v4l2_mbus_config *cfg)
{
	struct cam *priv = sd_to_cam(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->bus.mipi_csi2.num_data_lanes = priv->num_lanes;
	cfg->bus.mipi_csi2.flags = V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;

	/*
	 * __v4l2_get_link_freq_pad() prefers this over the V4L2_CID_LINK_FREQ
	 * control, so receivers that query the pad get the rate straight from
	 * the endpoint's link-frequencies property.
	 */
	cfg->link_freq = priv->link_freq;

	return 0;
}

static int cam_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
			struct v4l2_mbus_frame_desc *fd)
{
	struct cam *priv = sd_to_cam(sd);
	struct v4l2_subdev_route *route;
	struct v4l2_subdev_state *state;

	if (pad != CAM_PAD_SOURCE)
		return -EINVAL;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry *entry;
		struct v4l2_mbus_framefmt *format;

		if (route->source_pad != CAM_PAD_SOURCE)
			continue;

		if (route->source_stream >= priv->num_streams)
			continue;

		if (fd->num_entries == ARRAY_SIZE(fd->entry))
			break;

		format = v4l2_subdev_state_get_format(state, route->source_pad,
						      route->source_stream);
		if (!format)
			continue;

		entry = &fd->entry[fd->num_entries++];

		entry->stream = route->source_stream;
		entry->pixelcode = format->code;
		entry->bus.csi2.vc = priv->vcs[route->source_stream];
		entry->bus.csi2.dt = cam_code_to_dt(priv, format->code);
	}

	v4l2_subdev_unlock_state(state);

	return 0;
}

static int cam_set_routing(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state,
			enum v4l2_subdev_format_whence which,
			struct v4l2_subdev_krouting *routing)
{
	struct cam *priv = sd_to_cam(sd);
	unsigned int i;
	int ret;

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->enabled_streams)
		return -EBUSY;

	for (i = 0; i < routing->num_routes; i++) {
		const struct v4l2_subdev_route *route = &routing->routes[i];

		if (route->source_stream >= priv->num_streams ||
		    route->sink_pad != CAM_PAD_SINK(route->source_stream) ||
		    route->sink_stream != 0)
			return -EINVAL;
	}

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	return v4l2_subdev_set_routing(sd, state, routing);
}

static const struct v4l2_subdev_pad_ops cam_subdev_pad_ops = {
	.set_fmt = cam_set_fmt,
	.get_fmt = v4l2_subdev_get_fmt,
	.enum_mbus_code = cam_enum_mbus_code,
	.enum_frame_size = cam_enum_frame_size,
	.set_routing = cam_set_routing,
	.enable_streams = cam_enable_streams,
	.disable_streams = cam_disable_streams,
	.get_frame_desc = cam_get_frame_desc,
	.get_mbus_config = cam_get_mbus_config,
};

static const struct v4l2_subdev_ops cam_subdev_ops = {
	.pad = &cam_subdev_pad_ops,
};

static int cam_init_state(struct v4l2_subdev *sd,
			struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[CAM_MAX_STREAMS];
	struct cam *priv = sd_to_cam(sd);
	struct v4l2_subdev_krouting routing = {
		.len_routes = ARRAY_SIZE(routes),
		.num_routes = priv->num_streams,
		.routes = routes,
	};
	unsigned int i;
	int ret;

	memset(routes, 0, sizeof(routes));

	for (i = 0; i < priv->num_streams; i++) {
		routes[i].sink_pad = CAM_PAD_SINK(i);
		routes[i].sink_stream = 0;
		routes[i].source_pad = CAM_PAD_SOURCE;
		routes[i].source_stream = i;
		routes[i].flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
	}

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret)
		return ret;

	for (i = 0; i < priv->num_streams; i++) {
		struct v4l2_subdev_format fmt = {
			.which = V4L2_SUBDEV_FORMAT_TRY,
			.pad = CAM_PAD_SOURCE,
			.stream = i,
			.format = {
				.code = priv->info->code,
				.width = priv->width,
				.height = priv->height,
			},
		};

		ret = cam_set_fmt(sd, state, &fmt);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct v4l2_subdev_internal_ops cam_subdev_internal_ops = {
	.init_state = cam_init_state,
};

static const struct media_entity_operations cam_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int cam_parse_endpoint(struct cam *priv)
{
	struct device *dev = priv->sd.dev;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *endpoint;
	unsigned int i, seen = 0;
	int ret;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), 0, 0, 0);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL,
				     "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "could not parse endpoint\n");

	priv->num_lanes = ep_cfg.bus.mipi_csi2.num_data_lanes;
	if (!priv->num_lanes)
		priv->num_lanes = CAM_DEFAULT_LANES;

	if (ep_cfg.nr_of_link_frequencies)
		priv->link_freq = ep_cfg.link_frequencies[0];

	priv->num_streams = ep_cfg.bus.mipi_csi2.num_vc_ids;
	for (i = 0; i < priv->num_streams; i++) {
		u32 vc = ep_cfg.bus.mipi_csi2.vc_ids[i];

		if (vc >= CAM_MAX_STREAMS) {
			v4l2_fwnode_endpoint_free(&ep_cfg);
			return dev_err_probe(dev, -EINVAL,
					     "'vc-ids' entry %u is VC %u, max %u\n",
					     i, vc, CAM_MAX_STREAMS - 1);
		}

		if (seen & BIT(vc)) {
			v4l2_fwnode_endpoint_free(&ep_cfg);
			return dev_err_probe(dev, -EINVAL,
					     "'vc-ids' repeats VC %u\n", vc);
		}

		seen |= BIT(vc);
		priv->vcs[i] = vc;
	}

	if (!priv->num_streams) {
		priv->num_streams = 1;
		priv->vcs[0] = 0;
	}

	v4l2_fwnode_endpoint_free(&ep_cfg);

	return 0;
}

static int cam_parse_properties(struct cam *priv)
{
	struct device *dev = priv->sd.dev;
	unsigned int i;
	u32 dt;
	int ret;

	priv->width = priv->info->width;
	priv->height = priv->info->height;

	ret = device_property_read_u32(dev, "adi,default-width", &priv->width);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret,
				     "invalid 'adi,default-width'\n");

	ret = device_property_read_u32(dev, "adi,default-height",
				       &priv->height);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret,
				     "invalid 'adi,default-height'\n");

	if (priv->width < CAM_MIN_DIM || priv->width > CAM_MAX_DIM ||
	    priv->height < CAM_MIN_DIM || priv->height > CAM_MAX_DIM)
		return dev_err_probe(dev, -EINVAL,
				     "frame size %ux%u out of range [%u..%u]\n",
				     priv->width, priv->height,
				     CAM_MIN_DIM, CAM_MAX_DIM);

	memcpy(priv->res, cam_res, sizeof(cam_res));
	priv->num_res = ARRAY_SIZE(cam_res);

	for (i = 0; i < ARRAY_SIZE(cam_res); i++)
		if (cam_res[i].width == priv->width &&
		    cam_res[i].height == priv->height)
			break;

	if (i == ARRAY_SIZE(cam_res)) {
		priv->res[priv->num_res].width = priv->width;
		priv->res[priv->num_res].height = priv->height;
		priv->num_res++;
	}

	ret = device_property_read_u32(dev, "adi,csi2-data-type", &dt);
	if (!ret) {
		if (dt > CAM_MAX_CSI2_DT)
			return dev_err_probe(dev, -EINVAL,
					     "'adi,csi2-data-type' 0x%02x exceeds 0x%02x\n",
					     dt, CAM_MAX_CSI2_DT);

		priv->dt = dt;
		priv->dt_override = true;
	} else if (ret != -EINVAL) {
		return dev_err_probe(dev, ret,
				     "invalid 'adi,csi2-data-type'\n");
	}

	return 0;
}

static int cam_get_resources(struct cam *priv)
{
	struct device *dev = priv->sd.dev;

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset GPIO\n");

	priv->clk = devm_clk_get_optional(dev, "xclk");
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "failed to get clock\n");

	return 0;
}

static int cam_init_controls(struct cam *priv)
{
	struct v4l2_fwnode_device_properties props;
	struct device *dev = priv->sd.dev;
	int ret;

	ret = v4l2_ctrl_handler_init(&priv->ctrl_handler, 3);
	if (ret)
		return ret;

	if (priv->link_freq) {
		struct v4l2_ctrl *ctrl;

		ctrl = v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL,
					      V4L2_CID_LINK_FREQ, 0, 0,
					      &priv->link_freq);
		if (ctrl)
			ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret)
		goto err_free_handler;

	ret = v4l2_ctrl_new_fwnode_properties(&priv->ctrl_handler, NULL,
					      &props);
	if (ret)
		goto err_free_handler;

	if (priv->ctrl_handler.error) {
		ret = priv->ctrl_handler.error;
		goto err_free_handler;
	}

	priv->sd.ctrl_handler = &priv->ctrl_handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return dev_err_probe(dev, ret, "failed to init controls\n");
}

static int cam_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	char vcs[2 * CAM_MAX_STREAMS];
	unsigned int i, len;
	struct cam *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->info = i2c_get_match_data(client);
	if (!priv->info)
		return dev_err_probe(dev, -ENODEV, "no match data\n");

	v4l2_i2c_subdev_init(&priv->sd, client, &cam_subdev_ops);

	ret = cam_parse_endpoint(priv);
	if (ret)
		return ret;

	ret = cam_parse_properties(priv);
	if (ret)
		return ret;

	ret = cam_get_resources(priv);
	if (ret)
		return ret;

	ret = cam_init_controls(priv);
	if (ret)
		return ret;

	priv->sd.internal_ops = &cam_subdev_internal_ops;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	priv->sd.entity.ops = &cam_media_ops;
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	priv->pads[CAM_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	for (i = 0; i < priv->num_streams; i++)
		priv->pads[CAM_PAD_SINK(i)].flags = MEDIA_PAD_FL_SINK |
						    MEDIA_PAD_FL_INTERNAL;

	ret = media_entity_pads_init(&priv->sd.entity,
				     CAM_NUM_PADS(priv->num_streams),
				     priv->pads);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto err_free_ctrls;
	}

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto err_entity_cleanup;
	}

	ret = v4l2_async_register_subdev_sensor(&priv->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register sub-device\n");
		goto err_subdev_cleanup;
	}

	/* Two chars per stream: one digit and its separator or the NUL. */
	for (i = 0, len = 0; i < priv->num_streams; i++)
		len += scnprintf(vcs + len, sizeof(vcs) - len, "%s%u",
				 i ? "," : "", priv->vcs[i]);

	dev_info(dev,
		 "%ux%u, %u data lanes, link frequency %lld Hz, %u stream(s) on VC %s, DT 0x%02x, clock %lu Hz, reset GPIO %s\n",
		 priv->width, priv->height, priv->num_lanes, priv->link_freq,
		 priv->num_streams, vcs,
		 cam_code_to_dt(priv, priv->info->code),
		 clk_get_rate(priv->clk),
		 priv->reset_gpio ? "yes" : "no");

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	media_entity_cleanup(&priv->sd.entity);
err_free_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static void cam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct cam *priv = sd_to_cam(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
}

static const struct cam_info cam_info_generic = {
	.code = CAM_DEFAULT_DATAFMT,
	.width = CAM_DEFAULT_WIDTH,
	.height = CAM_DEFAULT_HEIGHT,
};

static const struct of_device_id cam_of_match[] = {
	{ .compatible = "adi,dummy-cam", .data = &cam_info_generic },
	{ }
};
MODULE_DEVICE_TABLE(of, cam_of_match);

static const struct i2c_device_id cam_id[] = {
	{ "dummy-cam", (kernel_ulong_t)&cam_info_generic },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cam_id);

static struct i2c_driver cam_i2c_driver = {
	.driver = {
		.name = "dummy-cam",
		.of_match_table = cam_of_match,
	},
	.probe = cam_probe,
	.remove = cam_remove,
	.id_table = cam_id,
};

module_i2c_driver(cam_i2c_driver);

MODULE_DESCRIPTION("Generic V4L2 camera driver");
MODULE_AUTHOR("Analog Devices Inc.");
MODULE_LICENSE("GPL v2");
