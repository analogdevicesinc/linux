// SPDX-License-Identifier: GPL-2.0+
/*
 * MX95MBCAM camera module driver. The module contains an OX03C10 camera and a Maxim MAX96717
 * GMSL2 serializer chip. It needs to be paired with a GMSL2 compatible deserializer.
 *
 * Copyright 2024 NXP
 *
 */

#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/i2c-mux.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "max96717.h"
#include "ox03c10.h"

struct mx95mbcam_priv {
	struct i2c_client *client;
	struct max96717 *ser;

	struct ox03c10 *sensor;
	unsigned int sensor_reset_pin;
	unsigned int sensor_clock_pin;
	struct regmap *sensor_rmap;

	struct v4l2_subdev sd;

	struct media_pad pad;
};

static int mx95mbcam_parse_dt(struct mx95mbcam_priv *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = of_property_read_u32(dev->of_node, "nxp,camera_sensor_reset_pin",
				   &priv->sensor_reset_pin);
	if (ret < 0) {
		dev_err(dev, "No camera sensor reset pin declared in DT.\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "nxp,camera_sensor_clock_pin",
				   &priv->sensor_clock_pin);
	if (ret < 0)
		dev_err(dev, "No camera sensor clock pin declared in DT.\n");

	return ret;
}

static int mx95mbcam_streaming_start(struct v4l2_subdev *sd)
{
	struct mx95mbcam_priv *priv = container_of(sd, struct mx95mbcam_priv, sd);
	struct max96717 *ser = priv->ser;
	int ret;

	/* enable CSI and video pipe on the serializer */
	ret = max96717_csi_port_en(ser, true);
	ret |= max96717_video_pipe_en(ser, true);
	ret |= max96717_video_pipe_tx_en(ser, true);

	/* start sensor streaming */
	ret |= ox03c10_streaming_start(priv->sensor, true);

	return ret ? -EIO : 0;
}

static int mx95mbcam_streaming_stop(struct v4l2_subdev *sd)
{
	struct mx95mbcam_priv *priv = container_of(sd, struct mx95mbcam_priv, sd);
	struct max96717 *ser = priv->ser;
	int ret;

	/* stop sensor streaming */
	ret = ox03c10_streaming_start(priv->sensor, false);

	/* disable CSI and video pipe on the serializer */
	ret |= max96717_csi_port_en(ser, false);
	ret |= max96717_video_pipe_en(ser, false);
	ret |= max96717_video_pipe_tx_en(ser, false);

	return ret ? -EIO : 0;
}

static int mx95mbcam_s_stream(struct v4l2_subdev *sd, int enable)
{
	if (enable)
		return mx95mbcam_streaming_start(sd);

	return mx95mbcam_streaming_stop(sd);
}

static int mx95mbcam_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR16_1X16;

	return 0;
}

static int mx95mbcam_enum_frame_size(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *state,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	struct ox03c10_mode *mode = ox03c10_get_mode(fse->index);

	if (PTR_ERR(mode) == -EINVAL)
		return -EINVAL;

	fse->min_width = mode->width;
	fse->max_width = fse->min_width;
	fse->min_height = mode->height;
	fse->max_height = fse->min_height;

	return 0;
}

static int mx95mbcam_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *sd_state,
			     struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	if (format->pad)
		return -EINVAL;

	mf->width		= OX03C10_PIXEL_ARRAY_WIDTH;
	mf->height		= OX03C10_PIXEL_ARRAY_HEIGHT;
	mf->code		= MEDIA_BUS_FMT_SBGGR16_1X16;
	mf->colorspace		= V4L2_COLORSPACE_RAW;
	mf->field		= V4L2_FIELD_NONE;
	mf->ycbcr_enc		= V4L2_YCBCR_ENC_601;
	mf->quantization	= V4L2_QUANTIZATION_FULL_RANGE;
	mf->xfer_func		= V4L2_XFER_FUNC_NONE;

	return 0;
}

static int mx95mbcam_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
			     struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	struct v4l2_subdev_format format;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	format.pad = 0;
	mx95mbcam_get_fmt(sd, state, &format);

	fd->entry[0].pixelcode = format.format.code;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MAX96717_DT_RAW16;

	v4l2_subdev_unlock_state(state);

	return 0;
}

static int mx95mbcam_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				   struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = OX03C10_NATIVE_WIDTH;
		sel->r.height = OX03C10_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = OX03C10_PIXEL_ARRAY_TOP;
		sel->r.left = OX03C10_PIXEL_ARRAY_LEFT;
		sel->r.width = OX03C10_PIXEL_ARRAY_WIDTH;
		sel->r.height = OX03C10_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int mx95mbcam_ctrl_type_op_validate(const struct v4l2_ctrl *ctrl, union v4l2_ctrl_ptr ptr)
{
	return true;
}

static const struct v4l2_ctrl_type_ops mx95mbcam_ctrl_type_ops = {
	.init		= v4l2_ctrl_type_op_init,
	.validate	= mx95mbcam_ctrl_type_op_validate,
};

static const struct v4l2_subdev_video_ops mx95mbcam_video_ops = {
	.s_stream	= mx95mbcam_s_stream,
};

static const struct v4l2_subdev_pad_ops mx95mbcam_subdev_pad_ops = {
	.enum_mbus_code		= mx95mbcam_enum_mbus_code,
	.enum_frame_size	= mx95mbcam_enum_frame_size,
	.get_fmt		= mx95mbcam_get_fmt,
	.set_fmt		= mx95mbcam_get_fmt,
	.get_frame_desc		= mx95mbcam_get_frame_desc,
	.get_selection		= mx95mbcam_get_selection,
};

static const struct v4l2_subdev_ops mx95mbcam_subdev_ops = {
	.video		= &mx95mbcam_video_ops,
	.pad		= &mx95mbcam_subdev_pad_ops,
};


static int mx95mbcam_v4l2_init_controls(struct mx95mbcam_priv *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = ox03c10_v4l2_controls_init(priv->sensor);
	if (ret < 0)
		dev_err(dev, "Could not initialize the ox03c10 sensor controls\n");

	return ret;
}

static int mx95mbcam_v4l2_init(struct mx95mbcam_priv *priv)
{
	int ret;

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &mx95mbcam_subdev_ops);

	ret = mx95mbcam_v4l2_init_controls(priv);
	if (ret < 0)
		return ret;

	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->sd.ctrl_handler = ox03c10_ctrl_handler_get(priv->sensor);
	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0)
		goto error_free_ctrl;

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret < 0)
		goto error_clean_entity;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto error_clean_entity;

	return 0;

error_clean_entity:
	media_entity_cleanup(&priv->sd.entity);

error_free_ctrl:
	ox03c10_ctrl_handler_free(priv->sensor);
	return ret;
}

static int mx95mbcam_config_csi_lanes(struct mx95mbcam_priv *priv)
{
	int serializer_to_sensor_csi_lane_map[4] = {0, 1, 2, 3}; /* map lanes 1 to 1 */

	return max96717_map_csi_lanes(priv->ser, serializer_to_sensor_csi_lane_map,
				      ARRAY_SIZE(serializer_to_sensor_csi_lane_map));
}

static int mx95mbcam_config_lanes_polarity(struct mx95mbcam_priv *priv)
{
	int pol[5] = {
		LANE_POL_NORMAL, /* clock lane */
		LANE_POL_NORMAL, /* lane 0 */
		LANE_POL_NORMAL, /* lane 1 */
		LANE_POL_NORMAL, /* lane 2 */
		LANE_POL_NORMAL, /* lane 3*/
	};

	return max96717_set_lanes_polarity(priv->ser, pol, ARRAY_SIZE(pol));
}

static int mx95mbcam_init(struct mx95mbcam_priv *priv)
{
	int ret;
	u8 data_types[2] = {
		MAX96717_DT_RAW16,
		MAX96717_DT_EMBEDDED,
	};

	ret = max96717_tunnel_mode_en(priv->ser, false);
	ret |= max96717_set_lanes_no(priv->ser, 4);
	ret |= mx95mbcam_config_csi_lanes(priv);
	ret |= mx95mbcam_config_lanes_polarity(priv);
	ret |= max96717_data_type_filter(priv->ser, data_types, ARRAY_SIZE(data_types));
	ret |= max96717_double_mode_en(priv->ser);
	ret |= max96717_soft_bpp_override(priv->ser, 16); /* RAW16 bpp */
	ret |= max96717_vc_filter(priv->ser, BIT(0)); /* process only VC0 */
	ret |= max96717_stream_id_set(priv->ser, 0);

	return ret ? -EIO : 0;
}

static int mx95mbcam_probe(struct i2c_client *client)
{
	struct mx95mbcam_priv *priv;
	struct device *dev = &client->dev;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->ser = max96717_init(client);
	if (IS_ERR(priv->ser))
		return PTR_ERR(priv->ser);

	if (!max96717_is_dev_id_valid(priv->ser))
		return -ENODEV;

	ret = mx95mbcam_parse_dt(priv);
	if (ret < 0)
		return ret;

	ret = max96717_hw_init(priv->ser, priv->sensor_reset_pin, priv->sensor_clock_pin);
	if (ret) {
		dev_err(dev, "Could not initialize the MAX96717 chip.\n");
		return ret;
	}

	priv->sensor = ox03c10_init_with_dummy_client(client, true);
	if (IS_ERR(priv->sensor))
		return PTR_ERR(priv->sensor);

	ret = mx95mbcam_init(priv);
	if (ret < 0)
		return ret;

	return mx95mbcam_v4l2_init(priv);
}

static void mx95mbcam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct mx95mbcam_priv *priv = container_of(subdev, struct mx95mbcam_priv, sd);

	ox03c10_ctrl_handler_free(priv->sensor);
	v4l2_async_unregister_subdev(&priv->sd);
}

static const struct of_device_id mx95mbcam_dt_ids[] = {
	{ .compatible = "nxp,mx95mbcam" },
	{},
};
MODULE_DEVICE_TABLE(of, mx95mbcam_dt_ids);

static struct i2c_driver mx95mbcam_i2c_driver = {
	.driver	= {
		.name		= "mx95mbcam",
		.of_match_table	= of_match_ptr(mx95mbcam_dt_ids),
	},
	.probe		= mx95mbcam_probe,
	.remove		= mx95mbcam_remove,
};

module_i2c_driver(mx95mbcam_i2c_driver);

MODULE_DESCRIPTION("MX95MBCAM OX03C10 camera module over GMSL2 link driver");
MODULE_AUTHOR("Laurentiu Palcu");
MODULE_LICENSE("GPL");
