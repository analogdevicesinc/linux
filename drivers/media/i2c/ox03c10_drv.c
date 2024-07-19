// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 NXP
 *
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#include "ox03c10.h"

#define DRIVER_NAME		"ox03c10_drv"

struct ox03c10_priv {
	struct device *dev;
	struct i2c_client *client;

	struct ox03c10 *sensor;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *fsin;
	struct clk *clock;

	struct media_pad pad;
	struct v4l2_subdev sd;
	struct v4l2_mbus_framefmt formats;
};

static inline struct ox03c10_priv *to_ox03c10_priv(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ox03c10_priv, sd);
}

static int ox03c10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0 || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR16_1X16;
	return 0;
}

static int ox03c10_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct ox03c10_mode *mode = ox03c10_get_mode(fse->index);

	if (fse->code != MEDIA_BUS_FMT_SBGGR16_1X16)
		return -EINVAL;

	if (PTR_ERR(mode) == -EINVAL)
		return -EINVAL;

	fse->min_width = mode->width;
	fse->max_width = fse->min_width;
	fse->min_height = mode->height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ox03c10_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	if (pad != 0 || !fd)
		return -EINVAL;

	memset(fd, 0x0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->entry[0].flags = 0;
	fd->entry[0].pixelcode = MEDIA_BUS_FMT_SBGGR16_1X16;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_RAW16;
	fd->num_entries = 1;

	return 0;
}

static int ox03c10_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
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

static int ox03c10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct ox03c10_priv *priv = to_ox03c10_priv(sd);
	const struct v4l2_mbus_framefmt *format = &priv->formats;

	if (fmt->pad)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static const struct v4l2_subdev_pad_ops ox03c10_subdev_pad_ops = {
	.enum_mbus_code = ox03c10_enum_mbus_code,
	.enum_frame_size = ox03c10_enum_frame_size,
	.get_frame_desc = ox03c10_get_frame_desc,
	.get_selection = ox03c10_get_selection,
	.get_fmt = ox03c10_get_fmt,
	.set_fmt = ox03c10_get_fmt,
};

static int ox03c10_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ox03c10_priv *priv = to_ox03c10_priv(sd);

	return ox03c10_streaming_start(priv->sensor, enable);
}

static const struct v4l2_subdev_video_ops ox03c10_subdev_video_ops = {
	.s_stream = ox03c10_s_stream,
};

static const struct v4l2_subdev_ops ox03c10_subdev_ops = {
	.pad = &ox03c10_subdev_pad_ops,
	.video = &ox03c10_subdev_video_ops,
};

static int ox03c10_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct ox03c10_priv *priv = to_ox03c10_priv(sd);
	struct v4l2_mbus_framefmt *fmt = &priv->formats;

	fmt->code = MEDIA_BUS_FMT_SBGGR16_1X16;
	fmt->width = OX03C10_PIXEL_ARRAY_WIDTH;
	fmt->height = OX03C10_PIXEL_ARRAY_HEIGHT;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_RAW);
	fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static const struct v4l2_subdev_internal_ops ox03c10_subdev_internal_ops = {
	.init_state = ox03c10_init_state,
};

static const struct media_entity_operations ox03c10_media_ops = {
	.link_validate = v4l2_subdev_link_validate
};

static int ox03c10_v4l2_init_controls(struct ox03c10_priv *priv)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = ox03c10_v4l2_controls_init(priv->sensor);
	if (ret < 0)
		dev_err(dev, "Could not initialize the ox03c10 sensor controls\n");

	return ret;
}

static int ox03c10_v4l2_init(struct ox03c10_priv *priv)
{
	struct v4l2_subdev *sd = &priv->sd;
	int ret;

	sd->dev = priv->dev;
	v4l2_i2c_subdev_init(sd, priv->client, &ox03c10_subdev_ops);

	ret = ox03c10_v4l2_init_controls(priv);
	if (ret)
		return ret;

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;
	sd->internal_ops = &ox03c10_subdev_internal_ops;
	sd->ctrl_handler = ox03c10_ctrl_handler_get(priv->sensor);
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &ox03c10_media_ops;

	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &priv->pad);
	if (ret) {
		ox03c10_ctrl_handler_free(priv->sensor);
		dev_err(priv->dev, "Failed to init entity pads\n");
		return ret;
	}

	return ox03c10_init_state(sd, NULL);
}

static int ox03c10_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ox03c10_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->client = client;

	priv->clock = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clock)) {
		ret = PTR_ERR(priv->clock);
		return dev_err_probe(dev, ret, "Failed to get clock\n");
	}

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						      GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio)) {
		ret = PTR_ERR(priv->reset_gpio);
		return dev_err_probe(dev, ret, "Can't get reset GPIO\n");
	}

	priv->fsin = devm_gpiod_get_optional(dev, "fsin", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->fsin)) {
		ret = PTR_ERR(priv->fsin);
		return dev_err_probe(dev, ret, "Can't get fsin GPIO\n");
	}

	ret = clk_prepare_enable(priv->clock);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clock\n");

	priv->sensor = ox03c10_init_with_dummy_client(client, false);
	if (IS_ERR(priv->sensor))
		return PTR_ERR(priv->sensor);

	ret = ox03c10_v4l2_init(priv);
	if (ret)
		return ret;

	ret = v4l2_async_register_subdev_sensor(&priv->sd);
	if (ret < 0) {
		dev_err(dev, "Could not register v4l2 subdev\n");
		goto free_entity;
	}

	return 0;

free_entity:
	media_entity_cleanup(&priv->sd.entity);
	return ret;
}

static void ox03c10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox03c10_priv *priv = to_ox03c10_priv(sd);

	ox03c10_ctrl_handler_free(priv->sensor);
	v4l2_async_unregister_subdev(&priv->sd);
}

static const struct of_device_id ox03c10_of_id_table[] = {
	{ .compatible = "ovti,ox03c10" },
	{ }
};
MODULE_DEVICE_TABLE(of, ox03c10_of_id_table);

static struct i2c_driver ox03c10_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= ox03c10_of_id_table,
	},
	.probe		= ox03c10_probe,
	.remove		= ox03c10_remove,
};

module_i2c_driver(ox03c10_i2c_driver);
MODULE_DESCRIPTION("OmniVision OX03C10 sensor driver");
MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_LICENSE("GPL");
