/*
 * gw5300.c - gw5300 ISP driver
 *
 * Copyright (c) 2024, Analog Devices Inc.  All rights reserved.
 * 
 * Based on:
 * tier4-gw5300.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>
#include <linux/i2c.h>
#include <linux/videodev2.h>
#include <linux/kconfig.h>
#include <linux/module.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define GW5300_SINK_PAD 1
#define GW5300_SOURCE_PAD 0

struct gw5300 {
	struct i2c_client *i2c_client;
	struct mutex lock;

	struct v4l2_subdev subdev;
	struct v4l2_subdev *sensor;
	struct media_pad pads[2];

	struct v4l2_async_notifier notifier;
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_fract frame_interval;

	struct clk *clk;
	struct gpio_desc *reset;

	bool external_trigger;
	bool streaming;
};

#define NO_ERROR 0
#define MS_TO_LINE_UNIT 80

#define V4L2_CID_LDC (V4L2_CID_USER_BASE | 0x1001)
#define V4L2_CID_FSYNC (V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_INT_TIME_MIN (V4L2_CID_USER_BASE | 0x1003)
#define V4L2_CID_INT_TIME_MAX (V4L2_CID_USER_BASE | 0x1004)

static const struct gw5300_mode {
	bool ext_trigger;
	u32 fps;
	const u8 reg[18];
} gw5300_modes[] = {
	{
		.ext_trigger = false,
		.fps = 30,
		.reg = { 0x33, 0x47, 0x0B, 0x00, 0x00, 0x00, 0x12, 0x00, 0x80,
			 0x03, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x6A },
	},
	{
		.ext_trigger = true,
		.fps = 30,
		.reg = { 0x33, 0x47, 0x0B, 0x00, 0x00, 0x00, 0x12, 0x00, 0x80,
			 0x03, 0x00, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00, 0x6F },
	},
	{ 
		.ext_trigger = false,
		.fps = 20,
		.reg = { 0x33, 0x47, 0x0B, 0x00, 0x00, 0x00, 0x12, 0x00, 0x80, 0x03,
			 0x00, 0x00, 0x00, 0x5A, 0x00, 0x00, 0x00, 0x74 } },
	{
		.ext_trigger = true,
		.fps = 20,
		.reg = { 0x33, 0x47, 0x0B, 0x00, 0x00, 0x00, 0x12, 0x00, 0x80,
			 0x03, 0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x79 },
	},
	{
		.ext_trigger = false,
		.fps = 10,
		.reg = { 0x33, 0x47, 0x0B, 0x00, 0x00, 0x00, 0x12, 0x00, 0x80,
			 0x03, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x38 },
	},
	{
		.ext_trigger = true,
		.fps = 10,
		.reg = { 0x33, 0x47, 0x0B, 0x00, 0x00, 0x00, 0x12, 0x00, 0x80,
			 0x03, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x42 },
	}
};

static inline struct gw5300 *to_gw5300(struct v4l2_subdev *sd)
{
	return container_of(sd, struct gw5300, subdev);
}

static int gw5300_send_and_recv_msg(struct gw5300 *priv, const u8 *wdata,
				    int wdata_size, u8 *rdata, int rdata_size)
{
	struct i2c_client *client = priv->i2c_client;
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = 0; // I2C Write
	msg[0].len = wdata_size;
	msg[0].buf = (u8 *)wdata;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD; // I2C Read
	msg[1].len = rdata_size;
	msg[1].buf = rdata;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret <= 0) {
		dev_err(&client->dev,
			"[%s] : i2c_transfer send message failed. %d: slave addr = 0x%x\n",
			__func__, ret, msg[0].addr);
	}

	return ret;
}

static uint8_t gw5300_calc_checksum(const uint8_t *data, size_t size)
{
	uint8_t result = 0;
	size_t i = 0;
	for (i = 0; i < size; i++) {
		result += data[i];
	}
	return result;
}

int gw5300_set_ae_integration_time_min(struct gw5300 *priv,
				       u16 min_integration_time)
{
	u8 buf[6];

	u8 cmd_integration_min[20] = { 0x33, 0x47, 0x0d, 0x00, 0x00, 0x00, 0x55,
				       0x00, 0x80, 0x05, 0x00, 0x21, 0x00, 0x01,
				       0x00, 0x02, 0x00, 0x70, 0x03, 0x00 };

	uint32_t min_line = (min_integration_time * MS_TO_LINE_UNIT) / 1000;

	cmd_integration_min[17] = min_line & 0xFF;
	cmd_integration_min[18] = (min_line >> 8) & 0xFF;

	cmd_integration_min[sizeof(cmd_integration_min) - 1] =
		gw5300_calc_checksum(cmd_integration_min,
				     sizeof(cmd_integration_min) - 1);

	return gw5300_send_and_recv_msg(priv, cmd_integration_min,
					sizeof(cmd_integration_min), buf,
					sizeof(buf));
}

int gw5300_set_ae_integration_time_max(struct gw5300 *priv,
				       u32 max_integration_time)
{
	u8 buf[6];

	u8 cmd_integration_max[22] = { 0x33, 0x47, 0x0f, 0x00, 0x00, 0x00,
				       0x55, 0x00, 0x80, 0x05, 0x00, 0x15,
				       0x00, 0x01, 0x00, 0x04, 0x00, 0x70,
				       0x03, 0x00, 0x00, 0x00 };

	uint32_t max_line = (max_integration_time * MS_TO_LINE_UNIT) / 1000;

	cmd_integration_max[17] = max_line & 0xFF;
	cmd_integration_max[18] = (max_line >> 8) & 0xFF;
	cmd_integration_max[19] = (max_line >> 16) & 0xFF;
	cmd_integration_max[20] = (max_line >> 24) & 0xFF;
	cmd_integration_max[sizeof(cmd_integration_max) - 1] =
		gw5300_calc_checksum(cmd_integration_max,
				     sizeof(cmd_integration_max) - 1);

	return gw5300_send_and_recv_msg(priv, cmd_integration_max,
					sizeof(cmd_integration_max), buf,
					sizeof(buf));
}

int gw5300_set_distortion_correction(struct gw5300 *priv, bool val)
{
	u8 read_buf[6];
	u8 cmd_dwp_on[] = { 0x33, 0x47, 0x06, 0x00, 0x00, 0x00, 0x4d,
			    0x00, 0x80, 0x04, 0x00, 0x01, 0x52 };
	u8 cmd_dwp_off[] = { 0x33, 0x47, 0x03, 0x00, 0x00,
			     0x00, 0x45, 0x00, 0x80, 0x42 };

	if (val)
		return gw5300_send_and_recv_msg(priv, cmd_dwp_on,
						sizeof(cmd_dwp_on), read_buf,
						sizeof(read_buf));
	else
		return gw5300_send_and_recv_msg(priv, cmd_dwp_off,
						sizeof(cmd_dwp_off), read_buf,
						sizeof(read_buf));
}

int gw5300_set_exposure(struct gw5300 *priv, u16 val)
{
	u8 read_buf[6];
	u8 integration_time[] = { 0x33, 0x47, 0x0d, 0x00, 0x00, 0x00, 0x55,
				  0x00, 0x80, 0x05, 0x00, 0x08, 0x00, 0x01,
				  0x00, 0x02, 0x00, 0xe8, 0x03, 0x57 };

	integration_time[17] = val & 0xFF;
	integration_time[18] = (val >> 8) & 0xFF;
	integration_time[sizeof(integration_time) - 1] = gw5300_calc_checksum(
		integration_time, sizeof(integration_time) - 1);

	return gw5300_send_and_recv_msg(priv, integration_time,
					sizeof(integration_time), read_buf,
					sizeof(read_buf));
}

int gw5300_set_mode(struct gw5300 *priv)
{
	u8 read_buf[6];
	const struct gw5300_mode *mode = gw5300_modes;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(gw5300_modes); i++) {
		if (mode[i].fps == priv->frame_interval.denominator) {
			if (mode[i].ext_trigger == priv->external_trigger) {
				ret = gw5300_send_and_recv_msg(
					priv, mode[i].reg, sizeof(mode[i].reg),
					read_buf, sizeof(read_buf));
				return ret;
			} else {
				continue;
			}
		} else {
			continue;
		}
	}

	return -EINVAL;
}

int gw5300_set_auto_exposure(struct gw5300 *priv, bool val)
{
	u8 read_buf[6];
	u8 cmd_ae_on[] = { 0x33, 0x47, 0x0c, 0x00, 0x00, 0x00, 0x55,
			   0x00, 0x80, 0x05, 0x00, 0x07, 0x00, 0x01,
			   0x00, 0x01, 0x00, 0x00, 0x69 };
	u8 cmd_ae_off[] = { 0x33, 0x47, 0x0c, 0x00, 0x00, 0x00, 0x55,
			    0x00, 0x80, 0x05, 0x00, 0x07, 0x00, 0x01,
			    0x00, 0x01, 0x00, 0x01, 0x6a };
	int ret = 0;

	if (val) {
		ret += gw5300_send_and_recv_msg(priv, cmd_ae_on,
						sizeof(cmd_ae_on), read_buf,
						sizeof(read_buf));
	} else {
		ret += gw5300_send_and_recv_msg(priv, cmd_ae_off,
						sizeof(cmd_ae_off), read_buf,
						sizeof(read_buf));
	}
	return ret;
}

static int gw5300_power_on(struct gw5300 *priv)
{
	int ret;

	ret = gpiod_direction_output(priv->reset, 0);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0)
		goto err_reset;

	fsleep(1000);

	return 0;

err_reset:
	gpiod_direction_output(priv->reset, 1);
	return ret;
}

static void gw5300_power_off(struct gw5300 *priv)
{
	clk_disable_unprepare(priv->clk);
	gpiod_direction_output(priv->reset, 1);
}

static int gw5300_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct gw5300 *priv = to_gw5300(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (priv->streaming == enable)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		priv->streaming = false;
		ret = 0;
		goto unlock;
	}

	ret = __v4l2_ctrl_handler_setup(&priv->ctrls);
	if (ret < 0) {
		dev_err(&priv->i2c_client->dev,
			"%s : Failed to setup controls\n", __func__);
		goto unlock;
	}

	ret = gw5300_set_mode(priv);
	if (ret < 0) {
		dev_err(&priv->i2c_client->dev, "%s : Failed to set mode\n",
			__func__);
		goto unlock;
	}

	priv->streaming = true;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int gw5300_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gw5300 *priv = to_gw5300(sd);

	if (fi->pad != 0)
		return -EINVAL;

	fi->interval = priv->frame_interval;

	return 0;
}

static int gw5300_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct gw5300 *priv = to_gw5300(sd);
	int ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	priv->frame_interval = fi->interval;

	return ret;
}

static int gw5300_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct gw5300 *priv = to_gw5300(sd);

	if (code->pad != GW5300_SOURCE_PAD)
		return -EINVAL;

	if (priv->sensor)
		v4l2_subdev_call(priv->sensor, pad, enum_mbus_code, sd_state,
				 code);

	return 0;
}

static int gw5300_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	if (fmt->pad != GW5300_SOURCE_PAD)
		return -EINVAL;

	fmt->format = *v4l2_subdev_get_pad_format(sd, sd_state, fmt->pad);

	return 0;
}

static int gw5300_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *fmt)
{
	struct gw5300 *priv = to_gw5300(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	dev_dbg(priv->subdev.dev, "%s for %d", __func__, fmt->pad);

	format = v4l2_subdev_get_pad_format(sd, sd_state, fmt->pad);

	if (fmt->pad != GW5300_SOURCE_PAD)
		return -EINVAL;

	mutex_lock(&priv->lock);

	if (priv->streaming) {
		ret = -EBUSY;
		goto error;
	}

	format->code = MEDIA_BUS_FMT_UYVY8_1X16;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

error:
	mutex_unlock(&priv->lock);
	return ret;
}

static int
gw5300_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;

	if (fie->pad != GW5300_SOURCE_PAD)
		return -EINVAL;
	if (fie->index >= 3)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = (fie->index + 1) * 10;

	fie->interval = tpf;
	return 0;
}

static const struct v4l2_subdev_video_ops gw5300_video_ops = {
	.s_stream = gw5300_s_stream,
	.g_frame_interval = gw5300_g_frame_interval,
	.s_frame_interval = gw5300_s_frame_interval,
};

static const struct v4l2_subdev_pad_ops gw5300_pad_ops = {
	.enum_mbus_code = gw5300_enum_mbus_code,
	.enum_frame_interval = gw5300_enum_frame_interval,
	.get_fmt = gw5300_get_fmt,
	.set_fmt = gw5300_set_fmt,
};

static const struct v4l2_subdev_ops gw5300_subdev_ops = {
	.video = &gw5300_video_ops,
	.pad = &gw5300_pad_ops,
};

static const struct media_entity_operations mipid02_subdev_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

/* -----------------------------------------------------------------------------
 * Controls
 */

static int gw5300_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct gw5300 *priv = container_of(ctrl->handler, struct gw5300, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	bool enable;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&priv->subdev);
	format = v4l2_subdev_get_pad_format(&priv->subdev, state, 0);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		gw5300_set_exposure(priv, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		enable = ctrl->val == V4L2_EXPOSURE_AUTO;
		gw5300_set_auto_exposure(priv, enable);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		break;

	case V4L2_CID_INT_TIME_MIN:
		gw5300_set_ae_integration_time_min(priv, ctrl->val);
		break;

	case V4L2_CID_INT_TIME_MAX:
		gw5300_set_ae_integration_time_max(priv, ctrl->val);
		break;

	case V4L2_CID_LDC:
		enable = ctrl->val != 0;
		gw5300_set_distortion_correction(priv, enable);
		break;

	case V4L2_CID_FSYNC:
		priv->external_trigger = ctrl->val == 1;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops gw5300_ctrl_ops = {
	.s_ctrl = gw5300_s_ctrl,
};

static const char *const gw5300_ctrl_fsync_options[] = {
	"Internal",
	"External",
};

static const struct v4l2_ctrl_config gw5300_ctrl_fsync = {
	.ops = &gw5300_ctrl_ops,
	.id = V4L2_CID_FSYNC,
	.name = "FSYNC source",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = ARRAY_SIZE(gw5300_ctrl_fsync_options) - 1,
	.def = 0,
	.qmenu = gw5300_ctrl_fsync_options,
};

static const struct v4l2_ctrl_config gw5300_ctrl_lens_distors_correct = {
	.ops = &gw5300_ctrl_ops,
	.id = V4L2_CID_LDC,
	.name = "Lens Distorsion Correction",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.min = 0,
	.max = 1,
	.step = 1,
	.def = 1,
};

static const struct v4l2_ctrl_config gw5300_ctrl_int_time_min = {
	.ops = &gw5300_ctrl_ops,
	.id = V4L2_CID_INT_TIME_MIN,
	.name = "Integration Time min",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 3000,
	.step = 1,
	.def = 1000,
};

static const struct v4l2_ctrl_config gw5300_ctrl_int_time_max = {
	.ops = &gw5300_ctrl_ops,
	.id = V4L2_CID_INT_TIME_MAX,
	.name = "Integration Time max",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = 30000,
	.step = 1,
	.def = 1000,
};

static int gw5300_ctrls_init(struct gw5300 *priv)
{
	struct device *dev = &priv->i2c_client->dev;

	v4l2_ctrl_handler_init(&priv->ctrls, 9);

	v4l2_ctrl_new_std(&priv->ctrls, &gw5300_ctrl_ops, V4L2_CID_EXPOSURE, 0,
			  3000, 1, 1000);
	v4l2_ctrl_new_std(&priv->ctrls, &gw5300_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 48, 1, 6);
	v4l2_ctrl_new_std_menu(&priv->ctrls, &gw5300_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0,
			       V4L2_EXPOSURE_AUTO);
	/*
	 * The sensor calculates the MIPI timings internally to achieve a bit
	 * rate between 1122 and 1198 Mbps. The exact value is unfortunately not
	 * reported, at least according to the documentation. Report a nominal
	 * rate of 1188 Mbps as that is used by the datasheet in multiple
	 * examples.
	 */
	v4l2_ctrl_new_std(&priv->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  1122000000 / 16, 1198000000 / 16, 1, 1188000000 / 16);

	v4l2_ctrl_new_custom(&priv->ctrls, &gw5300_ctrl_fsync, NULL);
	v4l2_ctrl_new_custom(&priv->ctrls, &gw5300_ctrl_lens_distors_correct,
			     NULL);
	v4l2_ctrl_new_custom(&priv->ctrls, &gw5300_ctrl_int_time_min, NULL);
	v4l2_ctrl_new_custom(&priv->ctrls, &gw5300_ctrl_int_time_max, NULL);

	if (priv->ctrls.error) {
		dev_err(dev, "failed to add controls (%d)\n",
			priv->ctrls.error);
		v4l2_ctrl_handler_free(&priv->ctrls);
		return priv->ctrls.error;
	}

	priv->subdev.ctrl_handler = &priv->ctrls;

	return 0;
}

static int gw5300_subdev_init(struct gw5300 *priv)
{
	struct i2c_client *client = priv->i2c_client;
	struct device *dev = &client->dev;
	int ret;

	dev_dbg(&client->dev, "[%s] : Init ISP subdev\n", __func__);

	v4l2_i2c_subdev_init(&priv->subdev, client, &gw5300_subdev_ops);

	ret = gw5300_ctrls_init(priv);
	if (ret < 0)
		return ret;

	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	priv->pads[GW5300_SINK_PAD].flags = MEDIA_PAD_FL_SINK;
	priv->pads[GW5300_SOURCE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	priv->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	ret = media_entity_pads_init(&priv->subdev.entity, 2, priv->pads);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&priv->ctrls);
		return ret;
	}

	v4l2_subdev_init_finalize(&priv->subdev);
	if (ret)
		goto err_entity_cleanup;

	ret = v4l2_async_register_subdev(&priv->subdev);
	if (ret) {
		dev_err_probe(dev, ret, "v4l2_async_register_subdev error\n");
		goto err_subdev_cleanup;
	}

	dev_dbg(&client->dev, "[%s] : Done init subdev\n", __func__);

	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&priv->subdev);
err_entity_cleanup:
	media_entity_cleanup(&priv->subdev.entity);

	return ret;
}

static int gw5300_probe(struct i2c_client *client)
{
	struct gw5300 *priv;
	int ret;

	dev_info(&client->dev, "[%s] : Probing GW5300 ISP\n", __func__);

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	priv->i2c_client = client;

	mutex_init(&priv->lock);

	priv->reset =
		devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset))
		return dev_err_probe(&client->dev, PTR_ERR(priv->reset),
				     "failed to get reset GPIO\n");

	priv->clk = devm_clk_get(&client->dev, "isp_clk");
	if (IS_ERR(priv->clk))
		return dev_err_probe(&client->dev, PTR_ERR(priv->clk),
				     "failed to get ISP clock\n");

	ret = gw5300_power_on(priv);
	if (ret < 0)
		return ret;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(&client->dev);
	pm_runtime_get_noresume(&client->dev);
	pm_runtime_enable(&client->dev);

	ret = gw5300_subdev_init(priv);
	if (ret < 0) {
		dev_err_probe(&client->dev, ret,
			      "v4l2 subdev notifier register failed\n");
		goto err_pm;
	}

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(&client->dev, 1000);
	pm_runtime_use_autosuspend(&client->dev);
	pm_runtime_put_autosuspend(&client->dev);

	return 0;

err_pm:
	pm_runtime_disable(&client->dev);
	pm_runtime_put_noidle(&client->dev);
	gw5300_power_off(priv);
	return ret;
}

static void gw5300_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct gw5300 *priv = to_gw5300(subdev);

	v4l2_async_unregister_subdev(subdev);

	media_entity_cleanup(&priv->subdev.entity);
	v4l2_ctrl_handler_free(&priv->ctrls);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		gw5300_power_off(priv);
	pm_runtime_set_suspended(&client->dev);

	mutex_destroy(&priv->lock);
}

const struct of_device_id gw5300_of_match[] = {
	{ .compatible = "geo,gw5300", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, gw5300_of_match);

static struct i2c_driver gw5300_i2c_driver = {
    .driver = {
		.of_match_table = gw5300_of_match,
        .name = "gw5300",
    },
    .probe_new = gw5300_probe,
    .remove = gw5300_remove,
};
module_i2c_driver(gw5300_i2c_driver);

MODULE_DESCRIPTION("GW5300 V4L2 ISP driver");
MODULE_AUTHOR("Bogdan Togorean");
MODULE_LICENSE("GPL v2");