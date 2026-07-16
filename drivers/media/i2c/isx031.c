// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ISX031 CMOS Image Sensor from Sony
 *
 * Copyright 2026 Tudor Cristea <tudor.cristea@analog.com>
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define ISX031_NATIVE_WIDTH 1920
#define ISX031_NATIVE_HEIGHT 1536

struct isx031_mode {
	u32 width;
	u32 height;
	u16 h_crop_offset;
	u16 v_crop_offset;
};

static const struct isx031_mode isx031_supported_modes[] = {
	{ 1920, 1536, 0, 0 },
	{ 1920, 1080, 0, 228 },
	{ 1280, 720, 320, 408 },
};

#define ISX031_NUM_MODES ARRAY_SIZE(isx031_supported_modes)
#define ISX031_DEFAULT_MODE (&isx031_supported_modes[0])

#define V4L2_CID_FSYNC (V4L2_CID_USER_BASE | 0x1002)

#define ISX031_SHUTTER_TIME_MIN	0
#define ISX031_SHUTTER_TIME_MID	11010
#define ISX031_SHUTTER_TIME_MAX	33000

static const char * const isx031_supply_names[] = {
	"dvdd",
};

static const char * const isx031_ctrl_fsync_options[] = {
	"Internal",
	"External Pulse Sync",
	"External Trigger Sync",
};

struct isx031 {
	struct device *dev;
	struct clk *clk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(isx031_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

	bool streaming;
	int trigger_mode;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
};

static inline struct isx031 *to_isx031(struct v4l2_subdev *sd)
{
	return container_of(sd, struct isx031, subdev);
}

static int isx031_write_reg(struct isx031 *sensor, u16 reg, u8 val)
{
	return regmap_write(sensor->regmap, reg, val);
}

static int isx031_read_reg(struct isx031 *sensor, u16 reg, u8 *val)
{
	unsigned int tmp;
	int ret;

	ret = regmap_read(sensor->regmap, reg, &tmp);
	if (ret)
		return ret;

	*val = tmp & 0xFF;

	return 0;
}

static int isx031_write_reg16(struct isx031 *sensor, u16 reg, u16 val)
{
	int ret;

	ret = isx031_write_reg(sensor, reg, val & 0xFF);
	if (ret)
		return ret;

	return isx031_write_reg(sensor, reg + 1, (val >> 8) & 0xFF);
}

static int isx031_read_reg16(struct isx031 *sensor, u16 reg, u16 *val)
{
	u8 lo, hi;
	int ret;

	ret = isx031_read_reg(sensor, reg, &lo);
	if (ret)
		return ret;

	ret = isx031_read_reg(sensor, reg + 1, &hi);
	if (ret)
		return ret;

	*val = ((u16)hi << 8) | lo;

	return 0;
}

static int isx031_write_reg32(struct isx031 *sensor, u16 reg, u32 val)
{
	int ret;

	ret = isx031_write_reg16(sensor, reg, val & 0xFFFF);
	if (ret)
		return ret;

	return isx031_write_reg16(sensor, reg + 2, (val >> 16) & 0xFFFF);
}

static int isx031_copy_reg(struct isx031 *sensor, u16 src, u16 dst)
{
	u8 val;
	int ret;

	ret = isx031_read_reg(sensor, src, &val);
	if (ret)
		return ret;

	return isx031_write_reg(sensor, dst, val);
}

static int isx031_copy_reg16(struct isx031 *sensor, u16 src, u16 dst)
{
	int ret;

	ret = isx031_copy_reg(sensor, src, dst);
	if (ret)
		return ret;

	return isx031_copy_reg(sensor, src + 1, dst + 1);
}

static int isx031_copy_reg32(struct isx031 *sensor, u16 src, u16 dst)
{
	int ret;

	ret = isx031_copy_reg16(sensor, src, dst);
	if (ret)
		return ret;

	return isx031_copy_reg16(sensor, src + 2, dst + 2);
}

/*
 * Sensor State and Mode Control
 */

static unsigned int isx031_mbus_code(const struct isx031 *sensor)
{
	return MEDIA_BUS_FMT_UYVY8_1X16;
}

static int isx031_set_response_mode(struct isx031 *sensor)
{
	int ret;

	fsleep(20000);

	ret = isx031_write_reg(sensor, 0xbef0, 0x53);
	if (ret)
		return ret;
	fsleep(100000);

	ret = isx031_write_reg(sensor, 0x8a01, 0x00);
	if (ret)
		return ret;
	fsleep(100000);

	return 0;
}

/*
 * Power Management
 */

static int isx031_power_on(struct isx031 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	if (ret < 0)
		return ret;
	fsleep(1);

	ret = gpiod_direction_output(sensor->reset, 0);
	if (ret < 0)
		goto err_supply;
	fsleep(1);

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto err_reset;
	fsleep(100000);

	ret = isx031_set_response_mode(sensor);
	if (ret < 0)
		goto err_clk;

	return 0;

err_clk:
	clk_disable_unprepare(sensor->clk);
err_reset:
	gpiod_direction_output(sensor->reset, 1);
err_supply:
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);

	return ret;
}

static void isx031_power_off(struct isx031 *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_direction_output(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

static int isx031_set_fsync_trigger_mode(struct isx031 *sensor, u8 sg_mode)
{
	int ret;

	ret = isx031_write_reg(sensor, 0xbef0, 0x53);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0x8a01, 0x00);
	if (ret)
		return ret;
	fsleep(120000);

	ret = isx031_write_reg(sensor, 0x8afe, 0x00);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0x8aff, 0xff);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0x0153, 0x00);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0x8af0, sg_mode);
	if (ret)
		return ret;
	fsleep(120000);

	/* APL mirror */
	ret = isx031_write_reg(sensor, 0xbf14, sg_mode);
	if (ret)
		return ret;
	fsleep(240000);

	return isx031_write_reg(sensor, 0x8af1, 0x01);
}

/*
 * Digital Crop
 */

static int isx031_set_crop(struct isx031 *sensor, const struct isx031_mode *mode)
{
	int ret;

	ret = isx031_write_reg(sensor, 0x8aa8, 0x01);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0x8aaa, mode->width);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0x8aac, mode->h_crop_offset * 2);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0x8aae, mode->height);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0x8ab0, mode->v_crop_offset * 2);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0x8ada, 0x03);
	if (ret)
		return ret;

	/* APL mirror */
	ret = isx031_write_reg(sensor, 0xbf04, 0x01);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0xbf06, mode->width);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0xbf08, mode->h_crop_offset * 2);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0xbf0a, mode->height);
	if (ret)
		return ret;

	return isx031_write_reg16(sensor, 0xbf0c, mode->v_crop_offset * 2);
}

/*
 * Gain Control
 */

static int isx031_set_gain(struct isx031 *sensor, s64 val)
{
	int ret;
	u16 gain;

	dev_dbg(sensor->dev, "Gain set to %lld\n", val);

	gain = (u16)(10 * val);

	ret = isx031_write_reg(sensor, 0xabf0, 0x03);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0xabf8, 0x03);
	if (ret)
		return ret;

	ret = isx031_copy_reg32(sensor, 0x616c, 0xabec);
	if (ret)
		return ret;

	ret = isx031_copy_reg32(sensor, 0x647c, 0xabf4);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0xabfa, gain);
	if (ret)
		return ret;

	ret = isx031_copy_reg16(sensor, 0x6480, 0xabfc);
	if (ret)
		return ret;

	ret = isx031_copy_reg16(sensor, 0x6484, 0xabfe);
	if (ret)
		return ret;

	ret = isx031_copy_reg16(sensor, 0x6488, 0xac00);
	if (ret)
		return ret;
	fsleep(100);

	ret = isx031_write_reg(sensor, 0xabc0, 0x03);
	if (ret)
		return ret;

	return isx031_write_reg16(sensor, 0xac0a, gain);
}

/*
 * Auto Exposure
 */

static int isx031_set_auto_exposure(struct isx031 *sensor, bool enable)
{
	int ret;

	if (!enable)
		return 0;
	fsleep(100);

	ret = isx031_write_reg(sensor, 0xabc0, 0x00);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0xac4c, 0x03);
	if (ret)
		return ret;
	ret = isx031_write_reg(sensor, 0xac4d, 0x03);
	if (ret)
		return ret;
	ret = isx031_write_reg(sensor, 0xac4e, 0x03);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0xac40, ISX031_SHUTTER_TIME_MIN);
	if (ret)
		return ret;

	ret = isx031_write_reg16(sensor, 0xac44, ISX031_SHUTTER_TIME_MID);
	if (ret)
		return ret;

	return isx031_write_reg16(sensor, 0xac48, ISX031_SHUTTER_TIME_MAX);
}

/*
 * Exposure Time Control
 */

static int isx031_set_exposure(struct isx031 *sensor, s64 val)
{
	int ret;

	ret = isx031_write_reg(sensor, 0xac4d, 0x03);
	if (ret)
		return ret;

	ret = isx031_write_reg(sensor, 0xac4e, 0x03);
	if (ret)
		return ret;

	ret = isx031_write_reg32(sensor, 0xac44, (u32)val);
	if (ret)
		return ret;

	return isx031_write_reg32(sensor, 0xac48, (u32)val);
}

/*
 * V4L2 Controls
 */

static int isx031_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx031 *sensor = container_of(ctrl->handler, struct isx031, ctrls);
	bool enable;

	if (!sensor->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		return isx031_set_exposure(sensor, ctrl->val);
	case V4L2_CID_EXPOSURE_AUTO:
		enable = ctrl->val == V4L2_EXPOSURE_AUTO;
		return isx031_set_auto_exposure(sensor, enable);
	case V4L2_CID_ANALOGUE_GAIN:
		return isx031_set_gain(sensor, ctrl->val);
	case V4L2_CID_FSYNC:
		sensor->trigger_mode = ctrl->val;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ctrl_ops isx031_ctrl_ops = {
	.s_ctrl = isx031_s_ctrl,
};

static const struct v4l2_ctrl_config isx031_ctrl_fsync = {
	.ops = &isx031_ctrl_ops,
	.id = V4L2_CID_FSYNC,
	.name = "FSYNC source",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = ARRAY_SIZE(isx031_ctrl_fsync_options) - 1,
	.def = 0,
	.qmenu = isx031_ctrl_fsync_options,
};

static int isx031_ctrls_init(struct isx031 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 8);

	v4l2_ctrl_new_std(&sensor->ctrls, &isx031_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 33000, 1, 11010);
	v4l2_ctrl_new_std(&sensor->ctrls, &isx031_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 48, 1, 6);
	v4l2_ctrl_new_std_menu(&sensor->ctrls, &isx031_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO,
			       V4L2_EXPOSURE_MANUAL, 0,
			       V4L2_EXPOSURE_AUTO);
	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  1122000000 / 16, 1198000000 / 16, 1,
			  1188000000 / 16);

	ret = v4l2_ctrl_new_fwnode_properties(&sensor->ctrls,
					      &isx031_ctrl_ops, &props);
	if (ret)
		goto free_ctrls;

	v4l2_ctrl_new_custom(&sensor->ctrls, &isx031_ctrl_fsync, NULL);

	if (sensor->ctrls.error) {
		ret = sensor->ctrls.error;
		goto free_ctrls;
	}

	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls);

	return ret;
}

/*
 * V4L2 Subdev Operations
 */

static const struct isx031_mode *isx031_find_mode(u32 width, u32 height)
{
	unsigned int i;

	for (i = 0; i < ISX031_NUM_MODES; i++) {
		if (isx031_supported_modes[i].width == width &&
		    isx031_supported_modes[i].height == height)
			return &isx031_supported_modes[i];
	}

	return ISX031_DEFAULT_MODE;
}

static int isx031_setup(struct isx031 *sensor,
			struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *fmt;
	const struct isx031_mode *mode;

	fmt = v4l2_subdev_state_get_format(state, 0);
	mode = isx031_find_mode(fmt->width, fmt->height);

	return isx031_set_crop(sensor, mode);
}

static int isx031_stream_on(struct isx031 *sensor)
{
	int ret;

	if (sensor->trigger_mode) {
		ret = isx031_set_fsync_trigger_mode(sensor, sensor->trigger_mode);
		if (ret)
			return ret;
		dev_dbg(sensor->dev, "Camera sensor configured for FSYNC %s mode\n",
			sensor->trigger_mode == 1 ? "pulse sync" : "trigger sync");
	}

	ret = isx031_write_reg(sensor, 0x8a01, 0x80);
	if (ret)
		return ret;
	fsleep(120000);

	return 0;
}

static int isx031_stream_off(struct isx031 *sensor)
{
	int ret;

	ret = isx031_write_reg(sensor, 0x8a01, 0x00);
	if (ret)
		return ret;
	fsleep(120000);

	return 0;
}

static int isx031_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx031 *sensor = to_isx031(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (sensor->streaming == enable)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = isx031_stream_off(sensor);

		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);

		sensor->streaming = false;

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = isx031_setup(sensor, state);
	if (ret < 0)
		goto err_pm;

	sensor->streaming = true;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret < 0)
		goto err_pm;

	ret = isx031_stream_on(sensor);
	if (ret)
		goto err_pm;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	pm_runtime_put_sync(sensor->dev);
	sensor->streaming = false;

	goto unlock;
}

static int isx031_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct isx031 *sensor = to_isx031(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = isx031_mbus_code(sensor);

	return 0;
}

static int isx031_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct isx031 *sensor = to_isx031(sd);

	if (fse->index >= ISX031_NUM_MODES ||
	    fse->code != isx031_mbus_code(sensor))
		return -EINVAL;

	fse->min_width = isx031_supported_modes[fse->index].width;
	fse->max_width = isx031_supported_modes[fse->index].width;
	fse->min_height = isx031_supported_modes[fse->index].height;
	fse->max_height = isx031_supported_modes[fse->index].height;

	return 0;
}

static int isx031_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *state,
				      struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->pad != 0 || fie->index >= 1)
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = 30;

	return 0;
}

static int isx031_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	fmt->format = *v4l2_subdev_state_get_format(state, fmt->pad);

	return 0;
}

static int isx031_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct isx031 *sensor = to_isx031(sd);
	struct v4l2_mbus_framefmt *format;
	const struct isx031_mode *mode;

	mode = isx031_find_mode(fmt->format.width, fmt->format.height);

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	format->width = mode->width;
	format->height = mode->height;
	format->code = isx031_mbus_code(sensor);
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

	return 0;
}

static int isx031_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = ISX031_DEFAULT_MODE->width,
			.height = ISX031_DEFAULT_MODE->height,
		},
	};

	isx031_set_format(sd, state, &format);

	return 0;
}

static int isx031_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct isx031 *sensor = to_isx031(sd);

	if (pad != 0)
		return -EINVAL;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry[0].pixelcode = isx031_mbus_code(sensor);
	fd->entry[0].stream = 0;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_YUV422_8B;

	return 0;
}

static const struct v4l2_subdev_video_ops isx031_subdev_video_ops = {
	.s_stream = isx031_s_stream,
};

static const struct v4l2_subdev_pad_ops isx031_subdev_pad_ops = {
	.enum_mbus_code = isx031_enum_mbus_code,
	.enum_frame_size = isx031_enum_frame_size,
	.enum_frame_interval = isx031_enum_frame_interval,
	.get_fmt = isx031_get_format,
	.set_fmt = isx031_set_format,
	.get_frame_desc = isx031_get_frame_desc,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int isx031_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct isx031 *sensor = container_of(sd, struct isx031, subdev);

	return isx031_write_reg(sensor, reg->reg, reg->val);
}

static int isx031_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct isx031 *sensor = container_of(sd, struct isx031, subdev);
	u8 val;
	int ret;

	reg->size = 1;
	ret = isx031_read_reg(sensor, reg->reg, &val);
	reg->val = val;

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops isx031_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = isx031_g_register,
	.s_register = isx031_s_register,
#endif
};

static const struct v4l2_subdev_ops isx031_subdev_ops = {
	.core = &isx031_core_ops,
	.video = &isx031_subdev_video_ops,
	.pad = &isx031_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops isx031_internal_ops = {
	.init_state = isx031_init_state,
};

/*
 * V4L2 Subdev Init / Cleanup
 */

static int isx031_subdev_init(struct isx031 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &isx031_subdev_ops);
	sensor->subdev.internal_ops = &isx031_internal_ops;

	ret = isx031_ctrls_init(sensor);
	if (ret < 0)
		return ret;

	sensor->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->subdev.entity, 1, &sensor->pad);
	if (ret < 0) {
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return ret;
	}

	sensor->subdev.state_lock = sensor->subdev.ctrl_handler->lock;

	v4l2_subdev_init_finalize(&sensor->subdev);

	return ret;
}

static void isx031_subdev_cleanup(struct isx031 *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

/*
 * Runtime Power Management
 */

static int isx031_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx031 *sensor = to_isx031(subdev);

	return isx031_power_on(sensor);
}

static int isx031_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx031 *sensor = to_isx031(subdev);

	isx031_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops isx031_pm_ops = {
	SET_RUNTIME_PM_OPS(isx031_runtime_suspend, isx031_runtime_resume, NULL)
};

/*
 * Probe and Remove
 */

static const struct regmap_config isx031_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static int isx031_identify(struct isx031 *sensor)
{
	u16 chip_id;
	int ret;

	ret = isx031_read_reg16(sensor, 0x7e8a, &chip_id);
	if (ret) {
		dev_err(sensor->dev, "Failed to read chip ID: %d\n", ret);
		return ret;
	}

	if (chip_id != 0x6031) {
		dev_err(sensor->dev, "Unexpected chip ID 0x%04x (expected 0x6031)\n",
			chip_id);
		return -ENODEV;
	}

	dev_info(sensor->dev, "ISX031 sensor detected (chip ID 0x%04x)\n",
		 chip_id);

	return 0;
}

static int isx031_probe(struct i2c_client *client)
{
	struct isx031 *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = isx031_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev,
				      ARRAY_SIZE(sensor->supplies),
				      sensor->supplies);
	if (ret) {
		dev_err_probe(sensor->dev, ret, "failed to get supplies\n");
		return ret;
	}

	sensor->reset = devm_gpiod_get_optional(sensor->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->reset))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->reset),
				     "failed to get reset GPIO\n");

	sensor->clk = devm_clk_get(sensor->dev, "xclk");
	if (IS_ERR(sensor->clk))
		return dev_err_probe(sensor->dev, PTR_ERR(sensor->clk),
				     "failed to get clock\n");

	sensor->regmap = devm_regmap_init_i2c(client, &isx031_regmap_config);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	sensor->trigger_mode = 0;

	ret = isx031_power_on(sensor);
	if (ret < 0)
		return ret;

	ret = isx031_identify(sensor);
	if (ret < 0)
		goto err_power;

	ret = isx031_subdev_init(sensor);
	if (ret < 0)
		goto err_power;

	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	ret = v4l2_async_register_subdev(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	isx031_subdev_cleanup(sensor);
err_power:
	isx031_power_off(sensor);

	return ret;
}

static void isx031_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx031 *sensor = to_isx031(subdev);

	v4l2_async_unregister_subdev(subdev);

	isx031_subdev_cleanup(sensor);

	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		isx031_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct of_device_id isx031_of_match[] = {
	{ .compatible = "sony,isx031" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, isx031_of_match);

static struct i2c_driver isx031_i2c_driver = {
	.driver = {
		.of_match_table = isx031_of_match,
		.name = "isx031",
		.pm = pm_ptr(&isx031_pm_ops),
	},
	.probe = isx031_probe,
	.remove = isx031_remove,
};

module_i2c_driver(isx031_i2c_driver);

MODULE_DESCRIPTION("Sony ISX031 CMOS image sensor driver");
MODULE_AUTHOR("Analog Devices Inc.");
MODULE_LICENSE("GPL");
