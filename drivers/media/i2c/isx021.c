// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for ISX021 CMOS Image Sensor from Sony
 *
 * Copyright 2024 Bogdan Togorean <bogdan.togorean@analog.com>
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

#define ISX021_WIDTH 1920
#define ISX021_HEIGHT 1280

#define V4L2_CID_FSYNC (V4L2_CID_USER_BASE | 0x1002)
#define V4L2_CID_TPG (V4L2_CID_USER_BASE | 0x1003)

#define ISX021_SHUTTER_TIME_MIN 0
#define ISX021_SHUTTER_TIME_MID 11010
#define ISX021_SHUTTER_TIME_MAX 33000

static const char *const isx021_supply_names[] = {
	"dvdd",
};

static const char *const isx021_ctrl_fsync_options[] = {
	"Internal",
	"External",
};

static const char *const isx021_ctrl_test_pattern_options[] = {
	"Disabled",
	"Enabled",
};

struct isx021 {
	struct device *dev;
	struct clk *clk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(isx021_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

	bool streaming;
	int trigger_mode;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
};

static int isx021_set_response_mode(struct isx021 *sensor);

static inline struct isx021 *to_isx021(struct v4l2_subdev *sd)
{
	return container_of(sd, struct isx021, subdev);
}

static int isx021_copy_reg_value(struct isx021 *sensor, u16 ae_addr,
				 u16 me_addr)
{
	unsigned int val;
	int ret;

	ret = regmap_read(sensor->regmap, ae_addr, &val);
	if (ret)
		return ret;

	return regmap_write(sensor->regmap, me_addr, val);
}

static unsigned int isx021_mbus_code(const struct isx021 *sensor)
{
	return MEDIA_BUS_FMT_UYVY8_1X16;
}

static int isx021_power_on(struct isx021 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
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

	/*
	 * The documentation doesn't explicitly say how much time is required
	 * after providing a clock and before starting I2C communication. It
	 * mentions a delay of 20µs in 4-wire mode, but tests showed that a
	 * delay of 100µs resulted in I2C communication failures.
	 */
	fsleep(100000);

	ret = isx021_set_response_mode(sensor);
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

static void isx021_power_off(struct isx021 *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_direction_output(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

/* ------------------------------------------------------------------------- */

static int isx021_write_mode_set_f_lock_register(struct isx021 *sensor, u8 val)
{
	int ret;

	fsleep(20000);

	ret = regmap_write(sensor->regmap, 0x8a55, 0x06);
	if (ret)
		return ret;

	fsleep(20000);

	ret = regmap_write(sensor->regmap, 0xbef0, val);
	if (ret)
		return ret;

	fsleep(20000);

	return regmap_write(sensor->regmap, 0x8a55, 0x02);
}

static int isx021_set_gain(struct isx021 *sensor, s64 val)
{
	int ret;

	s64 gain;
	u8 digital_gain_low_byte;
	u8 digital_gain_high_byte;

	dev_dbg(sensor->dev, "[%s] :Gain is set to %lld\n", __func__, val);

	gain = 10 * val;

	digital_gain_low_byte = gain & 0xFF;
	digital_gain_high_byte = (gain >> 8) & 0xFF;

	ret = regmap_write(sensor->regmap, 0xabf0, 0x01);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xabf8, 0x01);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x616c, 0xabec);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x616d, 0xabed);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x616e, 0xabee);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x616f, 0xabef);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x647c, 0xabf4);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x647d, 0xabf5);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x647e, 0xabf6);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x647f, 0xabf7);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0x6170, digital_gain_low_byte);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0x6171, digital_gain_high_byte);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x6480, 0xabfc);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x6481, 0xabfd);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x6484, 0xabfe);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x6485, 0xabff);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x6488, 0xac00);
	if (ret)
		return ret;

	ret = isx021_copy_reg_value(sensor, 0x6489, 0xac01);
	if (ret)
		return ret;

	fsleep(100);

	ret = regmap_write(sensor->regmap, 0xabc0, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac0a, digital_gain_low_byte);
	if (ret)
		return ret;

	return regmap_write(sensor->regmap, 0xac0b, digital_gain_high_byte);
}

static int isx021_set_auto_exposure(struct isx021 *sensor, bool enable)
{
	int ret;

	if (!enable)
		return 0;

	fsleep(100);

	ret = regmap_write(sensor->regmap, 0xabc0, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac4c, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac4d, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac4e, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac40, ISX021_SHUTTER_TIME_MIN & 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac41,
			   (ISX021_SHUTTER_TIME_MIN >> 8) & 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac44, ISX021_SHUTTER_TIME_MID & 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac45,
			   (ISX021_SHUTTER_TIME_MID >> 8) & 0xFF);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac48, ISX021_SHUTTER_TIME_MAX & 0xFF);
	if (ret)
		return ret;

	return regmap_write(sensor->regmap, 0xac49,
			   (ISX021_SHUTTER_TIME_MAX >> 8) & 0xFF);
}

static int isx021_set_exposure(struct isx021 *sensor, s64 val)
{
	u8 exp_time_byte0;
	u8 exp_time_byte1;
	u8 exp_time_byte2;
	u8 exp_time_byte3;
	int ret;

	exp_time_byte0 = val & 0xFF;
	exp_time_byte1 = (val >> 8) & 0xFF;
	exp_time_byte2 = (val >> 16) & 0xFF;
	exp_time_byte3 = (val >> 24) & 0xFF;

	ret = regmap_write(sensor->regmap, 0xac4d, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac4e, 0x03);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac44, exp_time_byte0);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac45, exp_time_byte1);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac46, exp_time_byte2);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac47, exp_time_byte3);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac48, exp_time_byte0);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac49, exp_time_byte1);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0xac4a, exp_time_byte2);
	if (ret)
		return ret;

	return regmap_write(sensor->regmap, 0xac4b, exp_time_byte3);
}

static int isx021_set_response_mode(struct isx021 *sensor)
{
	int es_number;
	unsigned int r_val;
	int ret;

	ret = isx021_write_mode_set_f_lock_register(sensor, 0x53);
	if (ret)
		return ret;

	fsleep(100000);

	ret = regmap_write(sensor->regmap, 0x8a01, 0x00);
	if (ret)
		return ret;

	fsleep(100000);

	ret = regmap_read(sensor->regmap, 0x8a55, &r_val);
	if (ret)
		return ret;

	if (r_val == 0x04)
		es_number = 2;
	else
		es_number = 3;

	ret = regmap_write(sensor->regmap, 0x8a55, 0x06);
	if (ret)
		return ret;

	fsleep(100000);

	return 0;
}

static int isx021_set_fsync_trigger_mode(struct isx021 *sensor)
{
	int ret;

	ret = isx021_write_mode_set_f_lock_register(sensor, 0x53);
	if (ret)
		return ret;

	ret = regmap_write(sensor->regmap, 0x8a01, 0x00);
	if (ret)
		return ret;

	fsleep(120000);

	ret = regmap_write(sensor->regmap, 0x8af0, 0x02);
	if (ret)
		return ret;

	fsleep(120000);

	ret = regmap_write(sensor->regmap, 0xbf14, 0x02);
	if (ret)
		return ret;

	fsleep(240000);

	return 0;
}

static int isx021_set_tpg(struct isx021 *sensor, s32 val)
{
	u32 enabled = 0;
	int ret;

	if (val)
		enabled = 1;

	ret = regmap_write(sensor->regmap, 0xbe14, enabled);
	if (ret)
		return ret;

	return regmap_write(sensor->regmap, 0xbf60, enabled);
}

/* -----------------------------------------------------------------------------
 * Controls
 */

static int isx021_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isx021 *sensor =
		container_of(ctrl->handler, struct isx021, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	bool enable;

	if (!sensor->streaming)
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_state_get_format(state, 0);

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		isx021_set_exposure(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		enable = ctrl->val == V4L2_EXPOSURE_AUTO;
		isx021_set_auto_exposure(sensor, enable);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		isx021_set_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_FSYNC:
		sensor->trigger_mode = ctrl->val;
		break;
	case V4L2_CID_TPG:
		isx021_set_tpg(sensor, ctrl->val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops isx021_ctrl_ops = {
	.s_ctrl = isx021_s_ctrl,
};

static const struct v4l2_ctrl_config isx021_ctrl_fsync = {
	.ops = &isx021_ctrl_ops,
	.id = V4L2_CID_FSYNC,
	.name = "FSYNC source",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = ARRAY_SIZE(isx021_ctrl_fsync_options) - 1,
	.def = 0,
	.qmenu = isx021_ctrl_fsync_options,
};

static const struct v4l2_ctrl_config isx021_ctrl_tpg = {
	.ops = &isx021_ctrl_ops,
	.id = V4L2_CID_TPG,
	.name = "Test Pattern Generator",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = ARRAY_SIZE(isx021_ctrl_test_pattern_options) - 1,
	.def = 0,
	.qmenu = isx021_ctrl_test_pattern_options,
};

static int isx021_ctrls_init(struct isx021 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 9);

	v4l2_ctrl_new_std(&sensor->ctrls, &isx021_ctrl_ops, V4L2_CID_EXPOSURE,
			  0, 33000, 1, 11010);
	v4l2_ctrl_new_std(&sensor->ctrls, &isx021_ctrl_ops,
			  V4L2_CID_ANALOGUE_GAIN, 0, 48, 1, 6);
	v4l2_ctrl_new_std_menu(&sensor->ctrls, &isx021_ctrl_ops,
			       V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, 0,
			       V4L2_EXPOSURE_AUTO);
	/*
	 * The sensor calculates the MIPI timings internally to achieve a bit
	 * rate between 1122 and 1198 Mbps. The exact value is unfortunately not
	 * reported, at least according to the documentation. Report a nominal
	 * rate of 1188 Mbps as that is used by the datasheet in multiple
	 * examples.
	 */
	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  1122000000 / 16, 1198000000 / 16, 1, 1188000000 / 16);

	ret = v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &isx021_ctrl_ops,
					      &props);
	if (ret)
		goto free_ctrls;

	v4l2_ctrl_new_custom(&sensor->ctrls, &isx021_ctrl_fsync, NULL);

	v4l2_ctrl_new_custom(&sensor->ctrls, &isx021_ctrl_tpg, NULL);

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

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static int isx021_setup(struct isx021 *sensor, struct v4l2_subdev_state *state)
{
	return 0;
}

static int isx021_stream_on(struct isx021 *sensor)
{
	int ret;

	if (sensor->trigger_mode) {
		ret = isx021_set_fsync_trigger_mode(sensor);
		dev_dbg(sensor->dev, "Putting camera sensor into Slave mode.\n");
	}

	ret = regmap_write(sensor->regmap, 0x8a01, 0x80);
	if (ret)
		return ret;

	fsleep(120000);

	return 0;
}

static int isx021_stream_off(struct isx021 *sensor)
{
	int ret;

	ret = regmap_write(sensor->regmap, 0x8a01, 0x00);
	if (ret)
		return ret;

	fsleep(120000);

	return 0;
}

static int isx021_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isx021 *sensor = to_isx021(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (sensor->streaming == enable)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = isx021_stream_off(sensor);

		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);

		sensor->streaming = false;

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = isx021_setup(sensor, state);
	if (ret < 0)
		goto err_pm;

	/*
	 * Set streaming to true to ensure __v4l2_ctrl_handler_setup() will set
	 * the controls. The flag is reset to false further down if an error
	 * occurs.
	 */
	sensor->streaming = true;

	ret = __v4l2_ctrl_handler_setup(&sensor->ctrls);
	if (ret < 0)
		goto err_pm;

	ret = isx021_stream_on(sensor);
	if (ret)
		goto err_pm;

unlock:
	v4l2_subdev_unlock_state(state);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync(sensor->dev);
	sensor->streaming = false;

	goto unlock;
}


static int isx021_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct isx021 *sensor = to_isx021(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = isx021_mbus_code(sensor);

	return 0;
}

static int isx021_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct isx021 *sensor = to_isx021(sd);

	if (fse->index >= 1 || fse->code != isx021_mbus_code(sensor))
		return -EINVAL;

	fse->min_width = ISX021_WIDTH / (fse->index + 1);
	fse->max_width = fse->min_width;
	fse->min_height = ISX021_HEIGHT / (fse->index + 1);
	fse->max_height = fse->min_height;

	return 0;
}

static int isx021_enum_frame_interval(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= 1)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = 30;

	fie->interval = tpf;
	return 0;
}

static int isx021_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	fmt->format = *v4l2_subdev_state_get_format(state, fmt->pad);

	return 0;
}

static int isx021_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct isx021 *sensor = to_isx021(sd);
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, fmt->pad);

	format->width = ISX021_WIDTH;
	format->height = ISX021_HEIGHT;

	format->code = isx021_mbus_code(sensor);
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

	return 0;
}

static int isx021_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = ISX021_WIDTH,
			.height = ISX021_HEIGHT,
		},
	};
	isx021_set_format(sd, state, &format);

	return 0;
}

static int isx021_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct isx021 *sensor = to_isx021(sd);

	if (pad != 0)
		return -EINVAL;

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	fd->entry[0].pixelcode = isx021_mbus_code(sensor);
	fd->entry[0].stream = 0;
	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = MIPI_CSI2_DT_YUV422_8B;

	return 0;
}

static const struct v4l2_subdev_video_ops isx021_subdev_video_ops = {
	.s_stream = isx021_s_stream,
};

static const struct v4l2_subdev_pad_ops isx021_subdev_pad_ops = {
	.enum_mbus_code = isx021_enum_mbus_code,
	.enum_frame_size = isx021_enum_frame_size,
	.enum_frame_interval = isx021_enum_frame_interval,
	.get_fmt = isx021_get_format,
	.set_fmt = isx021_set_format,
	.get_frame_desc = isx021_get_frame_desc,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int isx021_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct isx021 *sensor = container_of(sd, struct isx021, subdev);

	return regmap_write(sensor->regmap, reg->reg, reg->val);
}

static int isx021_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct isx021 *sensor = container_of(sd, struct isx021, subdev);
	unsigned int aux;
	int ret;

	reg->size = 1;
	ret = regmap_read(sensor->regmap, reg->reg, &aux);
	reg->val = aux;

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops isx021_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = isx021_g_register,
	.s_register = isx021_s_register,
#endif
};

static const struct v4l2_subdev_ops isx021_subdev_ops = {
	.core = &isx021_core_ops,
	.video = &isx021_subdev_video_ops,
	.pad = &isx021_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops isx021_internal_ops = {
	.init_state = isx021_init_state,
};

static int isx021_subdev_init(struct isx021 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &isx021_subdev_ops);
	sensor->subdev.internal_ops = &isx021_internal_ops;

	ret = isx021_ctrls_init(sensor);
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

static void isx021_subdev_cleanup(struct isx021 *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

/* -----------------------------------------------------------------------------
 * Power management
 */

static int isx021_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx021 *sensor = to_isx021(subdev);

	return isx021_power_on(sensor);
}

static int isx021_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx021 *sensor = to_isx021(subdev);

	isx021_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops isx021_pm_ops = {
	SET_RUNTIME_PM_OPS(isx021_runtime_suspend, isx021_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static const struct regmap_config isx021_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
};

static int isx021_probe(struct i2c_client *client)
{
	struct isx021 *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = isx021_supply_names[i];

	ret = devm_regulator_bulk_get(sensor->dev, ARRAY_SIZE(sensor->supplies),
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

	sensor->regmap = devm_regmap_init_i2c(client, &isx021_regmap_config);
	if (IS_ERR(sensor->regmap))
		return PTR_ERR(sensor->regmap);

	sensor->trigger_mode = 0;

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = isx021_power_on(sensor);
	if (ret < 0)
		return ret;

	ret = isx021_subdev_init(sensor);
	if (ret < 0)
		goto err_power;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(sensor->dev);
	pm_runtime_get_noresume(sensor->dev);
	pm_runtime_enable(sensor->dev);

	/* Register the V4L2 subdev. */
	ret = v4l2_async_register_subdev(&sensor->subdev);
	if (ret < 0)
		goto err_pm;

	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(sensor->dev, 1000);
	pm_runtime_use_autosuspend(sensor->dev);
	pm_runtime_put_autosuspend(sensor->dev);

	return 0;

err_pm:
	pm_runtime_disable(sensor->dev);
	pm_runtime_put_noidle(sensor->dev);
	isx021_subdev_cleanup(sensor);
err_power:
	isx021_power_off(sensor);
	return ret;
}

static void isx021_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct isx021 *sensor = to_isx021(subdev);

	v4l2_async_unregister_subdev(subdev);

	isx021_subdev_cleanup(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		isx021_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct of_device_id isx021_of_match[] = {
	{ .compatible = "sony,isx021", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, isx021_of_match);

static struct i2c_driver isx021_i2c_driver = {
	.driver = {
		.of_match_table = isx021_of_match,
		.name = "isx021",
		.pm = pm_ptr(&isx021_pm_ops),
	},
	.probe = isx021_probe,
	.remove = isx021_remove,
};

module_i2c_driver(isx021_i2c_driver);

MODULE_DESCRIPTION("Sony ISX021 sensor driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
