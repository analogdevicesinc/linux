// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for IMX490 CMOS Image Sensor from Sony
 *
 * Copyright 2024 Bogdan Togorean <bogdan.togorean@analog.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define IMX490_WIDTH	2880
#define IMX490_HEIGHT	1860

#define PLUS_10(x)  ((x)+(x)/10)

#define V4L2_CID_TPG		(V4L2_CID_USER_BASE | 0x1003)

static const char * const imx490_supply_names[] = {
	"dvdd",
};

static const char * const imx490_ctrl_test_pattern_options[] = {
	"Disabled",
	"Enabled",
};

struct imx490 {
	struct device *dev;
	struct clk *clk;
	struct regulator_bulk_data supplies[ARRAY_SIZE(imx490_supply_names)];
	struct gpio_desc *reset;

	bool streaming;
	int trigger_mode;

	struct v4l2_subdev subdev;
	struct media_pad pad;

	struct v4l2_ctrl_handler ctrls;
};

static inline struct imx490 *to_imx490(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx490, subdev);
}

static int imx490_read(struct imx490 *sensor, u16 addr, u8 *value)
{
	int ret = 0;

	return ret;
}

static int imx490_write(struct imx490 *sensor, u32 addr, u32 value)
{
	int ret = 0;

	return ret;
}

static u32 imx490_mbus_code(const struct imx490 *sensor)
{
	return MEDIA_BUS_FMT_UYVY8_1X16;
}

static int imx490_power_on(struct imx490 *sensor)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(sensor->supplies),
				    sensor->supplies);
	if (ret < 0)
		return ret;

	ret = gpiod_direction_output(sensor->reset, 0);
	if (ret < 0)
		goto err_supply;

	ret = clk_prepare_enable(sensor->clk);
	if (ret < 0)
		goto err_reset;

	/*
	 * The documentation doesn't explicitly say how much time is required
	 * after providing a clock and before starting I2C communication. It
	 * mentions a delay of 20µs in 4-wire mode, but tests showed that a
	 * delay of 100µs resulted in I2C communication failures, while 500µs
	 * seems to be enough. Be conservative.
	 */
	usleep_range(1000, PLUS_10(1000));

	return 0;

err_reset:
	gpiod_direction_output(sensor->reset, 1);
err_supply:
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
	return ret;
}

static void imx490_power_off(struct imx490 *sensor)
{
	clk_disable_unprepare(sensor->clk);
	gpiod_direction_output(sensor->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(sensor->supplies), sensor->supplies);
}

/* ------------------------------------------------------------------------- */

static int imx490_set_tpg(struct imx490 *sensor, s32 val)
{
	u32 enabled = 0;
	int ret = 0;

	if (val)
		enabled = 1;

	return ret;
}

/* -----------------------------------------------------------------------------
 * Controls
 */

static int imx490_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx490 *sensor = container_of(ctrl->handler, struct imx490, ctrls);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	bool enable;
	int ret = 0;

	if (!sensor->streaming)
		return 0;

	state = v4l2_subdev_get_locked_active_state(&sensor->subdev);
	format = v4l2_subdev_get_pad_format(&sensor->subdev, state, 0);

	switch (ctrl->id) {
	case V4L2_CID_TPG:
		imx490_set_tpg(sensor, ctrl->val);
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx490_ctrl_ops = {
	.s_ctrl = imx490_s_ctrl,
};

static const struct v4l2_ctrl_config imx490_ctrl_tpg = {
	.ops = &imx490_ctrl_ops,
	.id = V4L2_CID_TPG,
	.name = "Test Pattern Generator",
	.type = V4L2_CTRL_TYPE_MENU,
	.max = ARRAY_SIZE(imx490_ctrl_test_pattern_options) - 1,
	.def = 0,
	.qmenu = imx490_ctrl_test_pattern_options,
};

static int imx490_ctrls_init(struct imx490 *sensor)
{
	struct v4l2_fwnode_device_properties props;
	int ret;

	ret = v4l2_fwnode_device_parse(sensor->dev, &props);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(&sensor->ctrls, 9);
	/*
	 * The sensor calculates the MIPI timings internally to achieve a bit
	 * rate between 1122 and 1198 Mbps. The exact value is unfortunately not
	 * reported, at least according to the documentation. Report a nominal
	 * rate of 1188 Mbps as that is used by the datasheet in multiple
	 * examples.
	 */
	v4l2_ctrl_new_std(&sensor->ctrls, NULL, V4L2_CID_PIXEL_RATE,
			  1122000000 / 16, 1198000000 / 16, 1, 1188000000 / 16);

	v4l2_ctrl_new_fwnode_properties(&sensor->ctrls, &imx490_ctrl_ops,
					&props);

	v4l2_ctrl_new_custom(&sensor->ctrls,
				     &imx490_ctrl_tpg, NULL);

	if (sensor->ctrls.error) {
		dev_err(sensor->dev, "failed to add controls (%d)\n",
			sensor->ctrls.error);
		v4l2_ctrl_handler_free(&sensor->ctrls);
		return sensor->ctrls.error;
	}

	sensor->subdev.ctrl_handler = &sensor->ctrls;

	return 0;
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

/*
 * This table is extracted from vendor data that is entirely undocumented. The
 * first register write is required to activate the CSI-2 output. The other
 * entries may or may not be optional?
 */
static const struct {
	unsigned int reg;
	unsigned int value;
} imx490_init_table[] = {
};

static int imx490_setup(struct imx490 *sensor, struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;
	unsigned int i;
	int ret = 0;

	format = v4l2_subdev_get_pad_format(&sensor->subdev, state, 0);

	for (i = 0; i < ARRAY_SIZE(imx490_init_table); ++i)
		ret = imx490_write(sensor, imx490_init_table[i].reg,
			     imx490_init_table[i].value);

	return ret;
}

static int imx490_stream_on(struct imx490 *sensor)
{
	return 0;
}

static int imx490_stream_off(struct imx490 *sensor)
{
	return 0;
}

static int imx490_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx490 *sensor = to_imx490(sd);
	struct v4l2_subdev_state *state;
	int ret;

	if (sensor->streaming == enable)
		return 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (!enable) {
		ret = imx490_stream_off(sensor);

		pm_runtime_mark_last_busy(sensor->dev);
		pm_runtime_put_autosuspend(sensor->dev);

		sensor->streaming = false;

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(sensor->dev);
	if (ret < 0)
		goto unlock;

	ret = imx490_setup(sensor, state);
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

	ret = imx490_stream_on(sensor);
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

static int imx490_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int imx490_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct imx490 *sensor = to_imx490(sd);
	int ret = 0;

	if (fi->pad != 0)
		return -EINVAL;

	if (sensor->streaming)
		return -EBUSY;

	return ret;
}

static int imx490_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx490 *sensor = to_imx490(sd);

	if (code->index != 0)
		return -EINVAL;

	code->code = imx490_mbus_code(sensor);

	return 0;
}

static int imx490_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	const struct imx490 *sensor = to_imx490(sd);
	const struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_pad_format(sd, state, fse->pad);

	if (fse->index >= 1 || fse->code != imx490_mbus_code(sensor))
		return -EINVAL;

	fse->min_width = IMX490_WIDTH / (fse->index + 1);
	fse->max_width = fse->min_width;
	fse->min_height = IMX490_HEIGHT / (fse->index + 1);
	fse->max_height = fse->min_height;

	return 0;
}

static int imx490_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;

	if (fie->pad != 0)
		return -EINVAL;
	if (fie->index >= 3)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = (fie->index + 1) * 10;

	fie->interval = tpf;
	return 0;
}

static int imx490_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	fmt->format = *v4l2_subdev_get_pad_format(sd, state, fmt->pad);

	return 0;
}

static int imx490_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct imx490 *sensor = to_imx490(sd);
	struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_get_pad_format(sd, state, fmt->pad);

	format->width = IMX490_WIDTH;
	format->height = IMX490_HEIGHT;

	format->code = imx490_mbus_code(sensor);
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SRGB;
	format->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	format->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	format->xfer_func = V4L2_XFER_FUNC_NONE;

	fmt->format = *format;

	return 0;
}

static int imx490_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format format = {
		.format = {
			.width = IMX490_WIDTH,
			.height = IMX490_HEIGHT,
		},
	};
	imx490_set_format(sd, state, &format);

	return 0;
}

static const struct v4l2_subdev_video_ops imx490_subdev_video_ops = {
	.g_frame_interval = imx490_g_frame_interval,
	.s_frame_interval = imx490_s_frame_interval,
	.s_stream = imx490_s_stream,
};

static const struct v4l2_subdev_pad_ops imx490_subdev_pad_ops = {
	.enum_mbus_code = imx490_enum_mbus_code,
	.enum_frame_size = imx490_enum_frame_size,
	.enum_frame_interval = imx490_enum_frame_interval,
	.get_fmt = imx490_get_format,
	.set_fmt = imx490_set_format,
	.init_cfg = imx490_init_cfg,
};

static const struct v4l2_subdev_core_ops imx490_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
#endif
};

static const struct v4l2_subdev_ops imx490_subdev_ops = {
	.core = &imx490_core_ops,
	.video = &imx490_subdev_video_ops,
	.pad = &imx490_subdev_pad_ops,
};

static int imx490_subdev_init(struct imx490 *sensor)
{
	struct i2c_client *client = to_i2c_client(sensor->dev);
	int ret;

	v4l2_i2c_subdev_init(&sensor->subdev, client, &imx490_subdev_ops);

	ret = imx490_ctrls_init(sensor);
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

static void imx490_subdev_cleanup(struct imx490 *sensor)
{
	media_entity_cleanup(&sensor->subdev.entity);
	v4l2_ctrl_handler_free(&sensor->ctrls);
}

/* -----------------------------------------------------------------------------
 * Power management
 */

static int __maybe_unused imx490_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx490 *sensor = to_imx490(subdev);

	return imx490_power_on(sensor);
}

static int __maybe_unused imx490_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx490 *sensor = to_imx490(subdev);

	imx490_power_off(sensor);

	return 0;
}

static const struct dev_pm_ops imx490_pm_ops = {
	SET_RUNTIME_PM_OPS(imx490_runtime_suspend, imx490_runtime_resume, NULL)
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int imx490_probe(struct i2c_client *client)
{
	struct imx490 *sensor;
	unsigned int i;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->dev = &client->dev;

	/* Acquire resources. */
	for (i = 0; i < ARRAY_SIZE(sensor->supplies); ++i)
		sensor->supplies[i].supply = imx490_supply_names[i];

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

	sensor->trigger_mode = 0;

	/*
	 * Enable power management. The driver supports runtime PM, but needs to
	 * work when runtime PM is disabled in the kernel. To that end, power
	 * the sensor on manually here, identify it, and fully initialize it.
	 */
	ret = imx490_power_on(sensor);
	if (ret < 0)
		return ret;

	/* Initialize the V4L2 subdev. */
	ret = imx490_subdev_init(sensor);
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
	imx490_subdev_cleanup(sensor);
err_power:
	imx490_power_off(sensor);
	return ret;
}

static void imx490_remove(struct i2c_client *client)
{
	struct v4l2_subdev *subdev = i2c_get_clientdata(client);
	struct imx490 *sensor = to_imx490(subdev);

	v4l2_async_unregister_subdev(subdev);

	imx490_subdev_cleanup(sensor);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	 */
	pm_runtime_disable(sensor->dev);
	if (!pm_runtime_status_suspended(sensor->dev))
		imx490_power_off(sensor);
	pm_runtime_set_suspended(sensor->dev);
}

static const struct of_device_id imx490_of_match[] = {
	{ .compatible = "sony,imx490", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, imx490_of_match);

static struct i2c_driver imx490_i2c_driver = {
	.driver = {
		.of_match_table = imx490_of_match,
		.name = "imx490",
		.pm = &imx490_pm_ops
	},
	.probe_new = imx490_probe,
	.remove = imx490_remove,
};

module_i2c_driver(imx490_i2c_driver);

MODULE_DESCRIPTION("Sony IMX490 Camera driver");
MODULE_AUTHOR("Bogdan Togorean <bogdan.togorean@analog.com>");
MODULE_LICENSE("GPL");
