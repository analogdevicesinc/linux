// SPDX-License-Identifier: GPL-2.0
/* This driver only includes a modified support based on original RealSense D457 driver.
 * It contains I2C-ATR support in order to enable the use of D457's cameras
 * as individually controllable V4L2-subdevices.
 * 
 * ds5.c - Intel(R) RealSense(TM) D4XX camera driver
 *
 * Copyright (c) 2017-2023, INTEL CORPORATION.  All rights reserved.
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
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/videodev2.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "rs_d457.h"

static struct ds457_subdev_priv *next_subdev(struct ds457 *priv,
						struct ds457_subdev_priv *sd_priv)
{
	if (!sd_priv)
		sd_priv = &priv->sd_privs[0];
	else
		sd_priv++;

	for (; sd_priv < priv->sd_privs + priv->num_subdevs; sd_priv++) {
		if (sd_priv->fwnode)
			return sd_priv;
	}

	return NULL;
}

#define for_each_subdev(priv, sd_priv) \
	for ((sd_priv) = NULL; ((sd_priv) = next_subdev((priv), (sd_priv))); )

static inline struct ds457_asd *to_ds457_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct ds457_asd, base);
}

static inline struct ds457_subdev_priv *sd_to_ds457_subdev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ds457_subdev_priv, sd);
}

static int ds457_i2c_atr_attach_client(struct i2c_atr *atr, u32 chan_id,
					 const struct i2c_client *client, u16 alias)
{
	return 0;
}

static void ds457_i2c_atr_detach_client(struct i2c_atr *atr, u32 chan_id,
					  const struct i2c_client *client)
{
// do nothing
}

static const struct i2c_atr_ops ds457_i2c_atr_ops = {
	.attach_client = ds457_i2c_atr_attach_client,
	.detach_client = ds457_i2c_atr_detach_client,
};

static void ds457_i2c_atr_deinit(struct ds457 *priv)
{
	/* Deleting adapters that haven't been added does no harm. */
	unsigned int i;

	// delete the cameras that were not added in the structure
	for (i = 0; i < NUM_CAMS; i++) {
		struct ds457_cams *cam = &priv->cams[i]; 
		i2c_atr_del_adapter(priv->atr, cam->index);
	}
	i2c_atr_delete(priv->atr);
}

static int ds457_i2c_atr_init(struct ds457 *priv)
{
	int ret;
	unsigned int i;

	if (!i2c_check_functionality(priv->client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -ENODEV;

	priv->atr = i2c_atr_new(priv->client->adapter, priv->dev,
				&ds457_i2c_atr_ops, NUM_CAMS);
	
	if (!priv->atr)
		return -ENOMEM;

	i2c_atr_set_driver_data(priv->atr, priv);
	
	
	for (i=0; i < NUM_CAMS; i++)
	{
		struct ds457_cams *cam = &priv->cams[i];
		if (!cam->enabled)
			continue;
		
		ret = i2c_atr_add_adapter(priv->atr, cam->index, NULL, NULL);
		if (ret)
			goto err_add_adapters;
	}
	return 0;

err_add_adapters:
	ds457_i2c_atr_deinit(priv);
	return ret;
}

static int ds457_parse_dt(struct ds457 *priv)
{
	const char *cam_node_name = "cam";
	struct ds457_subdev_priv *sd_priv;
	struct fwnode_handle *fwnode;
	struct ds457_cams *cam;
	unsigned int i;
	u32 index;
	int ret;

	for (i = 0; i < priv->num_cams; i++) {
		cam = &priv->cams[i];
		cam->index = i;

	}

	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, cam_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		if (index >= NUM_CAMS) {
			dev_err(priv->dev, "Invalid cam number %u\n", index);
			return -EINVAL;
		}

		cam = &priv->cams[index];
		cam->enabled = true;

		if (index + 1 < priv->num_subdevs)
			continue;
		priv->num_subdevs = index + 1;
	}
	
	priv->sd_privs = devm_kcalloc(priv->dev, priv->num_subdevs,
				      sizeof(*priv->sd_privs), GFP_KERNEL);
	if (!priv->sd_privs)
		return -ENOMEM;

	device_for_each_child_node(priv->dev, fwnode) {
		struct device_node *of_node = to_of_node(fwnode);

		if (!of_node_name_eq(of_node, cam_node_name))
			continue;

		ret = fwnode_property_read_u32(fwnode, "reg", &index);
		if (ret) {
			dev_err(priv->dev, "Failed to read reg: %d\n", ret);
			continue;
		}

		sd_priv = &priv->sd_privs[index];
		sd_priv->fwnode = fwnode;
		sd_priv->priv = priv;
		sd_priv->index = index;
	}

	return 0;
}

static int ds457_read(struct ds457 *sensor, u16 reg, u16 *val)
{
	int ret = regmap_raw_read(sensor->regmap, reg, val, 2);
	if (ret < 0)
		dev_err(sensor->dev, "%s(): i2c read failed %d, 0x%04x\n",
				__func__, ret, reg);

	return ret;
}

static int ds457_raw_read(struct ds457 *sensor, u16 reg, void *val, size_t val_len)
{
	int ret = regmap_raw_read(sensor->regmap, reg, val, val_len);
	if (ret < 0)
		dev_err(sensor->dev, "%s(): i2c read failed %d, 0x%04x\n",
			__func__, ret, reg);

	return ret;
}

#define ds457_read_with_check(state, addr, val) {\
	if (ds457_read(state, addr, val))	\
		return -EINVAL; }

static int ds457_write(struct ds457 *sensor, u16 reg, u16 val)
{
	int ret;
	u8 value[2];

	value[1] = val >> 8;
	value[0] = val & 0x00FF;

	dev_dbg(sensor->dev,
			"%s(): writing to register: 0x%04x, value1: 0x%x, value2:0x%x\n",
			__func__, reg, value[1], value[0]);

	ret = regmap_raw_write(sensor->regmap, reg, value, sizeof(value));
	if (ret < 0)
		dev_err(sensor->dev,
				"%s(): i2c write failed %d, 0x%04x = 0x%x\n",
				__func__, ret, reg, val);
	
	return ret;
}

static int ds457_power_on(struct ds457 *priv)
{
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(priv->supplies),
				    priv->supplies);
	if (ret < 0)
		return ret;

	udelay(1);

	ret = gpiod_direction_output(priv->reset, 0);
	if (ret < 0)
		goto err_supply;

	udelay(1);

	ret = clk_prepare_enable(priv->clk);
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
	gpiod_direction_output(priv->reset, 1);
err_supply:
	regulator_bulk_disable(ARRAY_SIZE(priv->supplies), priv->supplies);
	return ret;
}

static void ds457_power_off(struct ds457 *priv)
{
	clk_disable_unprepare(priv->clk);
	gpiod_direction_output(priv->reset, 1);
	regulator_bulk_disable(ARRAY_SIZE(priv->supplies), priv->supplies);
}

/* Init setup is done for all the cameras of the module - all are set to max resolution 
and the corresponding max framerate */

static int ds457_init_setup (struct ds457 *sensor)
{
	u16 n_lanes = 2;
	u16 framerate = 30;
	int ret;


	ret = ds457_write(sensor, RS_D457_MIPI_LANE_NUMS, n_lanes - 1);
	if (ret)
		dev_err(sensor->dev, "[%s]: Failed to set number of lanes", __func__);

	ret = ds457_write(sensor, RS_D457_MIPI_LANE_DATARATE, MIPI_LANE_RATE);
	if (ret)
		dev_err(sensor->dev, "[%s]: Failed to set lane data rate", __func__);

		ret = ds457_write(sensor, RS_D457_DEPTH_STREAM_DT, CSI_DEPTH_DT);
	if (ret)
		dev_err(sensor->dev, "[%s]: Failed to set data type for depth module", __func__);
	
	ret = ds457_write(sensor, RS_D457_DEPTH_OVERRIDE, CSI_YUV422_DT);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set override format for depth module", __func__);
	
	ret = ds457_write(sensor, RS_D457_DEPTH_FPS, framerate);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init fps for RGB module", __func__);
	
	ret = ds457_write(sensor, RS_D457_DEPTH_RES_WIDTH, RS_D457_DEFAULT_WIDTH);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init width for RGB module", __func__);

	ret = ds457_write(sensor, RS_D457_DEPTH_RES_HEIGHT, RS_D457_DEFAULT_HEIGHT);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init height for RGB module", __func__);

	ret = ds457_write(sensor, RS_D457_RGB_STREAM_DT, CSI_YUV422_DT);
	if (ret)
		dev_err(sensor->dev, "[%s]: Failed to set data type for rgb module", __func__);
	
	ret = ds457_write(sensor, RS_D457_RGB_FPS, framerate);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init fps for RGB module", __func__);
	
	ret = ds457_write(sensor, RS_D457_RGB_RES_WIDTH, RS_D457_DEFAULT_WIDTH);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init width for RGB module", __func__);

	ret = ds457_write(sensor, RS_D457_RGB_RES_HEIGHT, RS_D457_DEFAULT_HEIGHT_RGB);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init height for RGB module", __func__);

	ret = ds457_write(sensor, RS_D457_IR_STREAM_DT, CSI_IR8_DT);
	if (ret)
		dev_err(sensor->dev, "[%s]: Failed to set init data type for IR module", __func__);
	
	ret = ds457_write(sensor, RS_D457_IR_OVERRIDE, CSI_RAW8_DT);
	if (ret)
		dev_err(sensor->dev, "[%s]: Failed to set override format for IR module", __func__);

	ret = ds457_write(sensor, RS_D457_IR_FPS, framerate);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init fps for IR module", __func__);
	
	ret = ds457_write(sensor, RS_D457_IR_RES_WIDTH, RS_D457_DEFAULT_WIDTH);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init width for IR module", __func__);

	ret = ds457_write(sensor, RS_D457_IR_RES_HEIGHT, RS_D457_DEFAULT_HEIGHT);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init height for IR module", __func__);

	// IMU has a default setup for width - height and the FPS is variable - max is 400
	ret = ds457_write(sensor, RS_D457_IMU_STREAM_DT, CSI_RAW8_DT);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init data type for IMU module", __func__);

	ret = ds457_write(sensor, RS_D457_IMU_RES_WIDTH, 38);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init width for IMU module", __func__);

	ret = ds457_write(sensor, RS_D457_IMU_RES_HEIGHT, 1);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init height for IMU module", __func__);

	ret = ds457_write(sensor, RS_D457_IMU_FPS, 400);
	if (ret)
	 	dev_err(sensor->dev, "[%s]: Failed to set init fps for IMU module", __func__);

	return ret;
}

static int ds457_stream_on_off(struct ds457_subdev_priv *sd_priv, unsigned int start_stop)
{
	struct ds457 *priv = sd_priv->priv;
	int ret = 0;
	unsigned int reg_addr_stream = RS_D457_START_STOP_STREAM;
	unsigned int cam_start_stop;
	unsigned int value_stream; 

	switch(sd_priv->index) {
		case 0:
			cam_start_stop = RS_D457_STREAM_DEPTH;
			break;
		case 1:
			cam_start_stop = RS_D457_STREAM_RGB;
			break;
		case 2:
			cam_start_stop = RS_D457_STREAM_IR;
			break;
		case 3:
			cam_start_stop = RS_D457_STREAM_IMU;
			break;
	}

	value_stream = (start_stop) ? (RS_D457_STREAM_START | cam_start_stop) : (RS_D457_STREAM_STOP | cam_start_stop); 
	ret = ds457_write(priv, reg_addr_stream, value_stream);
	if (ret)
	{
		if (start_stop)
			dev_err(priv->dev, "[%s]: Failed to start streaming on camera id %d", __func__, sd_priv->index);
		else
			dev_err(priv->dev, "[%s]: Failed to stop streaming on camera id %d", __func__, sd_priv->index);
	}
	return ret;
}

static int ds457_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;
	int ret;

	if (sd_priv->streaming == enable)
		return 0;

	mutex_lock(&priv->lock);

	if (!enable) {
		ret = ds457_stream_on_off(sd_priv,enable);

		pm_runtime_mark_last_busy(priv->dev);
		pm_runtime_put_autosuspend(priv->dev);

		sd_priv->streaming = false;

		goto unlock;
	}

	ret = pm_runtime_resume_and_get(priv->dev);
	if (ret < 0)
		goto unlock;

	/*
	 * Set streaming to true to ensure __v4l2_ctrl_handler_setup() will set
	 * the controls. The flag is reset to false further down if an error
	 * occurs.
	 */
	// set streaming to enable for each subdev_priv
	sd_priv->streaming = true;

	ret = ds457_stream_on_off(sd_priv,enable);
	if (ret)
		goto err_pm;

unlock:
	mutex_unlock(&priv->lock);

	return ret;

err_pm:
	/*
	 * In case of error, turn the power off synchronously as the device
	 * likely has no other chance to recover.
	 */
	pm_runtime_put_sync(priv->dev);
	sd_priv->streaming = false;

	goto unlock;
}

static int ds457_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;
	// the subdev's frame interval is set in s_frame_interval
	mutex_lock(&priv->lock);
	fi->interval = sd_priv->frame_interval;
	mutex_unlock(&priv->lock);
	return 0;
}

static inline bool val_is_in_range(unsigned int x, unsigned int min, unsigned int max)
{
	return x >= min && x <= max;
}

static unsigned int ds457_select_frame_interval(struct ds457_subdev_priv *sd_priv,
								struct v4l2_subdev_frame_interval *fi)
{
	unsigned int fps_val;

	if (sd_priv->index != 3)
		if (val_is_in_range(fi->interval.denominator, 1, 5))
			fps_val = 5;
		else if (val_is_in_range(fi->interval.denominator, 6, 15))
			fps_val = 15;
		else if (val_is_in_range(fi->interval.denominator, 16, 30))
			fps_val = 30;
		else if (val_is_in_range(fi->interval.denominator, 31, 60))
			fps_val = 60;
		else
			if (sd_priv->index == 0 || sd_priv->index == 2)
				fps_val = 90;
			else
				fps_val = 60;
	else
		if (val_is_in_range(fi->interval.denominator, 1, 50))
			fps_val = 50;
		else if (val_is_in_range(fi->interval.denominator, 51, 100))
			fps_val = 100;
		else if (val_is_in_range(fi->interval.denominator, 101, 200))
			fps_val = 200;
		else
			fps_val = 400;

	return fps_val;
	
}

static int ds457_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;
	int ret = 0;
	unsigned int fps, addr_fps;

	mutex_lock(&priv->lock);
	if (fi->pad != 0) {
		return -EINVAL;
		goto err;
	}

	if (sd_priv->streaming) {
		return -EBUSY;
		goto err;
	}

	fps = ds457_select_frame_interval(sd_priv, fi);
	
	sd_priv->frame_interval.numerator = 1;
	sd_priv->frame_interval.denominator = fps;

	switch(sd_priv->index) {
		case 0:
			addr_fps = RS_D457_DEPTH_FPS;
			break;
		case 1:
			addr_fps = RS_D457_RGB_FPS;
			break;
		case 2:
			addr_fps = RS_D457_IR_FPS;
			break;
		case 3:
			addr_fps = RS_D457_IMU_FPS;
			break;
	}

	ret = ds457_write(priv, addr_fps, fps);
	if (ret)
		dev_err(priv->dev, "(%s): Error when setting the new FPS value for camera with index: %d\n", __func__, sd_priv->index);

err:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ds457_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);

	if (sd_priv->index < 2)
		code->code = MEDIA_BUS_FMT_UYVY8_1X16;
	else
		code->code = MEDIA_BUS_FMT_SRGGB8_1X8;

	return 0;
}

static int ds457_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	unsigned int used_height = (sd_priv->index != 1) ? RS_D457_DEFAULT_HEIGHT : RS_D457_DEFAULT_HEIGHT_RGB;

	fse->min_width = RS_D457_DEFAULT_WIDTH;
	fse->max_width = fse->min_width;
	fse->min_height = used_height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ds457_enum_frame_interval(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;

	if (fie->pad != 0)
		return -EINVAL;

	tpf.numerator = 1;
	tpf.denominator = 30;

	fie->interval = tpf;
	return 0;
}

static int ds457_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;

	mutex_lock(&priv->lock);
	fmt->format = sd_priv->fmt;
	mutex_unlock(&priv->lock);

	return 0;
}

static int ds457_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *fmt)
{
	int ret;
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;
	const struct ds457_mode *mode;
	unsigned int addr_size_width, addr_size_height;

	mutex_lock(&priv->lock);

	// UYVY format for depth & RGB modules and RAW8 for Y8 and IMU ones
	if (sd_priv->index < 2)
		fmt->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	else
		fmt->format.code = MEDIA_BUS_FMT_SRGGB8_1X8;
	
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_SRGB;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	if (sd_priv->index != 1)
		mode = v4l2_find_nearest_size(supported_frame_sizes_depth_y8, ARRAY_SIZE(supported_frame_sizes_depth_y8),
							width, height,
							fmt->format.width, fmt->format.height);
	else
		mode = v4l2_find_nearest_size(supported_frame_sizes_rgb, ARRAY_SIZE(supported_frame_sizes_rgb),
							width, height,
							fmt->format.width, fmt->format.height);
	
	if (!mode) {
		ret = -EINVAL;
		goto complete;
	}

	switch(sd_priv->index) {
		case 0:
			addr_size_width = RS_D457_DEPTH_RES_WIDTH;
			addr_size_height = RS_D457_DEPTH_RES_HEIGHT;
			break;
		case 1:
			addr_size_width = RS_D457_RGB_RES_WIDTH;
			addr_size_height = RS_D457_RGB_RES_HEIGHT;
			break;
		case 2:
			addr_size_width = RS_D457_IR_RES_WIDTH;
			addr_size_height = RS_D457_IR_RES_HEIGHT;
			break;
	}

	// width and height are hardcoded for IMU
	if (sd_priv->index == 3)
	{
		fmt->format.width = 38;
		fmt->format.height = 1;
	}

	sd_priv->fmt = fmt->format;

	ret = ds457_write(priv, addr_size_width, fmt->format.width);
	if (ret)
		dev_err(priv->dev, "(%s): Error when setting the new width value for camera with index: %d\n", __func__, sd_priv->index);
	ret = ds457_write(priv, addr_size_height, fmt->format.height);
	if (ret)
		dev_err(priv->dev, "(%s): Error when setting the new height value for camera with index: %d\n", __func__, sd_priv->index);

complete:
	mutex_unlock(&priv->lock);
	return ret;
}

static int ds457_init_cfg(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	unsigned int used_height = (sd_priv->index != 1) ? RS_D457_DEFAULT_HEIGHT : RS_D457_DEFAULT_HEIGHT_RGB;
	struct v4l2_subdev_format format = {
			.format = {
				.width = RS_D457_DEFAULT_WIDTH,
				.height = used_height,
			},
	};
	
	ds457_set_format(sd, state, &format);
	
	return 0;
}

static const struct v4l2_subdev_video_ops ds457_subdev_video_ops = {
	.g_frame_interval = ds457_g_frame_interval,
	.s_frame_interval = ds457_s_frame_interval,
	.s_stream = ds457_s_stream,
};

static const struct v4l2_subdev_pad_ops ds457_subdev_pad_ops = {
	.enum_mbus_code = ds457_enum_mbus_code,
	.enum_frame_size = ds457_enum_frame_size,
	.enum_frame_interval = ds457_enum_frame_interval,
	.get_fmt = ds457_get_format,
	.set_fmt = ds457_set_format,
	.init_cfg = ds457_init_cfg,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ds457_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;
	int ret;

	ret = regmap_write(priv->regmap, reg->reg, reg->val);
	if (ret)
		return ret;
	
	return 0;
}

static int ds457_g_register(struct v4l2_subdev *sd,
			     struct v4l2_dbg_register *reg)
{
	struct ds457_subdev_priv *sd_priv = v4l2_get_subdevdata(sd);
	struct ds457 *priv = sd_priv->priv;
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, reg->reg, &val);
	if (ret)
		return ret;
	
	reg->val = val;
	reg->size = 1;

	return 0;
}
#endif

static const struct v4l2_subdev_core_ops ds457_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ds457_g_register,
	.s_register = ds457_s_register,
#endif
};

static const struct v4l2_subdev_ops ds457_subdev_ops = {
	.core = &ds457_core_ops,
	.video = &ds457_subdev_video_ops,
	.pad = &ds457_subdev_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static const struct regmap_config ds457_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xFFFF,
	.reg_format_endian = REGMAP_ENDIAN_NATIVE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

static const struct of_device_id ds457_of_match[] = {
	{ .compatible = "intel,rs_d457", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ds457_of_match);

static const struct media_entity_operations ds457_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int ds457_v4l2_register_sd(struct ds457_subdev_priv *sd_priv)
{
	struct ds457 *priv = sd_priv->priv;
	unsigned int index = sd_priv->index;
	char postfix[3];
	int ret;

	snprintf(postfix, sizeof(postfix), ":%d", index);

	v4l2_i2c_subdev_init(&sd_priv->sd, priv->client, &ds457_subdev_ops);
	v4l2_i2c_subdev_set_name(&sd_priv->sd, priv->client, NULL, postfix);
	sd_priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd_priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd_priv->sd.fwnode = sd_priv->fwnode;

	sd_priv->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd_priv->sd.entity, 1, &sd_priv->pad);
	if (ret < 0)
		goto error;

	v4l2_set_subdevdata(&sd_priv->sd, sd_priv);

	ret = v4l2_async_register_subdev(&sd_priv->sd);
	if (ret)
		goto error;

	return 0;

error:
	media_entity_cleanup(&sd_priv->sd.entity);
	fwnode_handle_put(sd_priv->sd.fwnode);

	return ret;
}

// unregister created subdev for d457 cameras

static void ds457_v4l2_unregister_sd(struct ds457_subdev_priv *sd_priv)
{
	v4l2_async_unregister_subdev(&sd_priv->sd);
	media_entity_cleanup(&sd_priv->sd.entity);
	fwnode_handle_put(sd_priv->sd.fwnode);
}

// register the subdevs to the physical device

static int ds457_v4l2_register(struct ds457 *priv)
{
	struct ds457_subdev_priv *sd_priv;
	int ret;

	for_each_subdev(priv, sd_priv) {
		ret = ds457_v4l2_register_sd(sd_priv);
		if (ret)
			return ret;
	}

	return 0;
}

static void ds457_v4l2_unregister(struct ds457 *priv)
{
	struct ds457_subdev_priv *sd_priv;

	for_each_subdev(priv, sd_priv)
		ds457_v4l2_unregister_sd(sd_priv);
}

static int ds457_probe(struct i2c_client *client)
{
	struct ds457 *priv;
	
	unsigned int i;
	int ret = 0;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
	mutex_init(&priv->lock);

	priv->dev = &client->dev;
	priv->client = client;
	i2c_set_clientdata(client, priv);

	// create cams structures
	priv->cams = devm_kcalloc(priv->dev, NUM_CAMS,
					sizeof(*priv->cams), GFP_KERNEL);
	if (!priv->cams)
		return -ENOMEM;

	priv->num_cams = NUM_CAMS;

	/* Acquire resources. */
	for (i = 0; i < ARRAY_SIZE(priv->supplies); ++i)
		priv->supplies[i].supply = ds457_supply_names[i];

	ret = devm_regulator_bulk_get(priv->dev, ARRAY_SIZE(priv->supplies),
				      priv->supplies);
	if (ret) {
		dev_err_probe(priv->dev, ret, "failed to get supplies\n");
		return ret;
	}

	priv->reset = devm_gpiod_get_optional(priv->dev, "reset",
						GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset))
		return dev_err_probe(priv->dev, PTR_ERR(priv->reset),
				     "failed to get reset GPIO\n");

	priv->clk = devm_clk_get(priv->dev, "xclk");
	if (IS_ERR(priv->clk))
		return dev_err_probe(priv->dev, PTR_ERR(priv->clk),
				     "failed to get clock\n");

	priv->xclk_freq = clk_get_rate(priv->clk);
	if (priv->xclk_freq != D457_24M_XCLK_FREQ) {
		dev_err(priv->dev, "xclk frequency not supported: %d Hz\n",
			priv->xclk_freq);
		return -EINVAL;
	}

	priv->regmap = devm_regmap_init_i2c(client, &ds457_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	ret = ds457_parse_dt(priv);
	if (ret)
		return ret;
	
	ret = ds457_i2c_atr_init(priv);
	if (ret)
		return ret;
	
	ret = ds457_power_on(priv);
	if (ret)
		return ret;

	/*
	 * Enable runtime PM. As the device has been powered manually, mark it
	 * as active, and increase the usage count without resuming the device.
	 */
	pm_runtime_set_active(priv->dev);
	pm_runtime_get_noresume(priv->dev);
	pm_runtime_enable(priv->dev);

	/* Register the V4L2 subdev. */
	ret = ds457_v4l2_register(priv);
	if (ret)
		goto err_pm;
	ret = ds457_init_setup(priv);
	if (ret)
		goto err_pm;
	
	/*
	 * Finally, enable autosuspend and decrease the usage count. The device
	 * will get suspended after the autosuspend delay, turning the power
	 * off.
	 */
	pm_runtime_set_autosuspend_delay(priv->dev, 1000);
	pm_runtime_use_autosuspend(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	return 0;

err_pm:
	pm_runtime_disable(priv->dev);
	pm_runtime_put_noidle(priv->dev);
	ds457_power_off(priv);
	mutex_destroy(&priv->lock);

	return 0;
}

static void ds457_remove(struct i2c_client *client)
{
	struct ds457 *priv = i2c_get_clientdata(client);

	ds457_v4l2_unregister(priv);

	ds457_i2c_atr_deinit(priv);

	/*
	 * Disable runtime PM. In case runtime PM is disabled in the kernel,
	 * make sure to turn power off manually.
	*/
	pm_runtime_disable(priv->dev);
	if (!pm_runtime_status_suspended(priv->dev))
		ds457_power_off(priv);
	pm_runtime_set_suspended(priv->dev);

	mutex_destroy(&priv->lock);
}

static struct i2c_driver ds457_i2c_driver = {
	.driver = {
		.of_match_table = ds457_of_match,
		.name = "ds457",
	},
	.probe_new = ds457_probe,
	.remove = ds457_remove,
};

module_i2c_driver(ds457_i2c_driver);

MODULE_DESCRIPTION("RealSense D4XX Camera Driver");
MODULE_AUTHOR("Guennadi Liakhovetski <guennadi.liakhovetski@intel.com>,\n\
				Nael Masalha <nael.masalha@intel.com>,\n\
				Alexander Gantman <alexander.gantman@intel.com>,\n\
				Emil Jahshan <emil.jahshan@intel.com>,\n\
				Xin Zhang <xin.x.zhang@intel.com>,\n\
				Qingwu Zhang <qingwu.zhang@intel.com>,\n\
				Evgeni Raikhel <evgeni.raikhel@intel.com>,\n\
				Shikun Ding <shikun.ding@intel.com>,\n\
				Dmitry Perchanov <dmitry.perchanov@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0.1.32");