
/*
 * Driver for the addi903x camera sensor.
 *
 * Copyright (c) 2011-2019, The Linux Foundation. All rights reserved.
 * Copyright (C) 2018-2019 Analog Devices, All Rights Reserved.
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>
#include <linux/signal.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

static DEFINE_MUTEX(addi903x_lock);

#ifdef dev_dbg
	#undef dev_dbg
	#define dev_dbg dev_err
#endif

struct addi903x_mode_info {
	u32 width;
	u32 height;
	const struct addi903x_reg_value *data;
	u32 data_size;
	u32 pixel_clock;
	u32 link_freq;
};

struct addi903x {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct clk *xclk;

	const struct addi903x_mode_info *current_mode;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_clock;
	struct v4l2_ctrl *link_freq;

	/* Cached register values */
	u8 aec_pk_manual;
	u8 timing_tc_reg20;
	u8 timing_tc_reg21;

	/* lock to protect power state */
	struct mutex power_lock;
	int power_count;

	struct gpio_desc *rst_gpio;

	/* indicate that CCI driver hack should be used */
	bool use_cci;
	struct v4l2_subdev *cci;

	/* controls */
	struct v4l2_ctrl *set_chip_config;
	struct v4l2_ctrl *reg_read;
};

static inline struct addi903x *to_addi903x(struct v4l2_subdev *sd)
{
	return container_of(sd, struct addi903x, sd);
}

struct addi903x_reg_value {
	u16 reg;
	u16 val;
};

#define V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY  (V4L2_CID_USER_ADDI903X_BASE + 0)
#define V4L2_CID_AD_DEV_REG_READ_QUERY  (V4L2_CID_USER_ADDI903X_BASE + 1)

static const struct addi903x_reg_value addi903x_powerup_setting[] = {
	{ 0xc4c0, 0x001c },
	{ 0xc4c3, 0x001c },
	{ 0xc4d7, 0x0000 },
	{ 0xc4d5, 0x0002 },
	{ 0xc4da, 0x0001 },
	{ 0xc4f0, 0x0000 },
	{ 0xc427, 0x0003 },
	{ 0xc427, 0x0001 },
	{ 0xc427, 0x0000 },
	{ 0xc426, 0x0030 },
	{ 0xc426, 0x0010 },
	{ 0xc426, 0x0000 },
	{ 0xc423, 0x0080 },
	{ 0xc431, 0x0080 },
	{ 0x4001, 0x0007 },
	{ 0x7c22, 0x0004 }
};

static const struct addi903x_reg_value addi903x_powerdown_setting[] = {
	{ 0xc022, 0x0001 },
	{ 0x4001, 0x0006 },
	{ 0x7c22, 0x0004 },
	{ 0xc431, 0x0082 },
	{ 0xc423, 0x0000 },
	{ 0xc426, 0x0020 },
	{ 0xc427, 0x0002 },
	{ 0xc4c0, 0x003c },
	{ 0xc4c3, 0x003c },
	{ 0xc4d5, 0x0003 },
	{ 0xc4da, 0x0000 },
	{ 0xc4d7, 0x0001 },
	{ 0xc4f0, 0x0001 }
};

//=====================================================================

static const struct addi903x_reg_value addi903x_setting_vga[] = {

};

static const s64 link_freq[] = {
	147456000,//36864000,
	30060000
};

static const struct addi903x_mode_info addi903x_mode_info_data[] = {
	{
		.width = 640,
		.height = 960,
		.data = addi903x_setting_vga,
		.data_size = ARRAY_SIZE(addi903x_setting_vga),
		.pixel_clock = 73728000,//18432000,
		.link_freq = 0 /* an index in link_freq[] */
	},
	{
		.width = 668,
		.height = 750,
		.data = addi903x_setting_vga,
		.data_size = ARRAY_SIZE(addi903x_setting_vga),
		.pixel_clock = 15030000,
		.link_freq = 1 /* an index in link_freq[] */
	},
};

static int addi903x_write_reg(struct addi903x *addi903x, u16 reg, u16 *val)
{
	int ret;
	u16 i2c_addr = addi903x->i2c_client->addr;
	struct i2c_msg msg;
	u8 data[4];

	data[0] = (reg & 0xff00) >> 8;
	data[1] = reg & 0xff;
	data[2] = (*val & 0xff00) >> 8;
	data[3] = *val & 0xff;

	msg.addr = i2c_addr;
	msg.len = 4;
	msg.buf = data;
	msg.flags = 0;

	i2c_transfer(addi903x->i2c_client->adapter, &msg, 1);

	ret = i2c_master_send(addi903x->i2c_client, data, 4);
	if (ret < 0) {
		dev_err(addi903x->dev,
			"%s: i2c write reg error %d: reg=%x, val=%x\n",
			__func__, ret, reg, *val);
		return ret;
	}

	return ret;
}

static int addi903x_read_reg(struct addi903x *addi903x, u16 reg, u16 *val)
{
	int ret;
	u8 data[2];

	data[0] = (reg & 0xff00) >> 8;
	data[1] = reg & 0xff;

	ret = i2c_master_send(addi903x->i2c_client, data, 2);
	if (ret < 0) {
		dev_err(addi903x->dev, "%s: write reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(addi903x->i2c_client, (u8 *)val, 2);
	if (ret < 0) {
		dev_err(addi903x->dev, "%s: read reg error %d: reg=%x\n",
			__func__, ret, reg);
		return ret;
	}

	return 0;
}

static int addi903x_set_register_array(struct addi903x *addi903x,
				      const struct addi903x_reg_value *settings,
				      unsigned int num_settings)
{
	unsigned int i;
	int ret;
	u16 val;

	for (i = 0; i < num_settings; ++i, ++settings) {
		val = settings->val;
		ret = addi903x_write_reg(addi903x, settings->reg, &val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int addi903x_set_power_on(struct addi903x *addi903x)
{
	int ret;

	dev_info(addi903x->dev, "%s\n", __func__);

	ret = clk_prepare_enable(addi903x->xclk);
	if (ret < 0) {
		dev_err(addi903x->dev, "clk prepare enable failed\n");
		return ret;
	}

	gpiod_set_value_cansleep(addi903x->rst_gpio, 0);

	ret = addi903x_set_register_array(addi903x,
					  addi903x_powerup_setting,
					  ARRAY_SIZE(addi903x_powerup_setting));
	if (ret < 0)
		dev_err(addi903x->dev, "could not set power up registers\n");

	return ret;
}

static int addi903x_set_power_off(struct addi903x *addi903x)
{
	int ret = 0;

	dev_info(addi903x->dev, "%s\n", __func__);

	ret = addi903x_set_register_array(addi903x, addi903x_powerdown_setting,
					ARRAY_SIZE(addi903x_powerdown_setting));
	if (ret < 0)
		dev_err(addi903x->dev, "could not set power down registers\n");

	gpiod_set_value_cansleep(addi903x->rst_gpio, 1);
	clk_disable_unprepare(addi903x->xclk);

	return ret;
}

static int addi903x_s_power(struct v4l2_subdev *sd, int on)
{
	struct addi903x *addi903x = to_addi903x(sd);
	int ret = 0;

	mutex_lock(&addi903x->power_lock);

	dev_info(addi903x->dev, "s_power %d\n", on);
	/* If the power count is modified from 0 to != 0 or from != 0 to 0,
	 * update the power state.
	 */
	if (addi903x->power_count == !on) {
		if (on) {
			mutex_lock(&addi903x_lock);

			ret = addi903x_set_power_on(addi903x);
			if (ret < 0) {
				mutex_unlock(&addi903x_lock);
				goto exit;
			}

			mutex_unlock(&addi903x_lock);

			dev_info(addi903x->dev, "power on complete\n");
		} else {
			mutex_lock(&addi903x_lock);

			ret = addi903x_set_power_off(addi903x);
			if (ret < 0) {
				mutex_unlock(&addi903x_lock);
				goto exit;
			}

			mutex_unlock(&addi903x_lock);

			dev_info(addi903x->dev, "power off complete\n");
		}

	}

	/* Update the power count. */
	addi903x->power_count += on ? 1 : -1;
	WARN_ON(addi903x->power_count < 0);

exit:
	mutex_unlock(&addi903x->power_lock);

	return ret;
}

static int addi903x_set_chip_config(struct addi903x *addi903x,
				    struct v4l2_ctrl *ctrl)
{
	int ret, index = 0;
	unsigned short *reg, *val;
	int count = 0;

	reg = (unsigned short *)(ctrl->p_new.p_u16);
	val = ((unsigned short *)(ctrl->p_new.p_u16 + 1));

	while (index < ctrl->elems) {
		if (((*reg >= 0xC000) && (*reg <= 0xC7FF)) ||
		    ((*reg >= 0x4000) && (*reg <= 0x6FFF)) ||
		    ((*reg >= 0x7C00) && (*reg <= 0x7FFF))) {

			ret = addi903x_write_reg(addi903x, *reg, val);
			if (ret > 0) {
				count++;
			} else {
				dev_err(addi903x->dev,
					"could not write to device register %x\n",
					*reg);
				break;
			}
		}

		index += 2;
		reg += 2;
		val += 2;
	}
	dev_info(addi903x->dev,
		 "%s, total successful write count %d\n",
		 __func__, count);

	return 0;
}

static int addi903x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct addi903x *addi903x = container_of(ctrl->handler,
						 struct addi903x, ctrls);
	int ret = 0;
	u16 val;

	dev_info(addi903x->dev, ">> %s\n", __func__);

	switch (ctrl->id) {
	case V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY:
		dev_info(addi903x->dev,
			 "%s > V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY\n",
			 __func__);

		ret = addi903x_set_chip_config(addi903x, ctrl);
		break;

	case V4L2_CID_AD_DEV_REG_READ_QUERY:
		dev_info(addi903x->dev,
			 "%s > V4L2_CID_AD_DEV_REG_READ_QUERY\n", __func__);
		ret = addi903x_read_reg(addi903x, *(u16 *)(ctrl->p_new.p_u16),
					&val);
		*(u16 *)(ctrl->p_new.p_u16) = val;
		break;

	case V4L2_CID_PIXEL_RATE:
	case V4L2_CID_LINK_FREQ:
		break;
	default:
		dev_dbg(addi903x->dev,
			"%s > Unhandled: %x  param=%x\n", __func__, ctrl->id,
			ctrl->val);
		ret = -EINVAL;
		break;
	}

	dev_info(addi903x->dev, "<< %s\n", __func__);

	return ret;
}

static const struct v4l2_ctrl_ops addi903x_ctrl_ops = {
	.s_ctrl = addi903x_s_ctrl,
};

static const struct v4l2_ctrl_config addi903x_ctrl_chip_config = {
	.ops		= &addi903x_ctrl_ops,
	.id		= V4L2_CID_AD_DEV_SET_CHIP_CONFIG_QUERY,
	.name		= "chip_config",
	.type		= V4L2_CTRL_TYPE_U16,
	.def		= 0xFF,
	.min		= 0x00,
	.max		= 0xFFFF,
	.step		= 1,
	.dims		= { 2048 },
	.elem_size	= 2
};

static const struct v4l2_ctrl_config addi903x_ctrl_reg_read = {
	.ops	= &addi903x_ctrl_ops,
	.id	= V4L2_CID_AD_DEV_REG_READ_QUERY,
	.name	= "reg_read",
	.type	= V4L2_CTRL_TYPE_U16,
	.def	= 0,
	.min	= 0x00,
	.max	= 0xFFFF,
	.step	= 1
};

static int addi903x_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct addi903x *addi903x = to_addi903x(sd);

	dev_info(addi903x->dev, "Mbus code index %d\n", code->index);
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR12_1X12;

	return 0;
}

static int addi903x_enum_frame_size(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct addi903x *addi903x = to_addi903x(subdev);

	dev_info(addi903x->dev, "Mbus fs code %d index %d\n", fse->code,
		 fse->index);

	if (fse->code != MEDIA_BUS_FMT_SBGGR12_1X12)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(addi903x_mode_info_data))
		return -EINVAL;

	fse->min_width = addi903x_mode_info_data[fse->index].width;
	fse->max_width = addi903x_mode_info_data[fse->index].width;
	fse->min_height = addi903x_mode_info_data[fse->index].height;
	fse->max_height = addi903x_mode_info_data[fse->index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__addi903x_get_pad_format(struct addi903x *addi903x,
			  struct v4l2_subdev_pad_config *cfg,
			  unsigned int pad,
			  enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(&addi903x->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &addi903x->fmt;
	default:
		/* Returning null causes 0 derefence in addi903x_get_format */
		return &addi903x->fmt;
	}
}

static int addi903x_get_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	struct addi903x *addi903x = to_addi903x(sd);

	dev_info(addi903x->dev, "get_fmt which %d\n", format->which);
	format->format = *__addi903x_get_pad_format(addi903x, cfg, format->pad,
						    format->which);
	dev_info(addi903x->dev, "get_fmt: %x %dx%d\n",
		 format->format.code, format->format.width,
		 format->format.height);
	return 0;
}

static struct v4l2_rect *
__addi903x_get_pad_crop(struct addi903x *addi903x,
			struct v4l2_subdev_pad_config *cfg,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&addi903x->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &addi903x->crop;
	default:
		return NULL;
	}
}

static const struct addi903x_mode_info *
addi903x_find_nearest_mode(unsigned int width, unsigned int height)
{
	unsigned int i;

	for (i = ARRAY_SIZE(addi903x_mode_info_data) - 1; i >= 0; i--) {
		if (addi903x_mode_info_data[i].width <= width &&
		    addi903x_mode_info_data[i].height <= height)
			break;
	}

	if (i < 0)
		i = 0;

	return &addi903x_mode_info_data[i];
}

static int addi903x_set_format(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *format)
{
	struct addi903x *addi903x = to_addi903x(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	const struct addi903x_mode_info *new_mode;
	int ret;

	dev_info(addi903x->dev, "set_fmt: %x %dx%d\n",
		 format->format.code, format->format.width,
		 format->format.height);

	__crop = __addi903x_get_pad_crop(addi903x, cfg, format->pad,
					 format->which);

	new_mode = addi903x_find_nearest_mode(format->format.width,
					      format->format.height);
	__crop->width = new_mode->width;
	__crop->height = new_mode->height;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		ret = v4l2_ctrl_s_ctrl_int64(addi903x->pixel_clock,
					     new_mode->pixel_clock);
		if (ret < 0)
			return ret;

		ret = v4l2_ctrl_s_ctrl(addi903x->link_freq,
				       new_mode->link_freq);
		if (ret < 0)
			return ret;

		addi903x->current_mode = new_mode;
	}

	__format = __addi903x_get_pad_format(addi903x, cfg, format->pad,
					     format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;
	__format->code = MEDIA_BUS_FMT_SBGGR12_1X12;
	__format->field = V4L2_FIELD_NONE;
	__format->colorspace = V4L2_COLORSPACE_SRGB;

	format->format = *__format;

	return 0;
}

static int addi903x_entity_init_cfg(struct v4l2_subdev *subdev,
				    struct v4l2_subdev_pad_config *cfg)
{
	struct v4l2_subdev_format fmt = { 0 };
	struct addi903x *addi903x = to_addi903x(subdev);

	dev_info(addi903x->dev, "%s: Enter\n", __func__);

	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 640;
	fmt.format.height = 960;

	addi903x_set_format(subdev, cfg, &fmt);

	return 0;
}

static int addi903x_get_selection(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_selection *sel)
{
	struct addi903x *addi903x = to_addi903x(sd);

	dev_info(addi903x->dev, "get_selection %d\n", sel->target);
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__addi903x_get_pad_crop(addi903x, cfg, sel->pad,
					  sel->which);
	return 0;
}

static int addi903x_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct addi903x *addi903x = to_addi903x(subdev);
	int ret;

	dev_info(addi903x->dev, "stream %d\n", enable);
	if (enable) {
		ret = addi903x_set_register_array(addi903x,
					addi903x->current_mode->data,
					addi903x->current_mode->data_size);
		if (ret < 0) {
			dev_err(addi903x->dev, "could not set mode %dx%d\n",
				addi903x->current_mode->width,
				addi903x->current_mode->height);
			return ret;
		}
	} else {

	}

	return 0;
}

static const struct v4l2_subdev_core_ops addi903x_core_ops = {
	.s_power	= addi903x_s_power,
};

static const struct v4l2_subdev_video_ops addi903x_video_ops = {
	.s_stream	= addi903x_s_stream,
};

static const struct v4l2_subdev_pad_ops addi903x_subdev_pad_ops = {
	.init_cfg		= addi903x_entity_init_cfg,
	.enum_mbus_code		= addi903x_enum_mbus_code,
	.enum_frame_size	= addi903x_enum_frame_size,
	.get_fmt		= addi903x_get_format,
	.set_fmt		= addi903x_set_format,
	.get_selection		= addi903x_get_selection,
};

static const struct v4l2_subdev_ops addi903x_subdev_ops = {
	.core	= &addi903x_core_ops,
	.video	= &addi903x_video_ops,
	.pad	= &addi903x_subdev_pad_ops,
};

static int addi903x_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct addi903x *addi903x;
	u32 xclk_freq;
	int ret;

	dev_dbg(dev, "%s: Enter, i2c addr = 0x%x\n", __func__, client->addr);

	addi903x = devm_kzalloc(dev, sizeof(struct addi903x), GFP_KERNEL);
	if (!addi903x)
		return -ENOMEM;

	addi903x->i2c_client = client;
	addi903x->dev = dev;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
					 &addi903x->ep);

	of_node_put(endpoint);

	if (ret < 0) {
		dev_err(dev, "parsing endpoint node failed\n");
		return ret;
	}

	if (addi903x->ep.bus_type != V4L2_MBUS_CSI2) {
		dev_err(dev, "invalid bus type, must be MIPI CSI2\n");
		return -EINVAL;
	}

	/* get system clock (xclk) */
	addi903x->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(addi903x->xclk)) {
		dev_err(dev, "could not get xclk");
		return PTR_ERR(addi903x->xclk);
	}

	ret = of_property_read_u32(dev->of_node, "clock-frequency", &xclk_freq);
	if (ret) {
		dev_err(dev, "could not get xclk frequency\n");
		return ret;
	}

	ret = clk_set_rate(addi903x->xclk, xclk_freq);
	if (ret) {
		dev_err(dev, "could not set xclk frequency\n");
		return ret;
	}

	addi903x->rst_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	dev_info(dev, "Reset LOW\n");
	if (IS_ERR(addi903x->rst_gpio)) {
		dev_err(dev, "cannot get reset gpio\n");
		return PTR_ERR(addi903x->rst_gpio);
	}

	mutex_init(&addi903x->power_lock);

	v4l2_ctrl_handler_init(&addi903x->ctrls, 4);

	addi903x->pixel_clock = v4l2_ctrl_new_std(&addi903x->ctrls,
						  &addi903x_ctrl_ops,
						  V4L2_CID_PIXEL_RATE,
						  1, INT_MAX, 1, 1);
	addi903x->link_freq = v4l2_ctrl_new_int_menu(&addi903x->ctrls,
						     &addi903x_ctrl_ops,
						     V4L2_CID_LINK_FREQ,
						     ARRAY_SIZE(link_freq) - 1,
						     0, link_freq);
	if (addi903x->link_freq)
		addi903x->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;


	addi903x->set_chip_config = v4l2_ctrl_new_custom(&addi903x->ctrls,
						&addi903x_ctrl_chip_config,
						NULL);

	addi903x->reg_read = v4l2_ctrl_new_custom(&addi903x->ctrls,
						  &addi903x_ctrl_reg_read,
						  NULL);

	addi903x->sd.ctrl_handler = &addi903x->ctrls;

	if (addi903x->ctrls.error) {
		dev_err(dev, "%s: control initialization error %d\n",
			__func__, addi903x->ctrls.error);
		ret = addi903x->ctrls.error;
		goto free_ctrl;
	}

	client->flags |= I2C_CLIENT_SCCB;
	v4l2_i2c_subdev_init(&addi903x->sd, client, &addi903x_subdev_ops);
	addi903x->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	addi903x->pad.flags = MEDIA_PAD_FL_SOURCE;
	addi903x->sd.dev = &client->dev;
	addi903x->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = media_entity_pads_init(&addi903x->sd.entity, 1, &addi903x->pad);

	if (ret < 0) {
		dev_err(dev, "could not register media entity\n");
		goto free_ctrl;
	}

	ret = addi903x_s_power(&addi903x->sd, true);
	if (ret < 0) {
		dev_err(dev, "could not power up addi903x\n");
		goto free_entity;
	}

	dev_info(dev, "addi903x detected at address 0x%02x\n", client->addr);

	ret = addi903x_s_power(&addi903x->sd, false);
	if (ret < 0) {
		dev_err(dev, "could not power down addi903x\n");
		goto free_entity;
	}

	addi903x_entity_init_cfg(&addi903x->sd, NULL);

	ret = v4l2_async_register_subdev(&addi903x->sd);
	if (ret < 0)
		dev_err(dev, "could not register v4l2 device\n");

	return 0;

free_entity:
	media_entity_cleanup(&addi903x->sd.entity);
free_ctrl:
	v4l2_ctrl_handler_free(&addi903x->ctrls);
	mutex_destroy(&addi903x->power_lock);

	return ret;
}

static int addi903x_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct addi903x *addi903x = to_addi903x(sd);

	v4l2_async_unregister_subdev(&addi903x->sd);
	media_entity_cleanup(&addi903x->sd.entity);
	v4l2_ctrl_handler_free(&addi903x->ctrls);
	mutex_destroy(&addi903x->power_lock);

	return 0;
}

static const struct i2c_device_id addi903x_id[] = {
	{ "addi903x", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, addi903x_id);

static const struct of_device_id addi903x_of_match[] = {
	{ .compatible = "adi,addi903x" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, addi903x_of_match);

static struct i2c_driver addi903x_i2c_driver = {
	.driver			= {
		.of_match_table = of_match_ptr(addi903x_of_match),
		.name		= "addi903x",
	},
	.probe			= addi903x_probe,
	.remove			= addi903x_remove,
	.id_table		= addi903x_id,
};

module_i2c_driver(addi903x_i2c_driver);

MODULE_DESCRIPTION("Analog Devices ADDI903x Camera Driver");
MODULE_LICENSE("GPL v2");
