// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Omnivision OX05B1S RGB-IR camera.
 * Copyright (C) 2024, NXP
 *
 * Inspired from Sony imx219, imx290, imx214 and imx334 camera drivers
 *
 */

#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OX05B1S_SENS_PAD_SOURCE	0
#define OX05B1S_SENS_PADS_NUM	1

#define client_to_ox05b1s(client)\
	container_of(i2c_get_clientdata(client), struct ox05b1s, subdev)

#define OX05B1S_MAX_SIZES 4
struct ox05b1s_sizes {
	u32	code;
	u32	sizes_count;
	u32	sizes[OX05B1S_MAX_SIZES][2];
};

struct ox05b1s;
struct ox05b1s_plat_data {
	char				name[20];
	u32				chip_id;
	u32				native_width;
	u32				native_height;
	u32				active_top;
	u32				active_left;
	u32				active_width;
	u32				active_height;
	const struct ox05b1s_mode		*supported_modes;
	u32				supported_modes_count;
	u32				default_mode_index;
	const struct ox05b1s_sizes	*supported_codes;
	u32				supported_codes_count;
	const char * const		*hdr_modes;
	u32				hdr_modes_count;
	int (*set_hdr_mode)(struct ox05b1s *sensor, u32 hdr_mode);
};

struct ox05b1s_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hdr_mode;
};

struct ox05b1s_reg {
	u32 addr;
	u32 data;
};

#include "os08a20_regs_1080p.h"
#include "os08a20_regs_4k.h"
#include "os08a20_regs_4k_hdr.h"
#include "ox05b1s_regs_5mp.h"

struct ox05b1s_mode {
	u32 index;
	u32 width;
	u32 height;
	u32 code;
	u32 bpp;
	u32 vts; /* default VTS */
	u32 hts; /* default HTS */
	u32 exp; /* max exposure */
	bool h_bin; /* horizontal binning */
	u32 fps;
	struct ox05b1s_reg *reg_data;
	u32 reg_data_count;
};

struct ox05b1s {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct gpio_desc *rst_gpio;
	struct clk *sensor_clk;
	const struct ox05b1s_plat_data *model;
	struct v4l2_subdev subdev;
	struct media_pad pads[OX05B1S_SENS_PADS_NUM];
	const struct ox05b1s_mode *mode;
	struct mutex lock; /* sensor lock */
	u32 stream_status;
	struct ox05b1s_ctrls ctrls;
};

static struct ox05b1s_mode os08a20_supported_modes[] = {
	{
		/* 1080p BGGR10, no hdr, 60fps */
		.index		= 0,
		.width		= 1920,
		.height		= 1080,
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp		= 10,
		.vts		= 0x4a4,
		.hts		= 0x790,
		.exp		= 0x4a4 - 8,
		.h_bin		= true,
		.fps		= 60,
		.reg_data	= os08a20_init_setting_1080p,
		.reg_data_count	= ARRAY_SIZE(os08a20_init_setting_1080p),
	},
	{
		/* 4k BGGR10, staggered hdr VC0/VC1, 15fps */
		.index		= 1,
		.width		= 3840,
		.height		= 2160,
		.code		= MEDIA_BUS_FMT_SBGGR10_1X10,
		.bpp		= 10,
		.vts		= 0x90a,
		.hts		= 0x818,
		.exp		= 0x90a - 8,
		.h_bin		= false,
		.fps		= 15,
		.reg_data	= os08a20_init_setting_4k_hdr,
		.reg_data_count	= ARRAY_SIZE(os08a20_init_setting_4k_hdr),
	},
	{
		/* 4k BGGR12, no hdr, 30fps */
		.index		= 2,
		.width		= 3840,
		.height		= 2160,
		.code		= MEDIA_BUS_FMT_SBGGR12_1X12,
		.bpp		= 12,
		.vts		= 0x8f0,
		.hts		= 0x814,
		.exp		= 0x8f0 - 8,
		.h_bin		= false,
		.fps		= 30,
		.reg_data	= os08a20_init_setting_4k,
		.reg_data_count	= ARRAY_SIZE(os08a20_init_setting_4k),
	},
};

/* keep in sync with os08a20_supported_modes*/
static const struct ox05b1s_sizes os08a20_supported_codes[] = {
	{
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.sizes_count = 2,
		.sizes = { {1920, 1080}, {3840, 2160} }
	},
	{
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.sizes_count = 1,
		.sizes = { {3840, 2160} }
	},
};

static struct ox05b1s_mode ox05b1s_supported_modes[] = {
	{
		.index		= 0,
		.width		= 2592,
		.height		= 1944,
		.code		= MEDIA_BUS_FMT_SGRBG10_1X10,
		.bpp		= 10,
		.vts		= 0x850, /* 2128 */
		.hts		= 0x2f0, /* 752 */
		.exp		= 0x850 - 8,
		.h_bin		= false,
		.fps		= 30,
		.reg_data	= ovx5b_init_setting_2592x1944,
		.reg_data_count	= ARRAY_SIZE(ovx5b_init_setting_2592x1944),
	},
};

/* keep in sync with ox05b1s_supported_modes*/
static const struct ox05b1s_sizes ox05b1s_supported_codes[] = {
	{
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.sizes_count = 1,
		.sizes = { {2592, 1944} }
	},
};

static const struct regmap_config ox05b1s_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};

static int ox05b1s_power_on(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	/* get out of powerdown and reset */
	gpiod_set_value_cansleep(sensor->rst_gpio, 0);

	ret = clk_prepare_enable(sensor->sensor_clk);
	if (ret < 0)
		dev_err(dev, "Enable sensor clk fail ret=%d\n", ret);

	/* with XVCLK@24MHz, t2 = 6ms required delay for ox05b1s before first SCCB transaction */
	fsleep(6000);

	return ret;
}

static int ox05b1s_power_off(struct ox05b1s *sensor)
{
	gpiod_set_value_cansleep(sensor->rst_gpio, 1);

	if (!sensor->sensor_clk)
		return 0;

	/* XVCLK must be active for 512 cycles (0.34 ms at 24MHz) after last SCCB transaction */
	fsleep(350);
	clk_disable_unprepare(sensor->sensor_clk);

	return 0;
}

static int ox05b1s_runtime_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	return ox05b1s_power_off(sensor);
}

static int ox05b1s_runtime_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	return ox05b1s_power_on(sensor);
}

static int ox05b1s_write_reg(struct ox05b1s *sensor, u16 reg, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	ret = regmap_write(sensor->regmap, reg, val);
	if (ret < 0)
		dev_err(dev, "Failed to write reg addr 0x%04x with 0x%02x\n", reg, val);

	return ret;
}

static int ox05b1s_read_reg(struct ox05b1s *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	ret = regmap_raw_read(sensor->regmap, reg, val, 1);
	if (ret)
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, *val);

	return ret;
}

static int ox05b1s_update_bits(struct ox05b1s *sensor, u16 reg, unsigned int mask, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	ret = regmap_update_bits(sensor->regmap, reg, mask, val);
	if (ret < 0)
		dev_err(dev, "Failed to update reg addr 0x%04x with 0x%02x\n", reg, val);

	return ret;
}

#define OX05B1S_MAX_REG_BULK 16
static int ox05b1s_write_reg_array(struct ox05b1s *sensor,
				   struct ox05b1s_reg *reg_array,
				   u32 size)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct ox05b1s_reg *table = reg_array;
	u8 vals[OX05B1S_MAX_REG_BULK];
	int i, j;
	int ret;

	for (j = 0; j < size; j++) {
		table = &reg_array[j];
		for (i = 0; i < OX05B1S_MAX_REG_BULK; i++) {
			if (table[i].addr != (table[0].addr + i))
				break;
			vals[i] = table[i].data;
		}
		ret = regmap_bulk_write(sensor->regmap, table->addr, vals, i);
		if (ret) {
			dev_err(dev, "Failed to write reg addr=%x, count %d\n", table->addr, i);
			return ret;
		}
		j += i - 1;
	}

	return 0;
}

static const char * const os08a20_hdr_modes[] = {
	"NO HDR",		/* No HDR, single exposure */
	"HDR Staggered",	/* Staggered HDR mode, 2 exposures on separate virtual channels */
};

#define OS08A20_REG_CORE1		0x3661
#define OS08A20_STG_HDR_ALIGN_EN	BIT(0)

#define OS08A20_REG_FORMAT2		0x3821
#define OS08A20_STG_HDR_EN		BIT(5)

#define OS08A20_REG_MIPI_CTRL_13	0x4813
#define OS08A20_MISTERY_BIT3		BIT(3)

#define OS08A20_REG_MIPI_CTRL_6E	0x486e
#define OS08A20_MIPI_VC_ENABLE		BIT(2)

static int os08a20_enable_staggered_hdr(struct ox05b1s *sensor)
{
	int ret = 0;

	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_CORE1, OS08A20_STG_HDR_ALIGN_EN,
				   OS08A20_STG_HDR_ALIGN_EN);
	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_FORMAT2, OS08A20_STG_HDR_EN,
				   OS08A20_STG_HDR_EN);
	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_MIPI_CTRL_13, OS08A20_MISTERY_BIT3,
				   OS08A20_MISTERY_BIT3);
	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_MIPI_CTRL_6E, OS08A20_MIPI_VC_ENABLE,
				   OS08A20_MIPI_VC_ENABLE);

	return ret;
}

static int os08a20_disable_staggered_hdr(struct ox05b1s *sensor)
{
	int ret = 0;

	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_CORE1, OS08A20_STG_HDR_ALIGN_EN, 0);
	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_FORMAT2, OS08A20_STG_HDR_EN, 0);
	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_MIPI_CTRL_13, OS08A20_MISTERY_BIT3, 0);
	ret |= ox05b1s_update_bits(sensor, OS08A20_REG_MIPI_CTRL_6E, OS08A20_MIPI_VC_ENABLE, 0);

	return ret;
}

static int os08a20_set_hdr_mode(struct ox05b1s *sensor, u32 hdr_mode)
{
	switch (hdr_mode) {
	case 0:
		return os08a20_disable_staggered_hdr(sensor);
	case 1:
		return os08a20_enable_staggered_hdr(sensor);
	default:
		return -EINVAL;
	}
}

static int ox05b1s_set_hts(struct ox05b1s *sensor, u32 hts)
{
	int ret = 0;

	ret |= ox05b1s_write_reg(sensor, 0x380c, (u8)(hts >> 8) & 0xff);
	ret |= ox05b1s_write_reg(sensor, 0x380d, (u8)(hts & 0xff));

	return ret;
}

static int ox05b1s_set_vts(struct ox05b1s *sensor, u32 vts)
{
	int ret = 0;

	ret |= ox05b1s_write_reg(sensor, 0x380e, (u8)(vts >> 8) & 0xff);
	ret |= ox05b1s_write_reg(sensor, 0x380f, (u8)(vts & 0xff));

	return ret;
}

static int ox05b1s_set_exp(struct ox05b1s *sensor, u32 exp)
{
	int ret = 0;

	ret |= ox05b1s_write_reg(sensor, 0x3501, (exp >> 8) & 0xff);
	ret |= ox05b1s_write_reg(sensor, 0x3502, exp & 0xff);

	return ret;
}

static int ox05b1s_set_analog_gain(struct ox05b1s *sensor, u32 again)
{
	int ret = 0;

	/* real gain */
	ret |= ox05b1s_write_reg(sensor, 0x3508, (again >> 8) & 0xff);
	ret |= ox05b1s_write_reg(sensor, 0x3509, again & 0xff);

	return ret;
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ox05b1s,
			     ctrls.handler)->subdev;
}

static int ox05b1s_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	u32 w = sensor->mode->width;
	u32 h = sensor->mode->height;
	int ret = 0;

	/* apply V4L2 controls values only if power is already up */
	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	/* s_ctrl holds sensor lock */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = ox05b1s_set_vts(sensor, h + ctrl->val);
		break;
	case V4L2_CID_HBLANK:
		if (sensor->mode->h_bin)
			ret = ox05b1s_set_hts(sensor, w + ctrl->val);
		else
			ret = ox05b1s_set_hts(sensor, (w + ctrl->val) / 2);
		break;
	case V4L2_CID_PIXEL_RATE:
		/* Read-only, but we adjust it based on mode. */
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = ox05b1s_set_analog_gain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = ox05b1s_set_exp(sensor, ctrl->val);
		break;
	case V4L2_CID_HDR_SENSOR_MODE:
		if (sensor->model->set_hdr_mode)
			ret = sensor->model->set_hdr_mode(sensor, ctrl->val);
		else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ox05b1s_ctrl_ops = {
	.s_ctrl = ox05b1s_s_ctrl,
};

/*
 * MIPI CSI-2 link frequencies.
 * link_freq = (pixel_rate * bpp) / (2 * data_lanes)
 */
static const s64 ox05b1s_csi2_link_freqs[] = {
	200000000,
};

/* Link freq for default mode: 1080p RAW10, 4 data lanes 800 Mbps/lane. */
#define OX05B1S_DEFAULT_LINK_FREQ	0

static int ox05b1s_init_controls(struct ox05b1s *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ox05b1s_ctrl_ops;
	struct ox05b1s_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fwnode_device_properties props;
	int ret;

	v4l2_ctrl_handler_init(hdl, 7);

	/* we can use our own mutex for the ctrl lock */
	hdl->lock = &sensor->lock;

	/* Clock related controls */
	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, ops,
						  V4L2_CID_LINK_FREQ,
						  ARRAY_SIZE(ox05b1s_csi2_link_freqs) - 1,
						  OX05B1S_DEFAULT_LINK_FREQ,
						  ox05b1s_csi2_link_freqs);

	/* mode dependent, actual range set in ox05b1s_update_controls */
	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE,
					      0, 0, 1, 0);

	ctrls->hblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HBLANK,
					  0, 0, 1, 0);

	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VBLANK,
					  0, 0, 1, 0);

	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 0, 1, 0);

	ctrls->gain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
					0, 0xFFFF, 1, 0x80);

	if (sensor->model->hdr_modes)
		ctrls->hdr_mode = v4l2_ctrl_new_std_menu_items(hdl, ops, V4L2_CID_HDR_SENSOR_MODE,
							       sensor->model->hdr_modes_count - 1,
								0, 0, sensor->model->hdr_modes);
	else
		ctrls->hdr_mode = NULL;

	if (hdl->error) {
		ret = hdl->error;
		goto free_ctrls;
	}

	ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	ret = v4l2_fwnode_device_parse(dev, &props);
	if (ret)
		goto free_ctrls;

	ret = v4l2_ctrl_new_fwnode_properties(hdl, ops, &props);
	if (ret)
		goto free_ctrls;

	sensor->subdev.ctrl_handler = hdl;
	return 0;

free_ctrls:
	dev_err(dev, "Failed to init controls\n");
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static int ox05b1s_apply_current_mode(struct ox05b1s *sensor);

static int ox05b1s_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	int ret = 0;

	if (enable) {
		ret = pm_runtime_resume_and_get(&client->dev);
		if (ret < 0)
			return ret;
		ret = ox05b1s_apply_current_mode(sensor);
		if (!ret)
			ret = ox05b1s_write_reg(sensor, 0x0100, 0x01);
	} else {
		ret = ox05b1s_write_reg(sensor, 0x0100, 0x00);
	}

	sensor->stream_status = enable;

	if (!enable || ret) {
		pm_runtime_mark_last_busy(&sensor->i2c_client->dev);
		pm_runtime_put_autosuspend(&client->dev);
	}

	return 0;
}

static void ox05b1s_update_pad_format(struct ox05b1s *sensor,
				      const struct ox05b1s_mode *mode,
				      struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = mode->code;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ox05b1s_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct v4l2_mbus_framefmt *format;

	/* Initialize the format. */
	format = v4l2_subdev_state_get_format(state, 0);
	ox05b1s_update_pad_format(sensor, &sensor->model->supported_modes[0], format);

	return 0;
}

static int ox05b1s_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	if (code->index >= sensor->model->supported_codes_count)
		return -EINVAL;

	code->code = sensor->model->supported_codes[code->index].code;

	return 0;
}

static int ox05b1s_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	const struct ox05b1s_sizes *frame_sizes = NULL;
	int i;

	if (fse->pad != 0)
		return -EINVAL;

	for (i = 0; i < sensor->model->supported_codes_count; i++) {
		if (sensor->model->supported_codes[i].code == fse->code) {
			frame_sizes = &sensor->model->supported_codes[i];
			break;
		}
	}

	if (!frame_sizes)
		return -EINVAL;

	if (fse->index >= frame_sizes->sizes_count)
		return -EINVAL;

	fse->min_width = frame_sizes->sizes[fse->index][0];
	fse->max_width = fse->min_width;
	fse->min_height = frame_sizes->sizes[fse->index][1];
	fse->max_height = fse->min_height;

	return 0;
}

/* Update control ranges based on current streaming mode, needs sensor lock */
static int ox05b1s_update_controls(struct ox05b1s *sensor)
{
	int ret;
	struct device *dev = &sensor->i2c_client->dev;
	u32 hts = sensor->mode->hts;
	u32 hblank;
	u32 vts = sensor->mode->vts;
	u32 vblank = vts - sensor->mode->height;
	u32 fps = sensor->mode->fps;
	u64 pixel_rate = (sensor->mode->h_bin) ? hts * vts * fps : 2 * hts * vts * fps;
	u32 min_exp = 8;
	u32 max_exp = vts - 8;

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.pixel_rate, pixel_rate,
				       pixel_rate, 1, pixel_rate);
	if (ret) {
		dev_err(dev, "Modify range for ctrl: pixel_rate %llu-%llu failed\n",
			pixel_rate, pixel_rate);
		goto out;
	}

	if (sensor->mode->h_bin)
		hblank = hts - sensor->mode->width;
	else
		hblank = 2 * hts - sensor->mode->width;

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.hblank, hblank, hblank,
				       1, hblank);
	if (ret) {
		dev_err(dev, "Modify range for ctrl: hblank %u-%u failed\n",
			hblank, hblank);
		goto out;
	}
	__v4l2_ctrl_s_ctrl(sensor->ctrls.hblank, sensor->ctrls.hblank->default_value);

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.vblank, 0, vblank * 4,
				       1, vblank);
	if (ret) {
		dev_err(dev, "Modify range for ctrl: vblank %u-%u failed\n",
			vblank, vblank);
		goto out;
	}
	__v4l2_ctrl_s_ctrl(sensor->ctrls.vblank, sensor->ctrls.vblank->default_value);

	ret = __v4l2_ctrl_modify_range(sensor->ctrls.exposure, min_exp, max_exp,
				       1, max_exp / 2);
	if (ret) {
		dev_err(dev, "Modify range for ctrl: exposure %u-%u failed\n",
			min_exp, max_exp);
		goto out;
	}
	__v4l2_ctrl_s_ctrl(sensor->ctrls.exposure, sensor->ctrls.exposure->default_value);

out:
	return ret;
}

/* needs sensor lock and power on */
static int ox05b1s_apply_current_mode(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct ox05b1s_reg *reg_data = NULL;
	int ret = 0;

	ox05b1s_write_reg(sensor, 0x103, 0x01);

	reg_data = sensor->mode->reg_data;
	ret = ox05b1s_write_reg_array(sensor, reg_data,
				      sensor->mode->reg_data_count);
	if (ret)
		goto out;

	/* setup handler will write actual controls into sensor registers */
	ret =  __v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	if (ret)
		goto out;

out:
	if (ret < 0)
		dev_err(dev, "Failed to apply mode %dx%d,bpp=%d\n", sensor->mode->width,
			sensor->mode->height, sensor->mode->bpp);

	return ret;
}

/* similar with v4l2_find_nearest_size but filter for mbus code, needs sensor lock */
static const struct ox05b1s_mode *ox05b1s_nearest_size(const struct ox05b1s_mode *supported_modes,
						       u32 supported_modes_count,
						       struct v4l2_subdev_format *fmt)
{
	u32 error, min_error = U32_MAX;
	const struct ox05b1s_mode *best = NULL;
	unsigned int i;

	if (!supported_modes)
		return NULL;

	for (i = 0; i < supported_modes_count; i++) {
		const u32 w = supported_modes[i].width;
		const u32 h = supported_modes[i].height;

		if (supported_modes[i].code != fmt->format.code)
			continue;

		error = abs(w - fmt->format.width) + abs(h - fmt->format.height);
		if (error > min_error)
			continue;

		min_error = error;
		best = &supported_modes[i];
		if (!error)
			break;
	}

	return best;
}

/* get a valid mbus code for the model, either the requested one or the default one */
static u32 ox05b1s_find_code(const struct ox05b1s_plat_data *model, u32 code)
{
	u32 found_code = 0;
	unsigned int i;

	for (i = 0; i < model->supported_codes_count; i++) {
		if (model->supported_codes[i].code == code) {
			found_code = code;
			break;
		}
	}

	if (!found_code)
		found_code = model->supported_codes[model->default_mode_index].code;

	return found_code;
}

static int ox05b1s_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_mbus_framefmt *format;


	/* if no matching mbus code is found, use the one from the default mode */
	fmt->format.code = ox05b1s_find_code(sensor->model, fmt->format.code);
	sensor->mode = ox05b1s_nearest_size(sensor->model->supported_modes,
					    sensor->model->supported_modes_count, fmt);
	/* update controls that depend on current mode */
	ox05b1s_update_controls(sensor);

	fmt->format.width = sensor->mode->width;
	fmt->format.height = sensor->mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	format = v4l2_subdev_state_get_format(state,0);
	*format = fmt->format;

	dev_dbg(dev, "Set mode index=%d, %d x %d, code=0x%x\n", sensor->mode->index,
		fmt->format.width, fmt->format.height, fmt->format.code);


	return ret;
}

static u8 ox05b1s_code2dt(const u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		return MIPI_CSI2_DT_RAW12;
	default:
		return MIPI_CSI2_DT_RAW10;
	}
}

static int ox05b1s_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_frame_desc *fd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	fd->num_entries = 1;

	/* get sensor current code*/
	mutex_lock(&sensor->lock);
	fd->entry[0].pixelcode = sensor->mode->code;
	mutex_unlock(&sensor->lock);

	fd->entry[0].bus.csi2.vc = 0;
	fd->entry[0].bus.csi2.dt = ox05b1s_code2dt(fd->entry[0].pixelcode);

	return 0;
}

static int ox05b1s_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ox05b1s *sensor = client_to_ox05b1s(client);

	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = sensor->model->native_width;
		sel->r.height = sensor->model->native_height;
		return 0;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.top = sensor->model->active_top;
		sel->r.left = sensor->model->active_left;
		sel->r.width = sensor->model->active_width;
		sel->r.height = sensor->model->active_height;
		return 0;
	}

	return -EINVAL;
}

static const struct v4l2_subdev_video_ops ox05b1s_subdev_video_ops = {
	.s_stream = ox05b1s_s_stream,
};

static const struct v4l2_subdev_pad_ops ox05b1s_subdev_pad_ops = {
	.set_fmt		= ox05b1s_set_fmt,
	.get_fmt		= v4l2_subdev_get_fmt,
	.get_frame_desc		= ox05b1s_get_frame_desc,
	.enum_mbus_code		= ox05b1s_enum_mbus_code,
	.enum_frame_size	= ox05b1s_enum_frame_size,
	.get_selection		= ox05b1s_get_selection,
};

static const struct v4l2_subdev_ops ox05b1s_subdev_ops = {
	.video = &ox05b1s_subdev_video_ops,
	.pad   = &ox05b1s_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops ox05b1s_internal_ops = {
	.init_state = ox05b1s_init_state,
};

static void ox05b1s_get_gpios(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;

	sensor->rst_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(sensor->rst_gpio))
		dev_warn(dev, "No sensor reset pin available");
}

static int ox05b1s_read_chip_id(struct ox05b1s *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u32 chip_id = 0;
	u8 reg_val = 0;
	char *camera_name;
	int ret = 0;

	ret = ox05b1s_read_reg(sensor, 0x300a, &reg_val);
	chip_id |= reg_val << 16;
	ret |= ox05b1s_read_reg(sensor, 0x300b, &reg_val);
	chip_id |= reg_val << 8;
	ret |= ox05b1s_read_reg(sensor, 0x300c, &reg_val);
	chip_id |= reg_val;
	if (ret) {
		dev_err(dev, "Camera chip_id read error\n");
		return -ENODEV;
	}

	switch (chip_id) {
	case 0x530841:
		camera_name = "os08a20";
		break;
	case 0x580542:
		camera_name = "ox05b1s";
		break;
	default:
		camera_name = "unknown";
		break;
	}

	if (chip_id == sensor->model->chip_id) {
		dev_info(dev, "Camera %s detected, chip_id=%x\n", camera_name, chip_id);
	} else {
		dev_err(dev, "Detected %s camera (chip_id=%x), but expected %s (chip_id=%x)\n",
			camera_name, chip_id, sensor->model->name, sensor->model->chip_id);
		ret = -ENODEV;
	}

	return ret;
}

static int ox05b1s_probe(struct i2c_client *client)
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct ox05b1s *sensor;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->regmap = devm_regmap_init_i2c(client, &ox05b1s_regmap_config);
	if (IS_ERR(sensor->regmap)) {
		dev_err(dev, "Failed to allocate sensor register map\n");
		return PTR_ERR(sensor->regmap);
	}

	sensor->i2c_client = client;

	sensor->model = of_device_get_match_data(dev);

	ox05b1s_get_gpios(sensor);

	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		sensor->sensor_clk = NULL;
		dev_warn(dev, "Sensor csi_mclk is missing, using oscillator from sensor module\n");
	}

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &ox05b1s_subdev_ops);
	sd->internal_ops = &ox05b1s_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[OX05B1S_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity, OX05B1S_SENS_PADS_NUM,
					sensor->pads);
	if (retval)
		goto probe_out;

	mutex_init(&sensor->lock);

	retval = ox05b1s_init_controls(sensor);
	if (retval)
		goto probe_err_entity_cleanup;

	/* power on manually */
	retval = ox05b1s_power_on(sensor);
	if (retval) {
		dev_err(dev, "Failed to power on\n");
		goto probe_err_free_ctrls;
	}

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	retval = ox05b1s_read_chip_id(sensor);
	if (retval)
		goto probe_err_pm_runtime;

	v4l2_i2c_subdev_set_name(sd, client, sensor->model->name, NULL);

	/* Centrally managed subdev active state */
	sd->state_lock = &sensor->lock;
	retval = v4l2_subdev_init_finalize(sd);
	if (retval < 0) {
		dev_err(dev, "Subdev init error: %d\n", retval);
		goto probe_err_pm_runtime;
	}

	retval = v4l2_async_register_subdev_sensor(sd);
	if (retval < 0) {
		dev_err(&client->dev, "Async register failed, ret=%d\n", retval);
		goto probe_err_subdev_cleanup;
	}

	sensor->mode = &sensor->model->supported_modes[0];
	ox05b1s_update_controls(sensor);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put_autosuspend(dev);

	return 0;

probe_err_subdev_cleanup:
	v4l2_subdev_cleanup(sd);
probe_err_pm_runtime:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	ox05b1s_runtime_suspend(dev);
probe_err_free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
probe_err_entity_cleanup:
	media_entity_cleanup(&sd->entity);
probe_out:
	return retval;
}

static void ox05b1s_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ox05b1s *sensor = client_to_ox05b1s(client);
	struct device *dev = &client->dev;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		ox05b1s_runtime_suspend(dev);
	pm_runtime_set_suspended(dev);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
}

static const struct dev_pm_ops ox05b1s_pm_ops = {
	SET_RUNTIME_PM_OPS(ox05b1s_runtime_suspend,
			   ox05b1s_runtime_resume, NULL)
};

static const struct ox05b1s_plat_data os08a20_data = {
	.name			= "os08a20",
	.chip_id		= 0x530841,
	.native_width		= 3872, /* 16 dummy + 3840 active pixels + 16 dummy */
	.native_height		= 2192, /* 16 dummy + 2160 active lines + 16 dummy */
	.active_top		= 16,
	.active_left		= 16,
	.active_width		= 3840,
	.active_height		= 2160,
	.supported_modes	= os08a20_supported_modes,
	.supported_modes_count	= ARRAY_SIZE(os08a20_supported_modes),
	.default_mode_index	= 0,
	.supported_codes	= os08a20_supported_codes,
	.supported_codes_count	= ARRAY_SIZE(os08a20_supported_codes),
	.hdr_modes		= os08a20_hdr_modes,
	.hdr_modes_count	= ARRAY_SIZE(os08a20_hdr_modes),
	.set_hdr_mode		= os08a20_set_hdr_mode,

};

static const struct ox05b1s_plat_data ox05b1s_data = {
	.name			= "ox05b1s",
	.chip_id		= 0x580542,
	.native_width		= 2608, /* 8 dummy + 2592 active pixels + 8 dummy */
	.native_height		= 1960, /* 8 dummy + 1944 active lines + 8 dummy */
	.active_top		= 8,
	.active_left		= 8,
	.active_width		= 2592,
	.active_height		= 1944,
	.supported_modes	= ox05b1s_supported_modes,
	.supported_modes_count	= ARRAY_SIZE(ox05b1s_supported_modes),
	.default_mode_index	= 0,
	.supported_codes	= ox05b1s_supported_codes,
	.supported_codes_count	= ARRAY_SIZE(ox05b1s_supported_codes),
	.hdr_modes		= NULL,
	.hdr_modes_count	= 0,
	.set_hdr_mode		= NULL,
};

static const struct i2c_device_id ox05b1s_id[] = {
	{"ox05b1s", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ox05b1s_id);

static const struct of_device_id ox05b1s_of_match[] = {
	{
		.compatible = "ovti,os08a20",
		.data = &os08a20_data,
	},
	{
		.compatible = "ovti,ox05b1s",
		.data = &ox05b1s_data,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ox05b1s_of_match);

static struct i2c_driver ox05b1s_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "ox05b1s",
		.pm = &ox05b1s_pm_ops,
		.of_match_table	= ox05b1s_of_match,
	},
	.probe	= ox05b1s_probe,
	.remove = ox05b1s_remove,
	.id_table = ox05b1s_id,
};

module_i2c_driver(ox05b1s_i2c_driver);
MODULE_DESCRIPTION("Omnivision OX05B1S MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
