// SPDX-License-Identifier: GPL-2.0
/* Macros-related part of ds5.c + V4l2-related structures
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

#ifndef _RS_D457_H
#define _RS_D457_H

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/i2c-atr.h>
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

#define NUM_CAMS						4
#define RS_D457_DEFAULT_WIDTH			1280
#define RS_D457_DEFAULT_HEIGHT			720

#define RS_D457_DEFAULT_HEIGHT_RGB 		800

#define RS_D457_MIPI_SUPPORT_LINES		0x0300
#define RS_D457_MIPI_SUPPORT_PHY		0x0304
#define RS_D457_MIPI_DATARATE_MIN		0x0308
#define RS_D457_MIPI_DATARATE_MAX		0x030A
#define RS_D457_FW_VERSION				0x030C
#define RS_D457_FW_BUILD				0x030E
#define RS_D457_DEVICE_TYPE				0x0310
#define RS_D457_DEVICE_TYPE_D45X		6
#define RS_D457_DEVICE_TYPE_D43X		5
#define RS_D457_DEVICE_TYPE_D46X		4

#define RS_D457_MIPI_LANE_NUMS			0x0400
#define RS_D457_MIPI_LANE_DATARATE		0x0402
#define RS_D457_MIPI_CONF_STATUS		0x0500

#define RS_D457_START_STOP_STREAM		0x1000
#define RS_D457_DEPTH_STREAM_STATUS		0x1004
#define RS_D457_RGB_STREAM_STATUS		0x1008
#define RS_D457_IMU_STREAM_STATUS		0x100C
#define RS_D457_IR_STREAM_STATUS		0x1014

#define RS_D457_STREAM_DEPTH			0x0
#define RS_D457_STREAM_RGB				0x1
#define RS_D457_STREAM_IMU				0x2
#define RS_D457_STREAM_IR				0x4
#define RS_D457_STREAM_STOP				0x100
#define RS_D457_STREAM_START			0x200
#define RS_D457_STREAM_IDLE				0x1
#define RS_D457_STREAM_STREAMING		0x2

#define RS_D457_DEPTH_STREAM_DT			0x4000
#define RS_D457_DEPTH_STREAM_MD			0x4002
#define RS_D457_DEPTH_RES_WIDTH			0x4004
#define RS_D457_DEPTH_RES_HEIGHT		0x4008
#define RS_D457_DEPTH_FPS				0x400C
#define RS_D457_DEPTH_OVERRIDE			0x401C

#define RS_D457_RGB_STREAM_DT			0x4020
#define RS_D457_RGB_STREAM_MD			0x4022
#define RS_D457_RGB_RES_WIDTH			0x4024
#define RS_D457_RGB_RES_HEIGHT			0x4028
#define RS_D457_RGB_FPS					0x402C

#define RS_D457_IMU_STREAM_DT			0x4040
#define RS_D457_IMU_STREAM_MD			0x4042
#define RS_D457_IMU_RES_WIDTH			0x4044
#define RS_D457_IMU_RES_HEIGHT			0x4048
#define RS_D457_IMU_FPS					0x404C

#define RS_D457_IR_STREAM_DT			0x4080
#define RS_D457_IR_STREAM_MD			0x4082
#define RS_D457_IR_RES_WIDTH			0x4084
#define RS_D457_IR_RES_HEIGHT			0x4088
#define RS_D457_IR_FPS					0x408C
#define RS_D457_IR_OVERRIDE				0x409C

#define RS_D457_DEPTH_CONTROL_BASE		0x4100
#define RS_D457_RGB_CONTROL_BASE		0x4200
#define RS_D457_MANUAL_EXPOSURE_LSB		0x0000
#define RS_D457_MANUAL_EXPOSURE_MSB		0x0002
#define RS_D457_MANUAL_GAIN				0x0004
#define RS_D457_LASER_POWER				0x0008
#define RS_D457_AUTO_EXPOSURE_MODE		0x000C
#define RS_D457_EXPOSURE_ROI_TOP		0x0010
#define RS_D457_EXPOSURE_ROI_LEFT		0x0014
#define RS_D457_EXPOSURE_ROI_BOTTOM		0x0018
#define RS_D457_EXPOSURE_ROI_RIGHT		0x001C
#define RS_D457_MANUAL_LASER_POWER		0x0024
#define RS_D457_PWM_FREQUENCY			0x0028

#define RS_D457_DEPTH_CONFIG_STATUS		0x4800
#define RS_D457_RGB_CONFIG_STATUS		0x4802
#define RS_D457_IMU_CONFIG_STATUS		0x4804
#define RS_D457_IR_CONFIG_STATUS		0x4808

#define RS_D457_STATUS_STREAMING		0x1
#define RS_D457_STATUS_INVALID_DT		0x2
#define RS_D457_STATUS_INVALID_RES		0x4
#define RS_D457_STATUS_INVALID_FPS		0x8

#define MIPI_LANE_RATE					1000
#define CSI_YUV422_DT					0x001e
#define CSI_DEPTH_DT					0x0031
#define CSI_IR8_DT						0x0032
#define CSI_RAW8_DT             		0x002a
#define CSI_DT_EMBED            		0x0012

#define MAX_DEPTH_EXP					200000
#define MAX_RGB_EXP						10000
#define DEF_DEPTH_EXP					33000
#define DEF_RGB_EXP						1660

#define D457_24M_XCLK_FREQ				24000000

#define PLUS_10(x)  ((x)+(x)/10)

static const char * const ds457_supply_names[] = {
	"dvdd",
};

struct ds457_asd {
	struct v4l2_async_subdev base;
	struct ds457_subdev_priv *sd_priv;
};

struct ds457_subdev_priv {
	struct fwnode_handle *fwnode;
	struct media_pad pad;
	struct v4l2_subdev sd;
	struct v4l2_fract frame_interval;
	struct v4l2_mbus_framefmt fmt;

	struct ds457 *priv;
	// cam_id (maps to same VC_ID) represents the index of the camera from RealSense D457 module:
	// 0 - for depth sensor,
	// 1 - for RGB sensor,
	// 2 - for luma sensor,
	// 3 - for IMU
	bool streaming;
	unsigned int index;
};

struct ds457_cams {
	unsigned int index;
	bool enabled;
};

struct ds457 {
	struct device *dev;
	struct i2c_client *client;
	struct i2c_atr *atr;

	struct ds457_cams *cams;
	
	struct clk *clk;
	u32 xclk_freq;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ds457_supply_names)];
	struct gpio_desc *reset;
	struct regmap *regmap;

	int trigger_mode;
	unsigned int num_subdevs;
	// max number of cams
	unsigned int num_cams;

	struct v4l2_ctrl_handler ctrls;
	struct mutex lock;
	struct ds457_subdev_priv *sd_privs;
};

struct ds457_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;
};

static const struct ds457_mode supported_frame_sizes_rgb[] = {
	{
		.width = 1280,
		.height = 800,
	},
	{
		.width = 1280,
		.height = 720,
	},
	{
		.width = 848,
		.height = 480,
	},
	{
		.width = 640,
		.height = 480,
	},
	{
		.width = 640,
		.height = 360,
	},
	{
		.width = 480,
		.height = 270,
	},
	{
		.width = 424,
		.height = 240,
	}
};

static const struct ds457_mode supported_frame_sizes_depth_y8[] = {
	{
		.width = 1280,
		.height = 720,
	},
	{
		.width = 848,
		.height = 480,
	},
	{
		.width = 640,
		.height = 480,
	},
	{
		.width = 640,
		.height = 360,
	},
	{
		.width = 480,
		.height = 270,
	},
	{
		.width = 424,
		.height = 240,
	}
};

#endif