/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2018 NXP
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define OV5640_VOLTAGE_ANALOG               2800000
#define OV5640_VOLTAGE_DIGITAL_CORE         1500000
#define OV5640_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV5640_XCLK_MIN 6000000
#define OV5640_XCLK_MAX 24000000

#define OV5640_CHIP_ID_HIGH_BYTE        0x300A
#define OV5640_CHIP_ID_LOW_BYTE         0x300B

#define OV5640_SENS_PAD_SOURCE	0
#define OV5640_SENS_PADS_NUM	1

enum ov5640_mode {
	ov5640_mode_MIN = 0,
	ov5640_mode_VGA_640_480 = 0,
	ov5640_mode_QVGA_320_240 = 1,
	ov5640_mode_480_272 = 2,
	ov5640_mode_720P_1280_720 = 3,
	ov5640_mode_1080P_1920_1080 = 4,
	ov5640_mode_MAX = 4
};

enum ov5640_frame_rate {
	ov5640_15_fps,
	ov5640_30_fps
};

static int ov5640_framerates[] = {
	[ov5640_15_fps] = 15,
	[ov5640_30_fps] = 30,
};

struct ov5640_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ov5640_mode_info {
	enum ov5640_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ov5640_pll_info {
	enum ov5640_mode mode;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ov5640_hs_info {
	u32 width;
	u32 height;
	u32 frame_rate;
	u32 val;
};

struct ov5640 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ov5640_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	struct media_pad pads[OV5640_SENS_PADS_NUM];
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(struct ov5640 *);
	int pwn_gpio, rst_gpio;
};

static struct reg_value ov5640_init_parm[] = {
	{0x3008, 0x42, 0, 0},

	/* System setting.0*/
	{0x3103, 0x03, 0, 0},
	{0x3000, 0x00, 0, 0},
	{0x3004, 0xff, 0, 0},
	{0x3002, 0x1c, 0, 0},
	{0x3006, 0xc3, 0, 0},
	{0x302e, 0x08, 0, 0},
	{0x3037, 0x13, 0, 0},
	{0x3108, 0x01, 0, 0},
	{0x3618, 0x00, 0, 0},
	{0x3612, 0x29, 0, 0},
	{0x3708, 0x64, 0, 0},
	{0x3709, 0x52, 0, 0},
	{0x370c, 0x03, 0, 0},
	{0x3820, 0x41, 0, 0},
	{0x3821, 0x07, 0, 0},
	{0x3630, 0x36, 0, 0},
	{0x3631, 0x0e, 0, 0},
	{0x3632, 0xe2, 0, 0},
	{0x3633, 0x12, 0, 0},
	{0x3621, 0xe0, 0, 0},
	{0x3704, 0xa0, 0, 0},
	{0x3703, 0x5a, 0, 0},
	{0x3715, 0x78, 0, 0},
	{0x3717, 0x01, 0, 0},
	{0x370b, 0x60, 0, 0},
	{0x3705, 0x1a, 0, 0},
	{0x3905, 0x02, 0, 0},
	{0x3906, 0x10, 0, 0},
	{0x3901, 0x0a, 0, 0},
	{0x3731, 0x12, 0, 0},
	{0x3600, 0x08, 0, 0},
	{0x3601, 0x33, 0, 0},
	{0x302d, 0x60, 0, 0},
	{0x3620, 0x52, 0, 0},
	{0x371b, 0x20, 0, 0},
	{0x471c, 0x50, 0, 0},
	{0x3a13, 0x43, 0, 0},
	{0x3a18, 0x00, 0, 0},
	{0x3a19, 0x7c, 0, 0},
	{0x3635, 0x13, 0, 0},
	{0x3636, 0x03, 0, 0},
	{0x3634, 0x40, 0, 0},
	{0x3622, 0x01, 0, 0},
	{0x3c01, 0x00, 0, 0},
	{0x3a00, 0x58, 0, 0},
	{0x4001, 0x02, 0, 0},
	{0x4004, 0x02, 0, 0},
	{0x4005, 0x1a, 0, 0},
	{0x5001, 0xa3, 0, 0},

	/* AEC */
	{0x3a0f, 0x30, 0, 0},
	{0x3a10, 0x28, 0, 0},
	{0x3a1b, 0x30, 0, 0},
	{0x3a1e, 0x26, 0, 0},
	{0x3a11, 0x60, 0, 0},
	{0x3a1f, 0x14, 0, 0},

	/* AWB */
	{0x5180, 0xff, 0, 0},
	{0x5181, 0xf2, 0, 0},
	{0x5182, 0x00, 0, 0},
	{0x5183, 0x14, 0, 0},
	{0x5184, 0x25, 0, 0},
	{0x5185, 0x24, 0, 0},
	{0x5186, 0x09, 0, 0},
	{0x5187, 0x09, 0, 0},
	{0x5188, 0x09, 0, 0},
	{0x5189, 0x88, 0, 0},
	{0x518a, 0x54, 0, 0},
	{0x518b, 0xee, 0, 0},
	{0x518c, 0xb2, 0, 0},
	{0x518d, 0x50, 0, 0},
	{0x518e, 0x34, 0, 0},
	{0x518f, 0x6b, 0, 0},
	{0x5190, 0x46, 0, 0},
	{0x5191, 0xf8, 0, 0},
	{0x5192, 0x04, 0, 0},
	{0x5193, 0x70, 0, 0},
	{0x5194, 0xf0, 0, 0},
	{0x5195, 0xf0, 0, 0},
	{0x5196, 0x03, 0, 0},
	{0x5197, 0x01, 0, 0},
	{0x5198, 0x04, 0, 0},
	{0x5199, 0x6c, 0, 0},
	{0x519a, 0x04, 0, 0},
	{0x519b, 0x00, 0, 0},
	{0x519c, 0x09, 0, 0},
	{0x519d, 0x2b, 0, 0},
	{0x519e, 0x38, 0, 0},

	/* Color Matrix */
	{0x5381, 0x1e, 0, 0},
	{0x5382, 0x5b, 0, 0},
	{0x5383, 0x08, 0, 0},
	{0x5384, 0x0a, 0, 0},
	{0x5385, 0x7e, 0, 0},
	{0x5386, 0x88, 0, 0},
	{0x5387, 0x7c, 0, 0},
	{0x5388, 0x6c, 0, 0},
	{0x5389, 0x10, 0, 0},
	{0x538a, 0x01, 0, 0},
	{0x538b, 0x98, 0, 0},

	/* sharp */
	{0x5300, 0x08, 0, 0},
	{0x5301, 0x30, 0, 0},
	{0x5302, 0x10, 0, 0},
	{0x5303, 0x00, 0, 0},
	{0x5304, 0x08, 0, 0},
	{0x5305, 0x30, 0, 0},
	{0x5306, 0x08, 0, 0},
	{0x5307, 0x16, 0, 0},
	{0x5309, 0x08, 0, 0},
	{0x530a, 0x30, 0, 0},
	{0x530b, 0x04, 0, 0},
	{0x530c, 0x06, 0, 0},

	/* Gamma */
	{0x5480, 0x01, 0, 0},
	{0x5481, 0x08, 0, 0},
	{0x5482, 0x14, 0, 0},
	{0x5483, 0x28, 0, 0},
	{0x5484, 0x51, 0, 0},
	{0x5485, 0x65, 0, 0},
	{0x5486, 0x71, 0, 0},
	{0x5487, 0x7d, 0, 0},
	{0x5488, 0x87, 0, 0},
	{0x5489, 0x91, 0, 0},
	{0x548a, 0x9a, 0, 0},
	{0x548b, 0xaa, 0, 0},
	{0x548c, 0xb8, 0, 0},
	{0x548d, 0xcd, 0, 0},
	{0x548e, 0xdd, 0, 0},
	{0x548f, 0xea, 0, 0},
	{0x5490, 0x1d, 0, 0},

	/* UV adjust. */
	{0x5580, 0x02, 0, 0},
	{0x5583, 0x40, 0, 0},
	{0x5584, 0x10, 0, 0},
	{0x5589, 0x10, 0, 0},
	{0x558a, 0x00, 0, 0},
	{0x558b, 0xf8, 0, 0},

	/* Lens correction0 */
	{0x5800, 0x23, 0, 0},
	{0x5801, 0x14, 0, 0},
	{0x5802, 0x0f, 0, 0},
	{0x5803, 0x0f, 0, 0},
	{0x5804, 0x12, 0, 0},
	{0x5805, 0x26, 0, 0},
	{0x5806, 0x0c, 0, 0},
	{0x5807, 0x08, 0, 0},
	{0x5808, 0x05, 0, 0},
	{0x5809, 0x05, 0, 0},
	{0x580a, 0x08, 0, 0},
	{0x580b, 0x0d, 0, 0},
	{0x580c, 0x08, 0, 0},
	{0x580d, 0x03, 0, 0},
	{0x580e, 0x00, 0, 0},
	{0x580f, 0x00, 0, 0},
	{0x5810, 0x03, 0, 0},
	{0x5811, 0x09, 0, 0},
	{0x5812, 0x07, 0, 0},
	{0x5813, 0x03, 0, 0},
	{0x5814, 0x00, 0, 0},
	{0x5815, 0x01, 0, 0},
	{0x5816, 0x03, 0, 0},
	{0x5817, 0x08, 0, 0},
	{0x5818, 0x0d, 0, 0},
	{0x5819, 0x08, 0, 0},
	{0x581a, 0x05, 0, 0},
	{0x581b, 0x06, 0, 0},
	{0x581c, 0x08, 0, 0},
	{0x581d, 0x0e, 0, 0},
	{0x581e, 0x29, 0, 0},
	{0x581f, 0x17, 0, 0},
	{0x5820, 0x11, 0, 0},
	{0x5821, 0x11, 0, 0},
	{0x5822, 0x15, 0, 0},
	{0x5823, 0x28, 0, 0},
	{0x5824, 0x46, 0, 0},
	{0x5825, 0x26, 0, 0},
	{0x5826, 0x08, 0, 0},
	{0x5827, 0x26, 0, 0},
	{0x5828, 0x64, 0, 0},
	{0x5829, 0x26, 0, 0},
	{0x582a, 0x24, 0, 0},
	{0x582b, 0x22, 0, 0},
	{0x582c, 0x24, 0, 0},
	{0x582d, 0x24, 0, 0},
	{0x582e, 0x06, 0, 0},
	{0x582f, 0x22, 0, 0},
	{0x5830, 0x40, 0, 0},
	{0x5831, 0x42, 0, 0},
	{0x5832, 0x24, 0, 0},
	{0x5833, 0x26, 0, 0},
	{0x5834, 0x24, 0, 0},
	{0x5835, 0x22, 0, 0},
	{0x5836, 0x22, 0, 0},
	{0x5837, 0x26, 0, 0},
	{0x5838, 0x44, 0, 0},
	{0x5839, 0x24, 0, 0},
	{0x583a, 0x26, 0, 0},
	{0x583b, 0x28, 0, 0},
	{0x583c, 0x42, 0, 0},
	{0x583d, 0xce, 0, 0},
	};

static struct reg_value ov5640_setting_VGA_640_480[] = {
	{0x3800, 0x00, 0, 0}, {0x3801, 0x00, 0, 0}, {0x3802, 0x00, 0, 0},
	{0x3803, 0x04, 0, 0}, {0x3804, 0x0a, 0, 0}, {0x3805, 0x3f, 0, 0},
	{0x3806, 0x07, 0, 0}, {0x3807, 0x9b, 0, 0}, {0x3808, 0x02, 0, 0},
	{0x3809, 0x80, 0, 0}, {0x380a, 0x01, 0, 0}, {0x380b, 0xe0, 0, 0},
	{0x380c, 0x07, 0, 0}, {0x380d, 0x68, 0, 0}, {0x380e, 0x03, 0, 0},
	{0x380f, 0xd8, 0, 0}, {0x3810, 0x00, 0, 0}, {0x3811, 0x10, 0, 0},
	{0x3812, 0x00, 0, 0}, {0x3813, 0x06, 0, 0}, {0x3814, 0x31, 0, 0},
	{0x3815, 0x31, 0, 0},
};

static struct reg_value ov5640_setting_QVGA_320_240[] = {
	{0x3800, 0x00, 0, 0}, {0x3801, 0x00, 0, 0}, {0x3802, 0x00, 0, 0},
	{0x3803, 0x04, 0, 0}, {0x3804, 0x0a, 0, 0}, {0x3805, 0x3f, 0, 0},
	{0x3806, 0x07, 0, 0}, {0x3807, 0x9b, 0, 0}, {0x3808, 0x01, 0, 0},
	{0x3809, 0x40, 0, 0}, {0x380a, 0x00, 0, 0}, {0x380b, 0xf0, 0, 0},
	{0x380c, 0x07, 0, 0}, {0x380d, 0x68, 0, 0}, {0x380e, 0x03, 0, 0},
	{0x380f, 0xd8, 0, 0}, {0x3810, 0x00, 0, 0}, {0x3811, 0x10, 0, 0},
	{0x3812, 0x00, 0, 0}, {0x3813, 0x06, 0, 0}, {0x3814, 0x31, 0, 0},
	{0x3815, 0x31, 0, 0},
};

static struct reg_value ov5640_setting_480_272[] = {
	{0x3800, 0x00, 0, 0}, {0x3801, 0x00, 0, 0}, {0x3802, 0x00, 0, 0},
	{0x3803, 0xfa, 0, 0}, {0x3804, 0x0a, 0, 0}, {0x3805, 0x3f, 0, 0},
	{0x3806, 0x06, 0, 0}, {0x3807, 0xa9, 0, 0}, {0x3808, 0x01, 0, 0},
	{0x3809, 0xE0, 0, 0}, {0x380a, 0x01, 0, 0}, {0x380b, 0x10, 0, 0},
	{0x380c, 0x07, 0, 0}, {0x380d, 0x64, 0, 0}, {0x380e, 0x02, 0, 0},
	{0x380f, 0xe4, 0, 0}, {0x3810, 0x00, 0, 0}, {0x3811, 0x10, 0, 0},
	{0x3812, 0x00, 0, 0}, {0x3813, 0x04, 0, 0}, {0x3814, 0x31, 0, 0},
	{0x3815, 0x31, 0, 0},
};

static struct reg_value ov5640_setting_720P_1280_720[] = {
	{0x3800, 0x00, 0, 0}, {0x3801, 0x00, 0, 0}, {0x3802, 0x00, 0, 0},
	{0x3803, 0xfa, 0, 0}, {0x3804, 0x0a, 0, 0}, {0x3805, 0x3f, 0, 0},
	{0x3806, 0x06, 0, 0}, {0x3807, 0xa9, 0, 0}, {0x3808, 0x05, 0, 0},
	{0x3809, 0x00, 0, 0}, {0x380a, 0x02, 0, 0}, {0x380b, 0xd0, 0, 0},
	{0x380c, 0x07, 0, 0}, {0x380d, 0x64, 0, 0}, {0x380e, 0x02, 0, 0},
	{0x380f, 0xe4, 0, 0}, {0x3810, 0x00, 0, 0}, {0x3811, 0x10, 0, 0},
	{0x3812, 0x00, 0, 0}, {0x3813, 0x04, 0, 0}, {0x3814, 0x31, 0, 0},
	{0x3815, 0x31, 0, 0},
};

static struct reg_value ov5640_setting_1080P_1920_1080[] = {
	{0x3800, 0x01, 0, 0}, {0x3801, 0x50, 0, 0}, {0x3802, 0x01, 0, 0},
	{0x3803, 0xb2, 0, 0}, {0x3804, 0x08, 0, 0}, {0x3805, 0xef, 0, 0},
	{0x3806, 0x05, 0, 0}, {0x3807, 0xf1, 0, 0}, {0x3808, 0x07, 0, 0},
	{0x3809, 0x80, 0, 0}, {0x380a, 0x04, 0, 0}, {0x380b, 0x38, 0, 0},
	{0x380c, 0x09, 0, 0}, {0x380d, 0xc4, 0, 0}, {0x380e, 0x04, 0, 0},
	{0x380f, 0x60, 0, 0}, {0x3810, 0x00, 0, 0}, {0x3811, 0x10, 0, 0},
	{0x3812, 0x00, 0, 0}, {0x3813, 0x04, 0, 0}, {0x3814, 0x11, 0, 0},
	{0x3815, 0x11, 0, 0},
};

/* MIPI */
static struct reg_value ov5640_mipi_pll_VGA_30fps_640_480[] = {
	{0x3035, 0x14, 0, 0}, {0x3036, 0x38, 0, 0}, {0x460c, 0x22, 0, 0},
	{0x3824, 0x02, 0, 0}, {0x4837, 0x0a, 0, 0},
};

static struct reg_value ov5640_mipi_pll_VGA_15fps_640_480[] = {
	{0x3035, 0x22, 0, 0}, {0x3036, 0x38, 0, 0}, {0x460c, 0x22, 0, 0},
	{0x3824, 0x02, 0, 0}, {0x4837, 0x0a, 0, 0},
};

static struct reg_value ov5640_mipi_pll_QVGA_30fps_320_240[] = {
	{0x3035, 0x14, 0, 0}, {0x3036, 0x38, 0, 0}, {0x460c, 0x22, 0, 0},
	{0x3824, 0x02, 0, 0}, {0x4837, 0x22, 0, 0},
};

static struct reg_value ov5640_mipi_pll_QVGA_15fps_320_240[] = {
	{0x3035, 0x22, 0, 0}, {0x3036, 0x38, 0, 0}, {0x460c, 0x22, 0, 0},
	{0x3824, 0x02, 0, 0}, {0x4837, 0x0a, 0, 0},
};

static struct reg_value ov5640_mipi_pll_30fps_480_272[] = {
	{0x3035, 0x21, 0, 0}, {0x3036, 0x69, 0, 0}, {0x460c, 0x20, 0, 0},
	{0x3824, 0x04, 0, 0}, {0x4837, 0x16, 0, 0},
};

static struct reg_value ov5640_mipi_pll_15fps_480_272[] = {
	{0x3035, 0x41, 0, 0}, {0x3036, 0x69, 0, 0}, {0x460c, 0x20, 0, 0},
	{0x3824, 0x04, 0, 0}, {0x4837, 0x16, 0, 0},
};

static struct reg_value ov5640_mipi_pll_720P_30fps_1280_720[] = {
	{0x3035, 0x21, 0, 0}, {0x3036, 0x54, 0, 0}, {0x460c, 0x20, 0, 0},
	{0x3824, 0x04, 0, 0}, {0x4837, 0x0a, 0, 0},
};

static struct reg_value ov5640_mipi_pll_720P_15fps_1280_720[] = {
	{0x3035, 0x41, 0, 0}, {0x3036, 0x54, 0, 0}, {0x460c, 0x20, 0, 0},
	{0x3824, 0x04, 0, 0}, {0x4837, 0x0a, 0, 0},
};

static struct reg_value ov5640_mipi_pll_1080P_15fps_1920_1080[] = {
	{0x3035, 0x21, 0, 0}, {0x3036, 0x54, 0, 0}, {0x460c, 0x20, 0, 0},
	{0x3824, 0x04, 0, 0}, {0x4837, 0x0a, 0, 0},
};

static struct ov5640_mode_info ov5640_mode_info_data[ov5640_mode_MAX + 1] = {
	{ov5640_mode_VGA_640_480, 640, 480, ov5640_setting_VGA_640_480,
	ARRAY_SIZE(ov5640_setting_VGA_640_480)},

	{ov5640_mode_QVGA_320_240, 320, 240, ov5640_setting_QVGA_320_240,
	ARRAY_SIZE(ov5640_setting_QVGA_320_240)},

	{ov5640_mode_480_272, 480, 272, ov5640_setting_480_272,
	ARRAY_SIZE(ov5640_setting_480_272)},

	{ov5640_mode_720P_1280_720, 1280, 720, ov5640_setting_720P_1280_720,
	ARRAY_SIZE(ov5640_setting_720P_1280_720)},

	{ov5640_mode_1080P_1920_1080, 1920, 1080, ov5640_setting_1080P_1920_1080,
	ARRAY_SIZE(ov5640_setting_1080P_1920_1080)},
};

static struct reg_value ov5640_mipi_config[] = {
	{0x302C, 0xc2, 0, 0}, /* Driver Capability */

	{0x4300, 0x3F, 0, 0}, /* YUV422 YVYU */
	{0x501f, 0x00, 0, 0}, /* YUV422 YVYU */

	{0x3034, 0x18, 0, 0}, /* MIPI 8bits mode */
	{0x3017, 0x00, 0, 0},
	{0x3018, 0x00, 0, 0},

	{0x300e, 0x45, 0, 0}, /* MIPI mode */
	{0x4800, 0x04, 0, 0},

	{0x4740, 0x23, 0, 0}, /* BIT5: Pixel BIT1: HSYNC BIT0: VSYNC */
						  /*   1: hight level */
						  /*   0: low level valid */
	{0x5000, 0xa7, 0, 0},
};

static struct ov5640_hs_info hs_setting[] = {
	{1920, 1080, 30, 0x0B},
	{1920, 1080, 15, 0x10},
	{1280, 720,  30, 0x11},
	{1280, 720,  15, 0x16},
	{640,  480,  30, 0x1E},
	{640,  480,  15, 0x23},
	{320,  240,  30, 0x1E},
	{320,  240,  15, 0x23},
};

static struct ov5640_pll_info ov5640_mipi_pll_info_data[2][ov5640_mode_MAX + 1] = {
	{
		{ov5640_mode_VGA_640_480, ov5640_mipi_pll_VGA_15fps_640_480,
		ARRAY_SIZE(ov5640_mipi_pll_VGA_15fps_640_480)},

		{ov5640_mode_QVGA_320_240, ov5640_mipi_pll_QVGA_15fps_320_240,
		ARRAY_SIZE(ov5640_mipi_pll_QVGA_15fps_320_240)},

		{ov5640_mode_480_272, ov5640_mipi_pll_15fps_480_272,
		ARRAY_SIZE(ov5640_mipi_pll_15fps_480_272)},

		{ov5640_mode_720P_1280_720, ov5640_mipi_pll_720P_15fps_1280_720,
		ARRAY_SIZE(ov5640_mipi_pll_720P_15fps_1280_720)},

		{ov5640_mode_1080P_1920_1080, ov5640_mipi_pll_1080P_15fps_1920_1080,
		ARRAY_SIZE(ov5640_mipi_pll_1080P_15fps_1920_1080)},
	},

	{
		{ov5640_mode_VGA_640_480, ov5640_mipi_pll_VGA_30fps_640_480,
		ARRAY_SIZE(ov5640_mipi_pll_VGA_30fps_640_480)},

		{ov5640_mode_QVGA_320_240, ov5640_mipi_pll_QVGA_30fps_320_240,
		ARRAY_SIZE(ov5640_mipi_pll_QVGA_30fps_320_240)},

		{ov5640_mode_480_272, ov5640_mipi_pll_30fps_480_272,
		ARRAY_SIZE(ov5640_mipi_pll_30fps_480_272)},

		{ov5640_mode_720P_1280_720, ov5640_mipi_pll_720P_30fps_1280_720,
		ARRAY_SIZE(ov5640_mipi_pll_720P_30fps_1280_720)},
	},
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;

static int ov5640_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov5640_remove(struct i2c_client *client);

static s32 ov5640_read_reg(struct ov5640 *sensor, u16 reg, u8 *val);
static s32 ov5640_write_reg(struct ov5640 *sensor, u16 reg, u8 val);

static const struct i2c_device_id ov5640_id[] = {
	{"ov5640_mipi_v3", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov5640_id);

static struct i2c_driver ov5640_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov5640_mipi_v3",
		  },
	.probe  = ov5640_probe,
	.remove = ov5640_remove,
	.id_table = ov5640_id,
};

static const struct ov5640_datafmt ov5640_colour_fmts[] = {
	{MEDIA_BUS_FMT_YVYU8_2X8, V4L2_COLORSPACE_JPEG},
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct ov5640 *to_ov5640(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov5640, subdev);
}

static uint16_t find_hs_configure(struct ov5640 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &sensor->streamcap.timeperframe;
	struct v4l2_pix_format *pix = &sensor->pix;
	u32 frame_rate = timeperframe->denominator / timeperframe->numerator;
	int i;

	for (i = 0; i < ARRAY_SIZE(hs_setting); i++) {
		if (hs_setting[i].width == pix->width &&
			hs_setting[i].height == pix->height &&
			hs_setting[i].frame_rate == frame_rate)
		  return hs_setting[i].val;
	}

	if (i == ARRAY_SIZE(hs_setting))
		dev_err(dev, "%s can not find hs configure\n", __func__);

	return -EINVAL;
}

/* Find a data format by a pixel code in an array */
static const struct ov5640_datafmt
			*ov5640_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5640_colour_fmts); i++)
		if (ov5640_colour_fmts[i].code == code)
			return ov5640_colour_fmts + i;

	return NULL;
}

static inline void ov5640_power_down(struct ov5640 *sensor, int enable)
{
	gpio_set_value_cansleep(sensor->pwn_gpio, enable);

	udelay(2000);
}

static inline void ov5640_reset(struct ov5640 *sensor)
{
	gpio_set_value_cansleep(sensor->pwn_gpio, 1);
	gpio_set_value_cansleep(sensor->rst_gpio, 0);
	udelay(5000);

	gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	udelay(1000);

	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	msleep(20);
}

static int ov5640_regulator_enable(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator,
				      OV5640_VOLTAGE_DIGITAL_IO,
				      OV5640_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		io_regulator = NULL;
		dev_warn(dev, "cannot get io voltage\n");
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OV5640_VOLTAGE_DIGITAL_CORE,
				      OV5640_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		core_regulator = NULL;
		dev_warn(dev, "cannot get core voltage\n");
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator,
				      OV5640_VOLTAGE_ANALOG,
				      OV5640_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret)
			dev_err(dev, "set analog voltage failed\n");
		 else
			dev_dbg(dev, "set analog voltage ok\n");
	} else {
		analog_regulator = NULL;
		dev_warn(dev, "cannot get analog voltage\n");
	}

	return ret;
}

static s32 ov5640_write_reg(struct ov5640 *sensor, u16 reg, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[3] = {0};

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(sensor->i2c_client, au8Buf, 3) < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -1;
	}

	return 0;
}

static s32 ov5640_read_reg(struct ov5640 *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

static int ov5640_set_clk_rate(struct ov5640 *sensor)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min_t(u32, tgt_xclk, (u32)OV5640_XCLK_MAX);
	tgt_xclk = max_t(u32, tgt_xclk, (u32)OV5640_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(sensor->sensor_clk, sensor->mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", sensor->mclk);
	return ret;
}

/* download ov5640 settings to sensor through i2c */
static int ov5640_download_firmware(struct ov5640 *sensor,
				struct reg_value *pModeSetting, s32 ArySize)
{
	register u32 Delay_ms = 0;
	register u16 RegAddr = 0;
	register u8 Mask = 0;
	register u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ov5640_read_reg(sensor, RegAddr, &RegVal);
			if (retval < 0)
				goto err;

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ov5640_write_reg(sensor, RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

static void ov5640_soft_reset(struct ov5640 *sensor)
{
	/* sysclk from pad */
	ov5640_write_reg(sensor, 0x3103, 0x11);

	/* software reset */
	ov5640_write_reg(sensor, 0x3008, 0x82);

	/* delay at least 5ms */
	msleep(10);
}

static int ov5640_config_init(struct ov5640 *sensor)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	/* Configure ov5640 initial parm */
	pModeSetting = ov5640_init_parm;
	ArySize = ARRAY_SIZE(ov5640_init_parm);

	retval = ov5640_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		return retval;

	return 0;
}

static int ov5640_config_resolution(struct ov5640 *sensor,
			enum ov5640_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	if (mode < ov5640_mode_MIN || mode > ov5640_mode_MAX)
		return -EINVAL;

	if (mode == ov5640_mode_1080P_1920_1080) {
		ov5640_write_reg(sensor, 0x3709, 0x12);
		ov5640_write_reg(sensor, 0x3821, 0x06);
	}

	if (mode == ov5640_mode_480_272) {
		mode = ov5640_mode_VGA_640_480;
		pr_warn("Not support 480*272, change to 640*480\n");
	}

	/* Configure ov5640 initial parm */
	pModeSetting = ov5640_mode_info_data[mode].init_data_ptr;
	ArySize = ov5640_mode_info_data[mode].init_data_size;

	retval = ov5640_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		return retval;

	return 0;
}

static int ov5640_config_others(struct ov5640 *sensor,
			enum ov5640_frame_rate rate, enum ov5640_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	int ArySize = 0, retval = 0;

	if (mode < ov5640_mode_MIN || mode > ov5640_mode_MAX)
		return -EINVAL;

	mode = (mode == ov5640_mode_480_272) ?
			ov5640_mode_VGA_640_480 : mode;
	pModeSetting = ov5640_mipi_pll_info_data[rate][mode].init_data_ptr;
	ArySize = ov5640_mipi_pll_info_data[rate][mode].init_data_size;
	retval = ov5640_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		return retval;

	/* Configure ov5640 initial parm */
	pModeSetting = ov5640_mipi_config;
	ArySize = ARRAY_SIZE(ov5640_mipi_config);
	retval = ov5640_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		return retval;

	return 0;
}

static void ov5640_start(struct ov5640 *sensor)
{
	ov5640_write_reg(sensor, 0x3008, 0x02);
	ov5640_write_reg(sensor, 0x3008, 0x02);
	udelay(1000);
}

static int ov5640_change_mode(struct ov5640 *sensor,
			enum ov5640_frame_rate rate, enum ov5640_mode mode)
{
	int retval;

	if (mode == ov5640_mode_1080P_1920_1080 && rate != ov5640_15_fps) {
		pr_warn("1080P only suppoert 15fps frame rate\n");
		rate = ov5640_15_fps;
	}

	retval = ov5640_config_resolution(sensor, mode);
	if (retval < 0) {
		pr_err("%s config resolution fail\n", __func__);
		return -EINVAL;
	}

	retval = ov5640_config_others(sensor, rate, mode);
	if (retval < 0) {
		pr_err("%s config others fail\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int init_device(struct ov5640 *sensor)
{
	int retval;

	ov5640_soft_reset(sensor);
	retval = ov5640_config_init(sensor);
	if (retval < 0)
		return retval;

	retval = ov5640_config_resolution(sensor, ov5640_mode_VGA_640_480);
	if (retval < 0)
		return retval;

	retval = ov5640_config_others(sensor, ov5640_30_fps,
				ov5640_mode_VGA_640_480);
	if (retval < 0)
		return retval;

	ov5640_start(sensor);

	return 0;
}

static void ov5640_stop(struct ov5640 *sensor)
{
	ov5640_write_reg(sensor, 0x3008, 0x42);
	udelay(1000);
}

/*!
 * ov5640_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ov5640_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *sensor = to_ov5640(client);

	if (on)
		clk_prepare_enable(sensor->sensor_clk);
	else
		clk_disable_unprepare(sensor->sensor_clk);

	sensor->on = on;
	return 0;
}

/*!
 * ov5640_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ov5640_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *sensor = to_ov5640(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ov5460_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ov5640_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *sensor = to_ov5640(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov5640_frame_rate frame_rate;
	enum ov5640_mode mode = a->parm.capture.capturemode;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		if (mode == ov5640_mode_1080P_1920_1080 && tgt_fps != 15) {
			pr_warn("1080P only suppoert 15fps frame rate\n");
			timeperframe->denominator = 15;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 30)
			frame_rate = ov5640_30_fps;
		else if (tgt_fps == 15)
			frame_rate = ov5640_15_fps;
		else {
			pr_err(" The camera frame rate is not supported!\n");
			goto error;
		}

		ret = ov5640_change_mode(sensor, frame_rate, mode);
		if (ret < 0)
			goto error;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = mode;
		sensor->pix.width = ov5640_mode_info_data[mode].width;
		sensor->pix.height = ov5640_mode_info_data[mode].height;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
					a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

error:
	return ret;
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *sensor = to_ov5640(client);

	if (enable)
		ov5640_start(sensor);
	else
		ov5640_stop(sensor);

	sensor->on = enable;
	return 0;
}

static int ov5640_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct ov5640_datafmt *fmt = ov5640_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *sensor = to_ov5640(client);

	if (format->pad)
		return -EINVAL;

	if (!fmt) {
		mf->code = ov5640_colour_fmts[1].code;
		mf->colorspace = ov5640_colour_fmts[1].colorspace;
	}

	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	sensor->fmt = fmt;

	return 0;
}

static int ov5640_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *sensor = to_ov5640(client);
	/*const struct ov5640_datafmt *fmt = sensor->fmt;*/

	if (format->pad)
		return -EINVAL;

	memset(mf, 0, sizeof(struct v4l2_mbus_framefmt));

	mf->code = ov5640_colour_fmts[1].code;
	mf->colorspace = ov5640_colour_fmts[1].colorspace;
	mf->width = sensor->pix.width;
	mf->height = sensor->pix.height;
	mf->field = V4L2_FIELD_NONE;
	mf->reserved[1] = find_hs_configure(sensor);

	dev_dbg(&client->dev, "%s code=0x%x, w/h=(%d,%d), colorspace=%d, field=%d\n",
		__func__, mf->code, mf->width, mf->height, mf->colorspace, mf->field);

	return 0;
}

static int ov5640_enum_code(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov5640_colour_fmts))
		return -EINVAL;

	code->code = ov5640_colour_fmts[code->index].code;
	return 0;
}

/*!
 * ov5640_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov5640_enum_framesizes(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > ov5640_mode_MAX)
		return -EINVAL;

	fse->max_width = ov5640_mode_info_data[fse->index].width;
	fse->min_width = fse->max_width;
	fse->max_height = ov5640_mode_info_data[fse->index].height;
	fse->min_height = fse->max_height;
	return 0;
}

/*!
 * ov5640_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov5640_enum_frameintervals(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_interval_enum *fie)
{
	int i, j, count;

	if (fie->index < 0 || fie->index > ov5640_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 ||
	    fie->code == 0) {
		pr_warn("Please assign pixel format, width and height.\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (i = 0; i < ARRAY_SIZE(ov5640_framerates); i++) {
		for (j = 0; j < (ov5640_mode_MAX + 1); j++) {
			if (fie->width == ov5640_mode_info_data[j].width
			 && fie->height == ov5640_mode_info_data[j].height
			 && ov5640_mode_info_data[j].init_data_ptr != NULL) {
				count++;
			}
			if (fie->index == (count - 1)) {
				fie->interval.denominator = ov5640_framerates[i];
				if (ov5640_mode_info_data[j].mode ==
						ov5640_mode_1080P_1920_1080)
					fie->interval.denominator = ov5640_framerates[0];
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int ov5640_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.g_parm = ov5640_g_parm,
	.s_parm = ov5640_s_parm,
	.s_stream = ov5640_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5640_subdev_pad_ops = {
	.enum_frame_size       = ov5640_enum_framesizes,
	.enum_frame_interval   = ov5640_enum_frameintervals,
	.enum_mbus_code        = ov5640_enum_code,
	.set_fmt               = ov5640_set_fmt,
	.get_fmt               = ov5640_get_fmt,
};

static struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.s_power	= ov5640_s_power,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core	= &ov5640_subdev_core_ops,
	.video	= &ov5640_subdev_video_ops,
	.pad	= &ov5640_subdev_pad_ops,
};

static const struct media_entity_operations ov5640_sd_media_ops = {
	.link_setup = ov5640_link_setup,
};

/*!
 * ov5640 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov5640_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	int retval;
	u8 chip_id_high, chip_id_low;
	struct ov5640 *sensor;

	sensor = kmalloc(sizeof(*sensor), GFP_KERNEL);
	/* Set initial values for the sensor struct. */
	memset(sensor, 0, sizeof(*sensor));

	/* ov5640 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* request power down pin */
	sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(sensor->pwn_gpio))
		dev_warn(dev, "No sensor pwdn pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
				GPIOF_OUT_INIT_HIGH, "ov5640_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	/* request reset pin */
	sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(sensor->rst_gpio))
		dev_warn(dev, "No sensor reset pin available");
	else {
		retval = devm_gpio_request_one(dev, sensor->rst_gpio,
				GPIOF_OUT_INIT_HIGH, "ov5640_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}

	/* Set initial values for the sensor struct. */
	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		/* assuming clock enabled by default */
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&(sensor->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(sensor->mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(sensor->csi));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}

	/* Set mclk rate before clk on */
	ov5640_set_clk_rate(sensor);

	retval = clk_prepare_enable(sensor->sensor_clk);
	if (retval < 0) {
		dev_err(dev, "%s: enable sensor clk fail\n", __func__);
		return -EINVAL;
	}

	sensor->io_init = ov5640_reset;
	sensor->i2c_client = client;

	sensor->pix.pixelformat = V4L2_PIX_FMT_UYVY;
	sensor->pix.width = ov5640_mode_info_data[0].width;
	sensor->pix.height =  ov5640_mode_info_data[0].height;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	ov5640_regulator_enable(&client->dev);

	ov5640_reset(sensor);

	retval = ov5640_read_reg(sensor, OV5640_CHIP_ID_HIGH_BYTE,
				&chip_id_high);
	if (retval < 0 || chip_id_high != 0x56) {
		clk_disable_unprepare(sensor->sensor_clk);
		pr_warn("camera ov5640 is not found\n");
		return -ENODEV;
	}
	retval = ov5640_read_reg(sensor, OV5640_CHIP_ID_LOW_BYTE,
				&chip_id_low);
	if (retval < 0 || chip_id_low != 0x40) {
		clk_disable_unprepare(sensor->sensor_clk);
		pr_warn("camera ov5640 is not found\n");
		return -ENODEV;
	}

	retval = init_device(sensor);
	if (retval < 0) {
		clk_disable_unprepare(sensor->sensor_clk);
		pr_warn("camera ov5640 init fail\n");
		return -ENODEV;
	}

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov5640_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[OV5640_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	retval = media_entity_pads_init(&sd->entity, OV5640_SENS_PADS_NUM,
							sensor->pads);
	sd->entity.ops = &ov5640_sd_media_ops;
	if (retval < 0)
		return retval;

	retval = v4l2_async_register_subdev(sd);
	if (retval < 0) {
		dev_err(&client->dev,
				"%s--Async register failed, ret=%d\n", __func__, retval);
		media_entity_cleanup(&sd->entity);
	}

	clk_disable_unprepare(sensor->sensor_clk);

	pr_info("%s camera mipi ov5640, is found\n", __func__);
	return retval;
}

/*!
 * ov5640 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov5640_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5640 *sensor = to_ov5640(client);

	v4l2_async_unregister_subdev(sd);

	clk_unprepare(sensor->sensor_clk);

	ov5640_power_down(sensor, 1);

	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	return 0;
}

module_i2c_driver(ov5640_i2c_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("OV5640 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("3.0");
MODULE_ALIAS("MIPI CSI");
