// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * mt9m114.c Aptina MT9M114 sensor driver
 *
 * Copyright (c) 2012 Analog Devices Inc.
 * Copyright 2022 NXP
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define MT9M114_PIXEL_ARRAY_TOP				0
#define MT9M114_PIXEL_ARRAY_LEFT			0
#define MT9M114_PIXEL_ARRAY_WIDTH			1280
#define MT9M114_PIXEL_ARRAY_HEIGHT			720

/* Sysctl registers */
#define MT9M114_CHIP_ID					0x0000
#define MT9M114_COMMAND_REGISTER			0x0080
#define MT9M114_COMMAND_REGISTER_APPLY_PATCH		(1 << 0)
#define MT9M114_COMMAND_REGISTER_SET_STATE		(1 << 1)
#define MT9M114_COMMAND_REGISTER_REFRESH		(1 << 2)
#define MT9M114_COMMAND_REGISTER_WAIT_FOR_EVENT		(1 << 3)
#define MT9M114_COMMAND_REGISTER_OK			(1 << 15)
#define MT9M114_SOFT_RESET				0x001a
#define MT9M114_PAD_SLEW				0x001e
#define MT9M114_PAD_CONTROL				0x0032

/* XDMA registers */
#define MT9M114_ACCESS_CTL_STAT				0x0982
#define MT9M114_PHYSICAL_ADDRESS_ACCESS			0x098a
#define MT9M114_LOGICAL_ADDRESS_ACCESS			0x098e

/* Core registers */
#define MT9M114_RESET_REGISTER				0x301a
#define MT9M114_FLASH					0x3046
#define MT9M114_CUSTOMER_REV				0x31fe

/* Camera Control registers */
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_START		0xc800
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_START		0xc802
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_END		0xc804
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_END		0xc806
#define MT9M114_CAM_SENSOR_CFG_PIXCLK			0xc808
#define MT9M114_CAM_SENSOR_CFG_ROW_SPEED		0xc80c
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN	0xc80e
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX	0xc810
#define MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES	0xc812
#define MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK		0xc814
#define MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION		0xc816
#define MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW		0xc818
#define MT9M114_CAM_SENSOR_CFG_REG_0_DATA		0xc826
#define MT9M114_CAM_SENSOR_CONTROL_READ_MODE		0xc834
#define MT9M114_CAM_CROP_WINDOW_XOFFSET			0xc854
#define MT9M114_CAM_CROP_WINDOW_YOFFSET			0xc856
#define MT9M114_CAM_CROP_WINDOW_WIDTH			0xc858
#define MT9M114_CAM_CROP_WINDOW_HEIGHT			0xc85a
#define MT9M114_CAM_CROP_CROPMODE			0xc85c
#define MT9M114_CAM_OUTPUT_WIDTH			0xc868
#define MT9M114_CAM_OUTPUT_HEIGHT			0xc86a
#define MT9M114_CAM_OUTPUT_FORMAT			0xc86c
#define MT9M114_CAM_OUTPUT_FORMAT_YUV			0xc86e
#define MT9M114_CAM_AET_AEMODE				0xc878
#define MT9M114_CAM_AET_MAX_FRAME_RATE			0xc88c
#define MT9M114_CAM_AET_MIN_FRAME_RATE			0xc88e
#define MT9M114_CAM_AWB_AWB_XSCALE			0xc8f2
#define MT9M114_CAM_AWB_AWB_YSCALE			0xc8f3
#define MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ		0xc904
#define MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ		0xc906
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART		0xc914
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART		0xc916
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND		0xc918
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND		0xc91a
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART	0xc91c
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART	0xc91e
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND		0xc920
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND		0xc922
#define MT9M114_CAM_SYSCTL_PLL_ENABLE			0xc97e
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N		0xc980
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_P		0xc982
#define MT9M114_CAM_PORT_OUTPUT_CONTROL			0xc984

/* System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE			0xdc00
#define MT9M114_SYSMGR_CURRENT_STATE			0xdc01
#define MT9M114_SYSMGR_CMD_STATUS			0xdc02

/* Patch Loader registers */
#define MT9M114_PATCHLDR_LOADER_ADDRESS			0xe000
#define MT9M114_PATCHLDR_PATCH_ID			0xe002
#define MT9M114_PATCHLDR_FIRMWARE_ID			0xe004
#define MT9M114_PATCHLDR_APPLY_STATUS			0xe008
#define MT9M114_PATCHLDR_NUM_PATCHES			0xe009
#define MT9M114_PATCHLDR_PATCH_ID_0			0xe00a
#define MT9M114_PATCHLDR_PATCH_ID_1			0xe00c
#define MT9M114_PATCHLDR_PATCH_ID_2			0xe00e
#define MT9M114_PATCHLDR_PATCH_ID_3			0xe010
#define MT9M114_PATCHLDR_PATCH_ID_4			0xe012
#define MT9M114_PATCHLDR_PATCH_ID_5			0xe014
#define MT9M114_PATCHLDR_PATCH_ID_6			0xe016
#define MT9M114_PATCHLDR_PATCH_ID_7			0xe018

/* SYS_STATE values (for SYSMGR_NEXT_STATE and SYSMGR_CURRENT_STATE) */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE		0x28
#define MT9M114_SYS_STATE_STREAMING			0x31
#define MT9M114_SYS_STATE_START_STREAMING		0x34
#define MT9M114_SYS_STATE_ENTER_SUSPEND			0x40
#define MT9M114_SYS_STATE_SUSPENDED			0x41
#define MT9M114_SYS_STATE_ENTER_STANDBY			0x50
#define MT9M114_SYS_STATE_STANDBY			0x52
#define MT9M114_SYS_STATE_LEAVE_STANDBY			0x54

/* Result status of last SET_STATE comamnd */
#define MT9M114_SET_STATE_RESULT_ENOERR			0x00
#define MT9M114_SET_STATE_RESULT_EINVAL			0x0c
#define MT9M114_SET_STATE_RESULT_ENOSPC			0x0d

#define MT9M114_SENS_PAD_SOURCE	0
#define MT9M114_SENS_PADS_NUM	1
#define MAX_FRAME_RATE 30

enum {
	MT9M114_QVGA,
	MT9M114_VGA,
	MT9M114_WVGA,
	MT9M114_720P,
};

struct mt9m114_resolution {
	u32 width;
	u32 height;
	u32 hact;
	u32 vact;
};

struct mt9m114_reg {
	u16 reg;
	u32 val;
	s32 width;
};

struct mt9m114_format {
	u32 mbus_code;
	enum v4l2_colorspace colorspace;
};

struct mt9m114 {
	struct v4l2_subdev sd;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_fract frame_interval;
	struct media_pad pads[MT9M114_SENS_PADS_NUM];

	struct i2c_client *i2c_client;
	const struct mt9m114_resolution *curr_mode;
	const struct mt9m114_resolution *last_mode;

	/* lock to protect all members below */
	struct mutex lock;

	struct clk *xclk;
	u32 xclk_freq;
	bool pending_mode_change;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;

	/* regulator supplies */
	struct regulator *dovdd; /* Digital I/O (1.8V) supply */
	struct regulator *avdd; /* Analog (2.8V) supply */
	struct regulator *dvdd; /* Digital Core (1.8V) supply */
	struct regulator *extclk; /* Digital Core (1.8V) supply */
	struct regulator *ctrl_4t245; /* EXT CLOCK ENABLE data pin supply*/
};

static const struct mt9m114_resolution mt9m114_resolutions[] = {
	[MT9M114_QVGA] = {
		.width  = 320,
		.height = 240,
		.hact   = 320,
		.vact   = 240,
	},
	[MT9M114_VGA] = {
		.width  = 640,
		.height = 480,
		.hact   = 640,
		.vact   = 480,
	},
	[MT9M114_WVGA] = {
		.width  = 800,
		.height = 480,
		.hact   = 800,
		.vact   = 480,
	},
	[MT9M114_720P] = {
		.width  = 1280,
		.height = 720,
		.hact   = 1280,
		.vact   = 720,
	},
};

static const struct mt9m114_reg mt9m114_init[] = {
	/* PLL settings */
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SYSCTL_PLL_ENABLE,                 0x01,   1 },

	/*
	 * R0xC980 [13:8]: PLL N divider value
	 *         [7:0] : PLL M divider value
	 * R0xC982 [13:8]: PLL P divider and word clock divider
	 *
	 * Fout = (fin*2*m)/((N+1)*(P+1))
	 */
	{ MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N,            0x0218, 2 },
	{ MT9M114_CAM_SYSCTL_PLL_DIVIDER_P,              0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_PIXCLK,                 0x2255100, 4 },

	/* Sensor optimization */
	{ 0x316A, 0x8270, 2 },
	{ 0x316C, 0x8270, 2 },
	{ 0x3ED0, 0x2305, 2 },
	{ 0x3ED2, 0x77CF, 2 },
	{ 0x316E, 0x8202, 2 },
	{ 0x3180, 0x87FF, 2 },
	{ 0x30D4, 0x6080, 2 },
	{ 0xA802, 0x0008, 2 },

	{ 0x3E14, 0xFF39, 2 },

	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED, 0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN, 0x00DB, 2},
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX, 0x07C2, 2},
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,  0x02FE, 2},
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK, 0x0845, 2},

	/*
	 *  cam_sensor_cfg_fine_correction = 96
	 *  cam_sensor_cfg_reg_0_data = 32
	 *  cam_sensor_control_read_mode = 0
	 *  cam_crop_window_xoffset = 0
	 *  cam_crop_window_yoffset = 0
	 *  cam_crop_cropmode = 3
	 *  cam_aet_aemode = 0
	 *  cam_aet_max_frame_rate = 7578
	 *  cam_aet_min_frame_rate = 7578
	 *  cam_stat_awb_clip_window_xstart = 0
	 *  cam_stat_awb_clip_window_ystart = 0
	 *  cam_stat_ae_initial_window_xstart = 0
	 *  cam_stat_ae_initial_window_ystart = 0
	 *  Pad slew rate
	 *  Must set cam_output_format_yuv_clip for CSI
	 */
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION, 0x0060, 2},
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA, 0x0020, 2},
	{ MT9M114_CAM_SENSOR_CONTROL_READ_MODE, 0x0000, 2},
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET, 0x0000, 2},
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET, 0x0000, 2},
	{ MT9M114_CAM_CROP_CROPMODE, 0x03, 1},
	{ MT9M114_CAM_AET_AEMODE, 0x00, 1},
	{ MT9M114_CAM_AET_MAX_FRAME_RATE, 0x1D9A, 2},
	{ MT9M114_CAM_AET_MIN_FRAME_RATE, 0x1D9A, 2},
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART, 0x0000, 2},
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART, 0x0000, 2},
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART, 0x0000, 2},
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART, 0x0000, 2},
	{ MT9M114_PAD_SLEW, 0x0777, 2},
	{ MT9M114_CAM_OUTPUT_FORMAT_YUV, 0x0038, 2},

	/* cam_sensor_cfg_line_length_pck will increase by 0.625*fv_to_lv_pck */
	{ 0xC986, 0x00F0, 2 },
};

static const struct mt9m114_reg mt9m114_regs_qvga[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CD, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050D, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x01C3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x00E0, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0280, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0140, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x00F0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x013F, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x00EF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x003F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x002F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_vga[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CD, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050D, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x01C3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x00E0, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0280, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0280, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x01E0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x027F, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x01DF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x007F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x005F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_wvga[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x00F4, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x00F4, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x02DB, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x041B, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x00DB, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x045F, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x0060, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0320, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0320, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x01E0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x031F, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x01DF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x009F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x005F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_720p[] = {
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0004, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0004, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CB, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050B, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x00DB, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x05B3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x03EE, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x0636, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x0060, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x03C3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0500, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x03C0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0500, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x02D0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x04FF, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x02CF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x00FF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x008F, 2 },
};

static const struct mt9m114_format mt9m114_formats[] = {
	{
		.mbus_code      = MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace     = V4L2_COLORSPACE_JPEG,
	}, {
		.mbus_code      = MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace     = V4L2_COLORSPACE_JPEG,
	}, {
		.mbus_code      = MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace     = V4L2_COLORSPACE_SRGB,
	}, {
		.mbus_code      = MEDIA_BUS_FMT_RGB565_1X16,
		.colorspace     = V4L2_COLORSPACE_SRGB,
	},
};

static inline struct mt9m114 *to_mt9m114(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m114, sd);
}

static int mt9m114_write8(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	struct {
		u16 reg;
		u8 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 3,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = val;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
	u16 rval;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 2,
			.buf    = (u8 *)&rval,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		v4l_err(client, "Failed to read register 0x%04x!\n", reg);
		return ret;
	}
	*val = swab16(rval);

	return 0;
}

static int mt9m114_write16(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;
	struct {
		u16 reg;
		u16 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 4,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = swab16(val);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_write32(struct i2c_client *client, u16 reg, u32 val)
{
	int ret;
	struct {
		u16 reg;
		u32 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 6,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = swab32(val);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_writeregs(struct i2c_client *client,
		const struct mt9m114_reg *regs, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		switch (regs[i].width) {
		case 1:
			mt9m114_write8(client, regs[i].reg, regs[i].val);
			break;
		case 2:
			mt9m114_write16(client, regs[i].reg, regs[i].val);
			break;
		case 4:
			mt9m114_write32(client, regs[i].reg, regs[i].val);
			break;
		default:
			return -EINVAL;
		}
	}
	return 0;
}

static int mt9m114_set_res(struct i2c_client *client, u32 width, u32 height)
{
	u16 read_mode;

	if ((width  == mt9m114_resolutions[MT9M114_QVGA].width) &&
	    (height == mt9m114_resolutions[MT9M114_QVGA].height)) {
		mt9m114_writeregs(client, mt9m114_regs_qvga,
				ARRAY_SIZE(mt9m114_regs_qvga));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode = (read_mode & 0xfccf) | 0x0330;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else if ((width == mt9m114_resolutions[MT9M114_VGA].width) &&
		   (height == mt9m114_resolutions[MT9M114_VGA].height)) {
		mt9m114_writeregs(client, mt9m114_regs_vga,
				ARRAY_SIZE(mt9m114_regs_vga));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode = (read_mode & 0xfccf) | 0x0330;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else if ((width == mt9m114_resolutions[MT9M114_WVGA].width)
		&& (height == mt9m114_resolutions[MT9M114_WVGA].height)) {
		mt9m114_writeregs(client, mt9m114_regs_wvga,
				ARRAY_SIZE(mt9m114_regs_wvga));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode &= 0xfccf;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else if ((width == mt9m114_resolutions[MT9M114_720P].width)
		&& (height == mt9m114_resolutions[MT9M114_720P].height)) {
		mt9m114_writeregs(client, mt9m114_regs_720p,
				ARRAY_SIZE(mt9m114_regs_720p));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode &= 0xfccf;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else {
		v4l_err(client, "Failed to select resolution!\n");
		return -EINVAL;
	}
	return 0;
}

static int mt9m114_set_state(struct i2c_client *client, u8 next_state)
{
	int timeout = 100, ret;
	u16 command;

	/* set the next desired state */
	ret = mt9m114_write8(client, MT9M114_SYSMGR_NEXT_STATE, next_state);
	if (ret < 0)
		return ret;

	/* wait for the state transition to complete */
	while (timeout) {
		ret = mt9m114_read16(client,
				MT9M114_COMMAND_REGISTER, &command);
		if (ret < 0)
			return ret;
		if (!(command & MT9M114_COMMAND_REGISTER_SET_STATE))
			break;
		msleep(10);
		timeout--;
	}

	if (!timeout) {
		v4l_err(client, "Failed to poll command register\n");
		return -ETIMEDOUT;
	}

	/* start state transition */
	ret = mt9m114_write16(client, MT9M114_COMMAND_REGISTER,
			(MT9M114_COMMAND_REGISTER_OK
			 | MT9M114_COMMAND_REGISTER_SET_STATE));
	if (ret < 0)
		return ret;

	timeout = 100;
	while (timeout) {
		ret = mt9m114_read16(client,
				MT9M114_COMMAND_REGISTER, &command);
		if (ret < 0)
			return ret;
		if (!(command & MT9M114_COMMAND_REGISTER_SET_STATE))
			break;
		msleep(10);
		timeout--;
	}

	if (!timeout) {
		v4l_err(client, "Failed to poll command register2\n");
		return -ETIMEDOUT;
	}

	/* check if the command is successful */
	ret = mt9m114_read16(client,
			MT9M114_COMMAND_REGISTER, &command);
	if (ret < 0)
		return ret;
	if (command & MT9M114_COMMAND_REGISTER_OK)
		return 0;
	else
		return -EFAULT;
}

static int mt9m114_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m114 *sensor = to_mt9m114(sd);
	const struct mt9m114_resolution *mode = sensor->curr_mode;
	int ret;

	if (enable) {
		ret = pm_runtime_resume_and_get(&sensor->i2c_client->dev);
		if (ret < 0)
			return ret;

		if (sensor->pending_mode_change) {
			sensor->last_mode = mode;
			ret = mt9m114_set_res(client, mode->width, mode->height);
			if (ret < 0)
				return ret;
			ret = mt9m114_set_state(client,
					MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
			if (ret) {
				dev_err(&client->dev,
					"Failed to configure initialization state\n");
				return ret;
			}
		}
		ret = mt9m114_set_state(client,
				MT9M114_SYS_STATE_START_STREAMING);
	} else {
		ret = mt9m114_set_state(client,
				MT9M114_SYS_STATE_ENTER_SUSPEND);
	}

	if (!enable || ret) {
		pm_runtime_mark_last_busy(&sensor->i2c_client->dev);
		pm_runtime_put_autosuspend(&sensor->i2c_client->dev);
	}

	if (ret < 0) {
		dev_err(&client->dev, "set state config change fail\n");
		return ret;
	}
	return 0;
}

static const struct mt9m114_resolution *
mt9m114_find_mode(struct mt9m114 *sensor, u32 width, u32 height)
{
	struct device *dev = &sensor->i2c_client->dev;
	const struct mt9m114_resolution *mode;

	mode = v4l2_find_nearest_size(mt9m114_resolutions,
				      ARRAY_SIZE(mt9m114_resolutions),
				      hact, vact,
				      width, height);
	if (!mode) {
		dev_err(dev, "Invalid resolution: w/h=(%d, %d)\n", width, height);
		return NULL;
	}

	return mode;
}

static int mt9m114_s_power(struct v4l2_subdev *sd, int on)
{
	return 0;
}

static const struct v4l2_subdev_core_ops mt9m114_core_ops = {
	.s_power = mt9m114_s_power,
};

static int mt9m114_get_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval *fi)
{
	struct mt9m114 *sensor = to_mt9m114(sd);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int mt9m114_set_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_frame_interval *fi)
{
	return 0;
}

static int mt9m114_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad != 0)
		return -EINVAL;
	if (code->index >= ARRAY_SIZE(mt9m114_formats))
		return -EINVAL;

	code->code = mt9m114_formats[code->index].mbus_code;
	return 0;
}

static int mt9m114_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct mt9m114 *sensor = to_mt9m114(sd);
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);
	fmt = &sensor->fmt;
	format->format = *fmt;
	mutex_unlock(&sensor->lock);

	return 0;
}

static const struct v4l2_mbus_framefmt mt9m114_default_fmt = {
	.code = MEDIA_BUS_FMT_UYVY8_2X8,
	.width = MT9M114_PIXEL_ARRAY_WIDTH,
	.height = MT9M114_PIXEL_ARRAY_HEIGHT,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(V4L2_COLORSPACE_SRGB),
	.quantization = V4L2_QUANTIZATION_FULL_RANGE,
	.xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(V4L2_COLORSPACE_SRGB),
	.field = V4L2_FIELD_NONE,
};

static const struct mt9m114_format *mt9m114_code_to_pixfmt(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mt9m114_formats); i++) {
		if (code == mt9m114_formats[i].mbus_code)
			return &mt9m114_formats[i];
	}

	return &mt9m114_formats[0];
}

static int mt9m114_try_fmt_internal(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt,
				const struct mt9m114_resolution **new_mode)
{
	struct mt9m114 *sensor = to_mt9m114(sd);
	static const struct mt9m114_resolution *mode;
	const struct mt9m114_format *pixfmt;

	mode = mt9m114_find_mode(sensor, fmt->width, fmt->height);

	pixfmt = mt9m114_code_to_pixfmt(fmt->code);

	fmt->width = mode->width;
	fmt->height = mode->height;

	if (new_mode)
		*new_mode = mode;
	fmt->code = pixfmt->mbus_code;
	fmt->colorspace = pixfmt->colorspace;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	return 0;
}

static int mt9m114_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *sd_state,
			  struct v4l2_subdev_format *format)
{
	struct mt9m114 *sensor = to_mt9m114(sd);
	struct v4l2_mbus_framefmt *fmt = &sensor->fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	const struct mt9m114_resolution *new_mode;
	int ret;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	ret = mt9m114_try_fmt_internal(sd, mbus_fmt, &new_mode);
	if (ret)
		goto out;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = *mbus_fmt;
		goto out;
	}

	sensor->curr_mode = new_mode;
	if (sensor->curr_mode != sensor->last_mode) {
		sensor->pending_mode_change = true;
		memcpy(fmt, mbus_fmt, sizeof(*fmt));
		fmt->width  = new_mode->hact;
		fmt->height = new_mode->vact;
	}

	/* update format even if code is unchanged, resolution might change */
	sensor->fmt = *mbus_fmt;

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int mt9m114_init_state(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt =
				v4l2_subdev_state_get_format(state, 0);
	struct v4l2_rect *crop = v4l2_subdev_state_get_crop(state, 0);

	*fmt =  mt9m114_default_fmt;

	crop->left = MT9M114_PIXEL_ARRAY_LEFT;
	crop->top = MT9M114_PIXEL_ARRAY_TOP;
	crop->width = MT9M114_PIXEL_ARRAY_WIDTH;
	crop->height = MT9M114_PIXEL_ARRAY_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_internal_ops mt9m114_internal_ops = {
	.init_state = mt9m114_init_state,
};

static int mt9m114_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct mt9m114 *sensor = to_mt9m114(sd);

	if (fse->pad != 0)
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(mt9m114_resolutions))
		return -EINVAL;

	mutex_lock(&sensor->lock);
	fse->min_width  = mt9m114_resolutions[fse->index].width;
	fse->min_height = mt9m114_resolutions[fse->index].height;
	fse->max_width  = mt9m114_resolutions[fse->index].width;
	fse->max_height = mt9m114_resolutions[fse->index].height;
	mutex_unlock(&sensor->lock);

	return 0;
}

static const struct v4l2_subdev_pad_ops mt9m114_pad_ops = {
	.enum_mbus_code  = mt9m114_enum_mbus_code,
	.enum_frame_size = mt9m114_enum_frame_size,
	.get_fmt         = mt9m114_get_fmt,
	.set_fmt         = mt9m114_set_fmt,
	.get_frame_interval = mt9m114_get_frame_interval,
	.set_frame_interval = mt9m114_set_frame_interval,
};

static const struct v4l2_subdev_video_ops mt9m114_video_ops = {
	.s_stream         = mt9m114_s_stream,
};

static const struct v4l2_subdev_ops mt9m114_subdev_ops = {
	.core  = &mt9m114_core_ops,
	.video = &mt9m114_video_ops,
	.pad   = &mt9m114_pad_ops,
};

static int mt9m114_get_regulators(struct mt9m114 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	sensor->dovdd = devm_regulator_get(dev, "DOVDD");
	if (IS_ERR(sensor->dovdd)) {
		dev_err(dev, "cannot get dovdd regulator\n");
		return PTR_ERR(sensor->dovdd);
	}

	sensor->avdd = devm_regulator_get(dev, "AVDD");
	if (IS_ERR(sensor->avdd)) {
		dev_err(dev, "cannot get avdd regulator\n");
		return PTR_ERR(sensor->avdd);
	}

	sensor->dvdd = devm_regulator_get(dev, "DVDD");
	if (IS_ERR(sensor->dvdd)) {
		dev_err(dev, "can't get dvdd regulator\n");
		return PTR_ERR(sensor->dvdd);
	}

	sensor->extclk = devm_regulator_get_optional(dev, "EXTCLK");
	if (IS_ERR(sensor->extclk)) {
		dev_warn(dev, "can't get extclk regulator\n");
		if (PTR_ERR(sensor->extclk) != -ENODEV)
			return PTR_ERR(sensor->extclk);

		sensor->extclk = NULL;
	}

	sensor->ctrl_4t245 = devm_regulator_get_optional(dev, "CTRL_4T245");
	if (IS_ERR(sensor->ctrl_4t245)) {
		dev_warn(dev, "can't get extclk enable\n");
		if (PTR_ERR(sensor->ctrl_4t245) != -ENODEV)
			return PTR_ERR(sensor->ctrl_4t245);

		sensor->ctrl_4t245 = NULL;
	}

	return ret;
}

static int mt9m114_set_power(struct mt9m114 *sensor, bool on)
{
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	if (on) {
		if (sensor->dovdd) {
			ret = regulator_enable(sensor->dovdd);
			if (ret < 0) {
				dev_err(dev, "failed to enable dovdd\n");
				return ret;
			}
		}

		if (sensor->avdd) {
			ret = regulator_enable(sensor->avdd);
			if (ret < 0) {
				dev_err(dev, "failed to enable avdd\n");
				return ret;
			}
		}

		if (sensor->dvdd) {
			ret = regulator_enable(sensor->dvdd);
			if (ret < 0) {
				dev_err(dev, "failed to enable dvdd\n");
				return ret;
			}
		}

		if (sensor->extclk) {
			ret = regulator_enable(sensor->extclk);
			if (ret < 0) {
				dev_err(dev, "failed to enable extclk\n");
				return ret;
			}
		}

		if (sensor->ctrl_4t245) {
			ret = regulator_enable(sensor->ctrl_4t245);
			if (ret < 0) {
				dev_err(dev, "failed to enable ctrl_4t245\n");
				return ret;
			}
		}

	} else {
		if (sensor->ctrl_4t245) {
			ret = regulator_disable(sensor->ctrl_4t245);
			if (ret < 0) {
				dev_err(dev, "failed to disable ctrl_4t245\n");
				return ret;
			}
		}

		if (sensor->extclk) {
			ret = regulator_disable(sensor->extclk);
			if (ret < 0) {
				dev_err(dev, "failed to disable extclk\n");
				return ret;
			}
		}

		if (sensor->dvdd) {
			ret = regulator_disable(sensor->dvdd);
			if (ret < 0) {
				dev_err(dev, "failed to disable dvdd\n");
				return ret;
			}
		}

		if (sensor->avdd) {
			ret = regulator_disable(sensor->avdd);
			if (ret < 0) {
				dev_err(dev, "failed to disable avdd\n");
				return ret;
			}
		}

		if (sensor->dovdd) {
			ret = regulator_disable(sensor->dovdd);
			if (ret < 0) {
				dev_err(dev, "failed to disable dovdd\n");
				return ret;
			}
		}
	}

	return ret;
}

static int mt9m114_get_gpios(struct mt9m114 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct gpio_desc *gpiod;
	int ret;

	/* request optional power down pin */
	gpiod = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(gpiod)) {
		ret = PTR_ERR(gpiod);
		dev_err(dev, "fail to get power down pin, ret=%d\n", ret);
		return ret;
	}
	sensor->pwdn_gpio = gpiod;

	/* request optional reset pin */
	gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gpiod)) {
		ret = PTR_ERR(gpiod);
		dev_err(dev, "fail to get reset pin, ret=%d\n", ret);
		return ret;
	}
	sensor->reset_gpio = gpiod;

	return 0;
}

static void mt9m114_hw_reset(struct mt9m114 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);
	msleep(50);
	gpiod_set_value_cansleep(sensor->reset_gpio, 1);
	msleep(45);
}

static int mt9m114_get_chip_id(struct mt9m114 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	u32 ret;
	u16 chip_id;

	ret = mt9m114_read16(client, MT9M114_CHIP_ID, &chip_id);
	if (ret < 0) {
		v4l_err(client, "Failed to get chip id\n");
		return -ENODEV;
	}
	if (chip_id != 0x2481) {
		v4l_err(client, "chip id 0x%04x mismatch\n", chip_id);
		return -ENODEV;
	}
	v4l_info(client, "chip found @ 0x%02x (%s)\n",
		 client->addr << 1, client->adapter->name);

	return 0;
}

static int mt9m114_soft_reset(struct mt9m114 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	/* reset the sensor */
	ret = mt9m114_write16(client, MT9M114_SOFT_RESET, 0x0001);
	if (ret < 0) {
		v4l_err(client, "Failed to reset the sensor\n");
		return ret;
	}
	mdelay(1);
	mt9m114_write16(client, MT9M114_SOFT_RESET, 0x0000);
	mdelay(45);

	return 0;
}

static int mt9m114_init_config(struct mt9m114 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	struct v4l2_mbus_framefmt *fmt = &sensor->fmt;
	u16 output_fmt;
	int ret;

	ret = mt9m114_writeregs(client, mt9m114_init, ARRAY_SIZE(mt9m114_init));
	if (ret < 0) {
		v4l_err(client, "Failed to initialize the sensor\n");
		return ret;
	}

	/* PIXCLK is only generated for valid output pixels. */
	mt9m114_write16(client, MT9M114_CAM_PORT_OUTPUT_CONTROL, 0x8020);

	/* Config 720P as default resolution */
	ret = mt9m114_set_res(client, fmt->width, fmt->height);
	if (ret < 0)
		return ret;

	/* Config UYVY as default format
	 * 0xC86C: [9:8] 0 -> YUV
	 *               1 -> RGB
	 *               2 -> Bayer
	 *               3 -> None
	 *           [4] cam_output_format_bt656_crop_scale_disable
	 *           [1] Swap output pixel hi byte with low byte
	 *           [0] Swap R/B or Cr/Cb channels
	 */
	mt9m114_read16(client, MT9M114_CAM_OUTPUT_FORMAT, &output_fmt);
	output_fmt |= 0x0002;
	ret = mt9m114_write16(client, MT9M114_CAM_OUTPUT_FORMAT, output_fmt);
	if (ret < 0)
		return ret;

	ret = mt9m114_set_state(client, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	if (ret) {
		v4l_err(client, "Failed to configure initialization state\n");
		return ret;
	}

	return 0;
}

static int mt9m114_sensor_suspend(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m114 *sensor = to_mt9m114(sd);

	return mt9m114_set_power(sensor, false);
}

static int mt9m114_sensor_resume(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct mt9m114 *sensor = to_mt9m114(sd);

	return mt9m114_set_power(sensor, true);
}

static const struct dev_pm_ops mt9m114_pm_ops = {
	SET_RUNTIME_PM_OPS(mt9m114_sensor_suspend, mt9m114_sensor_resume, NULL)
};

static int mt9m114_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations mt9m114_sd_media_ops = {
	.link_setup = mt9m114_link_setup,
};

static int mt9m114_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mt9m114 *sensor;
	struct v4l2_subdev *sd;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	sensor = devm_kzalloc(&client->dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;

	ret = mt9m114_get_regulators(sensor);
	if (ret < 0) {
		dev_err(&client->dev, "failed to get regulators\n");
		return ret;
	}

	ret = mt9m114_set_power(sensor, true);
	if (ret < 0)
		return ret;

	ret = mt9m114_get_gpios(sensor);
	if (ret)
		goto err_power;

	mt9m114_hw_reset(sensor);

	ret = mt9m114_get_chip_id(sensor);
	if (ret)
		goto err_power;

	ret = mt9m114_soft_reset(sensor);
	if (ret)
		goto err_power;

	/*
	 * config default format
	 */
	sensor->curr_mode = &mt9m114_resolutions[MT9M114_720P];
	sensor->last_mode = sensor->curr_mode;
	fmt = &sensor->fmt;
	fmt->code         = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace   = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc    = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func    = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width        = sensor->curr_mode->width;
	fmt->height       = sensor->curr_mode->height;
	fmt->field        = V4L2_FIELD_NONE;
	sensor->frame_interval.numerator   = 1;
	sensor->frame_interval.denominator = 30;

	ret = mt9m114_init_config(sensor);
	if (ret)
		goto err_power;

	v4l2_i2c_subdev_init(&sensor->sd, client, &mt9m114_subdev_ops);

	sd = &sensor->sd;
	sd->flags          |= V4L2_SUBDEV_FL_HAS_EVENTS
			   | V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.ops      = &mt9m114_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.internal_ops = &mt9m114_internal_ops;

	sensor->pads[MT9M114_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, MT9M114_SENS_PADS_NUM, sensor->pads);
	if (ret)
		goto err_power;
	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_power;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret)
		goto fail;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	v4l2_info(client, "MT9M114 is found\n");
	return 0;

fail:
	pm_runtime_put_noidle(dev);
	pm_runtime_disable(dev);
	media_entity_cleanup(&sensor->sd.entity);

err_power:
	mt9m114_set_power(sensor, false);
	return ret;
}

static void mt9m114_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9m114 *sensor = to_mt9m114(sd);
	struct device *dev = &client->dev;

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		mt9m114_sensor_suspend(dev);
	pm_runtime_set_suspended(dev);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sensor->sd.entity);
	kfree(sensor);
}

static const struct i2c_device_id mt9m114_id[] = {
	{"mt9m114", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static const struct of_device_id mt9m114_dt_ids[] = {
	{ .compatible = "on,mt9m114" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mt9m114_dt_ids);

static struct i2c_driver mt9m114_i2c_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "mt9m114",
		.of_match_table = mt9m114_dt_ids,
	},
	.probe	= mt9m114_probe,
	.remove    = mt9m114_remove,
	.id_table  = mt9m114_id,
};

module_i2c_driver(mt9m114_i2c_driver);

MODULE_DESCRIPTION("Aptina MT9M114 sensor driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL");
