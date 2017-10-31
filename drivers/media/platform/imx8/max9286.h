/*
 * Copyright (C) 2017 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later.
 */

#ifndef __MAX9286_H__
#define __MAX9286_H__

#define MIPI_CSI2_SENS_VC0_PAD_SOURCE	0
#define MIPI_CSI2_SENS_VC1_PAD_SOURCE	1
#define MIPI_CSI2_SENS_VC2_PAD_SOURCE	2
#define MIPI_CSI2_SENS_VC3_PAD_SOURCE	3
#define MIPI_CSI2_SENS_VCX_PADS_NUM		4

/*!
 * Maintains the information on the current state of the sesor.
 */
struct imxdpu_videomode {
	char name[64];		/* may not be needed */

	uint32_t pixelclock;	/* Hz */

	/* htotal (pixels) = hlen + hfp + hsync + hbp */
	uint32_t hlen;
	uint32_t hfp;
	uint32_t hbp;
	uint32_t hsync;

	/* field0 - vtotal (lines) = vlen + vfp + vsync + vbp */
	uint32_t vlen;
	uint32_t vfp;
	uint32_t vbp;
	uint32_t vsync;

	/* field1  */
	uint32_t vlen1;
	uint32_t vfp1;
	uint32_t vbp1;
	uint32_t vsync1;

	uint32_t flags;

	uint32_t format;
	uint32_t dest_format; /*buffer format for capture*/

	int16_t clip_top;
	int16_t clip_left;
	uint16_t clip_width;
	uint16_t clip_height;

};
struct sensor_data {
	struct v4l2_subdev	subdev;
	struct media_pad pads[MIPI_CSI2_SENS_VCX_PADS_NUM];
	struct i2c_client *i2c_client;
	struct v4l2_mbus_framefmt format;
	struct v4l2_captureparm streamcap;
	char running;

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
	int v_channel;
	bool is_mipi;
	struct imxdpu_videomode cap_mode;

	unsigned int sensor_num;       /* sensor num connect max9271 */
	unsigned char sensor_is_there; /* Bit 0~3 for 4 cameras, 0b1= is there; 0b0 = is not there */
};
#endif
