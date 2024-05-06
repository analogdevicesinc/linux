// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 NXP
 */

#ifndef _OX03C10_PRIV_H_
#define _OX03C10_PRIV_H_

struct ox03c10 {
	struct i2c_client *client;
	struct device *dev;
	struct regmap *rmap;

	struct v4l2_ctrl_handler ctrl_handler;

	struct ox03c10_mode *cur_mode;

	/* this needs to be the last element in the structure */
	struct v4l2_ctrl *ctrls[];
};

#define OX03C10_NATIVE_WIDTH		1936
#define OX03C10_NATIVE_HEIGHT		1296

#define OX03C10_PIXEL_ARRAY_TOP		8
#define OX03C10_PIXEL_ARRAY_LEFT	8
#define OX03C10_PIXEL_ARRAY_WIDTH	1920
#define OX03C10_PIXEL_ARRAY_HEIGHT	1282 /* first 2 lines is embedded data */

struct ox03c10_mode {
	u32 width;
	u32 height;
	u32 hts;
	u32 vts;
	u16 fps;
};

int ox03c10_v4l2_controls_init(struct ox03c10 *ox03c10);
struct ox03c10 *ox03c10_init(struct i2c_client *client);
int ox03c10_streaming_start(struct ox03c10 *sensor, bool start);
struct v4l2_ctrl_handler *ox03c10_ctrl_handler_get(struct ox03c10 *sensor);
void ox03c10_ctrl_handler_free(struct ox03c10 *sensor);
struct ox03c10_mode *ox03c10_get_mode(int index);

#endif