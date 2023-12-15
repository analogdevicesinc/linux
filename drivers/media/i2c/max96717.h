// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2024 NXP
 */

#ifndef _MAX96717_H_
#define _MAX96717_H_

struct max96717 {
	struct device *dev;
	struct regmap *rmap;
};

#define LANE_POL_NORMAL		0
#define LANE_POL_INVERSED	1

enum max96717_data_type {
	MAX96717_DT_EMBEDDED		= 0x12,
	MAX96717_DT_YUV422_8BIT		= 0x1E,
	MAX96717_DT_YUV422_10BIT	= 0x1F,
	MAX96717_DT_RGB565		= 0x22,
	MAX96717_DT_RGB666		= 0x23,
	MAX96717_DT_RGB888		= 0x24,
	MAX96717_DT_RAW8		= 0x2A,
	MAX96717_DT_RAW10		= 0x2B,
	MAX96717_DT_RAW12		= 0x2C,
	MAX96717_DT_RAW14		= 0x2D,
	MAX96717_DT_RAW16		= 0x2E,
	MAX96717_DT_RAW20		= 0x2F,
	MAX96717_DT_YUV422_12BIT	= 0x30,
};

enum max96717_gmsl_speed {
	MAX96717_GMSL_3G,
	MAX96717_GMSL_6G,
};

struct max96717 *max96717_init(struct i2c_client *client);
bool max96717_is_dev_id_valid(struct max96717 *ser);
int max96717_hw_init(struct max96717 *ser, unsigned int reset_pin, unsigned int clock_pin);
int max96717_tunnel_mode_en(struct max96717 *ser, bool en);
int max96717_set_lanes_no(struct max96717 *ser, int lanes_no);
int max96717_map_csi_lanes(struct max96717 *ser, int *map, int map_size);
int max96717_set_lanes_polarity(struct max96717 *ser, int *pol, int pol_size);
int max96717_csi_port_en(struct max96717 *ser, bool en);
int max96717_video_pipe_en(struct max96717 *ser, bool en);
int max96717_video_pipe_tx_en(struct max96717 *ser, bool en);
int max96717_data_type_filter(struct max96717 *ser, u8 *dt_array, u8 dt_array_size);
int max96717_vc_filter(struct max96717 *ser, u16 filter_map);
int max96717_stream_id_set(struct max96717 *ser, int id);
int max96717_gmsl_speed_set(struct max96717 *ser, enum max96717_gmsl_speed speed);
int max96717_double_mode_en(struct max96717 *ser);
int max96717_soft_bpp_override(struct max96717 *ser, int bpp);

#endif