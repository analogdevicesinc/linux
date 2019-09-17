/* SPDX-License-Identifier: GPL-2.0 */
// Copyright 2019 NXP


#ifndef _MXSFB_SII902X_H
#define _MXSFB_SII902X_H

#include <video/mxc_edid.h>

#define SII_EDID_LEN	512

struct sii902x_data {
	struct i2c_client *client;
	struct delayed_work det_work;
	struct fb_info *fbi;
	struct mxc_edid_cfg edid_cfg;
	u8 cable_plugin;
	u8 edid[SII_EDID_LEN];
	bool dft_mode_set;
	const char *mode_str;
	int bits_per_pixel;
};

void sii902x_register_audio_driver(struct device *dev);

#endif /* _MXSFB_SII902X_H */
