/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Audio driver for RPMSG WM8960
 *
 * Copyright 2007-11 Wolfson Microelectronics, plc
 * Copyright 2020 NXP
 */

#ifndef _RPMSG_WM8960_H
#define _RPMSG_WM8960_H

/* R25 - Power 1 */
#define WM8960_VMID_MASK 0x180
#define WM8960_VREF      0x40

/* R26 - Power 2 */
#define WM8960_PWR2_LOUT1	0x40
#define WM8960_PWR2_ROUT1	0x20
#define WM8960_PWR2_OUT3	0x02

/* R28 - Anti-pop 1 */
#define WM8960_POBCTRL   0x80
#define WM8960_BUFDCOPEN 0x10
#define WM8960_BUFIOEN   0x08
#define WM8960_SOFT_ST   0x04
#define WM8960_HPSTBY    0x01

/* R29 - Anti-pop 2 */
#define WM8960_DISOP     0x40
#define WM8960_DRES_MASK 0x30

#define wm8960_reset(c)	regmap_write(c, WM8960_RESET, 0)

struct rpmsg_wm8960_priv {
	struct clk *mclk;
	struct regmap *regmap;
	int (*set_bias_level)(struct snd_soc_component *,
			      enum snd_soc_bias_level level);
	struct snd_soc_dapm_widget *lout1;
	struct snd_soc_dapm_widget *rout1;
	struct snd_soc_dapm_widget *out3;
	bool deemph;
	int lrclk;
	int bclk;
	int sysclk;
	int clk_id;
	int freq_in;
	bool is_stream_in_use[2];
	struct wm8960_data pdata;
	struct rpmsg_info *info;
	int audioindex;
};

#endif
