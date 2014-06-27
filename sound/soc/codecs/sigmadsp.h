/*
 * Load firmware files from Analog Devices SigmaStudio
 *
 * Copyright 2009-2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __SIGMA_FIRMWARE_H__
#define __SIGMA_FIRMWARE_H__

#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/list.h>

#include <sound/pcm.h>

struct sigmadsp;
struct snd_soc_codec;
struct snd_pcm_substream;

struct sigmadsp_ops {
	int (*safeload)(struct sigmadsp *sigmadsp, unsigned int addr,
			const uint8_t *data, size_t len);
};

struct sigmadsp {
	const struct sigmadsp_ops *ops;

	struct list_head ctrl_list;
	struct list_head data_list;

	struct snd_pcm_hw_constraint_list rate_constraints;

	unsigned int current_samplerate;
	struct snd_soc_codec *codec;

	void *control_data;
	int (*write)(void *, unsigned int, const uint8_t *, size_t);
	int (*read)(void *, unsigned int, uint8_t *, size_t);
};

void sigmadsp_init(struct sigmadsp *sigmadsp, const struct sigmadsp_ops *ops);
void sigmadsp_reset(struct sigmadsp *sigmadsp);

int sigmadsp_restrict_params(struct sigmadsp *sigmadsp,
	struct snd_pcm_substream *substream);

struct i2c_client;

void sigmadsp_init_regmap(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops,	struct regmap *regmap);
void sigmadsp_init_i2c(struct sigmadsp *sigmadsp,
	const struct sigmadsp_ops *ops,	struct i2c_client *client);

int sigmadsp_firmware_load(struct sigmadsp *sigmadsp,
	struct snd_soc_codec *codec, const char *name);
int sigmadsp_setup(struct sigmadsp *sigmadsp, unsigned int rate);
void sigmadsp_firmware_release(struct sigmadsp *sigmadsp);
void sigmadsp_reset(struct sigmadsp *sigmadsp);

#endif
