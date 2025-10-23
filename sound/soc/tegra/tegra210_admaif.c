// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES.
// All rights reserved.
//
// tegra210_admaif.c - Tegra ADMAIF driver

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "tegra_isomgr_bw.h"
#include "tegra210_admaif.h"
#include "tegra_cif.h"
#include "tegra_pcm.h"

#define CH_REG(offset, reg, id)						       \
	((offset) + (reg) + (TEGRA_ADMAIF_CHANNEL_REG_STRIDE * (id)))

#define CH_TX_REG(reg, id) CH_REG(admaif->soc_data->tx_base, reg, id)

#define CH_RX_REG(reg, id) CH_REG(admaif->soc_data->rx_base, reg, id)

#define REG_DEFAULTS(id, rx_ctrl, tx_ctrl, tx_base, rx_base, cif_ctrl)	       \
	{ CH_REG(rx_base, TEGRA_ADMAIF_RX_INT_MASK, id), 0x00000001 },	       \
	{ CH_REG(rx_base, TEGRA_ADMAIF_CH_ACIF_RX_CTRL, id), cif_ctrl },     \
	{ CH_REG(rx_base, TEGRA_ADMAIF_RX_FIFO_CTRL, id), rx_ctrl },	       \
	{ CH_REG(tx_base, TEGRA_ADMAIF_TX_INT_MASK, id), 0x00000001 },	       \
	{ CH_REG(tx_base, TEGRA_ADMAIF_CH_ACIF_TX_CTRL, id), cif_ctrl },     \
	{ CH_REG(tx_base, TEGRA_ADMAIF_TX_FIFO_CTRL, id), tx_ctrl }

#define ADMAIF_REG_DEFAULTS(id, chip)					       \
	REG_DEFAULTS((id) - 1,						       \
		chip ## _ADMAIF_RX ## id ## _FIFO_CTRL_REG_DEFAULT,	       \
		chip ## _ADMAIF_TX ## id ## _FIFO_CTRL_REG_DEFAULT,	       \
		chip ## _ADMAIF_TX_BASE,				       \
		chip ## _ADMAIF_RX_BASE,				       \
		chip ## _ADMAIF_CIF_REG_DEFAULT)

static const struct reg_default tegra186_admaif_reg_defaults[] = {
	{(TEGRA_ADMAIF_GLOBAL_CG_0 + TEGRA186_ADMAIF_GLOBAL_BASE), 0x00000003},
	ADMAIF_REG_DEFAULTS(1, TEGRA186),
	ADMAIF_REG_DEFAULTS(2, TEGRA186),
	ADMAIF_REG_DEFAULTS(3, TEGRA186),
	ADMAIF_REG_DEFAULTS(4, TEGRA186),
	ADMAIF_REG_DEFAULTS(5, TEGRA186),
	ADMAIF_REG_DEFAULTS(6, TEGRA186),
	ADMAIF_REG_DEFAULTS(7, TEGRA186),
	ADMAIF_REG_DEFAULTS(8, TEGRA186),
	ADMAIF_REG_DEFAULTS(9, TEGRA186),
	ADMAIF_REG_DEFAULTS(10, TEGRA186),
	ADMAIF_REG_DEFAULTS(11, TEGRA186),
	ADMAIF_REG_DEFAULTS(12, TEGRA186),
	ADMAIF_REG_DEFAULTS(13, TEGRA186),
	ADMAIF_REG_DEFAULTS(14, TEGRA186),
	ADMAIF_REG_DEFAULTS(15, TEGRA186),
	ADMAIF_REG_DEFAULTS(16, TEGRA186),
	ADMAIF_REG_DEFAULTS(17, TEGRA186),
	ADMAIF_REG_DEFAULTS(18, TEGRA186),
	ADMAIF_REG_DEFAULTS(19, TEGRA186),
	ADMAIF_REG_DEFAULTS(20, TEGRA186)
};

static const struct reg_default tegra210_admaif_reg_defaults[] = {
	{(TEGRA_ADMAIF_GLOBAL_CG_0 + TEGRA210_ADMAIF_GLOBAL_BASE), 0x00000003},
	ADMAIF_REG_DEFAULTS(1, TEGRA210),
	ADMAIF_REG_DEFAULTS(2, TEGRA210),
	ADMAIF_REG_DEFAULTS(3, TEGRA210),
	ADMAIF_REG_DEFAULTS(4, TEGRA210),
	ADMAIF_REG_DEFAULTS(5, TEGRA210),
	ADMAIF_REG_DEFAULTS(6, TEGRA210),
	ADMAIF_REG_DEFAULTS(7, TEGRA210),
	ADMAIF_REG_DEFAULTS(8, TEGRA210),
	ADMAIF_REG_DEFAULTS(9, TEGRA210),
	ADMAIF_REG_DEFAULTS(10, TEGRA210)
};

static const struct reg_default tegra264_admaif_reg_defaults[] = {
	{(TEGRA_ADMAIF_GLOBAL_CG_0 + TEGRA264_ADMAIF_GLOBAL_BASE), 0x00000003},
	ADMAIF_REG_DEFAULTS(1, TEGRA264),
	ADMAIF_REG_DEFAULTS(2, TEGRA264),
	ADMAIF_REG_DEFAULTS(3, TEGRA264),
	ADMAIF_REG_DEFAULTS(4, TEGRA264),
	ADMAIF_REG_DEFAULTS(5, TEGRA264),
	ADMAIF_REG_DEFAULTS(6, TEGRA264),
	ADMAIF_REG_DEFAULTS(7, TEGRA264),
	ADMAIF_REG_DEFAULTS(8, TEGRA264),
	ADMAIF_REG_DEFAULTS(9, TEGRA264),
	ADMAIF_REG_DEFAULTS(10, TEGRA264),
	ADMAIF_REG_DEFAULTS(11, TEGRA264),
	ADMAIF_REG_DEFAULTS(12, TEGRA264),
	ADMAIF_REG_DEFAULTS(13, TEGRA264),
	ADMAIF_REG_DEFAULTS(14, TEGRA264),
	ADMAIF_REG_DEFAULTS(15, TEGRA264),
	ADMAIF_REG_DEFAULTS(16, TEGRA264),
	ADMAIF_REG_DEFAULTS(17, TEGRA264),
	ADMAIF_REG_DEFAULTS(18, TEGRA264),
	ADMAIF_REG_DEFAULTS(19, TEGRA264),
	ADMAIF_REG_DEFAULTS(20, TEGRA264),
	ADMAIF_REG_DEFAULTS(21, TEGRA264),
	ADMAIF_REG_DEFAULTS(22, TEGRA264),
	ADMAIF_REG_DEFAULTS(23, TEGRA264),
	ADMAIF_REG_DEFAULTS(24, TEGRA264),
	ADMAIF_REG_DEFAULTS(25, TEGRA264),
	ADMAIF_REG_DEFAULTS(26, TEGRA264),
	ADMAIF_REG_DEFAULTS(27, TEGRA264),
	ADMAIF_REG_DEFAULTS(28, TEGRA264),
	ADMAIF_REG_DEFAULTS(29, TEGRA264),
	ADMAIF_REG_DEFAULTS(30, TEGRA264),
	ADMAIF_REG_DEFAULTS(31, TEGRA264),
	ADMAIF_REG_DEFAULTS(32, TEGRA264)
};

static bool tegra_admaif_wr_reg(struct device *dev, unsigned int reg)
{
	struct tegra_admaif *admaif = dev_get_drvdata(dev);
	unsigned int ch_stride = TEGRA_ADMAIF_CHANNEL_REG_STRIDE;
	unsigned int num_ch = admaif->soc_data->num_ch;
	unsigned int rx_base = admaif->soc_data->rx_base;
	unsigned int tx_base = admaif->soc_data->tx_base;
	unsigned int global_base = admaif->soc_data->global_base;
	unsigned int reg_max = admaif->soc_data->regmap_conf->max_register;
	unsigned int rx_max = rx_base + (num_ch * ch_stride);
	unsigned int tx_max = tx_base + (num_ch * ch_stride);

	if ((reg >= rx_base) && (reg < rx_max)) {
		reg = (reg - rx_base) % ch_stride;
		if ((reg == TEGRA_ADMAIF_RX_ENABLE) ||
		    (reg == TEGRA_ADMAIF_RX_FIFO_CTRL) ||
		    (reg == TEGRA_ADMAIF_RX_SOFT_RESET) ||
		    (reg == TEGRA_ADMAIF_CH_ACIF_RX_CTRL))
			return true;
	} else if ((reg >= tx_base) && (reg < tx_max)) {
		reg = (reg - tx_base) % ch_stride;
		if ((reg == TEGRA_ADMAIF_TX_ENABLE) ||
		    (reg == TEGRA_ADMAIF_TX_FIFO_CTRL) ||
		    (reg == TEGRA_ADMAIF_TX_SOFT_RESET) ||
		    (reg == TEGRA_ADMAIF_CH_ACIF_TX_CTRL))
			return true;
	} else if ((reg >= global_base) && (reg < reg_max)) {
		if (reg == (global_base + TEGRA_ADMAIF_GLOBAL_ENABLE))
			return true;
	}

	return false;
}

static bool tegra_admaif_rd_reg(struct device *dev, unsigned int reg)
{
	struct tegra_admaif *admaif = dev_get_drvdata(dev);
	unsigned int ch_stride = TEGRA_ADMAIF_CHANNEL_REG_STRIDE;
	unsigned int num_ch = admaif->soc_data->num_ch;
	unsigned int rx_base = admaif->soc_data->rx_base;
	unsigned int tx_base = admaif->soc_data->tx_base;
	unsigned int global_base = admaif->soc_data->global_base;
	unsigned int reg_max = admaif->soc_data->regmap_conf->max_register;
	unsigned int rx_max = rx_base + (num_ch * ch_stride);
	unsigned int tx_max = tx_base + (num_ch * ch_stride);

	if ((reg >= rx_base) && (reg < rx_max)) {
		reg = (reg - rx_base) % ch_stride;
		if ((reg == TEGRA_ADMAIF_RX_ENABLE) ||
		    (reg == TEGRA_ADMAIF_RX_STATUS) ||
		    (reg == TEGRA_ADMAIF_RX_INT_STATUS) ||
		    (reg == TEGRA_ADMAIF_RX_FIFO_CTRL) ||
		    (reg == TEGRA_ADMAIF_RX_SOFT_RESET) ||
		    (reg == TEGRA_ADMAIF_CH_ACIF_RX_CTRL))
			return true;
	} else if ((reg >= tx_base) && (reg < tx_max)) {
		reg = (reg - tx_base) % ch_stride;
		if ((reg == TEGRA_ADMAIF_TX_ENABLE) ||
		    (reg == TEGRA_ADMAIF_TX_STATUS) ||
		    (reg == TEGRA_ADMAIF_TX_INT_STATUS) ||
		    (reg == TEGRA_ADMAIF_TX_FIFO_CTRL) ||
		    (reg == TEGRA_ADMAIF_TX_SOFT_RESET) ||
		    (reg == TEGRA_ADMAIF_CH_ACIF_TX_CTRL))
			return true;
	} else if ((reg >= global_base) && (reg < reg_max)) {
		if ((reg == (global_base + TEGRA_ADMAIF_GLOBAL_ENABLE)) ||
		    (reg == (global_base + TEGRA_ADMAIF_GLOBAL_CG_0)) ||
		    (reg == (global_base + TEGRA_ADMAIF_GLOBAL_STATUS)) ||
		    (reg == (global_base +
				TEGRA_ADMAIF_GLOBAL_RX_ENABLE_STATUS)) ||
		    (reg == (global_base +
				TEGRA_ADMAIF_GLOBAL_TX_ENABLE_STATUS)))
			return true;
	}

	return false;
}

static bool tegra_admaif_volatile_reg(struct device *dev, unsigned int reg)
{
	struct tegra_admaif *admaif = dev_get_drvdata(dev);
	unsigned int ch_stride = TEGRA_ADMAIF_CHANNEL_REG_STRIDE;
	unsigned int num_ch = admaif->soc_data->num_ch;
	unsigned int rx_base = admaif->soc_data->rx_base;
	unsigned int tx_base = admaif->soc_data->tx_base;
	unsigned int global_base = admaif->soc_data->global_base;
	unsigned int reg_max = admaif->soc_data->regmap_conf->max_register;
	unsigned int rx_max = rx_base + (num_ch * ch_stride);
	unsigned int tx_max = tx_base + (num_ch * ch_stride);

	if ((reg >= rx_base) && (reg < rx_max)) {
		reg = (reg - rx_base) % ch_stride;
		if ((reg == TEGRA_ADMAIF_RX_ENABLE) ||
		    (reg == TEGRA_ADMAIF_RX_STATUS) ||
		    (reg == TEGRA_ADMAIF_RX_INT_STATUS) ||
		    (reg == TEGRA_ADMAIF_RX_SOFT_RESET))
			return true;
	} else if ((reg >= tx_base) && (reg < tx_max)) {
		reg = (reg - tx_base) % ch_stride;
		if ((reg == TEGRA_ADMAIF_TX_ENABLE) ||
		    (reg == TEGRA_ADMAIF_TX_STATUS) ||
		    (reg == TEGRA_ADMAIF_TX_INT_STATUS) ||
		    (reg == TEGRA_ADMAIF_TX_SOFT_RESET))
			return true;
	} else if ((reg >= global_base) && (reg < reg_max)) {
		if ((reg == (global_base + TEGRA_ADMAIF_GLOBAL_STATUS)) ||
		    (reg == (global_base +
				TEGRA_ADMAIF_GLOBAL_RX_ENABLE_STATUS)) ||
		    (reg == (global_base +
				TEGRA_ADMAIF_GLOBAL_TX_ENABLE_STATUS)))
			return true;
	}

	return false;
}

static const struct regmap_config tegra210_admaif_regmap_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.max_register		= TEGRA210_ADMAIF_LAST_REG,
	.writeable_reg		= tegra_admaif_wr_reg,
	.readable_reg		= tegra_admaif_rd_reg,
	.volatile_reg		= tegra_admaif_volatile_reg,
	.reg_defaults		= tegra210_admaif_reg_defaults,
	.num_reg_defaults	= TEGRA210_ADMAIF_CHANNEL_COUNT * 6 + 1,
	.cache_type		= REGCACHE_FLAT,
};

static const struct regmap_config tegra186_admaif_regmap_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.max_register		= TEGRA186_ADMAIF_LAST_REG,
	.writeable_reg		= tegra_admaif_wr_reg,
	.readable_reg		= tegra_admaif_rd_reg,
	.volatile_reg		= tegra_admaif_volatile_reg,
	.reg_defaults		= tegra186_admaif_reg_defaults,
	.num_reg_defaults	= TEGRA186_ADMAIF_CHANNEL_COUNT * 6 + 1,
	.cache_type		= REGCACHE_FLAT,
};

static const struct regmap_config tegra264_admaif_regmap_config = {
	.reg_bits		= 32,
	.reg_stride		= 4,
	.val_bits		= 32,
	.max_register		= TEGRA264_ADMAIF_LAST_REG,
	.writeable_reg		= tegra_admaif_wr_reg,
	.readable_reg		= tegra_admaif_rd_reg,
	.volatile_reg		= tegra_admaif_volatile_reg,
	.reg_defaults		= tegra264_admaif_reg_defaults,
	.num_reg_defaults	= TEGRA264_ADMAIF_CHANNEL_COUNT * 6 + 1,
	.cache_type		= REGCACHE_FLAT,
};

static int tegra_admaif_runtime_suspend(struct device *dev)
{
	struct tegra_admaif *admaif = dev_get_drvdata(dev);

	regcache_cache_only(admaif->regmap, true);
	regcache_mark_dirty(admaif->regmap);

	return 0;
}

static int tegra_admaif_runtime_resume(struct device *dev)
{
	struct tegra_admaif *admaif = dev_get_drvdata(dev);

	regcache_cache_only(admaif->regmap, false);
	regcache_sync(admaif->regmap);

	return 0;
}

static int tegra_admaif_set_pack_mode(struct regmap *map, unsigned int reg,
				      int valid_bit)
{
	switch (valid_bit) {
	case DATA_8BIT:
		regmap_update_bits(map, reg, PACK8_EN_MASK, PACK8_EN);
		regmap_update_bits(map, reg, PACK16_EN_MASK, 0);
		break;
	case DATA_16BIT:
		regmap_update_bits(map, reg, PACK16_EN_MASK, PACK16_EN);
		regmap_update_bits(map, reg, PACK8_EN_MASK, 0);
		break;
	case DATA_32BIT:
		regmap_update_bits(map, reg, PACK16_EN_MASK, 0);
		regmap_update_bits(map, reg, PACK8_EN_MASK, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra_admaif_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	return tegra_isomgr_adma_setbw(substream, dai, true);
}

static void tegra_admaif_shutdown(struct snd_pcm_substream *substream,
				  struct snd_soc_dai *dai)
{
	tegra_isomgr_adma_setbw(substream, dai, false);
}

static int tegra_admaif_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params,
				  struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	struct tegra_cif_conf cif_conf;
	unsigned int reg, path;
	int valid_bit, channels;

	memset(&cif_conf, 0, sizeof(struct tegra_cif_conf));

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		cif_conf.audio_bits = TEGRA_ACIF_BITS_8;
		cif_conf.client_bits = TEGRA_ACIF_BITS_8;
		valid_bit = DATA_8BIT;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf.audio_bits = TEGRA_ACIF_BITS_16;
		cif_conf.client_bits = TEGRA_ACIF_BITS_16;
		valid_bit = DATA_16BIT;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		cif_conf.audio_bits = TEGRA_ACIF_BITS_32;
		cif_conf.client_bits = TEGRA_ACIF_BITS_24;
		valid_bit = DATA_32BIT;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf.audio_bits = TEGRA_ACIF_BITS_32;
		cif_conf.client_bits = TEGRA_ACIF_BITS_32;
		valid_bit  = DATA_32BIT;
		break;
	default:
		dev_err(dev, "unsupported format!\n");
		return -EOPNOTSUPP;
	}

	channels = params_channels(params);
	cif_conf.client_ch = channels;
	cif_conf.audio_ch = channels;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		path = ADMAIF_TX_PATH;
		reg = CH_TX_REG(TEGRA_ADMAIF_CH_ACIF_TX_CTRL, dai->id);
	} else {
		path = ADMAIF_RX_PATH;
		reg = CH_RX_REG(TEGRA_ADMAIF_CH_ACIF_RX_CTRL, dai->id);
	}

	cif_conf.mono_conv = admaif->mono_to_stereo[path][dai->id];
	cif_conf.stereo_conv = admaif->stereo_to_mono[path][dai->id];

	tegra_admaif_set_pack_mode(admaif->regmap, reg, valid_bit);

	if (admaif->soc_data->max_stream_ch == TEGRA264_ADMAIF_MAX_CHANNEL)
		tegra264_set_cif(admaif->regmap, reg, &cif_conf);
	else
		tegra_set_cif(admaif->regmap, reg, &cif_conf);

	return 0;
}

static int tegra_admaif_start(struct snd_soc_dai *dai, int direction)
{
	struct tegra_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int reg, mask, val;

	switch (direction) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		mask = TX_ENABLE_MASK;
		val = TX_ENABLE;
		reg = CH_TX_REG(TEGRA_ADMAIF_TX_ENABLE, dai->id);
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		mask = RX_ENABLE_MASK;
		val = RX_ENABLE;
		reg = CH_RX_REG(TEGRA_ADMAIF_RX_ENABLE, dai->id);
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(admaif->regmap, reg, mask, val);

	return 0;
}

static int tegra_admaif_stop(struct snd_soc_dai *dai, int direction)
{
	struct tegra_admaif *admaif = snd_soc_dai_get_drvdata(dai);
	unsigned int enable_reg, status_reg, reset_reg, mask, val;
	char *dir_name;
	int err, enable;

	switch (direction) {
	case SNDRV_PCM_STREAM_PLAYBACK:
		mask = TX_ENABLE_MASK;
		enable = TX_ENABLE;
		dir_name = "TX";
		enable_reg = CH_TX_REG(TEGRA_ADMAIF_TX_ENABLE, dai->id);
		status_reg = CH_TX_REG(TEGRA_ADMAIF_TX_STATUS, dai->id);
		reset_reg = CH_TX_REG(TEGRA_ADMAIF_TX_SOFT_RESET, dai->id);
		break;
	case SNDRV_PCM_STREAM_CAPTURE:
		mask = RX_ENABLE_MASK;
		enable = RX_ENABLE;
		dir_name = "RX";
		enable_reg = CH_RX_REG(TEGRA_ADMAIF_RX_ENABLE, dai->id);
		status_reg = CH_RX_REG(TEGRA_ADMAIF_RX_STATUS, dai->id);
		reset_reg = CH_RX_REG(TEGRA_ADMAIF_RX_SOFT_RESET, dai->id);
		break;
	default:
		return -EINVAL;
	}

	/* Disable TX/RX channel */
	regmap_update_bits(admaif->regmap, enable_reg, mask, ~enable);

	/* Wait until ADMAIF TX/RX status is disabled */
	err = regmap_read_poll_timeout_atomic(admaif->regmap, status_reg, val,
					      !(val & enable), 10, 10000);
	if (err < 0)
		dev_warn(dai->dev, "timeout: failed to disable ADMAIF%d_%s\n",
			 dai->id + 1, dir_name);

	/* SW reset */
	regmap_update_bits(admaif->regmap, reset_reg, SW_RESET_MASK, SW_RESET);

	/* Wait till SW reset is complete */
	err = regmap_read_poll_timeout_atomic(admaif->regmap, reset_reg, val,
					      !(val & SW_RESET_MASK & SW_RESET),
					      10, 10000);
	if (err) {
		dev_err(dai->dev, "timeout: SW reset failed for ADMAIF%d_%s\n",
			dai->id + 1, dir_name);
		return err;
	}

	return 0;
}

static int tegra_admaif_trigger(struct snd_pcm_substream *substream, int cmd,
				struct snd_soc_dai *dai)
{
	int err;

	err = snd_dmaengine_pcm_trigger(substream, cmd);
	if (err)
		return err;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		return tegra_admaif_start(dai, substream->stream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return tegra_admaif_stop(dai, substream->stream);
	default:
		return -EINVAL;
	}
}

static int tegra210_admaif_pget_mono_to_stereo(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;

	ucontrol->value.enumerated.item[0] =
		admaif->mono_to_stereo[ADMAIF_TX_PATH][ec->reg];

	return 0;
}

static int tegra210_admaif_pput_mono_to_stereo(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;
	unsigned int value = ucontrol->value.enumerated.item[0];

	if (value == admaif->mono_to_stereo[ADMAIF_TX_PATH][ec->reg])
		return 0;

	admaif->mono_to_stereo[ADMAIF_TX_PATH][ec->reg] = value;

	return 1;
}

static int tegra210_admaif_cget_mono_to_stereo(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;

	ucontrol->value.enumerated.item[0] =
		admaif->mono_to_stereo[ADMAIF_RX_PATH][ec->reg];

	return 0;
}

static int tegra210_admaif_cput_mono_to_stereo(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;
	unsigned int value = ucontrol->value.enumerated.item[0];

	if (value == admaif->mono_to_stereo[ADMAIF_RX_PATH][ec->reg])
		return 0;

	admaif->mono_to_stereo[ADMAIF_RX_PATH][ec->reg] = value;

	return 1;
}

static int tegra210_admaif_pget_stereo_to_mono(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;

	ucontrol->value.enumerated.item[0] =
		admaif->stereo_to_mono[ADMAIF_TX_PATH][ec->reg];

	return 0;
}

static int tegra210_admaif_pput_stereo_to_mono(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;
	unsigned int value = ucontrol->value.enumerated.item[0];

	if (value == admaif->stereo_to_mono[ADMAIF_TX_PATH][ec->reg])
		return 0;

	admaif->stereo_to_mono[ADMAIF_TX_PATH][ec->reg] = value;

	return 1;
}

static int tegra210_admaif_cget_stereo_to_mono(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;

	ucontrol->value.enumerated.item[0] =
		admaif->stereo_to_mono[ADMAIF_RX_PATH][ec->reg];

	return 0;
}

static int tegra210_admaif_cput_stereo_to_mono(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *cmpnt = snd_kcontrol_chip(kcontrol);
	struct tegra_admaif *admaif = snd_soc_component_get_drvdata(cmpnt);
	struct soc_enum *ec = (struct soc_enum *)kcontrol->private_value;
	unsigned int value = ucontrol->value.enumerated.item[0];

	if (value == admaif->stereo_to_mono[ADMAIF_RX_PATH][ec->reg])
		return 0;

	admaif->stereo_to_mono[ADMAIF_RX_PATH][ec->reg] = value;

	return 1;
}

static int tegra_admaif_dai_probe(struct snd_soc_dai *dai)
{
	struct tegra_admaif *admaif = snd_soc_dai_get_drvdata(dai);

	snd_soc_dai_init_dma_data(dai,	&admaif->playback_dma_data[dai->id],
					&admaif->capture_dma_data[dai->id]);

	return 0;
}

static const struct snd_soc_dai_ops tegra_admaif_dai_ops = {
	.probe		= tegra_admaif_dai_probe,
	.hw_params	= tegra_admaif_hw_params,
	.trigger	= tegra_admaif_trigger,
	.shutdown	= tegra_admaif_shutdown,
	.prepare	= tegra_admaif_prepare,
};

#define DAI(dai_name, channel)					\
	{							\
		.name = dai_name,				\
		.playback = {					\
			.stream_name = dai_name " Playback",	\
			.channels_min = 1,			\
			.channels_max = channel,		\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = dai_name " Capture",	\
			.channels_min = 1,			\
			.channels_max = channel,		\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.ops = &tegra_admaif_dai_ops,			\
	}

static struct snd_soc_dai_driver tegra210_admaif_cmpnt_dais[] = {
	DAI("ADMAIF1", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF2", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF3", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF4", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF5", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF6", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF7", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF8", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF9", TEGRA210_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF10", TEGRA210_ADMAIF_MAX_CHANNEL),
};

static struct snd_soc_dai_driver tegra186_admaif_cmpnt_dais[] = {
	DAI("ADMAIF1", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF2", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF3", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF4", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF5", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF6", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF7", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF8", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF9", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF10", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF11", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF12", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF13", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF14", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF15", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF16", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF17", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF18", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF19", TEGRA186_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF20", TEGRA186_ADMAIF_MAX_CHANNEL),
};

static struct snd_soc_dai_driver tegra264_admaif_cmpnt_dais[] = {
	DAI("ADMAIF1", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF2", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF3", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF4", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF5", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF6", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF7", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF8", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF9", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF10", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF11", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF12", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF13", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF14", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF15", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF16", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF17", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF18", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF19", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF20", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF21", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF22", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF23", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF24", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF25", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF26", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF27", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF28", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF29", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF30", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF31", TEGRA264_ADMAIF_MAX_CHANNEL),
	DAI("ADMAIF32", TEGRA264_ADMAIF_MAX_CHANNEL),
};

static const char * const tegra_admaif_stereo_conv_text[] = {
	"CH0", "CH1", "AVG",
};

static const char * const tegra_admaif_mono_conv_text[] = {
	"Zero", "Copy",
};

/*
 * Below macro is added to avoid looping over all ADMAIFx controls related
 * to mono/stereo conversions in get()/put() callbacks.
 */
#define NV_SOC_ENUM_EXT(xname, xreg, xhandler_get, xhandler_put, xenum_text)   \
{									       \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,				       \
	.info = snd_soc_info_enum_double,				       \
	.name = xname,							       \
	.get = xhandler_get,						       \
	.put = xhandler_put,						       \
	.private_value = (unsigned long)&(struct soc_enum)		       \
		SOC_ENUM_SINGLE(xreg, 0, ARRAY_SIZE(xenum_text), xenum_text)   \
}

#define TEGRA_ADMAIF_CIF_CTRL(reg)					       \
	NV_SOC_ENUM_EXT("ADMAIF" #reg " Playback Mono To Stereo", reg - 1,     \
			tegra210_admaif_pget_mono_to_stereo,		       \
			tegra210_admaif_pput_mono_to_stereo,		       \
			tegra_admaif_mono_conv_text),			       \
	NV_SOC_ENUM_EXT("ADMAIF" #reg " Playback Stereo To Mono", reg - 1,     \
			tegra210_admaif_pget_stereo_to_mono,		       \
			tegra210_admaif_pput_stereo_to_mono,		       \
			tegra_admaif_stereo_conv_text),			       \
	NV_SOC_ENUM_EXT("ADMAIF" #reg " Capture Mono To Stereo", reg - 1,      \
			tegra210_admaif_cget_mono_to_stereo,		       \
			tegra210_admaif_cput_mono_to_stereo,		       \
			tegra_admaif_mono_conv_text),			       \
	NV_SOC_ENUM_EXT("ADMAIF" #reg " Capture Stereo To Mono", reg - 1,      \
			tegra210_admaif_cget_stereo_to_mono,		       \
			tegra210_admaif_cput_stereo_to_mono,		       \
			tegra_admaif_stereo_conv_text)

static struct snd_kcontrol_new tegra210_admaif_controls[] = {
	TEGRA_ADMAIF_CIF_CTRL(1),
	TEGRA_ADMAIF_CIF_CTRL(2),
	TEGRA_ADMAIF_CIF_CTRL(3),
	TEGRA_ADMAIF_CIF_CTRL(4),
	TEGRA_ADMAIF_CIF_CTRL(5),
	TEGRA_ADMAIF_CIF_CTRL(6),
	TEGRA_ADMAIF_CIF_CTRL(7),
	TEGRA_ADMAIF_CIF_CTRL(8),
	TEGRA_ADMAIF_CIF_CTRL(9),
	TEGRA_ADMAIF_CIF_CTRL(10),
};

static struct snd_kcontrol_new tegra186_admaif_controls[] = {
	TEGRA_ADMAIF_CIF_CTRL(1),
	TEGRA_ADMAIF_CIF_CTRL(2),
	TEGRA_ADMAIF_CIF_CTRL(3),
	TEGRA_ADMAIF_CIF_CTRL(4),
	TEGRA_ADMAIF_CIF_CTRL(5),
	TEGRA_ADMAIF_CIF_CTRL(6),
	TEGRA_ADMAIF_CIF_CTRL(7),
	TEGRA_ADMAIF_CIF_CTRL(8),
	TEGRA_ADMAIF_CIF_CTRL(9),
	TEGRA_ADMAIF_CIF_CTRL(10),
	TEGRA_ADMAIF_CIF_CTRL(11),
	TEGRA_ADMAIF_CIF_CTRL(12),
	TEGRA_ADMAIF_CIF_CTRL(13),
	TEGRA_ADMAIF_CIF_CTRL(14),
	TEGRA_ADMAIF_CIF_CTRL(15),
	TEGRA_ADMAIF_CIF_CTRL(16),
	TEGRA_ADMAIF_CIF_CTRL(17),
	TEGRA_ADMAIF_CIF_CTRL(18),
	TEGRA_ADMAIF_CIF_CTRL(19),
	TEGRA_ADMAIF_CIF_CTRL(20),
};

static struct snd_kcontrol_new tegra264_admaif_controls[] = {
	TEGRA_ADMAIF_CIF_CTRL(1),
	TEGRA_ADMAIF_CIF_CTRL(2),
	TEGRA_ADMAIF_CIF_CTRL(3),
	TEGRA_ADMAIF_CIF_CTRL(4),
	TEGRA_ADMAIF_CIF_CTRL(5),
	TEGRA_ADMAIF_CIF_CTRL(6),
	TEGRA_ADMAIF_CIF_CTRL(7),
	TEGRA_ADMAIF_CIF_CTRL(8),
	TEGRA_ADMAIF_CIF_CTRL(9),
	TEGRA_ADMAIF_CIF_CTRL(10),
	TEGRA_ADMAIF_CIF_CTRL(11),
	TEGRA_ADMAIF_CIF_CTRL(12),
	TEGRA_ADMAIF_CIF_CTRL(13),
	TEGRA_ADMAIF_CIF_CTRL(14),
	TEGRA_ADMAIF_CIF_CTRL(15),
	TEGRA_ADMAIF_CIF_CTRL(16),
	TEGRA_ADMAIF_CIF_CTRL(17),
	TEGRA_ADMAIF_CIF_CTRL(18),
	TEGRA_ADMAIF_CIF_CTRL(19),
	TEGRA_ADMAIF_CIF_CTRL(20),
	TEGRA_ADMAIF_CIF_CTRL(21),
	TEGRA_ADMAIF_CIF_CTRL(22),
	TEGRA_ADMAIF_CIF_CTRL(23),
	TEGRA_ADMAIF_CIF_CTRL(24),
	TEGRA_ADMAIF_CIF_CTRL(25),
	TEGRA_ADMAIF_CIF_CTRL(26),
	TEGRA_ADMAIF_CIF_CTRL(27),
	TEGRA_ADMAIF_CIF_CTRL(28),
	TEGRA_ADMAIF_CIF_CTRL(29),
	TEGRA_ADMAIF_CIF_CTRL(30),
	TEGRA_ADMAIF_CIF_CTRL(31),
	TEGRA_ADMAIF_CIF_CTRL(32),
};

static const struct snd_soc_component_driver tegra210_admaif_cmpnt = {
	.controls		= tegra210_admaif_controls,
	.num_controls		= ARRAY_SIZE(tegra210_admaif_controls),
	.pcm_construct		= tegra_pcm_construct,
	.open			= tegra_pcm_open,
	.close			= tegra_pcm_close,
	.hw_params		= tegra_pcm_hw_params,
	.pointer		= tegra_pcm_pointer,
};

static const struct snd_soc_component_driver tegra186_admaif_cmpnt = {
	.controls		= tegra186_admaif_controls,
	.num_controls		= ARRAY_SIZE(tegra186_admaif_controls),
	.pcm_construct		= tegra_pcm_construct,
	.open			= tegra_pcm_open,
	.close			= tegra_pcm_close,
	.hw_params		= tegra_pcm_hw_params,
	.pointer		= tegra_pcm_pointer,
};

static const struct snd_soc_component_driver tegra264_admaif_cmpnt = {
	.controls		= tegra264_admaif_controls,
	.num_controls		= ARRAY_SIZE(tegra264_admaif_controls),
	.pcm_construct		= tegra_pcm_construct,
	.open			= tegra_pcm_open,
	.close			= tegra_pcm_close,
	.hw_params		= tegra_pcm_hw_params,
	.pointer		= tegra_pcm_pointer,
};

static const struct tegra_admaif_soc_data soc_data_tegra210 = {
	.num_ch		= TEGRA210_ADMAIF_CHANNEL_COUNT,
	.max_stream_ch	= TEGRA210_ADMAIF_MAX_CHANNEL,
	.cmpnt		= &tegra210_admaif_cmpnt,
	.dais		= tegra210_admaif_cmpnt_dais,
	.regmap_conf	= &tegra210_admaif_regmap_config,
	.global_base	= TEGRA210_ADMAIF_GLOBAL_BASE,
	.tx_base	= TEGRA210_ADMAIF_TX_BASE,
	.rx_base	= TEGRA210_ADMAIF_RX_BASE,
};

static const struct tegra_admaif_soc_data soc_data_tegra186 = {
	.num_ch		= TEGRA186_ADMAIF_CHANNEL_COUNT,
	.max_stream_ch	= TEGRA186_ADMAIF_MAX_CHANNEL,
	.cmpnt		= &tegra186_admaif_cmpnt,
	.dais		= tegra186_admaif_cmpnt_dais,
	.regmap_conf	= &tegra186_admaif_regmap_config,
	.global_base	= TEGRA186_ADMAIF_GLOBAL_BASE,
	.tx_base	= TEGRA186_ADMAIF_TX_BASE,
	.rx_base	= TEGRA186_ADMAIF_RX_BASE,
};

static const struct tegra_admaif_soc_data soc_data_tegra264 = {
	.num_ch		= TEGRA264_ADMAIF_CHANNEL_COUNT,
	.max_stream_ch	= TEGRA264_ADMAIF_MAX_CHANNEL,
	.cmpnt		= &tegra264_admaif_cmpnt,
	.dais		= tegra264_admaif_cmpnt_dais,
	.regmap_conf	= &tegra264_admaif_regmap_config,
	.global_base	= TEGRA264_ADMAIF_GLOBAL_BASE,
	.tx_base	= TEGRA264_ADMAIF_TX_BASE,
	.rx_base	= TEGRA264_ADMAIF_RX_BASE,
};

static const struct of_device_id tegra_admaif_of_match[] = {
	{ .compatible = "nvidia,tegra210-admaif", .data = &soc_data_tegra210 },
	{ .compatible = "nvidia,tegra186-admaif", .data = &soc_data_tegra186 },
	{ .compatible = "nvidia,tegra264-admaif", .data = &soc_data_tegra264 },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_admaif_of_match);

static int tegra_admaif_probe(struct platform_device *pdev)
{
	struct tegra_admaif *admaif;
	void __iomem *regs;
	struct resource *res;
	int err, i;

	admaif = devm_kzalloc(&pdev->dev, sizeof(*admaif), GFP_KERNEL);
	if (!admaif)
		return -ENOMEM;

	admaif->soc_data = of_device_get_match_data(&pdev->dev);

	dev_set_drvdata(&pdev->dev, admaif);

	admaif->capture_dma_data =
		devm_kcalloc(&pdev->dev,
			     admaif->soc_data->num_ch,
			     sizeof(struct snd_dmaengine_dai_dma_data),
			     GFP_KERNEL);
	if (!admaif->capture_dma_data)
		return -ENOMEM;

	admaif->playback_dma_data =
		devm_kcalloc(&pdev->dev,
			     admaif->soc_data->num_ch,
			     sizeof(struct snd_dmaengine_dai_dma_data),
			     GFP_KERNEL);
	if (!admaif->playback_dma_data)
		return -ENOMEM;

	for (i = 0; i < ADMAIF_PATHS; i++) {
		admaif->mono_to_stereo[i] =
			devm_kcalloc(&pdev->dev, admaif->soc_data->num_ch,
				     sizeof(unsigned int), GFP_KERNEL);
		if (!admaif->mono_to_stereo[i])
			return -ENOMEM;

		admaif->stereo_to_mono[i] =
			devm_kcalloc(&pdev->dev, admaif->soc_data->num_ch,
				     sizeof(unsigned int), GFP_KERNEL);
		if (!admaif->stereo_to_mono[i])
			return -ENOMEM;
	}

	regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	admaif->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					       admaif->soc_data->regmap_conf);
	if (IS_ERR(admaif->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(admaif->regmap);
	}

	regcache_cache_only(admaif->regmap, true);

	err = tegra_isomgr_adma_register(&pdev->dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to add interconnect path\n");
		return err;
	}

	regmap_update_bits(admaif->regmap, admaif->soc_data->global_base +
			   TEGRA_ADMAIF_GLOBAL_ENABLE, 1, 1);

	for (i = 0; i < admaif->soc_data->num_ch; i++) {
		admaif->playback_dma_data[i].addr = res->start +
			CH_TX_REG(TEGRA_ADMAIF_TX_FIFO_WRITE, i);

		admaif->capture_dma_data[i].addr = res->start +
			CH_RX_REG(TEGRA_ADMAIF_RX_FIFO_READ, i);

		admaif->playback_dma_data[i].addr_width = 32;

		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names", (i * 2) + 1,
				&admaif->playback_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"missing property nvidia,dma-names\n");

			return -ENODEV;
		}

		admaif->capture_dma_data[i].addr_width = 32;

		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(i * 2),
				&admaif->capture_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"missing property nvidia,dma-names\n");

			return -ENODEV;
		}
	}

	err = devm_snd_soc_register_component(&pdev->dev,
					      admaif->soc_data->cmpnt,
					      admaif->soc_data->dais,
					      admaif->soc_data->num_ch);
	if (err) {
		dev_err(&pdev->dev,
			"can't register ADMAIF component, err: %d\n", err);
		return err;
	}

	pm_runtime_enable(&pdev->dev);

	return 0;
}

static void tegra_admaif_remove(struct platform_device *pdev)
{
	tegra_isomgr_adma_unregister(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
}

static const struct dev_pm_ops tegra_admaif_pm_ops = {
	RUNTIME_PM_OPS(tegra_admaif_runtime_suspend,
		       tegra_admaif_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static struct platform_driver tegra_admaif_driver = {
	.probe = tegra_admaif_probe,
	.remove = tegra_admaif_remove,
	.driver = {
		.name = "tegra210-admaif",
		.of_match_table = tegra_admaif_of_match,
		.pm = pm_ptr(&tegra_admaif_pm_ops),
	},
};
module_platform_driver(tegra_admaif_driver);

MODULE_AUTHOR("Songhee Baek <sbaek@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 ASoC ADMAIF driver");
MODULE_LICENSE("GPL v2");
