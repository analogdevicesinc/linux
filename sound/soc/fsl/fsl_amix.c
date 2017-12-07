/*
 * NXP AMIX ALSA SoC Digital Audio Interface (DAI) driver
 *
 * Copyright 2017 NXP
 *
 * Author: Viorel Suman <viorel.suman@nxp.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "fsl_amix.h"

#define SOC_ENUM_SINGLE_S(xreg, xshift, xtexts) \
	SOC_ENUM_SINGLE(xreg, xshift, ARRAY_SIZE(xtexts), xtexts)

typedef int (*fsl_amix_state_handler)(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr);

static const char
	*tdm_sel[] = { "TDM1", "TDM2", },
	*mode_sel[] = { "Disabled", "TDM1", "TDM2", "Mixed", },
	*width_sel[] = { "16b", "18b", "20b", "24b", "32b", },
	*pol_sel[] = { "Positive edge", "Negative edge", },
	*endis_sel[] = { "Disabled", "Enabled", },
	*updn_sel[] = { "Downward", "Upward", },
	*mask_sel[] = { "Unmask", "Mask", };

static const struct soc_enum fsl_amix_enum[] = {
/* FSL_AMIX_CTR enums */
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_MIXCLK_SHIFT, tdm_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_OUTSRC_SHIFT, mode_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_OUTWIDTH_SHIFT, width_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_OUTCKPOL_SHIFT, pol_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_MASKRTDF_SHIFT, mask_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_MASKCKDF_SHIFT, mask_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_SYNCMODE_SHIFT, endis_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_CTR, FSL_AMIX_CTR_SYNCSRC_SHIFT, tdm_sel),
/* FSL_AMIX_ATCR0 enums */
SOC_ENUM_SINGLE_S(FSL_AMIX_ATCR0, 0, endis_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_ATCR0, 1, updn_sel),
/* FSL_AMIX_ATCR1 enums */
SOC_ENUM_SINGLE_S(FSL_AMIX_ATCR1, 0, endis_sel),
SOC_ENUM_SINGLE_S(FSL_AMIX_ATCR1, 1, updn_sel),
};

static int fsl_amix_state_dis_tdm1(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce the proper TDM is started */
	if (!(priv->tdms & BIT(0))) {
		dev_err(comp->dev, "DIS->TDM1: TDM1 is not started!\n");
		return -EINVAL;
	}
	/* Set mix clock */
	(*mask) |= FSL_AMIX_CTR_MIXCLK_MASK;
	(*ctr)  |= FSL_AMIX_CTR_MIXCLK(0);
	return 0;
}

static int fsl_amix_state_dis_tdm2(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce the proper TDM is started */
	if (!(priv->tdms & BIT(1))) {
		dev_err(comp->dev, "DIS->TDM2: TDM2 is not started!\n");
		return -EINVAL;
	}
	/* Set mix clock */
	(*mask) |= FSL_AMIX_CTR_MIXCLK_MASK;
	(*ctr)  |= FSL_AMIX_CTR_MIXCLK(1);
	return 0;
}

static int fsl_amix_state_tdm1_dis(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce the proper TDM is started */
	if (!(priv->tdms & BIT(0))) {
		dev_err(comp->dev, "TDM1->DIS: TDM1 is not started!\n");
		return -EINVAL;
	}
	/* Keep mix clock unchanged */
	return 0;
}

static int fsl_amix_state_tdm2_dis(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce the proper TDM is started */
	if (!(priv->tdms & BIT(1))) {
		dev_err(comp->dev, "TDM2->DIS: TDM2 is not started!\n");
		return -EINVAL;
	}
	/* Keep mix clock unchanged */
	return 0;
}

static int fsl_amix_state_dis_mix(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "DIS->MIX: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Keep mix clock unchanged */
	return 0;
}

static int fsl_amix_state_tdm1_tdm2(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "TDM1->TDM2: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Set mix clock */
	(*mask) |= FSL_AMIX_CTR_MIXCLK_MASK;
	(*ctr)  |= FSL_AMIX_CTR_MIXCLK(1);
	return 0;
}

static int fsl_amix_state_tdm2_tdm1(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "TDM2->TDM1: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Set mix clock */
	(*mask) |= FSL_AMIX_CTR_MIXCLK_MASK;
	(*ctr)  |= FSL_AMIX_CTR_MIXCLK(0);
	return 0;
}

static int fsl_amix_state_tdm1_mix(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "TDM1->MIX: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Keep mix clock unchanged */
	return 0;
}

static int fsl_amix_state_tdm2_mix(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "TDM2->MIX: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Keep mix clock unchanged */
	return 0;
}

static int fsl_amix_state_mix_tdm1(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "MIX->TDM1: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Set mix clock */
	(*mask) |= FSL_AMIX_CTR_MIXCLK_MASK;
	(*ctr)  |= FSL_AMIX_CTR_MIXCLK(0);
	return 0;
}

static int fsl_amix_state_mix_tdm2(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "MIX->TDM2: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Set mix clock */
	(*mask) |= FSL_AMIX_CTR_MIXCLK_MASK;
	(*ctr)  |= FSL_AMIX_CTR_MIXCLK(1);
	return 0;
}

static int fsl_amix_state_mix_dis(struct snd_soc_component *comp,
	unsigned int *mask, unsigned int *ctr)
{
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	/* Enforce all TDMs are started */
	if (priv->tdms != 3) {
		dev_err(comp->dev, "MIX->DIS: Please start both TDMs!\n");
		return -EINVAL;
	}
	/* Keep mix clock unchanged */
	return 0;
}

static const fsl_amix_state_handler state_machine[4][4] = {
	/* From Disabled */
	{
		0, /* To Disabled, do nothing */
		fsl_amix_state_dis_tdm1, /* To TDM1*/
		fsl_amix_state_dis_tdm2, /* To TDM2 */
		fsl_amix_state_dis_mix   /* To Mixed */
	},
	/* From TDM1 */
	{
		fsl_amix_state_tdm1_dis,  /* To Disabled */
		0, /* To TDM1, do nothing */
		fsl_amix_state_tdm1_tdm2, /* To TDM2 */
		fsl_amix_state_tdm1_mix   /* To Mixed */
	},
	/* From TDM2 */
	{
		fsl_amix_state_tdm2_dis,  /* To Disabled */
		fsl_amix_state_tdm2_tdm1, /* To TDM1 */
		0, /* To TDM2, do nothing */
		fsl_amix_state_tdm2_mix   /* To Mixed */
	},
	/* From Mixed */
	{
		fsl_amix_state_mix_dis,  /* To Disabled */
		fsl_amix_state_mix_tdm1, /* To TDM1 */
		fsl_amix_state_mix_tdm2, /* To TDM2 */
		0  /* To Mixed, do nothing */
	}
};

static int fsl_amix_put_mix_clk_src(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	unsigned int reg_val, val, mix_clk;
	int ret = 0;

	/* Get current state */
	ret = snd_soc_component_read(comp, FSL_AMIX_CTR, &reg_val);
	if (ret)
		return ret;

	mix_clk = reg_val & 1;
	val = snd_soc_enum_item_to_val(e, item[0]);

	dev_dbg(comp->dev, "[%s]: TDMs=x%08x, val=x%08x\n", __func__, priv->tdms, val);

	/**
	 * Ensure the current selected mixer clock is available
	 * for configuration propagation
	 */
	if (!(priv->tdms & BIT(mix_clk))) {
		dev_err(comp->dev, "MIXCLK: A started TDM%d is required "
			"for configuration propagation!\n", mix_clk + 1);
		return -EINVAL;
	}

	if (!(priv->tdms & BIT(val))) {
		dev_err(comp->dev, "The selected clock source has "
			"no TDM%d enabled!\n", val + 1);
		return -EINVAL;
	}

	return snd_soc_put_enum_double(kcontrol, ucontrol);
}

static int fsl_amix_put_out_src(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_component *comp = snd_kcontrol_chip(kcontrol);
	struct fsl_amix *priv = snd_soc_component_get_drvdata(comp);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	u32 out_src, mix_clk;
	unsigned int reg_val, val, mask = 0, ctr = 0;
	int ret = 0;

	/* Get current state */
	ret = snd_soc_component_read(comp, FSL_AMIX_CTR, &reg_val);
	if (ret)
		return ret;

	/* "From" state */
	out_src = ((reg_val & FSL_AMIX_CTR_OUTSRC_MASK) >> FSL_AMIX_CTR_OUTSRC_SHIFT);
	mix_clk = reg_val & 1;

	/* "To" state */
	val = snd_soc_enum_item_to_val(e, item[0]);

	dev_dbg(comp->dev, "[%s]: TDMs=x%08x, val=x%08x\n", __func__, priv->tdms, val);

	/* Check if state is changing ... */
	if (!state_machine[out_src][val])
		return 0;
	/**
	 * Ensure the current selected mixer clock is available
	 * for configuration propagation
	 */
	if (!(priv->tdms & BIT(mix_clk))) {
		dev_err(comp->dev, "MIXCLK: A started TDM%d is required "
			"for configuration propagation!\n", mix_clk + 1);
		return -EINVAL;
	}
	/* Check state transition constraints */
	ret = state_machine[out_src][val](comp, &mask, &ctr);
	if (ret)
		return ret;

	/* Complete transition to new state */
	mask |= FSL_AMIX_CTR_OUTSRC_MASK;
	ctr  |= FSL_AMIX_CTR_OUTSRC(val);

	return snd_soc_component_update_bits(comp, FSL_AMIX_CTR, mask, ctr);
}

static const struct snd_kcontrol_new fsl_amix_snd_controls[] = {
	/* FSL_AMIX_CTR controls */
	SOC_ENUM_EXT("Mixing Clock Source", fsl_amix_enum[0],
		snd_soc_get_enum_double, fsl_amix_put_mix_clk_src),
	SOC_ENUM_EXT("Output Source", fsl_amix_enum[1],
		snd_soc_get_enum_double, fsl_amix_put_out_src),
	SOC_ENUM("Output Width", fsl_amix_enum[2]),
	SOC_ENUM("Output Clock Polarity", fsl_amix_enum[3]),
	SOC_ENUM("Frame Rate Diff Error", fsl_amix_enum[4]),
	SOC_ENUM("Clock Freq Diff Error", fsl_amix_enum[5]),
	SOC_ENUM("Sync Mode Config", fsl_amix_enum[6]),
	SOC_ENUM("Sync Mode Clk Source", fsl_amix_enum[7]),
	/* TDM1 Attenuation controls */
	SOC_ENUM("TDM1 Attenuation", fsl_amix_enum[8]),
	SOC_ENUM("TDM1 Attenuation Direction", fsl_amix_enum[9]),
	SOC_SINGLE("TDM1 Attenuation Step Divider", FSL_AMIX_ATCR0,
			2, 0x00fff, 0),
	SOC_SINGLE("TDM1 Attenuation Initial Value", FSL_AMIX_ATIVAL0,
			0, 0x3ffff, 0),
	SOC_SINGLE("TDM1 Attenuation Step Up Factor", FSL_AMIX_ATSTPUP0,
			0, 0x3ffff, 0),
	SOC_SINGLE("TDM1 Attenuation Step Down Factor", FSL_AMIX_ATSTPDN0,
			0, 0x3ffff, 0),
	SOC_SINGLE("TDM1 Attenuation Step Target", FSL_AMIX_ATSTPTGT0,
			0, 0x3ffff, 0),
	/* TDM2 Attenuation controls */
	SOC_ENUM("TDM2 Attenuation", fsl_amix_enum[10]),
	SOC_ENUM("TDM2 Attenuation Direction", fsl_amix_enum[11]),
	SOC_SINGLE("TDM2 Attenuation Step Divider", FSL_AMIX_ATCR1,
			2, 0x00fff, 0),
	SOC_SINGLE("TDM2 Attenuation Initial Value", FSL_AMIX_ATIVAL1,
			0, 0x3ffff, 0),
	SOC_SINGLE("TDM2 Attenuation Step Up Factor", FSL_AMIX_ATSTPUP1,
			0, 0x3ffff, 0),
	SOC_SINGLE("TDM2 Attenuation Step Down Factor", FSL_AMIX_ATSTPDN1,
			0, 0x3ffff, 0),
	SOC_SINGLE("TDM2 Attenuation Step Target", FSL_AMIX_ATSTPTGT1,
			0, 0x3ffff, 0),
};

static int fsl_amix_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct fsl_amix *priv = snd_soc_dai_get_drvdata(dai);
	u32 mask = 0, ctr = 0;

	/* AMIX is working in DSP_A format only */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		break;
	default:
		return -EINVAL;
	}

	/* For playback the AMIX is slave, and for record is master */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_NF:
		/* Output data will be written on positive edge of the clock */
		ctr |= FSL_AMIX_CTR_OUTCKPOL(0);
		break;
	case SND_SOC_DAIFMT_NB_NF:
		/* Output data will be written on negative edge of the clock */
		ctr |= FSL_AMIX_CTR_OUTCKPOL(1);
		break;
	default:
		return -EINVAL;
	}

	mask |= FSL_AMIX_CTR_OUTCKPOL_MASK;

	return regmap_update_bits(priv->regmap, FSL_AMIX_CTR, mask, ctr);
}

static int fsl_amix_dai_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct fsl_amix *priv = snd_soc_dai_get_drvdata(dai);

	/* Capture stream shall not be handled */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		return 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		priv->tdms |= BIT(dai->driver->id);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		priv->tdms &= ~BIT(dai->driver->id);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct snd_soc_dai_ops fsl_amix_dai_ops = {
	.set_fmt      = fsl_amix_dai_set_fmt,
	.trigger      = fsl_amix_dai_trigger,
};

static struct snd_soc_dai_driver fsl_amix_dai[] = {
	{
		.id   = 0,
		.name = "amix-0",
		.playback = {
			.stream_name = "AMIX-Playback-0",
			.channels_min = 8,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 96000,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = FSL_AMIX_FORMATS,
		},
		.capture = {
			.stream_name = "AMIX-Capture-0",
			.channels_min = 8,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 96000,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = FSL_AMIX_FORMATS,
		},
		.ops = &fsl_amix_dai_ops,
	},
	{
		.id   = 1,
		.name = "amix-1",
		.playback = {
			.stream_name = "AMIX-Playback-1",
			.channels_min = 8,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 96000,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = FSL_AMIX_FORMATS,
		},
		.capture = {
			.stream_name = "AMIX-Capture-1",
			.channels_min = 8,
			.channels_max = 8,
			.rate_min = 8000,
			.rate_max = 96000,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = FSL_AMIX_FORMATS,
		},
		.ops = &fsl_amix_dai_ops,
	},
};

static const struct snd_soc_component_driver fsl_amix_component = {
	.name		  = "fsl-amix-dai",
	.controls	  = fsl_amix_snd_controls,
	.num_controls	  = ARRAY_SIZE(fsl_amix_snd_controls),
};

static bool fsl_amix_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_AMIX_CTR:
	case FSL_AMIX_STR:
	case FSL_AMIX_ATCR0:
	case FSL_AMIX_ATIVAL0:
	case FSL_AMIX_ATSTPUP0:
	case FSL_AMIX_ATSTPDN0:
	case FSL_AMIX_ATSTPTGT0:
	case FSL_AMIX_ATTNVAL0:
	case FSL_AMIX_ATSTP0:
	case FSL_AMIX_ATCR1:
	case FSL_AMIX_ATIVAL1:
	case FSL_AMIX_ATSTPUP1:
	case FSL_AMIX_ATSTPDN1:
	case FSL_AMIX_ATSTPTGT1:
	case FSL_AMIX_ATTNVAL1:
	case FSL_AMIX_ATSTP1:
		return true;
	default:
		return false;
	}
}

static bool fsl_amix_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FSL_AMIX_CTR:
	case FSL_AMIX_ATCR0:
	case FSL_AMIX_ATIVAL0:
	case FSL_AMIX_ATSTPUP0:
	case FSL_AMIX_ATSTPDN0:
	case FSL_AMIX_ATSTPTGT0:
	case FSL_AMIX_ATCR1:
	case FSL_AMIX_ATIVAL1:
	case FSL_AMIX_ATSTPUP1:
	case FSL_AMIX_ATSTPDN1:
	case FSL_AMIX_ATSTPTGT1:
		return true;
	default:
		return false;
	}
}

static struct reg_default fsl_amix_reg[] = {
	{ FSL_AMIX_CTR,       0x00060 },
	{ FSL_AMIX_STR,       0x00003 },
	{ FSL_AMIX_ATCR0,     0x00000 },
	{ FSL_AMIX_ATIVAL0,   0x3FFFF },
	{ FSL_AMIX_ATSTPUP0,  0x2AAAA },
	{ FSL_AMIX_ATSTPDN0,  0x30000 },
	{ FSL_AMIX_ATSTPTGT0, 0x00010 },
	{ FSL_AMIX_ATTNVAL0,  0x00000 },
	{ FSL_AMIX_ATSTP0,    0x00000 },
	{ FSL_AMIX_ATCR1,     0x00000 },
	{ FSL_AMIX_ATIVAL1,   0x3FFFF },
	{ FSL_AMIX_ATSTPUP1,  0x2AAAA },
	{ FSL_AMIX_ATSTPDN1,  0x30000 },
	{ FSL_AMIX_ATSTPTGT1, 0x00010 },
	{ FSL_AMIX_ATTNVAL1,  0x00000 },
	{ FSL_AMIX_ATSTP1,    0x00000 },
};

static const struct regmap_config fsl_amix_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = FSL_AMIX_ATSTP1,
	.reg_defaults = fsl_amix_reg,
	.num_reg_defaults = ARRAY_SIZE(fsl_amix_reg),
	.readable_reg = fsl_amix_readable_reg,
	.writeable_reg = fsl_amix_writeable_reg,
	.cache_type = REGCACHE_FLAT,
};

static int fsl_amix_probe(struct platform_device *pdev)
{
	struct fsl_amix *priv;
	struct resource *res;
	void __iomem *regs;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->pdev = pdev;

	/* Get the addresses */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	priv->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "ipg", regs,
						 &fsl_amix_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(&pdev->dev, "failed to init regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(priv->ipg_clk)) {
		dev_err(&pdev->dev, "failed to get ipg clock\n");
		return PTR_ERR(priv->ipg_clk);
	}

	platform_set_drvdata(pdev, priv);
	pm_runtime_enable(&pdev->dev);

	ret = devm_snd_soc_register_component(&pdev->dev, &fsl_amix_component,
					      fsl_amix_dai, ARRAY_SIZE(fsl_amix_dai));
	if (ret) {
		dev_err(&pdev->dev, "failed to register ASoC DAI\n");
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM
static int fsl_amix_runtime_resume(struct device *dev)
{
	struct fsl_amix *priv = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(priv->ipg_clk);
	if (ret) {
		dev_err(dev, "Failed to enable IPG clock: %d\n", ret);
		return ret;
	}

	regcache_cache_only(priv->regmap, false);
	regcache_mark_dirty(priv->regmap);

	return regcache_sync(priv->regmap);
}

static int fsl_amix_runtime_suspend(struct device *dev)
{
	struct fsl_amix *priv = dev_get_drvdata(dev);

	regcache_cache_only(priv->regmap, true);

	clk_disable_unprepare(priv->ipg_clk);

	return 0;
}
#endif /* CONFIG_PM */

static const struct dev_pm_ops fsl_amix_pm = {
	SET_RUNTIME_PM_OPS(fsl_amix_runtime_suspend, fsl_amix_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static const struct of_device_id fsl_amix_ids[] = {
	{ .compatible = "fsl,imx8qm-amix", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_amix_ids);

static struct platform_driver fsl_amix_driver = {
	.probe = fsl_amix_probe,
	.driver = {
		.name = "fsl-amix",
		.of_match_table = fsl_amix_ids,
		.pm = &fsl_amix_pm,
	},
};
module_platform_driver(fsl_amix_driver);

MODULE_DESCRIPTION("NXP AMIX ASoC DAI driver");
MODULE_AUTHOR("Viorel Suman <viorel.suman@nxp.com>");
MODULE_ALIAS("platform:fsl-amix");
MODULE_LICENSE("GPL v2");
