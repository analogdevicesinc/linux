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

static const struct snd_kcontrol_new fsl_amix_snd_controls[] = {
	/* FSL_AMIX_CTR controls */
	SOC_ENUM("Mixing Clock Source", fsl_amix_enum[0]),
	SOC_ENUM("Output Source", fsl_amix_enum[1]),
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

struct amix_dais_st {
	u8 input;
	u8 output;
};

static void fsl_amix_get_dais_status(struct snd_soc_pcm_runtime *be_rtd,
				     struct amix_dais_st *status)
{
	struct snd_soc_dpcm *dpcm;
	struct snd_soc_pcm_runtime *fe_rtd = NULL;
	struct snd_pcm_substream *fe_ss = NULL;
	struct snd_soc_dai *fe_dai, *be_dai = be_rtd->cpu_dai;
	int stream = SNDRV_PCM_STREAM_PLAYBACK;

	/* find active frontends for this backend */
	list_for_each_entry(dpcm, &be_rtd->dpcm[stream].fe_clients, list_fe) {
		if (dpcm->be != be_rtd)
			continue;

		fe_rtd = dpcm->fe;
		fe_dai = fe_rtd->cpu_dai;
		fe_ss  = snd_soc_dpcm_get_substream(fe_rtd, stream);

		if (fe_ss && fe_ss->pcm->device < FSL_AMIX_MAX_DAIS) {
			if (fe_dai->playback_active)
				status->input |= BIT(fe_ss->pcm->device);
			if (fe_dai->capture_active)
				status->output |= BIT(fe_ss->pcm->device);
		} else if (fe_ss) {
			dev_err(be_dai->dev, "Wrong device number: %d\n",
				fe_ss->pcm->device);
		}
	}
}

static int fsl_amix_update(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct fsl_amix *priv = snd_soc_dai_get_drvdata(dai);
	u32 mask = 0, ctr = 0, val, old_input;
	struct amix_dais_st status = { .input = 0, .output = 0 };

	fsl_amix_get_dais_status(rtd, &status);

	regmap_read(priv->regmap, FSL_AMIX_CTR, &val);
	old_input = ((val & FSL_AMIX_CTR_OUTSRC_MASK) >> FSL_AMIX_CTR_OUTSRC_SHIFT);

	dev_dbg(dai->dev, "old_input=x%08x, input=x%08x\n", old_input, status.input);

	if (old_input == status.input) {
		dev_dbg(dai->dev, "State unchanged, input=x%08x\n", status.input);
		return 0;
	}

	/**
	 * Mixing clock selection. Make sure the currently selected clock source
	 * has the TDM enabled. If not, switch clock source to enabled TDM.
	 */
	switch (status.input) {
	case 0: /* Both TDMs are disabled, keep the clock source unchanged */
	case 3: /* Both TDMs are enabled, keep the clock source unchanged */
		break;
	case 1:
	case 2:
		val &= 1;
		if (FSL_AMIX_CTR_MIXCLK(status.input) != val) {
			mask |= FSL_AMIX_CTR_MIXCLK_MASK;
			ctr  |= FSL_AMIX_CTR_MIXCLK(status.input);
		}
		break;
	}

	/* Output source selection */
	mask |= FSL_AMIX_CTR_OUTSRC_MASK;
	ctr  |= FSL_AMIX_CTR_OUTSRC(status.input);

	regmap_update_bits(priv->regmap, FSL_AMIX_CTR, mask, ctr);

	dev_dbg(dai->dev, "Set AMIX_CTR[0x%08x]=0x%08x, in=0x%08x, out=0x%08x\n",
		mask, ctr, status.input, status.output);

	return 0;
}

static int fsl_amix_dai_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_card *card = dai->component->card;
	struct snd_soc_pcm_runtime *rtd;

	rtd = snd_soc_get_pcm_runtime(card, "HiFi-AMIX-BE");

	return fsl_amix_update(rtd);
}

static void fsl_amix_dai_shutdown(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	fsl_amix_update(rtd);
}

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
	case SND_SOC_DAIFMT_NB_NF:
		/* Output data will be written on positive edge of the clock */
		ctr |= FSL_AMIX_CTR_OUTCKPOL(0);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		/* Output data will be written on negative edge of the clock */
		ctr |= FSL_AMIX_CTR_OUTCKPOL(1);
		break;
	default:
		return -EINVAL;
	}

	mask |= FSL_AMIX_CTR_OUTCKPOL_MASK;

	return regmap_update_bits(priv->regmap, FSL_AMIX_CTR, mask, ctr);
}

static struct snd_soc_dai_ops fsl_amix_dai_ops = {
	.shutdown     = fsl_amix_dai_shutdown,
	.mute_stream  = fsl_amix_dai_mute_stream,
	.set_fmt      = fsl_amix_dai_set_fmt,
};

static struct snd_soc_dai_driver fsl_amix_dai = {
	.playback = {
		.stream_name = "AMIX-Playback",
		.channels_min = 8,
		.channels_max = 8,
		.rate_min = 8000,
		.rate_max = 96000,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = FSL_AMIX_FORMATS,
	},
	.capture = {
		.stream_name = "AMIX-Capture",
		.channels_min = 8,
		.channels_max = 8,
		.rate_min = 8000,
		.rate_max = 96000,
		.rates = SNDRV_PCM_RATE_8000_96000,
		.formats = FSL_AMIX_FORMATS,
	},
	.ops = &fsl_amix_dai_ops,
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
					      &fsl_amix_dai, 1);
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

	return clk_prepare_enable(priv->ipg_clk);
}

static int fsl_amix_runtime_suspend(struct device *dev)
{
	struct fsl_amix *priv = dev_get_drvdata(dev);

	clk_disable_unprepare(priv->ipg_clk);

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
static int fsl_amix_suspend(struct device *dev)
{
	struct fsl_amix *priv = dev_get_drvdata(dev);

	regcache_cache_only(priv->regmap, true);
	regcache_mark_dirty(priv->regmap);

	return 0;
}

static int fsl_amix_resume(struct device *dev)
{
	struct fsl_amix *priv = dev_get_drvdata(dev);

	regcache_cache_only(priv->regmap, false);
	regcache_sync(priv->regmap);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops fsl_amix_pm = {
	SET_RUNTIME_PM_OPS(fsl_amix_runtime_suspend, fsl_amix_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(fsl_amix_suspend, fsl_amix_resume)
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
