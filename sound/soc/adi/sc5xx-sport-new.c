// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define SPORT_B_OFFSET	0x80
#define SPORT_CTL(_half)	(0x00 + (SPORT_B_OFFSET * (_half)))
#define SPORT_DIV(_half)	(0x04 + (SPORT_B_OFFSET * (_half)))
#define SPORT_MCTL(_half)	(0x08 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS0(_half)	(0x0c + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS1(_half)	(0x10 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS2(_half)	(0x14 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CS3(_half)	(0x18 + (SPORT_B_OFFSET * (_half)))
#define SPORT_ERR(_half)	(0x20 + (SPORT_B_OFFSET * (_half)))
#define SPORT_MSTAT(_half)	(0x24 + (SPORT_B_OFFSET * (_half)))
#define SPORT_CTL2(_half)	(0x28 + (SPORT_B_OFFSET * (_half)))
#define SPORT_TXPRI(_half)	(0x40 + (SPORT_B_OFFSET * (_half)))
#define SPORT_RXPRI(_half)	(0x44 + (SPORT_B_OFFSET * (_half)))
#define SPORT_TXSEC(_half)	(0x48 + (SPORT_B_OFFSET * (_half)))
#define SPORT_RXSEC(_half)	(0x4c + (SPORT_B_OFFSET * (_half)))

#define SPORT_CTL_SPENPRI	GENMASK(0, 0)
#define SPORT_CTL_DTYPE		GENMASK(2, 1)
#define SPORT_CTL_LSBF		GENMASK(3, 3)
#define SPORT_CTL_SLEN		GENMASK(8, 4)
#define SPORT_CTL_PACK		GENMASK(9, 9)
#define SPORT_CTL_ICLK		GENMASK(10, 10)
#define SPORT_CTL_OPMODE	GENMASK(11, 11)
#define SPORT_CTL_CKRE		GENMASK(12, 12)
#define SPORT_CTL_FSR		GENMASK(13, 13)
#define SPORT_CTL_IFS		GENMASK(14, 14)
#define SPORT_CTL_DIFS		GENMASK(15, 15)
#define SPORT_CTL_LFS		GENMASK(16, 16)
#define SPORT_CTL_LAFS		GENMASK(17, 17)
#define SPORT_CTL_RJUST		GENMASK(18, 18)
#define SPORT_CTL_FSED		GENMASK(19, 19)
#define SPORT_CTL_TFIEN		GENMASK(20, 20)
#define SPORT_CTL_GCLKEN	GENMASK(21, 21)
#define SPORT_CTL_SPENSEC	GENMASK(24, 24)
#define SPORT_CTL_SPTRAN	GENMASK(25, 25)
#define SPORT_CTL_DERRSEC	GENMASK(26, 26)
#define SPORT_CTL_DXSSEC	GENMASK(28, 27)
#define SPORT_CTL_DERRPRI	GENMASK(29, 29)
#define SPORT_CTL_DXSPRI	GENMASK(31, 30)

#define SPORT_DIV_CLKDIV	GENMASK(15, 0)
#define SPORT_DIV_FSDIV		GENMASK(31, 16)

#define SPORT_MCTL_MCE		GENMASK(0, 0)
#define SPORT_MCTL_MCPDE	GENMASK(2, 2)
#define SPORT_MCTL_MFD		GENMASK(7, 4)
#define SPORT_MCTL_WSIZE	GENMASK(14, 8)
#define SPORT_MCTL_WOFFSET	GENMASK(25, 16)

#define SPORT_CS0_VALUE		GENMASK(31, 0)
#define SPORT_CS1_VALUE		GENMASK(31, 0)
#define SPORT_CS2_VALUE		GENMASK(31, 0)
#define SPORT_CS3_VALUE		GENMASK(31, 0)

#define SPORT_ERR_DERRPMSK	GENMASK(0, 0)
#define SPORT_ERR_DERRSMSK	GENMASK(1, 1)
#define SPORT_ERR_FSERRMSK	GENMASK(2, 2)
#define SPORT_ERR_DERRPSTAT	GENMASK(4, 4)
#define SPORT_ERR_DERRSSTAT	GENMASK(5, 5)
#define SPORT_ERR_FSERRSTAT	GENMASK(6, 6)

#define SPORT_MSTAT_CURCHAN	GENMASK(9, 0)

#define SPORT_CTL2_FSMUXSEL	GENMASK(0, 0)
#define SPORT_CTL2_CKMUXSEL	GENMASK(1, 1)
#define SPORT_CTL2_RLRE		GENMASK(3, 3)

#define SPORT_TXPRI_VALUE	GENMASK(31, 0)
#define SPORT_RXPRI_VALUE	GENMASK(31, 0)
#define SPORT_TXSEC_VALUE	GENMASK(31, 0)
#define SPORT_RXSEC_VALUE	GENMASK(31, 0)

struct adsp_sport {
	void __iomem *regs;
	struct clk *clk;
};

static int adsp_sport_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct adsp_sport *sport = snd_soc_dai_get_drvdata(dai);
	bool is_i2s = false;
	unsigned int ctl;

	ctl = readl(sport->regs + SPORT_CTL(dai->id));
	ctl |= 0;

	/* Operating mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_MCE, &ctl, 0);
		is_i2s = true;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_MCE, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_MCE, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		FIELD_MODIFY(SPORT_CTL_OPMODE, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_LAFS, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_RJUST, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_MCE, &ctl, 1);
		/* TODO: Tune SYNC offset in MCTL */
		break;
	default:
		return -EINVAL;
	}

	/* Clock polarities */
	switch (fmt & SND_SOC_FAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 1);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		FIELD_MODIFY(SPORT_CTL_CKRE, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_LFS, &ctl, 1);
		break;
	default:
		return -EINVAL;
	}

	/* I2S frames start with falling edge, unlike the other modes */
	if (is_i2s)
		ctrl ^= SPORT_CTL_LFS;

	/* Always require frame sync */
	FIELD_MODIFY(SPORT_CTL_FSR, &ctl, 1);
	// TODO: DIFS?

	/* Clock consumer/provider roles */
	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		FIELD_MODIFY(SPORT_CTL_ICLK, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_IFS, &ctl, 1);
		break;
	case SND_SOC_DAIFMT_BC_FP:
		FIELD_MODIFY(SPORT_CTL_ICLK, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_IFS, &ctl, 1);
		break;
	case SND_SOC_DAIFMT_BP_FC:
		FIELD_MODIFY(SPORT_CTL_ICLK, &ctl, 1);
		FIELD_MODIFY(SPORT_CTL_IFS, &ctl, 0);
		break;
	case SND_SOC_DAIFMT_BC_FC:
		FIELD_MODIFY(SPORT_CTL_ICLK, &ctl, 0);
		FIELD_MODIFY(SPORT_CTL_IFS, &ctl, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int adsp_sport_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	struct adsp_sport *dev = snd_soc_dai_get_drvdata(dai);

	/* TODO */

	return 0;
}

static int adsp_sport_trigger(struct snd_pcm_substream *substream,
			    int cmd, struct snd_soc_dai *dai)
{
	struct adsp_sport *dev = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		/* TODO */
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		/* TODO */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct snd_soc_dai_ops adsp_sport_ops = {
	.set_fmt	= adsp_sport_set_fmt,
	.hw_params	= adsp_sport_hw_params,
	.trigger	= adsp_sport_trigger,
};

#define ADSP_SPORT_RATES	(SNDRV_PCM_RATE_8000_192000)
#define ADSP_SPORT_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver adsp_sport_dais[] = {
	[0] = {
		.name = "HSPORT-A",
		.playback = {
			.channels_min = 1,
			.channels_max = 128,
			.rates = ADSP_SPORT_RATES,
			.formats = ADSP_SPORT_FORMATS,
		},
		.ops = &adsp_sport_tx_ops,
	},
	[1] = {
		.name = "HSPORT-B",
		.capture = {
			.channels_min = 1,
			.channels_max = 128,
			.rates = ADSP_SPORT_RATES,
			.formats = ADSP_SPORT_FORMATS,
		},
		.ops = &adsp_sport_rx_ops,
	},
};

static const struct snd_soc_component_driver adsp_sport_component = {
	.name		= "adsp-sport",
};

static int adsp_sport_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adsp_sport *sport;
	int ret;

	sport = devm_kzalloc(dev, sizeof(*adsp_sport), GFP_KERNEL);
	if (!adsp_sport)
		return -ENOMEM;

	sport->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sport->regs))
		return PTR_ERR(sport->regs);

	/* TODO: SCLK0? Do we really have to enable it now? */
	sport->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(adsp_sport->clk))
		return PTR_ERR(sport->clk);

	platform_set_drvdata(pdev, sport);

	ret = devm_snd_dmaengine_pcm_register(dev, NULL, 0);
	if (ret)
		return ret;

	ret = devm_snd_soc_register_component(dev, &adsp_sport_component,
					      adsp_sport_dais,
					      ARRAY_SIZE(adsp_sport_dais));
	if (ret)
		return ret;

	return 0;
}

static const struct of_device_id adsp_sport_of_match[] = {
	{ .compatible = "adi,adsp-sc598-sport" },
	{},
};
MODULE_DEVICE_TABLE(of, adsp_sport_of_match);

static struct platform_driver adsp_sport_driver = {
	.probe = adsp_sport_probe,
	.driver = {
		.name = "adsp-sport",
		.of_match_table = adsp_sport_of_match,
	},
};
module_platform_driver(adsp_sport_driver);

MODULE_AUTHOR("Alvin Šipraga <alvin.sipraga@analog.com>");
MODULE_DESCRIPTION("ADI ADSP SPORT ASoC driver");
MODULE_LICENSE("GPL");
