#include <linux/module.h>
#include <linux/platform_device.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include "../codecs/ssm4329.h"

static const struct snd_soc_dapm_widget rpi_ssm4329_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker 1", NULL),
	SND_SOC_DAPM_SPK("Speaker 2", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route rpi_ssm4329_routes[] = {
	{ "Speaker 1", NULL, "OUT" },
	{ "Speaker 2", NULL, "Secondary OUT" },
	{ "AIN", NULL, "Mic" },
};

static int rpi_ssm4329_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_codec_set_pll(codec_dai->codec, SSM4329_PLL,
			SSM4329_PLL_SRC_MCLKIN, 12288000, 2048 * 48000);
	if (ret)
		return ret;

	return 0;
}

static const struct snd_soc_pcm_stream amp_params = {
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 192000,
	.rate_max = 192000,
	.channels_min = 1,
	.channels_max = 1,
};

static struct snd_soc_codec_conf codec_conf[] = {
	{
		.dev_name = "tdmc0.1",
		.name_prefix = "Secondary",
	},
};

static struct snd_soc_dai_link rpi_ssm4329_dais[] = {
	{
		.name		= "SSM4329",
		.stream_name	= "SSM4329 HiFi",
		.cpu_dai_name	= "bcm2708-i2s.0",
		.codec_dai_name	= "ssm4329-sp1",
		.platform_name	= "bcm2708-i2s.0",
		.codec_name	= "ssm4329.1-0034",
		.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
					SND_SOC_DAIFMT_CBS_CFS,
		.init		= rpi_ssm4329_init,
	}, {
		.name = "Secondary Amp",
		.stream_name = "Secondary Amp",
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM,
		.codec_name	= "ssm4329.1-0034",
		.codec_dai_name = "ssm4329-sp2",
		.cpu_name = "tdmc0.1",
		.cpu_dai_name = "ssm4567-hifi",
		.params = &amp_params,
	},
};

static struct snd_soc_card rpi_ssm4329_card = {
	.name = "RPI SSM4329",
	.dai_link = rpi_ssm4329_dais,
	.num_links = ARRAY_SIZE(rpi_ssm4329_dais),
	.dapm_widgets = rpi_ssm4329_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rpi_ssm4329_widgets),
	.dapm_routes = rpi_ssm4329_routes,
	.num_dapm_routes = ARRAY_SIZE(rpi_ssm4329_routes),
	.fully_routed = true,
	.codec_conf = codec_conf,
	.num_configs = 1,
};

static int rpi_ssm4329_probe(struct platform_device *pdev)
{
	rpi_ssm4329_card.dev = &pdev->dev;
	return snd_soc_register_card(&rpi_ssm4329_card);
}

static struct platform_driver rpi_ssm4329_driver = {
        .driver = {
                .name   = "snd-rpi-ssm4329",
                .owner  = THIS_MODULE,
        },
        .probe          = rpi_ssm4329_probe,
};
module_platform_driver(rpi_ssm4329_driver);

MODULE_LICENSE("GPL v2");
