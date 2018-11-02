/*
 * Copyright 2017-2018 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/control.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <sound/hdmi-codec.h>
#include "../../../drivers/gpu/drm/imx/hdp/imx-hdp.h"
#include "fsl_sai.h"

#define SUPPORT_RATE_NUM 10
#define SUPPORT_CHANNEL_NUM 10

struct imx_cdnhdmi_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	int protocol;
	u32 support_rates[SUPPORT_RATE_NUM];
	u32 support_rates_num;
	u32 support_channels[SUPPORT_CHANNEL_NUM];
	u32 support_channels_num;
	u32 edid_rates[SUPPORT_RATE_NUM];
	u32 edid_rates_count;
	u32 edid_channels[SUPPORT_CHANNEL_NUM];
	u32 edid_channels_count;
	uint8_t eld[MAX_ELD_BYTES];
};

static int imx_cdnhdmi_startup(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	static struct snd_pcm_hw_constraint_list constraint_rates;
	static struct snd_pcm_hw_constraint_list constraint_channels;
	int ret;

	constraint_rates.list = data->support_rates;
	constraint_rates.count = data->support_rates_num;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
						&constraint_rates);
	if (ret)
		return ret;

	constraint_channels.list = data->support_channels;
	constraint_channels.count = data->support_channels_num;

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_CHANNELS,
						&constraint_channels);
	if (ret)
		return ret;

	return 0;
}

static int imx_cdnhdmi_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = rtd->card;
	struct device *dev = card->dev;
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	int ret;

	/* set cpu DAI configuration */
	if (tx)
		ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS);
	else
		ret = snd_soc_dai_set_fmt(cpu_dai,
			SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBM_CFM);

	if (ret) {
		dev_err(dev, "failed to set cpu dai fmt: %d\n", ret);
		return ret;
	}


	if (of_device_is_compatible(dev->of_node,
				    "fsl,imx8mq-evk-cdnhdmi"))
		ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1,
				256 * params_rate(params),
				SND_SOC_CLOCK_OUT);
	else
		ret = snd_soc_dai_set_sysclk(cpu_dai, 0,
				0,
				tx ? SND_SOC_CLOCK_OUT : SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "failed to set cpu sysclk: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2, 32);
	if (ret) {
		dev_err(dev, "failed to set cpu dai tdm slot: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct snd_soc_ops imx_cdnhdmi_ops = {
	.startup = imx_cdnhdmi_startup,
	.hw_params = imx_cdnhdmi_hw_params,
};

static const unsigned int eld_rates[] = {
	32000,
	44100,
	48000,
	88200,
	96000,
	176400,
	192000,
};

static unsigned int sad_max_channels(const u8 *sad)
{
	return 1 + (sad[0] & 7);
}

static int get_edid_info(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = list_first_entry(
		&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct hdmi_codec_pdata *hcd = codec->dev->platform_data;
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int i, j, ret;
	const u8 *sad;
	unsigned int channel_max = 0;
	unsigned int rate_mask = 0;
	unsigned int rate_mask_eld = 0;

	ret = hcd->ops->get_eld(codec->dev->parent, hcd->data,
					    data->eld, sizeof(data->eld));
	sad = drm_eld_sad(data->eld);
	if (sad) {
		for (j = 0; j < data->support_rates_num; j++) {
			for (i = 0; i < ARRAY_SIZE(eld_rates); i++)
				if (eld_rates[i] == data->support_rates[j])
					rate_mask |= BIT(i);
		}

		for (i = drm_eld_sad_count(data->eld); i > 0; i--, sad += 3) {
			if (rate_mask & sad[1])
				channel_max = max(channel_max, sad_max_channels(sad));

			if (sad_max_channels(sad) >= 2)
				rate_mask_eld |= sad[1];
		}
	}

	rate_mask = rate_mask & rate_mask_eld;

	data->edid_rates_count = 0;
	data->edid_channels_count = 0;

	for (i = 0; i < ARRAY_SIZE(eld_rates); i++) {
		if (rate_mask & BIT(i)) {
			data->edid_rates[data->edid_rates_count] = eld_rates[i];
			data->edid_rates_count++;
		}
	}

	for (i = 0; i < data->support_channels_num; i++) {
		if (data->support_channels[i] <= channel_max) {
			data->edid_channels[data->edid_channels_count]
					= data->support_channels[i];
			data->edid_channels_count++;
		}
	}

	return 0;
}

static int imx_cdnhdmi_channels_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);

	get_edid_info(card);

	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = data->edid_channels_count;

	return 0;
}

static int imx_cdnhdmi_channels_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int i;

	get_edid_info(card);

	for (i = 0 ; i < data->edid_channels_count ; i++)
		uvalue->value.integer.value[i] = data->edid_channels[i];

	return 0;
}

static int imx_cdnhdmi_rates_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);

	get_edid_info(card);

	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = data->edid_rates_count;

	return 0;
}

static int imx_cdnhdmi_rates_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int i;

	get_edid_info(card);

	for (i = 0 ; i < data->edid_rates_count; i++)
		uvalue->value.integer.value[i] = data->edid_rates[i];

	return 0;
}

static int imx_cdnhdmi_formats_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 3;

	return 0;
}

static int imx_cdnhdmi_formats_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	uvalue->value.integer.value[0] = 16;
	uvalue->value.integer.value[1] = 24;
	uvalue->value.integer.value[2] = 32;

	return 0;
}

static int get_edid_rx_info(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd = list_first_entry(
		&card->rtd_list, struct snd_soc_pcm_runtime, list);
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct hdmi_codec_pdata *hcd = codec->dev->platform_data;
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	ret = hcd->ops->get_eld(codec->dev->parent, hcd->data,
					    data->eld, sizeof(data->eld));

	if (ret)
		return -EINVAL;

	data->edid_rates[0] = data->eld[0] +
		(data->eld[1] << 8) +
		(data->eld[2] << 16) +
		(data->eld[3] << 24);

	data->edid_channels[0] = data->eld[4] +
		(data->eld[5] << 8) +
		(data->eld[6] << 16) +
		(data->eld[7] << 24);


	return 0;
}

static int imx_cdnhdmi_rx_channels_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 2;
	uinfo->value.integer.max = 8;

	return 0;
}

static int imx_cdnhdmi_rx_channels_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	ret = get_edid_rx_info(card);
	if (ret)
		return ret;
	uvalue->value.integer.value[0] = data->edid_channels[0];

	return 0;
}

static int imx_cdnhdmi_rx_rates_info(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_info *uinfo)
{
	uinfo->type  = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 16000;
	uinfo->value.integer.max = 192000;

	return 0;
}

static int imx_cdnhdmi_rx_rates_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *uvalue)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct imx_cdnhdmi_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	ret = get_edid_rx_info(card);
	if (ret)
		return ret;

	uvalue->value.integer.value[0] = data->edid_rates[0];

	return 0;
}

static struct snd_kcontrol_new imx_cdnhdmi_ctrls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HDMI Support Channels",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = imx_cdnhdmi_channels_info,
		.get = imx_cdnhdmi_channels_get,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HDMI Support Rates",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = imx_cdnhdmi_rates_info,
		.get = imx_cdnhdmi_rates_get,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HDMI Support Formats",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = imx_cdnhdmi_formats_info,
		.get = imx_cdnhdmi_formats_get,
	},
};

static struct snd_kcontrol_new imx_cdnhdmi_rx_ctrls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HDMI Rx Channels",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = imx_cdnhdmi_rx_channels_info,
		.get = imx_cdnhdmi_rx_channels_get,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = "HDMI Rx Rates",
		.access = SNDRV_CTL_ELEM_ACCESS_READ |
			SNDRV_CTL_ELEM_ACCESS_VOLATILE,
		.info = imx_cdnhdmi_rx_rates_info,
		.get = imx_cdnhdmi_rx_rates_get,
	},
};

static int imx_cdnhdmi_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *cdnhdmi_np = NULL;
	struct platform_device *cpu_pdev;
	struct imx_cdnhdmi_data *data;
	int ret;
	int i;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "audio-cpu", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "cpu dai phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SAI platform device\n");
		ret = -EINVAL;
		goto fail;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < SUPPORT_RATE_NUM; i++) {
		ret = of_property_read_u32_index(pdev->dev.of_node,
						"constraint-rate",
						i, &data->support_rates[i]);
		if (!ret)
			data->support_rates_num = i + 1;
		else
			break;
	}

	if (data->support_rates_num == 0) {
		data->support_rates[0] = 48000;
		data->support_rates[1] = 96000;
		data->support_rates[2] = 32000;
		data->support_rates[3] = 192000;
		data->support_rates_num = 4;
	}

	data->support_channels[0] = 2;
	data->support_channels[1] = 4;
	data->support_channels[2] = 8;
	data->support_channels_num = 3;

	of_property_read_u32(pdev->dev.of_node, "protocol",
					&data->protocol);

	data->dai.name = "imx8 hdmi";
	data->dai.stream_name = "imx8 hdmi";
	data->dai.cpu_dai_name = dev_name(&cpu_pdev->dev);
	data->dai.platform_of_node = cpu_np;
	data->dai.ops = &imx_cdnhdmi_ops;
	data->dai.playback_only = true;
	data->dai.capture_only = false;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;

	if (of_property_read_bool(pdev->dev.of_node, "hdmi-out")) {
		data->dai.playback_only = true;
		data->dai.capture_only = false;
		data->dai.dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBS_CFS;
		data->dai.codec_dai_name = "i2s-hifi";
		data->dai.codec_name = "hdmi-audio-codec.1";
		data->card.controls	= imx_cdnhdmi_ctrls;
		data->card.num_controls	= ARRAY_SIZE(imx_cdnhdmi_ctrls);
	}

	if (of_property_read_bool(pdev->dev.of_node, "hdmi-in")) {
		data->dai.playback_only = false;
		data->dai.capture_only = true;
		data->dai.dai_fmt = SND_SOC_DAIFMT_I2S |
			    SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;
		data->dai.codec_dai_name = "i2s-hifi";
		data->dai.codec_name = "hdmi-audio-codec.2";
		data->card.controls	= imx_cdnhdmi_rx_ctrls;
		data->card.num_controls	= ARRAY_SIZE(imx_cdnhdmi_rx_ctrls);
	}

	if ((data->dai.playback_only && data->dai.capture_only)
		|| (!data->dai.playback_only && !data->dai.capture_only)) {
		dev_err(&pdev->dev, "Wrongly enable HDMI DAI link\n");
		goto fail;
	}

	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.dai_link = &data->dai;


	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);
	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (cdnhdmi_np)
		of_node_put(cdnhdmi_np);
	return ret;
}

static const struct of_device_id imx_cdnhdmi_dt_ids[] = {
	{ .compatible = "fsl,imx8mq-evk-cdnhdmi", },
	{ .compatible = "fsl,imx-audio-cdnhdmi", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_cdnhdmi_dt_ids);

static struct platform_driver imx_cdnhdmi_driver = {
	.driver = {
		.name = "imx-cdnhdmi",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_cdnhdmi_dt_ids,
	},
	.probe = imx_cdnhdmi_probe,
};
module_platform_driver(imx_cdnhdmi_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("Freescale i.MX hdmi audio ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-cdnhdmi");
