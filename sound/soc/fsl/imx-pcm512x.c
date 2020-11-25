/* i.MX pcm512x audio support
 *
 * Copyright 2020 NXP
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/simple_card.h>
#include <sound/soc-dapm.h>

#include "fsl_sai.h"
#include "../codecs/pcm512x.h"

#define DAC_CLK_EXT_44K 22579200UL
#define DAC_CLK_EXT_48K 24576000UL

struct imx_pcm512x_data {
	struct simple_util_priv *priv;
	struct gpio_desc *mute_gpio;
	bool dac_sclk;
	bool dac_pluspro;
	bool dac_led_status;
	bool dac_gain_limit;
	bool dac_gpio_unmute;
	bool dac_auto_mute;
	bool one2one_ratio;
	bool tdm_mode;
};

enum ext_osc {
	DAC_CLK_INT,
	DAC_CLK_EXT_44EN,
	DAC_CLK_EXT_48EN,
};

static const struct imx_pcm512x_fs_map {
	unsigned int rmin;
	unsigned int rmax;
	unsigned int wmin;
	unsigned int wmax;
} fs_map[] = {
	{ .rmin = 8000,   .rmax = 192000, .wmin = 128, .wmax = 3072, },
	{ .rmin = 384000, .rmax = 384000, .wmin = 64,  .wmax = 128,  },
};

static const u32 pcm512x_rates[] = {
	8000, 11025, 16000, 22050, 32000,
	44100, 48000, 64000, 88200, 96000,
	176400, 192000, 384000,
};

static int imx_pcm512x_select_ext_clk(struct snd_soc_component *comp,
				      int ext_osc)
{
	switch(ext_osc) {
	case DAC_CLK_INT:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x00);
		break;
	case DAC_CLK_EXT_44EN:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x20);
		break;
	case DAC_CLK_EXT_48EN:
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x24, 0x04);
		break;
	}

	return 0;
}

static bool imx_pcm512x_is_sclk(struct snd_soc_component *comp)
{
	unsigned int sclk;

	sclk = snd_soc_component_read(comp, PCM512x_RATE_DET_4);

	return (!(sclk & 0x40));
}

static bool imx_pcm512x_is_sclk_sleep(struct snd_soc_component *comp)
{
	msleep(2);
	return imx_pcm512x_is_sclk(comp);
}

static int imx_pcm512x_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	int ret;

	if (data->dac_gain_limit) {
		ret = snd_soc_limit_volume(card, "Digital Playback Volume", 207);
		if (ret)
			dev_warn(card->dev, "failed to set volume limit\n");
	}

	return 0;
}

static int imx_pcm512x_set_bias_level(struct snd_soc_card *card,
	struct snd_soc_dapm_context *dapm, enum snd_soc_bias_level level)
{
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *codec_dai;

	rtd = snd_soc_get_pcm_runtime(card, card->dai_link);
	codec_dai = snd_soc_rtd_to_codec(rtd, 0);

	if (dapm->dev != codec_dai->dev)
		return 0;

	switch (level) {
	case SND_SOC_BIAS_PREPARE:
		if (dapm->bias_level != SND_SOC_BIAS_STANDBY)
			break;
		/* unmute amp */
		gpiod_set_value_cansleep(data->mute_gpio, 1);
		break;
	case SND_SOC_BIAS_STANDBY:
		if (dapm->bias_level != SND_SOC_BIAS_PREPARE)
			break;
		/* mute amp */
		gpiod_set_value_cansleep(data->mute_gpio, 0);
		break;
	default:
		break;
	}

	return 0;
}

static unsigned long pcm512x_get_mclk_rate(struct snd_pcm_substream *substream,
					  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int channels = params_channels(params);
	unsigned int width = params_width(params);
	unsigned int rate = params_rate(params);
	unsigned int ratio = channels * width;
	int i;

	for (i = 0; i < ARRAY_SIZE(fs_map); i++) {
		if (rate >= fs_map[i].rmin && rate <= fs_map[i].rmax) {
			ratio = max(ratio, fs_map[i].wmin);
			ratio = min(ratio, fs_map[i].wmax);
			/* Adjust SAI bclk:mclk ratio */
			ratio *= data->one2one_ratio ? 1 : 2;
			return rate * ratio;
		}
	}

	/* Let DAI manage clk frequency by default */
	return 0;
}

static int imx_pcm512x_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *comp = codec_dai->component;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	unsigned int width = params_width(params);
	unsigned long mclk_freq;
	int ret, i;

	/* set MCLK freq */
	if (data->dac_pluspro && data->dac_sclk) {
		if (do_div(rate, 8000)) {
			mclk_freq = DAC_CLK_EXT_44K;
			imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_44EN);
			ret = snd_soc_dai_set_sysclk(codec_dai,
				PCM512x_SYSCLK_MCLK1, mclk_freq, SND_SOC_CLOCK_IN);
		} else {
			mclk_freq = DAC_CLK_EXT_48K;
			imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_48EN);
			ret = snd_soc_dai_set_sysclk(codec_dai,
				PCM512x_SYSCLK_MCLK2, mclk_freq, SND_SOC_CLOCK_IN);
		}
		if (ret < 0)
			dev_err(card->dev, "failed to set cpu dai mclk rate (%lu): %d\n",
				mclk_freq, ret);
	} else {
		mclk_freq = pcm512x_get_mclk_rate(substream, params);
		ret = snd_soc_dai_set_sysclk(cpu_dai, FSL_SAI_CLK_MAST1,
					     mclk_freq, SND_SOC_CLOCK_OUT);
		if (ret < 0)
			dev_err(card->dev, "failed to set cpu dai mclk1 rate (%lu): %d\n",
				mclk_freq, ret);
	}

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_bclk_ratio(codec_dai, (channels * width));
		if (ret) {
			dev_err(card->dev, "failed to set codec dai bclk ratio\n");
			return ret;
		}
	}

	dev_dbg(card->dev, "mclk_freq: %lu, bclk ratio: %u\n",
		mclk_freq, (channels * width));

	return 0;
}

static int imx_pcm512x_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	static struct snd_pcm_hw_constraint_list constraint_rates;
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *comp = codec_dai->component;
	bool ext_44sclk, ext_48sclk, ext_nosclk;
	int ret;

	constraint_rates.list = pcm512x_rates;
	constraint_rates.count = ARRAY_SIZE(pcm512x_rates);

	ret = snd_pcm_hw_constraint_list(runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &constraint_rates);
	if (ret)
		return ret;

	if (data->dac_led_status) {
		snd_soc_component_update_bits(comp, PCM512x_GPIO_EN, 0x08, 0x08);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_4, 0x0f, 0x02);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);
	}

	if (data->dac_sclk) {
		snd_soc_component_update_bits(comp, PCM512x_GPIO_EN, 0x24, 0x24);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_3, 0x0f, 0x02);
		snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_6, 0x0f, 0x02);

		imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_44EN);
		ext_44sclk = imx_pcm512x_is_sclk_sleep(comp);

		imx_pcm512x_select_ext_clk(comp, DAC_CLK_EXT_48EN);
		ext_48sclk = imx_pcm512x_is_sclk_sleep(comp);

		imx_pcm512x_select_ext_clk(comp, DAC_CLK_INT);
		ext_nosclk = imx_pcm512x_is_sclk_sleep(comp);

		data->dac_pluspro = (ext_44sclk && ext_48sclk && !ext_nosclk);
	}

	return 0;
}

static void imx_pcm512x_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct imx_pcm512x_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *comp = codec_dai->component;

	if (data->dac_led_status)
		snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x08, 0x00);

	if (data->dac_sclk && data->dac_pluspro)
		imx_pcm512x_select_ext_clk(comp, DAC_CLK_INT);
}

static struct snd_soc_ops imx_pcm512x_ops = {
	.hw_params = imx_pcm512x_hw_params,
	.startup = imx_pcm512x_startup,
	.shutdown = imx_pcm512x_shutdown,
};

static int imx_asoc_card_parse_dt(struct snd_soc_card *card,
				  struct simple_util_priv *priv)
{
	struct device_node *np, *cpu_np, *codec_np = NULL;
	struct snd_soc_dai_link_component *dlc;
	struct simple_util_dai *simple_dai;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *link;
	struct of_phandle_args args;
	int ret, num_links;

	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret) {
		dev_err(dev, "failed to find card model name\n");
		return ret;
	}

	if (of_property_read_bool(dev->of_node, "audio-routing")) {
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
		if (ret) {
			dev_err(dev, "failed to parse audio-routing\n");
			return ret;
		}
	}

	if (of_property_read_bool(dev->of_node, "audio-widgets")) {
		ret = snd_soc_of_parse_audio_simple_widgets(card,
							    "audio-widgets");
		if (ret) {
			dev_err(dev, "failed to parse audio-widgets\n");
			return ret;
		}
	}

	if (of_property_read_bool(dev->of_node, "aux-devs")) {
		ret = snd_soc_of_parse_aux_devs(card, "aux-devs");
		if (ret) {
			dev_err(dev, "failed to parse aux devs\n");
			return ret;
		}
	}

	num_links = of_get_child_count(dev->of_node);

	card->dai_link = devm_kcalloc(dev, num_links, sizeof(*link), GFP_KERNEL);
	if (!card->dai_link) {
		dev_err(dev, "failed to allocate memory for dai_link\n");
		return -ENOMEM;
	}

	simple_dai = devm_kcalloc(dev, num_links, sizeof(*simple_dai), GFP_KERNEL);
	if (!simple_dai) {
		dev_err(dev, "failed to allocate memory for simple_dai\n");
		return -ENOMEM;
	}

	link = card->dai_link;
	card->num_links = num_links;
	/* pupulate dai links */
	for_each_child_of_node(dev->of_node, np) {
		dlc = devm_kzalloc(dev, 2 * sizeof(*dlc), GFP_KERNEL);
		if (!dlc) {
			dev_err(dev, "failed to allocate memory for dlc\n");
			ret = -ENOMEM;
			goto err_fail;
		}

		link->cpus = &dlc[0];
		link->platforms = &dlc[1];
		link->num_cpus = 1;
		link->num_platforms = 1;

		if (of_property_read_bool(np, "link-name")) {
			ret = of_property_read_string(np, "link-name", &link->name);
			if (ret) {
				dev_err(dev, "failed to get dai_link name\n");
				goto fail;
			}
		}

		cpu_np = of_get_child_by_name(np, "cpu");
		if (!cpu_np) {
			dev_err(dev, "failed to get cpu phandle missing or invalid");
			ret = -EINVAL;
			goto fail;
		}

		ret = of_parse_phandle_with_args(cpu_np, "sound-dai",
						 "#sound-dai-cells", 0, &args);
		if (ret) {
			dev_err(dev, "failed to get cpu sound-dais\n");
			goto fail;
		}

		ret = snd_soc_of_get_dai_name(cpu_np, &link->cpus->dai_name, 0);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to get cpu dai name\n");
			goto fail;
		}

		link->cpus->of_node = args.np;
		link->id = args.args[0];
		link->platforms->of_node = link->cpus->of_node;

		codec_np = of_get_child_by_name(np, "codec");
		if (codec_np) {
			ret = snd_soc_of_get_dai_link_codecs(dev, codec_np, link);
			if (ret) {
				if (ret != -EPROBE_DEFER)
					dev_err(dev, "failed to get codec dais\n");
				goto fail;
			}
		} else {
			dlc = devm_kzalloc(dev, sizeof(*dlc), GFP_KERNEL);
			if (!dlc) {
				dev_err(dev, "failed to allocate memory for dlc\n");
				ret = -ENOMEM;
				goto err_fail;
			}

			link->codecs = dlc;
			link->num_codecs = 1;

			link->codecs->dai_name = "snd-soc-dummy-dai";
			link->codecs->name = "snd-soc-dummy";
		}

		ret = simple_util_parse_daifmt(dev, np, codec_np, NULL,
					       &link->dai_fmt);
		if (ret) {
			dev_warn(dev, "failed to parse dai format\n");
			link->dai_fmt = SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS |
				SND_SOC_DAIFMT_I2S;
		}

		ret = simple_util_parse_tdm(np, simple_dai);
		if (ret) {
			dev_err(dev, "failed to parse dai tdm\n");
		}

		link->stream_name = link->name;
		link->ignore_pmdown_time = 1;
		simple_dai++;
		link++;

		of_node_put(cpu_np);
		of_node_put(codec_np);
	}

	return 0;

fail:
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);
err_fail:
	if (np)
		of_node_put(np);

	return ret;
}

static int imx_pcm512x_parse_dt(struct imx_pcm512x_data *data)
{
	struct snd_soc_card *card = &data->priv->snd_card;
	struct device *dev = card->dev;
	struct device_node *np = dev->of_node;
	int ret;

	/* Multiple dais */
	ret = imx_asoc_card_parse_dt(card, data->priv);
	if (ret)
		return ret;

	data->dac_gain_limit =
		of_property_read_bool(np, "dac,24db_digital_gain");
	data->dac_auto_mute =
		of_property_read_bool(np, "dac,auto_mute_amp");
	data->dac_gpio_unmute =
		of_property_read_bool(np, "dac,unmute_amp");
	data->dac_led_status =
		of_property_read_bool(np, "dac,led_status");
	data->dac_sclk =
		of_property_read_bool(np, "dac,sclk");

	return 0;
}

static int imx_pcm512x_probe(struct platform_device *pdev)
{
	struct simple_util_priv *priv;
	struct imx_pcm512x_data *data;
	struct snd_soc_card *card;
	int ret, i;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!data->priv)
		return -ENOMEM;

	card = &data->priv->snd_card;
	dev_set_drvdata(&pdev->dev, &data);
	snd_soc_card_set_drvdata(card, data);

	card->owner = THIS_MODULE;
	card->dev = &pdev->dev;

	ret = imx_pcm512x_parse_dt(data);
	if (ret) {
		dev_err(&pdev->dev, "failed to parse sound card dts\n");
		return ret;
	}

	for (i = 0; i < card->num_links; i++) {
		card->dai_link->ops = &imx_pcm512x_ops;
		card->dai_link->init = &imx_pcm512x_dai_init;
	}

	if (data->dac_auto_mute || data->dac_gpio_unmute) {
		data->mute_gpio = devm_gpiod_get_optional(&pdev->dev,
						"mute-amp", GPIOD_OUT_LOW);
		if (IS_ERR(data->mute_gpio)) {
			dev_err(&pdev->dev, "failed to get mute amp gpio\n");
			return PTR_ERR(data->mute_gpio);
		}
	}

	if (data->dac_auto_mute && data->dac_gpio_unmute)
		card->set_bias_level = imx_pcm512x_set_bias_level;

	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret) {
		dev_err(&pdev->dev, "failed to register snd card\n");
		return ret;
	}

	if (data->dac_gpio_unmute && data->dac_auto_mute)
		gpiod_set_value_cansleep(data->mute_gpio, 1);

	return 0;
}

static int imx_pcm512x_remove(struct platform_device *pdev)
{
	struct imx_pcm512x_data *data = platform_get_drvdata(pdev);

	if (data->mute_gpio)
		gpiod_set_value_cansleep(data->mute_gpio, 0);

	snd_soc_unregister_card(&data->priv->snd_card);

	return 0;
}

static const struct of_device_id imx_pcm512x_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-pcm512x", },
	{ },
};
MODULE_DEVICE_TABLE(of, imx_pcm512x_dt_ids);

static struct platform_driver imx_pcm512x_driver = {
	.driver = {
		.name = "imx-pcm512x",
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_pcm512x_dt_ids,
	},
	.probe = imx_pcm512x_probe,
	.remove = imx_pcm512x_remove,
};
module_platform_driver(imx_pcm512x_driver);

MODULE_DESCRIPTION("NXP i.MX pcm512x ASoC machine driver");
MODULE_AUTHOR("Adrian Alonso <adrian.alonso@nxp.com>");
MODULE_ALIAS("platform:imx-pcm512x");
MODULE_LICENSE("GPL");
