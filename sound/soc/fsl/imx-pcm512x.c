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
#include "../codecs/pcm186x.h"

#define DAC_CLK_EXT_44K 22579200UL
#define DAC_CLK_EXT_48K 24576000UL

struct imx_pcm512x_data {
	struct simple_util_priv *priv;
	struct gpio_desc *mute_gpio;
	bool dac_sclk;
	bool adc_pluspro;
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

static const unsigned int pcm186x_adc_input_channel_sel_value[] = {
	0x00, 0x01, 0x02, 0x03, 0x10
};

static const char * const pcm186x_adcl_input_channel_sel_text[] = {
	"No Select",
	"VINL1[SE]", /* Default for ADCL */
	"VINL2[SE]",
	"VINL2[SE] + VINL1[SE]",
	"{VIN1P, VIN1M}[DIFF]"
};

static const char * const pcm186x_adcr_input_channel_sel_text[] = {
	"No Select",
	"VINR1[SE]", /* Default for ADCR */
	"VINR2[SE]",
	"VINR2[SE] + VINR1[SE]",
	"{VIN2P, VIN2M}[DIFF]"
};

static const struct soc_enum pcm186x_adc_input_channel_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_ADC1_INPUT_SEL_L, 0,
			      PCM186X_ADC_INPUT_SEL_MASK,
			      ARRAY_SIZE(pcm186x_adcl_input_channel_sel_text),
			      pcm186x_adcl_input_channel_sel_text,
			      pcm186x_adc_input_channel_sel_value),
	SOC_VALUE_ENUM_SINGLE(PCM186X_ADC1_INPUT_SEL_R, 0,
			      PCM186X_ADC_INPUT_SEL_MASK,
			      ARRAY_SIZE(pcm186x_adcr_input_channel_sel_text),
			      pcm186x_adcr_input_channel_sel_text,
			      pcm186x_adc_input_channel_sel_value),
};

static const unsigned int pcm186x_mic_bias_sel_value[] = {
	0x00, 0x01, 0x11
};

static const char * const pcm186x_mic_bias_sel_text[] = {
	"Mic Bias off",
	"Mic Bias on",
	"Mic Bias with Bypass Resistor"
};

static const struct soc_enum pcm186x_mic_bias_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_MIC_BIAS_CTRL, 0,
			      GENMASK(4, 0),
			      ARRAY_SIZE(pcm186x_mic_bias_sel_text),
			      pcm186x_mic_bias_sel_text,
			      pcm186x_mic_bias_sel_value),
};

static const unsigned int pcm186x_gain_sel_value[] = {
	0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef,
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
	0x50
};

static const char * const pcm186x_gain_sel_text[] = {
	"-12.0dB", "-11.5dB", "-11.0dB", "-10.5dB", "-10.0dB", "-9.5dB",
	"-9.0dB", "-8.5dB", "-8.0dB", "-7.5dB", "-7.0dB", "-6.5dB",
	"-6.0dB", "-5.5dB", "-5.0dB", "-4.5dB", "-4.0dB", "-3.5dB",
	"-3.0dB", "-2.5dB", "-2.0dB", "-1.5dB", "-1.0dB", "-0.5dB",
	"0.0dB", "0.5dB", "1.0dB", "1.5dB", "2.0dB", "2.5dB",
	"3.0dB", "3.5dB", "4.0dB", "4.5dB", "5.0dB", "5.5dB",
	"6.0dB", "6.5dB", "7.0dB", "7.5dB", "8.0dB", "8.5dB",
	"9.0dB", "9.5dB", "10.0dB", "10.5dB", "11.0dB", "11.5dB",
	"12.0dB", "12.5dB", "13.0dB", "13.5dB", "14.0dB", "14.5dB",
	"15.0dB", "15.5dB", "16.0dB", "16.5dB", "17.0dB", "17.5dB",
	"18.0dB", "18.5dB", "19.0dB", "19.5dB", "20.0dB", "20.5dB",
	"21.0dB", "21.5dB", "22.0dB", "22.5dB", "23.0dB", "23.5dB",
	"24.0dB", "24.5dB", "25.0dB", "25.5dB", "26.0dB", "26.5dB",
	"27.0dB", "27.5dB", "28.0dB", "28.5dB", "29.0dB", "29.5dB",
	"30.0dB", "30.5dB", "31.0dB", "31.5dB", "32.0dB", "32.5dB",
	"33.0dB", "33.5dB", "34.0dB", "34.5dB", "35.0dB", "35.5dB",
	"36.0dB", "36.5dB", "37.0dB", "37.5dB", "38.0dB", "38.5dB",
	"39.0dB", "39.5dB", "40.0dB"
};

static const struct soc_enum pcm186x_gain_sel[] = {
	SOC_VALUE_ENUM_SINGLE(PCM186X_PGA_VAL_CH1_L, 0, 0xff,
			      ARRAY_SIZE(pcm186x_gain_sel_text),
			      pcm186x_gain_sel_text,
			      pcm186x_gain_sel_value),
	SOC_VALUE_ENUM_SINGLE(PCM186X_PGA_VAL_CH1_R, 0, 0xff,
			      ARRAY_SIZE(pcm186x_gain_sel_text),
			      pcm186x_gain_sel_text,
			      pcm186x_gain_sel_value),
};

static const struct snd_kcontrol_new pcm1863_snd_controls_card[] = {
	SOC_ENUM("ADC Left Input", pcm186x_adc_input_channel_sel[0]),
	SOC_ENUM("ADC Right Input", pcm186x_adc_input_channel_sel[1]),
	SOC_ENUM("ADC Mic Bias", pcm186x_mic_bias_sel),
	SOC_ENUM("PGA Gain Left", pcm186x_gain_sel[0]),
	SOC_ENUM("PGA Gain Right", pcm186x_gain_sel[1]),
};

static int pcm1863_add_controls(struct snd_soc_component *component)
{
	snd_soc_add_component_controls(component, pcm1863_snd_controls_card,
				       ARRAY_SIZE(pcm1863_snd_controls_card));
	return 0;
}

static int imx_pcm186x_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *adc_dai = snd_soc_rtd_to_codec(rtd, 1);
	struct snd_soc_component *adc = adc_dai->component;
	int ret;

	ret = pcm1863_add_controls(adc);
	if (ret) {
		dev_err(rtd->dev, "Failed to add pcm1863 controls: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(adc_dai, SND_SOC_DAIFMT_CBS_CFS |
				  SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF);
	if (ret) {
		dev_err(rtd->dev, "Failed set set pcm1863 dai format: %d\n", ret);
		return ret;
	}

	return 0;
}

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

	if (data->adc_pluspro) {
		ret = imx_pcm186x_dai_init(rtd);
		if (ret)
			dev_warn(card->dev, "failed pcm186x dai init\n");
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
	int ret;

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

	ret = snd_soc_dai_set_bclk_ratio(codec_dai, (channels * width));
	if (ret) {
		dev_err(card->dev, "failed to set codec dai bclk ratio\n");
		return ret;
	}

	ret = snd_soc_dai_set_bclk_ratio(cpu_dai, (channels * width));
	if (ret) {
		dev_err(card->dev, "failed to set codec dai bclk ratio\n");
		return ret;
	}

	if (data->adc_pluspro) {
		struct snd_soc_dai_driver *drv = codec_dai->driver;
		const struct snd_soc_dai_ops *ops = drv->ops;

		ret = ops->hw_params(substream, params, codec_dai);
		if (ret)
			return ret;
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
		if (!substream->stream) {
			snd_soc_component_update_bits(comp, PCM512x_GPIO_EN, 0x08, 0x08);
			snd_soc_component_update_bits(comp, PCM512x_GPIO_OUTPUT_4, 0x0f, 0x02);
			snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);
		} else {
			if (data->adc_pluspro) {
				struct snd_soc_dai *adc_dai = snd_soc_rtd_to_codec(rtd, 1);
				struct snd_soc_component *adc = adc_dai->component;

				snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x40);
				snd_soc_component_write(adc, PCM186X_GPIO3_2_CTRL, 0x00);
				snd_soc_component_write(adc, PCM186X_GPIO3_2_DIR_CTRL, 0x04);
			}
		}
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

	if (data->dac_led_status) {
		if (!substream->stream)
			snd_soc_component_update_bits(comp, PCM512x_GPIO_CONTROL_1, 0x08, 0x00);
		else {
			if (data->adc_pluspro) {
				struct snd_soc_dai *adc_dai = snd_soc_rtd_to_codec(rtd, 1);
				struct snd_soc_component *adc = adc_dai->component;

				snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x00);
			}
		}
	}

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
	data->adc_pluspro =
		of_property_read_bool(np, "adc,pluspro");

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
	if (ret)
		return ret;

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

static void imx_pcm512x_remove(struct platform_device *pdev)
{
	struct imx_pcm512x_data *data = platform_get_drvdata(pdev);

	if (data->mute_gpio)
		gpiod_set_value_cansleep(data->mute_gpio, 0);

	snd_soc_unregister_card(&data->priv->snd_card);
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
