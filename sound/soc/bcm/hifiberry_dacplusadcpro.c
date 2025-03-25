/*
 * ASoC Driver for HiFiBerry DAC+ / DAC Pro with ADC PRO Version (SW control)
 *
 * Author:	Daniel Matuschek, Stuart MacLean <stuart@hifiberry.com>
 *		Copyright 2014-2015
 *		based on code by Florian Meier <florian.meier@koalo.de>
 *		ADC, HP added by Joerg Schambacher <joerg@hifiberry.com>
 *		Copyright 2018-21
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/tlv.h>

#include "../codecs/pcm512x.h"
#include "../codecs/pcm186x.h"
#include "hifiberry_adc_controls.h"

#define HIFIBERRY_DACPRO_NOCLOCK 0
#define HIFIBERRY_DACPRO_CLK44EN 1
#define HIFIBERRY_DACPRO_CLK48EN 2

struct pcm512x_priv {
	struct regmap *regmap;
	struct clk *sclk;
};

/* Clock rate of CLK44EN attached to GPIO6 pin */
#define CLK_44EN_RATE 22579200UL
/* Clock rate of CLK48EN attached to GPIO3 pin */
#define CLK_48EN_RATE 24576000UL

static bool slave;
static bool snd_rpi_hifiberry_is_dacpro;
static bool digital_gain_0db_limit = true;
static bool leds_off;

static int pcm1863_add_controls(struct snd_soc_component *component)
{
	snd_soc_add_component_controls(component,
			pcm1863_snd_controls_card,
			ARRAY_SIZE(pcm1863_snd_controls_card));
	return 0;
}

static void snd_rpi_hifiberry_dacplusadcpro_select_clk(
					struct snd_soc_component *component, int clk_id)
{
	switch (clk_id) {
	case HIFIBERRY_DACPRO_NOCLOCK:
		snd_soc_component_update_bits(component,
				PCM512x_GPIO_CONTROL_1, 0x24, 0x00);
		break;
	case HIFIBERRY_DACPRO_CLK44EN:
		snd_soc_component_update_bits(component,
				PCM512x_GPIO_CONTROL_1, 0x24, 0x20);
		break;
	case HIFIBERRY_DACPRO_CLK48EN:
		snd_soc_component_update_bits(component,
				PCM512x_GPIO_CONTROL_1, 0x24, 0x04);
		break;
	}
	usleep_range(3000, 4000);
}

static void snd_rpi_hifiberry_dacplusadcpro_clk_gpio(struct snd_soc_component *component)
{
	snd_soc_component_update_bits(component, PCM512x_GPIO_EN, 0x24, 0x24);
	snd_soc_component_update_bits(component, PCM512x_GPIO_OUTPUT_3, 0x0f, 0x02);
	snd_soc_component_update_bits(component, PCM512x_GPIO_OUTPUT_6, 0x0f, 0x02);
}

static bool snd_rpi_hifiberry_dacplusadcpro_is_sclk(struct snd_soc_component *component)
{
	unsigned int sck;

	sck = snd_soc_component_read(component, PCM512x_RATE_DET_4);
	return (!(sck & 0x40));
}

static bool snd_rpi_hifiberry_dacplusadcpro_is_pro_card(struct snd_soc_component *component)
{
	bool isClk44EN, isClk48En, isNoClk;

	snd_rpi_hifiberry_dacplusadcpro_clk_gpio(component);

	snd_rpi_hifiberry_dacplusadcpro_select_clk(component, HIFIBERRY_DACPRO_CLK44EN);
	isClk44EN = snd_rpi_hifiberry_dacplusadcpro_is_sclk(component);

	snd_rpi_hifiberry_dacplusadcpro_select_clk(component, HIFIBERRY_DACPRO_NOCLOCK);
	isNoClk = snd_rpi_hifiberry_dacplusadcpro_is_sclk(component);

	snd_rpi_hifiberry_dacplusadcpro_select_clk(component, HIFIBERRY_DACPRO_CLK48EN);
	isClk48En = snd_rpi_hifiberry_dacplusadcpro_is_sclk(component);

	return (isClk44EN && isClk48En && !isNoClk);
}

static int snd_rpi_hifiberry_dacplusadcpro_clk_for_rate(int sample_rate)
{
	int type;

	switch (sample_rate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
	case 352800:
		type = HIFIBERRY_DACPRO_CLK44EN;
		break;
	default:
		type = HIFIBERRY_DACPRO_CLK48EN;
		break;
	}
	return type;
}

static void snd_rpi_hifiberry_dacplusadcpro_set_sclk(struct snd_soc_component *component,
	int sample_rate)
{
	struct pcm512x_priv *pcm512x = snd_soc_component_get_drvdata(component);

	if (!IS_ERR(pcm512x->sclk)) {
		int ctype;

		ctype = snd_rpi_hifiberry_dacplusadcpro_clk_for_rate(sample_rate);
		clk_set_rate(pcm512x->sclk, (ctype == HIFIBERRY_DACPRO_CLK44EN)
			? CLK_44EN_RATE : CLK_48EN_RATE);
		snd_rpi_hifiberry_dacplusadcpro_select_clk(component, ctype);
	}
}

static int snd_rpi_hifiberry_dacplusadcpro_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *dac = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_component *adc = snd_soc_rtd_to_codec(rtd, 1)->component;
	struct snd_soc_dai_driver *adc_driver = snd_soc_rtd_to_codec(rtd, 1)->driver;
	struct pcm512x_priv *priv;
	int ret;

	if (slave)
		snd_rpi_hifiberry_is_dacpro = false;
	else
		snd_rpi_hifiberry_is_dacpro =
				snd_rpi_hifiberry_dacplusadcpro_is_pro_card(dac);

	if (snd_rpi_hifiberry_is_dacpro) {
		struct snd_soc_dai_link *dai = rtd->dai_link;

		dai->name = "HiFiBerry DAC+ADC Pro";
		dai->stream_name = "HiFiBerry DAC+ADC Pro HiFi";
		dai->dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBM_CFM;

		// set DAC DAI configuration
		ret = snd_soc_dai_set_fmt(snd_soc_rtd_to_codec(rtd, 0),
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBM_CFM);
		if (ret < 0)
			return ret;

		// set ADC DAI configuration
		ret = snd_soc_dai_set_fmt(snd_soc_rtd_to_codec(rtd, 1),
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
			| SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			return ret;

		// set CPU DAI configuration
		ret = snd_soc_dai_set_fmt(snd_soc_rtd_to_cpu(rtd, 0),
			SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
		if (ret < 0)
			return ret;

		snd_soc_component_update_bits(dac, PCM512x_BCLK_LRCLK_CFG, 0x31, 0x11);
		snd_soc_component_update_bits(dac, PCM512x_MASTER_MODE, 0x03, 0x03);
		snd_soc_component_update_bits(dac, PCM512x_MASTER_CLKDIV_2, 0x7f, 63);
	} else {
		priv = snd_soc_component_get_drvdata(dac);
		priv->sclk = ERR_PTR(-ENOENT);
	}

	/* disable 24bit mode as long as I2S module does not have sign extension fixed */
	adc_driver->capture.formats = SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S16_LE;

	snd_soc_component_update_bits(dac, PCM512x_GPIO_EN, 0x08, 0x08);
	snd_soc_component_update_bits(dac, PCM512x_GPIO_OUTPUT_4, 0x0f, 0x02);
	if (leds_off)
		snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x00);
	else
		snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);

	ret = pcm1863_add_controls(adc);
	if (ret < 0)
		dev_warn(rtd->dev, "Failed to add pcm1863 controls: %d\n",
		ret);

	/* set GPIO2 to output, GPIO3 input */
	snd_soc_component_write(adc, PCM186X_GPIO3_2_CTRL, 0x00);
	snd_soc_component_write(adc, PCM186X_GPIO3_2_DIR_CTRL, 0x04);
	if (leds_off)
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x00);
	else
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x40);

	if (digital_gain_0db_limit) {
		int ret;
		struct snd_soc_card *card = rtd->card;

		ret = snd_soc_limit_volume(card, "Digital Playback Volume", 207);
		if (ret < 0)
			dev_warn(card->dev, "Failed to set volume limit: %d\n", ret);
	}

	return 0;
}

static int snd_rpi_hifiberry_dacplusadcpro_update_rate_den(
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component = snd_soc_rtd_to_codec(rtd, 0)->component; /* only use DAC */
	struct pcm512x_priv *pcm512x = snd_soc_component_get_drvdata(component);
	struct snd_ratnum *rats_no_pll;
	unsigned int num = 0, den = 0;
	int err;

	rats_no_pll = devm_kzalloc(rtd->dev, sizeof(*rats_no_pll), GFP_KERNEL);
	if (!rats_no_pll)
		return -ENOMEM;

	rats_no_pll->num = clk_get_rate(pcm512x->sclk) / 64;
	rats_no_pll->den_min = 1;
	rats_no_pll->den_max = 128;
	rats_no_pll->den_step = 1;

	err = snd_interval_ratnum(hw_param_interval(params,
		SNDRV_PCM_HW_PARAM_RATE), 1, rats_no_pll, &num, &den);
	if (err >= 0 && den) {
		params->rate_num = num;
		params->rate_den = den;
	}

	devm_kfree(rtd->dev, rats_no_pll);
	return 0;
}

static int snd_rpi_hifiberry_dacplusadcpro_hw_params(
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	int channels = params_channels(params);
	int width = snd_pcm_format_width(params_format(params));
	struct snd_soc_component *dac = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_dai *dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_dai_driver *drv = dai->driver;
	const struct snd_soc_dai_ops *ops = drv->ops;

	/* Using powers of 2 allows for an integer clock divisor */
	width = width <= 16 ? 16 : 32;

	if (snd_rpi_hifiberry_is_dacpro) {
		snd_rpi_hifiberry_dacplusadcpro_set_sclk(dac,
			params_rate(params));

		ret = snd_rpi_hifiberry_dacplusadcpro_update_rate_den(
			substream, params);
		if (ret)
			return ret;
	}

	ret = snd_soc_dai_set_bclk_ratio(snd_soc_rtd_to_cpu(rtd, 0), channels * width);
	if (ret)
		return ret;
	ret = snd_soc_dai_set_bclk_ratio(snd_soc_rtd_to_codec(rtd, 0), channels * width);
	if (ret)
		return ret;
	if (snd_rpi_hifiberry_is_dacpro && ops->hw_params)
		ret = ops->hw_params(substream, params, dai);
	return ret;
}

static int snd_rpi_hifiberry_dacplusadcpro_startup(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *dac = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_component *adc = snd_soc_rtd_to_codec(rtd, 1)->component;

	if (leds_off)
		return 0;
	/* switch on respective LED */
	if (!substream->stream)
		snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x08);
	else
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x40);
	return 0;
}

static void snd_rpi_hifiberry_dacplusadcpro_shutdown(
	struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *dac = snd_soc_rtd_to_codec(rtd, 0)->component;
	struct snd_soc_component *adc = snd_soc_rtd_to_codec(rtd, 1)->component;

	/* switch off respective LED */
	if (!substream->stream)
		snd_soc_component_update_bits(dac, PCM512x_GPIO_CONTROL_1, 0x08, 0x00);
	else
		snd_soc_component_update_bits(adc, PCM186X_GPIO_IN_OUT, 0x40, 0x00);
}


/* machine stream operations */
static struct snd_soc_ops snd_rpi_hifiberry_dacplusadcpro_ops = {
	.hw_params = snd_rpi_hifiberry_dacplusadcpro_hw_params,
	.startup = snd_rpi_hifiberry_dacplusadcpro_startup,
	.shutdown = snd_rpi_hifiberry_dacplusadcpro_shutdown,
};

SND_SOC_DAILINK_DEFS(hifi,
	DAILINK_COMP_ARRAY(COMP_CPU("bcm2708-i2s.0")),
	DAILINK_COMP_ARRAY(COMP_CODEC("pcm512x.1-004d", "pcm512x-hifi"),
			   COMP_CODEC("pcm186x.1-004a", "pcm1863-aif")),
	DAILINK_COMP_ARRAY(COMP_PLATFORM("bcm2708-i2s.0")));

static struct snd_soc_dai_link snd_rpi_hifiberry_dacplusadcpro_dai[] = {
{
	.name		= "HiFiBerry DAC+ADC PRO",
	.stream_name	= "HiFiBerry DAC+ADC PRO HiFi",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
	.ops		= &snd_rpi_hifiberry_dacplusadcpro_ops,
	.init		= snd_rpi_hifiberry_dacplusadcpro_init,
	SND_SOC_DAILINK_REG(hifi),
},
};

/* aux device for optional headphone amp */
static struct snd_soc_aux_dev hifiberry_dacplusadcpro_aux_devs[] = {
	{
		.dlc = {
			.name = "tpa6130a2.1-0060",
		},
	},
};

/* audio machine driver */
static struct snd_soc_card snd_rpi_hifiberry_dacplusadcpro = {
	.name         = "snd_rpi_hifiberry_dacplusadcpro",
	.driver_name  = "HifiberryDacpAdcPro",
	.owner        = THIS_MODULE,
	.dai_link     = snd_rpi_hifiberry_dacplusadcpro_dai,
	.num_links    = ARRAY_SIZE(snd_rpi_hifiberry_dacplusadcpro_dai),
};

static int hb_hp_detect(void)
{
	struct i2c_adapter *adap = i2c_get_adapter(1);
	int ret;
	struct i2c_client tpa_i2c_client = {
		.addr = 0x60,
		.adapter = adap,
	};

	if (!adap)
		return -EPROBE_DEFER;	/* I2C module not yet available */

	ret = i2c_smbus_read_byte(&tpa_i2c_client) >= 0;
	i2c_put_adapter(adap);
	return ret;
};

static struct property tpa_enable_prop = {
	       .name = "status",
	       .length = 4 + 1, /* length 'okay' + 1 */
	       .value = "okay",
	};

static int snd_rpi_hifiberry_dacplusadcpro_probe(struct platform_device *pdev)
{
	int ret = 0, i = 0;
	struct snd_soc_card *card = &snd_rpi_hifiberry_dacplusadcpro;
	struct device_node *tpa_node;
	struct property *tpa_prop;
	struct of_changeset ocs;
	int len;

	/* probe for head phone amp */
	ret = hb_hp_detect();
	if (ret < 0)
		return ret;
	if (ret) {
		card->aux_dev = hifiberry_dacplusadcpro_aux_devs;
		card->num_aux_devs =
				ARRAY_SIZE(hifiberry_dacplusadcpro_aux_devs);
		tpa_node = of_find_compatible_node(NULL, NULL, "ti,tpa6130a2");
		tpa_prop = of_find_property(tpa_node, "status", &len);

		if (strcmp((char *)tpa_prop->value, "okay")) {
			/* and activate headphone using change_sets */
			dev_info(&pdev->dev, "activating headphone amplifier");
			of_changeset_init(&ocs);
			ret = of_changeset_update_property(&ocs, tpa_node,
							&tpa_enable_prop);
			if (ret) {
				dev_err(&pdev->dev,
				"cannot activate headphone amplifier\n");
				return -ENODEV;
			}
			ret = of_changeset_apply(&ocs);
			if (ret) {
				dev_err(&pdev->dev,
				"cannot activate headphone amplifier\n");
				return -ENODEV;
			}
		}
	}

	snd_rpi_hifiberry_dacplusadcpro.dev = &pdev->dev;
	if (pdev->dev.of_node) {
		struct device_node *i2s_node;
		struct snd_soc_dai_link *dai;

		dai = &snd_rpi_hifiberry_dacplusadcpro_dai[0];
		i2s_node = of_parse_phandle(pdev->dev.of_node,
			"i2s-controller", 0);
		if (i2s_node) {
			for (i = 0; i < card->num_links; i++) {
				dai->cpus->dai_name = NULL;
				dai->cpus->of_node = i2s_node;
				dai->platforms->name = NULL;
				dai->platforms->of_node = i2s_node;
			}
		}
	}
	digital_gain_0db_limit = !of_property_read_bool(
		pdev->dev.of_node, "hifiberry-dacplusadcpro,24db_digital_gain");
	slave = of_property_read_bool(pdev->dev.of_node,
					"hifiberry-dacplusadcpro,slave");
	leds_off = of_property_read_bool(pdev->dev.of_node,
					"hifiberry-dacplusadcpro,leds_off");
	ret = snd_soc_register_card(&snd_rpi_hifiberry_dacplusadcpro);
	if (ret && ret != -EPROBE_DEFER)
		dev_err(&pdev->dev,
			"snd_soc_register_card() failed: %d\n", ret);

	return ret;
}

static const struct of_device_id snd_rpi_hifiberry_dacplusadcpro_of_match[] = {
	{ .compatible = "hifiberry,hifiberry-dacplusadcpro", },
	{},
};

MODULE_DEVICE_TABLE(of, snd_rpi_hifiberry_dacplusadcpro_of_match);

static struct platform_driver snd_rpi_hifiberry_dacplusadcpro_driver = {
	.driver = {
		.name   = "snd-rpi-hifiberry-dacplusadcpro",
		.owner  = THIS_MODULE,
		.of_match_table = snd_rpi_hifiberry_dacplusadcpro_of_match,
	},
	.probe          = snd_rpi_hifiberry_dacplusadcpro_probe,
};

module_platform_driver(snd_rpi_hifiberry_dacplusadcpro_driver);

MODULE_AUTHOR("Joerg Schambacher <joerg@hifiberry.com>");
MODULE_AUTHOR("Daniel Matuschek <daniel@hifiberry.com>");
MODULE_DESCRIPTION("ASoC Driver for HiFiBerry DAC+ADC");
MODULE_LICENSE("GPL v2");
