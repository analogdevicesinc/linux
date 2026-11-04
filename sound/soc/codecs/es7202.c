// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2020 Everest Semiconductor Co Ltd.,
// Copyright (C) 2026 Jun Yan <jerrysteve1101@gmail.com>

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include "es7202.h"

/*
 * The main supply voltage selects the analog front-end configuration:
 * below this threshold the device is configured for 1.8 V, otherwise 3.3 V.
 */
#define ES7202_SUPPLY_1V8_MAX_UV	1980000
#define ES7202_SUPPLY_3V3_DEFAULT_UV	3300000

struct es7202_priv {
	struct regmap *regmap;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	unsigned int supply_uv;
};

static bool es7202_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ES7202_RESET:
	case ES7202_CHIP_STA:
		return true;
	default:
		return false;
	}
}

static bool es7202_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ES7202_DEV_ID1:
	case ES7202_DEV_ID0:
		return false;
	default:
		return true;
	}
}

static const struct reg_default es7202_reg_defaults[] = {
	{ ES7202_RESET,		0x10 },
	{ ES7202_SOFT_MODE,	0x00 },
	{ ES7202_CLK_DIV,	0x04 },
	{ ES7202_CLK_EN,	0x00 },
	{ ES7202_T1_VMID,	0x01 },
	{ ES7202_T2_VMID,	0x18 },
	{ ES7202_PDM_INF_CTL,	0x30 },
	{ ES7202_MISC_CTL,	0x02 },
	{ ES7202_ANALOG_EN,	0xff },
	{ ES7202_BIAS_VMID,	0x0c },
	{ ES7202_PGA1_BIAS,	0x55 },
	{ ES7202_PGA2_BIAS,	0x55 },
	{ ES7202_MOD1_BIAS,	0x8c },
	{ ES7202_MOD2_BIAS,	0x33 },
	{ ES7202_VREFP_BIAS,	0x33 },
	{ ES7202_VMMOD_BIAS,	0x33 },
	{ ES7202_MODS_BIAS,	0x44 },
	{ ES7202_ANALOG_LP1,	0x00 },
	{ ES7202_ANALOG_LP2,	0x00 },
	{ ES7202_ANALOG_MISC1,	0x00 },
	{ ES7202_ANALOG_MISC2,	0xf8 },
	{ ES7202_PGA1,		0x18 },
	{ ES7202_PGA2,		0x18 },
};

static const struct regmap_config es7202_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.use_single_read = true,
	.use_single_write = true,
	.max_register = ES7202_DEV_ID0,
	.reg_defaults = es7202_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es7202_reg_defaults),
	.writeable_reg = es7202_writeable_reg,
	.volatile_reg = es7202_volatile_reg,
	.cache_type = REGCACHE_MAPLE,
};

/*
 * PGA gain per the datasheet: code 0 is 0 dB, codes 1..10 step +3 dB from
 * 2.25 dB, and codes 11 and 12 are 30.75 dB and 32.25 dB respectively.  As
 * the steps are not uniform, a linear TLV (the vendor driver's
 * DECLARE_TLV_DB_SCALE(mic_boost_tlv, 0, 300, 0)) would misreport the gain,
 * so use a piecewise range matching the datasheet instead.
 */
static const DECLARE_TLV_DB_RANGE(mic_boost_tlv,
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 10, TLV_DB_SCALE_ITEM(225, 300, 0),
	11, 11, TLV_DB_SCALE_ITEM(3075, 0, 0),
	12, 12, TLV_DB_SCALE_ITEM(3225, 0, 0),
);

static const struct snd_kcontrol_new es7202_controls[] = {
	SOC_SINGLE_TLV("PGA1 Capture Volume", ES7202_PGA1, 0, 0x0c, 0,
		       mic_boost_tlv),
	SOC_SINGLE_TLV("PGA2 Capture Volume", ES7202_PGA2, 0, 0x0c, 0,
		       mic_boost_tlv),
};

static const struct snd_soc_dapm_widget es7202_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_ADC("ADC", "Capture", SND_SOC_NOPM, 0, 0),
};

static const struct snd_soc_dapm_route es7202_dapm_routes[] = {
	{ "ADC", NULL, "MIC1" },
	{ "ADC", NULL, "MIC2" },
};

static int es7202_mute_pdm(struct es7202_priv *es7202, bool mute)
{
	return regmap_update_bits(es7202->regmap, ES7202_PDM_INF_CTL,
				  ES7202_PDM_INF_CTL_MUTE,
				  mute ? ES7202_PDM_INF_CTL_MUTE : 0);
}

static int es7202_hw_init(struct es7202_priv *es7202)
{
	struct regmap *regmap = es7202->regmap;
	bool is_1v8 = es7202->supply_uv > 0 &&
		      es7202->supply_uv <= ES7202_SUPPLY_1V8_MAX_UV;
	/*
	 * Power-up sequence from the vendor driver.
	 * The analog front-end is configured for the actual supply voltage
	 * (1.8 V vs 3.3 V) and the analog blocks are ramped out of
	 * power-down in stages.
	 */
	const struct reg_sequence init_seq[] = {
		{ ES7202_SOFT_MODE,	ES7202_MODE_SOFT_MODE_SEL },
		{ ES7202_ANALOG_MISC1,	ES7202_ANALOG_MISC1_OFFSETSEL1 |
					(is_1v8 ? 0 : ES7202_ANALOG_MISC1_SUPPLYSEL) },

		/* analog on, rest powered down */
		{ ES7202_ANALOG_EN,	ES7202_ANALOG_EN_PDN_ADCVREFGEN |
					ES7202_ANALOG_EN_MODTOP1_RST |
					ES7202_ANALOG_EN_MODTOP2_RST |
					ES7202_ANALOG_EN_PDN_MOD1 |
					ES7202_ANALOG_EN_PDN_MOD2 |
					ES7202_ANALOG_EN_PDN_PGA1 |
					ES7202_ANALOG_EN_PDN_PGA2 },

		{ ES7202_BIAS_VMID,	FIELD_PREP(ES7202_BIAS_ADCBIAS_SWH1, 3) |
					ES7202_BIAS_ENDITHER |
					FIELD_PREP(ES7202_BIAS_VMIDSEL, 3) },

		/* VREFGEN on; 3.3 V also drops MODTOP reset */
		{ ES7202_ANALOG_EN,	is_1v8 ?
					(ES7202_ANALOG_EN_MODTOP1_RST |
					 ES7202_ANALOG_EN_MODTOP2_RST |
					 ES7202_ANALOG_EN_PDN_MOD1 |
					 ES7202_ANALOG_EN_PDN_MOD2 |
					 ES7202_ANALOG_EN_PDN_PGA1 |
					 ES7202_ANALOG_EN_PDN_PGA2) :
					(ES7202_ANALOG_EN_PDN_MOD1 |
					 ES7202_ANALOG_EN_PDN_MOD2 |
					 ES7202_ANALOG_EN_PDN_PGA1 |
					 ES7202_ANALOG_EN_PDN_PGA2) },

		{ ES7202_ANALOG_EN,	0 },
		{ ES7202_MOD1_BIAS,	FIELD_PREP(ES7202_MOD1_BIAS_SW0, 5) |
					FIELD_PREP(ES7202_MOD1_BIAS_SW1, 8) },
		{ ES7202_CLK_DIV,	FIELD_PREP(ES7202_CLK_DIV_MCLK_DIV, 1) },
		{ ES7202_T2_VMID,	0x01 },
		{ ES7202_MISC_CTL,	ES7202_MISC_DSM_DITHER_ON },
		{ ES7202_RESET,		ES7202_RESET_CSM_ON },
		{ ES7202_CLK_EN,	ES7202_CLK_ANA_CLK_ON | ES7202_CLK_MCLK_ON },
		{ ES7202_BIAS_VMID,	FIELD_PREP(ES7202_BIAS_ADCBIAS_SWH1, 3) |
					ES7202_BIAS_ENDITHER |
					FIELD_PREP(ES7202_BIAS_VMIDSEL, 2) },

		/* keep the PDM outputs muted until a capture stream starts */
		{ ES7202_PDM_INF_CTL,	ES7202_PDM_NEG_DATA_VALID |
					ES7202_PDM_POS_DATA_VALID |
					ES7202_PDM_INF_CTL_MUTE },
	};
	return regmap_multi_reg_write(regmap, init_seq, ARRAY_SIZE(init_seq));
}

static int es7202_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_component *component = dai->component;
	struct es7202_priv *es7202 = snd_soc_component_get_drvdata(component);

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	return es7202_mute_pdm(es7202, mute);
}

static const struct snd_soc_dai_ops es7202_dai_ops = {
	.mute_stream = es7202_mute_stream,
};

static struct snd_soc_dai_driver es7202_dai = {
	.name = "es7202-hifi",
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_96000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &es7202_dai_ops,
};

static int es7202_suspend(struct snd_soc_component *component)
{
	struct es7202_priv *es7202 = snd_soc_component_get_drvdata(component);

	return es7202_mute_pdm(es7202, true);
}

static int es7202_resume(struct snd_soc_component *component)
{
	struct es7202_priv *es7202 = snd_soc_component_get_drvdata(component);
	int ret;

	/* re-run the power-up sequence to bring the analog front-end back up */
	ret = es7202_hw_init(es7202);
	if (ret)
		return ret;

	regcache_mark_dirty(es7202->regmap);
	return regcache_sync(es7202->regmap);
}

static const struct snd_soc_component_driver es7202_component_driver = {
	.controls		= es7202_controls,
	.num_controls		= ARRAY_SIZE(es7202_controls),
	.dapm_widgets		= es7202_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es7202_dapm_widgets),
	.dapm_routes		= es7202_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(es7202_dapm_routes),
	.suspend		= es7202_suspend,
	.resume			= es7202_resume,
	.idle_bias_on		= 1,
	.endianness		= 1,
};

static void es7202_disable_supply(void *data)
{
	regulator_disable(data);
}

static int es7202_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct es7202_priv *es7202;
	unsigned int id1, id0, device_id;
	int ret;
	int voltage;

	es7202 = devm_kzalloc(dev, sizeof(*es7202), GFP_KERNEL);
	if (!es7202)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es7202);

	es7202->regmap = devm_regmap_init_i2c(i2c, &es7202_regmap);
	if (IS_ERR(es7202->regmap))
		return dev_err_probe(dev, PTR_ERR(es7202->regmap),
				     "failed to init regmap\n");

	es7202->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(es7202->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(es7202->reset_gpio),
				     "failed to get reset gpio\n");

	es7202->supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(es7202->supply))
		return dev_err_probe(dev, PTR_ERR(es7202->supply),
				     "failed to get supply\n");

	ret = regulator_enable(es7202->supply);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable supply\n");

	ret = devm_add_action_or_reset(dev, es7202_disable_supply, es7202->supply);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add supply disable action\n");

	voltage = regulator_get_voltage(es7202->supply);
	if (voltage < 0)
		es7202->supply_uv = ES7202_SUPPLY_3V3_DEFAULT_UV;
	else
		es7202->supply_uv = voltage;

	/* take the device out of reset */
	gpiod_set_value_cansleep(es7202->reset_gpio, 0);
	usleep_range(1000, 2000);

	/* verify the chip is present by reading its device id */
	ret = regmap_read(es7202->regmap, ES7202_DEV_ID1, &id1);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device id\n");
	ret = regmap_read(es7202->regmap, ES7202_DEV_ID0, &id0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device id\n");

	device_id = (id1 << 8) | id0;
	if (device_id != ES7202_DEVICE_ID)
		return dev_err_probe(dev, -ENODEV,
				     "unexpected device id 0x%04x, expected 0x%04x\n",
				     device_id, ES7202_DEVICE_ID);

	ret = es7202_hw_init(es7202);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init device\n");

	return devm_snd_soc_register_component(dev, &es7202_component_driver,
					       &es7202_dai, 1);
}

static void es7202_i2c_remove(struct i2c_client *i2c)
{
	struct es7202_priv *es7202 = i2c_get_clientdata(i2c);

	es7202_mute_pdm(es7202, true);
}

static void es7202_i2c_shutdown(struct i2c_client *i2c)
{
	struct es7202_priv *es7202 = i2c_get_clientdata(i2c);

	es7202_mute_pdm(es7202, true);
}

static const struct of_device_id es7202_of_match[] = {
	{ .compatible = "everest,es7202" },
	{ }
};
MODULE_DEVICE_TABLE(of, es7202_of_match);

static const struct i2c_device_id es7202_i2c_id[] = {
	{ "es7202" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, es7202_i2c_id);

static struct i2c_driver es7202_i2c_driver = {
	.driver = {
		.name		= "es7202",
		.of_match_table	= es7202_of_match,
	},
	.probe		= es7202_i2c_probe,
	.remove		= es7202_i2c_remove,
	.shutdown	= es7202_i2c_shutdown,
	.id_table	= es7202_i2c_id,
};
module_i2c_driver(es7202_i2c_driver);

MODULE_DESCRIPTION("Everest Semi ES7202 PDM ADC ALSA SoC Codec Driver");
MODULE_AUTHOR("David Yang <yangxiaohua@everest-semi.com>");
MODULE_AUTHOR("Jun Yan <jerrysteve1101@gmail.com>");
MODULE_LICENSE("GPL");
