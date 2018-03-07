/*
 * Copyright 2020 NXP
 *
 * Copyright 2007-11 Wolfson Microelectronics, plc
 *
 * Author: Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/wm8960.h>
#include "wm8960.h"
#include "../fsl/imx-pcm-rpmsg.h"
#include "rpmsg_wm8960.h"

extern struct regmap_config rpmsg_wm8960_regmap;
extern const struct snd_soc_component_driver rpmsg_wm8960_component;
extern struct snd_soc_dai_driver rpmsg_wm8960_codec_dai;

static void rpmsg_wm8960_set_pdata_from_of(struct i2c_client *i2c,
					   struct wm8960_data *pdata)
{
	const struct device_node *np = i2c->dev.of_node;

	if (of_property_read_bool(np, "wlf,capless"))
		pdata->capless = true;

	if (of_property_read_bool(np, "wlf,shared-lrclk"))
		pdata->shared_lrclk = true;
}

static int rpmsg_wm8960_i2c_probe(struct i2c_client *i2c)
{
	struct wm8960_data *pdata = dev_get_platdata(&i2c->dev);
	struct rpmsg_wm8960_priv *wm8960;
	int ret;
	int repeat_reset = 10;

	wm8960 = devm_kzalloc(&i2c->dev, sizeof(struct rpmsg_wm8960_priv),
			      GFP_KERNEL);
	if (wm8960 == NULL)
		return -ENOMEM;

	wm8960->mclk = devm_clk_get(&i2c->dev, "mclk");
	if (IS_ERR(wm8960->mclk)) {
		if (PTR_ERR(wm8960->mclk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	}

	rpmsg_wm8960_regmap.reg_read = NULL;
	rpmsg_wm8960_regmap.reg_write = NULL;

	wm8960->regmap = devm_regmap_init_i2c(i2c, &rpmsg_wm8960_regmap);
	if (IS_ERR(wm8960->regmap))
		return PTR_ERR(wm8960->regmap);

	if (pdata)
		memcpy(&wm8960->pdata, pdata, sizeof(struct wm8960_data));
	else if (i2c->dev.of_node)
		rpmsg_wm8960_set_pdata_from_of(i2c, &wm8960->pdata);

	i2c_set_clientdata(i2c, wm8960);

	do {
		ret = wm8960_reset(wm8960->regmap);
		repeat_reset--;
	} while (repeat_reset > 0 && ret != 0);

	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to issue reset\n");
		return ret;
	}

	if (wm8960->pdata.shared_lrclk) {
		ret = regmap_update_bits(wm8960->regmap, WM8960_ADDCTL2,
					 0x4, 0x4);
		if (ret != 0) {
			dev_err(&i2c->dev, "Failed to enable LRCM: %d\n",
				ret);
			return ret;
		}
	}

	/* Latch the update bits */
	regmap_update_bits(wm8960->regmap, WM8960_LINVOL, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_RINVOL, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_LADC, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_RADC, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_LDAC, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_RDAC, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_LOUT1, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_ROUT1, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_LOUT2, 0x100, 0x100);
	regmap_update_bits(wm8960->regmap, WM8960_ROUT2, 0x100, 0x100);

	/*
	 * codec ADCLRC pin configured as GPIO, DACLRC pin is used as a frame
	 * clock for ADCs and DACs
	 */
	regmap_update_bits(wm8960->regmap, WM8960_IFACE2, 1<<6, 1<<6);

	ret = devm_snd_soc_register_component(&i2c->dev,
			&rpmsg_wm8960_component, &rpmsg_wm8960_codec_dai, 1);

	return ret;
}

static void rpmsg_wm8960_i2c_remove(struct i2c_client *client)
{}

static const struct i2c_device_id rpmsg_wm8960_i2c_id[] = {
	{ "wm8960", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rpmsg_wm8960_i2c_id);

static const struct of_device_id rpmsg_wm8960_of_match[] = {
       { .compatible = "wlf,wm8960,lpa", },
       { }
};
MODULE_DEVICE_TABLE(of, rpmsg_wm8960_of_match);

static struct i2c_driver rpmsg_wm8960_i2c_driver = {
	.driver = {
		.name = RPMSG_CODEC_DRV_NAME_WM8960,
		.of_match_table = rpmsg_wm8960_of_match,
	},
	.probe =    rpmsg_wm8960_i2c_probe,
	.remove =   rpmsg_wm8960_i2c_remove,
	.id_table = rpmsg_wm8960_i2c_id,
};

module_i2c_driver(rpmsg_wm8960_i2c_driver);
MODULE_DESCRIPTION("rpmsg wm8960 I2C Codec Driver");
MODULE_LICENSE("GPL v2");
