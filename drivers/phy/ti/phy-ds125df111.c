// SPDX-License-Identifier: GPL-2.0
/* Copyright 2024 NXP */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>

#define DS125DF111_NUM_CH 2
#define DS125DF111_NUM_VCO_GROUP_REG 5

struct ds125df111_ch {
	struct phy *phy;
	struct ds125df111_priv *priv;
	int idx;
};

struct ds125df111_priv {
	struct ds125df111_ch ch[DS125DF111_NUM_CH];
	struct i2c_client *client;
	struct mutex mutex; /* protects access to shared registers */
};

enum ds125df111_mode {
	FREQ_1G,
	FREQ_10G,
};

static const struct ds125df111_config {
	u8 vco_group[5];
	u8 rate;
	u8 subrate;
} ds125df111_cfg[] = {
	[FREQ_1G] = {
		/* VCO group #0 = 10GHz, VCO group #1 = 10GHz */
		.vco_group = {0x00, 0xB2, 0x00, 0xB2, 0xCC},
		/* Divide ratios of 1, 2, 4, 8 on both groups */
		.rate = 0x1,
		.subrate = 0x2,
	},

	[FREQ_10G] = {
		/* VCO group #0 = 10.3125GHz, VCO group #1 = 10.3125GHz */
		.vco_group = {0x90, 0xB3, 0x90, 0xB3, 0xCD},
		/* Divide ratios of 1 on both groups */
		.rate = 0x1,
		.subrate = 0x3,
	},
};

static int ds125df111_configure(struct phy *phy, const struct ds125df111_config *cfg)
{
	struct ds125df111_ch *ch = phy_get_drvdata(phy);
	struct ds125df111_priv *priv = ch->priv;
	struct i2c_client *i2c = priv->client;
	u8 val;
	int err, i;

	mutex_lock(&priv->mutex);

	/* Select the registers only for the current channel */
	err = i2c_smbus_read_byte_data(i2c, 0xFF);
	if (err < 0)
		goto out;
	val = (u8)err;
	val &= GENMASK(3, 0);
	val |= BIT(2) | ch->idx;
	i2c_smbus_write_byte_data(i2c, 0xff, val);

	/* Reset Channel Registers */
	err = i2c_smbus_read_byte_data(i2c, 0x00);
	if (err < 0)
		goto out;
	val = (u8)err;
	val |= BIT(2);
	err = i2c_smbus_write_byte_data(i2c, 0x00, val);
	if (err < 0)
		goto out;

	/* Program the VCO group frequencies */
	for (i = 0; i < DS125DF111_NUM_VCO_GROUP_REG; i++) {
		err = i2c_smbus_write_byte_data(i2c, 0x60 + i, cfg->vco_group[i]);
		if (err < 0)
			goto out;
	}

	/* Set the Divide Ratios for the VCO Groups*/
	err = i2c_smbus_read_byte_data(i2c, 0x2F);
	if (err < 0)
		goto out;
	val = (u8)err;
	val &= ~GENMASK(7, 4);
	val |= cfg->rate << 6 | cfg->subrate << 4;
	err = i2c_smbus_write_byte_data(i2c, 0x2F, val);
	if (err < 0)
		goto out;

	mutex_unlock(&priv->mutex);

	return 0;

out:
	mutex_unlock(&priv->mutex);

	return err;
}

static int ds125df111_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	const struct ds125df111_config *cfg;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	switch (submode) {
	case PHY_INTERFACE_MODE_10GBASER:
		cfg = &ds125df111_cfg[FREQ_10G];
		break;
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_SGMII:
		cfg = &ds125df111_cfg[FREQ_1G];
		break;
	default:
		return -EOPNOTSUPP;
	}

	return ds125df111_configure(phy, cfg);
}

static const struct phy_ops ds125df111_ops = {
	.set_mode	= ds125df111_set_mode,
	.owner		= THIS_MODULE,
};

static struct phy *ds125df111_xlate(struct device *dev,
				    const struct of_phandle_args *args)
{
	struct ds125df111_priv *priv = dev_get_drvdata(dev);
	int idx = args->args[0];

	if (WARN_ON(idx >= DS125DF111_NUM_CH))
		return ERR_PTR(-EINVAL);

	return priv->ch[idx].phy;
}

static int ds125df111_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct phy_provider *provider;
	struct ds125df111_priv *priv;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->client = client;
	mutex_init(&priv->mutex);

	i2c_set_clientdata(client, priv);

	for (i = 0; i < DS125DF111_NUM_CH; i++) {
		struct ds125df111_ch *ch = &priv->ch[i];
		struct phy *phy;

		phy = devm_phy_create(dev, NULL, &ds125df111_ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		ch->idx = i;
		ch->priv = priv;
		ch->phy = phy;

		phy_set_drvdata(phy, ch);
	}

	provider = devm_of_phy_provider_register(dev, ds125df111_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static const struct of_device_id ds125df111_dt_ids[] = {
	{ .compatible = "ti,ds125df111", },
	{},
};
MODULE_DEVICE_TABLE(of, ds125df111_dt_ids);

static struct i2c_driver ds125df111_driver = {
	.driver = {
		.name = "ds125df111",
		.owner = THIS_MODULE,
		.of_match_table = ds125df111_dt_ids,
	},
	.probe = ds125df111_probe,
};
module_i2c_driver(ds125df111_driver);

MODULE_AUTHOR("Ioana Ciornei <ioana.ciornei@nxp.com>");
MODULE_DESCRIPTION("TI DS125DF111 Retimer driver");
MODULE_LICENSE("GPL");
