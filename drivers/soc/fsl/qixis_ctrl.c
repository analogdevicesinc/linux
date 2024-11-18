// SPDX-License-Identifier: GPL-2.0+

/* Freescale QIXIS system controller driver.
 *
 * Copyright 2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2019 NXP
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/of.h>
#include <linux/regmap.h>

/* QIXIS MAP */
struct fsl_qixis_regs {
	u8		id;		/* Identification Registers */
	u8		version;	/* Version Register */
	u8		qixis_ver;	/* QIXIS Version Register */
	u8		reserved1[0x1f];
};

struct qixis_priv {
	struct regmap		*regmap;
};

static struct regmap_config qixis_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct mfd_cell fsl_qixis_devs[] = {
	{
		.name = "reg-mux",
		.of_compatible = "reg-mux",
	},
};

static int fsl_qixis_i2c_probe(struct i2c_client *client)
{
	struct qixis_priv *priv;
	int ret = 0;
	u32 qver;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	priv = devm_kzalloc(&client->dev, sizeof(struct qixis_priv),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = regmap_init_i2c(client, &qixis_regmap_config);
	regmap_read(priv->regmap, offsetof(struct fsl_qixis_regs, qixis_ver),
		    &qver);
	pr_info("Freescale QIXIS Version: 0x%08x\n", qver);

	i2c_set_clientdata(client, priv);

	if (of_device_is_compatible(client->dev.of_node, "simple-mfd"))
		ret = devm_mfd_add_devices(&client->dev, -1, fsl_qixis_devs,
					   ARRAY_SIZE(fsl_qixis_devs), NULL, 0,
					   NULL);
	if (ret)
		goto error;

	return ret;
error:
	regmap_exit(priv->regmap);

	return ret;
}

static void fsl_qixis_i2c_remove(struct i2c_client *client)
{
	struct qixis_priv *priv;

	priv = i2c_get_clientdata(client);
	regmap_exit(priv->regmap);
}

static const struct of_device_id fsl_qixis_i2c_of_match[] = {
	{ .compatible = "fsl,fpga-qixis-i2c" },
	{}
};
MODULE_DEVICE_TABLE(of, fsl_qixis_i2c_of_match);

static struct i2c_driver fsl_qixis_i2c_driver = {
	.driver = {
		.name	= "qixis_ctrl_i2c",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(fsl_qixis_i2c_of_match),
	},
	.probe		= fsl_qixis_i2c_probe,
	.remove		= fsl_qixis_i2c_remove,
};
module_i2c_driver(fsl_qixis_i2c_driver);

MODULE_AUTHOR("Wang Dongsheng <dongsheng.wang@freescale.com>");
MODULE_DESCRIPTION("Freescale QIXIS system controller driver");
MODULE_LICENSE("GPL");

