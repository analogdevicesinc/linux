// SPDX-License-Identifier: GPL-2.0
/*
 * Renesas RZ/V2H(P) VBENCTL VBUS_SEL mux driver
 *
 * Copyright (C) 2026 Renesas Electronics Corp.
 */

#include <linux/auxiliary_bus.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mux/driver.h>
#include <linux/regmap.h>

#define RZV2H_VBENCTL		0xf0c

struct mux_rzv2h_vbenctl_priv {
	struct regmap_field *field;
};

static int mux_rzv2h_vbenctl_set(struct mux_control *mux, int state)
{
	struct mux_rzv2h_vbenctl_priv *priv = mux_chip_priv(mux->chip);

	return regmap_field_write(priv->field, state);
}

static const struct mux_control_ops mux_rzv2h_vbenctl_ops = {
	.set = mux_rzv2h_vbenctl_set,
};

static int mux_rzv2h_vbenctl_probe(struct auxiliary_device *adev,
				   const struct auxiliary_device_id *id)
{
	const struct reg_field reg_field = REG_FIELD(RZV2H_VBENCTL, 0, 0);
	struct mux_rzv2h_vbenctl_priv *priv;
	struct device *dev = &adev->dev;
	struct mux_chip *mux_chip;
	struct regmap *regmap;
	int ret;

	regmap = dev_get_regmap(dev->parent, NULL);
	if (!regmap)
		return -ENODEV;

	mux_chip = devm_mux_chip_alloc(dev, 1, sizeof(*priv));
	if (IS_ERR(mux_chip))
		return PTR_ERR(mux_chip);

	priv = mux_chip_priv(mux_chip);

	priv->field = devm_regmap_field_alloc(dev, regmap, reg_field);
	if (IS_ERR(priv->field))
		return PTR_ERR(priv->field);

	mux_chip->ops = &mux_rzv2h_vbenctl_ops;
	mux_chip->mux[0].states = 2;
	mux_chip->mux[0].idle_state = MUX_IDLE_AS_IS;

	ret = devm_mux_chip_register(dev, mux_chip);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to register mux chip\n");

	return 0;
}

static const struct auxiliary_device_id mux_rzv2h_vbenctl_ids[] = {
	{ .name = "rzv2h_usb2phy_reset.vbenctl" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(auxiliary, mux_rzv2h_vbenctl_ids);

static struct auxiliary_driver mux_rzv2h_vbenctl_driver = {
	.name		= "vbenctl",
	.probe		= mux_rzv2h_vbenctl_probe,
	.id_table	= mux_rzv2h_vbenctl_ids,
};
module_auxiliary_driver(mux_rzv2h_vbenctl_driver);

MODULE_DESCRIPTION("RZ/V2H VBENCTL VBUS_SEL mux driver");
MODULE_AUTHOR("Tommaso Merciai <tommaso.merciai.xr@bp.renesas.com>");
MODULE_LICENSE("GPL");
