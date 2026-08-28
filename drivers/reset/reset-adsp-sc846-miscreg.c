// SPDX-License-Identifier: GPL-2.0-only
/*
 * Reset controller for the MISCREG integration register file found on Analog
 * Devices ADSP-SC84x processors.
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/container_of.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/device/devres.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/types.h>

#include <dt-bindings/reset/adi,sc846-miscreg-reset.h>

#define SC846_MISCREG_CAN_SYSCTL		0x010
#define SC846_MISCREG_CAN_SYSCTL_CAN0_SRST	BIT(3)
#define SC846_MISCREG_CAN_SYSCTL_CAN1_SRST	BIT(7)

#define SC846_MISCREG_XSPI_RSTCTL		0x130
#define SC846_MISCREG_XSPI1_RSTCTL		0x330
/*
 * NHRST_MSK (controller system reset), NPRST_MSK (controller and PHY MMR
 * reset) and NRFRST_MSK (controller and PHY xSPI clock domain reset) mask the
 * respective active low reset inputs, so a set bit releases the domain. All
 * three are cleared out of power-on reset and the boot ROM only releases them
 * when it boots from the matching xSPI instance.
 */
#define SC846_MISCREG_XSPI_RSTCTL_MSK		GENMASK(2, 0)

#define to_sc846_miscreg_reset(_rcdev) \
	container_of(_rcdev, struct sc846_miscreg_reset, rcdev)

struct sc846_miscreg_reset_line {
	unsigned int	reg;
	u32		mask;
	bool		active_low;
};

struct sc846_miscreg_reset {
	struct reset_controller_dev rcdev;
	struct regmap *regmap;
};

static const struct sc846_miscreg_reset_line sc846_reset_lines[] = {
	[ADI_SC846_RESET_XSPI0] = {
		.reg		= SC846_MISCREG_XSPI_RSTCTL,
		.mask		= SC846_MISCREG_XSPI_RSTCTL_MSK,
		.active_low	= true,
	},
	[ADI_SC846_RESET_XSPI1] = {
		.reg		= SC846_MISCREG_XSPI1_RSTCTL,
		.mask		= SC846_MISCREG_XSPI_RSTCTL_MSK,
		.active_low	= true,
	},
	[ADI_SC846_RESET_CAN0] = {
		.reg	= SC846_MISCREG_CAN_SYSCTL,
		.mask	= SC846_MISCREG_CAN_SYSCTL_CAN0_SRST,
	},
	[ADI_SC846_RESET_CAN1] = {
		.reg	= SC846_MISCREG_CAN_SYSCTL,
		.mask	= SC846_MISCREG_CAN_SYSCTL_CAN1_SRST,
	},
};

static int sc846_miscreg_reset_update(struct reset_controller_dev *rcdev,
				      unsigned long id, bool assert)
{
	struct sc846_miscreg_reset *priv = to_sc846_miscreg_reset(rcdev);
	const struct sc846_miscreg_reset_line *line = &sc846_reset_lines[id];

	return regmap_assign_bits(priv->regmap, line->reg, line->mask,
				  assert != line->active_low);
}

static int sc846_miscreg_reset_assert(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	return sc846_miscreg_reset_update(rcdev, id, true);
}

static int sc846_miscreg_reset_deassert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	return sc846_miscreg_reset_update(rcdev, id, false);
}

static int sc846_miscreg_reset_reset(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	int ret;

	ret = sc846_miscreg_reset_assert(rcdev, id);
	if (ret)
		return ret;

	/*
	 * The minimum pulse width is not documented for any of these lines, so
	 * use a conservative delay.
	 */
	fsleep(1);

	return sc846_miscreg_reset_deassert(rcdev, id);
}

static int sc846_miscreg_reset_status(struct reset_controller_dev *rcdev,
				      unsigned long id)
{
	struct sc846_miscreg_reset *priv = to_sc846_miscreg_reset(rcdev);
	const struct sc846_miscreg_reset_line *line = &sc846_reset_lines[id];
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, line->reg, &val);
	if (ret)
		return ret;

	if (line->active_low)
		val = ~val;

	/* the line is asserted while any bit of the group is */
	return !!(val & line->mask);
}

static const struct reset_control_ops sc846_miscreg_reset_ops = {
	.assert		= sc846_miscreg_reset_assert,
	.deassert	= sc846_miscreg_reset_deassert,
	.reset		= sc846_miscreg_reset_reset,
	.status		= sc846_miscreg_reset_status,
};

static int sc846_miscreg_reset_probe(struct platform_device *pdev)
{
	struct sc846_miscreg_reset *priv;
	struct device *dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = syscon_node_to_regmap(dev_of_node(dev->parent));
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to get the MISCREG register map\n");

	priv->rcdev.ops = &sc846_miscreg_reset_ops;
	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.dev = dev;
	priv->rcdev.of_node = dev->of_node;
	priv->rcdev.nr_resets = ARRAY_SIZE(sc846_reset_lines);

	return devm_reset_controller_register(dev, &priv->rcdev);
}

static const struct of_device_id sc846_miscreg_reset_of_match[] = {
	{ .compatible = "adi,sc846-miscreg-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc846_miscreg_reset_of_match);

static struct platform_driver sc846_miscreg_reset_driver = {
	.probe = sc846_miscreg_reset_probe,
	.driver = {
		.name = "adsp-sc846-miscreg-reset",
		.of_match_table = sc846_miscreg_reset_of_match,
	},
};
module_platform_driver(sc846_miscreg_reset_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADSP-SC846 MISCREG reset controller");
MODULE_LICENSE("GPL");
