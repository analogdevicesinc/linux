// SPDX-License-Identifier: GPL-2.0
//
// Renesas USB VBUS output regulator driver
//
// Copyright (C) 2024 Renesas Electronics Corporation
//

#include <linux/module.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

static const struct regulator_ops rzg2l_usb_vbus_reg_ops = {
	.enable     = regulator_enable_regmap,
	.disable    = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct regulator_desc rzg2l_usb_vbus_rdesc = {
	.name = "vbus",
	.of_match = of_match_ptr("regulator-vbus"),
	.ops = &rzg2l_usb_vbus_reg_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
	.enable_reg  = 0,
	.enable_mask = BIT(0),
	.enable_is_inverted = true,
	.fixed_uV	= 5000000,
	.n_voltages	= 1,
};

static int rzg2l_usb_vbus_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;

	config.regmap = dev_get_regmap(dev->parent, NULL);
	if (!config.regmap)
		return dev_err_probe(dev, -ENOENT, "Failed to get regmap\n");

	config.dev = dev;
	config.of_node = of_get_child_by_name(dev->parent->of_node, "regulator-vbus");
	if (!config.of_node)
		return dev_err_probe(dev, -ENODEV, "regulator node not found\n");

	rdev = devm_regulator_register(dev, &rzg2l_usb_vbus_rdesc, &config);
	of_node_put(config.of_node);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev),
				     "not able to register vbus regulator\n");

	return 0;
}

#define RZG3L_USB_VBUS_REG(rname, en_mask)				\
	{								\
		.name			= #rname,			\
		.of_match		= of_match_ptr(#rname),		\
		.regulators_node	= of_match_ptr("regulators"),	\
		.type			= REGULATOR_VOLTAGE,		\
		.owner			= THIS_MODULE,			\
		.ops			= &rzg2l_usb_vbus_reg_ops,	\
		.enable_reg		= 0,				\
		.enable_mask		= (en_mask),			\
		.enable_is_inverted	= true,				\
		.fixed_uV		= 5000000,			\
		.n_voltages		= 1,				\
	}

static const struct regulator_desc rzg3l_usb_vbus_regulators[] = {
	RZG3L_USB_VBUS_REG(vbus0, BIT(0)),
	RZG3L_USB_VBUS_REG(vbus1, BIT(1)),
};

static int rzg3l_usb_vbus_regulator_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	struct device *dev = &pdev->dev;
	struct regulator_dev *rdev;

	config.dev = pdev->dev.parent;
	config.regmap = dev_get_regmap(dev->parent, NULL);
	if (!config.regmap)
		return dev_err_probe(dev, -ENOENT, "Failed to get regmap\n");

	for (unsigned int i = 0; i < ARRAY_SIZE(rzg3l_usb_vbus_regulators); i++) {
		rdev = devm_regulator_register(dev, &rzg3l_usb_vbus_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s regulator\n",
					     rzg3l_usb_vbus_regulators[i].name);
	}

	return 0;
}

static int rzg2l_usb_vbus_regulator_common_probe(struct platform_device *pdev)
{
	int (*probe_func)(struct platform_device *pdev);
	const struct platform_device_id *id;

	id = platform_get_device_id(pdev);
	if (!id)
		return dev_err_probe(&pdev->dev, -ENODEV, "No ID match found\n");

	probe_func = (int (*)(struct platform_device *))id->driver_data;

	return probe_func(pdev);
}

static const struct platform_device_id rzg2l_usb_vbus_regulator_ids[] = {
	{
		.name = "rzg2l-vbus-regulator",
		.driver_data = (kernel_ulong_t)rzg2l_usb_vbus_regulator_probe
	},
	{
		.name = "rzg3l-vbus-regulator",
		.driver_data = (kernel_ulong_t)rzg3l_usb_vbus_regulator_probe
	},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(platform, rzg2l_usb_vbus_regulator_ids);

static struct platform_driver rzg2l_usb_vbus_regulator_driver = {
	.probe = rzg2l_usb_vbus_regulator_common_probe,
	.id_table = rzg2l_usb_vbus_regulator_ids,
	.driver	= {
		.name = "rzg2l-vbus-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_platform_driver(rzg2l_usb_vbus_regulator_driver);

MODULE_AUTHOR("Biju Das <biju.das.jz@bp.renesas.com>");
MODULE_DESCRIPTION("Renesas RZ/G2L USB Vbus Regulator Driver");
MODULE_LICENSE("GPL");
