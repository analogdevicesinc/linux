// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 Analog Devices, Inc.
 * ADI driver for the MAX77643/54/58/59
 */

#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77658.h>
#include <linux/mod_devicetable.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

static const struct regmap_config max77658_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
};

enum max77658_device_types {
	MAX77658_REGULATOR,
	MAX77658_CHARGER,
	MAX77658_BATTERY,
	NUM_DEVICE_TYPES,
};

static const struct regmap_irq max77643_glbl0_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL0_GPIO0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL0_GPIO0_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_EN_F, },
	{ .mask = MAX77658_BIT_INT_GLBL0_EN_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_TJAL1_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_TJAL2_R, },
	{ .mask = MAX77643_BIT_INT_GLBL0_DOD0_R, },
};

static const struct regmap_irq_chip max77643_glbl0_irq_chip = {
	.name           = "max77643_glbl0",
	.status_base    = MAX77658_REG_INT_GLBL0,
	.mask_base      = MAX77643_REG_INTM_GLBL0,
	.num_regs       = 1,
	.irqs           = max77643_glbl0_irqs,
	.num_irqs       = ARRAY_SIZE(max77643_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77658_glbl0_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL0_GPIO0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL0_GPIO0_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_EN_F, },
	{ .mask = MAX77658_BIT_INT_GLBL0_EN_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_TJAL1_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_TJAL2_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_DOD1_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_DOD0_R, },
};

static const struct regmap_irq_chip max77654_glbl0_irq_chip = {
	.name           = "max77654_glbl0",
	.status_base    = MAX77658_REG_INT_GLBL0,
	.mask_base      = MAX77654_REG_INTM_GLBL0,
	.num_regs       = 1,
	.irqs           = max77658_glbl0_irqs,
	.num_irqs       = ARRAY_SIZE(max77658_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq_chip max77658_glbl0_irq_chip = {
	.name           = "max77658_glbl0",
	.status_base    = MAX77658_REG_INT_GLBL0,
	.mask_base      = MAX77658_REG_INTM_GLBL0,
	.num_regs       = 1,
	.irqs           = max77658_glbl0_irqs,
	.num_irqs       = ARRAY_SIZE(max77658_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77659_glbl0_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL0_GPIO0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL0_GPIO0_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_EN_F, },
	{ .mask = MAX77658_BIT_INT_GLBL0_EN_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_TJAL1_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_TJAL2_R, },
	{ .mask = MAX77658_BIT_INT_GLBL0_DOD0_R, },
};

static const struct regmap_irq_chip max77659_glbl0_irq_chip = {
	.name           = "max77659_glbl0",
	.status_base    = MAX77658_REG_INT_GLBL0,
	.mask_base      = MAX77654_REG_INTM_GLBL0,
	.num_regs       = 1,
	.irqs           = max77659_glbl0_irqs,
	.num_irqs       = ARRAY_SIZE(max77659_glbl0_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77643_glbl1_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_R, },
	{ .mask = MAX77658_BIT_INT_GLBL1_SBB0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_SBB1_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_SBB2_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_LDO0_F, },
};

static const struct regmap_irq_chip max77643_glbl1_irq_chip = {
	.name           = "max77643_glbl1",
	.status_base    = MAX77658_REG_INT_GLBL1,
	.mask_base      = MAX77643_REG_INTM_GLBL1,
	.num_regs       = 1,
	.irqs           = max77643_glbl1_irqs,
	.num_irqs       = ARRAY_SIZE(max77643_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77654_glbl1_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_R, },
	{ .mask = MAX77654_BIT_INT_GLBL1_GPI2_F, },
	{ .mask = MAX77654_BIT_INT_GLBL1_GPI2_R, },
	{ .mask = MAX77654_BIT_INT_GLBL1_SBB_TO, },
	{ .mask = MAX77658_BIT_INT_GLBL1_LDO0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_LDO1_F, },
};

static const struct regmap_irq_chip max77654_glbl1_irq_chip = {
	.name           = "max77654_glbl1",
	.status_base    = MAX77658_REG_INT_GLBL1,
	.mask_base      = MAX77654_REG_INTM_GLBL1,
	.num_regs       = 1,
	.irqs           = max77654_glbl1_irqs,
	.num_irqs       = ARRAY_SIZE(max77654_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77658_glbl1_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_R, },
	{ .mask = MAX77658_BIT_INT_GLBL1_SBB0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_SBB1_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_SBB2_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_LDO0_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_LDO1_F, },
};

static const struct regmap_irq_chip max77658_glbl1_irq_chip = {
	.name           = "max77658_glbl1",
	.status_base    = MAX77658_REG_INT_GLBL1,
	.mask_base      = MAX77658_REG_INTM_GLBL1,
	.num_regs       = 1,
	.irqs           = max77658_glbl1_irqs,
	.num_irqs       = ARRAY_SIZE(max77658_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77659_glbl1_irqs[] = {
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_F, },
	{ .mask = MAX77658_BIT_INT_GLBL1_GPI1_R, },
	{ .mask = MAX77659_BIT_INT_GLBL1_SBB_TO, },
	{ .mask = MAX77658_BIT_INT_GLBL1_LDO0_F, },
};

static const struct regmap_irq_chip max77659_glbl1_irq_chip = {
	.name           = "max77659_glbl1",
	.status_base    = MAX77658_REG_INT_GLBL1,
	.mask_base      = MAX77658_REG_INTM_GLBL1,
	.num_regs       = 1,
	.irqs           = max77659_glbl1_irqs,
	.num_irqs       = ARRAY_SIZE(max77659_glbl1_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77658_chg_irqs[] = {
	{ .mask = MAX77658_BIT_INT_THM, },
	{ .mask = MAX77658_BIT_INT_CHG, },
	{ .mask = MAX77658_BIT_INT_CHGIN, },
	{ .mask = MAX77658_BIT_INT_TJ_REG, },
	{ .mask = MAX77658_BIT_INT_CHGIN_CTRL, },
	{ .mask = MAX77658_BIT_INT_SYS_CTRL, },
	{ .mask = MAX77658_BIT_INT_SYS_CNFG, },
};

static const struct regmap_irq_chip max77654_chg_irq_chip = {
	.name           = "max77654_chg",
	.status_base    = MAX77658_REG_INT_CHG,
	.mask_base      = MAX77658_REG_INTM_CHG,
	.num_regs       = 1,
	.irqs           = max77658_chg_irqs,
	.num_irqs       = ARRAY_SIZE(max77658_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq_chip max77658_chg_irq_chip = {
	.name           = "max77658_chg",
	.status_base    = MAX77658_REG_INT_CHG,
	.mask_base      = MAX77658_REG_INTM_CHG,
	.num_regs       = 1,
	.irqs           = max77658_chg_irqs,
	.num_irqs       = ARRAY_SIZE(max77658_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct regmap_irq max77659_chg_irqs[] = {
	{ .mask = MAX77658_BIT_INT_THM, },
	{ .mask = MAX77658_BIT_INT_CHG, },
	{ .mask = MAX77658_BIT_INT_CHGIN, },
	{ .mask = MAX77658_BIT_INT_TJ_REG, },
	{ .mask = MAX77659_BIT_INT_SYS_CTRL, },
};

static const struct regmap_irq_chip max77659_chg_irq_chip = {
	.name           = "max77659_chg",
	.status_base    = MAX77658_REG_INT_CHG,
	.mask_base      = MAX77658_REG_INTM_CHG,
	.num_regs       = 1,
	.irqs           = max77659_chg_irqs,
	.num_irqs       = ARRAY_SIZE(max77659_chg_irqs),
	.type_in_mask		= true,
	.init_ack_masked	= true,
	.clear_on_unmask	= true,
};

static const struct resource max77658_charger_resources[] = {
	DEFINE_RES_IRQ_NAMED(1, "CHG"),
	DEFINE_RES_IRQ_NAMED(2, "CHGIN"),
};

static const struct mfd_cell max77643_devs[] = {
	[MAX77658_REGULATOR] = {
		.name = "max77658-regulator",
	},
};

static const struct mfd_cell max77654_devs[] = {
	[MAX77658_REGULATOR] = {
		.name = "max77658-regulator",
	},
	[MAX77658_CHARGER] = {
		.name = "max77654-charger",
		.of_compatible	= "adi,max77654-charger",
		.resources	= max77658_charger_resources,
		.num_resources	= ARRAY_SIZE(max77658_charger_resources),
	},
};

static const struct mfd_cell max77658_devs[] = {
	[MAX77658_REGULATOR] = {
		.name = "max77658-regulator",
	},
	[MAX77658_CHARGER] = {
		.name = "max77658-charger",
		.of_compatible	= "adi,max77658-charger",
		.resources	= max77658_charger_resources,
		.num_resources	= ARRAY_SIZE(max77658_charger_resources),
	},
	[MAX77658_BATTERY] = {
		.name = "max77658-battery",
		.of_compatible	= "adi,max77658-battery",
	},
};

static const struct mfd_cell max77659_devs[] = {
	[MAX77658_REGULATOR] = {
		.name = "max77658-regulator",
	},
	[MAX77658_CHARGER] = {
		.name = "max77659-charger",
		.of_compatible	= "adi,max77659-charger",
		.resources	= max77658_charger_resources,
		.num_resources	= ARRAY_SIZE(max77658_charger_resources),
	},
};

static int max77658_pmic_irq_init(struct device *dev)
{
	const struct regmap_irq_chip *glbl0_chip, *glbl1_chip, *chg_chip;
	struct max77658_dev *max77658 = dev_get_drvdata(dev);
	int ret;

	switch (max77658->id) {
	case ID_MAX77643:
		glbl0_chip = &max77643_glbl0_irq_chip;
		glbl1_chip = &max77643_glbl1_irq_chip;
		break;
	case ID_MAX77654:
		glbl0_chip = &max77654_glbl0_irq_chip;
		glbl1_chip = &max77654_glbl1_irq_chip;
		chg_chip = &max77654_chg_irq_chip;
		break;
	case ID_MAX77658:
		glbl0_chip = &max77658_glbl0_irq_chip;
		glbl1_chip = &max77658_glbl1_irq_chip;
		chg_chip = &max77658_chg_irq_chip;
		break;
	case ID_MAX77659:
		glbl0_chip = &max77659_glbl0_irq_chip;
		glbl1_chip = &max77659_glbl1_irq_chip;
		chg_chip = &max77659_chg_irq_chip;
		break;
	default:
		return -EINVAL;
	}

	if (max77658->id != ID_MAX77643) {
		ret = devm_regmap_add_irq_chip(dev, max77658->regmap,
					       max77658->irq,
					       IRQF_ONESHOT | IRQF_SHARED, 0,
					       chg_chip, &max77658->irqc_chg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to add charger IRQ chip\n");
	}

	ret = devm_regmap_add_irq_chip(dev, max77658->regmap, max77658->irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       glbl0_chip, &max77658->irqc_glbl0);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add global0 IRQ chip\n");

	return devm_regmap_add_irq_chip(dev, max77658->regmap, max77658->irq,
					IRQF_ONESHOT | IRQF_SHARED, 0,
					glbl1_chip, &max77658->irqc_glbl1);
}

static int max77658_i2c_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct max77658_dev *max77658;
	const struct mfd_cell *cells;
	struct irq_domain *domain;
	int ret, n_devs, i;

	max77658 = devm_kzalloc(dev, sizeof(*max77658), GFP_KERNEL);
	if (!max77658)
		return -ENOMEM;

	i2c_set_clientdata(client, max77658);
	max77658->irq = client->irq;

	max77658->id = (enum max77658_ids)device_get_match_data(dev);
	if (!max77658->id)
		max77658->id  = (enum max77658_ids)id->driver_data;

	if (!max77658->id)
		return -EINVAL;

	max77658->regmap = devm_regmap_init_i2c(client,
						&max77658_regmap_config);
	if (IS_ERR(max77658->regmap))
		return dev_err_probe(dev, PTR_ERR(max77658->regmap),
				     "Failed to initialize regmap\n");

	ret = max77658_pmic_irq_init(dev);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to initialize IRQ\n");

	switch (max77658->id) {
	case ID_MAX77643:
		cells = max77643_devs;
		n_devs = ARRAY_SIZE(max77643_devs);
		break;
	case ID_MAX77654:
		cells = max77654_devs;
		n_devs = ARRAY_SIZE(max77654_devs);
		break;
	case ID_MAX77658:
		cells = max77658_devs;
		n_devs = ARRAY_SIZE(max77658_devs);
		break;
	case ID_MAX77659:
		cells = max77659_devs;
		n_devs = ARRAY_SIZE(max77659_devs);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < n_devs; i++) {
		if (i == MAX77658_CHARGER)
			domain = regmap_irq_get_domain(max77658->irqc_chg);
		else
			domain = NULL;

		ret = devm_mfd_add_devices(dev, -1, cells + i, 1,
					   NULL, 0, domain);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to add sub-device\n");
	}

	return device_init_wakeup(dev, true);
}

static const struct of_device_id max77658_of_id[] = {
	{ .compatible = "adi,max77643", .data = (void *)ID_MAX77643 },
	{ .compatible = "adi,max77654", .data = (void *)ID_MAX77654 },
	{ .compatible = "adi,max77658", .data = (void *)ID_MAX77658 },
	{ .compatible = "adi,max77659", .data = (void *)ID_MAX77659 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77658_of_id);

static const struct i2c_device_id max77658_i2c_id[] = {
	{ "max77643", ID_MAX77643 },
	{ "max77654", ID_MAX77654 },
	{ "max77658", ID_MAX77658 },
	{ "max77659", ID_MAX77659 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max77658_i2c_id);

static struct i2c_driver max77658_driver = {
	.driver = {
		.name = "max77658",
		.of_match_table = max77658_of_id,
	},
	.probe = max77658_i2c_probe,
	.id_table = max77658_i2c_id,
};
module_i2c_driver(max77658_driver);

MODULE_DESCRIPTION("MAX77658 MFD Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com, Zeynep.Arslanbenzer@analog.com");
MODULE_LICENSE("GPL");
