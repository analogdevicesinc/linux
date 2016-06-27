/*
 * pf1550.c - mfd core driver for the PF1550
 *
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Robin Gong <yibin.gong@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This driver is based on max77693.c
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/pf1550.h>
#include <linux/of.h>
#include <linux/regmap.h>

static const struct mfd_cell pf1550_devs[] = {
	{
		.name = "pf1550-regulator",
		.of_compatible = "fsl,pf1550-regulator",
	},
	{
		.name = "pf1550-onkey",
		.of_compatible = "fsl,pf1550-onkey",
	},
	{
		.name = "pf1550-charger",
		.of_compatible = "fsl,pf1550-charger",
	},
};

static const struct regmap_config pf1550_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PF1550_PMIC_REG_END,
};

static const struct regmap_irq pf1550_regulator_irqs[] = {
	{ .reg_offset = 0, .mask = PMIC_IRQ_SW1_LS,		},
	{ .reg_offset = 0, .mask = PMIC_IRQ_SW2_LS,		},
	{ .reg_offset = 0, .mask = PMIC_IRQ_SW3_LS,		},

	{ .reg_offset = 3, .mask = PMIC_IRQ_SW1_HS,		},
	{ .reg_offset = 3, .mask = PMIC_IRQ_SW2_HS,		},
	{ .reg_offset = 3, .mask = PMIC_IRQ_SW3_HS,		},

	{ .reg_offset = 16, .mask = PMIC_IRQ_LDO1_FAULT,	},
	{ .reg_offset = 16, .mask = PMIC_IRQ_LDO2_FAULT,	},
	{ .reg_offset = 16, .mask = PMIC_IRQ_LDO3_FAULT,	},

	{ .reg_offset = 22, .mask = PMIC_IRQ_TEMP_110,	},
	{ .reg_offset = 22, .mask = PMIC_IRQ_TEMP_125,	},
};

static const struct regmap_irq_chip pf1550_regulator_irq_chip = {
	.name			= "pf1550-regulator",
	.status_base		= PF1550_PMIC_REG_SW_INT_STAT0,
	.mask_base		= PF1550_PMIC_REG_SW_INT_MASK0,
	.mask_invert		= false,
	.num_regs		= 23,
	.irqs			= pf1550_regulator_irqs,
	.num_irqs		= ARRAY_SIZE(pf1550_regulator_irqs),
};

static const struct regmap_irq pf1550_onkey_irqs[] = {
	{ .reg_offset = 0, .mask = ONKEY_IRQ_PUSHI,		},
	{ .reg_offset = 0, .mask = ONKEY_IRQ_1SI,		},
	{ .reg_offset = 0, .mask = ONKEY_IRQ_2SI,		},
	{ .reg_offset = 0, .mask = ONKEY_IRQ_3SI,		},
	{ .reg_offset = 0, .mask = ONKEY_IRQ_4SI,		},
	{ .reg_offset = 0, .mask = ONKEY_IRQ_8SI,		},
};

static const struct regmap_irq_chip pf1550_onkey_irq_chip = {
	.name			= "pf1550-onkey",
	.status_base		= PF1550_PMIC_REG_ONKEY_INT_STAT0,
	.ack_base		= PF1550_PMIC_REG_ONKEY_INT_STAT0,
	.mask_base		= PF1550_PMIC_REG_ONKEY_INT_MASK0,
	.mask_invert		= false,
	.use_ack                = 1,
	.init_ack_masked	= 1,
	.num_regs		= 1,
	.irqs			= pf1550_onkey_irqs,
	.num_irqs		= ARRAY_SIZE(pf1550_onkey_irqs),
};

static const struct regmap_irq pf1550_charger_irqs[] = {
	{ .reg_offset = 0, .mask = CHARG_IRQ_BAT2SOCI,		},
	{ .reg_offset = 0, .mask = CHARG_IRQ_BATI,		},
	{ .reg_offset = 0, .mask = CHARG_IRQ_CHGI,		},
	{ .reg_offset = 0, .mask = CHARG_IRQ_VBUSI,		},
	{ .reg_offset = 0, .mask = CHARG_IRQ_THMI,		},
};

static const struct regmap_irq_chip pf1550_charger_irq_chip = {
	.name			= "pf1550-charger",
	.status_base		= PF1550_CHARG_REG_CHG_INT,
	.mask_base		= PF1550_CHARG_REG_CHG_INT_MASK,
	.mask_invert		= false,
	.num_regs		= 1,
	.irqs			= pf1550_charger_irqs,
	.num_irqs		= ARRAY_SIZE(pf1550_charger_irqs),
};

int pf1550_read_otp(struct pf1550_dev *pf1550, unsigned int index,
			   unsigned int *val)
{
	int ret = 0;

	ret = regmap_write(pf1550->regmap, PF1550_PMIC_REG_KEY, 0x15);
	if (ret)
		goto read_err;
	ret = regmap_write(pf1550->regmap, PF1550_CHARG_REG_CHGR_KEY2, 0x50);
	if (ret)
		goto read_err;
	ret = regmap_write(pf1550->regmap, PF1550_TEST_REG_KEY3, 0xAB);
	if (ret)
		goto read_err;
	ret = regmap_write(pf1550->regmap, PF1550_TEST_REG_FMRADDR, index);
	if (ret)
		goto read_err;
	ret = regmap_read(pf1550->regmap, PF1550_TEST_REG_FMRDATA, val);
	if (ret)
		goto read_err;

	return 0;

read_err:
	dev_err(pf1550->dev, "read otp reg %x found!\n", index);
	return ret;
}

static int pf1550_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct pf1550_dev *pf1550;
	unsigned int reg_data = 0;
	int ret = 0;

	pf1550 = devm_kzalloc(&i2c->dev,
			sizeof(struct pf1550_dev), GFP_KERNEL);
	if (!pf1550)
		return -ENOMEM;

	i2c_set_clientdata(i2c, pf1550);
	pf1550->dev = &i2c->dev;
	pf1550->i2c = i2c;
	pf1550->irq = i2c->irq;

	pf1550->regmap = devm_regmap_init_i2c(i2c, &pf1550_regmap_config);
	if (IS_ERR(pf1550->regmap)) {
		ret = PTR_ERR(pf1550->regmap);
		dev_err(pf1550->dev, "failed to allocate register map: %d\n",
				ret);
		return ret;
	}

	ret = regmap_read(pf1550->regmap, PF1550_PMIC_REG_DEVICE_ID, &reg_data);
	if (ret < 0 || reg_data != 0x7c) {
		dev_err(pf1550->dev, "device not found!\n");
		return ret;
	}

	pf1550->type = PF1550;
	dev_info(pf1550->dev, "pf1550 found.\n");

	ret = regmap_add_irq_chip(pf1550->regmap, pf1550->irq,
				IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_FALLING, 0,
				&pf1550_regulator_irq_chip,
				&pf1550->irq_data_regulator);
	if (ret) {
		dev_err(pf1550->dev, "failed to add irq1 chip: %d\n", ret);
		goto err_regulator_irq;
	}

	ret = regmap_add_irq_chip(pf1550->regmap, pf1550->irq,
				IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_FALLING, 0,
				&pf1550_onkey_irq_chip,
				&pf1550->irq_data_onkey);
	if (ret) {
		dev_err(pf1550->dev, "failed to add irq3 chip: %d\n", ret);
		goto err_onkey_irq;
	}

	ret = regmap_add_irq_chip(pf1550->regmap, pf1550->irq,
				IRQF_ONESHOT | IRQF_SHARED |
				IRQF_TRIGGER_FALLING, 0,
				&pf1550_charger_irq_chip,
				&pf1550->irq_data_charger);
	if (ret) {
		dev_err(pf1550->dev, "failed to add irq4 chip: %d\n", ret);
		goto err_charger_irq;
	}

	ret = mfd_add_devices(pf1550->dev, -1, pf1550_devs,
			      ARRAY_SIZE(pf1550_devs), NULL, 0, NULL);
	if (ret < 0)
		goto err_mfd;

	return ret;

err_mfd:
	mfd_remove_devices(pf1550->dev);
err_charger_irq:
	regmap_del_irq_chip(pf1550->irq, pf1550->irq_data_charger);
err_onkey_irq:
	regmap_del_irq_chip(pf1550->irq, pf1550->irq_data_regulator);
err_regulator_irq:
	return ret;
}

static int pf1550_i2c_remove(struct i2c_client *i2c)
{
	struct pf1550_dev *pf1550 = i2c_get_clientdata(i2c);

	mfd_remove_devices(pf1550->dev);

	regmap_del_irq_chip(pf1550->irq, pf1550->irq_data_regulator);
	regmap_del_irq_chip(pf1550->irq, pf1550->irq_data_onkey);
	regmap_del_irq_chip(pf1550->irq, pf1550->irq_data_charger);

	return 0;
}

static const struct i2c_device_id pf1550_i2c_id[] = {
	{ "pf1550", PF1550 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pf1550_i2c_id);

static int pf1550_suspend(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct pf1550_dev *pf1550 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev)) {
		enable_irq_wake(pf1550->irq);
		disable_irq(pf1550->irq);
	}

	return 0;
}

static int pf1550_resume(struct device *dev)
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct pf1550_dev *pf1550 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev)) {
		disable_irq_wake(pf1550->irq);
		enable_irq(pf1550->irq);
	}

	return 0;
}

static const struct dev_pm_ops pf1550_pm = {
	.suspend = pf1550_suspend,
	.resume = pf1550_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id pf1550_dt_match[] = {
	{ .compatible = "fsl,pf1550" },
	{},
};
#endif

static struct i2c_driver pf1550_i2c_driver = {
	.driver = {
		   .name = "pf1550",
		   .owner = THIS_MODULE,
		   .pm = &pf1550_pm,
		   .of_match_table = of_match_ptr(pf1550_dt_match),
	},
	.probe = pf1550_i2c_probe,
	.remove = pf1550_i2c_remove,
	.id_table = pf1550_i2c_id,
};

static int __init pf1550_i2c_init(void)
{
	return i2c_add_driver(&pf1550_i2c_driver);
}
/* init early so consumer devices can complete system boot */
subsys_initcall(pf1550_i2c_init);

static void __exit pf1550_i2c_exit(void)
{
	i2c_del_driver(&pf1550_i2c_driver);
}
module_exit(pf1550_i2c_exit);

MODULE_DESCRIPTION("Freescale PF1550 multi-function core driver");
MODULE_AUTHOR("Robin Gong <yibin.gong@freescale.com>");
MODULE_LICENSE("GPL");

