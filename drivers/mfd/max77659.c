// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77659.h>
#include <linux/regmap.h>

static const struct regmap_config max77659_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
};

static const struct regmap_irq max77659_glbl0_irqs[] = {
	{ .mask = MAX77659_BIT_INT_GLBL0_GPIO0_F, },
	{ .mask = MAX77659_BIT_INT_GLBL0_GPIO0_R, },
	{ .mask = MAX77659_BIT_INT_GLBL0_EN_F, },
	{ .mask = MAX77659_BIT_INT_GLBL0_EN_R, },
	{ .mask = MAX77659_BIT_INT_GLBL0_TJAL1_R, },
	{ .mask = MAX77659_BIT_INT_GLBL0_TJAL2_R, },
	{ .mask = MAX77659_BIT_INT_GLBL0_DOD0_R, },
};

static const struct regmap_irq_chip max77659_glbl0_irq_chip = {
	.name           = "max77659_glbl0",
	.status_base    = MAX77659_REG_INT_GLBL0,
	.mask_base      = MAX77659_REG_INTM_GLBL0,
	.num_regs       = 1,
	.irqs           = max77659_glbl0_irqs,
	.num_irqs       = ARRAY_SIZE(max77659_glbl0_irqs),
};

static const struct regmap_irq max77659_glbl1_irqs[] = {
	{ .mask = MAX77659_BIT_INT_GLBL1_GPI1_F, },
	{ .mask = MAX77659_BIT_INT_GLBL1_GPI1_R, },
	{ .mask = MAX77659_BIT_INT_GLBL1_SBB_TO, },
	{ .mask = MAX77659_BIT_INT_GLBL1_LDO0_F, },
};

static const struct regmap_irq_chip max77659_glbl1_irq_chip = {
	.name           = "max77659_glbl1",
	.status_base    = MAX77659_REG_INT_GLBL1,
	.mask_base      = MAX77659_REG_INTM_GLBL1,
	.num_regs       = 1,
	.irqs           = max77659_glbl1_irqs,
	.num_irqs       = ARRAY_SIZE(max77659_glbl1_irqs),
};

static const struct regmap_irq max77659_chg_irqs[] = {
	{ .mask = MAX77659_BIT_INT_THM, },
	{ .mask = MAX77659_BIT_INT_CHG, },
	{ .mask = MAX77659_BIT_INT_CHGIN, },
	{ .mask = MAX77659_BIT_INT_TJ_REG, },
	{ .mask = MAX77659_BIT_INT_SYS_CTRL, },
};

static const struct regmap_irq_chip max77659_chg_irq_chip = {
	.name = "max77659_chg",
	.status_base = MAX77659_REG_INT_CHG,
	.mask_base = MAX77659_REG_INT_M_CHG,
	.num_regs = 1,
	.irqs = max77659_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77659_chg_irqs),
};

static const struct mfd_cell max77659_devs[] = {
	{ .name = MAX77659_REGULATOR_NAME, },
	{ .name = MAX77659_CHARGER_NAME, .of_compatible = "adi,max77659-charger", },
};

static int max77659_pmic_irq_init(struct max77659_dev *me)
{
	struct device *dev = me->dev;
	int ret;

	ret = regmap_write(me->regmap, MAX77659_REG_INTM_GLBL0, MAX77659_INT_GLBL0_MASK);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Unable to write Global0 Interrupt Masking register\n");
	}

	ret = regmap_write(me->regmap, MAX77659_REG_INTM_GLBL1, MAX77659_INT_GLBL1_MASK);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Unable to write Global1 Interrupt Masking register\n");
	}

	ret = devm_regmap_add_irq_chip(dev, me->regmap, me->irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &max77659_glbl0_irq_chip, &me->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add global0 IRQ chip\n");

	ret = devm_regmap_add_irq_chip(dev, me->regmap, me->irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &max77659_glbl1_irq_chip, &me->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add global1 IRQ chip\n");

	return devm_regmap_add_irq_chip(dev, me->regmap, me->irq,
					IRQF_ONESHOT | IRQF_SHARED, 0,
					&max77659_chg_irq_chip, &me->irqc_chg);
}

static int max77659_pmic_setup(struct max77659_dev *me)
{
	struct device *dev = me->dev;
	struct i2c_client *client = me->i2c;
	unsigned int val;
	int ret;

	if (client->irq <= 0)
		return dev_err_probe(dev, -EINVAL, "Invalid IRQ\n");

	me->irq = client->irq;

	ret = max77659_pmic_irq_init(me);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to initialize IRQ\n");

	ret = regmap_read(me->regmap, MAX77659_REG_INT_GLBL0, &val);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Unable to read Global0 Interrupt Status register\n");
	}

	ret = regmap_read(me->regmap, MAX77659_REG_INT_GLBL1, &val);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Unable to read Global1 Interrupt Status register\n");
	}

	ret = regmap_read(me->regmap, MAX77659_REG_INT_CHG, &val);
	if (ret) {
		return dev_err_probe(dev, ret,
				     "Unable to read Charger Interrupt Status register\n");
	}

	ret = device_init_wakeup(dev, true);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to init wakeup\n");

	ret = devm_mfd_add_devices(dev, -1, max77659_devs, ARRAY_SIZE(max77659_devs),
				   NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add sub-devices\n");

	if (me->irq > 0)
		enable_irq_wake(me->irq);

	return 0;
}

static int max77659_i2c_probe(struct i2c_client *client)
{
	struct max77659_dev *me;

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (!me)
		return -ENOMEM;

	i2c_set_clientdata(client, me);
	me->dev = &client->dev;
	me->i2c = client;

	me->regmap = devm_regmap_init_i2c(client, &max77659_regmap_config);
	if (IS_ERR(me->regmap)) {
		return dev_err_probe(&client->dev, PTR_ERR(me->regmap),
				     "Failed to allocate register map\n");
	}

	return max77659_pmic_setup(me);
}

static const struct of_device_id max77659_of_id[] = {
	{ .compatible = "adi,max77659" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77659_of_id);

static const struct i2c_device_id max77659_i2c_id[] = {
	{ MAX77659_NAME, 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max77659_i2c_id);

static struct i2c_driver max77659_i2c_driver = {
	.driver = {
		.name = MAX77659_NAME,
		.of_match_table = of_match_ptr(max77659_of_id),
	},
	.probe_new = max77659_i2c_probe,
	.id_table = max77659_i2c_id,
};

module_i2c_driver(max77659_i2c_driver);

MODULE_DESCRIPTION("max77659 MFD Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com, Zeynep.Arslanbenzer@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
