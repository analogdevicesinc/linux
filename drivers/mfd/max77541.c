// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 Analog Devices, Inc.
 * Mfd core driver for the MAX77540 and MAX77541
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77541.h>
#include <linux/regmap.h>

static const struct regmap_config max77541_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
};

static const struct regmap_irq max77541_src_irqs[] = {
	{ .mask = MAX77541_BIT_INT_SRC_TOPSYS, },
	{ .mask = MAX77541_BIT_INT_SRC_BUCK, },
};

static const struct regmap_irq_chip max77541_src_irq_chip = {
	.name		= "max77541-src",
	.status_base	= MAX77541_REG_INT_SRC,
	.mask_base	= MAX77541_REG_INT_SRC,
	.num_regs	= 1,
	.irqs		= max77541_src_irqs,
	.num_irqs       = ARRAY_SIZE(max77541_src_irqs),
};

static const struct regmap_irq max77541_topsys_irqs[] = {
	{ .mask = MAX77541_BIT_TOPSYS_INT_TJ_120C, },
	{ .mask = MAX77541_BIT_TOPSYS_INT_TJ_140C, },
	{ .mask = MAX77541_BIT_TOPSYS_INT_TSHDN, },
	{ .mask = MAX77541_BIT_TOPSYS_INT_UVLO, },
	{ .mask = MAX77541_BIT_TOPSYS_INT_ALT_SWO, },
	{ .mask = MAX77541_BIT_TOPSYS_INT_EXT_FREQ_DET, },
};

static const struct regmap_irq_chip max77541_topsys_irq_chip = {
	.name		= "max77541-topsys",
	.status_base	= MAX77541_REG_TOPSYS_INT,
	.mask_base	= MAX77541_REG_TOPSYS_INT_M,
	.num_regs	= 1,
	.irqs		= max77541_topsys_irqs,
	.num_irqs	= ARRAY_SIZE(max77541_topsys_irqs),
};

static const struct regmap_irq max77541_buck_irqs[] = {
	{ .mask = MAX77541_BIT_BUCK_INT_M1_POK_FLT, },
	{ .mask = MAX77541_BIT_BUCK_INT_M2_POK_FLT, },
	{ .mask = MAX77541_BIT_BUCK_INT_M1_SCFLT, },
	{ .mask = MAX77541_BIT_BUCK_INT_M2_SCFLT, },
};

static const struct regmap_irq_chip max77541_buck_irq_chip = {
	.name		= "max77541-buck",
	.status_base	= MAX77541_REG_BUCK_INT,
	.mask_base	= MAX77541_REG_BUCK_INT_M,
	.num_regs	= 1,
	.irqs		= max77541_buck_irqs,
	.num_irqs	= ARRAY_SIZE(max77541_buck_irqs),
};

static const struct regmap_irq max77541_adc_irqs[] = {
	{ .mask = MAX77541_BIT_ADC_INT_CH1_I, },
	{ .mask = MAX77541_BIT_ADC_INT_CH2_I, },
	{ .mask = MAX77541_BIT_ADC_INT_CH3_I, },
	{ .mask = MAX77541_BIT_ADC_INT_CH6_I, },
};

static const struct regmap_irq_chip max77541_adc_irq_chip = {
	.name		= "max77541-adc",
	.status_base	= MAX77541_REG_ADC_INT,
	.mask_base	= MAX77541_REG_ADC_MSK,
	.num_regs	= 1,
	.irqs		= max77541_adc_irqs,
	.num_irqs	= ARRAY_SIZE(max77541_adc_irqs),
};

static const struct mfd_cell max77540_devs[] = {
	{ .name = "max77540-regulator", .of_compatible = "adi,max77540-regulator"},
};

static const struct mfd_cell max77541_devs[] = {
	{ .name = "max77541-regulator", .of_compatible = "adi,max77541-regulator"},
	{ .name = "max77541-adc", .of_compatible = "adi,max77541-adc" },
};

static int max77541_pmic_irq_init(struct max77541_dev *me)
{
	struct device *dev = me->dev;
	struct regmap *regmap = me->regmap;
	int irq = me->i2c->irq;
	int ret;

	ret = devm_regmap_add_irq_chip(dev, regmap, irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &max77541_src_irq_chip, &me->irq_data);
	if (ret)
		return ret;

	ret = devm_regmap_add_irq_chip(dev, regmap, irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &max77541_topsys_irq_chip, &me->irq_topsys);
	if (ret)
		return ret;

	ret = devm_regmap_add_irq_chip(dev, regmap, irq,
				       IRQF_ONESHOT | IRQF_SHARED, 0,
				       &max77541_buck_irq_chip, &me->irq_buck);
	if (ret)
		return ret;

	if (me->type == MAX77541) {
		ret = devm_regmap_add_irq_chip(dev, regmap, irq,
					       IRQF_ONESHOT | IRQF_SHARED, 0,
					       &max77541_adc_irq_chip,
					       &me->irq_adc);
		if (ret)
			return ret;
	}

	return ret;
}

static int max77541_pmic_setup(struct max77541_dev *me)
{
	struct device *dev = me->dev;
	struct regmap *regmap = me->regmap;
	unsigned int val;
	int ret;

	ret = max77541_pmic_irq_init(me);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize IRQ\n");

	ret = regmap_read(regmap, MAX77541_REG_INT_SRC, &val);
	if (ret)
		return ret;

	ret = regmap_read(regmap, MAX77541_REG_TOPSYS_INT, &val);
	if (ret)
		return ret;

	ret = regmap_read(regmap, MAX77541_REG_BUCK_INT, &val);
	if (ret)
		return ret;

	ret = device_init_wakeup(dev, true);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to init wakeup\n");

	switch (me->type) {
	case MAX77540:
		return devm_mfd_add_devices(dev, -1, max77540_devs, ARRAY_SIZE(max77540_devs),
					    NULL, 0, NULL);
	case MAX77541:
		return devm_mfd_add_devices(dev, -1, max77541_devs, ARRAY_SIZE(max77541_devs),
					    NULL, 0, NULL);
	default:
		return -EINVAL;
	}
}

static int max77541_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *chip_id;
	struct max77541_dev *me;
	const void *chip_data;

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (!me)
		return -ENOMEM;

	i2c_set_clientdata(client, me);
	me->dev = &client->dev;
	me->i2c = client;

	chip_id = to_i2c_driver(client->dev.driver)->id_table;
	if (!chip_id)
		return -ENODEV;

	chip_data = device_get_match_data(me->dev);
	if (!chip_data) {
		chip_data = (void *)i2c_match_id(chip_id, client)->driver_data;
		if (!chip_data)
			return dev_err_probe(me->dev, -ENODEV, "Unable to find device\n");
	}
	me->type = (enum dev_type)chip_data;
	me->regmap = devm_regmap_init_i2c(client, &max77541_regmap_config);

	if (IS_ERR(me->regmap))
		return dev_err_probe(me->dev,  PTR_ERR(me->regmap),
					"Failed to allocate register map\n");

	return max77541_pmic_setup(me);
}

static const struct of_device_id max77541_of_id[] = {
	{
		.compatible = "adi,max77540",
		.data = (void *)MAX77540,
	},
	{
		.compatible = "adi,max77541",
		.data = (void *)MAX77541,
	},
	{ /* sentinel */  }
};
MODULE_DEVICE_TABLE(of, max77541_of_id);

static const struct i2c_device_id max77541_i2c_id[] = {
	{ "max77540", MAX77540 },
	{ "max77541", MAX77541 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, max77541_i2c_id);

static struct i2c_driver max77541_i2c_driver = {
	.driver = {
		.name = "max77541",
		.of_match_table = max77541_of_id,
	},
	.probe_new = max77541_i2c_probe,
	.id_table = max77541_i2c_id,
};

module_i2c_driver(max77541_i2c_driver);

MODULE_DESCRIPTION("MAX7740/MAX7741 MFD Driver");
MODULE_AUTHOR("Okan Sahin <okan.sahin@analog.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
