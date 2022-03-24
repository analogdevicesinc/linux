// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77658.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/version.h>

/* PMIC (CLOGIC/SAFELDOs/CHARGER) */
#define I2C_ADDR_PMIC			(0x90 >> 1)
#define I2C_ADDR_FUEL_GAUGE		(0x6C >> 1) /* Fuel Gauge */

static const struct regmap_config max77658_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regmap_config max77658_regmap_config_fuelgauge = {
	.reg_bits = 8,
	.val_bits = 16,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

/* Declare Interrupt */
/* Global 0 Interrupt */
static const struct regmap_irq max77658_glbl0_irqs[] = {
	{
		/* GPI Falling Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_GPIO0_F,
	},
	{
		/* GPI Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_GPIO0_R,
	},
	{
		/* nEN Falling Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_EN_F,
	},
	{
		/* nEN Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_EN_R,
	},
	{
		/* Thermal Alarm 1 Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_TJAL1_R,
	},
	{
		/* Thermal Alarm 2 Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_TJAL2_R,
	},
	{
		/* LDO Dropout Detector Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_DOD1_R,
	},
	{
		/* LDO Dropout Detector Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL0_DOD0_R,
	},
};

static const struct regmap_irq_chip max77658_glbl0_irq_chip = {
	.name = "max77658 glbl0",
	.status_base = REG_INT_GLBL0,
	.mask_base = REG_INTM_GLBL0,
	.num_regs = 1,
	.irqs = max77658_glbl0_irqs,
	.num_irqs = ARRAY_SIZE(max77658_glbl0_irqs),
};

/* Global 1 Interrupt */
static const struct regmap_irq max77658_glbl1_irqs[] = {
	{
		/* GPI Falling Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_GPIO1_F,
	},
	{
		/* GPI Rising Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_GPIO1_R,
	},
	{
		/* SBB0 Fault Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_SBB0_F,
	},
	{
		/* SBB1 Fault Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_SBB1_F,
	},
	{
		/* SBB2 Fault Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_SBB2_F,
	},
	{
		/* LDO0 Fault Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_LDO0_F,
	},
	{
		/* LDO1 Fault Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_GLBL1_LDO1_F,
	},
};

static const struct regmap_irq_chip max77658_glbl1_irq_chip = {
	.name = "max77658 glbl1",
	.status_base = REG_INT_GLBL1,
	.mask_base = REG_INTM_GLBL1,
	.num_regs = 1,
	.irqs = max77658_glbl1_irqs,
	.num_irqs = ARRAY_SIZE(max77658_glbl1_irqs),
};

/* Charger Interrupt */
static const struct regmap_irq max77658_chg_irqs[] = {
	{
		/* Thermistor Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_THM,
	},
	{
		/* Charger Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_CHG,
	},
	{
		/* CHGIN Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_CHGIN,
	},
	{
		/* Die Junction Temperature Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_TJ_REG,
	},
	{
		/* CHGIN Control-Loop Interrupt */
		.reg_offset = 0,
		.mask = BIT_INT_CHGIN_CTRL,
	},
	{
		/* Min System Voltage Regulation-Loop */
		.reg_offset = 0,
		.mask = BIT_INT_SYS_CTRL,
	},
	{
		/* System Voltage Configuration Error */
		.reg_offset = 0,
		.mask = BIT_INT_SYS_CNFG,
	},
};

static const struct regmap_irq_chip max77658_chg_irq_chip = {
	.name = "max77658 chg",
	.status_base = REG_INT_CHG,
	.mask_base = REG_INT_M_CHG,
	.num_regs = 1,
	.irqs = max77658_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77658_chg_irqs),
};

static int max77658_pmic_irq_int(struct max77658_dev *me)
{
	struct device *dev = me->dev;
	int ret = 0;

	/* disable all interrupt source */
	ret = regmap_write(me->regmap_pmic, REG_INTM_GLBL0, 0xFF);
	if (ret)
		return dev_err_probe(dev, ret,
			"Unable to write Global0 Interrupt Masking register\n");

	ret = regmap_write(me->regmap_pmic, REG_INTM_GLBL1, 0xFF);
	if (ret)
		return dev_err_probe(dev, ret,
			"Unable to write Global1 Interrupt Masking register\n");

	/* interrupt global 0 */
	ret = devm_regmap_add_irq_chip(dev, me->regmap_pmic, me->irq,
				       IRQF_ONESHOT | IRQF_SHARED, -1,
				       &max77658_glbl0_irq_chip,
				       &me->irqc_glbl0);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add global0 irq chip\n");

	/* interrupt global 1 */
	ret = devm_regmap_add_irq_chip(dev, me->regmap_pmic, me->irq,
				       IRQF_ONESHOT | IRQF_SHARED, -1,
				       &max77658_glbl1_irq_chip,
				       &me->irqc_glbl1);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add global1 irq chip\n");

	/* charger interrupt */
	return devm_regmap_add_irq_chip(dev, me->regmap_pmic, me->irq,
					IRQF_ONESHOT | IRQF_SHARED, -1,
					&max77658_chg_irq_chip,
					&me->irqc_chg);
}

static struct mfd_cell max77658_devices[] = {
	{ .name = MAX77658_CHARGER_NAME, },
	{ .name = MAX77658_FUELGAUGE_NAME, },
	{ .name = MAX77658_REGULATOR_NAME, },
};

static int max77658_pmic_setup(struct max77658_dev *me)
{
	struct device *dev = me->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;
	unsigned int chip_id, val;

	/* IRQ init */
	me->irq = client->irq;

	ret = max77658_pmic_irq_int(me);
	if (ret != 0)
		return dev_err_probe(dev, ret, "failed to initialize irq\n");

	chip_id = 0;
	ret = regmap_read(me->regmap_pmic, REG_CID,  &chip_id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip id\n");

	dev_dbg(dev, "Chip Identification Code : 0x%Xh\n", chip_id);

	/* clear IRQ */
	ret = regmap_read(me->regmap_pmic, REG_INT_GLBL0, &val);
	if (ret)
		return dev_err_probe(dev, ret,
			"Unable to read Global0 Interrupt Status register\n");

	ret = regmap_read(me->regmap_pmic, REG_INT_GLBL1, &val);
	if (ret)
		return dev_err_probe(dev, ret,
			"Unable to read Global1 Interrupt Status register\n");

	ret = regmap_read(me->regmap_pmic, REG_INT_CHG, &val);
	if (ret)
		return dev_err_probe(dev, ret,
			"Unable to read Charger Interrupt Status register\n");

	/* set device able to wake up system */
	ret = device_init_wakeup(dev, true);
	if (ret)
		return dev_err_probe(dev, ret, "Unable to init wakeup\n");

	ret = devm_mfd_add_devices(dev, 0, max77658_devices,
				   ARRAY_SIZE(max77658_devices), NULL, 0, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add sub-devices\n");

	if (me->irq > 0)
		enable_irq_wake(me->irq);

	return 0;
}

static int max77658_pre_init_data(struct max77658_dev *pmic)
{
	int size, cnt;
	struct device *dev = pmic->dev;
	u8 *init_data;

	size = device_property_count_u8(dev, "max77658,pmic-init");
	if (size < 0) {
		dev_warn(dev, "No max77658,pmic-init property\n");
	} else {
		init_data = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!init_data)
			return dev_err_probe(dev, -ENOMEM, "Failed to allocate memory\n");

		device_property_read_u8_array(dev, "max77658,pmic-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			regmap_write(pmic->regmap_pmic,
				     (unsigned int)init_data[cnt],
				     (unsigned int)init_data[cnt + 1]);
		}
	}

	size = device_property_count_u8(dev, "max77658,fg-init");
	if (size) {
		dev_warn(dev, "No max77658,fg-init property\n");
	} else {
		init_data = devm_kzalloc(dev, size, GFP_KERNEL);
		if (!init_data)
			return dev_err_probe(dev, -ENOMEM, "Failed to allocate memory\n");

		device_property_read_u8_array(dev, "max77658,fg-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 3) {
			regmap_write(pmic->regmap_fuel,
				     (unsigned int)init_data[cnt],
				     (unsigned int)((init_data[cnt + 1] << 8) | init_data[cnt + 2]));
		}
	}

	return 0;
}

static int max77658_i2c_probe(struct i2c_client *client)
{
	struct max77658_dev *me;
	int ret;

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (!me)
		return dev_err_probe(&client->dev, -ENOMEM, "Failed to allocate memory\n");

	i2c_set_clientdata(client, me);
	me->dev = &client->dev;
	me->pmic = client;

	me->regmap_pmic = devm_regmap_init_i2c(client, &max77658_regmap_config);
	if (IS_ERR(me->regmap_pmic))
		return dev_err_probe(me->dev,
				     PTR_ERR(me->regmap_pmic),
				     "failed to initialize i2c\n");

	me->fuel = i2c_new_dummy_device(client->adapter, I2C_ADDR_FUEL_GAUGE);
	if (IS_ERR(me->fuel))
		return dev_err_probe(me->dev, PTR_ERR(me->fuel),
				     "failed add i2c device[0x%Xh]\n", I2C_ADDR_FUEL_GAUGE);

	i2c_set_clientdata(me->fuel, me);
	me->regmap_fuel = devm_regmap_init_i2c(me->fuel, &max77658_regmap_config_fuelgauge);
	if (IS_ERR(me->regmap_fuel))
		return dev_err_probe(me->dev, PTR_ERR(me->regmap_fuel),
				     "failed to initialize i2c device[0x%Xh]\n", I2C_ADDR_FUEL_GAUGE);

	ret = max77658_pre_init_data(me);
	if (ret != 0)
		return dev_err_probe(me->dev, ret, "failed to parse device properties\n");

	return max77658_pmic_setup(me);
}

static const struct of_device_id max77658_of_id[] = {
	{ .compatible = "maxim,max77658"},
	{ },
};
MODULE_DEVICE_TABLE(of, max77658_of_id);

static const struct i2c_device_id max77658_i2c_id[] = {
	{ MAX77658_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77658_i2c_id);

static struct i2c_driver max77658_i2c_driver = {
	.driver = {
		.name = MAX77658_NAME,
		.of_match_table = max77658_of_id,
	},
	.id_table = max77658_i2c_id,
	.probe_new = max77658_i2c_probe,
};

module_i2c_driver(max77658_i2c_driver);

MODULE_DESCRIPTION("max77658 MFD Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
