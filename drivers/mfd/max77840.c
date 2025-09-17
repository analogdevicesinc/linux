// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Maxim Integrated Products, Inc.
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Author : Analog Devices <joan.na@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/string.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max77840.h>

#define DRIVER_VERSION	"1.0"

#define I2C_ADDR_PMIC           (0xCC >> 1) /* PMIC (CLOGIC/SAFELDOs) */
#define I2C_ADDR_CHARGER        (0xD2 >> 1) /* Charger */
#define I2C_ADDR_CHARGER_DETECT (0x4A >> 1) /* Charger Detect */
#define I2C_ADDR_FUEL_GAUGE     (0x6C >> 1) /* Fuel Gauge */

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

static const struct regmap_config max77840_regmap_config = {
	.reg_bits   = 8,
	.val_bits   = 8,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config max77840_regmap_config_fuelgauge = {
	.reg_bits   = 8,
	.val_bits   = 16,
	.cache_type = REGCACHE_NONE,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
};

/*******************************************************************************
 * Chip IO
 ******************************************************************************/
int max77840_read(struct regmap *regmap, u8 addr, u8 *val)
{
	unsigned int buf = 0;
	int rc;

	rc = regmap_read(regmap, (unsigned int)addr, &buf);
	if (rc < 0)
		return rc;

	*val = (u8)buf;

	return 0;
}
EXPORT_SYMBOL(max77840_read);

int max77840_write(struct regmap *regmap, u8 addr, u8 val)
{
	unsigned int buf = (unsigned int)val;

	return regmap_write(regmap, (unsigned int)addr, buf);
}
EXPORT_SYMBOL(max77840_write);

int max77840_fg_read(struct regmap *regmap, u8 addr, u16 *val)
{
	unsigned int buf = 0;
	int rc;

	rc = regmap_read(regmap, (unsigned int)addr, &buf);
	if (rc < 0)
		return rc;

	*val = (u16)buf;

	return 0;
}
EXPORT_SYMBOL(max77840_fg_read);

int max77840_fg_write(struct regmap *regmap, u8 addr, u16 val)
{
	unsigned int buf = (unsigned int)val;

	return regmap_write(regmap, (unsigned int)addr, buf);
}
EXPORT_SYMBOL(max77840_fg_write);

int max77840_bulk_read(struct regmap *regmap, u8 addr, u8 *dst, u16 len)
{
	return regmap_bulk_read(regmap, (unsigned int)addr, dst, (size_t)len);
}
EXPORT_SYMBOL(max77840_bulk_read);

int max77840_bulk_write(struct regmap *regmap, u8 addr, const u8 *src, u16 len)
{
	return regmap_bulk_write(regmap, (unsigned int)addr, src, (size_t)len);
}
EXPORT_SYMBOL(max77840_bulk_write);

/*******************************************************************************
 *  device
 ******************************************************************************/
static int max77840_add_devices(struct max77840_dev *me,
				struct mfd_cell *cells, int n_devs)
{
	struct device *dev = me->dev;
	int rc;

	rc = mfd_add_devices(dev, -1, cells, n_devs, NULL, 0, NULL);

	return rc;
}

/*******************************************************************************
 *** MAX77840 PMIC
 ******************************************************************************/

/* PMIC */
#define REG_PMICID			0x00

/* Declare Interrupt */
static const struct regmap_irq max77840_intsrc_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHGR_INT,},	/* CHGR_INT */
	{ .reg_offset = 0, .mask = BIT_SYS_INT,},	/* SYS_INT */
	{ .reg_offset = 0, .mask = BIT_FG_INT,},	/* FG_INT */
	{ .reg_offset = 0, .mask = BIT_CHGDET_INT,},	/* CHGDET_INT */
	{ .reg_offset = 0, .mask = BIT_B2SOVRC_INT,},	/* B2SOVRC_INT */
};

static const struct regmap_irq_chip max77840_intsrc_irq_chip = {
	.name = "max77840 intsrc",
	.status_base = REG_INTSRC,
	.mask_base = REG_INTSRCMASK,
	.num_regs = 1,
	.irqs = max77840_intsrc_irqs,
	.num_irqs = ARRAY_SIZE(max77840_intsrc_irqs),
};

static const struct regmap_irq max77840_sys_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_T120C_INT,},		/* T120C_INT */
	{ .reg_offset = 0, .mask = BIT_T140C_INT,},		/* T140C_INT */
	{ .reg_offset = 0, .mask = BIT_LOWSYS_INT,},	/* LOWSYS_INT */
	{ .reg_offset = 0, .mask = BIT_SYSUVLO_INT,},	/* SYSUVLO_INT */
	{ .reg_offset = 0, .mask = BIT_SYSOVLO_INT,},	/* SYSOVLO_INT */
	{ .reg_offset = 0, .mask = BIT_TSHDN_INT,},		/* TSHDN_INT */
};

static const struct regmap_irq_chip max77840_sys_irq_chip = {
	.name = "max77840 system",
	.status_base = REG_SYSINTSRC,
	.mask_base = REG_SYSINTMASK,
	.num_regs = 1,
	.irqs = max77840_sys_irqs,
	.num_irqs = ARRAY_SIZE(max77840_sys_irqs),
};

static const struct regmap_irq max77840_chgdet_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHGDET_CHGTYPE_I,},	/* CHGTYPE_I */
	{ .reg_offset = 0, .mask = BIT_CHGDET_CHGDETRUN_I,},	/* CHGDETRUN_I */
	{ .reg_offset = 0, .mask = BIT_CHGDET_DCDTMR_I,},	/* DCDTMR_I */
	{ .reg_offset = 0, .mask = BIT_CHGDET_DXOVP_I,},	/* DxOVP_I */
	{ .reg_offset = 0, .mask = BIT_CHGDET_VDNMON_I,},	/* VDCNMON_I */
};

static const struct regmap_irq_chip max77840_chgdet_irq_chip = {
	.name = "max77840 chgdet",
	.status_base = REG_CHGDET_INT,
	.mask_base = REG_CHGDET_INT_MASK,
	.num_regs = 1,
	.irqs = max77840_chgdet_irqs,
	.num_irqs = ARRAY_SIZE(max77840_chgdet_irqs),
};

static const struct regmap_irq max77840_chg_irqs[] = {
	{ .reg_offset = 0, .mask = BIT_CHG_BYP_I,},	/* BYP_I */
	{ .reg_offset = 0, .mask = BIT_CHG_BAT2SOC_I,},	/* BAT2SOC_I */
	{ .reg_offset = 0, .mask = BIT_CHG_BATP_I,},	/* BATP_I */
	{ .reg_offset = 0, .mask = BIT_CHG_BAT_I,},	/* BAT_I */
	{ .reg_offset = 0, .mask = BIT_CHG_CHG_I,},	/* CHG_I */
	{ .reg_offset = 0, .mask = BIT_CHG_TOPOFF_I,},	/* TOPOFF_I */
	{ .reg_offset = 0, .mask = BIT_CHG_CHGIN_I,},	/* CHGIN_I */
	{ .reg_offset = 0, .mask = BIT_CHG_AICL_I,},	/* AICL_I */
};

static const struct regmap_irq_chip max77840_chg_irq_chip = {
	.name = "max77840 chg",
	.status_base = REG_CHARGER_INT,
	.mask_base = REG_CHARGER_INT_MASK,
	.num_regs = 1,
	.irqs = max77840_chg_irqs,
	.num_irqs = ARRAY_SIZE(max77840_chg_irqs),
};

int max77840_map_irq(struct max77840_dev *max77840, int irq)
{
	return regmap_irq_get_virq(max77840->irqc_intsrc, irq);
}
EXPORT_SYMBOL_GPL(max77840_map_irq);

int max77840_map_sys_irq(struct max77840_dev *max77840, int irq)
{
	return regmap_irq_get_virq(max77840->irqc_sys, irq);
}
EXPORT_SYMBOL_GPL(max77840_map_sys_irq);

int max77840_map_chg_irq(struct max77840_dev *max77840, int irq)
{
	return regmap_irq_get_virq(max77840->irqc_chg, irq);
}
EXPORT_SYMBOL_GPL(max77840_map_chg_irq);

int max77840_map_chg_det_irq(struct max77840_dev *max77840, int irq)
{
	return regmap_irq_get_virq(max77840->irqc_chgdet, irq);
}
EXPORT_SYMBOL_GPL(max77840_map_chg_det_irq);

static int max77840_pmic_irq_int(struct max77840_dev *me)
{
	struct device *dev = me->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;

	/* disable all interrupt source */
	max77840_write(me->regmap_pmic, REG_INTSRCMASK, 0xFF);

	/* interrupt source */
	rc = regmap_add_irq_chip(me->regmap_pmic, me->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED, -1,
				 &max77840_intsrc_irq_chip,
				 &me->irqc_intsrc);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add insrc irq chip: %d\n", rc);
		goto out;
	}

	/* system interrupt */
	rc = regmap_add_irq_chip(me->regmap_pmic, me->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED, -1,
				 &max77840_sys_irq_chip,
				 &me->irqc_sys);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add system irq chip: %d\n",
			rc);
		goto err_irqc_sys;
	}

	/* charger interrupt */
	rc = regmap_add_irq_chip(me->regmap_chg, me->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED, -1,
				 &max77840_chg_irq_chip,
				 &me->irqc_chg);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add chg irq chip: %d\n", rc);
		goto err_irqc_chg;
	}

	/* charger detect interrupt */
	rc = regmap_add_irq_chip(me->regmap_chg_det, me->irq,
				 IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_SHARED, -1,
				 &max77840_chgdet_irq_chip,
				 &me->irqc_chgdet);
	if (rc != 0) {
		dev_err(&client->dev, "failed to add chgdet irq chip: %d\n", rc);
		goto err_irqc_chgdet;
	}

	pr_err("<%s> IRQ initialize done\n", client->name);
	return 0;

err_irqc_chgdet:
	regmap_del_irq_chip(me->irq, me->irqc_chg);
err_irqc_chg:
	regmap_del_irq_chip(me->irq, me->irqc_sys);
err_irqc_sys:
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);
out:
	return rc;
}

static int max77840_pre_init_data(struct max77840_dev *pmic)
{
#ifdef CONFIG_OF
	int size, cnt;
	const int *list;
	struct device *dev = pmic->dev;
	struct device_node *np = dev->of_node;

	u8 *init_data;

	list = of_get_property(np, "max77840,pmic-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77840,pmic-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max77840_write(pmic->regmap_pmic,
				       init_data[cnt], init_data[cnt + 1]);
		}
		kfree(init_data);
	}

	list = of_get_property(np, "max77840,chg-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77840,chg-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max77840_write(pmic->regmap_chg,
				       init_data[cnt], init_data[cnt + 1]);
		}
		kfree(init_data);
	}

	list = of_get_property(np, "max77840,chgdet-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77840,chgdet-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 2) {
			max77840_write(pmic->regmap_chg_det,
				       init_data[cnt], init_data[cnt + 1]);
		}
		kfree(init_data);
	}

	list = of_get_property(np, "max77840,fg-init", &size);
	if (list) {
		init_data = kmalloc(size, GFP_KERNEL);
		of_property_read_u8_array(np, "max77840,fg-init", init_data, size);
		for (cnt = 0; cnt < size; cnt += 3) {
			max77840_fg_write(pmic->regmap_fuel, init_data[cnt],
					  (init_data[cnt + 1] << 8) | init_data[cnt + 2]);
		}
		kfree(init_data);
	}
#endif
	return 0;
}

static struct mfd_cell max77840_devices[] = {
	{ .name = MAX77840_REGULATOR_NAME,		},
	{ .name = MAX77840_CHARGER_NAME,		},
	{ .name = MAX77840_CHARGER_DETECT_NAME,	},
	{ .name = MAX77840_FUELGAUGE_NAME,		},
};

static int max77840_pmic_setup(struct max77840_dev *me)
{
	struct device *dev = me->dev;
	struct i2c_client *client = to_i2c_client(dev);
	int rc = 0;
	u8 chip_id;
	u8 val = 0;

	/* IRQ init */
	pr_info("<%s> property:IRQ     %d\n", client->name, client->irq);
	me->irq = client->irq;

	pr_err("%s: max77840_pmic_irq_int\n", __func__);
	rc = max77840_pmic_irq_int(me);
	if (rc != 0) {
		dev_err(&client->dev, "failed to initialize irq: %d\n", rc);
		goto err_irq_init;
	}

	pr_err("%s: max77840_add_devices\n", __func__);
	rc = max77840_add_devices(me, max77840_devices,
				  ARRAY_SIZE(max77840_devices));
	if (rc < 0) {
		pr_err("<%s> failed to add sub-devices [%d]\n",
		       client->name, rc);
		goto err_add_devices;
	}

	pr_err("<%s> driver core " DRIVER_VERSION " installed\n", client->name);

	chip_id = 0;

	max77840_read(me->regmap_pmic, REG_PMICID,  &chip_id);
	pr_err("<%s> pmic id %Xh\n", client->name, chip_id);

	/* clear IRQ */
	max77840_read(me->regmap_pmic, REG_INTSRC, &val);
	pr_err("<%s> intsrc %Xh\n", client->name, val);

	max77840_read(me->regmap_pmic, REG_INTSRCMASK, &val);
	pr_err("<%s> intsrc_mask %Xh\n", client->name, val);

	/* set device able to wake up system */
	device_init_wakeup(dev, true);
	if (likely(me->irq > 0))
		enable_irq_wake((unsigned int)me->irq);

	return 0;

err_add_devices:
	regmap_del_irq_chip(me->irq, me->irqc_intsrc);
err_irq_init:
	return rc;
}

/*******************************************************************************
 *** MAX77840 MFD Core
 ******************************************************************************/

static __always_inline void max77840_destroy(struct max77840_dev *me)
{
	struct device *dev = me->dev;

	mfd_remove_devices(me->dev);

	if (likely(me->irq > 0))
		regmap_del_irq_chip(me->irq, me->irqc_intsrc);

	if (likely(me->irq_gpio >= 0))
		gpio_free((unsigned int)me->irq_gpio);

	if (likely(me->regmap_pmic))
		regmap_exit(me->regmap_pmic);

	if (likely(me->regmap_chg_det))
		regmap_exit(me->regmap_chg_det);

	if (likely(me->regmap_chg))
		regmap_exit(me->regmap_chg);

	if (likely(me->regmap_fuel))
		regmap_exit(me->regmap_fuel);

	mutex_destroy(&me->lock);
	devm_kfree(dev, me);
}

static int max77840_i2c_probe(struct i2c_client *client)
{
	struct max77840_dev *me;
	int rc;

	pr_err("<%s> attached\n", client->name);
	pr_err("%s: Max77840 I2C Driver Loading\n", __func__);

	me = devm_kzalloc(&client->dev, sizeof(*me), GFP_KERNEL);
	if (unlikely(!me)) {
		pr_err("<%s> out of memory (%uB requested)\n", client->name,
		       (unsigned int)sizeof(*me));
		return -ENOMEM;
	}

	i2c_set_clientdata(client, me);

	mutex_init(&me->lock);
	me->dev      = &client->dev;
	me->irq      = -1;
	me->irq_gpio = -1;

	me->pmic = client;

	me->regmap_pmic = devm_regmap_init_i2c(client, &max77840_regmap_config);
	if (IS_ERR(me->regmap_pmic)) {
		rc = PTR_ERR(me->regmap_pmic);
		me->regmap_pmic = NULL;
		pr_err("<%s> failed to initialize i2c\n",
		       client->name);
		pr_err("<%s> regmap pmic [%d]\n",
		       client->name,	rc);
		goto abort;
	}

	me->chg = i2c_new_dummy_device(client->adapter, I2C_ADDR_CHARGER);
	if (!me->chg) {
		rc = -ENOMEM;
		goto abort;
	}
	i2c_set_clientdata(me->chg, me);
	me->regmap_chg = devm_regmap_init_i2c(me->chg, &max77840_regmap_config);
	if (IS_ERR(me->regmap_chg)) {
		rc = PTR_ERR(me->regmap_chg);
		me->regmap_chg = NULL;
		pr_err("<%s> failed to initialize i2c\n",
		       client->name);
		pr_err("<%s> regmap chg [%d]\n",
		       client->name,	rc);
		goto abort;
	}

	me->chg_det = i2c_new_dummy_device(client->adapter, I2C_ADDR_CHARGER_DETECT);
	if (!me->chg_det) {
		rc = -ENOMEM;
		goto abort;
	}
	i2c_set_clientdata(me->chg_det, me);
	me->regmap_chg_det = devm_regmap_init_i2c(me->chg_det, &max77840_regmap_config);
	if (IS_ERR(me->regmap_chg_det)) {
		rc = PTR_ERR(me->regmap_chg_det);
		me->regmap_chg_det = NULL;
		pr_err("<%s> failed to initialize i2c\n",
		       client->name);
		pr_err("<%s> regmap chgdet [%d]\n",
		       client->name,	rc);
		goto abort;
	}

	me->fuel = i2c_new_dummy_device(client->adapter, I2C_ADDR_FUEL_GAUGE);
	if (!me->fuel) {
		rc = -ENOMEM;
		goto abort;
	}
	i2c_set_clientdata(me->fuel, me);
	me->regmap_fuel = devm_regmap_init_i2c(me->fuel, &max77840_regmap_config_fuelgauge);
	if (IS_ERR(me->regmap_fuel)) {
		rc = PTR_ERR(me->regmap_fuel);
		me->regmap_fuel = NULL;
		pr_err("<%s> failed to initialize i2c\n", client->name);
		pr_err("<%s> regmap fuelgauge [%d]\n", client->name, rc);
		goto abort;
	}

	max77840_pre_init_data(me);
	rc = max77840_pmic_setup(me);
	if (rc != 0) {
		pr_err("<%s> failed to set up interrupt\n",
		       client->name);
		pr_err("<%s> and add sub-device [%d]\n",
		       client->name, rc);
		goto abort;
	}

	return 0;

abort:
	pr_err("%s: Failed to probe max77840\n", __func__);
	i2c_set_clientdata(client, NULL);
	max77840_destroy(me);
	return rc;
}

static void max77840_i2c_remove(struct i2c_client *client)
{
	struct max77840_dev *me = i2c_get_clientdata(client);

	i2c_set_clientdata(client, NULL);
	max77840_destroy(me);
}

#ifdef CONFIG_PM_SLEEP
static int max77840_suspend(struct device *dev)
{
	struct max77840_dev *me = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(me);

	pr_info("<%s> suspending\n", client->name);

	__unlock(me);
	return 0;
}

static int max77840_resume(struct device *dev)
{
	struct max77840_dev *me = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	__lock(me);

	pr_info("<%s> resuming\n", client->name);

	__unlock(me);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77840_pm, max77840_suspend, max77840_resume);

#ifdef CONFIG_OF
static const struct of_device_id max77840_of_id[] = {
	{ .compatible = "maxim,max77840"},
	{ },
};
MODULE_DEVICE_TABLE(of, max77840_of_id);
#endif /* CONFIG_OF */

static const struct i2c_device_id max77840_i2c_id[] = {
	{ MAX77840_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max77840_i2c_id);

static struct i2c_driver max77840_i2c_driver = {
	.driver.name            = MAX77840_NAME,
	.driver.owner           = THIS_MODULE,
	.driver.pm              = &max77840_pm,
#ifdef CONFIG_OF
	.driver.of_match_table  = max77840_of_id,
#endif /* CONFIG_OF */
	.id_table               = max77840_i2c_id,
	.probe                  = max77840_i2c_probe,
	.remove                 = max77840_i2c_remove,
};

static __init int max77840_init(void)
{
	int rc = -ENODEV;

	rc = i2c_add_driver(&max77840_i2c_driver);
	if (rc != 0)
		pr_err("Failed to register I2C driver: %d\n", rc);
	pr_err("%s: Added I2C Driver\n", __func__);
	return rc;
}

static __exit void max77840_exit(void)
{
	i2c_del_driver(&max77840_i2c_driver);
}

module_init(max77840_init);
module_exit(max77840_exit);

MODULE_DESCRIPTION("MAX77840 MFD Gauge");
MODULE_AUTHOR("joan.na@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
