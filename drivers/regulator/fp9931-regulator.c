/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Fitipower FP9931 PMIC regulator driver
 *
 * Copyright 2021 NXP
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/fp9931.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/platform_device.h>

struct fp9931_data {
	int num_regulators;
	struct fp9931 *fp9931;
	struct regulator_dev **rdev;
};

/* array size is 0x40 */
static const int fp9931_vpos_vneg_voltages[] = {
	/* in uV */
	7040000,
	7040000,
	7040000,
	7040000,
	7040000,
	7260000,		/* 0x05 */
	7490000,
	7710000,
	7930000,
	8150000,
	8380000,
	8600000,
	8820000,
	9040000,
	9270000,
	9490000,
	9710000,
	9940000,
	10160000,
	10380000,
	10600000,
	10830000,
	11050000,
	11270000,
	11490000,
	11720000,
	11940000,
	12160000,
	12380000,
	12610000,
	12830000,
	13050000,
	13280000,
	13500000,
	13720000,
	13940000,
	14170000,
	14390000,
	14610000,
	14830000,
	15060000,		/* 0x28 */
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
	15060000,
};

/* trigger to power on VGL, VNEG, VGH, VPOS automatically */
static int fp9931_display_enable(struct regulator_dev *rdev)
{
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	gpio_set_value(fp9931->gpio_pmic_wakeup, 1);

	/* TODO: wait for power good */
	mdelay(15);

	return 0;
}

static int fp9931_display_disable(struct regulator_dev *rdev)
{
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	gpio_set_value(fp9931->gpio_pmic_wakeup, 0);

	mdelay(500);

	return 0;
}

static int fp9931_display_is_enabled(struct regulator_dev *rdev)
{
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	return !!gpio_get_value(fp9931->gpio_pmic_wakeup);
}

static int fp9931_display_set_soft_start(struct regulator_dev *rdev)
{
	int ret;
	u8 control_reg1;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	ret = fp9931_reg_read(fp9931->client, FP9931_CONTROL_REG1, &control_reg1);
	if (ret)
		return ret;

	control_reg1 &= ~CONTROL_REG1_SS_TIME;
	control_reg1 |= FIELD_PREP(CONTROL_REG1_SS_TIME, fp9931->ss_time);

	return fp9931_reg_write(fp9931->client, FP9931_CONTROL_REG1, control_reg1);
}

static int fp9931_v3p3_enable(struct regulator_dev *rdev)
{
	int ret;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);
	u8 control_reg1;

	ret = fp9931_reg_read(fp9931->client, FP9931_CONTROL_REG1, &control_reg1);
	if (ret)
		return ret;

	control_reg1 |= CONTROL_REG1_V3P3_EN;

	return fp9931_reg_write(fp9931->client, FP9931_CONTROL_REG1, control_reg1);
}

static int fp9931_v3p3_disable(struct regulator_dev *rdev)
{
	int ret;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);
	u8 control_reg1;

	ret = fp9931_reg_read(fp9931->client, FP9931_CONTROL_REG1, &control_reg1);
	if (ret)
		return ret;

	control_reg1 &= ~CONTROL_REG1_V3P3_EN;

	return fp9931_reg_write(fp9931->client, FP9931_CONTROL_REG1, control_reg1);
}

static int fp9931_v3p3_is_enabled(struct regulator_dev *rdev)
{
	int ret;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);
	u8 control_reg1;

	ret = fp9931_reg_read(fp9931->client, FP9931_CONTROL_REG1, &control_reg1);
	if (ret)
		return ret;

	return !!(control_reg1 & CONTROL_REG1_V3P3_EN);
}

static int fp9931_vcom_get_voltage_sel(struct regulator_dev *rdev)
{
	int ret;
	u8 vcom_setting;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	ret = fp9931_reg_read(fp9931->client, FP9931_VCOM_SETTING, &vcom_setting);
	if (ret)
		return ret;

	if (!vcom_setting)
		return -EINVAL;

	return --vcom_setting;
}

static int fp9931_vcom_set_voltage_sel(struct regulator_dev *rdev,
				       unsigned selector)
{
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	if (++selector > 0xFF)
		return -EINVAL;

	return fp9931_reg_write(fp9931->client, FP9931_VCOM_SETTING, selector);
}

/* VCOM = 0V + [(-5 / 255) * N]V, N = 1~255 (0 is meaningless) */
static int fp9931_vcom_list_voltage(struct regulator_dev *rdev,
				    unsigned selector)
{
	int vol_uV;

	if (++selector > 0xFF)
		return -EINVAL;

	vol_uV = selector * 1000 * 1000;

	return DIV_ROUND_CLOSEST(vol_uV * 5, 255);
}

static int fp9931_vpos_vneg_get_voltage_sel(struct regulator_dev *rdev)
{
	int ret;
	u8 vpos_vneg_setting;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	ret = fp9931_reg_read(fp9931->client, FP9931_VPOS_VNEG_SETTING,
			      &vpos_vneg_setting);
	if (ret)
		return ret;

	return FIELD_GET(VPOS_VNEG_SETTING, vpos_vneg_setting);
}

static int fp9931_vpos_vneg_set_voltage_sel(struct regulator_dev *rdev,
					    unsigned selector)
{
	u8 vpos_vneg_setting;
	struct fp9931 *fp9931 = rdev_get_drvdata(rdev);

	if (unlikely(selector > VPOS_VNEG_SETTING))
		return -EINVAL;

	vpos_vneg_setting = FIELD_PREP(VPOS_VNEG_SETTING, selector);

	return fp9931_reg_write(fp9931->client,
				FP9931_VPOS_VNEG_SETTING, vpos_vneg_setting);
}

static struct regulator_ops fp9931_display_ops = {
	.enable			= fp9931_display_enable,
	.disable		= fp9931_display_disable,
	.is_enabled		= fp9931_display_is_enabled,
	.set_soft_start		= fp9931_display_set_soft_start,
};

static struct regulator_ops fp9931_v3p3_ops = {
	.enable			= fp9931_v3p3_enable,
	.disable		= fp9931_v3p3_disable,
	.is_enabled		= fp9931_v3p3_is_enabled,
};

static struct regulator_ops fp9931_vcom_ops = {
	.get_voltage_sel	= fp9931_vcom_get_voltage_sel,
	.set_voltage_sel	= fp9931_vcom_set_voltage_sel,
	.list_voltage		= fp9931_vcom_list_voltage,
};

static struct regulator_ops fp9931_vpos_ops = {
	.get_voltage_sel	= fp9931_vpos_vneg_get_voltage_sel,
	.set_voltage_sel	= fp9931_vpos_vneg_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_table,
};

static struct regulator_ops fp9931_vneg_ops = {
	.get_voltage_sel	= fp9931_vpos_vneg_get_voltage_sel,
	.set_voltage_sel	= fp9931_vpos_vneg_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_table,
};

static struct regulator_ops fp9931_vgh_ops = {
};

static struct regulator_ops fp9931_vgl_ops = {
};

static struct regulator_desc fp9931_reg[] = {
	{
		.name		= "DISPLAY",
		.id		= FP9931_DISPLAY,
		.ops		= &fp9931_display_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VPOS-LDO",
		.id		= FP9931_VPOS,
		.ops		= &fp9931_vpos_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= ARRAY_SIZE(fp9931_vpos_vneg_voltages),
		.volt_table	= fp9931_vpos_vneg_voltages,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VNEG-LDO",
		.id		= FP9931_VNEG,
		.ops		= &fp9931_vneg_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= ARRAY_SIZE(fp9931_vpos_vneg_voltages),
		.volt_table	= fp9931_vpos_vneg_voltages,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VGH-CHARGE-PUMP",
		.id		= FP9931_VGH,
		.ops		= &fp9931_vgh_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VGL-CHARGE-PUMP",
		.id		= FP9931_VGL,
		.ops		= &fp9931_vgl_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "VCOM",
		.id		= FP9931_VCOM,
		.ops		= &fp9931_vcom_ops,
		.type		= REGULATOR_VOLTAGE,
		.n_voltages	= 255,
		.owner		= THIS_MODULE,
	},
	{
		.name		= "V3P3",
		.id		= FP9931_V3P3,
		.ops		= &fp9931_v3p3_ops,
		.type		= REGULATOR_VOLTAGE,
		.owner		= THIS_MODULE,
	},
};

#define GET_DELAY_TIME_PROPERTY(prop, field) \
do { \
	int ret = of_property_read_u32(fp9931->dev->of_node, \
				       prop, &fp9931->field); \
	if (ret < 0) 				\
		return ret;			\
\
	switch (fp9931->field) {		\
	case 0:					\
	case 1:					\
	case 2:					\
		break;				\
	case 4:					\
		fp9931->field = 0x3;		\
		break;				\
	default:				\
		return -EINVAL;			\
	}					\
} while (0);

static int fp9931_of_get_time_properties(struct fp9931 *fp9931)
{
	int ret;

	GET_DELAY_TIME_PROPERTY("vgl-pwrup",  vgl_pwrup);
	GET_DELAY_TIME_PROPERTY("vneg-pwrup", vneg_pwrup);
	GET_DELAY_TIME_PROPERTY("vgh-pwrup",  vgh_pwrup);
	GET_DELAY_TIME_PROPERTY("vpos-pwrup", vpos_pwrup);

	ret = of_property_read_u32(fp9931->dev->of_node, "ss-time",
				   &fp9931->ss_time);
	if (ret < 0)
		return ret;

	switch(fp9931->ss_time) {
	case 3:
		fp9931->ss_time = 0x0;
		break;
	case 4:
		fp9931->ss_time = 0x1;
		break;
	case 5:
		fp9931->ss_time = 0x2;
		break;
	case 6:
		fp9931->ss_time = 0x3;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int fp9931_pmic_dt_parse_pdata(struct platform_device *pdev,
				      struct fp9931_platform_data *pdata)
{
	struct fp9931 *fp9931 = dev_get_drvdata(pdev->dev.parent);
	struct device_node *pmic_np, *regulators_np, *reg_np;
	struct fp9931_regulator_data *rdata;
	int i, ret;

	pmic_np = of_node_get(fp9931->dev->of_node);
	if (!pmic_np) {
		dev_err(&pdev->dev, "could not find pmic sub-node\n");
		return -ENODEV;
	}

	regulators_np = of_find_node_by_name(pmic_np, "regulators");
	if (!regulators_np) {
		dev_err(&pdev->dev, "could not find regulators sub-node\n");
		of_node_put(pmic_np);
		return -EINVAL;
	}

	pdata->num_regulators = of_get_child_count(regulators_np);

	rdata = devm_kzalloc(&pdev->dev,
			     sizeof(*rdata) * pdata->num_regulators,
			     GFP_KERNEL);
	if (!rdata) {
		dev_err(&pdev->dev, "failed to allocate memory for regulator data\n");
		of_node_put(regulators_np);
		return -ENOMEM;
	}

	pdata->regulators = rdata;
	for_each_child_of_node(regulators_np, reg_np) {
		for (i = 0; i < ARRAY_SIZE(fp9931_reg); i++)
			if (!of_node_cmp(reg_np->name, fp9931_reg[i].name))
				break;

		if (i == ARRAY_SIZE(fp9931_reg)) {
			dev_warn(&pdev->dev, "unknown regulator %s\n", reg_np->name);
			continue;
		}

		rdata->id = i;
		rdata->initdata = of_get_regulator_init_data(&pdev->dev,
							     reg_np,
							     &fp9931_reg[i]);
		if (!rdata->initdata) {
			dev_err(&pdev->dev, "failed to get regulator init data\n");
			return -ENOMEM;
		}

		rdata->reg_node = reg_np;
		rdata++;
	}
	of_node_put(regulators_np);

	ret = fp9931_of_get_time_properties(fp9931);
	if (ret)
		return ret;

	fp9931->gpio_pmic_wakeup = of_get_named_gpio(pmic_np,
					"gpio-pmic-wakeup", 0);
	if (!gpio_is_valid(fp9931->gpio_pmic_wakeup)) {
		dev_err(&pdev->dev, "no epdc pmic wakeup pin available\n");
		return fp9931->gpio_pmic_wakeup;
	}
	ret = devm_gpio_request_one(&pdev->dev, fp9931->gpio_pmic_wakeup,
				    GPIOF_OUT_INIT_LOW, "epdc-pmic-wake");
	if (ret < 0)
		return ret;

	/* If 'gpio_pmic_wakeup' is high previously, after this gpio
	 * is pull-down, the I2C reg write will not work as expected
	 * (the value cannot be written to FP9931 correctly). And add
	 * 500ms delay can solve this problem.
	 */
	mdelay(500);

	fp9931->gpio_pmic_pwrgood = of_get_named_gpio(pmic_np,
					"gpio-pmic-pwrgood", 0);
	if (!gpio_is_valid(fp9931->gpio_pmic_pwrgood)) {
		dev_err(&pdev->dev, "no epdc pmic pwrgood pin available\n");
		return fp9931->gpio_pmic_pwrgood;
	}
	return devm_gpio_request_one(&pdev->dev, fp9931->gpio_pmic_pwrgood,
				     GPIOF_IN, "epdc-pwrstat");
}

/*
 * Config power on delay times for VGL, VNEG, VGH and VPOS.
 * The delay times are not ramp-delays. So config this in
 * probe.
 */
static int fp9931_config_power_on_delays(struct fp9931 *fp9931)
{
	u8 poweron_delay;

	poweron_delay = FIELD_PREP(PWRON_DELAY_tDLY1, fp9931->vgl_pwrup)  |
			FIELD_PREP(PWRON_DELAY_tDLY2, fp9931->vneg_pwrup) |
			FIELD_PREP(PWRON_DELAY_tDLY3, fp9931->vgh_pwrup)  |
			FIELD_PREP(PWRON_DELAY_tDLY4, fp9931->vpos_pwrup);
	return fp9931_reg_write(fp9931->client, FP9931_PWRON_DELAY, poweron_delay);
}

static int fp9931_regulator_probe(struct platform_device *pdev)
{
	struct fp9931 *fp9931 = dev_get_drvdata(pdev->dev.parent);
	struct fp9931_platform_data *pdata = fp9931->pdata;
	struct fp9931_data *priv;
	struct regulator_dev **rdev;
	struct regulator_config config = { };
	int size, i, ret = 0;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	ret = fp9931_pmic_dt_parse_pdata(pdev, pdata);
	if (ret)
		return ret;

	size = sizeof(*rdev) * pdata->num_regulators;
	rdev = devm_kzalloc(&pdev->dev, size, GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	priv->rdev = rdev;
	priv->num_regulators = pdata->num_regulators;
	platform_set_drvdata(pdev, priv);

	for (i = 0; i < pdata->num_regulators; i++) {
		int id = pdata->regulators[i].id;

		config.dev = &pdev->dev;
		config.init_data = pdata->regulators[i].initdata;
		config.driver_data = fp9931;
		config.of_node = pdata->regulators[i].reg_node;

		rdev[i] = devm_regulator_register(&pdev->dev, &fp9931_reg[id],
						  &config);
		if (IS_ERR(rdev[i])) {
			dev_err(&pdev->dev, "register regulator %s failed\n",
				fp9931_reg[id].name);
			return PTR_ERR(rdev[i]);
		}
	}

	return fp9931_config_power_on_delays(fp9931);
}

static void fp9931_regulator_remove(struct platform_device *pdev)
{
}

static const struct platform_device_id fp9931_pmic_id[] = {
	{ "fp9931-pmic", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, fp9931_pmic_id);

static struct platform_driver fp9931_regulator_driver = {
	.probe  = fp9931_regulator_probe,
	.remove = fp9931_regulator_remove,
	.id_table = fp9931_pmic_id,
	.driver = {
		.name = "fp9931-pmic",
	},
};

static int __init fp9931_regulator_init(void)
{
	return platform_driver_register(&fp9931_regulator_driver);
}
subsys_initcall_sync(fp9931_regulator_init);

static void __exit fp9931_regulator_exit(void)
{
	platform_driver_unregister(&fp9931_regulator_driver);
}
module_exit(fp9931_regulator_exit);

MODULE_DESCRIPTION("FP9931 regulator driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
