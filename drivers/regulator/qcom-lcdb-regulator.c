// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define LCDB_INT_RT_STS			0x10
#define  LCDB_VREG_OK_RT_STS		BIT(0)
#define  LCDB_SC_ERROR_RT_STS		BIT(1)

#define LCDB_BST_OUTPUT_VOLTAGE		0x41
#define  LCDB_BST_OUTPUT_VOLTAGE_MASK	GENMASK(5, 0)

#define LCDB_MODULE_RDY			0x45
#define  LCDB_MODULE_RDY_BIT		BIT(7)

#define LCDB_ENABLE_CTL1		0x46
#define  LCDB_MODULE_EN			BIT(7)

#define LCDB_LDO_OUTPUT_VOLTAGE		0x71
#define LCDB_LDO_FORCE_PD_CTL		0x79
#define  LCDB_LDO_FORCE_PD_EN		BIT(0)
#define  LCDB_LDO_FORCE_PD_MODE		BIT(7)

#define LCDB_NCP_OUTPUT_VOLTAGE		0x81

#define LCDB_OUTPUT_VOLTAGE_MASK	GENMASK(4, 0)

#define LCDB_MIN_UV			4000000
#define LCDB_STEP_100MV_UV		100000
#define LCDB_50MV_MIN_UV		4950000
#define LCDB_STEP_50MV_UV		50000
#define LCDB_50MV_MIN_SEL		10
#define LCDB_N_VOLTAGES			32

#define LCDB_BST_MIN_UV			4700000
#define LCDB_BST_MAX_UV			6275000
#define LCDB_BST_STEP_UV		25000
#define LCDB_BST_HEADROOM_UV		150000

#define LCDB_RAMP_START_UV		4500000
#define LCDB_RAMP_STEP_UV		500000

#define LCDB_OFF_ON_DELAY_US		20000
#define LCDB_VREG_OK_TIMEOUT_US		50000

#define LCDB_SC_RESET_CNT_DLY_US	1000000
#define LCDB_SC_CNT_MAX			10

enum {
	LCDB_LDO,
	LCDB_NCP,
	LCDB_NUM_RAILS,
};

struct qcom_lcdb {
	struct device *dev;
	struct regmap *regmap;
	u32 base;

	struct mutex lock;
	struct regulator_dev *rdev[LCDB_NUM_RAILS];
	int uV[LCDB_NUM_RAILS];
	unsigned int enabled_mask;
	ktime_t off_time;

	bool sc_disabled;
	int sc_count;
	ktime_t sc_time;
};

static const u8 qcom_lcdb_vreg[LCDB_NUM_RAILS] = {
	[LCDB_LDO] = LCDB_LDO_OUTPUT_VOLTAGE,
	[LCDB_NCP] = LCDB_NCP_OUTPUT_VOLTAGE,
};

static unsigned int qcom_lcdb_uV_to_sel(int uV)
{
	if (uV < LCDB_50MV_MIN_UV)
		return (uV - LCDB_MIN_UV) / LCDB_STEP_100MV_UV;

	return LCDB_50MV_MIN_SEL + (uV - LCDB_50MV_MIN_UV) / LCDB_STEP_50MV_UV;
}

static int qcom_lcdb_sel_to_uV(unsigned int sel)
{
	if (sel < LCDB_50MV_MIN_SEL)
		return LCDB_MIN_UV + sel * LCDB_STEP_100MV_UV;

	return LCDB_50MV_MIN_UV + (sel - LCDB_50MV_MIN_SEL) * LCDB_STEP_50MV_UV;
}

static int qcom_lcdb_write_voltages(struct qcom_lcdb *lcdb, int ldo_uV, int ncp_uV)
{
	const int uV[LCDB_NUM_RAILS] = { ldo_uV, ncp_uV };
	unsigned int bst_sel, cur_sel;
	int bst_uV, ret, i;

	bst_uV = clamp(max(ldo_uV, ncp_uV) + LCDB_BST_HEADROOM_UV,
		       LCDB_BST_MIN_UV, LCDB_BST_MAX_UV);
	bst_sel = DIV_ROUND_UP(bst_uV - LCDB_BST_MIN_UV, LCDB_BST_STEP_UV);

	ret = regmap_read(lcdb->regmap, lcdb->base + LCDB_BST_OUTPUT_VOLTAGE, &cur_sel);
	if (ret)
		return ret;
	cur_sel &= LCDB_BST_OUTPUT_VOLTAGE_MASK;

	/* The boost has to stay above both outputs while they are changing */
	if (bst_sel > cur_sel) {
		ret = regmap_update_bits(lcdb->regmap,
					 lcdb->base + LCDB_BST_OUTPUT_VOLTAGE,
					 LCDB_BST_OUTPUT_VOLTAGE_MASK, bst_sel);
		if (ret)
			return ret;
	}

	for (i = 0; i < LCDB_NUM_RAILS; i++) {
		ret = regmap_update_bits(lcdb->regmap,
					 lcdb->base + qcom_lcdb_vreg[i],
					 LCDB_OUTPUT_VOLTAGE_MASK,
					 qcom_lcdb_uV_to_sel(uV[i]));
		if (ret)
			return ret;
	}

	if (bst_sel < cur_sel)
		ret = regmap_update_bits(lcdb->regmap,
					 lcdb->base + LCDB_BST_OUTPUT_VOLTAGE,
					 LCDB_BST_OUTPUT_VOLTAGE_MASK, bst_sel);

	return ret;
}

static int qcom_lcdb_write_limited(struct qcom_lcdb *lcdb, int limit_uV)
{
	return qcom_lcdb_write_voltages(lcdb,
					min(lcdb->uV[LCDB_LDO], limit_uV),
					min(lcdb->uV[LCDB_NCP], limit_uV));
}

static int qcom_lcdb_module_disable(struct qcom_lcdb *lcdb)
{
	int ret, pd_ret;

	/* Force the LDO pull-down to discharge the output quickly */
	ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_LDO_FORCE_PD_CTL,
			   LCDB_LDO_FORCE_PD_EN | LCDB_LDO_FORCE_PD_MODE);
	if (ret)
		return ret;

	ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1, 0);
	if (!ret) {
		lcdb->off_time = ktime_get();
		usleep_range(10000, 11000);
	}

	pd_ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_LDO_FORCE_PD_CTL, 0);

	return ret ?: pd_ret;
}

static int qcom_lcdb_module_enable(struct qcom_lcdb *lcdb)
{
	int target_uV = max(lcdb->uV[LCDB_LDO], lcdb->uV[LCDB_NCP]);
	int uV = LCDB_RAMP_START_UV;
	unsigned int val;
	s64 off_us;
	int ret;

	off_us = ktime_us_delta(ktime_get(), lcdb->off_time);
	if (off_us < LCDB_OFF_ON_DELAY_US)
		usleep_range(LCDB_OFF_ON_DELAY_US - off_us,
			     LCDB_OFF_ON_DELAY_US - off_us + 1000);

	/* Start low and ramp the outputs up in steps to limit the inrush current */
	ret = qcom_lcdb_write_limited(lcdb, uV);
	if (ret)
		return ret;

	ret = regmap_write(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1,
			   LCDB_MODULE_EN);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(lcdb->regmap,
				       lcdb->base + LCDB_INT_RT_STS, val,
				       val & LCDB_VREG_OK_RT_STS,
				       1000, LCDB_VREG_OK_TIMEOUT_US);
	if (ret) {
		dev_err(lcdb->dev, "outputs failed to come up: %d\n", ret);
		goto err_disable;
	}

	usleep_range(10000, 11000);

	while (uV < target_uV) {
		uV += LCDB_RAMP_STEP_UV;

		ret = qcom_lcdb_write_limited(lcdb, uV);
		if (ret)
			goto err_disable;

		usleep_range(1000, 1100);
	}

	return 0;

err_disable:
	qcom_lcdb_module_disable(lcdb);

	return ret;
}

static int qcom_lcdb_enable(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);
	int ret = 0;

	guard(mutex)(&lcdb->lock);

	if (lcdb->sc_disabled)
		return -EIO;

	if (!lcdb->enabled_mask)
		ret = qcom_lcdb_module_enable(lcdb);
	if (ret)
		return ret;

	lcdb->enabled_mask |= BIT(rdev_get_id(rdev));

	return 0;
}

static int qcom_lcdb_disable(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);
	unsigned int mask;
	int ret;

	guard(mutex)(&lcdb->lock);

	mask = lcdb->enabled_mask & ~BIT(rdev_get_id(rdev));
	if (!mask && !lcdb->sc_disabled) {
		ret = qcom_lcdb_module_disable(lcdb);
		if (ret)
			return ret;
	}

	lcdb->enabled_mask = mask;

	return 0;
}

static int qcom_lcdb_is_enabled(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);

	guard(mutex)(&lcdb->lock);

	return !lcdb->sc_disabled && (lcdb->enabled_mask & BIT(rdev_get_id(rdev)));
}

static int qcom_lcdb_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);
	int id = rdev_get_id(rdev);
	int uV[LCDB_NUM_RAILS];
	int ret;

	guard(mutex)(&lcdb->lock);

	memcpy(uV, lcdb->uV, sizeof(uV));
	uV[id] = qcom_lcdb_sel_to_uV(sel);

	ret = qcom_lcdb_write_voltages(lcdb, uV[LCDB_LDO], uV[LCDB_NCP]);
	if (ret)
		return ret;

	lcdb->uV[id] = uV[id];

	return 0;
}

static int qcom_lcdb_get_voltage_sel(struct regulator_dev *rdev)
{
	struct qcom_lcdb *lcdb = rdev_get_drvdata(rdev);

	guard(mutex)(&lcdb->lock);

	return qcom_lcdb_uV_to_sel(lcdb->uV[rdev_get_id(rdev)]);
}

static const struct regulator_ops qcom_lcdb_ops = {
	.enable			= qcom_lcdb_enable,
	.disable		= qcom_lcdb_disable,
	.is_enabled		= qcom_lcdb_is_enabled,
	.set_voltage_sel	= qcom_lcdb_set_voltage_sel,
	.get_voltage_sel	= qcom_lcdb_get_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear_range,
	.map_voltage		= regulator_map_voltage_linear_range,
};

static const struct linear_range qcom_lcdb_ranges[] = {
	REGULATOR_LINEAR_RANGE(LCDB_MIN_UV, 0, LCDB_50MV_MIN_SEL - 1,
			       LCDB_STEP_100MV_UV),
	REGULATOR_LINEAR_RANGE(LCDB_50MV_MIN_UV, LCDB_50MV_MIN_SEL,
			       LCDB_N_VOLTAGES - 1, LCDB_STEP_50MV_UV),
};

static const struct regulator_desc qcom_lcdb_descs[LCDB_NUM_RAILS] = {
	[LCDB_LDO] = {
		.name			= "lcdb_ldo",
		.of_match		= "ldo",
		.id			= LCDB_LDO,
		.ops			= &qcom_lcdb_ops,
		.type			= REGULATOR_VOLTAGE,
		.linear_ranges		= qcom_lcdb_ranges,
		.n_linear_ranges	= ARRAY_SIZE(qcom_lcdb_ranges),
		.n_voltages		= LCDB_N_VOLTAGES,
		.owner			= THIS_MODULE,
	},
	[LCDB_NCP] = {
		.name			= "lcdb_ncp",
		.of_match		= "ncp",
		.id			= LCDB_NCP,
		.ops			= &qcom_lcdb_ops,
		.type			= REGULATOR_VOLTAGE,
		.linear_ranges		= qcom_lcdb_ranges,
		.n_linear_ranges	= ARRAY_SIZE(qcom_lcdb_ranges),
		.n_voltages		= LCDB_N_VOLTAGES,
		.owner			= THIS_MODULE,
	},
};

static bool qcom_lcdb_handle_sc(struct qcom_lcdb *lcdb)
{
	guard(mutex)(&lcdb->lock);

	if (!lcdb->enabled_mask || lcdb->sc_disabled)
		return false;

	qcom_lcdb_module_disable(lcdb);

	if (ktime_us_delta(ktime_get(), lcdb->sc_time) > LCDB_SC_RESET_CNT_DLY_US) {
		lcdb->sc_count = 0;
	} else if (lcdb->sc_count > LCDB_SC_CNT_MAX) {
		dev_err(lcdb->dev, "persistent short circuit, keeping the outputs off\n");
		lcdb->sc_disabled = true;
		return true;
	}

	lcdb->sc_count++;
	lcdb->sc_time = ktime_get();

	if (qcom_lcdb_module_enable(lcdb)) {
		dev_err(lcdb->dev, "failed to recover from a short circuit\n");
		lcdb->sc_disabled = true;
		return true;
	}

	return false;
}

static irqreturn_t qcom_lcdb_sc_irq(int irq, void *data)
{
	struct qcom_lcdb *lcdb = data;
	unsigned int val;
	int ret, i;

	ret = regmap_read(lcdb->regmap, lcdb->base + LCDB_INT_RT_STS, &val);
	if (ret || !(val & LCDB_SC_ERROR_RT_STS))
		return IRQ_HANDLED;

	/* Blanking time, a real short circuit is still reported afterwards */
	usleep_range(2000, 2100);

	ret = regmap_read(lcdb->regmap, lcdb->base + LCDB_INT_RT_STS, &val);
	if (ret || !(val & LCDB_SC_ERROR_RT_STS))
		return IRQ_HANDLED;

	if (qcom_lcdb_handle_sc(lcdb)) {
		for (i = 0; i < LCDB_NUM_RAILS; i++)
			regulator_notifier_call_chain(lcdb->rdev[i],
						      REGULATOR_EVENT_FAIL, NULL);
	}

	return IRQ_HANDLED;
}

static int qcom_lcdb_hw_init(struct qcom_lcdb *lcdb)
{
	unsigned int val;
	int ret, i;

	for (i = 0; i < LCDB_NUM_RAILS; i++) {
		ret = regmap_read(lcdb->regmap, lcdb->base + qcom_lcdb_vreg[i], &val);
		if (ret)
			return ret;

		lcdb->uV[i] = qcom_lcdb_sel_to_uV(val & LCDB_OUTPUT_VOLTAGE_MASK);
	}

	ret = regmap_read(lcdb->regmap, lcdb->base + LCDB_ENABLE_CTL1, &val);
	if (ret)
		return ret;

	if (val & LCDB_MODULE_EN) {
		lcdb->enabled_mask = GENMASK(LCDB_NUM_RAILS - 1, 0);
		return 0;
	}

	return regmap_set_bits(lcdb->regmap, lcdb->base + LCDB_MODULE_RDY,
			       LCDB_MODULE_RDY_BIT);
}

static int qcom_lcdb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config cfg = {};
	struct qcom_lcdb *lcdb;
	int irq, ret, i;

	lcdb = devm_kzalloc(dev, sizeof(*lcdb), GFP_KERNEL);
	if (!lcdb)
		return -ENOMEM;

	lcdb->dev = dev;

	lcdb->regmap = dev_get_regmap(dev->parent, NULL);
	if (!lcdb->regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get the parent regmap\n");

	ret = device_property_read_u32(dev, "reg", &lcdb->base);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get the base address\n");

	ret = devm_mutex_init(dev, &lcdb->lock);
	if (ret)
		return ret;

	ret = qcom_lcdb_hw_init(lcdb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read the initial state\n");

	cfg.dev = dev;
	cfg.regmap = lcdb->regmap;
	cfg.driver_data = lcdb;

	for (i = 0; i < LCDB_NUM_RAILS; i++) {
		lcdb->rdev[i] = devm_regulator_register(dev, &qcom_lcdb_descs[i], &cfg);
		if (IS_ERR(lcdb->rdev[i]))
			return dev_err_probe(dev, PTR_ERR(lcdb->rdev[i]),
					     "failed to register %s\n",
					     qcom_lcdb_descs[i].name);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	return devm_request_threaded_irq(dev, irq, NULL, qcom_lcdb_sc_irq,
					 IRQF_ONESHOT, "lcdb-sc", lcdb);
}

static const struct of_device_id qcom_lcdb_match[] = {
	{ .compatible = "qcom,pm8150l-lcdb" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_lcdb_match);

static struct platform_driver qcom_lcdb_driver = {
	.driver = {
		.name = "qcom-lcdb-regulator",
		.of_match_table = qcom_lcdb_match,
	},
	.probe = qcom_lcdb_probe,
};
module_platform_driver(qcom_lcdb_driver);

MODULE_DESCRIPTION("Qualcomm LCDB (LCD bias) regulator driver");
MODULE_LICENSE("GPL");
