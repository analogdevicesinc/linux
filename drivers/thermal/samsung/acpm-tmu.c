// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2019 Samsung Electronics Co., Ltd.
 * Copyright 2025 Google LLC.
 * Copyright 2026 Linaro Ltd.
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/device/devres.h>
#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/units.h>

#include "../thermal_hwmon.h"

#define EXYNOS_TMU_SENSOR(i)		BIT(i)
#define EXYNOS_TMU_SENSORS_MAX_COUNT	16

#define GS101_CPUCL2_SENSOR_MASK (EXYNOS_TMU_SENSOR(0) |	\
				  EXYNOS_TMU_SENSOR(6) |	\
				  EXYNOS_TMU_SENSOR(7) |	\
				  EXYNOS_TMU_SENSOR(8) |	\
				  EXYNOS_TMU_SENSOR(9))
#define GS101_CPUCL1_SENSOR_MASK (EXYNOS_TMU_SENSOR(4) |	\
				  EXYNOS_TMU_SENSOR(5))
#define GS101_CPUCL0_SENSOR_MASK (EXYNOS_TMU_SENSOR(1) |	\
				  EXYNOS_TMU_SENSOR(2))

#define GS101_REG_INTPEND(i)		((i) * 0x50 + 0xf8)

enum {
	P0_INTPEND,
	P1_INTPEND,
	P2_INTPEND,
	P3_INTPEND,
	P4_INTPEND,
	P5_INTPEND,
	P6_INTPEND,
	P7_INTPEND,
	P8_INTPEND,
	P9_INTPEND,
	P10_INTPEND,
	P11_INTPEND,
	P12_INTPEND,
	P13_INTPEND,
	P14_INTPEND,
	P15_INTPEND,
	REG_INTPEND_COUNT,
};

struct acpm_tmu_sensor_group {
	u16 mask;
	u8 id;
};

struct acpm_tmu_sensor {
	const struct acpm_tmu_sensor_group *group;
	struct thermal_zone_device *tzd;
	struct acpm_tmu_priv *priv;
	struct mutex lock; /* protects sensor state */
	enum thermal_device_mode mode;
	bool enabled;
};

struct acpm_tmu_priv {
	struct regmap_field *regmap_fields[REG_INTPEND_COUNT];
	struct acpm_handle *handle;
	struct device *dev;
	struct clk *clk;
	unsigned int mbox_chan_id;
	unsigned int num_sensors;
	int irq;
	struct acpm_tmu_sensor sensors[] __counted_by(num_sensors);
};

struct acpm_tmu_driver_data {
	const struct reg_field *reg_fields;
	const struct acpm_tmu_sensor_group *sensor_groups;
	unsigned int num_sensor_groups;
	unsigned int mbox_chan_id;
};

#define ACPM_TMU_SENSOR_GROUP(_mask, _id)		\
	{					\
		.mask	= _mask,		\
		.id	= _id,			\
	}

static const struct acpm_tmu_sensor_group gs101_sensor_groups[] = {
	ACPM_TMU_SENSOR_GROUP(GS101_CPUCL2_SENSOR_MASK, 0),
	ACPM_TMU_SENSOR_GROUP(GS101_CPUCL1_SENSOR_MASK, 1),
	ACPM_TMU_SENSOR_GROUP(GS101_CPUCL0_SENSOR_MASK, 2),
};

static const struct reg_field gs101_reg_fields[REG_INTPEND_COUNT] = {
	[P0_INTPEND] = REG_FIELD(GS101_REG_INTPEND(0), 0, 31),
	[P1_INTPEND] = REG_FIELD(GS101_REG_INTPEND(1), 0, 31),
	[P2_INTPEND] = REG_FIELD(GS101_REG_INTPEND(2), 0, 31),
	[P3_INTPEND] = REG_FIELD(GS101_REG_INTPEND(3), 0, 31),
	[P4_INTPEND] = REG_FIELD(GS101_REG_INTPEND(4), 0, 31),
	[P5_INTPEND] = REG_FIELD(GS101_REG_INTPEND(5), 0, 31),
	[P6_INTPEND] = REG_FIELD(GS101_REG_INTPEND(6), 0, 31),
	[P7_INTPEND] = REG_FIELD(GS101_REG_INTPEND(7), 0, 31),
	[P8_INTPEND] = REG_FIELD(GS101_REG_INTPEND(8), 0, 31),
	[P9_INTPEND] = REG_FIELD(GS101_REG_INTPEND(9), 0, 31),
	[P10_INTPEND] = REG_FIELD(GS101_REG_INTPEND(10), 0, 31),
	[P11_INTPEND] = REG_FIELD(GS101_REG_INTPEND(11), 0, 31),
	[P12_INTPEND] = REG_FIELD(GS101_REG_INTPEND(12), 0, 31),
	[P13_INTPEND] = REG_FIELD(GS101_REG_INTPEND(13), 0, 31),
	[P14_INTPEND] = REG_FIELD(GS101_REG_INTPEND(14), 0, 31),
	[P15_INTPEND] = REG_FIELD(GS101_REG_INTPEND(15), 0, 31),
};

static const struct regmap_config gs101_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.use_relaxed_mmio = true,
	.disable_locking = true,
	.max_register = GS101_REG_INTPEND(15),
};

static const struct acpm_tmu_driver_data acpm_tmu_gs101 = {
	.reg_fields = gs101_reg_fields,
	.sensor_groups = gs101_sensor_groups,
	.num_sensor_groups = ARRAY_SIZE(gs101_sensor_groups),
	.mbox_chan_id = 9,
};

static int acpm_tmu_op_tz_control(struct acpm_tmu_sensor *sensor, bool on)
{
	struct acpm_tmu_priv *priv = sensor->priv;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	int ret;

	ret = ops->tz_control(handle, priv->mbox_chan_id, sensor->group->id,
			      on);
	if (ret)
		return ret;

	WRITE_ONCE(sensor->enabled, on);

	return 0;
}

static int acpm_tmu_control(struct acpm_tmu_priv *priv, bool on)
{
	struct device *dev = priv->dev;
	int i, ret, err = 0;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	for (i = 0; i < priv->num_sensors; i++) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];

		/* Skip sensors that weren't found in DT */
		if (!sensor->tzd)
			continue;

		scoped_guard(mutex, &sensor->lock) {
			ret = 0;
			if (sensor->mode == THERMAL_DEVICE_ENABLED &&
			    sensor->enabled != on)
				ret = acpm_tmu_op_tz_control(sensor, on);
		}
		if (ret && !err)
			err = ret;
	}

	pm_runtime_put_autosuspend(dev);

	return err;
}

static int acpm_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct acpm_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct acpm_tmu_priv *priv = sensor->priv;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	struct device *dev = priv->dev;
	int acpm_temp = 0, ret;

	/*
	 * Fast path: check locklessly if the sensor is enabled to avoid
	 * expensive runtime PM operations when it is disabled. Any race with
	 * concurrent disabling is caught by the second check under the lock
	 * after PM resume.
	 */
	if (!READ_ONCE(sensor->enabled))
		return -ENODATA;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	scoped_guard(mutex, &sensor->lock) {
		if (!sensor->enabled) {
			ret = -ENODATA;
		} else {
			ret = ops->read_temp(handle, priv->mbox_chan_id,
					     sensor->group->id, &acpm_temp);
		}
	}

	pm_runtime_put_autosuspend(dev);

	if (ret)
		return ret;

	*temp = acpm_temp * MILLIDEGREE_PER_DEGREE;

	return 0;
}

static int acpm_tmu_update_thresholds(struct acpm_tmu_sensor *sensor,
				      u8 thresholds[2], u8 inten)
{
	struct acpm_tmu_priv *priv = sensor->priv;
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	unsigned int mbox_chan_id = priv->mbox_chan_id;
	u8 acpm_sensor_id = sensor->group->id;
	bool was_enabled;
	int ret, restore_ret;

	guard(mutex)(&sensor->lock);

	was_enabled = sensor->enabled;

	if (was_enabled) {
		ret = acpm_tmu_op_tz_control(sensor, false);
		if (ret)
			return ret;
	}

	ret = ops->set_threshold(handle, mbox_chan_id, acpm_sensor_id,
				 thresholds, 2);
	if (!ret) {
		ret = ops->set_interrupt_enable(handle, mbox_chan_id,
						acpm_sensor_id, inten);
	}

	if (was_enabled) {
		restore_ret = acpm_tmu_op_tz_control(sensor, true);
		if (restore_ret)
			dev_err(priv->dev, "Failed to restore sensor state: %d\n",
				restore_ret);
		if (!ret)
			ret = restore_ret;
	}

	return ret;
}

static int acpm_tmu_set_trips(struct thermal_zone_device *tz, int low, int high)
{
	struct acpm_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct acpm_tmu_priv *priv = sensor->priv;
	struct device *dev = priv->dev;
	u8 thresholds[2] = {};
	u8 inten = 0;
	int ret;

	/* If a valid lower bound exists, set the threshold and enable its interrupt */
	if (low > -INT_MAX) {
		thresholds[0] = clamp_val(low / MILLIDEGREE_PER_DEGREE, 0, 255);
		inten |= BIT(0);
	}

	/* If a valid upper bound exists, set the threshold and enable its interrupt */
	if (high < INT_MAX) {
		thresholds[1] = clamp_val(high / MILLIDEGREE_PER_DEGREE, 0, 255);
		inten |= BIT(1);
	}

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	ret = acpm_tmu_update_thresholds(sensor, thresholds, inten);

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static int acpm_tmu_change_mode(struct thermal_zone_device *tz,
				enum thermal_device_mode mode)
{
	struct acpm_tmu_sensor *sensor = thermal_zone_device_priv(tz);
	struct acpm_tmu_priv *priv = sensor->priv;
	struct device *dev = priv->dev;
	bool enable = (mode == THERMAL_DEVICE_ENABLED);
	int ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return ret;

	scoped_guard(mutex, &sensor->lock) {
		ret = 0;
		if (sensor->enabled != enable)
			ret = acpm_tmu_op_tz_control(sensor, enable);
		if (!ret)
			sensor->mode = mode;
	}

	pm_runtime_put_autosuspend(dev);

	return ret;
}

static const struct thermal_zone_device_ops acpm_tmu_sensor_ops = {
	.change_mode = acpm_tmu_change_mode,
	.get_temp = acpm_tmu_get_temp,
	.set_trips = acpm_tmu_set_trips,
};

static bool acpm_tmu_clear_pending_irq(struct acpm_tmu_sensor *sensor)
{
	struct acpm_tmu_priv *priv = sensor->priv;
	const struct acpm_tmu_ops *ops = &priv->handle->ops->tmu;
	unsigned long mask = sensor->group->mask;
	bool pending_irq = false;
	int i, ret;
	u32 val;

	guard(mutex)(&sensor->lock);

	for_each_set_bit(i, &mask, EXYNOS_TMU_SENSORS_MAX_COUNT) {
		ret = regmap_field_read(priv->regmap_fields[i], &val);
		if (ret)
			return false;

		if (val) {
			pending_irq = true;
			break;
		}
	}

	if (!pending_irq)
		return false;

	ret = ops->clear_tz_irq(priv->handle, priv->mbox_chan_id,
				sensor->group->id);
	if (ret) {
		dev_err_ratelimited(priv->dev,
				    "Sensor %u: failed to clear IRQ (%d)\n",
				    sensor->group->id, ret);
		return false;
	}

	return true;
}

static irqreturn_t acpm_tmu_thread_fn(int irq, void *id)
{
	struct acpm_tmu_priv *priv = id;
	struct device *dev = priv->dev;
	bool handled = false;
	int i, ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err_ratelimited(dev, "Failed to resume: %d\n", ret);
		return IRQ_NONE;
	}

	for (i = 0; i < priv->num_sensors; i++) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];

		if (!sensor->tzd || !acpm_tmu_clear_pending_irq(sensor))
			continue;

		handled = true;

		thermal_zone_device_update(sensor->tzd,
					   THERMAL_EVENT_UNSPECIFIED);
	}

	pm_runtime_put_autosuspend(dev);

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static const struct of_device_id acpm_tmu_match[] = {
	{ .compatible = "google,gs101-tmu-top" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, acpm_tmu_match);

static int acpm_tmu_probe(struct platform_device *pdev)
{
	const struct acpm_tmu_driver_data *data = &acpm_tmu_gs101;
	struct acpm_handle *acpm_handle;
	struct device *dev = &pdev->dev;
	struct acpm_tmu_priv *priv;
	struct regmap *regmap;
	void __iomem *base;
	int i, ret;

	acpm_handle = devm_acpm_get_by_phandle(dev);
	if (IS_ERR(acpm_handle))
		return dev_err_probe(dev, PTR_ERR(acpm_handle),
				     "Failed to get ACPM handle\n");

	priv = devm_kzalloc(dev,
			    struct_size(priv, sensors, data->num_sensor_groups),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->handle = acpm_handle;
	priv->mbox_chan_id = data->mbox_chan_id;
	priv->num_sensors = data->num_sensor_groups;

	platform_set_drvdata(pdev, priv);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base), "Failed to ioremap resource\n");

	regmap = devm_regmap_init_mmio(dev, base, &gs101_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to init regmap\n");

	ret = devm_regmap_field_bulk_alloc(dev, regmap, priv->regmap_fields,
					   data->reg_fields, REG_INTPEND_COUNT);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to allocate regmap fields\n");

	priv->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->clk))
		return dev_err_probe(dev, PTR_ERR(priv->clk),
				     "Failed to get the clock\n");

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	pm_runtime_set_autosuspend_delay(dev, 100);
	pm_runtime_use_autosuspend(dev);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable runtime PM\n");

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to resume device\n");

	ret = acpm_handle->ops->tmu.init(acpm_handle, priv->mbox_chan_id);
	if (ret) {
		ret = dev_err_probe(dev, ret, "Failed to init TMU\n");
		goto out_pm_put;
	}

	for (i = 0; i < priv->num_sensors; i++) {
		struct acpm_tmu_sensor *sensor = &priv->sensors[i];
		struct thermal_zone_device *tzd;

		mutex_init(&sensor->lock);
		sensor->group = &data->sensor_groups[i];
		sensor->priv = priv;

		tzd = devm_thermal_of_zone_register(dev, i, sensor,
						    &acpm_tmu_sensor_ops);
		if (IS_ERR(tzd)) {
			ret = PTR_ERR(tzd);
			if (ret == -ENODEV) {
				dev_dbg(dev, "Sensor %d not used in DT, skipping\n", i);
				continue;
			}

			ret = dev_err_probe(dev, ret, "Failed to register sensor %d\n", i);
			goto out_pm_put;
		}
		sensor->tzd = tzd;

		devm_thermal_add_hwmon_sysfs(dev, sensor->tzd);
	}

	ret = devm_request_threaded_irq(dev, priv->irq, NULL,
					acpm_tmu_thread_fn, IRQF_ONESHOT,
					dev_name(dev), priv);

out_pm_put:
	pm_runtime_put_autosuspend(dev);
	return ret;
}

static int acpm_tmu_suspend(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	int ret, restore_ret;

	disable_irq(priv->irq);

	ret = acpm_tmu_control(priv, false);
	if (ret)
		goto err_restore_sensors;

	/* APB clock not required for this specific msg */
	ret = ops->suspend(handle, priv->mbox_chan_id);
	if (ret)
		goto err_restore_sensors;

	ret = pm_runtime_force_suspend(dev);
	if (ret)
		goto err_resume_acpm;

	return 0;

err_resume_acpm:
	restore_ret = ops->resume(handle, priv->mbox_chan_id);
	if (restore_ret)
		dev_err(dev, "Failed to resume ACPM after force suspend failure: %d\n",
			restore_ret);

err_restore_sensors:
	restore_ret = acpm_tmu_control(priv, true);
	if (restore_ret)
		dev_err(dev, "Failed to re-enable TMU after suspend failure: %d\n",
			restore_ret);

	enable_irq(priv->irq);

	return ret;
}

static int acpm_tmu_resume(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);
	struct acpm_handle *handle = priv->handle;
	const struct acpm_tmu_ops *ops = &handle->ops->tmu;
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		goto out_enable_irq;

	/* APB clock not required for this specific msg */
	ret = ops->resume(handle, priv->mbox_chan_id);
	if (ret)
		goto out_enable_irq;

	ret = acpm_tmu_control(priv, true);

out_enable_irq:
	enable_irq(priv->irq);

	return ret;
}

static int acpm_tmu_runtime_suspend(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);

	clk_disable_unprepare(priv->clk);

	return 0;
}

static int acpm_tmu_runtime_resume(struct device *dev)
{
	struct acpm_tmu_priv *priv = dev_get_drvdata(dev);

	return clk_prepare_enable(priv->clk);
}

static const struct dev_pm_ops acpm_tmu_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(acpm_tmu_suspend, acpm_tmu_resume)
	RUNTIME_PM_OPS(acpm_tmu_runtime_suspend, acpm_tmu_runtime_resume, NULL)
};

static struct platform_driver acpm_tmu_driver = {
	.driver = {
		.name   = "exynos-acpm-tmu",
		.pm     = pm_ptr(&acpm_tmu_pm_ops),
		.of_match_table = acpm_tmu_match,
	},
	.probe = acpm_tmu_probe,
};
module_platform_driver(acpm_tmu_driver);

MODULE_AUTHOR("Tudor Ambarus <tudor.ambarus@linaro.org>");
MODULE_DESCRIPTION("Samsung Exynos ACPM TMU Driver");
MODULE_LICENSE("GPL");
