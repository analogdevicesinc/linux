/*
 * Copyright (C) 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpu_cooling.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

#define DC_VOLTAGE_MIN		900000
#define DC_VOLTAGE_MAX		1000000

static struct device *cpu_dev;
static bool free_opp;
static struct cpufreq_frequency_table *freq_table;
static struct mutex set_cpufreq_lock;
static unsigned int transition_latency;
static struct clk *a53_clk;
static struct clk *arm_a53_src_clk;
static struct clk *arm_pll_clk;
static struct clk *arm_pll_out_clk;
static struct clk *sys1_pll_800m_clk;
struct thermal_cooling_device *cdev;
static struct regulator *dc_reg;

static int imx8mq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct dev_pm_opp *opp;
	unsigned long freq_hz;
	unsigned int old_freq, new_freq;
	int ret;

	mutex_lock(&set_cpufreq_lock);

	new_freq = freq_table[index].frequency;
	freq_hz = new_freq * 1000;
	old_freq = policy->cur;

	rcu_read_lock();
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(cpu_dev, "failed to find OPP for %ld\n", freq_hz);
		mutex_unlock(&set_cpufreq_lock);
		return PTR_ERR(opp);
	}
	rcu_read_unlock();

	dev_dbg(cpu_dev, "%u MHz --> %u MHz\n",
		old_freq / 1000, new_freq / 1000);

	if (new_freq == policy->max) {
		if (!IS_ERR(dc_reg)) {
			ret = regulator_set_voltage_tol(dc_reg, DC_VOLTAGE_MAX, 0);
			if (ret) {
				dev_err(cpu_dev, "failed to scale dc_reg up: %d\n", ret);
				mutex_unlock(&set_cpufreq_lock);
				return ret;
			}
		}
	}

	clk_set_parent(arm_a53_src_clk, sys1_pll_800m_clk);
	clk_set_rate(arm_pll_clk, new_freq * 1000);
	clk_set_parent(arm_a53_src_clk, arm_pll_out_clk);

	if (old_freq == policy->max) {
		if (!IS_ERR(dc_reg)) {
			ret = regulator_set_voltage_tol(dc_reg, DC_VOLTAGE_MIN, 0);
			if (ret) {
				dev_err(cpu_dev, "failed to scale dc_reg down: %d\n", ret);
				mutex_unlock(&set_cpufreq_lock);
				return ret;
			}
		}
	}

	/* Ensure the arm clock divider is what we expect */
	ret = clk_set_rate(a53_clk, new_freq * 1000);
	if (ret)
		dev_err(cpu_dev, "failed to set clock rate: %d\n", ret);

	mutex_unlock(&set_cpufreq_lock);
	return ret;
}

static void imx8mq_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct device_node *np = of_get_cpu_node(policy->cpu, NULL);

	if (of_find_property(np, "#cooling-cells", NULL)) {
		cdev = of_cpufreq_cooling_register(np, policy);

		if (IS_ERR(cdev) && PTR_ERR(cdev) != -ENOSYS) {
			pr_err("cpu%d is not running as cooling device: %ld\n",
					policy->cpu, PTR_ERR(cdev));

			cdev = NULL;
		}
	}

	of_node_put(np);
}

static int imx8mq_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	policy->clk = a53_clk;
	policy->cur = clk_get_rate(a53_clk) / 1000;

	ret = cpufreq_generic_init(policy, freq_table, transition_latency);
	if (ret) {
		dev_err(cpu_dev, "imx8mq cpufreq init failed!\n");
		return ret;
	}

	policy->suspend_freq = policy->max;

	return 0;
}

static struct cpufreq_driver imx8mq_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = imx8mq_set_target,
	.get = cpufreq_generic_get,
	.init = imx8mq_cpufreq_init,
	.name = "imx8mq-cpufreq",
	.ready = imx8mq_cpufreq_ready,
	.attr = cpufreq_generic_attr,
#ifdef CONFIG_PM
	.suspend = cpufreq_generic_suspend,
#endif
};

static int imx8mq_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu0 node\n");
		return -ENOENT;
	}

	a53_clk = clk_get(cpu_dev, "a53");
	arm_a53_src_clk = clk_get(cpu_dev, "arm_a53_src");
	arm_pll_clk = clk_get(cpu_dev, "arm_pll");
	arm_pll_out_clk = clk_get(cpu_dev, "arm_pll_out");
	sys1_pll_800m_clk = clk_get(cpu_dev, "sys1_pll_800m");
	if (IS_ERR(a53_clk) || IS_ERR(arm_a53_src_clk)
		|| IS_ERR(arm_pll_out_clk) || IS_ERR(arm_pll_clk)
		|| IS_ERR(sys1_pll_800m_clk)) {
		dev_err(cpu_dev, "failed to get clocks\n");
		ret = -ENOENT;
		goto put_clk;
	}

	dc_reg = regulator_get_optional(cpu_dev, "dc");

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret < 0) {
		dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
		goto put_clk;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_opp;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	mutex_init(&set_cpufreq_lock);

	ret = cpufreq_register_driver(&imx8mq_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed register driver: %d\n", ret);
		goto free_freq_table;
	}

	of_node_put(np);
	dev_info(cpu_dev, "registered imx8mq-cpufreq\n");

	return 0;

free_freq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_free_opp:
	dev_pm_opp_of_remove_table(cpu_dev);
put_clk:
	if (!IS_ERR(a53_clk))
		clk_put(a53_clk);
	if (!IS_ERR(arm_a53_src_clk))
		clk_put(arm_a53_src_clk);
	if (!IS_ERR(arm_pll_clk))
		clk_put(arm_pll_clk);
	if (!IS_ERR(arm_pll_out_clk))
		clk_put(arm_pll_out_clk);
	if (!IS_ERR(sys1_pll_800m_clk))
		clk_put(sys1_pll_800m_clk);
	of_node_put(np);
	return ret;
}

static int imx8mq_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_cooling_unregister(cdev);
	cpufreq_unregister_driver(&imx8mq_cpufreq_driver);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
	if (free_opp)
		dev_pm_opp_of_remove_table(cpu_dev);
	clk_put(a53_clk);
	clk_put(arm_a53_src_clk);
	clk_put(arm_pll_clk);
	clk_put(arm_pll_out_clk);
	clk_put(sys1_pll_800m_clk);

	return 0;
}

static struct platform_driver imx8mq_cpufreq_platdrv = {
	.driver = {
		.name	= "imx8mq-cpufreq",
	},
	.probe		= imx8mq_cpufreq_probe,
	.remove		= imx8mq_cpufreq_remove,
};
module_platform_driver(imx8mq_cpufreq_platdrv);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("Freescale i.MX8MQ cpufreq driver");
MODULE_LICENSE("GPL");
