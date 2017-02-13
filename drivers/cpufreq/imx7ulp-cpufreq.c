 /*
 * Copyright 2017 NXP.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

#define MAX_RUN_FREQ	528000
#define SMC_PMPROT	0x8
#define SMC_PMCTRL	0x10

static struct clk *arm_clk;
static struct clk *core_div;
static struct clk *sys_sel;
static struct clk *hsrun_sys_sel;
static struct clk *hsrun_core;
static struct clk *spll_pfd0;
static struct clk *spll_sel;
static struct clk *firc_clk;

static struct pm_qos_request pm_qos_hsrun;

static void __iomem *smc_base;

static struct regulator *arm_reg;
static struct device *cpu_dev;
static struct cpufreq_frequency_table *freq_table;
static unsigned int transition_latency;
static struct mutex set_cpufreq_lock;

static int imx7ulp_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct dev_pm_opp *opp;
	unsigned long freq_hz, volt, volt_old;
	unsigned int old_freq, new_freq;
	u32 val;
	int ret;

	mutex_lock(&set_cpufreq_lock);

	new_freq = freq_table[index].frequency;
	freq_hz = new_freq * 1000;
	old_freq = clk_get_rate(arm_clk) / 1000;

	rcu_read_lock();
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(cpu_dev, "failed to find OPP for %ld\n", freq_hz);
		mutex_unlock(&set_cpufreq_lock);
		return PTR_ERR(opp);
	}
	volt = dev_pm_opp_get_voltage(opp);

	rcu_read_unlock();
	volt_old = regulator_get_voltage(arm_reg);

	dev_dbg(cpu_dev, "%u MHz, %ld mV --> %u MHz, %ld mV\n",
		old_freq / 1000, volt_old / 1000,
		new_freq / 1000, volt / 1000);

	/* Scaling up? scale voltage before frequency */
	if (new_freq > old_freq) {
		ret = regulator_set_voltage_tol(arm_reg, volt, 0);
		if (ret) {
			dev_err(cpu_dev, "failed to scale vddarm up: %d\n", ret);
			mutex_unlock(&set_cpufreq_lock);
			return ret;
		}
	}

	/* before changing pll_arm rate, change the arm_src's soure
	 * to firc clk first.
	 */
	if (new_freq > MAX_RUN_FREQ) {
		pm_qos_add_request(&pm_qos_hsrun, PM_QOS_CPU_DMA_LATENCY, 0);
		clk_set_parent(sys_sel, firc_clk);
		/* switch to HSRUN mode */
		val = readl_relaxed(smc_base + SMC_PMCTRL);
		val |= (0x3 << 8);
		writel_relaxed(val, smc_base + SMC_PMCTRL);
		/* change the clock rate in HSRUN */
		clk_set_rate(spll_pfd0, new_freq * 1000);
		clk_set_parent(hsrun_sys_sel, spll_sel);
		clk_set_parent(arm_clk, hsrun_core);
	} else {
		/* change the HSRUN clock to firc */
		clk_set_parent(hsrun_sys_sel, firc_clk);
		/* switch to RUN mode */
		val = readl_relaxed(smc_base + SMC_PMCTRL);
		val &= ~(0x3 << 8);
		writel_relaxed(val, smc_base + SMC_PMCTRL);

		clk_set_rate(spll_pfd0, new_freq * 1000);
		clk_set_parent(sys_sel, spll_sel);
		clk_set_parent(arm_clk, core_div);
		pm_qos_remove_request(&pm_qos_hsrun);
	}

	/* scaling down? scaling voltage after frequency */
	if (new_freq < old_freq) {
		ret = regulator_set_voltage_tol(arm_reg, volt, 0);
		if (ret) {
			dev_warn(cpu_dev, "failed to scale vddarm down: %d\n", ret);
			ret = 0;
		}
	}

	mutex_unlock(&set_cpufreq_lock);
	return 0;
}

static int imx7ulp_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;
	policy->clk = arm_clk;
	policy->cur = clk_get_rate(arm_clk) / 1000;
	policy->suspend_freq = freq_table[0].frequency;

	ret = cpufreq_generic_init(policy, freq_table, transition_latency);

	if (ret) {
		dev_err(cpu_dev, "imx7ulp cpufreq init failed\n");
		return ret;
	}

	return 0;
}

static struct cpufreq_driver imx7ulp_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = imx7ulp_set_target,
	.get = cpufreq_generic_get,
	.init = imx7ulp_cpufreq_init,
	.name = "imx7ulp-cpufreq",
	.attr = cpufreq_generic_attr,
#ifdef CONFIG_PM
	.suspend = cpufreq_generic_suspend,
#endif
};

static int imx7ulp_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENOENT;
	}

	np = of_find_compatible_node(NULL, NULL, "fsl,imx7ulp-smc1");
	smc_base = of_iomap(np, 0);
	of_node_put(np);
	if (!smc_base)
		return -ENOMEM;

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find the cpu0 node\n");
		return -ENOENT;
	}

	arm_clk = clk_get(cpu_dev, "arm");
	sys_sel = clk_get(cpu_dev, "sys_sel");
	core_div = clk_get(cpu_dev, "core_div");
	hsrun_sys_sel = clk_get(cpu_dev, "hsrun_sys_sel");
	hsrun_core = clk_get(cpu_dev, "hsrun_core");
	spll_pfd0 = clk_get(cpu_dev, "spll_pfd0");
	spll_sel = clk_get(cpu_dev, "spll_sel");
	firc_clk = clk_get(cpu_dev, "firc");

	if (IS_ERR(arm_clk) || IS_ERR(sys_sel) || IS_ERR(spll_sel) ||
	    IS_ERR(spll_sel) || IS_ERR(firc_clk) || IS_ERR(hsrun_sys_sel) ||
	    IS_ERR(hsrun_core)) {
		dev_err(cpu_dev, "failed to get cpu clock\n");
		ret = -ENOENT;
		goto put_clk;
	}

	arm_reg = regulator_get(cpu_dev, "arm");
	if (IS_ERR(arm_reg)) {
		dev_err(cpu_dev, "failed to get regulator\n");
		ret = -ENOENT;
		goto put_reg;
	}

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret < 0) {
		dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
		goto put_reg;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table\n");
		goto put_reg;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	mutex_init(&set_cpufreq_lock);
	ret = cpufreq_register_driver(&imx7ulp_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed to register driver\n");
		goto free_opp_table;
	}

	return 0;

free_opp_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
put_reg:
	regulator_put(arm_reg);
put_clk:
	if (!IS_ERR(arm_clk))
		clk_put(arm_clk);
	if (!IS_ERR(sys_sel))
		clk_put(sys_sel);
	if (!IS_ERR(core_div))
		clk_put(core_div);
	if (!IS_ERR(hsrun_sys_sel))
		clk_put(hsrun_sys_sel);
	if (!IS_ERR(hsrun_core))
		clk_put(hsrun_core);
	if (!IS_ERR(spll_pfd0))
		clk_put(spll_pfd0);
	if (!IS_ERR(spll_sel))
		clk_put(spll_sel);
	if (!IS_ERR(firc_clk))
		clk_put(firc_clk);

	return ret;
}

static int imx7ulp_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&imx7ulp_cpufreq_driver);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);

	regulator_put(arm_reg);
	clk_put(arm_clk);
	clk_put(sys_sel);
	clk_put(core_div);
	clk_put(hsrun_sys_sel);
	clk_put(hsrun_core);
	clk_put(spll_pfd0);
	clk_put(spll_sel);
	clk_put(firc_clk);

	return 0;
}

static struct platform_driver imx7ulp_cpufreq_platdrv = {
	.driver = {
		.name	= "imx7ulp-cpufreq",
		.owner	= THIS_MODULE,
	},
	.probe		= imx7ulp_cpufreq_probe,
	.remove		= imx7ulp_cpufreq_remove,
};

module_platform_driver(imx7ulp_cpufreq_platdrv);

MODULE_DESCRIPTION("NXP i.MX7ULP cpufreq driver");
MODULE_LICENSE("GPL v2");

