/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <soc/imx/fsl_sip.h>

#define MAX_CLUSTER_NUM	2

static DEFINE_SPINLOCK(cpufreq_psci_lock);

struct imx8_cpufreq {
	struct clk	*cpu_clk;
};

struct imx8_cpufreq cluster_freq[MAX_CLUSTER_NUM];
static struct cpufreq_frequency_table *freq_table[MAX_CLUSTER_NUM];
static unsigned int transition_latency[MAX_CLUSTER_NUM];
static unsigned int suspend_freq[MAX_CLUSTER_NUM];
static bool free_opp;
struct device *cpu_dev;

static int imx8_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct arm_smccc_res res;
	unsigned int old_freq, new_freq;
	unsigned int cluster_id = topology_physical_package_id(policy->cpu);

	new_freq = freq_table[cluster_id][index].frequency;
	old_freq = policy->cur;

	dev_dbg(cpu_dev, "%u MHz --> %u MHz\n",
		old_freq / 1000, new_freq / 1000);

	spin_lock(&cpufreq_psci_lock);
	arm_smccc_smc(FSL_SIP_CPUFREQ, FSL_SIP_SET_CPUFREQ,
		cluster_id, new_freq * 1000, 0, 0, 0, 0, &res);
	spin_unlock(&cpufreq_psci_lock);

	/*
	 * As we can only set CPU clock rate in ATF, clock
	 * framework does NOT know CPU clock rate is changed,
	 * so here do clk_get_rate once to update CPU clock
	 * rate, otherwise cat /sys/kernel/debug/clk/xxx/clk_rate
	 * will return incorrect rate as it does NOT do a
	 * recalculation.
	 */
	clk_get_rate(cluster_freq[cluster_id].cpu_clk);

	return 0;
}

static int imx8_cpufreq_init(struct cpufreq_policy *policy)
{
	int cluster_id = topology_physical_package_id(policy->cpu);
	int ret = 0;

	policy->clk = cluster_freq[cluster_id].cpu_clk;
	policy->cur = clk_get_rate(cluster_freq[cluster_id].cpu_clk) / 1000;
	policy->suspend_freq = suspend_freq[cluster_id];

	pr_info("%s: cluster %d running at freq %d MHz\n",
		__func__, cluster_id, policy->cur / 1000);
	/*
	 * The driver only supports the SMP configuartion where all processors
	 * share the clock and voltage and clock.
	 */
	cpumask_copy(policy->cpus, topology_core_cpumask(policy->cpu));

	ret = cpufreq_table_validate_and_show(policy, freq_table[cluster_id]);
	if (ret) {
		pr_err("%s: invalid frequency table: %d\n", __func__, ret);
		return ret;
	}

	policy->cpuinfo.transition_latency = transition_latency[cluster_id];

	return ret;
}

static struct cpufreq_driver imx8_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = imx8_set_target,
	.get = cpufreq_generic_get,
	.init = imx8_cpufreq_init,
	.name = "imx8-cpufreq",
	.attr = cpufreq_generic_attr,
#ifdef CONFIG_PM
	.suspend = cpufreq_generic_suspend,
#endif
};

static int imx8_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int num, ret = 0;
	int i, cluster_id;

	cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_err("failed to get cpu device 0\n");
		return -ENODEV;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu 0 node\n");
		return -ENODEV;
	}

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret < 0) {
		dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
		goto put_node;
	}

	cluster_id = topology_physical_package_id(0);
	cluster_freq[cluster_id].cpu_clk = devm_clk_get(cpu_dev, NULL);
	if (IS_ERR(cluster_freq[cluster_id].cpu_clk)) {
		dev_err(cpu_dev, "failed to get cluster %d clock\n", cluster_id);
		ret = -ENOENT;
		goto put_node;
	}

	/*
	 * We expect an OPP table supplied by platform.
	 * Just, in case the platform did not supply the OPP
	 * table, it will try to get it.
	 */
	num = dev_pm_opp_get_opp_count(cpu_dev);
	if (num < 0) {
		ret = dev_pm_opp_of_add_table(cpu_dev);
		if (ret < 0) {
			dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
			goto put_node;
		}

		/* Because we have added the OPPs here, we must free them */
		free_opp = true;

		num = dev_pm_opp_get_opp_count(cpu_dev);
		if (num < 0) {
			ret = num;
			dev_err(cpu_dev, "no OPP table is found: %d\n", ret);
			goto out_free_opp;
		}
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table[cluster_id]);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto put_node;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency[cluster_id]))
		transition_latency[cluster_id] = CPUFREQ_ETERNAL;

	/* use MAX freq to suspend */
	suspend_freq[cluster_id] = freq_table[cluster_id][num - 1].frequency;

	/* init next cluster if there is */
	for (i = 1; i < num_online_cpus(); i++) {
		if (topology_physical_package_id(i) == topology_physical_package_id(0))
			continue;
		cpu_dev = get_cpu_device(i);
		if (!cpu_dev) {
			pr_err("failed to get cpu device %d\n", i);
				return -ENODEV;
		}

		np = of_node_get(cpu_dev->of_node);
		if (!np) {
			pr_warn("failed to find cpu %d node\n", i);
			ret = -ENODEV;
			goto put_node;
		}

		cluster_id = topology_physical_package_id(i);
		cluster_freq[cluster_id].cpu_clk = devm_clk_get(cpu_dev, NULL);
		if (IS_ERR(cluster_freq[cluster_id].cpu_clk)) {
			dev_err(cpu_dev, "failed to get cluster %d clock\n", cluster_id);
			ret = -ENOENT;
			goto out_free_opp;
		}

		/*
		 * We expect an OPP table supplied by platform.
		 * Just, in case the platform did not supply the OPP
		 * table, it will try to get it.
		 */
		num = dev_pm_opp_get_opp_count(cpu_dev);
		if (num < 0) {
			ret = dev_pm_opp_of_add_table(cpu_dev);
			if (ret < 0) {
				dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
				goto put_node;
			}

			/* Because we have added the OPPs here, we must free them */
			free_opp = true;

			num = dev_pm_opp_get_opp_count(cpu_dev);
			if (num < 0) {
				ret = num;
				dev_err(cpu_dev, "no OPP table is found: %d\n", ret);
				goto out_free_opp;
			}
		}

		ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table[cluster_id]);
		if (ret) {
			dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
			goto out_free_opp;
		}

		if (of_property_read_u32(np, "clock-latency", &transition_latency[cluster_id]))
			transition_latency[cluster_id] = CPUFREQ_ETERNAL;

		/* use MAX freq to suspend */
		suspend_freq[cluster_id] = freq_table[cluster_id][num - 1].frequency;
		break;
	}

	ret = cpufreq_register_driver(&imx8_cpufreq_driver);
	if (ret)
		dev_err(cpu_dev, "failed register driver: %d\n", ret);

out_free_opp:
	if (free_opp)
		dev_pm_opp_of_remove_table(cpu_dev);
put_node:
	of_node_put(np);
	return ret;
}

static int imx8_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&imx8_cpufreq_driver);

	return 0;
}

static struct platform_driver imx8_cpufreq_platdrv = {
	.driver = {
		.name	= "imx8-cpufreq",
	},
	.probe		= imx8_cpufreq_probe,
	.remove		= imx8_cpufreq_remove,
};
module_platform_driver(imx8_cpufreq_platdrv);

MODULE_AUTHOR("Anson Huang <Anson.Huang@nxp.com>");
MODULE_DESCRIPTION("NXP i.MX8 cpufreq driver");
MODULE_LICENSE("GPL");
