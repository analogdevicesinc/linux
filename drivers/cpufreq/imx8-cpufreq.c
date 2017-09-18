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
#include <linux/cpu_cooling.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/syscalls.h>
#include <soc/imx/fsl_sip.h>

#define MAX_CLUSTER_NUM	2

static struct delayed_work cpufreq_governor_daemon;

static DEFINE_SPINLOCK(cpufreq_psci_lock);

struct imx8_cpufreq {
	struct clk	*cpu_clk;
};

struct imx8_cpufreq cluster_freq[MAX_CLUSTER_NUM];
static struct cpufreq_frequency_table *freq_table[MAX_CLUSTER_NUM];
static unsigned int transition_latency[MAX_CLUSTER_NUM];
struct device *cpu_dev;
static struct thermal_cooling_device *cdev[2];

static void cpufreq_governor_daemon_handler(struct work_struct *work)
{
	int fd, i;
	unsigned char cluster_governor[MAX_CLUSTER_NUM][54] = {
		"/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor",
		"",
	};

	/* generate second cluster's cpufreq governor path */
	sprintf(cluster_governor[MAX_CLUSTER_NUM - 1],
		"%s%d%s", "/sys/devices/system/cpu/cpu", num_online_cpus() - 1,
		"/cpufreq/scaling_governor");

	for (i = 0; i < MAX_CLUSTER_NUM; i++) {
		fd = sys_open((const char __user __force *)cluster_governor[i],
				O_RDWR, 0700);
		if (fd >= 0) {
			sys_write(fd, "schedutil", strlen("schedutil"));
			sys_close(fd);
			pr_info("switch cluster %d cpu-freq governor to schedutil\n",
				i);
		} else {
			/* re-schedule if sys write is NOT ready */
			schedule_delayed_work(&cpufreq_governor_daemon,
				msecs_to_jiffies(3000));
			break;
		}
	}
}

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
	policy->suspend_freq = policy->max;

	pr_info("%s: cluster %d running at freq %d MHz, suspend freq %d MHz\n",
		__func__, cluster_id, policy->cur / 1000,
		policy->suspend_freq / 1000);

	return ret;
}

static void imx8_cpufreq_ready(struct cpufreq_policy *policy)
{
	struct device_node *np = of_get_cpu_node(policy->cpu, NULL);
	unsigned int cluster_id = topology_physical_package_id(policy->cpu);

	if (of_find_property(np, "#cooling-cells", NULL)) {
		cdev[cluster_id] = of_cpufreq_cooling_register(np, policy);

		if (IS_ERR(cdev[cluster_id]) && PTR_ERR(cdev[cluster_id]) != -ENOSYS) {
			pr_err("cpu%d is not running as cooling device: %ld\n",
					policy->cpu, PTR_ERR(cdev[cluster_id]));

			cdev[cluster_id] = NULL;
		}
	}

	of_node_put(np);
}

static struct cpufreq_driver imx8_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = imx8_set_target,
	.get = cpufreq_generic_get,
	.init = imx8_cpufreq_init,
	.name = "imx8-cpufreq",
	.attr = cpufreq_generic_attr,
	.ready = imx8_cpufreq_ready,
#ifdef CONFIG_PM
	.suspend = cpufreq_generic_suspend,
#endif
};

static int imx8_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int ret = 0;
	int i, cluster_id;
	struct device *first_cpu_dev = NULL;

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

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table[cluster_id]);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_opp;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency[cluster_id]))
		transition_latency[cluster_id] = CPUFREQ_ETERNAL;

	/* init next cluster if there is */
	for (i = 1; i < num_online_cpus(); i++) {
		if (topology_physical_package_id(i) == topology_physical_package_id(0))
			continue;

		INIT_DELAYED_WORK(&cpufreq_governor_daemon,
			cpufreq_governor_daemon_handler);
		schedule_delayed_work(&cpufreq_governor_daemon,
			msecs_to_jiffies(3000));
		first_cpu_dev = cpu_dev;
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
			goto put_node;
		}

		ret = dev_pm_opp_of_add_table(cpu_dev);
		if (ret < 0) {
			dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
			goto put_node;
		}

		ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table[cluster_id]);
		if (ret) {
			dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
			goto out_free_opp;
		}

		if (of_property_read_u32(np, "clock-latency", &transition_latency[cluster_id]))
			transition_latency[cluster_id] = CPUFREQ_ETERNAL;
		break;
	}

	ret = cpufreq_register_driver(&imx8_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed register driver: %d\n", ret);
		if (cluster_id > 0 && first_cpu_dev != NULL) {
			dev_pm_opp_free_cpufreq_table(first_cpu_dev, &freq_table[0]);
			dev_pm_opp_of_remove_table(first_cpu_dev);
		}
		goto free_freq_table;
	}

	of_node_put(np);

	return 0;

free_freq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table[cluster_id]);
out_free_opp:
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
