/*
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/arm-smccc.h>
#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/cpumask.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp.h>
#include <linux/suspend.h>
#include <linux/sys_soc.h>

#define FSL_SIP_DDR_DVFS                0xc2000004

#define HIGH_FREQ_3200MTS	0x0
#define AUDIO_FREQ_400MTS	0x1
#define LOW_BUS_FREQ_100MTS	0x2
#define LOW_BUS_FREQ_667MTS	0x1
#define WAIT_BUS_FREQ_DONE	0xf
#define DLL_ON_DRATE		667

static struct device *busfreq_dev;
static int low_bus_freq_mode;
static int audio_bus_freq_mode;
static int high_bus_freq_mode;
static int bus_freq_scaling_initialized;
static int bus_freq_scaling_is_active;
static int high_bus_count, audio_bus_count, low_bus_count;
static int cur_bus_freq_mode;
static int busfreq_suspended;
static bool cancel_reduce_bus_freq;

static unsigned int fsp_table[4];
static unsigned long origin_noc_rate;
static int low_bus_mode_fsp_index;
/* no bypass or dll off mode support if lowest fsp > 667mts */
static bool bypass_support = true;

static struct clk *dram_pll_clk;
static struct clk *dram_pll;
static struct clk *sys1_pll_800m;
static struct clk *sys1_pll_400m;
static struct clk *sys1_pll_100m;
static struct clk *sys1_pll_40m;
static struct clk *dram_alt_src;
static struct clk *dram_alt_root;
static struct clk *dram_core_clk;
static struct clk *dram_apb_src;
static struct clk *dram_apb_pre_div;
static struct clk *noc_div;
static struct clk *main_axi_src;
static struct clk *ahb_div;
static struct clk *osc_25m;
static struct clk *sys2_pll_333m;

static struct delayed_work low_bus_freq_handler;
static struct delayed_work bus_freq_daemon;

DEFINE_MUTEX(bus_freq_mutex);

static void update_bus_freq(int target_freq)
{
	struct arm_smccc_res res;
	u32 online_cpus = 0;
	int cpu = 0;

	local_irq_disable();

	for_each_online_cpu(cpu) {
		online_cpus |= (1 << (cpu * 8));
	}
	/* change the ddr freqency */
	arm_smccc_smc(FSL_SIP_DDR_DVFS, target_freq, online_cpus,
		0, 0, 0, 0, 0, &res);

	local_irq_enable();
}

static void reduce_bus_freq(void)
{
	u32 rate;

	high_bus_freq_mode = 0;

	/*
	 * below piece of code has some redundant part, keep
	 * it at present, we may need update the audio freq
	 * in the future if needed.
	 */
	if (audio_bus_count) {
		if (cur_bus_freq_mode == BUS_FREQ_HIGH) {
			if (bypass_support) {
				/* prepare the necessary clk before frequency change */
				clk_prepare_enable(sys1_pll_40m);
				clk_prepare_enable(dram_alt_root);
				clk_prepare_enable(sys1_pll_100m);

				update_bus_freq(low_bus_mode_fsp_index);

				clk_set_parent(dram_alt_src, sys1_pll_100m);
				clk_set_parent(dram_core_clk, dram_alt_root);
				clk_set_parent(dram_apb_src, sys1_pll_40m);
				clk_set_rate(dram_apb_pre_div, 20000000);
				clk_disable_unprepare(sys1_pll_100m);
				clk_disable_unprepare(sys1_pll_40m);
				clk_disable_unprepare(dram_alt_root);
			} else {
				update_bus_freq(low_bus_mode_fsp_index);
				/*
				 * the dram_apb and dram_core clk rate is changed
				 * in ATF side, below two lines of code is just used
				 * to update the clock tree info in kernel side.
				 */
				clk_set_rate(dram_apb_pre_div, 160000000);
				clk_get_rate(dram_pll);
			}
			/* change the NOC rate */
			if (of_machine_is_compatible("fsl,imx8mq"))
				clk_set_rate(noc_div, origin_noc_rate / 8);
			else
				clk_set_rate(noc_div, origin_noc_rate / 5);

			rate = clk_get_rate(ahb_div);
			if (rate == 0) {
				WARN_ON(1);
				return;
			}
			clk_set_rate(ahb_div, rate / 6);
			clk_set_parent(main_axi_src, osc_25m);
		}

		low_bus_freq_mode = 0;
		audio_bus_freq_mode = 1;
		cur_bus_freq_mode = BUS_FREQ_AUDIO;
	} else {
		if (cur_bus_freq_mode == BUS_FREQ_HIGH) {
			if (bypass_support) {
				/* prepare the necessary clk before frequency change */
				clk_prepare_enable(sys1_pll_40m);
				clk_prepare_enable(dram_alt_root);
				clk_prepare_enable(sys1_pll_100m);

				update_bus_freq(low_bus_mode_fsp_index);

				clk_set_parent(dram_alt_src, sys1_pll_100m);
				clk_set_parent(dram_core_clk, dram_alt_root);
				clk_set_parent(dram_apb_src, sys1_pll_40m);
				clk_set_rate(dram_apb_pre_div, 20000000);
				clk_disable_unprepare(sys1_pll_100m);
				clk_disable_unprepare(sys1_pll_40m);
				clk_disable_unprepare(dram_alt_root);
			} else {
				update_bus_freq(low_bus_mode_fsp_index);
				/*
				 * the dram_apb and dram_core clk rate is changed
				 * in ATF side, below two lines of code is just used
				 * to update the clock tree info in kernel side.
				 */
				clk_set_rate(dram_apb_pre_div, 160000000);
				clk_get_rate(dram_pll);
			}

			/* change the NOC rate */
			if (of_machine_is_compatible("fsl,imx8mq"))
				clk_set_rate(noc_div, origin_noc_rate / 8);
			else
				clk_set_rate(noc_div, origin_noc_rate / 5);

			rate = clk_get_rate(ahb_div);
			if (rate == 0) {
				WARN_ON(1);
				return;
			}
			clk_set_rate(ahb_div, rate / 6);
			clk_set_parent(main_axi_src, osc_25m);
		}

		low_bus_freq_mode = 1;
		audio_bus_freq_mode = 0;
		cur_bus_freq_mode = BUS_FREQ_LOW;
	}

	if (audio_bus_freq_mode)
		printk(KERN_DEBUG "ddrc freq set to audio bus mode\n");
	if (low_bus_freq_mode)
		printk(KERN_DEBUG "ddrc freq set to low bus mode\n");
}

static void reduce_bus_freq_handler(struct work_struct *work)
{
	mutex_lock(&bus_freq_mutex);

	if (!cancel_reduce_bus_freq)
		reduce_bus_freq();

	mutex_unlock(&bus_freq_mutex);
}

static int set_low_bus_freq(void)
{
	if (busfreq_suspended)
		return 0;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	cancel_reduce_bus_freq = false;

	/*
	 * check to see if we need to got from low bus
	 * freq mode to audio bus freq mode.
	 * If so, the change needs to be done immediately.
	 */
	if (audio_bus_count && low_bus_freq_mode)
		reduce_bus_freq();
	else
		schedule_delayed_work(&low_bus_freq_handler,
					usecs_to_jiffies(1000000));

	return 0;
}

static inline void cancel_low_bus_freq_handler(void)
{
	cancel_delayed_work(&low_bus_freq_handler);
	cancel_reduce_bus_freq = true;
}

static int set_high_bus_freq(int high_bus_freq)
{
	if (bus_freq_scaling_initialized || bus_freq_scaling_is_active)
		cancel_low_bus_freq_handler();

	if (busfreq_suspended)
		return 0;

	if (!bus_freq_scaling_initialized || !bus_freq_scaling_is_active)
		return 0;

	if (high_bus_freq_mode)
		return 0;

	if (bypass_support) {
		/*  enable the clks needed in frequency */
		clk_prepare_enable(sys1_pll_800m);
		clk_prepare_enable(dram_pll_clk);

		/* switch the DDR freqeuncy */
		update_bus_freq(HIGH_FREQ_3200MTS);

		/* correct the clock tree info */
		clk_set_parent(dram_apb_src, sys1_pll_800m);
		clk_set_rate(dram_apb_pre_div, 160000000);
		clk_set_parent(dram_core_clk, dram_pll_clk);
		clk_disable_unprepare(sys1_pll_800m);
		clk_disable_unprepare(dram_pll_clk);
	} else {
		/* switch the DDR freqeuncy */
		update_bus_freq(HIGH_FREQ_3200MTS);

		clk_set_rate(dram_apb_pre_div, 200000000);
		clk_get_rate(dram_pll);
	}

	clk_set_rate(noc_div, origin_noc_rate);
	clk_set_rate(ahb_div, 133333333);
	clk_set_parent(main_axi_src, sys2_pll_333m);

	high_bus_freq_mode = 1;
	audio_bus_freq_mode = 0;
	low_bus_freq_mode = 0;
	cur_bus_freq_mode = BUS_FREQ_HIGH;

	if (high_bus_freq_mode)
		printk(KERN_DEBUG "ddrc freq set to high bus mode\n");

	return 0;
}

void request_bus_freq(enum bus_freq_mode mode)
{
	mutex_lock(&bus_freq_mutex);

	if (mode == BUS_FREQ_HIGH)
		high_bus_count++;
	else if (mode == BUS_FREQ_AUDIO)
		audio_bus_count++;
	else if (mode == BUS_FREQ_LOW)
		low_bus_count++;

	if (busfreq_suspended || !bus_freq_scaling_initialized ||
		!bus_freq_scaling_is_active) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	cancel_low_bus_freq_handler();

	if ((mode == BUS_FREQ_HIGH) && (!high_bus_freq_mode)) {
		set_high_bus_freq(1);
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	if ((mode == BUS_FREQ_AUDIO) && (!high_bus_freq_mode) &&
		 (!audio_bus_freq_mode)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	mutex_unlock(&bus_freq_mutex);
}
EXPORT_SYMBOL(request_bus_freq);

void release_bus_freq(enum bus_freq_mode mode)
{
	mutex_lock(&bus_freq_mutex);
	if (mode == BUS_FREQ_HIGH) {
		if (high_bus_count == 0) {
			dev_err(busfreq_dev, "high bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		high_bus_count--;
	} else if (mode == BUS_FREQ_AUDIO) {
		if (audio_bus_count == 0) {
			dev_err(busfreq_dev, "audio bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		audio_bus_count--;
	} else if (mode == BUS_FREQ_LOW) {
		if (low_bus_count == 0) {
			dev_err(busfreq_dev, "low bus count mismatch!\n");
			dump_stack();
			mutex_unlock(&bus_freq_mutex);
			return;
		}
		low_bus_count--;
	}

	if (busfreq_suspended || !bus_freq_scaling_initialized ||
		!bus_freq_scaling_is_active) {
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	if ((!audio_bus_freq_mode) && (high_bus_count == 0) &&
		(audio_bus_count != 0)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	if ((!low_bus_freq_mode) && (high_bus_count == 0) &&
		(audio_bus_count == 0)) {
		set_low_bus_freq();
		mutex_unlock(&bus_freq_mutex);
		return;
	}

	mutex_unlock(&bus_freq_mutex);
}
EXPORT_SYMBOL(release_bus_freq);

int get_bus_freq_mode(void)
{
	return cur_bus_freq_mode;
}
EXPORT_SYMBOL(get_bus_freq_mode);

static void bus_freq_daemon_handler(struct work_struct *work)
{
	mutex_lock(&bus_freq_mutex);
	if ((!low_bus_freq_mode) && (high_bus_count == 0) &&
		(audio_bus_count == 0))
		set_low_bus_freq();
	mutex_unlock(&bus_freq_mutex);
}

static ssize_t bus_freq_scaling_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	if (bus_freq_scaling_is_active)
		return sprintf(buf, "Bus frequency scaling is enabled\n");
	else
		return sprintf(buf, "Bus frequency scaling is disabled\n");
}

static ssize_t bus_freq_scaling_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	if (strncmp(buf, "1", 1) == 0) {
		bus_freq_scaling_is_active = 1;
		set_high_bus_freq(1);
		/*
		 * We set bus freq to higher at the beginning,
		 * so we use this daemon thread to make sure system
		 * can enter low bus mode if there is no high bus request pending
		 */
		schedule_delayed_work(&bus_freq_daemon,
			usecs_to_jiffies(5000000));
	} else if (strncmp(buf, "0", 1) == 0) {
		if (bus_freq_scaling_is_active)
			set_high_bus_freq(1);
		bus_freq_scaling_is_active = 0;
	}
	return size;
}

static int bus_freq_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	mutex_lock(&bus_freq_mutex);

	if (event == PM_SUSPEND_PREPARE) {
		high_bus_count++;
		set_high_bus_freq(1);
		busfreq_suspended = 1;
	} else if (event == PM_POST_SUSPEND) {
		busfreq_suspended = 0;
		high_bus_count--;
		schedule_delayed_work(&bus_freq_daemon,
			usecs_to_jiffies(5000000));
	}

	mutex_unlock(&bus_freq_mutex);

	return NOTIFY_OK;
}

static int busfreq_reboot_notifier_event(struct notifier_block *this,
						 unsigned long event, void *ptr)
{
	/* System is rebooting. Set the system into high_bus_freq_mode. */
	request_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static struct notifier_block imx_bus_freq_pm_notifier = {
	.notifier_call = bus_freq_pm_notify,
};

static struct notifier_block imx_busfreq_reboot_notifier = {
	.notifier_call = busfreq_reboot_notifier_event,
};

static DEVICE_ATTR(enable, 0644, bus_freq_scaling_enable_show,
			bus_freq_scaling_enable_store);

static int imx8mq_init_busfreq_clk(struct platform_device *pdev)
{
	dram_pll_clk = devm_clk_get(&pdev->dev, "dram_pll");
	sys1_pll_800m = devm_clk_get(&pdev->dev, "sys1_pll_800m");
	sys1_pll_400m = devm_clk_get(&pdev->dev, "sys1_pll_400m");
	sys1_pll_100m = devm_clk_get(&pdev->dev, "sys1_pll_100m");
	sys1_pll_40m = devm_clk_get(&pdev->dev, "sys1_pll_40m");
	dram_alt_src = devm_clk_get(&pdev->dev, "dram_alt_src");
	dram_alt_root = devm_clk_get(&pdev->dev, "dram_alt_root");
	dram_core_clk = devm_clk_get(&pdev->dev, "dram_core");
	dram_apb_src = devm_clk_get(&pdev->dev, "dram_apb_src");
	dram_apb_pre_div = devm_clk_get(&pdev->dev, "dram_apb_pre_div");
	noc_div = devm_clk_get(&pdev->dev, "noc_div");
	ahb_div = devm_clk_get(&pdev->dev, "ahb_div");
	main_axi_src = devm_clk_get(&pdev->dev, "main_axi_src");
	osc_25m = devm_clk_get(&pdev->dev, "osc_25m");
	sys2_pll_333m = devm_clk_get(&pdev->dev, "sys2_pll_333m");

	if (IS_ERR(dram_pll_clk) || IS_ERR(sys1_pll_400m) || IS_ERR(sys1_pll_100m) ||
	    IS_ERR(sys1_pll_40m) || IS_ERR(dram_alt_src) || IS_ERR(dram_alt_root) ||
	    IS_ERR(dram_core_clk) || IS_ERR(dram_apb_src) || IS_ERR(dram_apb_pre_div)
	    || IS_ERR(noc_div) || IS_ERR(main_axi_src) || IS_ERR(ahb_div)
	    || IS_ERR(osc_25m) || IS_ERR(sys2_pll_333m)) {
		dev_err(&pdev->dev, "failed to get busfreq clk\n");
		return -EINVAL;
	}

	return 0;
}

static int imx8mm_init_busfreq_clk(struct platform_device *pdev)
{
	dram_pll = devm_clk_get(&pdev->dev, "dram_pll_div");
	dram_pll_clk = devm_clk_get(&pdev->dev, "dram_pll");
	dram_alt_src = devm_clk_get(&pdev->dev, "dram_alt_src");
	dram_alt_root = devm_clk_get(&pdev->dev, "dram_alt_root");
	dram_core_clk = devm_clk_get(&pdev->dev, "dram_core");
	dram_apb_src = devm_clk_get(&pdev->dev, "dram_apb_src");
	dram_apb_pre_div = devm_clk_get(&pdev->dev, "dram_apb_pre_div");
	sys1_pll_800m = devm_clk_get(&pdev->dev, "sys_pll1_800m");
	sys1_pll_100m = devm_clk_get(&pdev->dev, "sys_pll1_100m");
	sys1_pll_40m = devm_clk_get(&pdev->dev, "sys_pll1_40m");
	noc_div = devm_clk_get(&pdev->dev, "noc_div");
	ahb_div = devm_clk_get(&pdev->dev, "ahb_div");
	main_axi_src = devm_clk_get(&pdev->dev, "main_axi_src");
	osc_25m = devm_clk_get(&pdev->dev, "osc_24m");
	sys2_pll_333m = devm_clk_get(&pdev->dev, "sys_pll2_333m");

	if (IS_ERR(dram_pll_clk) || IS_ERR(dram_alt_src) || IS_ERR(dram_alt_root) ||
	    IS_ERR(dram_core_clk) || IS_ERR(dram_apb_src) || IS_ERR(dram_apb_pre_div) ||
	    IS_ERR(sys1_pll_800m) || IS_ERR(sys1_pll_100m) || IS_ERR(sys1_pll_40m) ||
	    IS_ERR(osc_25m) || IS_ERR(noc_div) || IS_ERR(main_axi_src) || IS_ERR(ahb_div) ||
	    IS_ERR(sys2_pll_333m)) {
		dev_err(&pdev->dev, "failed to get busfreq clk\n");
		return -EINVAL;
	}

	return 0;
}

/*!
 * This is the probe routine for the bus frequency driver.
 *
 * @param   pdev   The platform device structure
 *
 * @return         The function returns 0 on success
 *
 */

static int busfreq_probe(struct platform_device *pdev)
{
	int i, err;
	struct arm_smccc_res res;

	busfreq_dev = &pdev->dev;

	/* get the clock for DDRC */
	if (of_machine_is_compatible("fsl,imx8mq"))
		err = imx8mq_init_busfreq_clk(pdev);
	else
		err = imx8mm_init_busfreq_clk(pdev);

	if (err) {
		dev_err(busfreq_dev, "init clk failed\n");
		return err;
	}

	origin_noc_rate = clk_get_rate(noc_div);
	if (origin_noc_rate == 0) {
		WARN_ON(1);
		return -EINVAL;
	}

	/*
	 * Get the supported frequency, normally the lowest frequency point
	 * is used for low bus & audio bus mode.
	 */
	for (i = 0; i < 4; i++) {
		arm_smccc_smc(FSL_SIP_DDR_DVFS, 0x11, i, 0, 0, 0, 0, 0, &res);
		err = res.a0;
		if (err < 0)
			return -EINVAL;

		fsp_table[i] = res.a0;
	}

	/* get the lowest fsp index */
	for (i = 0; i < 4; i++)
		if (fsp_table[i] == 0)
			break;

	low_bus_mode_fsp_index = i - 1;

	/*
	 * if lowest fsp data rate higher than 666mts, then no dll off mode or
	 * bypass mode support.
	 */
	if (fsp_table[low_bus_mode_fsp_index] >= DLL_ON_DRATE)
		bypass_support = false;

	/* create the sysfs file */
	err = sysfs_create_file(&busfreq_dev->kobj, &dev_attr_enable.attr);
	if (err) {
		dev_err(busfreq_dev,
			"Unable to register sysdev entry for BUSFREQ");
		return err;
	}

	high_bus_freq_mode = 1;
	low_bus_freq_mode = 0;
	audio_bus_freq_mode = 0;
	cur_bus_freq_mode = BUS_FREQ_HIGH;

	bus_freq_scaling_is_active = 1;
	bus_freq_scaling_initialized = 1;

	INIT_DELAYED_WORK(&low_bus_freq_handler, reduce_bus_freq_handler);
	INIT_DELAYED_WORK(&bus_freq_daemon, bus_freq_daemon_handler);
	register_pm_notifier(&imx_bus_freq_pm_notifier);
	register_reboot_notifier(&imx_busfreq_reboot_notifier);

	/* enter low bus mode if no high speed device enabled */
	schedule_delayed_work(&bus_freq_daemon, msecs_to_jiffies(10000));

	return 0;
}

static const struct of_device_id imx_busfreq_ids[] = {
	{ .compatible = "fsl,imx_busfreq", },
	{ /*sentinel */}
};

static struct platform_driver busfreq_driver = {
	.driver = {
		.name = "imx_busfreq",
		.owner = THIS_MODULE,
		.of_match_table = imx_busfreq_ids,
		},
	.probe = busfreq_probe,
};

/*!
 * Initialise the busfreq_driver.
 *
 * @return The function always returns 0.
 */
static int __init busfreq_init(void)
{
	if (platform_driver_register(&busfreq_driver) != 0)
		return -ENODEV;

	printk(KERN_INFO "Bus freq driver module loaded\n");

	return 0;
}

static void __exit busfreq_cleanup(void)
{
	sysfs_remove_file(&busfreq_dev->kobj, &dev_attr_enable.attr);

	/* Unregister the device structure */
	platform_driver_unregister(&busfreq_driver);
	bus_freq_scaling_initialized = 0;
}

module_init(busfreq_init);
module_exit(busfreq_cleanup);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("Busfreq driver");
MODULE_LICENSE("GPL");
