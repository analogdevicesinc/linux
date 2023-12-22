// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2014-2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <mali_kbase.h>
#include <tl/mali_kbase_tracepoints.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

#include <linux/of.h>
#include <linux/clk.h>
#include <linux/devfreq.h>
#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
#include <linux/devfreq_cooling.h>
#endif

#include <linux/version.h>
#include <linux/pm_opp.h>
#include "mali_kbase_devfreq.h"

/**
 * get_voltage() - Get the voltage value corresponding to the nominal frequency
 *                 used by devfreq.
 * @kbdev:    Device pointer
 * @freq:     Nominal frequency in Hz passed by devfreq.
 *
 * This function will be called only when the opp table which is compatible with
 * "operating-points-v2-mali", is not present in the devicetree for GPU device.
 *
 * Return: Voltage value in micro volts, 0 in case of error.
 */
static unsigned long get_voltage(struct kbase_device *kbdev, unsigned long freq)
{
	struct dev_pm_opp *opp;
	unsigned long voltage = 0;

#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_lock();
#endif

	opp = dev_pm_opp_find_freq_exact(kbdev->dev, freq, true);

	if (IS_ERR_OR_NULL(opp))
		dev_err(kbdev->dev, "Failed to get opp (%d)\n", PTR_ERR_OR_ZERO(opp));
	else {
		voltage = dev_pm_opp_get_voltage(opp);
#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
		dev_pm_opp_put(opp);
#endif
	}

#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_unlock();
#endif

	/* Return the voltage in micro volts */
	return voltage;
}

void kbase_devfreq_opp_translate(struct kbase_device *kbdev, unsigned long freq, u64 *core_mask,
				 unsigned long *freqs, unsigned long *volts)
{
	unsigned int i;

	for (i = 0; i < kbdev->num_opps; i++) {
		if (kbdev->devfreq_table[i].opp_freq == freq) {
			unsigned int j;

			*core_mask = kbdev->devfreq_table[i].core_mask;
			for (j = 0; j < kbdev->nr_clocks; j++) {
				freqs[j] = kbdev->devfreq_table[i].real_freqs[j];
				volts[j] = kbdev->devfreq_table[i].opp_volts[j];
			}

			break;
		}
	}

	/* If failed to find OPP, return all cores enabled
	 * and nominal frequency and the corresponding voltage.
	 */
	if (i == kbdev->num_opps) {
		unsigned long voltage = get_voltage(kbdev, freq);

		*core_mask = kbdev->gpu_props.shader_present;

		for (i = 0; i < kbdev->nr_clocks; i++) {
			freqs[i] = freq;
			volts[i] = voltage;
		}
	}
}

static int kbase_devfreq_target(struct device *dev, unsigned long *target_freq, u32 flags)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	unsigned long nominal_freq;
	unsigned long freqs[BASE_MAX_NR_CLOCKS_REGULATORS] = { 0 };
#if IS_ENABLED(CONFIG_REGULATOR)
	unsigned long original_freqs[BASE_MAX_NR_CLOCKS_REGULATORS] = { 0 };
#endif
	unsigned long volts[BASE_MAX_NR_CLOCKS_REGULATORS] = { 0 };
	unsigned int i;
	u64 core_mask;

	nominal_freq = *target_freq;

#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_lock();
#endif
	opp = devfreq_recommended_opp(dev, &nominal_freq, flags);
#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_unlock();
#endif
	if (IS_ERR_OR_NULL(opp)) {
		dev_err(dev, "Failed to get opp (%d)\n", PTR_ERR_OR_ZERO(opp));
		return IS_ERR(opp) ? PTR_ERR(opp) : -ENODEV;
	}
#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
	dev_pm_opp_put(opp);
#endif

	/*
	 * Only update if there is a change of frequency
	 */
	if (kbdev->current_nominal_freq == nominal_freq) {
		*target_freq = nominal_freq;
		return 0;
	}

	kbase_devfreq_opp_translate(kbdev, nominal_freq, &core_mask, freqs, volts);

#if IS_ENABLED(CONFIG_REGULATOR)
	/* Regulators and clocks work in pairs: every clock has a regulator,
	 * and we never expect to have more regulators than clocks.
	 *
	 * We always need to increase the voltage before increasing the number
	 * of shader cores and the frequency of a regulator/clock pair,
	 * otherwise the clock wouldn't have enough power to perform
	 * the transition.
	 *
	 * It's always safer to decrease the number of shader cores and
	 * the frequency before decreasing voltage of a regulator/clock pair,
	 * otherwise the clock could have problematic operation if it is
	 * deprived of the necessary power to sustain its current frequency
	 * (even if that happens for a short transition interval).
	 */
	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (kbdev->regulators[i] && kbdev->current_voltages[i] != volts[i] &&
		    kbdev->current_freqs[i] < freqs[i]) {
			int err;

			err = regulator_set_voltage(kbdev->regulators[i], volts[i], volts[i]);
			if (!err) {
				kbdev->current_voltages[i] = volts[i];
			} else {
				dev_err(dev, "Failed to increase voltage (%d) (target %lu)\n", err,
					volts[i]);
				return err;
			}
		}
	}
#endif

	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (kbdev->clocks[i]) {
			int err;

			err = clk_set_rate(kbdev->clocks[i], freqs[i]);
			if (!err) {
#if IS_ENABLED(CONFIG_REGULATOR)
				original_freqs[i] = kbdev->current_freqs[i];
#endif
				kbdev->current_freqs[i] = freqs[i];
			} else {
				dev_err(dev, "Failed to set clock %lu (target %lu)\n", freqs[i],
					*target_freq);
				return err;
			}
		}
	}

	kbase_devfreq_set_core_mask(kbdev, core_mask);

#if IS_ENABLED(CONFIG_REGULATOR)
	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (kbdev->regulators[i] && kbdev->current_voltages[i] != volts[i] &&
		    original_freqs[i] > freqs[i]) {
			int err;

			err = regulator_set_voltage(kbdev->regulators[i], volts[i], volts[i]);
			if (!err) {
				kbdev->current_voltages[i] = volts[i];
			} else {
				dev_err(dev, "Failed to decrease voltage (%d) (target %lu)\n", err,
					volts[i]);
				return err;
			}
		}
	}
#endif

	*target_freq = nominal_freq;
	kbdev->current_nominal_freq = nominal_freq;
	kbdev->current_core_mask = core_mask;

	KBASE_TLSTREAM_AUX_DEVFREQ_TARGET(kbdev, (u64)nominal_freq);

	return 0;
}

void kbase_devfreq_force_freq(struct kbase_device *kbdev, unsigned long freq)
{
	unsigned long target_freq = freq;

	kbase_devfreq_target(kbdev->dev, &target_freq, 0);
}

static int kbase_devfreq_cur_freq(struct device *dev, unsigned long *freq)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	*freq = kbdev->current_nominal_freq;

	return 0;
}

static int kbase_devfreq_status(struct device *dev, struct devfreq_dev_status *stat)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);
	struct kbasep_pm_metrics diff;

	kbase_pm_get_dvfs_metrics(kbdev, &kbdev->last_devfreq_metrics, &diff);

	stat->busy_time = diff.time_busy;
	stat->total_time = diff.time_busy + diff.time_idle;
	stat->current_frequency = kbdev->current_nominal_freq;
	stat->private_data = NULL;

#if MALI_USE_CSF && defined CONFIG_DEVFREQ_THERMAL
	kbase_ipa_reset_data(kbdev);
#endif

	return 0;
}

static int kbase_devfreq_init_freq_table(struct kbase_device *kbdev, struct devfreq_dev_profile *dp)
{
	int count;
	unsigned int i = 0;
	unsigned long freq;
	struct dev_pm_opp *opp;

#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_lock();
#endif
	count = dev_pm_opp_get_opp_count(kbdev->dev);
#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_unlock();
#endif
	if (count < 0)
		return count;

	dp->freq_table = kmalloc_array((size_t)count, sizeof(dp->freq_table[0]), GFP_KERNEL);
	if (!dp->freq_table)
		return -ENOMEM;

#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_lock();
#endif
	for (i = 0, freq = ULONG_MAX; i < (unsigned int)count; i++, freq--) {
		opp = dev_pm_opp_find_freq_floor(kbdev->dev, &freq);
		if (IS_ERR(opp))
			break;
#if KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE
		dev_pm_opp_put(opp);
#endif /* KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE */

		dp->freq_table[i] = freq;
	}
#if KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE
	rcu_read_unlock();
#endif

	if ((unsigned int)count != i)
		dev_warn(kbdev->dev, "Unable to enumerate all OPPs (%d!=%u\n", count, i);

	dp->max_state = i;


	/* Have the lowest clock as suspend clock.
	 * It may be overridden by 'opp-mali-errata-1485982'.
	 */
	if (kbdev->pm.backend.gpu_clock_slow_down_wa) {
		freq = 0;
		opp = dev_pm_opp_find_freq_ceil(kbdev->dev, &freq);
		if (IS_ERR(opp)) {
			dev_err(kbdev->dev, "failed to find slowest clock");
			return 0;
		}
		dev_info(kbdev->dev, "suspend clock %lu from slowest", freq);
		kbdev->pm.backend.gpu_clock_suspend_freq = freq;
	}

	return 0;
}

static void kbase_devfreq_term_freq_table(struct kbase_device *kbdev)
{
	struct devfreq_dev_profile *dp = &kbdev->devfreq_profile;

	kfree(dp->freq_table);
	dp->freq_table = NULL;
}

static void kbase_devfreq_term_core_mask_table(struct kbase_device *kbdev)
{
	kfree(kbdev->devfreq_table);
	kbdev->devfreq_table = NULL;
}

static void kbase_devfreq_exit(struct device *dev)
{
	struct kbase_device *kbdev = dev_get_drvdata(dev);

	if (kbdev)
		kbase_devfreq_term_freq_table(kbdev);
}

static void kbasep_devfreq_read_suspend_clock(struct kbase_device *kbdev, struct device_node *node)
{
	u64 freq = 0;
	int err = 0;

	/* Check if this node is the opp entry having 'opp-mali-errata-1485982'
	 * to get the suspend clock, otherwise skip it.
	 */
	if (!of_property_read_bool(node, "opp-mali-errata-1485982"))
		return;

	/* In kbase DevFreq, the clock will be read from 'opp-hz'
	 * and translated into the actual clock by opp_translate.
	 *
	 * In customer DVFS, the clock will be read from 'opp-hz-real'
	 * for clk driver. If 'opp-hz-real' does not exist,
	 * read from 'opp-hz'.
	 */
	if (IS_ENABLED(CONFIG_MALI_DEVFREQ))
		err = of_property_read_u64(node, "opp-hz", &freq);
	else {
		if (of_property_read_u64(node, "opp-hz-real", &freq))
			err = of_property_read_u64(node, "opp-hz", &freq);
	}

	if (WARN_ON(err || !freq))
		return;

	kbdev->pm.backend.gpu_clock_suspend_freq = freq;
	dev_info(kbdev->dev, "suspend clock %llu by opp-mali-errata-1485982", freq);
}

static int kbase_devfreq_init_core_mask_table(struct kbase_device *kbdev)
{
#ifndef CONFIG_OF
	/* OPP table initialization requires at least the capability to get
	 * regulators and clocks from the device tree, as well as parsing
	 * arrays of unsigned integer values.
	 *
	 * The whole initialization process shall simply be skipped if the
	 * minimum capability is not available.
	 */
	return 0;
#else
	struct device_node *opp_node =
		of_parse_phandle(kbdev->dev->of_node, "operating-points-v2", 0);
	struct device_node *node;
	unsigned int i = 0;
	int count;
	u64 shader_present = kbdev->gpu_props.shader_present;

	if (!opp_node)
		return 0;
	if (!of_device_is_compatible(opp_node, "operating-points-v2-mali"))
		return 0;

	count = dev_pm_opp_get_opp_count(kbdev->dev);
	kbdev->devfreq_table =
		kmalloc_array((size_t)count, sizeof(struct kbase_devfreq_opp), GFP_KERNEL);
	if (!kbdev->devfreq_table)
		return -ENOMEM;

	for_each_available_child_of_node(opp_node, node) {
		const void *core_count_p;
		u64 core_mask, opp_freq, real_freqs[BASE_MAX_NR_CLOCKS_REGULATORS];
		int err;
#if IS_ENABLED(CONFIG_REGULATOR)
		u32 opp_volts[BASE_MAX_NR_CLOCKS_REGULATORS];
#endif

		/* Read suspend clock from opp table */
		if (kbdev->pm.backend.gpu_clock_slow_down_wa)
			kbasep_devfreq_read_suspend_clock(kbdev, node);

		err = of_property_read_u64(node, "opp-hz", &opp_freq);
		if (err) {
			dev_warn(kbdev->dev, "Failed to read opp-hz property with error %d\n", err);
			continue;
		}

#if BASE_MAX_NR_CLOCKS_REGULATORS > 1
		err = of_property_read_u64_array(node, "opp-hz-real", real_freqs, kbdev->nr_clocks);
#else
		WARN_ON(kbdev->nr_clocks != 1);
		err = of_property_read_u64(node, "opp-hz-real", real_freqs);
#endif
		if (err < 0) {
			dev_warn(kbdev->dev, "Failed to read opp-hz-real property with error %d\n",
				 err);
			continue;
		}
#if IS_ENABLED(CONFIG_REGULATOR)
		err = of_property_read_u32_array(node, "opp-microvolt", opp_volts,
						 kbdev->nr_regulators);
		if (err < 0) {
			dev_warn(kbdev->dev,
				 "Failed to read opp-microvolt property with error %d\n", err);
			continue;
		}
#endif

		if (of_property_read_u64(node, "opp-core-mask", &core_mask))
			core_mask = shader_present;
		if (core_mask != shader_present && corestack_driver_control) {
			dev_warn(
				kbdev->dev,
				"Ignoring OPP %llu - Dynamic Core Scaling not supported on this GPU\n",
				opp_freq);
			continue;
		}

		core_count_p = of_get_property(node, "opp-core-count", NULL);
		if (core_count_p) {
			u64 remaining_core_mask = kbdev->gpu_props.shader_present;
			int core_count = be32_to_cpup(core_count_p);

			core_mask = 0;

			for (; core_count > 0; core_count--) {
				int core = ffs(remaining_core_mask);

				if (!core) {
					dev_err(kbdev->dev, "OPP has more cores than GPU\n");
					return -ENODEV;
				}

				core_mask |= (1ull << (core - 1));
				remaining_core_mask &= ~(1ull << (core - 1));
			}
		}

		if (!core_mask) {
			dev_err(kbdev->dev, "OPP has invalid core mask of 0\n");
			return -ENODEV;
		}

		kbdev->devfreq_table[i].opp_freq = opp_freq;
		kbdev->devfreq_table[i].core_mask = core_mask;
		if (kbdev->nr_clocks > 0) {
			unsigned int j;

			for (j = 0; j < kbdev->nr_clocks; j++)
				kbdev->devfreq_table[i].real_freqs[j] = real_freqs[j];
		}
#if IS_ENABLED(CONFIG_REGULATOR)
		if (kbdev->nr_regulators > 0) {
			unsigned int j;

			for (j = 0; j < kbdev->nr_regulators; j++)
				kbdev->devfreq_table[i].opp_volts[j] = opp_volts[j];
		}
#endif

		dev_info(kbdev->dev, "OPP %d : opp_freq=%llu core_mask=%llx\n", i, opp_freq,
			 core_mask);

		i++;
	}

	kbdev->num_opps = i;

	return 0;
#endif /* CONFIG_OF */
}

static const char *kbase_devfreq_req_type_name(enum kbase_devfreq_work_type type)
{
	const char *p;

	switch (type) {
	case DEVFREQ_WORK_NONE:
		p = "devfreq_none";
		break;
	case DEVFREQ_WORK_SUSPEND:
		p = "devfreq_suspend";
		break;
	case DEVFREQ_WORK_RESUME:
		p = "devfreq_resume";
		break;
	default:
		p = "Unknown devfreq_type";
	}
	return p;
}

static void kbase_devfreq_suspend_resume_worker(struct work_struct *work)
{
	struct kbase_devfreq_queue_info *info =
		container_of(work, struct kbase_devfreq_queue_info, work);
	struct kbase_device *kbdev = container_of(info, struct kbase_device, devfreq_queue);
	unsigned long flags;
	enum kbase_devfreq_work_type type, acted_type;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	type = kbdev->devfreq_queue.req_type;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	acted_type = kbdev->devfreq_queue.acted_type;
	dev_dbg(kbdev->dev, "Worker handles queued req: %s (acted: %s)\n",
		kbase_devfreq_req_type_name(type), kbase_devfreq_req_type_name(acted_type));
	switch (type) {
	case DEVFREQ_WORK_SUSPEND:
	case DEVFREQ_WORK_RESUME:
		if (type != acted_type) {
			if (type == DEVFREQ_WORK_RESUME)
				devfreq_resume_device(kbdev->devfreq);
			else
				devfreq_suspend_device(kbdev->devfreq);
			dev_dbg(kbdev->dev, "Devfreq transition occured: %s => %s\n",
				kbase_devfreq_req_type_name(acted_type),
				kbase_devfreq_req_type_name(type));
			kbdev->devfreq_queue.acted_type = type;
		}
		break;
	default:
		WARN_ON(1);
	}
}

void kbase_devfreq_enqueue_work(struct kbase_device *kbdev, enum kbase_devfreq_work_type work_type)
{
	unsigned long flags;

	WARN_ON(work_type == DEVFREQ_WORK_NONE);
	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	/* Skip enqueuing a work if workqueue has already been terminated. */
	if (likely(kbdev->devfreq_queue.workq)) {
		kbdev->devfreq_queue.req_type = work_type;
		queue_work(kbdev->devfreq_queue.workq, &kbdev->devfreq_queue.work);
	}
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
	dev_dbg(kbdev->dev, "Enqueuing devfreq req: %s\n", kbase_devfreq_req_type_name(work_type));
}

static int kbase_devfreq_work_init(struct kbase_device *kbdev)
{
	kbdev->devfreq_queue.req_type = DEVFREQ_WORK_NONE;
	kbdev->devfreq_queue.acted_type = DEVFREQ_WORK_RESUME;

	kbdev->devfreq_queue.workq = alloc_ordered_workqueue("devfreq_workq", 0);
	if (!kbdev->devfreq_queue.workq)
		return -ENOMEM;

	INIT_WORK(&kbdev->devfreq_queue.work, kbase_devfreq_suspend_resume_worker);
	return 0;
}

static void kbase_devfreq_work_term(struct kbase_device *kbdev)
{
	unsigned long flags;
	struct workqueue_struct *workq;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	workq = kbdev->devfreq_queue.workq;
	kbdev->devfreq_queue.workq = NULL;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	destroy_workqueue(workq);
}

int kbase_devfreq_init(struct kbase_device *kbdev)
{
	struct devfreq_dev_profile *dp;
	int err;
	unsigned int i;
	bool free_devfreq_freq_table = true;

	if (kbdev->nr_clocks == 0) {
		dev_err(kbdev->dev, "Clock not available for devfreq\n");
		return -ENODEV;
	}

	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (kbdev->clocks[i])
			kbdev->current_freqs[i] = clk_get_rate(kbdev->clocks[i]);
	}
	kbdev->current_nominal_freq = kbdev->current_freqs[0];

	dp = &kbdev->devfreq_profile;

	dp->initial_freq = kbdev->current_freqs[0];
	dp->polling_ms = 100;
	dp->target = kbase_devfreq_target;
	dp->get_dev_status = kbase_devfreq_status;
	dp->get_cur_freq = kbase_devfreq_cur_freq;
	dp->exit = kbase_devfreq_exit;

	if (kbase_devfreq_init_freq_table(kbdev, dp))
		return -EFAULT;

	if (dp->max_state > 0) {
		/* Record the maximum frequency possible */
		kbdev->gpu_props.gpu_freq_khz_max = dp->freq_table[0] / 1000;
	}

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
	err = kbase_ipa_init(kbdev);
	if (err) {
		dev_err(kbdev->dev, "IPA initialization failed");
		goto ipa_init_failed;
	}
#endif

	err = kbase_devfreq_init_core_mask_table(kbdev);
	if (err)
		goto init_core_mask_table_failed;

	kbdev->devfreq = devfreq_add_device(kbdev->dev, dp, "simple_ondemand", NULL);
	if (IS_ERR(kbdev->devfreq)) {
		err = PTR_ERR(kbdev->devfreq);
		kbdev->devfreq = NULL;
		dev_err(kbdev->dev, "Fail to add devfreq device(%d)", err);
		goto devfreq_add_dev_failed;
	}

	/* Explicit free of freq table isn't needed after devfreq_add_device() */
	free_devfreq_freq_table = false;

	/* Initialize devfreq suspend/resume workqueue */
	err = kbase_devfreq_work_init(kbdev);
	if (err) {
		dev_err(kbdev->dev, "Fail to init devfreq workqueue");
		goto devfreq_work_init_failed;
	}

	/* devfreq_add_device only copies a few of kbdev->dev's fields, so
	 * set drvdata explicitly so IPA models can access kbdev.
	 */
	dev_set_drvdata(&kbdev->devfreq->dev, kbdev);

	err = devfreq_register_opp_notifier(kbdev->dev, kbdev->devfreq);
	if (err) {
		dev_err(kbdev->dev, "Failed to register OPP notifier (%d)", err);
		goto opp_notifier_failed;
	}

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
	kbdev->devfreq_cooling = of_devfreq_cooling_register_power(
		kbdev->dev->of_node, kbdev->devfreq, &kbase_ipa_power_model_ops);
	if (IS_ERR_OR_NULL(kbdev->devfreq_cooling)) {
		err = PTR_ERR_OR_ZERO(kbdev->devfreq_cooling);
		dev_err(kbdev->dev, "Failed to register cooling device (%d)", err);
		err = err == 0 ? -ENODEV : err;
		goto cooling_reg_failed;
	}
#endif

	return 0;

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
cooling_reg_failed:
	devfreq_unregister_opp_notifier(kbdev->dev, kbdev->devfreq);
#endif /* CONFIG_DEVFREQ_THERMAL */

opp_notifier_failed:
	kbase_devfreq_work_term(kbdev);

devfreq_work_init_failed:
	if (devfreq_remove_device(kbdev->devfreq))
		dev_err(kbdev->dev, "Failed to terminate devfreq (%d)", err);

	kbdev->devfreq = NULL;

devfreq_add_dev_failed:
	kbase_devfreq_term_core_mask_table(kbdev);

init_core_mask_table_failed:
#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
	kbase_ipa_term(kbdev);
ipa_init_failed:
#endif
	if (free_devfreq_freq_table)
		kbase_devfreq_term_freq_table(kbdev);

	return err;
}

void kbase_devfreq_term(struct kbase_device *kbdev)
{
	int err;

	dev_dbg(kbdev->dev, "Term Mali devfreq\n");

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
	if (kbdev->devfreq_cooling)
		devfreq_cooling_unregister(kbdev->devfreq_cooling);
#endif

	devfreq_unregister_opp_notifier(kbdev->dev, kbdev->devfreq);

	kbase_devfreq_work_term(kbdev);

	err = devfreq_remove_device(kbdev->devfreq);
	if (err)
		dev_err(kbdev->dev, "Failed to terminate devfreq (%d)\n", err);
	else
		kbdev->devfreq = NULL;

	kbase_devfreq_term_core_mask_table(kbdev);

#if IS_ENABLED(CONFIG_DEVFREQ_THERMAL)
	kbase_ipa_term(kbdev);
#endif
}
