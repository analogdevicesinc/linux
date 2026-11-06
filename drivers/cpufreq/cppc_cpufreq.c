// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPPC (Collaborative Processor Performance Control) driver for
 * interfacing with the CPUfreq layer and governors. See
 * cppc_acpi.c for CPPC specific methods.
 *
 * (C) Copyright 2014, 2015 Linaro Ltd.
 * Author: Ashwin Chaugule <ashwin.chaugule@linaro.org>
 */

#define pr_fmt(fmt)	"CPPC Cpufreq:"	fmt

#include <linux/arch_topology.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/vmalloc.h>
#include <uapi/linux/sched/types.h>

#include <linux/unaligned.h>

#include <acpi/cppc_acpi.h>

static struct cpufreq_driver cppc_cpufreq_driver;

/*
 * OSPM-set CPPC registers tracked for save/restore. A value the OS wrote is
 * reapplied from online() across CPU hotplug, and the firmware value is
 * restored from offline().
 *
 * Autonomous Selection (auto_sel) is kept first, as the registers after it
 * only have meaning while it is enabled.
 */
enum cppc_saved_reg_id {
	CPPC_SAVED_AUTO_SEL,
	CPPC_SAVED_EPP,
	CPPC_SAVED_AUTO_ACT_WINDOW,
	CPPC_NR_SAVED_REGS,
};

struct cppc_saved_reg {
	const char *name;
	int (*get)(int cpu, u64 *val);
	int (*set)(int cpu, u64 val);
};

static const struct cppc_saved_reg cppc_saved_regs[CPPC_NR_SAVED_REGS] = {
	[CPPC_SAVED_AUTO_SEL] = {
		.name = "auto_sel",
		.get = cppc_get_auto_sel,
		.set = cppc_set_auto_sel,
	},
	[CPPC_SAVED_EPP] = {
		.name = "epp",
		.get = cppc_get_epp_perf,
		.set = cppc_set_epp,
	},
	[CPPC_SAVED_AUTO_ACT_WINDOW] = {
		.name = "auto_act_window",
		.get = cppc_get_auto_act_window,
		.set = cppc_set_auto_act_window,
	},
};

enum cppc_saved_type {
	CPPC_SAVED_FIRMWARE,
	CPPC_SAVED_REQUESTED,
	CPPC_NR_SAVED_TYPES,
};

/*
 * Per-policy values saved for each register in cppc_saved_regs[], U64_MAX
 * if the register is not implemented or not readable:
 *   CPPC_SAVED_FIRMWARE  - value before the driver touched it, saved at
 *                          init() and written back when the policy goes
 *                          offline
 *   CPPC_SAVED_REQUESTED - value the OS asked for, reapplied at online().
 *                          It starts as the firmware value and changes only
 *                          when a write from the OS succeeds.
 */
struct cppc_policy_state {
	u64 regs[CPPC_NR_SAVED_REGS][CPPC_NR_SAVED_TYPES];
	/*
	 * Set by suspend() after it restores the firmware values. While it is
	 * set, offline() skips that restore and resume() calls online().
	 * Cleared by init() and online().
	 */
	bool suspend_regs_handled;
};

static DEFINE_PER_CPU(struct cppc_policy_state, cppc_policy_state);

/*
 * Per-policy state is kept in the per-CPU variable of the first CPU the policy
 * manages. related_cpus (the policy's full set of CPUs) never changes while the
 * policy exists, so this CPU (unlike policy->cpu) stays the same across CPU
 * hotplug, and every callback reaches the same copy.
 */
static struct cppc_policy_state *
cppc_cpufreq_policy_state(struct cpufreq_policy *policy)
{
	const struct cpumask *policy_cpus = policy->related_cpus;

	/*
	 * related_cpus is empty during init(), as the core fills it in only
	 * after init() returns. Use policy->cpus then, which has the same
	 * first CPU.
	 */
	if (cpumask_empty(policy_cpus))
		policy_cpus = policy->cpus;

	return &per_cpu(cppc_policy_state, cpumask_first(policy_cpus));
}

/*
 * Save each tracked register's firmware value before the driver programs
 * anything. Also start the requested value from it. If the OS never writes a
 * register and the platform resets it while the CPU is offline, online() then
 * writes the firmware value back.
 *
 * Skip a register that is not implemented or not readable. Fail on any other
 * read error, as the firmware value is needed to put the register back later.
 */
static int cppc_cpufreq_save_firmware_regs(struct cpufreq_policy *policy)
{
	struct cppc_policy_state *st = cppc_cpufreq_policy_state(policy);
	unsigned int cpu = policy->cpu;
	u64 val;
	int i, ret;

	for (i = 0; i < CPPC_NR_SAVED_REGS; i++) {
		ret = cppc_saved_regs[i].get(cpu, &val);
		if (ret == -EOPNOTSUPP)
			val = U64_MAX;
		else if (ret)
			return ret;

		st->regs[i][CPPC_SAVED_FIRMWARE] = val;
		st->regs[i][CPPC_SAVED_REQUESTED] = val;
	}

	return 0;
}

/*
 * After a successful write to a tracked register, save the value so that
 * online() can reapply it.
 */
static void cppc_cpufreq_save_requested(struct cpufreq_policy *policy,
					enum cppc_saved_reg_id reg, u64 val)
{
	cppc_cpufreq_policy_state(policy)->regs[reg][CPPC_SAVED_REQUESTED] = val;
}

/*
 * Write one tracked register, skipping it when there is no saved value.
 * A register the platform does not allow writing is not an error.
 */
static void cppc_cpufreq_write_saved_reg(unsigned int cpu,
					 enum cppc_saved_reg_id reg, u64 val,
					 enum cppc_saved_type saved_type)
{
	const char *op = (saved_type == CPPC_SAVED_FIRMWARE) ?
			 "restore firmware" : "reapply requested";
	int ret;

	if (val == U64_MAX)
		return;

	ret = cppc_saved_regs[reg].set(cpu, val);
	if (ret == -EOPNOTSUPP)
		return;
	if (ret)
		pr_debug("Failed to %s %s=%llu on CPU%u (%d)\n", op,
			 cppc_saved_regs[reg].name, val, cpu, ret);
}

/*
 * Apply the saved firmware or requested value to each tracked register.
 *
 * Write auto_sel first when the value being applied enables autonomous
 * selection and last when it disables it, so the writes to the dependent
 * registers can still take effect. While autonomous selection stays disabled,
 * the platform may ignore those writes. Do not enable it temporarily to force
 * them through.
 */
static void cppc_cpufreq_apply_saved_regs(struct cpufreq_policy *policy,
					  enum cppc_saved_type saved_type)
{
	const struct cppc_policy_state *st = cppc_cpufreq_policy_state(policy);
	unsigned int cpu = policy->cpu;
	u64 auto_sel, val;
	int i;

	auto_sel = st->regs[CPPC_SAVED_AUTO_SEL][saved_type];

	if (auto_sel)
		cppc_cpufreq_write_saved_reg(cpu, CPPC_SAVED_AUTO_SEL, auto_sel,
					     saved_type);

	for (i = CPPC_SAVED_AUTO_SEL + 1; i < CPPC_NR_SAVED_REGS; i++) {
		val = st->regs[i][saved_type];
		cppc_cpufreq_write_saved_reg(cpu, i, val, saved_type);
	}

	if (!auto_sel)
		cppc_cpufreq_write_saved_reg(cpu, CPPC_SAVED_AUTO_SEL, auto_sel,
					     saved_type);
}

#ifdef CONFIG_ACPI_CPPC_CPUFREQ_FIE
static enum {
	FIE_UNSET = -1,
	FIE_ENABLED,
	FIE_DISABLED
} fie_disabled = FIE_UNSET;

module_param(fie_disabled, int, 0444);
MODULE_PARM_DESC(fie_disabled, "Disable Frequency Invariance Engine (FIE)");

/* Frequency invariance support */
struct cppc_freq_invariance {
	int cpu;
	bool pcc_work_initialized;
	struct irq_work irq_work;
	struct kthread_work work;
	struct cppc_perf_fb_ctrs prev_perf_fb_ctrs;
	struct cppc_cpudata *cpu_data;
};

static DEFINE_PER_CPU(struct cppc_freq_invariance, cppc_freq_inv);
static struct kthread_worker *kworker_fie;
static DEFINE_MUTEX(cppc_fie_lock);

static int cppc_perf_from_fbctrs(u64 reference_perf,
				 struct cppc_perf_fb_ctrs *fb_ctrs_t0,
				 struct cppc_perf_fb_ctrs *fb_ctrs_t1);
static int cppc_fie_kworker_init(void);

/**
 * __cppc_scale_freq_tick - CPPC arch_freq_scale updater for frequency invariance
 * @cppc_fi: per-cpu CPPC FIE data.
 *
 * The CPPC driver registers itself with the topology core to provide its own
 * implementation (cppc_scale_freq_tick()) of topology_scale_freq_tick() which
 * gets called by the scheduler on every tick.
 *
 * Note that the arch specific counters have higher priority than CPPC counters,
 * if available, though the CPPC driver doesn't need to have any special
 * handling for that.
 */
static void __cppc_scale_freq_tick(struct cppc_freq_invariance *cppc_fi)
{
	struct cppc_perf_fb_ctrs fb_ctrs = {0};
	struct cppc_cpudata *cpu_data;
	unsigned long local_freq_scale;
	u64 perf, ref_perf;

	cpu_data = cppc_fi->cpu_data;

	if (cppc_get_perf_ctrs(cppc_fi->cpu, &fb_ctrs)) {
		pr_warn("%s: failed to read perf counters\n", __func__);
		return;
	}

	ref_perf = cpu_data->perf_caps.reference_perf;
	perf = cppc_perf_from_fbctrs(ref_perf,
				     &cppc_fi->prev_perf_fb_ctrs, &fb_ctrs);
	if (!perf)
		return;

	cppc_fi->prev_perf_fb_ctrs = fb_ctrs;

	perf <<= SCHED_CAPACITY_SHIFT;
	local_freq_scale = div64_u64(perf, cpu_data->perf_caps.highest_perf);

	/* This can happen due to counter's overflow */
	if (unlikely(local_freq_scale > 1024))
		local_freq_scale = 1024;

	per_cpu(arch_freq_scale, cppc_fi->cpu) = local_freq_scale;
}

static void cppc_scale_freq_tick(void)
{
	__cppc_scale_freq_tick(&per_cpu(cppc_freq_inv, smp_processor_id()));
}

static struct scale_freq_data cppc_sftd = {
	.source = SCALE_FREQ_SOURCE_CPPC,
	.set_freq_scale = cppc_scale_freq_tick,
};

static void cppc_scale_freq_workfn(struct kthread_work *work)
{
	struct cppc_freq_invariance *cppc_fi;

	cppc_fi = container_of(work, struct cppc_freq_invariance, work);
	__cppc_scale_freq_tick(cppc_fi);
}

static void cppc_irq_work(struct irq_work *irq_work)
{
	struct cppc_freq_invariance *cppc_fi;

	cppc_fi = container_of(irq_work, struct cppc_freq_invariance, irq_work);
	kthread_queue_work(kworker_fie, &cppc_fi->work);
}

/*
 * Reading perf counters may sleep if the CPC regs are in PCC.  Thus, we
 * schedule an irq work in scale_freq_tick (since we reach here from hard-irq
 * context), which then schedules a normal work item cppc_scale_freq_workfn()
 * that updates the per_cpu arch_freq_scale variable based on the counter
 * updates since the last tick.
 */
static void cppc_scale_freq_tick_pcc(void)
{
	struct cppc_freq_invariance *cppc_fi = &per_cpu(cppc_freq_inv, smp_processor_id());

	/*
	 * cppc_get_perf_ctrs() can potentially sleep, call that from the right
	 * context.
	 */
	irq_work_queue(&cppc_fi->irq_work);
}

static struct scale_freq_data cppc_sftd_pcc = {
	.source = SCALE_FREQ_SOURCE_CPPC,
	.set_freq_scale = cppc_scale_freq_tick_pcc,
};

static void cppc_cpufreq_cpu_fie_init(struct cpufreq_policy *policy)
{
	const struct cpumask *cpus = policy->related_cpus;
	struct scale_freq_data *sftd;
	struct cppc_freq_invariance *cppc_fi;
	int cpu, ret;

	if (fie_disabled)
		return;

	/*
	 * When resume() calls online(), policy->cpus has only the online CPUs.
	 * online() does not run again for a CPU in the same policy that comes
	 * up later, so use related_cpus. It is empty only during init().
	 */
	if (cpumask_empty(cpus))
		cpus = policy->cpus;

	for_each_cpu(cpu, cpus) {
		cppc_fi = &per_cpu(cppc_freq_inv, cpu);
		cppc_fi->cpu = cpu;
		cppc_fi->cpu_data = policy->driver_data;
		if (cppc_perf_ctrs_in_pcc_cpu(cpu)) {
			if (cppc_fie_kworker_init())
				return;

			kthread_init_work(&cppc_fi->work, cppc_scale_freq_workfn);
			init_irq_work(&cppc_fi->irq_work, cppc_irq_work);
			cppc_fi->pcc_work_initialized = true;
		}

		ret = cppc_get_perf_ctrs(cpu, &cppc_fi->prev_perf_fb_ctrs);

		/* Don't abort for a CPU that is offline. */
		if (ret && cpu_online(cpu)) {
			pr_debug("%s: failed to read perf counters for cpu:%d: %d\n",
				__func__, cpu, ret);
			return;
		}
	}

	/* A shared policy may contain both PCC and non-PCC counters. */
	for_each_cpu(cpu, cpus) {
		cppc_fi = &per_cpu(cppc_freq_inv, cpu);
		if (cppc_fi->pcc_work_initialized)
			sftd = &cppc_sftd_pcc;
		else
			sftd = &cppc_sftd;
		topology_set_scale_freq_source(sftd, cpumask_of(cpu));
	}
}

/*
 * Drain work initialized by this policy even if processor removal has
 * already unpublished the CPU's CPC descriptor.
 */
static void cppc_cpufreq_cpu_fie_exit(struct cpufreq_policy *policy)
{
	struct cppc_freq_invariance *cppc_fi;
	int cpu;

	if (fie_disabled)
		return;

	/* policy->cpus has only the online CPUs, so use related_cpus */
	topology_clear_scale_freq_source(SCALE_FREQ_SOURCE_CPPC, policy->related_cpus);

	for_each_cpu(cpu, policy->related_cpus) {
		cppc_fi = &per_cpu(cppc_freq_inv, cpu);
		if (!cppc_fi->pcc_work_initialized)
			continue;
		irq_work_sync(&cppc_fi->irq_work);
		kthread_cancel_work_sync(&cppc_fi->work);
		cppc_fi->pcc_work_initialized = false;
	}
}

static int cppc_fie_kworker_init(void)
{
	struct kthread_worker *worker;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_nice	= 0,
		.sched_priority	= 0,
		/*
		 * Fake (unused) bandwidth; workaround to "fix"
		 * priority inheritance.
		 */
		.sched_runtime	= NSEC_PER_MSEC,
		.sched_deadline = 10 * NSEC_PER_MSEC,
		.sched_period	= 10 * NSEC_PER_MSEC,
	};
	int ret;

	guard(mutex)(&cppc_fie_lock);

	if (kworker_fie)
		return 0;

	worker = kthread_run_worker(0, "cppc_fie");
	if (IS_ERR(worker)) {
		pr_warn("%s: failed to create kworker_fie: %pe\n", __func__, worker);
		return PTR_ERR(worker);
	}

	ret = sched_setattr_nocheck(worker->task, &attr);
	if (ret) {
		pr_warn("%s: failed to set SCHED_DEADLINE: %d\n", __func__,
			ret);
		kthread_destroy_worker(worker);
		return ret;
	}

	kworker_fie = worker;
	return 0;
}

static void __init cppc_freq_invariance_init(void)
{
	bool perf_ctrs_in_pcc = cppc_perf_ctrs_in_pcc();

	if (fie_disabled == FIE_UNSET) {
		if (perf_ctrs_in_pcc) {
			pr_info("FIE not enabled on systems with registers in PCC\n");
			fie_disabled = FIE_DISABLED;
		} else {
			fie_disabled = FIE_ENABLED;
		}
	}
}

static void cppc_freq_invariance_exit(void)
{
	if (kworker_fie)
		kthread_destroy_worker(kworker_fie);
}

#else
static inline void cppc_cpufreq_cpu_fie_init(struct cpufreq_policy *policy)
{
}

static inline void cppc_cpufreq_cpu_fie_exit(struct cpufreq_policy *policy)
{
}

static inline void cppc_freq_invariance_init(void)
{
}

static inline void cppc_freq_invariance_exit(void)
{
}
#endif /* CONFIG_ACPI_CPPC_CPUFREQ_FIE */

static void cppc_cpufreq_get_perf_limits(struct cppc_cpudata *cpu_data,
					 struct cpufreq_policy *policy,
					 u32 *min_perf, u32 *max_perf)
{
	struct cppc_perf_caps *caps = &cpu_data->perf_caps;
	unsigned int min_freq, max_freq;
	u32 min, max;

	min_freq = READ_ONCE(policy->min);
	max_freq = READ_ONCE(policy->max);
	if (unlikely(min_freq > max_freq))
		min_freq = max_freq;

	min = cppc_khz_to_perf(caps, min_freq);
	max = cppc_khz_to_perf(caps, max_freq);

	*min_perf = clamp_t(u32, min, caps->lowest_perf, caps->highest_perf);
	*max_perf = clamp_t(u32, max, caps->lowest_perf, caps->highest_perf);
}

static int cppc_cpufreq_set_target(struct cpufreq_policy *policy,
				   unsigned int target_freq,
				   unsigned int relation)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	struct cppc_perf_ctrls perf_ctrls = {};
	unsigned int cpu = policy->cpu;
	struct cpufreq_freqs freqs;
	u32 desired_perf;
	int ret = 0;

	desired_perf = cppc_khz_to_perf(&cpu_data->perf_caps, target_freq);
	cppc_cpufreq_get_perf_limits(cpu_data, policy, &perf_ctrls.min_perf,
				     &perf_ctrls.max_perf);
	perf_ctrls.min_perf_valid = true;

	/*
	 * Keep the cached Desired within [MIN, MAX]. Rounding can put it just
	 * outside at the ends of the range. The policy limits may also change
	 * after the governor picked target_freq.
	 */
	perf_ctrls.desired_perf = clamp(desired_perf, perf_ctrls.min_perf,
					perf_ctrls.max_perf);

	freqs.old = policy->cur;
	freqs.new = target_freq;

	cpufreq_freq_transition_begin(policy, &freqs);
	ret = cppc_set_perf(cpu, &perf_ctrls);
	cpufreq_freq_transition_end(policy, &freqs, ret != 0);

	if (ret) {
		WRITE_ONCE(cpu_data->perf_limits_valid, false);
		pr_debug("Failed to set target on CPU:%d. ret:%d\n",
			 cpu, ret);
		return ret;
	}

	WRITE_ONCE(cpu_data->perf_ctrls.desired_perf,
		   perf_ctrls.desired_perf);
	WRITE_ONCE(cpu_data->perf_ctrls.min_perf, perf_ctrls.min_perf);
	WRITE_ONCE(cpu_data->perf_ctrls.max_perf, perf_ctrls.max_perf);
	WRITE_ONCE(cpu_data->perf_limits_valid, true);

	return 0;
}

static unsigned int cppc_cpufreq_fast_switch(struct cpufreq_policy *policy,
					      unsigned int target_freq)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	struct cppc_perf_ctrls perf_ctrls = {};
	unsigned int cpu = policy->cpu;
	u32 desired_perf, min_perf, max_perf;
	bool update_limits;
	int ret;

	desired_perf = cppc_khz_to_perf(&cpu_data->perf_caps, target_freq);
	cppc_cpufreq_get_perf_limits(cpu_data, policy, &min_perf, &max_perf);

	/*
	 * cppc_set_perf() does not check a Desired-only request, and rounding
	 * or a policy limit change can put Desired outside [MIN, MAX].
	 */
	desired_perf = clamp(desired_perf, min_perf, max_perf);
	perf_ctrls.desired_perf = desired_perf;
	update_limits = !READ_ONCE(cpu_data->perf_limits_valid) ||
			min_perf != READ_ONCE(cpu_data->perf_ctrls.min_perf) ||
			max_perf != READ_ONCE(cpu_data->perf_ctrls.max_perf);
	if (update_limits) {
		perf_ctrls.min_perf = min_perf;
		perf_ctrls.max_perf = max_perf;
		perf_ctrls.min_perf_valid = true;
	}

	ret = cppc_set_perf(cpu, &perf_ctrls);
	if (ret) {
		WRITE_ONCE(cpu_data->perf_limits_valid, false);
		pr_debug("Failed to set target on CPU:%d. ret:%d\n",
			 cpu, ret);
		return 0;
	}

	WRITE_ONCE(cpu_data->perf_ctrls.desired_perf, desired_perf);
	if (update_limits) {
		WRITE_ONCE(cpu_data->perf_ctrls.min_perf, min_perf);
		WRITE_ONCE(cpu_data->perf_ctrls.max_perf, max_perf);
		WRITE_ONCE(cpu_data->perf_limits_valid, true);
	}

	return target_freq;
}

static int cppc_verify_policy(struct cpufreq_policy_data *policy)
{
	cpufreq_verify_within_cpu_limits(policy);
	return 0;
}

static unsigned int __cppc_cpufreq_get_transition_delay_us(unsigned int cpu)
{
	int transition_latency_ns = cppc_get_transition_latency(cpu);

	if (transition_latency_ns < 0)
		return CPUFREQ_DEFAULT_TRANSITION_LATENCY_NS / NSEC_PER_USEC;

	return transition_latency_ns / NSEC_PER_USEC;
}

/*
 * The PCC subspace describes the rate at which platform can accept commands
 * on the shared PCC channel (including READs which do not count towards freq
 * transition requests), so ideally we need to use the PCC values as a fallback
 * if we don't have a platform specific transition_delay_us
 */
#ifdef CONFIG_ARM64
#include <asm/cputype.h>

static unsigned int cppc_cpufreq_get_transition_delay_us(unsigned int cpu)
{
	unsigned long implementor = read_cpuid_implementor();
	unsigned long part_num = read_cpuid_part_number();

	switch (implementor) {
	case ARM_CPU_IMP_QCOM:
		switch (part_num) {
		case QCOM_CPU_PART_FALKOR_V1:
		case QCOM_CPU_PART_FALKOR:
			return 10000;
		}
	}
	return __cppc_cpufreq_get_transition_delay_us(cpu);
}
#else
static unsigned int cppc_cpufreq_get_transition_delay_us(unsigned int cpu)
{
	return __cppc_cpufreq_get_transition_delay_us(cpu);
}
#endif

#if defined(CONFIG_ARM64) && defined(CONFIG_ENERGY_MODEL)

static DEFINE_PER_CPU(unsigned int, efficiency_class);

/* Create an artificial performance state every CPPC_EM_CAP_STEP capacity unit. */
#define CPPC_EM_CAP_STEP	(20)
/* Increase the cost value by CPPC_EM_COST_STEP every performance state. */
#define CPPC_EM_COST_STEP	(1)
/* Add a cost gap correspnding to the energy of 4 CPUs. */
#define CPPC_EM_COST_GAP	(4 * SCHED_CAPACITY_SCALE * CPPC_EM_COST_STEP \
				/ CPPC_EM_CAP_STEP)

static unsigned int get_perf_level_count(struct cpufreq_policy *policy)
{
	struct cppc_perf_caps *perf_caps;
	unsigned int min_cap, max_cap;
	struct cppc_cpudata *cpu_data;
	int cpu = policy->cpu;

	cpu_data = policy->driver_data;
	perf_caps = &cpu_data->perf_caps;
	max_cap = arch_scale_cpu_capacity(cpu);
	min_cap = div_u64((u64)max_cap * perf_caps->lowest_perf,
			  perf_caps->highest_perf);
	if ((min_cap == 0) || (max_cap < min_cap))
		return 0;
	return 1 + max_cap / CPPC_EM_CAP_STEP - min_cap / CPPC_EM_CAP_STEP;
}

/*
 * The cost is defined as:
 *   cost = power * max_frequency / frequency
 */
static inline unsigned long compute_cost(int cpu, int step)
{
	return CPPC_EM_COST_GAP * per_cpu(efficiency_class, cpu) +
			step * CPPC_EM_COST_STEP;
}

static int cppc_get_cpu_power(struct device *cpu_dev,
		unsigned long *power, unsigned long *KHz)
{
	unsigned long perf_step, perf_prev, perf, perf_check;
	unsigned int min_step, max_step, step, step_check;
	unsigned long prev_freq = *KHz;
	unsigned int min_cap, max_cap;
	struct cpufreq_policy *policy;

	struct cppc_perf_caps *perf_caps;
	struct cppc_cpudata *cpu_data;

	policy = cpufreq_cpu_get_raw(cpu_dev->id);
	if (!policy)
		return -EINVAL;

	cpu_data = policy->driver_data;
	perf_caps = &cpu_data->perf_caps;
	max_cap = arch_scale_cpu_capacity(cpu_dev->id);
	min_cap = div_u64((u64)max_cap * perf_caps->lowest_perf,
			  perf_caps->highest_perf);
	perf_step = div_u64((u64)CPPC_EM_CAP_STEP * perf_caps->highest_perf,
			    max_cap);
	min_step = min_cap / CPPC_EM_CAP_STEP;
	max_step = max_cap / CPPC_EM_CAP_STEP;

	perf_prev = cppc_khz_to_perf(perf_caps, *KHz);
	step = perf_prev / perf_step;

	if (step > max_step)
		return -EINVAL;

	if (min_step == max_step) {
		step = max_step;
		perf = perf_caps->highest_perf;
	} else if (step < min_step) {
		step = min_step;
		perf = perf_caps->lowest_perf;
	} else {
		step++;
		if (step == max_step)
			perf = perf_caps->highest_perf;
		else
			perf = step * perf_step;
	}

	*KHz = cppc_perf_to_khz(perf_caps, perf);
	perf_check = cppc_khz_to_perf(perf_caps, *KHz);
	step_check = perf_check / perf_step;

	/*
	 * To avoid bad integer approximation, check that new frequency value
	 * increased and that the new frequency will be converted to the
	 * desired step value.
	 */
	while ((*KHz == prev_freq) || (step_check != step)) {
		perf++;
		*KHz = cppc_perf_to_khz(perf_caps, perf);
		perf_check = cppc_khz_to_perf(perf_caps, *KHz);
		step_check = perf_check / perf_step;
	}

	/*
	 * With an artificial EM, only the cost value is used. Still the power
	 * is populated such as 0 < power < EM_MAX_POWER. This allows to add
	 * more sense to the artificial performance states.
	 */
	*power = compute_cost(cpu_dev->id, step);

	return 0;
}

static int cppc_get_cpu_cost(struct device *cpu_dev, unsigned long KHz,
		unsigned long *cost)
{
	unsigned long perf_step, perf_prev;
	struct cppc_perf_caps *perf_caps;
	struct cpufreq_policy *policy;
	struct cppc_cpudata *cpu_data;
	unsigned int max_cap;
	int step;

	policy = cpufreq_cpu_get_raw(cpu_dev->id);
	if (!policy)
		return -EINVAL;

	cpu_data = policy->driver_data;
	perf_caps = &cpu_data->perf_caps;
	max_cap = arch_scale_cpu_capacity(cpu_dev->id);

	perf_prev = cppc_khz_to_perf(perf_caps, KHz);
	perf_step = CPPC_EM_CAP_STEP * perf_caps->highest_perf / max_cap;
	step = perf_prev / perf_step;

	*cost = compute_cost(cpu_dev->id, step);

	return 0;
}

static void cppc_cpufreq_register_em(struct cpufreq_policy *policy)
{
	struct cppc_cpudata *cpu_data;
	struct em_data_callback em_cb =
		EM_ADV_DATA_CB(cppc_get_cpu_power, cppc_get_cpu_cost);

	cpu_data = policy->driver_data;
	em_dev_register_perf_domain(get_cpu_device(policy->cpu),
			get_perf_level_count(policy), &em_cb,
			cpu_data->shared_cpu_map, 0);
}

static void populate_efficiency_class(void)
{
	struct acpi_madt_generic_interrupt *gicc;
	DECLARE_BITMAP(used_classes, 256) = {};
	int class, cpu, index;

	for_each_possible_cpu(cpu) {
		gicc = acpi_cpu_get_madt_gicc(cpu);
		class = gicc->efficiency_class;
		bitmap_set(used_classes, class, 1);
	}

	if (bitmap_weight(used_classes, 256) <= 1) {
		pr_debug("Efficiency classes are all equal (=%d). "
			"No EM registered", class);
		return;
	}

	/*
	 * Squeeze efficiency class values on [0:#efficiency_class-1].
	 * Values are per spec in [0:255].
	 */
	index = 0;
	for_each_set_bit(class, used_classes, 256) {
		for_each_possible_cpu(cpu) {
			gicc = acpi_cpu_get_madt_gicc(cpu);
			if (gicc->efficiency_class == class)
				per_cpu(efficiency_class, cpu) = index;
		}
		index++;
	}
	cppc_cpufreq_driver.register_em = cppc_cpufreq_register_em;
}

#else
static void populate_efficiency_class(void)
{
}
#endif

static struct cppc_cpudata *cppc_cpufreq_get_cpu_data(unsigned int cpu)
{
	struct cppc_cpudata *cpu_data;
	int ret;

	cpu_data = kzalloc_obj(struct cppc_cpudata);
	if (!cpu_data)
		goto out;

	if (!zalloc_cpumask_var(&cpu_data->shared_cpu_map, GFP_KERNEL))
		goto free_cpu;

	ret = acpi_get_psd_map(cpu, cpu_data);
	if (ret) {
		pr_debug("Err parsing CPU%d PSD data: ret:%d\n", cpu, ret);
		goto free_mask;
	}

	ret = cppc_get_perf_caps(cpu, &cpu_data->perf_caps);
	if (ret) {
		pr_debug("Err reading CPU%d perf caps: ret:%d\n", cpu, ret);
		goto free_mask;
	}

	ret = cppc_get_perf(cpu, &cpu_data->perf_ctrls);
	if (ret) {
		pr_debug("Err reading CPU%d perf ctrls: ret:%d\n", cpu, ret);
		goto free_mask;
	}

	return cpu_data;

free_mask:
	free_cpumask_var(cpu_data->shared_cpu_map);
free_cpu:
	kfree(cpu_data);
out:
	return NULL;
}

static void cppc_cpufreq_put_cpu_data(struct cpufreq_policy *policy)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;

	free_cpumask_var(cpu_data->shared_cpu_map);
	kfree(cpu_data);
	policy->driver_data = NULL;
}

static int cppc_cpufreq_cpu_init(struct cpufreq_policy *policy)
{
	unsigned int cpu = policy->cpu;
	struct cppc_cpudata *cpu_data;
	struct cppc_perf_caps *caps;
	u32 desired_perf;
	int ret;

	cpu_data = cppc_cpufreq_get_cpu_data(cpu);
	if (!cpu_data) {
		pr_err("Error in acquiring _CPC/_PSD data for CPU%d.\n", cpu);
		return -ENODEV;
	}
	caps = &cpu_data->perf_caps;
	policy->driver_data = cpu_data;

	/*
	 * Set min to lowest nonlinear perf to avoid any efficiency penalty (see
	 * Section 8.4.7.1.1.5 of ACPI 6.1 spec)
	 */
	policy->min = cppc_perf_to_khz(caps, caps->lowest_nonlinear_perf);

	/*
	 * Set cpuinfo.min_freq to Lowest to make the full range of performance
	 * available if userspace wants to use any perf between lowest & lowest
	 * nonlinear perf
	 */
	policy->cpuinfo.min_freq = cppc_perf_to_khz(caps, caps->lowest_perf);
	policy->cpuinfo.max_freq = cppc_perf_to_khz(caps, policy->boost_enabled ?
						    caps->highest_perf : caps->nominal_perf);

	policy->transition_delay_us = cppc_cpufreq_get_transition_delay_us(cpu);
	policy->shared_type = cpu_data->shared_type;

	switch (policy->shared_type) {
	case CPUFREQ_SHARED_TYPE_HW:
	case CPUFREQ_SHARED_TYPE_NONE:
		/* Nothing to be done - we'll have a policy for each CPU */
		break;
	case CPUFREQ_SHARED_TYPE_ANY:
		/*
		 * All CPUs in the domain will share a policy and all cpufreq
		 * operations will use a single cppc_cpudata structure stored
		 * in policy->driver_data.
		 */
		cpumask_copy(policy->cpus, cpu_data->shared_cpu_map);
		break;
	default:
		pr_debug("Unsupported CPU co-ord type: %d\n",
			 policy->shared_type);
		ret = -EFAULT;
		goto out;
	}

	policy->fast_switch_possible = cppc_allow_fast_switch(policy->cpus);
	policy->dvfs_possible_from_any_cpu = true;

	/*
	 * If 'highest_perf' is greater than 'nominal_perf', we assume CPU Boost
	 * is supported.
	 */
	if (caps->highest_perf > caps->nominal_perf)
		policy->boost_supported = true;

	/*
	 * The request starts from the live limits, which may be unusable.
	 * Firmware may leave them zeroed. An interrupted earlier update may
	 * have left MIN above MAX. cppc_set_perf() would reject either, so
	 * request the full range instead.
	 */
	if (!cpu_data->perf_ctrls.max_perf ||
	    cpu_data->perf_ctrls.min_perf > cpu_data->perf_ctrls.max_perf) {
		cpu_data->perf_ctrls.min_perf = caps->lowest_perf;
		cpu_data->perf_ctrls.min_perf_valid = true;
		cpu_data->perf_ctrls.max_perf = caps->highest_perf;
	}

	/*
	 * Keep the initial request within controls left by an earlier driver
	 * instance or firmware. The governor will update the full tuple later.
	 */
	desired_perf = caps->highest_perf;
	if (cpu_data->perf_ctrls.max_perf)
		desired_perf = min(desired_perf, cpu_data->perf_ctrls.max_perf);
	if (cpu_data->perf_ctrls.min_perf_valid)
		desired_perf = max(desired_perf, cpu_data->perf_ctrls.min_perf);

	policy->cur = cppc_perf_to_khz(caps, desired_perf);
	cpu_data->perf_ctrls.desired_perf = desired_perf;

	cppc_cpufreq_policy_state(policy)->suspend_regs_handled = false;
	ret = cppc_cpufreq_save_firmware_regs(policy);
	if (ret) {
		pr_debug("Err reading CPU%d firmware values: ret:%d\n", cpu, ret);
		goto out;
	}

	ret = cppc_set_perf(cpu, &cpu_data->perf_ctrls);
	if (ret) {
		pr_debug("Err setting perf value:%d on CPU:%d. ret:%d\n",
			 desired_perf, cpu, ret);
		goto out;
	}
	cpu_data->perf_limits_valid = true;

	cppc_cpufreq_cpu_fie_init(policy);
	return 0;

out:
	cppc_cpufreq_put_cpu_data(policy);
	return ret;
}

static void cppc_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	/*
	 * The core always calls offline() first, so only the driver data is
	 * left to free.
	 */
	cppc_cpufreq_put_cpu_data(policy);
}

/*
 * Run when the policy's first CPU comes back online, the counterpart of
 * offline().
 *
 * The platform may have disabled CPPC and reset the performance controls
 * (desired, min and max performance) while the CPU was offline. Re-enable
 * CPPC, reprogram the controls and reapply the requested values of the
 * OSPM-set registers, which offline() reset to firmware values.
 *
 * Log failures but do not return them, as the core would free the policy.
 * Continue after a failure to enable CPPC, as the writes that follow fail
 * on their own if the platform does not take them. If the controls write
 * fails, still reapply the OSPM-set registers, as frequency changes rewrite
 * only the controls.
 */
static int cppc_cpufreq_cpu_online(struct cpufreq_policy *policy)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	u32 desired_perf, min_perf, max_perf;
	struct cppc_perf_ctrls perf_ctrls = {};
	unsigned int cpu = policy->cpu;
	int ret;

	cppc_cpufreq_policy_state(policy)->suspend_regs_handled = false;

	/*
	 * The platform may have reset MIN and MAX, so mark the copy that
	 * fast_switch() uses as out of date until the restore below succeeds.
	 */
	WRITE_ONCE(cpu_data->perf_limits_valid, false);

	ret = cppc_set_enable(cpu, true);
	if (ret && ret != -EOPNOTSUPP)
		pr_warn("Failed to re-enable CPPC for CPU%u (%d)\n", cpu, ret);

	/*
	 * Recompute min/max from the policy, clamp the last request into range,
	 * and reprogram the performance controls. Update the driver's copy only
	 * if the write succeeds. Otherwise the next frequency change writes the
	 * limits again.
	 */
	cppc_cpufreq_get_perf_limits(cpu_data, policy, &min_perf, &max_perf);
	desired_perf = clamp_t(u32, cpu_data->perf_ctrls.desired_perf,
			       min_perf, max_perf);

	perf_ctrls.desired_perf = desired_perf;
	perf_ctrls.min_perf = min_perf;
	perf_ctrls.max_perf = max_perf;
	perf_ctrls.min_perf_valid = true;

	ret = cppc_set_perf(cpu, &perf_ctrls);
	if (ret) {
		pr_debug("Failed to restore perf controls on CPU%u (%d)\n",
			 cpu, ret);
	} else {
		WRITE_ONCE(cpu_data->perf_ctrls.desired_perf, desired_perf);
		WRITE_ONCE(cpu_data->perf_ctrls.min_perf, min_perf);
		WRITE_ONCE(cpu_data->perf_ctrls.max_perf, max_perf);
		WRITE_ONCE(cpu_data->perf_limits_valid, true);
	}

	cppc_cpufreq_apply_saved_regs(policy, CPPC_SAVED_REQUESTED);

	/* Restart what offline() stopped, with a new counter snapshot. */
	cppc_cpufreq_cpu_fie_init(policy);

	return 0;
}

/*
 * Run when the policy's last online CPU goes down, the counterpart of online().
 * Having offline() makes the core keep the policy rather than call exit().
 */
static int cppc_cpufreq_cpu_offline(struct cpufreq_policy *policy)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	struct cppc_perf_ctrls perf_ctrls = cpu_data->perf_ctrls;
	unsigned int cpu = policy->cpu;
	int ret;

	/*
	 * Leave the platform in its pre-driver state, unless suspend() already
	 * did so earlier in this suspend cycle.
	 */
	if (!cppc_cpufreq_policy_state(policy)->suspend_regs_handled)
		cppc_cpufreq_apply_saved_regs(policy, CPPC_SAVED_FIRMWARE);

	/*
	 * Stop the frequency invariance updates and cancel the pending work, so
	 * that no sample spans the offline window. online() restarts them with
	 * a new counter snapshot.
	 */
	cppc_cpufreq_cpu_fie_exit(policy);

	/*
	 * Request the lowest performance while the policy has no online CPU,
	 * lowering MIN with it so that the request stays a valid tuple. MAX
	 * keeps the value the policy last programmed.
	 */
	perf_ctrls.desired_perf = cpu_data->perf_caps.lowest_perf;
	perf_ctrls.min_perf = cpu_data->perf_caps.lowest_perf;
	perf_ctrls.min_perf_valid = true;

	ret = cppc_set_perf(cpu, &perf_ctrls);
	if (ret)
		pr_debug("Err setting perf value:%u on CPU:%u. ret:%d\n",
			 cpu_data->perf_caps.lowest_perf, cpu, ret);

	return 0;
}

/*
 * Run for every active policy when the system suspends, before any CPU goes
 * offline.
 *
 * Restore the firmware values. Doing it here covers a policy that keeps a CPU
 * online, for which offline() never runs. offline() does not write them again
 * in the same suspend cycle.
 *
 * Stop the frequency invariance updates too, so that no sample spans the
 * suspend window. online() restarts them on the way back, from hotplug or
 * from resume().
 */
static int cppc_cpufreq_cpu_suspend(struct cpufreq_policy *policy)
{
	cppc_cpufreq_apply_saved_regs(policy, CPPC_SAVED_FIRMWARE);
	cppc_cpufreq_policy_state(policy)->suspend_regs_handled = true;

	cppc_cpufreq_cpu_fie_exit(policy);

	return 0;
}

/*
 * Run the online() restore for a policy that kept a CPU online through
 * suspend.
 *
 * For any other policy, online() ran when its first CPU came back, before the
 * core calls resume(). That cleared the flag. Only a policy that still has it
 * set needs online() here. Either way every policy goes through online(), so
 * resume() does not need to mark the copy of MIN and MAX that fast_switch()
 * uses as out of date.
 */
static int cppc_cpufreq_cpu_resume(struct cpufreq_policy *policy)
{
	if (!cppc_cpufreq_policy_state(policy)->suspend_regs_handled)
		return 0;

	return cppc_cpufreq_cpu_online(policy);
}

static inline u64 get_delta(u64 t1, u64 t0)
{
	if (t1 > t0 || t0 > ~(u32)0)
		return t1 - t0;

	return (u32)t1 - (u32)t0;
}

static int cppc_perf_from_fbctrs(u64 reference_perf,
				 struct cppc_perf_fb_ctrs *fb_ctrs_t0,
				 struct cppc_perf_fb_ctrs *fb_ctrs_t1)
{
	u64 delta_reference, delta_delivered;

	delta_reference = get_delta(fb_ctrs_t1->reference,
				    fb_ctrs_t0->reference);
	delta_delivered = get_delta(fb_ctrs_t1->delivered,
				    fb_ctrs_t0->delivered);

	/*
	 * Avoid divide-by zero and unchanged feedback counters.
	 * Leave it for callers to handle.
	 */
	if (!delta_reference || !delta_delivered)
		return 0;

	return (reference_perf * delta_delivered) / delta_reference;
}

static int cppc_get_perf_ctrs_sample(int cpu,
				     struct cppc_perf_fb_ctrs *fb_ctrs_t0,
				     struct cppc_perf_fb_ctrs *fb_ctrs_t1)
{
	int ret;

	ret = cppc_get_perf_ctrs(cpu, fb_ctrs_t0);
	if (ret)
		return ret;

	udelay(2); /* 2usec delay between sampling */

	return cppc_get_perf_ctrs(cpu, fb_ctrs_t1);
}

static unsigned int cppc_cpufreq_get_rate(unsigned int cpu)
{
	struct cpufreq_policy *policy __free(put_cpufreq_policy) = cpufreq_cpu_get(cpu);
	struct cppc_perf_fb_ctrs fb_ctrs_t0 = {0}, fb_ctrs_t1 = {0};
	struct cppc_cpudata *cpu_data;
	u64 delivered_perf, reference_perf;
	int ret;

	if (!policy)
		return 0;

	cpu_data = policy->driver_data;

	ret = cppc_get_perf_ctrs_sample(cpu, &fb_ctrs_t0, &fb_ctrs_t1);
	if (ret) {
		if (ret == -EFAULT)
			/* Any of the associated CPPC regs is 0. */
			goto out_invalid_counters;
		else
			return 0;
	}

	reference_perf = cpu_data->perf_caps.reference_perf;
	delivered_perf = cppc_perf_from_fbctrs(reference_perf,
					       &fb_ctrs_t0, &fb_ctrs_t1);
	if (!delivered_perf)
		goto out_invalid_counters;

	return cppc_perf_to_khz(&cpu_data->perf_caps, delivered_perf);

out_invalid_counters:
	/*
	 * Feedback counters could be unchanged or 0 when a cpu enters a
	 * low-power idle state, e.g. clock-gated or power-gated.
	 * Use desired perf for reflecting frequency.  Get the latest register
	 * value first as some platforms may update the actual delivered perf
	 * there; if failed, resort to the cached desired perf.
	 */
	if (cppc_get_desired_perf(cpu, &delivered_perf) || !delivered_perf)
		delivered_perf = cpu_data->perf_ctrls.desired_perf;

	return cppc_perf_to_khz(&cpu_data->perf_caps, delivered_perf);
}

static int cppc_cpufreq_set_boost(struct cpufreq_policy *policy, int state)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	struct cppc_perf_caps *caps = &cpu_data->perf_caps;

	if (state)
		policy->cpuinfo.max_freq = cppc_perf_to_khz(caps, caps->highest_perf);
	else
		policy->cpuinfo.max_freq = cppc_perf_to_khz(caps, caps->nominal_perf);

	return 0;
}

static ssize_t show_freqdomain_cpus(struct cpufreq_policy *policy, char *buf)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;

	return cpufreq_show_cpus(cpu_data->shared_cpu_map, buf);
}

static ssize_t show_auto_select(struct cpufreq_policy *policy, char *buf)
{
	u64 val;
	int ret;

	ret = cppc_get_auto_sel(policy->cpu, &val);

	/* show "<unsupported>" when this register is not supported by cpc */
	if (ret == -EOPNOTSUPP)
		return sysfs_emit(buf, "<unsupported>\n");

	if (ret)
		return ret;

	return sysfs_emit(buf, "%llu\n", val);
}

static ssize_t store_auto_select(struct cpufreq_policy *policy,
				 const char *buf, size_t count)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	bool old_auto_sel = cpu_data->perf_ctrls.auto_sel;
	bool val;
	int ret;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = cppc_set_auto_sel(policy->cpu, val);
	if (ret)
		return ret;

	cpu_data->perf_ctrls.auto_sel = val;

	if (val) {
		struct cppc_perf_ctrls perf_ctrls = {};

		/*
		 * When enabling autonomous selection, program MIN_PERF and
		 * MAX_PERF from current policy limits so that the platform
		 * uses the correct performance bounds immediately.
		 */
		cppc_cpufreq_get_perf_limits(cpu_data, policy,
					     &perf_ctrls.min_perf,
					     &perf_ctrls.max_perf);
		perf_ctrls.min_perf_valid = true;
		/* The cached Desired may be outside the new MIN and MAX. */
		perf_ctrls.desired_perf =
			clamp(cpu_data->perf_ctrls.desired_perf,
			      perf_ctrls.min_perf, perf_ctrls.max_perf);

		ret = cppc_set_perf(policy->cpu, &perf_ctrls);
		if (ret) {
			WRITE_ONCE(cpu_data->perf_limits_valid, false);
			cppc_set_auto_sel(policy->cpu, old_auto_sel);
			cpu_data->perf_ctrls.auto_sel = old_auto_sel;
			return ret;
		}
		WRITE_ONCE(cpu_data->perf_ctrls.desired_perf,
			   perf_ctrls.desired_perf);
		WRITE_ONCE(cpu_data->perf_ctrls.min_perf, perf_ctrls.min_perf);
		WRITE_ONCE(cpu_data->perf_ctrls.max_perf, perf_ctrls.max_perf);
		WRITE_ONCE(cpu_data->perf_limits_valid, true);
	}

	cppc_cpufreq_save_requested(policy, CPPC_SAVED_AUTO_SEL, val);

	return count;
}

static ssize_t cppc_cpufreq_sysfs_show_u64(unsigned int cpu,
					   int (*get_func)(int, u64 *),
					   char *buf)
{
	u64 val;
	int ret = get_func((int)cpu, &val);

	if (ret == -EOPNOTSUPP)
		return sysfs_emit(buf, "<unsupported>\n");

	if (ret)
		return ret;

	return sysfs_emit(buf, "%llu\n", val);
}

static ssize_t cppc_cpufreq_sysfs_store_u64(unsigned int cpu,
					    int (*set_func)(int, u64),
					    const char *buf, size_t count)
{
	u64 val;
	int ret;

	ret = kstrtou64(buf, 0, &val);
	if (ret)
		return ret;

	ret = set_func((int)cpu, val);

	return ret ? ret : count;
}

#define CPPC_CPUFREQ_ATTR_RW_U64(_name, _get_func, _set_func)		\
static ssize_t show_##_name(struct cpufreq_policy *policy, char *buf)	\
{									\
	return cppc_cpufreq_sysfs_show_u64(policy->cpu, _get_func, buf);\
}									\
static ssize_t store_##_name(struct cpufreq_policy *policy,		\
			     const char *buf, size_t count)		\
{									\
	return cppc_cpufreq_sysfs_store_u64(policy->cpu, _set_func,	\
					    buf, count);		\
}

static ssize_t show_auto_act_window(struct cpufreq_policy *policy, char *buf)
{
	return cppc_cpufreq_sysfs_show_u64(policy->cpu, cppc_get_auto_act_window,
					   buf);
}

static ssize_t store_auto_act_window(struct cpufreq_policy *policy,
				     const char *buf, size_t count)
{
	u64 val;
	int ret;

	ret = kstrtou64(buf, 0, &val);
	if (ret)
		return ret;

	ret = cppc_set_auto_act_window(policy->cpu, val);
	if (ret)
		return ret;

	cppc_cpufreq_save_requested(policy, CPPC_SAVED_AUTO_ACT_WINDOW, val);

	return count;
}

static ssize_t
show_energy_performance_preference_val(struct cpufreq_policy *policy, char *buf)
{
	return cppc_cpufreq_sysfs_show_u64(policy->cpu, cppc_get_epp_perf, buf);
}

static ssize_t
store_energy_performance_preference_val(struct cpufreq_policy *policy,
					const char *buf, size_t count)
{
	struct cppc_cpudata *cpu_data = policy->driver_data;
	u64 val;
	int ret;

	ret = kstrtou64(buf, 0, &val);
	if (ret)
		return ret;

	ret = cppc_set_epp(policy->cpu, val);
	if (ret)
		return ret;

	cpu_data->perf_ctrls.energy_perf = val;
	cppc_cpufreq_save_requested(policy, CPPC_SAVED_EPP, val);

	return count;
}

static int cppc_get_perf_limited_filtered(int cpu, u64 *perf_limited)
{
	struct cpufreq_policy *policy;
	struct cppc_cpudata *cpu_data;
	int ret;

	ret = cppc_get_perf_limited(cpu, perf_limited);
	if (ret)
		return ret;

	policy = cpufreq_cpu_get_raw(cpu);
	if (!policy)
		return -EINVAL;

	cpu_data = policy->driver_data;

	/*
	 * Desired Excursion is ignored when autonomous selection is
	 * enabled. Clear the bit to avoid exposing meaningless state
	 * to userspace.
	 */
	if (cpu_data && cpu_data->perf_ctrls.auto_sel)
		*perf_limited &= ~CPPC_PERF_LIMITED_DESIRED_EXCURSION;

	return 0;
}

CPPC_CPUFREQ_ATTR_RW_U64(perf_limited, cppc_get_perf_limited_filtered,
			 cppc_set_perf_limited)

cpufreq_freq_attr_ro(freqdomain_cpus);
cpufreq_freq_attr_rw(auto_select);
cpufreq_freq_attr_rw(auto_act_window);
cpufreq_freq_attr_rw(energy_performance_preference_val);
cpufreq_freq_attr_rw(perf_limited);

static struct freq_attr *cppc_cpufreq_attr[] = {
	&freqdomain_cpus,
	&auto_select,
	&auto_act_window,
	&energy_performance_preference_val,
	&perf_limited,
	NULL,
};

static struct cpufreq_driver cppc_cpufreq_driver = {
	.flags = CPUFREQ_CONST_LOOPS | CPUFREQ_NEED_UPDATE_LIMITS,
	.verify = cppc_verify_policy,
	.target = cppc_cpufreq_set_target,
	.get = cppc_cpufreq_get_rate,
	.fast_switch = cppc_cpufreq_fast_switch,
	.init = cppc_cpufreq_cpu_init,
	.exit = cppc_cpufreq_cpu_exit,
	.online = cppc_cpufreq_cpu_online,
	.offline = cppc_cpufreq_cpu_offline,
	.suspend = cppc_cpufreq_cpu_suspend,
	.resume = cppc_cpufreq_cpu_resume,
	.set_boost = cppc_cpufreq_set_boost,
	.attr = cppc_cpufreq_attr,
	.name = "cppc_cpufreq",
};

static int __init cppc_cpufreq_init(void)
{
	int ret;

	if (!acpi_cpc_valid())
		return -ENODEV;

	cppc_freq_invariance_init();
	populate_efficiency_class();

	ret = cpufreq_register_driver(&cppc_cpufreq_driver);
	if (ret)
		cppc_freq_invariance_exit();

	return ret;
}

static void __exit cppc_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&cppc_cpufreq_driver);
	cppc_freq_invariance_exit();
}

module_exit(cppc_cpufreq_exit);
MODULE_AUTHOR("Ashwin Chaugule");
MODULE_DESCRIPTION("CPUFreq driver based on the ACPI CPPC v5.0+ spec");
MODULE_LICENSE("GPL");

late_initcall(cppc_cpufreq_init);

static const struct acpi_device_id cppc_acpi_ids[] __used = {
	{ACPI_PROCESSOR_DEVICE_HID, },
	{}
};

MODULE_DEVICE_TABLE(acpi, cppc_acpi_ids);
