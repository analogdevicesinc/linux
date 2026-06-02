// SPDX-License-Identifier: GPL-2.0+
/*
 * Hazard-pointer module-based torture test facility
 *
 * Copyright (c) 2026 Meta Platforms, Inc. and affiliates.
 *
 * Author: Paul E. McKenney <paulmck@kernel.org>
 */

#define pr_fmt(fmt) fmt

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched/debug.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/torture.h>
#include <linux/hazptr.h>
#include <linux/rcupdate.h>

#include "rcu.h"

MODULE_DESCRIPTION("Hazard-pointer module-based torture test facility");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul E. McKenney <paulmckrcu@meta.com>");

torture_param(int, nreaders, -1, "Number of hazard-pointer reader threads");
torture_param(int, onoff_holdoff, 0, "Time after boot before CPU hotplugs (s)");
torture_param(int, onoff_interval, 0, "Time between CPU hotplugs (jiffies), 0=disable");
torture_param(int, preempt_duration, 0, "Preemption duration (ms), zero to disable");
torture_param(int, preempt_interval, MSEC_PER_SEC, "Interval between preemptions (ms)");
torture_param(int, reader_sleep_us, 0, "Reader sleep duration (us)");
torture_param(int, shuffle_interval, 3, "Number of seconds between shuffles");
torture_param(int, shutdown_secs, 0, "Shutdown time (s), <= zero to disable.");
torture_param(int, stat_interval, 60, "Number of seconds between stats printk()s");
torture_param(int, stutter, 5, "Number of seconds to run/halt test");
torture_param(int, verbose, 1, "Enable verbose debugging printk()s");

static char *torture_type = "hazptr";
module_param(torture_type, charp, 0444);
MODULE_PARM_DESC(torture_type, "Type of hazard pointers to torture (hazptr, ...)");

static int nrealreaders;
static struct task_struct *writer_task;
static struct task_struct *preempt_task;
static struct task_struct **reader_tasks;
static struct task_struct *stats_task;

#define HAZPTR_TORTURE_PIPE_LEN 10

// Update-side data structure used to check RCU readers.
struct hazptr_torture {
	void *obj_hazptr;
	int htort_pipe_count;
	struct list_head htort_free;
};

static LIST_HEAD(hazptr_torture_freelist);
static struct hazptr_torture *hazptr_torture_current;
static unsigned long hazptr_torture_current_version;
static struct hazptr_torture hazptr_tortures[10 * HAZPTR_TORTURE_PIPE_LEN];
static DEFINE_SPINLOCK(hazptr_torture_lock);
static DEFINE_PER_CPU(long [HAZPTR_TORTURE_PIPE_LEN + 1], hazptr_torture_count);
static atomic_t hazptr_torture_wcount[HAZPTR_TORTURE_PIPE_LEN + 1];
static atomic_t n_hazptr_torture_alloc;
static atomic_t n_hazptr_torture_alloc_fail;
static atomic_t n_hazptr_torture_free;
static atomic_t n_hazptr_torture_error;
static struct list_head hazptr_torture_removed;

static int hazptr_torture_writer_state;
#define HTWS_FIXED_DELAY	0
#define HTWS_DELAY		1
#define HTWS_REPLACE		2
#define HTWS_SYNC		3
#define HTWS_STUTTER		4
#define HTWS_STOPPING		5
static const char * const hazptr_torture_writer_state_names[] = {
	"HTWS_FIXED_DELAY",
	"HTWS_DELAY",
	"HTWS_REPLACE",
	"HTWS_SYNC",
	"HTWS_STUTTER",
	"HTWS_STOPPING",
};

static const char *hazptr_torture_writer_state_getname(void)
{
	unsigned int i = READ_ONCE(hazptr_torture_writer_state);

	if (i >= ARRAY_SIZE(hazptr_torture_writer_state_names))
		return "???";
	return hazptr_torture_writer_state_names[i];
}

/*
 * Allocate an element from the hazptr_tortures pool.
 */
static struct hazptr_torture *hazptr_torture_alloc(void)
{
	struct list_head *p;

	spin_lock_bh(&hazptr_torture_lock);
	if (list_empty(&hazptr_torture_freelist)) {
		atomic_inc(&n_hazptr_torture_alloc_fail);
		spin_unlock_bh(&hazptr_torture_lock);
		return NULL;
	}
	atomic_inc(&n_hazptr_torture_alloc);
	p = hazptr_torture_freelist.next;
	list_del_init(p);
	spin_unlock_bh(&hazptr_torture_lock);
	return container_of(p, struct hazptr_torture, htort_free);
}

/*
 * Free an element to the hazptr_tortures pool.
 */
static void
hazptr_torture_free(struct hazptr_torture *p)
{
	atomic_inc(&n_hazptr_torture_free);
	spin_lock_bh(&hazptr_torture_lock);
	list_add_tail(&p->htort_free, &hazptr_torture_freelist);
	spin_unlock_bh(&hazptr_torture_lock);
}

/*
 * Update object in the pipe.  This should be invoked after a suitable time.
 */
static bool
hazptr_torture_pipe_update_one(struct hazptr_torture *rp)
{
	int i;

	i = rp->htort_pipe_count;
	if (i > HAZPTR_TORTURE_PIPE_LEN)
		i = HAZPTR_TORTURE_PIPE_LEN;
	atomic_inc(&hazptr_torture_wcount[i]);
	WRITE_ONCE(rp->htort_pipe_count, i + 1);
	ASSERT_EXCLUSIVE_WRITER(rp->htort_pipe_count);
	if (i + 1 >= HAZPTR_TORTURE_PIPE_LEN)
		return true;
	return false;
}

/*
 * Update all callbacks in the pipe each time period.
 */
static void
hazptr_torture_pipe_update(struct hazptr_torture *old_rp)
{
	struct hazptr_torture *rp;
	struct hazptr_torture *rp1;

	if (old_rp)
		list_add(&old_rp->htort_free, &hazptr_torture_removed);
	list_for_each_entry_safe(rp, rp1, &hazptr_torture_removed, htort_free) {
		if (hazptr_torture_pipe_update_one(rp)) {
			list_del(&rp->htort_free);
			hazptr_torture_free(rp);
		}
	}
}

/*
 * Operations vector for selecting different types of tests.
 */

struct hazptr_torture_ops {
	void (*init)(void);
	void (*cleanup)(void);
	struct hazptr_torture *((*readlock)(struct hazptr_ctx **hcpp));
	void (*read_delay)(struct torture_random_state *rrsp);
	void (*readunlock)(struct hazptr_ctx *hcp, struct hazptr_torture *htp);
	void (*sync)(void *htp);
	int irq_capable;
	int onstack_ctx;
	const char *name;
};

static struct hazptr_torture_ops *cur_ops;

/*
 * Definitions for hazard-pointer torture testing using per-CPU hazptr_ctx
 * structures.
 */

static struct hazptr_torture *hazptr_torture_read_lock(struct hazptr_ctx **hcpp)
{
	struct hazptr_ctx *hcp = kmalloc(sizeof(*hcp), GFP_KERNEL);

	*hcpp = hcp;
	if (!hcp)
		return NULL;
	return (struct hazptr_torture *)hazptr_acquire(hcp, (void *)&hazptr_torture_current);
}

static void hazptr_read_delay(struct torture_random_state *rrsp)
{
	const bool can_sleep = !preempt_count() && !irqs_disabled();
	const unsigned long shortdelay_us = 200;
	unsigned long longdelay_ms = 300;
	const bool short_spin = irqs_disabled() || irq_count();

	 // We want a short delay sometimes to make a reader delay the grace
	 // period, and we want a long delay occasionally to trigger
	 // force_quiescent_state.

	if (!(torture_random(rrsp) % (nrealreaders * 2000 * longdelay_ms))) {
		if (short_spin)
			longdelay_ms = 5; /* Avoid triggering BH limits. */
		mdelay(longdelay_ms);
	}
	if (!(torture_random(rrsp) % (nrealreaders * 2 * shortdelay_us)))
		udelay(shortdelay_us);
	if (can_sleep && !(torture_random(rrsp) % (nrealreaders * 500)))
		torture_preempt_schedule();  /* QS only if preemptible. */
	if (can_sleep && reader_sleep_us > 0)
		torture_hrtimeout_us(reader_sleep_us, 0, NULL);
}

static void hazptr_torture_read_unlock(struct hazptr_ctx *hcp, struct hazptr_torture *htp)
{
	if (hcp) {
		hazptr_release(hcp, htp);
		if (cur_ops->onstack_ctx)
			kfree(hcp);
	}
}

static void hazptr_sync_torture_init(void)
{
	INIT_LIST_HEAD(&hazptr_torture_removed);
}

static struct hazptr_torture_ops hazptr_ops = {
	.init			= hazptr_sync_torture_init,
	.readlock		= hazptr_torture_read_lock,
	.read_delay		= hazptr_read_delay,
	.readunlock		= hazptr_torture_read_unlock,
	.sync			= hazptr_synchronize,
	.irq_capable		= 1,
	.name			= "hazptr"
};

/*
 * Definitions for hazard-pointer torture testing using on-stack
 * hazptr_ctx structures.
 */

static struct hazptr_torture *hazptr_torture_read_lock_stack(struct hazptr_ctx **hcpp)
{
	struct hazptr_torture *htp;

	htp = (struct hazptr_torture *)hazptr_acquire(*hcpp, (void *)&hazptr_torture_current);
	return htp;
}

static struct hazptr_torture_ops hazptr_stack_ops = {
	.init			= hazptr_sync_torture_init,
	.readlock		= hazptr_torture_read_lock_stack,
	.read_delay		= hazptr_read_delay,
	.readunlock		= hazptr_torture_read_unlock,
	.sync			= hazptr_synchronize,
	.irq_capable		= 1,
	.onstack_ctx		= 1,
	.name			= "hazptr-stack"
};

/*
 * Hazard-pointer torture writer kthread.  Repeatedly substitutes a new
 * structure for that pointed to by hazptr_torture_current, freeing the
 * old structure after a series of timeouts (the "pipeline").
 */
static int
hazptr_torture_writer(void *arg)
{
	bool booting_still = false;
	int i;
	unsigned long j;
	int oldnice = task_nice(current);
	struct hazptr_torture *rp;
	struct hazptr_torture *old_rp;
	static DEFINE_TORTURE_RANDOM(rand);
	bool stutter_waited;

	VERBOSE_TOROUT_STRING("hazptr_torture_writer task started");
	// If the system is still booting, let it finish.
	j = jiffies;
	while (!torture_must_stop() && !rcu_inkernel_boot_has_ended()) {
		booting_still = true;
		schedule_timeout_interruptible(HZ);
	}
	if (booting_still)
		pr_alert("%s" TORTURE_FLAG " Waited %lu jiffies for boot to complete.\n",
			 torture_type, jiffies - j);

	do {
		hazptr_torture_writer_state = HTWS_FIXED_DELAY;
		torture_hrtimeout_us(500, 1000, &rand);
		rp = hazptr_torture_alloc();
		if (rp == NULL)
			continue;
		rp->htort_pipe_count = 0;
		ASSERT_EXCLUSIVE_WRITER(rp->htort_pipe_count);
		hazptr_torture_writer_state = HTWS_DELAY;
		udelay(torture_random(&rand) & 0x3ff);
		hazptr_torture_writer_state = HTWS_REPLACE;
		old_rp = READ_ONCE(hazptr_torture_current);
		smp_store_release(&hazptr_torture_current, rp); // Publish new structure.
		smp_wmb(); /* Mods to old_rp must follow smp_store_release() */
		if (old_rp) {
			i = old_rp->htort_pipe_count;
			if (i > HAZPTR_TORTURE_PIPE_LEN)
				i = HAZPTR_TORTURE_PIPE_LEN;
			atomic_inc(&hazptr_torture_wcount[i]);
			WRITE_ONCE(old_rp->htort_pipe_count,
				   old_rp->htort_pipe_count + 1);
			ASSERT_EXCLUSIVE_WRITER(old_rp->htort_pipe_count);

			hazptr_torture_writer_state = HTWS_SYNC;
			cur_ops->sync((void *)old_rp);
			hazptr_torture_pipe_update(old_rp);
		}

		WRITE_ONCE(hazptr_torture_current_version, hazptr_torture_current_version + 1);
		hazptr_torture_writer_state = HTWS_STUTTER;
		stutter_waited = stutter_wait("hazptr_torture_writer");
		if (stutter_waited && !torture_must_stop())
			for (i = 0; i < ARRAY_SIZE(hazptr_tortures); i++)
				if (list_empty(&hazptr_tortures[i].htort_free) &&
				    READ_ONCE(hazptr_torture_current) != &hazptr_tortures[i]) {
					tracing_off();
					WARN(1, "%s: htort_pipe_count: %d\n", __func__, hazptr_tortures[i].htort_pipe_count);
					rcu_ftrace_dump(DUMP_ALL);
					break;
				}
		if (stutter_waited)
			sched_set_normal(current, oldnice);
	} while (!torture_must_stop());
	hazptr_torture_current = NULL;  // Let stats task know that we are done.
	hazptr_torture_writer_state = HTWS_STOPPING;
	torture_kthread_stopping("hazptr_torture_writer");
	return 0;
}

/*
 * Hazard-pointer torture reader kthread.  Repeatedly dereferences
 * hazptr_torture_current, incrementing the corresponding element of the
 * pipeline array.  The counter in the element should never be greater
 * than 1, otherwise, the hazard-pointer implementation is broken.
 */
static int hazptr_torture_reader(void *arg)
{
	struct hazptr_ctx hc;
	struct hazptr_ctx *hcp = &hc;
	struct hazptr_torture *htp;
	unsigned long lastsleep = jiffies;
	long myid = (long)arg;
	int mynumonline = myid % nr_cpu_ids;
	int pipe_count;
	DEFINE_TORTURE_RANDOM(rand);

	VERBOSE_TOROUT_STRING("hazptr_torture_reader task started");
	set_user_nice(current, MAX_NICE);
	do {
		htp = cur_ops->readlock(&hcp);
		if (!htp) {
			schedule_timeout_interruptible(HZ / 10);
			continue;
		}
		if (time_after(jiffies, lastsleep) && !torture_must_stop()) {
			torture_hrtimeout_us(500, 1000, &rand);
			lastsleep = jiffies + 10;
		}
		cur_ops->read_delay(&rand);
		preempt_disable();
		pipe_count = READ_ONCE(htp->htort_pipe_count);
		if (pipe_count > HAZPTR_TORTURE_PIPE_LEN) {
			// Should not happen in a correct RCU implementation,
			// happens quite often for torture_type=busted.
			pipe_count = HAZPTR_TORTURE_PIPE_LEN;
		}
		if (pipe_count > 1)
			rcu_ftrace_dump(DUMP_ALL);
		__this_cpu_inc(hazptr_torture_count[pipe_count]);
		preempt_enable();
		cur_ops->readunlock(hcp, htp);
		while (!torture_must_stop() &&
		       (torture_num_online_cpus() < mynumonline || !rcu_inkernel_boot_has_ended()))
			schedule_timeout_interruptible(HZ / 5);
		stutter_wait("hazptr_torture_reader");
	} while (!torture_must_stop());
	torture_kthread_stopping("hazptr_torture_reader");
	return 0;
}

/*
 * Print torture statistics.  Caller must ensure that there is only one
 * call to this function at a given time!!!  This is normally accomplished
 * by relying on the module system to only have one copy of the module
 * loaded, and then by giving the hazptr_torture_stats kthread full control
 * (or the init/cleanup functions when hazptr_torture_stats thread is
 * not running).
 */
static void
hazptr_torture_stats_print(void)
{
	const char *cp = hazptr_torture_writer_state_getname();
	int cpu;
	int i;
	long pipesummary[HAZPTR_TORTURE_PIPE_LEN + 1] = { 0 };
	long batchsummary[HAZPTR_TORTURE_PIPE_LEN + 1] = { 0 };
	struct hazptr_torture *rtcp;
	static unsigned long rtcv_snap = ULONG_MAX;
	static bool splatted;
	struct task_struct *wtp;

	for_each_possible_cpu(cpu)
		for (i = 0; i < HAZPTR_TORTURE_PIPE_LEN + 1; i++)
			pipesummary[i] += READ_ONCE(per_cpu(hazptr_torture_count, cpu)[i]);
	for (i = HAZPTR_TORTURE_PIPE_LEN; i >= 0; i--) {
		if (pipesummary[i] != 0)
			break;
	} // The value of variable "i" is used later, so don't clobber it!

	pr_alert("%s%s ", torture_type, TORTURE_FLAG);
	rtcp = READ_ONCE(hazptr_torture_current);
	pr_cont("rtc: %p %s: %lu %s tfle: %d rta: %d rtaf: %d rtf: %d ",
		rtcp,
		rtcp && !rcu_stall_is_suppressed_at_boot() ? "ver" : "VER",
		hazptr_torture_current_version,
		cp,
		list_empty(&hazptr_torture_freelist),
		atomic_read(&n_hazptr_torture_alloc),
		atomic_read(&n_hazptr_torture_alloc_fail),
		atomic_read(&n_hazptr_torture_free));
	torture_onoff_stats();

	pr_alert("%s%s ", torture_type, TORTURE_FLAG);
	if (i > 1) {
		pr_cont("%s", "!!! ");
		atomic_inc(&n_hazptr_torture_error);
		WARN_ON_ONCE(i > 1); // Too-short grace period
	}
	pr_cont("Reader Pipe: ");
	for (i = 0; i < HAZPTR_TORTURE_PIPE_LEN + 1; i++)
		pr_cont(" %ld", pipesummary[i]);
	pr_cont("\n");

	pr_alert("%s%s ", torture_type, TORTURE_FLAG);
	pr_cont("Reader Batch: ");
	for (i = 0; i < HAZPTR_TORTURE_PIPE_LEN + 1; i++)
		pr_cont(" %ld", batchsummary[i]);
	pr_cont("\n");

	pr_alert("%s%s ", torture_type, TORTURE_FLAG);
	pr_cont("Free-Block Circulation: ");
	for (i = 0; i < HAZPTR_TORTURE_PIPE_LEN + 1; i++)
		pr_cont(" %d", atomic_read(&hazptr_torture_wcount[i]));
	pr_cont("\n");

	if (rtcv_snap == hazptr_torture_current_version &&
	    READ_ONCE(hazptr_torture_current) &&
	    rcu_inkernel_boot_has_ended()) {
		int __maybe_unused flags = 0;
		unsigned long __maybe_unused gp_seq = 0;

		wtp = READ_ONCE(writer_task);
		pr_alert("??? Writer stall state %s(%d) g%lu f%#x ->state %#x cpu %d\n",
			 hazptr_torture_writer_state_getname(),
			 hazptr_torture_writer_state, gp_seq, flags,
			 wtp == NULL ? ~0U : wtp->__state,
			 wtp == NULL ? -1 : (int)task_cpu(wtp));
		if (!splatted && wtp) {
			sched_show_task(wtp);
			splatted = true;
		}
		rcu_ftrace_dump(DUMP_ALL);
	}
	rtcv_snap = hazptr_torture_current_version;
}

/*
 * Periodically prints torture statistics, if periodic statistics printing
 * was specified via the stat_interval module parameter.
 */
static int
hazptr_torture_stats(void *arg)
{
	VERBOSE_TOROUT_STRING("hazptr_torture_stats task started");
	do {
		schedule_timeout_interruptible(stat_interval * HZ);
		hazptr_torture_stats_print();
		torture_shutdown_absorb("hazptr_torture_stats");
	} while (!torture_must_stop());
	torture_kthread_stopping("hazptr_torture_stats");
	return 0;
}

static void
hazptr_torture_print_module_parms(struct hazptr_torture_ops *cur_ops, const char *tag)
{
	pr_alert("%s" TORTURE_FLAG
		 "--- %s: nreaders=%d "
		 "onoff_interval=%d onoff_holdoff=%d "
		 "preempt_duration=%d preempt_interval=%d "
		 "reader_sleep_us=%d "
		 "shuffle_interval=%d shutdown_secs=%d stat_interval=%d stutter=%d "
		 "verbose=%d\n",
		 torture_type, tag, nrealreaders,
		 onoff_interval, onoff_holdoff,
		 preempt_duration, preempt_interval,
		 reader_sleep_us,
		 shuffle_interval, shutdown_secs, stat_interval, stutter,
		 verbose);
}

// Randomly preempt online CPUs.
static int hazptr_torture_preempt(void *unused)
{
	int cpu = -1;
	DEFINE_TORTURE_RANDOM(rand);

	schedule_timeout_idle(onoff_holdoff * HZ);
	do {
		// Wait for preempt_interval ms with up to 100us fuzz.
		torture_hrtimeout_ms(preempt_interval, 100, &rand);
		// Select online CPU.
		cpu = cpumask_next(cpu, cpu_online_mask);
		if (cpu >= nr_cpu_ids)
			cpu = cpumask_next(-1, cpu_online_mask);
		WARN_ON_ONCE(cpu >= nr_cpu_ids);
		// Move to that CPU, if can't do so, retry later.
		if (torture_sched_setaffinity(current->pid, cpumask_of(cpu), false))
			continue;
		// Preempt at high-ish priority, then reset to normal.
		sched_set_fifo(current);
		torture_sched_setaffinity(current->pid, cpu_present_mask, true);
		mdelay(preempt_duration);
		sched_set_normal(current, 0);
		stutter_wait("hazptr_torture_preempt");
	} while (!torture_must_stop());
	torture_kthread_stopping("hazptr_torture_preempt");
	return 0;
}

static void
hazptr_torture_cleanup(void)
{
	int i;

	if (torture_cleanup_begin())
		return;
	if (!cur_ops) {
		torture_cleanup_end();
		return;
	}

	torture_stop_kthread(hazptr_torture_preempt, preempt_task);
	torture_stop_kthread(hazptr_torture_writer, writer_task);

	if (reader_tasks) {
		for (i = 0; i < nrealreaders; i++)
			torture_stop_kthread(hazptr_torture_reader,
					     reader_tasks[i]);
		kfree(reader_tasks);
		reader_tasks = NULL;
	}

	torture_stop_kthread(hazptr_torture_stats, stats_task);

	/* Do torture-type-specific cleanup operations.  */
	if (cur_ops->cleanup != NULL)
		cur_ops->cleanup();

	hazptr_torture_stats_print();  /* -After- the stats thread is stopped! */
	if (atomic_read(&n_hazptr_torture_error))
		hazptr_torture_print_module_parms(cur_ops, "End of test: FAILURE");
	else if (torture_onoff_failures())
		hazptr_torture_print_module_parms(cur_ops, "End of test: HAZPTR_HOTPLUG");
	else
		hazptr_torture_print_module_parms(cur_ops, "End of test: SUCCESS");
	torture_cleanup_end();
}

static int __init hazptr_torture_init(void)
{
	long i;
	int cpu;
	int firsterr = 0;
	static struct hazptr_torture_ops *torture_ops[] = { &hazptr_ops, &hazptr_stack_ops, };

	if (!torture_init_begin(torture_type, verbose))
		return -EBUSY;

	/* Process args and tell the world that the torturer is on the job. */
	for (i = 0; i < ARRAY_SIZE(torture_ops); i++) {
		cur_ops = torture_ops[i];
		if (strcmp(torture_type, cur_ops->name) == 0)
			break;
	}
	if (i == ARRAY_SIZE(torture_ops)) {
		pr_alert("hazptr-torture: invalid torture type: \"%s\"\n", torture_type);
		pr_alert("hazptr-torture types:");
		for (i = 0; i < ARRAY_SIZE(torture_ops); i++)
			pr_cont(" %s", torture_ops[i]->name);
		pr_cont("\n");
		firsterr = -EINVAL;
		cur_ops = NULL;
		goto unwind;
	}

	if (cur_ops->init)
		cur_ops->init();

	if (nreaders >= 0) {
		nrealreaders = nreaders;
	} else {
		nrealreaders = num_online_cpus() * -nreaders;
		if (nrealreaders <= 0)
			nrealreaders = 1;
	}
	hazptr_torture_print_module_parms(cur_ops, "Start of test");

	/* Set up the freelist. */
	INIT_LIST_HEAD(&hazptr_torture_freelist);
	for (i = 0; i < ARRAY_SIZE(hazptr_tortures); i++)
		list_add_tail(&hazptr_tortures[i].htort_free, &hazptr_torture_freelist);

	/* Initialize the statistics so that each run gets its own numbers. */

	hazptr_torture_current = NULL;
	hazptr_torture_current_version = 0;
	atomic_set(&n_hazptr_torture_alloc, 0);
	atomic_set(&n_hazptr_torture_alloc_fail, 0);
	atomic_set(&n_hazptr_torture_free, 0);
	atomic_set(&n_hazptr_torture_error, 0);
	for (i = 0; i < HAZPTR_TORTURE_PIPE_LEN + 1; i++)
		atomic_set(&hazptr_torture_wcount[i], 0);
	for_each_possible_cpu(cpu) {
		for (i = 0; i < HAZPTR_TORTURE_PIPE_LEN + 1; i++)
			per_cpu(hazptr_torture_count, cpu)[i] = 0;
	}

	/* Start up the kthreads. */

	reader_tasks = kzalloc_objs(reader_tasks[0], nrealreaders);
	for (i = 0; i < nrealreaders; i++) {
		firsterr = torture_create_kthread(hazptr_torture_reader, (void *)i,
						  reader_tasks[i]);
		if (torture_init_error(firsterr))
			goto unwind;
	}

	firsterr = torture_create_kthread(hazptr_torture_writer, NULL, writer_task);
	if (torture_init_error(firsterr))
		goto unwind;

	firsterr = torture_onoff_init(onoff_holdoff * HZ, onoff_interval, NULL);
	if (torture_init_error(firsterr))
		goto unwind;

	if (stat_interval > 0) {
		firsterr = torture_create_kthread(hazptr_torture_stats, NULL, stats_task);
		if (torture_init_error(firsterr))
			goto unwind;
	}
	if (shuffle_interval > 0) {
		firsterr = torture_shuffle_init(shuffle_interval * HZ);
		if (torture_init_error(firsterr))
			goto unwind;
	}
	if (stutter < 0)
		stutter = 0;
	if (stutter) {
		int t;

		t = stutter * HZ;
		firsterr = torture_stutter_init(stutter * HZ, t);
		if (torture_init_error(firsterr))
			goto unwind;
	}
	firsterr = torture_shutdown_init(shutdown_secs, hazptr_torture_cleanup);
	if (torture_init_error(firsterr))
		goto unwind;
	if (preempt_duration > 0) {
		firsterr = torture_create_kthread(hazptr_torture_preempt, NULL, preempt_task);
		if (torture_init_error(firsterr))
			goto unwind;
	}

	torture_init_end();
	return 0;

unwind:
	torture_init_end();
	hazptr_torture_cleanup();
	if (shutdown_secs) {
		WARN_ON(!IS_MODULE(CONFIG_HAZPTR_TORTURE_TEST));
		kernel_power_off();
	}
	return firsterr;
}

module_init(hazptr_torture_init);
module_exit(hazptr_torture_cleanup);
