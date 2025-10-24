// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/base/power/main.c - Where the driver meets power management.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 *
 * The driver model core calls device_pm_add() when a device is registered.
 * This will initialize the embedded device_pm_info object in the device
 * and add it to the list of power-controlled devices. sysfs entries for
 * controlling device power management will also be added.
 *
 * A separate list is used for keeping track of power info, because the power
 * domain dependencies may differ from the ancestral dependencies that the
 * subsystem list maintains.
 */

#define pr_fmt(fmt) "PM: " fmt
#define dev_fmt pr_fmt

#include <linux/device.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/pm-trace.h>
#include <linux/pm_wakeirq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/sched/debug.h>
#include <linux/async.h>
#include <linux/suspend.h>
#include <trace/events/power.h>
#include <linux/cpufreq.h>
#include <linux/devfreq.h>
#include <linux/timer.h>
#include <linux/nmi.h>

#include "../base.h"
#include "power.h"

typedef int (*pm_callback_t)(struct device *);

/*
 * The entries in the dpm_list list are in a depth first order, simply
 * because children are guaranteed to be discovered after parents, and
 * are inserted at the back of the list on discovery.
 *
 * Since device_pm_add() may be called with a device lock held,
 * we must never try to acquire a device lock while holding
 * dpm_list_mutex.
 */

LIST_HEAD(dpm_list);
static LIST_HEAD(dpm_prepared_list);
static LIST_HEAD(dpm_suspended_list);
static LIST_HEAD(dpm_late_early_list);
static LIST_HEAD(dpm_noirq_list);

static DEFINE_MUTEX(dpm_list_mtx);
static pm_message_t pm_transition;

static DEFINE_MUTEX(async_wip_mtx);
static int async_error;

/**
 * pm_hibernate_is_recovering - if recovering from hibernate due to error.
 *
 * Used to query if dev_pm_ops.thaw() is called for normal hibernation case or
 * recovering from some error.
 *
 * Return: true for error case, false for normal case.
 */
bool pm_hibernate_is_recovering(void)
{
	return pm_transition.event == PM_EVENT_RECOVER;
}
EXPORT_SYMBOL_GPL(pm_hibernate_is_recovering);

static const char *pm_verb(int event)
{
	switch (event) {
	case PM_EVENT_SUSPEND:
		return "suspend";
	case PM_EVENT_RESUME:
		return "resume";
	case PM_EVENT_FREEZE:
		return "freeze";
	case PM_EVENT_QUIESCE:
		return "quiesce";
	case PM_EVENT_HIBERNATE:
		return "hibernate";
	case PM_EVENT_THAW:
		return "thaw";
	case PM_EVENT_RESTORE:
		return "restore";
	case PM_EVENT_RECOVER:
		return "recover";
	default:
		return "(unknown PM event)";
	}
}

/**
 * device_pm_sleep_init - Initialize system suspend-related device fields.
 * @dev: Device object being initialized.
 */
void device_pm_sleep_init(struct device *dev)
{
	dev->power.is_prepared = false;
	dev->power.is_suspended = false;
	dev->power.is_noirq_suspended = false;
	dev->power.is_late_suspended = false;
	init_completion(&dev->power.completion);
	complete_all(&dev->power.completion);
	dev->power.wakeup = NULL;
	INIT_LIST_HEAD(&dev->power.entry);
}

/**
 * device_pm_lock - Lock the list of active devices used by the PM core.
 */
void device_pm_lock(void)
{
	mutex_lock(&dpm_list_mtx);
}

/**
 * device_pm_unlock - Unlock the list of active devices used by the PM core.
 */
void device_pm_unlock(void)
{
	mutex_unlock(&dpm_list_mtx);
}

/**
 * device_pm_add - Add a device to the PM core's list of active devices.
 * @dev: Device to add to the list.
 */
void device_pm_add(struct device *dev)
{
	/* Skip PM setup/initialization. */
	if (device_pm_not_required(dev))
		return;

	pr_debug("Adding info for %s:%s\n",
		 dev->bus ? dev->bus->name : "No Bus", dev_name(dev));
	device_pm_check_callbacks(dev);
	mutex_lock(&dpm_list_mtx);
	if (dev->parent && dev->parent->power.is_prepared)
		dev_warn(dev, "parent %s should not be sleeping\n",
			dev_name(dev->parent));
	list_add_tail(&dev->power.entry, &dpm_list);
	dev->power.in_dpm_list = true;
	mutex_unlock(&dpm_list_mtx);
}

/**
 * device_pm_remove - Remove a device from the PM core's list of active devices.
 * @dev: Device to be removed from the list.
 */
void device_pm_remove(struct device *dev)
{
	if (device_pm_not_required(dev))
		return;

	pr_debug("Removing info for %s:%s\n",
		 dev->bus ? dev->bus->name : "No Bus", dev_name(dev));
	complete_all(&dev->power.completion);
	mutex_lock(&dpm_list_mtx);
	list_del_init(&dev->power.entry);
	dev->power.in_dpm_list = false;
	mutex_unlock(&dpm_list_mtx);
	device_wakeup_disable(dev);
	pm_runtime_remove(dev);
	device_pm_check_callbacks(dev);
}

/**
 * device_pm_move_before - Move device in the PM core's list of active devices.
 * @deva: Device to move in dpm_list.
 * @devb: Device @deva should come before.
 */
void device_pm_move_before(struct device *deva, struct device *devb)
{
	pr_debug("Moving %s:%s before %s:%s\n",
		 deva->bus ? deva->bus->name : "No Bus", dev_name(deva),
		 devb->bus ? devb->bus->name : "No Bus", dev_name(devb));
	/* Delete deva from dpm_list and reinsert before devb. */
	list_move_tail(&deva->power.entry, &devb->power.entry);
}

/**
 * device_pm_move_after - Move device in the PM core's list of active devices.
 * @deva: Device to move in dpm_list.
 * @devb: Device @deva should come after.
 */
void device_pm_move_after(struct device *deva, struct device *devb)
{
	pr_debug("Moving %s:%s after %s:%s\n",
		 deva->bus ? deva->bus->name : "No Bus", dev_name(deva),
		 devb->bus ? devb->bus->name : "No Bus", dev_name(devb));
	/* Delete deva from dpm_list and reinsert after devb. */
	list_move(&deva->power.entry, &devb->power.entry);
}

/**
 * device_pm_move_last - Move device to end of the PM core's list of devices.
 * @dev: Device to move in dpm_list.
 */
void device_pm_move_last(struct device *dev)
{
	pr_debug("Moving %s:%s to end of list\n",
		 dev->bus ? dev->bus->name : "No Bus", dev_name(dev));
	list_move_tail(&dev->power.entry, &dpm_list);
}

static ktime_t initcall_debug_start(struct device *dev, void *cb)
{
	if (!pm_print_times_enabled)
		return 0;

	dev_info(dev, "calling %ps @ %i, parent: %s\n", cb,
		 task_pid_nr(current),
		 dev->parent ? dev_name(dev->parent) : "none");
	return ktime_get();
}

static void initcall_debug_report(struct device *dev, ktime_t calltime,
				  void *cb, int error)
{
	ktime_t rettime;

	if (!pm_print_times_enabled)
		return;

	rettime = ktime_get();
	dev_info(dev, "%ps returned %d after %Ld usecs\n", cb, error,
		 (unsigned long long)ktime_us_delta(rettime, calltime));
}

/**
 * dpm_wait - Wait for a PM operation to complete.
 * @dev: Device to wait for.
 * @async: If unset, wait only if the device's power.async_suspend flag is set.
 */
static void dpm_wait(struct device *dev, bool async)
{
	if (!dev)
		return;

	if (async || (pm_async_enabled && dev->power.async_suspend))
		wait_for_completion(&dev->power.completion);
}

static int dpm_wait_fn(struct device *dev, void *async_ptr)
{
	dpm_wait(dev, *((bool *)async_ptr));
	return 0;
}

static void dpm_wait_for_children(struct device *dev, bool async)
{
	device_for_each_child(dev, &async, dpm_wait_fn);
}

static void dpm_wait_for_suppliers(struct device *dev, bool async)
{
	struct device_link *link;
	int idx;

	idx = device_links_read_lock();

	/*
	 * If the supplier goes away right after we've checked the link to it,
	 * we'll wait for its completion to change the state, but that's fine,
	 * because the only things that will block as a result are the SRCU
	 * callbacks freeing the link objects for the links in the list we're
	 * walking.
	 */
	dev_for_each_link_to_supplier(link, dev)
		if (READ_ONCE(link->status) != DL_STATE_DORMANT &&
		    !device_link_flag_is_sync_state_only(link->flags))
			dpm_wait(link->supplier, async);

	device_links_read_unlock(idx);
}

static bool dpm_wait_for_superior(struct device *dev, bool async)
{
	struct device *parent;

	/*
	 * If the device is resumed asynchronously and the parent's callback
	 * deletes both the device and the parent itself, the parent object may
	 * be freed while this function is running, so avoid that by reference
	 * counting the parent once more unless the device has been deleted
	 * already (in which case return right away).
	 */
	mutex_lock(&dpm_list_mtx);

	if (!device_pm_initialized(dev)) {
		mutex_unlock(&dpm_list_mtx);
		return false;
	}

	parent = get_device(dev->parent);

	mutex_unlock(&dpm_list_mtx);

	dpm_wait(parent, async);
	put_device(parent);

	dpm_wait_for_suppliers(dev, async);

	/*
	 * If the parent's callback has deleted the device, attempting to resume
	 * it would be invalid, so avoid doing that then.
	 */
	return device_pm_initialized(dev);
}

static void dpm_wait_for_consumers(struct device *dev, bool async)
{
	struct device_link *link;
	int idx;

	idx = device_links_read_lock();

	/*
	 * The status of a device link can only be changed from "dormant" by a
	 * probe, but that cannot happen during system suspend/resume.  In
	 * theory it can change to "dormant" at that time, but then it is
	 * reasonable to wait for the target device anyway (eg. if it goes
	 * away, it's better to wait for it to go away completely and then
	 * continue instead of trying to continue in parallel with its
	 * unregistration).
	 */
	dev_for_each_link_to_consumer(link, dev)
		if (READ_ONCE(link->status) != DL_STATE_DORMANT &&
		    !device_link_flag_is_sync_state_only(link->flags))
			dpm_wait(link->consumer, async);

	device_links_read_unlock(idx);
}

static void dpm_wait_for_subordinate(struct device *dev, bool async)
{
	dpm_wait_for_children(dev, async);
	dpm_wait_for_consumers(dev, async);
}

/**
 * pm_op - Return the PM operation appropriate for given PM event.
 * @ops: PM operations to choose from.
 * @state: PM transition of the system being carried out.
 */
static pm_callback_t pm_op(const struct dev_pm_ops *ops, pm_message_t state)
{
	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		return ops->suspend;
	case PM_EVENT_RESUME:
		return ops->resume;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATE_CALLBACKS
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return ops->freeze;
	case PM_EVENT_HIBERNATE:
		return ops->poweroff;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		return ops->thaw;
	case PM_EVENT_RESTORE:
		return ops->restore;
#endif /* CONFIG_HIBERNATE_CALLBACKS */
	}

	return NULL;
}

/**
 * pm_late_early_op - Return the PM operation appropriate for given PM event.
 * @ops: PM operations to choose from.
 * @state: PM transition of the system being carried out.
 *
 * Runtime PM is disabled for @dev while this function is being executed.
 */
static pm_callback_t pm_late_early_op(const struct dev_pm_ops *ops,
				      pm_message_t state)
{
	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		return ops->suspend_late;
	case PM_EVENT_RESUME:
		return ops->resume_early;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATE_CALLBACKS
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return ops->freeze_late;
	case PM_EVENT_HIBERNATE:
		return ops->poweroff_late;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		return ops->thaw_early;
	case PM_EVENT_RESTORE:
		return ops->restore_early;
#endif /* CONFIG_HIBERNATE_CALLBACKS */
	}

	return NULL;
}

/**
 * pm_noirq_op - Return the PM operation appropriate for given PM event.
 * @ops: PM operations to choose from.
 * @state: PM transition of the system being carried out.
 *
 * The driver of @dev will not receive interrupts while this function is being
 * executed.
 */
static pm_callback_t pm_noirq_op(const struct dev_pm_ops *ops, pm_message_t state)
{
	switch (state.event) {
#ifdef CONFIG_SUSPEND
	case PM_EVENT_SUSPEND:
		return ops->suspend_noirq;
	case PM_EVENT_RESUME:
		return ops->resume_noirq;
#endif /* CONFIG_SUSPEND */
#ifdef CONFIG_HIBERNATE_CALLBACKS
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return ops->freeze_noirq;
	case PM_EVENT_HIBERNATE:
		return ops->poweroff_noirq;
	case PM_EVENT_THAW:
	case PM_EVENT_RECOVER:
		return ops->thaw_noirq;
	case PM_EVENT_RESTORE:
		return ops->restore_noirq;
#endif /* CONFIG_HIBERNATE_CALLBACKS */
	}

	return NULL;
}

static void pm_dev_dbg(struct device *dev, pm_message_t state, const char *info)
{
	dev_dbg(dev, "%s%s%s driver flags: %x\n", info, pm_verb(state.event),
		((state.event & PM_EVENT_SLEEP) && device_may_wakeup(dev)) ?
		", may wakeup" : "", dev->power.driver_flags);
}

static void pm_dev_err(struct device *dev, pm_message_t state, const char *info,
			int error)
{
	dev_err(dev, "failed to %s%s: error %d\n", pm_verb(state.event), info,
		error);
}

static void dpm_show_time(ktime_t starttime, pm_message_t state, int error,
			  const char *info)
{
	ktime_t calltime;
	u64 usecs64;
	int usecs;

	calltime = ktime_get();
	usecs64 = ktime_to_ns(ktime_sub(calltime, starttime));
	do_div(usecs64, NSEC_PER_USEC);
	usecs = usecs64;
	if (usecs == 0)
		usecs = 1;

	pm_pr_dbg("%s%s%s of devices %s after %ld.%03ld msecs\n",
		  info ?: "", info ? " " : "", pm_verb(state.event),
		  error ? "aborted" : "complete",
		  usecs / USEC_PER_MSEC, usecs % USEC_PER_MSEC);
}

static int dpm_run_callback(pm_callback_t cb, struct device *dev,
			    pm_message_t state, const char *info)
{
	ktime_t calltime;
	int error;

	if (!cb)
		return 0;

	calltime = initcall_debug_start(dev, cb);

	pm_dev_dbg(dev, state, info);
	trace_device_pm_callback_start(dev, info, state.event);
	error = cb(dev);
	trace_device_pm_callback_end(dev, error);
	suspend_report_result(dev, cb, error);

	initcall_debug_report(dev, calltime, cb, error);

	return error;
}

#ifdef CONFIG_DPM_WATCHDOG
struct dpm_watchdog {
	struct device		*dev;
	struct task_struct	*tsk;
	struct timer_list	timer;
	bool			fatal;
};

#define DECLARE_DPM_WATCHDOG_ON_STACK(wd) \
	struct dpm_watchdog wd

static bool __read_mostly dpm_watchdog_all_cpu_backtrace;
module_param(dpm_watchdog_all_cpu_backtrace, bool, 0644);
MODULE_PARM_DESC(dpm_watchdog_all_cpu_backtrace,
		 "Backtrace all CPUs on DPM watchdog timeout");

/**
 * dpm_watchdog_handler - Driver suspend / resume watchdog handler.
 * @t: The timer that PM watchdog depends on.
 *
 * Called when a driver has timed out suspending or resuming.
 * There's not much we can do here to recover so panic() to
 * capture a crash-dump in pstore.
 */
static void dpm_watchdog_handler(struct timer_list *t)
{
	struct dpm_watchdog *wd = timer_container_of(wd, t, timer);
	struct timer_list *timer = &wd->timer;
	unsigned int time_left;

	if (wd->fatal) {
		unsigned int this_cpu = smp_processor_id();

		dev_emerg(wd->dev, "**** DPM device timeout ****\n");
		show_stack(wd->tsk, NULL, KERN_EMERG);
		if (dpm_watchdog_all_cpu_backtrace)
			trigger_allbutcpu_cpu_backtrace(this_cpu);
		panic("%s %s: unrecoverable failure\n",
			dev_driver_string(wd->dev), dev_name(wd->dev));
	}

	time_left = CONFIG_DPM_WATCHDOG_TIMEOUT - CONFIG_DPM_WATCHDOG_WARNING_TIMEOUT;
	dev_warn(wd->dev, "**** DPM device timeout after %u seconds; %u seconds until panic ****\n",
		 CONFIG_DPM_WATCHDOG_WARNING_TIMEOUT, time_left);
	show_stack(wd->tsk, NULL, KERN_WARNING);

	wd->fatal = true;
	mod_timer(timer, jiffies + HZ * time_left);
}

/**
 * dpm_watchdog_set - Enable pm watchdog for given device.
 * @wd: Watchdog. Must be allocated on the stack.
 * @dev: Device to handle.
 */
static void dpm_watchdog_set(struct dpm_watchdog *wd, struct device *dev)
{
	struct timer_list *timer = &wd->timer;

	wd->dev = dev;
	wd->tsk = current;
	wd->fatal = CONFIG_DPM_WATCHDOG_TIMEOUT == CONFIG_DPM_WATCHDOG_WARNING_TIMEOUT;

	timer_setup_on_stack(timer, dpm_watchdog_handler, 0);
	/* use same timeout value for both suspend and resume */
	timer->expires = jiffies + HZ * CONFIG_DPM_WATCHDOG_WARNING_TIMEOUT;
	add_timer(timer);
}

/**
 * dpm_watchdog_clear - Disable suspend/resume watchdog.
 * @wd: Watchdog to disable.
 */
static void dpm_watchdog_clear(struct dpm_watchdog *wd)
{
	struct timer_list *timer = &wd->timer;

	timer_delete_sync(timer);
	timer_destroy_on_stack(timer);
}
#else
#define DECLARE_DPM_WATCHDOG_ON_STACK(wd)
#define dpm_watchdog_set(x, y)
#define dpm_watchdog_clear(x)
#endif

/*------------------------- Resume routines -------------------------*/

/**
 * dev_pm_skip_resume - System-wide device resume optimization check.
 * @dev: Target device.
 *
 * Return:
 * - %false if the transition under way is RESTORE.
 * - Return value of dev_pm_skip_suspend() if the transition under way is THAW.
 * - The logical negation of %power.must_resume otherwise (that is, when the
 *   transition under way is RESUME).
 */
bool dev_pm_skip_resume(struct device *dev)
{
	if (pm_transition.event == PM_EVENT_RESTORE)
		return false;

	if (pm_transition.event == PM_EVENT_THAW)
		return dev_pm_skip_suspend(dev);

	return !dev->power.must_resume;
}

static bool is_async(struct device *dev)
{
	return dev->power.async_suspend && pm_async_enabled
		&& !pm_trace_is_enabled();
}

static bool __dpm_async(struct device *dev, async_func_t func)
{
	if (dev->power.work_in_progress)
		return true;

	if (!is_async(dev))
		return false;

	dev->power.work_in_progress = true;

	get_device(dev);

	if (async_schedule_dev_nocall(func, dev))
		return true;

	put_device(dev);

	return false;
}

static bool dpm_async_fn(struct device *dev, async_func_t func)
{
	guard(mutex)(&async_wip_mtx);

	return __dpm_async(dev, func);
}

static int dpm_async_with_cleanup(struct device *dev, void *fn)
{
	guard(mutex)(&async_wip_mtx);

	if (!__dpm_async(dev, fn))
		dev->power.work_in_progress = false;

	return 0;
}

static void dpm_async_resume_children(struct device *dev, async_func_t func)
{
	/*
	 * Prevent racing with dpm_clear_async_state() during initial list
	 * walks in dpm_noirq_resume_devices(), dpm_resume_early(), and
	 * dpm_resume().
	 */
	guard(mutex)(&dpm_list_mtx);

	/*
	 * Start processing "async" children of the device unless it's been
	 * started already for them.
	 */
	device_for_each_child(dev, func, dpm_async_with_cleanup);
}

static void dpm_async_resume_subordinate(struct device *dev, async_func_t func)
{
	struct device_link *link;
	int idx;

	dpm_async_resume_children(dev, func);

	idx = device_links_read_lock();

	/* Start processing the device's "async" consumers. */
	dev_for_each_link_to_consumer(link, dev)
		if (READ_ONCE(link->status) != DL_STATE_DORMANT)
			dpm_async_with_cleanup(link->consumer, func);

	device_links_read_unlock(idx);
}

static void dpm_clear_async_state(struct device *dev)
{
	reinit_completion(&dev->power.completion);
	dev->power.work_in_progress = false;
}

static bool dpm_root_device(struct device *dev)
{
	lockdep_assert_held(&dpm_list_mtx);

	/*
	 * Since this function is required to run under dpm_list_mtx, the
	 * list_empty() below will only return true if the device's list of
	 * consumers is actually empty before calling it.
	 */
	return !dev->parent && list_empty(&dev->links.suppliers);
}

static void async_resume_noirq(void *data, async_cookie_t cookie);

/**
 * device_resume_noirq - Execute a "noirq resume" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being resumed asynchronously.
 *
 * The driver of @dev will not receive interrupts while this function is being
 * executed.
 */
static void device_resume_noirq(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	bool skip_resume;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->power.syscore || dev->power.direct_complete)
		goto Out;

	if (!dev->power.is_noirq_suspended) {
		/*
		 * This means that system suspend has been aborted in the noirq
		 * phase before invoking the noirq suspend callback for the
		 * device, so if device_suspend_late() has left it in suspend,
		 * device_resume_early() should leave it in suspend either in
		 * case the early resume of it depends on the noirq resume that
		 * has not run.
		 */
		if (dev_pm_skip_suspend(dev))
			dev->power.must_resume = false;

		goto Out;
	}

	if (!dpm_wait_for_superior(dev, async))
		goto Out;

	skip_resume = dev_pm_skip_resume(dev);
	/*
	 * If the driver callback is skipped below or by the middle layer
	 * callback and device_resume_early() also skips the driver callback for
	 * this device later, it needs to appear as "suspended" to PM-runtime,
	 * so change its status accordingly.
	 *
	 * Otherwise, the device is going to be resumed, so set its PM-runtime
	 * status to "active" unless its power.smart_suspend flag is clear, in
	 * which case it is not necessary to update its PM-runtime status.
	 */
	if (skip_resume)
		pm_runtime_set_suspended(dev);
	else if (dev_pm_smart_suspend(dev))
		pm_runtime_set_active(dev);

	if (dev->pm_domain) {
		info = "noirq power domain ";
		callback = pm_noirq_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "noirq type ";
		callback = pm_noirq_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "noirq class ";
		callback = pm_noirq_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "noirq bus ";
		callback = pm_noirq_op(dev->bus->pm, state);
	}
	if (callback)
		goto Run;

	if (skip_resume)
		goto Skip;

	if (dev->driver && dev->driver->pm) {
		info = "noirq driver ";
		callback = pm_noirq_op(dev->driver->pm, state);
	}

Run:
	error = dpm_run_callback(callback, dev, state, info);

Skip:
	dev->power.is_noirq_suspended = false;

Out:
	complete_all(&dev->power.completion);
	TRACE_RESUME(error);

	if (error) {
		WRITE_ONCE(async_error, error);
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, state, async ? " async noirq" : " noirq", error);
	}

	dpm_async_resume_subordinate(dev, async_resume_noirq);
}

static void async_resume_noirq(void *data, async_cookie_t cookie)
{
	struct device *dev = data;

	device_resume_noirq(dev, pm_transition, true);
	put_device(dev);
}

static void dpm_noirq_resume_devices(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	trace_suspend_resume(TPS("dpm_resume_noirq"), state.event, true);

	async_error = 0;
	pm_transition = state;

	mutex_lock(&dpm_list_mtx);

	/*
	 * Start processing "async" root devices upfront so they don't wait for
	 * the "sync" devices they don't depend on.
	 */
	list_for_each_entry(dev, &dpm_noirq_list, power.entry) {
		dpm_clear_async_state(dev);
		if (dpm_root_device(dev))
			dpm_async_with_cleanup(dev, async_resume_noirq);
	}

	while (!list_empty(&dpm_noirq_list)) {
		dev = to_device(dpm_noirq_list.next);
		list_move_tail(&dev->power.entry, &dpm_late_early_list);

		if (!dpm_async_fn(dev, async_resume_noirq)) {
			get_device(dev);

			mutex_unlock(&dpm_list_mtx);

			device_resume_noirq(dev, state, false);

			put_device(dev);

			mutex_lock(&dpm_list_mtx);
		}
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, 0, "noirq");
	if (READ_ONCE(async_error))
		dpm_save_failed_step(SUSPEND_RESUME_NOIRQ);

	trace_suspend_resume(TPS("dpm_resume_noirq"), state.event, false);
}

/**
 * dpm_resume_noirq - Execute "noirq resume" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 *
 * Invoke the "noirq" resume callbacks for all devices in dpm_noirq_list and
 * allow device drivers' interrupt handlers to be called.
 */
void dpm_resume_noirq(pm_message_t state)
{
	dpm_noirq_resume_devices(state);

	resume_device_irqs();
	device_wakeup_disarm_wake_irqs();
}

static void async_resume_early(void *data, async_cookie_t cookie);

/**
 * device_resume_early - Execute an "early resume" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being resumed asynchronously.
 *
 * Runtime PM is disabled for @dev while this function is being executed.
 */
static void device_resume_early(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->power.syscore || dev->power.direct_complete)
		goto Out;

	if (!dev->power.is_late_suspended)
		goto Out;

	if (!dpm_wait_for_superior(dev, async))
		goto Out;

	if (dev->pm_domain) {
		info = "early power domain ";
		callback = pm_late_early_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "early type ";
		callback = pm_late_early_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "early class ";
		callback = pm_late_early_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "early bus ";
		callback = pm_late_early_op(dev->bus->pm, state);
	}
	if (callback)
		goto Run;

	if (dev_pm_skip_resume(dev))
		goto Skip;

	if (dev->driver && dev->driver->pm) {
		info = "early driver ";
		callback = pm_late_early_op(dev->driver->pm, state);
	}

Run:
	error = dpm_run_callback(callback, dev, state, info);

Skip:
	dev->power.is_late_suspended = false;

Out:
	TRACE_RESUME(error);

	pm_runtime_enable(dev);
	complete_all(&dev->power.completion);

	if (error) {
		WRITE_ONCE(async_error, error);
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, state, async ? " async early" : " early", error);
	}

	dpm_async_resume_subordinate(dev, async_resume_early);
}

static void async_resume_early(void *data, async_cookie_t cookie)
{
	struct device *dev = data;

	device_resume_early(dev, pm_transition, true);
	put_device(dev);
}

/**
 * dpm_resume_early - Execute "early resume" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 */
void dpm_resume_early(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	trace_suspend_resume(TPS("dpm_resume_early"), state.event, true);

	async_error = 0;
	pm_transition = state;

	mutex_lock(&dpm_list_mtx);

	/*
	 * Start processing "async" root devices upfront so they don't wait for
	 * the "sync" devices they don't depend on.
	 */
	list_for_each_entry(dev, &dpm_late_early_list, power.entry) {
		dpm_clear_async_state(dev);
		if (dpm_root_device(dev))
			dpm_async_with_cleanup(dev, async_resume_early);
	}

	while (!list_empty(&dpm_late_early_list)) {
		dev = to_device(dpm_late_early_list.next);
		list_move_tail(&dev->power.entry, &dpm_suspended_list);

		if (!dpm_async_fn(dev, async_resume_early)) {
			get_device(dev);

			mutex_unlock(&dpm_list_mtx);

			device_resume_early(dev, state, false);

			put_device(dev);

			mutex_lock(&dpm_list_mtx);
		}
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, 0, "early");
	if (READ_ONCE(async_error))
		dpm_save_failed_step(SUSPEND_RESUME_EARLY);

	trace_suspend_resume(TPS("dpm_resume_early"), state.event, false);
}

/**
 * dpm_resume_start - Execute "noirq" and "early" device callbacks.
 * @state: PM transition of the system being carried out.
 */
void dpm_resume_start(pm_message_t state)
{
	dpm_resume_noirq(state);
	dpm_resume_early(state);
}
EXPORT_SYMBOL_GPL(dpm_resume_start);

static void async_resume(void *data, async_cookie_t cookie);

/**
 * device_resume - Execute "resume" callbacks for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being resumed asynchronously.
 */
static void device_resume(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;
	DECLARE_DPM_WATCHDOG_ON_STACK(wd);

	TRACE_DEVICE(dev);
	TRACE_RESUME(0);

	if (dev->power.syscore)
		goto Complete;

	if (!dev->power.is_suspended)
		goto Complete;

	dev->power.is_suspended = false;

	if (dev->power.direct_complete) {
		/*
		 * Allow new children to be added under the device after this
		 * point if it has no PM callbacks.
		 */
		if (dev->power.no_pm_callbacks)
			dev->power.is_prepared = false;

		/* Match the pm_runtime_disable() in device_suspend(). */
		pm_runtime_enable(dev);
		goto Complete;
	}

	if (!dpm_wait_for_superior(dev, async))
		goto Complete;

	dpm_watchdog_set(&wd, dev);
	device_lock(dev);

	/*
	 * This is a fib.  But we'll allow new children to be added below
	 * a resumed device, even if the device hasn't been completed yet.
	 */
	dev->power.is_prepared = false;

	if (dev->pm_domain) {
		info = "power domain ";
		callback = pm_op(&dev->pm_domain->ops, state);
		goto Driver;
	}

	if (dev->type && dev->type->pm) {
		info = "type ";
		callback = pm_op(dev->type->pm, state);
		goto Driver;
	}

	if (dev->class && dev->class->pm) {
		info = "class ";
		callback = pm_op(dev->class->pm, state);
		goto Driver;
	}

	if (dev->bus) {
		if (dev->bus->pm) {
			info = "bus ";
			callback = pm_op(dev->bus->pm, state);
		} else if (dev->bus->resume) {
			info = "legacy bus ";
			callback = dev->bus->resume;
			goto End;
		}
	}

 Driver:
	if (!callback && dev->driver && dev->driver->pm) {
		info = "driver ";
		callback = pm_op(dev->driver->pm, state);
	}

 End:
	error = dpm_run_callback(callback, dev, state, info);

	device_unlock(dev);
	dpm_watchdog_clear(&wd);

 Complete:
	complete_all(&dev->power.completion);

	TRACE_RESUME(error);

	if (error) {
		WRITE_ONCE(async_error, error);
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, state, async ? " async" : "", error);
	}

	dpm_async_resume_subordinate(dev, async_resume);
}

static void async_resume(void *data, async_cookie_t cookie)
{
	struct device *dev = data;

	device_resume(dev, pm_transition, true);
	put_device(dev);
}

/**
 * dpm_resume - Execute "resume" callbacks for non-sysdev devices.
 * @state: PM transition of the system being carried out.
 *
 * Execute the appropriate "resume" callback for all devices whose status
 * indicates that they are suspended.
 */
void dpm_resume(pm_message_t state)
{
	struct device *dev;
	ktime_t starttime = ktime_get();

	trace_suspend_resume(TPS("dpm_resume"), state.event, true);

	pm_transition = state;
	async_error = 0;

	mutex_lock(&dpm_list_mtx);

	/*
	 * Start processing "async" root devices upfront so they don't wait for
	 * the "sync" devices they don't depend on.
	 */
	list_for_each_entry(dev, &dpm_suspended_list, power.entry) {
		dpm_clear_async_state(dev);
		if (dpm_root_device(dev))
			dpm_async_with_cleanup(dev, async_resume);
	}

	while (!list_empty(&dpm_suspended_list)) {
		dev = to_device(dpm_suspended_list.next);
		list_move_tail(&dev->power.entry, &dpm_prepared_list);

		if (!dpm_async_fn(dev, async_resume)) {
			get_device(dev);

			mutex_unlock(&dpm_list_mtx);

			device_resume(dev, state, false);

			put_device(dev);

			mutex_lock(&dpm_list_mtx);
		}
	}
	mutex_unlock(&dpm_list_mtx);
	async_synchronize_full();
	dpm_show_time(starttime, state, 0, NULL);
	if (READ_ONCE(async_error))
		dpm_save_failed_step(SUSPEND_RESUME);

	cpufreq_resume();
	devfreq_resume();
	trace_suspend_resume(TPS("dpm_resume"), state.event, false);
}

/**
 * device_complete - Complete a PM transition for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 */
static void device_complete(struct device *dev, pm_message_t state)
{
	void (*callback)(struct device *) = NULL;
	const char *info = NULL;

	if (dev->power.syscore)
		goto out;

	device_lock(dev);

	if (dev->pm_domain) {
		info = "completing power domain ";
		callback = dev->pm_domain->ops.complete;
	} else if (dev->type && dev->type->pm) {
		info = "completing type ";
		callback = dev->type->pm->complete;
	} else if (dev->class && dev->class->pm) {
		info = "completing class ";
		callback = dev->class->pm->complete;
	} else if (dev->bus && dev->bus->pm) {
		info = "completing bus ";
		callback = dev->bus->pm->complete;
	}

	if (!callback && dev->driver && dev->driver->pm) {
		info = "completing driver ";
		callback = dev->driver->pm->complete;
	}

	if (callback) {
		pm_dev_dbg(dev, state, info);
		callback(dev);
	}

	device_unlock(dev);

out:
	/* If enabling runtime PM for the device is blocked, unblock it. */
	pm_runtime_unblock(dev);
	pm_runtime_put(dev);
}

/**
 * dpm_complete - Complete a PM transition for all non-sysdev devices.
 * @state: PM transition of the system being carried out.
 *
 * Execute the ->complete() callbacks for all devices whose PM status is not
 * DPM_ON (this allows new devices to be registered).
 */
void dpm_complete(pm_message_t state)
{
	struct list_head list;

	trace_suspend_resume(TPS("dpm_complete"), state.event, true);

	INIT_LIST_HEAD(&list);
	mutex_lock(&dpm_list_mtx);
	while (!list_empty(&dpm_prepared_list)) {
		struct device *dev = to_device(dpm_prepared_list.prev);

		get_device(dev);
		dev->power.is_prepared = false;
		list_move(&dev->power.entry, &list);

		mutex_unlock(&dpm_list_mtx);

		trace_device_pm_callback_start(dev, "", state.event);
		device_complete(dev, state);
		trace_device_pm_callback_end(dev, 0);

		put_device(dev);

		mutex_lock(&dpm_list_mtx);
	}
	list_splice(&list, &dpm_list);
	mutex_unlock(&dpm_list_mtx);

	/* Allow device probing and trigger re-probing of deferred devices */
	device_unblock_probing();
	trace_suspend_resume(TPS("dpm_complete"), state.event, false);
}

/**
 * dpm_resume_end - Execute "resume" callbacks and complete system transition.
 * @state: PM transition of the system being carried out.
 *
 * Execute "resume" callbacks for all devices and complete the PM transition of
 * the system.
 */
void dpm_resume_end(pm_message_t state)
{
	dpm_resume(state);
	pm_restore_gfp_mask();
	dpm_complete(state);
}
EXPORT_SYMBOL_GPL(dpm_resume_end);


/*------------------------- Suspend routines -------------------------*/

static bool dpm_leaf_device(struct device *dev)
{
	struct device *child;

	lockdep_assert_held(&dpm_list_mtx);

	child = device_find_any_child(dev);
	if (child) {
		put_device(child);

		return false;
	}

	/*
	 * Since this function is required to run under dpm_list_mtx, the
	 * list_empty() below will only return true if the device's list of
	 * consumers is actually empty before calling it.
	 */
	return list_empty(&dev->links.consumers);
}

static bool dpm_async_suspend_parent(struct device *dev, async_func_t func)
{
	guard(mutex)(&dpm_list_mtx);

	/*
	 * If the device is suspended asynchronously and the parent's callback
	 * deletes both the device and the parent itself, the parent object may
	 * be freed while this function is running, so avoid that by checking
	 * if the device has been deleted already as the parent cannot be
	 * deleted before it.
	 */
	if (!device_pm_initialized(dev))
		return false;

	/* Start processing the device's parent if it is "async". */
	if (dev->parent)
		dpm_async_with_cleanup(dev->parent, func);

	return true;
}

static void dpm_async_suspend_superior(struct device *dev, async_func_t func)
{
	struct device_link *link;
	int idx;

	if (!dpm_async_suspend_parent(dev, func))
		return;

	idx = device_links_read_lock();

	/* Start processing the device's "async" suppliers. */
	dev_for_each_link_to_supplier(link, dev)
		if (READ_ONCE(link->status) != DL_STATE_DORMANT)
			dpm_async_with_cleanup(link->supplier, func);

	device_links_read_unlock(idx);
}

static void dpm_async_suspend_complete_all(struct list_head *device_list)
{
	struct device *dev;

	guard(mutex)(&async_wip_mtx);

	list_for_each_entry_reverse(dev, device_list, power.entry) {
		/*
		 * In case the device is being waited for and async processing
		 * has not started for it yet, let the waiters make progress.
		 */
		if (!dev->power.work_in_progress)
			complete_all(&dev->power.completion);
	}
}

/**
 * resume_event - Return a "resume" message for given "suspend" sleep state.
 * @sleep_state: PM message representing a sleep state.
 *
 * Return a PM message representing the resume event corresponding to given
 * sleep state.
 */
static pm_message_t resume_event(pm_message_t sleep_state)
{
	switch (sleep_state.event) {
	case PM_EVENT_SUSPEND:
		return PMSG_RESUME;
	case PM_EVENT_FREEZE:
	case PM_EVENT_QUIESCE:
		return PMSG_RECOVER;
	case PM_EVENT_HIBERNATE:
		return PMSG_RESTORE;
	}
	return PMSG_ON;
}

static void dpm_superior_set_must_resume(struct device *dev)
{
	struct device_link *link;
	int idx;

	if (dev->parent)
		dev->parent->power.must_resume = true;

	idx = device_links_read_lock();

	dev_for_each_link_to_supplier(link, dev)
		link->supplier->power.must_resume = true;

	device_links_read_unlock(idx);
}

static void async_suspend_noirq(void *data, async_cookie_t cookie);

/**
 * device_suspend_noirq - Execute a "noirq suspend" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being suspended asynchronously.
 *
 * The driver of @dev will not receive interrupts while this function is being
 * executed.
 */
static void device_suspend_noirq(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_SUSPEND(0);

	dpm_wait_for_subordinate(dev, async);

	if (READ_ONCE(async_error))
		goto Complete;

	if (dev->power.syscore || dev->power.direct_complete)
		goto Complete;

	if (dev->pm_domain) {
		info = "noirq power domain ";
		callback = pm_noirq_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "noirq type ";
		callback = pm_noirq_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "noirq class ";
		callback = pm_noirq_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "noirq bus ";
		callback = pm_noirq_op(dev->bus->pm, state);
	}
	if (callback)
		goto Run;

	if (dev_pm_skip_suspend(dev))
		goto Skip;

	if (dev->driver && dev->driver->pm) {
		info = "noirq driver ";
		callback = pm_noirq_op(dev->driver->pm, state);
	}

Run:
	error = dpm_run_callback(callback, dev, state, info);
	if (error) {
		WRITE_ONCE(async_error, error);
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, state, async ? " async noirq" : " noirq", error);
		goto Complete;
	}

Skip:
	dev->power.is_noirq_suspended = true;

	/*
	 * Devices must be resumed unless they are explicitly allowed to be left
	 * in suspend, but even in that case skipping the resume of devices that
	 * were in use right before the system suspend (as indicated by their
	 * runtime PM usage counters and child counters) would be suboptimal.
	 */
	if (!(dev_pm_test_driver_flags(dev, DPM_FLAG_MAY_SKIP_RESUME) &&
	      dev->power.may_skip_resume) || !pm_runtime_need_not_resume(dev))
		dev->power.must_resume = true;

	if (dev->power.must_resume)
		dpm_superior_set_must_resume(dev);

Complete:
	complete_all(&dev->power.completion);
	TRACE_SUSPEND(error);

	if (error || READ_ONCE(async_error))
		return;

	dpm_async_suspend_superior(dev, async_suspend_noirq);
}

static void async_suspend_noirq(void *data, async_cookie_t cookie)
{
	struct device *dev = data;

	device_suspend_noirq(dev, pm_transition, true);
	put_device(dev);
}

static int dpm_noirq_suspend_devices(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	struct device *dev;
	int error;

	trace_suspend_resume(TPS("dpm_suspend_noirq"), state.event, true);

	pm_transition = state;
	async_error = 0;

	mutex_lock(&dpm_list_mtx);

	/*
	 * Start processing "async" leaf devices upfront so they don't need to
	 * wait for the "sync" devices they don't depend on.
	 */
	list_for_each_entry_reverse(dev, &dpm_late_early_list, power.entry) {
		dpm_clear_async_state(dev);
		if (dpm_leaf_device(dev))
			dpm_async_with_cleanup(dev, async_suspend_noirq);
	}

	while (!list_empty(&dpm_late_early_list)) {
		dev = to_device(dpm_late_early_list.prev);

		list_move(&dev->power.entry, &dpm_noirq_list);

		if (dpm_async_fn(dev, async_suspend_noirq))
			continue;

		get_device(dev);

		mutex_unlock(&dpm_list_mtx);

		device_suspend_noirq(dev, state, false);

		put_device(dev);

		mutex_lock(&dpm_list_mtx);

		if (READ_ONCE(async_error)) {
			dpm_async_suspend_complete_all(&dpm_late_early_list);
			/*
			 * Move all devices to the target list to resume them
			 * properly.
			 */
			list_splice_init(&dpm_late_early_list, &dpm_noirq_list);
			break;
		}
	}

	mutex_unlock(&dpm_list_mtx);

	async_synchronize_full();

	error = READ_ONCE(async_error);
	if (error)
		dpm_save_failed_step(SUSPEND_SUSPEND_NOIRQ);

	dpm_show_time(starttime, state, error, "noirq");
	trace_suspend_resume(TPS("dpm_suspend_noirq"), state.event, false);
	return error;
}

/**
 * dpm_suspend_noirq - Execute "noirq suspend" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 *
 * Prevent device drivers' interrupt handlers from being called and invoke
 * "noirq" suspend callbacks for all non-sysdev devices.
 */
int dpm_suspend_noirq(pm_message_t state)
{
	int ret;

	device_wakeup_arm_wake_irqs();
	suspend_device_irqs();

	ret = dpm_noirq_suspend_devices(state);
	if (ret)
		dpm_resume_noirq(resume_event(state));

	return ret;
}

static void dpm_propagate_wakeup_to_parent(struct device *dev)
{
	struct device *parent = dev->parent;

	if (!parent)
		return;

	spin_lock_irq(&parent->power.lock);

	if (device_wakeup_path(dev) && !parent->power.ignore_children)
		parent->power.wakeup_path = true;

	spin_unlock_irq(&parent->power.lock);
}

static void async_suspend_late(void *data, async_cookie_t cookie);

/**
 * device_suspend_late - Execute a "late suspend" callback for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being suspended asynchronously.
 *
 * Runtime PM is disabled for @dev while this function is being executed.
 */
static void device_suspend_late(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;

	TRACE_DEVICE(dev);
	TRACE_SUSPEND(0);

	/*
	 * Disable runtime PM for the device without checking if there is a
	 * pending resume request for it.
	 */
	__pm_runtime_disable(dev, false);

	dpm_wait_for_subordinate(dev, async);

	if (READ_ONCE(async_error))
		goto Complete;

	if (pm_wakeup_pending()) {
		WRITE_ONCE(async_error, -EBUSY);
		goto Complete;
	}

	if (dev->power.syscore || dev->power.direct_complete)
		goto Complete;

	if (dev->pm_domain) {
		info = "late power domain ";
		callback = pm_late_early_op(&dev->pm_domain->ops, state);
	} else if (dev->type && dev->type->pm) {
		info = "late type ";
		callback = pm_late_early_op(dev->type->pm, state);
	} else if (dev->class && dev->class->pm) {
		info = "late class ";
		callback = pm_late_early_op(dev->class->pm, state);
	} else if (dev->bus && dev->bus->pm) {
		info = "late bus ";
		callback = pm_late_early_op(dev->bus->pm, state);
	}
	if (callback)
		goto Run;

	if (dev_pm_skip_suspend(dev))
		goto Skip;

	if (dev->driver && dev->driver->pm) {
		info = "late driver ";
		callback = pm_late_early_op(dev->driver->pm, state);
	}

Run:
	error = dpm_run_callback(callback, dev, state, info);
	if (error) {
		WRITE_ONCE(async_error, error);
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, state, async ? " async late" : " late", error);
		goto Complete;
	}
	dpm_propagate_wakeup_to_parent(dev);

Skip:
	dev->power.is_late_suspended = true;

Complete:
	TRACE_SUSPEND(error);
	complete_all(&dev->power.completion);

	if (error || READ_ONCE(async_error))
		return;

	dpm_async_suspend_superior(dev, async_suspend_late);
}

static void async_suspend_late(void *data, async_cookie_t cookie)
{
	struct device *dev = data;

	device_suspend_late(dev, pm_transition, true);
	put_device(dev);
}

/**
 * dpm_suspend_late - Execute "late suspend" callbacks for all devices.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend_late(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	struct device *dev;
	int error;

	trace_suspend_resume(TPS("dpm_suspend_late"), state.event, true);

	pm_transition = state;
	async_error = 0;

	wake_up_all_idle_cpus();

	mutex_lock(&dpm_list_mtx);

	/*
	 * Start processing "async" leaf devices upfront so they don't need to
	 * wait for the "sync" devices they don't depend on.
	 */
	list_for_each_entry_reverse(dev, &dpm_suspended_list, power.entry) {
		dpm_clear_async_state(dev);
		if (dpm_leaf_device(dev))
			dpm_async_with_cleanup(dev, async_suspend_late);
	}

	while (!list_empty(&dpm_suspended_list)) {
		dev = to_device(dpm_suspended_list.prev);

		list_move(&dev->power.entry, &dpm_late_early_list);

		if (dpm_async_fn(dev, async_suspend_late))
			continue;

		get_device(dev);

		mutex_unlock(&dpm_list_mtx);

		device_suspend_late(dev, state, false);

		put_device(dev);

		mutex_lock(&dpm_list_mtx);

		if (READ_ONCE(async_error)) {
			dpm_async_suspend_complete_all(&dpm_suspended_list);
			/*
			 * Move all devices to the target list to resume them
			 * properly.
			 */
			list_splice_init(&dpm_suspended_list, &dpm_late_early_list);
			break;
		}
	}

	mutex_unlock(&dpm_list_mtx);

	async_synchronize_full();

	error = READ_ONCE(async_error);
	if (error) {
		dpm_save_failed_step(SUSPEND_SUSPEND_LATE);
		dpm_resume_early(resume_event(state));
	}
	dpm_show_time(starttime, state, error, "late");
	trace_suspend_resume(TPS("dpm_suspend_late"), state.event, false);
	return error;
}

/**
 * dpm_suspend_end - Execute "late" and "noirq" device suspend callbacks.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend_end(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	int error;

	error = dpm_suspend_late(state);
	if (error)
		goto out;

	error = dpm_suspend_noirq(state);
	if (error)
		dpm_resume_early(resume_event(state));

out:
	dpm_show_time(starttime, state, error, "end");
	return error;
}
EXPORT_SYMBOL_GPL(dpm_suspend_end);

/**
 * legacy_suspend - Execute a legacy (bus or class) suspend callback for device.
 * @dev: Device to suspend.
 * @state: PM transition of the system being carried out.
 * @cb: Suspend callback to execute.
 * @info: string description of caller.
 */
static int legacy_suspend(struct device *dev, pm_message_t state,
			  int (*cb)(struct device *dev, pm_message_t state),
			  const char *info)
{
	int error;
	ktime_t calltime;

	calltime = initcall_debug_start(dev, cb);

	trace_device_pm_callback_start(dev, info, state.event);
	error = cb(dev, state);
	trace_device_pm_callback_end(dev, error);
	suspend_report_result(dev, cb, error);

	initcall_debug_report(dev, calltime, cb, error);

	return error;
}

static void dpm_clear_superiors_direct_complete(struct device *dev)
{
	struct device_link *link;
	int idx;

	if (dev->parent) {
		spin_lock_irq(&dev->parent->power.lock);
		dev->parent->power.direct_complete = false;
		spin_unlock_irq(&dev->parent->power.lock);
	}

	idx = device_links_read_lock();

	dev_for_each_link_to_supplier(link, dev) {
		spin_lock_irq(&link->supplier->power.lock);
		link->supplier->power.direct_complete = false;
		spin_unlock_irq(&link->supplier->power.lock);
	}

	device_links_read_unlock(idx);
}

static void async_suspend(void *data, async_cookie_t cookie);

/**
 * device_suspend - Execute "suspend" callbacks for given device.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 * @async: If true, the device is being suspended asynchronously.
 */
static void device_suspend(struct device *dev, pm_message_t state, bool async)
{
	pm_callback_t callback = NULL;
	const char *info = NULL;
	int error = 0;
	DECLARE_DPM_WATCHDOG_ON_STACK(wd);

	TRACE_DEVICE(dev);
	TRACE_SUSPEND(0);

	dpm_wait_for_subordinate(dev, async);

	if (READ_ONCE(async_error)) {
		dev->power.direct_complete = false;
		goto Complete;
	}

	/*
	 * Wait for possible runtime PM transitions of the device in progress
	 * to complete and if there's a runtime resume request pending for it,
	 * resume it before proceeding with invoking the system-wide suspend
	 * callbacks for it.
	 *
	 * If the system-wide suspend callbacks below change the configuration
	 * of the device, they must disable runtime PM for it or otherwise
	 * ensure that its runtime-resume callbacks will not be confused by that
	 * change in case they are invoked going forward.
	 */
	pm_runtime_barrier(dev);

	if (pm_wakeup_pending()) {
		dev->power.direct_complete = false;
		WRITE_ONCE(async_error, -EBUSY);
		goto Complete;
	}

	if (dev->power.syscore)
		goto Complete;

	/* Avoid direct_complete to let wakeup_path propagate. */
	if (device_may_wakeup(dev) || device_wakeup_path(dev))
		dev->power.direct_complete = false;

	if (dev->power.direct_complete) {
		if (pm_runtime_status_suspended(dev)) {
			pm_runtime_disable(dev);
			if (pm_runtime_status_suspended(dev)) {
				pm_dev_dbg(dev, state, "direct-complete ");
				dev->power.is_suspended = true;
				goto Complete;
			}

			pm_runtime_enable(dev);
		}
		dev->power.direct_complete = false;
	}

	dev->power.may_skip_resume = true;
	dev->power.must_resume = !dev_pm_test_driver_flags(dev, DPM_FLAG_MAY_SKIP_RESUME);

	dpm_watchdog_set(&wd, dev);
	device_lock(dev);

	if (dev->pm_domain) {
		info = "power domain ";
		callback = pm_op(&dev->pm_domain->ops, state);
		goto Run;
	}

	if (dev->type && dev->type->pm) {
		info = "type ";
		callback = pm_op(dev->type->pm, state);
		goto Run;
	}

	if (dev->class && dev->class->pm) {
		info = "class ";
		callback = pm_op(dev->class->pm, state);
		goto Run;
	}

	if (dev->bus) {
		if (dev->bus->pm) {
			info = "bus ";
			callback = pm_op(dev->bus->pm, state);
		} else if (dev->bus->suspend) {
			pm_dev_dbg(dev, state, "legacy bus ");
			error = legacy_suspend(dev, state, dev->bus->suspend,
						"legacy bus ");
			goto End;
		}
	}

 Run:
	if (!callback && dev->driver && dev->driver->pm) {
		info = "driver ";
		callback = pm_op(dev->driver->pm, state);
	}

	error = dpm_run_callback(callback, dev, state, info);

 End:
	if (!error) {
		dev->power.is_suspended = true;
		if (device_may_wakeup(dev))
			dev->power.wakeup_path = true;

		dpm_propagate_wakeup_to_parent(dev);
		dpm_clear_superiors_direct_complete(dev);
	}

	device_unlock(dev);
	dpm_watchdog_clear(&wd);

 Complete:
	if (error) {
		WRITE_ONCE(async_error, error);
		dpm_save_failed_dev(dev_name(dev));
		pm_dev_err(dev, state, async ? " async" : "", error);
	}

	complete_all(&dev->power.completion);
	TRACE_SUSPEND(error);

	if (error || READ_ONCE(async_error))
		return;

	dpm_async_suspend_superior(dev, async_suspend);
}

static void async_suspend(void *data, async_cookie_t cookie)
{
	struct device *dev = data;

	device_suspend(dev, pm_transition, true);
	put_device(dev);
}

/**
 * dpm_suspend - Execute "suspend" callbacks for all non-sysdev devices.
 * @state: PM transition of the system being carried out.
 */
int dpm_suspend(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	struct device *dev;
	int error;

	trace_suspend_resume(TPS("dpm_suspend"), state.event, true);
	might_sleep();

	devfreq_suspend();
	cpufreq_suspend();

	pm_transition = state;
	async_error = 0;

	mutex_lock(&dpm_list_mtx);

	/*
	 * Start processing "async" leaf devices upfront so they don't need to
	 * wait for the "sync" devices they don't depend on.
	 */
	list_for_each_entry_reverse(dev, &dpm_prepared_list, power.entry) {
		dpm_clear_async_state(dev);
		if (dpm_leaf_device(dev))
			dpm_async_with_cleanup(dev, async_suspend);
	}

	while (!list_empty(&dpm_prepared_list)) {
		dev = to_device(dpm_prepared_list.prev);

		list_move(&dev->power.entry, &dpm_suspended_list);

		if (dpm_async_fn(dev, async_suspend))
			continue;

		get_device(dev);

		mutex_unlock(&dpm_list_mtx);

		device_suspend(dev, state, false);

		put_device(dev);

		mutex_lock(&dpm_list_mtx);

		if (READ_ONCE(async_error)) {
			dpm_async_suspend_complete_all(&dpm_prepared_list);
			/*
			 * Move all devices to the target list to resume them
			 * properly.
			 */
			list_splice_init(&dpm_prepared_list, &dpm_suspended_list);
			break;
		}
	}

	mutex_unlock(&dpm_list_mtx);

	async_synchronize_full();

	error = READ_ONCE(async_error);
	if (error)
		dpm_save_failed_step(SUSPEND_SUSPEND);

	dpm_show_time(starttime, state, error, NULL);
	trace_suspend_resume(TPS("dpm_suspend"), state.event, false);
	return error;
}

static bool device_prepare_smart_suspend(struct device *dev)
{
	struct device_link *link;
	bool ret = true;
	int idx;

	/*
	 * The "smart suspend" feature is enabled for devices whose drivers ask
	 * for it and for devices without PM callbacks.
	 *
	 * However, if "smart suspend" is not enabled for the device's parent
	 * or any of its suppliers that take runtime PM into account, it cannot
	 * be enabled for the device either.
	 */
	if (!dev->power.no_pm_callbacks &&
	    !dev_pm_test_driver_flags(dev, DPM_FLAG_SMART_SUSPEND))
		return false;

	if (dev->parent && !dev_pm_smart_suspend(dev->parent) &&
	    !dev->parent->power.ignore_children && !pm_runtime_blocked(dev->parent))
		return false;

	idx = device_links_read_lock();

	dev_for_each_link_to_supplier(link, dev) {
		if (!device_link_test(link, DL_FLAG_PM_RUNTIME))
			continue;

		if (!dev_pm_smart_suspend(link->supplier) &&
		    !pm_runtime_blocked(link->supplier)) {
			ret = false;
			break;
		}
	}

	device_links_read_unlock(idx);

	return ret;
}

/**
 * device_prepare - Prepare a device for system power transition.
 * @dev: Device to handle.
 * @state: PM transition of the system being carried out.
 *
 * Execute the ->prepare() callback(s) for given device.  No new children of the
 * device may be registered after this function has returned.
 */
static int device_prepare(struct device *dev, pm_message_t state)
{
	int (*callback)(struct device *) = NULL;
	bool smart_suspend;
	int ret = 0;

	/*
	 * If a device's parent goes into runtime suspend at the wrong time,
	 * it won't be possible to resume the device.  To prevent this we
	 * block runtime suspend here, during the prepare phase, and allow
	 * it again during the complete phase.
	 */
	pm_runtime_get_noresume(dev);
	/*
	 * If runtime PM is disabled for the device at this point and it has
	 * never been enabled so far, it should not be enabled until this system
	 * suspend-resume cycle is complete, so prepare to trigger a warning on
	 * subsequent attempts to enable it.
	 */
	smart_suspend = !pm_runtime_block_if_disabled(dev);

	if (dev->power.syscore)
		return 0;

	device_lock(dev);

	dev->power.wakeup_path = false;

	if (dev->power.no_pm_callbacks)
		goto unlock;

	if (dev->pm_domain)
		callback = dev->pm_domain->ops.prepare;
	else if (dev->type && dev->type->pm)
		callback = dev->type->pm->prepare;
	else if (dev->class && dev->class->pm)
		callback = dev->class->pm->prepare;
	else if (dev->bus && dev->bus->pm)
		callback = dev->bus->pm->prepare;

	if (!callback && dev->driver && dev->driver->pm)
		callback = dev->driver->pm->prepare;

	if (callback)
		ret = callback(dev);

unlock:
	device_unlock(dev);

	if (ret < 0) {
		suspend_report_result(dev, callback, ret);
		pm_runtime_put(dev);
		return ret;
	}
	/* Do not enable "smart suspend" for devices with disabled runtime PM. */
	if (smart_suspend)
		smart_suspend = device_prepare_smart_suspend(dev);

	spin_lock_irq(&dev->power.lock);

	dev->power.smart_suspend = smart_suspend;
	/*
	 * A positive return value from ->prepare() means "this device appears
	 * to be runtime-suspended and its state is fine, so if it really is
	 * runtime-suspended, you can leave it in that state provided that you
	 * will do the same thing with all of its descendants".  This only
	 * applies to suspend transitions, however.
	 */
	dev->power.direct_complete = state.event == PM_EVENT_SUSPEND &&
		(ret > 0 || dev->power.no_pm_callbacks) &&
		!dev_pm_test_driver_flags(dev, DPM_FLAG_NO_DIRECT_COMPLETE);

	spin_unlock_irq(&dev->power.lock);

	return 0;
}

/**
 * dpm_prepare - Prepare all non-sysdev devices for a system PM transition.
 * @state: PM transition of the system being carried out.
 *
 * Execute the ->prepare() callback(s) for all devices.
 */
int dpm_prepare(pm_message_t state)
{
	int error = 0;

	trace_suspend_resume(TPS("dpm_prepare"), state.event, true);

	/*
	 * Give a chance for the known devices to complete their probes, before
	 * disable probing of devices. This sync point is important at least
	 * at boot time + hibernation restore.
	 */
	wait_for_device_probe();
	/*
	 * It is unsafe if probing of devices will happen during suspend or
	 * hibernation and system behavior will be unpredictable in this case.
	 * So, let's prohibit device's probing here and defer their probes
	 * instead. The normal behavior will be restored in dpm_complete().
	 */
	device_block_probing();

	mutex_lock(&dpm_list_mtx);
	while (!list_empty(&dpm_list) && !error) {
		struct device *dev = to_device(dpm_list.next);

		get_device(dev);

		mutex_unlock(&dpm_list_mtx);

		trace_device_pm_callback_start(dev, "", state.event);
		error = device_prepare(dev, state);
		trace_device_pm_callback_end(dev, error);

		mutex_lock(&dpm_list_mtx);

		if (!error) {
			dev->power.is_prepared = true;
			if (!list_empty(&dev->power.entry))
				list_move_tail(&dev->power.entry, &dpm_prepared_list);
		} else if (error == -EAGAIN) {
			error = 0;
		} else {
			dev_info(dev, "not prepared for power transition: code %d\n",
				 error);
		}

		mutex_unlock(&dpm_list_mtx);

		put_device(dev);

		mutex_lock(&dpm_list_mtx);
	}
	mutex_unlock(&dpm_list_mtx);
	trace_suspend_resume(TPS("dpm_prepare"), state.event, false);
	return error;
}

/**
 * dpm_suspend_start - Prepare devices for PM transition and suspend them.
 * @state: PM transition of the system being carried out.
 *
 * Prepare all non-sysdev devices for system PM transition and execute "suspend"
 * callbacks for them.
 */
int dpm_suspend_start(pm_message_t state)
{
	ktime_t starttime = ktime_get();
	int error;

	error = dpm_prepare(state);
	if (error)
		dpm_save_failed_step(SUSPEND_PREPARE);
	else {
		pm_restrict_gfp_mask();
		error = dpm_suspend(state);
	}

	dpm_show_time(starttime, state, error, "start");
	return error;
}
EXPORT_SYMBOL_GPL(dpm_suspend_start);

void __suspend_report_result(const char *function, struct device *dev, void *fn, int ret)
{
	if (ret)
		dev_err(dev, "%s(): %ps returns %d\n", function, fn, ret);
}
EXPORT_SYMBOL_GPL(__suspend_report_result);

/**
 * device_pm_wait_for_dev - Wait for suspend/resume of a device to complete.
 * @subordinate: Device that needs to wait for @dev.
 * @dev: Device to wait for.
 */
int device_pm_wait_for_dev(struct device *subordinate, struct device *dev)
{
	dpm_wait(dev, subordinate->power.async_suspend);
	return async_error;
}
EXPORT_SYMBOL_GPL(device_pm_wait_for_dev);

/**
 * dpm_for_each_dev - device iterator.
 * @data: data for the callback.
 * @fn: function to be called for each device.
 *
 * Iterate over devices in dpm_list, and call @fn for each device,
 * passing it @data.
 */
void dpm_for_each_dev(void *data, void (*fn)(struct device *, void *))
{
	struct device *dev;

	if (!fn)
		return;

	device_pm_lock();
	list_for_each_entry(dev, &dpm_list, power.entry)
		fn(dev, data);
	device_pm_unlock();
}
EXPORT_SYMBOL_GPL(dpm_for_each_dev);

static bool pm_ops_is_empty(const struct dev_pm_ops *ops)
{
	if (!ops)
		return true;

	return !ops->prepare &&
	       !ops->suspend &&
	       !ops->suspend_late &&
	       !ops->suspend_noirq &&
	       !ops->resume_noirq &&
	       !ops->resume_early &&
	       !ops->resume &&
	       !ops->complete;
}

void device_pm_check_callbacks(struct device *dev)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->power.lock, flags);
	dev->power.no_pm_callbacks =
		(!dev->bus || (pm_ops_is_empty(dev->bus->pm) &&
		 !dev->bus->suspend && !dev->bus->resume)) &&
		(!dev->class || pm_ops_is_empty(dev->class->pm)) &&
		(!dev->type || pm_ops_is_empty(dev->type->pm)) &&
		(!dev->pm_domain || pm_ops_is_empty(&dev->pm_domain->ops)) &&
		(!dev->driver || (pm_ops_is_empty(dev->driver->pm) &&
		 !dev->driver->suspend && !dev->driver->resume));
	spin_unlock_irqrestore(&dev->power.lock, flags);
}

bool dev_pm_skip_suspend(struct device *dev)
{
	return dev_pm_smart_suspend(dev) && pm_runtime_status_suspended(dev);
}
