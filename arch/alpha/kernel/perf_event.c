// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware performance events for the Alpha.
 *
 * We implement HW counts on the EV67 and subsequent CPUs only.
 *
 * (C) 2010 Michael J. Cree
 *
 * Somewhat based on the Sparc code, and to a lesser extent the PowerPC and
 * ARM code, which are copyright by their respective authors.
 */

#include <linux/perf_event.h>
#include <linux/kprobes.h>
#include <linux/kernel.h>
#include <linux/kdebug.h>
#include <linux/mutex.h>
#include <linux/init.h>
#include <linux/sched/task_stack.h>

#include <asm/hwrpb.h>
#include <linux/atomic.h>
#include <asm/irq.h>
#include <asm/irq_regs.h>
#include <asm/pal.h>
#include <asm/wrperfmon.h>
#include <asm/hw_irq.h>


/* The maximum number of PMCs on any Alpha CPU whatsoever. */
#define MAX_HWEVENTS 3
#define PMC_NO_INDEX -1

/* The PS bit that user_mode() tests. */
#define ALPHA_PS_USER	8UL

/* Layout of attr.config for PERF_TYPE_RAW events; see format/ in sysfs. */
#define ALPHA_PERF_CONFIG_EVENT		0xffULL
#define ALPHA_PERF_CONFIG_PROFILEME	BIT_ULL(8)

/* hw_perf_event.flags */
#define ALPHA_PERF_FLAG_WRAPPED		0x1
#define ALPHA_PERF_FLAG_PROFILEME	0x2

/* For tracking PMCs and the hw events they monitor on each CPU. */
struct cpu_hw_events {
	int			enabled;
	/* Number of events scheduled; also number entries valid in arrays below. */
	int			n_events;
	/* Number events added since last hw_perf_disable(). */
	int			n_added;
	/* Events currently scheduled. */
	struct perf_event	*event[MAX_HWEVENTS];
	/* Event type of each scheduled event. */
	unsigned long		evtype[MAX_HWEVENTS];
	/* Current index of each scheduled event; if not yet determined
	 * contains PMC_NO_INDEX.
	 */
	int			current_idx[MAX_HWEVENTS];
	/* The active PMCs' config for easy use with wrperfmon(). */
	unsigned long		config;
	/* The active counters' indices for easy use with wrperfmon(). */
	unsigned long		idx_mask;
};
DEFINE_PER_CPU(struct cpu_hw_events, cpu_hw_events);



/*
 * A structure to hold the description of the PMCs available on a particular
 * type of Alpha CPU.
 */
struct alpha_pmu_t {
	/* Mapping of the perf system hw event types to indigenous event types */
	const int *event_map;
	/* The number of entries in the event_map */
	int  max_events;
	/* The number of PMCs on this Alpha */
	int  num_pmcs;
	/*
	 * All PMC counters reside in the IBOX register PCTR.  This is the
	 * LSB of the counter.
	 */
	int  pmc_count_shift[MAX_HWEVENTS];
	/*
	 * The mask that isolates the PMC bits when the LSB of the counter
	 * is shifted to bit 0.
	 */
	unsigned long pmc_count_mask[MAX_HWEVENTS];
	/* The maximum period the PMC can count. */
	unsigned long pmc_max_period[MAX_HWEVENTS];
	/*
	 * The maximum value that may be written to the counter due to
	 * hardware restrictions is pmc_max_period - pmc_left.
	 */
	long pmc_left[3];
	 /* Subroutine for allocation of PMCs.  Enforces constraints. */
	int (*check_constraints)(struct perf_event **, unsigned long *, int);
	/* Subroutine for checking validity of a raw event for this PMU. */
	int (*raw_event_valid)(u64 config);
};

/*
 * The Alpha CPU PMU description currently in operation.  This is set during
 * the boot process to the specific CPU of the machine.
 */
static const struct alpha_pmu_t *alpha_pmu;


#define HW_OP_UNSUPPORTED -1

/*
 * The hardware description of the EV67, EV68, EV69, EV7 and EV79 PMUs
 * follow. Since they are identical we refer to them collectively as the
 * EV67 henceforth.
 */

/*
 * EV67 PMC event types
 *
 * There is no one-to-one mapping of the possible hw event types to the
 * actual codes that are used to program the PMCs hence we introduce our
 * own hw event type identifiers.
 */
enum ev67_pmc_event_type {
	EV67_CYCLES = 1,
	EV67_INSTRUCTIONS,
	EV67_BCACHEMISS,
	EV67_MBOXREPLAY,
	EV67_LAST_ET
};
#define EV67_NUM_EVENT_TYPES (EV67_LAST_ET-EV67_CYCLES)


/* Mapping of the hw event types to the perf tool interface */
static const int ev67_perfmon_event_map[] = {
	[PERF_COUNT_HW_CPU_CYCLES]	 = EV67_CYCLES,
	[PERF_COUNT_HW_INSTRUCTIONS]	 = EV67_INSTRUCTIONS,
	[PERF_COUNT_HW_CACHE_REFERENCES] = HW_OP_UNSUPPORTED,
	[PERF_COUNT_HW_CACHE_MISSES]	 = EV67_BCACHEMISS,
};

struct ev67_mapping_t {
	int config;
	int idx;
};

/*
 * The mapping used for one event only - these must be in same order as enum
 * ev67_pmc_event_type definition.
 */
static const struct ev67_mapping_t ev67_mapping[] = {
	{EV67_PCTR_INSTR_CYCLES, 1},	 /* EV67_CYCLES, */
	{EV67_PCTR_INSTR_CYCLES, 0},	 /* EV67_INSTRUCTIONS */
	{EV67_PCTR_INSTR_BCACHEMISS, 1}, /* EV67_BCACHEMISS */
	{EV67_PCTR_CYCLES_MBOX, 1}	 /* EV67_MBOXREPLAY */
};


/*
 * In ProfileMe mode PCTR0 selects the profiled instruction and PCTR1 counts
 * during its window. PCTR0 counts instructions or cycles, and each choice
 * pairs with two PCTR1 inputs, so each event type picks one SL1 setting.
 * Indexed by enum ev67_pmc_event_type.
 */
static const unsigned long ev67_profileme_sl1[] = {
	[EV67_CYCLES]		= EV67_PCTR_CYCLES_RETIRE_DELAY, /* PCTR1: retire delay */
	[EV67_INSTRUCTIONS]	= EV67_PCTR_INSTR_CYCLES,	/* PCTR1: cycles */
	[EV67_BCACHEMISS]	= EV67_PCTR_INSTR_BCACHEMISS,	/* PCTR1: Bcache misses */
	[EV67_MBOXREPLAY]	= EV67_PCTR_CYCLES_MBOX,	/* PCTR1: Mbox replays */
};

/*
 * Check that a group of events can be simultaneously scheduled on to the
 * EV67 PMU.  Also allocate counter indices and config.
 */
static int ev67_check_constraints(struct perf_event **event,
				unsigned long *evtype, int n_ev)
{
	int idx0, j;
	unsigned long config;

	/* A ProfileMe event uses both counters, so it must be alone. */
	for (j = 0; j < n_ev; j++) {
		if (!(event[j]->hw.flags & ALPHA_PERF_FLAG_PROFILEME))
			continue;
		if (n_ev != 1)
			return -1;
		event[0]->hw.idx = 0;
		event[0]->hw.config_base = EV67_PCTR_MODE_PROFILEME |
					   ev67_profileme_sl1[evtype[0]];
		return 0;
	}

	idx0 = ev67_mapping[evtype[0]-1].idx;
	config = ev67_mapping[evtype[0]-1].config;
	if (n_ev == 1)
		goto success;

	BUG_ON(n_ev != 2);

	if (evtype[0] == EV67_MBOXREPLAY || evtype[1] == EV67_MBOXREPLAY) {
		/* MBOX replay traps must be on PMC 1 */
		idx0 = (evtype[0] == EV67_MBOXREPLAY) ? 1 : 0;
		/* Only cycles can accompany MBOX replay traps */
		if (evtype[idx0] == EV67_CYCLES) {
			config = EV67_PCTR_CYCLES_MBOX;
			goto success;
		}
	}

	if (evtype[0] == EV67_BCACHEMISS || evtype[1] == EV67_BCACHEMISS) {
		/* Bcache misses must be on PMC 1 */
		idx0 = (evtype[0] == EV67_BCACHEMISS) ? 1 : 0;
		/* Only instructions can accompany Bcache misses */
		if (evtype[idx0] == EV67_INSTRUCTIONS) {
			config = EV67_PCTR_INSTR_BCACHEMISS;
			goto success;
		}
	}

	if (evtype[0] == EV67_INSTRUCTIONS || evtype[1] == EV67_INSTRUCTIONS) {
		/* Instructions must be on PMC 0 */
		idx0 = (evtype[0] == EV67_INSTRUCTIONS) ? 0 : 1;
		/* By this point only cycles can accompany instructions */
		if (evtype[idx0^1] == EV67_CYCLES) {
			config = EV67_PCTR_INSTR_CYCLES;
			goto success;
		}
	}

	/* Otherwise, darn it, there is a conflict.  */
	return -1;

success:
	event[0]->hw.idx = idx0;
	event[0]->hw.config_base = config;
	if (n_ev == 2) {
		event[1]->hw.idx = idx0 ^ 1;
		event[1]->hw.config_base = config;
	}
	return 0;
}


static int ev67_raw_event_valid(u64 config)
{
	return config >= EV67_CYCLES && config < EV67_LAST_ET;
};


PMU_FORMAT_ATTR(event, "config:0-7");
PMU_FORMAT_ATTR(profileme, "config:8");

static struct attribute *ev67_format_attrs[] = {
	&format_attr_event.attr,
	&format_attr_profileme.attr,
	NULL,
};

static const struct attribute_group ev67_format_group = {
	.name = "format",
	.attrs = ev67_format_attrs,
};

/* Values must match enum ev67_pmc_event_type. */
PMU_EVENT_ATTR_STRING(cycles, ev67_event_cycles, "event=0x01");
PMU_EVENT_ATTR_STRING(instructions, ev67_event_instructions, "event=0x02");
PMU_EVENT_ATTR_STRING(bcache-misses, ev67_event_bcache_misses, "event=0x03");
PMU_EVENT_ATTR_STRING(mbox-replay, ev67_event_mbox_replay, "event=0x04");

static struct attribute *ev67_event_attrs[] = {
	&ev67_event_cycles.attr.attr,
	&ev67_event_instructions.attr.attr,
	&ev67_event_bcache_misses.attr.attr,
	&ev67_event_mbox_replay.attr.attr,
	NULL,
};

static const struct attribute_group ev67_events_group = {
	.name = "events",
	.attrs = ev67_event_attrs,
};

static const struct attribute_group *ev67_attr_groups[] = {
	&ev67_format_group,
	&ev67_events_group,
	NULL,
};


static const struct alpha_pmu_t ev67_pmu = {
	.event_map = ev67_perfmon_event_map,
	.max_events = ARRAY_SIZE(ev67_perfmon_event_map),
	.num_pmcs = 2,
	.pmc_count_shift = {EV67_PCTR_0_COUNT_SHIFT, EV67_PCTR_1_COUNT_SHIFT, 0},
	.pmc_count_mask = {EV67_PCTR_0_COUNT_MASK,  EV67_PCTR_1_COUNT_MASK,  0},
	.pmc_max_period = {(1UL<<20) - 1, (1UL<<20) - 1, 0},
	.pmc_left = {16, 4, 0},
	.check_constraints = ev67_check_constraints,
	.raw_event_valid = ev67_raw_event_valid,
};



/*
 * Helper routines to ensure that we read/write only the correct PMC bits
 * when calling the wrperfmon PALcall.
 */
static inline void alpha_write_pmc(int idx, unsigned long val)
{
	val &= alpha_pmu->pmc_count_mask[idx];
	val <<= alpha_pmu->pmc_count_shift[idx];
	val |= (1<<idx);
	wrperfmon(PERFMON_CMD_WRITE, val);
}

/* Write PCTR0 and clear PCTR1 in one call, as ProfileMe mode requires. */
static inline void alpha_write_pmc_profileme(unsigned long val)
{
	val &= alpha_pmu->pmc_count_mask[0];
	val <<= alpha_pmu->pmc_count_shift[0];
	val |= EV67_PCTR_0 | EV67_PCTR_1;
	/*
	 * PCTR0 stops when a ProfileMe window closes, and a stopped counter
	 * only clears its overflow state when written twice (EV67 HRM, PALcode
	 * restriction 38). A counter left signaling overflow blocks all
	 * other interrupts (restriction 32).
	 */
	wrperfmon(PERFMON_CMD_WRITE, val);
	wrperfmon(PERFMON_CMD_WRITE, val);
}

/* The counters an event occupies, for PERFMON_CMD_ENABLE and DISABLE. */
static inline unsigned long alpha_event_idx_mask(struct hw_perf_event *hwc,
						 int idx)
{
	if (hwc->flags & ALPHA_PERF_FLAG_PROFILEME)
		return EV67_PCTR_0 | EV67_PCTR_1;
	return 1UL<<idx;
}

static inline unsigned long alpha_read_pmc(int idx)
{
	unsigned long val;

	val = wrperfmon(PERFMON_CMD_READ, 0);
	val >>= alpha_pmu->pmc_count_shift[idx];
	val &= alpha_pmu->pmc_count_mask[idx];
	return val;
}

/*
 * Move a stopped ProfileMe counter away from overflow. See
 * alpha_write_pmc_profileme() for why. The caller has read the count, so
 * prev_count can be rebased on the value written.
 *
 * A pending PMI stays pending, but alpha_perf_event_ovf() ignores it, as
 * the counter is left no nearer zero than alpha_perf_event_set_period()
 * would leave it.
 */
static void alpha_perf_event_park(struct hw_perf_event *hwc)
{
	long left = local64_read(&hwc->period_left);

	left = clamp(left, alpha_pmu->pmc_left[0],
		     (long)(alpha_pmu->pmc_max_period[0] >> 1));
	local64_set(&hwc->prev_count, (unsigned long)(-left));
	hwc->flags &= ~ALPHA_PERF_FLAG_WRAPPED;
	alpha_write_pmc_profileme((unsigned long)(-left));
}

/* Set a new period to sample over */
static int alpha_perf_event_set_period(struct perf_event *event,
				struct hw_perf_event *hwc, int idx)
{
	long left = local64_read(&hwc->period_left);
	long period = hwc->sample_period;
	int ret = 0;

	if (unlikely(left <= -period)) {
		left = period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ret = 1;
	}

	if (unlikely(left <= 0)) {
		left += period;
		local64_set(&hwc->period_left, left);
		hwc->last_period = period;
		ret = 1;
	}

	/*
	 * Hardware restrictions require that the counters must not be
	 * written with values that are too close to the maximum period.
	 */
	if (unlikely(left < alpha_pmu->pmc_left[idx]))
		left = alpha_pmu->pmc_left[idx];

	/*
	 * Start at most half the counter's range below overflow, so that the
	 * value an overflow leaves, just past zero, is always below the value
	 * written. See alpha_perf_event_ovf().
	 */
	if (left > (long)(alpha_pmu->pmc_max_period[idx] >> 1))
		left = alpha_pmu->pmc_max_period[idx] >> 1;

	local64_set(&hwc->prev_count, (unsigned long)(-left));
	hwc->flags &= ~ALPHA_PERF_FLAG_WRAPPED;

	if (hwc->flags & ALPHA_PERF_FLAG_PROFILEME)
		alpha_write_pmc_profileme((unsigned long)(-left));
	else
		alpha_write_pmc(idx, (unsigned long)(-left));

	perf_event_update_userpage(event);

	return ret;
}


/*
 * Calculates the count (the 'delta') since the last time the PMC was read.
 *
 * As the PMCs' full period can easily be exceeded within the perf system
 * sampling period we cannot use any high order bits as a guard bit in the
 * PMCs to detect overflow as is done by other architectures.  The code here
 * calculates the delta on the basis that there is no overflow when ovf is
 * zero.  The value passed via ovf by the interrupt handler corrects for
 * overflow.
 *
 * A call to this routine can occur with an overflowed counter just before
 * the PMI service routine is called. A negative delta detects this, as the
 * counter is always written at most half its range below overflow.
 */
static unsigned long alpha_perf_event_update(struct perf_event *event,
					struct hw_perf_event *hwc, int idx, long ovf)
{
	long prev_raw_count, new_raw_count;
	long delta;

again:
	prev_raw_count = local64_read(&hwc->prev_count);
	new_raw_count = alpha_read_pmc(idx);

	if (local64_cmpxchg(&hwc->prev_count, prev_raw_count,
			     new_raw_count) != prev_raw_count)
		goto again;

	delta = (new_raw_count - (prev_raw_count & alpha_pmu->pmc_count_mask[idx])) + ovf;

	/* It is possible on very rare occasions that the PMC has overflowed
	 * but the interrupt is yet to come.  Detect and fix this situation,
	 * and tell the interrupt not to count the overflow again.
	 */
	if (unlikely(delta < 0)) {
		delta += alpha_pmu->pmc_max_period[idx] + 1;
		hwc->flags |= ALPHA_PERF_FLAG_WRAPPED;
	}

	local64_add(delta, &event->count);
	local64_sub(delta, &hwc->period_left);

	return new_raw_count;
}


/*
 * Collect all HW events into the array event[].
 */
static int collect_events(struct perf_event *group, int max_count,
			  struct perf_event *event[], unsigned long *evtype,
			  int *current_idx)
{
	struct perf_event *pe;
	int n = 0;

	if (!is_software_event(group)) {
		if (n >= max_count)
			return -1;
		event[n] = group;
		evtype[n] = group->hw.event_base;
		current_idx[n++] = PMC_NO_INDEX;
	}
	for_each_sibling_event(pe, group) {
		if (!is_software_event(pe) && pe->state != PERF_EVENT_STATE_OFF) {
			if (n >= max_count)
				return -1;
			event[n] = pe;
			evtype[n] = pe->hw.event_base;
			current_idx[n++] = PMC_NO_INDEX;
		}
	}
	return n;
}



/*
 * Check that a group of events can be simultaneously scheduled on to the PMU.
 */
static int alpha_check_constraints(struct perf_event **events,
				   unsigned long *evtypes, int n_ev)
{

	/* No HW events is possible from hw_perf_group_sched_in(). */
	if (n_ev == 0)
		return 0;

	if (n_ev > alpha_pmu->num_pmcs)
		return -1;

	return alpha_pmu->check_constraints(events, evtypes, n_ev);
}


/*
 * If new events have been scheduled then update cpuc with the new
 * configuration.  This may involve shifting cycle counts from one PMC to
 * another.
 */
static void maybe_change_configuration(struct cpu_hw_events *cpuc)
{
	int j;

	if (cpuc->n_added == 0)
		return;

	/* Find counters that are moving to another PMC and update */
	for (j = 0; j < cpuc->n_events; j++) {
		struct perf_event *pe = cpuc->event[j];

		if (cpuc->current_idx[j] != PMC_NO_INDEX &&
			cpuc->current_idx[j] != pe->hw.idx) {
			alpha_perf_event_update(pe, &pe->hw, cpuc->current_idx[j], 0);
			cpuc->current_idx[j] = PMC_NO_INDEX;
		}
	}

	/* Assign to counters all unassigned events. */
	cpuc->idx_mask = 0;
	for (j = 0; j < cpuc->n_events; j++) {
		struct perf_event *pe = cpuc->event[j];
		struct hw_perf_event *hwc = &pe->hw;
		int idx = hwc->idx;

		if (cpuc->current_idx[j] == PMC_NO_INDEX) {
			alpha_perf_event_set_period(pe, hwc, idx);
			cpuc->current_idx[j] = idx;
		}

		if (!(hwc->state & PERF_HES_STOPPED)) {
			/* The counter is live, so hwc->prev_count will go stale. */
			hwc->state &= ~PERF_HES_UPTODATE;
			cpuc->idx_mask |= alpha_event_idx_mask(hwc,
							cpuc->current_idx[j]);
		}
	}
	cpuc->config = cpuc->event[0]->hw.config_base;
}



/* Schedule perf HW event on to PMU.
 *  - this function is called from outside this module via the pmu struct
 *    returned from perf event initialisation.
 */
static int alpha_pmu_add(struct perf_event *event, int flags)
{
	struct cpu_hw_events *cpuc = this_cpu_ptr(&cpu_hw_events);
	struct hw_perf_event *hwc = &event->hw;
	int n0;
	int ret;
	unsigned long irq_flags;

	/*
	 * The Sparc code has the IRQ disable first followed by the perf
	 * disable, however this can lead to an overflowed counter with the
	 * PMI disabled on rare occasions.  The alpha_perf_event_update()
	 * routine should detect this situation by noting a negative delta,
	 * nevertheless we disable the PMCs first to enable a potential
	 * final PMI to occur before we disable interrupts.
	 */
	perf_pmu_disable(event->pmu);
	local_irq_save(irq_flags);

	/* Default to error to be returned */
	ret = -EAGAIN;

	/* Insert event on to PMU and if successful modify ret to valid return */
	n0 = cpuc->n_events;
	if (n0 < alpha_pmu->num_pmcs) {
		cpuc->event[n0] = event;
		cpuc->evtype[n0] = event->hw.event_base;
		cpuc->current_idx[n0] = PMC_NO_INDEX;

		if (!alpha_check_constraints(cpuc->event, cpuc->evtype, n0+1)) {
			cpuc->n_events++;
			cpuc->n_added++;
			ret = 0;
		}
	}

	hwc->state = PERF_HES_UPTODATE;
	if (!(flags & PERF_EF_START))
		hwc->state |= PERF_HES_STOPPED;

	local_irq_restore(irq_flags);
	perf_pmu_enable(event->pmu);

	return ret;
}



/* Disable performance monitoring unit
 *  - this function is called from outside this module via the pmu struct
 *    returned from perf event initialisation.
 */
static void alpha_pmu_del(struct perf_event *event, int flags)
{
	struct cpu_hw_events *cpuc = this_cpu_ptr(&cpu_hw_events);
	struct hw_perf_event *hwc = &event->hw;
	unsigned long irq_flags;
	int j;

	perf_pmu_disable(event->pmu);
	local_irq_save(irq_flags);

	for (j = 0; j < cpuc->n_events; j++) {
		if (event == cpuc->event[j]) {
			int idx = cpuc->current_idx[j];

			/* Shift remaining entries down into the existing
			 * slot.
			 */
			while (++j < cpuc->n_events) {
				cpuc->event[j - 1] = cpuc->event[j];
				cpuc->evtype[j - 1] = cpuc->evtype[j];
				cpuc->current_idx[j - 1] =
					cpuc->current_idx[j];
			}

			/*
			 * Absorb the final count and turn off the event,
			 * unless a failed group schedule rolled it back
			 * before it was given a counter.
			 */
			if (idx != PMC_NO_INDEX) {
				alpha_perf_event_update(event, hwc, idx, 0);
				perf_event_update_userpage(event);

				if (hwc->flags & ALPHA_PERF_FLAG_PROFILEME)
					alpha_perf_event_park(hwc);

				cpuc->idx_mask &= ~alpha_event_idx_mask(hwc, idx);
			}
			cpuc->n_events--;
			break;
		}
	}

	local_irq_restore(irq_flags);
	perf_pmu_enable(event->pmu);
}


static void alpha_pmu_read(struct perf_event *event)
{
	struct hw_perf_event *hwc = &event->hw;

	alpha_perf_event_update(event, hwc, hwc->idx, 0);
}


static void alpha_pmu_stop(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;
	struct cpu_hw_events *cpuc = this_cpu_ptr(&cpu_hw_events);

	if (!(hwc->state & PERF_HES_STOPPED)) {
		cpuc->idx_mask &= ~alpha_event_idx_mask(hwc, hwc->idx);
		hwc->state |= PERF_HES_STOPPED;
	}

	/* Parking a ProfileMe event overwrites the counter, so read it first. */
	if (((flags & PERF_EF_UPDATE) || (hwc->flags & ALPHA_PERF_FLAG_PROFILEME)) &&
	    !(hwc->state & PERF_HES_UPTODATE)) {
		alpha_perf_event_update(event, hwc, hwc->idx, 0);
		hwc->state |= PERF_HES_UPTODATE;
	}

	if (cpuc->enabled)
		wrperfmon(PERFMON_CMD_DISABLE, alpha_event_idx_mask(hwc, hwc->idx));

	if (hwc->flags & ALPHA_PERF_FLAG_PROFILEME)
		alpha_perf_event_park(hwc);
}


static void alpha_pmu_start(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;
	struct cpu_hw_events *cpuc = this_cpu_ptr(&cpu_hw_events);

	if (WARN_ON_ONCE(!(hwc->state & PERF_HES_STOPPED)))
		return;

	if (flags & PERF_EF_RELOAD) {
		WARN_ON_ONCE(!(hwc->state & PERF_HES_UPTODATE));
		alpha_perf_event_set_period(event, hwc, hwc->idx);
	}

	hwc->state = 0;

	cpuc->idx_mask |= alpha_event_idx_mask(hwc, hwc->idx);
	if (cpuc->enabled)
		wrperfmon(PERFMON_CMD_ENABLE, alpha_event_idx_mask(hwc, hwc->idx));
}


/*
 * Check that CPU performance counters are supported.
 * - currently support EV67 and later CPUs.
 * - actually some later revisions of the EV6 have the same PMC model as the
 *     EV67 but we don't do sufficiently deep CPU detection to detect them.
 *     Bad luck to the very few people who might have one, I guess.
 */
static int supported_cpu(void)
{
	struct percpu_struct *cpu;
	unsigned long cputype;

	/* Get cpu type from HW */
	cpu = (struct percpu_struct *)((char *)hwrpb + hwrpb->processor_offset);
	cputype = cpu->type & 0xffffffff;
	/* Include all of EV67, EV68, EV7, EV79 and EV69 as supported. */
	return (cputype >= EV67_CPU) && (cputype <= EV69_CPU);
}



static void hw_perf_event_destroy(struct perf_event *event)
{
	/* Nothing to be done! */
	return;
}



static int __hw_perf_event_init(struct perf_event *event)
{
	struct perf_event_attr *attr = &event->attr;
	struct hw_perf_event *hwc = &event->hw;
	struct perf_event *evts[MAX_HWEVENTS];
	unsigned long evtypes[MAX_HWEVENTS];
	int idx_rubbish_bin[MAX_HWEVENTS];
	bool profileme = false;
	int ev;
	int n;

	/* We only support a limited range of HARDWARE event types with one
	 * only programmable via a RAW event type.
	 */
	if (attr->type == PERF_TYPE_HARDWARE) {
		if (attr->config >= alpha_pmu->max_events)
			return -EINVAL;
		ev = alpha_pmu->event_map[attr->config];
	} else if (attr->type == PERF_TYPE_HW_CACHE) {
		return -EOPNOTSUPP;
	} else if (attr->type == PERF_TYPE_RAW) {
		if (attr->config & ~(ALPHA_PERF_CONFIG_EVENT |
				     ALPHA_PERF_CONFIG_PROFILEME))
			return -EINVAL;
		ev = attr->config & ALPHA_PERF_CONFIG_EVENT;
		if (!alpha_pmu->raw_event_valid(ev))
			return -EINVAL;
		profileme = attr->config & ALPHA_PERF_CONFIG_PROFILEME;
	} else {
		return -EOPNOTSUPP;
	}

	if (ev < 0) {
		return ev;
	}

	/*
	 * ProfileMe samples report the exact PC of the profiled instruction,
	 * so it also serves any request for precise samples. It only makes
	 * sense for sampling events. Level 3 promises no skid at all, but some
	 * samples keep the interrupted PC; see alpha_perf_event_profileme().
	 */
	if (attr->precise_ip > 2)
		return -EOPNOTSUPP;
	if (attr->precise_ip)
		profileme = true;
	if (profileme && !is_sampling_event(event))
		return -EINVAL;
	hwc->flags = profileme ? ALPHA_PERF_FLAG_PROFILEME : 0;

	/*
	 * We place the event type in event_base here and leave calculation
	 * of the codes to programme the PMU for alpha_pmu_enable() because
	 * it is only then we will know what HW events are actually
	 * scheduled on to the PMU.  At that point the code to programme the
	 * PMU is put into config_base and the PMC to use is placed into
	 * idx.  We initialise idx (below) to PMC_NO_INDEX to indicate that
	 * it is yet to be determined.
	 */
	hwc->event_base = ev;

	/* Collect events in a group together suitable for calling
	 * alpha_check_constraints() to verify that the group as a whole can
	 * be scheduled on to the PMU.
	 */
	n = 0;
	if (event->group_leader != event) {
		n = collect_events(event->group_leader,
				alpha_pmu->num_pmcs - 1,
				evts, evtypes, idx_rubbish_bin);
		if (n < 0)
			return -EINVAL;
	}
	evtypes[n] = hwc->event_base;
	evts[n] = event;

	if (alpha_check_constraints(evts, evtypes, n + 1))
		return -EINVAL;

	/* Indicate that PMU config and idx are yet to be determined. */
	hwc->config_base = 0;
	hwc->idx = PMC_NO_INDEX;

	event->destroy = hw_perf_event_destroy;

	/*
	 * Most architectures reserve the PMU for their use at this point.
	 * As there is no existing mechanism to arbitrate usage and there
	 * appears to be no other user of the Alpha PMU we just assume
	 * that we can just use it, hence a NO-OP here.
	 *
	 * Maybe an alpha_reserve_pmu() routine should be implemented but is
	 * anything else ever going to use it?
	 */

	if (!hwc->sample_period) {
		hwc->sample_period = alpha_pmu->pmc_max_period[0];
		hwc->last_period = hwc->sample_period;
		local64_set(&hwc->period_left, hwc->sample_period);
	}

	return 0;
}

/*
 * Main entry point to initialise a HW performance event.
 */
static int alpha_pmu_event_init(struct perf_event *event)
{
	/* does not support taken branch sampling */
	if (has_branch_stack(event))
		return -EOPNOTSUPP;

	switch (event->attr.type) {
	case PERF_TYPE_RAW:
	case PERF_TYPE_HARDWARE:
	case PERF_TYPE_HW_CACHE:
		break;

	default:
		return -ENOENT;
	}

	if (!alpha_pmu)
		return -ENODEV;

	/* Do the real initialisation work. */
	return __hw_perf_event_init(event);
}

/*
 * Main entry point - enable HW performance counters.
 */
static void alpha_pmu_enable(struct pmu *pmu)
{
	struct cpu_hw_events *cpuc = this_cpu_ptr(&cpu_hw_events);

	if (cpuc->enabled)
		return;

	cpuc->enabled = 1;
	barrier();

	if (cpuc->n_events > 0) {
		/* Update cpuc with information from any new scheduled events. */
		maybe_change_configuration(cpuc);

		/* Start counting the desired events. */
		wrperfmon(PERFMON_CMD_LOGGING_OPTIONS, PERFMON_CMD_LOGGING_ALL);
		wrperfmon(PERFMON_CMD_DESIRED_EVENTS, cpuc->config);
		wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);
	}
}


/*
 * Main entry point - disable HW performance counters.
 */

static void alpha_pmu_disable(struct pmu *pmu)
{
	struct cpu_hw_events *cpuc = this_cpu_ptr(&cpu_hw_events);

	if (!cpuc->enabled)
		return;

	cpuc->enabled = 0;
	cpuc->n_added = 0;

	wrperfmon(PERFMON_CMD_DISABLE, cpuc->idx_mask);
}

static struct pmu pmu = {
	.pmu_enable	= alpha_pmu_enable,
	.pmu_disable	= alpha_pmu_disable,
	.event_init	= alpha_pmu_event_init,
	.add		= alpha_pmu_add,
	.del		= alpha_pmu_del,
	.start		= alpha_pmu_start,
	.stop		= alpha_pmu_stop,
	.read		= alpha_pmu_read,
	.attr_groups	= ev67_attr_groups,
	.capabilities	= PERF_PMU_CAP_NO_EXCLUDE,
};


/*
 * Main entry point - don't know when this is called but it
 * obviously dumps debug info.
 */
void perf_event_print_debug(void)
{
	unsigned long flags;
	unsigned long pcr;
	int pcr0, pcr1;
	int cpu;

	if (!supported_cpu())
		return;

	local_irq_save(flags);

	cpu = smp_processor_id();

	pcr = wrperfmon(PERFMON_CMD_READ, 0);
	pcr0 = (pcr >> alpha_pmu->pmc_count_shift[0]) & alpha_pmu->pmc_count_mask[0];
	pcr1 = (pcr >> alpha_pmu->pmc_count_shift[1]) & alpha_pmu->pmc_count_mask[1];

	pr_info("CPU#%d: PCTR0[%06x] PCTR1[%06x]\n", cpu, pcr0, pcr1);

	local_irq_restore(flags);
}


/*
 * Return the overflow that an interrupt from counter idx adds to the count,
 * or -1 if the counter has not overflowed since it was written.
 *
 * Rewriting a counter does not cancel its pending interrupt: wrperfmon has
 * no way to, and PALcode clears the request only when delivering it. So an
 * interrupt can arrive after the counter was reloaded or given to another
 * event. The counter is then at or above the value written, whereas an
 * overflow leaves it just past zero, below that value.
 */
static long alpha_perf_event_ovf(struct hw_perf_event *hwc, int idx)
{
	unsigned long prev;

	/* alpha_perf_event_update() has already counted this overflow. */
	if (hwc->flags & ALPHA_PERF_FLAG_WRAPPED)
		return 0;

	prev = local64_read(&hwc->prev_count) & alpha_pmu->pmc_count_mask[idx];
	if (alpha_read_pmc(idx) >= prev)
		return -1;

	return alpha_pmu->pmc_max_period[idx] + 1;
}


/*
 * Frame holding only the PC, for a ProfileMe interrupt taken in a different
 * mode than the profiled instruction. Being off the stack,
 * perf_reg_value() reports no SP.
 */
static DEFINE_PER_CPU(struct pt_regs, alpha_profileme_regs);

/*
 * Report the sample with none of the given PERF_SAMPLE_REGS_* registers.
 * Having no user registers also means no user stack.
 */
static void alpha_profileme_drop_regs(struct perf_event *event,
				      struct perf_sample_data *data, u64 which)
{
	/* STACK_USER consults regs_user even without REGS_USER. */
	if (which & PERF_SAMPLE_REGS_USER)
		data->regs_user = (struct perf_regs){ .abi = PERF_SAMPLE_REGS_ABI_NONE };
	if (which & PERF_SAMPLE_REGS_INTR)
		data->regs_intr = (struct perf_regs){ .abi = PERF_SAMPLE_REGS_ABI_NONE };
	data->sample_flags |= which;
	/* Only the ABI word is written for each. */
	data->dyn_size += hweight64(event->attr.sample_type & which) * sizeof(u64);
}

/* Report the user registers from a frame known to be complete. */
static void alpha_profileme_set_regs_user(struct perf_event *event,
					  struct perf_sample_data *data)
{
	data->regs_user.regs = task_pt_regs(current);
	data->regs_user.abi = perf_reg_abi(current);
	data->sample_flags |= PERF_SAMPLE_REGS_USER;
	if (event->attr.sample_type & PERF_SAMPLE_REGS_USER)
		data->dyn_size += (1 + hweight64(event->attr.sample_regs_user)) *
				  sizeof(u64);
}

/*
 * Handle the close of a ProfileMe window on PCTR0.
 *
 * PCTR0 stops counting when the window closes and does not start again
 * until PCTR_CTL is written, so the status must be read first and the
 * counters then rearmed unconditionally.
 */
static void alpha_perf_event_profileme(struct perf_event *event,
				       struct hw_perf_event *hwc,
				       struct pt_regs *regs, long ovf)
{
	struct perf_sample_data data;
	struct pt_regs *pm_regs;
	bool pc_only = false;
	unsigned long i_stat, pmpc, ovr, pc, saved_pc, saved_ps;

	i_stat = wrperfmon(PERFMON_CMD_I_STAT, 0);
	pmpc = wrperfmon(PERFMON_CMD_PMPC, 0);

	alpha_perf_event_update(event, hwc, 0, ovf);

	/*
	 * When PCTR0 counts instructions, up to 7 of them are counted after
	 * the overflow but before the window opens. I_STAT reports how many.
	 */
	if (!(hwc->config_base & EV67_PCTR_PCTR0_CYCLES)) {
		ovr = (i_stat >> EV67_I_STAT_OVR_SHIFT) & EV67_I_STAT_OVR_MASK;
		local64_sub(ovr, &event->count);
		local64_add(ovr, &hwc->period_left);
	}

	/*
	 * Attribute the sample to the profiled instruction rather than to
	 * wherever the interrupt landed. perf takes the sample's mode,
	 * registers and stack from regs, so:
	 *
	 * - Interrupt taken in the same mode: the PC is set in the
	 *   interrupt's own frame, and restored afterwards.
	 * - User instruction, interrupt taken in the kernel, typically
	 *   because the instruction trapped: the frame at task_pt_regs() may
	 *   not be built yet. The interrupt is often taken on the first
	 *   instruction of the entry code, and its own frame then overlaps
	 *   that one. Report the PC alone, in a per-CPU frame.
	 * - Kernel instruction, interrupt taken just after returning to user
	 *   mode: the kernel state is gone, so a per-CPU frame that holds only
	 *   the PC. The user frame in regs is left alone, since perf samples
	 *   the user registers from it.
	 *
	 * A PALmode PMPC is a physical address that perf cannot resolve. That
	 * happens for ITB miss, Istream access violation and interrupt
	 * redirects, and for instructions inside PALcode. Keep the
	 * interrupted PC, which for redirects is the instruction that
	 * trapped. Do the same for a user PMPC in a kernel thread, which has
	 * no user frame.
	 */
	pm_regs = regs;
	pc = pmpc & EV67_PMPC_PC_MASK;
	if (pmpc & EV67_PMPC_PAL) {
		pc = regs->pc;
	} else if (pc < TASK_SIZE && !user_mode(regs)) {
		if (is_user_task(current)) {
			pc_only = true;
			pm_regs = this_cpu_ptr(&alpha_profileme_regs);
			memset(pm_regs, 0, sizeof(*pm_regs));
			pm_regs->ps = ALPHA_PS_USER;
		} else {
			pc = regs->pc;
		}
	} else if (pc >= TASK_SIZE && user_mode(regs)) {
		pm_regs = this_cpu_ptr(&alpha_profileme_regs);
		memset(pm_regs, 0, sizeof(*pm_regs));
	}

	perf_sample_data_init(&data, 0, hwc->last_period);
	if (pc_only)
		alpha_profileme_drop_regs(event, &data, PERF_SAMPLE_REGS_USER |
							PERF_SAMPLE_REGS_INTR);
	/*
	 * perf_get_regs_user() has the same check, but it will find the PMPC
	 * in regs->pc rather than where the interrupt was taken. Decide here
	 * from the interrupted PC, for either answer that would differ.
	 */
	else if (!user_mode(regs) && in_entry_save_code(regs->pc))
		alpha_profileme_drop_regs(event, &data, PERF_SAMPLE_REGS_USER);
	else if (in_entry_save_code(pc) && is_user_task(current))
		alpha_profileme_set_regs_user(event, &data);

	if (!alpha_perf_event_set_period(event, hwc, 0))
		return;

	saved_pc = pm_regs->pc;
	saved_ps = pm_regs->ps;
	pm_regs->pc = pc;
	if (pc >= TASK_SIZE)
		pm_regs->ps &= ~ALPHA_PS_USER;

	perf_event_overflow(event, &data, pm_regs);

	pm_regs->pc = saved_pc;
	pm_regs->ps = saved_ps;
}

/*
 * Performance Monitoring Interrupt Service Routine called when a PMC
 * overflows.  The PMC that overflowed is passed in la_ptr.
 */
static void alpha_perf_event_irq_handler(unsigned long la_ptr,
					struct pt_regs *regs)
{
	struct cpu_hw_events *cpuc;
	struct perf_sample_data data;
	struct perf_event *event;
	struct hw_perf_event *hwc;
	int idx, j;
	long ovf;

	__this_cpu_inc(irq_pmi_count);
	cpuc = this_cpu_ptr(&cpu_hw_events);

	/* Completely counting through the PMC's period to trigger a new PMC
	 * overflow interrupt while in this interrupt routine is utterly
	 * disastrous!  The EV6 and EV67 counters are sufficiently large to
	 * prevent this but to be really sure disable the PMCs.
	 */
	wrperfmon(PERFMON_CMD_DISABLE, cpuc->idx_mask);

	/* la_ptr is the counter that overflowed. */
	if (unlikely(la_ptr >= alpha_pmu->num_pmcs)) {
		/* This should never occur! */
		irq_err_count++;
		pr_warn("PMI: silly index %ld\n", la_ptr);
		wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);
		return;
	}

	idx = la_ptr;

	/* A ProfileMe event owns both counters but lives at index 0. */
	if (cpuc->config & EV67_PCTR_MODE_PROFILEME)
		idx = 0;

	for (j = 0; j < cpuc->n_events; j++) {
		if (cpuc->current_idx[j] == idx)
			break;
	}

	if (unlikely(j == cpuc->n_events)) {
		/* This can occur if the event is disabled right on a PMC overflow. */
		if (cpuc->config & EV67_PCTR_MODE_PROFILEME)
			alpha_write_pmc_profileme(-alpha_pmu->pmc_max_period[0]);
		wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);
		return;
	}

	event = cpuc->event[j];

	if (unlikely(!event)) {
		/* This should never occur! */
		irq_err_count++;
		pr_warn("PMI: No event at index %d!\n", idx);
		wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);
		return;
	}

	hwc = &event->hw;
	ovf = alpha_perf_event_ovf(hwc, idx);
	if (ovf < 0) {
		/*
		 * The interrupt was pending when the counter was written, or
		 * is from PCTR1 and was pending when a ProfileMe event was
		 * scheduled in.
		 */
		wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);
		return;
	}

	if (hwc->flags & ALPHA_PERF_FLAG_PROFILEME) {
		alpha_perf_event_profileme(event, hwc, regs, ovf);
		wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);
		return;
	}

	alpha_perf_event_update(event, hwc, idx, ovf);
	perf_sample_data_init(&data, 0, hwc->last_period);

	if (alpha_perf_event_set_period(event, hwc, idx))
		perf_event_overflow(event, &data, regs);

	wrperfmon(PERFMON_CMD_ENABLE, cpuc->idx_mask);

	return;
}



/*
 * Init call to initialise performance events at kernel startup.
 */
static int __init init_hw_perf_events(void)
{
	pr_info("Performance events: ");

	if (!supported_cpu()) {
		pr_cont("No support for your CPU.\n");
		return 0;
	}

	pr_cont("Supported CPU type!\n");

	/* Override performance counter IRQ vector */

	perf_irq = alpha_perf_event_irq_handler;

	/* And set up PMU specification */
	alpha_pmu = &ev67_pmu;

	perf_pmu_register(&pmu, "cpu", PERF_TYPE_RAW);

	return 0;
}
early_initcall(init_hw_perf_events);
