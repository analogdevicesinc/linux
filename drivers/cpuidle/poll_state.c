// SPDX-License-Identifier: GPL-2.0-only
/*
 * poll_state.c - Polling idle state
 */

#include <linux/cpuidle.h>
#include <linux/export.h>
#include <linux/irqflags.h>
#include <linux/sched/idle.h>
#include <linux/sprintf.h>
#include <linux/types.h>

static int __cpuidle poll_idle(struct cpuidle_device *dev,
			       struct cpuidle_driver *drv, int index)
{
	dev->poll_time_limit = false;

	raw_local_irq_enable();
	if (!current_set_polling_and_test()) {
		u64 limit;

		limit = cpuidle_poll_time(drv, dev);

		dev->poll_time_limit = !tif_need_resched_relaxed_wait(limit);
	}
	raw_local_irq_disable();

	current_clr_polling();

	return index;
}

void cpuidle_poll_state_init(struct cpuidle_driver *drv)
{
	struct cpuidle_state *state = &drv->states[0];

	snprintf(state->name, CPUIDLE_NAME_LEN, "POLL");
	snprintf(state->desc, CPUIDLE_DESC_LEN, "CPUIDLE CORE POLL IDLE");
	state->exit_latency = 0;
	state->target_residency = 0;
	state->exit_latency_ns = 0;
	state->target_residency_ns = 0;
	state->power_usage = -1;
	state->enter = poll_idle;
	state->flags = CPUIDLE_FLAG_POLLING;
}
EXPORT_SYMBOL_GPL(cpuidle_poll_state_init);
