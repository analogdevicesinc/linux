/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_SLEEP_H_
#define _XE_SLEEP_H_

#include <linux/delay.h>
#include <linux/math64.h>

/**
 * xe_sleep_relaxed_ms() - Sleep for an approximate time.
 * @delay_ms: time in msec to sleep
 *
 * For smaller timeouts, sleep with 0.5ms accuracy.
 */
static inline void xe_sleep_relaxed_ms(unsigned int delay_ms)
{
	unsigned long min_us, max_us;

	if (!delay_ms)
		return;

	if (delay_ms > 20) {
		msleep(delay_ms);
		return;
	}

	min_us = mul_u32_u32(delay_ms, 1000);
	max_us = min_us + 500;

	usleep_range(min_us, max_us);
}

#endif
