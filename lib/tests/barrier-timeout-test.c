// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests exercising smp_cond_load_relaxed_timeout().
 *
 * Copyright (c) 2026, Oracle Corp.
 * Author: Ankur Arora <ankur.a.arora@oracle.com>
 */

#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/sched/clock.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <asm/barrier.h>
#include <kunit/test.h>
#include <kunit/visibility.h>

MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");

struct clock_state {
	s64 start_time;
	s64 end_time;
};

#define TIMEOUT_MSEC	2
#define TEST_FLAG_VAL	BIT(2)
static unsigned int flag;

static s64 basic_clock(struct clock_state *clk)
{
	clk->end_time = local_clock();
	return clk->end_time;
}

static void update_flags(void)
{
	WRITE_ONCE(flag, TEST_FLAG_VAL);
}

static s64 mock_clock(struct clock_state *clk)
{
	s64 clk_mid = clk->start_time + (TIMEOUT_MSEC * NSEC_PER_MSEC)/2;

	clk->end_time = local_clock();
	if (clk->end_time >= clk_mid)
		update_flags();
	return clk->end_time;
}

typedef s64 (*clkfn_t)(struct clock_state *);

static void test_smp_cond_relaxed_timeout(struct kunit *test,
					  clkfn_t clock, bool succeeds)
{
	struct clock_state clk = {
		.start_time = local_clock(),
		.end_time = local_clock(),
	};
	s64 runtime, timeout_ns = TIMEOUT_MSEC * NSEC_PER_MSEC;
	unsigned int result;

	result = smp_cond_load_relaxed_timeout(&flag,
					       (VAL & TEST_FLAG_VAL),
					       clock(&clk),
					       timeout_ns);

	runtime = clk.end_time - clk.start_time;
	KUNIT_EXPECT_EQ(test, (bool)(result & TEST_FLAG_VAL), succeeds);
	KUNIT_EXPECT_EQ(test, runtime <= timeout_ns, succeeds);
}

static int smp_cond_threadfn(void *data)
{
	udelay(TIMEOUT_MSEC * USEC_PER_MSEC / 4);

	/*
	 * Update flags after a delay to give smp_cond_relaxed_timeout()
	 * time to get started.
	 */
	update_flags();
	return 0;
}

static void smp_cond_relaxed_timeout_succeeds(struct kunit *test)
{
	struct task_struct *task;

	flag = 0;

	task = kthread_run(smp_cond_threadfn, &flag, "smp_cond_thread");

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, task);
	test_smp_cond_relaxed_timeout(test, &basic_clock, true);

	kthread_stop(task);
}

static void smp_cond_relaxed_timeout_mocked(struct kunit *test)
{
	flag = 0;
	test_smp_cond_relaxed_timeout(test, &mock_clock, true);
}

static void smp_cond_relaxed_timeout_expires(struct kunit *test)
{
	flag = 0;
	test_smp_cond_relaxed_timeout(test, &basic_clock, false);
}

static struct kunit_case barrier_timeout_test_cases[] = {
	KUNIT_CASE(smp_cond_relaxed_timeout_mocked),
	KUNIT_CASE(smp_cond_relaxed_timeout_succeeds),
	KUNIT_CASE(smp_cond_relaxed_timeout_expires),
	{}
};

static struct kunit_suite barrier_timeout_test_suite = {
	.name = "smp-cond-load-relaxed-timeout",
	.test_cases = barrier_timeout_test_cases,
};

kunit_test_suite(barrier_timeout_test_suite);

MODULE_DESCRIPTION("KUnit tests for smp_cond_load_relaxed_timeout()");
MODULE_LICENSE("GPL");
