// SPDX-License-Identifier: GPL-2.0
/*
 * An API to allow a function, that may fail, to be executed, and recover in a
 * controlled manner.
 *
 * Copyright (C) 2019, Google LLC.
 * Author: Brendan Higgins <brendanhiggins@google.com>
 */

#include <kunit/test.h>
#include <linux/completion.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched/task.h>

#include "try-catch-impl.h"

void __noreturn kunit_try_catch_throw(struct kunit_try_catch *try_catch)
{
	try_catch->try_result = -EFAULT;
	kthread_exit(0);
}
EXPORT_SYMBOL_GPL(kunit_try_catch_throw);

static int kunit_generic_run_threadfn_adapter(void *data)
{
	struct kunit_try_catch *try_catch = data;

	try_catch->try_result = -EINTR;
	try_catch->try(try_catch->context);
	if (try_catch->try_result == -EINTR)
		try_catch->try_result = 0;

	return 0;
}

static unsigned long kunit_test_timeout(void)
{
	/*
	 * TODO(brendanhiggins@google.com): We should probably have some type of
	 * variable timeout here. The only question is what that timeout value
	 * should be.
	 *
	 * The intention has always been, at some point, to be able to label
	 * tests with some type of size bucket (unit/small, integration/medium,
	 * large/system/end-to-end, etc), where each size bucket would get a
	 * default timeout value kind of like what Bazel does:
	 * https://docs.bazel.build/versions/master/be/common-definitions.html#test.size
	 * There is still some debate to be had on exactly how we do this. (For
	 * one, we probably want to have some sort of test runner level
	 * timeout.)
	 *
	 * For more background on this topic, see:
	 * https://mike-bland.com/2011/11/01/small-medium-large.html
	 *
	 * If tests timeout due to exceeding sysctl_hung_task_timeout_secs,
	 * the task will be killed and an oops generated.
	 */
	return 300 * msecs_to_jiffies(MSEC_PER_SEC); /* 5 min */
}

void kunit_try_catch_run(struct kunit_try_catch *try_catch, void *context)
{
	struct kunit *test = try_catch->test;
	struct task_struct *task_struct;
	struct completion *task_done;
	int exit_code, time_remaining;

	try_catch->context = context;
	try_catch->try_result = 0;
	task_struct = kthread_create(kunit_generic_run_threadfn_adapter,
				     try_catch, "kunit_try_catch_thread");
	if (IS_ERR(task_struct)) {
		try_catch->try_result = PTR_ERR(task_struct);
		try_catch->catch(try_catch->context);
		return;
	}
	get_task_struct(task_struct);
	/*
	 * As for a vfork(2), task_struct->vfork_done (pointing to the
	 * underlying kthread->exited) can be used to wait for the end of a
	 * kernel thread. It is set to NULL when the thread exits, so we
	 * keep a copy here.
	 */
	task_done = task_struct->vfork_done;
	wake_up_process(task_struct);

	time_remaining = wait_for_completion_timeout(task_done,
						     kunit_test_timeout());
	if (time_remaining == 0) {
		try_catch->try_result = -ETIMEDOUT;
		kthread_stop(task_struct);
	}

	put_task_struct(task_struct);
	exit_code = try_catch->try_result;

	if (!exit_code)
		return;

	if (exit_code == -EFAULT)
		try_catch->try_result = 0;
	else if (exit_code == -EINTR) {
		if (test->last_seen.file)
			kunit_err(test, "try faulted: last line seen %s:%d\n",
				  test->last_seen.file, test->last_seen.line);
		else
			kunit_err(test, "try faulted\n");
	} else if (exit_code == -ETIMEDOUT)
		kunit_err(test, "try timed out\n");
	else if (exit_code)
		kunit_err(test, "Unknown error: %d\n", exit_code);

	try_catch->catch(try_catch->context);
}
EXPORT_SYMBOL_GPL(kunit_try_catch_run);
