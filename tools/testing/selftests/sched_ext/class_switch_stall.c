// SPDX-License-Identifier: GPL-2.0
/*
 * Check that a task which leaves SCHED_EXT while it's waiting doesn't carry
 * that wait over to its next stay on SCHED_EXT. If it did, the watchdog would
 * count the time the task spent outside SCHED_EXT as waiting and abort the
 * scheduler.
 *
 * Copyright (c) 2026 Google LLC.
 */
#define _GNU_SOURCE

#include <bpf/bpf.h>
#include <poll.h>
#include <sched.h>
#include <scx/common.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "class_switch_stall.bpf.skel.h"
#include "scx_test.h"

#ifndef SCHED_EXT
#define SCHED_EXT 7
#endif

/* watchdog timeout, the watchdog checks every half of it */
#define TIMEOUT_MS 3000
/* longer than the watchdog interval, but shorter than the timeout */
#define PARK_MS 2000
/* time spent outside of SCHED_EXT, a stale wait would exceed the timeout */
#define OUTSIDE_MS (TIMEOUT_MS + 1000)

static enum scx_test_status setup(void **ctx)
{
	struct class_switch_stall *skel;

	skel = class_switch_stall__open();
	SCX_FAIL_IF(!skel, "Failed to open");
	SCX_ENUM_INIT(skel);
	skel->struct_ops.class_switch_stall_ops->timeout_ms = TIMEOUT_MS;
	SCX_FAIL_IF(class_switch_stall__load(skel), "Failed to load skel");

	*ctx = skel;

	return SCX_TEST_PASS;
}

/* Echoes back each byte from @in_fd. */
static void child_main(int in_fd, int out_fd)
{
	char c;

	while (read(in_fd, &c, 1) == 1) {
		if (write(out_fd, &c, 1) != 1)
			_exit(1);
	}
	_exit(0);
}

static bool read_byte(int fd, int timeout_ms)
{
	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	char c;

	return poll(&pfd, 1, timeout_ms) == 1 && read(fd, &c, 1) == 1;
}

/* Waits for up to 1s for @pid to be in interruptible sleep. */
static bool wait_for_sleeping(pid_t pid)
{
	char path[64], buf[512], *state;
	bool sleeping;
	FILE *f;
	int i;

	snprintf(path, sizeof(path), "/proc/%d/stat", pid);

	for (i = 0; i < 1000; i++) {
		f = fopen(path, "r");
		if (!f)
			return false;
		/* The state follows "(comm)", and comm may contain ')'. */
		state = fgets(buf, sizeof(buf), f) ? strrchr(buf, ')') : NULL;
		sleeping = state && !strncmp(state, ") S", 3);
		fclose(f);
		if (sleeping)
			return true;
		usleep(1000);
	}
	return false;
}

/* Waits for up to 1s for the target to have been parked @cnt times in total. */
static bool wait_for_parked(struct class_switch_stall *skel, u64 cnt)
{
	int i;

	for (i = 0; i < 1000; i++) {
		if (skel->bss->nr_parked >= cnt)
			return true;
		usleep(1000);
	}
	return false;
}

static bool set_policy(pid_t pid, int policy)
{
	struct sched_param param = {};

	return !sched_setscheduler(pid, policy, &param);
}

static enum scx_test_status run(void *ctx)
{
	struct class_switch_stall *skel = ctx;
	enum scx_test_status status = SCX_TEST_FAIL;
	int to_child[2] = { -1, -1 }, from_child[2] = { -1, -1 };
	struct bpf_link *link = NULL;
	pid_t pid = -1;
	int i;

	if (pipe(to_child) || pipe(from_child)) {
		SCX_ERR("Failed to create pipes");
		goto out;
	}

	link = bpf_map__attach_struct_ops(skel->maps.class_switch_stall_ops);
	if (!link) {
		SCX_ERR("Failed to attach scheduler");
		goto out;
	}

	pid = fork();
	if (pid == 0)
		child_main(to_child[0], from_child[1]);
	if (pid < 0) {
		SCX_ERR("Failed to fork");
		goto out;
	}
	skel->bss->target_pid = pid;

	if (!wait_for_sleeping(pid) || !set_policy(pid, SCHED_EXT)) {
		SCX_ERR("Failed to switch the child to SCHED_EXT");
		goto out;
	}

	/*
	 * Keep the child waiting on the DSQ after it wakes up, and switch it to
	 * SCHED_OTHER while it's still waiting.
	 */
	skel->bss->park_ns = 0;
	skel->bss->park = true;
	if (write(to_child[1], "a", 1) != 1 || !wait_for_parked(skel, 1)) {
		SCX_ERR("The child wasn't parked");
		goto out;
	}
	if (!set_policy(pid, SCHED_OTHER) || !read_byte(from_child[0], 1000)) {
		SCX_ERR("The child didn't run after leaving SCHED_EXT");
		goto out;
	}

	/* Let time pass while the child sleeps outside of SCHED_EXT. */
	if (!wait_for_sleeping(pid)) {
		SCX_ERR("The child didn't block in read()");
		goto out;
	}
	usleep(OUTSIDE_MS * 1000);

	/*
	 * Bring the child back and keep it waiting for longer than the watchdog
	 * interval, but shorter than the timeout. Its wait must start now, not
	 * when it started waiting before it left SCHED_EXT.
	 */
	if (!set_policy(pid, SCHED_EXT)) {
		SCX_ERR("Failed to switch the child back to SCHED_EXT");
		goto out;
	}
	skel->bss->park_ns = PARK_MS * 1000000ULL;
	skel->bss->park = true;
	if (write(to_child[1], "b", 1) != 1 || !wait_for_parked(skel, 2)) {
		SCX_ERR("The child wasn't parked again");
		goto out;
	}
	/*
	 * A stall is detected up to a watchdog interval after the timeout. Wait
	 * long enough for it to be reported by the exit info below.
	 */
	if (!read_byte(from_child[0], 2 * TIMEOUT_MS)) {
		SCX_ERR("The child didn't run after the second wait");
		goto out;
	}

	/*
	 * Only the watchdog aborting the scheduler lets the child run before
	 * it's released. The exit info is recorded asynchronously, wait for it.
	 */
	if (skel->bss->nr_released != 1)
		SCX_TIMED_WAIT_UEI(skel, uei, 1000);

	if (UEI_EXITED(skel, uei)) {
		SCX_ERR("Scheduler exited (kind=%llu): %s",
			(unsigned long long)skel->data->uei.kind,
			skel->data->uei.msg);
		goto out;
	}
	if (skel->bss->nr_released != 1) {
		SCX_ERR("The child was released %llu times, expected 1",
			(unsigned long long)skel->bss->nr_released);
		goto out;
	}

	status = SCX_TEST_PASS;
out:
	if (pid > 0) {
		kill(pid, SIGKILL);
		waitpid(pid, NULL, 0);
	}
	if (link)
		bpf_link__destroy(link);
	for (i = 0; i < 2; i++) {
		if (to_child[i] >= 0)
			close(to_child[i]);
		if (from_child[i] >= 0)
			close(from_child[i]);
	}
	return status;
}

static void cleanup(void *ctx)
{
	struct class_switch_stall *skel = ctx;

	class_switch_stall__destroy(skel);
}

struct scx_test class_switch_stall = {
	.name = "class_switch_stall",
	.description = "Verify that leaving SCHED_EXT ends a task's wait",
	.setup = setup,
	.run = run,
	.cleanup = cleanup,
};
REGISTER_SCX_TEST(&class_switch_stall)
