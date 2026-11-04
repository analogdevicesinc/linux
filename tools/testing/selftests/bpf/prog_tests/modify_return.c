// SPDX-License-Identifier: GPL-2.0

/*
 * Copyright 2020 Google LLC.
 */

#include <test_progs.h>
#include <unistd.h>
#include "modify_return.skel.h"

#define LOWER(x) ((x) & 0xffff)
#define UPPER(x) ((x) >> 16)

#define MAX_INSTANCE_PATH_LEN	128
#define TRACEFS_DEFAULT		"/sys/kernel/tracing"
#define TRACEFS_IN_DEBUGFS	"/sys/kernel/debug/tracing"
#define TRACE_FTRACE_DEFAULT	TRACEFS_DEFAULT "/trace"

static void run_test(__u32 input_retval, __u16 want_side_effect, __s16 want_ret)
{
	struct modify_return *skel = NULL;
	int err, prog_fd;
	__u16 side_effect;
	__s16 ret;
	LIBBPF_OPTS(bpf_test_run_opts, topts);

	skel = modify_return__open_and_load();
	if (!ASSERT_OK_PTR(skel, "skel_load"))
		goto cleanup;

	skel->bss->input_retval = input_retval;
	skel->bss->test_pid = getpid();

	err = modify_return__attach(skel);
	if (!ASSERT_OK(err, "modify_return__attach failed"))
		goto cleanup;

	prog_fd = bpf_program__fd(skel->progs.fmod_ret_test);
	err = bpf_prog_test_run_opts(prog_fd, &topts);
	ASSERT_OK(err, "test_run");

	side_effect = UPPER(topts.retval);
	ret = LOWER(topts.retval);

	ASSERT_EQ(ret, want_ret, "test_run ret");
	ASSERT_EQ(side_effect, want_side_effect, "modify_return side_effect");
	ASSERT_EQ(skel->bss->fentry_result, 1, "modify_return fentry_result");
	ASSERT_EQ(skel->bss->fexit_result, 1, "modify_return fexit_result");
	ASSERT_EQ(skel->bss->fmod_ret_result, 1, "modify_return fmod_ret_result");

	ASSERT_EQ(skel->bss->fentry_result2, 1, "modify_return fentry_result2");
	ASSERT_EQ(skel->bss->fexit_result2, 1, "modify_return fexit_result2");
	ASSERT_EQ(skel->bss->fmod_ret_result2, 1, "modify_return fmod_ret_result2");

cleanup:
	modify_return__destroy(skel);
}

void test_modify_return(void)
{
	run_test(0 /* input_retval */,
		 2 /* want_side_effect */,
		 33 /* want_ret */);
	run_test(-EINVAL /* input_retval */,
		 0 /* want_side_effect */,
		 -EINVAL * 2 /* want_ret */);
}

void test_modify_return_graph(void)
{
	char instance[MAX_INSTANCE_PATH_LEN];
	char *ftrace_dir;
	int ret;

	ftrace_dir = access(TRACE_FTRACE_DEFAULT, F_OK) ? TRACEFS_IN_DEBUGFS :
							  TRACEFS_DEFAULT;

	ret = snprintf(instance, MAX_INSTANCE_PATH_LEN,
		       "%s/instances/bpf-XXXXXX", ftrace_dir);
	if (ret < 0)
		return;

	if (!ASSERT_OK_PTR(mkdtemp(instance), "mkdtemp"))
		return;

	SYS(out, "echo 'bpf_modify_return_test*' > %s/set_ftrace_filter",
	    instance);
	SYS(out, "echo function_graph > %s/current_tracer", instance);

	test_modify_return();

	SYS(out, "grep -q bpf_modify_return_test2 %s/trace", instance);
out:
	ASSERT_OK(rmdir(instance), "rmdir");
}
