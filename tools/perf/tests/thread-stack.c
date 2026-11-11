// SPDX-License-Identifier: GPL-2.0
#include <linux/kernel.h>
#include <unistd.h>
#include "tests.h"
#include "util/addr_location.h"
#include "util/event.h"
#include "util/sample.h"
#include "util/thread.h"
#include "util/thread-stack.h"

#define CALL_REF		1234UL
#define RET_REF			5678UL

struct return_check {
	unsigned int matched;
	unsigned int unmatched;
};

static int check_call_return(struct call_return *cr,
			     u64 *parent_db_id __maybe_unused, void *data)
{
	struct return_check *check = data;

	if (cr->call_ref == CALL_REF && cr->return_ref == RET_REF && !cr->flags)
		check->matched++;
	else
		check->unmatched++;

	return 0;
}

/* A zero expected_ret_addr asks the stack to use ip + insn_len. */
static int check_return_address(u64 expected_ret_addr, u64 actual_ret_addr,
				u32 flags)
{
	struct call_return_processor *crp;
	struct return_check check = { };
	struct thread *thread;
	struct addr_location from = { }, to = { };
	struct perf_sample sample = { };
	int ret = TEST_FAIL;

	thread = thread__new(getpid(), getpid());
	if (!thread)
		return TEST_FAIL;

	crp = call_return_processor__new(check_call_return, &check);
	if (!crp)
		goto out;

	sample.ip = 0x1000;		/* Call or exception source addr */
	sample.addr = 0x2000;		/* Callee or exception handler addr */
	sample.ret_addr = expected_ret_addr;
	sample.flags = PERF_IP_FLAG_BRANCH | PERF_IP_FLAG_CALL | flags;
	sample.time = 1;
	/* Model the opcode length after an instruction fetch. */
	sample.insn_len = 4;
	if (thread_stack__process(thread, thread__comm(thread), &sample,
				  &from, &to, CALL_REF, crp))
		goto out;

	sample.ip = 0x2000;		/* Return instruction addr */
	sample.addr = actual_ret_addr;	/* Return branch target addr */
	sample.ret_addr = 0;
	sample.flags = PERF_IP_FLAG_BRANCH | PERF_IP_FLAG_RETURN |
		       (flags & (PERF_IP_FLAG_INTERRUPT | PERF_IP_FLAG_SYSCALLRET));
	sample.time = 2;
	if (thread_stack__process(thread, thread__comm(thread), &sample,
				  &to, &from, RET_REF, crp))
		goto out;

	if (check.matched == 1 && !check.unmatched)
		ret = TEST_OK;

out:
	thread__put(thread);
	call_return_processor__free(crp);
	return ret;
}

static int test__thread_stack(struct test_suite *test __maybe_unused,
			      int subtest __maybe_unused)
{
	static const struct {
		const char *name;
		u64 expected_ret_addr;
		u64 actual_ret_addr;
		u32 flags;
	} cases[] = {
		{ "ordinary call", 0, 0x1004, 0 },
		{ "interrupt", 0x1000, 0x1000, PERF_IP_FLAG_ASYNC | PERF_IP_FLAG_INTERRUPT },
		{ "fault or trap", 0x1000, 0x1000, PERF_IP_FLAG_INTERRUPT },
		{ "SVC", 0x1004, 0x1004, PERF_IP_FLAG_SYSCALLRET },
	};

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		if (check_return_address(cases[i].expected_ret_addr,
					 cases[i].actual_ret_addr, cases[i].flags)) {
			pr_debug("Incorrect return address for %s\n", cases[i].name);
			return TEST_FAIL;
		}
	}
	return TEST_OK;
}

DEFINE_SUITE("Thread stack return addresses after instruction fetching", thread_stack);
