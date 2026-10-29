// SPDX-License-Identifier: GPL-2.0
#include <errno.h>
#include <limits.h>
#include <linux/capability.h>
#include <stdio.h>
#include <sys/msg.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "kselftest.h"

static int drop_cap_sys_resource(void)
{
	struct __user_cap_header_struct hdr = {
		.version = _LINUX_CAPABILITY_VERSION_3,
	};
	struct __user_cap_data_struct data[_LINUX_CAPABILITY_U32S_3];
	unsigned int index = CAP_SYS_RESOURCE / 32;
	__u32 mask = 1U << (CAP_SYS_RESOURCE % 32);

	if (syscall(SYS_capget, &hdr, data))
		return -1;

	data[index].effective &= ~mask;
	data[index].permitted &= ~mask;

	return syscall(SYS_capset, &hdr, data);
}

int main(void)
{
	struct msqid_ds ds;
	int msqid;
	int ret;

	ksft_print_header();
	ksft_set_plan(1);

#if ULONG_MAX <= UINT_MAX
	ksft_exit_skip("requires unsigned long wider than unsigned int\n");
#endif

	if (drop_cap_sys_resource()) {
		if (errno == ENOSYS)
			ksft_exit_skip("capabilities are unavailable\n");
		ksft_exit_fail_perror("drop CAP_SYS_RESOURCE");
	}

	msqid = msgget(IPC_PRIVATE, 0600);
	if (msqid == -1) {
		if (errno == ENOSYS)
			ksft_exit_skip("System V message queues are unavailable\n");
		ksft_exit_fail_perror("msgget");
	}

	ret = msgctl(msqid, IPC_STAT, &ds);
	if (ret == -1) {
		ksft_perror("msgctl(IPC_STAT)");
		goto out_fail;
	}

	/*
	 * Without CAP_SYS_RESOURCE, a value above UINT_MAX must be rejected
	 * rather than truncated to zero.
	 */
	ds.msg_qbytes = (unsigned long)UINT_MAX + 1;
	errno = 0;
	ret = msgctl(msqid, IPC_SET, &ds);
	if (ret == -1 && errno == EPERM) {
		ksft_test_result_pass("IPC_SET rejects oversized msg_qbytes\n");
		msgctl(msqid, IPC_RMID, NULL);
		ksft_exit_pass();
	}

	if (ret == -1)
		ksft_perror("msgctl(IPC_SET)");
	else
		ksft_print_msg("msgctl(IPC_SET) unexpectedly succeeded\n");

out_fail:
	msgctl(msqid, IPC_RMID, NULL);
	ksft_test_result_fail("IPC_SET accepted oversized msg_qbytes\n");
	ksft_exit_fail();
}
