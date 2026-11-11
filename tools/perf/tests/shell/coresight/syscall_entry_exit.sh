#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# CoreSight syscall entry and exit (exclusive)

. "$(dirname "$0")/../lib/coresight_exception.sh"

test_syscall_entry_exit()
{
	local attempt result

	# callchain() invokes gettid() through SVC, which returns to PC + 4.
	for attempt in 1 2 3; do
		echo "Recording syscalls (attempt $attempt/3)"
		if ! record_trace callchain; then
			cat "$tmpdir/record.log"
			echo "Failed to record the callchain workload"
			return 1
		fi
		decode_trace || return 1
		check_exception_pairs "syscall" "sysret" 4
		result=$?
		if [ "$result" -ne 2 ]; then
			return "$result"
		fi
	done

	echo "FAIL: No complete syscall pair after three recordings"
	return 1
}

setup_exception_test || exit $?
test_syscall_entry_exit
