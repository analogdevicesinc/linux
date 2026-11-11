#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# CoreSight IRQ entry and exit (exclusive)

. "$(dirname "$0")/../lib/coresight_exception.sh"

test_irq_entry_exit()
{
	# IRQs must return to the interrupted PC. Retry at different timer
	# frequencies if trace gaps leave no complete entry/exit pair.
	for freq in 1000 4000 10000; do
		echo "Recording timer IRQs at $freq Hz"
		if ! record_trace_with_cpu_clock "$freq" branch_not_taken_loop; then
			cat "$tmpdir/record.log"
			echo "Failed to record the IRQ workload"
			return 1
		fi

		decode_trace || return 1

		check_exception_pairs "hw int" "iret" 0
		result=$?
		case $result in
		0) return 0 ;;
		1) cat "$tmpdir/script.log"; return 1 ;;
		2) echo "No complete IRQ pair in this recording" ;;
		*) return "$result" ;;
		esac
	done

	echo "FAIL: No complete IRQ pair after three recordings"
	return 1
}

setup_exception_test || exit $?
test_irq_entry_exit
