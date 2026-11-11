#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# CoreSight emulated instruction entry and exit (exclusive)

. "$(dirname "$0")/../lib/coresight_exception.sh"

test_trap_entry_exit()
{
	local attempt result

	# traploop reads ID_AA64ISAR0_EL1 from EL0. The kernel emulates MRS and
	# advances the saved PC, so IRET must resume four bytes after the trap.
	for attempt in 1 2 3; do
		echo "Recording emulated instructions (attempt $attempt/3)"
		if ! record_trace traploop 256; then
			cat "$tmpdir/record.log"
			echo "Failed to record the traploop workload"
			return 1
		fi
		decode_trace || return 1
		# Select MRS Xt, ID_AA64ISAR0_EL1; the low five bits select Xt.
		check_exception_pairs "int.*insn: [01][[:xdigit:]] 06 38 d5" "iret" 4
		result=$?

		# An instruction page fault at MRS can also match the opcode filter.
		# Retry failed checks to tolerate such transient faults.
		case $result in
		0) return 0 ;;
		1|2) continue ;;
		*) return "$result" ;;
		esac
	done

	echo "FAIL: No valid trap pair after three recordings"
	return 1
}

setup_exception_test || exit $?
test_trap_entry_exit
