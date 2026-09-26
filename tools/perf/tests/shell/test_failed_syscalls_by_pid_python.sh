#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# failed-syscalls-by-pid python test

set -e

shelldir=$(dirname "$0")
# shellcheck source=lib/setup_python.sh
. "${shelldir}"/lib/setup_python.sh

if ! "$PYTHON" -c 'import perf' > /dev/null 2>&1; then
	echo "Skipping test, perf python module not found"
	exit 2
fi

script_dir="$(dirname "$0")/../../python"
script_path="${script_dir}/failed-syscalls-by-pid.py"

if ! perf check feature -q libtraceevent > /dev/null 2>&1; then
	echo "Skipping test, libtraceevent is disabled"
	exit 2
fi

if [ ! -f "$script_path" ]; then
	echo "Skipping test, failed-syscalls-by-pid.py not found at $script_path"
	exit 2
fi

err=0
temp_dir=$(mktemp -d /tmp/perf-failed-syscalls-by-pid-XXXXXX)
temp_data="${temp_dir}/perf.data"
temp_out="${temp_dir}/perf.out"

cleanup() {
	rm -rf "${temp_dir}"
}

trap 'cleanup' EXIT TERM INT

test_file_mode() {
	echo "Testing failed-syscalls-by-pid.py..."

	# Check if syscalls:sys_exit is supported/readable
	if ! perf record -e syscalls:sys_exit -o /dev/null -- true >/dev/null 2>&1; then
		if ! perf record -e raw_syscalls:sys_exit -o /dev/null -- true >/dev/null 2>&1; then
			echo "Skipping test, no syscalls:sys_exit or raw_syscalls:sys_exit event"
			exit 2
		else
			EVENT="raw_syscalls:sys_exit"
		fi
	else
		EVENT="syscalls:sys_exit"
	fi

	# Generate some events by running a command that should fail at least some syscall
	# (e.g. failing stat on non-existent file).
	# Using '|| true' because 'perf record' returns the exit code of 'ls',
	# which fails with ENOENT
	perf record -e "${EVENT}" -o "${temp_data}" -- ls /does_not_exist >/dev/null 2>&1 || true
	if [ ! -s "${temp_data}" ]; then
		echo "Skipping test, perf record failed to create data"
		exit 2
	fi

	# Run the script and check output
	if ! perf script failed-syscalls-by-pid -i "${temp_data}" > "${temp_out}"; then
		echo "failed-syscalls-by-pid test failed."
		err=1
	elif ! grep -n -q "err = ENOENT" "${temp_out}"; then
		echo "Failed to find expected failed syscalls"
		cat "${temp_out}"
		err=1
	elif ! perf script failed-syscalls-by-pid -i "${temp_data}" "ls" > "${temp_out}.comm" || \
	     ! grep -q "err = ENOENT" "${temp_out}.comm"; then
		echo "failed-syscalls-by-pid comm filter test failed."
		cat "${temp_out}.comm"
		err=1
	else
		ls_pid=$(sed -n 's/^ls \[\([0-9][0-9]*\)\].*/\1/p' "${temp_out}.comm" | head -n 1)
		if [ -z "${ls_pid}" ] || \
		   ! perf script failed-syscalls-by-pid -i "${temp_data}" \
			"${ls_pid}" > "${temp_out}.pid" || \
		   ! grep -q "err = ENOENT" "${temp_out}.pid"; then
			echo "failed-syscalls-by-pid PID filter test failed."
			cat "${temp_out}.pid" 2>/dev/null || true
			err=1
		else
			echo "failed-syscalls-by-pid test passed."
		fi
	fi
	rm -f "${temp_out}" "${temp_out}.comm" "${temp_out}.pid"
}

test_file_mode

exit $err
