#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# failed-syscalls python test

set -e

shelldir=$(dirname "$0")
# shellcheck source=lib/setup_python.sh
. "${shelldir}"/lib/setup_python.sh

if ! "$PYTHON" -c 'import perf' > /dev/null 2>&1; then
	echo "Skipping test, perf python module not found"
	exit 2
fi

script_dir="$(dirname "$0")/../../python"
script_path="${script_dir}/failed-syscalls.py"

if ! perf check feature -q libtraceevent > /dev/null 2>&1; then
	echo "Skipping test, libtraceevent is disabled"
	exit 2
fi

if [ ! -f "$script_path" ]; then
	echo "Skipping test, failed-syscalls.py not found at $script_path"
	exit 2
fi

err=0
temp_data=""
temp_out=""

cleanup() {
	rm -f "${temp_data}" "${temp_out}"
}
trap 'cleanup' EXIT TERM INT

temp_data=$(mktemp /tmp/perf.data.XXXXXX)
temp_out=$(mktemp /tmp/perf.out.XXXXXX)

echo "Testing failed-syscalls.py..."

# Check if sys_exit event can be recorded
if ! perf record -e raw_syscalls:sys_exit -o /dev/null -- true >/dev/null 2>&1; then
	if ! perf record -e syscalls:sys_exit -o /dev/null -- true >/dev/null 2>&1; then
		echo "Skipping test, no permission or support for sys_exit event"
		exit 2
	else
		EVENT="syscalls:sys_exit"
	fi
else
	EVENT="raw_syscalls:sys_exit"
fi

# Run perf record with a command that fails a syscall (ls non-existent file).
# ls exits with non-zero, so perf record returns non-zero exit code of the workload.
perf record -e "${EVENT}" -o "${temp_data}" \
	-- ls /nonexistent_file_for_test >/dev/null 2>&1 || true

if [ ! -s "${temp_data}" ]; then
	echo "Skipping test, perf record failed to create data"
	exit 2
fi

# Check that the script executes
if ! "$PYTHON" "$script_path" -i "${temp_data}" > "${temp_out}"; then
	echo "failed-syscalls.py test failed"
	err=1
else
	if ! grep -q "failed syscalls by comm" "${temp_out}" || \
		! grep -Eq '^ls[[:space:]]+[0-9]+' "${temp_out}"; then
		echo "Failed to find the metrics table header or expected error"
		err=1
	else
		echo "failed-syscalls test passed."
	fi
fi
rm -f "${temp_out}"

exit $err
