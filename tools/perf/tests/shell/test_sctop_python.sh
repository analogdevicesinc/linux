#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# sctop python test

set -e

shelldir=$(dirname "$0")
# shellcheck source=lib/setup_python.sh
. "${shelldir}"/lib/setup_python.sh

if ! "$PYTHON" -c 'import perf' > /dev/null 2>&1; then
	echo "Skipping test, perf python module not found"
	exit 2
fi

script_dir="$(dirname "$0")/../../python"
script_path="${script_dir}/sctop.py"

if ! perf check feature -q libtraceevent > /dev/null 2>&1; then
	echo "Skipping test, libtraceevent is disabled"
	exit 2
fi

if [ ! -f "$script_path" ]; then
	echo "Skipping test, sctop.py not found at $script_path"
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

echo "Testing sctop.py..."

# Create a perf.data file.
if perf list | grep -q "raw_syscalls:sys_enter"; then
	perf record -e raw_syscalls:sys_enter -a -o "${temp_data}" \
		-- sleep 0.1 >/dev/null 2>&1 || \
		{ echo "Skipping test, perf record failed"; exit 2; }
else
	echo "Skipping test, no raw_syscalls:sys_enter event"
	exit 2
fi

if [ ! -s "${temp_data}" ]; then
	echo "Skipping test, perf record failed to create data"
	exit 2
fi

# Check that the script executes
if ! "$PYTHON" "$script_path" -i "${temp_data}" > "${temp_out}"; then
	echo "sctop.py test failed"
	err=1
elif ! grep -E -q "[0-9]+$" "${temp_out}"; then
	echo "Failed to find metric data rows in default run"
	err=1
elif ! "$PYTHON" "$script_path" -i "${temp_data}" sleep 1 > "${temp_out}"; then
	echo "sctop.py comm+interval test failed"
	err=1
else
	if ! grep -E -q "[0-9]+$" "${temp_out}"; then
		echo "Failed to find metric data rows"
		err=1
	else
		echo "sctop test passed."
	fi
fi
rm -f "${temp_out}"

exit $err
