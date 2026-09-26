#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# intel-pt-events python test (exclusive)

set -e

shelldir=$(dirname "$0")
# shellcheck source=lib/setup_python.sh
. "${shelldir}"/lib/setup_python.sh

if ! "$PYTHON" -c 'import perf' > /dev/null 2>&1; then
	echo "Skipping test, perf python module not found"
	exit 2
fi

script_dir="$(dirname "$0")/../../python"
script_path="${script_dir}/intel-pt-events.py"

if [ ! -f "$script_path" ]; then
	echo "Skipping test, intel-pt-events.py not found at $script_path"
	exit 2
fi

err=0
temp_dir=""
temp_data=""
temp_out=""

cleanup() {
	[ -n "${temp_dir}" ] && rm -rf "${temp_dir}"
}

trap 'cleanup' EXIT TERM INT

temp_dir=$(mktemp -d /tmp/perf.ipt.XXXXXX)
temp_data="${temp_dir}/perf.data"
temp_out="${temp_dir}/perf.out"

test_intel_pt() {
	echo "Testing intel-pt-events.py with intel_pt..."

	rm -f "${temp_data}" "${temp_out}"
	# Generate some intel_pt events; use a subshell that waits for uname so
	# uname's AUX buffer is flushed before the parent workload exits.
	if ! perf record -B -N --no-bpf-event -e intel_pt//u -o "${temp_data}" \
		-- sh -c "uname; true" >/dev/null 2>&1; then
		echo "Skipping intel_pt test, intel_pt not available."
		exit 2
	fi

	# Run the script and check output
	if ! "$PYTHON" "$script_path" -i "${temp_data}" > "${temp_out}"; then
		echo "intel-pt-events.py test failed."
		err=1
	else
		if ! grep -q "Intel PT Branch Trace" "${temp_out}" || \
		   ! grep -q "uname" "${temp_out}"; then
			echo "Failed to find expected output: $(cat "${temp_out}")"
			err=1
		else
			echo "intel-pt-events test passed."
		fi
	fi
	rm -f "${temp_out}"
}

test_intel_pt

exit $err
