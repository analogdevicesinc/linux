#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
# task-analyzer python test

# shellcheck source=lib/setup_python.sh
. "$(dirname "$0")"/lib/setup_python.sh

tmpdir=$(mktemp -d /tmp/perf-script-task-analyzer-XXXXX)
perfdata="$tmpdir/perf.data"
csv="$tmpdir/csv"
csvsummary="$tmpdir/csvsummary"
err=0

# Set up perfdir and PERF_EXEC_PATH
if [ "x$PERF_EXEC_PATH" = "x" ]; then
  perfdir="$(dirname "$0")/../.."
  if [ -f "$perfdir/python/task-analyzer.py" ]; then
    export PERF_EXEC_PATH="$perfdir"
  fi
else
  perfdir="$PERF_EXEC_PATH"
fi

# Disable lsan to avoid warnings about python memory leaks.
export ASAN_OPTIONS=detect_leaks=0

cleanup() {
  rm -f "${perfdata}"
  rm -f "${perfdata}".old
  rm -rf "$tmpdir"

  trap - exit term int
}

trap_cleanup() {
  cleanup
  exit 1
}
trap trap_cleanup exit term int

report() {
	if [ "$1" = 0 ]; then
		echo "PASS: \"$2\""
	else
		echo "FAIL: \"$2\" Error message: \"$3\""
		err=1
	fi
}

check_exec_0() {
	if [ $? != 0 ]; then
		report 1 "invocation of $1 command failed"
	fi
}

find_str_or_fail() {
	grep -q "$1" "$2"
	if [ "$?" != 0 ]; then
		report 1 "$3" "Failed to find required string:'${1}'."
	else
		report 0 "$3"
	fi
}

# check if perf is compiled with libtraceevent support
skip_no_probe_record_support() {
	perf check feature -q libtraceevent && return 0
	return 2
}

prepare_perf_data() {
	# 1s should be sufficient to catch at least some switches
	perf record -e sched:sched_switch -a -o "${perfdata}" -- sleep 1 > /dev/null 2>&1
	# check if perf data file got created in above step.
	if [ ! -e "${perfdata}" ]; then
		printf "FAIL: perf record failed to create \"${perfdata}\" \n"
		return 1
	fi
}

# check standard inkvokation with no arguments
test_basic() {
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata}"
	find_str_or_fail "Comm" "$out" "${FUNCNAME[0]}"
}

test_ns_rename(){
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" \
		--ns \
		--rename-comms-by-tids 0:random > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--ns \
		--rename-comms-by-tids 0:random"
	find_str_or_fail "Comm" "$out" "${FUNCNAME[0]}"
}

test_ms_filtertasks_highlight(){
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" \
		--ms --filter-tasks perf --highlight-tasks perf \
	> "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--ms --filter-tasks perf --highlight-tasks perf"
	find_str_or_fail "Comm" "$out" "${FUNCNAME[0]}"
}

test_extended_times_timelimit_limittasks() {
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" --extended-times \
		--time-limit :99999 \
	--limit-to-tasks perf > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--extended-times \
		--time-limit :99999 --limit-to-tasks perf"
	find_str_or_fail "Out-Out" "$out" "${FUNCNAME[0]}"
}

test_summary() {
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" --summary > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} --summary"
	find_str_or_fail "Summary" "$out" "${FUNCNAME[0]}"
}

test_summaryextended() {
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" --summary-extended > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} --summary-extended"
	find_str_or_fail "Inter Task Times" "$out" "${FUNCNAME[0]}"
}

test_summaryonly() {
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" --summary-only > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} --summary-only"
	find_str_or_fail "Summary" "$out" "${FUNCNAME[0]}"
}

test_extended_times_summary_ns() {
	out="$tmpdir/perf.out"
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" --extended-times \
		--summary --ns > "$out"
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--extended-times \
		--summary \
		--ns"
	find_str_or_fail "Out-Out" "$out" "${FUNCNAME[0]}"
	find_str_or_fail "Summary" "$out" "${FUNCNAME[0]}"
}

test_csv() {
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" --csv "${csv}" > /dev/null
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} --csv ${csv}"
	find_str_or_fail "Comm;" "${csv}" "${FUNCNAME[0]}"
}

test_csv_extended_times() {
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" \
		--csv "${csv}" \
		--extended-times > /dev/null
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--csv ${csv} \
		--extended-times"
	find_str_or_fail "Out-Out;" "${csv}" "${FUNCNAME[0]}"
}

test_csvsummary() {
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" \
		--csv-summary "${csvsummary}" > /dev/null
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--csv-summary ${csvsummary}"
	find_str_or_fail "Comm;" "${csvsummary}" "${FUNCNAME[0]}"
}

test_csvsummary_extended() {
	$PYTHON $perfdir/python/task-analyzer.py -i "${perfdata}" \
		--csv-summary "${csvsummary}" --summary-extended \
	>/dev/null
	check_exec_0 "$PYTHON $perfdir/python/task-analyzer.py -i ${perfdata} \
		--csv-summary ${csvsummary} --summary-extended"
	find_str_or_fail "Out-Out;" "${csvsummary}" "${FUNCNAME[0]}"
}

skip_no_probe_record_support
err=$?
if [ $err -ne 0 ]; then
	echo "WARN: Skipping tests. No libtraceevent support"
	cleanup
	exit $err
fi
prepare_perf_data || {
	echo "Skipping tests, failed to prepare perf.data"
	cleanup
	exit 2
}
test_basic
test_ns_rename
test_ms_filtertasks_highlight
test_extended_times_timelimit_limittasks
test_summary
test_summaryextended
test_summaryonly
test_extended_times_summary_ns
test_csv
test_csvsummary
test_csv_extended_times
test_csvsummary_extended
cleanup
exit $err
