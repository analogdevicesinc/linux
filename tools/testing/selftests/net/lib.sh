#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

##############################################################################
# Defines

: "${WAIT_TIMEOUT:=20}"

# Whether to pause on after a failure.
: "${PAUSE_ON_FAIL:=no}"

BUSYWAIT_TIMEOUT=$((WAIT_TIMEOUT * 1000)) # ms

# Kselftest framework constants.
ksft_pass=0
ksft_fail=1
ksft_xfail=2
ksft_skip=4

# namespace list created by setup_ns
NS_LIST=()

# Exit status to return at the end. Set in case one of the tests fails.
EXIT_STATUS=0
# Per-test return value. Clear at the beginning of each test.
RET=0

##############################################################################
# Helpers

__ksft_status_merge()
{
	local a=$1; shift
	local b=$1; shift
	local -A weights
	local weight=0

	local i
	for i in "$@"; do
		weights[$i]=$((weight++))
	done

	if [[ ${weights[$a]} > ${weights[$b]} ]]; then
		echo "$a"
		return 0
	else
		echo "$b"
		return 1
	fi
}

ksft_status_merge()
{
	local a=$1; shift
	local b=$1; shift

	__ksft_status_merge "$a" "$b" \
		$ksft_pass $ksft_xfail $ksft_skip $ksft_fail
}

ksft_exit_status_merge()
{
	local a=$1; shift
	local b=$1; shift

	__ksft_status_merge "$a" "$b" \
		$ksft_xfail $ksft_pass $ksft_skip $ksft_fail
}

loopy_wait()
{
	local sleep_cmd=$1; shift
	local timeout_ms=$1; shift

	local start_time="$(date -u +%s%3N)"
	while true
	do
		local out
		if out=$("$@"); then
			echo -n "$out"
			return 0
		fi

		local current_time="$(date -u +%s%3N)"
		if ((current_time - start_time > timeout_ms)); then
			echo -n "$out"
			return 1
		fi

		$sleep_cmd
	done
}

busywait()
{
	local timeout_ms=$1; shift

	loopy_wait : "$timeout_ms" "$@"
}

# timeout in seconds
slowwait()
{
	local timeout_sec=$1; shift

	loopy_wait "sleep 0.1" "$((timeout_sec * 1000))" "$@"
}

until_counter_is()
{
	local expr=$1; shift
	local current=$("$@")

	echo $((current))
	((current $expr))
}

busywait_for_counter()
{
	local timeout=$1; shift
	local delta=$1; shift

	local base=$("$@")
	busywait "$timeout" until_counter_is ">= $((base + delta))" "$@"
}

slowwait_for_counter()
{
	local timeout=$1; shift
	local delta=$1; shift

	local base=$("$@")
	slowwait "$timeout" until_counter_is ">= $((base + delta))" "$@"
}

# Check for existence of tools which are built as part of selftests
# but may also already exist in $PATH
check_gen_prog()
{
	local prog_name=$1; shift

	if ! which $prog_name >/dev/null 2>/dev/null; then
		PATH=$PWD:$PATH
		if ! which $prog_name >/dev/null; then
			echo "'$prog_name' command not found; skipping tests"
			exit $ksft_skip
		fi
	fi
}

remove_ns_list()
{
	local item=$1
	local ns
	local ns_list=("${NS_LIST[@]}")
	NS_LIST=()

	for ns in "${ns_list[@]}"; do
		if [ "${ns}" != "${item}" ]; then
			NS_LIST+=("${ns}")
		fi
	done
}

cleanup_ns()
{
	local ns=""
	local ret=0

	for ns in "$@"; do
		[ -z "${ns}" ] && continue
		ip netns pids "${ns}" 2> /dev/null | xargs -r kill || true
		ip netns delete "${ns}" &> /dev/null || true
		if ! busywait $BUSYWAIT_TIMEOUT ip netns list \| grep -vq "^$ns$" &> /dev/null; then
			echo "Warn: Failed to remove namespace $ns"
			ret=1
		else
			remove_ns_list "${ns}"
		fi
	done

	return $ret
}

cleanup_all_ns()
{
	cleanup_ns "${NS_LIST[@]}"
}

# setup netns with given names as prefix. e.g
# setup_ns local remote
setup_ns()
{
	local ns_name=""
	local ns_list=()
	for ns_name in "$@"; do
		# avoid conflicts with local var: internal error
		if [ "${ns_name}" = "ns_name" ]; then
			echo "Failed to setup namespace '${ns_name}': invalid name"
			cleanup_ns "${ns_list[@]}"
			exit $ksft_fail
		fi

		# Some test may setup/remove same netns multi times
		if [ -z "${!ns_name}" ]; then
			eval "${ns_name}=${ns_name,,}-$(mktemp -u XXXXXX)"
		else
			cleanup_ns "${!ns_name}"
		fi

		if ! ip netns add "${!ns_name}"; then
			echo "Failed to create namespace $ns_name"
			cleanup_ns "${ns_list[@]}"
			return $ksft_skip
		fi
		ip -n "${!ns_name}" link set lo up
		ns_list+=("${!ns_name}")
	done
	NS_LIST+=("${ns_list[@]}")
}

tc_rule_stats_get()
{
	local dev=$1; shift
	local pref=$1; shift
	local dir=${1:-ingress}; shift
	local selector=${1:-.packets}; shift

	tc -j -s filter show dev $dev $dir pref $pref \
	    | jq ".[1].options.actions[].stats$selector"
}

tc_rule_handle_stats_get()
{
	local id=$1; shift
	local handle=$1; shift
	local selector=${1:-.packets}; shift
	local netns=${1:-""}; shift

	tc $netns -j -s filter show $id \
	    | jq ".[] | select(.options.handle == $handle) | \
		  .options.actions[0].stats$selector"
}

ret_set_ksft_status()
{
	local ksft_status=$1; shift
	local msg=$1; shift

	RET=$(ksft_status_merge $RET $ksft_status)
	if (( $? )); then
		retmsg=$msg
	fi
}

log_test_result()
{
	local test_name=$1; shift
	local opt_str=$1; shift
	local result=$1; shift
	local retmsg=$1

	printf "TEST: %-60s  [%s]\n" "$test_name $opt_str" "$result"
	if [[ $retmsg ]]; then
		printf "\t%s\n" "$retmsg"
	fi
}

pause_on_fail()
{
	if [[ $PAUSE_ON_FAIL == yes ]]; then
		echo "Hit enter to continue, 'q' to quit"
		read a
		[[ $a == q ]] && exit 1
	fi
}

handle_test_result_pass()
{
	local test_name=$1; shift
	local opt_str=$1; shift

	log_test_result "$test_name" "$opt_str" " OK "
}

handle_test_result_fail()
{
	local test_name=$1; shift
	local opt_str=$1; shift

	log_test_result "$test_name" "$opt_str" FAIL "$retmsg"
	pause_on_fail
}

handle_test_result_xfail()
{
	local test_name=$1; shift
	local opt_str=$1; shift

	log_test_result "$test_name" "$opt_str" XFAIL "$retmsg"
	pause_on_fail
}

handle_test_result_skip()
{
	local test_name=$1; shift
	local opt_str=$1; shift

	log_test_result "$test_name" "$opt_str" SKIP "$retmsg"
}

log_test()
{
	local test_name=$1
	local opt_str=$2

	if [[ $# -eq 2 ]]; then
		opt_str="($opt_str)"
	fi

	if ((RET == ksft_pass)); then
		handle_test_result_pass "$test_name" "$opt_str"
	elif ((RET == ksft_xfail)); then
		handle_test_result_xfail "$test_name" "$opt_str"
	elif ((RET == ksft_skip)); then
		handle_test_result_skip "$test_name" "$opt_str"
	else
		handle_test_result_fail "$test_name" "$opt_str"
	fi

	EXIT_STATUS=$(ksft_exit_status_merge $EXIT_STATUS $RET)
	return $RET
}

log_test_skip()
{
	RET=$ksft_skip retmsg= log_test "$@"
}

log_test_xfail()
{
	RET=$ksft_xfail retmsg= log_test "$@"
}

log_info()
{
	local msg=$1

	echo "INFO: $msg"
}
