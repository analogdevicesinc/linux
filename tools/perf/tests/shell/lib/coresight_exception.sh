# SPDX-License-Identifier: GPL-2.0
# Common helpers for CoreSight exception entry and return tests.

cleanup()
{
	rm -rf "$tmpdir"
}

record_trace_with_cpu_clock()
{
	local cf="$tmpdir/ctl"
	local af="$tmpdir/ack"
	local clock_opts=()

	record_freq=$1
	shift

	# A zero frequency omits the cpu-clock event.
	if [ "$record_freq" -gt 0 ]; then
		# Generate timer IRQs without delivering a signal to the workload.
		clock_opts=(-e cpu-clock:u -F "$record_freq")
	fi

	# FIFO control bounds the trace to the workload.
	# --kcore supplies the running kernel's instructions for decoding.
	taskset -c "$cpu" perf record -B --no-bpf-event --per-thread --kcore \
		-e cs_etm/timestamp=0/uk "${clock_opts[@]}" \
		-m,4M -D -1 --control "fifo:$cf,$af" \
		-o "$tmpdir/data" -- \
		perf test --record-ctl "fifo:$cf,$af" -w "$@" \
		> "$tmpdir/record.log" 2>&1
}

record_trace()
{
	record_trace_with_cpu_clock 0 "$@"
}

decode_trace()
{
	local sw_fields=()

	# perf script rejects -F sw: when no software event was recorded.
	if [ "$record_freq" -gt 0 ]; then
		sw_fields=(-F sw:)
	fi

	if ! perf script -i "$tmpdir/data" --itrace=b \
		-F hw:ip,addr,flags,insn "${sw_fields[@]}" \
		> "$tmpdir/script" 2> "$tmpdir/script.log"; then
		cat "$tmpdir/script.log"
		echo "Failed to decode the exception trace"
		return 1
	fi
}

# Arguments: entry pattern, exit pattern, expected return PC - entry PC.
# Return 2 if no complete pair survives the trace gaps.
check_exception_pairs()
{
	local difference=$3 entry_pc resume_pc pairs=0

	awk -v entry_pattern="$1" -v exit_pattern="$2" '
	function address(pc) {
		sub(/^0[xX]/, "", pc)
		sub(/^0+/, "", pc)
		return "0x" (pc == "" ? "0" : tolower(pc))
	}
	BEGIN {
		entry_pattern = "^[[:space:]]*(" entry_pattern ")([[:space:]]|$)"
		exit_pattern = "^[[:space:]]*(" exit_pattern ")([[:space:]]|$)"
	}
	NF {
		# A trace boundary invalidates all pending entries, including nested ones.
		if ($1 == "tr" && ($2 == "strt" || $2 == "end")) {
			depth = 0
			next
		}

		is_entry = ($1 == "hw" && $2 == "int") || $1 == "int" || $1 == "syscall"
		is_exit = $1 == "iret" || $1 == "sysret"
		if (!is_entry && !is_exit)
			next

		# Branch fields are: flags source => destination [insn: bytes].
		arrow = 0
		for (i = 1; i <= NF; i++) {
			if ($i == "=>") {
				arrow = i
				break
			}
		}
		if (arrow < 3 || arrow == NF) {
			print "FAIL: Unexpected perf script output: " $0 > "/dev/stderr"
			errors++
			depth = 0
			next
		}

		# Track every exception so an unrelated nested IRET cannot close
		# a selected outer entry. Match the requested flags at each depth.
		if (is_entry) {
			entry_pc[++depth] = address($(arrow - 1))
			selected[depth] = $0 ~ entry_pattern
			next
		}
		if (!depth)
			next

		if (selected[depth] && $0 ~ exit_pattern)
			print entry_pc[depth], address($(arrow + 1))
		depth--
	}
	END {
		if (errors)
			exit 1
	}' "$tmpdir/script" > "$tmpdir/exception_pairs" || return 1

	# Bash uses integer arithmetic, preserving all bits of 64-bit PCs.
	while read -r entry_pc resume_pc; do
		if [ "$((resume_pc - entry_pc))" -ne "$difference" ]; then
			printf "FAIL: Entry %s returns to %s, expected PC difference %d\n" \
				"$entry_pc" "$resume_pc" "$difference"
			return 1
		fi
		pairs=$((pairs + 1))
	done < "$tmpdir/exception_pairs"

	if [ "$pairs" -eq 0 ]; then
		echo "No complete exception entry/exit pair"
		return 2
	fi
	printf "Checked %d exception pairs with PC difference %d\n" "$pairs" "$difference"
}

setup_exception_test()
{
	[ "$(uname -m)" = aarch64 ] || return 2
	perf check feature -q libopencsd || return 2
	[ -d /sys/bus/event_source/devices/cs_etm ] || return 2

	export LC_ALL=C
	command -v taskset >/dev/null 2>&1 || return 2

	tmpdir=$(mktemp -d /tmp/perf-cs-etm-exception.XXXXXX) || return 1
	trap cleanup EXIT
	trap 'exit 1' HUP INT TERM
	mkfifo "$tmpdir/ctl" "$tmpdir/ack" || return 1

	# Keep each exception entry and return in the same trace queue.
	cpu=$(awk '/Cpus_allowed_list:/ {
		split($2, cpus, /[-,]/)
		print cpus[1]
	}' /proc/self/status)

	# Check permissions, FIFO control and the trace sink before testing.
	if ! record_trace callchain; then
		cat "$tmpdir/record.log"
		echo "[Skip] Cannot record user/kernel CoreSight trace with --kcore"
		return 2
	fi

	return 0
}
