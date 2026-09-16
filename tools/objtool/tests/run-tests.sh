#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Run the objtool klp tests.  Each test-*.sh prints one TAP result line.
#
# Tests live in generic/ and in a directory per architecture.  A run executes
# generic/ plus the one matching this architecture, so a test which cannot
# apply here is not run rather than reporting a skip; what was left out is
# reported once, as a comment, so differing coverage is still visible.
#
# A run covers one compiler and one architecture; CI runs the combinations.
# The harness checks the environment once up front and fails the run if the
# suite cannot execute, rather than letting every test skip and exit 0.

set -u

# Determinism: the order test-*.sh expands in, and how grep's character ranges
# and sort's collation behave inside the tests, are all locale-dependent.  A
# suite whose results depend on the invoking shell's locale is a suite whose
# failures cannot be reproduced.
export LC_ALL=C

usage()
{
	cat <<EOF
usage: $(basename "$0") [-k|--keep] [test...]

Run the objtool klp tests for this architecture: everything in generic/, plus
everything in the directory named for it.  With no arguments, runs all of them.
A test may be named with or without its "test-" prefix and ".sh" suffix, and is
looked for in both directories.

Options:
    -k, --keep    do not delete each test's working directory; print its path,
                  so the objects a failing test built can be looked at

Environment:
    OBJTOOL       objtool binary to test (default ../objtool)
    CC            compiler used to build fixtures (default gcc)
    ARCH          architecture the tests are for (default: uname -m)

A test which needs something of its own says so in its skip message.
EOF
	exit "${1:-0}"
}

cd "$(dirname "$0")" || exit 1

while [ $# -gt 0 ]; do
	case "$1" in
	-h|--help)	usage ;;
	-k|--keep)	export KLP_TEST_KEEP=1; shift ;;
	--)		shift; break ;;
	-*)		echo "unknown option: $1" >&2; usage 1 ;;
	*)		break ;;
	esac
done

echo "TAP version 13"

# Sourcing the harness runs its preflight, which decides which architecture
# this run is for -- so the test list cannot be built before it has, and the
# tests inherit the answers rather than working them out again.
. ./lib.sh

dirs=( generic )
[ -d "$KLP_TEST_ARCH" ] && dirs+=( "$KLP_TEST_ARCH" )

if [ $# -gt 0 ]; then
	tests=()
	for arg in "$@"; do
		name="test-${arg#test-}"; name="${name%.sh}.sh"
		found=
		for d in "${dirs[@]}"; do
			[ -f "$d/$name" ] || continue
			[ -x "$d/$name" ] ||
				{ echo "not executable: $d/$name" >&2; exit 1; }
			tests+=( "$d/$name" ); found=y
		done
		[ -n "$found" ] ||
			{ echo "no such test for $KLP_TEST_ARCH: $arg" >&2; exit 1; }
	done
else
	tests=()
	for d in "${dirs[@]}"; do
		for t in "$d"/test-*.sh; do
			[ -f "$t" ] && tests+=( "$t" )
		done
	done
	[ "${#tests[@]}" -gt 0 ] ||
		{ echo "1..0 # SKIP no tests found"; exit 0; }

	# Tests for another architecture are absent from this run entirely.  Say
	# how many, so a run which covers less than the tree holds does not look
	# like one that covers all of it.
	for d in */; do
		d="${d%/}"
		case "$d" in generic|"$KLP_TEST_ARCH") continue ;; esac
		n=$(ls "$d"/test-*.sh 2>/dev/null | wc -l)
		[ "$n" -gt 0 ] || continue
		echo "# not run: $n test$( [ "$n" = 1 ] || echo s ) in $d/" \
		     "(this run is $KLP_TEST_ARCH)"
	done
fi

# One directory for the whole run, one per test inside it, mirroring the
# source layout.  A run then leaves a single thing behind instead of 39
# scattered among everything else using mktemp.
rundir="$(mktemp -d "${TMPDIR:-/tmp}/klp-tests.XXXXXXXX")" ||
	{ echo "Bail out! cannot create a working directory" >&2; exit 1; }

echo "1..${#tests[@]}"

pass=0 fail=0 static_skip=0 probe_skip=0 xfail=0 xpass=0

for t in "${tests[@]}"; do
	out="$(KLP_TEST_WORKDIR="$rundir/${t%.sh}" ./"$t" 2>&1)"
	rc=$?

	# A test prints one result line, but it is not necessarily the only
	# thing it prints: objtool warns on stderr, and the runner captures
	# that.  Classify the result line itself rather than the whole of the
	# output, or a stray line ahead of it makes every pattern below miss and
	# the exit status decide -- which would count an expected failure, which
	# exits 0, as a pass.
	result="$(printf '%s\n' "$out" | grep -E '^(ok|not ok)' | tail -1)"
	rest="$(printf '%s\n' "$out" | grep -Ev '^(ok|not ok)')"

	# Classify from the result line, not the exit status: a skip and a pass
	# both exit 0, and telling them apart is the point of counting.
	#
	# The two skip kinds differ in what they promise.  A static skip was
	# declared before the test ran ("clang does not do this"), so it is
	# expected indefinitely.  A probe skip means the construct did not turn
	# up this time, which is weaker and worth watching: one that becomes
	# permanent is a fixture that quietly stopped testing anything.
	case "$result" in
	*"# SKIP (declared)"*)	static_skip=$((static_skip + 1)) ;;
	*"# SKIP (probe)"*)	probe_skip=$((probe_skip + 1)) ;;
	*"# SKIP"*)
		# An undeclared skip: the test gave up for a reason it never
		# said it might.  That is a hole, not an expected outcome.
		#
		# Replace the line rather than adding one.  Every test owes the
		# plan exactly one result, and a consumer counting them is
		# entitled to say so when the totals disagree.
		rest="$rest${rest:+$'\n'}was: $result"
		result="not ok - $(basename "$t" .sh): undeclared skip"
		result="$result (use gcc_only/clang_only or require_input_*)"
		fail=$((fail + 1)) ;;
	"not ok"*"# TODO"*)	xfail=$((xfail + 1)) ;;
	"ok"*"# TODO"*)		xpass=$((xpass + 1)) ;;
	"not ok"*)		fail=$((fail + 1)) ;;
	"ok"*)			pass=$((pass + 1)) ;;
	*)
		# No result line at all: the test died before reporting.
		rest="$rest${rest:+$'\n'}exited $rc without a result line"
		result="not ok - $(basename "$t" .sh): no TAP result"
		fail=$((fail + 1)) ;;
	esac

	echo "$result"
	[ -n "$rest" ] && printf '%s\n' "$rest" | sed 's/^[^#]/# &/'

done

echo "# pass:$pass fail:$fail static-skip:$static_skip" \
     "probe-skip:$probe_skip xfail:$xfail xpass:$xpass"

# A failure is the one time the objects matter, and by default they are
# already gone.  Say so then rather than in the usage text nobody reads while
# something is broken.
if [ -n "${KLP_TEST_KEEP:-}" ]; then
	echo "# working directories kept in $rundir -- inspect, then rm -rf it"
elif ! rmdir "$rundir"/*/ "$rundir" 2>/dev/null; then
	echo "# $rundir was not empty; a test did not clean up after itself"
elif [ "$fail" != 0 ] || [ "$xpass" != 0 ]; then
	echo "# re-run with --keep to hold on to what a failing test built"
fi

[ "$fail" = 0 ] && [ "$xpass" = 0 ]
