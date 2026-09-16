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
# The harness checks the environment once up front and fails the run if the
# suite cannot execute, rather than letting every test skip and exit 0.

set -u

cd "$(dirname "$0")" || exit 1

# Sourcing the harness runs its preflight, which decides which architecture
# this run is for -- so the test list cannot be built before it has, and the
# tests inherit the answers rather than working them out again.
. ./lib.sh

dirs=( generic )
[ -d "$KLP_TEST_ARCH" ] && dirs+=( "$KLP_TEST_ARCH" )

tests=()
for d in "${dirs[@]}"; do
	for t in "$d"/test-*.sh; do
		[ -f "$t" ] && tests+=( "$t" )
	done
done
[ "${#tests[@]}" -gt 0 ] || { echo "1..0 # SKIP no tests found"; exit 0; }

# Tests for another architecture are absent from this run entirely.  Say how
# many, so a run which covers less than the tree holds does not look like one
# that covers all of it.
for d in */; do
	d="${d%/}"
	case "$d" in generic|"$KLP_TEST_ARCH") continue ;; esac
	n=$(ls "$d"/test-*.sh 2>/dev/null | wc -l)
	[ "$n" -gt 0 ] || continue
	echo "# not run: $n test$( [ "$n" = 1 ] || echo s ) in $d/" \
	     "(this run is $KLP_TEST_ARCH)"
done

echo "1..${#tests[@]}"

rc=0
for t in "${tests[@]}"; do
	./"$t" || rc=1
done

exit $rc
