#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Run the objtool klp tests.  Each test-*.sh prints one TAP result line.
#
# The harness checks the environment once up front and fails the run if the
# suite cannot execute, rather than letting every test skip and exit 0.

set -u

cd "$(dirname "$0")" || exit 1

tests=( test-*.sh )
[ "${tests[0]}" = "test-*.sh" ] && { echo "1..0 # SKIP no tests found"; exit 0; }

# Sourcing the harness runs its preflight, and exports what it found so the
# tests inherit it rather than working it out again.
. ./lib.sh

echo "1..${#tests[@]}"

rc=0
for t in "${tests[@]}"; do
	./"$t" || rc=1
done

exit $rc
