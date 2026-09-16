#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Run the objtool klp tests.  Each test-*.sh prints one TAP result line.

set -u

cd "$(dirname "$0")" || exit 1

tests=( test-*.sh )
[ "${tests[0]}" = "test-*.sh" ] && { echo "1..0 # SKIP no tests found"; exit 0; }

echo "1..${#tests[@]}"

rc=0
for t in "${tests[@]}"; do
	./"$t" || rc=1
done

exit $rc
