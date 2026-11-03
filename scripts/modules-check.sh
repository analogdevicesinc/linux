#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

set -e

if [ $# != 1 ]; then
	echo "Usage: $0 <modules.order>" >& 2
	exit 1
fi

exit_code=0

# Check uniqueness of module names
check_same_name_modules()
{
	for m in $(sed 's:.*/::; s/-/_/g' "$1" | sort | uniq -d)
	do
		echo "error: the following would cause module name conflict:" >&2
		awk -v name="$m" '
			{
				basename = $0
				sub(/^.*\//, "", basename)
				gsub(/-/, "_", basename)
				if (basename == name) {
					sub(/\.o$/, ".ko")
					print "  " $0
				}
			}' "$1" >&2
		exit_code=1
	done
}

check_same_name_modules "$1"

exit $exit_code
