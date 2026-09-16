# SPDX-License-Identifier: GPL-2.0
#
# Helpers for the objtool klp tests.  A test builds a fixture twice, as the
# original and (with -DPATCHED) the patched object, runs both through
# "klp checksum" and diffs them, then asserts on the result.
#
# Assertions check properties rather than compare against recorded output:
# codegen varies between compilers and golden files would report churn instead
# of regressions.

TESTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURES_DIR="$TESTS_DIR/fixtures"

OBJTOOL="${OBJTOOL:-$TESTS_DIR/../objtool}"
CC="${CC:-gcc}"

# klp-build compiles the kernel this way; klp diff needs per-symbol sections to
# extract individual functions.
FIXTURE_CFLAGS="-c -O2 -ffunction-sections -fdata-sections -fno-asynchronous-unwind-tables"

test_name="$(basename "$0" .sh)"
workdir=

pass() { echo "ok - $test_name${1:+: $1}"; exit 0; }
fail() { echo "not ok - $test_name: $1" >&2; exit 1; }
skip() { echo "ok - $test_name # SKIP $1"; exit 0; }

cleanup() { [ -n "$workdir" ] && rm -rf "$workdir"; }

# setup [exported symbol...]
setup()
{
	# A relative $OBJTOOL is relative to the objtool directory, not to the
	# tests which run from tests/.
	[ -x "$OBJTOOL" ] || [ ! -x "$TESTS_DIR/../$OBJTOOL" ] ||
		OBJTOOL="$TESTS_DIR/../$OBJTOOL"

	# Not finding objtool is a broken invocation, not an environment which
	# cannot run the test.  Skipping here would read as a pass.
	[ -x "$OBJTOOL" ] || fail "objtool not found at '$OBJTOOL', build it first"

	# run_diff() runs objtool from inside the test's working directory, so
	# a relative path would resolve against that instead.
	OBJTOOL="$(realpath "$OBJTOOL")"

	"$OBJTOOL" klp 2>&1 | grep -q checksum ||
		skip "objtool built without klp support (needs libxxhash)"
	command -v "${CC%% *}" >/dev/null || skip "no compiler ($CC)"

	workdir="$(mktemp -d)" || fail "mktemp failed"
	trap cleanup EXIT

	export_syms "$@"
}

# export_syms [symbol...]
#
# Rewrite Module.symvers so exactly these symbols are exported by vmlinux.
# Whether a symbol is listed decides between an ordinary relocation and a klp
# relocation, so tests flip it to cover both.
export_syms()
{
	: > "$workdir/Module.symvers"
	for sym in "$@"; do
		printf '0x00000000\t%s\tvmlinux\tEXPORT_SYMBOL\t\n' \
			"$sym" >> "$workdir/Module.symvers"
	done
}

# build_pair <fixture.c> [cflags...]
build_pair()
{
	local fixture="$FIXTURES_DIR/$1"; shift

	[ -f "$fixture" ] || fail "missing fixture $fixture"

	$CC $FIXTURE_CFLAGS "$@" -o "$workdir/orig.o" "$fixture" 2>"$workdir/cc.log" ||
		skip "fixture does not build here: $(tail -1 "$workdir/cc.log")"
	$CC $FIXTURE_CFLAGS "$@" -DPATCHED -o "$workdir/patched.o" "$fixture" 2>"$workdir/cc.log" ||
		skip "fixture does not build here: $(tail -1 "$workdir/cc.log")"
}

# run_diff [expected exit status]
run_diff()
{
	local expect="${1:-0}" rc=0

	# Checksums live in the objects, so only generate them once even when a
	# test diffs the same pair again with a different Module.symvers.
	if [ ! -e "$workdir/.checksummed" ]; then
		"$OBJTOOL" klp checksum "$workdir/orig.o" ||
			fail "klp checksum orig.o failed"
		"$OBJTOOL" klp checksum "$workdir/patched.o" ||
			fail "klp checksum patched.o failed"
		touch "$workdir/.checksummed"
	fi

	# klp diff looks for Module.symvers relative to the working directory.
	( cd "$workdir" && "$OBJTOOL" klp diff orig.o patched.o out.o ) \
		> "$workdir/diff.log" 2>&1 || rc=$?

	[ "$rc" = "$expect" ] ||
		fail "klp diff exited $rc, expected $expect: $(tail -2 "$workdir/diff.log")"
}

cc_supports()
{
	echo 'int f(void) { return 0; }' > "$workdir/flagtest.c"
	$CC $1 -c "$workdir/flagtest.c" -o "$workdir/flagtest.o" 2>/dev/null
}

# partial_link <output> <object...>
#
# "ld -r" through the compiler driver so the link targets the same
# architecture as the objects.
partial_link()
{
	local out="$1"; shift

	$CC -r -nostdlib -o "$out" "$@" 2>/dev/null ||
		$CC -r -nostdlib -fuse-ld=lld -o "$out" "$@" 2>/dev/null
}

# find_thinlto_toolchain
#
# Set $THIN_CC and $THIN_LD to a clang and lld from the same LLVM release.  A
# mismatched pair fails with "Invalid summary version", which reads like a
# broken test rather than a broken environment.
find_thinlto_toolchain()
{
	local cc ld ver

	for cc in "${THIN_CC:-}" "$CC" clang; do
		[ -n "$cc" ] || continue
		command -v "${cc%% *}" >/dev/null 2>&1 || continue

		ver=$($cc -dumpversion 2>/dev/null | cut -d. -f1)

		for ld in "${THIN_LD:-}" "ld.lld-$ver" ld.lld; do
			[ -n "$ld" ] || continue
			command -v "$ld" >/dev/null 2>&1 || continue

			echo 'int probe(void) { return 0; }' > "$workdir/probe.c"
			$cc -flto=thin -O2 -c "$workdir/probe.c" \
				-o "$workdir/probe.o" 2>/dev/null || continue
			"$ld" -r "$workdir/probe.o" -o "$workdir/probe.elf" \
				2>/dev/null || continue

			THIN_CC="$cc"
			THIN_LD="$ld"
			return 0
		done
	done

	return 1
}

out_sections() { readelf -S -W "$workdir/out.o" 2>/dev/null; }
out_relocs()   { readelf -r -W "$workdir/out.o" 2>/dev/null; }
out_symbols()  { readelf -s -W "$workdir/out.o" 2>/dev/null; }
diff_log()     { cat "$workdir/diff.log"; }

assert_section()
{
	out_sections | grep -q "[[:space:]]$1[[:space:]]" ||
		fail "expected section '$1' in output"
}

assert_patched()
{
	assert_section ".text.$1"
}

assert_not_patched()
{
	out_sections | grep -q "[[:space:]].text.$1[[:space:]]" &&
		fail "function '$1' should not have been cloned"
	return 0
}
