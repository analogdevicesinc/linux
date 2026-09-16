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

# Tests live in generic/ or in an architecture directory beside it, and each
# carries its own fixtures.
FIXTURES_DIR="$(cd "$(dirname "$0")/fixtures" 2>/dev/null && pwd)"

# The kernel's convention: CROSS_COMPILE is the one knob, with per-tool
# overrides for what it does not cover.  objtool itself is always a host binary
# -- it is built with HOSTCC and only reads ELF -- so an arm64 machine can run
# the x86 tests against x86 objects given a compiler that emits them.
#
# readelf reads any target, so it rarely needs overriding, and either GNU
# readelf or llvm-readelf will do: the assertions match on fields rather than
# on columns, and where the two spell something differently -- "OS [0xff20]"
# against "OS[0xff20]" for SHN_LIVEPATCH -- they accept both.  BFD's objcopy is
# usually built for the host's target alone, and llvm-objcopy is the
# target-agnostic replacement.
CROSS_COMPILE="${CROSS_COMPILE:-}"
CC="${CC:-${CROSS_COMPILE}gcc}"
LD="${LD:-${CROSS_COMPILE}ld}"
READELF="${READELF:-${CROSS_COMPILE}readelf}"
OBJCOPY="${OBJCOPY:-${CROSS_COMPILE}objcopy}"

OBJTOOL="${OBJTOOL:-$TESTS_DIR/../objtool}"

# klp_preflight
#
# Check the environment once, before any test runs, and report what was found.
#
klp_preflight()
{
	local tmp tool cc_version arch host cc_arch

	bail() { echo "Bail out! $*" >&2; exit 1; }

	# A relative $OBJTOOL is relative to the objtool directory, not tests/.
	[ -x "$OBJTOOL" ] || [ ! -x "$TESTS_DIR/../$OBJTOOL" ] ||
		OBJTOOL="$TESTS_DIR/../$OBJTOOL"

	[ -x "$OBJTOOL" ] ||
		bail "objtool not found at '$OBJTOOL' -- build it first"

	# run_diff() runs objtool from inside the test's working directory, so
	# a relative path would resolve against that instead.
	OBJTOOL="$(realpath "$OBJTOOL")"

	"$OBJTOOL" klp 2>&1 | grep -q checksum ||
		bail "objtool was built without klp support; install libxxhash (>= 0.8) and rebuild"

	command -v "${CC%% *}" >/dev/null || bail "compiler not found: $CC"

	for tool in "$READELF" "$OBJCOPY" "$LD"; do
		command -v "${tool%% *}" >/dev/null || bail "$tool not found"
	done

	tmp="$(mktemp -d)" || bail "mktemp failed"
	echo 'int probe(void) { return 0; }' > "$tmp/probe.c"
	$CC -c -o "$tmp/probe.o" "$tmp/probe.c" 2>/dev/null ||
		{ rm -rf "$tmp"; bail "$CC cannot compile a trivial object"; }

	# $CC, $ARCH and objtool have to agree about the target, and cross runs
	# are where they stop agreeing: plain "CC=clang ARCH=x86_64" on an arm64
	# box selects the x86 tests and then builds arm64 objects, because clang
	# needs --target= to emit anything but the host's.
	#
	# Ask objtool rather than comparing machine names.  It rejects an object
	# it was not built for -- "unexpected ELF machine type" -- so one check
	# covers every way the three can disagree, and says so once instead of
	# failing every test for the same reason.
	"$OBJTOOL" klp checksum "$tmp/probe.o" >/dev/null 2>&1 ||
		{ rm -rf "$tmp"
		  bail "objtool rejects an object built by '$CC'; they target" \
		       "different architectures (set CROSS_COMPILE, or" \
		       "--target= for clang)"; }

	# BFD objcopy is usually built for the host's target alone, and
	# checksum_of() needs it to read the object under test.
	$OBJCOPY -O binary --only-section=.text "$tmp/probe.o" "$tmp/probe.bin" 2>/dev/null ||
		{ rm -rf "$tmp"
		  bail "$OBJCOPY cannot read objects built by '$CC'; install" \
		       "binutils-multiarch or set OBJCOPY=llvm-objcopy"; }
	# $ARCH only chooses which directory of tests runs, so it can disagree
	# with what $CC builds without objtool noticing -- and the result is the
	# wrong set of tests, quietly.
	case "$($READELF -hW "$tmp/probe.o" | sed -n 's/.*Machine: *//p')" in
	*X86-64*|*Intel*80386*)	cc_arch=x86 ;;
	*AArch64*)		cc_arch=arm64 ;;
	*)			cc_arch= ;;
	esac
	rm -rf "$tmp"

	# Normalize to the kernel's SRCARCH.
	case "${ARCH:-$(uname -m)}" in
	x86_64|i?86)	arch=x86 ;;
	aarch64*)	arch=arm64 ;;
	*)		arch="${ARCH:-$(uname -m)}" ;;
	esac

	case "$(uname -m)" in
	x86_64|i?86)	host=x86 ;;
	aarch64*)	host=arm64 ;;
	*)		host="$(uname -m)" ;;
	esac

	[ -z "$cc_arch" ] || [ "$cc_arch" = "$arch" ] ||
		bail "ARCH says $arch but '$CC' builds $cc_arch objects;" \
		     "the $arch tests would run against the wrong architecture"

	KLP_TEST_ARCH="$arch"
	KLP_TEST_PREFLIGHT=done
	export OBJTOOL CC KLP_TEST_ARCH KLP_TEST_PREFLIGHT

	cc_version="$($CC --version 2>/dev/null | head -1)"
	cat <<EOF
# preflight
#   objtool   $OBJTOOL (klp: yes)
#   compiler  $cc_version
#   arch      $KLP_TEST_ARCH$([ "$arch" = "$host" ] || echo "  (host $host, cross)")
EOF
}

[ -n "${KLP_TEST_PREFLIGHT:-}" ] || klp_preflight

# What every fixture is built with.  These describe the kernel a fixture stands
# in for; -c is build_one's contract rather than a property of that kernel, so
# it lives at the compile where an override cannot drop it.
#
#   -O2					the kernel's default
#   -ffunction-sections -fdata-sections	klp-build passes these itself, through
#					KCFLAGS, whatever the configuration
#   -fno-asynchronous-unwind-tables	arch/x86/Makefile sets this always, so
#					kernel objects carry no .eh_frame
#   -fno-common				the kernel's Makefile sets it, so an
#					uninitialised global there lands in
#					.bss rather than being SHN_COMMON,
#					which has no section and so no
#					checksum
#
# A test overrides it; see tools/objtool/Documentation/klp-write-tests.txt.
FIXTURE_CFLAGS="-O2 -ffunction-sections -fdata-sections -fno-common \
		-fno-asynchronous-unwind-tables"

test_name="$(basename "$0" .sh)"
workdir=

# The pair run_diff() and the checksum helpers work on.  build_pair() names
# them again and make_vmlinux_pair() repoints orig_obj at the image it links,
# but a test which builds its objects itself with build_one() sets neither, so
# the default belongs here.
orig_obj=orig.o
patched_obj=patched.o

pass() { echo "ok - $test_name${1:+: $1}"; exit 0; }
fail() { echo "not ok - $test_name: $1"; exit 1; }

# Two kinds of skip, and the runner tells them apart.
#
#   declared_skip  the test said in advance it does not apply here, e.g.
#                  gcc_only on a clang run.  Expected indefinitely.
#   probe_skip     the construct did not turn up in the built object this
#                  time.  Weaker: it may appear on another compiler version,
#                  and one which becomes permanent is a fixture that quietly
#                  stopped testing anything.
#
# A bare skip() is neither, and the runner counts it as a failure: a test which
# gives up for a reason it never declared is a hole, not an outcome.
declared_skip() { echo "ok - $test_name # SKIP (declared) $*"; exit 0; }
probe_skip()    { echo "ok - $test_name # SKIP (probe) $*"; exit 0; }
skip()          { echo "ok - $test_name # SKIP $*"; exit 0; }

# TAP directives.  A test which is known to fail reports it rather than being
# commented out and forgotten, and one which starts passing again says so
# instead of quietly going green: the expectation has to be removed by hand,
# which is the point.
xfail()
{
	echo "not ok - $test_name${1:+: $1} # TODO known failure"
	exit 0
}

xpass()
{
	echo "ok - $test_name${1:+: $1} # TODO expected failure, but passed"
	exit 1
}

cleanup() { [ -n "$workdir" ] && rm -rf "$workdir"; }

# setup [exported symbol...]
setup()
{
	# The environment was checked once when this file was sourced, so there
	# is nothing to verify here: objtool exists at the resolved path, has
	# klp support, and $CC works.
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

# gcc_only / clang_only <reason>
gcc_only()
{
	case "$($CC --version 2>/dev/null | head -1)" in
	*[Gg][Cc][Cc]*)	return 0 ;;
	esac
	declared_skip "gcc only${1:+: $1}"
}

clang_only()
{
	case "$($CC --version 2>/dev/null | head -1)" in
	*clang*)	return 0 ;;
	esac
	declared_skip "clang only${1:+: $1}"
}

# build_one <fixture.c> <output object> [cflags...]
build_one()
{
	local fixture out
	fixture="$FIXTURES_DIR/$1"
	out="$workdir/$2"
	shift 2

	[ -f "$fixture" ] || fail "missing fixture $fixture"

	# run_checksum only runs once per workdir.  A fresh object has no
	# checksums in it, so anything built now needs that to happen again.
	rm -f "$workdir/.checksummed"

	$CC -c $FIXTURE_CFLAGS "$@" -o "$out" "$fixture" 2>"$workdir/cc.log" ||
		fail "$(basename "$fixture") does not build: $(tail -1 "$workdir/cc.log")"
}

# build_pair <fixture.c> [cflags...]
build_pair()
{
	local fixture="$1"; shift

	# Name what this builds.  A test may run several segments, and
	# make_vmlinux_pair() repoints orig_obj at the image it links, so
	# without this the next run_diff() would still be reading that.
	orig_obj=orig.o
	patched_obj=patched.o

	build_one "$fixture" orig.o "$@"
	build_one "$fixture" patched.o "$@" -DPATCHED
}

# run_objtool_check <objtool arguments...>
#
# Run objtool's ordinary check pass over the pair, as the kernel build does.
#
# Some of what klp diff consumes is produced by this pass rather than by the
# compiler: .static_call_sites, .mcount_loc, .ibt_endbr_seal, ORC.
#
# Only module objects see it before klp-build -- with CONFIG_KLP_BUILD the
# per-object pass is deferred, so built-in objects reach klp diff exactly as
# the compiler left them.
run_objtool_check()
{
	local obj

	# This rewrites both objects, so checksums taken before it describe
	# something that no longer exists.  As in build_one(), drop the marker
	# so run_checksum() takes them again.
	rm -f "$workdir/.checksummed"

	for obj in "$orig_obj" "$patched_obj"; do
		"$OBJTOOL" "$@" "$workdir/$obj" ||
			fail "objtool $* failed on $obj"
	done
}

run_checksum()
{
	# Checksums live in the objects, and a test may ask for them more than
	# once -- diffing the same pair again with a different Module.symvers,
	# say.  objtool does the right thing when asked twice, leaving the
	# object alone, but it says so, and that warning would be most of what
	# a passing run prints.  Remember instead, and keep quiet.
	[ -e "$workdir/.checksummed" ] && return 0

	"$OBJTOOL" klp checksum "$workdir/$orig_obj" ||
		fail "klp checksum $orig_obj failed"
	"$OBJTOOL" klp checksum "$workdir/$patched_obj" ||
		fail "klp checksum $patched_obj failed"
	touch "$workdir/.checksummed"
}

# run_diff [expected exit status]
run_diff()
{
	local expect="${1:-0}" rc=0

	run_checksum

	# klp diff looks for Module.symvers relative to the working directory.
	( cd "$workdir" && "$OBJTOOL" klp diff "$orig_obj" "$patched_obj" out.o ) \
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

	rm -f "$workdir/.checksummed"

	$CC -r -nostdlib -o "$out" "$@" 2>/dev/null ||
		$CC -r -nostdlib -fuse-ld=lld -o "$out" "$@" 2>/dev/null
}

# link_vmlinux <output> <object...>
#
# Link objects into an executable, the way the kernel's final link produces
# vmlinux from vmlinux.o.  Entry point 0 and no libc: nothing runs it, it only
# has to be a linked image with resolved addresses.
#
# The sub-sections have to come out in name order rather than object order,
# the way the kernel's linker script gathers .text.unlikely and .data.. apart
# from the rest.  That reordering is the entire reason .klp.symid exists: a
# link which preserves order cannot tell a correct sympos from one that merely
# counted, and the caller checks the two orders really did diverge.
#
# A linker script rather than --sort-section=name, because lld accepts that
# option and ignores it -- so on a host where only lld can link the target, the
# test would quietly stop testing the thing it is named for.
#
# Three attempts because a cross run has neither $LD nor the compiler's default
# linker able to touch the target: on an arm64 host linking x86 objects, only
# lld will do it.
link_vmlinux()
{
	local out="$1" lds="$workdir/sort.lds"; shift

	echo 'SECTIONS { .data : { *(SORT_BY_NAME(.data.*)) } }' > "$lds"

	$LD -e 0 -T "$lds" -o "$out" "$@" 2>/dev/null ||
		$CC -nostdlib -Wl,-e,0 -Wl,-T,"$lds" \
			-o "$out" "$@" 2>/dev/null ||
		$CC -nostdlib -fuse-ld=lld -Wl,-e,0 -Wl,-T,"$lds" \
			-o "$out" "$@" 2>/dev/null
}

# make_vmlinux_pair <orig object...> -- <patched object...>
#
# Build the vmlinux.o / vmlinux pair klp diff needs to resolve sympos the way
# it does for built-in code, and point the diff at it.
#
# For a module, sympos is a count in symbol table order, which klp diff can do
# from the object alone.  vmlinux is different: the final link reorders
# sub-sections, so the position comes from the linked image, bridged by
# .klp.symid.  klp diff only looks for that when the object it was handed is
# called vmlinux.o and a vmlinux sits beside it -- so both the name and the
# linked image matter.
make_vmlinux_pair()
{
	local orig=() patched=() seen= arg

	for arg in "$@"; do
		if [ "$arg" = -- ]; then seen=y; continue; fi
		if [ -n "$seen" ]; then patched+=( "$arg" ); else orig+=( "$arg" ); fi
	done

	# Both sides have to have been named.  Without this, forgetting the --
	# leaves one list empty, the link of nothing fails, and the test skips
	# saying the toolchain cannot link -- which is a test bug wearing the
	# costume of an environment one.
	[ "${#orig[@]}" -gt 0 ] && [ "${#patched[@]}" -gt 0 ] ||
		fail "make_vmlinux_pair needs objects either side of --"

	partial_link "$workdir/vmlinux.o" "${orig[@]}" ||
		probe_skip "partial link unavailable"
	partial_link "$workdir/patched.o" "${patched[@]}" ||
		probe_skip "partial link unavailable"

	"$OBJTOOL" --klp-symids --link "$workdir/vmlinux.o" ||
		fail "objtool --klp-symids failed"

	link_vmlinux "$workdir/vmlinux" "$workdir/vmlinux.o" ||
		probe_skip "cannot link a vmlinux here"

	orig_obj=vmlinux.o
}

# build_module_pair <fixture.c> <module name> [cflags...]
#
# Build the pair as objects belonging to a module rather than to vmlinux.  klp
# diff reads the object's module name from .modinfo, and that decides which
# object a relocation is attributed to and whether a reference counts as
# cross-module, so a good deal of the code has a module path the vmlinux
# fixtures never reach.
#
# The fixture defines its .modinfo name from MODNAME.  Passing that through
# -D needs two levels of quoting, which is easy to get wrong at the call site.
build_module_pair()
{
	local fixture="$1" modname="$2"; shift 2

	build_pair "$fixture" -DMODNAME="\"$modname\"" "$@"
}

# find_thinlto_toolchain
#
# Set $THIN_LD to an lld from the same LLVM release as $CC (or THIN_CC).  A
# mismatched pair fails with "Invalid summary version", which reads like a
# broken test rather than a broken environment.
#
# ThinLTO is clang-only; callers must use clang_only before calling this.
# Only $CC (or an explicit THIN_CC override) is consulted -- the harness does
# not search for a second compiler beside a gcc $CC.
find_thinlto_toolchain()
{
	local cc ver ld

	for cc in "${THIN_CC:-}" "$CC"; do
		[ -n "$cc" ] || continue
		command -v "${cc%% *}" >/dev/null 2>&1 || return 1

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

out_sections() { $READELF -S -W "$workdir/out.o" 2>/dev/null; }
out_relocs()   { $READELF -r -W "$workdir/out.o" 2>/dev/null; }
out_symbols()  { $READELF -s -W "$workdir/out.o" 2>/dev/null; }
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
