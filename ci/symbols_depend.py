#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2025 Analog Devices Inc.

import sys
from os import path, environ
sys.path.insert(0, path.dirname(path.abspath(__file__)))
import kconfiglib

from sys import argv, stderr
import re

debug = False

arch_map = {
    'arm' : ['ARCH_SC59X', 'ARCH_SC5XX'],
    'arm64': ['ARCH_SC59X_64'],
}

# Only non-identity ARCH -> SRCARCH mappings
_srcarch_overrides = {
    'x86_64': 'x86',
    'i386':   'x86',
}


def init_kconfig():
    arch_env = environ.get('ARCH', 'x86')
    environ.setdefault('ARCH', arch_env)
    environ.setdefault('SRCARCH', _srcarch_overrides.get(arch_env, arch_env))
    environ.setdefault('KERNELVERSION', '0.0')
    environ.setdefault('srctree', '.')
    environ.setdefault('CC', 'gcc')
    environ.setdefault('LD', 'ld')

    try:
        return kconfiglib.Kconfig(warn=False)
    except Exception as e:
        print(f"Failed to load Kconfig: {e}", file=stderr)
        sys.exit(1)


def filter_symbols(symbols, arch, allow=None):
    """
    Remove architecture symbols (ARCH_*, CPU_*, bare arch names).
    Keep known-entangled ARCH_ symbols for the active architecture.
    Drop ARCH_ entirely when COMPILE_TEST is present (it satisfies the OR).
    """
    skip_archs = {'ARM', 'ARM64', 'M68K', 'RISCV', 'SUPERH', 'X86',
                  'X86_32', 'XTENSA'}
    result = {
        s for s in symbols
        if s not in skip_archs
        and not s.startswith('CPU_')
        and (not s.startswith('ARCH_') or s == allow)
    }

    # COMPILE_TEST satisfies any ARCH_ OR-branch; drop ARCH_ if present
    if 'COMPILE_TEST' in result:
        result = {s for s in result if not s.startswith('ARCH_')}

    # Re-add known-entangled ARCH_ symbols for the active arch
    if arch and arch in arch_map:
        result |= symbols & set(arch_map[arch])

    return result


def collect_syms_from_expr(expr):
    """
    Recursively collect symbol names that need to be enabled from a
    kconfiglib expression tree.

    Skips: NOT, comparisons to 'n', inequality/ordering ops, Choice nodes.
    Collects: bare symbol references, SYM=y, AND/OR children.
    """
    if isinstance(expr, kconfiglib.Symbol):
        if expr.name in ('y', 'n', 'm'):
            return set()
        return {expr.name}

    if not isinstance(expr, tuple):
        return set()

    op = expr[0]

    # Negation and ordering — symbol must be disabled/bounded, skip
    if op in (kconfiglib.NOT, kconfiglib.LESS, kconfiglib.LESS_EQUAL,
              kconfiglib.GREATER, kconfiglib.GREATER_EQUAL):
        return set()

    # Equality / inequality — only collect when comparing to non-'n'
    if op in (kconfiglib.EQUAL, kconfiglib.UNEQUAL):
        left, right = expr[1], expr[2]
        rval = right.name if isinstance(right, kconfiglib.Symbol) else right
        if rval == 'n':
            return set()
        if isinstance(left, kconfiglib.Symbol) and left.name not in ('y', 'n', 'm'):
            return {left.name}
        return set()

    # AND / OR — recurse into children
    result = set()
    for item in expr[1:]:
        result |= collect_syms_from_expr(item)
    return result


def get_symbol_deps(kconf, symbol, arch, allowlist):
    """
    Direct dependencies of *symbol* (includes enclosing if-block conditions).
    """
    sym = kconf.syms.get(symbol)
    if sym is None:
        if debug:
            print(f"Symbol {symbol} not found in Kconfig", file=stderr)
        return set()

    deps = collect_syms_from_expr(sym.direct_dep)
    if sym.choice is not None:
        deps |= collect_syms_from_expr(sym.choice.direct_dep)

    allow = symbol if symbol in allowlist else None
    return filter_symbols(deps, arch, allow)


def _collect_selectors(expr):
    """
    Extract the selector symbols from a rev_dep expression.

    kconfiglib builds rev_dep as OR(AND(selector, deps), ...)
    (see _make_and(sym, cond)).
    """
    if isinstance(expr, kconfiglib.Symbol):
        if expr.name in ('y', 'n', 'm'):
            return set()
        return {expr.name}

    if not isinstance(expr, tuple):
        return set()

    op = expr[0]

    if op == kconfiglib.OR:
        result = set()
        for item in expr[1:]:
            result |= _collect_selectors(item)
        return result

    if op == kconfiglib.AND:
        left = expr[1]
        if isinstance(left, kconfiglib.Symbol) and left.name not in ('y', 'n', 'm'):
            return {left.name}
        return set()

    return set()


def get_selectors(kconf, symbol):
    """
    Return symbols that select a symbol, extracted from rev_dep.
    """
    sym = kconf.syms.get(symbol)
    if sym is None:
        return set()
    return _collect_selectors(sym.rev_dep)


def is_hidden_sym(sym):
    return bool(sym.nodes) and not any(node.prompt for node in sym.nodes)


def _pick_selector(kconf, selectors, arch):
    """
    Pick a single visible selector to resolve a hidden dependency. Prefer on
    the active arch, then the fewest dependencies, then name.
    """
    arch_syms = set(arch_map.get(arch, []))
    cands = []
    for s in selectors:
        sym = kconf.syms.get(s)
        if sym is None or is_hidden_sym(sym):
            continue
        deps = collect_syms_from_expr(sym.direct_dep)
        cands.append((not (deps & arch_syms), len(deps), s))
    return min(cands)[2] if cands else None


def resolve_all(kconf, seeds, arch):
    """
    Depth-first walk over the dependency graph starting from *seeds*.
    Returns the full set of transitive dependencies (including seeds).

    Hidden symbols cannot be set directly, and are excluded from the result:
    * hidden seeds: collect all the symbols that select them.
    * hidden dependencies: if no selector was already collected, collect a
    single selector (see _pick_selector()).
    """
    seed_set = set(seeds)
    allowlist = seed_set.copy()
    visited = set()
    hidden = set()
    hidden_deps = set()
    queue = list(seeds)

    while True:
        while queue:
            sym_name = queue.pop()
            if sym_name in visited:
                continue
            visited.add(sym_name)
            if sym_name not in kconf.syms:
                continue
            sym = kconf.syms[sym_name]
            if is_hidden_sym(sym):
                hidden.add(sym_name)
                if sym_name in seed_set:
                    for sel in get_selectors(kconf, sym_name):
                        if sel not in visited:
                            queue.append(sel)
                else:
                    hidden_deps.add(sym_name)
                continue
            for dep in get_symbol_deps(kconf, sym_name, arch, allowlist):
                if dep not in visited:
                    queue.append(dep)

        # Satisfy hidden deps no collected symbol selects yet.
        for sym_name in sorted(hidden_deps):
            selectors = get_selectors(kconf, sym_name)
            if selectors & visited:
                continue
            sel = _pick_selector(kconf, selectors, arch)
            if sel is not None and sel not in visited:
                if debug:
                    print(f"{sym_name} is hidden, selecting it with {sel}",
                          file=stderr)
                queue.append(sel)
                break
        if not queue:
            break

    return visited - hidden


def _to_c(obj):
    return obj[:-2] + '.c'


def ensure_single_choice(kconf, symbols, file_syms):
    """
    The last item of a choice on .config takes effect, ensure one is
    chosen, preferring non-default members.

    Returns the reduced symbols and a list of (dropped_file, [kept_files]).
    """
    by_choice = {}
    for s in symbols:
        sym = kconf.syms.get(s)
        if sym is not None and sym.choice is not None:
            by_choice.setdefault(sym.choice, []).append(s)

    result = set(symbols)
    exclusive = []
    for members in sorted(by_choice.values(), key=min):
        if len(members) < 2:
            continue
        choice = kconf.syms[members[0]].choice
        # First declared 'default' (or first member if none)
        if choice.defaults:
            default = choice.defaults[0][0].name
        else:
            default = choice.syms[0].name if choice.syms else None
        keep = min(members, key=lambda s: (s == default, s))
        dropped = set(members) - {keep}
        result -= dropped

        def needs(paths, syms):
            return bool(paths) and all(p & syms for p in paths)

        kept_files = sorted(_to_c(f) for f, paths in file_syms.items()
                            if needs(paths, {keep}))
        for f, paths in sorted(file_syms.items()):
            if not needs(paths, dropped):
                continue
            exclusive.append((_to_c(f), kept_files))
            print(f"::warning file={_to_c(f)},line=0::"
                  f"File cannot be compiled with "
                  f"{', '.join(kept_files) or keep} "
                  f"(choice {', '.join(sorted(dropped))} vs {keep}).",
                  "Are you sure this is reasonable?",
                  file=stderr)
    return result, exclusive


# Makefile .o -> CONFIG_SYMBOL resolution
def _read_makefile_lines(mk):
    """Read a Makefile, joining backslash-continued lines."""
    with open(mk) as f:
        raw = f.readlines()
    joined, buf = [], ""
    for line in raw:
        buf += line.rstrip('\n')
        if line.endswith('\\\n'):
            continue
        joined.append(buf)
        buf = ""
    return joined


def _has_token(line, token):
    """
    Whether *token* (e.g. 'foo.o', 'foo/') appears as a whole word in a
    Makefile line, so 'foo.o' does not match 'bar-foo.o'.
    """
    return re.search(rf'(?<![\w.-]){re.escape(token)}(?![\w.-])',
                     line) is not None


def get_top_level_symbol_for(mk):
    """
    For an obj-y line in drivers/foo/Makefile, look in drivers/Makefile
    for the CONFIG_* that gates foo/.
    """
    obj = f"{path.basename(path.dirname(mk))}/"
    parent_mk = path.join(path.dirname(path.dirname(mk)), "Makefile")
    if not path.isfile(parent_mk):
        return None, None

    for line in _read_makefile_lines(parent_mk):
        if not _has_token(line, obj):
            continue
        match = re.search(r'obj-\$\(([^)]+)\)\s*[+:]?=', line)
        if match:
            return match.group(1), None

    return None, None


def _iter_makefile_matches(mk, obj, l_obj):
    """
    Yield (CONFIG_sym_or_None, sub_obj_or_None) for every Makefile line that
    references *obj*.
    """
    if obj == l_obj:
        print(f"{mk}: Infinite recursion for '{obj}' detected", file=stderr)
        return
    if debug:
        print(f"{mk}: Looking for '{obj}'", file=stderr)

    for line in _read_makefile_lines(mk):
        if not _has_token(line, obj):
            continue

        # obj-$(CONFIG_SYMBOL)
        m = re.search(r'obj-\$\(([^)]+)\)\s*[+:]?=', line)
        if m:
            yield m.group(1), None
            continue

        # obj-y / lib-y
        if re.match(r'\s*(obj|lib)-y\s*[+:]?=', line):
            yield get_top_level_symbol_for(mk)
            continue

        # driver-$(CONFIG_SYMBOL)
        m = re.search(r'([-\w\d]+)-\$\(([^)]+)\)\s*[+:]?=', line)
        if m:
            yield m.group(2), m.group(1)
            continue

        # driver-y / driver-objs
        m = re.search(r'([-\w\d]+)-(y|objs)\s*[+:]?=', line)
        if m:
            if m.group(1) == obj[:-2]:
                continue                # mconf-objs := mconf.o — skip self
            yield None, m.group(1)


def _resolve_obj_in_mk(mk, obj, l_obj):
    """Chase Makefile references until fully resolved. Returns set of symbols."""
    paths = []
    found = False
    for symbol, sub_obj in _iter_makefile_matches(mk, obj, l_obj):
        if symbol is None and sub_obj is None:
            continue
        found = True
        syms = set()
        if symbol:
            if not symbol.startswith("CONFIG_"):
                print(f"Symbol '{symbol}' does not start with 'CONFIG_' "
                      f"at '{mk}'", file=stderr)
            else:
                syms.add(symbol[7:])
        if sub_obj:
            sub_paths, _ = _resolve_obj_in_mk(mk, f"{sub_obj}.o", obj)
            if sub_paths:
                paths += [p | syms for p in sub_paths]
                continue
        paths.append(frozenset(syms))
    return paths, found


def get_symbols_from_files(files, arch):
    """
    Resolve .o targets and plain symbol names into a set of CONFIG symbols.
    Also returns the symbols gating each .o target.
    """
    files = {f for f in files if f}
    obj_files = sorted(f for f in files if f.endswith('.o'))
    plain_syms = {f for f in files if not f.endswith('.o')}
    symbols = filter_symbols(plain_syms, arch)
    file_syms = {}

    for f in obj_files:
        file_syms[f] = []
        found = False
        base = path.basename(f)
        ldir = path.dirname(f)
        rdir = ""
        while ldir:
            mk = path.join(ldir, "Makefile")
            if path.isfile(mk):
                paths, found = _resolve_obj_in_mk(mk, path.join(rdir, base),
                                                  None)
                for p in paths:
                    symbols |= p
                file_syms[f] += paths
                if found:
                    break
            rdir = path.join(path.basename(ldir), rdir)
            ldir = path.dirname(ldir)
        if not found:
            print(f"Failed to find Makefile targeting {f}", file=stderr)

    return symbols, file_syms


def main():
    """
    Resolve dependencies of symbols / .o targets.

    Usage:  symbols=$(ci/symbols_depend.py [SYMBOLS] [O_FILES])
    """
    arch = environ.get('ARCH', None)
    if arch not in arch_map:
        arch = None

    kconf = init_kconfig()

    seeds, file_syms = get_symbols_from_files(set(argv[1:]), arch)
    print("Symbols of touched files:", file=stderr)
    print(seeds, file=stderr)

    resolved = resolve_all(kconf, seeds, arch)
    result, exclusive = ensure_single_choice(kconf, filter_symbols(resolved, arch),
                                             file_syms)
    result = sorted(result)

    # Persist for assert_compiled, which must not fail on these files.
    interlock_file = environ.get('INTERLOCK_FILE')
    if interlock_file:
        with open(interlock_file, 'w') as f:
            for dropped, kept in exclusive:
                f.write(f"{dropped} {' '.join(kept)}\n")

    print("Resolved symbols:", file=stderr)
    print(result, file=stderr)
    print(' '.join(result))


main()
