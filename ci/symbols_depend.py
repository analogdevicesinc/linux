#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2025 Analog Devices Inc.

from sys import argv, stderr
from os import path, getcwd
from glob import glob
import re

debug = False

symbols_ = set()
deps_ = set()
map_ = {}
max_recursion = 200


def generate_map():
    """
    Build a map with all symbols.
    """
    glob_ = path.join('**', 'Kconfig')
    kconfig = glob(glob_, recursive=True)
    for k in kconfig:
        stack = []
        with open(k) as f:
            lines = f.readlines()
        for line in lines:
            line = line.strip()
            if line.startswith('config '):
                stack.append(line[7:].strip())
            elif line.startswith('menuconfig '):
                stack.append(line[11:].strip())

        map_[path.dirname(k)] = set(stack)


def get_all_parent_kconfigs(start_dir):
    """
    Walk directory tree after Kconfig files.
    """
    kconfig = path.abspath(start_dir)
    kconfigs = []
    while True:
        kconfig_path = path.join(kconfig, 'Kconfig')
        if path.isfile(kconfig_path):
            kconfigs.append(kconfig_path)
        new_path = path.dirname(kconfig)
        if new_path == kconfig:
            break
        kconfig = new_path
    return kconfigs


def track_if_blocks(symbol, target_kconfig):
    """
    Looks for the symbol or the source kconfig that includes the symbol.
    Return list of 'if' that hides symbol.
    """
    if_blocks = []
    target_abs = path.abspath(target_kconfig)

    for kconfig in get_all_parent_kconfigs(path.dirname(target_kconfig)):
        if debug:
            print(f"{target_kconfig}: Tracking if blocks at '{kconfig}'",
                  file=stderr)
        with open(kconfig, 'r') as f:
            lines = f.readlines()

        stack = []
        for line in lines:
            line_ = line.strip()
            if line.startswith('if '):
                stack.append(line[3:].strip())
            elif line_ == 'endif':
                if stack:
                    stack.pop()
            elif line_.startswith('source') and line_.endswith('Kconfig"'):
                m = re.search(r'"([^"]+)"', line)
                if m:
                    src_path = path.normpath(path.join(getcwd(), m.group(1)))
                    if path.abspath(src_path) == target_abs:
                        if_blocks += stack
                        break
            elif re.match(rf'(menu)?config\s+{symbol}\b', line.strip()):
                if_blocks += stack
                break

    return if_blocks


def find_symbol_block(symbol, kconfig_path):
    if debug:
        print(f"{kconfig_path}: Looking for symbol '{symbol}'", file=stderr)
    with open(kconfig_path) as f:
        lines = f.readlines()

    block = []
    in_block = False
    for line in lines:
        if in_block:
            if not re.match(r'^(?:\s+|\s*$)', line):
                break
            block.append(line)
        elif re.match(rf'(menu)?config\s+{symbol}\b', line.strip()):
            in_block = True
            block = [line]
    return ''.join(block) if block else None


def filter_symbols(symbols):
    """
    Remove architecture symbols.
    """
    archs = ['ARM', 'ARM64', 'M68K', 'RISCV', 'SUPERH', 'X86', 'X86_32',
             'XTENSA']
    return {sym
            for sym in symbols
            if not sym.startswith('ARCH_') and sym not in archs
            and not sym.startswith('CPU_')}


def extract_dependencies(symbol_block, if_blocks):
    depends = re.findall(r'depends on\s+(.+)', symbol_block)
    all_conds = depends + if_blocks
    joined = ' && '.join(all_conds)
    if joined == '':
        return []
    deps = sorted(set(re.split(r'\s*(?:&&|\|\|)\s*', joined)))
    deps = {sym.replace('(', '').replace(')', '') for sym in deps}
    deps = {sym[:-2] if sym.endswith('=y') else sym
            for sym in deps
            if not sym.endswith('=n') and not sym.startswith('!')}
    return filter_symbols(deps)


def get_symbol_dependencies(symbol, path_to_kconfig_dir):
    kconfig_file = path.join(path_to_kconfig_dir, 'Kconfig')
    if_blocks = track_if_blocks(symbol, kconfig_file)
    block = find_symbol_block(symbol, kconfig_file)
    if not block:
        print(f"Symbol {symbol} not found in {kconfig_file}", file=stderr)
        return []
    return extract_dependencies(block, if_blocks)


def get_top_level_kconfig(symbol):
    found = False
    for kconfig in map_:
        if symbol in map_[kconfig]:
            found = True
            break
    if not found:
        print(f"Failed to find kconfig with {symbol}", file=stderr)
    return kconfig if found else None


def resolve_tree(symbol, path):
    global max_recursion

    max_recursion = max_recursion - 1
    deps = get_symbol_dependencies(symbol, path)
    for s in deps:
        if s not in deps_:
            kconfig = get_top_level_kconfig(s)
            if kconfig:
                if max_recursion:
                    resolve_tree(s, kconfig)
                deps_.add(s)


def get_makefile_symbol_and_obj(mk, obj):
    """
    Get Kconfig symbol and obj, example:

        driver-y += devices/driver/public/src/driver_extra.o
        driver-objs += some_obj.o
        driver-$(CONFIG_OF) += driver_of.o
        obj-$(CONFIG_DRIVER) += driver.o

    Resolution:

        driver_of.o -> ("CONFIG_OF", "driver")
        driver_extra.o -> (None, "driver")
        driver.o -> ("CONFIG_DRIVER", None)

    """
    if debug:
        print(f"{mk}: Looking for '{obj}'", file=stderr)

    with open(mk, "r") as file:
        lines = file.readlines()
    file_ = []
    buffer = ""
    for line in lines:
        buffer += line.rstrip('\n')
        if line.endswith('\\\n'):
            continue
        file_.append(buffer)
        buffer = ""

    for line in file_:
        if obj not in line:
            continue

        # obj-$(CONFIG_SYMBOL)
        match = re.search(r'obj-\$\(([^)]+)\)\s*[+:]?=', line)
        if match:
            return (match.group(1), None)

        # driver-$(CONFIG_SYMBOL)
        match = re.search(r'([\w\d_]+)-\$\(([^)]+)\)\s*[+:]?=', line)
        if match:
            return (match.group(2), match.group(1))

        # driver-y
        match = re.search(r'([\w\d_]+)-(y|objs)\s*[+:]?=', line)
        if match:
            return (None, match.group(1))

    return (None, None)


def _get_symbols_from_mk(mk, obj):
    """
    Exhaust Makefile until no object and no symbol.
    If returns False, the parent method will proceed to search on ../Makefile,
    if True, the obj was fully resolved into symbols
    """
    global symbols_

    symbol, obj = get_makefile_symbol_and_obj(mk, obj)
    if not symbol and not obj:
        return False
    if symbol:
        if not symbol.startswith("CONFIG_"):
            print(f"Symbol '{symbol}' does not start with 'CONFIG_' at 'mk'",
                  file=stderr)
        symbols_.add(symbol[7:])
        if not obj:
            return True

    return _get_symbols_from_mk(mk, f"{obj}.o")


def get_symbols_from_o(files):
    """
    Resolve the base symbols for .o targets.
    """
    for f in files:
        found = False
        base = path.basename(f)
        ldir = path.dirname(f)
        rdir = ""
        while ldir != "":
            mk = path.join(ldir, "Makefile")
            if path.isfile(mk):
                if _get_symbols_from_mk(mk, path.join(rdir, base)):
                    found = True
                    break

            rdir = path.join(path.basename(ldir), rdir)
            ldir = path.dirname(ldir)
        if not found:
            print(f"Failed to find Makefile targeting {f}", file=stderr)


def main():
    """
    Resolve dependencies of a symbol based on the .o target.
    usage:

        symbols=$(ci/symbols_depend.py [O_FILES])

    """
    argv.pop(0)
    get_symbols_from_o(set(argv))
    print("Symbols of touched files:", file=stderr)
    print(symbols_, file=stderr)
    generate_map()
    for s in symbols_:
        if s not in deps_:
            kconfig = get_top_level_kconfig(s)
            if kconfig and max_recursion:
                resolve_tree(s, kconfig)
            deps_.add(s)

    if not max_recursion:
        print("Max allowed recursion call reached", file=stderr)
    symbols__ = filter_symbols(deps_)
    print("Resolved symbols:", file=stderr)
    print(symbols__, file=stderr)
    print(' '.join(symbols__))


main()
