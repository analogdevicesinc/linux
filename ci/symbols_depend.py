#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2025 Analog Devices Inc.

from sys import argv, stderr
from os import path, getcwd
from glob import glob
import re

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
    archs = ['ARM', 'ARM64', 'M68K', 'RISCV', 'SUPERH', 'X86', 'X86_32', 'XTENSA']
    return {sym
            for sym in symbols
            if not sym.startswith('ARCH_') and not sym in archs
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


def main():
    """
    Resolve dependencies of a symbol.
    usage:

        all_symbols=$(ci/symbols_depend.py [SYMBOL...] 2> /dev/tty)

    """
    argv.pop(0)
    symbols = set(argv)

    generate_map()
    for s in symbols:
        if s not in deps_:
            kconfig = get_top_level_kconfig(s)
            if kconfig and max_recursion:
                resolve_tree(s, kconfig)
            deps_.add(s)

    if not max_recursion:
        print("Max allowed recursion call reached", file=stderr)
    print(' '.join(filter_symbols(deps_)))


main()
