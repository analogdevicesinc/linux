#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2026 Analog Devices Inc.
#
# Layer on top of git-rerere(1) to store the recorded merge conflict resolutions
# in the tree.
# The rules directory holds the rr-cache entries (<id>/preimage and
# <id>/postimage) of the merge and "paths", that maps them to the files.

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys

CONFLICTS = "merge-conflict.json"


def git(repo, *args, env=None):
    return subprocess.check_output(["git", "-C", str(repo), *args], env=env)


def git_path(repo, name):
    path = Path(git(repo, "rev-parse", "--git-path", name).decode().strip())
    return path if path.is_absolute() else repo / path


def rerere(repo, autoupdate=False):
    ret = subprocess.run(["git", "-C", str(repo), "-c", "rerere.enabled=true", "-c",
                          f"rerere.autoupdate={str(autoupdate).lower()}", "rerere"],
                         stderr=subprocess.PIPE, text=True)
    if ret.returncode:
        raise ValueError(f"git rerere failed: {ret.stderr.strip()}")


def merge_rr(repo):
    """Conflicts handled by rerere and not resolved yet, as {path: id}."""
    path = git_path(repo, "MERGE_RR")
    if not path.exists():
        return {}
    items = [item.decode().split("\t", 1) for item in path.read_bytes().split(b"\0") if item]
    return {name: rid for rid, name in items}


def unmerged(repo):
    items = git(repo, "ls-files", "-u", "-z").split(b"\0")
    return sorted({item.split(b"\t", 1)[1].decode() for item in items if item})


def save(rules, entries):
    rules.mkdir(parents=True, exist_ok=True)
    (rules / "paths").write_text("".join(f"{rid}\t{name}\n"
                                         for name, rid in sorted(entries.items())))


def clear(rules):
    for item in rules.iterdir() if rules.exists() else []:
        if item.is_dir() and re.fullmatch(r"[0-9a-f]{40,64}", item.name):
            shutil.rmtree(item)


def store(repo, rules, entries, unresolved=()):
    """Copy the resolutions of the merge from the rr-cache to the rules."""
    cache = git_path(repo, "rr-cache")
    clear(rules)
    stored = {}
    for name, rid in entries.items():
        if name in unresolved:
            continue
        for post in (cache / rid).glob("postimage*"):
            pre = post.with_name(post.name.replace("post", "pre"))
            if pre.exists():
                (rules / rid).mkdir(parents=True, exist_ok=True)
                shutil.copyfile(pre, rules / rid / pre.name)
                shutil.copyfile(post, rules / rid / post.name)
                stored[name] = rid
    save(rules, stored)


def conflicts_file(repo):
    return git_path(repo, CONFLICTS)


def replay(repo, rules):
    if not unmerged(repo):
        return []
    head = git(repo, "rev-parse", "MERGE_HEAD").decode().strip()
    state = conflicts_file(repo)
    known = json.loads(state.read_text()) if state.exists() else {}
    entries = known["paths"] if known.get("merge") == head else {}
    # Get the id of every conflict, the id depends on the conflict style
    for name in unmerged(repo):
        ret = subprocess.run(["git", "-C", str(repo), "checkout", "--conflict=diff3", "--", name],
                             stderr=subprocess.DEVNULL).returncode
        if ret:
            raise ValueError(f"unsupported conflict type: {name}")
    rerere(repo)
    entries.update({name: rid.partition(".")[0] for name, rid in merge_rr(repo).items()})
    unsupported = [name for name in unmerged(repo) if name not in entries]
    if unsupported:
        raise ValueError(f"unsupported conflict type: {' '.join(unsupported)}")
    # Leave only the recorded resolutions of those conflicts in the rr-cache
    cache = git_path(repo, "rr-cache")
    for rid in set(entries.values()):
        shutil.rmtree(cache / rid, ignore_errors=True)
        if (rules / rid).is_dir():
            shutil.copytree(rules / rid, cache / rid)
    # Apply and stage them, rerere only tries on new entries
    git_path(repo, "MERGE_RR").unlink(missing_ok=True)
    rerere(repo, autoupdate=True)
    unresolved = sorted(merge_rr(repo))
    state.write_text(json.dumps({"merge": head, "paths": entries}))
    store(repo, rules, entries, unresolved)
    return unresolved


def record(repo, rules):
    state = conflicts_file(repo)
    if not state.exists():
        raise ValueError("no replayed merge: run replay first")
    entries = json.loads(state.read_text())["paths"]
    entries.update({name: rid.partition(".")[0] for name, rid in merge_rr(repo).items()})
    rerere(repo)
    unresolved = sorted(merge_rr(repo))
    if unresolved:
        raise ValueError(f"unresolved markers: {' '.join(unresolved)}")
    resolved = [name for name in unmerged(repo) if name in entries]
    if resolved:
        git(repo, "add", "--", *resolved)
    store(repo, rules, entries)
    state.unlink()
    return resolved


def main():
    parser = argparse.ArgumentParser(description="Replay and record merge conflict resolutions")
    parser.add_argument("action", choices=["replay", "record"])
    parser.add_argument("rules", type=Path)
    parser.add_argument("--repo", type=Path, default=Path("."))
    args = parser.parse_args()
    try:
        paths = globals()[args.action](args.repo.resolve(), args.rules.resolve())
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"{args.action}: {exc}", file=sys.stderr)
        return 1
    for path in paths:
        print(path)
    return 2 if args.action == "replay" and paths else 0


if __name__ == "__main__":
    sys.exit(main())
