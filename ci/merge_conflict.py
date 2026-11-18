#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2026 Analog Devices Inc.

import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

STATE = "merge-resolve"
CONFLICTS = "merge-conflict.json"
SPARSE = {"GIT_CONFIG_COUNT": "2",
          "GIT_CONFIG_KEY_0": "core.sparseCheckout", "GIT_CONFIG_VALUE_0": "true",
          "GIT_CONFIG_KEY_1": "core.sparseCheckoutCone", "GIT_CONFIG_VALUE_1": "false"}
RULES = ".github/merge-conflict"
FIXUPS = ".github/merge-fixups"
TEMPLATE = """\
TODO: <subsystem>: <subject>

TODO: Stable commit <sha> ("<subject>") <what it changed>. Downstream
<how it diverges>, so <what breaks>. <What this fixup does>.
"""


def git(repo, *args, env=None):
    return subprocess.check_output(["git", "-C", str(repo), *args], env=env)


def git_path(repo, name):
    path = Path(git(repo, "rev-parse", "--git-path", name).decode().strip())
    return path if path.is_absolute() else repo / path


def index_file(repo):
    return git_path(repo, "index")


def relative(repo, path):
    return os.path.relpath(path, repo)


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


def load(rules):
    paths = rules / "paths"
    if not paths.exists():
        return {}
    lines = [line.split("\t", 1) for line in paths.read_text().splitlines() if line]
    return {name: rid for rid, name in lines}


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


def fixups(repo, directory):
    patches = sorted(directory.glob("*.patch"))
    if not patches:
        return []
    failed = []
    with tempfile.TemporaryDirectory(prefix="merge-fixups-") as tmp:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(tmp) / "index"))
        shutil.copyfile(index_file(repo), env["GIT_INDEX_FILE"])
        for patch in patches:
            ret = subprocess.run(["git", "-C", str(repo), "apply", "--cached", str(patch)],
                                 env=env, stderr=subprocess.DEVNULL).returncode
            if ret:
                failed.append(str(patch.relative_to(repo)) if patch.is_relative_to(repo) else str(patch))
    return failed


def description(patch):
    text = []
    for line in patch.read_text(errors="replace").splitlines():
        if line.startswith(("diff --git ", "--- ")):
            break
        text.append(line)
    return "\n".join(text).strip()


def stable_merge(repo, rev):
    merge = git(repo, "log", "--first-parent", "--merges", "-1", "--format=%H", rev)
    merge = merge.decode().strip()
    if not merge:
        raise ValueError(f"no merge commit in the first-parent history of {rev}")
    return merge


def new_fixup(repo, directory, rules, name, paths, message):
    slug = re.sub(r"[^a-z0-9]+", "-", name.lower()).strip("-")
    if not slug:
        raise ValueError(f"invalid fixup name: {name!r}")
    if not paths:
        paths = [".", *(f":(exclude){relative(repo, d)}" for d in (directory, rules))]
    diff = git(repo, "diff", "HEAD", "--", *paths)
    if not diff:
        raise ValueError("no changes against HEAD: edit the sources first")
    numbers = [int(p.name[:4]) for p in directory.glob("[0-9][0-9][0-9][0-9]-*.patch")]
    patch = directory / f"{max(numbers, default=0) + 1:04d}-{slug}.patch"
    header = "\n\n".join(message) if message else TEMPLATE
    directory.mkdir(parents=True, exist_ok=True)
    patch.write_bytes(header.rstrip("\n").encode() + b"\n\n" + diff)
    if not message:
        print(f"fixup: write the description in {relative(repo, patch)}", file=sys.stderr)
    return [relative(repo, patch)]


def resolutions(repo, rules, rev=None):
    """Blobs of the resolution of each file, in the working tree or at a revision."""
    rel = relative(repo, rules)
    if rev:
        ret = subprocess.run(["git", "-C", str(repo), "ls-tree", "-r", "-z", rev, "--", rel],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        blobs = {name: info.split()[2] for info, name in
                 (item.decode().split("\t", 1) for item in ret.stdout.split(b"\0") if item)}
        if f"{rel}/paths" not in blobs:
            return {}
        lines = git(repo, "show", f"{rev}:{rel}/paths").decode().splitlines()
    else:
        files = sorted(relative(repo, file) for file in rules.glob("*/*"))
        oids = git(repo, "hash-object", "--", *files).decode().split() if files else []
        blobs = dict(zip(files, oids))
        lines = (rules / "paths").read_text().splitlines() if (rules / "paths").exists() else []
    entries = [line.split("\t", 1) for line in lines if line]
    return {name: sorted(item for item in blobs.items() if item[0].startswith(f"{rel}/{rid}/"))
            for rid, name in entries}


def changed_resolutions(repo, rules, base):
    merged, current = resolutions(repo, rules, base), resolutions(repo, rules)
    return {name for name in {*merged, *current} if merged.get(name) != current.get(name)}


def verify(repo, directory, rules, base):
    base = base or stable_merge(repo, "HEAD")
    changed = changed_resolutions(repo, rules, base)
    problems = []
    with tempfile.TemporaryDirectory(prefix="merge-verify-") as tmp:
        replayed = dict(os.environ, GIT_INDEX_FILE=str(Path(tmp) / "replayed"))
        current = dict(os.environ, GIT_INDEX_FILE=str(Path(tmp) / "current"))
        git(repo, "read-tree", base, env=replayed)
        for patch in sorted(directory.glob("*.patch")):
            name = relative(repo, patch)
            text = description(patch)
            if not text or "TODO" in text:
                problems.append(f"{name}: missing description")
            ret = subprocess.run(["git", "-C", str(repo), "apply", "--cached", str(patch)],
                                 env=replayed, stderr=subprocess.PIPE, text=True)
            if ret.returncode:
                problems.append(f"{name}: does not apply on {base[:12]}: "
                                f"{(ret.stderr.strip().splitlines() or ['?'])[0]}")
        shutil.copyfile(index_file(repo), current["GIT_INDEX_FILE"])
        git(repo, "add", "-u", env=current)
        recipes = [str(d) for d in (directory, rules) if d.exists()]
        for env in (replayed, current):
            if recipes:
                git(repo, "add", "-f", "-A", "--", *recipes, env=env)
        trees = [git(repo, "write-tree", env=env).decode().strip() for env in (replayed, current)]
        if trees[0] != trees[1]:
            diff = git(repo, "diff-tree", "-r", "--name-only", *trees).decode().split("\n")
            problems.extend(f"{path}: replayed fixups differ from the working tree"
                            for path in diff if path and path not in changed)
    for path in sorted(changed):
        print(f"verify: {path}: resolution changed, not compared", file=sys.stderr)
    return problems


def forget(repo, rules, paths):
    entries = load(rules)
    missing = [name for name in paths if name not in entries]
    if missing:
        raise ValueError(f"no recorded resolution: {' '.join(missing)}")
    for name in paths:
        rid = entries.pop(name)
        if rid not in entries.values():
            shutil.rmtree(rules / rid, ignore_errors=True)
    save(rules, entries)
    return paths


def apply_fixups(repo, directory, env):
    failed = []
    for patch in sorted(directory.glob("*.patch")):
        ret = subprocess.run(["git", "-C", str(repo), "apply", "--cached", str(patch)],
                             env=env, stderr=subprocess.DEVNULL).returncode
        if ret:
            failed.append(patch)
    return failed


def scratch_load(repo):
    state = git_path(repo, STATE)
    if not state.exists():
        raise ValueError("no resolve in progress")
    tmp = Path(state.read_text().strip())
    if not (tmp / "worktree").exists():
        state.unlink()
        raise ValueError(f"resolve state lost at {tmp}: start again")
    return state, tmp, sorted(merge_rr(tmp / "worktree"))


def resolve_start(repo, rules, paths):
    if git_path(repo, STATE).exists():
        raise ValueError("resolve in progress: use resolve --continue or resolve --abort")
    if not paths:
        raise ValueError("no paths to resolve")
    merge = stable_merge(repo, "HEAD")
    ours, theirs = (git(repo, "rev-parse", f"{merge}^{n}").decode().strip() for n in (1, 2))
    tmp = Path(tempfile.mkdtemp(prefix="merge-resolve-"))
    scratch, scratch_rules = tmp / "worktree", tmp / "rules"
    try:
        if rules.exists():
            shutil.copytree(rules, scratch_rules)
        forget(repo, scratch_rules, paths)
        # Redo the merge in a sparse repository that borrows the objects
        git(tmp, "init", "-q", str(scratch))
        objects = git_path(repo, "objects").resolve()
        (scratch / ".git/objects/info/alternates").write_text(f"{objects}\n")
        sparse = scratch / ".git/info/sparse-checkout"
        sparse.parent.mkdir(parents=True, exist_ok=True)
        sparse.write_text("".join(f"/{name}\n" for name in paths))
        git(scratch, "checkout", "-q", "--detach", ours)
        subprocess.run(["git", "-C", str(scratch), "-c", "rerere.enabled=false", "merge", "-q",
                        "--no-ff", "--no-commit", theirs],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if not (scratch / ".git/MERGE_HEAD").exists():
            raise ValueError(f"cannot redo the merge {merge[:12]}")
        sparse.write_text("/*\n")
        unresolved = replay(scratch, scratch_rules)
        shared = [name for name in paths if name not in unresolved]
        if shared:
            raise ValueError("resolution shared with other files, resolve them together: "
                             f"{' '.join(shared)}")
        if git(repo, "status", "--porcelain", "--", *unresolved):
            raise ValueError(f"local changes: commit or restore {' '.join(unresolved)}")
        for name in unresolved:
            shutil.copyfile(scratch / name, repo / name)
    except BaseException:
        shutil.rmtree(tmp, ignore_errors=True)
        raise
    git_path(repo, STATE).write_text(f"{tmp}\n")
    print("resolve: edit the conflict blocks (ours is the branch, theirs is stable) of the "
          "files below,\nthen run 'resolve --continue', or 'resolve --abort' to give up",
          file=sys.stderr)
    return unresolved


def resolve_continue(repo, rules, directory):
    state, tmp, paths = scratch_load(repo)
    scratch, scratch_rules = tmp / "worktree", tmp / "rules"
    for name in paths:
        shutil.copyfile(repo / name, scratch / name)
    record(scratch, scratch_rules)
    clear(rules)
    shutil.copytree(scratch_rules, rules, dirs_exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="merge-fixups-") as env_tmp:
        env = dict(os.environ, GIT_INDEX_FILE=str(Path(env_tmp) / "index"))
        shutil.copyfile(index_file(scratch), env["GIT_INDEX_FILE"])
        failed = apply_fixups(scratch, directory, env)
        for name in paths:
            (repo / name).write_bytes(git(scratch, "show", f":{name}", env=env))
    shutil.rmtree(tmp, ignore_errors=True)
    state.unlink()
    print(f"resolve: recorded {' '.join(paths)}", file=sys.stderr)
    if failed:
        print("resolve: update the fixups below, they do not apply on the new resolution",
              file=sys.stderr)
    return [relative(repo, patch) for patch in failed]


def resolve_abort(repo):
    state, tmp, paths = scratch_load(repo)
    if paths:
        git(repo, "checkout", "HEAD", "--", *paths)
    shutil.rmtree(tmp, ignore_errors=True)
    state.unlink()
    return paths


def show(repo, rules, directory):
    lines = [f"resolution {name}" for name in sorted(load(rules))]
    for patch in sorted(directory.glob("*.patch")):
        subject = (description(patch).splitlines() or ["(no description)"])[0]
        lines.append(f"fixup {patch.name}: {subject}")
    return lines


def main():
    parser = argparse.ArgumentParser(
        description="Replay and record merge conflict resolutions and fixups",
        epilog="Run from the root of an lts/<ref> worktree. The directories default to "
               f"{RULES} and {FIXUPS}.")
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--repo", type=Path, default=Path("."))
    sub = parser.add_subparsers(dest="action", required=True, metavar="action")

    for action, kind, text in (
            ("replay", "rules", "apply recorded resolutions, print unresolved paths"),
            ("record", "rules", "record the conflicts left by replay, once resolved"),
            ("fixups", "fixups", "print fixups that do not apply on the index")):
        p = sub.add_parser(action, help=text, parents=[common])
        p.add_argument(kind, type=Path, nargs="?")

    sub.add_parser("list", help="list recorded resolutions and fixups", parents=[common])
    p = sub.add_parser("fixup", parents=[common],
                       help="write the next fixup from the changes against HEAD")
    p.add_argument("name", help="short name, e.g. spi-nor-use-get-params")
    p.add_argument("paths", nargs="*", help="limit the fixup to paths")
    p.add_argument("-m", "--message", action="append", default=[],
                   help="description paragraph, may be repeated (subject first)")
    p = sub.add_parser("verify", parents=[common],
                       help="check the fixups replay on the stable merge into the working tree")
    p.add_argument("--base", help="resolved merge, defaults to the last first-parent merge of HEAD")
    p = sub.add_parser("forget", parents=[common],
                       help="drop recorded resolutions, to resolve them again")
    p.add_argument("paths", nargs="+")
    p = sub.add_parser("resolve", parents=[common],
                       help="resolve recorded conflicts again, by hand, in the working tree")
    p.add_argument("paths", nargs="*")
    group = p.add_mutually_exclusive_group()
    group.add_argument("--continue", dest="cont", action="store_true",
                       help="record the resolutions")
    group.add_argument("--abort", action="store_true", help="restore the files")
    for name in ("list", "fixup", "verify", "forget", "resolve"):
        p = sub.choices[name]
        p.add_argument("--rules", type=Path)
        if name != "forget":
            p.add_argument("--fixups", type=Path)

    args = parser.parse_args()
    repo = args.repo.resolve()
    rules = (getattr(args, "rules", None) or repo / RULES).resolve()
    fixups_dir = (getattr(args, "fixups", None) or repo / FIXUPS).resolve()
    actions = {
        "replay": lambda: replay(repo, rules),
        "record": lambda: record(repo, rules),
        "fixups": lambda: fixups(repo, fixups_dir),
        "list": lambda: show(repo, rules, fixups_dir),
        "fixup": lambda: new_fixup(repo, fixups_dir, rules, args.name, args.paths, args.message),
        "verify": lambda: verify(repo, fixups_dir, rules, args.base),
        "forget": lambda: forget(repo, rules, args.paths),
        "resolve": lambda: resolve_start(repo, rules, args.paths),
        "resolve --continue": lambda: resolve_continue(repo, rules, fixups_dir),
        "resolve --abort": lambda: resolve_abort(repo),
    }
    action = args.action
    if action == "resolve":
        os.environ.update(SPARSE)
        action += " --continue" if args.cont else " --abort" if args.abort else ""
    try:
        paths = actions[action]()
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(f"{action}: {exc}", file=sys.stderr)
        return 1
    for path in paths:
        print(path)
    if args.action == "verify" and not paths:
        print("verify: fixups replay into the working tree", file=sys.stderr)
    return 2 if action in ("replay", "fixups", "verify", "resolve --continue") and paths else 0


if __name__ == "__main__":
    sys.exit(main())
