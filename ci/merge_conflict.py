#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2026 Analog Devices Inc.

import argparse
import difflib
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

MARKERS = ("<<<<<<< ", "||||||| ", ">>>>>>> ")


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args])


def split_lines(text):
    return io.StringIO(text, newline="\n").readlines()


def digest(data):
    return hashlib.sha256(data).hexdigest()


def conflicts(repo):
    entries = {}
    for item in git(repo, "ls-files", "-u", "-z").split(b"\0"):
        if item:
            info, path = item.split(b"\t", 1)
            mode, oid, stage = info.decode().split()
            entries.setdefault(path.decode(), {})[stage] = mode
    for path, stages in entries.items():
        if set(stages) not in ({"1", "2", "3"}, {"2", "3"}) or set(stages.values()) != {"100644"}:
            raise ValueError(f"unsupported conflict type: {path}")
        if any(c in path for c in "\n\r\t"):
            raise ValueError(f"unsupported path: {path!r}")
    return sorted(entries)


def blocks(data):
    if b"\0" in data:
        raise ValueError("binary conflict")
    lines = split_lines(data.decode("utf-8"))
    spans, start = [], None
    for i, line in enumerate(lines):
        if line == "<<<<<<< ours\n" and start is None:
            start = i
        elif line == ">>>>>>> theirs\n" and start is not None:
            spans.append((start, i + 1))
            start = None
    if start is not None or not spans:
        raise ValueError("not a text conflict")
    return lines, spans


def normalized(repo, paths):
    result = {}
    for path in paths:
        git(repo, "checkout", "--conflict=diff3", "--", path)
        result[path] = (repo / path).read_bytes()
        blocks(result[path])
    return result


def load(directory):
    manifest = directory / "manifest.json"
    if not manifest.exists():
        return {"version": 3, "files": {}}
    data = json.loads(manifest.read_text())
    if data.get("version") != 3:
        raise ValueError("unsupported manifest version")
    return data


def save(directory, manifest):
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


def resolve_block(directory, chunk, original):
    patch = (directory / chunk["patch"]).resolve()
    if patch.parent != directory.resolve():
        raise ValueError("patch outside rules directory")
    if digest(patch.read_bytes()) != chunk["patch_hash"]:
        raise ValueError(f"patch checksum mismatch: {chunk['patch']}")
    with tempfile.TemporaryDirectory(prefix="merge-conflict-") as tmp:
        tmp = Path(tmp)
        subprocess.check_call(["git", "init", "-q", str(tmp)])
        (tmp / "conflict").write_bytes(original)
        stats = git(tmp, "apply", "--numstat", "-z", str(patch)).split(b"\0")
        if [s.split(b"\t", 2)[2] for s in stats if s] != [b"conflict"]:
            raise ValueError(f"patch changes unexpected paths: {chunk['patch']}")
        git(tmp, "apply", str(patch))
        resolved = (tmp / "conflict").read_bytes()
    if digest(resolved) != chunk["resolved"]:
        raise ValueError(f"resolved content mismatch: {chunk['patch']}")
    return split_lines(resolved.decode())


def match_groups(lines, spans, chunks):
    groups, i = [], 0
    while i < len(spans):
        for j in range(len(spans) - 1, i - 1, -1):
            lo, hi = spans[i][0], spans[j][1]
            key = digest("".join(lines[lo:hi]).encode())
            if key in chunks:
                groups.append((lo, hi, key))
                i = j + 1
                break
        else:
            return None
    return groups


def group_spans(spans, ops):
    groups = [list(span) for span in spans]
    for tag, a, b, _, _ in ops:
        if tag == "equal":
            continue
        hits = [i for i, (lo, hi) in enumerate(groups) if lo <= b and a <= hi]
        if not hits:
            raise ValueError("edits outside conflict blocks")
        lo, hi = groups[hits[0]][0], groups[hits[-1]][1]
        if a < lo or b > hi:
            raise ValueError("edits outside conflict blocks")
        groups[hits[0]:hits[-1] + 1] = [[lo, hi]]
    return [tuple(g) for g in groups]


def replay(repo, directory):
    paths = conflicts(repo)
    if not paths:
        return []
    manifest = load(directory)
    resolved, unresolved = {}, []
    for path, data in normalized(repo, paths).items():
        chunks = {c["preimage"]: c for c in manifest["files"].get(path, {}).get("chunks", [])}
        lines, spans = blocks(data)
        groups = match_groups(lines, spans, chunks)
        if groups is None:
            unresolved.append(path)
            continue
        for lo, hi, key in reversed(groups):
            lines[lo:hi] = resolve_block(directory, chunks[key], "".join(lines[lo:hi]).encode())
        resolved[path] = "".join(lines).encode()
    for path, data in resolved.items():
        (repo / path).write_bytes(data)
    if resolved:
        git(repo, "add", "--", *resolved)
    return unresolved


def capture(repo, directory):
    paths = conflicts(repo)
    if not paths:
        raise ValueError("no conflicts to capture")
    pending = directory / ".pending"
    shutil.rmtree(pending, ignore_errors=True)
    pending.mkdir(parents=True)
    index = {}
    for path, data in normalized(repo, paths).items():
        key = digest(path.encode())
        (pending / key).write_bytes(data)
        index[path] = key
    (pending / "index.json").write_text(json.dumps(index, indent=2) + "\n")
    return paths


def record_file(repo, path, before, key, manifest):
    after = (repo / path).read_bytes()
    if not after.endswith(b"\n"):
        raise ValueError("missing trailing newline")
    old, spans = blocks(before)
    new = split_lines(after.decode("utf-8"))
    ops = difflib.SequenceMatcher(None, old, new, autojunk=False).get_opcodes()
    spans = group_spans(spans, ops)
    chunks = {c["preimage"]: c for c in manifest["files"].get(path, {}).get("chunks", [])}
    patches = {}
    rebuilt = old[:]
    for lo, hi in reversed(spans):
        out = []
        for tag, a, b, c, d in ops:
            if tag == "equal":
                out.extend(old[max(a, lo):min(b, hi)])
            elif lo <= a <= b <= hi:
                out.extend(new[c:d])
        if any(line.startswith(MARKERS) for line in out):
            raise ValueError("unresolved markers")
        original = "".join(old[lo:hi])
        preimage = digest(original.encode())
        name = f"{key[:16]}-{preimage[:16]}.patch"
        patch = "".join(difflib.unified_diff(split_lines(original), out,
                                             fromfile="a/conflict", tofile="b/conflict"))
        patches[name] = patch
        chunks[preimage] = {"preimage": preimage, "patch": name,
                            "patch_hash": digest(patch.encode()),
                            "resolved": digest("".join(out).encode())}
        rebuilt[lo:hi] = out
    if "".join(rebuilt).encode() != after:
        raise ValueError("cannot isolate resolution")
    manifest["files"][path] = {"chunks": sorted(chunks.values(), key=lambda c: c["preimage"])}
    return patches


def record(repo, directory):
    pending = directory / ".pending"
    index = json.loads((pending / "index.json").read_text())
    if set(index) != set(conflicts(repo)):
        raise ValueError("index changed: resolve the files without staging them")
    manifest = load(directory)
    patches, errors = {}, []
    for path, key in index.items():
        try:
            patches.update(record_file(repo, path, (pending / key).read_bytes(), key, manifest))
        except ValueError as exc:
            errors.append(f"{exc}: {path}")
    if errors:
        raise ValueError("\n".join(errors))
    for name, patch in patches.items():
        (directory / name).write_text(patch)
    save(directory, manifest)
    shutil.rmtree(pending)
    git(repo, "add", "--", *index)
    return sorted(index)


def main():
    parser = argparse.ArgumentParser(description="Replay and record merge conflict resolutions")
    parser.add_argument("action", choices=["replay", "capture", "record"])
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
