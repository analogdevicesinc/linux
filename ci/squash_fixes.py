#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Copyright (C) 2025 Analog Devices Inc.

import sys
import re
import subprocess
import argparse


def git_rev_parse_batch(hashes):
    """Resolve a collection of short hashes to full hashes in a single git call.

    Returns a dict mapping each input token to its resolved full hash.
    Tokens that cannot be resolved are omitted from the result.
    """
    if not hashes:
        return {}

    tokens = list(hashes)
    result = subprocess.run(
        ["git", "rev-parse"] + tokens,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )

    if result.returncode == 0:
        resolved = result.stdout.strip().splitlines()
        if len(resolved) == len(tokens):
            return dict(zip(tokens, resolved))

    # Fallback: resolve one-by-one so that a single bad token does not
    # discard all the good ones.
    mapping = {}
    for token in tokens:
        r = subprocess.run(
            ["git", "rev-parse", token],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if r.returncode == 0:
            mapping[token] = r.stdout.strip()
        else:
            print(f"squash_fixes: warning: cannot resolve hash '{token}', "
                  f"line will be left as-is", file=sys.stderr)
    return mapping


def git_log_bodies(full_hashes):
    """Return a dict mapping each full hash to its commit message body.

    Uses a single `git log` call with NUL record separators to avoid
    per-commit subprocess overhead.
    """
    if not full_hashes:
        return {}

    hashes = list(full_hashes)
    # --no-walk reads each commit exactly once without traversal.
    # %x00 is NUL; we use it to split records unambiguously.
    result = subprocess.run(
        ["git", "log", "--no-walk=unsorted", "--format=%H%x00%B%x00"] + hashes,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        print(f"squash_fixes: warning: git log failed: {result.stderr.strip()}",
              file=sys.stderr)
        return {}

    bodies = {}
    # Records are separated by NUL pairs: <hash> NUL <body> NUL
    parts = result.stdout.split("\x00")
    # parts = [hash0, body0, hash1, body1, ..., trailing_empty]
    it = iter(parts)
    for full_hash in it:
        full_hash = full_hash.strip()
        body = next(it, "")
        if full_hash:
            bodies[full_hash] = body
    return bodies


def main():
    parser = argparse.ArgumentParser(
        usage='%(prog)s [--dry-run] TODO_FILE',
        description='Git sequence editor to auto-squash commits with Fixes: tags',
        epilog='''
DESCRIPTION
    Automatically reorders and squashes commits during an interactive rebase.
    Parses the rebase todo list and finds commits containing "Fixes: <hash>"
    tags in their commit messages. When the target commit is also in the
    rebase sequence, the fixing commit is moved immediately after it and
    marked for squashing.

    Commits with multiple Fixes: tags are attached to every matching target
    in the rebase sequence.  Chains of fixups (a fixup that itself is fixed
    by another commit) are resolved with BFS so that each fixup always
    appears immediately after the commit it fixes.

EXAMPLES
    Use as GIT_SEQUENCE_EDITOR:
        $ GIT_SEQUENCE_EDITOR=./squash_fixes.py git rebase -i HEAD~10

    Preview changes without modifying the todo file:
        $ ./squash_fixes.py --dry-run .git/rebase-merge/git-rebase-todo

    Given a todo list:
        pick abc1234 Add feature X
        pick def5678 Add feature Y
        pick 789abcd Fix typo in feature X

    If commit 789abcd contains "Fixes: abc1234", the result is:
        pick abc1234 Add feature X
        s 789abcd Fix typo in feature X
        pick def5678 Add feature Y
''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("todo_file", metavar="TODO_FILE",
                        help="path to the git-rebase-todo file")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the modified todo list to stdout without saving")
    args = parser.parse_args()

    with open(args.todo_file, "r") as fh:
        lines = fh.readlines()

    todo_commits = []

    # 1. Parse the todo list — collect short hashes first so we can
    #    resolve them all in a single git call.
    COMMIT_CMDS = frozenset((
        "p", "pick", "r", "reword", "e", "edit",
        "s", "squash", "f", "fixup", "d", "drop",
    ))

    pending = []   # (line_index, cmd, short_hash, subject, raw)
    for line in lines:
        if line.startswith("#") or not line.strip():
            todo_commits.append({"type": "comment", "raw": line})
            continue

        parts = line.strip().split(maxsplit=2)
        if len(parts) >= 2 and parts[0] in COMMIT_CMDS:
            cmd, short_hash = parts[0], parts[1]
            subject = parts[2] if len(parts) > 2 else ""
            pending.append((len(todo_commits), cmd, short_hash, subject, line))
            todo_commits.append(None)   # placeholder; filled in below
        else:
            todo_commits.append({"type": "unknown", "raw": line})

    # Batch-resolve all short hashes from the todo list.
    short_hashes = [p[2] for p in pending]
    hash_map = git_rev_parse_batch(short_hashes)

    for idx, cmd, short_hash, subject, raw in pending:
        if short_hash in hash_map:
            todo_commits[idx] = {
                "type":       "commit",
                "cmd":        cmd,
                "short_hash": short_hash,
                "full_hash":  hash_map[short_hash],
                "subject":    subject,
                "raw":        raw,
            }
        else:
            # git_rev_parse_batch already printed a warning for this token.
            todo_commits[idx] = {"type": "unknown", "raw": raw}

    # 2. Map full hashes present in the todo list.
    todo_full_hashes = {
        c["full_hash"]: c
        for c in todo_commits
        if c and c["type"] == "commit"
    }

    # 3. Fetch all commit bodies in one git log call, then find Fixes: tags.
    bodies = git_log_bodies(todo_full_hashes.keys())

    # target_to_fixups: full_hash_of_target -> [fix_commit, ...]
    # is_fixup: set of full hashes that fix something else in the sequence
    target_to_fixups = {}
    is_fixup = set()

    for c in todo_commits:
        if not c or c["type"] != "commit":
            continue

        body = bodies.get(c["full_hash"], "")

        # Collect all Fixes: tags in the commit message.
        fix_hashes = re.findall(r"Fixes:\s*([0-9a-fA-F]+)", body)
        if not fix_hashes:
            continue

        # Resolve all Fixes: hashes in one batch call.
        fix_map = git_rev_parse_batch(fix_hashes)

        attached = False
        for raw_hash, target_full in fix_map.items():
            # Only squash if the target is ALSO in this rebase sequence.
            if target_full not in todo_full_hashes:
                continue
            target_to_fixups.setdefault(target_full, []).append(c)
            is_fixup.add(c["full_hash"])
            attached = True

        if not attached and fix_hashes:
            print(f"squash_fixes: note: '{c['subject']}' has Fixes: tag(s) "
                  f"but none target a commit in this rebase sequence",
                  file=sys.stderr)

    # 4. Generate the new todo sequence using BFS so that fixup chains
    #    (a fixup that is itself fixed by another commit) are ordered
    #    correctly.  We walk the original todo order and, for each
    #    non-fixup commit, immediately append its fixups — then recurse
    #    into each fixup to append *its* fixups, and so on.
    new_lines = []
    emitted = set()   # full hashes already written to new_lines

    def emit(c):
        """Emit commit c and then BFS-emit all of its fixups."""
        if c["full_hash"] in emitted:
            return
        emitted.add(c["full_hash"])
        new_lines.append(c["raw"])
        for fix_commit in target_to_fixups.get(c["full_hash"], []):
            new_lines.append(f"s {fix_commit['short_hash']} {fix_commit['subject']}\n")
            emit(fix_commit)   # recurse: emit any fixups of the fixup

    for c in todo_commits:
        if not c or c["type"] != "commit":
            new_lines.append(c["raw"] if c else "")
            continue

        # Fixups are emitted when their target is processed.
        if c["full_hash"] in is_fixup:
            continue

        emit(c)

    # 5. Output.
    result_text = "".join(new_lines)

    n_fixups = len(is_fixup)
    if args.dry_run:
        print("--- DRY RUN: New git-rebase-todo ---")
        print(result_text, end="")
        print("------------------------------------")
        print(f"squash_fixes: {n_fixups} fixup(s) would be applied", file=sys.stderr)
    else:
        with open(args.todo_file, "w") as fh:
            fh.write(result_text)
        print(f"squash_fixes: {n_fixups} fixup(s) applied", file=sys.stderr)


if __name__ == "__main__":
    main()
