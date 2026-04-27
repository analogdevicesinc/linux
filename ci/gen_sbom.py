#!/usr/bin/env python3
"""
Generate a (tiny) CycloneDX 1.6 SBOM for a Linux kernel image.

Expects inside DIST path:
  compile_commands.json, context.txt, ...
"""

import datetime
import hashlib
import json
import sys
import uuid

from os import path, sep, environ

def _hash(filepath, alg):
    if not path.isfile(filepath):
        return None

    h = hashlib.new(alg)
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _load_context(path):
    ctx = {}
    with open(path) as f:
        for line in f:
            if "=" in line:
                k, _, v = line.strip().partition("=")
                ctx[k] = v
    return ctx


def build_bom(dist):
    ctx = _load_context(path.join(dist, "context.txt"))

    kernel_release = ctx.get("kernel_release")
    kernel_image   = ctx.get("kernel")
    arch           = ctx.get("compiler_arch")
    defconfig      = ctx.get("kernel_defconfig")
    compiler_name  = ctx.get("compiler_name")
    compiler_ver   = ctx.get("compiler_version")
    git_sha        = ctx.get("git_sha")
    git_sha_ct     = ctx.get("git_sha_ct")

    commit_iso = datetime.datetime.fromtimestamp(
        int(git_sha_ct), tz=datetime.timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")

    image_ref  = f"linux-{defconfig}-{compiler_name}-{arch}-{git_sha}"
    image_path = path.join(dist, "boot", kernel_image)

    with open(path.join(dist, "compile_commands.json")) as f:
        db = json.load(f)

    cwd = path.abspath(".") + sep
    source_components = []
    for entry in db:
        fp = entry["file"]
        if not fp.startswith(cwd):
            # skip templated .c files
            continue
        rel = fp[len(cwd):]
        component = {"type": "file", "bom-ref": f"file:{rel}", "name": rel}
        h = _hash(fp, "sha256")
        if h:
            component["hashes"] = [{"alg": "SHA-256", "content": h}]
        source_components.append(component)

    github_repository = environ.get('GITHUB_REPOSITORY', '')

    return {
        "bomFormat": "CycloneDX",
        "specVersion": "1.6",
        "serialNumber": f"urn:uuid:{uuid.uuid4()}",
        "version": 1,
        "metadata": {
            "timestamp": commit_iso,
            "component": {
                "type": "firmware",
                "bom-ref": image_ref,
                "name": f"Linux Kernel {kernel_image} {defconfig} {compiler_name} {arch}",
                "version": kernel_release,
                "description": f"Linux Kernel {kernel_image} and modules for defconfig {defconfig} arch {arch}, commit {git_sha} based on {kernel_release}",
                "licenses": [{"expression": "GPL-2.0 WITH Linux-syscall-note"}],
                **({"hashes": [
                    {"alg": "MD5",     "content": _hash(image_path, "md5")},
                    {"alg": "SHA-256", "content": _hash(image_path, "sha256")},
                ]} if _hash(image_path, "md5") else {}),
                "externalReferences": [{
                    "type": "vcs",
                    "url": f"https://github.com/{github_repository}",
                    **({"comment": f"commit {git_sha}"} if git_sha else {}),
                }],
                "components": source_components,
            },
        },
        "components": [],
        "dependencies": [],
    }


def main():
    dist = environ.get('DIST', None)
    if dist is None:
        print(f"error: Env variable 'DIST' must be set", file=sys.stderr)
        sys.exit(1)

    output = path.join(dist, "sbom.cdx.json")
    with open(output, "w") as f:
        json.dump(build_bom(dist), f)
    print(f"sbom written to {output}", file=sys.stderr)


if __name__ == "__main__":
    main()
