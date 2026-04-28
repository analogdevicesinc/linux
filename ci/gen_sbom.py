#!/usr/bin/env python3
"""
Generate (tiny) SBOMs for a Linux kernel image.
  - CycloneDX 1.6 (sbom.cdx.json)
  - SPDX 3.0.1 (sbom.spdx.json)


Expects inside DIST path:
  compile_commands.json, context.txt, ...
"""

import datetime
import hashlib
import json
import re
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


def _load_context(ctx_path):
    ctx = {}
    with open(ctx_path) as f:
        for line in f:
            if "=" in line:
                k, _, v = line.strip().partition("=")
                ctx[k] = v
    return ctx


def _spdx_license(filepath):
    if not path.isfile(filepath):
        return None

    with open(filepath, errors="replace") as f:
        head = f.read(1024)
    m = re.search(r"SPDX-License-Identifier:\s*(.+?)(?:\s*\*/|\s*$)", head)
    if m:
        return m.group(1).strip()
    return None

def build_cdx(dist, ctx, source_files, src_root, main_c_command=None):
    """Return a CycloneDX 1.6 dict."""
    kernel_release = ctx.get("kernel_release")
    kernel_image   = ctx.get("kernel")
    arch           = ctx.get("compiler_arch")
    defconfig      = ctx.get("kernel_defconfig")
    compiler_name  = ctx.get("compiler_name")
    git_sha        = ctx.get("git_sha")
    git_sha_ct     = ctx.get("git_sha_ct")

    commit_iso = datetime.datetime.fromtimestamp(
        int(git_sha_ct), tz=datetime.timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")

    image_ref  = f"linux-{defconfig}-{compiler_name}-{arch}-{git_sha}"
    image_path = path.join(dist, "boot", kernel_image)

    source_components = []
    for rel in source_files:
        component = {"type": "file", "bom-ref": f"file:{rel}", "name": rel}
        lic = _spdx_license(path.join(src_root, rel))
        if lic:
            component["licenses"] = [{"expression": lic}]
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
                "description": (
                    f"Linux Kernel {kernel_image} and modules for defconfig"
                    f" {defconfig} arch {arch}, commit {git_sha}"
                    f" based on {kernel_release}"
                ),
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
        **({
            "formulation": [{
                "bom-ref": "workflow:kbuild",
                "uid":      "kbuild",
                "taskTypes": ["build"],
                "steps": [{
                    "name": "compile",
                    "commands": [{"executed": main_c_command}],
                }],
            }],
        } if main_c_command else {}),
    }


def build_spdx(dist, ctx, source_files, src_root, main_c_command=None):
    """Return an SPDX 3.0.1 JSON-LD dict.

    Profiles: Core, Software, SimpleLicensing, Build (light).

    Structure:
      SpdxDocument
        └─ software_Sbom  (rootElement)
             ├─ software_Package  (the kernel)
             │    └─ contains [software_File, ...]
             ├─ software_File     (the output Image)
             ├─ build_Build       (build provenance)
             │    ├─ hasInput  [software_Package]
             │    └─ hasOutput [software_File (Image)]
             └─ Relationship: hasDeclaredLicense per file
    """
    doc_uuid   = str(uuid.uuid4())
    ns         = f"urn:spdx.dev:{doc_uuid}"

    kernel_release = ctx.get("kernel_release")
    kernel_image   = ctx.get("kernel")
    arch           = ctx.get("compiler_arch")
    defconfig      = ctx.get("kernel_defconfig")
    compiler_name  = ctx.get("compiler_name")
    compiler_ver   = ctx.get("compiler_version")
    git_sha        = ctx.get("git_sha")
    git_sha_ct     = ctx.get("git_sha_ct")

    created = datetime.datetime.fromtimestamp(
        int(git_sha_ct), tz=datetime.timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")

    image_path = path.join(dist, "boot", kernel_image)
    github_repository = environ.get("GITHUB_REPOSITORY", "")

    # Compact id helpers
    def _id(tag):
        return f"{ns}/{tag}"

    id_doc     = _id("document")
    id_sbom    = _id("sbom")
    id_pkg     = _id("pkg/linux")
    id_image   = _id("file/image")
    id_build   = _id("build/0")
    id_agent   = _id("agent/gen_sbom")

    creation_info = {
        "type":        "CreationInfo",
        "@id":         "_:creationinfo",
        "specVersion": "3.0.1",
        "createdBy":   [id_agent],
        "created":     created,
    }

    agent = {
        "type":         "SoftwareAgent",
        "spdxId":       id_agent,
        "creationInfo": "_:creationinfo",
        "name":         "gen_sbom.py",
    }

    # Deduplicated inline LicenseExpression nodes
    lic_expr_ids = {}
    lic_expr_elems = []

    def _lic_id(expr):
        if expr not in lic_expr_ids:
            lid = _id(f"lic/{len(lic_expr_ids)}")
            lic_expr_ids[expr] = lid
            lic_expr_elems.append({
                "type":                              "simplelicensing_LicenseExpression",
                "spdxId":                            lid,
                "creationInfo":                      "_:creationinfo",
                "simplelicensing_licenseExpression": expr,
            })
        return lic_expr_ids[expr]

    # Source file elements and relationships
    file_elements = []
    license_rels  = []
    file_ids      = []

    for idx, rel in enumerate(source_files):
        fid = _id(f"file/src/{idx}")
        file_ids.append(fid)

        file_elements.append({
            "type":                    "software_File",
            "spdxId":                  fid,
            "creationInfo":            "_:creationinfo",
            "name":                    rel,
            "software_primaryPurpose": "source",
        })

        lic = _spdx_license(path.join(src_root, rel))
        if lic:
            license_rels.append({
                "type":             "Relationship",
                "spdxId":           _id(f"rel/license/{idx}"),
                "creationInfo":     "_:creationinfo",
                "relationshipType": "hasDeclaredLicense",
                "from":             fid,
                "to":               [_lic_id(lic)],
            })

    package = {
        "type":             "software_Package",
        "spdxId":           id_pkg,
        "creationInfo":     "_:creationinfo",
        "name":             f"linux-{defconfig}-{arch}",
        "software_packageVersion": kernel_release,
        "software_primaryPurpose": "operatingSystem",
        "description":      (
            f"Linux Kernel for defconfig {defconfig} arch {arch},"
            f" commit {git_sha}"
        ),
    }
    if github_repository:
        package["externalRef"] = [{
            "type":           "ExternalRef",
            "externalRefType": "other",
            "locator":        [f"https://github.com/{github_repository}"],
            **({"comment": f"commit {git_sha}"} if git_sha else {}),
        }]

    contains_rel = {
        "type":             "Relationship",
        "spdxId":           _id("rel/contains"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "contains",
        "from":             id_pkg,
        "to":               file_ids,
        "completeness":     "incomplete",
    }

    pkg_license_rel = {
        "type":             "Relationship",
        "spdxId":           _id("rel/pkg-license"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "hasDeclaredLicense",
        "from":             id_pkg,
        "to":               [_lic_id("GPL-2.0-only")],
    }

    image_elem = {
        "type":             "software_File",
        "spdxId":           id_image,
        "creationInfo":     "_:creationinfo",
        "name":             kernel_image,
        "software_fileKind": "file",
        "software_primaryPurpose": "executable",
    }
    image_sha = _hash(image_path, "sha256")
    if image_sha:
        image_elem["verifiedUsing"] = [{
            "type":      "Hash",
            "algorithm": "sha256",
            "hashValue": image_sha,
        }]

    build_elem = {
        "type":              "build_Build",
        "spdxId":            id_build,
        "creationInfo":      "_:creationinfo",
        "build_buildType":   "urn:spdx.dev:Kbuild",
    }
    if compiler_name and compiler_ver:
        build_elem["build_environment"] = [{
            "type":  "DictionaryEntry",
            "key":   "compiler",
            "value": f"{compiler_name}-{compiler_ver}",
        }]
    if defconfig:
        build_elem["build_configSourceEntrypoint"] = [defconfig]
    if main_c_command:
        build_elem["build_parameter"] = [{
            "type":  "DictionaryEntry",
            "key":   "command",
            "value": main_c_command,
        }]

    build_input_rel = {
        "type":             "Relationship",
        "spdxId":           _id("rel/build-input"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "hasInput",
        "from":             id_build,
        "to":               [id_pkg],
        "completeness":     "incomplete",
    }

    build_output_rel = {
        "type":             "Relationship",
        "spdxId":           _id("rel/build-output"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "hasOutput",
        "from":             id_build,
        "to":               [id_image],
        "completeness":     "incomplete",
    }

    all_element_ids = (
        [id_pkg, id_image, id_build,
         _id("rel/contains"), _id("rel/pkg-license"),
         _id("rel/build-input"), _id("rel/build-output")]
        + file_ids
        + [r["spdxId"] for r in license_rels]
    )

    sbom_elem = {
        "type":             "software_Sbom",
        "spdxId":           id_sbom,
        "creationInfo":     "_:creationinfo",
        "name":             f"Linux Kernel SBOM {defconfig} {compiler_name} {arch}",
        "rootElement":      [id_pkg],
        "element":          all_element_ids,
        "software_sbomType": ["source"],
    }

    doc_element_ids = [id_sbom, id_agent] + all_element_ids + [e["spdxId"] for e in lic_expr_elems]

    document = {
        "type":               "SpdxDocument",
        "spdxId":             id_doc,
        "creationInfo":       "_:creationinfo",
        "rootElement":        [id_sbom],
        "element":            doc_element_ids,
        "profileConformance": ["core", "software", "simpleLicensing", "build"],
    }

    graph = [
        creation_info,
        document,
        agent,
        sbom_elem,
        package,
        pkg_license_rel,
        contains_rel,
        image_elem,
        build_elem,
        build_input_rel,
        build_output_rel,
    ] + file_elements + license_rels + lic_expr_elems

    return {
        "@context": "https://spdx.org/rdf/3.0.1/spdx-context.jsonld",
        "@graph":   graph,
    }


def main():
    dist = environ.get('DIST', None)
    if dist is None:
        print("error: Env variable 'DIST' must be set", file=sys.stderr)
        sys.exit(1)

    ctx = _load_context(path.join(dist, "context.txt"))

    with open(path.join(dist, "compile_commands.json")) as f:
        db = json.load(f)

    src_root = None
    main_c_command = None
    for entry in db:
        fp = entry["file"]
        if fp.endswith(sep + "init" + sep + "main.c"):
            src_root = fp[:-len("init" + sep + "main.c")]
            main_c_command = entry.get("command")
            break
    if src_root is None:
        print("error: cannot find init/main.c in compile_commands.json",
              file=sys.stderr)
        sys.exit(1)

    source_files = set()
    for entry in db:
        fp = entry["file"]
        if not fp.startswith(src_root):
            continue # skip generated
        rel = fp[len(src_root):]
        source_files.add(rel)

    cdx_path = path.join(dist, "sbom.cdx.json")
    with open(cdx_path, "w") as f:
        json.dump(build_cdx(dist, ctx, source_files, src_root, main_c_command), f)
    print(f"sbom written to {cdx_path}", file=sys.stderr)

    spdx_path = path.join(dist, "sbom.spdx.json")
    with open(spdx_path, "w") as f:
        json.dump(build_spdx(dist, ctx, source_files, src_root, main_c_command), f)
    print(f"sbom written to {spdx_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
