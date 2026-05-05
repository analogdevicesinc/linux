#!/usr/bin/env python3
# cat ./sbom.cdx.json | jq > ./sbom.cdx.pretty.json ; cat ./sbom.spdx.json | jq > ./sbom.spdx.pretty.json
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

from urllib.parse import quote
from os import path, sep, environ

tool_name = "linux-sbom-gen"
tool_version = "1.0.0"

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

def get_gitref(git_ref):
    for ref_pre in ['refs/tags/', 'refs/heads/', 'refs/']:
        # flattens tags and heads, for implicit branch to tag promotion.
        if git_ref.startswith(ref_pre):
            git_ref = git_ref[len(ref_pre):]
            break

    return git_ref.replace('/', '-')

def get_component(ctx):
    git_ref = get_gitref(ctx.get("git_ref", ""))

    return f"linux-{git_ref}"

def get_artifact(ctx):
    compiler_name    = ctx.get("compiler_name", "")
    defconfig        = ctx.get("kernel_defconfig")
    arch             = ctx.get("compiler_arch")

    return f"{defconfig}-{compiler_name}-{arch}"

def get_purl_src(ctx):
    git_sha = ctx.get("git_sha", "")

    return f"pkg:generic/analog/linux@{git_sha}"

def get_purl_out(ctx):
    kernel_release   = ctx.get("kernel_release", "")
    kernel           = ctx.get("kernel", "")
    arch             = ctx.get("compiler_arch", "")
    defconfig        = ctx.get("kernel_defconfig", "")
    compiler_name    = ctx.get("compiler_name", "")
    compiler_version = ctx.get("compiler_version", "")
    git_ref          = ctx.get("git_ref", "")
    git_sha          = ctx.get("git_sha", "")

    qualifiers = {
        "sha": git_sha,
        "compiler": f"{compiler_name}-{compiler_version}",
        "arch": arch,
        "config": defconfig,
        "image": kernel,
        "ref": git_ref
    }

    qualifiers_ = []
    for key, value in qualifiers.items():
        safe = quote(value, safe='')
        qualifiers_.append(f"{key}={safe}")

    qualifiers_ = '&'.join(qualifiers_)

    return f"pkg:generic/analog/linux@{kernel_release}?{qualifiers_}"

def get_name(ctx):
    return "Linux Kernel"

def get_description(ctx):
    return "The Linux kernel is the core of any Linux operating system"

def build_cdx(dist, ctx, source_files, src_root, main_c_command=None):
    """Return a CycloneDX 1.6 dict."""
    kernel_release   = ctx.get("kernel_release")
    kernel           = ctx.get("kernel")
    git_sha          = ctx.get("git_sha")
    git_sha_ct       = ctx.get("git_sha_ct")

    commit_iso = datetime.datetime.fromtimestamp(
        int(git_sha_ct), tz=datetime.timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")

    image_path = path.join(dist, "boot", kernel)

    source_components = []
    for rel in source_files:
        component = {"type": "file", "bom-ref": f"file:{rel}", "name": rel}
        lic = _spdx_license(path.join(src_root, rel))
        if lic:
            component["licenses"] = [{"expression": lic}]
        source_components.append(component)

    git_url = environ.get('GIT_URL', '')
    purl_out = get_purl_out(ctx)

    return {
        "bomFormat": "CycloneDX",
        "specVersion": "1.7",
        "serialNumber": f"urn:uuid:{uuid.uuid4()}",
        "version": 1,
        "metadata": {
            "timestamp": commit_iso,
            "tools": {
                "components": [{
                    "type":    "application",
                    "author":  "analog",
                    "name":    tool_name,
                    "version": tool_version,
                }],
            },
            "component": {
                "type": "firmware",
                "bom-ref": get_artifact(ctx),
                "name": get_name(ctx),
                "version": kernel_release,
                "description": get_description(ctx),
                "purl": purl_out,
                "licenses": [{"expression": "GPL-2.0 WITH Linux-syscall-note"}],
                **({"hashes": [
                    {"alg": "MD5",     "content": _hash(image_path, "md5")},
                    {"alg": "SHA-256", "content": _hash(image_path, "sha256")},
                ]} if _hash(image_path, "md5") else {}),
                "externalReferences": [{
                    "type": "vcs",
                    "url": git_url,
                    **({"comment": git_sha} if git_sha else {}),
                }],
                "components": source_components,
                "properties": [{
                    "name":  "hub.analog.com/component-id",
                    "value": get_component(ctx),
                }],
            },
        },
        "components": [],
        "dependencies": [],
        **({
            "formulation": [{
                "workflows": [{
                    "bom-ref":   "workflow:kbuild",
                    "uid":       "kbuild",
                    "taskTypes": ["build"],
                    "steps": [{
                        "name": "compile",
                        "commands": [{"executed": main_c_command}],
                    }],
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
    purl_src = f"urn:purl:{get_purl_src(ctx)}"
    purl_out = f"urn:purl:{get_purl_out(ctx)}"

    kernel_release = ctx.get("kernel_release")
    kernel         = ctx.get("kernel")
    arch           = ctx.get("compiler_arch")
    defconfig      = ctx.get("kernel_defconfig")
    compiler_name  = ctx.get("compiler_name")
    compiler_ver   = ctx.get("compiler_version")
    git_sha        = ctx.get("git_sha")
    git_sha_ct     = ctx.get("git_sha_ct")

    created = datetime.datetime.fromtimestamp(
        int(git_sha_ct), tz=datetime.timezone.utc
    ).strftime("%Y-%m-%dT%H:%M:%SZ")

    image_path = path.join(dist, "boot", kernel)
    git_url = environ.get('GIT_URL', '')

    def _id_out(tag):
        return f"{purl_out}/{tag}"

    def _id_src(tag):
        return f"{purl_src}/{tag}"

    id_doc     = _id_out("document")
    id_sbom    = _id_out("sbom")
    id_pkg     = _id_out("pkg/linux")
    id_image   = _id_out("file/image")
    id_build   = _id_src("build/0")
    id_tool    = _id_src(f"tool/{tool_name}")

    creation_info = {
        "type":         "CreationInfo",
        "@id":          "_:creationinfo",
        "specVersion":  "3.0.1",
        "createdUsing": [id_tool],
        "created":      created,
    }

    tool = {
        "type":         "Tool",
        "spdxId":       id_tool,
        "creationInfo": "_:creationinfo",
        "name":         tool_name,
        "summary":      f"{tool_name} {tool_version}",
    }

    # Deduplicated inline LicenseExpression nodes
    lic_expr_ids = {}
    lic_expr_elems = []

    def _lic_id(expr):
        if expr not in lic_expr_ids:
            lid = _id_src(f"lic/{len(lic_expr_ids)}")
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
        fid = _id_src(f"file/src/{idx}")
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
                "spdxId":           _id_src(f"rel/license/{idx}"),
                "creationInfo":     "_:creationinfo",
                "relationshipType": "hasDeclaredLicense",
                "from":             fid,
                "to":               [_lic_id(lic)],
            })

    package = {
        "type":             "software_Package",
        "spdxId":           id_pkg,
        "creationInfo":     "_:creationinfo",
        "name":             get_name(ctx),
        "software_packageVersion": kernel_release,
        "software_primaryPurpose": "operatingSystem",
        "description": get_description(ctx),
    }
    package["externalRef"] = [{
        "type":            "ExternalRef",
        "externalRefType": "packageManager",
        "locator":         [get_purl_src(ctx)],
    }]
    package["extension"] = [{
        "type":    "CustomExtension",
        "name":    "hub.analog.com/component-id",
        "payload": {"component_id": get_component(ctx)},
    }]
    if git_url:
        package["externalRef"].append({
            "type":            "ExternalRef",
            "externalRefType": "other",
            "locator":         [git_url],
            **({"comment": git_sha} if git_sha else {}),
        })

    contains_rel = {
        "type":             "Relationship",
        "spdxId":           _id_src("rel/contains"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "contains",
        "from":             id_pkg,
        "to":               file_ids,
        "completeness":     "incomplete",
    }

    pkg_license_rel = {
        "type":             "Relationship",
        "spdxId":           _id_src("rel/pkg-license"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "hasDeclaredLicense",
        "from":             id_pkg,
        "to":               [_lic_id("GPL-2.0-only")],
    }

    image_elem = {
        "type":             "software_File",
        "spdxId":           id_image,
        "creationInfo":     "_:creationinfo",
        "name":             kernel,
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
        "build_buildType":   "urn:analog.com:Kbuild",
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
        "spdxId":           _id_src("rel/build-input"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "hasInput",
        "from":             id_build,
        "to":               [id_pkg],
        "completeness":     "incomplete",
    }

    build_output_rel = {
        "type":             "Relationship",
        "spdxId":           _id_src("rel/build-output"),
        "creationInfo":     "_:creationinfo",
        "relationshipType": "hasOutput",
        "from":             id_build,
        "to":               [id_image],
        "completeness":     "incomplete",
    }

    all_element_ids = (
        [id_pkg, id_image, id_build,
         _id_src("rel/contains"), _id_src("rel/pkg-license"),
         _id_src("rel/build-input"), _id_src("rel/build-output")]
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

    doc_element_ids = [id_sbom, id_tool] + all_element_ids + [e["spdxId"] for e in lic_expr_elems]

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
        tool,
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
