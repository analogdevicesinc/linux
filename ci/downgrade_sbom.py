#!/usr/bin/env python3
"""
Downgrade SPDX 3.0.1 JSON-LD -> SPDX 2.3 JSON for a Linux kernel SBOM.
"""

import hashlib
import json
import re
import sys
from os import path, environ


def _ref(urn):
    """Turn a 3.0.1 spdxId URN into a legal SPDXRef-<tag>."""
    tag = urn.rsplit("/", 2)
    tag = "-".join(tag[-2:]) if len(tag) >= 2 else tag[0]
    tag = re.sub(r"[^a-zA-Z0-9.\-]", "-", tag)
    tag = re.sub(r"-{2,}", "-", tag).strip("-")
    return f"SPDXRef-{tag}"


def downgrade(spdx3, dist):
    elems = {e["spdxId"]: e for e in spdx3["@graph"] if "spdxId" in e}
    by_type = {}
    for e in elems.values():
        by_type.setdefault(e["type"], []).append(e)

    first = lambda t: by_type.get(t, [None])[0]

    creation = next(e for e in spdx3["@graph"] if e.get("type") == "CreationInfo")
    document = first("SpdxDocument")
    org      = first("Organization")
    tool     = first("Tool")
    sbom     = first("software_Sbom")
    package  = first("software_Package")
    build    = first("build_Build")

    lic_map = {e["spdxId"]: e.get("simplelicensing_licenseExpression", "NOASSERTION")
               for e in by_type.get("simplelicensing_LicenseExpression", [])}

    lic_decl = {}
    rels = []
    for r in by_type.get("Relationship", []):
        if r["relationshipType"] == "hasDeclaredLicense":
            lic_decl.setdefault(r["from"], []).extend(
                lic_map.get(t, "NOASSERTION") for t in r.get("to", []))
        else:
            rels.append(r)

    skip = {e["spdxId"] for e in (org, tool, sbom) if e}

    ref = _ref  # alias

    creators = []
    if tool:
        creators.append(f"Tool: {tool.get('summary') or tool.get('name', '')}")
    if org:
        label = org.get("name", "")
        emails = [ei["identifier"] for ei in org.get("externalIdentifier", [])
                  if ei.get("externalIdentifierType") == "email"]
        if emails:
            label += f" ({emails[0]})"
        creators.append(f"Organization: {label}")

    doc_ns = document["spdxId"]
    if doc_ns.endswith("/document"):
        doc_ns = doc_ns[:-len("/document")]

    pkg_lic = " AND ".join(lic_decl.get(package["spdxId"], [])) or "NOASSERTION"

    purl = next((ei["identifier"] for ei in package.get("externalIdentifier", [])
                 if ei.get("externalIdentifierType") == "packageUrl"), None)

    vcs_url, vcs_sha = "NOASSERTION", None
    for er in package.get("externalRef", []):
        if er.get("externalRefType") == "vcs":
            locs = er.get("locator", [])
            if locs:
                vcs_url = locs[0]
            vcs_sha = er.get("comment")
            break

    ext_refs = []
    if purl:
        ext_refs.append({"referenceCategory": "PACKAGE-MANAGER",
                         "referenceType": "purl", "referenceLocator": purl})
    if vcs_sha:
        ext_refs.append({"referenceCategory": "OTHER",
                         "referenceType": "vcs-revision", "referenceLocator": vcs_sha})
    for ext in package.get("extension", []):
        for prop in ext.get("extension_cdxProperty", []):
            n, v = prop.get("extension_cdxPropName", ""), prop.get("extension_cdxPropValue", "")
            if n and v:
                ext_refs.append({"referenceCategory": "OTHER",
                                 "referenceType": n, "referenceLocator": v})

    pkg_entry = {
        "SPDXID":           ref(package["spdxId"]),
        "name":             package.get("name", ""),
        "versionInfo":      package.get("software_packageVersion", ""),
        "downloadLocation": vcs_url,
        "filesAnalyzed":    True,
        "description":      package.get("description", ""),
        "licenseConcluded": "NOASSERTION",
        "licenseDeclared":  pkg_lic,
        "copyrightText":    "NOASSERTION",
        **({"externalRefs": ext_refs} if ext_refs else {}),
    }

    build_pkgs = []
    if build:
        env = {e["key"]: e["value"] for e in build.get("build_environment", [])
               if "key" in e}
        compiler = env.get("compiler", "")
        defconfigs = build.get("build_configSourceEntrypoint", [])
        params = {e["key"]: e["value"] for e in build.get("build_parameter", [])
                  if "key" in e}

        comment = "; ".join(filter(None, [
            f"buildType={build.get('build_buildType', '')}",
            f"defconfig={defconfigs[0]}" if defconfigs else "",
            f"compiler={compiler}" if compiler else "",
        ]))

        bpkg = {
            "SPDXID":           ref(build["spdxId"]),
            "name":             "kbuild",
            "versionInfo":      compiler,
            "downloadLocation": "NOASSERTION",
            "filesAnalyzed":    False,
            "licenseConcluded": "NOASSERTION",
            "licenseDeclared":  "NOASSERTION",
            "copyrightText":    "NOASSERTION",
            "comment":          comment,
        }
        cmd = params.get("command", "")
        if cmd:
            bpkg["sourceInfo"] = cmd[:512] + ("..." if len(cmd) > 512 else "")
        build_pkgs.append(bpkg)

    def _sha1(filepath):
        if not path.isfile(filepath):
            return None
        h = hashlib.sha1()
        with open(filepath, "rb") as fh:
            for chunk in iter(lambda: fh.read(65536), b""):
                h.update(chunk)
        return h.hexdigest()

    def _file_entry(f, resolve_path):
        lics = lic_decl.get(f["spdxId"], []) or ["NOASSERTION"]
        entry = {
            "SPDXID":             ref(f["spdxId"]),
            "fileName":           f.get("name", ""),
            "licenseConcluded":   "NOASSERTION",
            "licenseInfoInFiles": lics,
            "copyrightText":      "NOASSERTION",
        }
        checksums = []
        for vu in f.get("verifiedUsing", []):
            alg = re.sub(r"[^a-zA-Z0-9]", "", vu.get("algorithm", "")).upper()
            val = vu.get("hashValue", "")
            if alg and val:
                checksums.append({"algorithm": alg, "checksumValue": val})
        if not any(c["algorithm"] == "SHA1" for c in checksums):
            sha1 = _sha1(resolve_path)
            if sha1:
                checksums.insert(0, {"algorithm": "SHA1", "checksumValue": sha1})
        if checksums:
            entry["checksums"] = checksums
        return entry

    all_files = by_type.get("software_File", [])
    image_files = [f for f in all_files if f.get("software_primaryPurpose") == "executable"]
    src_files   = [f for f in all_files if f not in image_files]
    file_entries = ([_file_entry(f, path.join(dist, "boot", f.get("name", "")))
                     for f in image_files] +
                    [_file_entry(f, f.get("name", ""))
                     for f in src_files])

    file_sha1s = sorted(
        c["checksumValue"]
        for fe in file_entries
        for c in fe.get("checksums", [])
        if c["algorithm"] == "SHA1"
    )
    if file_sha1s:
        pkg_entry["packageVerificationCode"] = {
            "packageVerificationCodeValue":
                hashlib.sha1("".join(file_sha1s).encode()).hexdigest(),
        }

    rel_entries = [{
        "spdxElementId":      "SPDXRef-DOCUMENT",
        "relationshipType":   "DESCRIBES",
        "relatedSpdxElement": ref(package["spdxId"]),
    }]

    for r in rels:
        from_urn = r["from"]
        to_urns  = [t for t in r.get("to", []) if t not in skip]
        if from_urn in skip or not to_urns:
            continue

        rtype = r["relationshipType"]
        if rtype == "contains":
            incomplete = r.get("completeness") == "incomplete"
            for t in to_urns:
                e = {"spdxElementId": ref(from_urn), "relationshipType": "CONTAINS",
                     "relatedSpdxElement": ref(t)}
                if incomplete:
                    e["comment"] = "completeness=incomplete"
                rel_entries.append(e)
        elif rtype == "hasInput":
            for t in to_urns:
                rel_entries.append({"spdxElementId": ref(t),
                                    "relationshipType": "BUILD_DEPENDENCY_OF",
                                    "relatedSpdxElement": ref(from_urn)})
        elif rtype == "hasOutput":
            for t in to_urns:
                rel_entries.append({"spdxElementId": ref(t),
                                    "relationshipType": "GENERATED_FROM",
                                    "relatedSpdxElement": ref(from_urn)})

    return {
        "SPDXID":            "SPDXRef-DOCUMENT",
        "spdxVersion":       "SPDX-2.3",
        "dataLicense":       "CC0-1.0",
        "name":              sbom.get("name", "Linux Kernel SBOM") if sbom else "Linux Kernel SBOM",
        "documentNamespace": doc_ns,
        "creationInfo":      {"created": creation.get("created", ""), "creators": creators},
        "packages":          [pkg_entry] + build_pkgs,
        "files":             file_entries,
        "relationships":     rel_entries,
    }


def main():
    dist = environ.get("DIST")
    if not dist:
        print("error: DIST env variable must be set", file=sys.stderr)
        sys.exit(1)

    in_path  = path.join(dist, "sbom.spdx30.json")
    out_path = path.join(dist, "sbom.spdx23.json")

    with open(in_path) as f:
        spdx3 = json.load(f)

    with open(out_path, "w") as f:
        json.dump(downgrade(spdx3, dist), f, indent=2)

    print(f"sbom written to {out_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
