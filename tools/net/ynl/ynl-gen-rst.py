#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
# -*- coding: utf-8; mode: python -*-

"""
    Script to auto generate the documentation for Netlink specifications.

    :copyright:  Copyright (C) 2023  Breno Leitao <leitao@debian.org>
    :license:    GPL Version 2, June 1991 see linux/COPYING for details.

    This script performs extensive parsing to the Linux kernel's netlink YAML
    spec files, in an effort to avoid needing to heavily mark up the original
    YAML file.

    This code is split in three big parts:
        1) RST formatters: Use to convert a string to a RST output
        2) Parser helpers: Functions to parse the YAML data structure
        3) Main function and small helpers
"""

from typing import Any, Dict, List
import os.path
import sys
import argparse
import logging
import yaml


SPACE_PER_LEVEL = 4


# RST Formatters
# ==============
def headroom(level: int) -> str:
    """Return space to format"""
    return " " * (level * SPACE_PER_LEVEL)


def bold(text: str) -> str:
    """Format bold text"""
    return f"**{text}**"


def inline(text: str) -> str:
    """Format inline text"""
    return f"``{text}``"


def sanitize(text: str) -> str:
    """Remove newlines and multiple spaces"""
    # This is useful for some fields that are spread across multiple lines
    return str(text).replace("\n", "").strip()


def rst_fields(key: str, value: str, level: int = 0) -> str:
    """Return a RST formatted field"""
    return headroom(level) + f":{key}: {value}"


def rst_definition(key: str, value: Any, level: int = 0) -> str:
    """Format a single rst definition"""
    return headroom(level) + key + "\n" + headroom(level + 1) + str(value)


def rst_paragraph(paragraph: str, level: int = 0) -> str:
    """Return a formatted paragraph"""
    return headroom(level) + paragraph


def rst_bullet(item: str, level: int = 0) -> str:
    """Return a formatted a bullet"""
    return headroom(level) + f"- {item}"


def rst_subsection(title: str) -> str:
    """Add a sub-section to the document"""
    return f"{title}\n" + "-" * len(title)


def rst_subsubsection(title: str) -> str:
    """Add a sub-sub-section to the document"""
    return f"{title}\n" + "~" * len(title)


def rst_section(namespace: str, prefix: str, title: str) -> str:
    """Add a section to the document"""
    return f".. _{namespace}-{prefix}-{title}:\n\n{title}\n" + "=" * len(title)


def rst_subtitle(title: str) -> str:
    """Add a subtitle to the document"""
    return "\n" + "-" * len(title) + f"\n{title}\n" + "-" * len(title) + "\n\n"


def rst_title(title: str) -> str:
    """Add a title to the document"""
    return "=" * len(title) + f"\n{title}\n" + "=" * len(title) + "\n\n"


def rst_list_inline(list_: List[str], level: int = 0) -> str:
    """Format a list using inlines"""
    return headroom(level) + "[" + ", ".join(inline(i) for i in list_) + "]"


def rst_ref(namespace: str, prefix: str, name: str) -> str:
    """Add a hyperlink to the document"""
    mappings = {'enum': 'definition',
                'fixed-header': 'definition',
                'nested-attributes': 'attribute-set',
                'struct': 'definition'}
    if prefix in mappings:
        prefix = mappings[prefix]
    return f":ref:`{namespace}-{prefix}-{name}`"


def rst_header() -> str:
    """The headers for all the auto generated RST files"""
    lines = []

    lines.append(rst_paragraph(".. SPDX-License-Identifier: GPL-2.0"))
    lines.append(rst_paragraph(".. NOTE: This document was auto-generated.\n\n"))

    return "\n".join(lines)


def rst_toctree(maxdepth: int = 2) -> str:
    """Generate a toctree RST primitive"""
    lines = []

    lines.append(".. toctree::")
    lines.append(f"   :maxdepth: {maxdepth}\n\n")

    return "\n".join(lines)


def rst_label(title: str) -> str:
    """Return a formatted label"""
    return f".. _{title}:\n\n"


# Parsers
# =======


def parse_mcast_group(mcast_group: List[Dict[str, Any]]) -> str:
    """Parse 'multicast' group list and return a formatted string"""
    lines = []
    for group in mcast_group:
        lines.append(rst_bullet(group["name"]))

    return "\n".join(lines)


def parse_do(do_dict: Dict[str, Any], level: int = 0) -> str:
    """Parse 'do' section and return a formatted string"""
    lines = []
    for key in do_dict.keys():
        lines.append(rst_paragraph(bold(key), level + 1))
        lines.append(parse_do_attributes(do_dict[key], level + 1) + "\n")

    return "\n".join(lines)


def parse_do_attributes(attrs: Dict[str, Any], level: int = 0) -> str:
    """Parse 'attributes' section"""
    if "attributes" not in attrs:
        return ""
    lines = [rst_fields("attributes", rst_list_inline(attrs["attributes"]), level + 1)]

    return "\n".join(lines)


def parse_operations(operations: List[Dict[str, Any]], namespace: str) -> str:
    """Parse operations block"""
    preprocessed = ["name", "doc", "title", "do", "dump"]
    linkable = ["fixed-header", "attribute-set"]
    lines = []

    for operation in operations:
        lines.append(rst_section(namespace, 'operation', operation["name"]))
        lines.append(rst_paragraph(sanitize(operation["doc"])) + "\n")

        for key in operation.keys():
            if key in preprocessed:
                # Skip the special fields
                continue
            value = operation[key]
            if key in linkable:
                value = rst_ref(namespace, key, value)
            lines.append(rst_fields(key, value, 0))

        if "do" in operation:
            lines.append(rst_paragraph(":do:", 0))
            lines.append(parse_do(operation["do"], 0))
        if "dump" in operation:
            lines.append(rst_paragraph(":dump:", 0))
            lines.append(parse_do(operation["dump"], 0))

        # New line after fields
        lines.append("\n")

    return "\n".join(lines)


def parse_entries(entries: List[Dict[str, Any]], level: int) -> str:
    """Parse a list of entries"""
    ignored = ["pad"]
    lines = []
    for entry in entries:
        if isinstance(entry, dict):
            # entries could be a list or a dictionary
            field_name = entry.get("name", "")
            if field_name in ignored:
                continue
            type_ = entry.get("type")
            if type_:
                field_name += f" ({inline(type_)})"
            lines.append(
                rst_fields(field_name, sanitize(entry.get("doc", "")), level)
            )
        elif isinstance(entry, list):
            lines.append(rst_list_inline(entry, level))
        else:
            lines.append(rst_bullet(inline(sanitize(entry)), level))

    lines.append("\n")
    return "\n".join(lines)


def parse_definitions(defs: Dict[str, Any], namespace: str) -> str:
    """Parse definitions section"""
    preprocessed = ["name", "entries", "members"]
    ignored = ["render-max"]  # This is not printed
    lines = []

    for definition in defs:
        lines.append(rst_section(namespace, 'definition', definition["name"]))
        for k in definition.keys():
            if k in preprocessed + ignored:
                continue
            lines.append(rst_fields(k, sanitize(definition[k]), 0))

        # Field list needs to finish with a new line
        lines.append("\n")
        if "entries" in definition:
            lines.append(rst_paragraph(":entries:", 0))
            lines.append(parse_entries(definition["entries"], 1))
        if "members" in definition:
            lines.append(rst_paragraph(":members:", 0))
            lines.append(parse_entries(definition["members"], 1))

    return "\n".join(lines)


def parse_attr_sets(entries: List[Dict[str, Any]], namespace: str) -> str:
    """Parse attribute from attribute-set"""
    preprocessed = ["name", "type"]
    linkable = ["enum", "nested-attributes", "struct", "sub-message"]
    ignored = ["checks"]
    lines = []

    for entry in entries:
        lines.append(rst_section(namespace, 'attribute-set', entry["name"]))
        for attr in entry["attributes"]:
            type_ = attr.get("type")
            attr_line = attr["name"]
            if type_:
                # Add the attribute type in the same line
                attr_line += f" ({inline(type_)})"

            lines.append(rst_subsubsection(attr_line))

            for k in attr.keys():
                if k in preprocessed + ignored:
                    continue
                if k in linkable:
                    value = rst_ref(namespace, k, attr[k])
                else:
                    value = sanitize(attr[k])
                lines.append(rst_fields(k, value, 0))
            lines.append("\n")

    return "\n".join(lines)


def parse_sub_messages(entries: List[Dict[str, Any]], namespace: str) -> str:
    """Parse sub-message definitions"""
    lines = []

    for entry in entries:
        lines.append(rst_section(namespace, 'sub-message', entry["name"]))
        for fmt in entry["formats"]:
            value = fmt["value"]

            lines.append(rst_bullet(bold(value)))
            for attr in ['fixed-header', 'attribute-set']:
                if attr in fmt:
                    lines.append(rst_fields(attr,
                                            rst_ref(namespace, attr, fmt[attr]),
                                            1))
            lines.append("\n")

    return "\n".join(lines)


def parse_yaml(obj: Dict[str, Any]) -> str:
    """Format the whole YAML into a RST string"""
    lines = []

    # Main header

    lines.append(rst_header())

    family = obj['name']

    title = f"Family ``{family}`` netlink specification"
    lines.append(rst_title(title))
    lines.append(rst_paragraph(".. contents:: :depth: 3\n"))

    if "doc" in obj:
        lines.append(rst_subtitle("Summary"))
        lines.append(rst_paragraph(obj["doc"], 0))

    # Operations
    if "operations" in obj:
        lines.append(rst_subtitle("Operations"))
        lines.append(parse_operations(obj["operations"]["list"], family))

    # Multicast groups
    if "mcast-groups" in obj:
        lines.append(rst_subtitle("Multicast groups"))
        lines.append(parse_mcast_group(obj["mcast-groups"]["list"]))

    # Definitions
    if "definitions" in obj:
        lines.append(rst_subtitle("Definitions"))
        lines.append(parse_definitions(obj["definitions"], family))

    # Attributes set
    if "attribute-sets" in obj:
        lines.append(rst_subtitle("Attribute sets"))
        lines.append(parse_attr_sets(obj["attribute-sets"], family))

    # Sub-messages
    if "sub-messages" in obj:
        lines.append(rst_subtitle("Sub-messages"))
        lines.append(parse_sub_messages(obj["sub-messages"], family))

    return "\n".join(lines)


# Main functions
# ==============


def parse_arguments() -> argparse.Namespace:
    """Parse arguments from user"""
    parser = argparse.ArgumentParser(description="Netlink RST generator")

    parser.add_argument("-v", "--verbose", action="store_true")
    parser.add_argument("-o", "--output", help="Output file name")

    # Index and input are mutually exclusive
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "-x", "--index", action="store_true", help="Generate the index page"
    )
    group.add_argument("-i", "--input", help="YAML file name")

    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    if args.input and not os.path.isfile(args.input):
        logging.warning("%s is not a valid file.", args.input)
        sys.exit(-1)

    if not args.output:
        logging.error("No output file specified.")
        sys.exit(-1)

    if os.path.isfile(args.output):
        logging.debug("%s already exists. Overwriting it.", args.output)

    return args


def parse_yaml_file(filename: str) -> str:
    """Transform the YAML specified by filename into a rst-formmated string"""
    with open(filename, "r", encoding="utf-8") as spec_file:
        yaml_data = yaml.safe_load(spec_file)
        content = parse_yaml(yaml_data)

    return content


def write_to_rstfile(content: str, filename: str) -> None:
    """Write the generated content into an RST file"""
    logging.debug("Saving RST file to %s", filename)

    with open(filename, "w", encoding="utf-8") as rst_file:
        rst_file.write(content)


def generate_main_index_rst(output: str) -> None:
    """Generate the `networking_spec/index` content and write to the file"""
    lines = []

    lines.append(rst_header())
    lines.append(rst_label("specs"))
    lines.append(rst_title("Netlink Family Specifications"))
    lines.append(rst_toctree(1))

    index_dir = os.path.dirname(output)
    logging.debug("Looking for .rst files in %s", index_dir)
    for filename in sorted(os.listdir(index_dir)):
        if not filename.endswith(".rst") or filename == "index.rst":
            continue
        lines.append(f"   {filename.replace('.rst', '')}\n")

    logging.debug("Writing an index file at %s", output)
    write_to_rstfile("".join(lines), output)


def main() -> None:
    """Main function that reads the YAML files and generates the RST files"""

    args = parse_arguments()

    if args.input:
        logging.debug("Parsing %s", args.input)
        try:
            content = parse_yaml_file(os.path.join(args.input))
        except Exception as exception:
            logging.warning("Failed to parse %s.", args.input)
            logging.warning(exception)
            sys.exit(-1)

        write_to_rstfile(content, args.output)

    if args.index:
        # Generate the index RST file
        generate_main_index_rst(args.output)


if __name__ == "__main__":
    main()
