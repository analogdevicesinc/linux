#!/usr/bin/env python3

import copy
import json
import sys
from pathlib import Path


def materialize_locations(value, run):
    if isinstance(value, list):
        return [materialize_locations(item, run) for item in value]
    if not isinstance(value, dict):
        return value

    value = copy.deepcopy(value)
    for key, table_name, location_key in (
        ("artifactLocation", "artifacts", "location"),
    ):
        location = value.get(key)
        if not isinstance(location, dict) or "index" not in location:
            continue
        table = run.get(table_name, [])
        index = location.pop("index")
        if index >= len(table):
            raise ValueError(f"invalid {key} index {index}")
        referenced = table[index]
        if location_key is not None:
            referenced = referenced[location_key]
        value[key] = {**copy.deepcopy(referenced), **location}

    logical_locations = value.get("logicalLocations", [])
    for index, location in enumerate(logical_locations):
        if not isinstance(location, dict) or "index" not in location:
            continue
        table = run.get("logicalLocations", [])
        table_index = location.pop("index")
        if table_index >= len(table):
            raise ValueError(f"invalid logical location index {table_index}")
        logical_locations[index] = {
            **copy.deepcopy(table[table_index]),
            **location,
        }

    return {
        key: materialize_locations(item, run)
        for key, item in value.items()
    }


def merge_sarif(paths):
    documents = [json.loads(path.read_text()) for path in paths]
    runs = [run for document in documents for run in document.get("runs", [])]
    if not runs:
        raise ValueError("no SARIF runs found")

    tool_names = {run["tool"]["driver"]["name"] for run in runs}
    if len(tool_names) != 1:
        raise ValueError("cannot merge runs from different tools")

    merged = copy.deepcopy(runs[0])
    merged.pop("artifacts", None)
    merged.pop("logicalLocations", None)
    merged["invocations"] = []
    merged["results"] = []

    merged_rules = []
    rule_indexes = {}
    for run in runs:
        source_rules = run.get("tool", {}).get("driver", {}).get("rules", [])
        for rule in source_rules:
            rule_id = rule["id"]
            if rule_id not in rule_indexes:
                rule_indexes[rule_id] = len(merged_rules)
                merged_rules.append(copy.deepcopy(rule))

        merged["invocations"].extend(
            materialize_locations(invocation, run)
            for invocation in run.get("invocations", [])
        )
        for source_result in run.get("results", []):
            result = materialize_locations(source_result, run)
            rule_id = result.get("ruleId")
            if rule_id is None and "ruleIndex" in result:
                rule_id = source_rules[result["ruleIndex"]]["id"]
                result["ruleId"] = rule_id
            if rule_id in rule_indexes:
                result["ruleIndex"] = rule_indexes[rule_id]
            else:
                result.pop("ruleIndex", None)
            merged["results"].append(result)

    merged["tool"]["driver"]["rules"] = merged_rules
    return {
        "$schema": documents[0].get("$schema"),
        "version": "2.1.0",
        "runs": [merged],
    }


def main():
    input_dir, output_file = map(Path, sys.argv[1:])
    paths = sorted(input_dir.rglob("*.sarif"))
    if not paths:
        raise SystemExit(f"no SARIF files in {input_dir}")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(merge_sarif(paths), indent=2) + "\n")


if __name__ == "__main__":
    main()
