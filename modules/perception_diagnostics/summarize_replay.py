#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Independently verify replay coverage and freeze artifacts, not accuracy."""

import argparse
import csv
import hashlib
import json
import statistics
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(data)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.input / "manifest.json").read_text())
    arguments = (args.results / "arguments.txt").read_text().splitlines()
    rows = list(csv.DictReader((args.results / "summary.csv").open()))
    expected = {
        (sample["file"], str(mode))
        for sample in manifest["samples"]
        for mode in range(4)
    }
    actual = {(row["frame"], row["mode"]) for row in rows}
    if actual != expected or len(rows) != len(expected):
        raise ValueError("incomplete or duplicate frame/mode coverage")
    if any(int(row["free_cells"]) for row in rows):
        raise ValueError("recorded approximate visibility must not emit FREE")
    identities = {}
    for sample in manifest["samples"]:
        cloud = args.input / sample["file"]
        if digest(cloud) != sample["sha256"]:
            raise ValueError(f"input changed: {cloud}")
        for suffix in (".fusion.pb", ".det.pb", ".seg.pb"):
            artifact = args.results / (sample["file"] + suffix)
            identities[str(artifact)] = digest(artifact)
        plot = args.results / (sample["file"] + ".svg")
        if plot.exists():
            ET.parse(plot)
            identities[str(plot)] = digest(plot)
    for name in ("config.pb", "arguments.txt", "summary.csv"):
        artifact = args.results / name
        identities[str(artifact)] = digest(artifact)
    paths = [
        args.input / "manifest.json",
        args.input / "fusion.pb.txt",
        Path(arguments[0]),
        Path(arguments[4]),
    ]
    if len(arguments) == 7:
        paths.append(Path(arguments[3]))
    for path in paths:
        identities[str(path)] = digest(path)
    source = {}
    for root in (Path("modules/lidar_fusion"), Path(__file__).parent):
        for path in root.rglob("*"):
            if path.is_file() and (
                path.suffix in (".cc", ".h", ".proto", ".py") or path.name == "BUILD"
            ):
                source[str(path)] = digest(path)
    summaries = {}
    for mode in range(4):
        selected = [row for row in rows if int(row["mode"]) == mode]
        summaries[str(mode)] = {
            "frames": len(selected),
            "totals": {
                field: sum(int(row[field]) for row in selected)
                for field in (
                    "objects",
                    "unknowns",
                    "conflicts",
                    "free_cells",
                    "repairs",
                )
            },
            "median_ms": {
                field: statistics.median(float(row[field]) for row in selected)
                for field in ("det_ms", "seg_ms", "fusion_ms")
            },
        }
    report = {
        "scope": f"{len(manifest['samples'])}/{manifest['total_clouds']} clouds",
        "coverage": "four modes and wrong-sweep injection per sampled frame",
        "base_frame": manifest["output_frame"],
        "actual_tf_parent": manifest["calibration_parent"],
        "arguments": arguments,
        "repository_head": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True
        ).strip(),
        "module_source_sha256": source,
        "artifact_sha256": identities,
        "modes": summaries,
        "not_proven": [
            "model accuracy: no labels; normalized intensity is diagnostic",
            "expected ground z=0 is not an independently verified ground reference",
            "expanded engine equivalence: changed fixture predictions observed",
            "real-time latency: CPU reference, diagnostic rays and four modes",
            "FREE: uncompensated per-ray origins and vehicle limits unavailable",
            "tracking/static accumulation: no verified continuous local ODOM",
            "downstream closed-loop/road safety",
        ],
    }
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps({"scope": report["scope"], "modes": summaries}, indent=2))


if __name__ == "__main__":
    main()
