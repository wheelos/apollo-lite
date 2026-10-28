#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Prepare explicitly diagnostic semantic-GT substitutions, not model predictions."""

import argparse
import json
from pathlib import Path

import numpy as np
from evaluate_baseline import digest, require
from proto import Reader
from semantickitti import learning_labels

PREFIX = "apollo.lidar_semantic_segmentation."


def substitute(message, packed):
    """Replace valid GT points only; ignored truth retains original prediction."""
    truth = learning_labels(packed)
    require(len(message.point_label) == len(truth), "oracle point count mismatch")
    indices = [p.point_index for p in message.point_label]
    require(set(indices) == set(range(len(truth))), "oracle point index mismatch")
    count = 0
    for label in message.point_label:
        mapped = int(truth[label.point_index])
        if mapped:
            label.semantic_label = mapped
            label.confidence = 1.0
            count += 1
    return count


def prepare(inputs, models, schema, output):
    output.mkdir(parents=True, exist_ok=False)
    manifest_path = inputs / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    reader = Reader(schema)
    rows = []
    for sample in manifest["samples"]:
        name = sample["file"]
        path = models / (name + ".seg.pb")
        label_path = Path(sample["labels"])
        require(digest(label_path) == sample["labels_sha256"], "changed GT labels")
        message = reader.read(path, PREFIX + "LidarSemanticSegmentationResult")
        identity = message.SerializeToString()
        replaced = substitute(message, np.fromfile(label_path, dtype="<u4"))
        target = output / (name + ".seg.pb")
        target.write_bytes(message.SerializeToString())
        require(
            reader.read(
                path, PREFIX + "LidarSemanticSegmentationResult"
            ).SerializeToString()
            == identity,
            "original segmentation changed",
        )
        rows.append(
            {
                "file": name,
                "valid_gt_points_replaced": replaced,
                "original_segmentation": str(path.resolve()),
                "original_sha256": digest(path),
                "labels": str(label_path),
                "labels_sha256": digest(label_path),
                "oracle_sha256": digest(target),
            }
        )
    provenance = {
        "diagnostic_only": True,
        "semantic_ground_truth_substitution": True,
        "not_model_performance": True,
        "substitution": "valid learning GT labels, confidence=1; ignored GT unchanged",
        "not_substituted": "detector, instance IDs, geometry, ground, configuration",
        "input_manifest_sha256": digest(manifest_path),
        "schema_sha256": digest(schema),
        "samples": rows,
    }
    (output / "oracle.json").write_text(json.dumps(provenance, indent=2) + "\n")
    return provenance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input", "models", "schema", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--allow-ground-truth-diagnostic", action="store_true")
    args = parser.parse_args()
    if not args.allow_ground_truth_diagnostic:
        parser.error("requires --allow-ground-truth-diagnostic; not model predictions")
    result = prepare(args.input, args.models, args.schema, args.output)
    print(json.dumps({"diagnostic_only": True, "frames": len(result["samples"])}))


if __name__ == "__main__":
    main()
