#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Expand only the fixed input capacity of the pinned NVIDIA diagnostic ONNX."""

import argparse
import hashlib
import json
from pathlib import Path

import onnx


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    expected = "564a50999a8a005213f3ad19a61a7004ed8a01fa10047371fc0beaa3eabf341b"
    if hashlib.sha256(args.source.read_bytes()).hexdigest() != expected:
        raise ValueError("not the pinned NVIDIA 092affc PointPillars export")
    if args.output.exists() or args.output.with_suffix(".json").exists():
        raise ValueError("refusing to overwrite derived artifact")
    model = onnx.load(str(args.source))

    def checker_error(value):
        try:
            onnx.checker.check_model(value)
        except onnx.checker.ValidationError as exc:
            return str(exc)
        return None

    original_error = checker_error(model)
    shapes = {
        i.name: [d.dim_value for d in i.type.tensor_type.shape.dim]
        for i in model.graph.input
    }
    if shapes != {
        "voxels": [10000, 32, 10],
        "voxel_idxs": [10000, 4],
        "voxel_num": [1],
    }:
        raise ValueError(f"unexpected source shapes: {shapes}")
    for value in model.graph.input:
        if value.name in ("voxels", "voxel_idxs"):
            value.type.tensor_type.shape.dim[0].dim_value = 40000
    # No graph nodes or trained weights are changed. Runtime MAX_VOXELS=40000.
    if checker_error(model) != original_error:
        raise ValueError("capacity change introduced an ONNX checker failure")
    onnx.save(model, str(args.output))
    args.output.with_suffix(".json").write_text(
        json.dumps(
            {
                "source_sha256": expected,
                "derived_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
                "change": "input capacity only: voxels/voxel_idxs 10000 -> 40000",
                "weights_changed": False,
                "diagnostic_only": True,
                "preexisting_source_checker_error": original_error,
                "validation": "same checker result; TensorRT build/runtime required",
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
