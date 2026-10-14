#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import csv
import json
import math
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile


STAGES = {
    "Voxelization": "voxelization_ms",
    "3D Backbone": "sparse_backbone_ms",
    "RPN + Head": "rpn_centerhead_ms",
    "Decode + NMS": "decode_nms_ms",
}
BASELINE_REVISION = "7c1623f234d5a9794fd862c9456f68f87ae7b900"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run NVIDIA CUDA-CenterPoint on a directory of PCD files."
    )
    parser.add_argument("--pcd_dir", required=True, type=pathlib.Path)
    parser.add_argument("--output_dir", required=True, type=pathlib.Path)
    parser.add_argument("--config", required=True, type=pathlib.Path)
    parser.add_argument("--converter_bin", required=True, type=pathlib.Path)
    parser.add_argument("--centerpoint_bin", required=True, type=pathlib.Path)
    parser.add_argument("--scn_onnx", required=True, type=pathlib.Path)
    parser.add_argument("--rpn_centerhead_engine", required=True, type=pathlib.Path)
    return parser.parse_args()


def run_checked(command, **kwargs):
    completed = subprocess.run(
        [str(item) for item in command],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        errors="replace",
        **kwargs,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "Command failed ({}): {}\n{}".format(
                completed.returncode, command[0], completed.stdout
            )
        )
    return completed.stdout


def parse_frame_timings(log_text):
    timings = {}
    current_frame = None
    frame_pattern = re.compile(r"load file:\s*(.+?)(?:\.bin)?\s*$")
    time_pattern = re.compile(r"\[TIME\]\s*(.+?):\s*([0-9.eE+-]+)\s*ms")
    for line in log_text.splitlines():
        frame_match = frame_pattern.search(line)
        if frame_match:
            current_frame = pathlib.Path(frame_match.group(1)).stem
            timings.setdefault(current_frame, {})
            continue
        time_match = time_pattern.search(line)
        if time_match and current_frame is not None:
            field = STAGES.get(time_match.group(1).strip())
            if field:
                timings[current_frame][field] = float(time_match.group(2))
    return timings


def read_detections(path):
    detections = []
    with path.open(encoding="utf-8") as result_file:
        for line_number, line in enumerate(result_file, 1):
            values = line.split()
            if len(values) != 11:
                raise ValueError(
                    "{}:{}: expected 11 NVIDIA detection fields, got {}".format(
                        path, line_number, len(values)
                    )
                )
            x, y, z, width, length, height, vx, vy, yaw = map(
                float, values[:9]
            )
            class_id = int(values[9])
            score = float(values[10])
            if not all(
                math.isfinite(value)
                for value in (x, y, z, width, length, height, vx, vy, yaw, score)
            ):
                raise ValueError(
                    "{}:{}: detection contains a non-finite value".format(
                        path, line_number
                    )
                )
            if width <= 0.0 or length <= 0.0 or height <= 0.0:
                raise ValueError(
                    "{}:{}: detection dimensions must be positive".format(
                        path, line_number
                    )
                )
            if class_id < 0 or not 0.0 <= score <= 1.0:
                raise ValueError(
                    "{}:{}: class_id or score is outside its valid range".format(
                        path, line_number
                    )
                )
            detections.append(
                {
                    "center": [x, y, z],
                    "size": [width, length, height],
                    "velocity": [vx, vy],
                    "yaw": yaw,
                    "class_id": class_id,
                    "score": score,
                }
            )
    return detections


def main():
    args = parse_args()
    pcd_dir = args.pcd_dir.resolve(strict=True)
    output_dir = args.output_dir.resolve()
    config = args.config.resolve(strict=True)
    converter_bin = args.converter_bin.resolve(strict=True)
    centerpoint_bin = args.centerpoint_bin.resolve(strict=True)
    scn_onnx = args.scn_onnx.resolve(strict=True)
    engine = args.rpn_centerhead_engine.resolve(strict=True)

    pcd_files = sorted(pcd_dir.glob("*.pcd"))
    if not pcd_files:
        raise ValueError("No .pcd files found in {}".format(pcd_dir))
    output_dir.mkdir(parents=True, exist_ok=True)
    predictions_dir = output_dir / "predictions"
    predictions_dir.mkdir(exist_ok=True)
    if any((predictions_dir / (pcd.stem + ".txt")).exists() for pcd in pcd_files):
        raise FileExistsError(
            "Prediction output already exists; choose a fresh output directory"
        )

    with tempfile.TemporaryDirectory(prefix="lidar_object_detection_") as temp:
        run_root = pathlib.Path(temp)
        input_dir = run_root / "input"
        model_dir = run_root / "model"
        prediction_dir = run_root / "data" / "prediction"
        build_dir = run_root / "build"
        input_dir.mkdir()
        model_dir.mkdir()
        prediction_dir.mkdir(parents=True)
        build_dir.mkdir()
        (model_dir / "centerpoint.scn.onnx").symlink_to(scn_onnx)
        (model_dir / "rpn_centerhead_sim.plan").symlink_to(engine)

        point_counts = {}
        for pcd in pcd_files:
            output_bin = input_dir / (pcd.stem + ".bin")
            converter_log = run_checked(
                [
                    converter_bin,
                    "--config={}".format(config),
                    "--input_pcd={}".format(pcd),
                    "--output_bin={}".format(output_bin),
                ]
            )
            count_match = re.search(r"Wrote\s+(\d+)\s+points", converter_log)
            if not count_match:
                raise RuntimeError(
                    "Could not read point count from converter output:\n"
                    + converter_log
                )
            point_counts[pcd.stem] = int(count_match.group(1))

        inference_log = run_checked(
            [centerpoint_bin, input_dir, "--verbose"], cwd=build_dir
        )
        (output_dir / "nvidia_centerpoint.log").write_text(
            inference_log, encoding="utf-8"
        )
        timings = parse_frame_timings(inference_log)

        with (output_dir / "predictions.jsonl").open(
            "w", encoding="utf-8"
        ) as jsonl:
            with (output_dir / "timings.csv").open(
                "w", encoding="utf-8", newline=""
            ) as timing_file:
                fields = [
                    "frame_id",
                    "point_count",
                    "voxelization_ms",
                    "sparse_backbone_ms",
                    "rpn_centerhead_ms",
                    "decode_nms_ms",
                    "total_inference_ms",
                ]
                writer = csv.DictWriter(timing_file, fieldnames=fields)
                writer.writeheader()
                for pcd in pcd_files:
                    stem = pcd.stem
                    result_path = prediction_dir / (stem + ".txt")
                    if not result_path.is_file():
                        raise FileNotFoundError(
                            "NVIDIA CenterPoint produced no result for {}".format(
                                stem
                            )
                        )
                    frame_timings = timings.get(stem, {})
                    missing_stages = set(STAGES.values()) - set(frame_timings)
                    if missing_stages:
                        raise ValueError(
                            "Missing per-frame timings for {}: {}".format(
                                stem, sorted(missing_stages)
                            )
                        )
                    total_ms = sum(frame_timings.values())
                    output_txt = predictions_dir / result_path.name
                    shutil.copyfile(result_path, output_txt)
                    frame = {
                        "frame_id": stem,
                        "source_path": str(pcd),
                        "model_revision": BASELINE_REVISION,
                        "point_count": point_counts[stem],
                        "detections": read_detections(result_path),
                        "timings_ms": frame_timings,
                    }
                    jsonl.write(json.dumps(frame, separators=(",", ":")) + "\n")
                    writer.writerow(
                        {
                            "frame_id": stem,
                            "point_count": point_counts[stem],
                            **frame_timings,
                            "total_inference_ms": "{:.6f}".format(total_ms),
                        }
                    )

    print("Wrote predictions and per-frame timings to {}".format(output_dir))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileExistsError, FileNotFoundError, RuntimeError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
