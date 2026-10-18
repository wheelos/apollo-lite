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
import os
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
        description="Run NVIDIA CUDA-CenterPoint on PCD files or a nuScenes manifest."
    )
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--pcd_dir", type=pathlib.Path)
    source_group.add_argument("--nuscenes_manifest", type=pathlib.Path)
    parser.add_argument("--output_dir", required=True, type=pathlib.Path)
    parser.add_argument("--config", required=True, type=pathlib.Path)
    parser.add_argument("--converter_bin", required=True, type=pathlib.Path)
    parser.add_argument("--centerpoint_bin", required=True, type=pathlib.Path)
    parser.add_argument("--scn_onnx", required=True, type=pathlib.Path)
    parser.add_argument("--rpn_centerhead_engine", required=True, type=pathlib.Path)
    return parser.parse_args()


def read_nuscenes_manifest(path):
    records = []
    frame_ids = set()
    sample_tokens = set()
    with path.open(encoding="utf-8") as manifest_file:
        for line_number, line in enumerate(manifest_file, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    "{}:{}: invalid manifest JSON: {}".format(
                        path, line_number, exc
                    )
                ) from exc
            frame_id = record.get("frame_id")
            sample_token = record.get("sample_token")
            source_path = record.get("source_path")
            if not all(
                isinstance(value, str) and value
                for value in (frame_id, sample_token, source_path)
            ):
                raise ValueError(
                    "{}:{}: expected non-empty frame_id, sample_token, "
                    "and source_path".format(path, line_number)
                )
            if frame_id in frame_ids or sample_token in sample_tokens:
                raise ValueError(
                    "{}:{}: duplicate frame_id or sample_token".format(
                        path, line_number
                    )
                )
            source = pathlib.Path(source_path).resolve(strict=True)
            if source.name != frame_id + ".pcd.bin":
                raise ValueError(
                    "{}:{}: frame_id does not match source filename".format(
                        path, line_number
                    )
                )
            if not source.name.endswith(".pcd.bin"):
                raise ValueError(
                    "{}:{}: nuScenes source must be a .pcd.bin file".format(
                        path, line_number
                    )
                )
            sweeps = record.get("sweeps")
            if sweeps is not None:
                if not isinstance(sweeps, list) or len(sweeps) != 9:
                    raise ValueError(
                        "{}:{}: expected exactly nine historical sweeps".format(
                            path, line_number
                        )
                    )
                for sweep in sweeps:
                    sweep_path = pathlib.Path(
                        sweep.get("source_path", "")
                    ).resolve(strict=True)
                    if sweep_path.suffix != ".bin":
                        raise ValueError(
                            "{}:{}: sweep source must be a binary point file".format(
                                path, line_number
                            )
                        )
                    sweep["source_path"] = str(sweep_path)
            frame_ids.add(frame_id)
            sample_tokens.add(sample_token)
            records.append((frame_id, source, sample_token, sweeps))
    if not records:
        raise ValueError("No frames found in nuScenes manifest {}".format(path))
    return records


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
                    "{}:{}: expected 11 CenterPoint detection fields, got {}".format(
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
    output_dir = args.output_dir.resolve()
    config = args.config.resolve(strict=True)
    converter_bin = args.converter_bin.resolve(strict=True)
    centerpoint_bin = args.centerpoint_bin.resolve(strict=True)
    scn_onnx = args.scn_onnx.resolve(strict=True)
    engine = args.rpn_centerhead_engine.resolve(strict=True)

    if args.pcd_dir:
        if not args.pcd_dir.is_dir():
            raise ValueError("PCD input directory does not exist")
        source_dir = args.pcd_dir.resolve(strict=True)
        frame_items = [
            (path.stem, path, None, None)
            for path in sorted(source_dir.glob("*.pcd"))
        ]
        if not frame_items:
            raise ValueError("No .pcd files found in {}".format(source_dir))
        input_is_nuscenes_bin = False
    else:
        manifest = args.nuscenes_manifest.resolve(strict=True)
        frame_items = read_nuscenes_manifest(manifest)
        input_is_nuscenes_bin = True

    output_dir.mkdir(parents=True, exist_ok=True)
    predictions_dir = output_dir / "predictions"
    predictions_dir.mkdir(exist_ok=True)
    if any(
        (predictions_dir / (frame_id + ".txt")).exists()
        for frame_id, _, _, _ in frame_items
    ):
        raise FileExistsError(
            "Prediction output already exists; choose a fresh output directory"
        )

    with tempfile.TemporaryDirectory(prefix="lidar_object_detection_") as temp:
        run_root = pathlib.Path(temp)
        input_dir = run_root / "input"
        model_dir = run_root / "model"
        prediction_dir = run_root / "data" / "prediction"
        build_dir = run_root / "build"
        model_input_dir = run_root / "model_input"
        input_dir.mkdir()
        model_dir.mkdir()
        prediction_dir.mkdir(parents=True)
        build_dir.mkdir()
        model_input_dir.mkdir()
        (model_dir / "centerpoint.scn.onnx").symlink_to(scn_onnx)
        (model_dir / "rpn_centerhead_sim.plan").symlink_to(engine)

        point_counts = {}
        for frame_id, source_path, _, sweeps in frame_items:
            output_bin = input_dir / (frame_id + ".bin")
            if input_is_nuscenes_bin and sweeps is not None:
                from nuscenes_sweeps import build_model_input

                model_source = model_input_dir / (frame_id + ".bin")
                build_model_input(
                    {
                        "source_path": str(source_path),
                        "sweeps": sweeps,
                    },
                    model_source,
                )
                input_flag = "--input_nuscenes_model_bin"
                converter_source = model_source
            else:
                input_flag = (
                    "--input_nuscenes_bin"
                    if input_is_nuscenes_bin
                    else "--input_pcd"
                )
                converter_source = source_path
            converter_log = run_checked(
                [
                    converter_bin,
                    "--config={}".format(config),
                    "{}={}".format(input_flag, converter_source),
                    "--output_bin={}".format(output_bin),
                    "--scn_onnx={}".format(scn_onnx),
                    "--rpn_centerhead_engine={}".format(engine),
                ]
            )
            count_match = re.search(r"Wrote\s+(\d+)\s+points", converter_log)
            if not count_match:
                raise RuntimeError(
                    "Could not read point count from converter output:\n"
                    + converter_log
                )
            point_counts[frame_id] = int(count_match.group(1))

        inference_log = run_checked(
            [
                centerpoint_bin,
                str(input_dir) + os.sep,
                "--verbose",
            ],
            cwd=build_dir,
        )
        (output_dir / "centerpoint.log").write_text(
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
                for stem, source_path, sample_token, _ in frame_items:
                    result_path = prediction_dir / (stem + ".txt")
                    if not result_path.is_file():
                        raise FileNotFoundError(
                            "CenterPoint produced no result for {}".format(
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
                        "source_path": str(source_path),
                        "model_revision": BASELINE_REVISION,
                        "point_count": point_counts[stem],
                        "detections": read_detections(result_path),
                        "timings_ms": frame_timings,
                    }
                    if sample_token is not None:
                        frame["sample_token"] = sample_token
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
