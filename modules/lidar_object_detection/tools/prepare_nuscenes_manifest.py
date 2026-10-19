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
import copy
import json
import math
import pathlib
import sys
import tempfile
import pickle


CENTERPOINT_SWEEP_COUNT = 9


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create a token-to-LiDAR manifest from nuScenes infos."
    )
    parser.add_argument("--data_root", required=True, type=pathlib.Path)
    parser.add_argument("--info_pickle", required=True, type=pathlib.Path)
    parser.add_argument("--output", required=True, type=pathlib.Path)
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Optional leading sample count for a smoke test; 0 selects all.",
    )
    return parser.parse_args()


def resolve_lidar_path(data_root, lidar_path):
    path = pathlib.Path(lidar_path)
    candidates = [path] if path.is_absolute() else [data_root / path]
    candidates.extend(
        data_root / directory / "LIDAR_TOP" / path.name
        for directory in ("samples", "sweeps")
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve(strict=True)
    raise FileNotFoundError("nuScenes LiDAR file does not exist: {}".format(path))


def validate_matrix(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("{} must be a 4x4 matrix".format(name))
    matrix = []
    for row in value:
        if not isinstance(row, (list, tuple)) or len(row) != 4:
            raise ValueError("{} must be a 4x4 matrix".format(name))
        converted = [float(element) for element in row]
        if not all(math.isfinite(element) for element in converted):
            raise ValueError("{} contains a non-finite value".format(name))
        matrix.append(converted)
    return matrix


def invert_rigid_transform(matrix, name):
    rotation = [row[:3] for row in matrix[:3]]
    translation = [matrix[row][3] for row in range(3)]
    for row in range(3):
        for column in range(3):
            product = sum(
                rotation[k][row] * rotation[k][column] for k in range(3)
            )
            if abs(product - (1.0 if row == column else 0.0)) > 1.0e-3:
                raise ValueError("{} rotation is not orthonormal".format(name))
    if any(abs(value) > 1.0e-6 for value in matrix[3][:3]) or abs(
        matrix[3][3] - 1.0
    ) > 1.0e-6:
        raise ValueError("{} has an invalid homogeneous row".format(name))
    inverse_rotation = [
        [rotation[column][row] for column in range(3)] for row in range(3)
    ]
    inverse_translation = [
        -sum(inverse_rotation[row][column] * translation[column] for column in range(3))
        for row in range(3)
    ]
    return [
        inverse_rotation[row] + [inverse_translation[row]] for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def create_model_sweeps(info, data_root, current_path):
    current_timestamp = float(info.get("timestamp"))
    if not math.isfinite(current_timestamp):
        raise ValueError("Info record has a non-finite timestamp")

    sweeps = []
    lidar_sweeps = info.get("lidar_sweeps", [])
    if not isinstance(lidar_sweeps, list):
        raise ValueError("Info lidar_sweeps must be a list")
    for index, sweep in enumerate(lidar_sweeps[:CENTERPOINT_SWEEP_COUNT]):
        lidar_points = sweep.get("lidar_points", {})
        lidar_path = lidar_points.get("lidar_path")
        if not isinstance(lidar_path, str) or not lidar_path:
            raise ValueError("Sweep {} has no LiDAR path".format(index))
        source_path = resolve_lidar_path(data_root, lidar_path)
        if source_path.stat().st_size % 20 != 0:
            raise ValueError(
                "nuScenes sweep is not five float32 features: {}".format(
                    source_path
                )
            )
        timestamp = float(sweep.get("timestamp"))
        time_lag = current_timestamp - timestamp
        if not math.isfinite(time_lag) or time_lag < -1.0e-6:
            raise ValueError(
                "Sweep {} is newer than its reference frame".format(index)
            )
        transform = validate_matrix(
            lidar_points.get("lidar2sensor"), "sweep lidar2sensor"
        )
        sweeps.append(
            {
                "source_path": str(source_path),
                "sweep_to_lidar": invert_rigid_transform(
                    transform, "sweep lidar2sensor"
                ),
                "time_lag": max(0.0, time_lag),
                "remove_close": True,
            }
        )

    if not sweeps:
        sweeps.append(
            {
                "source_path": str(current_path),
                "sweep_to_lidar": [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                "time_lag": 0.0,
                "remove_close": True,
            }
        )
    while len(sweeps) < CENTERPOINT_SWEEP_COUNT:
        sweeps.append(copy.deepcopy(sweeps[-1]))
    return sweeps


def create_manifest(args):
    data_root = args.data_root.resolve(strict=True)
    info_pickle = args.info_pickle.resolve(strict=True)
    if args.limit < 0:
        raise ValueError("--limit must not be negative")
    with info_pickle.open("rb") as info_file:
        dataset = pickle.load(info_file)
    metainfo = dataset.get("metainfo", {})
    if metainfo.get("dataset") != "nuscenes":
        raise ValueError("Info pickle is not a nuScenes dataset")
    if metainfo.get("version") != "v1.0-trainval":
        raise ValueError(
            "Expected v1.0-trainval infos, got {}".format(
                metainfo.get("version")
            )
        )
    infos = dataset.get("data_list")
    if not isinstance(infos, list) or not infos:
        raise ValueError("Info pickle contains no data_list records")
    if args.limit:
        infos = infos[: args.limit]

    tokens = set()
    frame_ids = set()
    records = []
    for index, info in enumerate(infos):
        token = info.get("token")
        lidar_points = info.get("lidar_points", {})
        lidar_path = lidar_points.get("lidar_path")
        if not isinstance(token, str) or not token:
            raise ValueError("Info record {} has no sample token".format(index))
        if not isinstance(lidar_path, str) or not lidar_path:
            raise ValueError("Info record {} has no LiDAR path".format(index))
        if token in tokens:
            raise ValueError("Duplicate sample token: {}".format(token))
        source_path = resolve_lidar_path(data_root, lidar_path)
        if not source_path.name.endswith(".pcd.bin"):
            raise ValueError(
                "Expected a nuScenes .pcd.bin input, got {}".format(source_path)
            )
        if source_path.stat().st_size % (5 * 4) != 0:
            raise ValueError(
                "nuScenes point file is not five float32 features: {}".format(
                    source_path
                )
            )
        frame_id = source_path.name[: -len(".pcd.bin")]
        if frame_id in frame_ids:
            raise ValueError("Duplicate LiDAR frame identifier: {}".format(frame_id))
        record = {
            "frame_id": frame_id,
            "sample_token": token,
            "source_path": str(source_path),
            "timestamp": float(info["timestamp"]),
            "lidar2ego": validate_matrix(lidar_points.get("lidar2ego"), "lidar2ego"),
            "ego2global": validate_matrix(info.get("ego2global"), "ego2global"),
            "sweeps": create_model_sweeps(info, data_root, source_path),
        }
        records.append(record)
        tokens.add(token)
        frame_ids.add(frame_id)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=args.output.parent,
            delete=False,
        ) as output_file:
            temporary_path = pathlib.Path(output_file.name)
            for record in records:
                output_file.write(
                    json.dumps(record, separators=(",", ":")) + "\n"
                )
        temporary_path.replace(args.output)
    except Exception:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise
    return len(records)


def main():
    args = parse_args()
    count = create_manifest(args)
    print("Wrote {} sample records to {}".format(count, args.output))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (FileNotFoundError, OSError, ValueError, pickle.UnpicklingError) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
