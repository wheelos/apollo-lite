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
import json
import math
import pathlib
import sys


MODEL_CLASSES = (
    "car",
    "truck",
    "construction_vehicle",
    "bus",
    "trailer",
    "barrier",
    "motorcycle",
    "bicycle",
    "pedestrian",
    "traffic_cone",
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert token-aligned predictions and run the nuScenes evaluator."
    )
    parser.add_argument("--manifest", required=True, type=pathlib.Path)
    parser.add_argument("--predictions", required=True, type=pathlib.Path)
    parser.add_argument("--data_root", required=True, type=pathlib.Path)
    parser.add_argument("--output_dir", required=True, type=pathlib.Path)
    parser.add_argument(
        "--format-only",
        action="store_true",
        help="Write the official result JSON without importing or running the devkit.",
    )
    return parser.parse_args()


def _matrix(value, name):
    if not isinstance(value, list) or len(value) != 4:
        raise ValueError("{} must be a 4x4 matrix".format(name))
    matrix = []
    for row in value:
        if not isinstance(row, list) or len(row) != 4:
            raise ValueError("{} must be a 4x4 matrix".format(name))
        converted = [float(item) for item in row]
        if not all(math.isfinite(item) for item in converted):
            raise ValueError("{} contains a non-finite value".format(name))
        matrix.append(converted)
    return matrix


def _read_manifest(path):
    records = {}
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    "{}:{}: invalid manifest JSON: {}".format(
                        path, line_number, exc
                    )
                ) from exc
            token = record.get("sample_token")
            frame_id = record.get("frame_id")
            if not isinstance(token, str) or not token:
                raise ValueError(
                    "{}:{}: missing sample_token".format(path, line_number)
                )
            if not isinstance(frame_id, str) or not frame_id:
                raise ValueError("{}:{}: missing frame_id".format(path, line_number))
            if token in records:
                raise ValueError(
                    "{}:{}: duplicate sample_token {}".format(
                        path, line_number, token
                    )
                )
            record["lidar2ego"] = _matrix(record.get("lidar2ego"), "lidar2ego")
            record["ego2global"] = _matrix(record.get("ego2global"), "ego2global")
            records[token] = record
    if not records:
        raise ValueError("No samples found in manifest {}".format(path))
    return records


def _read_predictions(path):
    predictions = {}
    with path.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            try:
                frame = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(
                    "{}:{}: invalid prediction JSON: {}".format(
                        path, line_number, exc
                    )
                ) from exc
            token = frame.get("sample_token")
            if not isinstance(token, str) or not token:
                raise ValueError(
                    "{}:{}: prediction has no sample_token".format(
                        path, line_number
                    )
                )
            if token in predictions:
                raise ValueError(
                    "{}:{}: duplicate prediction for {}".format(
                        path, line_number, token
                    )
                )
            detections = frame.get("detections")
            if not isinstance(detections, list):
                raise ValueError(
                    "{}:{}: detections must be a list".format(path, line_number)
                )
            predictions[token] = detections
    if not predictions:
        raise ValueError("No predictions found in {}".format(path))
    return predictions


def _transform_position(matrix, point):
    return [
        sum(matrix[row][column] * point[column] for column in range(3))
        + matrix[row][3]
        for row in range(3)
    ]


def _transform_vector(matrix, vector):
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def _multiply_rotation(left, right):
    return [
        [
            sum(left[row][k] * right[k][column] for k in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _quaternion_from_rotation(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = [
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        ]
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        quaternion = [
            (matrix[2][1] - matrix[1][2]) / scale,
            0.25 * scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
        ]
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        quaternion = [
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[0][1] + matrix[1][0]) / scale,
            0.25 * scale,
            (matrix[1][2] + matrix[2][1]) / scale,
        ]
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        quaternion = [
            (matrix[1][0] - matrix[0][1]) / scale,
            (matrix[0][2] + matrix[2][0]) / scale,
            (matrix[1][2] + matrix[2][1]) / scale,
            0.25 * scale,
        ]
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm == 0.0:
        raise ValueError("Could not convert rotation matrix to quaternion")
    return [value / norm for value in quaternion]


def _attribute_name(detection_name, velocity):
    speed = math.hypot(velocity[0], velocity[1])
    if detection_name in {
        "car",
        "truck",
        "construction_vehicle",
        "bus",
        "trailer",
    }:
        return "vehicle.moving" if speed > 0.2 else "vehicle.parked"
    if detection_name in {"bicycle", "motorcycle"}:
        return "cycle.with_rider" if speed > 0.2 else "cycle.without_rider"
    if detection_name == "pedestrian":
        return "pedestrian.moving" if speed > 0.2 else "pedestrian.standing"
    return ""


def _nuscenes_yaw(yaw):
    return -float(yaw) - math.pi / 2.0


def _convert_detection(detection, record):
    class_id = detection.get("class_id")
    if not isinstance(class_id, int) or not 0 <= class_id < len(MODEL_CLASSES):
        raise ValueError("Detection class_id is outside the model class range")
    center = detection.get("center")
    size = detection.get("size")
    velocity = detection.get("velocity")
    yaw = detection.get("yaw")
    score = detection.get("score")
    vectors = ((center, 3), (size, 3), (velocity, 2))
    if any(not isinstance(vector, list) or len(vector) != size for vector, size in vectors):
        raise ValueError("Detection center, size, or velocity has an invalid shape")
    try:
        center = [float(value) for value in center]
        size = [float(value) for value in size]
        velocity = [float(value) for value in velocity]
        yaw = float(yaw)
        score = float(score)
    except (TypeError, ValueError) as exc:
        raise ValueError("Detection contains a non-numeric value") from exc
    values = center + size + velocity + [yaw, score]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Detection contains a non-finite value")
    if min(size) <= 0.0 or not 0.0 <= score <= 1.0:
        raise ValueError("Detection size or score is outside its valid range")

    point_ego = _transform_position(record["lidar2ego"], center)
    point_global = _transform_position(record["ego2global"], point_ego)
    velocity_ego = _transform_vector(
        record["lidar2ego"], [velocity[0], velocity[1], 0.0]
    )
    velocity_global = _transform_vector(
        record["ego2global"], velocity_ego
    )

    lidar_to_global = _multiply_rotation(
        [row[:3] for row in record["ego2global"][:3]],
        [row[:3] for row in record["lidar2ego"][:3]],
    )
    converted_yaw = _nuscenes_yaw(yaw)
    cosine = math.cos(converted_yaw)
    sine = math.sin(converted_yaw)
    local_rotation = [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]
    orientation = _quaternion_from_rotation(
        _multiply_rotation(lidar_to_global, local_rotation)
    )
    detection_name = MODEL_CLASSES[class_id]
    return {
        "sample_token": record["sample_token"],
        "translation": point_global,
        "size": [float(size[0]), float(size[1]), float(size[2])],
        "rotation": orientation,
        "velocity": velocity_global[:2],
        "detection_name": detection_name,
        "detection_score": float(score),
        "attribute_name": _attribute_name(detection_name, velocity_global),
    }


def create_result(manifest, predictions):
    missing = set(manifest) - set(predictions)
    unexpected = set(predictions) - set(manifest)
    if missing or unexpected:
        raise ValueError(
            "Prediction/manifest token mismatch: {} missing, {} unexpected".format(
                len(missing), len(unexpected)
            )
        )
    result = {}
    for token, record in manifest.items():
        result[token] = [
            _convert_detection(detection, record)
            for detection in predictions[token]
        ]
    return {
        "meta": {
            "use_camera": False,
            "use_lidar": True,
            "use_radar": False,
            "use_map": False,
            "use_external": False,
        },
        "results": result,
    }


def main():
    args = parse_args()
    manifest_path = args.manifest.resolve(strict=True)
    predictions_path = args.predictions.resolve(strict=True)
    data_root = args.data_root.resolve(strict=True)
    output_dir = args.output_dir.resolve()
    if not args.format_only:
        try:
            from nuscenes import NuScenes
            from nuscenes.eval.detection.config import config_factory
            from nuscenes.eval.detection.evaluate import NuScenesEval
        except ImportError as exc:
            raise RuntimeError(
                "nuScenes official evaluation requires nuscenes-devkit; "
                "use --format-only or install the evaluator dependency"
            ) from exc

    result = create_result(
        _read_manifest(manifest_path), _read_predictions(predictions_path)
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    result_path = output_dir / "nuscenes_results.json"
    if result_path.exists():
        raise FileExistsError(
            "nuScenes result already exists; choose a fresh output directory"
        )
    result_path.write_text(
        json.dumps(result, separators=(",", ":")) + "\n", encoding="utf-8"
    )
    print("Wrote nuScenes result JSON to {}".format(result_path))
    if args.format_only:
        return 0

    metrics_dir = output_dir / "metrics"
    metrics_dir.mkdir(exist_ok=True)
    nusc = NuScenes(
        version="v1.0-trainval", dataroot=str(data_root), verbose=True
    )
    evaluator = NuScenesEval(
        nusc,
        config_factory("detection_cvpr_2019"),
        str(result_path),
        eval_set="val",
        output_dir=str(metrics_dir),
        verbose=True,
    )
    evaluator.main(plot_examples=0, render_curves=False)
    summary_path = metrics_dir / "metrics_summary.json"
    if not summary_path.is_file():
        raise RuntimeError("nuScenes evaluator did not write a metrics summary")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    print(
        "Official nuScenes val: mAP={:.6f}, NDS={:.6f}".format(
            summary["mean_ap"], summary["nd_score"]
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (
        FileExistsError,
        FileNotFoundError,
        ImportError,
        OSError,
        RuntimeError,
        ValueError,
    ) as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
