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

import math
import pathlib

import numpy as np


def _read_nuscenes_points(path):
    path = pathlib.Path(path)
    if path.stat().st_size % (5 * np.dtype("<f4").itemsize) != 0:
        raise ValueError(
            "nuScenes point file is not five float32 features: {}".format(path)
        )
    return np.fromfile(path, dtype="<f4").reshape(-1, 5)


def _validate_sweep(sweep):
    matrix = np.asarray(sweep.get("sweep_to_lidar"), dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Sweep transform must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError("Sweep transform has an invalid homogeneous row")
    try:
        time_lag = float(sweep.get("time_lag"))
    except (TypeError, ValueError) as exc:
        raise ValueError("Sweep time_lag must be numeric") from exc
    if not math.isfinite(time_lag) or time_lag < 0.0:
        raise ValueError("Sweep time_lag must be finite and non-negative")
    return matrix, time_lag


def build_model_input(record, output_path):
    current = _read_nuscenes_points(record["source_path"])
    keyframe = np.empty((current.shape[0], 5), dtype=np.float32)
    keyframe[:, :4] = current[:, :4]
    keyframe[:, 4] = 0.0
    point_sets = [keyframe]

    for sweep in record["sweeps"]:
        source = _read_nuscenes_points(sweep["source_path"])
        matrix, time_lag = _validate_sweep(sweep)
        if sweep.get("remove_close", False):
            keep = ~((np.abs(source[:, 0]) < 1.0) & (np.abs(source[:, 1]) < 1.0))
            source = source[keep]

        transformed = np.empty((source.shape[0], 5), dtype=np.float32)
        transformed[:, :3] = (
            source[:, :3] @ matrix[:3, :3].T + matrix[:3, 3]
        )
        transformed[:, 3] = source[:, 3]
        transformed[:, 4] = time_lag
        point_sets.append(transformed)

    points = np.concatenate(point_sets, axis=0)
    if not np.isfinite(points[:, :4]).all():
        invalid_xyz_or_intensity = ~np.isfinite(points[:, :4]).all(axis=1)
        points = points[~invalid_xyz_or_intensity]
    if points.shape[0] == 0:
        raise ValueError("nuScenes sweeps contain no valid points")

    output_path = pathlib.Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    points.tofile(output_path)
    return int(points.shape[0])
