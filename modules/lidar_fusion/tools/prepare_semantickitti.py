#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Freeze original SemanticKITTI validation sweeps; labels never configure fusion."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from google.protobuf import text_format

sys.path.insert(
    0, str(Path(__file__).resolve().parents[2] / "tools" / "perception_diagnostics")
)
from proto import Reader  # noqa: E402


def quantize_remission(values):
    if not np.isfinite(values).all() or np.any(values < 0) or np.any(values > 1):
        raise ValueError("expected finite SemanticKITTI remission in [0,1]")
    intensity = np.rint(values.astype(np.float64) * 255).astype(np.uint32)
    error = np.abs(intensity.astype(np.float64) / 255 - values)
    return intensity, float(error.max()), float(error.mean())


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("data-root", "schema", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--starts", type=int, nargs="+", default=[0, 1000, 2000])
    parser.add_argument("--frames-per-block", type=int, default=4)
    args = parser.parse_args()
    if not 1 <= len(args.starts) <= 10 or not 1 <= args.frames_per_block <= 20:
        parser.error("bounded scope: 1..10 blocks of 1..20 frames")
    ordinals = [
        i for start in args.starts for i in range(start, start + args.frames_per_block)
    ]
    if len(set(ordinals)) != len(ordinals) or min(ordinals) < 0:
        parser.error("blocks must be nonnegative and nonoverlapping")
    sequence = args.data_root / "dataset/sequences/08"
    times_path = sequence / "times.txt"
    times = [float(t) for t in times_path.read_text().splitlines()]
    reader = Reader(args.schema)
    config = reader.message("apollo.lidar_fusion.FusionConfig")
    text_format.Parse(
        Path("modules/lidar_fusion/conf/lidar_fusion.pb.txt").read_text(), config
    )
    config.pointcloud_channel = "/offline/semantickitti/08/velodyne"
    config.detection_channel = "/offline/semantickitti/pointpillars"
    config.segmentation_channel = "/offline/semantickitti/rangeret"
    config.sensor_frame = config.base_frame = "kitti_velodyne"
    config.sensor_to_base.version = "same-native-sensor-frame-identity"
    config.detection_backend = 2
    config.ClearField("detection_mapping")
    for raw, unified in enumerate((1, 4, 5)):
        config.detection_mapping.add(raw_id=raw, group=1, unified_class=unified)
    config.min_x, config.max_x = -20, 60
    config.min_y, config.max_y = -40, 40
    # Sensor-height diagnostic assumption, not annotation-derived ground truth.
    config.expected_ground_z = -1.73
    config.spatial.point_views = config.spatial.trace_rays = True
    config.spatial.verified_single_origin = False
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = args.output / "inputs"
    inputs.mkdir()
    rows = []
    for ordinal in ordinals:
        source = sequence / "velodyne" / f"{ordinal:06d}.bin"
        label = sequence / "labels" / f"{ordinal:06d}.label"
        raw = np.fromfile(source, dtype="<f4")
        if raw.size % 4:
            raise ValueError("invalid original four-float point file")
        points = raw.reshape(-1, 4)
        if not len(points) or not np.isfinite(points).all():
            raise ValueError("nonfinite or empty original cloud")
        if label.stat().st_size != len(points) * 4:
            raise ValueError("original point/label count mismatch")
        intensities, error_max, error_mean = quantize_remission(points[:, 3])
        cloud = reader.message("apollo.drivers.PointCloud")
        cloud.frame_id = cloud.header.frame_id = "kitti_velodyne"
        cloud.header.sequence_num = ordinal + 1
        # Source elapsed time starts at zero; a documented +1 s rebase permits
        # the positive source-identity contract without pretending wall time.
        cloud.measurement_time = cloud.header.timestamp_sec = times[ordinal] + 1
        cloud.header.lidar_timestamp = round(cloud.measurement_time * 1e9)
        for xyz, intensity in zip(points[:, :3], intensities):
            point = cloud.point.add()
            point.x, point.y, point.z = (float(v) for v in xyz)
            point.intensity = int(intensity)
        name = f"{ordinal:06d}.cloud.pb"
        payload = cloud.SerializeToString()
        (inputs / name).write_bytes(payload)
        rows.append(
            {
                "file": name,
                "ordinal": ordinal,
                "points": len(points),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "source": str(source),
                "source_sha256": sha256(source),
                "labels": str(label),
                "labels_sha256": sha256(label),
                "measurement_time": cloud.measurement_time,
                "remission_max_abs_error": error_max,
                "remission_mean_abs_error": error_mean,
            }
        )
    manifest = {
        "dataset": "SemanticKITTI official validation sequence 08",
        "record": str(sequence),
        "total_clouds": len(times),
        "output_frame": "kitti_velodyne",
        "calibration_parent": "same native sensor frame",
        "selection": {"starts": args.starts, "frames_per_block": args.frames_per_block},
        "time": "official elapsed time + 1 second; no pose, no per-ray timestamps",
        "intensity": "round(raw float remission * 255); both model scales 1/255",
        "ground": "expected_ground_z=-1.73 diagnostic sensor-height assumption",
        "samples": rows,
        "times_sha256": sha256(times_path),
    }
    (inputs / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (inputs / "fusion.pb.txt").write_text(text_format.MessageToString(config))
    print(json.dumps({"frames": len(rows), "points": sum(r["points"] for r in rows)}))


if __name__ == "__main__":
    main()
