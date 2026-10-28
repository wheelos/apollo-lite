#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Explicitly derive diagnostic-only relative poses; never claim local ODOM."""

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

from cyber_record.record import Record


def multiply(a, b):
    x, y, z, w = a
    u, v, s, t = b
    return (
        w * u + x * t + y * s - z * v,
        w * v - x * s + y * t + z * u,
        w * s + x * v - y * u + z * t,
        w * t - x * u - y * v - z * s,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--topic", default="/apollo/localization/pose")
    parser.add_argument("--allow-absolute-pose-diagnostic", action="store_true")
    args = parser.parse_args()
    if not args.allow_absolute_pose_diagnostic:
        parser.error("explicit diagnostic-only acknowledgement is required")
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((args.input / "manifest.json").read_text())
    samples = manifest["samples"]
    poses = []
    with Record(manifest["record"]) as record:
        for _, msg, _ in record.read_messages(topics=[args.topic]):
            stamp = msg.header.timestamp_sec
            p, q = msg.pose.position, msg.pose.orientation
            values = (q.qx, q.qy, q.qz, q.qw)
            norm = math.sqrt(sum(x * x for x in values))
            if not all(math.isfinite(x) for x in (*values, p.x, p.y, p.z, stamp)):
                raise ValueError("nonfinite recorded pose")
            if abs(norm - 1) > 1e-5:
                raise ValueError("non-unit recorded pose")
            poses.append((stamp, (p.x, p.y, p.z), values))
    poses.sort()
    times = [p[0] for p in poses]
    if not poses or len(set(times)) != len(times):
        raise ValueError("missing poses or duplicate pose timestamps")
    chosen = []
    for sample in samples:
        stamp = sample["measurement_time"]
        index = bisect.bisect_left(times, stamp)
        candidates = poses[max(0, index - 1) : min(len(poses), index + 1)]
        pose = min(candidates, key=lambda x: abs(x[0] - stamp))
        if abs(pose[0] - stamp) > 0.02:
            raise ValueError(f"pose/cloud clock alignment failed: {sample['file']}")
        chosen.append(pose)
    origin = chosen[0]
    inverse = tuple(-x for x in origin[2][:3]) + (origin[2][3],)
    gaps = []
    rows = []
    for sample, (stamp, position, quaternion) in zip(samples, chosen):
        d = tuple(x - y for x, y in zip(position, origin[1])) + (0,)
        translation = multiply(multiply(inverse, d), origin[2])[:3]
        q = multiply(inverse, quaternion)
        text = (
            f"stamp: {stamp!r}\n"
            'odom_frame: "diagnostic_relative_localization"\n'
            f'base_frame: "{manifest["output_frame"]}"\n'
            "epoch: 1\nvalid: true\ndiagnostic_only: true\n"
            f"translation {{x: {translation[0]!r} y: {translation[1]!r} "
            f"z: {translation[2]!r}}}\n"
            f"qx: {q[0]!r}\nqy: {q[1]!r}\nqz: {q[2]!r}\nqw: {q[3]!r}\n"
        )
        path = args.output / (sample["file"] + ".pose.txt")
        path.write_text(text)
        rows.append(
            {
                "file": path.name,
                "sha256": hashlib.sha256(text.encode()).hexdigest(),
                "dt": stamp - sample["measurement_time"],
            }
        )
    for a, b in zip(samples, samples[1:]):
        gaps.append(b["measurement_time"] - a["measurement_time"])
    # Sampling interval is explicit for this sparse replay, not a sensor default.
    max_dt = max(gaps, default=0.1) + 0.01
    steps = [
        math.sqrt(sum((x - y) ** 2 for x, y in zip(a[1], b[1])))
        for a, b in zip(chosen, chosen[1:])
    ]
    rotations = [
        2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(a[2], b[2])))))
        for a, b in zip(chosen, chosen[1:])
    ]
    max_translation = max(steps, default=0) + 0.01
    max_rotation = max(rotations, default=0) + 0.01
    config = (args.input / "fusion.pb.txt").read_text()
    config += f"""
candidates {{
  oriented_recall: true
  convex_unknown: true
  separate_semantic_instances: true
  far_connectivity_size: 0.45
  far_distance: 30
}}
temporal {{
  enabled: true
  odom_frame: "diagnostic_relative_localization"
  max_dt: {max_dt!r}
  max_pose_translation_step: {max_translation!r}
  max_pose_rotation_step: {max_rotation!r}
  lost_seconds: {max_dt * 2!r}
  history_seconds: {max_dt * 2!r}
  max_tracks: 1024
  static_map: true
  feedback: true
}}
"""
    (args.output / "fusion.pb.txt").write_text(config)
    (args.output / "pose-manifest.json").write_text(
        json.dumps(
            {
                "diagnostic_only": True,
                "source_topic": args.topic,
                "input_manifest_sha256": hashlib.sha256(
                    (args.input / "manifest.json").read_bytes()
                ).hexdigest(),
                "normalization": "T_first^-1 * T_current; payload clock; not ODOM",
                "sparse_replay_limits": {
                    "max_dt": max_dt,
                    "max_pose_translation_step": max_translation,
                    "max_pose_rotation_step": max_rotation,
                    "purpose": "exercise sparse diagnostic state, not continuity proof",
                },
                "pose_rows": rows,
                "warning": "absolute localization continuity/ODOM NOT verified",
            },
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
