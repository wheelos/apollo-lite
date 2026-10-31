#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Freeze bounded original cloud protobufs and recorded calibration for replay."""

import argparse
import hashlib
import json
import math
from pathlib import Path

from cyber_record.record import Record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--base-frame", required=True)
    parser.add_argument(
        "--template",
        type=Path,
        default=Path("modules/lidar_fusion/conf/lidar_fusion.pb.txt"),
    )
    args = parser.parse_args()
    if not 1 <= args.samples <= 1000:
        parser.error("samples must be in [1,1000]")
    args.output.mkdir(parents=True, exist_ok=False)
    with Record(str(args.record)) as record:
        # cyber_record 0.1.13's count filter excludes, rather than includes,
        # matching names. Select the indexed channel explicitly.
        total = sum(
            channel.message_number
            for channel in record.get_channel_cache()
            if channel.name == args.topic
        )
        if total == 0:
            raise ValueError("record has no clouds on requested topic")
        n = min(total, args.samples)
        selected = {round(i * (total - 1) / max(1, n - 1)) for i in range(n)}
        calibration = None
        clocks = {}
        topics = [
            args.topic,
            "/tf_static",
            "/tf",
            "/apollo/localization/pose",
            "/apollo/sensor/gnss/odometry",
        ]
        rows = []
        cloud_index = 0
        for topic, msg, stamp in record.read_messages(topics=topics):
            if topic not in clocks:
                clocks[topic] = {
                    "record_stamp_ns": stamp,
                    "header": str(msg.header) if hasattr(msg, "header") else None,
                }
            if topic == "/tf_static":
                for transform in msg.transforms:
                    if transform.child_frame_id == "velodyne64":
                        calibration = transform
            if topic != args.topic:
                continue
            if cloud_index in selected:
                payload = msg.SerializeToString()
                name = f"{cloud_index:06d}.cloud.pb"
                (args.output / name).write_bytes(payload)
                rows.append(
                    {
                        "file": name,
                        "record_stamp_ns": stamp,
                        "measurement_time": msg.measurement_time,
                        "sequence_num": msg.header.sequence_num,
                        "sensor_frame": msg.frame_id,
                        "points": len(msg.point),
                        "sha256": hashlib.sha256(payload).hexdigest(),
                    }
                )
            cloud_index += 1
        if len(rows) != n or calibration is None:
            raise ValueError("incomplete sample or missing velodyne64 static TF")
        if len({r["sensor_frame"] for r in rows}) != 1:
            raise ValueError("cloud sensor frame changed")
        sensor = rows[0]["sensor_frame"]
        if sensor != calibration.child_frame_id:
            raise ValueError("cloud and calibration frames differ")
        t = calibration.transform.translation
        q = calibration.transform.rotation
        norm = math.sqrt(q.qx**2 + q.qy**2 + q.qz**2 + q.qw**2)
        if abs(norm - 1) > 1e-5:
            raise ValueError("non-unit recorded calibration quaternion")
        x, y, z, w = q.qx, q.qy, q.qz, q.qw
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        text = args.template.read_text()
        text = text.replace(
            'pointcloud_channel: ""', f'pointcloud_channel: "{args.topic}"'
        )
        text = text.replace(
            'detection_channel: ""', 'detection_channel: "/offline/detection"'
        )
        text = text.replace(
            'segmentation_channel: ""', 'segmentation_channel: "/offline/segmentation"'
        )
        text = text.replace('sensor_frame: ""', f'sensor_frame: "{sensor}"')
        text = text.replace(
            'base_frame: "base_link"', f'base_frame: "{args.base_frame}"'
        )
        start = text.index("sensor_to_base {")
        end = text.index("\n}", start) + 2
        version = hashlib.sha256(calibration.SerializeToString()).hexdigest()[:16]
        text = (
            text[:start]
            + (
                f'sensor_to_base {{\n version: "record-tf-{version}"\n'
                f" x: {t.x!r}\n y: {t.y!r}\n z: {t.z!r}\n"
                f" roll: {roll!r}\n pitch: {pitch!r}\n yaw: {yaw!r}\n"
                "}"
            )
            + text[end:]
        )
        text = text.replace(
            "LIDAR_OBJECT_DETECTION_BACKEND_CENTERPOINT",
            "LIDAR_OBJECT_DETECTION_BACKEND_POINTPILLARS",
        )
        text = "\n".join(
            line
            for line in text.splitlines()
            if not line.startswith("detection_mapping")
        )
        text += """
detection_mapping { raw_id: 0 group: THING unified_class: CAR }
detection_mapping { raw_id: 1 group: THING unified_class: PEDESTRIAN }
detection_mapping { raw_id: 2 group: THING unified_class: CYCLIST }
spatial { point_views: true trace_rays: true verified_single_origin: false }
"""
        (args.output / "fusion.pb.txt").write_text(text)
        manifest = {
            "record": str(args.record),
            "record_bytes": args.record.stat().st_size,
            "channel": args.topic,
            "total_clouds": total,
            "sample_strategy": "uniform ordinal including first/last",
            "samples": rows,
            "first_payload_clocks": clocks,
            "calibration": str(calibration),
            "calibration_parent": calibration.header.frame_id,
            "output_frame": args.base_frame,
            "base_equivalence": "explicit operator/user assertion, not inferred TF",
            "visibility": "approximate compensated-cloud origin; FREE disabled",
        }
        (args.output / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
        print(
            f"Extracted {len(rows)}/{total} clouds; actual calibration parent "
            f"{calibration.header.frame_id}; FREE disabled"
        )


if __name__ == "__main__":
    main()
