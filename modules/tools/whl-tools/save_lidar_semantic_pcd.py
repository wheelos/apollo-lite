#!/usr/bin/env python3
"""Save matched PointCloud and semantic labels from a Cyber record as PCD."""

import argparse
from pathlib import Path

def sequence_number(message):
    if not message.HasField("header") or not message.header.HasField("sequence_num"):
        raise ValueError("message header.sequence_num is required")
    return message.header.sequence_num


def write_pcd(path, cloud, result):
    labels = {item.point_index: item for item in result.point_label}
    cloud_point_count = len(cloud.point)
    if result.point_count != cloud_point_count:
        raise ValueError(
            f"point count mismatch for sequence {sequence_number(cloud)}: "
            f"{cloud_point_count} != {result.point_count}"
        )
    with path.open("w", encoding="ascii") as output:
        output.write(
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity label confidence\n"
            "SIZE 4 4 4 4 4 4\n"
            "TYPE F F F U U F\n"
            "COUNT 1 1 1 1 1 1\n"
            f"WIDTH {cloud_point_count}\nHEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {cloud_point_count}\nDATA ascii\n"
        )
        for index, point in enumerate(cloud.point):
            label = labels.get(index)
            if label is None:
                raise ValueError(f"missing semantic label for point index {index}")
            output.write(
                f"{point.x} {point.y} {point.z} {point.intensity} "
                f"{label.semantic_label} {label.confidence}\n"
            )


def main():
    parser = argparse.ArgumentParser(
        description="Save matched lidar PointCloud and semantic results as PCD."
    )
    parser.add_argument("record", type=Path, help="Apollo record file")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--input-topic",
        default="/apollo/sensor/lidar128/compensator/PointCloud2",
    )
    parser.add_argument(
        "--output-topic",
        default="/perception/lidar_semantic_segmentation",
    )
    args = parser.parse_args()
    from cyber.python.cyber_py3.record import RecordReader
    from wheelos_msgs.sensor_msgs.pointcloud_pb2 import PointCloud
    from modules.lidar_semantic_segmentation.proto.lidar_semantic_segmentation_pb2 import (
        LidarSemanticSegmentationResult,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)

    pending_clouds = {}
    pending_results = {}
    cloud_sequences = []
    result_sequences = []
    written = 0
    for topic, raw_data, data_type, _ in RecordReader(str(args.record)).read_messages():
        if topic == args.input_topic:
            message = PointCloud.FromString(raw_data)
            sequence = sequence_number(message)
            cloud_sequences.append(sequence)
            pending_clouds[sequence] = message
        elif topic == args.output_topic:
            message = LidarSemanticSegmentationResult.FromString(raw_data)
            sequence = sequence_number(message)
            result_sequences.append(sequence)
            pending_results[sequence] = message
        else:
            continue

        sequence = sequence_number(message)
        if sequence not in pending_clouds or sequence not in pending_results:
            continue
        write_pcd(args.output_dir / f"{sequence:010d}.pcd",
                  pending_clouds.pop(sequence), pending_results.pop(sequence))
        written += 1

    if written == 0:
        raise RuntimeError(
            "record contained no matched input/output messages; "
            f"input sequences={cloud_sequences[:5]}...{cloud_sequences[-5:]}, "
            f"output sequences={result_sequences[:5]}...{result_sequences[-5:]}"
        )
    unmatched = len(set(pending_clouds) | set(pending_results))
    print(
        f"Wrote {written} PCD file(s) to {args.output_dir}; "
        f"skipped {unmatched} unmatched sequence(s)"
    )


if __name__ == "__main__":
    main()
