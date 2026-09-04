#!/usr/bin/env python3
"""Convert PandaSet lidar frames to ego-frame binaries and publish them as
Cyber PointCloud messages.

Two stages, because PandaSet frames are pickled pandas DataFrames and the
Apollo container has no pandas:

  1. `convert` (run where pandas is available, e.g. the host):
       pandaset/<seq>/lidar/NN.pkl.gz  ->  <out>/NN.bin + manifest.json
     Keeps only the Pandar64 spinning lidar (d == 0) and transforms world
     coordinates into the lidar/ego frame using lidar/poses.json.
     Each .bin holds little-endian float32 quads: x y z intensity.

  2. `publish` (run inside the container via bazel-bin, stdlib only):
       <out>/*.bin  ->  apollo.drivers.PointCloud on a Cyber channel,
     with header.sequence_num set so results can be paired afterwards by
     save_lidar_semantic_pcd.py.
"""

import argparse
import array
import json
import math
import struct
import sys
import time
from pathlib import Path


MANIFEST_NAME = "manifest.json"


# ---------------------------------------------------------------------------
# convert
# ---------------------------------------------------------------------------

def quaternion_to_rotation(w, x, y, z):
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def convert(args):
    import numpy as np
    import pandas as pd

    lidar_dir = args.sequence_dir / "lidar"
    poses = json.loads((lidar_dir / "poses.json").read_text())
    timestamps = json.loads(
        (args.sequence_dir / "meta" / "timestamps.json").read_text())
    frames = sorted(lidar_dir.glob("*.pkl.gz"))
    if not frames:
        raise RuntimeError(f"no lidar frames found in {lidar_dir}")
    if args.max_frames:
        frames = frames[:args.max_frames]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    manifest = []
    for path in frames:
        index = int(path.name.split(".")[0])
        pose = poses[index]
        frame = pd.read_pickle(path)
        frame = frame[frame["d"] == args.device]
        world = frame[["x", "y", "z"]].to_numpy(dtype=np.float64)
        rotation = np.asarray(quaternion_to_rotation(
            pose["heading"]["w"], pose["heading"]["x"],
            pose["heading"]["y"], pose["heading"]["z"]))
        translation = np.asarray(
            [pose["position"]["x"], pose["position"]["y"],
             pose["position"]["z"]])
        ego = (world - translation) @ rotation  # == R^T (p - t)
        intensity = frame["i"].to_numpy(dtype=np.float64)
        data = np.column_stack([ego, intensity]).astype("<f4")
        output = args.output_dir / f"{index:02d}.bin"
        data.tofile(output)
        z_lo, z_mid, z_hi = np.percentile(ego[:, 2], [5, 50, 95])
        manifest.append({
            "index": index,
            "file": output.name,
            "point_count": int(len(data)),
            "timestamp_sec": timestamps[index],
            "source": str(path),
        })
        print(
            f"converted {path.name}: {len(data)} points, "
            f"z p5/p50/p95 = {z_lo:.2f}/{z_mid:.2f}/{z_hi:.2f} m, "
            f"intensity [{intensity.min():.0f}, {intensity.max():.0f}]",
            flush=True)
    (args.output_dir / MANIFEST_NAME).write_text(
        json.dumps(manifest, indent=2))
    print(f"wrote {len(manifest)} frame(s) to {args.output_dir}")


# ---------------------------------------------------------------------------
# publish
# ---------------------------------------------------------------------------

def read_frame(path):
    points = array.array("f")
    with path.open("rb") as handle:
        points.frombytes(handle.read())
    if sys.byteorder != "little":
        points.byteswap()
    if len(points) % 4 != 0:
        raise RuntimeError(f"{path}: size is not a multiple of 4 floats")
    return points


def build_message(message_type, points, sequence, timestamp_sec, frame_id):
    message = message_type()
    message.header.timestamp_sec = timestamp_sec
    message.header.module_name = "whl_pandaset_pointcloud_publisher"
    message.header.sequence_num = sequence
    message.header.frame_id = frame_id
    message.header.lidar_timestamp = int(timestamp_sec * 1e9)
    message.frame_id = frame_id
    message.is_dense = False
    message.measurement_time = timestamp_sec
    count = len(points) // 4
    message.width = count
    message.height = 1
    timestamp_ns = int(timestamp_sec * 1e9)
    for offset in range(0, len(points), 4):
        point = message.point.add()
        point.x = points[offset]
        point.y = points[offset + 1]
        point.z = points[offset + 2]
        point.intensity = max(0, min(255, int(round(points[offset + 3]))))
        point.timestamp = timestamp_ns
    return message


def publish(args):
    try:
        from cyber.python.cyber_py3 import cyber
        from wheelos_msgs.sensor_msgs import pointcloud_pb2
    except ImportError as error:
        raise SystemExit(
            f"cyber python bindings not importable ({error}); run via "
            "bazel-bin/modules/tools/whl-tools/pandaset_pointcloud_publisher "
            "inside the Apollo container")

    manifest_path = args.input_dir / MANIFEST_NAME
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
    else:
        manifest = [{"file": p.name, "timestamp_sec": None}
                    for p in sorted(args.input_dir.glob("*.bin"))]
    if not manifest:
        raise RuntimeError(f"no frames found in {args.input_dir}")
    if args.max_frames:
        manifest = manifest[:args.max_frames]

    interval = 1.0 / args.fps
    start_wall = time.time()
    first_dataset_ts = manifest[0].get("timestamp_sec")

    cyber.init("whl_pandaset_pointcloud_publisher")
    try:
        node = cyber.Node("whl_pandaset_pointcloud_publisher")
        writer = node.create_writer(args.channel, pointcloud_pb2.PointCloud, 8)
        time.sleep(args.startup_wait_sec)
        for number, entry in enumerate(manifest, start=1):
            points = read_frame(args.input_dir / entry["file"])
            if args.use_dataset_time and entry.get("timestamp_sec"):
                timestamp_sec = entry["timestamp_sec"]
            elif entry.get("timestamp_sec") and first_dataset_ts:
                # Keep dataset inter-frame spacing but anchor at "now".
                timestamp_sec = start_wall + (
                    entry["timestamp_sec"] - first_dataset_ts)
            else:
                timestamp_sec = start_wall + (number - 1) * interval
            message = build_message(
                pointcloud_pb2.PointCloud, points, number, timestamp_sec,
                args.frame_id)
            writer.write(message)
            print(
                f"published {number} ({entry['file']}, {message.width} points) "
                f"at {timestamp_sec:.6f} s", flush=True)
            time.sleep(interval)
    finally:
        cyber.shutdown()


# ---------------------------------------------------------------------------
# cli
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    subparsers = parser.add_subparsers(dest="command", required=True)

    conv = subparsers.add_parser(
        "convert", help="PandaSet pkl.gz frames -> ego-frame .bin (needs pandas)")
    conv.add_argument("--sequence-dir", type=Path, required=True,
                      help="e.g. <pandaset>/019 (contains lidar/ and meta/)")
    conv.add_argument("--output-dir", type=Path, required=True)
    conv.add_argument("--max-frames", type=int, default=0)
    conv.add_argument("--device", type=int, default=0,
                      help="0 = Pandar64 spinning lidar, 1 = PandarGT")
    conv.set_defaults(func=convert)

    pub = subparsers.add_parser(
        "publish", help="publish converted .bin frames as Cyber PointCloud")
    pub.add_argument("--input-dir", type=Path, required=True)
    pub.add_argument(
        "--channel", default="/apollo/sensor/lidar128/compensator/PointCloud2")
    pub.add_argument("--frame-id", default="velodyne128")
    pub.add_argument("--fps", type=float, default=2.0)
    pub.add_argument("--max-frames", type=int, default=0)
    pub.add_argument("--startup-wait-sec", type=float, default=3.0)
    pub.add_argument("--use-dataset-time", action="store_true",
                     help="stamp messages with the original PandaSet time")
    pub.set_defaults(func=publish)

    args = parser.parse_args()
    if getattr(args, "fps", 1.0) <= 0:
        parser.error("--fps must be positive")
    if getattr(args, "max_frames", 0) < 0:
        parser.error("--max-frames must be non-negative")
    return args


def main():
    args = parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
