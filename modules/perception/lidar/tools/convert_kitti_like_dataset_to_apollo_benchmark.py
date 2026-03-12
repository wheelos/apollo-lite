#!/usr/bin/env python3

import argparse
import math
import struct
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

import numpy as np


TYPE_MAP = {
    "car": "vehicle",
    "van": "vehicle",
    "truck": "vehicle",
    "bus": "vehicle",
    "tram": "vehicle",
    "misc": "others",
    "pedestrian": "pedestrian",
    "person_sitting": "pedestrian",
    "cyclist": "cyclist",
    "motorcyclist": "cyclist",
    "bicyclist": "cyclist",
    "dontcare": "others",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert KITTI-style lidar dataset (bin + label txt) into Apollo "
            "offline benchmark inputs (pcd + gt txt)."
        )
    )
    parser.add_argument("--bin_dir", required=True, help="Input .bin directory")
    parser.add_argument("--label_dir", required=True, help="Input KITTI-style label directory")
    parser.add_argument("--pcd_dir", required=True, help="Output .pcd directory")
    parser.add_argument("--gt_dir", required=True, help="Output Apollo gt txt directory")
    parser.add_argument(
        "--stem",
        default="",
        help="Only convert one frame stem, e.g. enshicar6000347",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Maximum number of frames to convert, 0 means all",
    )
    parser.add_argument(
        "--sensor_name",
        default="velodyne_64",
        help="Sensor name written into Apollo gt txt",
    )
    parser.add_argument(
        "--source_z_anchor",
        choices=("bottom", "center", "top"),
        default="bottom",
        help=(
            "How to interpret the source label z value. Apollo gt expects bottom "
            "center z_min."
        ),
    )
    parser.add_argument(
        "--yaw_offset",
        type=float,
        default=0.0,
        help="Extra yaw offset (radians) applied to every source box",
    )
    parser.add_argument(
        "--min_points",
        type=int,
        default=1,
        help="Skip GT objects with fewer than this many enclosed points",
    )
    parser.add_argument(
        "--intensity_scale",
        type=float,
        default=255.0,
        help="Scale factor when source intensity looks normalized to [0, 1]",
    )
    parser.add_argument(
        "--force_intensity_uint8",
        action="store_true",
        help="Always clamp source intensity to uint8 directly without auto scaling",
    )
    return parser.parse_args()


def normalize_source_type(raw_type: str) -> str:
    return TYPE_MAP.get(raw_type.strip().lower(), "others")


def convert_z_to_bottom(source_z: float, height: float, anchor: str) -> float:
    if anchor == "bottom":
        return source_z
    if anchor == "center":
        return source_z - 0.5 * height
    return source_z - height


def load_bin(bin_path: Path) -> np.ndarray:
    points = np.fromfile(bin_path, dtype=np.float32)
    if points.size % 4 != 0:
        raise ValueError(f"{bin_path} does not contain x y z intensity tuples")
    return points.reshape(-1, 4)


def load_labels(label_path: Path, yaw_offset: float,
                source_z_anchor: str) -> List[dict]:
    objects = []
    with label_path.open() as infile:
        for line in infile:
            fields = line.strip().split()
            if len(fields) < 15:
                continue
            source_type = fields[0]
            target_type = normalize_source_type(source_type)
            if target_type == "others" and source_type.lower() == "dontcare":
                continue
            truncated = float(fields[1])
            occluded = float(fields[2])
            height = float(fields[8])
            width = float(fields[9])
            length = float(fields[10])
            x = float(fields[11])
            y = float(fields[12])
            z_raw = float(fields[13])
            yaw = float(fields[14]) + yaw_offset
            z_bottom = convert_z_to_bottom(z_raw, height, source_z_anchor)
            objects.append({
                "source_type": source_type,
                "type": target_type,
                "truncated": truncated,
                "occluded": occluded,
                "height": height,
                "width": width,
                "length": length,
                "center_x": x,
                "center_y": y,
                "center_z": z_bottom,
                "yaw": yaw,
            })
    return objects


def select_points_in_box(points_xyz: np.ndarray, obj: dict) -> np.ndarray:
    dx = points_xyz[:, 0] - obj["center_x"]
    dy = points_xyz[:, 1] - obj["center_y"]
    cos_yaw = math.cos(obj["yaw"])
    sin_yaw = math.sin(obj["yaw"])
    local_x = dx * cos_yaw + dy * sin_yaw
    local_y = -dx * sin_yaw + dy * cos_yaw
    z_min = obj["center_z"]
    z_max = z_min + obj["height"]
    return (
        (np.abs(local_x) <= obj["length"] * 0.5)
        & (np.abs(local_y) <= obj["width"] * 0.5)
        & (points_xyz[:, 2] >= z_min)
        & (points_xyz[:, 2] <= z_max)
    )


def choose_intensity_encoding(points: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    intensity = np.nan_to_num(
        points[:, 3], nan=0.0, posinf=255.0, neginf=0.0
    ).astype(np.float64, copy=False)
    if args.force_intensity_uint8:
        return np.clip(np.rint(intensity), 0, 255).astype(np.uint8)
    finite = intensity[np.isfinite(intensity)]
    max_value = float(finite.max()) if finite.size > 0 else 0.0
    if max_value <= 1.5:
        scaled = intensity * args.intensity_scale
    else:
        scaled = intensity
    return np.clip(np.rint(scaled), 0, 255).astype(np.uint8)


def sanitize_gt_point(point: np.ndarray) -> Tuple[float, float, float, float]:
    x = float(point[0])
    y = float(point[1])
    z = float(point[2])
    intensity = float(point[3])
    if not math.isfinite(intensity):
        intensity = 0.0
    return x, y, z, intensity


def write_xyzit_pcd(bin_points: np.ndarray, pcd_path: Path,
                    args: argparse.Namespace) -> None:
    intensity_uint8 = choose_intensity_encoding(bin_points, args)
    xyz = bin_points[:, :3].astype(np.float32, copy=False)
    timestamp = np.zeros((bin_points.shape[0],), dtype=np.float64)

    with pcd_path.open("wb") as outfile:
        header = (
            "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity timestamp\n"
            "SIZE 4 4 4 1 8\n"
            "TYPE F F F U F\n"
            "COUNT 1 1 1 1 1\n"
            f"WIDTH {bin_points.shape[0]}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {bin_points.shape[0]}\n"
            "DATA binary\n"
        )
        outfile.write(header.encode("ascii"))
        for i in range(bin_points.shape[0]):
            outfile.write(
                struct.pack(
                    "<fffBd",
                    float(xyz[i, 0]),
                    float(xyz[i, 1]),
                    float(xyz[i, 2]),
                    int(intensity_uint8[i]),
                    float(timestamp[i]),
                )
            )


def write_apollo_gt(points: np.ndarray, objects: Sequence[dict], gt_path: Path,
                    sensor_name: str, min_points: int) -> Tuple[int, int]:
    kept_objects = []
    points_xyz = points[:, :3]
    for obj_id, obj in enumerate(objects):
        mask = select_points_in_box(points_xyz, obj)
        indices = np.flatnonzero(mask)
        if indices.size < min_points:
            continue
        kept_objects.append((obj_id, obj, indices))

    with gt_path.open("w", encoding="utf-8") as outfile:
        outfile.write(f"0 {len(kept_objects)}\n")
        for obj_id, obj, indices in kept_objects:
            object_points = points[indices]
            fields = [
                sensor_name,
                str(obj_id),
                "0",
                "0",
                "1",
                obj["type"],
                f"{obj['center_x']:.6f}",
                f"{obj['center_y']:.6f}",
                f"{obj['center_z']:.6f}",
                f"{obj['length']:.6f}",
                f"{obj['width']:.6f}",
                f"{obj['height']:.6f}",
                f"{obj['yaw']:.6f}",
                "0",
                "0",
                f"{obj['truncated']:.6f}",
                f"{obj['occluded']:.6f}",
                "0",
                "0",
                "0",
                str(indices.size),
            ]
            for point in object_points:
                x, y, z, intensity = sanitize_gt_point(point)
                fields.extend(
                    [
                        f"{x:.6f}",
                        f"{y:.6f}",
                        f"{z:.6f}",
                        f"{intensity:.6f}",
                    ]
                )
            fields.append(str(indices.size))
            fields.extend(str(int(index)) for index in indices)
            outfile.write(" ".join(fields))
            outfile.write("\n")

    return len(objects), len(kept_objects)


def iter_stems(bin_dir: Path, label_dir: Path, stem: str,
               limit: int) -> Iterable[str]:
    if stem:
        yield stem
        return
    stems = sorted(path.stem for path in bin_dir.glob("*.bin"))
    count = 0
    for current in stems:
        if not (label_dir / f"{current}.txt").exists():
            continue
        yield current
        count += 1
        if limit > 0 and count >= limit:
            break


def main() -> None:
    args = parse_args()
    bin_dir = Path(args.bin_dir)
    label_dir = Path(args.label_dir)
    pcd_dir = Path(args.pcd_dir)
    gt_dir = Path(args.gt_dir)
    pcd_dir.mkdir(parents=True, exist_ok=True)
    gt_dir.mkdir(parents=True, exist_ok=True)

    converted = 0
    total_objects = 0
    kept_objects = 0
    for stem in iter_stems(bin_dir, label_dir, args.stem, args.limit):
        bin_path = bin_dir / f"{stem}.bin"
        label_path = label_dir / f"{stem}.txt"
        if not bin_path.exists() or not label_path.exists():
            print(f"skip {stem}: missing bin or label")
            continue
        points = load_bin(bin_path)
        objects = load_labels(label_path, args.yaw_offset, args.source_z_anchor)
        write_xyzit_pcd(points, pcd_dir / f"{stem}.pcd", args)
        frame_total, frame_kept = write_apollo_gt(
            points,
            objects,
            gt_dir / f"{stem}.txt",
            args.sensor_name,
            args.min_points,
        )
        total_objects += frame_total
        kept_objects += frame_kept
        converted += 1
        print(
            f"{stem}: points={points.shape[0]} labels={frame_total} kept={frame_kept}"
        )

    print(
        f"done: frames={converted} labels={total_objects} kept={kept_objects} "
        f"pcd_dir={pcd_dir} gt_dir={gt_dir}"
    )


if __name__ == "__main__":
    main()
