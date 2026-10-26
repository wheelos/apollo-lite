#!/usr/bin/env python3
"""Evaluate StaticLaneCalibrator against OpenLane-V2 3D point labels."""

import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
import subprocess
import sys


DISTANCE_BINS = ((3.0, 10.0), (10.0, 20.0), (20.0, 30.0), (30.0, 50.000001))
AXES = ("x_forward_m", "y_left_m", "z_up_m")


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def transform_ego_to_camera(point, camera):
    rotation = camera["extrinsic"]["rotation"]
    translation = camera["extrinsic"]["translation"]
    relative = [point[i] - translation[i] for i in range(3)]
    return [
        sum(rotation[row][column] * relative[row] for row in range(3))
        for column in range(3)
    ]


def transform_camera_to_ego(point, camera):
    rotation = camera["extrinsic"]["rotation"]
    translation = camera["extrinsic"]["translation"]
    return [
        translation[row]
        + sum(rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    ]


def pinhole_project(point, intrinsic):
    if point[2] <= 1.0e-6:
        return None
    return (
        intrinsic[0][0] * point[0] / point[2] + intrinsic[0][2],
        intrinsic[1][1] * point[1] / point[2] + intrinsic[1][2],
    )


def nearest_rank(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def summarize(rows):
    eligible = [row for row in rows if row["in_gt_roi"]]
    visible = [row for row in eligible if row["status"] in (
        "projected", "projection_rejected")]
    projected = [row for row in eligible if row["status"] == "projected"]
    output = {
        "gt_roi_points": len(eligible),
        "geometrically_visible_points": len(visible),
        "projection_valid_points": len(projected),
        "projection_rejected_points": sum(
            row["status"] == "projection_rejected" for row in eligible),
        "projection_coverage_of_gt_roi": (
            len(projected) / len(eligible) if eligible else None),
        "projection_coverage_of_geometrically_visible": (
            len(projected) / len(visible) if visible else None),
    }
    for index, axis in enumerate(AXES):
        errors = [
            row["error_ego_m"][index] for row in projected
            if row["error_ego_m"] is not None
        ]
        absolute = [abs(error) for error in errors]
        output[axis] = {
            "mean_signed": sum(errors) / len(errors) if errors else None,
            "mae": sum(absolute) / len(absolute) if absolute else None,
            "rmse": (
                math.sqrt(sum(error * error for error in errors) / len(errors))
                if errors else None),
            "abs_p50": nearest_rank(absolute, 0.50),
            "abs_p95": nearest_rank(absolute, 0.95),
            "abs_max": max(absolute) if absolute else None,
        }
    return output


def ffprobe_size(image_path):
    result = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=width,height", "-of", "json",
            str(image_path),
        ],
        check=True, capture_output=True, text=True,
    )
    stream = json.loads(result.stdout)["streams"][0]
    return int(stream["width"]), int(stream["height"])


def decode_rgb(image_path, width, height):
    result = subprocess.run(
        [
            "ffmpeg", "-v", "error", "-i", str(image_path), "-frames:v", "1",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
        ],
        check=True, capture_output=True,
    )
    if len(result.stdout) != width * height * 3:
        raise RuntimeError(f"unexpected decoded image size: {image_path}")
    return bytearray(result.stdout)


def set_pixel(pixels, width, height, x, y, color):
    if 0 <= x < width and 0 <= y < height:
        offset = (y * width + x) * 3
        pixels[offset:offset + 3] = bytes(color)


def draw_line(pixels, width, height, start, end, color):
    x0, y0 = round(start[0]), round(start[1])
    x1, y1 = round(end[0]), round(end[1])
    steps = max(abs(x1 - x0), abs(y1 - y0))
    if steps == 0:
        set_pixel(pixels, width, height, x0, y0, color)
        return
    for step in range(steps + 1):
        ratio = step / steps
        x = round(x0 + (x1 - x0) * ratio)
        y = round(y0 + (y1 - y0) * ratio)
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                set_pixel(pixels, width, height, x + dx, y + dy, color)


def write_image_overlay(image_path, output_path, rows, width, height,
                        intrinsic):
    pixels = decode_rgb(image_path, width, height)
    by_lane = {}
    for row in rows:
        if not row["in_gt_roi"] or row["status"] not in (
                "projected", "projection_rejected"):
            continue
        lane_id = row["lane_id"]
        by_lane.setdefault(lane_id, []).append(row)
    for lane_rows in by_lane.values():
        lane_rows.sort(key=lambda row: row["point_index"])
        gt_pixels = [row["gt_pixel"] for row in lane_rows]
        recovered_pixels = [
            pinhole_project(row["projected_camera_xyz_m"], intrinsic)
            if row["projected_camera_xyz_m"] is not None else None
            for row in lane_rows
        ]
        for points, color in (
                (gt_pixels, (32, 255, 64)),
                (recovered_pixels, (255, 48, 48))):
            segments = [
                (start, end) for start, end in zip(points, points[1:])
                if start is not None and end is not None
            ]
            for start, end in segments:
                draw_line(pixels, width, height, start, end, color)
    result = subprocess.run(
        [
            "ffmpeg", "-v", "error", "-f", "rawvideo", "-pix_fmt",
            "rgb24", "-s", f"{width}x{height}", "-i", "pipe:0",
            "-frames:v", "1", "-q:v", "3", str(output_path),
        ],
        input=bytes(pixels), check=False, capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.decode(errors="replace"))


def svg_bev(rows, title):
    width, height = 800, 600
    margin = 45

    def to_xy(point):
        return (
            margin + point[1] / 30.0 * (width - 2 * margin),
            height - margin - point[0] / 50.0 * (height - 2 * margin),
        )

    groups = {}
    for row in rows:
        if row["in_gt_roi"]:
            groups.setdefault(row["lane_id"], []).append(row)
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="#171717"/>',
        f'<text x="{margin}" y="24" fill="white">{html.escape(title)}</text>',
        f'<path d="M{margin},{margin}V{height-margin}H{width-margin}" '
        'fill="none" stroke="#aaa"/>',
        f'<text x="{width/2}" y="{height-8}" fill="white">ego left (+y), m</text>',
        f'<text x="2" y="42" fill="white">forward x, m</text>',
    ]
    for lateral in (-15, -10, -5, 0, 5, 10, 15):
        x, _ = to_xy((0, lateral, 0))
        parts.append(
            f'<path d="M{x:.1f},{margin}V{height-margin}" '
            f'stroke="#333"/><text x="{x:.1f}" y="{height-margin+16}" '
            f'fill="#bbb" text-anchor="middle">{lateral}</text>')
    for forward in (0, 10, 20, 30, 40, 50):
        _, y = to_xy((forward, 0, 0))
        parts.append(
            f'<path d="M{margin},{y:.1f}H{width-margin}" '
            f'stroke="#333"/><text x="{margin-6}" y="{y+4:.1f}" '
            f'fill="#bbb" text-anchor="end">{forward}</text>')
    for lane_rows in groups.values():
        lane_rows.sort(key=lambda row: row["point_index"])
        for field, color in (
                ("gt_ego_xyz_m", "#20ff40"),
                ("projected_ego_xyz_m", "#ff3030")):
            points = [
                to_xy(row[field]) for row in lane_rows
                if row[field] is not None
            ]
            if len(points) > 1:
                encoded = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
                parts.append(
                    f'<polyline points="{encoded}" fill="none" '
                    f'stroke="{color}" stroke-width="2"/>')
    parts.append("</svg>")
    return "".join(parts)


def svg_errors(rows, title):
    width, height = 900, 360
    left, right, top, bottom = 60, 20, 30, 45
    usable_width = width - left - right
    usable_height = height - top - bottom
    colors = ("#ff5555", "#55ff77", "#55aaff")
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}">',
        f'<rect width="{width}" height="{height}" fill="#171717"/>',
        f'<text x="{left}" y="20" fill="white">{html.escape(title)}</text>',
    ]
    for error in (-5, -2, 0, 2, 5):
        y = top + (5 - error) / 10 * usable_height
        parts.append(
            f'<path d="M{left},{y:.1f}H{width-right}" stroke="#333"/>'
            f'<text x="{left-8}" y="{y+4:.1f}" fill="#bbb" '
            f'text-anchor="end">{error}</text>')
    parts.append(
        f'<text x="4" y="{height/2}" fill="white">signed error, m</text>')
    parts.append(
        f'<text x="{width/2}" y="{height-8}" fill="white">GT forward, m</text>')
    eligible = [row for row in rows if row["error_ego_m"] is not None]
    for axis, color in enumerate(colors):
        for row in eligible:
            forward = row["gt_ego_xyz_m"][0]
            error = row["error_ego_m"][axis]
            x = left + (forward - 3) / 47 * usable_width
            y = top + (5 - max(-5, min(5, error))) / 10 * usable_height
            parts.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="1.5" '
                f'fill="{color}"/>')
    for index, label in enumerate(("x forward", "y left", "z up")):
        parts.append(
            f'<text x="{left+index*130}" y="{height-25}" '
            f'fill="{colors[index]}">{label}</text>')
    parts.append("</svg>")
    return "".join(parts)


def frame_metadata_paths(dataset_root, sequence, camera_name):
    info_root = dataset_root / "train" / sequence / "info"
    for path in sorted(info_root.glob("*.json")):
        frame = json.loads(path.read_text(encoding="utf-8"))
        if camera_name in frame["sensor"]:
            yield frame


def evaluate(args):
    dataset_root = args.dataset_root.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    input_rows = []
    rows = []
    frames = []
    by_id = {}
    selected_timestamps = set()
    for sequence in args.sequences:
        for frame in frame_metadata_paths(dataset_root, sequence, args.camera):
            if (args.timestamps and
                    int(frame["timestamp"]) not in args.timestamps):
                continue
            sensor = frame["sensor"][args.camera]
            relative_image = sensor["image_path"]
            image_path = dataset_root / relative_image
            if not image_path.is_file():
                raise FileNotFoundError(image_path)
            intrinsic = sensor["intrinsic"]["K"]
            width, height = ffprobe_size(image_path)
            frame_id = f"{sequence}/{frame['timestamp']}"
            calibration = args.calibration
            frame_rows = []
            metadata_path = (
                dataset_root / "train" / sequence / "info"
                / f"{frame['timestamp']}.json")
            for lane in frame["annotation"]["lane_centerline"]:
                lane_id = str(lane["id"])
                for point_index, gt in enumerate(lane["points"]):
                    in_roi = (
                        3.0 <= gt[0] <= 50.0 and abs(gt[1]) <= 15.0
                    )
                    record = {
                        "sequence": sequence,
                        "timestamp_ns": frame["timestamp"],
                        "frame_id": frame_id,
                        "relative_image": relative_image,
                        "lane_id": lane_id,
                        "point_index": point_index,
                        "gt_ego_xyz_m": gt,
                        "gt_pixel": None,
                        "camera_gt_xyz_m": None,
                        "projected_camera_xyz_m": None,
                        "projected_ego_xyz_m": None,
                        "error_ego_m": None,
                        "in_gt_roi": in_roi,
                        "status": "outside_gt_roi",
                    }
                    if in_roi:
                        camera_gt = transform_ego_to_camera(gt, sensor)
                        record["camera_gt_xyz_m"] = camera_gt
                        if camera_gt[2] <= 1.0e-6:
                            record["status"] = "behind_camera"
                        else:
                            pixel = pinhole_project(camera_gt, intrinsic)
                            record["gt_pixel"] = pixel
                            if (pixel[0] < 0 or pixel[0] >= width
                                    or pixel[1] < 0 or pixel[1] >= height):
                                record["status"] = "outside_image"
                            else:
                                record["status"] = "pending_projection"
                                point_id = f"{sequence}:{frame['timestamp']}:{lane_id}:{point_index}"
                                if point_id in by_id:
                                    raise ValueError(f"duplicate point id: {point_id}")
                                by_id[point_id] = (record, sensor)
                                input_rows.append(
                                    "\t".join([
                                        point_id,
                                        f"{pixel[0]:.17g}", f"{pixel[1]:.17g}",
                                        f"{intrinsic[0][0]:.17g}",
                                        f"{intrinsic[1][1]:.17g}",
                                        f"{intrinsic[0][2]:.17g}",
                                        f"{intrinsic[1][2]:.17g}",
                                        f"{calibration['camera_height_m']:.17g}",
                                        f"{calibration['pitch_rad']:.17g}",
                                        f"{calibration['k1']:.17g}",
                                        f"{calibration['k2']:.17g}",
                                        f"{calibration['k3']:.17g}",
                                    ]))
                    rows.append(record)
                    frame_rows.append(record)
            frames.append({
                "sequence": sequence,
                "timestamp_ns": frame["timestamp"],
                "relative_image": relative_image,
                "image_width": width,
                "image_height": height,
                "metadata_path": metadata_path,
                "metadata_sha256": sha256_file(metadata_path),
                "rows": frame_rows,
                "sensor": sensor,
                "intrinsic": intrinsic,
                "image_path": image_path,
            })
            selected_timestamps.add(int(frame["timestamp"]))
    if args.timestamps:
        missing_timestamps = set(args.timestamps) - selected_timestamps
        if missing_timestamps:
            raise ValueError(
                f"requested timestamps were not found: "
                f"{sorted(missing_timestamps)}")

    (output_dir / "projector_input.tsv").write_text(
        "\n".join(input_rows) + ("\n" if input_rows else ""),
        encoding="utf-8")
    result = subprocess.run(
        [str(args.projector.resolve())],
        input=("\n".join(input_rows) + ("\n" if input_rows else "")),
        text=True, check=True, capture_output=True,
    )
    output_by_id = {}
    for line in result.stdout.splitlines():
        fields = line.split("\t")
        if len(fields) != 5:
            raise ValueError(f"malformed production projection output: {line}")
        point_id, status, *xyz = fields
        if point_id in output_by_id:
            raise ValueError(f"duplicate projector output id: {point_id}")
        output_by_id[point_id] = (status, [float(value) for value in xyz])
    if set(output_by_id) != set(by_id):
        raise ValueError(
            f"projector point identity mismatch: input={len(by_id)}, "
            f"output={len(output_by_id)}")

    for point_id, (record, sensor) in by_id.items():
        status, projected_camera = output_by_id[point_id]
        if status == "rejected":
            record["status"] = "projection_rejected"
            continue
        if status != "valid":
            raise ValueError(f"unexpected projector status for {point_id}: {status}")
        projected_ego = transform_camera_to_ego(projected_camera, sensor)
        record["status"] = "projected"
        record["projected_camera_xyz_m"] = projected_camera
        record["projected_ego_xyz_m"] = projected_ego
        record["error_ego_m"] = [
            projected_ego[i] - record["gt_ego_xyz_m"][i]
            for i in range(3)
        ]

    with (output_dir / "points.jsonl").open("x", encoding="utf-8") as target:
        for row in rows:
            target.write(json.dumps(row, sort_keys=True) + "\n")

    groups = {"all": rows}
    for sequence in args.sequences:
        groups[f"sequence:{sequence}"] = [
            row for row in rows if row["sequence"] == sequence
        ]
    for lower, upper in DISTANCE_BINS:
        label = f"{lower:g}-{upper:g}m"
        groups[f"distance:{label}"] = [
            row for row in rows
            if row["in_gt_roi"] and lower <= row["gt_ego_xyz_m"][0] < upper
        ]
        for sequence in args.sequences:
            groups[f"sequence:{sequence}/distance:{label}"] = [
                row for row in rows
                if row["sequence"] == sequence and row["in_gt_roi"]
                and lower <= row["gt_ego_xyz_m"][0] < upper
            ]
    metrics = {
        "schema_version": 1,
        "dataset": "OpenLane-V2 OpenLaneV2_V0.1",
        "camera": args.camera,
        "sequences": args.sequences,
        "image_domain": "rectified pinhole; distortion coefficients set to zero",
        "projection_implementation": "modules/lane:static_lane_projection_point -> StaticLaneCalibrator::PopulateCameraCoordinates",
        "coordinate_conventions": {
            "ego": "x forward, y left, z up",
            "camera": "x right, y down, z forward",
            "projection_error": "predicted ego XYZ minus the same-identity GT ego XYZ",
        },
        "assumptions": {
            "ground_plane": "static flat ground using the configured camera height and pitch",
            "occlusion": "not modeled; in-image points are geometric visibility only",
            "roi": {"ego_forward_m": [3, 50], "ego_lateral_abs_max_m": 15},
            "distance_bins": [[3, 10], [10, 20], [20, 30], [30, 50]],
            "percentile": "nearest rank, ceil(p*n)-1, 1-based",
            "failure_denominator": "all geometrically visible GT ROI points",
        },
        "calibration": args.calibration,
        "counts": {
            "frames": len(frames),
            "gt_points_total": len(rows),
            "gt_points_in_roi": sum(row["in_gt_roi"] for row in rows),
            "geometrically_visible_points": sum(
                row["status"] in ("pending_projection", "projected",
                                  "projection_rejected") for row in rows),
            "projection_inputs": len(input_rows),
            "projection_valid": sum(row["status"] == "projected" for row in rows),
            "projection_rejected": sum(
                row["status"] == "projection_rejected" for row in rows),
            "status_counts": {
                status: sum(row["status"] == status for row in rows)
                for status in sorted({row["status"] for row in rows})
            },
        },
        "groups": {
            name: summarize(group_rows) for name, group_rows in groups.items()
        },
        "frame_metrics": [
            {
                "sequence": frame["sequence"],
                "timestamp_ns": frame["timestamp_ns"],
                "relative_image": frame["relative_image"],
                "metrics": summarize(frame["rows"]),
            }
            for frame in frames
        ],
        "interpretation": (
            "Geometric projection diagnostic only. It does not evaluate "
            "UFLDv2 detection accuracy or real road pose/grade variation."),
    }
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    repository_revision = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    (output_dir / "manifest.json").write_text(
        json.dumps({
            "dataset": metrics["dataset"],
            "dataset_root": str(dataset_root),
            "camera": args.camera,
            "sequences": args.sequences,
            "selected_timestamps_ns": [
                frame["timestamp_ns"] for frame in frames
            ],
            "image_domain": metrics["image_domain"],
            "coordinate_conventions": metrics["coordinate_conventions"],
            "roi": metrics["assumptions"]["roi"],
            "calibration": args.calibration,
            "calibration_source": args.calibration_source,
            "projection_implementation": metrics["projection_implementation"],
            "projector_path": str(args.projector.resolve()),
            "projector_sha256": sha256_file(args.projector.resolve()),
            "evaluator_source_sha256": sha256_file(Path(__file__).resolve()),
            "repository_revision": repository_revision,
            "frames": [
                {
                    "sequence": frame["sequence"],
                    "timestamp_ns": frame["timestamp_ns"],
                    "relative_image": frame["relative_image"],
                    "metadata_path": str(frame["metadata_path"]),
                    "metadata_sha256": frame["metadata_sha256"],
                    "image_width": frame["image_width"],
                    "image_height": frame["image_height"],
                    "gt_points": len(frame["rows"]),
                }
                for frame in frames
            ],
            "counts": metrics["counts"],
            "interpretation": metrics["interpretation"],
        }, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    web_dir = output_dir / "visualization"
    web_dir.mkdir()
    gallery = []
    for frame in frames:
        stem = f"{frame['sequence']}-{Path(frame['relative_image']).stem}"
        image_overlay = web_dir / f"{stem}-image.jpg"
        write_image_overlay(
            frame["image_path"], image_overlay, frame["rows"],
            frame["image_width"], frame["image_height"], frame["intrinsic"])
        bev = svg_bev(frame["rows"], f"{frame['sequence']} {stem} BEV")
        error = svg_errors(frame["rows"], f"{frame['sequence']} {stem} errors")
        (web_dir / f"{stem}-bev.svg").write_text(bev, encoding="utf-8")
        (web_dir / f"{stem}-error.svg").write_text(error, encoding="utf-8")
        gallery.append(
            f"<h2>{html.escape(frame['sequence'])} "
            f"{html.escape(stem)}</h2><p>Green: same-identity GT points; "
            "red: C++ ground-projected points reprojected to the image "
            "(overlap is only a ray consistency check).</p>"
            f'<img src="visualization/{stem}-image.jpg" width="775">'
            '<div class="charts">'
            f'<img src="visualization/{stem}-bev.svg">'
            f'<img src="visualization/{stem}-error.svg"></div>')
    page = (
        "<!doctype html><meta charset='utf-8'><title>Projection accuracy</title>"
        "<style>body{background:#171717;color:#eee;font:16px sans-serif;margin:24px}"
        ".charts{display:flex;flex-wrap:wrap}.charts img{width:min(49%,900px)}"
        "img{max-width:100%}a{color:#7cf}</style>"
        "<h1>OpenLane GT-to-production-projector diagnostic</h1>"
        "<p>BEV green=GT, red=ground-projected points; signed errors use x forward, "
        "y left, z up. Static flat-ground assumptions apply.</p>"
        '<p><a href="../metrics.json">metrics.json</a> | '
        '<a href="../points.jsonl">point-level JSONL</a></p>'
        f"<pre>{html.escape(json.dumps(metrics['groups']['all'], indent=2))}</pre>"
        + "".join(gallery) + "</body>"
    )
    (output_dir / "index.html").write_text(page, encoding="utf-8")
    return metrics


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--projector", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--camera", default="ring_front_center")
    parser.add_argument("--sequences", nargs="+", default=["00556"])
    parser.add_argument("--timestamps", nargs="+", type=int)
    parser.add_argument("--camera-height-m", type=float, default=1.3921812)
    parser.add_argument("--pitch-rad", type=float, default=-0.0019857357)
    parser.add_argument(
        "--calibration-source",
        default=(
            "Fixed CLI values; defaults approximate camera-to-ego "
            "translation z and pitch from the first 00556 frame. "
            "Camera height is not corrected for ego-origin offset from "
            "the road plane."),
    )
    args = parser.parse_args()
    args.calibration = {
        "camera_height_m": args.camera_height_m,
        "pitch_rad": args.pitch_rad,
        "pitch_deg": math.degrees(args.pitch_rad),
        "k1": 0.0,
        "k2": 0.0,
        "k3": 0.0,
    }
    return args


def main():
    args = parse_args()
    if not args.projector.is_file() or not shutil.which("ffmpeg") or not shutil.which("ffprobe"):
        print("projector, ffmpeg, and ffprobe are required", file=sys.stderr)
        return 2
    metrics = evaluate(args)
    print(json.dumps({
        "frames": metrics["counts"]["frames"],
        "gt_points_in_roi": metrics["counts"]["gt_points_in_roi"],
        "geometrically_visible_points": metrics["counts"]["geometrically_visible_points"],
        "projection_valid": metrics["counts"]["projection_valid"],
        "projection_coverage_of_visible":
            metrics["groups"]["all"]["projection_coverage_of_geometrically_visible"],
        "x_forward": metrics["groups"]["all"]["x_forward_m"],
        "y_left": metrics["groups"]["all"]["y_left_m"],
        "z_up": metrics["groups"]["all"]["z_up_m"],
        "output_dir": str(args.output_dir.resolve()),
    }, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
