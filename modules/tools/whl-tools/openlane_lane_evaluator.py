#!/usr/bin/env python3
"""Render OpenLane-V2 3D lane labels over predictions and report diagnostics."""

import argparse
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path


def project_ego_point(point, camera, image_domain="rectified"):
    rotation = camera["extrinsic"]["rotation"]
    translation = camera["extrinsic"]["translation"]
    intrinsic = camera["intrinsic"]["K"]
    distortion = (
        camera["intrinsic"]["distortion"]
        if image_domain == "radial" else (0.0, 0.0, 0.0)
    )
    relative = [point[index] - translation[index] for index in range(3)]
    camera_point = [
        sum(rotation[row][column] * relative[row] for row in range(3))
        for column in range(3)
    ]
    if camera_point[2] <= 1.0e-6:
        return None
    x = camera_point[0] / camera_point[2]
    y = camera_point[1] / camera_point[2]
    radius_squared = x * x + y * y
    radial = (
        1.0 + distortion[0] * radius_squared
        + distortion[1] * radius_squared ** 2
        + distortion[2] * radius_squared ** 3
    )
    return (
        intrinsic[0][0] * x * radial + intrinsic[0][2],
        intrinsic[1][1] * y * radial + intrinsic[1][2],
    )


def camera_point_to_ego(point, camera):
    rotation = camera["extrinsic"]["rotation"]
    translation = camera["extrinsic"]["translation"]
    return [
        translation[row]
        + sum(rotation[row][column] * point[column] for column in range(3))
        for row in range(3)
    ]


def point_segment_distance(point, start, end):
    direction = [end[index] - start[index] for index in range(len(point))]
    offset = [point[index] - start[index] for index in range(len(point))]
    length_squared = sum(value * value for value in direction)
    if length_squared == 0.0:
        return math.sqrt(sum(value * value for value in offset))
    ratio = max(
        0.0,
        min(
            1.0,
            sum(offset[i] * direction[i] for i in range(len(point)))
            / length_squared,
        ),
    )
    return math.sqrt(sum(
        (offset[i] - ratio * direction[i]) ** 2 for i in range(len(point))
    ))


def directed_distances(points, polyline):
    if len(points) < 1 or len(polyline) < 2:
        return []
    segments = list(zip(polyline, polyline[1:]))
    return [
        min(point_segment_distance(point, start, end)
            for start, end in segments)
        for point in points
    ]


def symmetric_chamfer(left, right):
    left_to_right = directed_distances(left, right)
    right_to_left = directed_distances(right, left)
    if not left_to_right or not right_to_left:
        return None
    return (
        sum(left_to_right) / len(left_to_right)
        + sum(right_to_left) / len(right_to_left)
    ) * 0.5


def greedy_matches(costs, threshold):
    pairs = sorted(
        (cost, pred_index, gt_index)
        for pred_index, row in enumerate(costs)
        for gt_index, cost in enumerate(row)
        if cost is not None
    )
    used_predictions = set()
    used_ground_truth = set()
    matches = []
    for cost, pred_index, gt_index in pairs:
        if cost > threshold:
            break
        if pred_index in used_predictions or gt_index in used_ground_truth:
            continue
        used_predictions.add(pred_index)
        used_ground_truth.add(gt_index)
        matches.append((cost, pred_index, gt_index))
    return matches, used_predictions, used_ground_truth


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def set_pixel(pixels, width, height, x, y, color):
    if 0 <= x < width and 0 <= y < height:
        offset = (y * width + x) * 3
        pixels[offset:offset + 3] = bytes(color)


def draw_line(pixels, width, height, start, end, color):
    x0, y0 = (round(start[0]), round(start[1]))
    x1, y1 = (round(end[0]), round(end[1]))
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


def decode_rgb(image_path, width, height):
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to decode dataset images")
    result = subprocess.run(
        [
            "ffmpeg", "-v", "error", "-i", str(image_path), "-frames:v", "1",
            "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1",
        ],
        check=False,
        capture_output=True,
    )
    expected = width * height * 3
    if result.returncode != 0 or len(result.stdout) != expected:
        raise RuntimeError(
            f"unable to decode {image_path}: "
            f"{result.stderr.decode(errors='replace').strip()}")
    return bytearray(result.stdout)


def render_comparison(image_path, output_path, width, height, predictions,
                      ground_truth):
    pixels = decode_rgb(image_path, width, height)
    for lane in ground_truth:
        for start, end in zip(lane, lane[1:]):
            draw_line(pixels, width, height, start, end, (32, 255, 64))
    for lane in predictions:
        for start, end in zip(lane, lane[1:]):
            draw_line(pixels, width, height, start, end, (255, 48, 48))
    with output_path.open("xb") as output:
        output.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        output.write(pixels)


def render_bev(output_path, predicted_lanes, ground_truth_lanes,
               max_forward_m=50.0, max_lateral_m=15.0):
    width = 1000
    height = 800
    pixels = bytearray([24, 24, 24]) * (width * height)

    def to_pixel(point):
        x, y = point[0], point[1]
        return (
            round(width * 0.5 - y / max_lateral_m * (width * 0.5)),
            round(height - x / max_forward_m * height),
        )

    for x in range(0, width, 50):
        for y in range(0, height):
            set_pixel(pixels, width, height, x, y, (38, 38, 38))
    for y in range(0, height, 50):
        for x in range(0, width):
            set_pixel(pixels, width, height, x, y, (38, 38, 38))
    for lane in ground_truth_lanes:
        points = [to_pixel(point) for point in lane]
        for start, end in zip(points, points[1:]):
            draw_line(pixels, width, height, start, end, (32, 255, 64))
    for lane in predicted_lanes:
        points = [to_pixel(point) for point in lane]
        for start, end in zip(points, points[1:]):
            draw_line(pixels, width, height, start, end, (255, 48, 48))
    with output_path.open("xb") as output:
        output.write(f"P6\n{width} {height}\n255\n".encode("ascii"))
        output.write(pixels)


def read_jsonl(path):
    with Path(path).open(encoding="utf-8") as source:
        return [json.loads(line) for line in source if line.strip()]


def metadata_path(dataset_root, relative_path):
    parts = Path(relative_path).parts
    if len(parts) < 5 or parts[0] != "train" or parts[2] != "image":
        raise ValueError(f"unexpected OpenLane image path: {relative_path}")
    return Path(dataset_root) / parts[0] / parts[1] / "info" / (
        Path(parts[-1]).stem + ".json")


def summarize_frame(prediction, dataset_root, output_dir,
                    camera_name, pixel_threshold, metric_3d_threshold,
                    image_domain):
    relative_path = prediction["relative_path"]
    frame_metadata = json.loads(
        metadata_path(dataset_root, relative_path).read_text(encoding="utf-8"))
    camera = frame_metadata["sensor"][camera_name]
    if camera["image_path"] != relative_path:
        raise ValueError(f"camera/annotation image mismatch: {relative_path}")
    image_path = Path(dataset_root) / relative_path
    width = prediction.get("width")
    height = prediction.get("height")
    if width is None or height is None:
        raise ValueError("prediction records must include image dimensions")

    gt_ego = [
        lane["points"]
        for lane in frame_metadata["annotation"]["lane_centerline"]
    ]
    gt_image = []
    gt_visible_ego = []
    for lane in gt_ego:
        visible_ego = []
        projected = []
        for point in lane:
            if not (3.0 <= point[0] <= 50.0 and abs(point[1]) <= 15.0):
                continue
            pixel = project_ego_point(point, camera, image_domain)
            if (pixel is not None and 0 <= pixel[0] < width
                    and 0 <= pixel[1] < height):
                visible_ego.append(point)
                projected.append(pixel)
        if len(projected) >= 2:
            gt_image.append(projected)
            gt_visible_ego.append(visible_ego)

    pred_image = [lane["image_points"] for lane in prediction["lanes"]]
    pred_camera = [lane["camera_points"] for lane in prediction["lanes"]]
    if any(not points for points in pred_camera):
        raise ValueError(
            f"prediction camera points missing at {prediction['camera_timestamp_ns']}")
    pred_ego = [
        [ego for point in lane
         for ego in [camera_point_to_ego(point, camera)]
         if 3.0 <= ego[0] <= 50.0 and abs(ego[1]) <= 15.0]
        for lane in pred_camera
    ]

    pixel_costs = [
        [symmetric_chamfer(pred, gt) for gt in gt_image]
        for pred in pred_image
    ]
    pixel_matches, pixel_pred_used, pixel_gt_used = greedy_matches(
        pixel_costs, pixel_threshold)
    metric_costs = [
        [symmetric_chamfer(pred, gt) for gt in gt_visible_ego]
        for pred in pred_ego
    ]
    metric_matches, metric_pred_used, metric_gt_used = greedy_matches(
        metric_costs, metric_3d_threshold)

    output_path = output_dir / f"{Path(relative_path).stem}_comparison.ppm"
    render_comparison(
        image_path, output_path, width, height, pred_image, gt_image)
    bev_path = output_dir / f"{Path(relative_path).stem}_bev.ppm"
    render_bev(bev_path, pred_ego, gt_visible_ego)
    return {
        "relative_path": relative_path,
        "prediction_lanes": len(pred_image),
        "visible_ground_truth_centerlines": len(gt_image),
        "2d_matches": len(pixel_matches),
        "2d_false_positives": len(pred_image) - len(pixel_pred_used),
        "2d_false_negatives": len(gt_image) - len(pixel_gt_used),
        "2d_matched_chamfer_px": [match[0] for match in pixel_matches],
        "3d_matches": len(metric_matches),
        "3d_false_positives": len(pred_ego) - len(metric_pred_used),
        "3d_false_negatives": len(gt_visible_ego) - len(metric_gt_used),
        "3d_matched_chamfer_m": [match[0] for match in metric_matches],
        "comparison_overlay": str(output_path),
        "bev_3d_overlay": str(bev_path),
    }


def aggregate(frames, distance_key, prefix):
    distances = [
        value for frame in frames for value in frame[distance_key]
    ]
    true_positives = sum(frame[f"{prefix}_matches"] for frame in frames)
    false_positives = sum(frame[f"{prefix}_false_positives"] for frame in frames)
    false_negatives = sum(frame[f"{prefix}_false_negatives"] for frame in frames)
    precision = (
        true_positives / (true_positives + false_positives)
        if true_positives + false_positives else 0.0
    )
    recall = (
        true_positives / (true_positives + false_negatives)
        if true_positives + false_negatives else 0.0
    )
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if precision + recall else 0.0
    )
    return {
        "matches": true_positives,
        "false_positives": false_positives,
        "false_negatives": false_negatives,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "matched_chamfer_mean": (
            sum(distances) / len(distances) if distances else None),
        "matched_chamfer_p95": percentile(distances, 0.95),
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Project OpenLane-V2 3D centerlines into camera images, draw them "
            "against lane predictions, and report diagnostic distances."))
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--predictions", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--camera", default="ring_front_center")
    parser.add_argument(
        "--image-domain", choices=("rectified", "radial"), required=True,
        help="Rectified pinhole images or raw radial k1/k2/k3 images.")
    parser.add_argument("--pixel-match-threshold", type=float, default=50.0)
    parser.add_argument("--metric-3d-match-threshold", type=float, default=1.5)
    parser.add_argument("--max-frames", type=int, default=0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.max_frames < 0:
        print("--max-frames must be non-negative", file=sys.stderr)
        return 2
    manifests = read_jsonl(args.manifest)
    predictions = read_jsonl(args.predictions)
    manifest_by_timestamp = {
        record["camera_timestamp_ns"]: record for record in manifests
    }
    prediction_by_timestamp = {
        record["camera_timestamp_ns"]: record for record in predictions
    }
    if (len(manifest_by_timestamp) != len(manifests)
            or len(prediction_by_timestamp) != len(predictions)):
        raise ValueError("duplicate timestamps in manifest or predictions")
    if set(manifest_by_timestamp) != set(prediction_by_timestamp):
        raise ValueError("manifest/prediction timestamp sets differ")
    frame_ids = sorted(
        set(manifest_by_timestamp)
        & set(prediction_by_timestamp)
    )
    if args.max_frames:
        frame_ids = frame_ids[:args.max_frames]
    if not frame_ids:
        print("no matching manifest/prediction timestamps", file=sys.stderr)
        return 2
    args.output_dir.mkdir(parents=True, exist_ok=True)
    frames = []
    for timestamp in frame_ids:
        manifest = manifest_by_timestamp[timestamp]
        prediction = dict(prediction_by_timestamp[timestamp])
        if (manifest["relative_path"] != prediction["relative_path"]
                or manifest["sequence_num"] != prediction["sequence_num"]):
            raise ValueError(f"source identity mismatch at {timestamp}")
        prediction.update(
            width=manifest["width"], height=manifest["height"])
        frames.append(summarize_frame(
            prediction,
            args.dataset_root,
            args.output_dir,
            args.camera,
            args.pixel_match_threshold,
            args.metric_3d_match_threshold,
            args.image_domain,
        ))

    report = {
        "dataset": "OpenLane-V2",
        "camera": args.camera,
        "frames": len(frames),
        "image_domain": args.image_domain,
        "roi_ego_m": {"forward": [3, 50], "lateral": [-15, 15]},
        "visibility": "in-image geometric visibility; no occlusion labels",
        "thresholds": {
            "2d_pixel_distance": args.pixel_match_threshold,
            "3d_metric_distance_m": args.metric_3d_match_threshold,
        },
        "2d_diagnostic": aggregate(
            frames, "2d_matched_chamfer_px", "2d"),
        "3d_diagnostic": aggregate(
            frames, "3d_matched_chamfer_m", "3d"),
        "interpretation": (
            "Diagnostic only, not the official OpenLane-V2 score: the model "
            "predicts lane boundaries while this sample labels lane "
            "centerlines; greedy matching and thresholds are local proxies."),
        "frame_results": frames,
    }
    report_path = args.output_dir / "metrics.json"
    with report_path.open("x", encoding="utf-8") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps({
        "frames": report["frames"],
        "2d_diagnostic": report["2d_diagnostic"],
        "3d_diagnostic": report["3d_diagnostic"],
        "metrics_path": str(report_path),
        "overlay_dir": str(args.output_dir),
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
