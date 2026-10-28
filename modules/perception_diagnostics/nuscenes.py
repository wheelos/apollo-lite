#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Read-only labeled subset evaluation and offline fusion visualization."""

import argparse
import csv
import json
import math
import shutil
from collections import Counter
from pathlib import Path

from evaluate_baseline import audit_frame, coverage, digest, distribution, require
from PIL import Image
from proto import Reader

CLASSES = ("car", "truck", "bus", "pedestrian", "bicycle", "motorcycle")
UNIFIED = {
    1: "car",
    2: "truck",
    3: "bus",
    4: "pedestrian",
    5: "bicycle",
    6: "motorcycle",
}
RAW = (
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


def match(predictions, truth, distance=2.0):
    """Score-ordered class-aware, one-to-one XY center matching, inclusive gate."""
    counts = {name: Counter(tp=0, fp=0, fn=0) for name in CLASSES}
    available = set(range(len(truth)))
    assignments = []
    for i in sorted(
        range(len(predictions)), key=lambda i: (-predictions[i]["score"], i)
    ):
        pred = predictions[i]
        candidates = [
            (math.dist(pred["center"][:2], truth[j]["center"][:2]), j)
            for j in available
            if truth[j]["class"] == pred["class"]
        ]
        best = min(candidates, default=(math.inf, -1))
        accepted = best[0] <= distance
        counts[pred["class"]]["tp" if accepted else "fp"] += 1
        if accepted:
            available.remove(best[1])
        assignments.append(
            {
                "prediction": i,
                "truth": best[1] if accepted else None,
                "distance": best[0] if accepted else None,
            }
        )
    for j in available:
        counts[truth[j]["class"]]["fn"] += 1
    return counts, assignments


def rates(counts):
    result = dict(counts)
    tp, fp, fn = (result.get(k, 0) for k in ("tp", "fp", "fn"))
    result["precision"] = tp / (tp + fp) if tp + fp else None
    result["recall"] = tp / (tp + fn) if tp + fn else None
    return result


def box(item):
    b = item.box
    yaw = math.atan2(
        2 * (b.qw * b.qz + b.qx * b.qy), 1 - 2 * (b.qy * b.qy + b.qz * b.qz)
    )
    return {
        "center": [b.center.x, b.center.y, b.center.z],
        "size": [b.size.x, b.size.y, b.size.z],
        "yaw": yaw,
    }


def in_roi(item, config):
    x, y = item["center"][:2]
    return config.min_x <= x < config.max_x and config.min_y <= y < config.max_y


def enum_names(reader, name):
    return {
        v.number: v.name
        for v in reader.pool.FindEnumTypeByName("apollo.lidar_fusion." + name).values
    }


def export(experiment_path, schema, output, max_points=8000):
    experiment = json.loads(experiment_path.read_text())
    output.mkdir(parents=True, exist_ok=False)
    (output / "camera").mkdir()
    reader = Reader(schema)
    prefix = "apollo.lidar_fusion."
    populations = {
        variant: {cls: Counter(tp=0, fp=0, fn=0) for cls in CLASSES}
        for variant in (
            "raw_detector",
            "raw_detector_standard",
            "accepted_detector",
            "full_fusion",
            "classified_fusion",
        )
    }
    source_hashes = {
        str(experiment_path): digest(experiment_path),
        str(schema): digest(schema),
    }
    report = {
        "schema_version": 1,
        "official_nuscenes_metrics": False,
        "protocol": {
            "classes": CLASSES,
            "xy_center_distance_m": 2.0,
            "gate": "inclusive",
            "matching": "score descending, nearest unmatched same-class truth",
            "roi": "configured base-frame half-open XY rectangle",
            "truth_filter": "official detection categories with num_lidar_pts > 0",
            "unsupported_classes": [
                "barrier",
                "traffic_cone",
                "trailer",
                "construction_vehicle",
            ],
            "score": (
                "raw and DET >= evidence_score_min; SEG_RECALL uses fixed 1.0 ordering"
            ),
            "limitations": [
                "Small keyframe subset is not official mAP/NDS or temporal evaluation",
                "RangeRet cross-domain from SemanticKITTI; no semantic ground truth",
                "CenterPoint from token-aligned actual 10-sweep runtime cache",
                "No FREE accuracy acceptance; unverified origins keep FREE disabled",
                "GT boxes are visualization/evaluation only; never supplied to fusion",
                "Camera image is contextual; timestamp skew shown, no box projection",
            ],
        },
        "experiment": experiment,
        "frames": [],
        "accuracy": {},
        "timing_conditions": {
            "detector": "cached outputs; det_ms=-1 means unavailable",
            "seg_ms": "fresh RangeRet Segment call; initialization excluded",
            "fusion_ms": "CPU SingleFrameFusion Process; serialization excluded",
            "execution": "Bazel dev/GPU; sequential frames/modes on shared host",
            "not_claimed": "controlled throughput, E2E latency or 5-10 ms acceptance",
        },
    }
    labels = {
        name: enum_names(reader, name)
        for name in ("Group", "Explanation", "Source", "Reason", "UnifiedClass")
    }
    viewer = {"frames": [], "labels": labels, "protocol": report["protocol"]}
    summaries = []
    excluded = Counter()
    for run in experiment["runs"]:
        inputs, results = Path(run["inputs"]), Path(run["results"])
        manifest_path, truth_path = inputs / "manifest.json", Path(run["truth"])
        manifest = json.loads(manifest_path.read_text())
        truth_frames = json.loads(truth_path.read_text())
        config_path = results / "config.pb"
        config = reader.read(config_path, prefix + "FusionConfig")
        for path in (manifest_path, truth_path, config_path, results / "arguments.txt"):
            source_hashes[str(path)] = digest(path)
        arguments = (results / "arguments.txt").read_text().splitlines()
        require(len(arguments) in (8, 9), "expected cached-detector replay")
        if len(arguments) == 9:
            report["timing_conditions"][
                "seg_ms"
            ] = "cached immutable semantic predictions; seg_ms=-1 unavailable"
            viewer["title"] = "LiDAR fusion: nuScenes frozen-model quality comparison"
            viewer["banner"] = (
                "Cached CenterPoint + cached cross-domain RangeRet. "
                "Not official mAP/NDS; UNKNOWN is not FREE. "
                "Identity-uncertain candidates remain inspectable."
            )
        runtime_paths = [Path(arguments[0]), results / "summary.csv"]
        if len(arguments) == 8:
            runtime_paths.append(Path(arguments[4]))
        for path in runtime_paths:
            source_hashes[str(path)] = digest(path)
        report["protocol"]["evidence_score_min"] = config.evidence_score_min
        report["protocol"][
            "raw_detector_standard_score_min"
        ] = config.unsupported_score_min
        report["protocol"]["roi_xy"] = [
            config.min_x,
            config.max_x,
            config.min_y,
            config.max_y,
        ]
        rows = list(csv.DictReader((results / "summary.csv").open()))
        coverage(manifest, rows)
        summaries.extend(rows)
        require(
            all(float(row["det_ms"]) == -1 for row in rows),
            "cached detector timing misrepresented",
        )
        for sample in manifest["samples"]:
            name = sample["file"]
            require(digest(inputs / name) == sample["sha256"], "changed cloud")
            require(
                digest(Path(sample["source"])) == sample["source_sha256"],
                "changed original sweep",
            )
            out_path = results / (name + ".fusion.pb")
            out = reader.read(out_path, prefix + "FusionOutput")
            require(
                out.health.det_valid and out.health.seg_valid and out.health.mode == 0,
                "invalid NORMAL inputs",
            )
            require(len(out.point_evidence) == sample["points"], "point count mismatch")
            audit = dict(audit_frame(out, config))
            det_path, seg_path = (results / (name + s) for s in (".det.pb", ".seg.pb"))
            det = reader.read(
                det_path,
                "apollo.lidar_object_detection.LidarObjectDetectionFrameResult",
            )
            seg = reader.read(
                seg_path,
                "apollo.lidar_semantic_segmentation.LidarSemanticSegmentationResult",
            )
            if len(arguments) == 9:
                cached_seg = Path(arguments[8]) / (name + ".seg.pb")
                require(
                    seg_path.read_bytes() == cached_seg.read_bytes(),
                    "cached segmentation altered",
                )
                source_hashes[str(cached_seg)] = digest(cached_seg)
            cached_path = Path(run["detections"]) / (name + ".det.pb")
            require(
                det_path.read_bytes() == cached_path.read_bytes(),
                "cached detections altered",
            )
            labels_by_id = {p.point_index: p for p in seg.point_label}
            require(
                len(labels_by_id) == sample["points"], "incomplete semantic mapping"
            )
            for p in out.point_evidence:
                label = labels_by_id[p.point_id]
                require(
                    p.raw_semantic_label == label.semantic_label
                    and p.semantic_probability == label.confidence,
                    "altered raw semantics",
                )
            raw, objects = [], []
            for h in out.hypothesis:
                require(
                    h.raw_detection == det.detection[h.detection_index],
                    "altered raw detection",
                )
                item = box(h)
                item.update(
                    {
                        "class": RAW[h.raw_detection.class_id],
                        "score": h.raw_detection.score,
                        "id": h.detection_index,
                    }
                )
                raw.append(item)
            for obj in out.object:
                item = box(obj)
                item.update(
                    {
                        "id": obj.id,
                        "class": UNIFIED.get(obj.unified_class, "other"),
                        "score": obj.raw_score if obj.HasField("raw_score") else 1.0,
                        "source": labels["Source"][obj.source],
                        "conflict": obj.class_conflict,
                        "classification_use": obj.classification_use,
                        "points": len(obj.point_id),
                        "reason": [labels["Reason"][r] for r in obj.reason],
                    }
                )
                objects.append(item)
            gt = truth_frames[name]
            truth = [
                b for b in gt["boxes"] if b["class"] in CLASSES and in_roi(b, config)
            ]
            excluded["gt_unsupported_or_outside_roi"] += len(gt["boxes"]) - len(truth)
            variants = {
                "raw_detector": [
                    b
                    for b in raw
                    if b["class"] in CLASSES
                    and in_roi(b, config)
                    and b["score"] >= config.evidence_score_min
                ],
                "raw_detector_standard": [
                    b
                    for b in raw
                    if b["class"] in CLASSES
                    and in_roi(b, config)
                    and b["score"] >= config.unsupported_score_min
                ],
                "accepted_detector": [
                    b
                    for b in objects
                    if b["source"] == "DET"
                    and b["class"] in CLASSES
                    and in_roi(b, config)
                    and b["score"] >= config.evidence_score_min
                ],
                "classified_fusion": [
                    b
                    for b in objects
                    if b["classification_use"] == 1
                    and b["class"] in CLASSES
                    and in_roi(b, config)
                ],
                "full_fusion": [
                    b
                    for b in objects
                    if b["class"] in CLASSES
                    and in_roi(b, config)
                    and (
                        b["source"] != "DET" or b["score"] >= config.evidence_score_min
                    )
                ],
            }
            excluded["fused_unknown_or_outside_roi"] += len(objects) - len(
                variants["full_fusion"]
            )
            metrics, assignments = {}, {}
            for variant, predictions in variants.items():
                counts, assignments[variant] = match(predictions, truth)
                total = Counter()
                for cls in CLASSES:
                    populations[variant][cls].update(counts[cls])
                    total.update(counts[cls])
                metrics[variant] = rates(total)
            audit.update(
                {
                    "objects": len(out.object),
                    "unknowns": len(out.unknown),
                    "conflicts": sum(o.class_conflict for o in out.object),
                    "ground_valid_fraction": audit.get("ground_valid_points", 0)
                    / sample["points"],
                    "repairs": out.health.consistency_repairs,
                    "latency_ms": out.health.latency_ms,
                }
            )
            report["frames"].append(
                {
                    "scene": run["scene"],
                    "file": name,
                    "sample_token": sample["sample_token"],
                    "accuracy": metrics,
                    "audit": audit,
                    "matches": assignments,
                }
            )
            points = []
            trusted_ids = {
                point_id
                for view in out.point_view
                if view.kind == 6
                for point_id in view.point_id
            }
            stride = max(1, math.ceil(len(out.point_evidence) / max_points))
            for p, d in zip(out.point_evidence[::stride], out.point_decision[::stride]):
                if p.finite:
                    points.append(
                        [
                            round(p.position.x, 3),
                            round(p.position.y, 3),
                            round(p.position.z, 3),
                            p.raw_semantic_label,
                            p.group,
                            d.explanation,
                            p.point_id,
                            round(p.semantic_probability, 4),
                            d.object_id if d.HasField("object_id") else None,
                            list(d.reason),
                            d.source,
                            None,
                            int(p.point_id in trusted_ids),
                        ]
                    )
            cells = [
                [
                    c.x_index,
                    c.y_index,
                    int(c.occupancy_obs),
                    int(c.occupancy_det),
                    int(c.occupancy_unknown),
                    c.traversability,
                    int(c.ground_supported),
                    c.visibility_mask,
                ]
                for c in out.space_map.cell
            ]
            unknowns = [
                {
                    "id": u.id,
                    "polygon": [[v.x, v.y] for v in u.polygon],
                    "points": len(u.point_id),
                    "related": list(u.related_object_id),
                    "corridor": u.in_corridor,
                    "confirmed": u.confirmed,
                    "reason": [labels["Reason"][r] for r in u.reason],
                    "source": labels["Source"][u.source],
                    "point_supported_footprint": u.point_supported_footprint,
                }
                for u in out.unknown
            ]
            camera = None
            if gt["camera"]:
                camera = f"camera/{len(viewer['frames']):04d}.jpg"
                with Image.open(gt["camera"]) as image:
                    image.thumbnail((800, 450))
                    image.convert("RGB").save(output / camera, quality=80)
            viewer["frames"].append(
                {
                    "scene": run["scene"],
                    "file": name,
                    "sample_token": sample["sample_token"],
                    "stamp": sample["measurement_time"],
                    "points": points,
                    "stride": stride,
                    "original_points": sample["points"],
                    "raw": raw,
                    "objects": objects,
                    "truth": gt["boxes"],
                    "unknowns": unknowns,
                    "cells": cells,
                    "grid": {
                        "x": out.space_map.origin_x,
                        "y": out.space_map.origin_y,
                        "resolution": out.space_map.resolution,
                    },
                    "camera": camera,
                    "camera_skew": gt["camera_timestamp_skew_seconds"],
                    "accuracy": metrics,
                    "audit": audit,
                    "detection_model_revision": det.model_revision,
                }
            )
            for path in (inputs / name, out_path, det_path, seg_path, cached_path):
                source_hashes[str(path)] = digest(path)
    for variant, classes in populations.items():
        total = Counter()
        for counts in classes.values():
            total.update(counts)
        report["accuracy"][variant] = {
            "total": rates(total),
            "per_class": {cls: rates(c) for cls, c in classes.items()},
        }
    report["excluded"] = dict(excluded)
    report["modes"] = {
        str(mode): {
            key: distribution(
                [float(r[key]) for r in summaries if int(r["mode"]) == mode]
            )
            for key in (
                "objects",
                "unknowns",
                "conflicts",
                "free_cells",
                "repairs",
                "seg_ms",
                "fusion_ms",
            )
        }
        for mode in range(4)
    }
    report["source_sha256"] = source_hashes
    for root in (Path(__file__).parent, Path("modules/lidar_fusion")):
        for path in sorted(root.rglob("*")):
            if path.is_file() and (
                path.suffix in (".py", ".cc", ".h", ".proto", ".txt", ".js", ".html")
                or path.name == "BUILD"
            ):
                source_hashes[str(path)] = digest(path)
    (output / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    with (output / "frames.csv").open("x") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "scene",
                "file",
                "objects",
                "unknowns",
                "conflicts",
                "ground_valid_fraction",
                "repairs",
                "latency_ms",
            ],
        )
        writer.writeheader()
        for frame in report["frames"]:
            writer.writerow(
                {
                    "scene": frame["scene"],
                    "file": frame["file"],
                    **{k: frame["audit"][k] for k in writer.fieldnames[2:]},
                }
            )
    payload = json.dumps(viewer, separators=(",", ":"), allow_nan=False).replace(
        "<", "\\u003c"
    )
    (output / "data.js").write_text("window.DATA=" + payload + ";\n")
    for name in ("index.html", "viewer.js"):
        shutil.copyfile(Path(__file__).with_name(name), output / name)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("experiment", "schema", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--max-points", type=int, default=8000)
    args = parser.parse_args()
    if not 100 <= args.max_points <= 100000:
        parser.error("--max-points must be in [100, 100000]; display only")
    result = export(args.experiment, args.schema, args.output, args.max_points)
    print(
        json.dumps(
            {"frames": len(result["frames"]), "accuracy": result["accuracy"]}, indent=2
        )
    )


if __name__ == "__main__":
    main()
