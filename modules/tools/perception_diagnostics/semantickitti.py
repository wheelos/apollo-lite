#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Independent SemanticKITTI point/visible-instance diagnostics, never box AP."""

import argparse
import csv
import json
import math
import shutil
from collections import Counter
from pathlib import Path

import numpy as np
from evaluate_baseline import audit_frame, coverage, digest, distribution, require
from nuscenes import (
    UNIFIED,
    box,
    enum_names,
    native_fields,
    object_view,
    ranking_score,
    rates,
    unknown_view,
)
from proto import Reader, require_full_evidence

# Official semantic-kitti-api/config/semantic-kitti.yaml learning_map.
LEARNING_MAP = {
    0: 0,
    1: 0,
    10: 1,
    11: 2,
    13: 5,
    15: 3,
    16: 5,
    18: 4,
    20: 5,
    30: 6,
    31: 7,
    32: 8,
    40: 9,
    44: 10,
    48: 11,
    49: 12,
    50: 13,
    51: 14,
    52: 0,
    60: 9,
    70: 15,
    71: 16,
    72: 17,
    80: 18,
    81: 19,
    99: 0,
    252: 1,
    253: 7,
    254: 6,
    255: 8,
    256: 5,
    257: 5,
    258: 4,
    259: 5,
}
NAMES = (
    "ignored",
    "car",
    "bicycle",
    "motorcycle",
    "truck",
    "other-vehicle",
    "person",
    "bicyclist",
    "motorcyclist",
    "road",
    "parking",
    "sidewalk",
    "other-ground",
    "building",
    "fence",
    "vegetation",
    "trunk",
    "terrain",
    "pole",
    "traffic-sign",
)
CLASS_MAP = {1: 1, 2: 5, 3: 6, 4: 2, 5: 7, 6: 4, 7: 5, 8: 6}
GROUND = (9, 10, 11, 12, 17)


def learning_labels(raw):
    values = np.asarray(raw, dtype=np.uint32) & 65535
    unknown = set(np.unique(values).tolist()) - LEARNING_MAP.keys()
    require(not unknown, f"unmapped official raw classes: {sorted(unknown)}")
    result = np.zeros(len(values), dtype=np.int64)
    for label, mapped in LEARNING_MAP.items():
        result[values == label] = mapped
    return result


def confusion(truth, predicted):
    truth, predicted = np.asarray(truth), np.asarray(predicted)
    require(truth.shape == predicted.shape, "semantic point mapping mismatch")
    require(np.all((predicted >= 0) & (predicted < 20)), "invalid prediction class")
    keep = truth != 0
    return np.bincount(truth[keep] * 20 + predicted[keep], minlength=400).reshape(
        20, 20
    )


def semantic_metrics(matrix):
    tp = matrix.diagonal()
    union = matrix.sum(0) + matrix.sum(1) - tp
    iou = np.divide(tp, union, out=np.zeros(20), where=union != 0)
    present = union[1:] != 0
    return {
        "valid_points": int(matrix.sum()),
        "accuracy": float(tp.sum() / matrix.sum()) if matrix.sum() else None,
        "miou_present_classes": (
            float(iou[1:][present].mean()) if present.any() else None
        ),
        "miou_19_zero_for_absent": float(iou[1:].mean()),
        "classes_with_nonzero_union": int(present.sum()),
        "per_class": {
            NAMES[i]: {
                "iou": float(iou[i]) if union[i] else None,
                "truth_points": int(matrix[i].sum()),
                "predicted_points": int(matrix[:, i].sum()),
            }
            for i in range(1, 20)
        },
        "confusion_true_rows_predicted_columns": matrix.tolist(),
    }


def binary_counts(truth, predicted, mask):
    return Counter(
        tp=int(np.count_nonzero(mask & truth & predicted)),
        fp=int(np.count_nonzero(mask & ~truth & predicted)),
        fn=int(np.count_nonzero(mask & truth & ~predicted)),
        tn=int(np.count_nonzero(mask & ~truth & ~predicted)),
    )


def unknown_support_metrics(points, truth, components):
    """Audit actual support; semantic ground is not physical free-space truth."""
    points, truth = np.asarray(points), np.asarray(truth)
    require(points.shape == (len(truth), 3), "unknown support point mapping mismatch")
    summaries = []
    union = np.zeros(len(truth), dtype=bool)
    for ids in components:
        indices = np.unique(np.asarray(ids, dtype=np.int64))
        require(
            np.all((indices >= 0) & (indices < len(truth))),
            "unknown support point index out of range",
        )
        indices = indices[np.isfinite(points[indices]).all(axis=1)]
        union[indices] = True
        valid = indices[truth[indices] != 0]
        low = points[indices, :2].min(axis=0) if len(indices) else None
        high = points[indices, :2].max(axis=0) if len(indices) else None
        summaries.append(
            {
                "points": len(indices),
                "valid_semantic_points": len(valid),
                "semantic_ground_proxy_points": int(
                    np.count_nonzero(np.isin(truth[valid], GROUND))
                ),
                "semantic_thing_points": int(
                    np.count_nonzero((truth[valid] >= 1) & (truth[valid] <= 8))
                ),
                "support_aabb_area": (
                    float(np.prod(high - low)) if len(indices) else 0.0
                ),
                "support_aabb_contains_origin": bool(
                    len(indices) and np.all(low <= 0) and np.all(high >= 0)
                ),
            }
        )
    return {
        "components": len(summaries),
        "unique_finite_support_points": int(union.sum()),
        "largest_component": max(summaries, key=lambda s: s["points"], default=None),
        "components_spanning_origin_aabb": sum(
            s["support_aabb_contains_origin"] for s in summaries
        ),
    }


def inside_box(points, item):
    center, size = np.asarray(item["center"]), np.asarray(item["size"])
    yaw = item["yaw"]
    c, s = math.cos(yaw), math.sin(yaw)
    relative = points - center
    local = relative @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.all(np.abs(local) <= size / 2 + 1e-6, axis=1)


def visible_instances(points, truth, instance, mask):
    """Return original visible point sets; no amodal GT boxes are invented."""
    instances, outlines = [], []
    for identity in np.unique(instance[mask & (truth > 0) & (truth <= 8)]):
        if identity == 0:
            continue
        indices = np.flatnonzero(
            mask & (instance == identity) & (truth > 0) & (truth <= 8)
        )
        if len(indices) < 5:
            continue
        unified = Counter(CLASS_MAP[int(truth[i])] for i in indices).most_common(1)[0][
            0
        ]
        if unified not in (1, 4, 5):
            continue
        instances.append({"id": int(identity), "class_id": unified, "indices": indices})
        low, high = points[indices].min(0), points[indices].max(0)
        outlines.append(
            {
                "id": int(identity),
                "class": UNIFIED[unified],
                "center": ((low + high) / 2).tolist(),
                "size": (high - low).tolist(),
                "yaw": 0.0,
                "points": len(indices),
                "source": "visible labeled points enclosure, NOT amodal GT box",
            }
        )
    return instances, outlines


def instance_counts(candidates, instances, valid):
    available = set(range(len(instances)))
    result = Counter(tp=0, fp=0, fn=0)
    matched = []
    for candidate in sorted(candidates, key=lambda p: -ranking_score(p)):
        indices = np.flatnonzero(candidate["mask"] & valid)
        best, value = None, 0.0
        for j in sorted(available):
            gt = instances[j]
            if candidate["class_id"] != gt["class_id"]:
                continue
            intersection = len(
                np.intersect1d(indices, gt["indices"], assume_unique=True)
            )
            union = len(indices) + len(gt["indices"]) - intersection
            iou = intersection / union if union else 0
            if iou > value:
                best, value = j, iou
        if best is not None and value >= 0.5:
            result["tp"] += 1
            available.remove(best)
            matched.append(
                {
                    "candidate": candidate["id"],
                    "instance": instances[best]["id"],
                    "visible_point_iou": value,
                }
            )
        else:
            result["fp"] += 1
    result["fn"] = len(available)
    return result, matched


def replay_caches(arguments):
    options = arguments[4:]
    flags = {"--profile", "--shadow-partitions"}
    cached = len(arguments) in (4, 5, 6) or any(option in flags for option in options)
    if not cached:
        if len(arguments) == 7:
            return None
        if len(arguments) == 9:
            return Path(arguments[7]), Path(arguments[8])
    require(cached and 4 <= len(arguments) <= 7, "unsupported cached replay arguments")
    if options and not options[0].startswith("--"):
        require(
            options[0].isdigit() and 1 <= int(options[0]) <= 100,
            "invalid cached replay repeats",
        )
        options = options[1:]
    require(
        all(option in flags for option in options)
        and len(set(options)) == len(options),
        "invalid cached replay options",
    )
    return Path(arguments[2]), Path(arguments[2])


def export(inputs, results, schema, output, allow_semantic_oracle=False):
    output.mkdir(parents=True, exist_ok=False)
    manifest_path = inputs / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    rows = list(csv.DictReader((results / "summary.csv").open()))
    coverage(manifest, rows)
    reader = Reader(schema)
    prefix = "apollo.lidar_fusion."
    config_path = results / "config.pb"
    config = reader.read(config_path, prefix + "FusionConfig")
    hashes = {
        str(p): digest(p)
        for p in (
            manifest_path,
            schema,
            config_path,
            inputs / "fusion.pb.txt",
            results / "summary.csv",
        )
    }
    arguments = (results / "arguments.txt").read_text().splitlines()
    caches = replay_caches(arguments)
    oracle = None
    if caches is not None:
        oracle_path = caches[1] / "oracle.json"
        if oracle_path.exists():
            require(
                allow_semantic_oracle, "requires explicit semantic-oracle diagnostic"
            )
            oracle = json.loads(oracle_path.read_text())
            require(
                oracle.get("diagnostic_only")
                and oracle.get("semantic_ground_truth_substitution"),
                "invalid oracle provenance",
            )
            require(
                oracle["input_manifest_sha256"] == digest(manifest_path)
                and oracle["schema_sha256"] == digest(schema),
                "oracle input/schema mismatch",
            )
            hashes[str(oracle_path)] = digest(oracle_path)
        else:
            require(not allow_semantic_oracle, "no semantic-oracle provenance")
    else:
        require(not allow_semantic_oracle, "oracle requires fully cached replay")
    model_indices = (0, 3, 4) if caches is None else (0,)
    for path in [results / "arguments.txt"] + [
        Path(arguments[i]) for i in model_indices
    ]:
        hashes[str(path)] = digest(path)
    labels = {
        n: enum_names(reader, n)
        for n in ("Group", "UnifiedClass", "Explanation", "Source", "Reason")
    }
    protocol = {
        "semantic": (
            "official learning_map; ignore GT class 0, count prediction 0 as error"
        ),
        "point_views": "THING truth 1..8; separate geometric ground proxy",
        "instance_scope": "x in [0,60), y in [-39.68,39.68), valid semantic labels",
        "visible_instances": "original instance ID, >=5 points, CAR/PED/CYCLIST",
        "matching": "score-ordered class-aware one-to-one visible-point IoU >=0.5",
        "candidate_membership": "geometric membership, all variants; not support IDs",
        "view_scope": "point_views: whole sweep; common_domain: instance XY ROI",
        "limitations": [
            "Not official SemanticKITTI leaderboard score; selected validation sweeps",
            "NO amodal box GT/AP; green boxes enclose visible labeled points only",
            "Ground vs semantic surface is a proxy, not physical support or FREE",
            "Remission quantized to uint8; sensor z=-1.73 is diagnostic assumption",
            "No pose/temporal or physical free-space accuracy claim",
            "Unknown support AABB and semantic ground counts are diagnostics, "
            "not occupied polygon truth or a physical false-obstacle label",
        ],
    }
    viewer = {
        "projection_version": 2,
        "title": "LiDAR fusion: SemanticKITTI sequence 08 diagnostics",
        "banner": (
            "Labeled subset. Green = visible-point enclosure, NOT amodal GT. "
            "No box AP or verified FREE. Display subsampling only."
        ),
        "semantic_ground_truth": True,
        "semantic_labels": dict(enumerate(NAMES)),
        "frames": [],
        "labels": labels,
        "protocol": protocol,
    }
    protocol["semantic_ground_truth_substitution"] = bool(oracle)
    protocol["model_inference_recomputed"] = caches is None
    if oracle:
        viewer["title"] += " SEMANTIC ORACLE"
        viewer["banner"] = (
            "SEMANTIC GT SUBSTITUTION, NOT MODEL PERFORMANCE. "
            "Detector/geometry unchanged; ignored labels keep original prediction. "
            "Green = visible-point enclosure, NOT amodal GT."
        )
    matrix = np.zeros((20, 20), dtype=np.int64)
    views = {
        name: Counter()
        for name in (
            "semantic_thing",
            "geometric_ground_proxy",
            "object_points_thing",
            "trusted_object_points_thing",
            "non_ground_points_thing",
            "foreground_explanation_thing",
        )
    }
    totals = {
        name: Counter()
        for name in (
            "raw_detector_0_1",
            "raw_detector_0_3",
            "accepted_detector",
            "accepted_detector_0_3",
            "full_fusion",
            "full_fusion_det_0_3",
            "classified_fusion",
        )
    }
    common_views = {name: Counter() for name in views}
    candidate_audit = Counter()
    frames = []
    for sample in manifest["samples"]:
        name = sample["file"]
        source, label_path = Path(sample["source"]), Path(sample["labels"])
        for path, expected in (
            (inputs / name, sample["sha256"]),
            (source, sample["source_sha256"]),
            (label_path, sample["labels_sha256"]),
        ):
            require(digest(path) == expected, f"changed input: {path}")
        source_points = np.fromfile(source, dtype="<f4").reshape(-1, 4)
        packed = np.fromfile(label_path, dtype="<u4")
        truth, identity = learning_labels(packed), packed >> 16
        cloud = reader.read(inputs / name, "apollo.drivers.PointCloud")
        out_path = results / (name + ".fusion.pb")
        out = reader.read(out_path, prefix + "FusionOutput")
        require_full_evidence(out)
        require(
            out.health.mode == 0 and out.health.det_valid and out.health.seg_valid,
            "NORMAL source invalid",
        )
        require(
            len(truth) == len(out.point_evidence) == sample["points"], "point count"
        )
        points = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
        )
        require(
            np.array_equal(source_points[:, :3], points), "source XYZ/order altered"
        )
        require(
            np.array_equal(
                source_points[:, :3], np.array([[p.x, p.y, p.z] for p in cloud.point])
            ),
            "cloud XYZ/order altered",
        )
        require(
            np.array_equal(
                np.rint(source_points[:, 3].astype(float) * 255),
                [p.intensity for p in cloud.point],
            ),
            "intensity conversion altered",
        )
        det_path, seg_path = (results / (name + s) for s in (".det.pb", ".seg.pb"))
        det = reader.read(
            det_path, "apollo.lidar_object_detection.LidarObjectDetectionFrameResult"
        )
        seg = reader.read(
            seg_path,
            "apollo.lidar_semantic_segmentation.LidarSemanticSegmentationResult",
        )
        if caches is not None:
            for directory, artifact in zip(caches, (det_path, seg_path)):
                original = directory / artifact.name
                require(
                    artifact.read_bytes() == original.read_bytes(),
                    "cached model evidence changed",
                )
                hashes[str(original)] = digest(original)
        if oracle:
            original = next(row for row in oracle["samples"] if row["file"] == name)
            require(
                digest(seg_path) == original["oracle_sha256"],
                "semantic oracle payload mismatch",
            )
        semantic = {p.point_index: p for p in seg.point_label}
        require(len(semantic) == len(truth), "incomplete semantic result")
        predicted = np.array([p.raw_semantic_label for p in out.point_evidence])
        for p in out.point_evidence:
            require(
                p.raw_semantic_label == semantic[p.point_id].semantic_label
                and p.semantic_probability == semantic[p.point_id].confidence,
                "altered raw semantic evidence",
            )
        current = confusion(truth, predicted)
        matrix += current
        mask = truth != 0
        thing = (truth > 0) & (truth <= 8)
        common = (
            mask
            & (points[:, 0] >= 0)
            & (points[:, 0] < 60)
            & (points[:, 1] >= -39.68)
            & (points[:, 1] < 39.68)
        )
        view_ids = {v.kind: np.asarray(v.point_id, dtype=int) for v in out.point_view}

        def view(kind):
            result = np.zeros(len(truth), dtype=bool)
            result[view_ids.get(kind, np.array([], dtype=int))] = True
            return result

        explanation = np.array([d.explanation for d in out.point_decision])
        current_views = {
            "semantic_thing": binary_counts(
                thing, (predicted > 0) & (predicted <= 8), mask
            ),
            "geometric_ground_proxy": binary_counts(
                np.isin(truth, GROUND), view(0), mask
            ),
            "object_points_thing": binary_counts(thing, view(2), mask),
            "trusted_object_points_thing": binary_counts(thing, view(6), mask),
            "non_ground_points_thing": binary_counts(thing, view(1), mask),
            "foreground_explanation_thing": binary_counts(
                thing, ~np.isin(explanation, (1, 2, 5, 6, 7)), mask
            ),
        }
        for key, counts in current_views.items():
            views[key].update(counts)
        common_predictions = {
            "semantic_thing": ((predicted > 0) & (predicted <= 8), thing),
            "geometric_ground_proxy": (view(0), np.isin(truth, GROUND)),
            "object_points_thing": (view(2), thing),
            "trusted_object_points_thing": (view(6), thing),
            "non_ground_points_thing": (view(1), thing),
            "foreground_explanation_thing": (
                ~np.isin(explanation, (1, 2, 5, 6, 7)),
                thing,
            ),
        }
        current_common = {
            k: binary_counts(target, prediction, common)
            for k, (prediction, target) in common_predictions.items()
        }
        for key, counts in current_common.items():
            common_views[key].update(counts)
        raw, objects, raw_candidates, candidates = [], [], [], []
        for h in out.hypothesis:
            require(
                h.raw_detection == det.detection[h.detection_index],
                "altered raw detection",
            )
            item = box(h)
            item.update(
                {
                    "class": UNIFIED.get(h.unified_class, "other"),
                    "id": h.detection_index,
                    "score": h.raw_detection.score,
                    "source": "raw PointPillars",
                    "evidence": native_fields(h),
                }
            )
            raw.append(item)
            raw_candidates.append(
                {**item, "class_id": h.unified_class, "mask": inside_box(points, item)}
            )
        for obj in out.object:
            item = object_view(obj, labels)
            objects.append(item)
            state = ("INSUFFICIENT", "SUPPORT", "CONFLICT")[obj.consistency]
            score_bin = (
                "score_lt_0_3"
                if ranking_score(item) < config.unsupported_score_min
                else "score_ge_0_3"
            )
            candidate_audit[f"{item['source']}:{state}:{score_bin}"] += 1
            candidates.append(
                {
                    **item,
                    "class_id": obj.unified_class,
                    "mask": inside_box(points, item),
                }
            )
        instances, outlines = visible_instances(points, truth, identity, common)

        def selected(items, threshold):
            return [
                p
                for p in items
                if p["class_id"] in (1, 4, 5)
                and ranking_score(p) >= threshold
                and 0 <= p["center"][0] < 60
                and -39.68 <= p["center"][1] < 39.68
            ]

        variants = {
            "raw_detector_0_1": selected(raw_candidates, config.evidence_score_min),
            "raw_detector_0_3": selected(raw_candidates, config.unsupported_score_min),
            "accepted_detector": selected(
                [c for c in candidates if c["source"] == "DET"], 0
            ),
            "accepted_detector_0_3": selected(
                [c for c in candidates if c["source"] == "DET"],
                config.unsupported_score_min,
            ),
            "full_fusion": selected(candidates, 0),
            "full_fusion_det_0_3": selected(
                [
                    c
                    for c in candidates
                    if c["source"] != "DET"
                    or ranking_score(c) >= config.unsupported_score_min
                ],
                0,
            ),
            "classified_fusion": selected(
                [c for c in candidates if c["classification_use"] == 1], 0
            ),
        }
        instance_metrics, matches = {}, {}
        annotation_coverage = {}
        for variant, values in variants.items():
            counts, matches[variant] = instance_counts(values, instances, common)
            totals[variant].update(counts)
            instance_metrics[variant] = rates(counts)
            annotation_coverage[variant] = {
                "predictions": len(values),
                "no_valid_labeled_returns": sum(
                    not np.any(p["mask"] & common) for p in values
                ),
                "boundary": (
                    "Unmatched remains protocol FP; no valid returns is "
                    "insufficient annotation, not proof of physical absence"
                ),
            }
        audit = dict(audit_frame(out, config))
        audit["identity_uncertain_objects"] = sum(
            obj.classification_use == 2 for obj in out.object
        )
        audit["retained_weak_hypotheses"] = sum(h.decision == 2 for h in out.hypothesis)
        audit.update(
            {
                "objects": len(out.object),
                "unknowns": len(out.unknown),
                "conflicts": sum(o.class_conflict for o in out.object),
                "thing_points_misclassified_as_ground": int(
                    np.count_nonzero(thing & view(0))
                ),
                "repairs": out.health.consistency_repairs,
                "latency_ms": out.health.latency_ms,
            }
        )
        frame_report = {
            "file": name,
            "ordinal": sample["ordinal"],
            "semantic": semantic_metrics(current),
            "point_views": {k: rates(v) for k, v in current_views.items()},
            "point_views_common_domain": {
                k: rates(v) for k, v in current_common.items()
            },
            "visible_instance_metrics": instance_metrics,
            "matches": matches,
            "annotation_coverage": annotation_coverage,
            "unknown_support": unknown_support_metrics(
                points, truth, (u.point_id for u in out.unknown)
            ),
            "audit": audit,
        }
        frames.append(frame_report)
        stride = max(1, math.ceil(len(truth) / 8000))
        point_data = []
        trusted_ids = set(view_ids.get(6, []))
        for i in range(0, len(truth), stride):
            p, d = out.point_evidence[i], out.point_decision[i]
            if p.finite:
                point_data.append(
                    [
                        round(p.position.x, 3),
                        round(p.position.y, 3),
                        round(p.position.z, 3),
                        p.raw_semantic_label,
                        p.group,
                        d.explanation,
                        i,
                        round(p.semantic_probability, 4),
                        d.object_id if d.HasField("object_id") else None,
                        list(d.reason),
                        d.source,
                        int(truth[i]),
                        int(i in trusted_ids),
                        int(identity[i]),
                        native_fields(p),
                        native_fields(d),
                    ]
                )
        viewer["frames"].append(
            {
                "scene": "SemanticKITTI-08",
                "file": name,
                "sample_token": f"08/{sample['ordinal']:06d}",
                "stamp": sample["measurement_time"],
                "points": point_data,
                "stride": stride,
                "original_points": len(truth),
                "raw": raw,
                "objects": objects,
                "truth": outlines,
                "unknowns": [unknown_view(u, labels) for u in out.unknown],
                "shadow_partitions": [
                    native_fields(p) for p in getattr(out, "shadow_partition", ())
                ],
                "cells": [
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
                ],
                "grid": {
                    "x": out.space_map.origin_x,
                    "y": out.space_map.origin_y,
                    "resolution": out.space_map.resolution,
                },
                "camera": None,
                "camera_skew": None,
                "accuracy": {
                    "semantic": semantic_metrics(current),
                    "point_views": frame_report["point_views"],
                    "visible_instances": instance_metrics,
                },
                "audit": audit,
                "detection_model_revision": det.model_revision,
                "health": native_fields(out.health),
                "semantic_identity_qualified": (
                    out.health.seg_valid
                    and not config.quality.semantic_identity_unverified
                ),
            }
        )
        for path in (source, label_path, inputs / name, out_path, det_path, seg_path):
            hashes[str(path)] = digest(path)
    report = {
        "dataset": manifest["dataset"],
        "manifest": manifest,
        "protocol": protocol,
        "official_metrics": False,
        "semantic_oracle_diagnostic_only": bool(oracle),
        "model_performance": not bool(oracle),
        "oracle_provenance": oracle,
        "frames": frames,
        "semantic": semantic_metrics(matrix),
        "point_views": {k: rates(v) for k, v in views.items()},
        "point_views_common_domain": {k: rates(v) for k, v in common_views.items()},
        "accepted_candidate_breakdown": dict(candidate_audit),
        "visible_instance_metrics": {k: rates(v) for k, v in totals.items()},
        "timing_conditions": "CPU dev, shared host; not controlled performance",
        "modes": {
            str(mode): {
                key: distribution(
                    [float(r[key]) for r in rows if int(r["mode"]) == mode]
                )
                for key in (
                    "objects",
                    "unknowns",
                    "conflicts",
                    "free_cells",
                    "repairs",
                    "det_ms",
                    "seg_ms",
                    "fusion_ms",
                )
            }
            for mode in range(4)
        },
        "source_sha256": hashes,
    }
    for root in (Path(__file__).parent, Path("modules/lidar_fusion")):
        for path in root.rglob("*"):
            if path.is_file() and (
                path.suffix in (".cc", ".h", ".py", ".proto", ".js", ".html")
                or path.name == "BUILD"
            ):
                hashes[str(path)] = digest(path)
    (output / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    with (output / "frames.csv").open("x") as stream:
        fields = [
            "file",
            "semantic_accuracy",
            "miou_present_classes",
            "raw_tp",
            "raw_fp",
            "fusion_tp",
            "fusion_fp",
            "objects",
            "unknowns",
            "conflicts",
            "thing_ground_errors",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for frame in frames:
            raw = frame["visible_instance_metrics"]["raw_detector_0_3"]
            fused = frame["visible_instance_metrics"]["full_fusion"]
            audit = frame["audit"]
            writer.writerow(
                {
                    "file": frame["file"],
                    "semantic_accuracy": frame["semantic"]["accuracy"],
                    "miou_present_classes": frame["semantic"]["miou_present_classes"],
                    "raw_tp": raw["tp"],
                    "raw_fp": raw["fp"],
                    "fusion_tp": fused["tp"],
                    "fusion_fp": fused["fp"],
                    "objects": audit["objects"],
                    "unknowns": audit["unknowns"],
                    "conflicts": audit["conflicts"],
                    "thing_ground_errors": audit[
                        "thing_points_misclassified_as_ground"
                    ],
                }
            )
    (output / "data.js").write_text(
        "window.DATA="
        + json.dumps(viewer, separators=(",", ":"), allow_nan=False).replace(
            "<", "\\u003c"
        )
        + ";\n"
    )
    for name in ("index.html", "viewer.js"):
        shutil.copyfile(Path(__file__).with_name(name), output / name)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("input", "results", "schema", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--allow-semantic-oracle", action="store_true")
    args = parser.parse_args()
    report = export(
        args.input, args.results, args.schema, args.output, args.allow_semantic_oracle
    )
    print(
        json.dumps(
            {key: report[key] for key in ("point_views", "visible_instance_metrics")},
            indent=2,
        )
    )
    print(
        "semantic accuracy/mIoU",
        report["semantic"]["accuracy"],
        report["semantic"]["miou_present_classes"],
    )


if __name__ == "__main__":
    main()
