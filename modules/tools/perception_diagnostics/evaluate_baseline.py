#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Audit frozen replay outputs; report operational metrics, never unlabeled accuracy."""

import argparse
import csv
import hashlib
import json
import math
import statistics
from collections import Counter
from pathlib import Path

from proto import Reader


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for data in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(data)
    return value.hexdigest()


def distribution(values):
    ordered = sorted(values)
    require(bool(ordered), "empty metric population")

    def percentile(p):
        rank = (len(ordered) - 1) * p
        low = math.floor(rank)
        return ordered[low] + (ordered[math.ceil(rank)] - ordered[low]) * (rank - low)

    return {
        "count": len(ordered),
        "total": sum(ordered),
        "mean": statistics.mean(ordered),
        "p50": percentile(0.5),
        "p95": percentile(0.95),
        "p99": percentile(0.99),
        "min": ordered[0],
        "max": ordered[-1],
    }


def coverage(manifest, rows):
    expected = {
        (sample["file"], str(mode))
        for sample in manifest["samples"]
        for mode in range(4)
    }
    actual = {(row["frame"], row["mode"]) for row in rows}
    require(
        actual == expected and len(rows) == len(expected),
        "incomplete or duplicate frame/mode coverage",
    )
    require(
        all(int(row["free_cells"]) == 0 for row in rows),
        "diagnostic input unexpectedly emitted FREE",
    )


def audit_frame(out, config):
    points, decisions = out.point_evidence, out.point_decision
    require(len(points) == len(decisions), "point/decision size mismatch")
    object_ids = {item.id for item in out.object}
    require(len(object_ids) == len(out.object), "duplicate object IDs")
    cells = {(cell.x_index, cell.y_index): cell for cell in out.space_map.cell}
    require(len(cells) == len(out.space_map.cell), "duplicate cells")
    expected = set()
    metrics = Counter()
    for i, (p, decision) in enumerate(zip(points, decisions)):
        require(p.point_id == i and decision.point_id == i, "point ID mismatch")
        require(
            not decision.HasField("object_id") or decision.object_id in object_ids,
            "dangling point owner",
        )
        metrics[f"explanation_{decision.explanation}"] += 1
        metrics["ground_points"] += p.is_ground_geo
        metrics["ground_valid_points"] += p.ground_valid
        if not p.finite:
            continue
        require(
            all(math.isfinite(v) for v in (p.position.x, p.position.y, p.position.z)),
            "nonfinite evidence",
        )
        if not (
            config.min_x <= p.position.x < config.max_x
            and config.min_y <= p.position.y < config.max_y
        ):
            continue
        metrics["roi_points"] += 1
        metrics[f"roi_group_{p.group}"] += 1
        metrics["roi_ground_valid_points"] += p.ground_valid
        eligible = (
            not p.ground_valid
            or config.obstacle_min_height
            <= p.height_above_ground
            <= config.clearance_height
        )
        if decision.explanation != 2 and not p.is_ground_geo and eligible:
            expected.add(
                (
                    math.floor((p.position.x - config.min_x) / config.resolution),
                    math.floor((p.position.y - config.min_y) / config.resolution),
                )
            )
    measured = {key for key, cell in cells.items() if cell.occupancy_obs}
    require(expected == measured, "point-derived measured occupancy differs")
    for cell in cells.values():
        require(all(ref in object_ids for ref in cell.object_ref), "dangling cell ref")
        require(cell.traversability not in (2, 3), "unverified FREE")
        require(
            not (cell.occupancy_obs or cell.occupancy_det or cell.occupancy_unknown)
            or cell.traversability == 1,
            "occupied/traversability contradiction",
        )
        metrics["ground_supported_cells"] += cell.ground_supported
        metrics["endpoint_observed_cells"] += cell.endpoint_observed
        metrics["diagnostic_visibility_cells"] += cell.visibility_mask != 0
        metrics["unknown_traversability_cells"] += cell.traversability == 0
    for obj in out.object:
        require(obj.consistency != 2 or obj.class_conflict, "unflagged conflict")
        require(all(i < len(points) for i in obj.point_id), "invalid object point")
        metrics[f"object_source_{obj.source}"] += 1
        metrics[f"object_class_{obj.unified_class}"] += 1
    for unknown in out.unknown:
        require(
            all(ref in object_ids for ref in unknown.related_object_id),
            "dangling unknown ref",
        )
        require(
            all(rel.object_id in unknown.related_object_id for rel in unknown.relation),
            "inconsistent relationship",
        )
        require(all(i < len(points) for i in unknown.point_id), "invalid unknown point")
        metrics["corridor_unknowns"] += unknown.in_corridor
        metrics["small_corridor_unknowns"] += (
            unknown.in_corridor and len(unknown.point_id) < config.unknown_min_points
        )
        metrics["related_unknowns"] += bool(unknown.related_object_id)
        for relation in unknown.relation:
            metrics[f"relation_{relation.kind}"] += 1
    for view in out.point_view:
        require(all(i < len(points) for i in view.point_id), "invalid view index")
        metrics[f"view_{view.kind}_points"] += len(view.point_id)
    metrics["measured_cells"] = len(measured)
    metrics["grid_cells"] = out.space_map.width * out.space_map.height
    return metrics


def write_rows(path, rows):
    fields = sorted(set().union(*(row.keys() for row in rows)))
    with path.open("x") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in (
        "input",
        "results",
        "temporal-input",
        "temporal-results",
        "schema",
        "output",
    ):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    manifest = json.loads((args.input / "manifest.json").read_text())
    temporal_manifest = json.loads((args.temporal_input / "manifest.json").read_text())
    rows = list(csv.DictReader((args.results / "summary.csv").open()))
    coverage(manifest, rows)
    reader = Reader(args.schema)
    prefix = "apollo.lidar_fusion."
    config = reader.read(args.results / "config.pb", prefix + "FusionConfig")
    report = {"schema_version": 1, "accuracy_evaluated": False, "modes": {}}
    report["label_names"] = {
        name: {
            value.number: value.name
            for value in reader.pool.FindEnumTypeByName(prefix + name).values
        }
        for name in (
            "Group",
            "Explanation",
            "Source",
            "Consistency",
            "UnifiedClass",
            "ViewKind",
            "RelationKind",
        )
    }
    mode_names = ("NORMAL", "NO_SEG", "NO_DET", "GEO_ONLY")
    for mode, name in enumerate(mode_names):
        selected = [row for row in rows if int(row["mode"]) == mode]
        report["modes"][name] = {
            field: distribution([float(row[field]) for row in selected])
            for field in selected[0]
            if field not in ("frame", "mode")
        }
    identities = {str(args.schema): digest(args.schema)}
    frame_rows = []
    normal_rows = {row["frame"]: row for row in rows if row["mode"] == "0"}
    for sample in manifest["samples"]:
        name = sample["file"]
        require(digest(args.input / name) == sample["sha256"], "changed cloud")
        frame_path = args.results / (name + ".fusion.pb")
        out = reader.read(frame_path, prefix + "FusionOutput")
        require(
            out.health.mode == 0 and out.health.det_valid and out.health.seg_valid,
            "NORMAL frame source invalid",
        )
        require(len(out.point_evidence) == sample["points"], "point count mismatch")
        det_path, seg_path = (
            args.results / (name + suffix) for suffix in (".det.pb", ".seg.pb")
        )
        det = reader.read(
            det_path, "apollo.lidar_object_detection.LidarObjectDetectionFrameResult"
        )
        seg = reader.read(
            seg_path,
            "apollo.lidar_semantic_segmentation.LidarSemanticSegmentationResult",
        )
        require(len(seg.point_label) == sample["points"], "incomplete semantic result")
        labels = {p.point_index: p for p in seg.point_label}
        require(len(labels) == sample["points"], "duplicate semantic mapping")
        for p in out.point_evidence:
            label = labels[p.point_id]
            require(
                p.raw_semantic_label == label.semantic_label
                and p.semantic_probability == label.confidence,
                "altered raw semantics",
            )
        for hypothesis in out.hypothesis:
            require(
                hypothesis.raw_detection == det.detection[hypothesis.detection_index],
                "altered raw detection",
            )
        metrics = audit_frame(out, config)
        require(
            metrics["measured_cells"] == int(normal_rows[name]["observed_cells"]),
            "CSV occupancy mismatch",
        )
        for h in out.hypothesis:
            metrics[f"hypothesis_state_{h.consistency}"] += 1
            metrics["rejected_instances_retaining_inference"] += not h.HasField(
                "object_id"
            )
        metrics["raw_detection_score_ge_0_3"] = sum(
            d.score >= 0.3 for d in det.detection
        )
        frame_rows.append({"frame": name, **metrics})
        for path in (frame_path, det_path, seg_path):
            identities[str(path)] = digest(path)
    write_rows(args.output / "single-detail.csv", frame_rows)
    numeric = sorted(set().union(*(set(row) - {"frame"} for row in frame_rows)))
    report["single_detail"] = {
        key: distribution([row.get(key, 0) for row in frame_rows]) for key in numeric
    }
    report["review_frames_not_error_labels"] = {
        field: [
            {"frame": row["frame"], field: int(row[field])}
            for row in sorted(
                normal_rows.values(), key=lambda row: int(row[field]), reverse=True
            )[:5]
        ]
        for field in ("conflicts", "unknowns", "repairs")
    }
    temporal_rows = []
    timing_rows = list(csv.DictReader((args.temporal_results / "temporal.csv").open()))
    require(
        len(timing_rows) == len(temporal_manifest["samples"])
        and {row["frame"] for row in timing_rows}
        == {sample["file"] for sample in temporal_manifest["samples"]},
        "incomplete temporal timing coverage",
    )
    temporal_config = reader.read(
        args.temporal_results / "config.pb", prefix + "FusionConfig"
    )
    for variant, suffix in (
        ("single", ".single.pb"),
        ("normal", ".temporal.pb"),
        ("dropout", ".dropout.pb"),
    ):
        previous = set()
        measured_by_frame = {}
        for index, sample in enumerate(temporal_manifest["samples"]):
            path = args.temporal_results / (sample["file"] + suffix)
            out = reader.read(path, prefix + "FusionOutput")
            if variant != "single":
                require(
                    out.health.temporal_enabled
                    and out.health.pose_valid
                    and out.health.diagnostic_pose,
                    "invalid diagnostic temporal frame",
                )
            require(
                out.health.mode
                == (2 if variant == "dropout" and index % 4 == 3 else 0),
                "dropout mode was not actually triggered",
            )
            require(
                len(out.point_evidence) == sample["points"],
                "temporal point count mismatch",
            )
            audit_frame(out, temporal_config)
            tracks = {t.id: t for t in out.track}
            require(len(tracks) == len(out.track), "duplicate track ID")
            if variant != "single":
                require(
                    all(o.track_id in tracks for o in out.object),
                    "dangling object track",
                )
                require(
                    all(u.track_id in tracks for u in out.unknown),
                    "dangling unknown track",
                )
            require(
                all(
                    not (t.source == 3 and t.observed)
                    or any(o.track_id == t.id and o.point_id for o in out.object)
                    for t in out.track
                    if not t.unknown
                ),
                "unsupported recovery",
            )
            current = set(tracks)
            temporal_rows.append(
                {
                    "frame": sample["file"],
                    "variant": variant,
                    "detection_dropped": int(variant == "dropout" and index % 4 == 3),
                    "objects": len(out.object),
                    "unknowns": len(out.unknown),
                    "tracks": len(tracks),
                    "new_track_ids": len(current - previous),
                    "continued_track_ids": len(current & previous),
                    "confirmed": sum(t.confirmed for t in tracks.values()),
                    "confirmed_unknowns": sum(u.confirmed for u in out.unknown),
                    "unobserved_tracks": sum(not t.observed for t in tracks.values()),
                    "recovered_objects": sum(o.source == 3 for o in out.object),
                    "history_cells": len(out.history_cell),
                    "repairs": out.health.consistency_repairs,
                    "reset_events": sum(r == 23 for r in out.risk_event),
                }
            )
            measured_by_frame[sample["file"]] = {
                (c.x_index, c.y_index) for c in out.space_map.cell if c.occupancy_obs
            }
            require(
                all(c.traversability not in (2, 3) for c in out.space_map.cell),
                "temporal emitted FREE",
            )
            identities[str(path)] = digest(path)
            previous = current
        if variant == "single":
            reference = measured_by_frame
        else:
            require(
                reference == measured_by_frame,
                "detector dropout changed measured occupancy",
            )
    write_rows(args.output / "temporal-detail.csv", temporal_rows)
    report["temporal"] = {
        variant: {
            key: distribution(
                [row[key] for row in temporal_rows if row["variant"] == variant]
            )
            for key in temporal_rows[0]
            if key not in ("frame", "variant")
        }
        for variant in ("single", "normal", "dropout")
    }
    for variant, column in (
        ("single", "baseline_ms"),
        ("normal", "fusion_ms"),
        ("dropout", "dropout_ms"),
    ):
        report["temporal"][variant]["fusion_ms"] = distribution(
            [float(row[column]) for row in timing_rows]
        )
    injected = {row["frame"] for row in temporal_rows if row["detection_dropped"]}
    report["injected_frames_comparison"] = {
        variant: {
            key: distribution(
                [
                    row[key]
                    for row in temporal_rows
                    if row["variant"] == variant and row["frame"] in injected
                ]
            )
            for key in ("objects", "unknowns", "recovered_objects")
        }
        for variant in ("single", "normal", "dropout")
    }
    report["injected_frames_comparison"]["injected_frame_count"] = len(injected)
    report["injected_frames_comparison"]["frames_with_point_supported_recovery"] = sum(
        row["recovered_objects"] > 0
        for row in temporal_rows
        if row["variant"] == "dropout" and row["frame"] in injected
    )
    stamps = [s["measurement_time"] for s in manifest["samples"]]
    report["dataset"] = {
        "frames": len(manifest["samples"]),
        "record_clouds": manifest["total_clouds"],
        "total_points": sum(s["points"] for s in manifest["samples"]),
        "payload_span_seconds": stamps[-1] - stamps[0],
        "frame_dt_seconds": distribution([b - a for a, b in zip(stamps, stamps[1:])]),
        "temporal_frames": len(temporal_manifest["samples"]),
        "temporal_payload_span_seconds": (
            temporal_manifest["samples"][-1]["measurement_time"]
            - temporal_manifest["samples"][0]["measurement_time"]
        ),
        "actual_tf_parent": manifest["calibration_parent"],
    }
    report["metric_scope"] = (
        "Instance counts are object-frame outputs, not unique ground-truth objects. "
        "Continuation/confirmation are track-frame outputs, not IDF1/MOTA/ID switches. "
        "Timing is sequential offline diagnostic execution, not deployed throughput."
    )
    report["timing_conditions"] = {
        "runs": 1,
        "warmup": "First infer included; engine init excluded; no samples discarded.",
        "det_ms": "Runtime infer only; excludes BuildPointPillarsInput XYZI packing.",
        "seg_ms": "RangeRet Segment: projection, inference and output construction.",
        "fusion_ms": "SingleFrameFusion::Process; excludes serialization and file IO.",
        "temporal_fusion_ms": "StatefulFusion::Process with candidates/history.",
        "execution": "Offline sequential; all four modes run in fixed enum order.",
        "build": "Managed-container Bazel dev/fastbuild; GPU flag for neural replay.",
        "isolation": "Shared host; no affinity/power control or exclusive GPU claim.",
        "not_measured": "Cyber queue, model concurrency, throughput or E2E latency.",
    }
    report["not_evaluable"] = [
        "AP/precision/recall, unknown-obstacle accuracy, duplicate false positives: "
        "no labels",
        "MOTA/IDF1/ID switches and physical velocity error: no identity/velocity truth",
        "FREE precision/traversability: ray origins, ground/vehicle contracts "
        "unverified",
        "Production ODOM continuity: relative localization is explicitly "
        "diagnostic-only",
        "False braking, closed-loop safety and deployment real-time acceptance",
    ]
    for directory in (
        args.input,
        args.results,
        args.temporal_input,
        args.temporal_results,
    ):
        for path in directory.iterdir():
            if path.name in (
                "manifest.json",
                "fusion.pb.txt",
                "config.pb",
                "summary.csv",
                "temporal.csv",
                "summary.txt",
                "arguments.txt",
                "pose-manifest.json",
            ):
                identities[str(path)] = digest(path)
    arguments = (args.results / "arguments.txt").read_text().splitlines()
    for path in (
        Path(arguments[i]) for i in ((0, 3, 4) if len(arguments) == 7 else (0, 4))
    ):
        identities[str(path)] = digest(path)
    report["timing_conditions"]["overlapping_workers"] = (
        "Full-record neural/four-mode replay and continuous CPU replay overlap "
        "on the shared host; no controlled performance comparison is claimed."
    )
    report["artifact_sha256"] = identities
    report["module_source_sha256"] = {
        str(p): digest(p)
        for root in (Path("modules/lidar_fusion"), Path(__file__).parent)
        for p in root.rglob("*")
        if p.is_file()
        and (p.suffix in (".cc", ".h", ".proto", ".py") or p.name == "BUILD")
    }
    (args.output / "baseline.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"dataset": report["dataset"], "status": "audited"}, indent=2))


if __name__ == "__main__":
    main()
