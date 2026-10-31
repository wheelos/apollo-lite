#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Recompute frozen terrain inputs with diagnostic traversability limits."""

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np
import terrain_surface_experiment as surface_experiment
from evaluate_baseline import audit_frame, digest, require
from proto import Reader, require_full_evidence
from semantickitti import GROUND, learning_labels
from terrain_surface import TerrainSurfaceConfig, estimate_terrain_surface
from terrain_traversability import (
    MEASURED,
    OBJECT,
    UNKNOWN_RISK,
    TraversabilityLimits,
    classify_traversability,
)

ROOT = Path(__file__).resolve().parents[3]
V4_ROOT = Path(".cache/lidar-fusion-validation/terrain-v4")
V5_ROOT = Path(".cache/lidar-fusion-validation/terrain-v5")
V6_ROOT = Path(".cache/lidar-fusion-validation/terrain-v6")
INVENTORY_RELATIVE = Path(
    ".cache/lidar-fusion-validation/three-role-validation-20261031/"
    "raw-non08-gt-inventory.json"
)

CLASSIFIER_FIELDS = (
    "terrain_state",
    "decision_state",
    "reason_bits",
    "terrain_candidate",
    "risk_constrained_candidate",
    "terrain_connected",
    "risk_constrained_connected",
    "risk_layers",
    "risk_observed",
)


def _write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def _jsonable(value):
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def _load_limits(path):
    payload = json.loads(Path(path).read_text())
    return payload, _limits_from_payload(payload)


def _limits_from_payload(payload):
    required = {
        "diagnostic_only",
        "max_slope_gradient",
        "max_roughness_m",
        "max_step_m",
        "max_depression_depth_m",
    }
    require(isinstance(payload, dict), "limits JSON must be an object")
    require(
        set(payload) == required,
        "limits JSON must contain diagnostic_only and exactly four terrain limits",
    )
    require(
        payload["diagnostic_only"] is True,
        "limits JSON must explicitly set diagnostic_only to true",
    )
    limits = TraversabilityLimits(
        max_slope_gradient=payload["max_slope_gradient"],
        max_roughness_m=payload["max_roughness_m"],
        max_step_m=payload["max_step_m"],
        max_depression_depth_m=payload["max_depression_depth_m"],
    )
    limits.validate()
    return limits


def _hash_tree(root):
    root = Path(root)
    return {
        str(path.relative_to(root)): digest(path)
        for path in sorted(root.rglob("*"))
        if path.is_file()
    }


def _prior_snapshot(root):
    snapshot = {}
    for relative in (V4_ROOT, V5_ROOT):
        base = root / relative
        require(base.is_dir(), f"missing frozen prior evidence: {base}")
        for name, sha256 in _hash_tree(base).items():
            path = relative / name
            snapshot[str(path)] = sha256
    return snapshot


def _verify_prior_snapshot(root, snapshot):
    changed = []
    for relative_path, expected in snapshot.items():
        path = root / relative_path
        if not path.is_file() or digest(path) != expected:
            changed.append(relative_path)
    require(not changed, f"v4/v5 artifacts changed during run: {changed[:5]}")


def _source_code_hashes():
    paths = (
        Path(__file__),
        Path(__file__).with_name("terrain_surface.py"),
        Path(__file__).with_name("terrain_surface_experiment.py"),
        Path(__file__).with_name("terrain_traversability.py"),
        Path(__file__).with_name("proto.py"),
        Path(__file__).with_name("evaluate_baseline.py"),
        Path(__file__).with_name("semantickitti.py"),
    )
    return {str(path): digest(path) for path in paths}


def _validate_v5_source_audit(v5_report):
    require(
        v5_report.get("status") == "PASS_RAW56_GT44_IDENTITY_AND_POSTFIT_PROXY_ONLY",
        "v5 identity audit is not a passing exact-source audit",
    )
    entries = v5_report.get("source_identity", [])
    require(len(entries) == 56, f"expected 56 v5 source audits, got {len(entries)}")
    by_path = {}
    for entry in entries:
        require(
            entry.get("status") == "PASS_EXACT_RAW_SHA_XYZ_POINT_ORDER_FRAME",
            "v5 source audit contains a non-passing frame",
        )
        path = str(Path(entry["source_path"]))
        require(path not in by_path, f"duplicate v5 source audit path: {path}")
        by_path[path] = entry
    return by_path


def _validate_source_binary(path, expected_sha, v5_audit, *, allow_archived_raw):
    path = Path(path)
    audited = v5_audit.get(str(path))
    require(audited is not None, f"source missing from v5 identity audit: {path}")
    require(
        audited["expected_raw_sha256"] == expected_sha
        and audited["current_raw_sha256"] == expected_sha,
        f"v5 and frozen manifest disagree on raw source hash: {path}",
    )
    if path.is_file():
        actual = digest(path)
        require(actual == expected_sha, f"raw source binary changed: {path}")
        return {
            "path": str(path),
            "expected_sha256": expected_sha,
            "actual_sha256": actual,
            "current_binary_verified": True,
            "prior_v5_identity_verified": True,
        }
    require(
        allow_archived_raw,
        f"raw source unavailable: {path}; use --allow-archived-raw only if needed",
    )
    return {
        "path": str(path),
        "expected_sha256": expected_sha,
        "actual_sha256": None,
        "current_binary_verified": False,
        "prior_v5_identity_verified": True,
    }


def _verify_v4_frame(root, row, points):
    artifact = root / row["artifact_path"]
    expected_hash = row["artifact_sha256"]
    require(digest(artifact) == expected_hash, f"frozen v4 NPZ changed: {artifact}")
    root_id = int(row["root_anchor"]["point_id"])
    with np.load(artifact, allow_pickle=False) as archive:
        ids = np.asarray(archive["point_id"])
        xyz = np.asarray(archive["xyz_m"], dtype=np.float64)
        stored_root = int(np.asarray(archive["root_anchor_requested_id"]).item())
    require(
        ids.shape == (len(points),)
        and np.array_equal(ids, np.arange(len(points), dtype=ids.dtype)),
        f"v4 point IDs are not original-order IDs: {artifact}",
    )
    require(
        np.array_equal(xyz, points, equal_nan=True),
        f"v4 frozen XYZ does not match the current original source: {artifact}",
    )
    require(stored_root == root_id, f"v4 root ID mismatch: {artifact}")
    require(0 <= root_id < len(points), f"invalid fixed v4 root ID: {artifact}")
    require(
        np.array_equal(
            np.asarray(row["root_anchor"]["xyz_m"], dtype=np.float64),
            points[root_id],
            equal_nan=True,
        ),
        f"v4 fixed root XYZ mismatch: {artifact}",
    )
    config_payload = row.get("terrain_config")
    require(isinstance(config_payload, dict), f"missing v4 terrain config: {artifact}")
    config = TerrainSurfaceConfig(**config_payload)
    return root_id, config


def _native_point_inputs(out, point_count):
    point_layers = np.zeros(point_count, dtype=np.uint8)
    for entity, bit in ((out.object, OBJECT), (out.unknown, UNKNOWN_RISK)):
        for item in entity:
            for point_id in item.point_id:
                point_id = int(point_id)
                require(
                    0 <= point_id < point_count, "native risk point ID out of range"
                )
                point_layers[point_id] |= bit

    decisions = list(out.point_decision)
    require(len(decisions) == point_count, "native point decision count mismatch")
    explanation = np.zeros(point_count, dtype=np.int16)
    reason_bits = np.zeros(point_count, dtype=np.uint64)
    reason_offsets = [0]
    reason_values = []
    decision_source = np.full(point_count, -1, dtype=np.int16)
    decision_object_id = np.full(point_count, -1, dtype=np.int64)
    competition = np.zeros(point_count, dtype=bool)
    for index, decision in enumerate(decisions):
        require(
            int(decision.point_id) == index, "native decisions are not point aligned"
        )
        explanation[index] = int(decision.explanation)
        values = [int(reason) for reason in decision.reason]
        reason_values.extend(values)
        reason_offsets.append(len(reason_values))
        bits = np.uint64(0)
        for value in values:
            require(0 <= value < 64, "native reason enum does not fit uint64 bitfield")
            bits |= np.uint64(1) << np.uint64(value)
        reason_bits[index] = bits
        if decision.HasField("source"):
            decision_source[index] = int(decision.source)
        if decision.HasField("object_id"):
            decision_object_id[index] = int(decision.object_id)
        competition[index] = bool(decision.competition)
    return point_layers, {
        "native_point_decision_explanation": explanation,
        "native_point_decision_reason_bits": reason_bits,
        "native_point_decision_reason_offsets": np.asarray(
            reason_offsets, dtype=np.int64
        ),
        "native_point_decision_reason_values": np.asarray(
            reason_values, dtype=np.int16
        ),
        "native_point_decision_source": decision_source,
        "native_point_decision_object_id": decision_object_id,
        "native_point_decision_competition": competition,
    }


def _overlap_axis_indices(low, high, origin, cell_size, count):
    first = max(0, math.floor((low - origin) / cell_size))
    past_last = min(count, math.ceil((high - origin) / cell_size))
    return range(first, max(first, past_last))


def _positive_overlap(a_low, a_high, b_low, b_high):
    overlap = min(a_high, b_high) - max(a_low, b_low)
    tolerance = 1e-12 * max(1.0, abs(a_low), abs(a_high), abs(b_low), abs(b_high))
    return overlap if overlap > tolerance else 0.0


def rasterize_native_risk(
    space_map,
    terrain_config,
    *,
    point_xyz=None,
    point_risk_layers=None,
    audited=False,
):
    """Project native occupied rectangles and explicit-cell coverage.

    ``audited`` must be true only after ``require_full_evidence`` and the
    existing full FusionOutput audit pass. Sparse omitted native cells do not
    provide coverage; serialized empty cells do. Coverage is not ray visibility
    or evidence of safety.
    """
    require(audited is True, "native risk rasterization requires audited output")
    shape = terrain_config.shape
    required = ("resolution", "origin_x", "origin_y", "width", "height")
    require(
        all(space_map.HasField(name) for name in required),
        "native SpaceMap is missing grid geometry",
    )
    native_resolution = float(space_map.resolution)
    origin_x = float(space_map.origin_x)
    origin_y = float(space_map.origin_y)
    width = int(space_map.width)
    height = int(space_map.height)
    require(
        all(math.isfinite(value) for value in (native_resolution, origin_x, origin_y))
        and native_resolution > 0
        and width > 0
        and height > 0,
        "invalid native SpaceMap geometry",
    )

    layers = np.zeros(shape, dtype=np.uint8)
    coverage_area = np.zeros(shape, dtype=np.float64)
    cell_seen = set()
    terrain_x_min = float(terrain_config.x_min)
    terrain_y_min = float(terrain_config.y_min)
    terrain_size = float(terrain_config.cell_size)
    native_roi = (
        origin_x,
        origin_y,
        origin_x + width * native_resolution,
        origin_y + height * native_resolution,
    )
    for cell in space_map.cell:
        require(
            cell.HasField("x_index") and cell.HasField("y_index"),
            "native serialized SpaceCell is missing indices",
        )
        nx = int(cell.x_index)
        ny = int(cell.y_index)
        require(
            0 <= nx < width and 0 <= ny < height,
            "native serialized SpaceCell lies outside the declared map",
        )
        require((nx, ny) not in cell_seen, "duplicate native SpaceCell")
        cell_seen.add((nx, ny))
        x_low = origin_x + nx * native_resolution
        x_high = x_low + native_resolution
        y_low = origin_y + ny * native_resolution
        y_high = y_low + native_resolution
        native_bits = (
            (MEASURED if bool(cell.occupancy_obs) else 0)
            | (OBJECT if bool(cell.occupancy_det) else 0)
            | (UNKNOWN_RISK if bool(cell.occupancy_unknown) else 0)
        )
        x_indices = _overlap_axis_indices(
            x_low, x_high, terrain_x_min, terrain_size, shape[0]
        )
        y_indices = _overlap_axis_indices(
            y_low, y_high, terrain_y_min, terrain_size, shape[1]
        )
        for ix in x_indices:
            terrain_x0 = terrain_x_min + ix * terrain_size
            overlap_x = _positive_overlap(
                x_low, x_high, terrain_x0, terrain_x0 + terrain_size
            )
            if overlap_x == 0:
                continue
            for iy in y_indices:
                terrain_y0 = terrain_y_min + iy * terrain_size
                overlap_y = _positive_overlap(
                    y_low, y_high, terrain_y0, terrain_y0 + terrain_size
                )
                if overlap_y == 0:
                    continue
                coverage_area[ix, iy] += overlap_x * overlap_y
                if native_bits:
                    layers[ix, iy] |= np.uint8(native_bits)

    terrain_area = terrain_size * terrain_size
    coverage_tolerance = 1e-10 * max(1.0, terrain_area)
    observed = np.isclose(
        coverage_area, terrain_area, rtol=0.0, atol=coverage_tolerance
    )
    if point_xyz is not None or point_risk_layers is not None:
        require(
            point_xyz is not None and point_risk_layers is not None,
            "point XYZ and point risk layers must be provided together",
        )
        points = np.asarray(point_xyz, dtype=np.float64)
        point_bits = np.asarray(point_risk_layers)
        require(
            points.ndim == 2
            and points.shape[1] == 3
            and point_bits.shape == (len(points),)
            and point_bits.dtype.kind in "iu"
            and np.all(point_bits >= 0)
            and np.all(point_bits <= (MEASURED | OBJECT | UNKNOWN_RISK)),
            "invalid native point risk inputs",
        )
        finite = np.isfinite(points).all(axis=1) & (point_bits != 0)
        ids = np.flatnonzero(finite)
        keys = np.floor(
            (points[ids, :2] - [terrain_x_min, terrain_y_min]) / terrain_size
        ).astype(np.int64)
        valid = (
            (keys[:, 0] >= 0)
            & (keys[:, 0] < shape[0])
            & (keys[:, 1] >= 0)
            & (keys[:, 1] < shape[1])
        )
        for point_id, (ix, iy) in zip(ids[valid], keys[valid]):
            layers[ix, iy] |= np.uint8(point_bits[point_id])

    return (
        layers,
        observed,
        {
            "native_roi_xy": [origin_x, origin_y, native_roi[2], native_roi[3]],
            "native_grid_shape": [width, height],
            "native_serialized_cell_count": len(cell_seen),
            "native_omitted_cell_count": width * height - len(cell_seen),
            "fully_covered_terrain_cell_count": int(np.count_nonzero(observed)),
            "partially_covered_terrain_cell_count": int(
                np.count_nonzero((coverage_area > coverage_tolerance) & ~observed)
            ),
            "uncovered_terrain_cell_count": int(
                np.count_nonzero(coverage_area <= coverage_tolerance)
            ),
            "coverage_contract": (
                "Union of explicit serialized native SpaceCell rectangles must "
                "fully cover the terrain cell; omitted sparse cells are unknown. "
                "This is native cell evidence, not verified ray visibility or safety."
            ),
        },
    )


def _array_equal(before, after):
    if before.shape != after.shape or before.dtype != after.dtype:
        return False
    if before.dtype.kind in "fc":
        return bool(np.array_equal(before, after, equal_nan=True))
    return bool(np.array_equal(before, after))


def _store_frame(
    path,
    points,
    surface,
    estimate,
    point_ids,
    root_point_id,
    source_hashes,
    native_arrays,
):
    arrays = {
        name: np.asarray(getattr(estimate, name)).copy() for name in CLASSIFIER_FIELDS
    }
    arrays.update(
        {
            "point_id": np.asarray(point_ids, dtype=np.uint32),
            "xyz_m": np.asarray(points, dtype=np.float64),
            "point_cell_id": np.asarray(surface.point_cell_id, dtype=np.int64),
            "point_layer_id": np.asarray(surface.point_layer_id, dtype=np.int32),
            "surface_point_risk_mask": np.asarray(surface.point_risk_mask, dtype=bool),
            "surface_cell_status": np.asarray(surface.cell_status),
            "surface_cell_elevation_z_m": np.asarray(
                surface.cell_elevation_z, dtype=np.float64
            ),
            "surface_cell_slope_gradient": np.asarray(
                surface.cell_slope_gradient, dtype=np.float64
            ),
            "surface_cell_roughness_m": np.asarray(
                surface.cell_roughness_m, dtype=np.float64
            ),
            "surface_cell_step_m": np.asarray(surface.cell_step_m, dtype=np.float64),
            "surface_cell_step_status": np.asarray(surface.cell_step_status),
            "surface_pit_status": np.asarray(surface.pit_status),
            "surface_pit_depth_m": np.asarray(surface.pit_depth_m, dtype=np.float64),
            "root_cell": np.asarray(
                [-1, -1] if estimate.root_cell is None else estimate.root_cell,
                dtype=np.int32,
            ),
            "root_anchor_point_id": np.asarray([root_point_id], dtype=np.int64),
            "overall_status": np.asarray(estimate.overall_status),
            "source_hashes_json": np.asarray(
                json.dumps(source_hashes, sort_keys=True, allow_nan=False)
            ),
        }
    )
    arrays.update(native_arrays)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)
    roundtrip = {}
    with np.load(path, allow_pickle=False) as persisted:
        require(set(persisted.files) == set(arrays), "NPZ field set changed")
        for name, expected in arrays.items():
            require(
                expected.dtype.kind != "O",
                f"object array is forbidden in persisted output: {name}",
            )
            got = np.asarray(persisted[name])
            equal = _array_equal(expected, got)
            require(equal, f"NPZ round-trip mismatch for {path.name}:{name}")
            roundtrip[name] = equal
    return arrays, digest(path), roundtrip


def _risk_rejection_metrics(arrays, cell_area):
    candidate = np.asarray(arrays["terrain_candidate"], dtype=bool)
    constrained = np.asarray(arrays["risk_constrained_candidate"], dtype=bool)
    bits = np.asarray(arrays["risk_layers"], dtype=np.uint8)
    observed = np.asarray(arrays["risk_observed"], dtype=bool)
    reasons = np.asarray(arrays["reason_bits"], dtype=np.uint32)
    risk_rejected = candidate & ~constrained
    source_masks = {
        "measured": (bits & MEASURED) != 0,
        "object": (bits & OBJECT) != 0,
        "unknown": (bits & UNKNOWN_RISK) != 0,
        "risk_unobserved": ~observed,
    }
    source_counts = {
        name: int(np.count_nonzero(candidate & mask))
        for name, mask in source_masks.items()
    }
    membership = Counter()
    sole = Counter()
    overlap = Counter()
    for index in zip(*np.nonzero(risk_rejected)):
        active = tuple(name for name, mask in source_masks.items() if bool(mask[index]))
        label = "+".join(active) if active else "unattributed"
        membership[label] += 1
        if len(active) == 1:
            sole[label] += 1
        elif len(active) > 1:
            overlap[label] += 1
    risk_reasons = {
        "measured": 1 << 10,
        "object": 1 << 11,
        "unknown": 1 << 12,
        "risk_unobserved": 1 << 13,
    }
    return {
        "source_membership_cells_within_terrain_candidate": source_counts,
        "risk_rejected_candidate_cells": int(np.count_nonzero(risk_rejected)),
        "risk_rejected_candidate_area_m2": float(
            np.count_nonzero(risk_rejected) * cell_area
        ),
        "sole_risk_rejection_counts": dict(sorted(sole.items())),
        "overlapping_risk_rejection_counts": dict(sorted(overlap.items())),
        "risk_reason_bit_counts_within_terrain_candidate": {
            name: int(np.count_nonzero(candidate & ((reasons & bit) != 0)))
            for name, bit in risk_reasons.items()
        },
        "risk_membership_combinations": dict(sorted(membership.items())),
    }


def _frame_metrics(arrays, cell_size, overall_status):
    area = cell_size * cell_size
    counts = {
        "terrain_candidate": int(np.count_nonzero(arrays["terrain_candidate"])),
        "risk_constrained_candidate": int(
            np.count_nonzero(arrays["risk_constrained_candidate"])
        ),
        "terrain_connected": int(np.count_nonzero(arrays["terrain_connected"])),
        "risk_constrained_connected": int(
            np.count_nonzero(arrays["risk_constrained_connected"])
        ),
    }
    return {
        "overall_status": overall_status,
        "cell_area_m2": area,
        "terrain_candidate_cells": counts["terrain_candidate"],
        "terrain_candidate_area_m2": counts["terrain_candidate"] * area,
        "risk_constrained_candidate_cells": counts["risk_constrained_candidate"],
        "risk_constrained_candidate_area_m2": (
            counts["risk_constrained_candidate"] * area
        ),
        "terrain_connected_cells": counts["terrain_connected"],
        "terrain_connected_area_m2": counts["terrain_connected"] * area,
        "risk_constrained_connected_cells": counts["risk_constrained_connected"],
        "risk_constrained_connected_area_m2": (
            counts["risk_constrained_connected"] * area
        ),
        "terrain_rejection_reason_bits": {
            str(bit): int(np.count_nonzero((arrays["reason_bits"] & bit) != 0))
            for bit in (1 << index for index in range(15))
        },
        "risk_rejection": _risk_rejection_metrics(arrays, area),
        "risk_observed_cells": int(np.count_nonzero(arrays["risk_observed"])),
        "risk_layer_cells": {
            "measured": int(np.count_nonzero(arrays["risk_layers"] & MEASURED)),
            "object": int(np.count_nonzero(arrays["risk_layers"] & OBJECT)),
            "unknown": int(np.count_nonzero(arrays["risk_layers"] & UNKNOWN_RISK)),
        },
    }


def _summarize_frames(frames):
    fields = (
        "terrain_candidate",
        "risk_constrained_candidate",
        "terrain_connected",
        "risk_constrained_connected",
    )
    summary = {"frame_count": len(frames)}
    for field in fields:
        cell_key = f"{field}_cells"
        area_key = f"{field}_area_m2"
        summary[cell_key] = sum(frame["metrics"][cell_key] for frame in frames)
        summary[area_key] = sum(frame["metrics"][area_key] for frame in frames)
    summary["risk_observed_cells"] = sum(
        frame["metrics"]["risk_observed_cells"] for frame in frames
    )
    summary["overall_status_counts"] = dict(
        sorted(Counter(frame["metrics"]["overall_status"] for frame in frames).items())
    )
    source_membership = Counter()
    reason_bits = Counter()
    combinations = Counter()
    summary["risk_rejection"] = {
        "candidate_cells": sum(
            frame["metrics"]["risk_rejection"]["risk_rejected_candidate_cells"]
            for frame in frames
        ),
        "candidate_area_m2": sum(
            frame["metrics"]["risk_rejection"]["risk_rejected_candidate_area_m2"]
            for frame in frames
        ),
        "sole_counts": dict(
            sum(
                (
                    Counter(
                        frame["metrics"]["risk_rejection"]["sole_risk_rejection_counts"]
                    )
                    for frame in frames
                ),
                Counter(),
            )
        ),
        "overlap_counts": dict(
            sum(
                (
                    Counter(
                        frame["metrics"]["risk_rejection"][
                            "overlapping_risk_rejection_counts"
                        ]
                    )
                    for frame in frames
                ),
                Counter(),
            )
        ),
        "source_membership_cells_within_terrain_candidate": dict(
            sum(
                (
                    Counter(
                        frame["metrics"]["risk_rejection"][
                            "source_membership_cells_within_terrain_candidate"
                        ]
                    )
                    for frame in frames
                ),
                source_membership,
            )
        ),
        "reason_bit_counts_within_terrain_candidate": dict(
            sum(
                (
                    Counter(
                        frame["metrics"]["risk_rejection"][
                            "risk_reason_bit_counts_within_terrain_candidate"
                        ]
                    )
                    for frame in frames
                ),
                reason_bits,
            )
        ),
        "membership_combinations": dict(
            sum(
                (
                    Counter(
                        frame["metrics"]["risk_rejection"][
                            "risk_membership_combinations"
                        ]
                    )
                    for frame in frames
                ),
                combinations,
            )
        ),
    }
    return summary


def _score_semantic_proxy(root, frames):
    scores = []
    for frame in frames:
        label_path = frame.get("labels_path")
        expected_hash = frame.get("labels_sha256")
        if not label_path or not expected_hash:
            frame["semantic_ground_proxy"] = {
                "status": "NOT_APPLICABLE_NO_SEMANTIC_GROUND_TRUTH"
            }
            continue
        label_path = Path(label_path)
        require(
            label_path.is_file(),
            f"semantic GT unavailable after inference: {label_path}",
        )
        actual_hash = digest(label_path)
        require(actual_hash == expected_hash, f"semantic GT hash changed: {label_path}")
        with np.load(root / frame["artifact_path"], allow_pickle=False) as archive:
            points = np.asarray(archive["xyz_m"], dtype=np.float64)
            point_cell_id = np.asarray(archive["point_cell_id"], dtype=np.int64)
            terrain = np.asarray(archive["terrain_candidate"], dtype=bool)
            constrained = np.asarray(archive["risk_constrained_candidate"], dtype=bool)
            risk_layers = np.asarray(archive["risk_layers"], dtype=np.uint8)
        raw_labels = np.fromfile(label_path, dtype="<u4")
        require(
            len(raw_labels) == len(points),
            f"semantic GT point count mismatch: {label_path}",
        )
        labels = learning_labels(raw_labels)
        in_roi = (
            np.isfinite(points).all(axis=1)
            & (point_cell_id >= 0)
            & (point_cell_id < terrain.size)
        )
        gt_ground = np.isin(labels, GROUND) & in_roi
        valid_gt = (labels != 0) & in_roi
        point_terrain = np.zeros(len(points), dtype=bool)
        point_constrained = np.zeros(len(points), dtype=bool)
        point_risk = np.zeros(len(points), dtype=np.uint8)
        ids = np.flatnonzero(in_roi)
        flat_terrain = terrain.reshape(-1)
        flat_constrained = constrained.reshape(-1)
        flat_risk = risk_layers.reshape(-1)
        point_terrain[ids] = flat_terrain[point_cell_id[ids]]
        point_constrained[ids] = flat_constrained[point_cell_id[ids]]
        point_risk[ids] = flat_risk[point_cell_id[ids]]

        def metric(prediction):
            prediction = prediction & valid_gt
            tp = int(np.count_nonzero(gt_ground & prediction))
            fp = int(np.count_nonzero(prediction & ~gt_ground))
            fn = int(np.count_nonzero(gt_ground & ~prediction))
            return {
                "tp": tp,
                "fp_valid_semantic": fp,
                "fn": fn,
                "precision": tp / (tp + fp) if tp + fp else None,
                "recall": tp / (tp + fn) if tp + fn else None,
                "valid_gt_points_in_roi": int(np.count_nonzero(valid_gt)),
                "ignored_gt_points_in_roi": int(
                    np.count_nonzero((labels == 0) & in_roi)
                ),
            }

        ground_risk_overlap = gt_ground & point_terrain & (point_risk != 0)
        frame_score = {
            "status": "SEMANTIC_GROUND_PROXY_ONLY_NOT_PHYSICAL_TERRAIN_TRUTH",
            "terrain_candidate_point_metrics": metric(point_terrain),
            "risk_constrained_candidate_point_metrics": metric(point_constrained),
            "terrain_candidate_semantic_ground_risk_overlap_points": int(
                np.count_nonzero(ground_risk_overlap)
            ),
            "risk_overlap_interpretation": (
                "Diagnostic overlap only; semantic-ground agreement does not clear "
                "native risk or establish a physical false positive."
            ),
            "truth_read_after_all_classifier_outputs": True,
            "label_sha256": actual_hash,
        }
        frame["semantic_ground_proxy"] = frame_score
        scores.append(frame_score)
    return {
        "evaluated_semantickitti_frames": len(scores),
        "physical_pit_or_bridge_ground_truth": "BLOCKED_NOT_AVAILABLE",
        "nuScenes_ground_truth": "NOT_USED_NOT_AVAILABLE",
        "aggregate": {
            key: _aggregate_semantic_metric(scores, key)
            for key in (
                "terrain_candidate_point_metrics",
                "risk_constrained_candidate_point_metrics",
            )
        },
        "risk_overlap_interpretation": (
            "Diagnostic semantic overlap is not a false-positive finding and does "
            "not authorize clearing native risk."
        ),
    }


def _aggregate_semantic_metric(scores, key):
    rows = [item[key] for item in scores if key in item]
    tp = sum(row["tp"] for row in rows)
    fp = sum(row["fp_valid_semantic"] for row in rows)
    fn = sum(row["fn"] for row in rows)
    return {
        "tp": tp,
        "fp_valid_semantic": fp,
        "fn": fn,
        "precision": tp / (tp + fp) if tp + fp else None,
        "recall": tp / (tp + fn) if tp + fn else None,
        "frames": len(rows),
    }


def _load_non08_inputs(root, sample, history, v4_row, allow_archived_raw):
    raw_path = Path(sample["raw_scan"])
    raw_sha = sample["raw_sha256"]
    if raw_path.is_file():
        actual = digest(raw_path)
        require(actual == raw_sha, f"non-08 raw source hash mismatch: {raw_path}")
        raw = np.fromfile(raw_path, dtype="<f4")
        require(raw.size % 4 == 0, f"invalid raw scan dimensions: {raw_path}")
        points = raw.reshape(-1, 4)[:, :3].astype(np.float64, copy=True)
        require(
            len(points) == int(sample["point_count"]),
            f"non-08 point count mismatch: {raw_path}",
        )
        return points, {
            "kind": "verified_raw_source",
            "raw_source_path": str(raw_path),
            "raw_source_sha256": raw_sha,
            "raw_source_hash_verified": True,
            "coordinate_contract": (
                "first three original float32 scan columns; native sensor frame"
            ),
        }

    require(
        allow_archived_raw,
        f"raw source unavailable: {raw_path}; archived input requires explicit opt-in",
    )
    archive_path = (
        root
        / ".cache/lidar-fusion-validation/continuation-v3"
        / f"patch-raw-{sample['sequence']}-{int(sample['ordinal']):06d}.npz"
    )
    require(
        v4_row["archive_input_provenance"]["archive_path"]
        == str(archive_path.relative_to(root)),
        f"unexpected archived XYZ path for {raw_path}",
    )
    require(
        digest(archive_path) == v4_row["archive_input_provenance"]["archive_sha256"],
        f"archived XYZ hash mismatch: {archive_path}",
    )
    with np.load(archive_path, allow_pickle=False) as archive:
        point_ids = np.asarray(archive["point_id"])
        points = np.asarray(archive["xyz_m"], dtype=np.float64)
    require(
        point_ids.shape == (len(points),)
        and np.array_equal(point_ids, np.arange(len(points), dtype=point_ids.dtype)),
        f"archived point IDs are not original order: {archive_path}",
    )
    require(len(points) == int(sample["point_count"]), "archived point count mismatch")
    return points, {
        "kind": "archived_original_points_fallback",
        "archive_path": str(archive_path.relative_to(root)),
        "archive_sha256": digest(archive_path),
        "raw_source_path": str(raw_path),
        "raw_source_expected_sha256": raw_sha,
        "raw_source_hash_verified": False,
        "historical_v4_identity_is_not_binary_verification": True,
    }


def run_experiment(
    *,
    root=ROOT,
    limits_path,
    output=None,
    allow_archived_raw=False,
):
    root = Path(root).resolve()
    output = Path(output).resolve() if output is not None else root / V6_ROOT
    output.mkdir(parents=True, exist_ok=True)
    limits_path = Path(limits_path).resolve()
    limits_payload, limits = _load_limits(limits_path)
    limits_sha = digest(limits_path)
    prior_snapshot = _prior_snapshot(root)
    prior_snapshot_sha = hashlib.sha256(
        json.dumps(prior_snapshot, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()

    v4_report_path = root / V4_ROOT / "real-data-report.json"
    v5_report_path = root / V5_ROOT / "data-report.json"
    v4_report_sha = digest(v4_report_path)
    v5_report_sha = digest(v5_report_path)
    v4_report = json.loads(v4_report_path.read_text())
    v5_report = json.loads(v5_report_path.read_text())
    v5_sources = _validate_v5_source_audit(v5_report)
    require(
        len(v4_report["cached_frames"]) == 36
        and len(v4_report["raw_non08_frames"]) == 20,
        "frozen v4 report does not contain the expected 36+20 frames",
    )
    cached_v4 = {(row["case"], row["file"]): row for row in v4_report["cached_frames"]}
    raw_v4 = {
        (row["sequence"], int(row["ordinal"])): row
        for row in v4_report["raw_non08_frames"]
    }
    require(len(cached_v4) == 36 and len(raw_v4) == 20, "duplicate v4 frame identity")

    schema_path = root / ".cache/lidar-fusion-validation/next-iteration/schema.pb"
    schema_sha = digest(schema_path)
    reader = Reader(schema_path)
    history_path = root / ".cache/lidar-fusion-validation/continuation-v3/report.json"
    history = json.loads(history_path.read_text())
    inventory_path = root / INVENTORY_RELATIVE
    inventory = json.loads(inventory_path.read_text())
    require(
        inventory.get("frame_count") == 20
        and len(inventory["selection_details"]) == 20,
        "non-08 raw inventory is not the frozen 20-frame selection",
    )

    code_hashes = _source_code_hashes()
    source_hashes = {
        str(v4_report_path): v4_report_sha,
        str(v5_report_path): v5_report_sha,
        str(schema_path): schema_sha,
        str(history_path): digest(history_path),
        str(inventory_path): digest(inventory_path),
        str(limits_path): limits_sha,
        **code_hashes,
    }
    native_audits = []
    source_identity_records = []
    frame_rows = []
    cached_input_count = 0
    for run, sample, manifest in surface_experiment._iter_cached_sources(
        root, reader, source_hashes
    ):
        frozen = cached_v4[(run["name"], sample["file"])]
        raw_audit = _validate_source_binary(
            sample["source"],
            sample["source_sha256"],
            v5_sources,
            allow_archived_raw=allow_archived_raw,
        )
        native_path = run["results"] / (sample["file"] + ".fusion.pb")
        native_before = digest(native_path)
        output_before = None
        if Path(sample["source"]).is_file():
            out, source_config, points, _ = surface_experiment.source_frame(
                reader, run, sample, source_hashes
            )
            provenance = {
                "kind": "verified_raw_source",
                "raw_source_path": sample["source"],
                "raw_source_sha256": sample["source_sha256"],
                "raw_source_hash_verified": True,
            }
        else:
            require(
                allow_archived_raw,
                "archived cached input requires explicit opt-in",
            )
            out, source_config, points, provenance = (
                surface_experiment._cached_archive_frame(
                    root,
                    reader,
                    run,
                    sample,
                    source_hashes,
                    history,
                )
            )
        output_before = out.SerializeToString(deterministic=True)
        require_full_evidence(out)
        audit_frame(out, source_config)
        root_id, surface_config = _verify_v4_frame(root, frozen, points)
        point_ids = np.arange(len(points), dtype=np.uint32)
        point_risk_mask, _ = surface_experiment._risk_inputs(out, len(points))
        point_risk_layers, native_point_arrays = _native_point_inputs(out, len(points))
        require(
            np.array_equal(point_risk_mask, point_risk_layers != 0),
            "native object/unknown point sources disagree with original risk mask",
        )
        risk_layers, risk_observed, raster_info = rasterize_native_risk(
            out.space_map,
            surface_config,
            point_xyz=points,
            point_risk_layers=point_risk_layers,
            audited=True,
        )
        surface = estimate_terrain_surface(
            points,
            point_ids=point_ids,
            risk_mask=point_risk_mask,
            root_anchor_id=root_id,
            config=surface_config,
        )
        require(
            np.array_equal(surface.point_ids, point_ids)
            and np.array_equal(surface.point_risk_mask, point_risk_mask),
            "terrain surface changed source IDs or original point risk",
        )
        result = classify_traversability(
            surface,
            limits,
            risk_layers=risk_layers,
            risk_observed=risk_observed,
        )
        require(
            result.root_cell == surface.root_cell,
            "traversability classifier changed the exact fixed root cell",
        )
        native_after = digest(native_path)
        require(native_before == native_after, "cached native output file changed")
        require(
            output_before == out.SerializeToString(deterministic=True),
            "native FusionOutput mutated during classification",
        )
        source_hashes[str(native_path)] = native_before
        config_path = run["results"] / "config.pb"
        source_hashes[str(config_path)] = digest(config_path)
        manifest_path = run["inputs"] / "manifest.json"
        source_hashes[str(manifest_path)] = digest(manifest_path)
        frame_id = f"{run['name']}/{Path(sample['file']).stem}"
        out_path = (
            output
            / "frames"
            / run["name"]
            / (Path(sample["file"]).stem + ".traversability.npz")
        )
        frame_sources = {
            "raw_source_path": sample["source"],
            "raw_source_sha256": sample["source_sha256"],
            "current_raw_binary_sha256": raw_audit["actual_sha256"] or "",
            "cloud_input_sha256": sample["sha256"],
            "native_output_sha256": native_before,
            "native_config_sha256": digest(config_path),
            "manifest_sha256": digest(manifest_path),
            "v4_surface_artifact_sha256": frozen["artifact_sha256"],
            "schema_sha256": schema_sha,
            "terrain_surface_sha256": code_hashes[
                str(Path(__file__).with_name("terrain_surface.py"))
            ],
            "traversability_sha256": code_hashes[str(Path(__file__))],
        }
        npz_arrays, artifact_sha, roundtrip = _store_frame(
            out_path,
            points,
            surface,
            result,
            point_ids,
            root_id,
            frame_sources,
            native_point_arrays,
        )
        cell_size = float(surface_config.cell_size)
        metrics = _frame_metrics(npz_arrays, cell_size, result.overall_status)
        semantic_labels_path = frozen.get("labels_path")
        frame_row = {
            "frame_id": frame_id,
            "dataset_group": (
                "cached_nuscenes12"
                if "nuscenes" in run["name"]
                else "cached_semantickitti_seq08"
            ),
            "dataset": "nuScenes" if "nuscenes" in run["name"] else "SemanticKITTI",
            "file": sample["file"],
            "point_count": len(points),
            "source_path": sample["source"],
            "source_sha256": sample["source_sha256"],
            "source_binary_identity": raw_audit,
            "input_provenance": provenance,
            "raw_source_labels_loaded_for_inference": False,
            "fixed_root_point_id": root_id,
            "root_cell": (None if result.root_cell is None else list(result.root_cell)),
            "overall_status": result.overall_status,
            "terrain_config": surface_config.__dict__,
            "risk_input": {
                "native_fusion_output_sha256": native_before,
                "full_evidence_audit_passed": True,
                **raster_info,
                "native_grid_cells_are_not_ray_visibility": True,
            },
            "point_decision_reason_bitfield_persisted_separately": True,
            "artifact_path": str(out_path.relative_to(output)),
            "artifact_sha256": artifact_sha,
            "roundtrip_arrays_equal": all(roundtrip.values()),
            "roundtrip_array_count": len(roundtrip),
            "labels_path": semantic_labels_path,
            "labels_sha256": frozen.get("labels_sha256"),
            "metrics": metrics,
        }
        frame_rows.append(frame_row)
        source_identity_records.append(raw_audit)
        native_audits.append(
            {
                "path": str(native_path),
                "sha256_before": native_before,
                "sha256_after": native_after,
                "full_evidence_and_frame_audit_passed": True,
            }
        )
        cached_input_count += 1

    require(
        cached_input_count == 36, f"expected 36 cached frames, got {cached_input_count}"
    )

    raw_input_count = 0
    for sample in inventory["selection_details"]:
        sequence = str(sample["sequence"])
        ordinal = int(sample["ordinal"])
        frozen = raw_v4[(sequence, ordinal)]
        source_audit = _validate_source_binary(
            sample["raw_scan"],
            sample["raw_sha256"],
            v5_sources,
            allow_archived_raw=allow_archived_raw,
        )
        points, provenance = _load_non08_inputs(
            root, sample, history, frozen, allow_archived_raw
        )
        root_id, surface_config = _verify_v4_frame(root, frozen, points)
        point_ids = np.arange(len(points), dtype=np.uint32)
        point_risk_mask = np.zeros(len(points), dtype=bool)
        surface = estimate_terrain_surface(
            points,
            point_ids=point_ids,
            risk_mask=point_risk_mask,
            root_anchor_id=root_id,
            config=surface_config,
        )
        require(
            np.array_equal(surface.point_ids, point_ids)
            and not np.any(surface.point_risk_mask),
            "geometry-only surface changed IDs or invented risk",
        )
        # No independent risk input exists for these raw-only frames.
        risk_layers = np.zeros(surface.config.shape, dtype=np.uint8)
        risk_observed = np.zeros(surface.config.shape, dtype=bool)
        result = classify_traversability(
            surface,
            limits,
            risk_layers=risk_layers,
            risk_observed=risk_observed,
        )
        require(
            result.root_cell == surface.root_cell,
            "traversability classifier changed the exact fixed root cell",
        )
        raw_sha = sample["raw_sha256"]
        if source_audit["current_binary_verified"]:
            source_hashes[str(sample["raw_scan"])] = source_audit["actual_sha256"]
        frame_id = f"semantickitti-non08/{sequence}-{ordinal:06d}"
        out_path = (
            output
            / "frames"
            / "semantickitti-non08-raw"
            / (f"{sequence}-{ordinal:06d}.traversability.npz")
        )
        frame_sources = {
            "raw_source_path": sample["raw_scan"],
            "raw_source_sha256": raw_sha,
            "current_raw_binary_sha256": source_audit["actual_sha256"] or "",
            "cloud_input_sha256": "",
            "native_output_sha256": "",
            "native_config_sha256": "",
            "manifest_sha256": digest(inventory_path),
            "v4_surface_artifact_sha256": frozen["artifact_sha256"],
            "schema_sha256": schema_sha,
            "terrain_surface_sha256": code_hashes[
                str(Path(__file__).with_name("terrain_surface.py"))
            ],
            "traversability_sha256": code_hashes[str(Path(__file__))],
        }
        npz_arrays, artifact_sha, roundtrip = _store_frame(
            out_path,
            points,
            surface,
            result,
            point_ids,
            root_id,
            frame_sources,
            {},
        )
        metrics = _frame_metrics(
            npz_arrays,
            float(surface_config.cell_size),
            result.overall_status,
        )
        frame_rows.append(
            {
                "frame_id": frame_id,
                "dataset_group": "raw_semantickitti_non08",
                "dataset": "SemanticKITTI",
                "sequence": sequence,
                "ordinal": ordinal,
                "point_count": len(points),
                "source_path": sample["raw_scan"],
                "source_sha256": raw_sha,
                "source_binary_identity": source_audit,
                "input_provenance": provenance,
                "raw_source_labels_loaded_for_inference": False,
                "fixed_root_point_id": root_id,
                "root_cell": (
                    None if result.root_cell is None else list(result.root_cell)
                ),
                "overall_status": result.overall_status,
                "terrain_config": surface_config.__dict__,
                "risk_input": {
                    "status": "NO_NATIVE_RISK_INPUT_GEOMETRY_ONLY",
                    "risk_observed_cells": 0,
                    "coverage_contract": (
                        "False everywhere; zero risk bits do not qualify a "
                        "risk-constrained candidate."
                    ),
                    "native_grid_cells_are_not_ray_visibility": True,
                },
                "point_decision_reason_bitfield_persisted_separately": False,
                "artifact_path": str(out_path.relative_to(output)),
                "artifact_sha256": artifact_sha,
                "roundtrip_arrays_equal": all(roundtrip.values()),
                "roundtrip_array_count": len(roundtrip),
                "labels_path": sample.get("labels"),
                "labels_sha256": sample.get("labels_sha256"),
                "metrics": metrics,
            }
        )
        source_identity_records.append(source_audit)
        raw_input_count += 1

    require(
        raw_input_count == 20, f"expected 20 raw-only frames, got {raw_input_count}"
    )
    require(
        len(frame_rows) == 56, f"expected 56 classifier calls, got {len(frame_rows)}"
    )
    require(
        len({frame["fixed_root_point_id"] for frame in frame_rows}) > 0,
        "no fixed root IDs were preserved",
    )
    require(
        all(frame["roundtrip_arrays_equal"] for frame in frame_rows),
        "one or more frame NPZ round trips failed",
    )

    semantic = _score_semantic_proxy(output, frame_rows)
    require(
        semantic["evaluated_semantickitti_frames"] == 44,
        "expected postfit semantic evaluation for exactly 44 SemanticKITTI frames",
    )
    for frame in frame_rows:
        if frame.get("labels_path") and frame.get("labels_sha256"):
            source_hashes[frame["labels_path"]] = frame["labels_sha256"]
    for path, expected in list(source_hashes.items()):
        require(digest(Path(path)) == expected, f"source changed during run: {path}")
    for record in source_identity_records:
        path = record["path"]
        if record["current_binary_verified"]:
            require(
                digest(Path(path)) == record["expected_sha256"],
                f"raw source changed: {path}",
            )
    for audit in native_audits:
        require(
            digest(Path(audit["path"])) == audit["sha256_before"],
            f"native FusionOutput changed: {audit['path']}",
        )
    _verify_prior_snapshot(root, prior_snapshot)

    group_names = (
        "cached_semantickitti_seq08",
        "cached_nuscenes12",
        "raw_semantickitti_non08",
    )
    frame_groups = {
        name: _summarize_frames(
            [frame for frame in frame_rows if frame["dataset_group"] == name]
        )
        for name in group_names
    }
    persisted = {
        "status": "PASS_ALL_56_NPZ_ROUNDTRIPS",
        "frame_artifact_count": len(frame_rows),
        "input_frame_count": len(frame_rows),
        "all_point_ids_original_order": True,
        "all_classifier_arrays_roundtrip_exact": True,
        "string_arrays_compared_by_exact_equality": True,
        "numeric_arrays_compared_with_nan_equal": True,
        "v4_v5_prior_files_modified": False,
        "v4_v5_file_count_hash_checked": len(prior_snapshot),
        "v4_v5_prior_file_sha256": prior_snapshot,
        "v4_v5_prior_snapshot_sha256": prior_snapshot_sha,
        "frames": [
            {
                "frame_id": frame["frame_id"],
                "artifact_path": frame["artifact_path"],
                "artifact_sha256": frame["artifact_sha256"],
                "roundtrip_arrays_equal": frame["roundtrip_arrays_equal"],
                "roundtrip_array_count": frame["roundtrip_array_count"],
                "root_point_id": frame["fixed_root_point_id"],
            }
            for frame in frame_rows
        ],
    }
    case_matrix = {
        "schema_version": 1,
        "diagnostic_only": True,
        "case_count": len(frame_rows),
        "limits": limits_payload,
        "cases": [
            {
                "id": frame["frame_id"],
                "dataset_group": frame["dataset_group"],
                "fixed_root_point_id": frame["fixed_root_point_id"],
                "functional_verdict": (
                    "PASS_ROUNDTRIP_AND_FIXED_ROOT"
                    if frame["roundtrip_arrays_equal"]
                    else "FAIL"
                ),
                "terrain_candidate_area_m2": frame["metrics"][
                    "terrain_candidate_area_m2"
                ],
                "risk_constrained_candidate_area_m2": frame["metrics"][
                    "risk_constrained_candidate_area_m2"
                ],
                "terrain_connected_area_m2": frame["metrics"][
                    "terrain_connected_area_m2"
                ],
                "risk_constrained_connected_area_m2": frame["metrics"][
                    "risk_constrained_connected_area_m2"
                ],
                "risk_observed_cells": frame["metrics"]["risk_observed_cells"],
                "first_wrong_boundary": None,
            }
            for frame in frame_rows
        ],
    }
    report = {
        "schema_version": 1,
        "status": "PASS_DIAGNOSTIC_56_FRAME_REEXECUTION",
        "data_report_path": "report.json",
        "diagnostic_only": True,
        "prediction_source": (
            "56 new terrain-surface estimates from verified original raw inputs "
            "and frozen original IDs/root IDs; GT is opened only after all "
            "classifier outputs are persisted."
        ),
        "production_mutated": False,
        "FREE_or_safe_field_emitted": False,
        "vehicle_qualified": False,
        "limits": {
            **limits_payload,
            "path": str(limits_path),
            "sha256": limits_sha,
            "interpretation": (
                "Explicit diagnostic experiment hypotheses only; not calibrated "
                "vehicle limits or a safety qualification."
            ),
        },
        "inputs": {
            "cached_native_frames": cached_input_count,
            "raw_semantickitti_geometry_only_frames": raw_input_count,
            "total_fresh_classifier_calls": len(frame_rows),
            "v4_report_path": str(v4_report_path.relative_to(root)),
            "v4_report_sha256": v4_report_sha,
            "v5_identity_report_path": str(v5_report_path.relative_to(root)),
            "v5_identity_report_sha256": v5_report_sha,
            "v5_source_identity_entries": len(v5_sources),
            "source_identity": source_identity_records,
            "native_output_audits": native_audits,
            "source_sha256": source_hashes,
            "frozen_v4_frame_artifacts_verified": 56,
            "v4_v5_prior_artifacts_unchanged": True,
            "v4_v5_prior_file_count_hash_checked": len(prior_snapshot),
            "v4_v5_prior_file_sha256": prior_snapshot,
            "v4_v5_prior_snapshot_sha256": prior_snapshot_sha,
        },
        "groups": frame_groups,
        "total": _summarize_frames(frame_rows),
        "semantic_evaluation": semantic,
        "native_risk_coverage_contract": (
            "Native occupied rectangles are conservatively projected by "
            "positive-area overlap. Risk coverage requires complete area "
            "coverage by explicitly serialized native SpaceCell rectangles; "
            "sparse omitted cells remain unobserved. It is not ray visibility "
            "or a safety statement. Original object/unknown point risks are "
            "ORed into their terrain cells and native point decision reason "
            "bitfields are persisted independently."
        ),
        "limits_of_claim": [
            (
                "terrain_candidate is a diagnostic geometry candidate, "
                "not vehicle safety"
            ),
            (
                "risk_constrained_candidate requires observed native cell "
                "risk coverage"
            ),
            (
                "raw non-08 frames have no independent native risk input "
                "and remain unqualified"
            ),
            (
                "semantic-ground precision/recall is a postfit proxy for "
                "44 SemanticKITTI frames only"
            ),
            (
                "nuScenes ground truth and physical pit/bridge ground truth "
                "are unavailable"
            ),
            (
                "semantic-ground overlap with detector risk is diagnostic "
                "and never clears risk"
            ),
            (
                "pit, bridge, visibility, vehicle footprint, ego reachability, "
                "and safety qualification remain unavailable"
            ),
        ],
        "frames": frame_rows,
        "case_matrix_path": "case-matrix.json",
        "persisted_output_check_path": "persisted-output-check.json",
    }
    copied_limits_path = output / "limits.json"
    if limits_path != copied_limits_path.resolve():
        copied_limits_path.write_bytes(limits_path.read_bytes())
    _write_json(output / "case-matrix.json", case_matrix)
    _write_json(output / "persisted-output-check.json", persisted)
    _write_json(output / "report.json", report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--limits", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-archived-raw", action="store_true")
    args = parser.parse_args(argv)
    run_experiment(
        root=args.root,
        limits_path=args.limits,
        output=args.output,
        allow_archived_raw=args.allow_archived_raw,
    )


if __name__ == "__main__":
    main()
