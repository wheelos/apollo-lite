#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Frozen raw patch/roughness comparator; no production mutation or FREE."""

import argparse
import json
import math
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from evaluate_baseline import audit_frame, digest, require
from ground_continuity import connected_ground, grid_ids, instance_retention
from ground_experiment import measured_cells, obstacle_mask, radial_ground
from ground_validation import BANDS, proxy
from ground_validation_closure import common_comparison, components
from nuscenes import rates
from proto import Reader, require_full_evidence
from semantickitti import learning_labels
from spatial_support.adapter import scenes
from spatial_support.terrain import build_scene


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def source_frame(reader, run, sample, hashes):
    """Validate the original source boundary before any candidate execution."""
    inputs, results = Path(run["inputs"]), Path(run["results"])
    file = sample["file"]
    require(Path(file).name == file and file not in (".", ".."), "invalid sample name")
    for path, expected in (
        (inputs / file, sample["sha256"]),
        (Path(sample["source"]), sample["source_sha256"]),
    ):
        require(digest(path) == expected, f"changed source: {path}")
        hashes[str(path)] = expected
    config_path = results / "config.pb"
    native_path = results / (file + ".fusion.pb")
    hashes[str(config_path)] = digest(config_path)
    hashes[str(native_path)] = digest(native_path)
    config = reader.read(config_path, "apollo.lidar_fusion.FusionConfig")
    cloud = reader.read(inputs / file, "apollo.drivers.PointCloud")
    out = reader.read(native_path, "apollo.lidar_fusion.FusionOutput")
    require_full_evidence(out)
    audit_frame(out, config)
    require(
        len(cloud.point) == sample["points"] == len(out.point_evidence),
        "cloud/evidence point count mismatch",
    )
    header = type(out.header)()
    header.CopyFrom(cloud.header)
    header.timestamp_sec = cloud.measurement_time
    header.frame_id = config.base_frame
    header.module_name = "lidar_fusion"
    require(
        header == out.header
        and cloud.frame_id == config.sensor_frame
        and out.calibration_version == config.sensor_to_base.version,
        "cloud/output identity mismatch",
    )
    c = config.sensor_to_base
    cr, sr = math.cos(c.roll), math.sin(c.roll)
    cp, sp = math.cos(c.pitch), math.sin(c.pitch)
    cy, sy = math.cos(c.yaw), math.sin(c.yaw)
    rotation = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )
    raw_xyz = np.array([[p.x, p.y, p.z] for p in cloud.point])
    base = np.array(
        [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
    )
    finite = np.array([p.finite for p in out.point_evidence], dtype=bool)
    require(
        np.allclose(
            (raw_xyz @ rotation.T + [c.x, c.y, c.z])[finite],
            base[finite],
            rtol=0,
            atol=1e-9,
        ),
        "cloud/base transform or original point order mismatch",
    )
    if "labels" in sample:
        raw = np.fromfile(sample["source"], dtype="<f4")
        require(
            raw.size == 4 * len(raw_xyz)
            and np.array_equal(raw.reshape(-1, 4)[:, :3], raw_xyz),
            "original raw XYZ/order mismatch",
        )
    base[~finite] = np.nan
    return out, config, base, raw_xyz


def validate_prediction(points, estimate, ground, tolerance):
    require(
        estimate.shape == ground.shape == (len(points),)
        and ground.dtype == bool
        and np.array_equal(
            ground,
            np.isfinite(estimate) & (np.abs(points[:, 2] - estimate) <= tolerance),
        ),
        "ground artifact identity/classification mismatch",
    )
    require(
        not np.isfinite(estimate[~np.isfinite(points).all(axis=1)]).any(),
        "nonfinite points cannot have ground estimates",
    )


def summary(values):
    values = np.asarray(values)
    finite = values[np.isfinite(values)]
    return {
        "available_cells": int(len(finite)),
        "unavailable_cells": int(values.size - len(finite)),
        "min": float(finite.min()) if len(finite) else None,
        "median": float(np.median(finite)) if len(finite) else None,
        "p95": float(np.quantile(finite, 0.95)) if len(finite) else None,
        "max": float(finite.max()) if len(finite) else None,
    }


def persisted_arrays(path, points):
    with np.load(path, allow_pickle=False) as data:
        for name in data.files:
            require(data[name].dtype != object, f"pickle metadata forbidden: {name}")
        require(
            np.array_equal(data["point_id"], np.arange(len(points)))
            and np.array_equal(data["xyz_m"], points, equal_nan=True),
            "persisted raw point identity mismatch",
        )


def lost_cells(points, native_z, native, patch_z, patch, noise, config, estimate=None):
    old = obstacle_mask(points, native_z, native, noise, config)
    new = obstacle_mask(points, patch_z, patch, noise, config)
    old_report, old_region = connected_ground(points, native_z, native, old)
    new_report, new_region = connected_ground(points, patch_z, patch, new)
    buckets = {}
    for name, mask in (
        ("native_ground", native),
        ("patch_ground", patch),
        ("native_obstacle", old),
        ("patch_obstacle", new),
    ):
        ids, keys = grid_ids(points, mask, 0.5)
        buckets[name] = {}
        for i, key in zip(ids, keys):
            buckets[name].setdefault(tuple(key), []).append(int(i))
    counts, witnesses = Counter(), []
    for key in sorted(old_region - new_region):
        if key not in buckets["patch_ground"]:
            reason = "patch support/membership lost before connectivity"
        elif key in buckets["patch_obstacle"]:
            reason = "retained patch obstacle blocks observed ground"
        else:
            reason = "height/topology/original nearest-ring component selection"
        counts[reason] += 1
        if len(witnesses) < 20:
            ids = sorted(
                set(
                    buckets["native_ground"].get(key, [])
                    + buckets["patch_obstacle"].get(key, [])
                )
            )
            witnesses.append(
                {
                    "cell_xy": list(map(int, key)),
                    "first_boundary": reason,
                    "point_ids": ids,
                    "xyz_m": points[ids].tolist(),
                    "patch_supported": np.isfinite(patch_z[ids]).tolist(),
                    "native_ground": native[ids].tolist(),
                    "patch_ground": patch[ids].tolist(),
                    "new_blocker_point_ids": [
                        i for i in buckets["patch_obstacle"].get(key, []) if not old[i]
                    ],
                }
            )
    result = {
        "native": old_report,
        "patch": new_report,
        "lost_native_connected_cells": len(old_region - new_region),
        "first_boundary_counts": dict(counts),
        "witnesses": witnesses,
        "obstacles_removed_for_connectivity": False,
    }
    if estimate is not None:
        ny = estimate.patch_supported.shape[1]
        point_status = []
        for i in np.flatnonzero(native & ~patch):
            cell = int(estimate.cell_id[i])
            point_status.append(
                str(estimate.patch_status[divmod(cell, ny)])
                if cell >= 0
                else "outside_or_nonfinite"
            )
        result["native_ground_now_missed_patch_status_counts"] = dict(
            Counter(point_status)
        )
        for witness in witnesses:
            patches = sorted(
                {
                    int(estimate.cell_id[i])
                    for i in witness["point_ids"]
                    if estimate.cell_id[i] >= 0
                }
            )
            witness["patch_provenance"] = []
            for cell in patches:
                ix, iy = divmod(cell, ny)
                fit = estimate.plane_coefficients[ix, iy]
                seed_ids = np.flatnonzero(
                    (estimate.cell_id == cell) & estimate.seed_mask
                )
                raw_patch_ids = np.flatnonzero(estimate.cell_id == cell)
                witness["patch_provenance"].append(
                    {
                        "patch_id": cell,
                        "patch_ix_iy": [ix, iy],
                        "status": str(estimate.patch_status[ix, iy]),
                        "seed_count": int(estimate.seed_count[ix, iy]),
                        "seed_original_point_ids": seed_ids.tolist(),
                        "seed_xyz_m": points[seed_ids].tolist(),
                        "seed_mask_is_accepted_fit_only": True,
                        "raw_patch_point_count": len(raw_patch_ids),
                        "first_raw_patch_point_ids": raw_patch_ids[:20].tolist(),
                        "full_patch_reproduction": (
                            "NPZ cell_id equality + xyz_m + frozen options"
                        ),
                        "plane_coefficients": (
                            fit.tolist() if np.isfinite(fit).all() else None
                        ),
                        "fit_condition_number": (
                            float(estimate.condition_number[ix, iy])
                            if np.isfinite(estimate.condition_number[ix, iy])
                            else None
                        ),
                        "fit_residual_max_m": (
                            float(estimate.fit_residual_max[ix, iy])
                            if np.isfinite(estimate.fit_residual_max[ix, iy])
                            else None
                        ),
                        "diagnostics_are_not_calibrated_uncertainty": True,
                    }
                )
    return result


def compare(points, estimates, masks, noise, config):
    continuity, regions, obstacles = {}, {}, {}
    for name in estimates:
        obstacles[name] = obstacle_mask(
            points, estimates[name], masks[name], noise, config
        )
        continuity[name], region = connected_ground(
            points, estimates[name], masks[name], obstacles[name]
        )
        regions[name] = np.asarray(sorted(region), dtype=np.int64).reshape(-1, 2)
    common = {}
    for name in ("native", "radial"):
        if name in estimates:
            common[name + "_patch"] = common_comparison(
                components(points, estimates[name], masks[name], obstacles[name]),
                components(
                    points, estimates["patch"], masks["patch"], obstacles["patch"]
                ),
                continuity[name]["effective_seed_radius_m"],
                continuity["patch"]["effective_seed_radius_m"],
            )
    return continuity, regions, obstacles, common


def evaluate_labels(points, packed, masks, obstacles, config):
    truth = learning_labels(packed)
    require(len(truth) == len(points), "raw/label order/count mismatch")
    roi = (
        np.isfinite(points).all(axis=1)
        & (points[:, 0] >= config.min_x)
        & (points[:, 0] < config.max_x)
        & (points[:, 1] >= config.min_y)
        & (points[:, 1] < config.max_y)
    )
    thing = roi & (truth > 0) & (truth <= 8)
    baseline = "native" if "native" in obstacles else "radial"
    return {
        "proxy_by_distance": proxy(points, truth, masks, roi),
        "retention": {
            name: instance_retention(
                points, packed >> 16, thing, obstacles[baseline], mask, 0.1
            )
            for name, mask in obstacles.items()
        },
        "retention_baseline": baseline,
        "GT_join_after_estimation": True,
        "terrain_accuracy_calibrated": False,
    }


def case_matrix(report, output, hashes):
    cases = []
    report_path = output / "ground-report.json"

    def add(name, expected, actual, status, source=report_path, boundary=None):
        source = Path(source)
        cases.append(
            {
                "case": name,
                "expected": expected,
                "actual": actual,
                "status": status,
                "source": str(source),
                "hash": digest(source),
                "first_wrong_boundary": boundary,
                "verification_complete": True,
                "feature_accepted": False,
                "nextaction": (
                    "Retain failure; inspect source-bound support/fit/"
                    "blocker witnesses, "
                    "without GT tuning, blind fill or risk filtering."
                    if status == "FAIL"
                    else "Retain bounded observed diagnostic evidence; calibration and "
                    "vehicle/production qualification remain absent."
                ),
            }
        )

    cached = report["cached_36"]["frames"]
    add(
        "real-raw-new-patch-execution",
        "Recompute independent patch on all 24 SK + 12 nuScenes exact original frames",
        {
            "frames": len(cached),
            "labeled": sum(r["labeled"] for r in cached),
            "raw_source_checked": all(r["source_boundary_validated"] for r in cached),
        },
        "PASS" if len(cached) == 36 else "FAIL",
    )
    add(
        "native-radial-unchanged-controls",
        "Native immutable and radial bitwise identical to frozen v2 NPZ",
        {
            "native_immutable": all(r["native_immutable"] for r in cached),
            "radial_matches": sum(r["radial_matches_previous"] for r in cached),
        },
        (
            "PASS"
            if all(
                r["radial_matches_previous"] and r["native_immutable"] for r in cached
            )
            else "FAIL"
        ),
    )
    old_matrix_path = output.parent / "continuation-v2/ground-case-matrix.json"
    previous = json.loads(old_matrix_path.read_text())
    six = {
        c["case"] for c in previous["cases"] if c["case"].startswith("semantickitti")
    }
    for row in cached:
        name = row["case"] + "/" + row["file"]
        old = row["continuity"]["native"]
        patch = row["continuity"]["patch"]
        radial = row["continuity"]["radial"]
        if name in six:
            add(
                "six08-continuity/" + name,
                "Patch must not regress versus native under unchanged original seeds",
                {
                    "native_cells": old["ego_connected_cells"],
                    "radial_cells": radial["ego_connected_cells"],
                    "patch_cells": patch["ego_connected_cells"],
                    "common_observed_seed": row["common_seed"],
                    "cause": row["cause"],
                },
                (
                    "PASS"
                    if patch["ego_connected_cells"] >= old["ego_connected_cells"]
                    else "FAIL"
                ),
                Path(row["algorithm_artifact"]),
                "raw patch fit/support -> candidate membership/obstacle "
                "-> observed topology",
            )
        for radius in (5, 10):
            key = f"within_{radius}m"
            add(
                f"near-continuity/{radius}m/" + name,
                "Original observed connected cells must not regress; no blind fill",
                {n: c[key] for n, c in row["continuity"].items()},
                (
                    "PASS"
                    if patch[key]["connected_cells"] >= old[key]["connected_cells"]
                    else "FAIL"
                ),
                Path(row["algorithm_artifact"]),
            )
        add(
            "raw-roughness/" + name,
            "Execute raw elevation/plane-normal residual metrics; preserve sparse NaN",
            row["roughness"],
            "DIAGNOSTIC",
            Path(row["algorithm_artifact"]),
        )
        if not row["labeled"]:
            add(
                "nuscenes-no-ground-accuracy/" + name,
                "Mechanism only; no joined ground GT means accuracy cannot be assessed",
                {
                    "ground_GT": False,
                    "continuity": row["continuity"],
                    "patch": row["patch_support"],
                },
                "DIAGNOSTIC",
                Path(row["algorithm_artifact"]),
            )
    for lo, hi in BANDS:
        actual = {}
        for name in ("native", "radial", "patch"):
            counts = Counter()
            for row in cached:
                metric = row.get("proxy_by_distance", {}).get(f"{lo}-{hi}m", {})
                if name in metric:
                    counts.update(
                        {k: metric[name][k] for k in ("tp", "fp", "fn", "tn")}
                    )
            actual[name] = rates(counts)
        add(
            f"distance-coverage/{lo}-{hi}m",
            "Patch semantic-ground proxy recall must not regress against native",
            actual,
            (
                "PASS"
                if actual["patch"]["recall"] >= actual["native"]["recall"]
                else "FAIL"
            ),
        )
    totals = report["summary"]["cached_retention"]
    for name in ("radial", "patch"):
        total = totals[name]
        add(
            "all-original-support-complete/" + name,
            "Zero completely erased original supported instance-frames; include sparse",
            total,
            (
                "PASS"
                if total["all_completely_lost_instances"] == 0
                and total["fully_replaced_original_support_instances"] == 0
                else "FAIL"
            ),
        )
        add(
            "all-original-support-partial/" + name,
            "Stricter conservative diagnostic: no original voxel support loss; "
            "partial loss is separately disclosed, replacement does not compensate",
            total,
            "PASS" if total["partially_lost_instances"] == 0 else "FAIL",
        )
    raw = report["cross_sequence_raw_20"]["frames"]
    for row in raw:
        add(
            f"raw-non08/{row['sequence']}/{row['ordinal']:06d}",
            "Frozen raw patch and radial each execute all three "
            "fixed height hypotheses",
            row,
            "DIAGNOSTIC",
            Path(row["algorithm_artifact"]),
        )
    add(
        "calibration-physical-roughness-accuracy",
        "Verified sensor/base/reference-point/ground-height and physical terrain GT",
        {
            "calibrated": False,
            "physical_terrain_GT": False,
            "raw_height_sensitivity_not_holdout": True,
        },
        "BLOCKED",
    )
    add(
        "no-FREE-or-production",
        "Never infer FREE, mutate source production or filter risks "
        "to improve continuity",
        {"FREE_emitted": False, "production_mutated": False, "risk_filter_used": False},
        "PASS",
    )
    write(
        output / "case-matrix.json",
        {
            "freeze": str(output / "ground-frozen-case-contracts.json"),
            "freeze_sha256": hashes[str(output / "ground-frozen-case-contracts.json")],
            "cases": cases,
            "status_counts": dict(Counter(c["status"] for c in cases)),
            "previous_evidence": {
                "source": str(old_matrix_path),
                "sha256": digest(old_matrix_path),
                "referenced_cases": len(previous["cases"]),
                "newly_executed": False,
            },
            "verification_complete": True,
            "feature_accepted": False,
            "meaning": "Execution/diagnostic coverage is complete; failed effects and "
            "missing calibration remain independent acceptance blockers.",
        },
    )


def patch_run(points, config):
    from ground_patch import GroundPatchConfig, estimate_ground_patch
    from spatial_support.observed_terrain import observed_terrain_metrics

    options = GroundPatchConfig(
        coordinate_frame=config.base_frame,
        x_min=config.min_x,
        x_max=config.max_x,
        y_min=config.min_y,
        y_max=config.max_y,
        expected_ground_z=config.expected_ground_z,
        ground_seed_band=config.ground_seed_band,
        max_slope_gradient=config.ground_max_slope,
        max_fit_residual=config.ground_max_residual,
        ground_tolerance=config.ground_tolerance,
    )
    estimate = estimate_ground_patch(points, config=options)
    validate_prediction(
        points, estimate.ground_z, estimate.ground_mask, config.ground_tolerance
    )
    observed = observed_terrain_metrics(points, estimate)
    arrays = {
        name: value
        for name, value in vars(estimate).items()
        if isinstance(value, np.ndarray) and name != "ground_z"
    }
    arrays.update(
        {
            "observed_" + name: value
            for name, value in vars(observed).items()
            if isinstance(value, np.ndarray)
        }
    )
    arrays.update(
        {
            "coordinate_frame": np.asarray(config.base_frame),
            "patch_options_json": np.asarray(
                json.dumps(asdict(options), sort_keys=True)
            ),
            "expected_ground_z_m": np.asarray(config.expected_ground_z),
        }
    )
    shape = estimate.patch_supported.shape
    fitted = build_scene(
        points,
        estimate.ground_z,
        estimate.ground_mask,
        options.patch_size,
        [options.x_min, options.y_min],
        np.zeros(shape, dtype=np.uint8),
        np.zeros(shape, dtype=bool),
        options.max_slope_gradient,
        0.2,
        options.max_fit_residual,
    )
    arrays["fitted_proxy_elevation"] = fitted.ground_z
    arrays["fitted_proxy_roughness"] = fitted.roughness
    roughness = {
        "coordinate_frame": config.base_frame,
        "raw_Z_spread_frame_dependent": True,
        "normal_distance_orthogonal_metric": True,
        "raw_metrics": {
            name: summary(value)
            for name, value in vars(observed).items()
            if isinstance(value, np.ndarray) and np.issubdtype(value.dtype, np.floating)
        },
        "seed_fit_residual_rms_m": summary(estimate.fit_residual_rms),
        "seed_fit_residual_max_m": summary(estimate.fit_residual_max),
        "fitted_height_local_plane_max_residual_m": summary(fitted.roughness),
        "fitted_proxy_grid_matches_raw_patch_layout": True,
        "interpretation": (
            "All-return Z/normal-distance spread may include obstacles. Candidate "
            "ground residual uses raw returns selected by the estimator and is "
            "selection-conditioned, not independent physical terrain truth. "
            "Seed-fit residual is in-sample. Fitted proxy uses cell medians of "
            "predicted z, not raw measured Z. No terrain accuracy calibration."
        ),
    }
    support = {
        "points": len(points),
        "supported_points": int(np.isfinite(estimate.ground_z).sum()),
        "ground_points": int(estimate.ground_mask.sum()),
        "seed_points": int(estimate.seed_mask.sum()),
        "supported_patches": int(estimate.patch_supported.sum()),
        "unsupported_patches": int(
            shape[0] * shape[1] - estimate.patch_supported.sum()
        ),
        "supported_but_not_ground_points": int(
            (np.isfinite(estimate.ground_z) & ~estimate.ground_mask).sum()
        ),
        "patch_status_counts": (
            {
                str(k): int(v)
                for k, v in zip(*np.unique(estimate.patch_status, return_counts=True))
            }
            if hasattr(estimate, "patch_status")
            else None
        ),
    }
    return estimate, arrays, roughness, support, asdict(options)


def execute(args):
    root = args.output
    root.mkdir(parents=True, exist_ok=True)
    previous = json.loads(args.previous.read_text())
    reader = Reader(args.schema)
    frozen_path = root / "ground-frozen-case-contracts.json"
    require(
        json.loads(frozen_path.read_text())["frozen_before_new_predictions"],
        "case assertions must be frozen before new prediction execution",
    )
    source_files = [
        args.previous,
        args.schema,
        args.inventory,
        frozen_path,
        *(
            Path(__file__).with_name(n)
            for n in (
                "ground_patch_experiment.py",
                "ground_patch.py",
                "ground_experiment.py",
                "ground_continuity.py",
                "ground_validation.py",
                "ground_validation_closure.py",
                "proto.py",
                "evaluate_baseline.py",
                "semantickitti.py",
                "nuscenes.py",
            )
        ),
        *Path(__file__).with_name("spatial_support").glob("*.py"),
    ]
    hashes = {str(p): digest(p) for p in source_files}
    cached, raw_rows, options_by_run = [], [], {}
    for run in previous["experiment"]["runs"]:
        require(Path(run["name"]).name == run["name"], "invalid run name")
        manifest_path = Path(run["inputs"]) / "manifest.json"
        hashes[str(manifest_path)] = digest(manifest_path)
        manifest = json.loads(manifest_path.read_text())
        require(
            len({s["file"] for s in manifest["samples"]}) == len(manifest["samples"]),
            "duplicate samples",
        )
        for sample in manifest["samples"]:
            out, config, points, sensor_xyz = source_frame(reader, run, sample, hashes)
            before = out.SerializeToString(deterministic=True)
            native = np.array([p.is_ground_geo for p in out.point_evidence], dtype=bool)
            native_z = np.array(
                [p.z_ground if p.ground_valid else np.nan for p in out.point_evidence]
            )
            noise = np.array(
                [d.explanation == 2 for d in out.point_decision], dtype=bool
            )
            require(
                measured_cells(points, native_z, native, noise, config)
                == {
                    (c.x_index, c.y_index)
                    for c in out.space_map.cell
                    if c.occupancy_obs
                },
                "native measured occupancy source mismatch",
            )
            radial_z = radial_ground(points, config)
            radial = np.isfinite(radial_z) & (
                np.abs(points[:, 2] - radial_z) <= config.ground_tolerance
            )
            frozen = (
                args.previous.parent / run["name"] / (sample["file"] + ".ground.npz")
            )
            hashes[str(frozen)] = digest(frozen)
            with np.load(frozen, allow_pickle=False) as data:
                require(
                    np.array_equal(radial_z, data["ground_z"], equal_nan=True)
                    and np.array_equal(radial, data["ground"])
                    and np.array_equal(data["point_id"], np.arange(len(points))),
                    "unchanged radial control differs from frozen v2",
                )
            patch, arrays, roughness, support, options = patch_run(points, config)
            options_by_run[run["name"]] = options
            estimates = {
                "native": native_z,
                "radial": radial_z,
                "patch": patch.ground_z,
            }
            masks = {"native": native, "radial": radial, "patch": patch.ground_mask}
            continuity, regions, obstacles, common = compare(
                points, estimates, masks, noise, config
            )
            # Exercise the existing immutable ground-source adapter, including its
            # exact classification and connected-region validation, not a bypass.
            source_scenes = scenes(
                out,
                config,
                patch.ground_z,
                patch.ground_mask,
                0.2,
                {"baseline": regions["native"], "candidate": regions["patch"]},
                0.5,
            )
            require(
                not source_scenes["candidate"].verified_free.any(),
                "hypothetical patch cannot authorize FREE",
            )
            roughness["native_grid_fitted_proxy_m"] = {
                name: summary(scene.roughness) for name, scene in source_scenes.items()
            }
            del source_scenes
            row = {
                "case": run["name"],
                "file": sample["file"],
                "labeled": "labels" in sample,
                "source_boundary_validated": True,
                "radial_matches_previous": True,
                "raw_source": sample["source"],
                "raw_sha256": sample["source_sha256"],
                "frame_id": out.header.frame_id,
                "sensor_frame": config.sensor_frame,
                "estimator_frame": config.base_frame,
                "height_hypothesis_in_estimator_frame_m": config.expected_ground_z,
                "sensor_to_estimator_transform": {
                    name: getattr(config.sensor_to_base, name)
                    for name in ("x", "y", "z", "roll", "pitch", "yaw", "version")
                },
                "base_transform_physical_calibration_verified": False,
                "ground_height_reference_point_calibrated": False,
                "candidate_ground_points": int(patch.ground_mask.sum()),
                "candidate_supported_points": int(np.isfinite(patch.ground_z).sum()),
                "baseline_ground_points": int(native.sum()),
                "continuity": continuity,
                "common_seed": common,
                "patch_support": support,
                "roughness": roughness,
                "production_mutated": False,
                "FREE_emitted": False,
                "candidate_measured_occupancy_hypothesis": {
                    "native_cells": len(
                        measured_cells(points, native_z, native, noise, config)
                    ),
                    "patch_cells": len(
                        measured_cells(
                            points, patch.ground_z, patch.ground_mask, noise, config
                        )
                    ),
                    "source_native_occupancy_unchanged": True,
                    "physical_authorization": False,
                },
            }
            if "labels" in sample:
                require(
                    "SemanticKITTI" in manifest["dataset"],
                    "only SemanticKITTI labels implemented",
                )
                label = Path(sample["labels"])
                require(digest(label) == sample["labels_sha256"], "changed GT")
                hashes[str(label)] = sample["labels_sha256"]
                row.update(
                    evaluate_labels(
                        points,
                        np.fromfile(label, dtype="<u4"),
                        masks,
                        obstacles,
                        config,
                    )
                )
            row["cause"] = lost_cells(
                points,
                native_z,
                native,
                patch.ground_z,
                patch.ground_mask,
                noise,
                config,
                estimate=patch,
            )
            path = root / run["name"] / (sample["file"] + ".ground.npz")
            path.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(
                path,
                point_id=np.arange(len(points), dtype=np.uint32),
                xyz_m=points,
                sensor_xyz_m=sensor_xyz,
                ground_z=patch.ground_z,
                ground=patch.ground_mask,
                radial_ground_z=radial_z,
                radial_ground=radial,
                native_ground_z=native_z,
                native_ground=native,
                baseline_connected_xy=regions["native"],
                candidate_connected_xy=regions["patch"],
                radial_connected_xy=regions["radial"],
                native_raw_obstacle=obstacles["native"],
                candidate_raw_obstacle=obstacles["patch"],
                **arrays,
            )
            persisted_arrays(path, points)
            hashes[str(path)] = digest(path)
            row["algorithm_artifact"] = str(path)
            row["algorithm_artifact_sha256"] = hashes[str(path)]
            require(
                before == out.SerializeToString(deterministic=True), "native mutated"
            )
            row["native_immutable"] = True
            cached.append(row)
            write(root / "ground-progress.json", {"cached_frames_complete": cached})
            print(
                f"patch cached {run['name']}/{sample['file']} "
                f"connected native/radial/patch "
                f"{continuity['native']['ego_connected_cells']}/"
                f"{continuity['radial']['ego_connected_cells']}/"
                f"{continuity['patch']['ego_connected_cells']}",
                flush=True,
            )
    inventory = json.loads(args.inventory.read_text())
    heights = (-1.43, -1.73, -2.03)
    for sample in inventory["selection_details"]:
        raw_path, label_path = Path(sample["raw_scan"]), Path(sample["labels"])
        for path, expected in (
            (raw_path, sample["raw_sha256"]),
            (label_path, sample["labels_sha256"]),
        ):
            require(digest(path) == expected, "raw inventory source changed")
            hashes[str(path)] = expected
        points = np.fromfile(raw_path, dtype="<f4").reshape(-1, 4)[:, :3]
        require(len(points) == sample["point_count"], "raw inventory count mismatch")
        variants, arrays = {}, {
            "point_id": np.arange(len(points), dtype=np.uint32),
            "xyz_m": points,
        }
        label_inputs = {}
        frozen_raw = args.previous.parent.parent / (
            f"ground-raw-{sample['sequence']}-{sample['ordinal']:06d}.npz"
        )
        hashes[str(frozen_raw)] = digest(frozen_raw)
        for height in heights:
            config = SimpleNamespace(
                min_x=-60,
                max_x=60,
                min_y=-40,
                max_y=40,
                expected_ground_z=height,
                base_frame="kitti_velodyne",
                ground_seed_band=0.3,
                ground_max_slope=0.3,
                ground_max_residual=0.08,
                ground_tolerance=0.1,
                obstacle_min_height=0.1,
                clearance_height=2.0,
            )
            radial_z = radial_ground(points, config)
            radial = np.isfinite(radial_z) & (np.abs(points[:, 2] - radial_z) <= 0.1)
            with np.load(frozen_raw, allow_pickle=False) as data:
                require(
                    np.array_equal(radial_z, data[f"z_{height}"], equal_nan=True)
                    and np.array_equal(radial, data[f"ground_{height}"]),
                    "raw radial frozen control changed",
                )
            patch, extra, roughness, support, options = patch_run(points, config)
            estimates = {"radial": radial_z, "patch": patch.ground_z}
            masks = {"radial": radial, "patch": patch.ground_mask}
            continuity, regions, obstacles, common = compare(
                points, estimates, masks, np.zeros(len(points), dtype=bool), config
            )
            variants[str(height)] = {
                "patch_options": options,
                "patch_support": support,
                "roughness": roughness,
                "continuity": continuity,
                "common_seed": common,
                "radial_matches_previous": True,
                "height_status": "uncalibrated fixed hypothesis, NOT holdout/tuned",
                "obstacle_contract": (
                    "Raw ground-only comparison; no native fusion/noise/detection "
                    "cache. Height eligibility 0.1-2m is diagnostic assumed, not "
                    "vehicle clearance qualification."
                ),
            }
            label_inputs[str(height)] = (masks, obstacles, config)
            arrays.update({f"{height}_{n}": v for n, v in extra.items()})
            arrays.update(
                {
                    f"{height}_ground_z": patch.ground_z,
                    f"{height}_ground": patch.ground_mask,
                    f"{height}_radial_ground_z": radial_z,
                    f"{height}_radial_ground": radial,
                }
            )
        packed = np.fromfile(label_path, dtype="<u4")
        for name, (masks, obstacles, config) in label_inputs.items():
            variants[name].update(
                evaluate_labels(points, packed, masks, obstacles, config)
            )
        path = root / f"patch-raw-{sample['sequence']}-{sample['ordinal']:06d}.npz"
        np.savez_compressed(path, **arrays)
        persisted_arrays(path, points)
        hashes[str(path)] = digest(path)
        raw_rows.append(
            {
                "sequence": sample["sequence"],
                "ordinal": sample["ordinal"],
                "points": len(points),
                "variants": variants,
                "raw_source": str(raw_path),
                "raw_sha256": sample["raw_sha256"],
                "algorithm_artifact": str(path),
                "algorithm_artifact_sha256": hashes[str(path)],
                "GT_join_after_all_three_predictions": True,
                "frame": (
                    "native Velodyne sensor, not calibrated vehicle "
                    "base/reference point"
                ),
            }
        )
        print(
            f"patch raw {sample['sequence']}/{sample['ordinal']:06d} 3 heights",
            flush=True,
        )
        write(
            root / "ground-progress.json",
            {
                "cached_frames": len(cached),
                "raw_frames_complete": raw_rows,
            },
        )
    for path, expected in hashes.items():
        require(digest(Path(path)) == expected, f"source changed during run: {path}")
    retention = {}
    for name in ("radial", "patch"):
        counts = Counter()
        replaced = 0
        for row in cached:
            metrics = row.get("retention", {}).get(name, {})
            counts.update({k: v for k, v in metrics.items() if isinstance(v, int)})
            replaced += len(
                metrics.get("baseline_support_completely_replaced_instance_ids", [])
            )
        retention[name] = dict(counts)
        retention[name]["fully_replaced_original_support_instances"] = replaced
    report = {
        "diagnostic_only": True,
        "algorithm": "independent observed XY patch plane v1",
        "options_by_run": options_by_run,
        "continuity_options": previous["continuity_options"],
        "experiment": previous["experiment"],
        "frames": cached,
        "cached_36": {"frames": cached},
        "cross_sequence_raw_20": {
            "frames": raw_rows,
            "fixed_heights_m": list(heights),
            "selection": inventory["selection_rule_frozen_before_predictions"],
            "not_holdout": True,
            "height_calibrated": False,
        },
        "summary": {
            "cached_frames": len(cached),
            "cached_labeled_frames": sum(r["labeled"] for r in cached),
            "raw_non08_frames": len(raw_rows),
            "raw_non08_patch_executions": 3 * len(raw_rows),
            "native_radial_controls_unchanged": True,
            "cached_retention": retention,
            "connected_area_regressions_vs_native": sum(
                r["continuity"]["patch"]["ego_connected_cells"]
                < r["continuity"]["native"]["ego_connected_cells"]
                for r in cached
                if r["labeled"]
            ),
        },
        "source_sha256": hashes,
        "verification_complete": True,
        "feature_accepted": False,
        "production_mutated": False,
        "FREE_emitted": False,
        "risk_filter_used": False,
        "terrain_accuracy_calibrated": False,
    }
    write(root / "ground-report.json", report)
    write(root / "report.json", report)
    case_matrix(report, root, hashes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("previous", "schema", "inventory", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    execute(args)


if __name__ == "__main__":
    main()
