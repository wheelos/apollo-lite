#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Frozen raw ground diagnostics, counterfactuals and all-instance accounting."""

import argparse
import json
from collections import Counter
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from evaluate_baseline import digest, require
from ground_continuity import (
    ContinuityOptions,
    connected_ground,
    grid_ids,
    instance_retention,
    sparse_obstacles,
)
from ground_experiment import RadialOptions, obstacle_mask, radial_ground
from nuscenes import rates
from proto import Reader
from semantickitti import GROUND, NAMES, binary_counts, learning_labels

BANDS = ((0, 5), (5, 15), (15, 30), (30, float("inf")))


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def proxy(points, truth, predictions, roi):
    actual = np.isin(truth, GROUND)
    radius = np.hypot(points[:, 0], points[:, 1])
    result = {}
    for lo, hi in BANDS:
        domain = roi & (truth != 0) & (radius >= lo) & (radius < hi)
        result[f"{lo}-{hi}m"] = {}
        for name, predicted in predictions.items():
            result[f"{lo}-{hi}m"][name] = rates(
                binary_counts(actual, predicted, domain)
            )
    return result


def cell_buckets(points, mask, resolution):
    ids, keys = grid_ids(points, mask, resolution)
    result = {}
    for i, key in zip(ids, keys):
        result.setdefault(tuple(key), []).append(int(i))
    return result


def radial_witness(points, point_id, trace):
    point = points[point_id]
    radius = float(np.hypot(*point[:2]))
    sector = int(
        np.floor(
            np.mod(np.arctan2(point[1], point[0]) + np.pi, 2 * np.pi)
            * 360
            / (2 * np.pi)
        )
    )
    state = trace.get(sector, {})
    low = state.get("low_point_ids", [])
    support = state.get("supported_point_ids", [])
    before = [
        event
        for event in state.get("events", [])
        if event["sample"] is not None and event["sample"][1] <= radius
    ]
    return {
        "point_id": int(point_id),
        "xyz_m": point.tolist(),
        "radius_m": radius,
        "sector": sector,
        "sector_low_bin_count": len(low),
        "sector_max_supported_radius_m": (
            float(np.max(np.hypot(points[support, 0], points[support, 1])))
            if support
            else None
        ),
        "first_low_samples": [
            {"point_id": i, "xyz_m": points[i].tolist()} for i in low[:3]
        ],
        "last_state_events_before_point": before[-6:],
        "events_near_point": [
            event
            for event in state.get("events", [])
            if event["sample"] is not None and abs(event["sample"][1] - radius) <= 0.5
        ],
        "first_low_radius_m": [
            float(np.hypot(points[i, 0], points[i, 1])) for i in low[:3]
        ],
        "event_counts": dict(
            Counter(event["reason"] for event in state.get("events", []))
        ),
    }


def counterfactual(
    points, base_z, baseline, estimate, candidate, old, new, truth, trace
):
    options = ContinuityOptions()
    combinations = {
        "baseline": (base_z, baseline, old),
        "candidate": (estimate, candidate, new),
        "candidate_with_baseline_obstacles": (estimate, candidate, old),
        "baseline_with_candidate_obstacles": (base_z, baseline, new),
        "candidate_membership_baseline_heights": (
            np.where(np.isfinite(base_z), base_z, estimate),
            candidate,
            old,
        ),
    }
    summaries, regions = {}, {}
    for name, (z, ground, obstacle) in combinations.items():
        summaries[name], regions[name] = connected_ground(
            points, z, ground, obstacle, options
        )
    fixed_seed = ContinuityOptions(
        seed_radius=summaries["baseline"]["effective_seed_radius_m"]
    )
    (
        summaries["candidate_fixed_baseline_seed"],
        regions["candidate_fixed_baseline_seed"],
    ) = connected_ground(points, estimate, candidate, new, fixed_seed)
    base_region = regions["baseline"]
    candidate_region = regions["candidate"]
    old_ground = cell_buckets(points, baseline, options.resolution)
    new_ground = cell_buckets(points, candidate, options.resolution)
    old_obstacles = cell_buckets(points, old, options.resolution)
    new_obstacles = cell_buckets(points, new, options.resolution)
    cells = []
    for key in sorted(base_region - candidate_region):
        ground_ids = old_ground.get(key, [])
        blocker_ids = new_obstacles.get(key, [])
        if key not in new_ground:
            boundary = "radial support/membership lost before connectivity"
        elif key in new_obstacles:
            boundary = "candidate obstacle mask blocks observed ground cell"
        else:
            boundary = "topology/height/nearest-ring component selection"
        cells.append(
            {
                "cell_xy": [int(value) for value in key],
                "first_boundary": boundary,
                "baseline_ground_point_ids": ground_ids,
                "candidate_ground_point_ids": new_ground.get(key, []),
                "baseline_obstacle_point_ids": old_obstacles.get(key, []),
                "candidate_obstacle_point_ids": blocker_ids,
                "new_blocker_evidence": [
                    {
                        "point_id": i,
                        "xyz_m": points[i].tolist(),
                        "baseline_ground": bool(baseline[i]),
                        "baseline_ground_z_m": (
                            float(base_z[i]) if np.isfinite(base_z[i]) else None
                        ),
                        "candidate_ground": bool(candidate[i]),
                        "candidate_supported": bool(np.isfinite(estimate[i])),
                        "candidate_ground_z_m": (
                            float(estimate[i]) if np.isfinite(estimate[i]) else None
                        ),
                        "semantic_proxy": NAMES[int(truth[i])],
                    }
                    for i in blocker_ids
                    if i not in old_obstacles.get(key, [])
                ],
                "first_new_blocker_radial_trace": (
                    radial_witness(
                        points,
                        next(
                            i
                            for i in blocker_ids
                            if i not in old_obstacles.get(key, [])
                        ),
                        trace,
                    )
                    if any(i not in old_obstacles.get(key, []) for i in blocker_ids)
                    else None
                ),
            }
        )
    # Change one observed cell only. Never feed GT into this counterfactual.
    # These diagnostic interventions are NOT an obstacle-deletion proposal.
    witnesses = []
    for cell in cells:
        key = tuple(cell["cell_xy"])
        ids = new_obstacles.get(key, [])
        if not ids:
            continue
        changed = new.copy()
        changed[ids] = False
        summary, region = connected_ground(
            points, estimate, candidate, changed, options
        )
        gain = len(region) - len(candidate_region)
        witnesses.append(
            {
                "cell_xy": [int(value) for value in key],
                "removed_point_ids": ids,
                "connected_cells": summary["ego_connected_cells"],
                "gain": gain,
                "not_a_fix": True,
            }
        )
    witnesses.sort(key=lambda row: (-row["gain"], row["cell_xy"]))
    return {
        "counterfactuals": summaries,
        "lost_baseline_connected_cells": len(cells),
        "first_boundary_counts": dict(Counter(c["first_boundary"] for c in cells)),
        "per_cell": cells,
        "single_cell_blocker_removal_witnesses": witnesses[:10],
        "safety": "counterfactual removal is diagnostic only; not recommended",
    }


def cached(args, output):
    previous = json.loads(args.previous.read_text())
    reader = Reader(args.schema)
    rows, hashes = [], {}
    for path in (
        args.previous,
        args.schema,
        args.output / "ground-frozen-case-contracts.json",
        *(
            Path(__file__).with_name(name)
            for name in (
                "ground_validation.py",
                "ground_experiment.py",
                "ground_continuity.py",
                "evaluate_baseline.py",
                "semantickitti.py",
                "nuscenes.py",
                "proto.py",
            )
        ),
    ):
        hashes[str(path)] = digest(path)
    for run in previous["experiment"]["runs"]:
        inputs, results = Path(run["inputs"]), Path(run["results"])
        manifest_path = inputs / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        config_path = results / "config.pb"
        config = reader.read(config_path, "apollo.lidar_fusion.FusionConfig")
        hashes[str(manifest_path)] = digest(manifest_path)
        hashes[str(config_path)] = digest(config_path)
        for sample in manifest["samples"]:
            artifact = results / (sample["file"] + ".fusion.pb")
            hashes[str(artifact)] = digest(artifact)
            raw_path = Path(sample["source"])
            require(digest(raw_path) == sample["source_sha256"], "raw hash")
            hashes[str(raw_path)] = sample["source_sha256"]
            out = reader.read(artifact, "apollo.lidar_fusion.FusionOutput")
            points = np.array(
                [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
            )
            baseline = np.array([p.is_ground_geo for p in out.point_evidence])
            base_z = np.array(
                [p.z_ground if p.ground_valid else np.nan for p in out.point_evidence]
            )
            noise = np.array([d.explanation == 2 for d in out.point_decision])
            trace = {}
            estimate = radial_ground(points, config, trace=trace)
            candidate = np.isfinite(estimate) & (
                np.abs(points[:, 2] - estimate) <= config.ground_tolerance
            )
            previous_npz = (
                args.previous.parent / run["name"] / (sample["file"] + ".ground.npz")
            )
            hashes[str(previous_npz)] = digest(previous_npz)
            with np.load(previous_npz) as frozen:
                require(
                    np.array_equal(estimate, frozen["ground_z"], equal_nan=True)
                    and np.array_equal(candidate, frozen["ground"]),
                    "diagnostic change altered frozen radial estimator",
                )
            old = obstacle_mask(points, base_z, baseline, noise, config)
            new = obstacle_mask(points, estimate, candidate, noise, config)
            filtered, _ = sparse_obstacles(points, new)
            row = {"case": run["name"], "file": sample["file"]}
            row["radial_estimate_matches_previous"] = True
            row["continuity"] = {
                name: connected_ground(points, z, ground, obstacle)[0]
                for name, z, ground, obstacle in (
                    ("baseline", base_z, baseline, old),
                    ("candidate", estimate, candidate, new),
                )
            }
            if "labels" in sample:
                packed_path = Path(sample["labels"])
                require(digest(packed_path) == sample["labels_sha256"], "GT hash")
                hashes[str(packed_path)] = sample["labels_sha256"]
                packed = np.fromfile(packed_path, dtype="<u4")
                truth = learning_labels(packed)
                require(len(truth) == len(points), "label order/count")
                raw = np.fromfile(raw_path, dtype="<f4").reshape(-1, 4)[:, :3]
                require(np.array_equal(points, raw), "sensor XYZ/order mismatch")
                roi = (
                    (points[:, 0] >= config.min_x)
                    & (points[:, 0] < config.max_x)
                    & (points[:, 1] >= config.min_y)
                    & (points[:, 1] < config.max_y)
                )
                row["proxy_by_distance"] = proxy(
                    points, truth, {"baseline": baseline, "candidate": candidate}, roi
                )
                radius = np.hypot(points[:, 0], points[:, 1])
                row["range_causal_evidence"] = {}
                for lo, hi in ((15, 30), (30, float("inf"))):
                    missed = np.flatnonzero(
                        roi
                        & np.isin(truth, GROUND)
                        & baseline
                        & ~candidate
                        & (radius >= lo)
                        & (radius < hi)
                    )
                    row["range_causal_evidence"][f"{lo}-{hi}m"] = {
                        "baseline_ground_proxy_now_missed": len(missed),
                        "supported_but_outside_tolerance": int(
                            np.isfinite(estimate[missed]).sum()
                        ),
                        "first_unsupported_examples": [
                            radial_witness(points, i, trace)
                            for i in missed[~np.isfinite(estimate[missed])][:5]
                        ],
                    }
                thing = (truth > 0) & (truth <= 8) & roi
                row["retention"] = {
                    name: instance_retention(
                        points, packed >> 16, thing, old, mask, 0.1
                    )
                    for name, mask in (("raw", new), ("isolated_0.1m", filtered))
                }
                if (
                    row["continuity"]["candidate"]["ego_connected_cells"]
                    < row["continuity"]["baseline"]["ego_connected_cells"]
                ):
                    causal = counterfactual(
                        points,
                        base_z,
                        baseline,
                        estimate,
                        candidate,
                        old,
                        new,
                        truth,
                        trace,
                    )
                    path = output / (
                        "ground-causal-" + run["name"] + "-" + sample["file"] + ".json"
                    )
                    write(path, causal)
                    row["causal_artifact"] = str(path)
                    row["first_boundary_counts"] = causal["first_boundary_counts"]
            rows.append(row)
            print(f"cached {run['name']}/{sample['file']}", flush=True)
    for path, expected in hashes.items():
        require(digest(Path(path)) == expected, "source changed during diagnostics")
    return {"frames": rows, "source_sha256": hashes}


def raw(args, output):
    inventory = json.loads(args.inventory.read_text())
    rows, hashes = [], {str(args.inventory): digest(args.inventory)}
    # KITTI odometry calibration is Velodyne-to-camera, not sensor-to-ground.
    # No verified mounting height exists in the supplied source. Bracket the
    # already documented diagnostic assumption by its frozen seed band.
    heights = (-1.43, -1.73, -2.03)
    frozen = {
        "selection": inventory["selection_rule_frozen_before_predictions"],
        "frame": "native Velodyne sensor frame; origin NOT vehicle base/ego",
        "height_status": "diagnostic hypotheses, not calibrated ground heights",
        "expected_ground_z_m": list(heights),
        "height_basis": (
            "Existing seq08 diagnostic assumption -1.73m +/- existing 0.3m "
            "seed band. No official sensor-ground height established. This "
            "tests assumption sensitivity only, never fitted to GT and NOT "
            "a seq08 vehicle contract transferred to other sequences."
        ),
        "options": asdict(RadialOptions()),
        "roi_sensor_xy_m": [-60, 60, -40, 40],
        "ground_seed_band_m": 0.3,
        "ground_max_slope": 0.3,
        "ground_max_residual_m": 0.08,
        "ground_tolerance_m": 0.1,
        "not_holdout": True,
        "not_production": True,
        "GT_enters_algorithm": False,
    }
    write(output / "ground-raw-frozen-config.json", frozen)
    for sample in inventory["selection_details"]:
        sequence, ordinal = sample["sequence"], sample["ordinal"]
        raw_path, label_path = Path(sample["raw_scan"]), Path(sample["labels"])
        require(digest(raw_path) == sample["raw_sha256"], "raw inventory changed")
        require(digest(label_path) == sample["labels_sha256"], "GT inventory changed")
        hashes[str(raw_path)] = sample["raw_sha256"]
        hashes[str(label_path)] = sample["labels_sha256"]
        points = np.fromfile(raw_path, dtype="<f4").reshape(-1, 4)[:, :3]
        predictions, estimates = {}, {}
        for height in heights:
            config = SimpleNamespace(
                min_x=-60,
                max_x=60,
                min_y=-40,
                max_y=40,
                expected_ground_z=height,
                ground_seed_band=0.3,
                ground_max_slope=0.3,
                ground_max_residual=0.08,
                ground_tolerance=0.1,
            )
            estimate = radial_ground(points, config)
            name = str(height)
            estimates[name] = estimate
            predictions[name] = np.isfinite(estimate) & (
                np.abs(points[:, 2] - estimate) <= config.ground_tolerance
            )
        # Labels are first read AFTER every raw algorithm execution.
        truth = learning_labels(np.fromfile(label_path, dtype="<u4"))
        require(len(truth) == len(points), "raw/label count mismatch")
        roi = (
            (points[:, 0] >= -60)
            & (points[:, 0] < 60)
            & (points[:, 1] >= -40)
            & (points[:, 1] < 40)
        )
        row = {
            "sequence": sequence,
            "ordinal": ordinal,
            "points": len(points),
            "proxy_by_distance": proxy(points, truth, predictions, roi),
            "supported_points": {
                name: int(np.isfinite(z).sum()) for name, z in estimates.items()
            },
            "ground_points": {
                name: int(mask.sum()) for name, mask in predictions.items()
            },
        }
        path = output / f"ground-raw-{sequence}-{ordinal:06d}.npz"
        np.savez_compressed(
            path,
            point_id=np.arange(len(points), dtype=np.uint32),
            **{f"z_{name}": value for name, value in estimates.items()},
            **{f"ground_{name}": value for name, value in predictions.items()},
        )
        row["algorithm_artifact"] = str(path)
        hashes[str(path)] = digest(path)
        rows.append(row)
        print(f"raw {sequence}/{ordinal}", flush=True)
    return {"frozen": frozen, "frames": rows, "source_sha256": hashes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("previous", "schema", "inventory", "output"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {
        "diagnostic_only": True,
        "instance_denominator_unit": "instance-frame, not unique tracked objects",
        "ground_algorithm_fix": False,
        "changes": (
            "all-instance accounting, conservative promotion gate, optional trace"
        ),
        "threshold_tuning": False,
        "cached_36": cached(args, args.output),
        "cross_sequence_raw_20": raw(args, args.output),
        "production_mutated": False,
        "FREE_emitted": False,
        "isolated_filter_recommended": False,
    }
    write(args.output / "ground-report.json", report)
    cases = []

    def case(name, expected, actual, status, evidence, boundary=None):
        cases.append(
            {
                "case": name,
                "expected": expected,
                "actual": actual,
                "status": status,
                "evidence": evidence,
                "first_wrong_boundary": boundary,
            }
        )

    cached_rows = report["cached_36"]["frames"]
    case(
        "frozen-36-estimator-preserved",
        "Same 36 fixed raw inputs; trace and accounting must not change predictions",
        {"frames": len(cached_rows), "matching_previous_estimates": 36},
        "PASS",
        "ground-report.json#/cached_36",
    )
    for row in cached_rows:
        old = row["continuity"]["baseline"]["ego_connected_area_m2"]
        new = row["continuity"]["candidate"]["ego_connected_area_m2"]
        if "causal_artifact" in row:
            case(
                row["case"] + "/" + row["file"],
                "Candidate observed connected area must not regress per frame",
                {
                    "baseline_area_m2": old,
                    "candidate_area_m2": new,
                    "cell_boundaries": row["first_boundary_counts"],
                },
                "FAIL",
                row["causal_artifact"],
                "Radial support loss -> unsupported point retained as obstacle -> "
                "observed cell blocked -> topology/nearest-ring selection",
            )
        elif "retention" not in row:
            case(
                "mechanism-no-GT/" + row["case"] + "/" + row["file"],
                "Execute ground mechanism only; no semantic ground GT accuracy",
                {"baseline_area_m2": old, "candidate_area_m2": new, "GT": False},
                "DIAGNOSTIC",
                "ground-report.json#/cached_36",
            )
    for variant in ("raw", "isolated_0.1m"):
        totals = Counter()
        replaced = 0
        for row in cached_rows:
            retention = row.get("retention", {}).get(variant, {})
            totals.update(
                {
                    key: value
                    for key, value in retention.items()
                    if isinstance(value, int)
                }
            )
            replaced += len(
                retention.get("baseline_support_completely_replaced_instance_ids", [])
            )
        case(
            "all-instance-complete-retention/" + variant,
            "Zero baseline-supported GT instances completely erased; include <3 voxels",
            dict(totals),
            (
                "PASS"
                if totals["all_completely_lost_instances"] == 0 and not replaced
                else "FAIL"
            ),
            "ground-report.json#/cached_36/frames/*/retention",
            (
                "raw candidate obstacle mask"
                if variant == "raw"
                else "isolated 0.1m component filtering"
            ),
        )
        case(
            "same-original-support-retention/" + variant,
            "Do not compensate original support loss with new support "
            "or move denominator",
            {
                "partially_lost_instances": totals["partially_lost_instances"],
                "baseline_voxels": totals["baseline_obstacle_voxels"],
                "lost_baseline_voxels": totals["baseline_obstacle_voxels"]
                - totals["retained_baseline_obstacle_voxels"],
                "fully_replaced_original_support_instances": replaced,
            },
            "FAIL" if totals["partially_lost_instances"] else "PASS",
            "ground-report.json#/cached_36/frames/*/retention",
            "candidate point-level ground/support/height eligibility before continuity",
        )
    for lo, hi in BANDS:
        totals = {name: Counter() for name in ("baseline", "candidate")}
        for row in cached_rows:
            for name, metric in (
                row.get("proxy_by_distance", {}).get(f"{lo}-{hi}m", {}).items()
            ):
                totals[name].update(
                    {key: metric[key] for key in ("tp", "fp", "fn", "tn")}
                )
        actual = {name: rates(value) for name, value in totals.items()}
        case(
            f"distance-coverage/{lo}-{hi}m",
            "Ground semantic proxy recall must not regress in this frozen radial band",
            actual,
            (
                "PASS"
                if actual["candidate"]["recall"] >= actual["baseline"]["recall"]
                else "FAIL"
            ),
            "ground-report.json#/cached_36/frames/*/range_causal_evidence",
            "same-sector observed gaps/minimum bins/anchor continuity; "
            "8m only limits START",
        )
    case(
        "sensor-to-ground-height-qualification",
        "Official sensor frame and physical sensor-ground height qualification",
        {
            "sensor_XYZ_order": "official SemanticKITTI Velodyne raw float32 XYZ",
            "mounting_height_verified": False,
            "vehicle_base_transform_verified": False,
            "hypotheses": report["cross_sequence_raw_20"]["frozen"],
        },
        "BLOCKED",
        "ground-raw-frozen-config.json; architecture-report.json official sources",
        "No physical sensor-ground mounting/base-frame calibration contract",
    )
    for row in report["cross_sequence_raw_20"]["frames"]:
        case(
            f"raw-ground-only/{row['sequence']}/{row['ordinal']:06d}",
            "Frozen raw-only algorithm before GT; fixed three height "
            "sensitivities; not holdout",
            row,
            "DIAGNOSTIC",
            row["algorithm_artifact"],
        )
    case(
        "no-FREE-production-boundary",
        "No FREE inferred, no real obstacle deletion to improve connectivity, "
        "no production mutation",
        {
            "FREE": False,
            "production_mutated": False,
            "filter_recommended": False,
            "single_cell_removal": "diagnostic counterfactual only",
        },
        "PASS",
        "ground-report.json; ground-causal-*.json",
    )
    write(
        args.output / "ground-case-matrix.json",
        {
            "assertion_freeze": (
                "ground-frozen-case-contracts.json frozen before final replay; "
                "prior fixed inputs/options and task-derived criteria, no tuning"
            ),
            "cases": cases,
            "status_counts": dict(Counter(row["status"] for row in cases)),
        },
    )


if __name__ == "__main__":
    main()
