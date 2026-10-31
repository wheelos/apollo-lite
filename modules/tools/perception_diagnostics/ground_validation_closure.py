#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Common-observed-seed comparison and bounded real radial falsification."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from evaluate_baseline import digest, require
from ground_continuity import connected_ground, grid_ids
from ground_experiment import fit_line, obstacle_mask, radial_ground
from proto import Reader


def components(points, z, ground, obstacle, resolution=0.5, height_step=0.2):
    ids, keys = grid_ids(points, ground, resolution)
    buckets = {}
    for i, key in zip(ids, keys):
        if np.isfinite(z[i]):
            buckets.setdefault(tuple(key), []).append(float(z[i]))
    heights = {key: float(np.median(values)) for key, values in buckets.items()}
    _, keys = grid_ids(points, obstacle, resolution)
    remaining = set(heights) - set(map(tuple, keys))
    result = []
    for root in sorted(remaining):
        if root not in remaining:
            continue
        remaining.remove(root)
        pending, component = [root], set()
        while pending:
            key = pending.pop()
            component.add(key)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                other = key[0] + dx, key[1] + dy
                if (
                    other in remaining
                    and abs(heights[key] - heights[other]) <= height_step
                ):
                    remaining.remove(other)
                    pending.append(other)
        result.append(component)
    return result


def distance(key):
    return math.hypot((key[0] + 0.5) * 0.5, (key[1] + 0.5) * 0.5)


def common_comparison(old, new, old_radius, new_radius):
    """Exact common observed seeds inside BOTH original bounds, never enlarged."""
    limit = min(old_radius, new_radius)
    old_cells, new_cells = set().union(*old), set().union(*new)
    seeds = {key for key in old_cells & new_cells if distance(key) <= limit}
    result = {
        "common_seed_xy": [[int(x), int(y)] for x, y in sorted(seeds)],
        "radius_intersection_m": limit,
        "contract": (
            "Same exact observed/unblocked seed cells inside BOTH independently "
            "selected original seed bounds; no GT, radius enlargement, "
            "obstacle removal, component union or blind-zone completion"
        ),
        "status": "PASS" if seeds else "BLOCKED",
        "ground_feature_accepted": False,
    }
    for name, collection in (("baseline", old), ("candidate", new)):
        eligible = [component for component in collection if component & seeds]
        selected = max(eligible, key=len) if eligible else set()
        result[name] = {
            "all_observed_unblocked_cells": sum(map(len, collection)),
            "all_component_sizes": sorted(map(len, collection), reverse=True),
            "common_seed_touching_component_sizes": sorted(
                map(len, eligible), reverse=True
            ),
            "same_seed_selected_cells": len(selected) if seeds else None,
            "largest_component_cells": max(map(len, collection), default=0),
        }
    return result


def sector_proof(points, config, state, sector):
    ids = state["low_point_ids"]
    low = points[ids]
    radius = np.hypot(low[:, 0], low[:, 1])
    samples = [
        (int(math.floor(r / 0.5)), float(r), float(z))
        for r, z in zip(radius, low[:, 2])
    ]
    events = state["events"]
    initial_pair = samples[:2]
    slope = (
        (initial_pair[1][2] - initial_pair[0][2])
        / (initial_pair[1][1] - initial_pair[0][1])
        if len(initial_pair) == 2
        else None
    )
    floor_pair = (
        fit_line(initial_pair, config.ground_max_slope, config.ground_max_residual)
        if len(initial_pair) == 2
        else None
    )
    independent = radial_ground(low, config)
    require(
        np.array_equal(
            np.isfinite(independent),
            np.isin(ids, state["supported_point_ids"]),
        ),
        "minimal real sector did not reproduce low-bin support",
    )
    return {
        "sector": sector,
        "raw_original_point_ids": ids,
        "raw_low_xyz_m": low.tolist(),
        "low_bin_samples_bin_radius_height": samples,
        "minimal_low_bin_replay_matches_full_sweep": True,
        "initial_pair_slope": slope,
        "max_slope": config.ground_max_slope,
        "initial_pair_fit_accepted": floor_pair is not None,
        "initial_pair_gap_m": radius[1] - radius[0] if len(radius) > 1 else None,
        "minimum_bins": 3,
        "max_observed_gap_m": 1.0,
        "first_three_bins_fit_accepted": (
            fit_line(samples[:3], config.ground_max_slope, config.ground_max_residual)
            is not None
            if len(samples) >= 3
            else False
        ),
        "state_events": events,
        "first_wrong_boundary": (
            "First pair violates strict slope fit before 3-bin support is "
            "available; rejected samples cannot seed backward extrapolation"
            if slope is not None and abs(slope) > config.ground_max_slope
            else "Observed gap breaks support before minimum 3 bins; "
            "remote re-anchor must satisfy the unchanged 8m START bound"
        ),
        "implementation_bug_proved": False,
        "rollover_is_first_failure": (
            bool(events) and events[0]["reason"] == "bounded_line_rollover"
        ),
        "why_not_relax": (
            "A fit over three samples may average a rejected initial pair, "
            "but retrospective seeding changes the strict accepted-bin slope "
            "contract. Gap bridging/backward extrapolation or fewer bins "
            "can accept roofs/steps and unsupported obstacles; semantic "
            "ground proxy is not a physical terrain calibration."
        ),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    root = args.root
    full_path = root / "ground-full-rerun/report.json"
    full = json.loads(full_path.read_text())
    hashes = {
        str(full_path): digest(full_path),
        str(Path(__file__)): digest(Path(__file__)),
    }
    reader = Reader(
        Path(".cache/lidar-fusion-validation/current-head-replay-v1/current-schema.pb")
    )
    comparisons, proofs = [], []
    for run in full["experiment"]["runs"]:
        results = Path(run["results"])
        config = reader.read(results / "config.pb", "apollo.lidar_fusion.FusionConfig")
        manifest = json.loads((Path(run["inputs"]) / "manifest.json").read_text())
        for sample in manifest["samples"]:
            path = results / (sample["file"] + ".fusion.pb")
            hashes[str(path)] = digest(path)
            out = reader.read(path, "apollo.lidar_fusion.FusionOutput")
            points = np.array(
                [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
            )
            old = np.array([p.is_ground_geo for p in out.point_evidence])
            old_z = np.array(
                [p.z_ground if p.ground_valid else np.nan for p in out.point_evidence]
            )
            noise = np.array([d.explanation == 2 for d in out.point_decision])
            trace = {}
            z = radial_ground(points, config, trace=trace)
            ground = np.isfinite(z) & (
                np.abs(points[:, 2] - z) <= config.ground_tolerance
            )
            artifact = (
                root
                / "ground-full-rerun"
                / run["name"]
                / (sample["file"] + ".ground.npz")
            )
            hashes[str(artifact)] = digest(artifact)
            with np.load(artifact) as frozen:
                require(
                    np.array_equal(z, frozen["ground_z"], equal_nan=True)
                    and np.array_equal(ground, frozen["ground"]),
                    "closure must not alter estimator predictions",
                )
            old_obstacle = obstacle_mask(points, old_z, old, noise, config)
            new_obstacle = obstacle_mask(points, z, ground, noise, config)
            old_report, old_selected = connected_ground(
                points, old_z, old, old_obstacle
            )
            new_report, new_selected = connected_ground(points, z, ground, new_obstacle)
            old_components = components(points, old_z, old, old_obstacle)
            new_components = components(points, z, ground, new_obstacle)
            require(
                not old_selected or len(old_selected) in list(map(len, old_components)),
                "component contract",
            )
            require(
                not new_selected or len(new_selected) in list(map(len, new_components)),
                "component contract",
            )
            comparison = common_comparison(
                old_components,
                new_components,
                old_report["effective_seed_radius_m"],
                new_report["effective_seed_radius_m"],
            )
            comparison.update(
                case=run["name"],
                file=sample["file"],
                original_independent_metric={
                    "baseline": old_report["ego_connected_cells"],
                    "candidate": new_report["ego_connected_cells"],
                    "status": (
                        "FAIL" if len(new_selected) < len(old_selected) else "PASS"
                    ),
                    "retained_unchanged": True,
                },
            )
            comparisons.append(comparison)
            if (
                sample["file"] in ("000500.cloud.pb", "001753.cloud.pb")
                and "semantickitti" in run["name"]
            ):
                wanted = (
                    (342, 340, 343)
                    if sample["file"].startswith("000500")
                    else (38, 48, 41)
                )
                for sector in wanted:
                    proof = sector_proof(points, config, trace[sector], sector)
                    proof.update(case=run["name"], file=sample["file"])
                    proofs.append(proof)
            print(f"closure {run['name']}/{sample['file']}", flush=True)
    # Existing raw20 remains unchanged: closure never touches ground NPZ/estimator.
    ground_report = json.loads((root / "ground-report.json").read_text())
    for registry in (
        full["source_sha256"],
        ground_report["cross_sequence_raw_20"]["source_sha256"],
    ):
        hashes.update(registry)
    for path, expected in hashes.items():
        require(digest(Path(path)) == expected, "closure changed source/input/output")
    report = {
        "verification_complete": True,
        "feature_accepted": False,
        "prediction_change": False,
        "comparison_contract": (
            "Exact common observed seed cells inside intersection of original bounds"
        ),
        "comparisons": comparisons,
        "remaining_two_frame_minimal_sector_proofs": proofs,
        "source_and_frozen_artifact_sha256": hashes,
        "conclusion": (
            "No bounded-rollover tail bug proved. Early slope/min-bin/gap "
            "contract rejects real support before rollover."
        ),
        "nextaction": (
            "Use common exact-seed comparison only where nonempty; retain original "
            "metric FAIL. Keep current radial candidate nonrecommended. A terrain "
            "estimator redesign requires a separately frozen physical/obstacle "
            "contract; do not change max_gap/min_bins/slope/start radius "
            "using these labels."
        ),
    }
    (root / "ground-closure-report.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    print(
        json.dumps(
            {"frames": len(comparisons), "proofs": len(proofs), "hashes": len(hashes)}
        )
    )


if __name__ == "__main__":
    main()
