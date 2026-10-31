#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Read-only anchored radial ground experiment; never publishes FREE or fusion."""

import argparse
import json
import math
from collections import Counter
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from evaluate_baseline import audit_frame, digest, require
from ground_continuity import (
    ContinuityOptions,
    connected_ground,
    grid_ids,
    instance_retention,
    sparse_obstacles,
)
from nuscenes import rates
from proto import Reader, require_full_evidence
from semantickitti import GROUND, NAMES, binary_counts, learning_labels


@dataclass(frozen=True)
class RadialOptions:
    sectors: int = 360
    bin_size: float = 0.5
    max_gap: float = 1.0
    max_start_radius: float = 8.0
    max_line_length: float = 4.0
    min_bins: int = 3

    def validate(self):
        require(
            isinstance(self.sectors, int)
            and 1 <= self.sectors <= 4096
            and isinstance(self.min_bins, int)
            and self.min_bins >= 3,
            "invalid radial bin counts",
        )
        values = (
            self.bin_size,
            self.max_gap,
            self.max_start_radius,
            self.max_line_length,
        )
        require(all(math.isfinite(v) and v > 0 for v in values), "invalid distances")
        require(self.max_gap >= self.bin_size, "gap smaller than radial bin")
        require(
            self.max_line_length >= (self.min_bins - 1) * self.bin_size,
            "line span cannot support minimum bins",
        )


def fit_line(samples, max_slope, max_residual):
    values = np.asarray(samples)
    radius, height = values[:, 1], values[:, 2]
    center = radius.mean()
    delta = radius - center
    denominator = delta @ delta
    if denominator <= 1e-12:
        return None
    slope = float(delta @ (height - height.mean()) / denominator)
    intercept = float(height.mean() - slope * center)
    residual = float(np.max(np.abs(slope * radius + intercept - height)))
    if abs(slope) > max_slope or residual > max_residual:
        return None
    return slope, intercept


def radial_ground(points, config, options=RadialOptions(), *, trace=None):
    """Return original-order z estimates; NaN means unsupported, not free.

    Points are in the configured base frame (metres). Only same-sector,
    observed low bins support a line. No GT, semantics, neighboring sectors,
    poses, history or baseline ground predictions enter this estimator.
    """
    options.validate()
    points = np.asarray(points, dtype=float)
    require(points.ndim == 2 and points.shape[1] == 3, "expected Nx3 base points")
    require(
        all(
            math.isfinite(v) and v > 0
            for v in (config.ground_seed_band, config.ground_max_residual)
        )
        and math.isfinite(config.expected_ground_z)
        and math.isfinite(config.ground_max_slope)
        and config.ground_max_slope >= 0,
        "invalid ground contract",
    )
    estimate = np.full(len(points), np.nan)
    valid = (
        np.isfinite(points).all(axis=1)
        & (np.abs(points) < 10000).all(axis=1)
        & (points[:, 0] >= config.min_x)
        & (points[:, 0] < config.max_x)
        & (points[:, 1] >= config.min_y)
        & (points[:, 1] < config.max_y)
    )
    ids = np.flatnonzero(valid)
    if not len(ids):
        return estimate
    xyz = points[ids]
    radius = np.hypot(xyz[:, 0], xyz[:, 1])
    sector = np.floor(
        np.mod(np.arctan2(xyz[:, 1], xyz[:, 0]) + np.pi, 2 * np.pi)
        * options.sectors
        / (2 * np.pi)
    ).astype(int)
    bins = np.floor(radius / options.bin_size).astype(int)
    order = np.lexsort((radius, xyz[:, 2], bins, sector))
    sector, bins, radius = sector[order], bins[order], radius[order]
    ids, xyz = ids[order], xyz[order]
    starts = np.r_[0, np.flatnonzero(np.diff(sector)) + 1, len(ids)]
    for first, last in zip(starts[:-1], starts[1:]):
        local_bins = bins[first:last]
        low_ids = np.r_[0, np.flatnonzero(np.diff(local_bins)) + 1] + first
        samples = []
        previous = None
        events = []

        def record(reason, sample=None):
            if trace is not None:
                events.append(
                    {
                        "reason": reason,
                        "sample": (
                            [int(sample[0]), float(sample[1]), float(sample[2])]
                            if sample is not None
                            else None
                        ),
                        "accepted_bins": [int(s[0]) for s in samples],
                    }
                )

        def finish():
            nonlocal previous
            if len(samples) < options.min_bins:
                record("finish_insufficient_bins")
                return
            line = fit_line(
                samples, config.ground_max_slope, config.ground_max_residual
            )
            if line is None:
                record("finish_fit_rejected")
                return
            select = (local_bins >= samples[0][0]) & (local_bins <= samples[-1][0])
            selected = np.flatnonzero(select) + first
            estimate[ids[selected]] = line[0] * radius[selected] + line[1]
            previous = (samples[-1], line)
            record("line_published")

        for i in low_ids:
            sample = (bins[i], radius[i], xyz[i, 2])
            if samples and (
                sample[1] - samples[-1][1] > options.max_gap
                or sample[1] - samples[0][1] > options.max_line_length
            ):
                record(
                    (
                        "observed_gap_reset"
                        if sample[1] - samples[-1][1] > options.max_gap
                        else "bounded_line_rollover"
                    ),
                    sample,
                )
                finish()
                samples = (
                    samples[-2:]
                    if len(samples) >= options.min_bins
                    and sample[1] - samples[-1][1] <= options.max_gap
                    else []
                )
            if not samples:
                anchored = (
                    sample[1] <= options.max_start_radius
                    and abs(sample[2] - config.expected_ground_z)
                    <= config.ground_seed_band
                )
                if previous is not None:
                    endpoint, line = previous
                    if (
                        0 < sample[1] - endpoint[1] <= options.max_gap
                        and abs(sample[2] - (line[0] * sample[1] + line[1]))
                        <= config.ground_max_residual
                    ):
                        samples = [endpoint]
                        anchored = True
                if anchored:
                    samples.append(sample)
                else:
                    record("initial_or_continuation_anchor_rejected", sample)
                continue
            trial = samples + [sample]
            if (
                fit_line(trial, config.ground_max_slope, config.ground_max_residual)
                is not None
            ):
                samples = trial
            else:
                record("trial_slope_or_residual_rejected", sample)
                finish()
                samples = []
                # Retry only from the last accepted endpoint, never a roof.
                if previous is not None:
                    endpoint, line = previous
                    if (
                        0 < sample[1] - endpoint[1] <= options.max_gap
                        and abs(sample[2] - (line[0] * sample[1] + line[1]))
                        <= config.ground_max_residual
                    ):
                        samples = [endpoint, sample]
        finish()
        if trace is not None:
            trace[int(sector[first])] = {
                "low_point_ids": [int(ids[i]) for i in low_ids],
                "events": events,
                "supported_point_ids": [
                    int(i) for i in ids[first:last] if np.isfinite(estimate[i])
                ],
            }
    return estimate


def obstacle_mask(points, ground_z, ground, noise, config):
    finite = np.isfinite(points).all(axis=1)
    roi = (
        finite
        & (points[:, 0] >= config.min_x)
        & (points[:, 0] < config.max_x)
        & (points[:, 1] >= config.min_y)
        & (points[:, 1] < config.max_y)
    )
    height = points[:, 2] - ground_z
    eligible = ~np.isfinite(ground_z) | (
        (height >= config.obstacle_min_height) & (height <= config.clearance_height)
    )
    return roi & ~ground & ~noise & eligible


def measured_cells(points, ground_z, ground, noise, config):
    ids = np.flatnonzero(obstacle_mask(points, ground_z, ground, noise, config))
    cells = np.floor(
        (points[ids, :2] - [config.min_x, config.min_y]) / config.resolution
    ).astype(int)
    return set(map(tuple, cells))


def compare_frame(
    out,
    config,
    options,
    truth=None,
    identity=None,
    continuity_options=ContinuityOptions(),
):
    require_full_evidence(out)
    audit_frame(out, config)
    points = np.array(
        [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
    )
    finite = np.array([p.finite for p in out.point_evidence])
    points[~finite] = np.nan
    estimate = radial_ground(points, config, options)
    candidate = np.isfinite(estimate) & (
        np.abs(points[:, 2] - estimate) <= config.ground_tolerance
    )
    baseline = np.array([p.is_ground_geo for p in out.point_evidence])
    base_z = np.array(
        [p.z_ground if p.ground_valid else np.nan for p in out.point_evidence]
    )
    noise = np.array([d.explanation == 2 for d in out.point_decision])
    measured = measured_cells(points, base_z, baseline, noise, config)
    expected = {(c.x_index, c.y_index) for c in out.space_map.cell if c.occupancy_obs}
    require(measured == expected, "baseline measured occupancy mismatch")
    proposed = measured_cells(points, estimate, candidate, noise, config)
    old_obstacle = obstacle_mask(points, base_z, baseline, noise, config)
    new_obstacle = obstacle_mask(points, estimate, candidate, noise, config)
    old_filtered, old_filter_stats = sparse_obstacles(
        points, old_obstacle, continuity_options
    )
    new_filtered, new_filter_stats = sparse_obstacles(
        points, new_obstacle, continuity_options
    )
    continuity, regions = {}, {}
    for name, z, ground, obstacle in (
        ("baseline", base_z, baseline, old_obstacle),
        ("baseline_filtered", base_z, baseline, old_filtered),
        ("candidate", estimate, candidate, new_obstacle),
        ("candidate_filtered", estimate, candidate, new_filtered),
    ):
        continuity[name], cells = connected_ground(
            points, z, ground, obstacle, continuity_options
        )
        regions[name + "_connected_xy"] = np.asarray(
            sorted(cells), dtype=np.int64
        ).reshape(-1, 2)
    retained = {
        (c.x_index, c.y_index)
        for c in out.space_map.cell
        if c.occupancy_det or c.occupancy_unknown
    }
    removed = measured - proposed
    report = {
        "points": len(points),
        "baseline_ground_points": int(baseline.sum()),
        "candidate_ground_points": int(candidate.sum()),
        "candidate_supported_points": int(np.isfinite(estimate).sum()),
        "measured_cells_baseline": len(measured),
        "measured_cells_candidate": len(proposed),
        "measured_cells_removed": len(removed),
        "measured_cells_added": len(proposed - measured),
        "effective_occupied_cells_removed": len(
            (measured | retained) - (proposed | retained)
        ),
        "inferred_and_unknown_layers": "retained unchanged in hypothetical union",
        "production_mutated": False,
        "free_cells_emitted": 0,
        "continuity": continuity,
        "isolated_filter": {
            "baseline": old_filter_stats,
            "candidate": new_filter_stats,
        },
    }
    roi = (
        finite
        & (points[:, 0] >= config.min_x)
        & (points[:, 0] < config.max_x)
        & (points[:, 1] >= config.min_y)
        & (points[:, 1] < config.max_y)
    )
    if truth is not None:
        require(len(truth) == len(points), "GT point count mismatch")
        actual = np.isin(truth, GROUND)
        thing = (truth > 0) & (truth <= 8)
        domains = {
            "roi": roi & (truth != 0),
            "front": roi
            & (truth != 0)
            & (points[:, 0] >= 0)
            & (points[:, 0] < 60)
            & (points[:, 1] >= -39.68)
            & (points[:, 1] < 39.68),
        }
        report["ground_proxy"] = {
            name: {
                "baseline": rates(binary_counts(actual, baseline, mask)),
                "candidate": rates(binary_counts(actual, candidate, mask)),
            }
            for name, mask in domains.items()
        }
        report["baseline_thing_ground_errors"] = int((thing & baseline & roi).sum())
        report["candidate_thing_ground_errors"] = int((thing & candidate & roi).sum())
        report["new_thing_ground_errors"] = int(
            (thing & candidate & ~baseline & roi).sum()
        )
        bad_ids = np.flatnonzero(roi & (truth != 0) & ~actual & candidate & ~baseline)
        report["new_non_ground_errors"] = len(bad_ids)
        report["new_non_ground_error_examples"] = bad_ids[:20].tolist()
        report["new_non_ground_errors_by_class"] = {
            NAMES[label]: int(count)
            for label, count in zip(*np.unique(truth[bad_ids], return_counts=True))
        }
        excluded = (
            obstacle_mask(points, base_z, baseline, noise, config)
            & ~obstacle_mask(points, estimate, candidate, noise, config)
            & roi
            & (truth != 0)
            & ~actual
        )
        report["new_gt_non_ground_excluded_from_measured"] = int(excluded.sum())
        report["isolated_filter"]["removed_gt_thing_points"] = int(
            (thing & new_obstacle & ~new_filtered).sum()
        )
        if identity is not None:
            report["obstacle_instances_raw"] = instance_retention(
                points,
                identity,
                thing & roi,
                old_obstacle,
                new_obstacle,
                continuity_options.voxel_size,
            )
            report["obstacle_instances"] = instance_retention(
                points,
                identity,
                thing & roi,
                old_obstacle,
                new_filtered,
                continuity_options.voxel_size,
            )
        _, gt_keys = grid_ids(points, actual & roi, continuity_options.resolution)
        gt_cells = set(map(tuple, gt_keys))
        for variant in continuity:
            selected = set(map(tuple, regions[variant + "_connected_xy"]))
            continuity[variant]["semantic_ground_cell_overlap"] = len(
                selected & gt_cells
            )
            continuity[variant]["semantic_ground_cell_recall"] = (
                len(selected & gt_cells) / len(gt_cells) if gt_cells else None
            )
    regions["candidate_filtered_obstacle"] = new_filtered
    regions["candidate_raw_obstacle"] = new_obstacle
    regions["baseline_filtered_obstacle"] = old_filtered
    return report, estimate, candidate, regions


def promotion_checks(frames, totals):
    labeled = [row for row in frames if row["labeled"]]
    instance_evidence = bool(labeled) and all(
        "all_completely_lost_instances" in row.get("obstacle_instances_raw", {})
        for row in labeled
    )
    safety = instance_evidence and all(
        row["obstacle_instances_raw"]["all_completely_lost_instances"] == 0
        and not row["obstacle_instances_raw"].get(
            "baseline_support_completely_replaced_instance_ids", []
        )
        for row in labeled
    )
    seed_evidence = bool(labeled) and all(
        row["continuity"]["candidate"]["seed_observed"]
        and row["continuity"]["baseline"]["seed_observed"]
        for row in labeled
    )
    continuity_gain = seed_evidence and (
        sum(row["continuity"]["candidate"]["ego_connected_cells"] for row in labeled)
        > sum(row["continuity"]["baseline"]["ego_connected_cells"] for row in labeled)
    )
    recall_gain = bool(labeled) and (
        totals["roi"]["candidate"]["tp"] > totals["roi"]["baseline"]["tp"]
    )
    filtered_safety = bool(labeled) and all(
        row.get("obstacle_instances", {}).get("all_completely_lost_instances") == 0
        and not row["obstacle_instances"].get(
            "baseline_support_completely_replaced_instance_ids", []
        )
        for row in labeled
    )
    regressions = sum(
        row["continuity"]["candidate"]["ego_connected_cells"]
        < row["continuity"]["baseline"]["ego_connected_cells"]
        for row in labeled
    )
    return {
        "labeled_frames": len(labeled),
        "primary_variant": "candidate without isolated-point filtering",
        "point_errors_are_diagnostic_not_zero_tolerance_gate": True,
        "no_supported_gt_instance_completely_lost": safety,
        "instance_safety_denominator": "all baseline-supported GT instances",
        "seed_observation_evidence_complete": seed_evidence,
        "ego_connected_ground_increased": continuity_gain,
        "frames_with_connected_area_regression": regressions,
        "roi_ground_recall_increased": recall_gain,
        "eligible_for_native_integration_experiment": (
            safety and continuity_gain and regressions == 0
        ),
        "filtered_variant_preserves_supported_instances": filtered_safety,
        "isolated_point_filter_qualified": False,
        "production_qualified": False,
    }


def export(cases_path, schema, output):
    experiment = json.loads(cases_path.read_text())
    require(bool(experiment["runs"]), "no experiment runs")
    options = RadialOptions(**experiment.get("options", {}))
    options.validate()
    continuity_options = ContinuityOptions(**experiment.get("continuity", {}))
    continuity_options.validate()
    reader = Reader(schema)
    frames, hashes = [], {}
    implementation = [
        Path(__file__).with_name(file)
        for file in (
            "ground_experiment.py",
            "ground_continuity.py",
            "evaluate_baseline.py",
            "semantickitti.py",
            "nuscenes.py",
            "proto.py",
        )
    ]
    for path in [cases_path, schema] + implementation:
        hashes[str(path)] = digest(path)
    output.mkdir(parents=True, exist_ok=False)
    seen = set()
    for run in experiment["runs"]:
        name = run["name"]
        require(
            name
            and name not in (".", "..")
            and Path(name).name == name
            and name not in seen,
            "case name",
        )
        seen.add(name)
        inputs, results = Path(run["inputs"]), Path(run["results"])
        manifest_path = inputs / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        require(
            bool(manifest["samples"])
            and len({s["file"] for s in manifest["samples"]})
            == len(manifest["samples"]),
            "empty or duplicate samples",
        )
        config_path = results / "config.pb"
        config = reader.read(config_path, "apollo.lidar_fusion.FusionConfig")
        for path in (manifest_path, config_path):
            hashes[str(path)] = digest(path)
        case_dir = output / name
        case_dir.mkdir()
        for sample in manifest["samples"]:
            file = sample["file"]
            require(
                file not in (".", "..") and Path(file).name == file,
                "invalid sample name",
            )
            for path, expected in (
                (inputs / file, sample["sha256"]),
                (Path(sample["source"]), sample["source_sha256"]),
            ):
                require(digest(path) == expected, f"changed source: {path}")
                hashes[str(path)] = expected
            artifact = results / (file + ".fusion.pb")
            hashes[str(artifact)] = digest(artifact)
            cloud = reader.read(inputs / file, "apollo.drivers.PointCloud")
            out = reader.read(artifact, "apollo.lidar_fusion.FusionOutput")
            require(
                len(cloud.point) == sample["points"] == len(out.point_evidence),
                "cloud/evidence size mismatch",
            )
            expected_header = type(out.header)()
            expected_header.CopyFrom(cloud.header)
            expected_header.timestamp_sec = cloud.measurement_time
            expected_header.frame_id = config.base_frame
            expected_header.module_name = "lidar_fusion"
            require(
                expected_header == out.header
                and cloud.frame_id == config.sensor_frame
                and out.calibration_version == config.sensor_to_base.version,
                "cloud/output identity mismatch",
            )
            calibration = config.sensor_to_base
            cr, sr = math.cos(calibration.roll), math.sin(calibration.roll)
            cp, sp = math.cos(calibration.pitch), math.sin(calibration.pitch)
            cy, sy = math.cos(calibration.yaw), math.sin(calibration.yaw)
            rotation = np.array(
                [
                    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                    [-sp, cp * sr, cp * cr],
                ]
            )
            xyz = np.array([[p.x, p.y, p.z] for p in cloud.point])
            transformed = xyz @ rotation.T + [
                calibration.x,
                calibration.y,
                calibration.z,
            ]
            base = np.array(
                [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
            )
            finite = np.array([p.finite for p in out.point_evidence])
            require(
                np.allclose(transformed[finite], base[finite], rtol=0, atol=1e-9),
                "cloud/base transform or original point order mismatch",
            )
            before = out.SerializeToString(deterministic=True)
            row, estimate, candidate, regions = compare_frame(
                out, config, options, continuity_options=continuity_options
            )
            truth = None
            if "labels" in sample:
                require(
                    "SemanticKITTI" in manifest["dataset"],
                    "only SemanticKITTI point-label adaptation is implemented",
                )
                original = np.fromfile(sample["source"], dtype="<f4")
                require(
                    original.size == 4 * len(xyz)
                    and np.array_equal(original.reshape(-1, 4)[:, :3], xyz),
                    "original labeled sweep XYZ/order differs from cloud",
                )
                labels = Path(sample["labels"])
                require(digest(labels) == sample["labels_sha256"], "changed GT")
                hashes[str(labels)] = sample["labels_sha256"]
                packed = np.fromfile(labels, dtype="<u4")
                truth = learning_labels(packed)
                # GT is loaded only after the estimate; it cannot seed the fit.
                row, second, _, regions = compare_frame(
                    out, config, options, truth, packed >> 16, continuity_options
                )
                require(np.array_equal(estimate, second, equal_nan=True), "GT leaked")
            require(before == out.SerializeToString(deterministic=True), "mutation")
            np.savez_compressed(
                case_dir / (file + ".ground.npz"),
                point_id=np.arange(len(estimate), dtype=np.uint32),
                ground_z=estimate,
                ground=candidate,
                **regions,
            )
            row.update(case=name, file=file, labeled=truth is not None)
            frames.append(row)
            print(
                f"{name}/{file}: ground {row['baseline_ground_points']} -> "
                f"{row['candidate_ground_points']}; shadow only",
                flush=True,
            )
    totals = {}
    for domain in ("roi", "front"):
        totals[domain] = {}
        for variant in ("baseline", "candidate"):
            counts = Counter()
            for row in frames:
                if row["labeled"]:
                    metric = row["ground_proxy"][domain][variant]
                    counts.update({k: metric[k] for k in ("tp", "fp", "fn", "tn")})
            totals[domain][variant] = rates(counts) if counts else None
    error_classes = Counter()
    for row in frames:
        error_classes.update(row.get("new_non_ground_errors_by_class", {}))
    retention = {
        key: sum(row.get(key, 0) for row in frames)
        for key in (
            "baseline_thing_ground_errors",
            "candidate_thing_ground_errors",
            "new_thing_ground_errors",
            "new_gt_non_ground_excluded_from_measured",
        )
    }
    for path, expected in hashes.items():
        require(digest(Path(path)) == expected, f"source changed during run: {path}")
    report = {
        "diagnostic_only": True,
        "algorithm": "anchored radial minimum-bin piecewise least-squares lines v1",
        "options": asdict(options),
        "continuity_options": asdict(continuity_options),
        "experiment": experiment,
        "frames": frames,
        "ground_proxy": totals,
        "obstacle_retention": retention,
        "new_non_ground_errors_by_class": dict(error_classes),
        "promotion_checks": promotion_checks(frames, totals),
        "limitations": [
            "Semantic ground is a proxy, not physical traversability GT",
            "A radial slope is not a full 2D terrain normal or cross-slope",
            "Hypothetical measured occupancy only; candidate/unknowns not rerun",
            "No FREE, vehicle footprint, negative-obstacle or clearance proof",
            "No online adaptive state, neighboring-sector or long-gap extrapolation",
            "Connected region contains observed cells only, not a drivable polygon",
            "Sparse filtering is shadow-only and can remove a real small obstacle",
            "Nearest observed seed-ring fallback does not prove access "
            "across ego blind zone",
        ],
        "source_sha256": hashes,
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"ground_proxy": totals, "checks": report["promotion_checks"]}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("cases", "schema", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args()
    export(args.cases, args.schema, args.output)


if __name__ == "__main__":
    main()
