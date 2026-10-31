#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Frozen native/ground-artifact -> terrain, layered clearance and path risk."""

import argparse
import json
from collections import Counter
from dataclasses import asdict
from itertools import combinations
from pathlib import Path

import numpy as np
from evaluate_baseline import digest, require
from proto import Reader
from spatial_support.adapter import scenes
from spatial_support.cost import layers as cost_layers
from spatial_support.passage import query as passage_query
from spatial_support.terrain import INFERRED, MEASURED, UNKNOWN_RISK
from spatial_support.trajectory import Vehicle, query

BLOCKER_REASONS = {
    "measured": ("measured",),
    "detection": ("inferred",),
    "unknown": ("unknown_risk",),
    "terrain": ("terrain_limit",),
    "missing": ("missing_ground", "missing_terrain"),
    "disconnected": ("disconnected_ground",),
    "visibility": ("unverified_free",),
    "boundary": ("roi_boundary",),
}


def blocker_statistics(masks):
    """Count overlapping blockers without interpreting layer removal as safety."""
    names = list(masks)
    values = np.stack([np.asarray(masks[name], dtype=bool).ravel() for name in names])
    multiplicity = values.sum(axis=0)
    signatures = Counter(
        "|".join(name for name, active in zip(names, column) if active)
        for column in values.T
        if column.any()
    )
    return {
        "total": int(len(multiplicity)),
        "blocked_union": int((multiplicity > 0).sum()),
        "no_listed_blocker": int((multiplicity == 0).sum()),
        "multiple_blockers": int((multiplicity > 1).sum()),
        "counts": {name: int(mask.sum()) for name, mask in zip(names, values)},
        "exclusive": {
            name: int((mask & (multiplicity == 1)).sum())
            for name, mask in zip(names, values)
        },
        "pairwise_intersections": {
            names[i] + "|" + names[j]: int((values[i] & values[j]).sum())
            for i, j in combinations(range(len(names)), 2)
        },
        "exact_intersections": dict(sorted(signatures.items())),
        "interpretation": (
            "Reasons overlap; exclusive counts are observations with only that "
            "listed blocker, not counterfactual safety or driving authorization."
        ),
    }


def layer_blockers(scene):
    """ROI cell diagnostics; visibility means absence of qualified FREE evidence."""
    return blocker_statistics(
        {
            "measured": scene.layers & MEASURED != 0,
            "detection": scene.layers & INFERRED != 0,
            "unknown": scene.layers & UNKNOWN_RISK != 0,
            "terrain": scene.terrain_limit,
            "missing": ~(
                np.isfinite(scene.ground_z)
                & np.isfinite(scene.slope)
                & np.isfinite(scene.step)
                & np.isfinite(scene.roughness)
            ),
            "disconnected": ~scene.ego_connected,
            "visibility": ~scene.verified_free,
        }
    )


def path_blockers(result):
    """Sample-level reason counts, not an equivalence metric for path geometry."""
    return blocker_statistics(
        {
            name: [
                bool(set(reasons).intersection(sample["reasons"]))
                for sample in result["samples"]
            ]
            for name, reasons in BLOCKER_REASONS.items()
        }
    )


def export(cases_path, output):
    options = json.loads(cases_path.read_text())
    require(
        options.get("diagnostic_only") is True, "only diagnostic execution supported"
    )
    vehicle = Vehicle(**options["vehicle"])
    vehicle.validate()
    paths = options["paths"]
    require(
        bool(paths)
        and len(paths) <= 20
        and len({p["name"] for p in paths}) == len(paths),
        "empty, duplicate or excessive path probes",
    )
    for path in paths:
        require(
            path.get("kind") == "diagnostic_probe" and bool(path["name"]),
            "path is not an explicit diagnostic probe",
        )
    ground_root = Path(options["ground_results"])
    ground_report_path = ground_root / "report.json"
    ground_report = json.loads(ground_report_path.read_text())
    require(ground_report["diagnostic_only"] is True, "ground experiment scope")
    hashes = {
        str(path): digest(path)
        for path in [cases_path, ground_report_path, Path(__file__)]
        + list(Path(__file__).with_name("spatial_support").glob("*.py"))
        + [
            Path(__file__).with_name(file)
            for file in (
                "proto.py",
                "evaluate_baseline.py",
                "ground_experiment.py",
                "ground_continuity.py",
            )
        ]
    }
    for source, expected in ground_report["source_sha256"].items():
        require(digest(Path(source)) == expected, f"ground source changed: {source}")
        hashes[source] = expected
    schema = Path(options["schema"])
    hashes[str(schema)] = digest(schema)
    reader = Reader(schema)
    runs = {r["name"]: r for r in ground_report["experiment"]["runs"]}
    output.mkdir(parents=True, exist_ok=False)
    frames = []
    for frame in ground_report["frames"]:
        require(
            frame["case"] in runs
            and Path(frame["case"]).name == frame["case"]
            and frame["case"] not in (".", ".."),
            "invalid ground case name",
        )
        run = runs[frame["case"]]
        file = frame["file"]
        require(Path(file).name == file and file not in (".", ".."), "sample name")
        out = reader.read(
            Path(run["results"]) / (file + ".fusion.pb"),
            "apollo.lidar_fusion.FusionOutput",
        )
        config = reader.read(
            Path(run["results"]) / "config.pb", "apollo.lidar_fusion.FusionConfig"
        )
        artifact = ground_root / frame["case"] / (file + ".ground.npz")
        hashes[str(artifact)] = digest(artifact)
        before = out.SerializeToString(deterministic=True)
        with np.load(artifact, allow_pickle=False) as data:
            require(
                np.array_equal(data["point_id"], np.arange(len(out.point_evidence)))
                and int(data["ground"].sum()) == frame["candidate_ground_points"],
                "ground source point IDs/counts mismatch",
            )
            variants = scenes(
                out,
                config,
                data["ground_z"],
                data["ground"],
                options["diagnostic_max_step_m"],
                {
                    name: data[name + "_connected_xy"]
                    for name in ("baseline", "candidate")
                },
                ground_report["continuity_options"]["resolution"],
            )
        record = {
            "case": frame["case"],
            "file": file,
            "mode": int(out.health.mode),
            "frame_id": out.header.frame_id,
            "stamp": out.header.timestamp_sec,
            "variants": {},
        }
        for name, scene in variants.items():
            directory = output / frame["case"] / name
            directory.mkdir(parents=True, exist_ok=True)
            costs = cost_layers(
                scene,
                config.ground_max_slope,
                options["diagnostic_max_step_m"],
                config.ground_max_residual,
            )
            np.savez_compressed(
                directory / (file + ".spatial.npz"),
                resolution=scene.resolution,
                origin=scene.origin,
                layers=scene.layers,
                ground_z=scene.ground_z,
                slope=scene.slope,
                step=scene.step,
                roughness=scene.roughness,
                terrain_limit=scene.terrain_limit,
                verified_free=scene.verified_free,
                ego_connected=scene.ego_connected,
                **{"cost_" + key: value for key, value in costs.items()},
            )
            record["variants"][name] = {
                "observed_ground_cells": int(np.isfinite(scene.ground_z).sum()),
                "local_plane_cells": int(np.isfinite(scene.slope).sum()),
                "terrain_limit_cells": int(scene.terrain_limit.sum()),
                "verified_free_cells": int(scene.verified_free.sum()),
                "ego_connected_ground_cells": int(scene.ego_connected.sum()),
                "paths": {p["name"]: query(scene, p["xy"], vehicle) for p in paths},
                "directional_paths": {
                    p["name"]: passage_query(scene, p["xy"], vehicle, p.get("yaw"))
                    for p in paths
                },
                "admissible_diagnostic_cells": int(
                    costs["admissible_diagnostic"].sum()
                ),
            }
            variant = record["variants"][name]
            variant["layer_blockers"] = layer_blockers(scene)
            variant["path_blockers"] = {
                geometry: {
                    path_name: path_blockers(result)
                    for path_name, result in variant[geometry].items()
                }
                for geometry in ("paths", "directional_paths")
            }
        require(before == out.SerializeToString(deterministic=True), "native mutation")
        frames.append(record)
        print(
            f"{frame['case']}/{file}: terrain/clearance/path probes complete",
            flush=True,
        )
    require(bool(frames), "no frames evaluated")
    for source, expected in hashes.items():
        require(
            digest(Path(source)) == expected, f"source changed during run: {source}"
        )
    totals = {}
    for name in ("baseline", "candidate"):
        counts = Counter()
        for frame in frames:
            for path in frame["variants"][name]["paths"].values():
                counts.update(path["state_counts"])
        totals[name] = dict(counts)
    report = {
        "diagnostic_only": True,
        "production_mutated": False,
        "production_qualified": False,
        "options": options,
        "vehicle": asdict(vehicle),
        "frames": frames,
        "probe_state_counts": totals,
        "source_sha256": hashes,
        "limitations": [
            "Paths and vehicle dimensions are illustrative diagnostic probes",
            "No actual planner trajectory, speed, reaction time or braking validation",
            "Local plane residual is a terrain proxy, not true roughness GT",
            "Native inferred and unknown risks retained; sparse filtering disabled",
            "No FREE authorized; candidate never inherits baseline FREE",
            "Swept disk is conservative and may reject a usable narrow passage",
            "Same-frame static geometry; no future dynamic occupancy prediction",
        ],
    }
    (output / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    print(json.dumps({"frames": len(frames), "probe_state_counts": totals}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    export(args.cases, args.output)


if __name__ == "__main__":
    main()
