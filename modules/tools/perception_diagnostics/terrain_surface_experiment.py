#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Frozen analytic and real-record evaluation for local terrain surfaces."""

import argparse
import hashlib
import json
import math
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from evaluate_baseline import audit_frame, digest, require
from ground_experiment import RadialOptions, radial_ground
from ground_patch_experiment import source_frame
from proto import Reader, require_full_evidence
from semantickitti import GROUND, learning_labels
from terrain_surface import TerrainSurfaceConfig, estimate_terrain_surface

ROOT = Path(__file__).resolve().parents[3]
ARTIFACT_ROOT = ROOT / ".cache/lidar-fusion-validation/terrain-v4"
FRAME_ROOT = ARTIFACT_ROOT / "frames"
CASE_FILE = ARTIFACT_ROOT / "case-matrix.json"
REAL_FILE = ARTIFACT_ROOT / "real-data-report.json"
PERSIST_FILE = ARTIFACT_ROOT / "persisted-output-check.json"

RADIAL_CONTROL = RadialOptions(
    sectors=360,
    bin_size=0.5,
    max_gap=1.0,
    max_start_radius=8.0,
    max_line_length=4.0,
    min_bins=3,
)
CONTINUITY_STEP_M = 0.20
RADIAL_BANDS = ((0.0, 5.0), (5.0, 15.0), (15.0, 30.0), (30.0, math.inf))

CASE_CONTRACTS = (
    {
        "id": "flat-ground",
        "objective": "Recover a supported flat surface from observed points.",
        "expected": (
            "Root-connected selected elevation is 0 m within "
            "0.02 m; slope gradient <= 0.02."
        ),
        "truth_source": "Independent analytic equation z=0.",
    },
    {
        "id": "continuous-ramp",
        "objective": (
            "Preserve a locally sloped surface instead of "
            "imposing one global height."
        ),
        "expected": (
            "Observed elevation changes with x; slope gradient "
            "is 0.10 +/- 0.02; support is connected."
        ),
        "truth_source": "Independent analytic equation z=0.10*x.",
    },
    {
        "id": "abrupt-step",
        "objective": (
            "Measure a real discontinuity without smoothing it "
            "into a ramp or flat plane."
        ),
        "expected": (
            "Both observed levels remain distinct; the "
            "cell-aligned shared-boundary step is 0.30 +/- 0.03 "
            "m where rooted support exists."
        ),
        "truth_source": (
            "Independent piecewise analytic surface z=0 / "
            "z=0.30; boundary aligned to the frozen 0.5m cells."
        ),
    },
    {
        "id": "subcell-two-layer-boundary",
        "objective": "Expose a height discontinuity that falls within one 0.5m cell.",
        "expected": (
            "Both original height layers and their source IDs "
            "remain visible; the 0.30m raw span is reported "
            "separately from the cell step, and an unsupported "
            "target cell remains UNKNOWN."
        ),
        "truth_source": (
            "Independent piecewise analytic surface with a "
            "discontinuity inside one XY cell."
        ),
    },
    {
        "id": "wide-shallow-pit",
        "objective": (
            "Measure a broad shallow depression from original " "observed returns."
        ),
        "expected": (
            "Pit status is measured; depth is 0.30 +/- 0.05 m; "
            "widths remain finite and positive."
        ),
        "truth_source": "Independent analytic bowl with 0.30 m maximum depth.",
    },
    {
        "id": "narrow-deep-pit",
        "objective": (
            "Retain a narrow deep depression or explicitly mark "
            "its unsupported floor unknown."
        ),
        "expected": (
            (
                "Measured floor depth is not "
                + "flattened; otherwise status is "
                + "unknown_no_floor/unknown_no_rim/"
                + "unknown_insufficient_floor and depth stays NaN."
            )
        ),
        "truth_source": "Independent analytic narrow bowl with 0.60 m maximum depth.",
    },
    {
        "id": "undersupported-pit-floor",
        "objective": (
            "Keep a pit floor with only one raw return unknown "
            "rather than inventing depth."
        ),
        "expected": (
            "Pit validity is unknown and depth is NaN at the " "one-return floor cell."
        ),
        "truth_source": "Independent sparse floor input with a one-point lower layer.",
    },
    {
        "id": "bridge-two-level",
        "objective": (
            "Preserve upper and lower observed surfaces at the "
            "same XY without min-height blending."
        ),
        "expected": (
            "Original point layers remain distinct by >2.5 m; "
            "lower-root selection cannot erase upper observed "
            "layer IDs."
        ),
        "truth_source": "Independent two-level geometry z=0 and z=3.0 at matching XY.",
    },
    {
        "id": "unknown-gap",
        "objective": (
            "Keep an unobserved region unknown; do not "
            "interpolate or connect through it."
        ),
        "expected": (
            "Every unobserved gap cell has UNKNOWN status and "
            "NaN selected elevation; rooted graph does not "
            "traverse it."
        ),
        "truth_source": "Input deliberately omits all returns in a fixed ROI band.",
    },
    {
        "id": "no-anchor-unknown",
        "objective": (
            "Keep observed cells UNKNOWN when no valid frozen " "root can be selected."
        ),
        "expected": (
            "No selected elevations, no rooted connectivity, and "
            "every observed unsupported cell has an UNKNOWN status."
        ),
        "truth_source": (
            "Observed points are outside the configured " "initial-height anchor band."
        ),
    },
    {
        "id": "multilevel-local-surface",
        "objective": (
            "Represent nearby distinct elevation layers without "
            "forcing the local surface to one height."
        ),
        "expected": (
            "Layer membership and observed source IDs "
            "distinguish both levels; unsupported layers are "
            "explicit."
        ),
        "truth_source": (
            "Independent two-level geometry with adjacent, "
            "non-overlapping XY supports."
        ),
    },
    {
        "id": "small-risk-object",
        "objective": (
            "Prevent a source-identified obstacle from being "
            "absorbed into selected ground."
        ),
        "expected": (
            "Independent risk mask is unchanged; risk point is "
            "never in final ground; fit/layer evidence is "
            "unchanged when only risk eligibility changes."
        ),
        "truth_source": "Synthetic source point IDs and risk mask, not semantic GT.",
    },
    {
        "id": "coplanar-thing-ambiguity",
        "objective": (
            "Expose that geometry alone cannot distinguish a "
            "coplanar thing from terrain."
        ),
        "expected": (
            "Geometry-only outputs are identical with/without an "
            "external semantic interpretation; report as "
            "intrinsic ambiguity, not a geometric miss."
        ),
        "truth_source": (
            "Same XYZ assigned different external labels only " "after fitting."
        ),
    },
    {
        "id": "fixed-common-root",
        "objective": (
            "Compare candidate surfaces using one root selected "
            "from raw support before either fit."
        ),
        "expected": (
            "Both runs receive the identical observed point ID; "
            "root cell is identical; candidate footprint/area "
            "cannot move it."
        ),
        "truth_source": "Input geometry and explicit frozen anchor ID.",
    },
    {
        "id": "isolated-near-root-return",
        "objective": (
            "Prevent one isolated raw return from defining a "
            "false continuous-region root."
        ),
        "expected": (
            "Root selection requires supported observed evidence "
            "or explicitly reports insufficient root support; it "
            "must not claim a connected terrain region from one "
            "point."
        ),
        "truth_source": (
            "Independent geometry with one low isolated return "
            "and a broader nearby plane."
        ),
    },
    {
        "id": "z-translation-invariance",
        "objective": (
            "Check local terrain geometry under a rigid vertical " "frame translation."
        ),
        "expected": (
            "Adding +4 m and translating the initialization by "
            "+4 m preserves layers, slope, step, and pit "
            "depth/width."
        ),
        "truth_source": "Independent coordinate transformation.",
    },
)

CACHED_RUNS = (
    {
        "name": "semantickitti-750-1750-2750",
        "inputs": ".cache/lidar-fusion-validation/next-iteration/new-inputs",
        "results": ".cache/lidar-fusion-validation/current-head-replay-v1",
    },
    {
        "name": "semantickitti-500-1500-2500",
        "inputs": (
            ".cache/lidar-fusion-validation/semantickitti-quality-" + "unseen-v1/inputs"
        ),
        "results": ".cache/lidar-fusion-validation/current-head-replay-test-v1",
    },
    *(
        {
            "name": f"nuscenes-{scene}",
            "inputs": (
                ".cache/lidar-fusion-validation/nuscenes-v1/scene-" + f"{scene}/inputs"
            ),
            "results": (
                ".cache/lidar-fusion-validation/current-head-nuscenes-v1/scene-"
                + f"{scene}-supported-replay"
            ),
        }
        for scene in ("0003", "0012", "0013")
    ),
)


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "__dataclass_fields__"):
        return {key: _jsonable(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    return value


def _write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_jsonable(payload), indent=2, sort_keys=True, allow_nan=False) + "\n"
    )


def _finite_or_none(value):
    return float(value) if np.isfinite(value) else None


def _surface_metrics(result):
    fields = (
        "point_ground_candidate_mask",
        "point_ground_mask",
        "point_risk_mask",
        "point_cell_id",
        "point_layer_id",
        "point_surface_z",
        "point_residual_z",
        "cell_status",
        "cell_elevation_z",
        "cell_raw_return_count",
        "cell_roughness_m",
        "cell_slope_gradient",
        "cell_step_m",
        "cell_step_status",
        "cell_observed_layer_span_m",
        "pit_status",
        "pit_depth_m",
        "pit_width_x_m",
        "pit_width_y_m",
    )
    return {name: np.asarray(getattr(result, name)).copy() for name in fields}


def _layer_arrays(result):
    rows = []
    point_ids = []
    offsets = [0]
    for ix, iy in np.ndindex(result.cell_observed_layers.shape):
        for layer in result.cell_observed_layers[ix, iy]:
            source_ids = [int(value) for value in layer.source_point_ids]
            rows.append(
                (
                    ix,
                    iy,
                    layer.layer_id,
                    layer.elevation_z,
                    layer.point_count,
                    layer.selected,
                )
            )
            point_ids.extend(source_ids)
            offsets.append(len(point_ids))
    values = np.asarray(rows, dtype=float).reshape(-1, 6)
    return {
        "observed_layer_cell_xy": values[:, :2].astype(np.int32),
        "observed_layer_id": values[:, 2].astype(np.int32),
        "observed_layer_elevation_z_m": values[:, 3].astype(np.float64),
        "observed_layer_point_count": values[:, 4].astype(np.int32),
        "observed_layer_selected": values[:, 5].astype(bool),
        "observed_layer_source_offsets": np.asarray(offsets, dtype=np.int64),
        "observed_layer_source_point_ids": np.asarray(point_ids, dtype=np.uint32),
    }


def _grid_fixture(surface, *, x_min=-3.0, x_max=7.0, y_min=-3.0, y_max=3.0):
    xs = np.arange(x_min, x_max + 1e-9, 0.25)
    ys = np.arange(y_min, y_max + 1e-9, 0.25)
    xx, yy = np.meshgrid(xs, ys, indexing="ij")
    xyz = np.column_stack((xx.ravel(), yy.ravel(), surface(xx, yy).ravel()))
    return xyz


def _anchor_id(points, height=0.0):
    finite = np.isfinite(points).all(axis=1)
    radial = np.hypot(points[:, 0], points[:, 1])
    near = finite & (radial <= 1.0) & (np.abs(points[:, 2] - height) <= 0.5)
    ids = np.flatnonzero(near)
    require(len(ids) > 0, "analytic fixture has no fixed observed anchor")
    return int(ids[np.argmin(radial[ids])])


def _analytic_config(points, *, expected_ground_z=0.0, **kwargs):
    values = {
        "x_min": float(np.nanmin(points[:, 0]) - 0.25),
        "x_max": float(np.nanmax(points[:, 0]) + 0.25),
        "y_min": float(np.nanmin(points[:, 1]) - 0.25),
        "y_max": float(np.nanmax(points[:, 1]) + 0.25),
        "cell_size": 0.5,
        "expected_ground_z": expected_ground_z,
        "seed_height_band": 0.35,
        "anchor_radius": 8.0,
        "layer_gap": 0.20,
        "ground_tolerance": 0.10,
        "max_slope_gradient": 0.30,
        "max_neighbor_height_change": CONTINUITY_STEP_M,
    }
    values.update(kwargs)
    return TerrainSurfaceConfig(**values)


def _run(points, *, risk=None, root_id=None, config=None, auto_root=False):
    points = np.asarray(points, dtype=np.float64)
    ids = np.arange(len(points), dtype=np.uint32)
    risk = (
        np.zeros(len(points), dtype=bool)
        if risk is None
        else np.asarray(risk, dtype=bool)
    )
    if root_id is None and not auto_root:
        root_id = _anchor_id(points, config.expected_ground_z if config else 0.0)
    return estimate_terrain_surface(
        points,
        point_ids=ids,
        risk_mask=risk,
        root_anchor_id=None if root_id is None else int(root_id),
        config=config or _analytic_config(points),
    )


def _assert_close(actual, expected, tolerance, message):
    if not np.isfinite(actual) or abs(float(actual) - expected) > tolerance:
        raise AssertionError(
            f"{message}: got {actual}, expected {expected} +/- {tolerance}"
        )


def run_analytic_cases():
    cases = []
    results = {}

    def record(case_id, actual, status, first_boundary=None):
        contract = next(item for item in CASE_CONTRACTS if item["id"] == case_id)
        cases.append(
            {
                **contract,
                "actual": actual,
                "functional_verdict": status,
                "real_record_effect": "not applicable: analytic fixture",
                "truth_level_ambiguity": (
                    "inherent geometry-only ambiguity"
                    if case_id == "coplanar-thing-ambiguity"
                    else "none for the constructed analytic observable"
                ),
                "source_hash": hashlib.sha256(
                    json.dumps(contract, sort_keys=True).encode()
                ).hexdigest(),
                "first_wrong_boundary": first_boundary,
            }
        )

    flat = _grid_fixture(lambda x, y: np.zeros_like(x))
    flat_result = _run(flat)
    results["flat"] = (flat, flat_result)
    values = flat_result.cell_elevation_z[flat_result.cell_status == "supported"]
    _assert_close(float(np.median(values)), 0.0, 0.02, "flat elevation")
    require(
        float(np.nanmax(flat_result.cell_slope_gradient)) <= 0.02,
        "flat slope gradient exceeds tolerance",
    )
    record(
        "flat-ground",
        {
            "supported_cells": int(np.isfinite(values).sum()),
            "median_z_m": float(np.median(values)),
        },
        "PASS",
    )

    ramp = _grid_fixture(lambda x, y: 0.1 * x)
    ramp_config = _analytic_config(ramp)
    ramp_result = _run(ramp, config=ramp_config)
    results["ramp"] = (ramp, ramp_result)
    slope = ramp_result.cell_slope_gradient[
        (ramp_result.cell_status == "supported")
        & np.isfinite(ramp_result.cell_slope_gradient)
    ]
    _assert_close(float(np.median(slope)), 0.1, 0.02, "ramp slope gradient")
    require(
        float(
            np.nanmax(ramp_result.cell_elevation_z)
            - np.nanmin(ramp_result.cell_elevation_z)
        )
        > 0.7,
        "ramp elevation was forced to a constant",
    )
    record(
        "continuous-ramp",
        {
            "supported_cells": int(len(slope)),
            "median_slope_gradient": float(np.median(slope)),
        },
        "PASS",
    )

    step_points = _grid_fixture(lambda x, y: np.where(x < 0.25, 0.0, 0.30))
    step_result = _run(step_points)
    results["step"] = (step_points, step_result)
    step_values = step_result.cell_step_m[np.isfinite(step_result.cell_step_m)]
    require(len(step_values) > 0, "step metric has no valid adjacent surface")
    _assert_close(float(np.max(step_values)), 0.30, 0.03, "step height")
    record(
        "abrupt-step",
        {
            "max_valid_step_m": float(np.max(step_values)),
            "step_cells": int(len(step_values)),
        },
        "PASS",
    )

    subcell_points = _grid_fixture(lambda x, y: np.where(x < 0.0, 0.0, 0.30))
    subcell_result = _run(subcell_points)
    results["subcell_step"] = (subcell_points, subcell_result)
    boundary_row = int(
        np.argmin(np.sum((subcell_points[:, :2] - [0.0, 0.0]) ** 2, axis=1))
    )
    boundary_key = divmod(
        int(subcell_result.point_cell_id[boundary_row]),
        subcell_result.cell_status.shape[1],
    )
    boundary_layers = subcell_result.cell_observed_layers[boundary_key]
    require(
        len(boundary_layers) == 2,
        "sub-cell discontinuity lost one observed height layer",
    )
    layer_separation = max(layer.elevation_z for layer in boundary_layers) - min(
        layer.elevation_z for layer in boundary_layers
    )
    _assert_close(layer_separation, 0.30, 0.03, "sub-cell raw layer separation")
    cell_layer_span = float(subcell_result.cell_observed_layer_span_m[boundary_key])
    _assert_close(cell_layer_span, 0.30, 0.03, "sub-cell observed layer span")
    cell_step = subcell_result.cell_step_m[boundary_key]
    record(
        "subcell-two-layer-boundary",
        {
            "cell_layer_separation_m": float(layer_separation),
            "cell_observed_layer_span_m": cell_layer_span,
            "target_cell_status": str(subcell_result.cell_status[boundary_key]),
            "step_metric_m": _finite_or_none(cell_step),
            "step_metric_status": str(subcell_result.cell_step_status[boundary_key]),
            "source_ids_preserved": all(
                layer.source_point_ids for layer in boundary_layers
            ),
        },
        "DIAGNOSTIC",
        (
            "same-cell layer span alone does not distinguish an "
            "abrupt step from a bridge/overlap"
        ),
    )

    wide_pit = _grid_fixture(
        lambda x, y: np.where((x >= 2.0) & (x < 4.0) & (np.abs(y) < 1.0), -0.30, 0.0)
    )
    wide_result = _run(wide_pit)
    results["wide_pit"] = (wide_pit, wide_result)
    measured = wide_result.pit_status == "measured"
    require(measured.any(), "wide shallow pit was not measured")
    _assert_close(
        float(np.nanmax(wide_result.pit_depth_m[measured])),
        0.30,
        0.05,
        "wide pit depth",
    )
    require(
        np.nanmax(wide_result.pit_width_x_m[measured]) >= 1.5
        and np.nanmax(wide_result.pit_width_y_m[measured]) >= 1.5,
        "wide pit measured connected-cell widths are too small",
    )
    record(
        "wide-shallow-pit",
        {
            "measured_cells": int(measured.sum()),
            "max_depth_m": float(np.nanmax(wide_result.pit_depth_m[measured])),
            "max_width_x_m": float(np.nanmax(wide_result.pit_width_x_m[measured])),
            "max_width_y_m": float(np.nanmax(wide_result.pit_width_y_m[measured])),
            "width_semantics": (
                "connected observed depressed-cell extents; not "
                "physical pit boundaries"
            ),
        },
        "PASS",
    )

    narrow_pit = _grid_fixture(
        lambda x, y: np.where((x >= 2.5) & (x < 3.5) & (np.abs(y) < 0.5), -0.60, 0.0)
    )
    narrow_result = _run(narrow_pit)
    results["narrow_pit"] = (narrow_pit, narrow_result)
    narrow_measured = narrow_result.pit_status == "measured"
    narrow_floor_row = int(
        np.argmin(np.sum((narrow_pit[:, :2] - [3.0, 0.0]) ** 2, axis=1))
    )
    narrow_floor_cell = divmod(
        int(narrow_result.point_cell_id[narrow_floor_row]),
        narrow_result.cell_status.shape[1],
    )
    narrow_floor_status = str(narrow_result.pit_status[narrow_floor_cell])
    require(
        narrow_floor_status
        in (
            "measured",
            "unknown_no_floor",
            "unknown_no_rim",
            "unknown_insufficient_floor",
        ),
        "narrow pit floor must be measured or explicitly unknown",
    )
    if narrow_floor_status == "measured":
        require(
            float(narrow_result.pit_depth_m[narrow_floor_cell]) >= 0.45,
            "narrow pit depth was smoothed below observable depth",
        )
    else:
        require(
            np.isnan(narrow_result.pit_depth_m[narrow_floor_cell]),
            "unsupported narrow floor must have unknown pit depth",
        )
    record(
        "narrow-deep-pit",
        {
            "measured_cells": int(narrow_measured.sum()),
            "floor_cell_status": narrow_floor_status,
            "max_depth_m": _finite_or_none(
                narrow_result.pit_depth_m[narrow_floor_cell]
            ),
            "width_x_m": _finite_or_none(
                narrow_result.pit_width_x_m[narrow_floor_cell]
            ),
            "width_y_m": _finite_or_none(
                narrow_result.pit_width_y_m[narrow_floor_cell]
            ),
        },
        "PASS",
    )

    sparse_floor = _grid_fixture(lambda x, y: np.zeros_like(x))
    sparse_floor = sparse_floor[
        ~(
            (sparse_floor[:, 0] >= 2.0)
            & (sparse_floor[:, 0] < 4.0)
            & (np.abs(sparse_floor[:, 1]) < 1.0)
        )
    ]
    sparse_floor = np.concatenate(
        (sparse_floor, np.array([[2.25, 0.0, -0.60]], dtype=float))
    )
    sparse_result = _run(sparse_floor)
    results["sparse_pit_floor"] = (sparse_floor, sparse_result)
    sparse_floor_id = int(
        np.argmin(np.sum((sparse_floor[:, :2] - [2.25, 0.0]) ** 2, axis=1))
    )
    sparse_ix, sparse_iy = divmod(
        int(sparse_result.point_cell_id[sparse_floor_id]),
        sparse_result.cell_status.shape[1],
    )
    require(
        sparse_result.pit_status[sparse_ix, sparse_iy]
        in ("unknown_no_floor", "unknown_no_rim", "unknown_insufficient_floor"),
        "a one-return depression floor must remain explicitly unknown",
    )
    require(
        not np.isfinite(sparse_result.pit_depth_m[sparse_ix, sparse_iy]),
        "unsupported pit floor cannot publish a depth",
    )
    record(
        "undersupported-pit-floor",
        {
            "floor_returns": 1,
            "status": str(sparse_result.pit_status[sparse_ix, sparse_iy]),
            "depth_is_nan": bool(
                np.isnan(sparse_result.pit_depth_m[sparse_ix, sparse_iy])
            ),
        },
        "PASS",
    )

    lower = _grid_fixture(lambda x, y: np.zeros_like(x))
    upper = lower.copy()
    upper[:, 2] += 3.0
    bridge_points = np.concatenate((lower, upper), axis=0)
    bridge_result = _run(bridge_points, root_id=_anchor_id(lower))
    results["bridge"] = (bridge_points, bridge_result)
    anchor_row = _anchor_id(lower)
    bridge_cell_id = int(bridge_result.point_cell_id[anchor_row])
    bridge_ix, bridge_iy = divmod(bridge_cell_id, bridge_result.cell_status.shape[1])
    bridge_layers = bridge_result.cell_observed_layers[bridge_ix, bridge_iy]
    require(
        len(bridge_layers) == 2, "bridge cell did not preserve both observed layers"
    )
    bridge_heights = sorted(float(layer.elevation_z) for layer in bridge_layers)
    _assert_close(
        bridge_heights[1] - bridge_heights[0], 3.0, 1e-9, "bridge layer separation"
    )
    require(
        all(layer.source_point_ids for layer in bridge_layers),
        "bridge layers lost their observed source point IDs",
    )
    upper_root_id = len(lower) + anchor_row
    upper_root_result = _run(bridge_points, root_id=upper_root_id)
    require(
        abs(float(upper_root_result.cell_elevation_z[bridge_ix, bridge_iy]) - 3.0)
        <= 0.02,
        "upper bridge surface cannot be selected from its own raw observed root",
    )
    require(
        len(upper_root_result.cell_observed_layers[bridge_ix, bridge_iy]) == 2,
        "selecting upper bridge layer erased the lower source layer",
    )
    raw_layer_gap = np.median(upper[:, 2]) - np.median(lower[:, 2])
    _assert_close(raw_layer_gap, 3.0, 1e-9, "bridge source level gap")
    record(
        "bridge-two-level",
        {
            "cell_layer_ids": [int(layer.layer_id) for layer in bridge_layers],
            "cell_layer_point_counts": [
                int(layer.point_count) for layer in bridge_layers
            ],
            "cell_layer_source_ids_preserved": True,
            "upper_root_selects_upper_layer": True,
            "raw_layer_separation_m": float(raw_layer_gap),
            "root_anchor_id": int(bridge_result.root_anchor_id),
        },
        "PASS",
    )

    gap_points = _grid_fixture(lambda x, y: np.zeros_like(x))
    gap_keep = ~((gap_points[:, 0] >= 0.25) & (gap_points[:, 0] < 2.25))
    gap_points = gap_points[gap_keep]
    gap_result = _run(gap_points)
    results["gap"] = (gap_points, gap_result)
    cell_x = (
        np.asarray(gap_result.config.x_min)
        + (np.arange(gap_result.cell_status.shape[0]) + 0.5)
        * gap_result.config.cell_size
    )
    gap_x = (cell_x >= 0.5) & (cell_x < 2.5)
    gap_status = gap_result.cell_status[gap_x]
    require(len(gap_status) > 0, "gap fixture did not cover any cells")
    require(
        np.all(
            np.isin(gap_status, ("unknown_empty", "unknown_gap", "unknown_no_anchor"))
        )
        and np.isnan(gap_result.cell_elevation_z[gap_x]).all(),
        "unobserved gap was filled or selected as ground",
    )
    record(
        "unknown-gap",
        {
            "gap_cells": int(gap_x.sum() * gap_result.cell_status.shape[1]),
            "gap_status_counts": {
                str(name): int(count)
                for name, count in zip(*np.unique(gap_status, return_counts=True))
            },
            "gap_elevation_nan": bool(
                np.isnan(gap_result.cell_elevation_z[gap_x]).all()
            ),
        },
        "PASS",
    )

    multi_points = np.concatenate(
        (
            _grid_fixture(lambda x, y: np.zeros_like(x), x_min=-3, x_max=1),
            _grid_fixture(lambda x, y: np.full_like(x, 1.0), x_min=1.5, x_max=5.0),
        )
    )
    multi_result = _run(multi_points, root_id=_anchor_id(multi_points))
    results["multilevel"] = (multi_points, multi_result)
    low_row = int(np.argmin(np.sum((multi_points[:, :2] - [-1.0, 0.0]) ** 2, axis=1)))
    high_row = int(np.argmin(np.sum((multi_points[:, :2] - [2.0, 0.0]) ** 2, axis=1)))
    low_key = divmod(
        int(multi_result.point_cell_id[low_row]), multi_result.cell_status.shape[1]
    )
    high_key = divmod(
        int(multi_result.point_cell_id[high_row]), multi_result.cell_status.shape[1]
    )
    low_layers = multi_result.cell_observed_layers[low_key]
    high_layers = multi_result.cell_observed_layers[high_key]
    require(
        len(low_layers) == len(high_layers) == 1
        and abs(float(low_layers[0].elevation_z) - float(high_layers[0].elevation_z))
        > 0.8
        and high_layers[0].source_point_ids,
        "nearby elevated surface was flattened or lost from observed-layer evidence",
    )
    record(
        "multilevel-local-surface",
        {
            "lower_cell_z_m": float(low_layers[0].elevation_z),
            "upper_cell_z_m": float(high_layers[0].elevation_z),
            "upper_source_ids_preserved": bool(high_layers[0].source_point_ids),
            "upper_selected_from_lower_root": bool(
                multi_result.cell_status[high_key] == "supported"
            ),
        },
        "PASS",
    )

    obstacle_ground = _grid_fixture(lambda x, y: np.zeros_like(x))
    obstacle = np.array(
        [[1.0, 0.0, 0.5], [1.0, 0.25, 0.5], [1.25, 0.0, 0.5]], dtype=float
    )
    obstacle_points = np.concatenate((obstacle_ground, obstacle))
    obstacle_risk = np.zeros(len(obstacle_points), dtype=bool)
    obstacle_risk[-len(obstacle) :] = True
    obstacle_root = _anchor_id(obstacle_ground)
    obstacle_result = _run(obstacle_points, risk=obstacle_risk, root_id=obstacle_root)
    no_risk_result = _run(
        obstacle_points,
        risk=np.zeros(len(obstacle_points), dtype=bool),
        root_id=obstacle_root,
    )
    results["obstacle"] = (obstacle_points, obstacle_result)
    require(
        np.array_equal(
            obstacle_result.point_ground_candidate_mask,
            no_risk_result.point_ground_candidate_mask,
        )
        and np.array_equal(
            obstacle_result.point_layer_id, no_risk_result.point_layer_id
        ),
        "independent risk changed surface fit/layer evidence",
    )
    require(
        obstacle_result.point_risk_mask[-len(obstacle) :].all()
        and not obstacle_result.point_ground_mask[-len(obstacle) :].any(),
        "source-identified obstacle risk was absorbed into final ground",
    )
    root_risk = np.zeros(len(obstacle_points), dtype=bool)
    root_risk[obstacle_root] = True
    root_risk_result = _run(obstacle_points, risk=root_risk, root_id=obstacle_root)
    require(
        int(root_risk_result.root_anchor_id) == obstacle_root
        and np.array_equal(
            root_risk_result.point_ground_candidate_mask,
            no_risk_result.point_ground_candidate_mask,
        )
        and not root_risk_result.point_ground_mask[obstacle_root],
        "risk mask changed raw root/support instead of suppressing only final ground",
    )
    record(
        "small-risk-object",
        {
            "risk_point_count": int(obstacle_risk.sum()),
            "candidate_ground_under_risk": int(
                obstacle_result.point_ground_candidate_mask[-len(obstacle) :].sum()
            ),
            "final_ground_under_risk": int(
                obstacle_result.point_ground_mask[-len(obstacle) :].sum()
            ),
            "fit_identical_with_and_without_risk": True,
            "risk_marked_root_still_selects_same_raw_surface": True,
        },
        "PASS",
    )

    ambiguous_points = _grid_fixture(lambda x, y: np.zeros_like(x))
    ambiguous_result = _run(ambiguous_points)
    duplicate_geometry_result = _run(ambiguous_points.copy())
    require(
        np.array_equal(
            ambiguous_result.point_ground_mask,
            duplicate_geometry_result.point_ground_mask,
        )
        and np.array_equal(
            ambiguous_result.cell_elevation_z,
            duplicate_geometry_result.cell_elevation_z,
            equal_nan=True,
        ),
        "semantically different but geometrically identical observations diverged",
    )
    results["coplanar"] = (ambiguous_points, ambiguous_result)
    record(
        "coplanar-thing-ambiguity",
        {
            "geometry_identical_outputs": True,
            "ground_truth_used_by_estimator": False,
            "verdict": "AMBIGUOUS_BY_CONSTRUCTION",
        },
        "DIAGNOSTIC",
    )

    root = _anchor_id(flat)
    explicit_root = _run(flat, root_id=root)
    widened = np.concatenate(
        (flat, _grid_fixture(lambda x, y: np.zeros_like(x), x_min=7.5, x_max=9.0))
    )
    same_root = _run(widened, root_id=root)
    require(
        int(explicit_root.root_anchor_id) == int(same_root.root_anchor_id) == root,
        "candidate area change moved the frozen raw anchor",
    )
    require(
        tuple(explicit_root.root_cell) == tuple(same_root.root_cell),
        "candidate area change moved the common root cell",
    )
    record(
        "fixed-common-root",
        {"root_anchor_id": root, "root_cell": list(map(int, explicit_root.root_cell))},
        "PASS",
    )

    broad = _grid_fixture(
        lambda x, y: np.zeros_like(x), x_min=0.5, x_max=7.0, y_min=-2.0, y_max=2.0
    )
    isolated = np.array([[0.0, 0.0, -1.0]], dtype=float)
    root_probe = np.concatenate((isolated, broad), axis=0)
    root_probe_config = _analytic_config(root_probe, expected_ground_z=None)
    root_probe_result = _run(root_probe, config=root_probe_config, auto_root=True)
    root_key = tuple(map(int, root_probe_result.root_cell))
    root_support_count = int(root_probe_result.cell_raw_return_count[root_key])
    root_connected_cells = int(
        np.count_nonzero(root_probe_result.cell_status == "supported")
    )
    root_isolated = int(root_probe_result.root_anchor_id) == 0
    root_contract_met = root_support_count >= 3 and root_connected_cells > 1
    record(
        "isolated-near-root-return",
        {
            "root_anchor_id": int(root_probe_result.root_anchor_id),
            "root_isolated_point_selected": root_isolated,
            "raw_support_in_root_cell": root_support_count,
            "root_connected_cells": root_connected_cells,
            "overall_status": root_probe_result.overall_status,
        },
        "PASS" if root_contract_met else "FAIL",
        (
            None
            if root_contract_met
            else (
                "auto-root accepts nearest one-return cell as "
                "supported root; no minimum observed root support"
            )
        ),
    )

    translated = flat.copy()
    translated[:, 2] += 4.0
    _analytic_config(translated, expected_ground_z=4.0)
    auto_original = _run(
        flat,
        config=_analytic_config(flat, expected_ground_z=None),
        auto_root=True,
    )
    explicit_original = _run(
        flat,
        root_id=root,
        config=_analytic_config(flat, expected_ground_z=None),
    )
    explicit_translated = _run(
        translated,
        root_id=root,
        config=_analytic_config(translated, expected_ground_z=None),
    )
    translated_result = _run(
        translated,
        config=_analytic_config(translated, expected_ground_z=None),
        auto_root=True,
    )
    require(
        int(auto_original.root_anchor_id) == int(translated_result.root_anchor_id),
        "automatic root changed under a pure vertical translation",
    )
    require(
        np.array_equal(auto_original.cell_status, translated_result.cell_status),
        "vertical translation changed cell support status",
    )
    require(
        int(explicit_original.root_anchor_id)
        == int(explicit_translated.root_anchor_id)
        == root
        and np.array_equal(
            explicit_original.cell_status, explicit_translated.cell_status
        ),
        "vertical translation changed explicit-root support",
    )
    for field in (
        "cell_slope_gradient",
        "cell_step_m",
        "pit_depth_m",
        "pit_width_x_m",
        "pit_width_y_m",
    ):
        a = np.asarray(getattr(auto_original, field))
        b = np.asarray(getattr(translated_result, field))
        matches = (
            np.allclose(a, b, rtol=1e-12, atol=1e-12, equal_nan=True)
            if field == "cell_slope_gradient"
            else np.array_equal(a, b, equal_nan=True)
        )
        require(
            matches,
            f"vertical translation changed local {field}",
        )
        explicit_a = np.asarray(getattr(explicit_original, field))
        explicit_b = np.asarray(getattr(explicit_translated, field))
        explicit_matches = (
            np.allclose(explicit_a, explicit_b, rtol=1e-12, atol=1e-12, equal_nan=True)
            if field == "cell_slope_gradient"
            else np.array_equal(explicit_a, explicit_b, equal_nan=True)
        )
        require(
            explicit_matches,
            f"vertical translation changed explicit-root {field}",
        )
    record(
        "z-translation-invariance",
        {
            "z_shift_m": 4.0,
            "auto_root_ids_equal": int(auto_original.root_anchor_id)
            == int(translated_result.root_anchor_id),
            "explicit_root_id_preserved": int(explicit_original.root_anchor_id)
            == int(explicit_translated.root_anchor_id)
            == root,
            "auto_and_explicit_slope_step_pit_invariant": True,
        },
        "PASS",
    )

    no_anchor_config = _analytic_config(
        flat, expected_ground_z=5.0, seed_height_band=0.1
    )
    no_anchor_result = _run(flat, root_id=None, config=no_anchor_config, auto_root=True)
    results["no_anchor"] = (flat, no_anchor_result)
    require(
        no_anchor_result.root_anchor_id is None
        and no_anchor_result.overall_status == "unknown_no_anchor"
        and np.isnan(no_anchor_result.cell_elevation_z).all(),
        "no-anchor case published a surface or a root",
    )
    require(
        np.all(
            no_anchor_result.cell_status[no_anchor_result.cell_raw_return_count > 0]
            == "unknown_no_anchor"
        ),
        "observed cells without an anchor must be UNKNOWN",
    )
    record(
        "no-anchor-unknown",
        {
            "overall_status": no_anchor_result.overall_status,
            "root_anchor_id": no_anchor_result.root_anchor_id,
            "observed_cells_unknown": int(
                np.count_nonzero(no_anchor_result.cell_raw_return_count)
            ),
            "selected_elevation_cells": int(
                np.isfinite(no_anchor_result.cell_elevation_z).sum()
            ),
        },
        "PASS",
    )

    return {
        "case_count": len(cases),
        "cases": cases,
        "frozen_contract_sha256": hashlib.sha256(
            json.dumps(CASE_CONTRACTS, sort_keys=True).encode()
        ).hexdigest(),
    }, results


def _choose_root_id(
    points,
    *,
    expected_z=None,
    radius_m=8.0,
    x_min=-60.0,
    y_min=-40.0,
    cell_size=0.5,
):
    """Choose one observed raw anchor before any candidate fit or GT access."""
    xyz = np.asarray(points, dtype=float)
    finite = np.isfinite(xyz).all(axis=1)
    radius = np.hypot(xyz[:, 0], xyz[:, 1])
    ids = np.flatnonzero(finite & (radius <= radius_m))
    require(len(ids) > 0, "no finite observed point exists in the frozen anchor radius")
    if expected_z is not None:
        expected = np.abs(xyz[ids, 2] - expected_z) <= 0.30
        if expected.any():
            ids = ids[expected]
    z_bin = np.floor(xyz[ids, 2] / 0.10).astype(np.int64)
    bins, counts = np.unique(z_bin, return_counts=True)
    max_count = int(counts.max())
    modal_bins = bins[counts == max_count]
    if expected_z is not None:
        chosen_bin = min(
            modal_bins,
            key=lambda value: (abs((value + 0.5) * 0.10 - expected_z), value),
        )
    else:
        chosen_bin = int(min(modal_bins))
    support_ids = ids[z_bin == chosen_bin]
    cell_keys = np.floor((xyz[support_ids, :2] - [x_min, y_min]) / cell_size).astype(
        np.int64
    )
    buckets = {}
    for point_id, key in zip(support_ids, cell_keys):
        buckets.setdefault(tuple(map(int, key)), []).append(int(point_id))
    supported = [key for key, point_rows in buckets.items() if len(point_rows) >= 3]
    require(
        supported, "no raw 0.5m anchor cell has three returns in the frozen modal layer"
    )
    key = min(
        supported,
        key=lambda cell: (
            math.hypot(
                x_min + (cell[0] + 0.5) * cell_size,
                y_min + (cell[1] + 0.5) * cell_size,
            ),
            cell,
        ),
    )
    point_rows = buckets[key]
    center = np.array(
        [
            x_min + (key[0] + 0.5) * cell_size,
            y_min + (key[1] + 0.5) * cell_size,
        ]
    )
    distances = np.sum((xyz[point_rows, :2] - center) ** 2, axis=1)
    nearest = min(
        range(len(point_rows)),
        key=lambda i: (distances[i], point_rows[i]),
    )
    return int(point_rows[nearest])


def _root_layer_support(result, root_id):
    root_id = int(root_id)
    row = int(np.flatnonzero(result.point_ids == root_id)[0])
    cell = int(result.point_cell_id[row])
    ix, iy = divmod(cell, result.cell_status.shape[1])
    layer_id = int(result.point_layer_id[row])
    for layer in result.cell_observed_layers[ix, iy]:
        if int(layer.layer_id) == layer_id:
            return int(layer.point_count)
    return 0


def _terrain_config(source_config, *, raw_anchor_z=None):
    if source_config is None:
        return TerrainSurfaceConfig(
            x_min=-60.0,
            x_max=60.0,
            y_min=-40.0,
            y_max=40.0,
            cell_size=0.5,
            expected_ground_z=None,
            seed_height_band=0.30,
            anchor_radius=8.0,
            layer_gap=0.20,
            ground_tolerance=0.10,
            max_slope_gradient=0.30,
            max_neighbor_height_change=CONTINUITY_STEP_M,
        )
    return TerrainSurfaceConfig(
        x_min=float(source_config.min_x),
        x_max=float(source_config.max_x),
        y_min=float(source_config.min_y),
        y_max=float(source_config.max_y),
        cell_size=0.5,
        expected_ground_z=float(source_config.expected_ground_z),
        seed_height_band=float(source_config.ground_seed_band),
        anchor_radius=8.0,
        layer_gap=0.20,
        ground_tolerance=float(source_config.ground_tolerance),
        max_slope_gradient=float(source_config.ground_max_slope),
        max_neighbor_height_change=CONTINUITY_STEP_M,
    )


def _radial_config(source_config, *, raw_anchor_z=None):
    if source_config is not None:
        return source_config
    return SimpleNamespace(
        min_x=-60.0,
        max_x=60.0,
        min_y=-40.0,
        max_y=40.0,
        expected_ground_z=float(raw_anchor_z),
        base_frame="semantickitti-native-velodyne-unverified-base",
        ground_seed_band=0.30,
        ground_max_slope=0.30,
        ground_max_residual=0.08,
        ground_tolerance=0.10,
        obstacle_min_height=0.10,
        clearance_height=2.0,
        resolution=0.5,
    )


def _risk_inputs(out, point_count):
    mask = np.zeros(point_count, dtype=bool)
    sources = {}
    for source_name in ("object", "unknown"):
        ids = sorted(
            {
                int(point_id)
                for entity in getattr(out, source_name)
                for point_id in entity.point_id
            }
        )
        require(
            all(0 <= point_id < point_count for point_id in ids),
            "native risk ID outside source points",
        )
        mask[ids] = True
        sources[source_name] = np.asarray(ids, dtype=np.uint32)
    cells = [
        cell
        for cell in out.space_map.cell
        if cell.occupancy_obs or cell.occupancy_det or cell.occupancy_unknown
    ]
    risk_cells = np.array(
        [[cell.x_index, cell.y_index] for cell in cells], dtype=np.int32
    ).reshape(-1, 2)
    risk_flags = np.array(
        [
            [
                bool(cell.occupancy_obs),
                bool(cell.occupancy_det),
                bool(cell.occupancy_unknown),
            ]
            for cell in cells
        ],
        dtype=bool,
    ).reshape(-1, 3)
    sources["native_risk_cells_xy"] = risk_cells
    sources["native_risk_cell_flags"] = risk_flags
    return mask, sources


def _rooted_connectivity(
    points, selected_ground, result, elevation_values=None, root_id=None
):
    """Count observed selected-ground cells from the preselected result root."""
    config = result.config
    xyz = np.asarray(points, dtype=float)
    ids = np.asarray(result.point_ids, dtype=np.int64)
    require(
        np.array_equal(ids, np.arange(len(xyz))),
        "point IDs no longer align to raw order",
    )
    mask = np.asarray(selected_ground, dtype=bool)
    cell_ids = np.asarray(result.point_cell_id, dtype=np.int64)
    selected_ids = np.flatnonzero(mask & (cell_ids >= 0))
    keys = np.floor(
        (xyz[selected_ids, :2] - [config.x_min, config.y_min]) / config.cell_size
    ).astype(np.int64)
    keys = {
        tuple(map(int, key))
        for key in keys
        if 0 <= key[0] < result.cell_status.shape[0]
        and 0 <= key[1] < result.cell_status.shape[1]
    }
    if root_id is None:
        root = (
            tuple(map(int, np.asarray(result.root_cell).reshape(-1)))
            if result.root_cell is not None
            else None
        )
    else:
        root_point = xyz[int(root_id), :2]
        root = tuple(
            np.floor(
                (root_point - [config.x_min, config.y_min]) / config.cell_size
            ).astype(int)
        )
    if root is None or len(root) != 2 or root not in keys:
        return {
            "root_cell": list(root) if root is not None else None,
            "root_selected_ground_observed": False,
            "connected_cells": 0,
            "connected_area_m2": 0.0,
            "unknown_cells": int(
                np.count_nonzero(
                    np.char.startswith(result.cell_status.astype(str), "unknown")
                )
            ),
        }
    if elevation_values is None:
        elevations = np.asarray(result.cell_elevation_z)
    else:
        elevations = np.full(result.cell_status.shape, np.nan)
        grouped = {}
        for point_id in np.flatnonzero(mask):
            key = tuple(
                np.floor(
                    (xyz[point_id, :2] - [config.x_min, config.y_min])
                    / config.cell_size
                ).astype(int)
            )
            z_value = float(elevation_values[point_id])
            if (
                np.isfinite(z_value)
                and 0 <= key[0] < result.cell_status.shape[0]
                and 0 <= key[1] < result.cell_status.shape[1]
            ):
                grouped.setdefault(key, []).append(z_value)
        for key, values in grouped.items():
            elevations[key] = float(np.median(values))
    connected = {root}
    frontier = [root]
    while frontier:
        ix, iy = frontier.pop()
        for neighbor in ((ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)):
            if neighbor not in keys or neighbor in connected:
                continue
            if not np.isfinite(elevations[neighbor]) or not np.isfinite(
                elevations[ix, iy]
            ):
                continue
            if (
                abs(float(elevations[neighbor] - elevations[ix, iy]))
                > config.max_neighbor_height_change
            ):
                continue
            connected.add(neighbor)
            frontier.append(neighbor)
    return {
        "root_cell": list(root),
        "root_selected_ground_observed": True,
        "connected_cells": len(connected),
        "connected_area_m2": len(connected) * config.cell_size**2,
        "observed_selected_ground_cells": len(keys),
        "unknown_cells": int(
            np.count_nonzero(
                np.char.startswith(result.cell_status.astype(str), "unknown")
            )
        ),
        "graph_is_observed_only": True,
        "gap_filled": False,
    }


def _persist_frame(
    path,
    points,
    result,
    risk_mask,
    risk_sources,
    radial_z,
    radial_mask,
    requested_root_id,
    input_provenance,
):
    arrays = _surface_metrics(result)
    arrays.update(_layer_arrays(result))
    arrays.update(
        {
            "point_id": np.asarray(result.point_ids, dtype=np.uint32),
            "xyz_m": np.asarray(points, dtype=np.float64),
            "point_risk_input": np.asarray(risk_mask, dtype=bool),
            "radial_ground_z_m": np.asarray(radial_z, dtype=np.float64),
            "radial_ground_mask": np.asarray(radial_mask, dtype=bool),
            "root_anchor_requested_id": np.asarray([requested_root_id], dtype=np.int64),
            "root_anchor_selected_id": np.asarray(
                [-1 if result.root_anchor_id is None else result.root_anchor_id],
                dtype=np.int64,
            ),
            "root_cell_requested": np.floor(
                (
                    points[requested_root_id, :2]
                    - [result.config.x_min, result.config.y_min]
                )
                / result.config.cell_size
            ).astype(np.int32),
            "root_cell_selected": np.asarray(
                [-1, -1] if result.root_cell is None else result.root_cell,
                dtype=np.int32,
            ).reshape(2),
            "input_provenance_json": np.asarray(
                json.dumps(input_provenance, sort_keys=True, allow_nan=False)
            ),
        }
    )
    for key, value in risk_sources.items():
        arrays[key] = np.asarray(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)
    with np.load(path, allow_pickle=False) as stored:
        require(set(stored.files) == set(arrays), "persisted terrain field set changed")
        for name, expected in arrays.items():
            got = stored[name]
            equal = (
                np.array_equal(got, expected, equal_nan=True)
                if got.dtype.kind in "fc"
                else np.array_equal(got, expected)
            )
            require(
                equal,
                f"persisted terrain array differs after round trip: {name}",
            )
    return arrays, digest(path)


def _metrics_from_arrays(arrays, config):
    ground = np.asarray(arrays["point_ground_mask"], dtype=bool)
    candidate = np.asarray(arrays["point_ground_candidate_mask"], dtype=bool)
    risk = np.asarray(arrays["point_risk_mask"], dtype=bool)
    require(
        np.array_equal(ground, candidate & ~risk),
        "risk eligibility changed candidate fit",
    )
    status = np.asarray(arrays["cell_status"]).astype(str)
    return {
        "original_point_count": int(len(arrays["point_id"])),
        "selected_ground_points": int(ground.sum()),
        "candidate_ground_points": int(candidate.sum()),
        "independent_risk_points": int(risk.sum()),
        "selected_ground_over_risk_points": int(np.count_nonzero(ground & risk)),
        "supported_cells": int(np.count_nonzero(status == "supported")),
        "unknown_cells": int(np.count_nonzero(np.char.startswith(status, "unknown"))),
        "other_layer_cells": int(np.count_nonzero(status == "other_layer")),
        "raw_roughness_available_cells": int(
            np.isfinite(arrays["cell_roughness_m"]).sum()
        ),
        "slope_available_cells": int(np.isfinite(arrays["cell_slope_gradient"]).sum()),
        "step_available_cells": int(np.isfinite(arrays["cell_step_m"]).sum()),
        "pit_measured_cells": int(np.count_nonzero(arrays["pit_status"] == "measured")),
        "pit_unknown_cells": int(
            np.count_nonzero(
                np.isin(
                    arrays["pit_status"],
                    (
                        "unknown_no_floor",
                        "unknown_no_rim",
                        "unknown_insufficient_floor",
                        "unknown_layer_ambiguity",
                    ),
                )
            )
        ),
    }


def _label_metrics(points, arrays, label_path, expected_hash):
    require(digest(label_path) == expected_hash, f"changed semantic GT: {label_path}")
    packed = np.fromfile(label_path, dtype="<u4")
    require(len(packed) == len(points), "semantic GT point count mismatch")
    truth = learning_labels(packed)
    in_roi = np.asarray(arrays["point_cell_id"]) >= 0
    ground_truth = np.isin(truth, GROUND) & in_roi
    candidate_prediction = (
        np.asarray(arrays["point_ground_candidate_mask"], dtype=bool) & in_roi
    )
    risk = np.asarray(arrays["point_risk_mask"], dtype=bool) & in_roi
    prediction = np.asarray(arrays["point_ground_mask"], dtype=bool) & in_roi
    tp = int(np.count_nonzero(ground_truth & prediction))
    fp = int(np.count_nonzero(~ground_truth & prediction & in_roi & (truth != 0)))
    fn = int(np.count_nonzero(ground_truth & ~prediction))
    thing = np.isin(truth, np.arange(1, 9)) & in_roi
    radius = np.hypot(points[:, 0], points[:, 1])
    by_range = {}
    for low, high in RADIAL_BANDS:
        inside = in_roi & (radius >= low) & (radius < high)
        gt_band = ground_truth & inside
        pred_band = prediction & inside & (truth != 0)
        band_tp = int(np.count_nonzero(gt_band & prediction))
        band_fp = int(np.count_nonzero(pred_band & ~ground_truth))
        band_fn = int(np.count_nonzero(gt_band & ~prediction))
        key = f"{low:g}-{high:g}m" if math.isfinite(high) else f"{low:g}m-plus"
        by_range[key] = {
            "tp": band_tp,
            "fp_valid_semantic": band_fp,
            "fn": band_fn,
            "precision": band_tp / (band_tp + band_fp) if band_tp + band_fp else None,
            "recall": band_tp / (band_tp + band_fn) if band_tp + band_fn else None,
        }
    coplanar_thing = (
        thing
        & prediction
        & np.isfinite(arrays["point_residual_z"])
        & (np.abs(arrays["point_residual_z"]) <= 0.10)
    )
    return {
        "semantic_ground_proxy": {
            "tp": tp,
            "fp_valid_semantic": fp,
            "fn": fn,
            "precision": tp / (tp + fp) if tp + fp else None,
            "recall": tp / (tp + fn) if tp + fn else None,
            "valid_gt_points_in_roi": int(np.count_nonzero((truth != 0) & in_roi)),
            "ignored_gt_points_in_roi": int(np.count_nonzero((truth == 0) & in_roi)),
            "by_radial_range": by_range,
            "candidate_ground_removed_by_independent_risk": int(
                np.count_nonzero(candidate_prediction & risk & ~prediction)
            ),
        },
        "intrinsic_coplanar_semantic_thing_ambiguity": {
            "thing_points_in_roi": int(thing.sum()),
            "predicted_ground_coplanar_thing_points": int(coplanar_thing.sum()),
            "interpretation": (
                (
                    "Geometry-only terrain fitting "
                    + "cannot identify a coplanar semantic "
                    + "thing; this count is an "
                    + "observability ambiguity, not a "
                    + "physical pit/ground accuracy claim."
                )
            ),
        },
        "truth_read_after_all_predictions": True,
        "ground_truth_is_semantic_proxy_not_physical_terrain_truth": True,
    }


def _iter_cached_sources(root, reader, hashes):
    for run in CACHED_RUNS:
        run = {
            "name": run["name"],
            "inputs": root / run["inputs"],
            "results": root / run["results"],
        }
        manifest_path = run["inputs"] / "manifest.json"
        hashes[str(manifest_path)] = digest(manifest_path)
        manifest = json.loads(manifest_path.read_text())
        require(
            (
                len(manifest["samples"]) == 12
                if "semantickitti" in run["name"]
                else len(manifest["samples"]) == 4
            ),
            f"unexpected frozen sample count: {run['name']}",
        )
        for sample in manifest["samples"]:
            yield run, sample, manifest


def _cached_archive_frame(root, reader, run, sample, hashes, history):
    archive_path = (
        root
        / ".cache/lidar-fusion-validation/continuation-v3"
        / run["name"]
        / f"{sample['file']}.ground.npz"
    )
    report_path = root / ".cache/lidar-fusion-validation/continuation-v3/report.json"
    archive_sha = digest(archive_path)
    history_row = next(
        row
        for row in history["frames"]
        if row["case"] == run["name"] and row["file"] == sample["file"]
    )
    require(
        history_row["algorithm_artifact"] == str(archive_path.relative_to(root))
        and history_row["algorithm_artifact_sha256"] == archive_sha,
        f"archived point source changed from v3 report: {archive_path}",
    )
    require(
        history_row["raw_sha256"] == sample["source_sha256"],
        "frozen manifest and historical raw-source digest disagree",
    )
    with np.load(archive_path, allow_pickle=False) as archive:
        point_ids = np.asarray(archive["point_id"])
        points = np.asarray(archive["xyz_m"], dtype=np.float64)
        sensor_xyz = np.asarray(archive["sensor_xyz_m"], dtype=np.float64)
        coordinate_frame = str(np.asarray(archive["coordinate_frame"]).item())
    require(
        point_ids.shape == (len(points),)
        and np.array_equal(point_ids, np.arange(len(points), dtype=point_ids.dtype)),
        "archived raw point IDs are not original-order IDs",
    )
    require(
        points.shape == sensor_xyz.shape == (len(points), 3),
        "archived original XYZ arrays have inconsistent shapes",
    )

    input_path = run["inputs"] / sample["file"]
    config_path = run["results"] / "config.pb"
    native_path = run["results"] / (sample["file"] + ".fusion.pb")
    require(
        digest(input_path) == sample["sha256"], f"changed cached input: {input_path}"
    )
    input_sha = digest(input_path)
    config_sha = digest(config_path)
    native_sha = digest(native_path)
    cloud = reader.read(input_path, "apollo.drivers.PointCloud")
    source_config = reader.read(config_path, "apollo.lidar_fusion.FusionConfig")
    out = reader.read(native_path, "apollo.lidar_fusion.FusionOutput")
    require_full_evidence(out)
    audit_frame(out, source_config)
    require(
        len(cloud.point) == sample["points"] == len(out.point_evidence) == len(points),
        "cached cloud/native/archive point counts disagree",
    )
    require(
        coordinate_frame == source_config.base_frame == out.header.frame_id,
        "archived XYZ coordinate frame does not match cached native output frame",
    )
    require(
        cloud.frame_id == source_config.sensor_frame
        and out.calibration_version == source_config.sensor_to_base.version,
        "cached cloud/native calibration identity mismatch",
    )
    c = source_config.sensor_to_base
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
    cloud_xyz = np.array([[p.x, p.y, p.z] for p in cloud.point], dtype=np.float64)
    native_xyz = np.array(
        [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence],
        dtype=np.float64,
    )
    finite = np.array([p.finite for p in out.point_evidence], dtype=bool)
    require(
        np.array_equal(sensor_xyz, cloud_xyz, equal_nan=True),
        "archived sensor XYZ differs from cached cloud point order",
    )
    require(
        np.allclose(
            (cloud_xyz @ rotation.T + [c.x, c.y, c.z])[finite],
            native_xyz[finite],
            rtol=0,
            atol=1e-9,
        ),
        "cached cloud/native transform or point order mismatch",
    )
    native_xyz[~finite] = np.nan
    require(
        np.array_equal(points, native_xyz, equal_nan=True),
        "archived estimator XYZ differs from cached native point evidence",
    )
    hashes[str(input_path)] = input_sha
    hashes[str(config_path)] = config_sha
    hashes[str(native_path)] = native_sha
    hashes[str(archive_path)] = archive_sha
    hashes[str(report_path)] = digest(report_path)
    return (
        out,
        source_config,
        points,
        {
            "kind": "archived_original_points_fallback",
            "archive_path": str(archive_path.relative_to(root)),
            "archive_sha256": archive_sha,
            "historical_report_path": str(report_path.relative_to(root)),
            "historical_report_sha256": hashes[str(report_path)],
            "source_point_arrays_used": ["point_id", "xyz_m"],
            "source_point_arrays_not_used": [
                "all prior ground/terrain prediction arrays",
                "sensor_xyz_m (audit only, matched to frozen cloud)",
            ],
            "point_id_order_verified": True,
            "sensor_xyz_matches_cached_cloud": True,
            "xyz_matches_cached_native_frame": True,
            "raw_mount_source_path": sample["source"],
            "raw_mount_expected_sha256": sample["source_sha256"],
            "raw_mount_source_status": (
                "BLOCKED_MISSING; no " "archive/raw-binary equality claim"
            ),
        },
    )


def _run_cached_predictions(
    root, schema_path, hashes, persisted_rows, *, allow_archived_raw=False
):
    reader = Reader(schema_path)
    hashes[str(schema_path)] = digest(schema_path)
    history_path = root / ".cache/lidar-fusion-validation/continuation-v3/report.json"
    history = json.loads(history_path.read_text())
    hashes[str(history_path)] = digest(history_path)
    rows = []
    for run, sample, manifest in _iter_cached_sources(root, reader, hashes):
        native_path = run["results"] / (sample["file"] + ".fusion.pb")
        native_before = digest(native_path)
        if Path(sample["source"]).is_file():
            out, source_config, points, _ = source_frame(reader, run, sample, hashes)
            input_provenance = {
                "kind": "verified_raw_source",
                "raw_source_path": sample["source"],
                "raw_source_sha256": sample["source_sha256"],
                "raw_source_hash_verified": True,
            }
        else:
            require(
                allow_archived_raw,
                (
                    f"raw source unavailable: {sample['source']}; "
                    "rerun with --allow-archived-raw to opt into archived XYZ"
                ),
            )
            out, source_config, points, input_provenance = _cached_archive_frame(
                root, reader, run, sample, hashes, history
            )
        raw_before = out.SerializeToString(deterministic=True)
        risk_mask, risk_sources = _risk_inputs(out, len(points))
        terrain_template = _terrain_config(source_config)
        root_id = _choose_root_id(
            points,
            expected_z=float(source_config.expected_ground_z),
            x_min=terrain_template.x_min,
            y_min=terrain_template.y_min,
            cell_size=terrain_template.cell_size,
        )
        config = _terrain_config(source_config)
        estimate = estimate_terrain_surface(
            points,
            point_ids=np.arange(len(points), dtype=np.uint32),
            risk_mask=risk_mask,
            root_anchor_id=root_id,
            config=config,
        )
        require(
            estimate.root_anchor_id is None or int(estimate.root_anchor_id) == root_id,
            "terrain estimator changed frozen root ID",
        )
        root_support = _root_layer_support(estimate, root_id)
        root_accepted = estimate.root_anchor_id is not None
        root_support_valid = root_support >= 3 and root_accepted
        require(
            np.array_equal(estimate.point_ids, np.arange(len(points))),
            "terrain estimator changed original point ID order",
        )
        require(
            np.array_equal(estimate.point_risk_mask, risk_mask),
            "terrain estimator changed independent native risk eligibility",
        )
        radial_z = radial_ground(points, source_config, RADIAL_CONTROL)
        radial_mask = np.isfinite(radial_z) & (
            np.abs(points[:, 2] - radial_z) <= source_config.ground_tolerance
        )
        output_path = (
            FRAME_ROOT / run["name"] / f"{Path(sample['file']).stem}.terrain.npz"
        )
        arrays, artifact_hash = _persist_frame(
            output_path,
            points,
            estimate,
            risk_mask,
            risk_sources,
            radial_z,
            radial_mask,
            root_id,
            input_provenance,
        )
        metrics = _metrics_from_arrays(arrays, config)
        continuity = _rooted_connectivity(
            points, arrays["point_ground_mask"], estimate, root_id=root_id
        )
        native_ground = np.array(
            [point.is_ground_geo for point in out.point_evidence], dtype=bool
        )
        native_z = np.array(
            [
                point.z_ground if point.ground_valid else np.nan
                for point in out.point_evidence
            ],
            dtype=float,
        )
        native_continuity = _rooted_connectivity(
            points, native_ground, estimate, elevation_values=native_z, root_id=root_id
        )
        radial_continuity = _rooted_connectivity(
            points, radial_mask, estimate, elevation_values=radial_z, root_id=root_id
        )
        native_after = digest(native_path)
        require(native_before == native_after, "cached native producer output changed")
        require(
            raw_before == out.SerializeToString(deterministic=True),
            "native FusionOutput mutated during terrain estimation",
        )
        row = {
            "dataset": (
                "SemanticKITTI" if "semantickitti" in run["name"] else "nuScenes"
            ),
            "case": run["name"],
            "file": sample["file"],
            "source_path": sample["source"],
            "source_sha256": sample["source_sha256"],
            "source_sha256_status": input_provenance["kind"],
            "raw_source_sha256_verified": input_provenance.get(
                "raw_source_hash_verified", False
            ),
            "archive_input_provenance": input_provenance,
            "input_cloud_path": str(run["inputs"] / sample["file"]),
            "input_cloud_sha256": sample["sha256"],
            "labels_path": sample.get("labels"),
            "labels_sha256": sample.get("labels_sha256"),
            "points": len(points),
            "coordinate_frame": out.header.frame_id,
            "native_sensor_frame": source_config.sensor_frame,
            "estimator_frame": source_config.base_frame,
            "sensor_to_base_transform": {
                key: getattr(source_config.sensor_to_base, key)
                for key in ("x", "y", "z", "roll", "pitch", "yaw", "version")
            },
            "ground_height_assumption_m": float(source_config.expected_ground_z),
            "ground_height_physical_calibration_verified": False,
            "root_anchor": {
                "point_id": root_id,
                "xyz_m": points[root_id].tolist(),
                "selected_layer_raw_return_count": root_support,
                "minimum_root_support_observed": root_support_valid,
                "estimator_accepted_root_id": estimate.root_anchor_id,
                "estimator_overall_status": estimate.overall_status,
                "selection": (
                    "raw-only 0.1m modal-height bin inside 8m, "
                    "nearest-radius then lowest point ID; no candidate, "
                    "label, GT, or risk input"
                ),
            },
            "terrain_config": asdict(config),
            "radial_control_options": asdict(RADIAL_CONTROL),
            "independent_risk_point_ids": {
                name: value.tolist()
                for name, value in risk_sources.items()
                if name in ("object", "unknown")
            },
            "native_risk_cell_count": int(len(risk_sources["native_risk_cells_xy"])),
            "native_occupancy_immutable": True,
            "native_output_sha256_before_after": [native_before, native_after],
            "source_labels_loaded": False,
            "numerical_metrics": metrics,
            "fixed_root_continuity": continuity,
            "same_root_continuity_controls": {
                "native_ground": native_continuity,
                "frozen_radial": radial_continuity,
                "terrain_surface": continuity,
            },
            "artifact_path": str(output_path.relative_to(root)),
            "artifact_sha256": artifact_hash,
            "raw_xyz_is_new_prediction_input": True,
            "old_producer_npz_used_as_prediction": False,
            "archived_xyz_recomputed_by_v4": input_provenance["kind"]
            == "archived_original_points_fallback",
        }
        rows.append(row)
        persisted_rows.append(
            {
                "artifact_path": str(output_path.relative_to(root)),
                "artifact_sha256": artifact_hash,
                "round_trip_equal_all_arrays": True,
                "source_point_ids_preserved": bool(
                    np.array_equal(arrays["point_id"], np.arange(len(points)))
                ),
                "input_provenance": input_provenance,
                "metrics_before": metrics,
                "metrics_after": _metrics_from_arrays(arrays, config),
                "source_hashes_unchanged": None,
            }
        )
    return rows


def _run_non08_predictions(
    root, inventory_path, hashes, persisted_rows, *, allow_archived_raw=False
):
    inventory = json.loads(inventory_path.read_text())
    hashes[str(inventory_path)] = digest(inventory_path)
    require(
        inventory.get("frame_count") == 20, "non-08 frozen inventory is not 20 frames"
    )
    require(inventory.get("sequence_count") == 10, "unexpected non-08 sequence count")
    require(
        inventory["coordinate_and_experiment_boundary"]["algorithm_result"] is False,
        "non-08 selection has prior algorithm predictions",
    )
    history_path = root / ".cache/lidar-fusion-validation/continuation-v3/report.json"
    history = json.loads(history_path.read_text())
    hashes[str(history_path)] = digest(history_path)
    rows = []
    predictions = []
    for sample in inventory["selection_details"]:
        raw_path = Path(sample["raw_scan"])
        previous = next(
            item
            for item in history["cross_sequence_raw_20"]["frames"]
            if item["sequence"] == sample["sequence"]
            and int(item["ordinal"]) == int(sample["ordinal"])
        )
        require(
            previous["raw_sha256"] == sample["raw_sha256"]
            and int(previous["points"]) == int(sample["point_count"]),
            "frozen inventory and historical raw-source record disagree",
        )
        if raw_path.is_file():
            require(
                digest(raw_path) == sample["raw_sha256"],
                f"changed raw source: {raw_path}",
            )
            hashes[str(raw_path)] = sample["raw_sha256"]
            xyz_i = np.fromfile(raw_path, dtype="<f4").reshape(-1, 4)
            points = xyz_i[:, :3].astype(np.float64, copy=True)
            require(
                len(points) == sample["point_count"],
                "non-08 source point count mismatch",
            )
            input_provenance = {
                "kind": "verified_raw_source",
                "raw_source_path": str(raw_path),
                "raw_source_sha256": sample["raw_sha256"],
                "raw_source_hash_verified": True,
                "coordinate_contract": (
                    "first three raw scan float32 columns XYZ; no " "transform applied"
                ),
            }
        else:
            require(
                allow_archived_raw,
                (
                    f"raw source unavailable: {raw_path}; "
                    "rerun with --allow-archived-raw to opt into archived XYZ"
                ),
            )
            archive_path = (
                root
                / ".cache/lidar-fusion-validation/continuation-v3"
                / f"patch-raw-{sample['sequence']}-{int(sample['ordinal']):06d}.npz"
            )
            archive_sha = digest(archive_path)
            require(
                previous["algorithm_artifact"] == str(archive_path.relative_to(root))
                and previous["algorithm_artifact_sha256"] == archive_sha,
                f"raw archive changed from historical v3 report: {archive_path}",
            )
            with np.load(archive_path, allow_pickle=False) as archive:
                point_ids = np.asarray(archive["point_id"])
                points = np.asarray(archive["xyz_m"], dtype=np.float64)
                point_arrays = [
                    np.asarray(archive[name], dtype=np.float64)
                    for name in archive.files
                    if name.endswith("_point_xyz")
                ]
            require(
                point_ids.shape == (len(points),)
                and np.array_equal(
                    point_ids, np.arange(len(points), dtype=point_ids.dtype)
                ),
                "archived non-08 point IDs are not original-order IDs",
            )
            require(
                len(points) == sample["point_count"],
                "archived non-08 point count mismatch",
            )
            require(
                len(point_arrays) == 3
                and all(
                    np.allclose(points, other, rtol=0, atol=1e-6, equal_nan=True)
                    for other in point_arrays
                ),
                "archived non-08 original XYZ arrays disagree",
            )
            hashes[str(archive_path)] = archive_sha
            input_provenance = {
                "kind": "archived_original_points_fallback",
                "archive_path": str(archive_path.relative_to(root)),
                "archive_sha256": archive_sha,
                "historical_report_path": str(history_path.relative_to(root)),
                "historical_report_sha256": hashes[str(history_path)],
                "source_point_arrays_used": ["point_id", "xyz_m"],
                "source_point_arrays_not_used": [
                    "all prior ground/terrain prediction arrays",
                    "height-hypothesis point_xyz arrays (consistency audit only)",
                ],
                "point_id_order_verified": True,
                "xyz_arrays_consistent_within_archive": True,
                "raw_mount_source_path": str(raw_path),
                "raw_mount_expected_sha256": sample["raw_sha256"],
                "raw_mount_source_status": (
                    "BLOCKED_MISSING; no " "archive/raw-binary equality claim"
                ),
                "coordinate_contract": (
                    (
                        "frozen caller input is archived "
                        + "original scan XYZ in sensor-native "
                        + "coordinates; no sensor_xyz array or "
                        + "vehicle transform is used or inferred"
                    )
                ),
            }
        root_id = _choose_root_id(points, x_min=-60.0, y_min=-40.0, cell_size=0.5)
        config = _terrain_config(None, raw_anchor_z=float(points[root_id, 2]))
        risk_mask = np.zeros(len(points), dtype=bool)
        risk_sources = {
            "object": np.empty(0, dtype=np.uint32),
            "unknown": np.empty(0, dtype=np.uint32),
            "native_risk_cells_xy": np.empty((0, 2), dtype=np.int32),
            "native_risk_cell_flags": np.empty((0, 3), dtype=bool),
        }
        estimate = estimate_terrain_surface(
            points,
            point_ids=np.arange(len(points), dtype=np.uint32),
            risk_mask=risk_mask,
            root_anchor_id=root_id,
            config=config,
        )
        require(
            estimate.root_anchor_id is None or int(estimate.root_anchor_id) == root_id,
            "non-08 terrain estimator changed frozen root",
        )
        root_support = _root_layer_support(estimate, root_id)
        root_accepted = estimate.root_anchor_id is not None
        root_support_valid = root_support >= 3 and root_accepted
        radial_cfg = _radial_config(None, raw_anchor_z=float(points[root_id, 2]))
        radial_z = radial_ground(points, radial_cfg, RADIAL_CONTROL)
        radial_mask = np.isfinite(radial_z) & (
            np.abs(points[:, 2] - radial_z) <= radial_cfg.ground_tolerance
        )
        output_path = (
            FRAME_ROOT
            / "semantickitti-non08-raw"
            / f"{sample['sequence']}-{sample['ordinal']:06d}.terrain.npz"
        )
        arrays, artifact_hash = _persist_frame(
            output_path,
            points,
            estimate,
            risk_mask,
            risk_sources,
            radial_z,
            radial_mask,
            root_id,
            input_provenance,
        )
        metrics = _metrics_from_arrays(arrays, config)
        predictions.append(
            {
                "sample": sample,
                "points": points,
                "arrays": arrays,
                "config": config,
                "path": output_path,
            }
        )
        rows.append(
            {
                "dataset": "SemanticKITTI",
                "sequence": sample["sequence"],
                "ordinal": sample["ordinal"],
                "source_path": str(raw_path),
                "source_sha256": sample["raw_sha256"],
                "source_sha256_status": input_provenance["kind"],
                "raw_source_sha256_verified": input_provenance.get(
                    "raw_source_hash_verified", False
                ),
                "archive_input_provenance": input_provenance,
                "point_count": len(points),
                "coordinate_frame": (
                    "native Velodyne sensor coordinates; no raw-to-base "
                    "transform in frozen manifest"
                ),
                "calibration_path": sample["calibration"],
                "calibration_sha256": sample["calibration_sha256"],
                "calibrated_base_height_used": False,
                "sensor_height_hypothesis_sensitivity": (
                    "not required: estimator receives a raw observed "
                    "root point ID; no fixed-height sweep or GT-selected "
                    "height"
                ),
                "root_anchor": {
                    "point_id": root_id,
                    "xyz_m": points[root_id].tolist(),
                    "selected_layer_raw_return_count": root_support,
                    "minimum_root_support_observed": root_support_valid,
                    "estimator_accepted_root_id": estimate.root_anchor_id,
                    "estimator_overall_status": estimate.overall_status,
                    "selection": (
                        "raw-only 0.1m modal-height bin inside 8m, "
                        "nearest-radius then lowest point ID; no label, GT, "
                        "or risk input"
                    ),
                },
                "terrain_config": asdict(config),
                "radial_control_options": asdict(RADIAL_CONTROL),
                "independent_risk_mask": (
                    "geometry-only: no native "
                    "fusion/detector/segmentation risk source exists for "
                    "these 20 frames"
                ),
                "source_labels_loaded": False,
                "numerical_metrics": metrics,
                "fixed_root_continuity": _rooted_connectivity(
                    points, arrays["point_ground_mask"], estimate, root_id=root_id
                ),
                "same_root_continuity_controls": {
                    "frozen_radial": _rooted_connectivity(
                        points,
                        radial_mask,
                        estimate,
                        elevation_values=radial_z,
                        root_id=root_id,
                    ),
                    "terrain_surface": _rooted_connectivity(
                        points, arrays["point_ground_mask"], estimate, root_id=root_id
                    ),
                },
                "artifact_path": str(output_path.relative_to(root)),
                "artifact_sha256": artifact_hash,
                "algorithm_predictions_executed_before_truth": True,
                "old_producer_npz_used_as_prediction": False,
                "physical_pit_accuracy_status": "BLOCKED: no physical pit/bridge GT",
                "archived_xyz_recomputed_by_v4": input_provenance["kind"]
                == "archived_original_points_fallback",
            }
        )
        persisted_rows.append(
            {
                "artifact_path": str(output_path.relative_to(root)),
                "artifact_sha256": artifact_hash,
                "round_trip_equal_all_arrays": True,
                "source_point_ids_preserved": bool(
                    np.array_equal(arrays["point_id"], np.arange(len(points)))
                ),
                "input_provenance": input_provenance,
                "metrics_before": metrics,
                "metrics_after": _metrics_from_arrays(arrays, config),
                "source_hashes_unchanged": None,
            }
        )
    return rows, predictions


def _attach_postfit_truth(rows, predictions, hashes):
    metrics_by_id = {}
    for row in rows:
        if not row.get("labels_path"):
            row["ground_truth_status"] = (
                "BLOCKED: no joined semantic or physical ground GT"
            )
            continue
        if not Path(row["labels_path"]).is_file():
            row["ground_truth_status"] = (
                "BLOCKED: semantic label source unavailable after all predictions; "
                "physical terrain GT unavailable"
            )
            row.pop("_points", None)
            row.pop("_arrays", None)
            continue
        points = row.pop("_points")
        arrays = row.pop("_arrays")
        truth = _label_metrics(
            points, arrays, Path(row["labels_path"]), row["labels_sha256"]
        )
        hashes[row["labels_path"]] = row["labels_sha256"]
        row["postfit_semantic_evaluation"] = truth
        row["ground_truth_status"] = (
            "semantic-ground proxy only; physical surface truth unavailable"
        )
        metrics_by_id[(row["case"], row["file"])] = truth
    for item in predictions:
        sample = item["sample"]
        label_path = Path(sample["labels"])
        if not label_path.is_file():
            item["postfit_semantic_evaluation"] = None
            continue
        require(
            digest(label_path) == sample["labels_sha256"],
            f"changed non-08 labels: {label_path}",
        )
        hashes[str(label_path)] = sample["labels_sha256"]
        truth = _label_metrics(
            item["points"],
            item["arrays"],
            label_path,
            sample["labels_sha256"],
        )
        metrics_by_id[(sample["sequence"], sample["ordinal"])] = truth
        item["postfit_semantic_evaluation"] = truth
    return metrics_by_id


def _score_cached_truth(rows, root, hashes):
    scores = {}
    for row in rows:
        label_path = row.get("labels_path")
        if not label_path:
            row["ground_truth_status"] = (
                "BLOCKED: nuScenes has no ground GT in this experiment"
            )
            row["physical_pit_accuracy_status"] = "BLOCKED: no physical pit/bridge GT"
            continue
        if not Path(label_path).is_file():
            row["ground_truth_status"] = (
                "BLOCKED: semantic label source unavailable after all predictions; "
                "physical terrain GT unavailable"
            )
            row["physical_pit_accuracy_status"] = "BLOCKED: no physical pit/bridge GT"
            continue
        with np.load(root / row["artifact_path"], allow_pickle=False) as arrays:
            truth = _label_metrics(
                arrays["xyz_m"],
                arrays,
                Path(label_path),
                row["labels_sha256"],
            )
        hashes[label_path] = row["labels_sha256"]
        row["postfit_semantic_evaluation"] = truth
        row["ground_truth_status"] = (
            "semantic-ground proxy only; physical surface truth unavailable"
        )
        row["physical_pit_accuracy_status"] = "BLOCKED: no physical pit/bridge GT"
        scores[(row["case"], row["file"])] = truth
    return scores


def _postfit_non08(rows, predictions, hashes):
    by_id = {}
    for item in predictions:
        sample = item["sample"]
        label_path = Path(sample["labels"])
        if not label_path.is_file():
            item["postfit_semantic_evaluation"] = None
            continue
        truth = _label_metrics(
            item["points"],
            item["arrays"],
            label_path,
            sample["labels_sha256"],
        )
        hashes[sample["labels"]] = sample["labels_sha256"]
        by_id[(sample["sequence"], sample["ordinal"])] = truth
    for row in rows:
        key = (row["sequence"], row["ordinal"])
        if key in by_id:
            row["postfit_semantic_evaluation"] = by_id[key]
            row["ground_truth_status"] = (
                "semantic-ground proxy only; not physical terrain truth"
            )
        else:
            row["ground_truth_status"] = (
                "BLOCKED: semantic label source unavailable after all predictions; "
                "physical terrain GT unavailable"
            )
    return by_id


def _execution_summary(rows):
    metrics = [row["numerical_metrics"] for row in rows]
    return {
        "frames": len(rows),
        "archived_input_frames": sum(
            row.get("archived_xyz_recomputed_by_v4", False) for row in rows
        ),
        "raw_source_hash_verified_frames": sum(
            row.get("raw_source_sha256_verified", False) for row in rows
        ),
        "overall_status_counts": {
            status: sum(
                row["root_anchor"]["estimator_overall_status"] == status for row in rows
            )
            for status in sorted(
                {row["root_anchor"]["estimator_overall_status"] for row in rows}
            )
        },
        "root_support_valid_frames": sum(
            row["root_anchor"]["minimum_root_support_observed"] for row in rows
        ),
        "original_points": sum(item["original_point_count"] for item in metrics),
        "selected_ground_points": sum(
            item["selected_ground_points"] for item in metrics
        ),
        "candidate_ground_points": sum(
            item["candidate_ground_points"] for item in metrics
        ),
        "independent_risk_points": sum(
            item["independent_risk_points"] for item in metrics
        ),
        "supported_cells": sum(item["supported_cells"] for item in metrics),
        "unknown_cells": sum(item["unknown_cells"] for item in metrics),
        "slope_available_cells": sum(item["slope_available_cells"] for item in metrics),
        "step_available_cells": sum(item["step_available_cells"] for item in metrics),
        "pit_measured_cells": sum(item["pit_measured_cells"] for item in metrics),
        "pit_unknown_cells": sum(item["pit_unknown_cells"] for item in metrics),
        "root_connected_cells": sum(
            row["fixed_root_continuity"]["connected_cells"] for row in rows
        ),
        "ground_truth_evidence": {
            status: sum(row.get("ground_truth_status") == status for row in rows)
            for status in sorted(
                {row.get("ground_truth_status", "not_scored") for row in rows}
            )
        },
    }


def run_real_data(root=ROOT, inventory_path=None, *, allow_archived_raw=False):
    root = Path(root).resolve()
    schema_path = root / ".cache/lidar-fusion-validation/next-iteration/schema.pb"
    if inventory_path is None:
        inventory_path = root / (
            ".cache/lidar-fusion-validation/three-role-validation-20261031/"
            + "raw-non08-gt-inventory.json"
        )
    inventory_path = Path(inventory_path).resolve()
    hashes = {
        str(path): digest(path)
        for path in (
            Path(__file__),
            Path(__file__).with_name("terrain_surface.py"),
            Path(__file__).with_name("ground_experiment.py"),
            Path(__file__).with_name("ground_patch_experiment.py"),
            Path(__file__).with_name("proto.py"),
            Path(__file__).with_name("evaluate_baseline.py"),
            Path(__file__).with_name("semantickitti.py"),
        )
    }
    persisted_rows = []
    started = time.monotonic()
    cached_rows = _run_cached_predictions(
        root,
        schema_path,
        hashes,
        persisted_rows,
        allow_archived_raw=allow_archived_raw,
    )
    non08_rows, non08_predictions = _run_non08_predictions(
        root,
        inventory_path,
        hashes,
        persisted_rows,
        allow_archived_raw=allow_archived_raw,
    )
    require(
        len(cached_rows) == 36, f"expected 36 cached frames, got {len(cached_rows)}"
    )
    require(
        len(non08_rows) == 20, f"expected 20 non-08 raw frames, got {len(non08_rows)}"
    )

    # Labels are opened only after all 56 new terrain estimates and NPZ round trips.
    cached_scores = _score_cached_truth(cached_rows, root, hashes)
    non08_scores = _postfit_non08(non08_rows, non08_predictions, hashes)

    for path, expected in list(hashes.items()):
        require(
            digest(Path(path)) == expected,
            f"source changed during terrain experiment: {path}",
        )
    for item in persisted_rows:
        item["observed_artifact_hashes_unchanged"] = True
        item["raw_mount_source_hash_verified"] = item["input_provenance"].get(
            "raw_source_hash_verified", False
        )
        require(
            item["metrics_before"] == item["metrics_after"],
            f"round-trip metrics mismatch for {item['artifact_path']}",
        )
    persist_payload = {
        "diagnostic_only": True,
        "frame_artifact_count": len(persisted_rows),
        "input_frame_count": 56,
        "all_point_ids_original_order": all(
            item["source_point_ids_preserved"] for item in persisted_rows
        ),
        "all_npz_round_trips_equal": all(
            item["round_trip_equal_all_arrays"] for item in persisted_rows
        ),
        "all_round_trip_metrics_equal": all(
            item["metrics_before"] == item["metrics_after"] for item in persisted_rows
        ),
        "all_observed_artifact_hashes_unchanged": True,
        "raw_mount_source_hash_audit_status": (
            "PASS"
            if all(item["raw_mount_source_hash_verified"] for item in persisted_rows)
            else (
                "BLOCKED: original raw mount binaries unavailable; "
                "archived hashes do not substitute"
            )
        ),
        "old_artifacts_modified": False,
        "frames": persisted_rows,
        "source_sha256": hashes,
    }
    _write(PERSIST_FILE, persist_payload)

    report = {
        "diagnostic_only": True,
        "execution_status": (
            "PASS_ARCHIVED_INPUT_RECOMPUTED"
            if len(cached_rows) == 36 and len(non08_rows) == 20
            else "FAIL_INCOMPLETE_EXECUTION"
        ),
        "archived_raw_fallback_opt_in": bool(allow_archived_raw),
        "production_mutated": False,
        "FREE_emitted": False,
        "planning_or_vehicle_qualified": False,
        "prediction_source": (
            (
                "new estimate_terrain_surface "
                + "recomputed from original-order "
                + "XYZ/point IDs. When raw mount "
                + "binaries are absent, only archived "
                + "raw XYZ fields are read; prior "
                + "terrain prediction arrays are never "
                + "loaded as predictions."
            )
        ),
        "input_counts": {
            "cached_frames": len(cached_rows),
            "cached_semantickitti_sequence_08": sum(
                row["dataset"] == "SemanticKITTI" for row in cached_rows
            ),
            "cached_nuscenes": sum(row["dataset"] == "nuScenes" for row in cached_rows),
            "non08_raw_semantickitti_frames": len(non08_rows),
            "non08_sequences": sorted({row["sequence"] for row in non08_rows}),
        },
        "intended_input_counts": {
            "cached_frames": 36,
            "non08_raw_semantickitti_frames": 20,
            "total_frames": 56,
        },
        "raw_mount_source_hash_audit": {
            "status": (
                "PASS"
                if all(
                    row.get("raw_source_sha256_verified", False)
                    for row in cached_rows + non08_rows
                )
                else "BLOCKED"
            ),
            "verified_frame_count": sum(
                row.get("raw_source_sha256_verified", False)
                for row in cached_rows + non08_rows
            ),
            "blocked_frame_count": sum(
                not row.get("raw_source_sha256_verified", False)
                for row in cached_rows + non08_rows
            ),
            "blocked_sources": [
                {
                    "path": row["archive_input_provenance"].get(
                        "raw_mount_source_path", row.get("source_path")
                    ),
                    "expected_sha256": row["archive_input_provenance"].get(
                        "raw_mount_expected_sha256", row.get("source_sha256")
                    ),
                    "archive_path": row["archive_input_provenance"].get("archive_path"),
                    "archive_sha256": row["archive_input_provenance"].get(
                        "archive_sha256"
                    ),
                }
                for row in cached_rows + non08_rows
                if not row.get("raw_source_sha256_verified", False)
            ],
            "identity_claim": (
                "none: archive digest/order/correspondence do not "
                "prove equality to unavailable original raw binary"
            ),
        },
        "per_dataset_execution_summary": {
            "cached_semantickitti_seq08": _execution_summary(
                [row for row in cached_rows if row["dataset"] == "SemanticKITTI"]
            ),
            "cached_nuscenes": _execution_summary(
                [row for row in cached_rows if row["dataset"] == "nuScenes"]
            ),
            "non08_semantickitti_geometry_only": _execution_summary(non08_rows),
        },
        "risk_eligibility_audit": {
            "cached_native_risk_input_unchanged": all(
                row["native_occupancy_immutable"] for row in cached_rows
            ),
            "cached_independent_risk_point_ids_preserved": all(
                row["native_occupancy_immutable"] and not row["source_labels_loaded"]
                for row in cached_rows
            ),
            "non08_risk_source": (
                "none available; all 20 frames are geometry-only and "
                "contain no inferred object risk"
            ),
            "coplanar_semantic_thing_limit": (
                "cannot distinguish coplanar semantic things from "
                "terrain from geometry alone"
            ),
        },
        "selection_limitations": {
            "cached_36": (
                "fixed previously inspected 24 sequence-08 frames "
                "plus 12 nuScenes samples; correlated, not an "
                "independent holdout"
            ),
            "non08_20": (
                "two frames per sequence from 00-07,09,10 selected "
                "by frozen paired-stem quartiles; source inventory "
                "only, not previously run by this estimator"
            ),
            "nuScenes_ground_truth": (
                "no ground labels or physical pit/bridge truth; "
                "mechanism observability only"
            ),
            "non08_frame": (
                "raw sensor frame; no verified sensor-to-base "
                "transform or height calibration"
            ),
        },
        "frozen_controls": {
            "radial_options": asdict(RADIAL_CONTROL),
            "terrain_config_by_frame": (
                "source ROI and native ground "
                "seed-height/slope/residual/tolerance values for the "
                "cached frames; fixed 0.5m cells and 0.20m max "
                "neighbor change"
            ),
            "common_root": (
                "one raw-only point ID per frame selected before "
                "fit; same root used for connected region graph; no "
                "candidate-area, label, GT, or risk-based selection"
            ),
            "risk_gate": (
                "native object/unknown point IDs supplied separately "
                "on cached frames and risk layers persisted "
                "unchanged; geometry-only raw20 has no native risk "
                "source"
            ),
        },
        "cached_frames": cached_rows,
        "raw_non08_frames": non08_rows,
        "aggregate_semantic_ground_proxy": {
            "cached_semantickitti_frames": len(
                [key for key in cached_scores if key[0].startswith("semantickitti")]
            ),
            "non08_frames": len(non08_scores),
            "interpretation": (
                "SemanticKITTI semantic ground is a classification "
                "proxy, not physical terrain truth. Coplanar "
                "semantic thing observations are reported as "
                "intrinsic geometry ambiguity."
            ),
        },
        "physical_terrain_accuracy": {
            "status": "BLOCKED",
            "reason": (
                "No joined physical pit/bridge GT is present in the "
                "selected recordings; analytic fixtures provide math "
                "evidence only."
            ),
        },
        "elapsed_seconds": time.monotonic() - started,
        "source_sha256": hashes,
        "artifact_status": (
            "new v4 artifacts only; all prior reports and NPZs " "were read-only"
        ),
    }
    _write(REAL_FILE, report)
    return report, persist_payload


def run_all(root=ROOT, *, real_data=True, allow_archived_raw=False):
    matrix, _ = run_analytic_cases()
    _write(CASE_FILE, matrix)
    if not real_data:
        return matrix
    report, persist = run_real_data(root=root, allow_archived_raw=allow_archived_raw)
    record_cases = [
        {
            "id": "cached-36-original-point-records",
            "objective": (
                "Execute the new numerical surface on all fixed "
                "cached raw-point frames."
            ),
            "expected": (
                "36 fresh v4 calls from verified raw inputs or "
                "hash-verified archived original XYZ; point order "
                "and native risk preserved; same raw-selected root; "
                "no FREE."
            ),
            "actual": {
                "frames": report["input_counts"]["cached_frames"],
                "artifact_count": persist["frame_artifact_count"],
                "archived_xyz_recomputed": report["per_dataset_execution_summary"][
                    "cached_semantickitti_seq08"
                ]["archived_input_frames"]
                + report["per_dataset_execution_summary"]["cached_nuscenes"][
                    "archived_input_frames"
                ],
                "original_mount_hash_verified": report["raw_mount_source_hash_audit"][
                    "verified_frame_count"
                ],
            },
            "status": (
                "PASS" if report["input_counts"]["cached_frames"] == 36 else "FAIL"
            ),
            "real_record_effect": (
                "fresh v4 recomputation on archived original points; "
                "semantic-ground proxy only; no physical pit GT"
            ),
            "truth_level_ambiguity": (
                "coplanar semantic THING vs terrain is not " "geometrically observable"
            ),
            "source_hash": hashlib.sha256(
                json.dumps(report["source_sha256"], sort_keys=True).encode()
            ).hexdigest(),
            "first_wrong_boundary": (
                (
                    "raw mount source hash unavailable; "
                    "archive/cached-cloud correspondence validated "
                    "separately"
                )
                if report["raw_mount_source_hash_audit"]["status"] == "BLOCKED"
                else None
            ),
        },
        {
            "id": "raw-non08-20-original-records",
            "objective": (
                "Execute the new numerical surface on 20 frozen "
                "non-08 raw scans without using GT as estimator input."
            ),
            "expected": (
                "20 fresh v4 calls from verified raw scans or "
                "hash-verified archived original XYZ; original-order "
                "IDs; one raw-only fixed root each; geometry-only risk."
            ),
            "actual": {
                "frames": report["input_counts"]["non08_raw_semantickitti_frames"],
                "sequences": report["input_counts"]["non08_sequences"],
                "archived_xyz_recomputed": report["per_dataset_execution_summary"][
                    "non08_semantickitti_geometry_only"
                ]["archived_input_frames"],
                "original_mount_hash_verified": report["raw_mount_source_hash_audit"][
                    "verified_frame_count"
                ],
            },
            "status": (
                "PASS"
                if report["input_counts"]["non08_raw_semantickitti_frames"] == 20
                else "FAIL"
            ),
            "real_record_effect": (
                "fresh geometry-only v4 recomputation on archived "
                "original coordinates; no verified base calibration"
            ),
            "truth_level_ambiguity": (
                "coplanar thing class cannot be separated from "
                "terrain by geometry alone; physical pit accuracy "
                "blocked"
            ),
            "source_hash": hashlib.sha256(
                json.dumps(report["source_sha256"], sort_keys=True).encode()
            ).hexdigest(),
            "first_wrong_boundary": (
                (
                    "raw mount source hash unavailable; archive "
                    "digest/order audited without raw-file identity claim"
                )
                if report["raw_mount_source_hash_audit"]["status"] == "BLOCKED"
                else None
            ),
        },
        {
            "id": "raw-mount-source-hash-audit",
            "objective": (
                "Verify original raw binary SHA256 for all 56 " "selected frames."
            ),
            "expected": (
                "All 56 original source files mounted and matching "
                "their frozen expected SHA256."
            ),
            "actual": report["raw_mount_source_hash_audit"],
            "status": report["raw_mount_source_hash_audit"]["status"],
            "real_record_effect": (
                "separate provenance audit; does not negate fresh "
                "archive-input execution"
            ),
            "truth_level_ambiguity": (
                "archive NPZ digest and cached cloud/native "
                "correspondence cannot establish unavailable raw "
                "binary identity"
            ),
            "source_hash": hashlib.sha256(
                json.dumps(report["source_sha256"], sort_keys=True).encode()
            ).hexdigest(),
            "first_wrong_boundary": (
                "source path absent in managed dev mount"
                if report["raw_mount_source_hash_audit"]["status"] == "BLOCKED"
                else None
            ),
        },
        {
            "id": "physical-pit-bridge-accuracy",
            "objective": (
                "Validate pit depth/width and bridge surface against "
                "physical ground truth."
            ),
            "expected": "Requires physical surface GT co-registered to the raw frame.",
            "actual": {"status": report["physical_terrain_accuracy"]["status"]},
            "status": "BLOCKED",
            "real_record_effect": "not validated by current records",
            "truth_level_ambiguity": report["physical_terrain_accuracy"]["reason"],
            "source_hash": hashlib.sha256(
                json.dumps(report["source_sha256"], sort_keys=True).encode()
            ).hexdigest(),
            "first_wrong_boundary": "physical GT unavailable",
        },
    ]
    matrix["real_record_cases"] = record_cases
    matrix["real_record_artifact"] = str(REAL_FILE.relative_to(Path(root)))
    _write(CASE_FILE, matrix)
    return matrix


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--analytic-only", action="store_true")
    parser.add_argument(
        "--allow-archived-raw",
        action="store_true",
        help=(
            "opt in to recomputing from archived original XYZ "
            "when raw source files are unavailable"
        ),
    )
    args = parser.parse_args(argv)
    run_all(
        args.root,
        real_data=not args.analytic_only,
        allow_archived_raw=args.allow_archived_raw,
    )


if __name__ == "__main__":
    main()
