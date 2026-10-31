# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""One-way immutable native FusionOutput to diagnostic numerical scene."""

import numpy as np
from evaluate_baseline import audit_frame, require
from ground_experiment import obstacle_mask
from proto import require_full_evidence
from spatial_support.terrain import INFERRED, MEASURED, UNKNOWN_RISK, build_scene


def scenes(
    out, config, estimate, candidate, max_step, connected_keys, connected_resolution
):
    require_full_evidence(out)
    audit_frame(out, config)
    grid = out.space_map
    require(
        grid.HasField("resolution")
        and grid.resolution > 0
        and 0 < grid.width * grid.height <= 1000000,
        "invalid native grid",
    )
    points = np.array(
        [[p.position.x, p.position.y, p.position.z] for p in out.point_evidence]
    ).reshape(-1, 3)
    finite = np.array([p.finite for p in out.point_evidence], dtype=bool)
    points[~finite] = np.nan
    baseline = np.array([p.is_ground_geo for p in out.point_evidence], dtype=bool)
    base_z = np.array(
        [p.z_ground if p.ground_valid else np.nan for p in out.point_evidence]
    )
    require(
        estimate.shape == candidate.shape == baseline.shape
        and np.array_equal(
            candidate,
            np.isfinite(estimate)
            & (np.abs(points[:, 2] - estimate) <= config.ground_tolerance),
        ),
        "ground artifact identity/classification mismatch",
    )
    layers = np.zeros((grid.width, grid.height), dtype=np.uint8)
    verified_free = np.zeros(layers.shape, dtype=bool)
    for cell in grid.cell:
        key = (cell.x_index, cell.y_index)
        require(np.all(np.asarray(key) < layers.shape), "native cell outside ROI")
        layers[key] = (
            MEASURED * cell.occupancy_obs
            + INFERRED * cell.occupancy_det
            + UNKNOWN_RISK * cell.occupancy_unknown
        )
        verified_free[key] = (
            cell.traversability in (2, 3)
            and cell.ground_supported
            and cell.physically_traversable
            and cell.reachable
            and not cell.visibility_approximate
            and not layers[key]
        )
    noise = np.array([d.explanation == 2 for d in out.point_decision], dtype=bool)
    candidate_layers = layers & np.uint8(INFERRED | UNKNOWN_RISK)
    obstacle = obstacle_mask(points, estimate, candidate, noise, config)
    keys = np.floor(
        (points[obstacle, :2] - [grid.origin_x, grid.origin_y]) / grid.resolution
    ).astype(np.int64)
    require(
        np.all(keys >= 0) and np.all(keys < layers.shape),
        "candidate point outside native grid",
    )
    candidate_layers[keys[:, 0], keys[:, 1]] |= MEASURED
    origin = np.array([grid.origin_x, grid.origin_y])
    require(
        np.isfinite(connected_resolution) and connected_resolution > 0,
        "invalid connected-region resolution",
    )
    connected_masks = {}
    for name, ground in (("baseline", baseline), ("candidate", candidate)):
        region = np.asarray(connected_keys[name])
        require(
            region.ndim == 2 and region.shape[1] == 2, "invalid connected-region keys"
        )
        selected = set(map(tuple, region))
        ids = np.flatnonzero(ground & finite)
        coarse = np.floor(points[ids, :2] / connected_resolution).astype(np.int64)
        fine = np.floor((points[ids, :2] - origin) / grid.resolution).astype(np.int64)
        support = {}
        for key, coarse_key in zip(fine, coarse):
            if np.all(key >= 0) and np.all(key < layers.shape):
                key = tuple(key)
                support[key] = support.get(key, True) and tuple(coarse_key) in selected
        mask = np.zeros(layers.shape, dtype=bool)
        for key, connected in support.items():
            mask[key] = connected
        connected_masks[name] = mask
    native = build_scene(
        points,
        base_z,
        baseline,
        grid.resolution,
        origin,
        layers,
        verified_free,
        config.ground_max_slope,
        max_step,
        config.ground_max_residual,
        connected_masks["baseline"],
    )
    proposed = build_scene(
        points,
        estimate,
        candidate,
        grid.resolution,
        origin,
        candidate_layers,
        # A hypothetical ground change cannot inherit native FREE authorization.
        np.zeros(layers.shape, dtype=bool),
        config.ground_max_slope,
        max_step,
        config.ground_max_residual,
        connected_masks["candidate"],
    )
    return {"baseline": native, "candidate": proposed}
