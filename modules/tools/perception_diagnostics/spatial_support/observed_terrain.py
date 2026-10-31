# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Raw observed point-elevation and fitted-plane terrain diagnostics."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ObservedTerrainConfig:
    """Minimum observed raw point count for sample-standard-deviation metrics."""

    min_raw_points: int = 2

    def validate(self):
        if not isinstance(self.min_raw_points, int) or self.min_raw_points < 2:
            raise ValueError("min_raw_points must be at least two")


@dataclass
class ObservedTerrainMetrics:
    """Per-patch metrics computed from original observed points.

    The ``all_return_*`` standard deviations include every finite observed
    return, including obstacles; they are not terrain roughness. The
    ``ground_candidate_*`` statistic conditions on the estimator's observed
    point labels and is likewise not terrain GT. All are point-level sample
    statistics, not ETH-style aggregation of cell-elevation samples.
    """

    raw_elevation_median: np.ndarray
    all_return_height_std: np.ndarray
    all_return_plane_distance_std: np.ndarray
    ground_candidate_plane_distance_std: np.ndarray
    slope_gradient: np.ndarray
    step: np.ndarray
    raw_return_count: np.ndarray
    ground_candidate_count: np.ndarray
    availability: np.ndarray
    plane_supported: np.ndarray
    status: np.ndarray
    config: ObservedTerrainConfig


def observed_terrain_metrics(
    points, ground_estimate, *, config=ObservedTerrainConfig()
):
    """Compute observed return-spread and independent slope/step/availability.

    All-return height statistics use every finite original XYZ point in each
    patch, including non-ground returns. Ground-candidate plane-distance
    statistics use only original rows labeled by ``ground_mask``. Both sets
    are selection-conditioned observations, not physical terrain truth.
    ``step`` is the maximum absolute height disagreement of adjacent accepted
    fitted planes evaluated at their shared boundary midpoint. This removes
    the continuous plane's expected height change from the discontinuity
    metric. Unsupported patches or patches without a supported neighbor are
    NaN; no gap is filled.
    """
    config.validate()
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] not in (3, 4):
        raise ValueError("expected Nx3 XYZ or Nx4 XYZI frame-local points")
    xyz = values[:, :3]
    if xyz.shape != ground_estimate.point_xyz.shape or not np.array_equal(
        xyz, ground_estimate.point_xyz, equal_nan=True
    ):
        raise ValueError("points must match the GroundEstimate input and ordering")
    patch_config = ground_estimate.config
    nx, ny = patch_config.shape
    raw_median = np.full((nx, ny), np.nan)
    all_return_height_std = np.full((nx, ny), np.nan)
    all_return_plane_distance_std = np.full((nx, ny), np.nan)
    ground_candidate_plane_distance_std = np.full((nx, ny), np.nan)
    slope = np.full((nx, ny), np.nan)
    step = np.full((nx, ny), np.nan)
    counts = np.zeros((nx, ny), dtype=np.int64)
    availability = np.zeros((nx, ny), dtype=bool)
    plane_supported = ground_estimate.patch_supported.copy()
    status = np.full((nx, ny), "unavailable", dtype="<U40")

    finite = np.isfinite(xyz).all(axis=1)
    ids = np.flatnonzero(finite & (ground_estimate.cell_id >= 0))
    for point_id in ids:
        patch_id = int(ground_estimate.cell_id[point_id])
        ix, iy = divmod(patch_id, ny)
        counts[ix, iy] += 1
    availability = counts > 0
    for ix, iy in zip(*np.nonzero(availability)):
        patch_ids = ids[ground_estimate.cell_id[ids] == ix * ny + iy]
        raw_z = xyz[patch_ids, 2]
        raw_median[ix, iy] = float(np.median(raw_z))
        if len(raw_z) >= config.min_raw_points:
            all_return_height_std[ix, iy] = float(np.std(raw_z, ddof=1))
        if not plane_supported[ix, iy]:
            status[ix, iy] = "raw_observed_plane_unsupported"
            continue
        a, b, c = ground_estimate.plane_coefficients[ix, iy]
        signed_distance = (
            xyz[patch_ids, 2] - (a * xyz[patch_ids, 0] + b * xyz[patch_ids, 1] + c)
        ) / math.sqrt(1.0 + a * a + b * b)
        if len(signed_distance) >= config.min_raw_points:
            all_return_plane_distance_std[ix, iy] = float(
                np.std(signed_distance, ddof=1)
            )
        ground_ids = patch_ids[ground_estimate.ground_mask[patch_ids]]
        if len(ground_ids) >= config.min_raw_points:
            ground_candidate_distance = (
                xyz[ground_ids, 2]
                - (a * xyz[ground_ids, 0] + b * xyz[ground_ids, 1] + c)
            ) / math.sqrt(1.0 + a * a + b * b)
            ground_candidate_plane_distance_std[ix, iy] = float(
                np.std(ground_candidate_distance, ddof=1)
            )
        slope[ix, iy] = math.hypot(a, b)
        status[ix, iy] = "supported"

    def plane_at(ix, iy, x, y):
        a, b, c = ground_estimate.plane_coefficients[ix, iy]
        return a * x + b * y + c

    # Compare each supported pair at the same shared-boundary midpoint.
    for ix, iy in zip(*np.nonzero(plane_supported)):
        differences = []
        for dx, dy in ((1, 0), (0, 1)):
            xx, yy = ix + dx, iy + dy
            if xx < nx and yy < ny and plane_supported[xx, yy]:
                if dx:
                    x = patch_config.x_min + xx * patch_config.patch_size
                    y = patch_config.y_min + (iy + 0.5) * patch_config.patch_size
                else:
                    x = patch_config.x_min + (ix + 0.5) * patch_config.patch_size
                    y = patch_config.y_min + yy * patch_config.patch_size
                difference = abs(plane_at(ix, iy, x, y) - plane_at(xx, yy, x, y))
                differences.append(difference)
                step[ix, iy] = (
                    difference
                    if not np.isfinite(step[ix, iy])
                    else max(step[ix, iy], difference)
                )
                step[xx, yy] = (
                    difference
                    if not np.isfinite(step[xx, yy])
                    else max(step[xx, yy], difference)
                )

    return ObservedTerrainMetrics(
        raw_elevation_median=raw_median,
        all_return_height_std=all_return_height_std,
        all_return_plane_distance_std=all_return_plane_distance_std,
        ground_candidate_plane_distance_std=ground_candidate_plane_distance_std,
        slope_gradient=slope,
        step=step,
        raw_return_count=counts,
        ground_candidate_count=np.bincount(
            ground_estimate.cell_id[ground_estimate.ground_mask & finite],
            minlength=nx * ny,
        ).reshape(nx, ny),
        availability=availability,
        plane_supported=plane_supported,
        status=status,
        config=config,
    )
