# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Exact XY distance to independent occupied cell rectangles, within the ROI."""

import numpy as np
from evaluate_baseline import require
from spatial_support.terrain import INFERRED, MEASURED, UNKNOWN_RISK


def cell_distance(scene, xy, mask):
    """Distance to closed occupied cell squares, not their centers.

    An empty layer has no observed barrier; infinity is not free-space evidence.
    Chunking bounds temporary memory without changing Euclidean distances.
    """
    require(mask.shape == scene.shape, "clearance mask shape mismatch")
    centers = (np.argwhere(mask) + 0.5) * scene.resolution + scene.origin
    distances = np.full(len(xy), np.inf)
    for start in range(0, len(xy), 64):
        for first in range(0, len(centers), 2048):
            delta = np.maximum(
                np.abs(
                    xy[start : start + 64, None, :]
                    - centers[None, first : first + 2048, :]
                )
                - scene.resolution / 2,
                0,
            )
            distances[start : start + 64] = np.minimum(
                distances[start : start + 64],
                np.sqrt(np.sum(delta**2, axis=2)).min(axis=1),
            )
    return distances


def distances(scene, xy):
    xy = np.asarray(xy, dtype=float)
    require(
        xy.ndim == 2
        and xy.shape[1] == 2
        and len(xy) <= 20000
        and np.isfinite(xy).all(),
        "invalid bounded clearance queries",
    )
    high = scene.origin + np.asarray(scene.shape) * scene.resolution
    boundary = np.minimum(xy - scene.origin, high - xy).min(axis=1)
    return {
        "measured": cell_distance(scene, xy, scene.layers & MEASURED != 0),
        "inferred": cell_distance(scene, xy, scene.layers & INFERRED != 0),
        "unknown_risk": cell_distance(scene, xy, scene.layers & UNKNOWN_RISK != 0),
        "missing_ground": cell_distance(scene, xy, ~np.isfinite(scene.ground_z)),
        "disconnected_ground": cell_distance(scene, xy, ~scene.ego_connected),
        "missing_terrain": cell_distance(
            scene,
            xy,
            ~(
                np.isfinite(scene.slope)
                & np.isfinite(scene.step)
                & np.isfinite(scene.roughness)
            ),
        ),
        "terrain_limit": cell_distance(scene, xy, scene.terrain_limit),
        "roi_boundary": np.maximum(boundary, 0),
    }
