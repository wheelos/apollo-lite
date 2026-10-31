# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Observed elevation and local plane diagnostics on the native grid."""

from dataclasses import dataclass

import numpy as np
from evaluate_baseline import require

MEASURED = 1
INFERRED = 2
UNKNOWN_RISK = 4


@dataclass
class Scene:
    resolution: float
    origin: np.ndarray
    layers: np.ndarray
    verified_free: np.ndarray
    ground_z: np.ndarray
    slope: np.ndarray
    step: np.ndarray
    roughness: np.ndarray
    terrain_limit: np.ndarray
    ego_connected: np.ndarray

    @property
    def shape(self):
        return self.layers.shape


def build_scene(
    points,
    ground_z,
    ground,
    resolution,
    origin,
    layers,
    verified_free,
    max_slope,
    max_step,
    max_roughness,
    ego_connected=None,
):
    """Fit local planes to observed cell medians; unsupported values stay NaN.

    Step is the maximum observed cardinal height difference. Roughness is
    the local median-height plane residual, not point noise or vehicle proof.
    """
    require(
        np.isfinite(resolution)
        and resolution > 0
        and layers.ndim == 2
        and 0 < layers.size <= 1000000
        and np.isfinite(origin).all()
        and verified_free.shape == layers.shape,
        "invalid bounded spatial grid",
    )
    require(
        all(np.isfinite(v) and v >= 0 for v in (max_slope, max_step, max_roughness)),
        "invalid diagnostic terrain thresholds",
    )
    points, ground_z, ground = map(np.asarray, (points, ground_z, ground))
    require(
        points.shape == (len(ground), 3) and ground_z.shape == ground.shape,
        "ground point alignment mismatch",
    )
    connected = (
        np.zeros(layers.shape, dtype=bool)
        if ego_connected is None
        else np.asarray(ego_connected)
    )
    require(
        connected.shape == layers.shape and connected.dtype == bool,
        "invalid observed connected-region mask",
    )
    elevation = np.full(layers.shape, np.nan)
    buckets = {}
    ids = np.flatnonzero(
        ground & np.isfinite(points).all(axis=1) & np.isfinite(ground_z)
    )
    keys = np.floor((points[ids, :2] - origin) / resolution).astype(np.int64)
    for i, key in zip(ids, keys):
        if np.all(key >= 0) and np.all(key < layers.shape):
            buckets.setdefault(tuple(key), []).append(ground_z[i])
    for key, values in buckets.items():
        elevation[key] = np.median(values)
    slope, step, roughness = (np.full(layers.shape, np.nan) for _ in range(3))
    width, height = layers.shape
    for x, y in buckets:
        neighbors = []
        differences = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                xx, yy = x + dx, y + dy
                if 0 <= xx < width and 0 <= yy < height:
                    z = elevation[xx, yy]
                    if np.isfinite(z):
                        neighbors.append((dx * resolution, dy * resolution, z))
                        if abs(dx) + abs(dy) == 1:
                            differences.append(abs(z - elevation[x, y]))
        if differences:
            step[x, y] = max(differences)
        if len(neighbors) >= 3:
            samples = np.asarray(neighbors)
            design = np.column_stack((samples[:, :2], np.ones(len(samples))))
            coefficients, _, rank, _ = np.linalg.lstsq(
                design, samples[:, 2], rcond=None
            )
            if rank == 3:
                slope[x, y] = np.hypot(*coefficients[:2])
                roughness[x, y] = np.max(np.abs(design @ coefficients - samples[:, 2]))
    limited = (slope > max_slope) | (step > max_step) | (roughness > max_roughness)
    return Scene(
        resolution,
        np.asarray(origin),
        layers.copy(),
        verified_free.copy(),
        elevation,
        slope,
        step,
        roughness,
        limited,
        connected.copy() & np.isfinite(elevation),
    )
