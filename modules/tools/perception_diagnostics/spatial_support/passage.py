# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Directional rectangle swept-path diagnostics using separating axes."""

import math

import numpy as np
from evaluate_baseline import require
from spatial_support.terrain import INFERRED, MEASURED, UNKNOWN_RISK
from spatial_support.trajectory import sample_path


def touched(scene, center, yaw, half_length, half_width):
    """Closed rectangle/cell-square overlap, including diagonal contacts."""
    c, s = math.cos(yaw), math.sin(yaw)
    extent = np.array(
        [
            abs(c) * half_length + abs(s) * half_width,
            abs(s) * half_length + abs(c) * half_width,
        ]
    )
    low = np.ceil((center - extent - scene.origin) / scene.resolution).astype(int) - 1
    high = np.floor((center + extent - scene.origin) / scene.resolution).astype(int)
    shape = np.asarray(scene.shape)
    outside = bool(np.any(low < 0) or np.any(high >= shape))
    low, high = np.maximum(low, 0), np.minimum(high, shape - 1)
    if np.any(low > high):
        return np.empty((0, 2), dtype=int), True
    keys = np.array(
        [(x, y) for x in range(low[0], high[0] + 1) for y in range(low[1], high[1] + 1)]
    )
    delta = (keys + 0.5) * scene.resolution + scene.origin - center
    half_cell = scene.resolution / 2
    overlap = (
        (np.abs(delta[:, 0]) <= extent[0] + half_cell)
        & (np.abs(delta[:, 1]) <= extent[1] + half_cell)
        & (np.abs(delta @ [c, s]) <= half_length + half_cell * (abs(c) + abs(s)))
        & (np.abs(delta @ [-s, c]) <= half_width + half_cell * (abs(c) + abs(s)))
    )
    return keys[overlap], outside


def query(scene, path, vehicle, yaw=None):
    """Fixed heading per segment; heading changes include a rotational envelope.

    Translation is conservatively inflated by half the sample gap. For actual
    steering motions a time/heading trajectory adapter is still required. XY
    refers to the rectangle center, not a qualified rear-axle reference.
    """
    vehicle.validate()
    path = np.asarray(path, dtype=float)
    sample_path(path, scene.resolution / 2)
    require(yaw is None or np.isfinite(yaw), "invalid explicit path heading")
    vertices = []
    for point in path:
        if vertices and np.array_equal(point, vertices[-1]):
            continue
        while len(vertices) >= 2:
            incoming = vertices[-1] - vertices[-2]
            outgoing = point - vertices[-1]
            # Exact forward collinearity only: no turn-angle tolerance may hide
            # a genuine heading change (including a reversal).
            cross = incoming[0] * outgoing[1] - incoming[1] * outgoing[0]
            if cross != 0 or np.dot(incoming, outgoing) <= 0:
                break
            vertices.pop()
        vertices.append(point)
    path = np.asarray(vertices)
    results = []
    arc_offset = 0.0
    half_length = vehicle.length / 2 + vehicle.margin
    half_width = vehicle.width / 2 + vehicle.margin
    radius = math.hypot(half_length, half_width)

    def add(center, heading, distance, length, width, corner=False):
        keys, outside = touched(scene, center, heading, length, width)
        idx = tuple(keys.T)
        bits = scene.layers[idx] if len(keys) else np.empty(0, np.uint8)
        reasons = []
        for name, bit in (
            ("measured", MEASURED),
            ("inferred", INFERRED),
            ("unknown_risk", UNKNOWN_RISK),
        ):
            if np.any(bits & bit):
                reasons.append(name)
        masks = {
            "missing_ground": ~np.isfinite(scene.ground_z),
            "missing_terrain": ~(
                np.isfinite(scene.slope)
                & np.isfinite(scene.step)
                & np.isfinite(scene.roughness)
            ),
            "disconnected_ground": ~scene.ego_connected,
            "terrain_limit": scene.terrain_limit,
            "unverified_free": ~scene.verified_free,
        }
        for name, mask in masks.items():
            if len(keys) and np.any(mask[idx]):
                reasons.append(name)
        if outside:
            reasons.append("roi_boundary")
        state = (
            "OBSTACLE_RISK"
            if np.any(bits)
            else (
                "TERRAIN_LIMIT"
                if "terrain_limit" in reasons
                else "UNKNOWN" if reasons else "SUPPORTED_ONLY"
            )
        )
        results.append(
            {
                "xy": center.tolist(),
                "yaw": float(heading),
                "arc_m": float(distance),
                "state": state,
                "reasons": reasons,
                "touched_cells": len(keys),
                "corner_rotation_envelope": corner,
            }
        )

    if len(path) == 1:
        require(yaw is not None, "single-pose directional query requires heading")
        add(path[0], yaw, 0, half_length, half_width)
    for i, (a, b) in enumerate(zip(path[:-1], path[1:])):
        delta = b - a
        heading = float(yaw) if yaw is not None else math.atan2(delta[1], delta[0])
        xy, arc, gap = sample_path([a, b], scene.resolution / 2)
        for center, distance in zip(xy, arc):
            add(
                center,
                heading,
                arc_offset + distance,
                half_length + gap / 2,
                half_width + gap / 2,
            )
        arc_offset += arc[-1]
        if i + 1 < len(path) - 1 and yaw is None:
            # A square bounding all intermediate headings is conservative.
            add(b, 0, arc_offset, radius, radius, True)
    return {
        "diagnostic_only": True,
        "production_qualified": False,
        "collision_free_authorized": False,
        "geometry": "directional rectangle with conservative segment/corner envelopes",
        "state_counts": {
            state: sum(s["state"] == state for s in results)
            for state in sorted({s["state"] for s in results})
        },
        "samples": results,
    }
