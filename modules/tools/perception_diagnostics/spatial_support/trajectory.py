# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Conservative polyline swept-disk risk queries, never vehicle control."""

import math
from dataclasses import dataclass

import numpy as np
from evaluate_baseline import require
from spatial_support.clearance import cell_distance, distances


@dataclass(frozen=True)
class Vehicle:
    length: float
    width: float
    margin: float

    def validate(self):
        require(
            all(math.isfinite(v) for v in (self.length, self.width, self.margin))
            and self.length > 0
            and self.width > 0
            and self.margin >= 0,
            "invalid explicit diagnostic vehicle footprint",
        )


def sample_path(path, spacing):
    path = np.asarray(path, dtype=float)
    require(
        path.ndim == 2
        and path.shape[1] == 2
        and 0 < len(path) <= 2000
        and np.isfinite(path).all()
        and np.isfinite(spacing)
        and spacing > 0,
        "invalid trajectory polyline",
    )
    samples = [path[0]]
    arc = [0.0]
    maximum_gap = 0.0
    for a, b in zip(path[:-1], path[1:]):
        length = float(np.linalg.norm(b - a))
        count = max(1, math.ceil(length / spacing))
        require(len(samples) + count <= 20000, "trajectory sample budget exceeded")
        maximum_gap = max(maximum_gap, length / count)
        for k in range(1, count + 1):
            samples.append(a + (b - a) * k / count)
            arc.append(arc[-1] + length / count)
    return np.asarray(samples), np.asarray(arc), maximum_gap


def query(scene, path, vehicle):
    vehicle.validate()
    xy, arc, gap = sample_path(path, scene.resolution / 2)
    # Every segment point is within gap/2 of a sample; cover all headings with
    # the circumscribed vehicle disk and explicitly include interpolation error.
    radius = math.hypot(vehicle.length, vehicle.width) / 2 + vehicle.margin
    swept_radius = radius + gap / 2
    metrics = distances(scene, xy)
    metrics["unverified_free"] = cell_distance(scene, xy, ~scene.verified_free)
    reasons = []
    for i in range(len(xy)):
        flags = [name for name, values in metrics.items() if values[i] <= swept_radius]
        reasons.append(flags)
    blocked = {"measured", "inferred", "unknown_risk"}
    states = [
        (
            "OBSTACLE_RISK"
            if blocked.intersection(flags)
            else (
                "TERRAIN_LIMIT"
                if "terrain_limit" in flags
                else "UNKNOWN" if flags else "SUPPORTED_ONLY"
            )
        )
        for flags in reasons
    ]
    first = next(
        (i for i, state in enumerate(states) if state != "SUPPORTED_ONLY"), None
    )
    return {
        "diagnostic_only": True,
        "production_qualified": False,
        "collision_free_authorized": False,
        "physical_footprint_radius_m": radius,
        "swept_query_radius_m": swept_radius,
        "first_risk_arc_m": float(arc[first]) if first is not None else None,
        "state_counts": {state: states.count(state) for state in sorted(set(states))},
        "samples": [
            {
                "xy": xy[i].tolist(),
                "arc_m": float(arc[i]),
                "state": states[i],
                "reasons": reasons[i],
                "distance_m": {
                    name: float(values[i]) if np.isfinite(values[i]) else None
                    for name, values in metrics.items()
                },
            }
            for i in range(len(xy))
        ],
    }
