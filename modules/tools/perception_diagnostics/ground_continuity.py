# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Observed ego-centered ground connectivity, never free-space completion."""

import math
from dataclasses import dataclass
from itertools import product

import numpy as np
from evaluate_baseline import require


@dataclass(frozen=True)
class ContinuityOptions:
    resolution: float = 0.5
    seed_radius: float = 3.0
    max_seed_radius: float = 8.0
    max_height_step: float = 0.2
    voxel_size: float = 0.1
    max_isolated_points: int = 3

    def validate(self):
        require(
            all(
                math.isfinite(v) and v > 0
                for v in (
                    self.resolution,
                    self.seed_radius,
                    self.max_seed_radius,
                    self.max_height_step,
                    self.voxel_size,
                )
            )
            and isinstance(self.max_isolated_points, int)
            and self.max_seed_radius >= self.seed_radius
            and 0 <= self.max_isolated_points <= 10,
            "invalid ground continuity options",
        )


def sparse_obstacles(points, obstacle, options=ContinuityOptions()):
    """Keep voxel-connected components except isolated <=N *original* points.

    Multiple returns in one voxel are one spatial sample but retain their raw
    count: downsampling must not turn a dense narrow obstacle into an outlier.
    """
    options.validate()
    ids = np.flatnonzero(obstacle & np.isfinite(points).all(axis=1))
    keep = np.zeros(len(points), dtype=bool)
    if not len(ids):
        return keep, {"input_voxels": 0, "removed_points": 0, "removed_components": 0}
    keys, inverse, counts = np.unique(
        np.floor(points[ids] / options.voxel_size).astype(np.int64),
        axis=0,
        return_inverse=True,
        return_counts=True,
    )
    lookup = {tuple(key): i for i, key in enumerate(keys)}
    remaining = set(lookup)
    accepted = np.zeros(len(keys), dtype=bool)
    removed_components = 0
    offsets = [
        offset for offset in product((-1, 0, 1), repeat=3) if offset != (0, 0, 0)
    ]
    for root in lookup:
        if root not in remaining:
            continue
        remaining.remove(root)
        pending, component = [root], []
        while pending:
            key = pending.pop()
            component.append(lookup[key])
            for dx, dy, dz in offsets:
                neighbor = (key[0] + dx, key[1] + dy, key[2] + dz)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    pending.append(neighbor)
        if counts[component].sum() > options.max_isolated_points:
            accepted[component] = True
        else:
            removed_components += 1
    keep[ids] = accepted[inverse]
    return keep, {
        "input_voxels": len(keys),
        "removed_points": int(len(ids) - keep.sum()),
        "removed_components": removed_components,
    }


def grid_ids(points, mask, resolution):
    ids = np.flatnonzero(mask & np.isfinite(points).all(axis=1))
    return ids, np.floor(points[ids, :2] / resolution).astype(np.int64)


def connected_ground(points, ground_z, ground, obstacle, options=ContinuityOptions()):
    """Four-neighbor, height-compatible observed cells; no hole filling.

    Select the largest component touching the near-ego seed disk. If blind,
    use the nearest observed ring, bounded by max_seed_radius. Neither case
    creates support beneath the ego blind spot or joins separate sides.
    Coordinates are base-frame XY; a sensor-frame dataset is diagnostic only.
    """
    options.validate()
    ids, keys = grid_ids(points, ground, options.resolution)
    buckets = {}
    for i, key in zip(ids, keys):
        if np.isfinite(ground_z[i]):
            buckets.setdefault(tuple(key), []).append(float(ground_z[i]))
    heights = {key: float(np.median(z)) for key, z in buckets.items()}
    _, blocked_keys = grid_ids(points, obstacle, options.resolution)
    blocked = set(map(tuple, blocked_keys))
    remaining = set(heights) - blocked
    nearest = min(
        (
            math.hypot(
                (key[0] + 0.5) * options.resolution,
                (key[1] + 0.5) * options.resolution,
            )
            for key in remaining
        ),
        default=None,
    )
    seed_radius = options.seed_radius
    if nearest is not None and seed_radius < nearest <= options.max_seed_radius:
        seed_radius = min(options.max_seed_radius, nearest + options.resolution / 2)
    components, seeded = [], []
    for root in sorted(remaining):
        if root not in remaining:
            continue
        remaining.remove(root)
        pending, component, anchored = [root], set(), False
        while pending:
            key = pending.pop()
            component.add(key)
            anchored |= (
                math.hypot(
                    (key[0] + 0.5) * options.resolution,
                    (key[1] + 0.5) * options.resolution,
                )
                <= seed_radius
            )
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                other = key[0] + dx, key[1] + dy
                if (
                    other in remaining
                    and abs(heights[key] - heights[other]) <= options.max_height_step
                ):
                    remaining.remove(other)
                    pending.append(other)
        components.append(component)
        if anchored:
            seeded.append(component)
    selected = max(seeded, key=len) if seeded else set()
    report = {
        "observed_ground_cells": len(heights),
        "ground_cells_blocked_by_obstacles": len(set(heights) & blocked),
        "components": len(components),
        "near_ego_components": len(seeded),
        "ego_connected_cells": len(selected),
        "ego_connected_area_m2": len(selected) * options.resolution**2,
        "seed_observed": bool(seeded),
        "nearest_unblocked_ground_m": nearest,
        "effective_seed_radius_m": seed_radius,
        "seed_expanded_outside_blind_zone": seed_radius > options.seed_radius,
        "connected_fraction": len(selected) / len(heights) if heights else None,
    }
    for radius in (5, 10, 20, 30):
        inside = {
            key
            for key in heights
            if math.hypot(
                (key[0] + 0.5) * options.resolution,
                (key[1] + 0.5) * options.resolution,
            )
            <= radius
        }
        report[f"within_{radius}m"] = {
            "observed_ground_cells": len(inside),
            "connected_cells": len(inside & selected),
        }
    return report, selected


def instance_retention(points, identity, thing, baseline, candidate, voxel_size):
    """All baseline-supported instances, with the old >=3-voxel cohort retained.

    Retention means the same original support survived, not that replacement
    points elsewhere in an instance compensate for lost obstacle evidence.
    """
    ids = np.unique(identity[thing & baseline & (identity != 0)])
    supported, lost, all_lost, partial, replaced, rows = 0, [], [], [], [], []
    for instance in ids:
        member = thing & (identity == instance)
        old = np.unique(
            np.floor(points[member & baseline] / voxel_size).astype(np.int64), axis=0
        )
        new = np.unique(
            np.floor(points[member & candidate] / voxel_size).astype(np.int64), axis=0
        )
        retained = np.unique(
            np.floor(points[member & baseline & candidate] / voxel_size).astype(
                np.int64
            ),
            axis=0,
        )
        instance = int(instance)
        rows.append(
            {
                "instance_id": instance,
                "baseline_points": int((member & baseline).sum()),
                "candidate_points": int((member & candidate).sum()),
                "retained_baseline_points": int((member & baseline & candidate).sum()),
                "baseline_voxels": len(old),
                "candidate_voxels": len(new),
                "retained_baseline_voxels": len(retained),
                "lost_baseline_voxels": len(old) - len(retained),
            }
        )
        if not len(new):
            all_lost.append(instance)
        elif len(retained) < len(old):
            partial.append(instance)
            if not len(retained):
                replaced.append(instance)
        if len(old) >= 3:
            supported += 1
            if not len(new):
                lost.append(instance)
    return {
        "baseline_supported_instances": supported,
        "newly_completely_lost_instances": len(lost),
        "lost_instance_ids": lost,
        "legacy_cohort": ">=3 baseline voxels; not the all-instance denominator",
        "all_baseline_instances": len(ids),
        "sparse_baseline_instances": len(ids) - supported,
        "all_completely_lost_instances": len(all_lost),
        "all_completely_lost_instance_ids": all_lost,
        "partially_lost_instances": len(partial),
        "partially_lost_instance_ids": partial,
        "baseline_support_completely_replaced_instance_ids": replaced,
        "baseline_obstacle_voxels": sum(r["baseline_voxels"] for r in rows),
        "retained_baseline_obstacle_voxels": sum(
            r["retained_baseline_voxels"] for r in rows
        ),
        "instances": rows,
    }
