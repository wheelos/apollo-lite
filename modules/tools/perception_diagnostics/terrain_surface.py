# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Observed-only, anchored local terrain surface diagnostics.

This module estimates one four-connected observed surface from a point cloud.
It preserves every observed height layer separately; unsupported cells remain
unknown and no interpolation, free-space inference, semantic labels, or
ground-truth inputs are used. All defaults are experimental diagnostics, not
production thresholds or vehicle capability limits.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class TerrainSurfaceConfig:
    """Fixed diagnostic grid and local terrain contracts, in metres."""

    x_min: float = -60.0
    x_max: float = 60.0
    y_min: float = -40.0
    y_max: float = 40.0
    cell_size: float = 0.5
    expected_ground_z: Optional[float] = None
    seed_height_band: float = 0.30
    anchor_radius: float = 8.0
    layer_gap: float = 0.20
    ground_tolerance: float = 0.10
    max_slope_gradient: float = 0.30
    max_neighbor_height_change: float = 0.20
    pit_rim_search_radius: float = 3.0
    min_anchor_points: int = 3
    min_pit_floor_points: int = 3
    coordinate_frame: str = "gravity-aligned frame-local XYZ; z up"

    def validate(self):
        finite = (
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.cell_size,
            self.seed_height_band,
            self.anchor_radius,
            self.layer_gap,
            self.ground_tolerance,
            self.max_slope_gradient,
            self.max_neighbor_height_change,
            self.pit_rim_search_radius,
        )
        if self.expected_ground_z is not None:
            finite += (self.expected_ground_z,)
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("terrain surface config must be finite")
        if self.x_max <= self.x_min or self.y_max <= self.y_min or self.cell_size <= 0:
            raise ValueError("invalid terrain surface bounds or cell size")
        nx = (self.x_max - self.x_min) / self.cell_size
        ny = (self.y_max - self.y_min) / self.cell_size
        if not math.isclose(nx, round(nx), abs_tol=1e-9) or not math.isclose(
            ny, round(ny), abs_tol=1e-9
        ):
            raise ValueError("terrain surface bounds must align to cell size")
        if (
            self.seed_height_band <= 0
            or self.anchor_radius <= 0
            or self.layer_gap <= 0
            or self.ground_tolerance < 0
            or self.max_slope_gradient < 0
            or self.max_neighbor_height_change <= 0
            or self.pit_rim_search_radius <= 0
        ):
            raise ValueError("invalid terrain surface thresholds")
        if not isinstance(self.min_anchor_points, int) or self.min_anchor_points < 1:
            raise ValueError("min_anchor_points must be positive")
        if (
            not isinstance(self.min_pit_floor_points, int)
            or self.min_pit_floor_points < 3
        ):
            raise ValueError("min_pit_floor_points must be at least three")
        if not isinstance(self.coordinate_frame, str) or not self.coordinate_frame:
            raise ValueError("coordinate_frame must be a non-empty label")

    @property
    def shape(self):
        self.validate()
        return (
            int(round((self.x_max - self.x_min) / self.cell_size)),
            int(round((self.y_max - self.y_min) / self.cell_size)),
        )


@dataclass(frozen=True)
class TerrainLayerObservation:
    """A raw observed height layer in one XY cell."""

    layer_id: int
    elevation_z: float
    point_count: int
    source_point_ids: tuple
    selected: bool


@dataclass
class TerrainSurfaceEstimate:
    """Point-aligned labels plus raw-layer and observed-cell diagnostics.

    ``point_ground_candidate_mask`` is the selected layer within vertical
    tolerance before applying the independent ``risk_mask``. The final
    ``point_ground_mask`` excludes risk rows but does not remove them from any
    raw layer or surface fit. ``point_surface_z`` is the selected observed
    layer elevation repeated for rows in a supported cell; its residual is
    always the original raw z minus that cell elevation.

    ``cell_observed_layer_span_m`` is the raw min/max layer-height difference
    within one cell. ``cell_step_m`` includes this measurable vertical
    separation and adjacent-cell discontinuity after local slope compensation.
    A within-cell span can be a step, bridge, or overlapping surface; status
    preserves that ambiguity instead of joining height layers.

    Cell slope and roughness use a local plane fit to selected observed cell
    elevations. Roughness is the sample standard deviation of those cell
    elevations' residuals from that local plane, not calibrated surface
    roughness. ``pit_status="measured"`` reports only observed depth relative
    to an independently supported surrounding rim; it does not classify a
    physical pit. Selected cells with multiple observed layers have
    ``pit_status="unknown_layer_ambiguity"``. Pit width fields are the
    connected observed depression-cell extent, not inferred physical pit
    boundaries; each floor component needs at least three raw returns (the
    existing local-seed support floor) before depth/width output.
    """

    point_ids: np.ndarray
    point_ground_candidate_mask: np.ndarray
    point_ground_mask: np.ndarray
    point_risk_mask: np.ndarray
    point_cell_id: np.ndarray
    point_layer_id: np.ndarray
    point_surface_z: np.ndarray
    point_residual_z: np.ndarray
    cell_status: np.ndarray
    cell_elevation_z: np.ndarray
    cell_observed_layers: np.ndarray
    cell_raw_return_count: np.ndarray
    cell_observed_layer_span_m: np.ndarray
    cell_roughness_m: np.ndarray
    cell_slope_gradient: np.ndarray
    cell_step_m: np.ndarray
    cell_step_status: np.ndarray
    pit_status: np.ndarray
    pit_depth_m: np.ndarray
    pit_width_x_m: np.ndarray
    pit_width_y_m: np.ndarray
    root_anchor_id: Optional[int]
    root_cell: Optional[tuple]
    overall_status: str
    config: TerrainSurfaceConfig


def _xyz(points_xyz):
    points = np.asarray(points_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("expected Nx3 upright XYZ points in metres")
    return points


def _local_plane(cell_elevation, selected, x_min, y_min, cell_size, ix, iy):
    """Fit z=a*x+b*y+c to selected 8-neighbors; return None if unobservable."""
    nx, ny = selected.shape
    rows = []
    values = []
    for xx in range(max(0, ix - 1), min(nx, ix + 2)):
        for yy in range(max(0, iy - 1), min(ny, iy + 2)):
            if selected[xx, yy]:
                rows.append(
                    (
                        x_min + (xx + 0.5) * cell_size,
                        y_min + (yy + 0.5) * cell_size,
                    )
                )
                values.append(cell_elevation[xx, yy])
    if len(rows) < 4:
        return None
    center = np.mean(rows, axis=0)
    offsets = np.asarray(rows) - center
    design = np.column_stack((offsets, np.ones(len(rows))))
    if np.linalg.matrix_rank(design) != 3:
        return None
    coefficients, _, rank, _ = np.linalg.lstsq(design, np.asarray(values), rcond=None)
    if rank != 3 or not np.isfinite(coefficients).all():
        return None
    residuals = design @ coefficients - values
    return (
        float(coefficients[0]),
        float(coefficients[1]),
        float(coefficients[2]),
        float(np.std(residuals, ddof=1)),
        center,
    )


def _contains_origin(points):
    """Return whether the convex hull of 2D points contains the origin."""
    ordered = sorted(set(map(tuple, np.asarray(points, dtype=float).tolist())))
    if len(ordered) < 3:
        return False

    def cross(origin, first, second):
        return (first[0] - origin[0]) * (second[1] - origin[1]) - (
            first[1] - origin[1]
        ) * (second[0] - origin[0])

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    hull = lower[:-1] + upper[:-1]
    return all(
        first[0] * second[1] - first[1] * second[0] > 1e-12
        for first, second in zip(hull, hull[1:] + hull[:1])
    )


def estimate_terrain_surface(
    points_xyz,
    point_ids=None,
    risk_mask=None,
    *,
    root_anchor_id=None,
    config=TerrainSurfaceConfig(),
):
    """Estimate an anchored, observed-only local terrain surface.

    ``points_xyz`` is Nx3 gravity-aligned, frame-local XYZ in metres, with +z
    up; the caller owns and documents any transform. ``point_ids`` are unique
    source identifiers, defaulting to input row indices. ``root_anchor_id``,
    when set, must identify an observed finite in-ROI row and selects that
    row's observed layer. Without an explicit anchor, the lowest deterministic
    observed layer in the nearest occupied XY cell inside ``anchor_radius`` is
    used. An optional
    ``expected_ground_z`` further gates only this local initializer; its
    default is absent, so the automatic anchor is invariant to global z
    translation. Selected surface heights may vary locally anywhere in ROI.

    Layers are formed by splitting sorted raw heights at gaps greater than
    ``layer_gap``. Four-connected propagation selects the closest neighboring
    observed layer only when its height change and implied gradient satisfy
    the fixed diagnostic limits. The graph boundary is not a physical pit
    detector: an unselected lower observed layer is reported as relative
    depth only with a full-rank surrounding supported rim. Selected cells
    with multiple observed layers have unknown pit status. Missing returns
    stay unknown.
    """
    config.validate()
    xyz = _xyz(points_xyz)
    n = len(xyz)
    if point_ids is None:
        ids = np.arange(n, dtype=np.int64)
    else:
        ids = np.asarray(point_ids)
        if ids.shape != (n,) or ids.dtype.kind not in "iu":
            raise ValueError("point_ids must be an integer vector matching rows")
        ids = ids.copy()
    if len(np.unique(ids)) != n:
        raise ValueError("point_ids must be unique")
    if risk_mask is None:
        risk = np.zeros(n, dtype=bool)
    else:
        risk = np.asarray(risk_mask)
        if risk.shape != (n,) or risk.dtype.kind != "b":
            raise ValueError("risk_mask must be a boolean vector matching rows")
        risk = risk.copy()

    nx, ny = config.shape
    shape = (nx, ny)
    cell_id = np.full(n, -1, dtype=np.int64)
    layer_id = np.full(n, -1, dtype=np.int64)
    point_surface_z = np.full(n, np.nan, dtype=float)
    point_residual_z = np.full(n, np.nan, dtype=float)
    ground_candidate = np.zeros(n, dtype=bool)
    ground = np.zeros(n, dtype=bool)
    raw_count = np.zeros(shape, dtype=np.int64)
    observed_layer_span = np.full(shape, np.nan)
    cell_layers = np.empty(shape, dtype=object)
    cell_layers.fill(())
    cell_status = np.full(shape, "unknown_empty", dtype="<U24")
    cell_elevation = np.full(shape, np.nan)
    selected = np.zeros(shape, dtype=bool)
    selected_layer = np.full(shape, -1, dtype=np.int64)
    layer_points = {}
    layer_heights = {}

    finite = np.isfinite(xyz).all(axis=1)
    in_roi = (
        finite
        & (xyz[:, 0] >= config.x_min)
        & (xyz[:, 0] < config.x_max)
        & (xyz[:, 1] >= config.y_min)
        & (xyz[:, 1] < config.y_max)
    )
    rows = np.flatnonzero(in_roi)
    buckets = {}
    for row in rows:
        ix = int(math.floor((xyz[row, 0] - config.x_min) / config.cell_size))
        iy = int(math.floor((xyz[row, 1] - config.y_min) / config.cell_size))
        cell_id[row] = ix * ny + iy
        buckets.setdefault((ix, iy), []).append(int(row))

    for (ix, iy), bucket in buckets.items():
        bucket = np.asarray(bucket, dtype=np.int64)
        raw_count[ix, iy] = len(bucket)
        order = bucket[np.argsort(xyz[bucket, 2], kind="stable")]
        groups = []
        current = [int(order[0])]
        for row in order[1:]:
            if xyz[row, 2] - xyz[current[-1], 2] > config.layer_gap:
                groups.append(current)
                current = []
            current.append(int(row))
        groups.append(current)
        observations = []
        for local_id, group in enumerate(groups):
            layer_rows = np.asarray(group, dtype=np.int64)
            height = float(np.median(xyz[layer_rows, 2]))
            observation = TerrainLayerObservation(
                layer_id=local_id,
                elevation_z=height,
                point_count=len(group),
                source_point_ids=tuple(ids[layer_rows].tolist()),
                selected=False,
            )
            observations.append(observation)
            layer_points[(ix, iy, local_id)] = layer_rows
            layer_heights[(ix, iy, local_id)] = height
            layer_id[layer_rows] = local_id
        cell_layers[ix, iy] = tuple(observations)
        observed_layer_span[ix, iy] = max(
            layer_heights[(ix, iy, local_id)] for local_id in range(len(groups))
        ) - min(layer_heights[(ix, iy, local_id)] for local_id in range(len(groups)))
        cell_status[ix, iy] = "other_layer"

    def row_layer(row):
        cell = int(cell_id[row])
        ix, iy = divmod(cell, ny)
        return ix, iy, int(layer_id[row])

    def anchor_has_local_support(candidate):
        ax, ay, anchor_layer = candidate
        anchor_z = layer_heights[(ax, ay, anchor_layer)]
        support_xy = []
        support_z = []
        for ix in range(max(0, ax - 1), min(nx, ax + 2)):
            for iy in range(max(0, ay - 1), min(ny, ay + 2)):
                observations = cell_layers[ix, iy]
                if not observations:
                    continue
                x = config.x_min + (ix + 0.5) * config.cell_size
                y = config.y_min + (iy + 0.5) * config.cell_size
                anchor_x = config.x_min + (ax + 0.5) * config.cell_size
                anchor_y = config.y_min + (ay + 0.5) * config.cell_size
                distance = math.hypot(x - anchor_x, y - anchor_y)
                compatible = [
                    item
                    for item in observations
                    if abs(item.elevation_z - anchor_z)
                    <= config.max_slope_gradient * distance + config.ground_tolerance
                ]
                if compatible:
                    layer = min(
                        compatible,
                        key=lambda item: abs(item.elevation_z - anchor_z),
                    )
                    support_xy.append((x, y))
                    support_z.append(layer.elevation_z)
        if len(support_xy) < 3:
            return False
        offsets = np.asarray(support_xy) - np.mean(support_xy, axis=0)
        design = np.column_stack((offsets, np.ones(len(offsets))))
        if np.linalg.matrix_rank(design) != 3:
            return False
        coefficients, _, rank, _ = np.linalg.lstsq(
            design, np.asarray(support_z), rcond=None
        )
        if rank != 3 or not np.isfinite(coefficients).all():
            return False
        residuals = design @ coefficients - support_z
        return (
            math.hypot(coefficients[0], coefficients[1]) <= config.max_slope_gradient
            and float(np.max(np.abs(residuals))) <= config.ground_tolerance
        )

    anchor = None
    if root_anchor_id is not None:
        matches = np.flatnonzero(ids == root_anchor_id)
        if len(matches) != 1:
            raise ValueError("root_anchor_id must identify one input source row")
        row = int(matches[0])
        if not in_roi[row]:
            raise ValueError("root_anchor_id must identify a finite in-ROI point")
        candidate = row_layer(row)
        if layer_points[
            candidate
        ].size >= config.min_anchor_points and anchor_has_local_support(candidate):
            anchor = candidate
    else:
        candidates = []
        for (ix, iy), observations in np.ndenumerate(cell_layers):
            if not observations:
                continue
            cx = config.x_min + (ix + 0.5) * config.cell_size
            cy = config.y_min + (iy + 0.5) * config.cell_size
            radius = math.hypot(cx, cy)
            if radius > config.anchor_radius:
                continue
            for observation in observations:
                if observation.point_count >= config.min_anchor_points and (
                    config.expected_ground_z is None
                    or abs(observation.elevation_z - config.expected_ground_z)
                    <= config.seed_height_band
                ):
                    candidates.append(
                        (
                            radius,
                            observation.elevation_z,
                            ix,
                            iy,
                            observation.layer_id,
                        )
                    )
        if candidates:
            for _, _, ix, iy, local_id in sorted(candidates):
                candidate = (ix, iy, local_id)
                if anchor_has_local_support(candidate):
                    anchor = candidate
                    break

    if anchor is not None:
        ax, ay, anchor_layer = anchor
        selected[ax, ay] = True
        selected_layer[ax, ay] = anchor_layer
        cell_elevation[ax, ay] = layer_heights[(ax, ay, anchor_layer)]
        cell_status[ax, ay] = "supported"
        queue = deque([(ax, ay)])
        while queue:
            ix, iy = queue.popleft()
            source_z = cell_elevation[ix, iy]
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                xx, yy = ix + dx, iy + dy
                if not (0 <= xx < nx and 0 <= yy < ny) or selected[xx, yy]:
                    continue
                observations = cell_layers[xx, yy]
                if not observations:
                    continue
                choices = sorted(
                    (
                        abs(layer_heights[(xx, yy, obs.layer_id)] - source_z),
                        layer_heights[(xx, yy, obs.layer_id)],
                        obs.layer_id,
                    )
                    for obs in observations
                )
                delta, height, local_id = choices[0]
                if (
                    delta <= config.max_neighbor_height_change
                    and delta / config.cell_size <= config.max_slope_gradient
                ):
                    selected[xx, yy] = True
                    selected_layer[xx, yy] = local_id
                    cell_elevation[xx, yy] = height
                    cell_status[xx, yy] = "supported"
                    queue.append((xx, yy))
        cell_status[(raw_count > 0) & ~selected] = "unknown_gap"
    else:
        cell_status[raw_count > 0] = "unknown_no_anchor"

    selected_ids = np.flatnonzero(selected)
    for ix, iy in zip(*np.nonzero(selected)):
        local_id = int(selected_layer[ix, iy])
        observations = list(cell_layers[ix, iy])
        cell_layers[ix, iy] = tuple(
            TerrainLayerObservation(
                layer_id=item.layer_id,
                elevation_z=item.elevation_z,
                point_count=item.point_count,
                source_point_ids=item.source_point_ids,
                selected=(item.layer_id == local_id),
            )
            for item in observations
        )
        layer_rows = layer_points[(ix, iy, local_id)]
        point_surface_z[buckets[(ix, iy)]] = cell_elevation[ix, iy]
        point_residual_z[buckets[(ix, iy)]] = (
            xyz[buckets[(ix, iy)], 2] - cell_elevation[ix, iy]
        )
        ground_candidate[layer_rows] = (
            np.abs(xyz[layer_rows, 2] - cell_elevation[ix, iy])
            <= config.ground_tolerance
        )
    ground[:] = ground_candidate & ~risk

    slope = np.full(shape, np.nan)
    roughness = np.full(shape, np.nan)
    step = np.full(shape, np.nan)
    step_status = np.full(shape, "unknown_unsupported", dtype="<U40")
    for ix, iy in zip(*np.nonzero(selected)):
        plane = _local_plane(
            cell_elevation,
            selected,
            config.x_min,
            config.y_min,
            config.cell_size,
            ix,
            iy,
        )
        if plane is not None:
            slope[ix, iy] = math.hypot(plane[0], plane[1])
            roughness[ix, iy] = plane[3]
        discontinuities = []
        has_layer_span = (
            np.isfinite(observed_layer_span[ix, iy])
            and observed_layer_span[ix, iy] > config.max_neighbor_height_change
        )
        if has_layer_span:
            discontinuities.append(observed_layer_span[ix, iy])
        has_boundary_discontinuity = False
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            xx, yy = ix + dx, iy + dy
            if not (0 <= xx < nx and 0 <= yy < ny):
                continue
            neighbor_layers = cell_layers[xx, yy]
            if not neighbor_layers:
                continue
            neighbor_heights = [
                layer_heights[(xx, yy, obs.layer_id)] for obs in neighbor_layers
            ]
            closest = min(
                neighbor_heights, key=lambda value: abs(value - cell_elevation[ix, iy])
            )
            expected_delta = 0.0
            if plane is not None:
                expected_delta = (
                    plane[0] * dx * config.cell_size + plane[1] * dy * config.cell_size
                )
            difference = abs(closest - cell_elevation[ix, iy] - expected_delta)
            if difference > config.max_neighbor_height_change:
                discontinuities.append(difference)
                has_boundary_discontinuity = True
        if discontinuities:
            step[ix, iy] = max(discontinuities)
            if has_layer_span and has_boundary_discontinuity:
                step_status[ix, iy] = "layer_span_and_boundary_discontinuity"
            elif has_layer_span:
                step_status[ix, iy] = "observed_within_cell_layer_span"
            else:
                step_status[ix, iy] = "observed_boundary_discontinuity"
        else:
            step_status[ix, iy] = "no_discontinuity_observed"

    pit_status = np.full(shape, "unknown_no_floor", dtype="<U32")
    pit_depth = np.full(shape, np.nan)
    pit_width_x = np.full(shape, np.nan)
    pit_width_y = np.full(shape, np.nan)
    for ix, iy in np.ndindex(shape):
        if selected[ix, iy]:
            pit_status[ix, iy] = (
                "unknown_layer_ambiguity"
                if len(cell_layers[ix, iy]) > 1
                else "no_depression_observed"
            )
        elif cell_layers[ix, iy]:
            pit_status[ix, iy] = "unknown_no_rim"

    low_cells = set()
    floor_by_cell = {}
    depth_by_cell = {}
    for ix, iy in np.ndindex(shape):
        if selected[ix, iy] or not cell_layers[ix, iy]:
            continue
        floor = min(cell_layers[ix, iy], key=lambda item: item.elevation_z)
        cx = config.x_min + (ix + 0.5) * config.cell_size
        cy = config.y_min + (iy + 0.5) * config.cell_size
        cell_radius = int(math.ceil(config.pit_rim_search_radius / config.cell_size))
        ring_cells = []
        for xx in range(max(0, ix - cell_radius), min(nx, ix + cell_radius + 1)):
            for yy in range(max(0, iy - cell_radius), min(ny, iy + cell_radius + 1)):
                if not selected[xx, yy]:
                    continue
                rim_x = config.x_min + (xx + 0.5) * config.cell_size
                rim_y = config.y_min + (yy + 0.5) * config.cell_size
                if math.hypot(rim_x - cx, rim_y - cy) <= config.pit_rim_search_radius:
                    ring_cells.append((xx, yy))
        ring_xy = [
            (
                config.x_min + (xx + 0.5) * config.cell_size,
                config.y_min + (yy + 0.5) * config.cell_size,
            )
            for xx, yy in ring_cells
        ]
        if len(ring_xy) < 4:
            continue
        encloses = _contains_origin(np.asarray(ring_xy) - np.array([cx, cy]))
        if not encloses:
            continue
        ring_center = np.mean(ring_xy, axis=0)
        design = np.column_stack(
            (np.asarray(ring_xy) - ring_center, np.ones(len(ring_xy)))
        )
        if np.linalg.matrix_rank(design) != 3:
            continue
        coefficients, _, rank, _ = np.linalg.lstsq(
            design,
            np.asarray([cell_elevation[xx, yy] for xx, yy in ring_cells]),
            rcond=None,
        )
        if rank != 3 or not np.isfinite(coefficients).all():
            continue
        reference = float(
            np.array([cx - ring_center[0], cy - ring_center[1], 1.0]) @ coefficients
        )
        depth = reference - floor.elevation_z
        if depth <= config.max_neighbor_height_change:
            pit_status[ix, iy] = "no_depression_observed"
            continue
        low_cells.add((ix, iy))
        floor_by_cell[(ix, iy)] = floor
        depth_by_cell[(ix, iy)] = depth

    depression_components = []
    while low_cells:
        seed = min(low_cells)
        low_cells.remove(seed)
        component = {seed}
        queue = deque([seed])
        while queue:
            ix, iy = queue.popleft()
            for neighbor in ((ix - 1, iy), (ix + 1, iy), (ix, iy - 1), (ix, iy + 1)):
                if neighbor in low_cells:
                    low_cells.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        depression_components.append(component)

    for component in depression_components:
        floor_support = sum(floor_by_cell[cell].point_count for cell in component)
        if floor_support < config.min_pit_floor_points:
            for cell in component:
                pit_status[cell] = "unknown_insufficient_floor"
            continue
        width_x = (
            max(ix for ix, _ in component) - min(ix for ix, _ in component) + 1
        ) * config.cell_size
        width_y = (
            max(iy for _, iy in component) - min(iy for _, iy in component) + 1
        ) * config.cell_size
        for ix, iy in component:
            pit_status[ix, iy] = "measured"
            pit_depth[ix, iy] = depth_by_cell[(ix, iy)]
            pit_width_x[ix, iy] = width_x
            pit_width_y[ix, iy] = width_y

    if anchor is None:
        overall_status = "unknown_no_anchor"
        root_id = None
        root_cell = None
    else:
        root_row = (
            int(np.flatnonzero(ids == root_anchor_id)[0])
            if root_anchor_id is not None
            else min(
                layer_points[anchor],
                key=lambda row: (
                    math.hypot(xyz[row, 0], xyz[row, 1]),
                    int(ids[row]),
                ),
            )
        )
        root_id = int(ids[root_row])
        root_cell = (int(anchor[0]), int(anchor[1]))
        overall_status = "supported" if selected_ids.size else "unknown_no_anchor"

    return TerrainSurfaceEstimate(
        point_ids=ids,
        point_ground_candidate_mask=ground_candidate,
        point_ground_mask=ground,
        point_risk_mask=risk,
        point_cell_id=cell_id,
        point_layer_id=layer_id,
        point_surface_z=point_surface_z,
        point_residual_z=point_residual_z,
        cell_status=cell_status,
        cell_elevation_z=cell_elevation,
        cell_observed_layers=cell_layers,
        cell_raw_return_count=raw_count,
        cell_observed_layer_span_m=observed_layer_span,
        cell_roughness_m=roughness,
        cell_slope_gradient=slope,
        cell_step_m=step,
        cell_step_status=step_status,
        pit_status=pit_status,
        pit_depth_m=pit_depth,
        pit_width_x_m=pit_width_x,
        pit_width_y_m=pit_width_y,
        root_anchor_id=root_id,
        root_cell=root_cell,
        overall_status=overall_status,
        config=config,
    )
