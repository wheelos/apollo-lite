# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Stateless observed-terrain candidates, not vehicle safety or free space."""

import math
from collections import deque
from dataclasses import dataclass
from numbers import Real
from typing import Optional

import numpy as np
from terrain_surface import TerrainSurfaceEstimate

MEASURED = 1
OBJECT = 2
UNKNOWN_RISK = 4

REASON_UNSUPPORTED = 1 << 0
REASON_SLOPE_UNOBSERVED = 1 << 1
REASON_ROUGHNESS_UNOBSERVED = 1 << 2
REASON_STEP_UNKNOWN = 1 << 3
REASON_PIT_UNKNOWN = 1 << 4
REASON_LAYER_AMBIGUITY = 1 << 5
REASON_SLOPE_EXCEEDS = 1 << 6
REASON_ROUGHNESS_EXCEEDS = 1 << 7
REASON_STEP_EXCEEDS = 1 << 8
REASON_PIT_EXCEEDS = 1 << 9
REASON_MEASURED_RISK = 1 << 10
REASON_OBJECT_RISK = 1 << 11
REASON_UNKNOWN_RISK = 1 << 12
REASON_RISK_UNOBSERVED = 1 << 13
REASON_DISCONNECTED = 1 << 14


@dataclass(frozen=True)
class TraversabilityLimits:
    """Caller-owned positive diagnostic limits; no vehicle defaults."""

    max_slope_gradient: float
    max_roughness_m: float
    max_step_m: float
    max_depression_depth_m: float

    def validate(self):
        for value in (
            self.max_slope_gradient,
            self.max_roughness_m,
            self.max_step_m,
            self.max_depression_depth_m,
        ):
            if (
                not isinstance(value, Real)
                or isinstance(value, (bool, np.bool_))
                or not math.isfinite(value)
                or value <= 0
            ):
                raise ValueError(
                    "traversability limits must be finite positive numbers"
                )


@dataclass
class TraversabilityEstimate:
    """Independent cell decisions and exact observed-root components."""

    terrain_state: np.ndarray
    decision_state: np.ndarray
    reason_bits: np.ndarray
    terrain_candidate: np.ndarray
    risk_constrained_candidate: np.ndarray
    terrain_connected: np.ndarray
    risk_constrained_connected: np.ndarray
    risk_layers: np.ndarray
    risk_observed: np.ndarray
    root_cell: Optional[tuple]
    overall_status: str


def _risk_grids(surface, risk_layers, risk_observed):
    shape = surface.cell_status.shape
    if risk_layers is None:
        layers = np.zeros(shape, dtype=np.uint8)
    else:
        layers = np.asarray(risk_layers)
        if (
            layers.shape != shape
            or layers.dtype.kind not in "iu"
            or np.any(layers < 0)
            or np.any(layers > (MEASURED | OBJECT | UNKNOWN_RISK))
        ):
            raise ValueError("risk_layers must be an integer grid of bits 1/2/4")
        layers = layers.astype(np.uint8, order="C", copy=True)
    if risk_observed is None:
        observed = np.zeros(shape, dtype=bool)
    else:
        observed = np.asarray(risk_observed)
        if observed.shape != shape or observed.dtype.kind != "b":
            raise ValueError("risk_observed must be a matching boolean grid")
        observed = observed.copy()

    valid = (
        surface.point_risk_mask
        & (surface.point_cell_id >= 0)
        & (surface.point_cell_id < layers.size)
    )
    # Risks on any raw height layer veto the cell, not only the selected floor.
    cells = np.unique(surface.point_cell_id[valid])
    flat = layers.reshape(-1)
    unmapped = cells[flat[cells] == 0]
    flat[unmapped] |= UNKNOWN_RISK
    return layers, observed


def _component(candidate, surface, limits):
    connected = np.zeros(candidate.shape, dtype=bool)
    root = surface.root_cell
    if root is None or not candidate[root]:
        return connected
    queue = deque([root])
    connected[root] = True
    edge_limit = min(
        surface.config.max_neighbor_height_change,
        surface.config.cell_size
        * min(surface.config.max_slope_gradient, limits.max_slope_gradient),
    )
    nx, ny = candidate.shape
    while queue:
        ix, iy = queue.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (ix + dx, iy + dy)
            xx, yy = neighbor
            if not (0 <= xx < nx and 0 <= yy < ny):
                continue
            if connected[neighbor] or not candidate[neighbor]:
                continue
            delta = abs(
                surface.cell_elevation_z[neighbor] - surface.cell_elevation_z[ix, iy]
            )
            if not np.isfinite(delta) or delta > edge_limit:
                continue
            connected[neighbor] = True
            queue.append(neighbor)
    return connected


def classify_traversability(surface, limits, *, risk_layers=None, risk_observed=None):
    """Classify observed geometry without erasing independently retained risk.

    All returned arrays are newly allocated. Zero risk bits qualify a candidate
    only with explicit risk coverage. Connectivity uses the exact surface root,
    four neighbors and observed elevation edge checks; it is not ego reachability.
    A combined layer-span/boundary value is a conservative discontinuity veto:
    the surface's maximum cannot identify a physical boundary-step height.
    """
    if not isinstance(surface, TerrainSurfaceEstimate):
        raise ValueError("surface must be a TerrainSurfaceEstimate")
    if not isinstance(limits, TraversabilityLimits):
        raise ValueError("limits must be TraversabilityLimits")
    limits.validate()
    shape = surface.config.shape
    grid_fields = (
        "cell_status",
        "cell_elevation_z",
        "cell_observed_layers",
        "cell_slope_gradient",
        "cell_roughness_m",
        "cell_step_m",
        "cell_step_status",
        "pit_status",
        "pit_depth_m",
    )
    if any(np.shape(getattr(surface, name)) != shape for name in grid_fields):
        raise ValueError("surface grids must match the configured shape")
    root = surface.root_cell
    if root is not None:
        if (
            not isinstance(root, tuple)
            or len(root) != 2
            or any(
                not isinstance(v, (int, np.integer)) or isinstance(v, bool)
                for v in root
            )
            or any(v < 0 or v >= size for v, size in zip(root, shape))
        ):
            raise ValueError("surface root_cell must be an in-grid integer pair")
    layers, observed = _risk_grids(surface, risk_layers, risk_observed)
    reasons = np.zeros(shape, dtype=np.uint32)

    def mark(mask, reason):
        reasons[mask] |= reason

    supported = surface.cell_status == "supported"
    slope_known = np.isfinite(surface.cell_slope_gradient)
    roughness_known = np.isfinite(surface.cell_roughness_m)
    ambiguity = np.fromiter(
        (len(cell) > 1 for cell in surface.cell_observed_layers.flat),
        dtype=bool,
        count=np.prod(shape),
    ).reshape(shape)
    ambiguity |= np.isin(
        surface.cell_step_status,
        ("observed_within_cell_layer_span", "layer_span_and_boundary_discontinuity"),
    )
    ambiguity |= surface.pit_status == "unknown_layer_ambiguity"
    boundary = np.isin(
        surface.cell_step_status,
        ("observed_boundary_discontinuity", "layer_span_and_boundary_discontinuity"),
    )
    step_measured = boundary & np.isfinite(surface.cell_step_m)
    step_known = (
        surface.cell_step_status == "no_discontinuity_observed"
    ) | step_measured
    pit_measured = (surface.pit_status == "measured") & np.isfinite(surface.pit_depth_m)
    pit_known = (surface.pit_status == "no_depression_observed") | pit_measured
    slope_exceeds = slope_known & (
        surface.cell_slope_gradient > limits.max_slope_gradient
    )
    roughness_exceeds = roughness_known & (
        surface.cell_roughness_m > limits.max_roughness_m
    )
    step_exceeds = step_measured & (surface.cell_step_m > limits.max_step_m)
    pit_exceeds = pit_measured & (surface.pit_depth_m > limits.max_depression_depth_m)
    for mask, reason in (
        (~supported, REASON_UNSUPPORTED),
        (~slope_known, REASON_SLOPE_UNOBSERVED),
        (~roughness_known, REASON_ROUGHNESS_UNOBSERVED),
        (~step_known, REASON_STEP_UNKNOWN),
        (~pit_known, REASON_PIT_UNKNOWN),
        (ambiguity, REASON_LAYER_AMBIGUITY),
        (slope_exceeds, REASON_SLOPE_EXCEEDS),
        (roughness_exceeds, REASON_ROUGHNESS_EXCEEDS),
        (step_exceeds, REASON_STEP_EXCEEDS),
        (pit_exceeds, REASON_PIT_EXCEEDS),
        ((layers & MEASURED) != 0, REASON_MEASURED_RISK),
        ((layers & OBJECT) != 0, REASON_OBJECT_RISK),
        ((layers & UNKNOWN_RISK) != 0, REASON_UNKNOWN_RISK),
        (~observed, REASON_RISK_UNOBSERVED),
    ):
        mark(mask, reason)
    blockers = slope_exceeds | roughness_exceeds | step_exceeds | pit_exceeds
    candidate = (
        supported
        & np.isfinite(surface.cell_elevation_z)
        & slope_known
        & roughness_known
        & step_known
        & pit_known
        & ~ambiguity
        & ~blockers
    )
    terrain_state = np.full(shape, "UNKNOWN", dtype="<U20")
    terrain_state[supported & ~ambiguity] = "GROUND_SUPPORTED"
    terrain_state[candidate] = "TERRAIN_PASSABLE"
    terrain_state[blockers] = "BLOCKED"
    risk_candidate = candidate & observed & (layers == 0)
    decision_state = np.full(shape, "UNKNOWN", dtype="<U9")
    decision_state[risk_candidate] = "CANDIDATE"
    decision_state[blockers | ((layers & (MEASURED | OBJECT)) != 0)] = "BLOCKED"
    terrain_connected = _component(candidate, surface, limits)
    risk_connected = _component(risk_candidate, surface, limits)
    mark(
        (candidate & ~terrain_connected) | (risk_candidate & ~risk_connected),
        REASON_DISCONNECTED,
    )
    overall = "UNKNOWN_NO_ROOT"
    if root is not None:
        overall = (
            "OBSERVED_ROOT_CONNECTED"
            if terrain_connected[root]
            else "OBSERVED_ROOT_VETOED"
        )
    return TraversabilityEstimate(
        terrain_state=terrain_state,
        decision_state=decision_state,
        reason_bits=reasons,
        terrain_candidate=candidate,
        risk_constrained_candidate=risk_candidate,
        terrain_connected=terrain_connected,
        risk_constrained_connected=risk_connected,
        risk_layers=layers,
        risk_observed=observed,
        root_cell=root,
        overall_status=overall,
    )
