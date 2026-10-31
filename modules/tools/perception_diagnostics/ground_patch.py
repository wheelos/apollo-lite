# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Experimental, observed-only 2D patch ground-plane comparator."""

import math
from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class GroundPatchConfig:
    """Fixed diagnostic patch layout and plane acceptance contract.

    Coordinates are frame-local metres. ``expected_ground_z`` and all XY
    bounds use this same caller-selected upright frame. The 1 m XY layout is
    experimental, not a calibrated vehicle-base map or production setting.
    """

    x_min: float = -60.0
    x_max: float = 60.0
    y_min: float = -40.0
    y_max: float = 40.0
    patch_size: float = 1.0
    seed_quantile: float = 0.20
    min_seed_points: int = 3
    expected_ground_z: float = -1.73
    ground_seed_band: float = 0.30
    max_slope_gradient: float = 0.30
    max_fit_residual: float = 0.08
    ground_tolerance: float = 0.10
    max_condition: float = 1e8
    coordinate_frame: str = "frame-local"

    def validate(self):
        finite = (
            self.x_min,
            self.x_max,
            self.y_min,
            self.y_max,
            self.patch_size,
            self.seed_quantile,
            self.expected_ground_z,
            self.ground_seed_band,
            self.max_slope_gradient,
            self.max_fit_residual,
            self.ground_tolerance,
            self.max_condition,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("ground patch config must be finite")
        if self.x_max <= self.x_min or self.y_max <= self.y_min or self.patch_size <= 0:
            raise ValueError("invalid ground patch bounds or size")
        nx = (self.x_max - self.x_min) / self.patch_size
        ny = (self.y_max - self.y_min) / self.patch_size
        if not math.isclose(nx, round(nx), abs_tol=1e-9) or not math.isclose(
            ny, round(ny), abs_tol=1e-9
        ):
            raise ValueError("ground patch bounds must align to patch size")
        if not 0 < self.seed_quantile <= 1:
            raise ValueError("seed_quantile must be in (0, 1]")
        if not isinstance(self.min_seed_points, int) or self.min_seed_points < 3:
            raise ValueError("min_seed_points must be at least three")
        if (
            self.ground_seed_band <= 0
            or self.max_slope_gradient < 0
            or self.max_fit_residual < 0
            or self.ground_tolerance < 0
            or self.max_condition <= 1
        ):
            raise ValueError("invalid ground patch acceptance limits")
        if not isinstance(self.coordinate_frame, str) or not self.coordinate_frame:
            raise ValueError("coordinate_frame must be a non-empty label")

    @property
    def shape(self):
        self.validate()
        return (
            int(round((self.x_max - self.x_min) / self.patch_size)),
            int(round((self.y_max - self.y_min) / self.patch_size)),
        )


@dataclass
class GroundEstimate:
    """Point-aligned fit results and per-patch support provenance.

    ``support_mask`` means a row lies in a successfully fitted observed patch;
    it is not free-space evidence. ``seed_mask`` identifies rows used to fit.
    ``ground_mask`` labels only observed input rows within ground tolerance.
    Per-patch arrays use ``[ix, iy]`` indexing and planes are ``[a, b, c]``.
    """

    ground_mask: np.ndarray
    ground_z: np.ndarray
    support_mask: np.ndarray
    seed_mask: np.ndarray
    cell_id: np.ndarray
    patch_supported: np.ndarray
    plane_coefficients: np.ndarray
    seed_count: np.ndarray
    fit_residual_rms: np.ndarray
    fit_residual_max: np.ndarray
    condition_number: np.ndarray
    uprightness: np.ndarray
    patch_status: np.ndarray
    point_xyz: np.ndarray
    config: GroundPatchConfig


def _points_xyz(points):
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] not in (3, 4):
        raise ValueError("expected Nx3 XYZ or Nx4 XYZI frame-local points")
    return values[:, :3]


def estimate_ground_patch(points, *, config=GroundPatchConfig()):
    """Fit independent local planes without filling unsupported patches.

    Input is Nx3 XYZ or Nx4 XYZI in an upright frame-local coordinate system,
    in metres; the configured expected height and XY bounds are in that same
    frame. The caller owns and reports any coordinate transform. Each patch
    selects the lowest ``max(min_seed_points, ceil(seed_quantile*N))``
    observations in the seed-height band, then fits ``z = a*x + b*y + c`` by
    ordinary least squares. Only seed rows enter the fit. A patch is accepted
    iff it has at least three seeds, full 2D rank, bounded condition number,
    finite fit, residual <= 0.08 m and gradient magnitude <= 0.3 by default.
    The gradient is not an angle; its plane normal has
    ``nz = 1/sqrt(1+a²+b²)``.

    ``ground_z`` is evaluated only for observed rows in accepted patches;
    ground labels additionally require raw point-to-plane vertical residual
    within the configured tolerance. No semantic labels or GT are accepted.
    """
    config.validate()
    xyz = _points_xyz(points)
    n = len(xyz)
    nx, ny = config.shape
    cell_id = np.full(n, -1, dtype=np.int64)
    ground_z = np.full(n, np.nan, dtype=float)
    support_mask = np.zeros(n, dtype=bool)
    seed_mask = np.zeros(n, dtype=bool)
    ground_mask = np.zeros(n, dtype=bool)
    patch_supported = np.zeros((nx, ny), dtype=bool)
    coefficients = np.full((nx, ny, 3), np.nan)
    seed_count = np.zeros((nx, ny), dtype=np.int64)
    residual_rms = np.full((nx, ny), np.nan)
    residual_max = np.full((nx, ny), np.nan)
    condition_number = np.full((nx, ny), np.nan)
    uprightness = np.full((nx, ny), np.nan)
    status = np.full((nx, ny), "unsupported_empty", dtype="<U40")

    finite = np.isfinite(xyz).all(axis=1)
    in_roi = (
        finite
        & (xyz[:, 0] >= config.x_min)
        & (xyz[:, 0] < config.x_max)
        & (xyz[:, 1] >= config.y_min)
        & (xyz[:, 1] < config.y_max)
    )
    ids = np.flatnonzero(in_roi)
    if len(ids):
        ix = np.floor((xyz[ids, 0] - config.x_min) / config.patch_size).astype(int)
        iy = np.floor((xyz[ids, 1] - config.y_min) / config.patch_size).astype(int)
        cell_id[ids] = ix * ny + iy

    buckets = {}
    for point_id in ids:
        ix, iy = divmod(int(cell_id[point_id]), ny)
        buckets.setdefault((ix, iy), []).append(int(point_id))

    for (ix, iy), patch_ids_list in buckets.items():
        patch_ids = np.asarray(patch_ids_list, dtype=np.int64)
        eligible = patch_ids[
            np.abs(xyz[patch_ids, 2] - config.expected_ground_z)
            <= config.ground_seed_band
        ]
        if not len(eligible):
            status[ix, iy] = "unsupported_no_height_seeds"
            continue
        count = max(
            config.min_seed_points,
            int(math.ceil(config.seed_quantile * len(eligible))),
        )
        seeds = eligible[
            np.argsort(xyz[eligible, 2], kind="stable")[: min(count, len(eligible))]
        ]
        seed_count[ix, iy] = len(seeds)
        if len(seeds) < config.min_seed_points:
            status[ix, iy] = "unsupported_insufficient_seeds"
            continue
        center = np.array(
            [
                config.x_min + (ix + 0.5) * config.patch_size,
                config.y_min + (iy + 0.5) * config.patch_size,
            ]
        )
        offsets = xyz[seeds, :2] - center
        design = np.column_stack((offsets, np.ones(len(seeds))))
        try:
            singular_values = np.linalg.svd(design, compute_uv=False)
            if (
                not np.isfinite(singular_values).all()
                or singular_values[0] <= 0
                or singular_values[-1] <= 0
            ):
                status[ix, iy] = "unsupported_rank"
                continue
            rank = int(np.linalg.matrix_rank(design))
            cond = float(singular_values[0] / singular_values[-1])
            condition_number[ix, iy] = cond
            if rank != 3:
                status[ix, iy] = "unsupported_rank"
                continue
            if not math.isfinite(cond) or cond > config.max_condition:
                status[ix, iy] = "unsupported_condition"
                continue
            local_coefficients, _, fitted_rank, _ = np.linalg.lstsq(
                design, xyz[seeds, 2], rcond=None
            )
        except np.linalg.LinAlgError:
            status[ix, iy] = "unsupported_rank"
            continue
        if fitted_rank != 3 or not np.isfinite(local_coefficients).all():
            status[ix, iy] = "unsupported_nonfinite_fit"
            continue
        a, b, local_c = map(float, local_coefficients)
        residuals = design @ local_coefficients - xyz[seeds, 2]
        max_resid = float(np.max(np.abs(residuals)))
        rms = float(np.sqrt(np.mean(np.square(residuals))))
        gradient = math.hypot(a, b)
        normal_z = 1.0 / math.sqrt(1.0 + a * a + b * b)
        if not all(math.isfinite(value) for value in (max_resid, rms, gradient)):
            status[ix, iy] = "unsupported_nonfinite_fit"
            continue
        if gradient > config.max_slope_gradient:
            status[ix, iy] = "unsupported_uprightness"
            continue
        if max_resid > config.max_fit_residual:
            status[ix, iy] = "unsupported_residual"
            continue
        c = local_c - a * center[0] - b * center[1]
        patch_supported[ix, iy] = True
        coefficients[ix, iy] = (a, b, c)
        residual_rms[ix, iy] = rms
        residual_max[ix, iy] = max_resid
        uprightness[ix, iy] = normal_z
        status[ix, iy] = "supported"
        support_mask[patch_ids] = True
        seed_mask[seeds] = True
        ground_z[patch_ids] = xyz[patch_ids, 0] * a + xyz[patch_ids, 1] * b + c
        ground_mask[patch_ids] = (
            np.abs(xyz[patch_ids, 2] - ground_z[patch_ids]) <= config.ground_tolerance
        )

    return GroundEstimate(
        ground_mask=ground_mask,
        ground_z=ground_z,
        support_mask=support_mask,
        seed_mask=seed_mask,
        cell_id=cell_id,
        patch_supported=patch_supported,
        plane_coefficients=coefficients,
        seed_count=seed_count,
        fit_residual_rms=residual_rms,
        fit_residual_max=residual_max,
        condition_number=condition_number,
        uprightness=uprightness,
        patch_status=status,
        point_xyz=xyz.copy(),
        config=config,
    )


def ground_estimate_metadata(estimate):
    """Return JSON-safe layout and parameter provenance for an estimate."""
    config = estimate.config
    return {
        "frame": {
            "coordinates": estimate.config.coordinate_frame,
            "contract": (
                "frame-local upright XYZ metres; config XY and expected height "
                "use this same frame; caller documents any source transform"
            ),
        },
        "layout": {
            "x_min_m": config.x_min,
            "x_max_m_exclusive": config.x_max,
            "y_min_m": config.y_min,
            "y_max_m_exclusive": config.y_max,
            "patch_size_m": config.patch_size,
            "patch_array_indexing": "[ix, iy]",
            "experimental": True,
        },
        "fit": {
            **asdict(config),
            "plane": "z = a*x + b*y + c",
            "slope_contract": "gradient hypot(a,b), not angle in radians",
            "uprightness_contract": "normal_z = 1/sqrt(1+a^2+b^2)",
            "seed_selection": (
                "observed finite points in expected_ground_z +/- seed band; "
                "lowest max(min_seed_points, ceil(seed_quantile*N)); minimum "
                "seed count is an explicit floor, so selected fraction can "
                "exceed seed_quantile in sparse patches"
            ),
            "fit_method": "least squares on seed observations only",
            "ground_label": (
                "observed input point with abs(raw_z - fitted_z) <= tolerance"
            ),
            "unsupported_policy": "NaN; no interpolation or extrapolation",
        },
    }
