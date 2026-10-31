# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Independent planning cost/availability layers; no guessed weighted sum."""

import numpy as np
from evaluate_baseline import require
from spatial_support.terrain import INFERRED, MEASURED, UNKNOWN_RISK


def layers(scene, max_slope, max_step, max_roughness):
    """Use caller limits for both ratios and diagnostic terrain admissibility.

    Scene limits belong to its other queries; they are not cost normalization
    limits. Missing metrics remain unsupported, never an available zero cost.
    """
    require(
        all(np.isfinite(v) and v > 0 for v in (max_slope, max_step, max_roughness)),
        "positive explicit cost normalization limits required",
    )
    supported = (
        np.isfinite(scene.ground_z)
        & np.isfinite(scene.slope)
        & np.isfinite(scene.step)
        & np.isfinite(scene.roughness)
        & scene.ego_connected
    )
    limited = (
        (scene.slope > max_slope)
        | (scene.step > max_step)
        | (scene.roughness > max_roughness)
    )
    return {
        "measured_obstacle": (scene.layers & MEASURED) != 0,
        "inferred_obstacle": (scene.layers & INFERRED) != 0,
        "unknown_obstacle": (scene.layers & UNKNOWN_RISK) != 0,
        "unsupported": ~supported,
        "unverified_free": ~scene.verified_free,
        "slope_ratio": scene.slope / max_slope,
        "step_ratio": scene.step / max_step,
        "roughness_ratio": scene.roughness / max_roughness,
        "terrain_limit": limited,
        "admissible_diagnostic": (
            supported & scene.verified_free & ~limited & (scene.layers == 0)
        ),
    }
