# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Native adapter order and numerical comparison contracts."""

import tempfile
import unittest
from pathlib import Path

import numpy as np
from terrain_native_validation import (
    BOOL_FIELDS,
    CELL_FIELDS,
    CONFIG_FIELDS,
    STRING_FIELDS,
    compare_fields,
    read_output,
    write_input,
)


class NativeValidationTest(unittest.TestCase):
    def test_native_layer_table_does_not_shift_point_identity(self):
        fields = ["ix", "iy"] + list(CELL_FIELDS)
        values = ["UNKNOWN" if name in STRING_FIELDS else "0" for name in fields]
        text = (
            "TERRAIN_V1_OUTPUT\n"
            'META 1 1 91 0 0 supported OBSERVED_ROOT_VETOED "z up"\n'
            f"CELL_COLUMNS {' '.join(fields)}\n{' '.join(values)}\n"
            "LAYER_COLUMNS ix iy layer_id elevation_z point_count selected "
            "source_point_ids\n"
            "0 0 0 -1.5 2 1 91,7\n"
            "POINT_COLUMNS id cell_id layer_id ground_candidate ground risk "
            "surface_z residual_z\n"
            "91 0 0 1 1 0 -1.5 0\n7 0 0 1 1 0 -1.5 0\n"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "native.txt"
            path.write_text(text)
            meta, cells, points, layers = read_output(path)
        self.assertEqual(meta[7], "OBSERVED_ROOT_VETOED")
        np.testing.assert_array_equal(points["id"], [91, 7])
        self.assertEqual(layers[0]["point_ids"], [91, 7])
        for name in BOOL_FIELDS:
            self.assertEqual(cells[name].dtype.kind, "b")

    def test_input_preserves_noncontiguous_ids_xyz_and_risk_coverage(self):
        arrays = {
            "terrain_state": np.full((1, 2), "UNKNOWN"),
            "point_id": np.array([91, 7]),
            "xyz_m": np.array([[1, 2, 3], [4, 5, np.nan]]),
            "surface_point_risk_mask": np.array([False, True]),
            "risk_layers": np.array([[0, 7]], dtype=np.uint8),
            "risk_observed": np.array([[True, False]]),
        }
        config = dict.fromkeys(CONFIG_FIELDS, 1)
        config["expected_ground_z"] = None
        limits = dict.fromkeys(
            (
                "max_slope_gradient",
                "max_roughness_m",
                "max_step_m",
                "max_depression_depth_m",
            ),
            0.2,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.txt"
            write_input(path, arrays, config, limits, 91)
            lines = path.read_text().splitlines()
        self.assertEqual(lines[0], "TERRAIN_V1")
        self.assertEqual(lines[4], "91 1 2 3 0")
        self.assertEqual(lines[5], "7 4 5 nan 4")
        self.assertEqual(lines[-2:], ["0 0 0 1", "0 1 7 0"])
        self.assertTrue(lines[1].endswith(" 91"))

    def test_state_differences_are_exact_not_numeric_tolerance(self):
        native = {}
        reference = {}
        for name, target in CELL_FIELDS.items():
            value = np.full((1, 1), "UNKNOWN" if name in STRING_FIELDS else 0.0)
            native[name] = value.copy()
            reference[target] = value.copy()
        self.assertEqual(compare_fields(native, reference), {})
        native["elevation_z"][0, 0] = 1e-10
        self.assertEqual(compare_fields(native, reference), {})
        native["decision_state"][0, 0] = "BLOCKED"
        self.assertIn("decision_state", compare_fields(native, reference))


if __name__ == "__main__":
    unittest.main()
