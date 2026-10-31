# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Coordinate and occupancy regressions for the BEV exporter."""

import struct
import unittest
import zlib
from pathlib import Path

import numpy as np
from terrain_debug import (
    RISK_COLORS,
    _frame_directory,
    bev_image,
    png_bytes,
    render_panels,
)


class TerrainDebugTest(unittest.TestCase):
    def test_different_scenes_with_same_frame_name_do_not_overwrite(self):
        root = Path("data/debug")
        frame = {"dataset_group": "nuscenes", "frame_id": "scene1/000000.cloud"}
        first = _frame_directory(root, frame)
        frame["frame_id"] = "scene2/000000.cloud"
        self.assertNotEqual(first, _frame_directory(root, frame))
        frame["frame_id"] = "../../outside"
        with self.assertRaises(ValueError):
            _frame_directory(root, frame)

    def test_png_lossless_and_asymmetric_bev_axes(self):
        cells = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        image = bev_image(cells, 2)
        np.testing.assert_array_equal(image[0, 0], cells[1, 2])
        np.testing.assert_array_equal(image[-1, -1], cells[0, 0])
        png = png_bytes(image)
        self.assertEqual(struct.unpack(">II", png[16:24]), (6, 4))
        offset, pixels = 8, b""
        while offset < len(png):
            length = struct.unpack(">I", png[offset : offset + 4])[0]
            kind = png[offset + 4 : offset + 8]
            data = png[offset + 8 : offset + 8 + length]
            crc = struct.unpack(">I", png[offset + 8 + length : offset + 12 + length])[
                0
            ]
            self.assertEqual(crc, zlib.crc32(kind + data) & 0xFFFFFFFF)
            if kind == b"IDAT":
                pixels += data
            offset += 12 + length
        rows = np.frombuffer(zlib.decompress(pixels), dtype=np.uint8).reshape(4, 19)
        self.assertTrue((rows[:, 0] == 0).all())
        np.testing.assert_array_equal(rows[:, 1:].reshape(4, 6, 3), image)

    def test_overlap_bits_missing_coverage_and_raw_heights_preserved(self):
        shape = (8, 8)
        bits = np.zeros(shape, dtype=np.uint8)
        bits[4, 4], bits[4, 5] = 7, 4
        arrays = {
            "terrain_state": np.full(shape, "UNKNOWN"),
            "decision_state": np.full(shape, "UNKNOWN"),
            "risk_layers": bits,
            "risk_observed": np.zeros(shape, dtype=bool),
            "terrain_connected": np.zeros(shape, dtype=bool),
            "risk_constrained_connected": np.zeros(shape, dtype=bool),
            "xyz_m": np.array([[4.5, 4.5, -2], [4.5, 4.5, 2], [np.nan, 0, 0]]),
            "point_cell_id": np.array([36, 36, -1]),
            "root_cell": np.array([-1, -1]),
        }
        before = bits.copy()
        images = render_panels(arrays, {"x_min": 100, "y_min": 100, "cell_size": 1})
        np.testing.assert_array_equal(images["occupancy"][4, 4], RISK_COLORS[7])
        np.testing.assert_array_equal(images["occupancy"][4, 5], RISK_COLORS[4])
        self.assertFalse(np.array_equal(images["occupancy"][0, 0], RISK_COLORS[0]))
        self.assertGreater(images["bev"][4, 4, 0], 150)
        np.testing.assert_array_equal(before, bits)

    def test_invalid_shapes_bits_and_scale_rejected(self):
        with self.assertRaises(ValueError):
            png_bytes(np.zeros((2, 2, 3)))
        with self.assertRaises(ValueError):
            bev_image(np.zeros((2, 2, 3)), 0)


if __name__ == "__main__":
    unittest.main()
