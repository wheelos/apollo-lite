#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import tempfile
import unittest
from pathlib import Path

import numpy as np

from nuscenes_sweeps import build_model_input
from prepare_nuscenes_manifest import invert_rigid_transform


class NuScenesSweepsTest(unittest.TestCase):
    def test_inverts_reference_to_sweep_transform(self):
        lidar2sensor = [
            [1.0, 0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0, -2.0],
            [0.0, 0.0, 1.0, -3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

        self.assertEqual(
            invert_rigid_transform(lidar2sensor, "lidar2sensor"),
            [
                [1.0, 0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0, 2.0],
                [0.0, 0.0, 1.0, 3.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        )

    def test_builds_transformed_time_lagged_model_features(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary = Path(temporary_directory)
            current_path = temporary / "current.pcd.bin"
            sweep_path = temporary / "sweep.pcd.bin"
            output_path = temporary / "model.bin"
            np.array(
                [[10.0, 0.0, 1.0, 0.25, 3.0]],
                dtype="<f4",
            ).tofile(current_path)
            np.array(
                [
                    [0.5, 0.5, 0.0, 1.0, 4.0],
                    [1.5, 0.0, 1.0, 2.0, 5.0],
                    [0.0, 2.0, 0.0, 3.0, 6.0],
                ],
                dtype="<f4",
            ).tofile(sweep_path)
            record = {
                "source_path": str(current_path),
                "sweeps": [
                    {
                        "source_path": str(sweep_path),
                        "sweep_to_lidar": [
                            [1.0, 0.0, 0.0, 1.0],
                            [0.0, 1.0, 0.0, -1.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0],
                        ],
                        "time_lag": 0.1,
                        "remove_close": True,
                    }
                ],
            }

            count = build_model_input(record, output_path)
            result = np.fromfile(output_path, dtype="<f4").reshape(-1, 5)

        self.assertEqual(count, 3)
        np.testing.assert_allclose(
            result,
            [
                [10.0, 0.0, 1.0, 0.25, 0.0],
                [2.5, -1.0, 1.0, 2.0, 0.1],
                [1.0, 1.0, 0.0, 3.0, 0.1],
            ],
            atol=1.0e-6,
        )


if __name__ == "__main__":
    unittest.main()
