#!/usr/bin/env python3
# Copyright 2026 WheelOS. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import math
import unittest

from evaluate_nuscenes import create_result


class EvaluateNuscenesTest(unittest.TestCase):
    def test_converts_center_size_yaw_and_sample_token(self):
        token = "fd8420396768425eabec9bdddf7e64b6"
        identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        manifest = {
            token: {
                "sample_token": token,
                "lidar2ego": identity,
                "ego2global": identity,
            }
        }
        predictions = {
            token: [
                {
                    "center": [1.0, 2.0, 3.0],
                    "size": [2.0, 4.0, 6.0],
                    "velocity": [0.0, 0.0],
                    "yaw": -math.pi / 2.0,
                    "class_id": 0,
                    "score": 0.75,
                }
            ]
        }

        result = create_result(manifest, predictions)
        detection = result["results"][token][0]

        self.assertEqual(detection["translation"], [1.0, 2.0, 3.0])
        self.assertEqual(detection["size"], [2.0, 4.0, 6.0])
        self.assertEqual(detection["rotation"], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(detection["detection_name"], "car")
        self.assertEqual(detection["attribute_name"], "vehicle.parked")

    def test_rejects_incomplete_prediction_tokens(self):
        with self.assertRaisesRegex(ValueError, "token mismatch"):
            create_result({"expected": {}}, {})

    def test_converts_bevfusion_output_and_bottom_center_z(self):
        token = "fd8420396768425eabec9bdddf7e64b6"
        identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        manifest = {
            token: {
                "sample_token": token,
                "lidar2ego": identity,
                "ego2global": identity,
            }
        }
        predictions = {
            token: [
                {
                    "center_x": 1.0,
                    "center_y": 2.0,
                    "center_z": -1.0,
                    "width": 2.0,
                    "length": 4.0,
                    "height": 2.0,
                    "velocity_x": 0.0,
                    "velocity_y": 0.0,
                    "yaw": -math.pi / 2.0,
                    "class_id": 0,
                    "score": 0.75,
                }
            ]
        }

        result = create_result(
            manifest,
            predictions,
            {
                token: {
                    "center_z": (
                        "bottom-center z, as decoded by the NVIDIA upstream core"
                    )
                }
            },
        )
        detection = result["results"][token][0]

        self.assertFalse(result["meta"]["use_camera"])
        self.assertEqual(detection["translation"], [1.0, 2.0, 0.0])
        self.assertEqual(detection["size"], [2.0, 4.0, 2.0])
        self.assertEqual(detection["detection_name"], "car")


if __name__ == "__main__":
    unittest.main()
