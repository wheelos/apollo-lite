"""Geometry regression tests for the diagnostic OpenLane evaluator."""

import unittest

from openlane_lane_evaluator import (
    camera_point_to_ego,
    greedy_matches,
    project_ego_point,
    symmetric_chamfer,
)


class OpenLaneEvaluatorTest(unittest.TestCase):
    def setUp(self):
        self.camera = {
            "extrinsic": {
                "rotation": [[0, 0, 1], [-1, 0, 0], [0, -1, 0]],
                "translation": [1, 0, 1.5],
            },
            "intrinsic": {
                "K": [[100, 0, 50], [0, 100, 50], [0, 0, 1]],
                "distortion": [0.1, 0, 0],
            },
        }

    def test_rectified_projection_uses_camera_to_ego_inverse(self):
        self.assertEqual(
            project_ego_point([11, -2, 0], self.camera), (70.0, 65.0))
        self.assertEqual(
            camera_point_to_ego([2, 1.5, 10], self.camera), [11, -2, 0])
        self.assertIsNone(project_ego_point([0, 0, 0], self.camera))

    def test_radial_projection_is_explicit_not_applied_twice(self):
        projected = project_ego_point(
            [11, -2, 0], self.camera, image_domain="radial")
        self.assertAlmostEqual(projected[0], 70.125)
        self.assertAlmostEqual(projected[1], 65.09375)

    def test_chamfer_measures_polylines_not_only_vertices(self):
        self.assertAlmostEqual(
            symmetric_chamfer([[0, 0], [2, 0]], [[0, 1], [2, 1]]), 1.0)
        self.assertEqual(
            symmetric_chamfer(
                [[0, 0], [1, 0], [2, 0]], [[0, 0], [2, 0]]), 0.0)
        self.assertIsNone(symmetric_chamfer([], [[0, 0], [2, 0]]))

    def test_matching_is_one_to_one_and_thresholded(self):
        matches, predictions, labels = greedy_matches(
            [[0.1, 2.0], [0.2, 3.0]], 1.0)
        self.assertEqual(matches, [(0.1, 0, 0)])
        self.assertEqual(predictions, {0})
        self.assertEqual(labels, {0})


if __name__ == "__main__":
    unittest.main()
