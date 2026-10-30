"""Verify customer previews against real Bazel analysis in the dev container."""

import subprocess
import unittest


class ProductProfileTest(unittest.TestCase):
    def preview(self, *args):
        return subprocess.run(
            ["bash", "apollo.sh", "build", *args, "--show"],
            capture_output=True,
            text=True,
            check=False,
        )

    def test_product_shows_effective_choices_and_output_roots_without_actions(self):
        result = self.preview("--product=devkit-x86")
        self.assertEqual(result.returncode, 0, result.stderr)
        for expected in [
            "Compute: CPU",
            "Mode: opt",
            '"gnss": "huace"',
            '"vehicle": "devkit"',
            '"lidar_livox": False',
            "Modules: rtk_localization, open_space_planning, control",
            "modules/rtk_localization/librtk_localization_component.so",
            "modules/open_space_planning/libopen_space_planning_component.so",
        ]:
            self.assertIn(expected, result.stdout)
        self.assertIn("0 total actions", result.stderr)
        self.assertNotIn("Done building", result.stdout + result.stderr)

    def test_debug_preview_uses_effective_mode_override(self):
        result = self.preview("--product=devkit-x86", "--mode=dbg")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Mode: dbg", result.stdout)
        self.assertIn("0 total actions", result.stderr)

    def test_alternative_product_selects_ndt_and_public_road_planning(self):
        result = self.preview("--product=devkit-x86-ndt")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("Modules: ndt_localization, planning, control", result.stdout)
        self.assertNotIn("librtk_localization_component.so", result.stdout)
        self.assertNotIn("libopen_space_planning_component.so", result.stdout)

    def test_custom_module_typo_fails_in_analysis(self):
        result = self.preview("--modules=rtk_localiztion", "--compute=cpu")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Unknown algorithm modules", result.stderr)
        self.assertNotIn("Done building", result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
