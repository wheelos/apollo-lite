"""Verify whole-module selection using the real configured Bazel graph."""

import json
import subprocess
import unittest

ROOT = "//tools/modules:algorithms"
OPTIONS = ["--config=cpu", "--config=opt"]
PROFILE = ["--config=algo_rtk_open_space"]


def bazel(command, *args, options=PROFILE):
    return subprocess.run(
        ["bazel", command, *(OPTIONS if command != "query" else []), *options, *args],
        capture_output=True,
        text=True,
        check=False,
    )


class AlgorithmProfileTest(unittest.TestCase):
    def closure(self, *options):
        result = bazel(
            "cquery",
            f"deps({ROOT})",
            "--output=label",
            options=options or PROFILE,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return result.stdout

    def test_selected_production_roots_exclude_other_components_and_tools(self):
        labels = self.closure()
        for selected in [
            "//modules/rtk_localization:librtk_localization_component.so",
            "//modules/open_space_planning:libopen_space_planning_component.so",
            "//modules/control:libcontrol_component.so",
        ]:
            self.assertIn(selected, labels)
        for excluded in [
            "//modules/ndt_localization:",
            "//modules/planning:libplanning_component.so",
            "//modules/prediction:libprediction_component.so",
            "//modules/drivers/",
            "//modules/dreamview:",
            "//modules/simulation:",
            "//modules/control/tools",
            "//modules/rtk_localization:rtk_localization_test",
        ]:
            self.assertNotIn(excluded, labels)

    def test_overriding_inventory_replaces_previous_modules(self):
        labels = self.closure(
            "--config=algo_rtk_open_space",
            "--//tools/modules:include=ndt_localization,planning,control",
        )
        self.assertIn(
            "//modules/ndt_localization:libndt_localization_component.so", labels
        )
        self.assertIn("//modules/planning:libplanning_component.so", labels)
        self.assertNotIn("//modules/rtk_localization:", labels)
        self.assertNotIn(
            "//modules/open_space_planning:libopen_space_planning_component.so", labels
        )

    def test_local_planning_selects_library_not_simulator(self):
        labels = self.closure("--//tools/modules:include=local_planning")
        self.assertIn("//modules/local_planning:lane_follow_planner", labels)
        self.assertNotIn("//modules/local_planning:closed_loop", labels)
        self.assertNotIn("//modules/simulation:", labels)

    def test_empty_and_unknown_inventories_are_rejected(self):
        for options, message in [
            ([], "Select algorithm modules"),
            (["--//tools/modules:include="], "Select algorithm modules"),
            (["--//tools/modules:include=rtk_localization,typo"], "Unknown algorithm"),
        ]:
            with self.subTest(options=options):
                result = bazel("build", "--nobuild", ROOT, options=options)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)

    def test_action_graph_does_not_compile_unselected_modules(self):
        result = bazel(
            "aquery",
            f"deps({ROOT})",
            "--output=jsonproto",
            "--include_artifacts=false",
            "--include_commandline=false",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        graph = json.loads(result.stdout)
        targets = {target["id"]: target["label"] for target in graph["targets"]}
        labels = {
            targets[action["targetId"]]
            for action in graph["actions"]
            if action["mnemonic"] in ("CppCompile", "CppLink")
        }
        self.assertIn("//modules/rtk_localization:rtk_localization_lib", labels)
        for label in labels:
            self.assertFalse(
                label.startswith(
                    (
                        "//modules/ndt_localization/",
                        "//modules/ndt_localization:",
                        "//modules/drivers/",
                        "//modules/dreamview/",
                    )
                ),
                label,
            )
        self.assertNotIn("//modules/planning:libplanning_component.so", labels)

    def test_all_registered_roots_exist_and_are_not_coverage_targets(self):
        result = bazel(
            "query",
            f"labels(srcs, {ROOT}) except //tools/modules:include",
            "--output=label_kind",
            options=[],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("//modules/control:libcontrol_component.so", result.stdout)
        self.assertNotIn("cc_test rule", result.stdout)
        self.assertNotIn(":install", result.stdout)


if __name__ == "__main__":
    unittest.main()
