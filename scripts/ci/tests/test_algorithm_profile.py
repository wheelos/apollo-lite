"""Verify whole-module selection using the real configured Bazel graph."""

import json
import re
import subprocess
import sys
import unittest

ROOT = "//tools/modules:algorithms"
RTK = "@wheelos_localization//modules/rtk_localization"
NDT = "@wheelos_localization//modules/ndt_localization"
CONTROL = "@wheelos_control//modules/control"
SIMULATION = "@wheelos_simulation//modules/simulation"
OPTIONS = ["--config=cpu", "--config=opt"]
PROFILE = ["--config=algo_rtk_open_space"]


def bazel(command, *args, options=PROFILE):
    return subprocess.run(
        [
            sys.executable,
            "scripts/bzlmod.py",
            "dev",
            command,
            *(OPTIONS if command != "query" else []),
            *options,
            *args,
        ],
        capture_output=True,
        text=True,
        check=False,
    )


class AlgorithmProfileTest(unittest.TestCase):
    def test_cpu_extracted_packages_have_no_product_dependencies(self):
        roots = [
            f"{CONTROL}:libcontrol_component.so",
            "@wheelos_world_model//modules/lane:ufldv2",
            "@wheelos_mission//modules/mission:libmission_component.so",
            "@wheelos_routing//modules/routing:routing_service",
            f"{SIMULATION}:libsimulation_component.so",
        ]
        result = bazel(
            "cquery",
            'filter("^//", deps(set(' + " ".join(roots) + ")))",
            options=[],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "")

    def test_cpu_localization_packages_have_no_product_dependencies(self):
        roots = [
            f"{RTK}:librtk_localization_component.so",
            f"{NDT}:libndt_localization_component.so",
            "@wheelos_localization//modules/localization:liblocalization_component.so",
        ]
        result = bazel(
            "cquery",
            'filter("^//", deps(set(' + " ".join(roots) + ")))",
            options=[],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "")

    def test_capability_roots_have_no_product_or_hmi_implementation_dependencies(self):
        roots = [
            "@wheelos_lidar_perception//modules/lidar_fusion:"
            "liblidar_fusion_component.so",
            "@wheelos_camera_perception//modules/"
            "camera_semantic_segmentation:segformer",
            "@wheelos_multimodal_perception//modules/bevfusion:bevfusion_input",
            "@wheelos_world_model//modules/world_model/local_map:local_scene_builder",
            "@wheelos_planning//modules/open_space_planning:"
            "libopen_space_planning_component.so",
            "@wheelos_planning//modules/map/tools:map_tool",
            "@wheelos_prediction//modules/prediction:libprediction_component.so",
            "@wheelos_world_model//modules/storytelling:libstorytelling_component.so",
            "@wheelos_safety//modules/monitor:monitor_lib",
            "@wheelos_hmi//modules/dreamview:dreamview",
        ]
        result = bazel("cquery", 'filter("^//", deps(set(' + " ".join(roots) + ")))")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "")
        result = bazel(
            "cquery",
            'filter("wheelos_hmi", deps(@wheelos_safety//modules/monitor:monitor_lib))',
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "")

    def closure(self, *options):
        result = bazel(
            "cquery",
            f"deps({ROOT})",
            "--output=label",
            options=options or PROFILE,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        return re.sub(r"@@(wheelos_[^/+]+)\+", r"@\1", result.stdout)

    def test_selected_production_roots_exclude_other_components_and_tools(self):
        labels = self.closure()
        for selected in [
            f"{RTK}:librtk_localization_component.so",
            "@wheelos_planning//modules/open_space_planning:"
            "libopen_space_planning_component.so",
            f"{CONTROL}:libcontrol_component.so",
        ]:
            self.assertIn(selected, labels)
        for excluded in [
            f"{NDT}:",
            "@wheelos_planning//modules/planning:libplanning_component.so",
            "@wheelos_prediction//modules/prediction:libprediction_component.so",
            "@hardware.camera//",
            "@hardware.camera_gst//",
            "@hardware.image_decompress//",
            "@hardware.hal//",
            "@hardware.canbus//",
            "@hardware.lidar//",
            "@hardware.radar//",
            "@hardware.gnss//",
            "@hardware.audio//",
            "@hardware.video//",
            "@hardware.build//",
            "@wheelos_hmi//modules/dreamview:",
            f"{SIMULATION}:",
            f"{CONTROL}/tools",
            f"{RTK}:rtk_localization_test",
        ]:
            self.assertNotIn(excluded, labels)

    def test_overriding_inventory_replaces_previous_modules(self):
        labels = self.closure(
            "--config=algo_rtk_open_space",
            "--//tools/modules:include=ndt_localization,planning,control",
        )
        self.assertIn(f"{NDT}:libndt_localization_component.so", labels)
        self.assertIn(
            "@wheelos_planning//modules/planning:libplanning_component.so", labels
        )
        self.assertNotIn(f"{RTK}:", labels)
        self.assertNotIn(
            "@wheelos_planning//modules/open_space_planning:"
            "libopen_space_planning_component.so",
            labels,
        )

    def test_local_planning_selects_library_not_simulator(self):
        labels = self.closure("--//tools/modules:include=local_planning")
        self.assertIn(
            "@wheelos_planning//modules/local_planning:lane_follow_planner", labels
        )
        self.assertNotIn(
            "@wheelos_planning//modules/local_planning:closed_loop", labels
        )
        self.assertNotIn(f"{SIMULATION}:", labels)

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
            re.sub(r"@@(wheelos_[^/+]+)\+", r"@\1", targets[action["targetId"]])
            for action in graph["actions"]
            if action["mnemonic"] in ("CppCompile", "CppLink")
        }
        self.assertIn(f"{RTK}:rtk_localization_lib", labels)
        for label in labels:
            self.assertFalse(
                label.startswith(
                    (
                        f"{NDT}/",
                        f"{NDT}:",
                        "@hardware.camera//",
                        "@hardware.camera_gst//",
                        "@hardware.image_decompress//",
                        "@hardware.hal//",
                        "@hardware.canbus//",
                        "@hardware.lidar//",
                        "@hardware.radar//",
                        "@hardware.gnss//",
                        "@hardware.audio//",
                        "@hardware.video//",
                        "@hardware.build//",
                        "@wheelos_hmi//modules/dreamview/",
                    )
                ),
                label,
            )
        self.assertNotIn(
            "@wheelos_planning//modules/planning:libplanning_component.so", labels
        )

    def test_all_registered_roots_exist_and_are_not_coverage_targets(self):
        result = bazel(
            "query",
            f"labels(srcs, {ROOT}) except //tools/modules:include",
            "--output=label_kind",
            options=[],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(f"{CONTROL}:libcontrol_component.so", result.stdout)
        self.assertNotIn("cc_test rule", result.stdout)
        self.assertNotIn(":install", result.stdout)


if __name__ == "__main__":
    unittest.main()
