"""Configured-graph and negative checks; run in the managed dev container."""

import json
import re
import subprocess
import tarfile
import unittest
from pathlib import Path

ROOT = "//tools/hardware:hardware"
OPTIONS = ["--config=cpu", "--config=opt", "--config=hw_x86_devkit"]


def bazel(*args, options=OPTIONS):
    return subprocess.run(
        ["bazel", args[0], *options, *args[1:]],
        capture_output=True,
        text=True,
        check=False,
    )


class HardwareProfileTest(unittest.TestCase):
    def assert_rejected(self, option, message):
        result = bazel("build", "--nobuild", ROOT, option)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(message, result.stdout + result.stderr)

    def test_configured_closure_excludes_unselected_implementations(self):
        result = bazel("cquery", f"deps({ROOT})", "--output=label")
        self.assertEqual(result.returncode, 0, result.stderr)
        labels = result.stdout
        self.assertIn("//modules/drivers/gnss/parser/huace:", labels)
        self.assertIn("//modules/canbus/vehicle/devkit:", labels)
        self.assertIn("//modules/drivers/lidar/hesai:", labels)
        for path in [
            "gnss/parser/novatel:",
            "gnss/parser/forsense:",
            "lidar/velodyne/",
            "lidar/rslidar:",
            "lidar/livox:",
            "camera_gst:",
            "can_client/fake:",
            "vehicle/lincoln:",
            "vehicle/ge3:",
            "vehicle/gem:",
        ]:
            self.assertNotIn(path, labels)

    def test_action_graph_has_no_other_brand_compiles(self):
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
        compiled = [
            targets[action["targetId"]]
            for action in graph["actions"]
            if action["mnemonic"] in ("CppCompile", "CppLink")
        ]
        self.assertTrue(any("huace_parser" in label for label in compiled))
        for label in compiled:
            for excluded in [
                "/novatel:",
                "/forsense:",
                "/vehicle/lincoln",
                "/lidar/rslidar",
                "livox-sdk",
                "robosense-lidar-sdk",
                "vanjee-lidar-sdk",
                "inno-lidar-sdk",
            ]:
                self.assertNotIn(excluded, label)

    def test_invalid_enum(self):
        self.assert_rejected("--//tools/hardware:gnss=typo", "invalid value")

    def test_legacy_vehicle_define_still_selects_devkit(self):
        result = bazel(
            "cquery",
            "deps(//modules/canbus/vehicle:vehicle_factory)",
            "--output=label",
            options=["--config=cpu", "--config=opt", "--define=vehicle=devkit"],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("//modules/canbus/vehicle/devkit:", result.stdout)
        self.assertNotIn("//modules/canbus/vehicle/lincoln:", result.stdout)

    def test_conflicting_vehicle_flags(self):
        self.assert_rejected("--define=vehicle=lincoln", "Conflicting vehicle")

    def test_vehicle_requires_can(self):
        self.assert_rejected("--//tools/hardware:canbus=none", "requires CAN transport")

    def test_wrong_camera_platform(self):
        self.assert_rejected("--//tools/hardware:camera=camera_gst", "incompatible")

    def test_multiple_lidar_brands(self):
        result = bazel(
            "cquery",
            f"deps({ROOT})",
            "--output=label",
            "--//tools/hardware:lidar_livox=true",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("//modules/drivers/lidar/hesai:", result.stdout)
        self.assertIn("//modules/drivers/lidar/livox:", result.stdout)

    def test_selected_package_is_consistent(self):
        archive = Path("bazel-bin/tools/hardware/install.tar.gz")
        self.assertTrue(archive.is_file(), "build //tools/hardware:install first")
        with tarfile.open(archive) as package:
            members = {
                member.name.removeprefix("./"): member
                for member in package.getmembers()
                if member.isfile()
            }
            manifest = json.load(package.extractfile(members["hardware/manifest.json"]))
            self.assertEqual(manifest["choices"]["gnss"], "huace")
            self.assertEqual(manifest["choices"]["vehicle"], "devkit")
            for label in manifest["components"]:
                path = "bazel-bin/" + label.split("//", 1)[1].replace(":", "/")
                self.assertIn(path, members)
            for path, member in members.items():
                if path.endswith(".dag"):
                    text = package.extractfile(member).read().decode()
                    for reference in re.findall(
                        r'(?:module_library|config_file_path)\s*:\s*"/apollo/([^"]+)"',
                        text,
                    ):
                        self.assertIn(reference, members, path)
            gnss = package.extractfile(
                members["modules/drivers/gnss/conf/gnss_conf.pb.txt"]
            ).read()
            self.assertIn(b"HUACE_TEXT", gnss)
            self.assertNotIn(b"NOVATEL_BINARY", gnss)
            vehicle = package.extractfile(
                members["modules/canbus/conf/canbus_conf.pb.txt"]
            ).read()
            self.assertIn(b"brand: DKIT", vehicle)
            self.assertIn(b"COMPLETE_MANUAL", vehicle)
            self.assertFalse(any("velodyne" in name for name in members))

    def test_switching_profile_does_not_package_stale_outputs(self):
        result = bazel(
            "build",
            "//tools/hardware:install",
            "--//tools/hardware:camera=none",
            "--//tools/hardware:canbus=none",
            "--//tools/hardware:gnss=none",
            "--//tools/hardware:vehicle=none",
            "--define=vehicle=none",
            "--//tools/hardware:lidar_hesai=false",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        try:
            with tarfile.open("bazel-bin/tools/hardware/install.tar.gz") as package:
                files = [
                    member.name.removeprefix("./")
                    for member in package.getmembers()
                    if member.isfile()
                ]
                self.assertFalse(
                    any(name.startswith(("modules/", "bazel-bin/")) for name in files)
                )
        finally:
            restored = bazel("build", "//tools/hardware:install", "--jobs=4")
            self.assertEqual(restored.returncode, 0, restored.stderr)


if __name__ == "__main__":
    unittest.main()
