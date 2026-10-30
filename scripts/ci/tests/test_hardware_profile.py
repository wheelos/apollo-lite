"""Configured-graph and negative checks; run in the managed dev container."""

import json
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from pathlib import Path

ROOT = "//tools/hardware:hardware"
OPTIONS = ["--config=cpu", "--config=opt", "--config=hw_x86_devkit"]


def bazel(*args, options=OPTIONS):
    result = subprocess.run(
        [sys.executable, "scripts/bzlmod.py", "dev", args[0], *options, *args[1:]],
        capture_output=True,
        text=True,
        check=False,
    )
    result.stdout = re.sub(r"@@(hardware\.[a-z_]+)\+", r"@\1", result.stdout)
    return result


class HardwareProfileTest(unittest.TestCase):
    def test_functional_sources_are_shallow_and_complete(self):
        hardware = Path(__file__).resolve().parents[3] / "pkgs/hardware"
        for module, source in {
            "camera": "camera_component.cc",
            "camera_gst": "camera_gst_component.cc",
            "canbus": "can_client/can_client_factory.cc",
            "gnss": "gnss_component.cc",
            "hal": "stream/stream_factory.cc",
            "lidar": "lidar_driver_component.cc",
            "audio": "microphone_component.cc",
            "radar": "conti_radar/conti_radar_canbus_component.cc",
            "image_decompress": "image_decompress.cc",
            "video": "video_driver_component.cc",
            "vehicle": "canbus_component.cc",
        }.items():
            with self.subTest(module=module):
                self.assertTrue((hardware / module / "MODULE.bazel").is_file())
                self.assertTrue((hardware / module / source).is_file())
                self.assertFalse((hardware / module / "modules").exists())
        self.assertTrue((hardware / "vehicle/brands/devkit").is_dir())
        self.assertFalse((hardware / "vehicle/vehicle").exists())
        self.assertFalse((hardware / "common").exists())
        self.assertTrue((hardware / "build/MODULE.bazel").is_file())

    def test_camera_only_consumer_does_not_resolve_other_hardware(self):
        workspace = Path(__file__).resolve().parents[3]
        overrides = []
        for line in (workspace / ".bazelrc.dev").read_text().splitlines():
            if line.startswith("common:dev --override_module="):
                option = line.split(maxsplit=1)[1]
                name = option.split("=", 2)[1]
                if name in (
                    "hardware.camera",
                    "hardware.build",
                    "wheelos_core",
                    "wheelos_common",
                    "wheelos_msgs",
                ):
                    overrides.append(option.replace("%workspace%", str(workspace)))
        registries = [
            line.split(maxsplit=1)[1]
            for line in (workspace / ".bazelrc").read_text().splitlines()
            if line.startswith("common --registry=")
        ]
        registries.append(f"--registry=file://{workspace}/pkgs/bazel-central-registry")
        startup = [
            shutil.which("bazelisk") or "bazel",
            "--batch",
            "--ignore_all_rc_files",
            "--host_jvm_args=-Xmx1g",
        ]
        resolution = ["--lockfile_mode=off", *registries, *overrides]
        with tempfile.TemporaryDirectory(prefix="camera-only-") as directory:
            fixture = Path(directory)
            (fixture / "MODULE.bazel").write_text(
                'module(name = "camera_only_consumer")\n'
                'bazel_dep(name = "hardware.camera", version = "0.1.0")\n'
            )
            (fixture / ".bazelversion").write_text(
                (workspace / ".bazelversion").read_text()
            )
            (fixture / "BUILD.bazel").write_text('exports_files(["MODULE.bazel"])\n')
            result = subprocess.run(
                [*startup, "mod", "graph", *resolution],
                cwd=fixture,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            component = subprocess.run(
                [
                    *startup,
                    "cquery",
                    "@hardware.camera//:component",
                    "--output=files",
                    *resolution,
                ],
                cwd=fixture,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(component.returncode, 0, component.stderr)
            self.assertIn("libcamera_component.so", component.stdout)
        self.assertIn("hardware.camera@", result.stdout)
        self.assertIn("hardware.build@", result.stdout)
        for excluded in (
            "wheelos_transform",
            "hardware.canbus",
            "hardware.camera_gst",
            "hardware.image_decompress",
            "hardware.hal",
            "hardware.vehicle",
            "hardware.lidar",
            "hardware.radar",
            "hardware.gnss",
            "hardware.audio",
            "hardware.video",
            "hesai-lidar-sdk",
            "livox-sdk",
            "inno-lidar-sdk",
            "vanjee-lidar-sdk",
            "robosense-lidar-sdk",
        ):
            self.assertNotIn(excluded + "@", result.stdout)

    def assert_rejected(self, option, message):
        result = bazel("build", "--nobuild", ROOT, option)
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(message, result.stdout + result.stderr)

    def test_legacy_install_rules_share_build_owned_provider(self):
        result = bazel(
            "cquery",
            "set(@hardware.camera//:install "
            "@wheelos_transform//modules/transform:install)",
            "--output=starlark",
            "--starlark:expr=str(sorted(providers(target).keys()))",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = result.stdout.splitlines()
        self.assertEqual(len(rows), 2, result.stdout)
        for row in rows:
            self.assertIn("hardware.build", row)
            self.assertIn("//tools/install:providers.bzl%InstallInfo", row)
            self.assertNotIn("wheelos_transform", row)

    def test_configured_closure_excludes_unselected_implementations(self):
        result = bazel("cquery", f"deps({ROOT})", "--output=label")
        self.assertEqual(result.returncode, 0, result.stderr)
        labels = result.stdout
        self.assertIn("//tools/hardware:runtime", labels)
        self.assertNotIn("//tools/hardware:install", labels)
        self.assertIn("@hardware.gnss//parser/huace:", labels)
        self.assertIn("@hardware.vehicle//brands/devkit:", labels)
        self.assertIn("@hardware.lidar//hesai:", labels)
        for path in [
            "@hardware.gnss//parser/novatel:",
            "@hardware.gnss//parser/forsense:",
            "@hardware.lidar//velodyne/",
            "@hardware.lidar//rslidar:",
            "@hardware.lidar//livox:",
            "@hardware.camera_gst//",
            "can_client/fake:",
            "@hardware.vehicle//brands/lincoln:",
            "@hardware.vehicle//brands/ge3:",
            "@hardware.vehicle//brands/gem:",
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
        self.assertNotIn(
            "PackageTar", [action["mnemonic"] for action in graph["actions"]]
        )
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
                "//brands/lincoln",
                "//rslidar:",
                "livox-sdk",
                "robosense-lidar-sdk",
                "vanjee-lidar-sdk",
                "inno-lidar-sdk",
            ]:
                self.assertNotIn(excluded, label)

    def test_invalid_enum(self):
        self.assert_rejected(
            "--@hardware.build//tools/hardware:gnss=typo", "invalid value"
        )

    def test_legacy_vehicle_define_still_selects_devkit(self):
        result = bazel(
            "cquery",
            "deps(@hardware.vehicle//brands:vehicle_factory)",
            "--output=label",
            options=["--config=cpu", "--config=opt", "--define=vehicle=devkit"],
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("@hardware.vehicle//brands/devkit:", result.stdout)
        self.assertNotIn("@hardware.vehicle//brands/lincoln:", result.stdout)

    def test_conflicting_vehicle_flags(self):
        self.assert_rejected("--define=vehicle=lincoln", "Conflicting vehicle")

    def test_vehicle_requires_can(self):
        self.assert_rejected(
            "--@hardware.build//tools/hardware:canbus=none", "requires CAN transport"
        )

    def test_wrong_camera_platform(self):
        self.assert_rejected(
            "--@hardware.build//tools/hardware:camera=camera_gst", "incompatible"
        )

    def test_multiple_lidar_brands(self):
        result = bazel(
            "cquery",
            f"deps({ROOT})",
            "--output=label",
            "--@hardware.build//tools/hardware:lidar_livox=true",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("@hardware.lidar//hesai:", result.stdout)
        self.assertIn("@hardware.lidar//livox:", result.stdout)

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
            for path in manifest["component_files"]:
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
            "--@hardware.build//tools/hardware:camera=none",
            "--@hardware.build//tools/hardware:canbus=none",
            "--@hardware.build//tools/hardware:gnss=none",
            "--@hardware.build//tools/hardware:vehicle=none",
            "--define=vehicle=none",
            "--@hardware.build//tools/hardware:lidar_hesai=false",
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
