"""Transform module boundaries; run in the managed development container."""

import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]
FFMPEG_OPTIONS = [
    "--repo_env=FFMPEG_MODE=development",
    "--repo_env=FFMPEG_INCLUDE_DIR=/opt/apollo/sysroot/include",
    "--repo_env=FFMPEG_LIBRARY_DIR=/opt/apollo/sysroot/lib",
    "--repo_env=FFMPEG_RUNTIME_LIBRARY_DIR=/opt/apollo/sysroot/lib",
    "--repo_env=FFMPEG_RUNTIME_LIBRARIES="
    "libavcodec.so.61,libavformat.so.61,libavutil.so.59,"
    "libswscale.so.8,libswresample.so.5",
]


def bazel(*args):
    return subprocess.run(
        [
            sys.executable,
            "scripts/bzlmod.py",
            "dev",
            args[0],
            *([] if args[0] == "query" else ["--config=cpu", "--config=opt"]),
            *FFMPEG_OPTIONS,
            *args[1:],
        ],
        cwd=WORKSPACE,
        capture_output=True,
        text=True,
        check=False,
    )


def tree_contents(directory, include_modes=False):
    return {
        path.relative_to(directory).as_posix(): (
            hashlib.sha256(path.read_bytes()).hexdigest(),
            path.stat().st_mode & 0o777 if include_modes else None,
        )
        for path in directory.rglob("*")
        if path.is_file()
    }


class OfficialInstallerTest(unittest.TestCase):
    def test_product_configuration_installs_with_resolvable_vehicle_path(self):
        with tempfile.TemporaryDirectory(prefix="product-config-") as directory:
            result = bazel("run", "//conf:install_dir", "--", "--destdir", directory)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            installed = Path(directory)
            flags = (installed / "conf/global_flagfile.txt").read_text()
            vehicle = next(
                line.split("=", 1)[1]
                for line in flags.splitlines()
                if line.startswith("--vehicle_config_path=")
            )
            self.assertFalse(Path(vehicle).is_absolute())
            self.assertEqual(
                (installed / vehicle).read_bytes(),
                (WORKSPACE / "conf/vehicles/default/vehicle_param.pb.txt").read_bytes(),
            )

    def test_extracted_cpu_runtime_installation_preserves_resources(self):
        for module, resource, library in (
            ("control", "conf/control_conf.pb.txt", "libcontrol_component.so"),
            ("mission", "conf/bt_trees/loop.xml", "libmission_component.so"),
            ("routing", "conf/routing_config.pb.txt", None),
            ("simulation", "model/ackermann_vehicle.xml", "libsimulation_component.so"),
        ):
            with (
                self.subTest(module=module),
                tempfile.TemporaryDirectory(
                    prefix=f"{module}-runtime-install-"
                ) as directory,
            ):
                result = bazel(
                    "run",
                    f"@wheelos_{module}//modules/{module}:install_dir",
                    "--",
                    "--destdir",
                    directory,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                installed = Path(directory) / module / resource
                source = WORKSPACE / "pkgs" / module / "modules" / module / resource
                self.assertEqual(installed.read_bytes(), source.read_bytes())
                self.assertEqual(installed.stat().st_mode & 0o777, 0o644)
                if library:
                    plugin = Path(directory) / module / "lib" / library
                    self.assertEqual(plugin.stat().st_mode & 0o777, 0o755)

    def test_source_installation_includes_nested_bazel_packages(self):
        for module, source in (
            ("control", "safety/safety_manager.cc"),
            ("map", "pnc_map/path.cc"),
        ):
            label = (
                "@wheelos_control//modules/control"
                if module == "control"
                else f"@wheelos_planning//modules/{module}"
            )
            source_root = (
                WORKSPACE / "pkgs/control/modules/control"
                if module == "control"
                else WORKSPACE / "pkgs/planning/modules/map"
            )
            with (
                self.subTest(module=module),
                tempfile.TemporaryDirectory(
                    prefix="module-source-install-"
                ) as directory,
            ):
                result = bazel(
                    "run",
                    f"{label}:source_dir",
                    "--",
                    "--destdir",
                    directory,
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                installed = Path(directory) / module / "src" / source
                self.assertEqual(
                    installed.read_bytes(),
                    (source_root / source).read_bytes(),
                )
                self.assertEqual(installed.stat().st_mode & 0o777, 0o644)

    def test_directory_installation_preserves_source_runtime(self):
        for module in ("transform", "hardware"):
            with (
                self.subTest(module=module),
                tempfile.TemporaryDirectory(
                    prefix=f"{module}-official-install-"
                ) as directory,
            ):
                destination = Path(directory) / "runtime"
                options = ["--config=hw_x86_devkit"] if module == "hardware" else []
                result = bazel(
                    "run",
                    *options,
                    f"//tools/{module}:install_dir",
                    "--",
                    "--destdir",
                    str(destination),
                )
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                source = WORKSPACE / f"bazel-bin/tools/{module}/runtime"
                contents = tree_contents(source)
                self.assertTrue(contents)
                self.assertEqual(tree_contents(destination), contents)
                if module == "hardware":
                    flags = (
                        destination / "conf/global_config/global_flagfile.txt"
                    ).read_text()
                    vehicle = next(
                        line.split("=", 1)[1]
                        for line in flags.splitlines()
                        if line.startswith("--vehicle_config_path=")
                    )
                    self.assertFalse(Path(vehicle).is_absolute())
                    self.assertEqual(
                        (destination / vehicle).read_bytes(),
                        (
                            WORKSPACE / "conf/vehicles/default/vehicle_param.pb.txt"
                        ).read_bytes(),
                    )
                if module == "transform":
                    self.assertTrue(
                        (destination / "dag/transform/static_transform.dag").is_file()
                    )
                    self.assertTrue(
                        (
                            destination / "launch/transform/static_transform.launch"
                        ).is_file()
                    )
                    self.assertTrue(
                        (
                            destination
                            / "lib/components/transform"
                            / "libstatic_transform_component.so"
                        ).is_file()
                    )
                self.assertEqual(
                    (destination / "bin/mainboard-hardware").stat().st_mode & 0o777,
                    0o755,
                )
                for path in destination.rglob("*.dag"):
                    self.assertEqual(path.stat().st_mode & 0o777, 0o644)
                archive = bazel("build", *options, f"//tools/{module}:package")
                self.assertEqual(archive.returncode, 0, archive.stderr)
                extracted = Path(directory) / "extracted"
                extracted.mkdir()
                subprocess.run(
                    [
                        "tar",
                        "--no-same-owner",
                        "-xzf",
                        str(WORKSPACE / f"bazel-bin/tools/{module}/install.tar.gz"),
                        "-C",
                        str(extracted),
                    ],
                    check=True,
                )
                installed_files = tree_contents(destination, include_modes=True)
                archived_files = tree_contents(extracted, include_modes=True)
                self.assertEqual(installed_files.keys(), archived_files.keys())
                self.assertFalse(
                    [
                        path
                        for path in installed_files
                        if installed_files[path] != archived_files[path]
                    ],
                    "Installed and archived files must match in content and mode",
                )


class TransformProfileTest(unittest.TestCase):
    def test_legacy_install_and_lint_targets_are_removed(self):
        for label in (
            "//tools/transform:install",
            "//tools/hardware:install",
            "@hardware.camera//:install",
            "@hardware.camera//:camera_component_lib_cpplint",
        ):
            with self.subTest(label=label):
                result = bazel("query", label)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("not declared", result.stderr)

    def test_buffer_uses_only_module_owned_tf2(self):
        result = bazel("cquery", "deps(@wheelos_transform//modules/transform:buffer)")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertRegex(
            result.stdout, r"@+wheelos_transform[^/]*//third_party/tf2:tf2\b"
        )
        self.assertNotRegex(result.stdout, r"(?m)^//third_party/tf2:")

    def test_source_runtime_does_not_schedule_archiving(self):
        result = bazel(
            "aquery",
            'mnemonic("PackageTar|RuntimePackageMapping", '
            "deps(//tools/transform:run))",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "")
        packaged = bazel(
            "aquery", 'mnemonic("PackageTar", deps(//tools/transform:package))'
        )
        self.assertEqual(packaged.returncode, 0, packaged.stderr)
        self.assertIn("Mnemonic: PackageTar", packaged.stdout)

    def test_internal_interfaces_reject_unlisted_consumers(self):
        with tempfile.TemporaryDirectory(
            prefix="transform-visibility-", dir=WORKSPACE
        ) as directory:
            fixture = Path(directory)
            for dependency in (
                "@wheelos_transform//modules/transform:buffer_test_peer",
                "@wheelos_transform//modules/transform:static_transform_loader",
                "@wheelos_transform//third_party/tf2:tf2",
            ):
                with self.subTest(dependency=dependency):
                    (fixture / "BUILD.bazel").write_text(
                        'load("@rules_cc//cc:defs.bzl", "cc_library")\n'
                        'cc_library(name = "probe", testonly = True, '
                        f'deps = ["{dependency}"])\n'
                    )
                    result = bazel("build", "--nobuild", f"//{fixture.name}:probe")
                    self.assertNotEqual(result.returncode, 0)
                    self.assertIn("Visibility error", result.stderr)


if __name__ == "__main__":
    unittest.main()
