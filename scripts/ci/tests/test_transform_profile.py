"""Transform module boundaries; run in the managed development container."""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[3]


def bazel(*args):
    return subprocess.run(
        [
            sys.executable,
            "scripts/bzlmod.py",
            "dev",
            args[0],
            "--config=cpu",
            "--config=opt",
            *args[1:],
        ],
        cwd=WORKSPACE,
        capture_output=True,
        text=True,
        check=False,
    )


class TransformProfileTest(unittest.TestCase):
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
            'mnemonic("PackageTar", deps(//tools/transform:runtime))',
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "")
        packaged = bazel(
            "aquery", 'mnemonic("PackageTar", deps(//tools/transform:install))'
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
