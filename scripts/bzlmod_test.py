"""Tests for isolation of local and registry-based consumers."""

import tempfile
import unittest
from pathlib import Path

import bzlmod


class ConsumerModeTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        (self.root / "MODULE.bazel").write_text('module(name = "consumer")\n')

    def test_registry_and_release_exclude_ambient_rcs_and_dev_overrides(self):
        for mode in ("registry", "release"):
            with self.subTest(mode=mode):
                argv = bzlmod.command(
                    self.root, "bazelisk", mode, ["build", "//:target"]
                )
                self.assertIn("--nohome_rc", argv)
                self.assertIn("--noworkspace_rc", argv)
                self.assertIn(f"--bazelrc={self.root}/.bazelrc.{mode}", argv)
                self.assertNotIn("--config=dev", argv)
                self.assertNotIn("--lockfile_mode=update", argv)

    def test_dev_is_explicit_and_lock_updates_are_separate(self):
        argv = bzlmod.command(self.root, "bazelisk", "dev", ["mod", "graph"])
        self.assertIn("--config=dev", argv)
        argv = bzlmod.command(self.root, "bazelisk", "lock-update", ["mod", "deps"])
        self.assertIn("--lockfile_mode=update", argv)

    def test_rejects_source_or_registry_injection(self):
        for flag in (
            "--override_module=wheelos_core=/tmp/core",
            "--override_repository=core=/tmp/core",
            "--registry=file:///tmp/registry",
            "--lockfile_mode=off",
            "--override_module",
            "--flagfile=/tmp/flags",
        ):
            with self.subTest(flag=flag), self.assertRaises(ValueError):
                bzlmod.command(
                    self.root, "bazelisk", "release", ["build", flag, "//:all"]
                )

    def test_rejects_module_source_override_and_missing_command(self):
        (self.root / "MODULE.bazel").write_text(
            'local_path_override(module_name = "core", path = "pkgs/core")\n'
        )
        with self.assertRaises(ValueError):
            bzlmod.command(self.root, "bazelisk", "registry", ["mod", "graph"])
        with self.assertRaises(ValueError):
            bzlmod.command(self.root, "bazelisk", "release", [])


if __name__ == "__main__":
    unittest.main()
