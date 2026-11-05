"""No-device proof of one- and two-component products across delivery layouts."""

import hashlib
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from python.runfiles import runfiles

INPUTS = sys.argv[1:]
del sys.argv[1:]


def contents(root, include_modes=True):
    return {
        path.relative_to(root).as_posix(): (
            hashlib.sha256(path.read_bytes()).hexdigest(),
            path.stat().st_mode & 0o777 if include_modes else None,
        )
        for path in root.rglob("*")
        if path.is_file()
    }


class ProductIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="product runtime ")
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        resolver = runfiles.Create()
        self.single, self.combined, self.archive, self.installer, self.strict = [
            Path(resolver.Rlocation(value)) for value in INPUTS
        ]
        self.environment = dict(os.environ)
        for key in (
            "WHEELOS_CONFIG_ROOT",
            "WHEELOS_ASSET_ROOT",
            "WHEELOS_RUNTIME_ROOT",
            "WHEELOS_RUNTIME_PRODUCT",
            "WHEELOS_RUNTIME_LIBRARY_PATH",
            "APOLLO_ROOT_DIR",
            "CYBER_PATH",
            "PYTHONPATH",
            "LD_LIBRARY_PATH",
        ):
            self.environment.pop(key, None)
        self.environment.update(
            CYBER_DOMAIN_ID=str(180 + os.getpid() % 40),
            CYBER_IP="127.0.0.1",
            GLOG_logtostderr="1",
        )

    def exercise(self, root, expected, environment=None, settle=0):
        output = self.directory / "process.log"
        env = dict(self.environment, **(environment or {}))
        with output.open("w") as stream:
            process = subprocess.Popen(
                [str(root / "bin/run")],
                cwd=self.directory,
                env=env,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    text = output.read_text()
                    if all(marker in text for marker in expected):
                        time.sleep(settle)
                        self.assertIsNone(process.poll(), output.read_text())
                        break
                    if process.poll() is not None:
                        self.fail(f"product exited {process.returncode}:\n{text}")
                    time.sleep(0.1)
                else:
                    self.fail(f"missing output {expected}:\n{output.read_text()}")
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                    self.fail(f"product did not stop gracefully:\n{output.read_text()}")

    def test_single_and_combined_products_run_through_identical_entry(self):
        self.exercise(self.single, ["PRODUCT_ALPHA alpha"])
        self.exercise(self.combined, ["PRODUCT_ALPHA alpha", "PRODUCT_BETA beta"])

    def test_archive_and_official_install_are_identical_and_relocatable(self):
        extracted = self.directory / "archive relocated"
        extracted.mkdir()
        subprocess.run(
            ["tar", "--no-same-owner", "-xzf", str(self.archive), "-C", str(extracted)],
            check=True,
        )
        installed = self.directory / "installed relocated"
        result = subprocess.run(
            [str(self.installer), "--destdir", str(installed)],
            env=self.environment,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        # Bazel normalizes tree-artifact permissions; compare delivery modes below.
        self.assertEqual(
            contents(self.combined, include_modes=False),
            contents(extracted, include_modes=False),
        )
        self.assertEqual(contents(extracted), contents(installed))
        for root in (extracted, installed):
            self.assertFalse(any(path.is_symlink() for path in root.rglob("*")))
            self.exercise(root, ["PRODUCT_ALPHA alpha", "PRODUCT_BETA beta"])

        diagnostic = installed / "bin/modules/alpha/diagnose"
        result = subprocess.run(
            [str(diagnostic)],
            env=self.environment,
            cwd=self.directory,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "fixture-model-not-for-inference")

        external = self.directory / "external assets"
        shutil.move(str(installed / "assets"), external)
        configuration = self.directory / "configuration"
        (configuration / "conf/alpha").mkdir(parents=True)
        (configuration / "conf/alpha/component.conf").write_text("alpha-override\n")
        self.exercise(
            installed,
            ["PRODUCT_ALPHA alpha-override", "PRODUCT_BETA beta"],
            {
                "WHEELOS_ASSET_ROOT": str(external),
                "WHEELOS_CONFIG_ROOT": str(configuration),
            },
        )
        model = external / "models/alpha/fixture/model.bin"
        model.write_text("corrupted-model\n")
        failed = subprocess.run(
            [str(installed / "bin/run")],
            env=dict(self.environment, WHEELOS_ASSET_ROOT=str(external)),
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        self.assertNotEqual(failed.returncode, 0)
        self.assertNotIn("PRODUCT_ALPHA", failed.stdout)

    def test_required_configuration_never_falls_back_to_packaged_defaults(self):
        failed = subprocess.run(
            [str(self.strict / "bin/run")],
            env=self.environment,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        self.assertNotEqual(failed.returncode, 0)
        self.assertNotIn("PRODUCT_ALPHA", failed.stdout)
        configuration = self.directory / "required configuration"
        (configuration / "conf/alpha").mkdir(parents=True)
        (configuration / "conf/alpha/component.conf").write_text("strict-alpha\n")
        self.exercise(
            self.strict,
            ["PRODUCT_ALPHA strict-alpha"],
            {"WHEELOS_CONFIG_ROOT": str(configuration)},
        )

    def test_recorder_uses_product_core_and_preserves_caller_data_directory(self):
        output = self.directory / "recorder.log"
        recorder = self.combined / "bin/commands/cyber_recorder"
        with output.open("w") as stream:
            process = subprocess.Popen(
                [
                    str(recorder),
                    "record",
                    "-c",
                    "/runtime_product/chassis",
                    "-o",
                    "./product.record",
                    "-i",
                    "0",
                    "-m",
                    "0",
                ],
                env=self.environment,
                cwd=self.directory,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                self.exercise(
                    self.combined,
                    ["PRODUCT_ALPHA alpha", "PRODUCT_BETA beta"],
                    settle=3,
                )
                self.assertIsNone(process.poll(), output.read_text())
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
                    self.fail(output.read_text())
        records = list(self.directory.glob("product.record*"))
        self.assertEqual(len(records), 1, output.read_text())
        info = subprocess.run(
            [str(recorder), "info", str(records[0])],
            env=self.environment,
            cwd=self.directory,
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        self.assertEqual(info.returncode, 0, info.stdout + info.stderr)
        self.assertIn("/runtime_product/chassis", info.stdout + info.stderr)


if __name__ == "__main__":
    unittest.main()
