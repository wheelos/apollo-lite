"""Selection and process argument contracts for concise runtime commands."""

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cli = load(ROOT / "scripts/ci/apollo_runtime.py")
entry = load(ROOT / "tools/hardware/runtime_entry.py")


class RuntimeCliTest(unittest.TestCase):
    def test_development_run_does_not_install_or_package(self):
        argv = cli.command(["run", "transform"])
        self.assertIn("dev", argv)
        self.assertIn("//tools/transform:run", argv)
        self.assertNotIn("--destdir", argv)

    def test_delivery_defaults_to_release_with_explicit_dev_opt_in(self):
        argv = cli.command(["package", "transform"])
        self.assertIn("release", argv)
        self.assertIn("build", argv)
        self.assertIn("//tools/transform:package", argv)
        argv = cli.command(
            ["install", "transform", "--source=dev", "--destdir=/tmp/test"]
        )
        self.assertIn("dev", argv)
        self.assertEqual(argv[-3:], ["--", "--destdir", "/tmp/test"])

    def test_invalid_selection_fails_before_building(self):
        for arguments in (
            ["run", "hardware"],
            ["run", "--product=devkit-x86"],
            ["run", "transform", "--source=release"],
            ["install", "transform"],
            ["install", "transform", "--destdir=relative"],
            ["package", "transform", "--jobs=0"],
            ["run", "transform", "--unrecognized"],
        ):
            with self.subTest(arguments=arguments), self.assertRaises(SystemExit):
                cli.command(arguments)

    def test_runtime_args_are_preserved_without_shell_interpretation(self):
        argv = cli.command(["run", "transform", "--", "--dag", "path with spaces.dag"])
        self.assertEqual(argv[-3:], ["--", "--dag", "path with spaces.dag"])

    def test_entry_uses_exact_runtime_mainboard_and_launch(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "test.dag").write_text("")
            (root / "test.launch").write_text("")
            self.assertEqual(
                entry.command(root, ["--dag", "test.dag", "--", "-p", "test"]),
                [
                    str(root / "bin/mainboard"),
                    "-d",
                    str(root / "test.dag"),
                    "-p",
                    "test",
                ],
            )
            self.assertEqual(
                entry.command(root, ["--launch", "test.launch"])[1:],
                [str(root / "bin/cyber_launch.py"), "start", str(root / "test.launch")],
            )
            # Core owns default/overlay resolution, including override-only keys.
            self.assertEqual(
                entry.command(root, ["--dag", "override-only.dag"])[-2:],
                ["-d", str(root / "override-only.dag")],
            )
            self.assertEqual(
                entry.command(root, ["--launch", "override-only.launch"])[-1],
                str(root / "override-only.launch"),
            )
            with self.assertRaises(SystemExit):
                entry.command(root, ["--dag", "test.dag", "--launch", "test.launch"])
            with self.assertRaises(SystemExit):
                entry.command(root, ["--launch", "test.launch", "--", "-p", "test"])
            (root / "conf").mkdir()
            (root / "conf/runtime.json").write_text(
                json.dumps({"default_dag": "test.dag"})
            )
            self.assertEqual(
                entry.command(root, [])[-2:], ["-d", str(root / "test.dag")]
            )
            (root / "conf/runtime.json").write_text(json.dumps({"default_dag": ""}))
            with self.assertRaises(SystemExit):
                entry.command(root, [])


if __name__ == "__main__":
    unittest.main()
