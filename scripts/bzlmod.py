#!/usr/bin/env python3
"""Run isolated Bzlmod consumer modes or record local checkout provenance."""

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RC_FILES = {
    "dev": ".bazelrc",
    "registry": ".bazelrc.registry",
    "release": ".bazelrc.release",
    "lock-update": ".bazelrc.release",
}


def snapshot(root):
    """Include dirty state: a commit alone does not identify local edits."""
    checkouts = [root]
    packages = root / "pkgs"
    if packages.is_dir():
        checkouts.extend(
            path for path in sorted(packages.iterdir()) if (path / ".git").exists()
        )
    result = {}
    for path in checkouts:
        head = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "--verify", "--quiet", "HEAD"],
            text=True,
            capture_output=True,
            check=False,
        )
        if head.returncode not in (0, 1):
            head.check_returncode()
        revision = head.stdout.strip() if head.returncode == 0 else None
        changes = subprocess.check_output(
            ["git", "-C", str(path), "status", "--porcelain"], text=True
        ).splitlines()
        result[str(path.relative_to(root))] = {
            "revision": revision,
            "dirty": bool(changes),
            "changes": changes,
        }
    return result


def command(root, bazel, mode, arguments):
    if not arguments or arguments[0] not in {
        "build",
        "test",
        "run",
        "query",
        "cquery",
        "aquery",
        "mod",
        "fetch",
        "info",
    }:
        raise ValueError("Specify a Bazel build/test/run/query/mod/fetch/info command.")
    if mode != "dev":
        forbidden = {
            "--override_module",
            "--override_repository",
            "--inject_repository",
            "--registry",
            "--lockfile_mode",
            "--flagfile",
        }
        for argument in arguments:
            if argument.split("=", 1)[0] in forbidden:
                raise ValueError(f"{mode} mode does not permit {argument}.")
        module = (root / "MODULE.bazel").read_text()
        if re.search(r"(?m)^\s*(local_path|git|archive)_override\s*\(", module):
            raise ValueError(
                "Registry consumption requires no source overrides in MODULE.bazel."
            )
    options = [
        bazel,
        "--nosystem_rc",
        "--nohome_rc",
        "--noworkspace_rc",
        f"--bazelrc={root / RC_FILES[mode]}",
        arguments[0],
    ]
    if mode == "dev":
        options.append("--config=dev")
    elif mode == "lock-update":
        options.append("--lockfile_mode=update")
    return options + arguments[1:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=[*RC_FILES, "snapshot"])
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.mode == "snapshot":
        if args.arguments:
            parser.error("snapshot takes no Bazel arguments")
        print(json.dumps(snapshot(ROOT), indent=2))
        return 0
    if os.geteuid() == 0 or not Path("/.dockerenv").exists():
        parser.error("Run through whl exec dev as the mapped non-root user.")
    bazel = shutil.which("bazelisk") or shutil.which("bazel")
    if bazel is None:
        parser.error("Bazelisk or Bazel is required.")
    version = (ROOT / ".bazelversion").read_text().strip()
    actual = subprocess.check_output([bazel, "--version"], cwd=ROOT, text=True).strip()
    if actual != f"bazel {version}":
        parser.error(f"Expected Bazel {version}, got {actual}; use Bazelisk.")
    try:
        argv = command(ROOT, bazel, args.mode, args.arguments)
    except ValueError as error:
        parser.error(str(error))
    print(
        f"Bzlmod mode={args.mode}, Bazel={version}, rc={RC_FILES[args.mode]}",
        flush=True,
        file=sys.stderr,
    )
    return subprocess.call(argv, cwd=ROOT)


if __name__ == "__main__":
    raise SystemExit(main())
