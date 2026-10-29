#!/usr/bin/env python3
"""Translate customer build choices into the existing Bazel build entrypoint."""

import argparse
import os
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def profiles(path, prefix):
    """Read config names only; Bazel remains responsible for resolving values."""
    return sorted(
        set(
            re.findall(
                rf"^build:{re.escape(prefix)}([\w-]+)\s",
                path.read_text(),
                re.MULTILINE,
            )
        )
    )


def main():
    products = profiles(ROOT / "tools/products/profiles.bazelrc", "product_")
    hardware = profiles(ROOT / "tools/hardware/profiles.bazelrc", "")
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--product", choices=products)
    parser.add_argument("--hardware", choices=hardware)
    parser.add_argument("--modules", help="comma-separated whole-module names")
    parser.add_argument("--compute", choices=["cpu", "gpu"])
    parser.add_argument("--mode", choices=["opt", "dbg"])
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--jobs", type=int)
    parser.add_argument("--cpus", type=int)
    parser.add_argument("--rams", type=int, help="Bazel action memory budget in MB")
    args = parser.parse_args()
    if args.list:
        if any(value for key, value in vars(args).items() if key != "list"):
            parser.error("profiles does not accept build options")
        print("Example products (build inventories, not deployment qualification):")
        print("\n".join(f"  {name}" for name in products))
        print("\nHardware presets:")
        print("\n".join(f"  {name}" for name in hardware))
        print("\nPreview: bash apollo.sh build --product=<name> --show")
        return 0
    custom = args.hardware is not None or args.modules is not None
    if args.product and (custom or args.compute):
        parser.error("--product cannot be combined with --hardware/--modules/--compute")
    if args.all and (args.product or custom or args.show):
        parser.error("--all cannot be combined with a selection or --show")
    if not (args.product or custom or args.all):
        parser.error("select --product, --hardware/--modules, or --all")
    if custom and not args.compute:
        parser.error("custom builds require --compute=cpu or --compute=gpu")
    if args.modules is not None and not args.modules:
        parser.error("--modules must not be empty")
    for key in ["jobs", "cpus", "rams"]:
        value = getattr(args, key)
        if value is not None and value < 1:
            parser.error(f"--{key} must be positive")

    options = []
    targets = []
    if args.product:
        options.append(f"--config=product_{args.product}")
        targets = ["hardware", "algorithms"]
    else:
        if args.hardware:
            options.append(f"--config={args.hardware}")
            targets.append("hardware")
        if args.modules is not None:
            options.append(f"--//tools/modules:include={args.modules}")
            targets.append("algorithms")
        options.append(f"--config={args.compute or 'cpu'}")
    if args.mode or custom or args.all:
        options.append(f"--config={args.mode or 'opt'}")
    for key in ["jobs", "cpus", "rams"]:
        if getattr(args, key) is not None:
            options.append(f"--{key}={getattr(args, key)}")

    # Explicit selection is not inferred from the host GPU or device-tree model.
    env = dict(os.environ, APOLLO_BUILD_PHASE="1")
    command = [
        "bash",
        str(ROOT / "scripts/ci/apollo_build.sh"),
        "--explicit-selection",
        *targets,
        *options,
    ]
    print(
        f"Product: {args.product or ('ALL modules' if args.all else 'custom')}",
        flush=True,
    )
    if not args.all:
        print(
            "Effective configuration and outputs (Bazel analysis; no compilation):",
            flush=True,
        )
        preview = subprocess.run([*command, "--show"], cwd=ROOT, env=env, check=False)
        if preview.returncode or args.show:
            return preview.returncode
    return subprocess.run(command, cwd=ROOT, env=env, check=False).returncode


if __name__ == "__main__":
    raise SystemExit(main())
