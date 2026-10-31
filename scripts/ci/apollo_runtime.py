#!/usr/bin/env python3
"""Run or deliver explicitly selected product runtime targets."""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TARGETS = {"transform": "//tools/transform", "hardware": "//tools/hardware"}


def command(arguments):
    runtime_args = []
    if "--" in arguments:
        boundary = arguments.index("--")
        runtime_args = arguments[boundary + 1 :]
        arguments = arguments[:boundary]
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("action", choices=["run", "install", "package"])
    parser.add_argument("module", nargs="?", choices=sorted(TARGETS))
    parser.add_argument(
        "--product",
        choices=sorted(TARGETS),
        help="runtime assembly, not a build inventory profile",
    )
    parser.add_argument("--source", choices=["dev", "release"])
    parser.add_argument("--compute", choices=["cpu", "gpu"], default="cpu")
    parser.add_argument("--mode", choices=["opt", "dbg"], default="opt")
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument("--jobs", type=int)
    parser.add_argument("--destdir", type=Path)
    args = parser.parse_args(arguments)
    if args.module and args.product:
        parser.error("choose a module or --product, not both")
    selected = args.module or args.product
    if not selected:
        parser.error("select a module or an explicit runtime product")
    if runtime_args:
        if args.action != "run":
            parser.error("runtime arguments must follow -- and are only valid for run")
    if args.jobs is not None and args.jobs < 1:
        parser.error("--jobs must be positive")
    if args.action == "install":
        if args.destdir is None or not args.destdir.is_absolute():
            parser.error("install requires an absolute --destdir")
    elif args.destdir is not None:
        parser.error("--destdir is only valid for install")
    source = args.source or ("dev" if args.action == "run" else "release")
    if args.action == "run" and source != "dev":
        parser.error(
            "run builds local development sources; deployed startup uses bin/run"
        )
    if selected == "hardware" and args.action == "run" and not runtime_args:
        parser.error(
            "hardware requires -- --dag <dag> or --launch <launch>; "
            "no hardware starts implicitly"
        )
    options = [f"--config={args.compute}", f"--config={args.mode}"]
    options.extend(f"--config={config}" for config in args.config)
    if args.jobs is not None:
        options.append(f"--jobs={args.jobs}")
    action = "build" if args.action == "package" else "run"
    target = {"run": "run", "install": "install_dir", "package": "package"}[args.action]
    argv = [
        sys.executable,
        str(ROOT / "scripts/bzlmod.py"),
        source,
        action,
        *options,
        f"{TARGETS[selected]}:{target}",
    ]
    if args.action == "install":
        argv.extend(["--", "--destdir", str(args.destdir)])
    elif runtime_args:
        argv.extend(["--", *runtime_args])
    return argv


def main():
    argv = command(sys.argv[1:])
    os.chdir(ROOT)
    os.execv(sys.executable, argv)


if __name__ == "__main__":
    main()
