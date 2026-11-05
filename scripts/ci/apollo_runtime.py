#!/usr/bin/env python3
"""Run or deliver explicitly selected product runtime targets."""

import argparse
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TARGETS = {
    "transform": "//tools/transform",
    "hardware": "//tools/hardware",
    "system": "//tools/system",
}

PRODUCT_LABEL = re.compile(r"//[A-Za-z0-9_./+-]*:[A-Za-z0-9_.+-]+")


def product_label(value):
    if (
        not PRODUCT_LABEL.fullmatch(value)
        or any(part in (".", "..") for part in value[2:].split(":", 1)[0].split("/"))
        or value.rsplit(":", 1)[-1] in (".", "..")
    ):
        raise argparse.ArgumentTypeError(
            "use an explicit local product label, e.g. //products/demo:runtime"
        )
    return value


def command(arguments):
    runtime_args = []
    if "--" in arguments:
        boundary = arguments.index("--")
        runtime_args = arguments[boundary + 1 :]
        arguments = arguments[:boundary]
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument("action", choices=["build", "run", "install", "package"])
    parser.add_argument("module", nargs="?", choices=sorted(TARGETS))
    parser.add_argument(
        "--product",
        help="runtime product label (//package:name), or a legacy runtime assembly",
    )
    parser.add_argument("--source", choices=["dev", "release"])
    parser.add_argument("--compute", choices=["cpu", "gpu"], default="cpu")
    parser.add_argument("--mode", choices=["opt", "dbg"], default="opt")
    parser.add_argument("--config", action="append", default=[])
    parser.add_argument("--jobs", type=int)
    parser.add_argument("--destdir", type=Path)
    parser.add_argument("--show", action="store_true", help="analyze a build only")
    args = parser.parse_args(arguments)
    if args.module and args.product:
        parser.error("choose a module or --product, not both")
    selected = args.module or args.product
    if not selected:
        parser.error("select a module or an explicit runtime product")
    is_label = selected.startswith("//")
    if is_label:
        try:
            product_label(selected)
        except argparse.ArgumentTypeError as error:
            parser.error(str(error))
    elif selected not in TARGETS:
        parser.error(
            "select a runtime product label, not a legacy build inventory; "
            f"legacy runtime assemblies: {', '.join(sorted(TARGETS))}"
        )
    if args.show and args.action != "build":
        parser.error("--show is only valid for build")
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
    source = args.source or ("dev" if args.action in ("build", "run") else "release")
    if args.action == "run" and source != "dev":
        parser.error(
            "run builds local development sources; deployed startup uses bin/run"
        )
    if selected in ("hardware", "system") and args.action == "run" and not runtime_args:
        parser.error(
            f"{selected} requires -- --dag <dag> or --launch <launch>; "
            "no modules start implicitly"
        )
    options = [f"--config={args.compute}", f"--config={args.mode}"]
    options.extend(f"--config={config}" for config in args.config)
    if args.jobs is not None:
        options.append(f"--jobs={args.jobs}")
    action = "build" if args.action in ("build", "package") else "run"
    if args.show:
        action = "cquery"
    if is_label:
        suffix = {
            "build": "",
            "run": "_run",
            "install": "_install",
            "package": "_package",
        }[args.action]
        target = selected + suffix
    else:
        target = (
            TARGETS[selected]
            + ":"
            + {
                "build": "runtime",
                "run": "run",
                "install": "install_dir",
                "package": "package",
            }[args.action]
        )
    argv = [
        sys.executable,
        str(ROOT / "scripts/bzlmod.py"),
        source,
        action,
        *options,
        target,
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
