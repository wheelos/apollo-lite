#!/usr/bin/env python3
"""Start declared DAGs or a Cyber launch using this runtime's exact binaries."""

import argparse
import json
import os
import sys
from pathlib import Path


def command(root, arguments):
    extra = []
    if "--" in arguments:
        boundary = arguments.index("--")
        extra = arguments[boundary + 1 :]
        arguments = arguments[:boundary]
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--dag", action="append")
    selection.add_argument("--launch")
    args = parser.parse_args(arguments)
    if args.launch:
        if extra:
            parser.error("mainboard arguments cannot be passed to cyber_launch")
        launch = root / args.launch
        return [sys.executable, str(root / "bin/cyber_launch.py"), "start", str(launch)]
    dags = args.dag or [
        json.loads((root / "conf/runtime.json").read_text())["default_dag"]
    ]
    if not all(dags):
        parser.error("this runtime requires an explicit --dag or --launch")
    argv = [str(root / "bin/mainboard")]
    for dag in dags:
        path = root / dag
        argv.extend(["-d", str(path)])
    return argv + extra


def main():
    root = Path(__file__).resolve().parents[1]
    argv = command(root, sys.argv[1:])
    os.execv(argv[0], argv)


if __name__ == "__main__":
    main()
