#!/usr/bin/env python3
"""Resolve a Bazel directory output and exec its deployed runtime entry."""

import os
import sys
from pathlib import Path

from python.runfiles import runfiles


def main():
    locator = runfiles.Create()
    resolved = locator.Rlocation(sys.argv[1]) if locator else None
    if not resolved or not Path(resolved).is_dir():
        raise SystemExit(f"Runtime directory is missing from runfiles: {sys.argv[1]}")
    runtime = Path(resolved)
    os.execv(str(runtime / "bin/run"), [str(runtime / "bin/run"), *sys.argv[2:]])


if __name__ == "__main__":
    main()
