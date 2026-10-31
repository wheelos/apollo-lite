#!/usr/bin/env python3
"""Dispatch familiar Cyber commands to one explicitly selected runtime."""

import os
import re
import sys
from pathlib import Path


def command(workspace, environment, tool, arguments):
    root_value = environment.get("WHEELOS_RUNTIME_ROOT", "")
    product = environment.get("WHEELOS_RUNTIME_PRODUCT", "")
    if root_value and product:
        raise ValueError(
            "Set WHEELOS_RUNTIME_ROOT or WHEELOS_RUNTIME_PRODUCT, not both"
        )
    if root_value:
        root = Path(root_value)
        if not root.is_absolute():
            raise ValueError("WHEELOS_RUNTIME_ROOT must be an absolute path")
    elif product:
        if not re.fullmatch(r"[a-z][a-z0-9_-]*", product):
            raise ValueError("WHEELOS_RUNTIME_PRODUCT must be a safe product name")
        root = workspace / "bazel-bin/tools" / product / "runtime"
    else:
        raise ValueError(
            "Select WHEELOS_RUNTIME_PRODUCT for development or "
            "WHEELOS_RUNTIME_ROOT for an installed runtime"
        )
    if not (root / "conf/runtime.json").is_file():
        raise ValueError(
            f"Selected runtime is not built/installed: {root}; "
            "build its explicit //tools/<product>:runtime target first"
        )
    entry = root / "bin/commands" / tool
    if not entry.is_file() or not os.access(entry, os.X_OK):
        raise ValueError(f"Runtime command is missing or not executable: {entry}")
    return [str(entry), *arguments]


def main():
    workspace = Path(__file__).resolve().parents[2]
    try:
        argv = command(workspace, os.environ, sys.argv[1], sys.argv[2:])
    except ValueError as error:
        raise SystemExit(str(error)) from error
    os.execv(argv[0], argv)


if __name__ == "__main__":
    main()
