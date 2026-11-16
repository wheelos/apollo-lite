#!/usr/bin/env python3
"""Dispatch familiar Cyber commands to one explicitly selected runtime."""

import os
import re
import sys
from pathlib import Path


def command(workspace, environment, tool, arguments):
    if not re.fullmatch(r"[A-Za-z0-9_.+-]+", tool):
        raise ValueError("Runtime command name must be a single safe path segment")
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
        if product.startswith("//"):
            if not re.fullmatch(r"//[A-Za-z0-9_./+-]*:[A-Za-z0-9_.+-]+", product):
                raise ValueError("WHEELOS_RUNTIME_PRODUCT must be a product label")
            package, name = product[2:].split(":", 1)
            if name in (".", "..") or any(
                part in (".", "..") for part in package.split("/")
            ):
                raise ValueError("WHEELOS_RUNTIME_PRODUCT must not escape its root")
            root = workspace / "bazel-bin" / package / name
        else:
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
        module_entry = root / "bin/modules" / tool / tool
        if module_entry.is_file() and os.access(module_entry, os.X_OK):
            entry = module_entry
        else:
            modules = root / "bin/modules"
            candidates = (
                sorted(
                    path / tool
                    for path in modules.iterdir()
                    if path.is_dir()
                    and (path / tool).is_file()
                    and os.access(path / tool, os.X_OK)
                )
                if modules.is_dir()
                else []
            )
            if len(candidates) == 1:
                entry = candidates[0]
            elif len(candidates) > 1:
                raise ValueError(
                    f"Runtime command is ambiguous across module executables: {tool}"
                )
            else:
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
