#!/usr/bin/env python3
"""Verify that the versioned BEVFusion C ABI remains exported by the DSO."""

import subprocess
import sys


REQUIRED_SYMBOLS = {
    "apollo_bevfusion_runtime_abi_version",
    "apollo_bevfusion_runtime_create",
    "apollo_bevfusion_runtime_destroy",
    "apollo_bevfusion_runtime_infer",
}


def main():
    if len(sys.argv) != 2:
        print("usage: check_runtime_symbols.py LIBBEVFUSION_RUNTIME_SO", file=sys.stderr)
        return 2
    result = subprocess.run(
        ["nm", "-D", "--defined-only", sys.argv[1]],
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(result.stderr.strip() or "nm failed", file=sys.stderr)
        return 1
    exported = {line.split()[-1].split("@")[0] for line in result.stdout.splitlines() if line.split()}
    missing = sorted(REQUIRED_SYMBOLS - exported)
    if missing:
        print("missing exported runtime ABI symbols: " + ", ".join(missing), file=sys.stderr)
        return 1
    print("BEVFusion runtime ABI symbols are exported")
    return 0


if __name__ == "__main__":
    sys.exit(main())
