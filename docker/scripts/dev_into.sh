#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

echo "==================================================================" >&2
echo "WARNING: dev_into.sh WILL BE REMOVED IN THE NEXT RELEASE." >&2
echo "Please switch now to: whl enter dev" >&2
echo "==================================================================" >&2

case "${1:-}" in
  "")
    exec "${SCRIPT_DIR}/whl.sh" enter dev
    ;;
  -h | --help)
    exec "${SCRIPT_DIR}/whl.sh" help
    ;;
  *)
    echo "Legacy dev_into options are no longer supported." >&2
    echo "Use 'whl enter dev'." >&2
    exit 2
    ;;
esac
