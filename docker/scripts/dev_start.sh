#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

echo "==================================================================" >&2
echo "WARNING: dev_start.sh WILL BE REMOVED IN THE NEXT RELEASE." >&2
echo "Please switch now to: whl start dev" >&2
echo "==================================================================" >&2

case "${1:-}" in
  "")
    ;;
  -y)
    shift
    ;;
  stop)
    exec "${SCRIPT_DIR}/whl.sh" stop dev
    ;;
  -h | --help)
    exec "${SCRIPT_DIR}/whl.sh" help
    ;;
  *)
    echo "Unsupported legacy option '${1}'. Use 'whl start dev --help'." >&2
    exit 2
    ;;
esac

if [[ $# -gt 0 ]]; then
  echo "Unsupported legacy options: $*" >&2
  echo "Use 'whl start dev --help'." >&2
  exit 2
fi

exec "${SCRIPT_DIR}/whl.sh" start dev
