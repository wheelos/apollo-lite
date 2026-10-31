#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
LAUNCH_FILE="/apollo/modules/dreamview/launch/dreamview.launch"

case "${1:-}" in
  start | stop)
    action="$1"
    ;;
  *)
    echo "Usage: $0 {start|stop}" >&2
    exit 2
    ;;
esac

if [[ "${action}" == "start" ]]; then
  "${PROJECT_ROOT}/docker/scripts/whl.sh" exec prod -- \
    /bin/bash -lc "if pgrep -x dreamview >/dev/null; then exit 0; fi; cd /apollo && ./scripts/cyber_launch.sh start ${LAUNCH_FILE}"

  if "${PROJECT_ROOT}/docker/scripts/whl.sh" exec prod -- \
    /bin/bash -lc 'for attempt in {1..30}; do pgrep -x dreamview >/dev/null && exit 0; sleep 1; done; exit 1'; then
    exit 0
  fi

  echo "Dreamview did not start within 30 seconds." >&2
  "${PROJECT_ROOT}/docker/scripts/whl.sh" exec prod -- \
    /bin/bash -lc 'tail -n 60 /apollo/data/log/dreamview.launch.start.log' >&2 || true
  exit 1
fi

"${PROJECT_ROOT}/docker/scripts/whl.sh" exec prod -- \
  /bin/bash -lc "cd /apollo && ./scripts/cyber_launch.sh stop ${LAUNCH_FILE}"

if "${PROJECT_ROOT}/docker/scripts/whl.sh" exec prod -- \
  /bin/bash -lc 'for attempt in {1..30}; do ! pgrep -x dreamview >/dev/null && exit 0; sleep 1; done; exit 1'; then
  exit 0
fi

echo "Dreamview did not stop within 30 seconds." >&2
exit 1
