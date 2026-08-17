#!/usr/bin/env bash

CYBER_CORE_TARGET="@core//cyber:cyber_core"
CYBER_RUNTIME_TOOL_TARGETS=(
  "@core//cyber/mainboard:mainboard"
  "@core//cyber/tools/cyber_launch:cyber_launch"
  "@core//cyber/tools/cyber_recorder:cyber_recorder"
  "@core//cyber/tools/cyber_monitor:cyber_monitor"
  "@core//cyber/tools/cyber_channel:cyber_channel"
  "@core//cyber/tools/cyber_node:cyber_node"
  "@core//cyber/tools/cyber_service:cyber_service"
)

cyber_core_target() {
  echo "${CYBER_CORE_TARGET}"
}

cyber_runtime_targets() {
  local expr="${CYBER_CORE_TARGET}"
  local target
  for target in "${CYBER_RUNTIME_TOOL_TARGETS[@]}"; do
    expr="${expr} union ${target}"
  done
  echo "${expr}"
}
