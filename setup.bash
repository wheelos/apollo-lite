#!/usr/bin/env bash

APOLLO_LITE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
export APOLLO_LITE_ROOT

if ! command -v bazel >/dev/null 2>&1; then
  echo "Bazel is required to locate the built runtime tools." >&2
  return 1 2>/dev/null || exit 1
fi

runtime_tool_labels=(
  mainboard
  cyber_launch
  cyber_recorder
  cyber_monitor
  cyber_channel
  cyber_node
  cyber_service
)

for tool_name in "${runtime_tool_labels[@]}"; do
  tool_path="$(cd "${APOLLO_LITE_ROOT}" &&
    bazel cquery "//tools:${tool_name}" --output=files 2>/dev/null |
    awk -v name="${tool_name}" '$0 ~ "/" name "$" { print; exit }')"
  if [[ -z "${tool_path}" ]]; then
    echo "Runtime tool '${tool_name}' is not built. Run: ./apollo.sh build cyber" >&2
    return 1 2>/dev/null || exit 1
  fi
  tool_dir="${APOLLO_LITE_ROOT}/${tool_path%/*}"
  case ":${PATH}:" in
    *":${tool_dir}:"*) ;;
    *) PATH="${tool_dir}:${PATH}" ;;
  esac
done

export PATH
