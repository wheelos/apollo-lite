#!/usr/bin/env bash

# Unified runtime environment bootstrap for cyber tools.
# Priority:
# 1) External wheelos_core runtime script (preferred)
# 2) Existing PATH/PYTHONPATH state

if [[ "${APOLLO_RUNTIME_ENV_SOURCED:-0}" == "1" ]]; then
  return 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
APOLLO_ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd -P)"

_pathprepend() {
  local value="$1"
  local variable="${2:-PATH}"
  local current="${!variable:-}"
  local IFS=':'
  local item
  local new_value=""
  for item in ${current}; do
    if [[ "${item}" != "${value}" ]]; then
      if [[ -z "${new_value}" ]]; then
        new_value="${item}"
      else
        new_value="${new_value}:${item}"
      fi
    fi
  done
  if [[ -z "${new_value}" ]]; then
    export "${variable}=${value}"
  else
    export "${variable}=${value}:${new_value}"
  fi
}

_source_if_exists() {
  local script_path="$1"
  if [[ -f "${script_path}" ]]; then
    # shellcheck disable=SC1090
    source "${script_path}"
    export APOLLO_RUNTIME_ENV_SOURCE="${script_path}"
    export APOLLO_RUNTIME_ENV_SOURCED=1
    return 0
  fi
  return 1
}

_source_core_runtime() {
  local core_root="$1"
  if [[ -z "${core_root}" ]]; then
    return 1
  fi
  if _source_if_exists "${core_root}/scripts/env/runtime.bash"; then
    export APOLLO_CORE_ROOT="${core_root}"
    return 0
  fi
  return 1
}

_inject_apollo_external_core_outputs() {
  local repo_name
  local proj_repo_name
  local output_root
  local core_execroot
  local py_internal_dir
  local proj_data_dir
  local proj_data_found=0
  local bazel_bin_from_info
  local repo_candidates=(wheelos_core~ core~ core+ core)
  local proj_repo_candidates=(proj~ proj+ proj)
  local output_roots=("${APOLLO_ROOT_DIR}/bazel-bin")
  local tool_dirs=(
    "cyber/mainboard"
    "cyber/tools/cyber_launch"
    "cyber/tools/cyber_recorder"
    "cyber/tools/cyber_monitor"
    "cyber/tools/cyber_channel"
    "cyber/tools/cyber_node"
    "cyber/tools/cyber_service"
  )
  if command -v bazel >/dev/null 2>&1; then
    bazel_bin_from_info="$(cd "${APOLLO_ROOT_DIR}" && bazel info bazel-bin 2>/dev/null | head -n 1)"
    if [[ -n "${bazel_bin_from_info}" ]]; then
      output_roots+=("${bazel_bin_from_info}")
    fi
  fi
  for output_root in "${APOLLO_ROOT_DIR}"/bazel-out/*/bin; do
    if [[ -d "${output_root}" ]]; then
      output_roots+=("${output_root}")
    fi
  done
  for output_root in "${output_roots[@]}"; do
    for repo_name in "${repo_candidates[@]}"; do
      core_execroot="${output_root}/external/${repo_name}"
      for tool_dir in "${tool_dirs[@]}"; do
        if [[ -d "${core_execroot}/${tool_dir}" ]]; then
          _pathprepend "${tool_dir}" PATH
        fi
      done
      py_internal_dir="${core_execroot}/cyber/python/internal"
      if [[ -d "${py_internal_dir}" ]]; then
        _pathprepend "${py_internal_dir}" PYTHONPATH
      fi
    done
    for proj_repo_name in "${proj_repo_candidates[@]}"; do
      proj_data_dir="${output_root}/external/${proj_repo_name}/data"
      if [[ -f "${proj_data_dir}/proj.db" ]]; then
        export PROJ_DATA="${proj_data_dir}"
        proj_data_found=1
        break
      fi
    done
    if [[ "${proj_data_found}" == "1" ]]; then
      break
    fi
  done
}

# Preferred: external wheelos_core runtime env.
for core_root in \
  "${APOLLO_CORE_ROOT:-}" \
  "${WHEELOS_CORE_ROOT:-}" \
  "${APOLLO_ROOT_DIR}/../core"; do
  if _source_core_runtime "${core_root}"; then
    _inject_apollo_external_core_outputs
    if [[ -z "${CYBER_PATH:-}" && -d "${core_root}/cyber" ]]; then
      export CYBER_PATH="${core_root}/cyber"
    fi
    return 0
  fi
done

# If tooling is already on PATH, keep current shell state.
_inject_apollo_external_core_outputs
if command -v cyber_launch >/dev/null 2>&1 && command -v cyber_recorder >/dev/null 2>&1; then
  export APOLLO_RUNTIME_ENV_SOURCE="existing-path"
  export APOLLO_RUNTIME_ENV_SOURCED=1
  return 0
fi

echo "[WARNING] cyber runtime environment is not configured." >&2
echo "[WARNING] Set APOLLO_CORE_ROOT (or WHEELOS_CORE_ROOT) to your core checkout," >&2
echo "[WARNING] then source ${APOLLO_ROOT_DIR}/scripts/runtime_env.sh." >&2
return 0
