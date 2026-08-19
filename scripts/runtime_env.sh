#!/usr/bin/env bash

# Unified runtime environment bootstrap for cyber tools.
# Production path: use Bazel module dependency `@core` / `wheelos_core`.
# Local source-checkout overrides are allowed only when explicitly configured;
# they are never implicit compatibility fallbacks.

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

_detect_module_python_version() {
  local module_file="${APOLLO_ROOT_DIR}/MODULE.bazel"
  if [[ ! -f "${module_file}" ]]; then
    return 0
  fi
  local version
  version="$(grep -E "python\.(defaults|toolchain)\(python_version = \"[0-9]+\.[0-9]+\"\)" "${module_file}" | head -n 1 | sed -E 's/.*python_version = "([0-9]+\.[0-9]+)".*/\1/' || true)"
  if [[ -n "${version}" ]]; then
    echo "${version}"
  fi
}

_check_runtime_python_abi() {
  if ! command -v python3 >/dev/null 2>&1; then
    return 0
  fi
  local expected_version
  expected_version="$(_detect_module_python_version)"
  if [[ -z "${expected_version}" ]]; then
    return 0
  fi
  local actual_version
  actual_version="$(python3 -c 'import sys; print(f"{sys.version_info[0]}.{sys.version_info[1]}")' 2>/dev/null || true)"
  if [[ -z "${actual_version}" ]]; then
    return 0
  fi
  if [[ "${actual_version}" != "${expected_version}" ]]; then
    echo "[WARNING] Python ABI mismatch: Bazel module is configured for Python ${expected_version}, but python3 resolves to ${actual_version}." >&2
    echo "[WARNING] This causes native Cyber extensions such as cyber_channel / cyber_node to fail at import time." >&2
    echo "[WARNING] Align the container Python and MODULE.bazel, then rebuild in the same environment." >&2
  fi
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
  local bazel_output_base_from_info
  local lib_dir
  local solib_dir
  local cyber_conf
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
    bazel_bin_from_info="$(cd "${APOLLO_ROOT_DIR}" && bazel info bazel-bin 2>/dev/null | tail -n 1)"
    if [[ -n "${bazel_bin_from_info}" ]]; then
      output_roots+=("${bazel_bin_from_info}")
    fi
    bazel_output_base_from_info="$(
      cd "${APOLLO_ROOT_DIR}" &&
        bazel info output_base 2>/dev/null | tail -n 1
    )"
    if [[ -n "${bazel_output_base_from_info}" ]]; then
      output_roots+=("${bazel_output_base_from_info}")
    fi
  fi
  for output_root in "${APOLLO_ROOT_DIR}"/bazel-out/*/bin; do
    if [[ -d "${output_root}" ]]; then
      output_roots+=("${output_root}")
    fi
  done
  for output_root in "${output_roots[@]}"; do
    while IFS= read -r solib_dir; do
      _pathprepend "${solib_dir}" LD_LIBRARY_PATH
    done < <(find "${output_root}" -type d -path "*/_solib_*/*" 2>/dev/null | sort -u)
    for repo_name in "${repo_candidates[@]}"; do
      core_execroot="${output_root}/external/${repo_name}"
      for tool_dir in "${tool_dirs[@]}"; do
        if [[ -d "${core_execroot}/${tool_dir}" ]]; then
          _pathprepend "${core_execroot}/${tool_dir}" PATH
        fi
      done
      py_internal_dir="${core_execroot}/cyber/python/internal"
      _pathprepend "${core_execroot}" PYTHONPATH
      if [[ -d "${py_internal_dir}" ]]; then
        _pathprepend "${py_internal_dir}" PYTHONPATH
        _pathprepend "${py_internal_dir}" LD_LIBRARY_PATH
      fi
      if [[ -z "${CYBER_PATH:-}" ]]; then
        if [[ -f "${core_execroot}/cyber/conf/cyber.pb.conf" ]]; then
          export CYBER_PATH="${core_execroot}/cyber"
        else
          cyber_conf="$(find "${core_execroot}/cyber/tools" \
            -path "*/cyber/conf/cyber.pb.conf" -print -quit 2>/dev/null || true)"
          if [[ -n "${cyber_conf}" ]]; then
            export CYBER_PATH="${cyber_conf%/conf/cyber.pb.conf}"
          fi
        fi
      fi
      if [[ -d "${core_execroot}" ]]; then
        while IFS= read -r lib_dir; do
          _pathprepend "${lib_dir}" LD_LIBRARY_PATH
        done < <(find "${core_execroot}" -type d \( -name lib -o -name lib64 \) 2>/dev/null | sort -u)
        while IFS= read -r solib_dir; do
          _pathprepend "${solib_dir}" LD_LIBRARY_PATH
        done < <(find "${core_execroot}" -type d -path "*/_solib_*/*" 2>/dev/null | sort -u)
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

_check_runtime_python_abi

# Keep terminal capabilities available for interactive Cyber tools such as
# cyber_monitor, including shells entered without a host-provided TERM.
export TERM="${TERM:-xterm-256color}"
export TERMINFO="${TERMINFO:-/lib/terminfo/}"

# The Bzlmod protobuf extension may be built against a different CPython
# minor version than the container interpreter. Use protobuf's supported pure
# Python implementation unless the caller explicitly selects another backend.

# Explicit local source-checkout overrides are supported only when intentionally set.
for core_root in \
  "${APOLLO_CORE_ROOT:-}" \
  "${WHEELOS_CORE_ROOT:-}"; do
  if _source_core_runtime "${core_root}"; then
    _inject_apollo_external_core_outputs
    if [[ -z "${CYBER_PATH:-}" && -d "${core_root}/cyber" ]]; then
      export CYBER_PATH="${core_root}/cyber"
    fi
    return 0
  fi
done

# Prefer Bazel module outputs (`@core` / `wheelos_core`) when available.
_inject_apollo_external_core_outputs
if command -v cyber_launch >/dev/null 2>&1 && command -v cyber_recorder >/dev/null 2>&1; then
  export APOLLO_RUNTIME_ENV_SOURCE="existing-path"
  export APOLLO_RUNTIME_ENV_SOURCED=1
  return 0
fi

echo "[WARNING] cyber runtime environment is not configured." >&2
echo '[WARNING] Use Bazel module dependency @core / wheelos_core or set APOLLO_CORE_ROOT' >&2
echo '[WARNING] explicitly for a temporary local source-checkout override.' >&2
return 0
