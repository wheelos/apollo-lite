#!/usr/bin/env bash

PROJECT_GLOBAL_ENV_NAME=".env.global"

function project_hash_suffix() {
  local project_root="$1"
  echo "$(echo "${project_root}" | md5sum | cut -c1-8)"
}

function project_global_env_path() {
  local project_root="$1"
  echo "${project_root}/${PROJECT_GLOBAL_ENV_NAME}"
}

function generated_runtime_env_name() {
  local mode="$1"
  echo ".env.${mode}.local"
}

function generated_runtime_env_path() {
  local mode="$1"
  local docker_dir="$2"
  echo "${docker_dir}/$(generated_runtime_env_name "${mode}")"
}

function resolve_project_env_file() {
  local project_root="$1"
  local global_env_file
  global_env_file="$(project_global_env_path "${project_root}")"

  if [[ -f "${global_env_file}" ]]; then
    echo "${global_env_file}"
    return 0
  fi

  return 1
}

function mode_env_prefix() {
  local mode="$1"
  case "${mode}" in
    dev)
      echo "DEV"
      ;;
    test)
      echo "TEST"
      ;;
    prod)
      echo "PROD"
      ;;
    *)
      return 1
      ;;
  esac
}

function is_supported_project_env_key() {
  local key="$1"
  case "${key}" in
    AUTO_BOOTSTRAP | DISPLAY | OS | TARGET_ARCH | WHL_DISPLAY | WHL_PORT_OFFSET | WHL_PROJECT_SUFFIX | DEV_APOLLO_IMAGE | DEV_BAZEL_CACHE_DIR | DEV_SERVER_PORT | DEV_SHM_SIZE | DEV_USE_GPU | DEV_USE_GPU_HOST | TEST_APOLLO_IMAGE | TEST_BAZEL_CACHE_DIR | TEST_CPUS | TEST_MEMORY | TEST_SERVER_PORT | TEST_SHM_SIZE | TEST_USE_GPU | TEST_USE_GPU_HOST | PROD_APOLLO_IMAGE | PROD_BAZEL_CACHE_DIR | PROD_SERVER_PORT | PROD_SHM_SIZE | PROD_USE_GPU | PROD_USE_GPU_HOST)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

function load_env_file_exports() {
  local env_file="$1"
  while IFS= read -r _line || [[ -n "$_line" ]]; do
    local line="$_line"
    line="$(echo "${line}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
    [[ -z "${line}" || "${line:0:1}" == "#" ]] && continue
    if [[ "${line}" =~ ^([A-Za-z_][A-Za-z0-9_]*)=(.*)$ ]]; then
      local key="${BASH_REMATCH[1]}"
      local val="${BASH_REMATCH[2]}"
      is_supported_project_env_key "${key}" || continue
      if [[ -n "${!key+x}" ]]; then
        continue
      fi
      if [[ "${val}" =~ ^\"(.*)\"$ ]]; then
        val="${BASH_REMATCH[1]}"
      elif [[ "${val}" =~ ^\'(.*)\'$ ]]; then
        val="${BASH_REMATCH[1]}"
      fi
      export "${key}"="${val}"
    fi
  done < "${env_file}"
}

function normalize_boolean_value() {
  local value="${1:-}"
  if [[ -z "${value}" ]]; then
    return 1
  fi

  case "${value,,}" in
    1 | true | yes)
      echo "true"
      return 0
      ;;
    0 | false | no)
      echo "false"
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

function gpu_host_flag_from_preference() {
  local use_gpu="$1"
  if [[ "${use_gpu}" == "true" ]]; then
    echo "1"
  else
    echo "0"
  fi
}

function mode_override_var_name() {
  local mode="$1"
  local suffix="$2"
  local prefix
  prefix="$(mode_env_prefix "${mode}")"
  echo "${prefix}_${suffix}"
}

function mode_override_value() {
  local mode="$1"
  local suffix="$2"
  local var_name
  var_name="$(mode_override_var_name "${mode}" "${suffix}")"
  if [[ -n "${!var_name:-}" ]]; then
    echo "${!var_name}"
  fi
}

function load_project_env_overrides() {
  local project_root="$1"
  local env_file=""
  if ! env_file="$(resolve_project_env_file "${project_root}")"; then
    return 0
  fi

  echo ">>> Loading project overrides from ${env_file}"
  load_env_file_exports "${env_file}"
}

# 1. Resolve DISPLAY deterministically for automation-friendly startup
function detect_display() {
  if [[ -n "${WHL_DISPLAY:-}" ]]; then
    echo "${WHL_DISPLAY}"
    return
  fi
  echo "${DISPLAY:-:0}"
}

# 2. Check if GPU is physically available
function gpu_available() {
  local host_arch="$(uname -m)"
  if [[ "${host_arch}" == "aarch64" ]]; then
    if [[ -x "$(command -v nvidia-smi)" ]] || lsmod | grep -q "^nvgpu"; then
      return 0
    fi
  elif [[ "${host_arch}" == "x86_64" ]]; then
    if [[ -x "$(command -v nvidia-smi)" ]] && nvidia-smi -L &>/dev/null; then
      return 0
    fi
  fi
  return 1
}

# 3. Resolve GPU preference deterministically when mode-specific overrides are absent
function detect_gpu_use_interactive() {
  local auto_gpu="false"
  if gpu_available; then
    auto_gpu="true"
  fi
  echo "${auto_gpu}"
}

function calculate_dreamview_port() {
  local base_port=${1:-8888}
  local project_root="${2:-${PROJECT_ROOT:-}}"
  local uid_offset=$(( $(id -u) % 1000 ))
  local project_offset=0
  local extra_offset=0
  if [[ -n "${project_root}" ]]; then
    project_offset=$(( 0x$(project_hash_suffix "${project_root}" | cut -c1-3) % 200 ))
  fi
  if [[ -n "${WHL_PORT_OFFSET:-}" && "${WHL_PORT_OFFSET}" =~ ^[0-9]+$ ]]; then
    extra_offset=$(( WHL_PORT_OFFSET % 1000 ))
  fi
  echo $(( base_port + uid_offset + project_offset + extra_offset ))
}

function detect_os_version() {
  local os_version="${OS:-}"
  if [[ -n "${os_version}" ]]; then
    echo "${os_version}"
    return
  fi

  local detected_os="22.04"
  if [[ -f /etc/os-release ]]; then
    # shellcheck source=/dev/null
    source /etc/os-release 2>/dev/null || true
    if [[ "${ID:-}" == "ubuntu" && -n "${VERSION_ID:-}" ]]; then
      detected_os="${VERSION_ID}"
    fi
  fi

  echo "${detected_os}"
}

function detect_timezone() {
  if command -v timedatectl 2>&1 >/dev/null; then
    timedatectl | grep "Time zone" | awk '{print $3}'
  elif [[ -f /etc/timezone ]]; then
    cat /etc/timezone
  elif [[ -L /etc/localtime ]]; then
    readlink -f /etc/localtime | sed 's|.*/zoneinfo/||'
  else
    local tzoffset="$(date +"%z")"
    local tzoffset_sign="${tzoffset:0:1}"
    local tzoffset_sign_r
    tzoffset_sign_r="$(echo "${tzoffset_sign}" | sed 's/+/@/g; s/-/+/g; s/@/-/g')"
    local tzoffset_hours=$((10#${tzoffset:1:2}))
    echo "Etc/GMT${tzoffset_sign_r}${tzoffset_hours}"
  fi
}

function resolve_mode_use_gpu() {
  local mode="$1"
  local normalized
  local use_gpu

  use_gpu="$(mode_override_value "${mode}" "USE_GPU")"
  if normalized="$(normalize_boolean_value "${use_gpu}")"; then
    echo "${normalized}"
    return 0
  fi

  use_gpu="$(mode_override_value "${mode}" "USE_GPU_HOST")"
  if normalized="$(normalize_boolean_value "${use_gpu}")"; then
    echo "${normalized}"
    return 0
  fi

  return 1
}

function resolve_mode_image_override() {
  local mode="$1"
  mode_override_value "${mode}" "APOLLO_IMAGE"
}

function resolve_mode_container_name() {
  local mode="$1"
  local project_hash="$2"
  echo "apollo_${mode}_${USER}_${project_hash}"
}

function resolve_mode_server_port() {
  local mode="$1"
  local project_root="$2"
  local override_port
  override_port="$(mode_override_value "${mode}" "SERVER_PORT")"
  if [[ -n "${override_port}" ]]; then
    echo "${override_port}"
    return 0
  fi
  calculate_dreamview_port 8888 "${project_root}"
}

function resolve_mode_bazel_cache_dir() {
  local mode="$1"
  local project_root="$2"
  local override_dir
  override_dir="$(mode_override_value "${mode}" "BAZEL_CACHE_DIR")"
  if [[ -n "${override_dir}" ]]; then
    echo "${override_dir}"
    return 0
  fi
  echo "${project_root}/.cache/bazel/${mode}/repo_cache"
}

function resolve_mode_shm_size() {
  local mode="$1"
  local override_size
  override_size="$(mode_override_value "${mode}" "SHM_SIZE")"
  if [[ -n "${override_size}" ]]; then
    echo "${override_size}"
    return 0
  fi
  echo "2g"
}

function resolve_mode_test_cpus() {
  local override_value
  override_value="$(mode_override_value "test" "CPUS")"
  if [[ -n "${override_value}" ]]; then
    echo "${override_value}"
    return 0
  fi
  echo "4"
}

function resolve_mode_test_memory() {
  local override_value
  override_value="$(mode_override_value "test" "MEMORY")"
  if [[ -n "${override_value}" ]]; then
    echo "${override_value}"
    return 0
  fi
  echo "8G"
}

function generate_env() {
  local mode="$1"
  local project_root="$2"
  local docker_dir="$3"
  local apollo_image="$4"

  local env_file
  env_file="$(generated_runtime_env_path "${mode}" "${docker_dir}")"
  local runtime_env_name
  runtime_env_name="$(generated_runtime_env_name "${mode}")"
  local project_hash
  project_hash="$(project_hash_suffix "${project_root}")"
  local container_name
  container_name="$(resolve_mode_container_name "${mode}" "${project_hash}")"
  local prod_env_file="${docker_dir}/.env.prod"
  local prod_env_template="${docker_dir}/.env.prod.template"

  echo ">>> Checking Environment Config..."
  local final_display
  final_display="$(detect_display)"
  local final_tz
  final_tz="$(detect_timezone)"
  local dreamview_port
  dreamview_port="$(resolve_mode_server_port "${mode}" "${project_root}")"
  local target_arch="${TARGET_ARCH:-$(uname -m)}"
  local bazel_cache_dir
  bazel_cache_dir="$(resolve_mode_bazel_cache_dir "${mode}" "${project_root}")"
  local shm_size
  shm_size="$(resolve_mode_shm_size "${mode}")"
  local test_cpus
  test_cpus="$(resolve_mode_test_cpus)"
  local test_memory
  test_memory="$(resolve_mode_test_memory)"
  local use_gpu_host="${USE_GPU_HOST:-0}"

  echo ">>> Generating ${env_file} for [${mode}]..."
  CONTAINER_NAME="${container_name}"
  export CONTAINER_NAME
  export DISPLAY="${final_display}"
  export DREAMVIEW_PORT="${dreamview_port}"
  export SERVER_PORT="${dreamview_port}"
  export USE_GPU_HOST="${use_gpu_host}"

  mkdir -p "${bazel_cache_dir}"

  local tmp_env_file
  tmp_env_file="${env_file}.tmp.$(date +%s).$$"
  cat >"${tmp_env_file}" <<ENV_EOF
APOLLO_ROOT=${project_root}
APOLLO_IMAGE=${apollo_image}
CONTAINER_NAME=${container_name}
USER_NAME=${USER}
USER_ID=$(id -u)
GROUP_ID=$(id -g)
BAZEL_CACHE_DIR=${bazel_cache_dir}
TARGET_ARCH=${target_arch}
TZ=${final_tz}
DISPLAY=${final_display}
SHM_SIZE=${shm_size}
RUNTIME_ENV_FILE=${runtime_env_name}

# Dynamic port
SERVER_PORT=${dreamview_port}
DREAMVIEW_PORT=${dreamview_port}
USE_GPU_HOST=${use_gpu_host}
AUTO_BOOTSTRAP=${AUTO_BOOTSTRAP:-false}
ENV_EOF

  if [[ "${mode}" == "test" ]]; then
    cat >>"${tmp_env_file}" <<ENV_EOF
TEST_CPUS=${test_cpus}
TEST_MEMORY=${test_memory}
ENV_EOF
  fi

  mv "${tmp_env_file}" "${env_file}"
  chmod 644 "${env_file}" || true

  if [[ "${mode}" == "prod" ]]; then
    if [[ ! -f "${prod_env_file}" && -f "${prod_env_template}" ]]; then
      cp "${prod_env_template}" "${prod_env_file}"
      echo ">>> Created ${prod_env_file} from template"
    fi

    if [[ ! -f "${prod_env_file}" ]]; then
      echo ">>> ERROR: Missing prod env file: ${prod_env_file}"
      echo ">>> Hint: copy ${prod_env_template} to ${prod_env_file} and update values."
      exit 1
    fi
    echo ">>> Using prod env file: ${prod_env_file}"
  fi

  export DREAMVIEW_PORT="${dreamview_port}"
}
