#!/usr/bin/env bash

# Interactive prompt helper with timeout
function prompt_with_default() {
  local prompt_msg=$1
  local default_val=$2
  local timeout_sec=${3:-5}

  # Non-interactive shell, just return default
  if [[ ! -t 0 ]]; then
    echo "${default_val}"
    return
  fi

  # Interactive shell, read user input
  local user_input
  if read -t "${timeout_sec}" -rp "${prompt_msg} [${default_val}]: " user_input; then
    echo "${user_input:-${default_val}}"
  else
    echo "" >&2 # Add newline after timeout
    echo "${default_val}"
  fi
}

# 1. Provide an interactive fallback for DISPLAY detection
function detect_display() {
  if [[ -n "${WHL_DISPLAY:-}" ]]; then
    echo "${WHL_DISPLAY}"
    return
  fi
  local current_display="${DISPLAY:-:0}"
  prompt_with_default "Select X11 DISPLAY" "${current_display}" 3
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

# 3. Ask whether to use GPU if applicable
function detect_gpu_use_interactive() {
  local auto_gpu="false"
  if gpu_available; then
      auto_gpu="true"
  fi
  # If already set by command line/env, just return it
  if [[ "${USE_GPU:-auto}" != "auto" ]]; then
      echo "${USE_GPU}"
      return
  fi
  # Otherwise, ask user interactively
  prompt_with_default "Enable GPU support (true/false)?" "${auto_gpu}" 3
}

function calculate_dreamview_port() {
  local base_port=${1:-8888}
  local offset=$(($(id -u) % 1000))
  echo $((base_port + offset))
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

  if [[ -t 0 ]]; then
    prompt_with_default "Detected OS Version" "${detected_os}" 3
  else
    echo "${detected_os}"
  fi
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
    local tzoffset_sign_r=$(echo "${tzoffset_sign}" | sed 's/+/@/g; s/-/+/g; s/@/-/g')
    local tzoffset_hours=$((10#${tzoffset:1:2}))
    echo "Etc/GMT${tzoffset_sign_r}${tzoffset_hours}"
  fi
}

# Try to reuse an existing env file to avoid interactive prompts.
# Usage: try_reuse_env_file <env_file>
try_reuse_env_file() {
  local env_file="$1"
  if [[ -z "${WHL_FORCE_REGENERATE_ENV:-}" && -f "${env_file}" && -s "${env_file}" ]]; then
    echo ">>> Using existing ${env_file}; set WHL_FORCE_REGENERATE_ENV=1 to force regeneration."
    # Preserve caller-provided AUTO_BOOTSTRAP (e.g., systemd autostart service)
    # so that reusing .env does not silently override runtime intent.
    local had_auto_bootstrap="0"
    local inherited_auto_bootstrap=""
    if [[ -n "${AUTO_BOOTSTRAP+x}" ]]; then
      had_auto_bootstrap="1"
      inherited_auto_bootstrap="${AUTO_BOOTSTRAP}"
    fi

    # Safely parse KEY=VALUE pairs only (avoid sourcing arbitrary code).
    # Supports quoted and unquoted values, ignores comments and blank lines.
    while IFS= read -r _line || [[ -n "$_line" ]]; do
      local line="$_line"
      # Trim leading/trailing whitespace
      line="$(echo "${line}" | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//')"
      # Skip empty or comment lines
      [[ -z "${line}" || "${line:0:1}" == "#" ]] && continue
      if [[ "${line}" =~ ^([A-Za-z_][A-Za-z0-9_]*)=(.*)$ ]]; then
        local key="${BASH_REMATCH[1]}"
        local val="${BASH_REMATCH[2]}"
        # Remove surrounding quotes if present
        if [[ "${val}" =~ ^\"(.*)\"$ ]]; then
          val="${BASH_REMATCH[1]}"
        elif [[ "${val}" =~ ^\'(.*)\'$ ]]; then
          val="${BASH_REMATCH[1]}"
        fi
        export "${key}"="${val}"
      fi
    done < "${env_file}"

    if [[ "${had_auto_bootstrap}" == "1" ]]; then
      export AUTO_BOOTSTRAP="${inherited_auto_bootstrap}"
    fi

    # Ensure DREAMVIEW_PORT exported (fall back to calculation if missing)
    if [[ -n "${SERVER_PORT:-}" ]]; then
      export DREAMVIEW_PORT="${SERVER_PORT}"
    elif [[ -n "${DREAMVIEW_PORT:-}" ]]; then
      export DREAMVIEW_PORT="${DREAMVIEW_PORT}"
    else
      export DREAMVIEW_PORT="$(calculate_dreamview_port)"
    fi
    return 0
  fi
  return 1
}

function generate_env() {
  local mode="$1"
  local project_root="$2"
  local docker_dir="$3"
  local apollo_image="$4"
  local custom_container_name="$5"

  local env_file="${docker_dir}/.env"
  if try_reuse_env_file "${env_file}"; then
    return 0
  fi

  local container_name="apollo_dev_${USER}"
  local prod_env_file="${docker_dir}/.env.prod"
  local prod_env_template="${docker_dir}/.env.prod.template"

  if [[ "${mode}" == "test" ]]; then
    container_name="apollo_test_${USER}"
  elif [[ "${mode}" == "prod" ]]; then
    container_name="apollo_prod_${USER}"
  fi

  # Override with custom container name if specified
  if [[ -n "${custom_container_name}" ]]; then
    container_name="${custom_container_name}"
  fi

  # Interactively gather variables and configuration
  echo ">>> Checking Environment Config..."
  local final_display="$(detect_display)"
  local final_tz="$(detect_timezone)"
  local dreamview_port="$(calculate_dreamview_port)"
  local target_arch="${TARGET_ARCH:-$(uname -m)}"
  local bazel_cache_dir="${BAZEL_CACHE_DIR:-/var/cache/bazel/repo_cache}"

  # If running prod mode and a prod env file already exists, reuse it to avoid prompts.
  if [[ "${mode}" == "prod" ]] && try_reuse_env_file "${prod_env_file}"; then
    return 0
  fi

  echo ">>> Generating .env for [${mode}]..."
  CONTAINER_NAME="${container_name}"
  export CONTAINER_NAME
  export DISPLAY="${final_display}"

  # Write .env atomically to avoid readers seeing a partially-written file.
  local tmp_env_file
  tmp_env_file="${docker_dir}/.env.tmp.$(date +%s).$$"
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
SHM_SIZE=2g

# Dynamic port (Test mode)
SERVER_PORT=${dreamview_port}

# Also export DREAMVIEW_PORT for consistency with scripts that read it directly.
DREAMVIEW_PORT=${dreamview_port}

# Controlling Entrypoint Behavior
AUTO_BOOTSTRAP=${AUTO_BOOTSTRAP:-false}
ENV_EOF

  # Move into place atomically
  mv "${tmp_env_file}" "${docker_dir}/.env"
  chmod 644 "${docker_dir}/.env" || true

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

  # Output some runtime context for parent script
  export DREAMVIEW_PORT="${dreamview_port}"
}
