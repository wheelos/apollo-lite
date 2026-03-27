#!/usr/bin/env bash

set -euo pipefail

# ----- Constants -----
# Resolve symlink if script is invoked from a symlink
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null 2>&1 && pwd )"
PROJECT_ROOT="$(cd "${DIR}/../.." && pwd -P)"
DOCKER_DIR="${PROJECT_ROOT}/docker"
DOCKER_SERVICE_DIR="${DOCKER_DIR}/services"
CACHE_ROOT_DIR="${PROJECT_ROOT}/.cache"
HOST_READY_MARKER="/etc/wheelos_setup_host.done"

# Ensure cache dir exists early
[ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

# Dynamic port calculation


# ----- OS Detection -----
# Detect Ubuntu version, fallback to 22.04 for non-Ubuntu or detection failure




# Detect DISPLAY environment when multiple candidates exist.
# Priority: user override `WHL_DISPLAY`, existing `$DISPLAY`, then common values `:0` and `:1`.


# ----- Command Line Arguments -----
CUSTOM_CONTAINER_NAME=""
CUSTOM_IMAGE=""

function parse_args() {
  while [[ $# -gt 0 ]]; do
    case "$1" in
      -n | --name)
        if [[ -z "${2:-}" || "${2}" == -* ]]; then
          echo ">>> ERROR: --name requires a container name argument"
          exit 2
        fi
        CUSTOM_CONTAINER_NAME="$2"
        shift 2
        ;;
      -i | --image)
        if [[ -z "${2:-}" || "${2}" == -* ]]; then
          echo ">>> ERROR: --image requires an image name argument"
          exit 2
        fi
        CUSTOM_IMAGE="$2"
        shift 2
        ;;
      --os)
        if [[ -z "${2:-}" || "${2}" == -* ]]; then
          echo ">>> ERROR: --os requires an OS version argument"
          exit 2
        fi
        OS="$2"
        shift 2
        ;;
      enter | start | stop | status | update | prune | help | --help | -h)
        # These are commands, stop parsing options
        break
        ;;
      dev | test | prod)
        # These are modes, stop parsing options
        break
        ;;
      *)
        # Unknown option or positional argument
        break
        ;;
    esac
  done
}

# Parse options before the command
parse_args "$@"

# Shift parsed options, keep remaining args for command routing
remaining_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n | --name)
      shift 2
      ;;
    -i | --image)
      shift 2
      ;;
    --os)
      shift 2
      ;;
    *)
      remaining_args+=("$1")
      shift
      ;;
  esac
done
set -- "${remaining_args[@]}"

# Prepare to call the container selection script


# ----- Phase 1: Container Selection -----

# Source environment utilities
source "${DOCKER_DIR}/scripts/env_setup.sh"
# Source container selection once so select_container is always available.
source "${DOCKER_DIR}/scripts/container_selection.sh"

# Helper: prepare APOLLO_IMAGE and CONTAINER_NAME. If a valid non-empty
# docker/.env exists (and WHL_FORCE_REGENERATE_ENV is not set), reuse it to
# avoid prompting; otherwise detect host and run selection logic.
prepare_image_selection() {
  local env_file="${DOCKER_DIR}/.env"
  ARCH=$(uname -m)

  if [[ -z "${WHL_FORCE_REGENERATE_ENV:-}" && -f "${env_file}" && -s "${env_file}" ]]; then
    echo ">>> Reusing existing ${env_file} to avoid interactive prompts."
    try_reuse_env_file "${env_file}" || true
    # If APOLLO_IMAGE is already provided from env, we're done.
    if [[ -n "${APOLLO_IMAGE:-}" ]]; then
      echo ">>> Using image from .env: ${APOLLO_IMAGE}"
      # Ensure USE_GPU is defined to avoid unbound variable errors later.
      if [[ -z "${USE_GPU:-}" ]]; then
        if gpu_available; then
          USE_GPU="true"
        else
          USE_GPU="false"
        fi
      fi
      return 0
    fi
  fi

  # Fallback: detect host and ask interactively as before
  OS=$(detect_os_version)
  USE_GPU=$(detect_gpu_use_interactive)
  select_container "$ARCH" "$OS" "$USE_GPU"
}

# Prepare image/container name
prepare_image_selection

function require_host_ready() {
  if [[ "${WHL_SKIP_HOST_CHECK:-0}" == "1" ]]; then
    return 0
  fi
  if [[ ! -f "${HOST_READY_MARKER}" ]]; then
    echo ">>> ERROR: Host is not initialized. Run setup_host first."
    echo ">>> Hint: sudo ${PROJECT_ROOT}/docker/setup_host/setup_host.sh"
    exit 1
  fi
}

function ensure_env_generated() {
  local env_file="${DOCKER_DIR}/.env"
  if [[ ! -f "${env_file}" ]]; then
    echo ">>> ERROR: Expected env file not found: ${env_file}"
    echo ">>> Hint: rerun command and check output from generate_env."
    exit 1
  fi
}

function verify_gpu_ready() {
  if [[ "${USE_GPU}" != "true" ]]; then
    return 0
  fi

  # no need to check gpu on this function(already checked above)
  # if ! command -v nvidia-smi >/dev/null  2>&1; then
  #   echo ">>> ERROR: GPU requested but 'nvidia-smi' is not available on host."
  #   exit 1
  # fi

  local docker_info_output
  if ! docker_info_output="$(docker info --format '{{json .Runtimes}}' 2>/dev/null)"; then
    echo ">>> ERROR: Unable to query Docker daemon."
    echo ">>> Hint: check Docker service or add user to the docker group and re-login."
    exit 1
  fi

  if [[ -z "${docker_info_output}" ]]; then
    echo ">>> ERROR: Docker returned empty runtime information."
    echo ">>> Hint: check Docker daemon status and permissions."
    exit 1
  fi

  if ! echo "${docker_info_output}" | grep -q 'nvidia'; then
    echo ">>> ERROR: GPU requested but Docker runtime 'nvidia' is not configured."
    echo ">>> Hint: run setup_host to install NVIDIA Container Toolkit."
    exit 1
  fi
}

require_host_ready
verify_gpu_ready

# Use custom image or select container automatically
if [[ -n "${CUSTOM_IMAGE}" ]]; then
  echo ">>> Using custom image: ${CUSTOM_IMAGE}"
  APOLLO_IMAGE="${CUSTOM_IMAGE}"
elif [[ -n "${APOLLO_IMAGE:-}" ]]; then
  echo ">>> Using image from environment: ${APOLLO_IMAGE}"
else
  # Ensure OS/USE_GPU are defined before calling interactive selector
  OS=${OS:-$(detect_os_version)}
  USE_GPU=${USE_GPU:-$(detect_gpu_use_interactive)}
  select_container "$ARCH" "$OS" "$USE_GPU"
fi

# ----- Phase 2: Environment Variable Generation -----


function get_compose_cmd() {
  if docker compose version >/dev/null  2>&1; then
    echo "docker compose"
  else
    echo "docker-compose"
  fi
}

function get_cmd() {
  local mode="$1"
  local base_file="${DOCKER_SERVICE_DIR}/docker-compose.yml"
  local mode_file="${DOCKER_SERVICE_DIR}/docker-compose.dev.yml"
  if [[ "${mode}" == "test" ]]; then
    mode_file="${DOCKER_SERVICE_DIR}/docker-compose.test.yml"
  elif [[ "${mode}" == "prod" ]]; then
    mode_file="${DOCKER_SERVICE_DIR}/docker-compose.prod.yml"
  fi

  # Generate unique project name based on user and project directory
  # This ensures container isolation between different users and projects
  local project_hash
  project_hash=$(echo "${PROJECT_ROOT}" | md5sum | cut -c1-8)
  local project_name="apollo_${USER}_${project_hash}"

  local compose_cmd
  compose_cmd="$(get_compose_cmd)"
  echo "${compose_cmd} --project-name ${project_name} --project-directory ${DOCKER_DIR} -f ${base_file} -f ${mode_file}"
}

function validate_mode() {
  local mode="$1"
  if [[ "${mode}" != "dev" && "${mode}" != "test" && "${mode}" != "prod" ]]; then
    echo ">>> ERROR: Invalid mode '${mode}'. Use 'dev', 'test' or 'prod'."
    exit 2
  fi
}



# ----- Phase 3: Start Container -----
function cmd_start() {
  local mode="${1:-dev}"
  validate_mode "${mode}"
  require_host_ready
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated

  echo ">>> Starting Apollo [Mode: ${mode}]..."
  if [[ "${mode}" == "test" ]]; then
    echo ">>> Test Mode: Dreamview mapped to http://localhost:${DREAMVIEW_PORT}"
  fi

  $(get_cmd "${mode}") up -d --remove-orphans
  # Check if the startup was successful.
  sleep 1
  if ! $(get_cmd "${mode}") ps --services --filter "status=running" | grep -q "core"; then
    echo ">>> ERROR: Container failed to start. Logs:"
    $(get_cmd "${mode}") logs core
    exit 1
  fi

  echo ">>> Container is running."
}

function cmd_enter() {
  local mode=${1:-dev}
  validate_mode "${mode}"
  require_host_ready
  # Ensure the container is running.
  cmd_start "$mode"

  # Allow X11
  xhost +local:root >/dev/null  2>&1 || true

  echo ">>> Entering container ${CONTAINER_NAME}..."
  $(get_cmd "${mode}") exec -u "${USER}" -it core /bin/bash
}

function cmd_status() {
  local mode=${1:-dev}
  validate_mode "${mode}"
  require_host_ready
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated
  $(get_cmd "${mode}") ps
}

function cmd_update() {
  local mode=${1:-dev}
  validate_mode "${mode}"
  require_host_ready
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated
  echo ">>> Updating Apollo image and restarting [Mode: ${mode}]..."
  $(get_cmd "${mode}") pull
  $(get_cmd "${mode}") up -d --remove-orphans
}

function cmd_stop() {
  echo ">>> Stopping containers..."
  generate_env "dev" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated
  $(get_cmd "dev") down 2>/dev/null  || true
  generate_env "test" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated
  $(get_cmd "test") down 2>/dev/null  || true
  generate_env "prod" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated
  $(get_cmd "prod") down 2>/dev/null  || true
  echo ">>> Stopped."
}

function cmd_prune() {
  local running_containers
  running_containers="$(docker ps -a --format '{{.Names}}')"
  for container in ${running_containers[*]}; do
    if [[ "${container}" =~ apollo_.*_${USER} ]]; then
      echo ">>> Removing container ${container} ..."
      docker stop "${container}" >/dev/null  2>&1 || true
      docker rm -v -f "${container}" >/dev/null  2>&1 || true
    fi
  done
  echo ">>> Prune complete."
}

# ----- Main Script Execution -----
function show_help() {
  echo "Usage: whl [OPTIONS] [COMMAND] [MODE]"
  echo ""
  echo "Options:"
  echo "  -n, --name NAME    Specify container name (default: apollo_{dev|test|prod}_{USER})"
  echo "  -i, --image IMAGE  Specify Docker image (skip auto-selection)"
  echo "  --os VERSION       Specify OS version (default: auto-detect, fallback: 22.04)"
  echo "  -h, --help         Show this help message"
  echo "                     (Prod env file: docker/.env.prod; template: docker/.env.prod.template)"
  echo ""
  echo "Commands:"
  echo "  start      Start container (skips if running, restarts if stopped)"
  echo "  enter      Enter container (starts first if not running)"
  echo "  stop       Stop and remove containers"
  echo "  status     Show Apollo container status"
  echo "  update     Pull latest image and restart"
  echo "  prune      Remove legacy Apollo containers for current user"
  echo "  help       Show this help message"
  echo ""
  echo "Modes:"
  echo "  dev        Standard development mode (Host net, Privileged)"
  echo "  test       Isolated test mode (Bridge net, Dynamic ports)"
  echo "  prod       Production mode (Host net, Restart enabled)"
  echo ""
  echo "Examples:"
  echo "  whl enter                    # Enter dev container (auto-select image)"
  echo "  whl -n my_container enter    # Enter container with custom name"
  echo "  whl -i myimage:latest start  # Start with custom image"
  echo "  whl start prod               # Start production container"
  echo "  whl --os 20.04 enter dev     # Enter with Ubuntu 20.04 image"
}

# Simple routing
ACTION="${1:-enter}"
case "${ACTION}" in
  enter)
    shift
    cmd_enter "$@"
    ;;
  start)
    shift
    cmd_start "$@"
    ;;
  stop) cmd_stop ;;
  status)
    shift
    cmd_status "$@"
    ;;
  update)
    shift
    cmd_update "$@"
    ;;
  prune) cmd_prune ;;
  help | --help | -h) show_help ;;
  *)
    echo ">>> ERROR: Unknown command '${ACTION}'"
    show_help
    exit 2
    ;;
esac
