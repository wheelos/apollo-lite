#!/usr/bin/env bash

set -euo pipefail

# ----- Constants -----
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

[ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

function show_help() {
  echo "Usage: whl [OPTIONS] [COMMAND] [MODE]"
  echo ""
  echo "Options:"
  echo "  -n, --name NAME    Specify container name for the selected mode"
  echo "  -i, --image IMAGE  Specify Docker image for the selected mode"
  echo "  --os VERSION       Specify OS version (default: auto-detect, fallback: 22.04)"
  echo "  -h, --help         Show this help message"
  echo "                     (Prod env file: docker/.env.prod; template: docker/.env.prod.template)"
  echo ""
  echo "User-maintained overrides live in .env.global at the project root."
  echo "Use mode-specific keys such as DEV_CONTAINER_NAME, DEV_USE_GPU,"
  echo "DEV_BAZEL_CACHE_DIR, TEST_CONTAINER_NAME, TEST_SERVER_PORT,"
  echo "TEST_USE_GPU, TEST_CPUS, TEST_MEMORY, and TEST_BAZEL_CACHE_DIR."
  echo "whl regenerates docker/.env.<mode>.local on every run for compose/runtime use."
  echo ""
  echo "Extra env overrides (useful for CI):"
  echo "  WHL_PROJECT_SUFFIX  Append suffix to compose project name for multi-job isolation"
  echo "  WHL_PORT_OFFSET     Numeric offset added to Dreamview dynamic port (helps avoid port collision)"
  echo ""
  echo "Commands:"
  echo "  start      Start container for the selected mode"
  echo "  enter      Enter container (starts first if not running)"
  echo "  stop       Stop container for the selected mode, or 'all'"
  echo "  status     Show Apollo container status for the selected mode"
  echo "  update     Pull latest image and restart the selected mode"
  echo "  prune      Remove legacy Apollo containers for current user"
  echo "  help       Show this help message"
  echo ""
  echo "Modes:"
  echo "  dev        Standard development mode (Host net, Privileged)"
  echo "  test       Isolated test mode (Bridge net, Dynamic ports, optional GPU runtime)"
  echo "  prod       Production mode (Host net, Restart enabled)"
  echo "  all        Stop all managed modes (stop only)"
  echo ""
  echo "Examples:"
  echo "  whl start dev                # Start dev container using DEV_* overrides"
  echo "  whl start test               # Start test container using TEST_* overrides"
  echo "  whl stop test                # Stop only the test container"
  echo "  whl stop all                 # Stop dev/test and prod when configured"
  echo "  whl -n my_test start test    # Override the selected mode container name"
}

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
        break
        ;;
      dev | test | prod | all)
        break
        ;;
      *)
        break
        ;;
    esac
  done
}

parse_args "$@"

remaining_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    -n | --name | -i | --image | --os)
      shift 2
      ;;
    *)
      remaining_args+=("$1")
      shift
      ;;
  esac
done
set -- "${remaining_args[@]}"

ACTION="${1:-enter}"
MODE_ARG="${2:-}"
if [[ "${ACTION}" == "help" || "${ACTION}" == "--help" || "${ACTION}" == "-h" ]]; then
  show_help
  exit 0
fi

source "${DOCKER_DIR}/scripts/env_setup.sh"
source "${DOCKER_DIR}/scripts/container_selection.sh"
load_project_env_overrides "${PROJECT_ROOT}"

function get_compose_cmd() {
  if docker compose version >/dev/null 2>&1; then
    echo "docker compose"
  else
    echo "docker-compose"
  fi
}

function validate_mode() {
  local mode="$1"
  if [[ "${mode}" != "dev" && "${mode}" != "test" && "${mode}" != "prod" ]]; then
    echo ">>> ERROR: Invalid mode '${mode}'. Use 'dev', 'test' or 'prod'."
    exit 2
  fi
}

function validate_stop_target() {
  local target="$1"
  if [[ "${target}" == "all" ]]; then
    return 0
  fi
  validate_mode "${target}"
}

function action_requires_local_image() {
  local action="$1"
  case "${action}" in
    start | enter | update)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

function compose_project_name() {
  local mode="$1"
  local project_hash
  project_hash="$(project_hash_suffix "${PROJECT_ROOT}")"
  local project_name="apollo_${USER}_${project_hash}_${mode}"
  if [[ -n "${WHL_PROJECT_SUFFIX:-}" ]]; then
    project_name="${project_name}_${WHL_PROJECT_SUFFIX}"
  fi
  echo "${project_name}"
}

function prepare_mode_context() {
  local mode="$1"
  local ensure_local_image="${2:-false}"
  local mode_image=""
  local mode_use_gpu=""

  ARCH="$(uname -m)"

  if mode_use_gpu="$(resolve_mode_use_gpu "${mode}")"; then
    USE_GPU="${mode_use_gpu}"
  else
    USE_GPU="$(detect_gpu_use_interactive)"
  fi
  USE_GPU_HOST="$(gpu_host_flag_from_preference "${USE_GPU}")"
  export USE_GPU
  export USE_GPU_HOST

  if [[ -n "${CUSTOM_IMAGE}" ]]; then
    echo ">>> Using custom image: ${CUSTOM_IMAGE}"
    APOLLO_IMAGE="${CUSTOM_IMAGE}"
    export APOLLO_IMAGE
    return 0
  fi

  mode_image="$(resolve_mode_image_override "${mode}")"
  if [[ -n "${mode_image}" ]]; then
    echo ">>> Using ${mode} image from .env.global: ${mode_image}"
    APOLLO_IMAGE="${mode_image}"
    export APOLLO_IMAGE
    return 0
  fi

  OS="${OS:-$(detect_os_version)}"
  select_container "${ARCH}" "${OS}" "${USE_GPU}" "${ensure_local_image}"
  export APOLLO_IMAGE
}

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

function verify_gpu_ready() {
  if [[ "${USE_GPU}" != "true" ]]; then
    return 0
  fi

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

function ensure_env_generated() {
  local mode="$1"
  local env_file
  env_file="$(generated_runtime_env_path "${mode}" "${DOCKER_DIR}")"
  if [[ ! -f "${env_file}" ]]; then
    echo ">>> ERROR: Expected env file not found: ${env_file}"
    echo ">>> Hint: rerun command and check output from generate_env."
    exit 1
  fi
}

function get_cmd() {
  local mode="$1"
  local base_file="${DOCKER_SERVICE_DIR}/docker-compose.yml"
  local mode_file="${DOCKER_SERVICE_DIR}/docker-compose.dev.yml"
  local extra_files=""
  local env_file
  env_file="$(generated_runtime_env_path "${mode}" "${DOCKER_DIR}")"

  if [[ "${mode}" == "test" ]]; then
    mode_file="${DOCKER_SERVICE_DIR}/docker-compose.test.yml"
    if [[ "${USE_GPU}" == "true" ]]; then
      extra_files=" -f ${DOCKER_SERVICE_DIR}/docker-compose.test.gpu.yml"
    fi
  elif [[ "${mode}" == "prod" ]]; then
    mode_file="${DOCKER_SERVICE_DIR}/docker-compose.prod.yml"
  elif [[ "${mode}" == "dev" && "${USE_GPU}" == "true" ]]; then
    extra_files=" -f ${DOCKER_SERVICE_DIR}/docker-compose.dev.gpu.yml"
  fi

  echo "$(get_compose_cmd) --env-file ${env_file} --project-name $(compose_project_name "${mode}") --project-directory ${DOCKER_DIR} -f ${base_file} -f ${mode_file}${extra_files}"
}

function stop_mode() {
  local mode="$1"
  prepare_mode_context "${mode}" "false"
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated "${mode}"
  $(get_cmd "${mode}") down 2>/dev/null || true
}

function cmd_start() {
  local mode="${1:-dev}"
  validate_mode "${mode}"
  prepare_mode_context "${mode}" "true"
  require_host_ready
  verify_gpu_ready
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated "${mode}"

  echo ">>> Starting Apollo [Mode: ${mode}]..."
  if [[ "${mode}" == "test" ]]; then
    echo ">>> Test Mode: Dreamview mapped to http://localhost:${DREAMVIEW_PORT}"
  fi

  $(get_cmd "${mode}") up -d --remove-orphans
  sleep 1
  if ! $(get_cmd "${mode}") ps --services --filter "status=running" | grep -q "core"; then
    echo ">>> ERROR: Container failed to start. Logs:"
    $(get_cmd "${mode}") logs core
    exit 1
  fi

  echo ">>> Container is running."
}

function cmd_enter() {
  local mode="${1:-dev}"
  validate_mode "${mode}"
  cmd_start "${mode}"
  xhost +local:root >/dev/null 2>&1 || true
  echo ">>> Entering container ${CONTAINER_NAME}..."
  $(get_cmd "${mode}") exec -u "${USER}" -it core /bin/bash
}

function cmd_status() {
  local mode="${1:-dev}"
  validate_mode "${mode}"
  prepare_mode_context "${mode}" "false"
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated "${mode}"
  $(get_cmd "${mode}") ps
}

function cmd_update() {
  local mode="${1:-dev}"
  validate_mode "${mode}"
  prepare_mode_context "${mode}" "true"
  require_host_ready
  verify_gpu_ready
  generate_env "${mode}" "${PROJECT_ROOT}" "${DOCKER_DIR}" "${APOLLO_IMAGE}" "${CUSTOM_CONTAINER_NAME}"
  ensure_env_generated "${mode}"
  echo ">>> Updating Apollo image and restarting [Mode: ${mode}]..."
  $(get_cmd "${mode}") pull
  $(get_cmd "${mode}") up -d --remove-orphans
}

function cmd_stop() {
  local target="${1:-dev}"
  local prod_env_file="${DOCKER_DIR}/.env.prod"
  validate_stop_target "${target}"
  echo ">>> Stopping Apollo [Target: ${target}]..."

  if [[ "${target}" == "all" ]]; then
    stop_mode "dev"
    stop_mode "test"
    if [[ -f "${prod_env_file}" ]]; then
      stop_mode "prod"
    else
      echo ">>> Skipping prod stop: ${prod_env_file} not found."
    fi
  else
    stop_mode "${target}"
  fi

  echo ">>> Stopped."
}

function cmd_prune() {
  local running_containers
  running_containers="$(docker ps -a --format '{{.Names}}')"
  for container in ${running_containers[*]}; do
    if [[ "${container}" =~ apollo_.*_${USER} ]]; then
      echo ">>> Removing container ${container} ..."
      docker stop "${container}" >/dev/null 2>&1 || true
      docker rm -v -f "${container}" >/dev/null 2>&1 || true
    fi
  done
  echo ">>> Prune complete."
}

case "${ACTION}" in
  enter)
    shift
    cmd_enter "$@"
    ;;
  start)
    shift
    cmd_start "$@"
    ;;
  stop)
    shift
    cmd_stop "$@"
    ;;
  status)
    shift
    cmd_status "$@"
    ;;
  update)
    shift
    cmd_update "$@"
    ;;
  prune)
    cmd_prune
    ;;
  help | --help | -h)
    show_help
    ;;
  *)
    echo ">>> ERROR: Unknown command '${ACTION}'"
    show_help
    exit 2
    ;;
esac
