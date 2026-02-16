#!/usr/bin/env bash

set -euo pipefail

# ----- Constants -----
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
DOCKER_DIR="${PROJECT_ROOT}/docker"
DOCKER_SERVICE_DIR="${DOCKER_DIR}/services"
CACHE_ROOT_DIR="${PROJECT_ROOT}/.cache"
HOST_READY_MARKER="/etc/wheelos_setup_host.done"

# Ensure cache dir exists early
[ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

# Dynamic port calculation
function calculate_dreamview_port() {
    local base_port=8888
    local offset=$(($(id -u) % 1000))
    local dreamview_port=$((base_port + offset))
    echo $dreamview_port
}
DREAMVIEW_PORT=$(calculate_dreamview_port)

# Prepare to call the container selection script
ARCH=$(uname -m)
OS="${OS:-20.04}"
USE_GPU="${USE_GPU:-auto}"
BAZEL_CACHE_DIR="${BAZEL_CACHE_DIR:-/var/cache/bazel/repo_cache}"
TARGET_ARCH="${TARGET_ARCH:-${ARCH}}"
SYSTEM_TZ="${SYSTEM_TZ:-$(cat /etc/timezone 2>/dev/null || date +%Z)}"

# ----- Phase 1: Container Selection -----

# Call the container selection script
source "${DOCKER_DIR}/container_selection.sh"

function detect_gpu_use() {
    if [[ "${ARCH}" == "aarch64" ]]; then
        if lsmod | grep -q "^nvgpu"; then
            echo "true"
        else
            echo "false"
        fi
    else
        if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
            echo "true"
        else
            echo "false"
        fi
    fi
}

function require_host_ready() {
    if [[ "${WHL_SKIP_HOST_CHECK:-0}" == "1" ]]; then
        return 0
    fi
    if [[ ! -f "${HOST_READY_MARKER}" ]]; then
        echo ">>> ERROR: Host is not initialized. Run setup_host first."
        echo ">>> Hint: sudo ${DOCKER_DIR}/setup_host/setup_host.sh"
        exit 1
    fi
}

function verify_gpu_ready() {
    if [[ "${USE_GPU}" != "true" ]]; then
        return 0
    fi

    if ! command -v nvidia-smi >/dev/null 2>&1; then
        echo ">>> ERROR: GPU requested but 'nvidia-smi' is not available on host."
        exit 1
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

if [[ "${USE_GPU}" == "auto" ]]; then
    USE_GPU="$(detect_gpu_use)"
fi
require_host_ready
verify_gpu_ready
select_container "$ARCH" "$OS" "$USE_GPU"

# ----- Phase 2: Environment Variable Generation -----
function generate_env() {
    local mode="$1"
    local container_name="apollo_dev_${USER}"

    if [[ "${mode}" == "test" ]]; then
        container_name="apollo_test_${USER}"
    fi

    echo ">>> Generating .env for [${mode}]..."

    CONTAINER_NAME="${container_name}"
    export CONTAINER_NAME

    cat > "${DOCKER_DIR}/.env" <<EOF
APOLLO_ROOT=${PROJECT_ROOT}
APOLLO_IMAGE=${APOLLO_IMAGE}
CONTAINER_NAME=${container_name}
USER_NAME=${USER}
USER_ID=$(id -u)
GROUP_ID=$(id -g)
BAZEL_CACHE_DIR=${BAZEL_CACHE_DIR}
TARGET_ARCH=${TARGET_ARCH}
TZ=${SYSTEM_TZ}
DISPLAY=${DISPLAY:-:0}
SHM_SIZE=2g

# Dynamic port (Test mode)
SERVER_PORT=${DREAMVIEW_PORT}

# Controlling Entrypoint Behavior
AUTO_BOOTSTRAP=${AUTO_BOOTSTRAP:-false}
EOF
}

function get_compose_cmd() {
    if docker compose version >/dev/null 2>&1; then
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
    fi

    local compose_cmd
    compose_cmd="$(get_compose_cmd)"
    echo "${compose_cmd} --project-directory ${DOCKER_DIR} --env-file ${DOCKER_DIR}/.env -f ${base_file} -f ${mode_file}"
}

function validate_mode() {
    local mode="$1"
    if [[ "${mode}" != "dev" && "${mode}" != "test" ]]; then
        echo ">>> ERROR: Invalid mode '${mode}'. Use 'dev' or 'test'."
        exit 2
    fi
}

# ----- Phase 3: Start Container -----
function cmd_start() {
    local mode="${1:-dev}"
    validate_mode "${mode}"
    require_host_ready
    generate_env "${mode}"

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
    xhost +local:root >/dev/null 2>&1 || true

    echo ">>> Entering container ${CONTAINER_NAME}..."
    $(get_cmd "${mode}") exec -it core /bin/bash
}

function cmd_status() {
    local mode=${1:-dev}
    validate_mode "${mode}"
    require_host_ready
    generate_env "${mode}"
    $(get_cmd "${mode}") ps
}

function cmd_update() {
    local mode=${1:-dev}
    validate_mode "${mode}"
    require_host_ready
    generate_env "${mode}"
    echo ">>> Updating Apollo image and restarting [Mode: ${mode}]..."
    $(get_cmd "${mode}") pull
    $(get_cmd "${mode}") up -d --remove-orphans
}

function cmd_stop() {
    echo ">>> Stopping containers..."
    generate_env "dev"
    $(get_cmd "dev") down 2>/dev/null || true
    generate_env "test"
    $(get_cmd "test") down 2>/dev/null || true
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


# ----- Main Script Execution -----
function show_help() {
    echo "Usage: bash whl.sh [COMMAND] [MODE]"
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
    echo "  test    Isolated test mode (Bridge net, Dynamic ports)"
}

# Simple routing
ACTION="${1:-enter}"
case "${ACTION}" in
    enter)  shift; cmd_enter "$@" ;;
    start)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    status) shift; cmd_status "$@" ;;
    update) shift; cmd_update "$@" ;;
    prune)  cmd_prune ;;
    help|--help|-h) show_help ;;
    *)      echo ">>> ERROR: Unknown command '${ACTION}'"; show_help; exit 2 ;;
esac
