#!/usr/bin/env bash

set -euo pipefail

# ----- Constants -----
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
DOCKER_DIR="${PROJECT_ROOT}/docker"
DOCKER_SERVICE_DIR="${DOCKER_DIR}/services"
CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"

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
OS="20.04"  # Specify OS version
USE_GPU="true"

# ----- Phase 1: Container Selection -----

# Call the container selection script
source "${DOCKER_DIR}/container_selection.sh"
select_container "$ARCH" "$OS" "$USE_GPU"

# ----- Phase 2: Environment Variable Generation -----
function generate_env() {
    local mode="$1"
    local container_name="apollo_dev_${USER}"

    if [[ "${mode}" == "testing" ]]; then
        container_name="apollo_test_${USER}"
    fi

    echo ">>> Generating .env for [${mode}]..."

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
AUTO_BOOTSTRAP=false
EOF
}

# ----- Phase 3: Start Container -----
function cmd_start() {
    local mode="${1:-dev}"
    generate_env "${mode}"

    echo ">>> Starting Apollo [Mode: ${mode}]..."
    if [[ "${mode}" == "testing" ]]; then
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
    # Ensure the container is running.
    cmd_start "$mode"

    # Allow X11
    xhost +local:root >/dev/null 2>&1 || true

    echo ">>> Entering container ${CONTAINER_NAME}..."
    $(get_cmd "${mode}") exec -it core /bin/bash
}

function cmd_stop() {
    echo ">>> Stopping containers..."
    generate_env "dev"
    $(get_cmd "dev") down 2>/dev/null || true
    $(get_cmd "testing") down 2>/dev/null || true
    echo ">>> Stopped."
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
    echo "  help       Show this help message"
    echo ""
    echo "Modes:"
    echo "  dev        Standard development mode (Host net, Privileged)"
    echo "  testing    Isolated test mode (Bridge net, Dynamic ports)"
}

# Simple routing
ACTION="${1:-enter}"
case "${ACTION}" in
    enter)  shift; cmd_start "$@" ;;
    stop)   cmd_stop ;;
    update) shift; cmd_update "$@" ;;
    help|--help|-h) show_help ;;
    *)      cmd_start "$@" ;;
esac
