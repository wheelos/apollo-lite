#!/usr/bin/env bash

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -euo pipefail # Added for robustness

# --- Constants ---
DEV_CONTAINER_PREFIX='apollo_dev_'
DEV_INSIDE="in-dev-docker" # Consistent with start_container.sh

# --- Global Variables (will be set by parse_arguments) ---
# DOCKER_USER will be the username used for 'docker exec -u'
# Default to host's USER, can be overridden by --user
DOCKER_USER="${USER}"
# DEV_CONTAINER will be the name of the container to connect to
# Default to apollo_dev_host_user, can be overridden by --user or -n
DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}" # Initialize with default

# Placeholder for user-specified container name via -n
USER_EXPLICIT_CONTAINER_NAME=""

# --- Helper Functions ---
function show_usage() {
    cat <<EOF
Usage: $0 [options]
Connects to a running Apollo development Docker container.

OPTIONS:
    -h, --help              Display this help and exit.
    -n, --name <container_name> Specify the *full* name of the docker container to connect to.
                            Overrides default naming based on user.
    --user <username>       Specify the username to log into inside the container.
                            This also influences the container name if -n is not used.
                            (Default: current host user for both container name and login user).
EOF
}

function parse_arguments() {
    local opt_n_value=""
    local opt_user_value=""

    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case "${opt}" in
            -h | --help)
                show_usage
                exit 0
                ;;
            -n | --name)
                opt_n_value="$1"
                shift
                if [[ -z "${opt_n_value}" ]]; then
                    echo "Error: Missing argument for -n/--name." >&2
                    exit 1
                fi
                USER_EXPLICIT_CONTAINER_NAME="${opt_n_value}" # <-- Here's the change: no prefixing
                ;;
            --user)
                opt_user_value="$1"
                shift
                if [[ -z "${opt_user_value}" ]]; then
                    echo "Error: Missing argument for --user." >&2
                    exit 1
                fi
                DOCKER_USER="${opt_user_value}"
                ;;
            *)
                echo "Error: Unknown option: ${opt}" >&2
                show_usage
                exit 1
                ;;
        esac
    done

    # --- Determine the final DEV_CONTAINER name ---
    # Priority: -n (explicit full name) > --user (implied prefixed name) > host USER (default prefixed name)
    if [[ -n "${USER_EXPLICIT_CONTAINER_NAME}" ]]; then
        DEV_CONTAINER="${USER_EXPLICIT_CONTAINER_NAME}"
    elif [[ -n "${opt_user_value}" ]]; then
        # If --user was provided, but -n was not, use --user's value for container name suffix
        DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${opt_user_value}"
    else
        # Fallback to host's USER for container name suffix (default)
        DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}"
    fi
}

function restart_stopped_container() {
    echo "Checking for container '${DEV_CONTAINER}'..."
    if docker ps -a --format "{{.Names}}" | grep -q "^${DEV_CONTAINER}$"; then
        if docker inspect "${DEV_CONTAINER}" --format "{{.State.Running}}" | grep -q "false"; then
            echo "Container '${DEV_CONTAINER}' found but is stopped. Starting it..."
            if ! docker start "${DEV_CONTAINER}"; then
                echo "Error: Failed to start container '${DEV_CONTAINER}'." >&2
                exit 1
            fi
        else
            echo "Container '${DEV_CONTAINER}' is running."
        fi
    else
        echo "Error: Container '${DEV_CONTAINER}' not found. Please ensure it's started with start_container.sh." >&2
        exit 1
    fi
}

# --- Main Script Execution ---

xhost +local:root 1>/dev/null 2>&1 || { echo "Warning: xhost command failed. Display may not work." >&2; }

parse_arguments "$@"

restart_stopped_container # Ensures the target container is running

echo "Connecting to container '${DEV_CONTAINER}' as user '${DOCKER_USER}'..."

# Execute bash inside the container
docker exec \
    -u "${DOCKER_USER}" \
    -e HISTFILE=/apollo/.dev_bash_hist \
    -it "${DEV_CONTAINER}" \
    /bin/bash

# Cleanup xhost (execute only if docker exec was successful, or cleanup anyway)
# The `|| true` ensures the cleanup command runs even if docker exec fails.
xhost -local:root 1>/dev/null 2>&1 || true

echo "Disconnected from container."
