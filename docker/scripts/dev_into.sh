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

set -euo pipefail

# --- Constants ---
DEV_CONTAINER_PREFIX='apollo_dev_'
DEV_INSIDE="in-dev-docker"

# --- Global Variables ---
DOCKER_USER="${USER}"
DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}"
USER_EXPLICIT_CONTAINER_NAME=""
#
# === [MODIFICATION 1]: Add a new global variable for the command ===
#
NON_INTERACTIVE_CMD=""

# --- Helper Functions ---
function show_usage() {
    cat <<EOF
Usage: $0 [options]
Connects to a running Apollo development Docker container.

MODES:
  Interactive (default): If no command is provided, starts an interactive bash session.
  Non-Interactive:       If a command is provided, executes it inside the container and exits.

OPTIONS:
    -h, --help                  Display this help and exit.
    -n, --name <container_name> Specify the *full* name of the container to connect to.
    -c, --command <"command">   Specify a command to execute in non-interactive mode.
    --user <username>           Specify the username for the container session.
                                Influences default container name if -n is not used.
EOF
}

function parse_arguments() {
    local opt_n_value=""
    local opt_user_value=""

    #
    # === [MODIFICATION 2]: Update argument parsing logic ===
    # Using a while loop to process options, allowing a command to be passed at the end.
    # The previous logic would fail if options and commands were mixed.
    #
    while [[ $# -gt 0 ]]; do
        case "$1" in
            -h | --help)
                show_usage
                exit 0
                ;;
            -n | --name)
                shift
                if [[ -z "${1-}" ]]; then
                    echo "Error: Missing argument for -n/--name." >&2; exit 1
                fi
                USER_EXPLICIT_CONTAINER_NAME="$1"
                shift
                ;;
            --user)
                shift
                if [[ -z "${1-}" ]]; then
                    echo "Error: Missing argument for --user." >&2; exit 1
                fi
                DOCKER_USER="$1"
                shift
                ;;
            -c | --command)
                shift
                if [[ -z "${1-}" ]]; then
                    echo "Error: Missing argument for -c/--command." >&2; exit 1
                fi
                NON_INTERACTIVE_CMD="$1"
                shift
                ;;
            *)
                # If an unknown option is found, assume it's the start of the command
                NON_INTERACTIVE_CMD="$*"
                break # Exit the loop, the rest of the arguments are the command
                ;;
        esac
    done


    # --- Determine the final DEV_CONTAINER name ---
    if [[ -n "${USER_EXPLICIT_CONTAINER_NAME}" ]]; then
        DEV_CONTAINER="${USER_EXPLICIT_CONTAINER_NAME}"
    elif [[ "${DOCKER_USER}" != "${USER}" ]]; then
        # If --user was provided AND it's different from the host user, use it for the container name
        DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${DOCKER_USER}"
    fi # Otherwise, the default DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}" is used
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

parse_arguments "$@"

restart_stopped_container # Ensures the target container is running


#
# === [MODIFICATION 3]: Add the core logic to switch between modes ===
#
if [[ -n "${NON_INTERACTIVE_CMD}" ]]; then
    # === NON-INTERACTIVE MODE ===
    echo "Executing command in container '${DEV_CONTAINER}':"
    echo "  ${NON_INTERACTIVE_CMD}"

    # Use 'docker exec' without '-it'. The command is passed as arguments to /bin/bash -c
    docker exec \
        -u "${DOCKER_USER}" \
        "${DEV_CONTAINER}" \
        /bin/bash -c "${NON_INTERACTIVE_CMD}"

    echo "Command execution finished."

else
    # === INTERACTIVE MODE ===
    echo "Connecting to container '${DEV_CONTAINER}' as user '${DOCKER_USER}'..."

    # Add display access for GUI applications
    xhost +local:root &>/dev/null || true

    # Use 'docker exec' with '-it' for an interactive terminal.
    docker exec \
        -u "${DOCKER_USER}" \
        -e HISTFILE=/apollo/.dev_bash_hist \
        -it \
        "${DEV_CONTAINER}" \
        /bin/bash

    # Clean up display access after session ends
    xhost -local:root &>/dev/null || true
    echo "Disconnected from container."
fi
