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
#

# =============================================================================
# MINIMAL Apollo Development Container Launcher Script (Host Side)
#
# This script is a bare-bones launcher. It prepares the host environment,
# handles basic arguments, determines the correct Docker image, pulls it,
# mounts essential volumes (code, config, system), and starts the container.
#
# ALL model/tool installation, model/map data download, and container-specific
# setup MUST be handled *INSIDE* the container by separate scripts executed
# by the user *after* logging in.
# =============================================================================

# Exit immediately if a command exits with a non-zero status.
set -e
# Treat unset variables as an error when substituting.
set -u
# Exit if any command in a pipeline fails.
set -o pipefail

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# Assuming docker_base.sh provides helper functions like info, warning, error,
# ok, check_agreement, remove_container_if_exists, determine_gpu_use_host,
# geo_specific_config, postrun_start_user, optarg_check_for_opt, setup_device,
# APOLLO_ROOT_DIR, APOLLO_CONFIG_HOME etc.
# Also assumes DOCKER_RUN_CMD is defined in docker_base.sh (usually 'docker run')
source "${CURR_DIR}/docker_base.sh"

# --- Constants: Directories and Container Naming ---
# CACHE_ROOT_DIR is still relevant for general Apollo caching, not just volumes.
CACHE_ROOT_DIR="${APOLLO_ROOT_DIR}/.cache"
BAZEL_CACHE_DIR="/var/cache/bazel/repo_cache"

DEV_CONTAINER_PREFIX='apollo_dev_'
DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${USER}"
DEV_INSIDE="in-dev-docker" # Hostname inside the container

# Ensure cache dir exists early
[ -d "${CACHE_ROOT_DIR}" ] || mkdir -p "${CACHE_ROOT_DIR}"

# --- Constants: Host Environment ---
SUPPORTED_ARCHS=(x86_64 aarch64)
TARGET_ARCH="$(uname -m)"
TIMEZONE_CN=(
    "+0800"
    "+0800 CST"
    "Time zone: Asia/Shanghai (CST, +0800)"
)

# --- Constants: Container Resources ---
# Resource limits (can be overridden by environment variables)
DOCKER_CPUS="${DOCKER_CPUS:-6}"
DOCKER_MEMORY="${DOCKER_MEMORY:-8g}"

# --- Constants: Image Name and Versions ---
# Default development image versions based on architecture and distribution
# can be overridden by environment variables.
DOCKER_IMAGE_REPO=${DOCKER_IMAGE_REPO:="apolloauto/apollo"}
DOCKER_IMAGE_TAG_X86_64=${DOCKER_IMAGE_TAG_X86_64:="dev-x86_64-18.04-20221124_1708"}
DOCKER_IMAGE_TAG_X86_64_TESTING=${DOCKER_IMAGE_TAG_X86_64_TESTING:="dev-x86_64-18.04-testing-20210112_0008"}
DOCKER_IMAGE_TAG_AARCH64=${DOCKER_IMAGE_TAG_AARCH64:="dev-aarch64-20.04-20231024_1054"}

# --- Script Global Variables (Modified by arguments/logic) ---
DOCKER_IMAGE_TAG=${DOCKER_IMAGE_TAG:=""} # Default empty, will be set by arguments or overrided by environment variables
GEOLOC=""            # Default: auto-detect ('us', 'cn', 'none')
SHM_SIZE="2G"        # Default shared memory size
USE_LOCAL_IMAGE=1    # Flag to use local image (0 or 1)
CUSTOM_DIST="stable" # Apollo distribution (stable/testing)
USER_AGREED="no"     # Flag for Apollo License Agreement ('yes' or 'no')

# --- Helper Functions ---

# Display script usage
function show_usage() {
    cat <<EOF
Usage: $0 [options] ...
OPTIONS:
    -h, --help                  Display this help and exit.
    -g, --geo <us|cn|none>      Pull docker image from geolocation specific registry mirror.
    -l, --local                 Use local docker image if available, skip pulling from remote.
    -t, --tag <TAG>             Specify docker image with tag <TAG> to start.
    -d, --dist <stable|testing> Specify Apollo distribution (stable/testing). Default: ${CUSTOM_DIST}.
    -n, --name <envname>        Specify the name of the docker container, default is current user name.
    --shm-size <bytes>          Size of /dev/shm. Passed directly to "docker run". Default: ${SHM_SIZE}.
    -y                          Agree to Apollo License Agreement non-interactively.
    stop                        Stop all running Apollo containers for the current user.
EOF
}

# Parse command line arguments
function parse_arguments() {
    local custom_version_arg=""
    local custom_dist_arg=""
    local shm_size_arg=""
    local geo_arg=""
    local user_agreed_arg="no"

    while [ $# -gt 0 ]; do
        local opt="$1"
        shift
        case "${opt}" in
            -t | --tag)
                if [ -n "${custom_version_arg}" ]; then
                    warning "Multiple option ${opt} specified, only the last one will take effect."
                fi
                custom_version_arg="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_version_arg}"
                ;;

            -d | --dist)
                custom_dist_arg="$1"
                shift
                optarg_check_for_opt "${opt}" "${custom_dist_arg}"
                ;;

            -h | --help)
                show_usage
                exit 0 # Use 0 for help
                ;;

            -g | --geo)
                geo_arg="$1"
                shift
                optarg_check_for_opt "${opt}" "${geo_arg}"
                ;;

            -l | --local)
                USE_LOCAL_IMAGE=1
                ;;

            --user)
                export CUSTOM_USER="$1"
                shift
                ;;

            --uid)
                export CUSTOM_UID="$1"
                shift
                ;;

            --group)
                export CUSTOM_GROUP="$1"
                shift
                ;;
            --gid)
                export CUSTOM_GID="$1"
                shift
                ;;

            -n | --name)
                DEV_CONTAINER="${DEV_CONTAINER_PREFIX}${1}"
                shift
                ;;

            --shm-size)
                shm_size_arg="$1"
                shift
                optarg_check_for_opt "${opt}" "${shm_size_arg}"
                ;;

            -y)
                user_agreed_arg="yes"
                ;;

            stop)
                info "Stopping all Apollo containers created by ${USER}..."
                # Assuming remove_container_if_exists can stop running containers
                remove_container_if_exists "${DEV_CONTAINER}" -f # Stop and remove force
                exit 0
                ;;

            *)
                warning "Unknown option: ${opt}"
                show_usage
                exit 2
                ;;
        esac
    done # End while loop

    # Assign parsed values to global variables
    [[ -n "${geo_arg}" ]] && GEOLOC="${geo_arg}"
    [[ -n "${custom_version_arg}" ]] && DOCKER_IMAGE_TAG="${custom_version_arg}"
    [[ -n "${custom_dist_arg}" ]] && CUSTOM_DIST="${custom_dist_arg}"
    [[ -n "${shm_size_arg}" ]] && SHM_SIZE="${shm_size_arg}"
    USER_AGREED="${user_agreed_arg}"
}

image_contains_registry() {
    image="${1}"
    if [[ "${image}" =~ ^(([a-zA-Z0-9-]+\.)+[a-zA-Z0-9-]+|localhost|([0-9]{1,3}\.){3}[0-9]{1,3})(:[0-9]+)?/ ]]; then
        echo "1" # Image contains a registry
    fi
    echo "0" # Image does not contain a registry
}

# Determine the final Docker image tag based on architecture, distribution, or user override
function determine_dev_image() {
    local custom_version="$1"
    local version=""

    if [[ -n "${custom_version}" ]]; then
        version="${custom_version}"
    else
        case "${TARGET_ARCH}" in
            x86_64)
                if [[ "${CUSTOM_DIST}" == "testing" ]]; then
                    version="${DOCKER_IMAGE_TAG_X86_64_TESTING}"
                else
                    version="${DOCKER_IMAGE_TAG_X86_64}"
                fi
                ;;
            aarch64)
                version="${DOCKER_IMAGE_TAG_AARCH64}"
                ;;
            *)
                # This case should ideally be caught by check_target_arch earlier, but keep for robustness
                error "Unsupported target architecture: ${TARGET_ARCH}. Should not reach here."
                exit 3
                ;;
        esac
    fi
    DEV_IMAGE="${DOCKER_IMAGE_REPO}:${version}"
    info "Determined development image: ${DEV_IMAGE}"
}

# Check if host OS is supported
function check_host_environment() {
    if [[ "$(uname -s)" != "Linux" ]]; then
        warning "Running Apollo dev container on $(uname -s) is UNTESTED, exiting..."
        exit 1
    fi
    info "Host environment check passed."
}

# Check if host architecture is supported
function check_target_arch() {
    local arch_supported=0
    for ent in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${ent}" == "${TARGET_ARCH}" ]]; then
            arch_supported=1
            break
        fi
    done

    if [[ "${arch_supported}" -eq 0 ]]; then
        error "Unsupported target architecture: ${TARGET_ARCH}."
        error "Supported architectures: ${SUPPORTED_ARCHS[*]}"
        exit 1
    fi
    info "Target architecture check passed (${TARGET_ARCH})."
}

# Auto-detect China timezone for geo location if GEOLOC is not explicitly set
function determine_timezone_cn() {
    # https://en.wikipedia.org/wiki/List_of_tz_database_time_zones
    local time_zone=
    if command -v timedatectl 2>&1 >/dev/null; then
        # Use timedatectl if available (systemd based systems)
        # Use xargs to trim whitespace, echo "" if grep fails
        time_zone=$(timedatectl | grep "Time zone" | xargs || echo "")
    else
        # Fallback to date command for other systems
        time_zone=$(date +%z)
    fi

    if [[ -z "${GEOLOC}" ]]; then # Only auto-detect if GEOLOC wasn't set by argument
        for tz in "${TIMEZONE_CN[@]}"; do
            if [[ "${time_zone}" == "${tz}" ]]; then
                GEOLOC="cn"
                info "Detected China timezone. Setting GEOLOC=cn for potential mirror usage."
                return 0
            fi
        done
        info "Did not detect China timezone. GEOLOC remains unset or user-specified."
    fi
}

# Prepare standard host volumes to mount into the container.
# This excludes any map or model specific data volumes.
# Uses a return variable name passed as argument to set the result string.
function prepare_docker_volumes() {
    local __retval="$1" # Name of the variable to set with volume strings

    local volumes=""

    # Mount the Apollo root directory (codebase)
    volumes+="-v ${APOLLO_ROOT_DIR}:/apollo"

    # Mount Apollo config directory (ensure it exists)
    [ -d "${APOLLO_CONFIG_HOME}" ] || mkdir -p "${APOLLO_CONFIG_HOME}"
    volumes+=" -v ${APOLLO_CONFIG_HOME}:${APOLLO_CONFIG_HOME}"

    # Mount apollo-teleop if the directory exists next to apollo
    local teleop_dir="${APOLLO_ROOT_DIR}/../apollo-teleop"
    if [ -d "${teleop_dir}" ]; then
        volumes+=" -v ${teleop_dir}:/apollo/modules/teleop"
    else
        info "apollo-teleop directory not found at ${teleop_dir}. Skipping mount."
    fi

    # Mount apollo-tools if the directory exists next to apollo
    local apollo_tools_dir="${APOLLO_ROOT_DIR}/../apollo-tools"
    if [ -d "${apollo_tools_dir}" ]; then
        volumes+=" -v ${apollo_tools_dir}:/tools"
    else
        info "apollo-tools directory not found at ${apollo_tools_dir}. Skipping mount."
    fi

    # Mount /dev directly. Needed for device access (GPU, sensors, etc.).
    volumes+=" -v /dev:/dev"

    # bazel cache
    if [[ -d "${BAZEL_CACHE_DIR}" ]]; then
        info "✅ Cache directory exists: ${BAZEL_CACHE_DIR}"
        # Directory exists – add mount volume option
        volumes+=" -v ${BAZEL_CACHE_DIR}:${BAZEL_CACHE_DIR}"
    else
        # Directory does not exist – inform user to create it with sudo
        warning "⚠️ Cache directory does NOT exist: ${BAZEL_CACHE_DIR}"
        warning "Usually it is automatically created by docker."
        warning "If not, please create it with sudo manually:"
        warning "  sudo mkdir -p \"${BAZEL_CACHE_DIR}\""
        warning "Then re-run this script"
        volumes+=" -v ${BAZEL_CACHE_DIR}:${BAZEL_CACHE_DIR}"
    fi

    # Optional: Mount NVIDIA specific directories for AARCH64 Jetson
    # local tegra_dir="/usr/lib/aarch64-linux-gnu/tegra"
    # if [[ "${TARGET_ARCH}" == "aarch64" && -d "${tegra_dir}" ]]; then
    #     volumes+=" -v ${tegra_dir}:${tegra_dir}:ro"
    # fi

    # Standard mounts required for typical X/GUI/system integration
    volumes+=" -v /media:/media"                  # Removable media
    volumes+=" -v /tmp/.X11-unix:/tmp/.X11-unix:rw" # X server access
    volumes+=" -v /etc/localtime:/etc/localtime:ro" # Sync timezone
    volumes+=" -v /usr/src:/usr/src"              # Mount kernel sources (often needed by drivers/modules)
    volumes+=" -v /lib/modules:/lib/modules"      # Mount kernel modules (often needed by drivers)
    volumes+=" -v /dev/null:/dev/raw1394"         # Workaround for some older libraries

    # Clean up any potential multiple spaces generated
    volumes="$(echo "${volumes}" | tr -s " ")"

    # Set the return variable with the collected volume mount strings
    eval "${__retval}='${volumes}'"
    info "Prepared standard docker volumes."
}

# Pull Docker image, check local cache first if requested.
# Includes geo-specific registry logic.
function docker_pull() {
    local img_tag="$1"
    local full_img="${img_tag}"

    # Add geo-specific registry if configured by geo_specific_config
    if [[ -n "${GEO_REGISTRY:-}" ]] && [[ "$(image_contains_registry "${img_tag}")" == "0" ]]; then
        full_img="${GEO_REGISTRY}/${img_tag}"
        info "Using geo-specific registry: ${GEO_REGISTRY}"
    else
        info "Using default registry for image: ${img_tag}"
    fi

    if [[ "${USE_LOCAL_IMAGE}" -gt 0 ]]; then
        info "Local image mode enabled. Checking for local image '${full_img}'."
        # Using docker image inspect is a robust way to check for image existence
        if docker image inspect "${full_img}" >/dev/null 2>&1; then
            info "Local image '${full_img}' found. Using it."
            return 0 # Success
        else
            warning "Local image '${full_img}' not found. Falling back to pulling from remote."
        fi
    fi

    info "Starting pull of docker image '${full_img}' ..."
    # Add retry logic for pulling if needed
    if ! docker pull "${full_img}"; then
        error "Failed to pull docker image: '${full_img}'"
        return 1 # Failure
    fi
    info "Docker image '${full_img}' pulled successfully."
    return 0 # Success
}

# --- Main Script Execution ---

function main() {
    # --- Phase 1: Environment and Arguments ---
    check_host_environment
    check_target_arch

    parse_arguments "$@" # Parses arguments and sets global variables

    if [[ "${USER_AGREED}" != "yes" ]]; then
        # check_agreement is assumed to be provided by docker_base.sh
        check_agreement
    fi

    determine_dev_image "${DOCKER_IMAGE_TAG}" # Sets DEV_IMAGE

    determine_timezone_cn # Sets GEOLOC if not already set and timezone is CN
    # geo_specific_config is assumed to be provided by docker_base.sh
    # It might set GEO_REGISTRY based on GEOLOC
    geo_specific_config "${GEOLOC}"

    # --- Phase 2: Docker Image and Container Preparation ---
    # docker_pull function now handles the full image name with registry and local check
    if ! docker_pull "${DEV_IMAGE}"; then # Pass DEV_IMAGE (tag only), docker_pull adds registry
        error "Failed prerequisite: Docker image pull failed. Exiting."
        exit 1
    fi

    info "Removing existing Apollo Development container '${DEV_CONTAINER}' (if any)..."
    # remove_container_if_exists is assumed to be provided by docker_base.sh
    # Use -f to force stop and remove if running
    remove_container_if_exists "${DEV_CONTAINER}" -f

    info "Determining whether host GPU is available..."
    # determine_gpu_use_host is assumed to be provided by docker_base.sh
    # It should set USE_GPU_HOST ('yes' or 'no')
    determine_gpu_use_host
    info "USE_GPU_HOST: ${USE_GPU_HOST}"

    # Prepare standard host volumes to mount (code, config, /dev, etc.).
    # This version does NOT include any map or model specific data mounts.
    local standard_volumes=""
    prepare_docker_volumes "standard_volumes"

    # --- Phase 3: Docker Run Command Construction ---
    # Define the arrays for docker run options
    local run_opts=(
        -itd     # Interactive, TTY, Detached (run in background)
        --privileged # Grant extended privileges (often needed for device access)
        --name "${DEV_CONTAINER}"
        --net host # Use host network
        --pid=host # Use host process namespace (allows host process inspection/signals)
        --shm-size "${SHM_SIZE}"
        -w /apollo             # Set working directory inside container
        --hostname "${DEV_INSIDE}" # Set hostname inside container for easy identification
        --label "owner=${USER}" # Label container for easy filtering/management
    )

    # Add GPU options based on detection
    local gpu_opts=()
    if [[ "${USE_GPU_HOST}" -eq 1 ]]; then
        info "Adding GPU options for NVIDIA."
        # Using environment variables for compatibility, modern approach uses --gpus all
        # If docker/nvidia-container-toolkit version supports it, replace env vars with:
        # run_opts+=(--gpus all)
        gpu_opts=(
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=compute,video,graphics,utility
            # -e DOCKER_HOST_GPU=1 # Custom env var if container needs to know GPU is available
        )
    else
        warning "GPU not detected or available on host. Skipping GPU options."
    fi

    local local_host="$(hostname)"
    local display="${DISPLAY:-:0}" # Default DISPLAY if not set
    local user="${CUSTOM_USER-$USER}"
    local uid="${CUSTOM_UID-$(id -u)}"
    local group="${CUSTOM_GROUP-$(id -g -n)}"
    local gid="${CUSTOM_GID-$(id -g)}"

    # Define environment variables to pass into the container
    # NOTE: This version does NOT pass any environment variables related to
    # specific models, tools, or maps to be installed/downloaded.
    local env_opts=(
        -e DISPLAY="${display}"
        -e DOCKER_USER="${user}"
        -e USER="${user}"                        # Pass host username
        -e DOCKER_USER_ID="${uid}"               # Pass host user ID
        -e DOCKER_GRP="${group}"                 # Pass host group name
        -e DOCKER_GRP_ID="${gid}"                # Pass host group ID
        -e DOCKER_IMG="${DEV_IMAGE}"             # Original image name (tag only)
        -e PYTHON_VERSION="${PYTHON_VERSION:-3}" # Assume Python 3 by default
        -e USE_GPU_HOST="${USE_GPU_HOST}"        # Pass GPU availability status
        -e CROSS_PLATFORM="${CROSS_PLATFORM_FLAG:-}" # Pass cross-platform build flag if applicable
        # Any other environment variables needed for the base container environment
    )

    # Define host entries to add to the container's /etc/hosts
    local host_opts=(
        --add-host "${DEV_INSIDE}:127.0.0.1"
        --add-host "${local_host}:127.0.0.1"
    )

    # Combine all volume mount strings (standard mounts only)
    local volume_opts=(
        ${standard_volumes} # Only standard code, config, system mounts
    )

    # Add resource limits (cpus, memory)
    local resource_opts=(
        --cpus="${DOCKER_CPUS}"
        --memory="${DOCKER_MEMORY}"
    )

    # --- Phase 4: Run the Container ---
    local full_image_name="${DEV_IMAGE}" # Start with base image tag
    if [[ -n "${GEO_REGISTRY:-}" ]] && [[ "$(image_contains_registry "${full_image_name}")" == "0" ]]; then
        full_image_name="${GEO_REGISTRY}/${full_image_name}"
    fi

    info "Starting Docker container \"${DEV_CONTAINER}\" from image: ${full_image_name} ..."
    info "Using SHM_SIZE=${SHM_SIZE}, CPUS=${DOCKER_CPUS}, MEMORY=${DOCKER_MEMORY}"
    # Be cautious printing all volumes if paths are sensitive
    # info "Mounted volumes: ${standard_volumes}"

    # Print the command being executed for debugging before running
    set -x
    # Execute the docker run command. The final argument is the image name.
    # The command run inside the container will be the image's default ENTRYPOINT/CMD.
    ${DOCKER_RUN_CMD} \
        "${run_opts[@]}" \
        "${resource_opts[@]}" \
        "${gpu_opts[@]}" \
        "${env_opts[@]}" \
        "${host_opts[@]}" \
        "${volume_opts[@]}" \
        "${full_image_name}"
    # No explicit command here, relying on the image's default ENTRYPOINT/CMD (likely /bin/bash)

    local docker_run_exit_code=$?
    set +x # Stop printing commands after docker run finishes

    if [ "${docker_run_exit_code}" -ne 0 ]; then
        error "Failed to start docker container \"${DEV_CONTAINER}\"."
        error "Docker run command exited with code: ${docker_run_exit_code}"
        info "For debugging, try running the docker command printed above manually."
        exit 1
    fi

    # postrun_start_user is assumed to be provided by docker_base.sh
    # This function might attach to the container and run initial commands,
    # such as switching to the correct user or running a *very basic* setup script.
    # It should NOT attempt to install/download models/maps/tools in this minimal version.
    postrun_start_user "${DEV_CONTAINER}" # Pass container name, no specific setup args now

    # TODO(All): a better rootless solution for the bazel cache directory
    # ensure read-write permissions of the global bazel cache dir
    execute_in_container "${DEV_CONTAINER}" "chmod a+rwx ${BAZEL_CACHE_DIR}"

    # --- Phase 5: Completion ---
    ok "Congratulations! Apollo Dev Environment container '${DEV_CONTAINER}' is running."
    ok "To login into the container, please run:"
    ok "  bash docker/scripts/dev_into.sh"

    warning "--- Next Steps (Run INSIDE the Container) ---"
    info "This host script ONLY launched the container."
    info "ALL further environment setup (installing tools, downloading models, downloading map data) MUST be done *INSIDE* the container."
    info "After logging in, locate and run the necessary setup scripts within the /apollo directory or as provided by your Apollo distribution."
    info "You will need to handle persistent storage for models/maps yourself (e.g., by manually mounting volumes/bind mounts to specific data paths like /opt/apollo/data/models and /apollo/modules/map/data when starting the container, or by configuring internal download scripts to use specific locations)."
    ok "Enjoy!"
}

main "$@"
