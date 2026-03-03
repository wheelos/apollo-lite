#!/usr/bin/env bash

# ----- Constants -----
DOCKER_IMAGE_REPO=${DOCKER_IMAGE_REPO:="wheelos/apollo"}
GEOLOC=${GEOLOC:="cn"}
GEO_REGISTRY=""

function geo_specific_config() {
    local geo="$1"
    if [[ -z "${geo}" ]]; then
        echo "Use default GeoLocation settings"
        return
    fi
    echo "Setup geolocation specific configurations for ${geo}"

    if [[ "${geo}" == "cn" ]]; then
        echo "GeoLocation settings for Mainland China"
        GEO_REGISTRY="registry.cn-hangzhou.aliyuncs.com"
    else
        echo "GeoLocation settings for ${geo} is not ready, fallback to default"
    fi
}

# Function: Pull Docker image, check local cache first if requested
function docker_pull() {
    local base_image_name="$1"
    local __final_image_var_name="$2"

    # Determine the pull candidate based on geolocation
    local pull_candidate_image_name="${base_image_name}"
    if [[ -n "${GEO_REGISTRY}" ]]; then
        pull_candidate_image_name="${GEO_REGISTRY}/${base_image_name}"
    fi

    # Check if local image exists
    if docker image inspect "${pull_candidate_image_name}" >/dev/null 2>&1; then
        echo "Using local image '${pull_candidate_image_name}'."
    else
        echo "Starting pull of docker image '${pull_candidate_image_name}' ..."
        if ! docker pull "${pull_candidate_image_name}"; then
            echo "Failed to pull docker image: '${pull_candidate_image_name}'"
            return 1
        fi
    fi

    eval "${__final_image_var_name}='${pull_candidate_image_name}'"  # Store the final image name
    return 0
}

# Function: Determine image based on architecture and GPU usage
function determine_image() {
    local arch="$1"     # e.g., x86 or arm
    local os_ver="$2"   # e.g., 20.04
    local gpu="$3"      # e.g., true (for GPU) or false (for CPU)

    # Construct the base image name using the specified format
    local base_image_name="dev-${arch}-${os_ver}"
    local image_name=""

    # Append GPU or CPU suffix based on the input
    if [[ "$gpu" == "true" ]]; then
        image_name="${DOCKER_IMAGE_REPO}:${base_image_name}-gpu"
    else
        image_name="${DOCKER_IMAGE_REPO}:${base_image_name}-cpu"
    fi

    # Pull the Docker image or use the existing one
    if ! docker_pull "${image_name}" "APOLLO_IMAGE"; then
        echo "Failed to determine image."
        exit 1
    fi
}

# Main function to call from whl.sh
function select_container() {
    local arch="$1"
    local os_ver="$2"
    local gpu="$3"

    geo_specific_config "${GEOLOC}"
    determine_image "${arch}" "${os_ver}" "${gpu}"
}

# Allow invocation of select_container from the command line for testing
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    select_container "$@"
fi
