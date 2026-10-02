#!/usr/bin/env bash

# ----- Constants -----
DEFAULT_CONTAINER_OS="22.04"

function resolve_apollo_repo() {
    local geoloc="${GEOLOC:-cn}"
    local default_repo

    case "${geoloc}" in
        cn)
            default_repo="registry.cn-hangzhou.aliyuncs.com/wheelos/apollo"
            ;;
        us)
            default_repo="wheelos/apollo"
            ;;
        *)
            echo "Unsupported geolocation '${geoloc}'; expected cn or us." >&2
            return 1
            ;;
    esac

    APOLLO_REPO="${APOLLO_REPO:-${default_repo}}"
}

# Function: Pull Docker image, check local cache first if requested
function docker_pull() {
    local image_name="$1"

    # Check if local image exists
    if docker image inspect "${image_name}" >/dev/null 2>&1; then
        echo "Using local image '${image_name}'."
    else
        echo "Starting pull of docker image '${image_name}' ..."
        if ! docker pull "${image_name}"; then
            echo "Failed to pull docker image: '${image_name}'"
            return 1
        fi
    fi

    APOLLO_IMAGE="${image_name}"
    return 0
}

function resolve_image_name() {
    APOLLO_IMAGE="$1"
    return 0
}

# Resolve only image variants present in docker/build/docker-bake.hcl.
function determine_image() {
    local arch="$1"
    local os_ver="$2"
    local gpu="$3"
    local ensure_local="${4:-true}"

    local image_name=""

    if [[ "${gpu}" != "true" && "${gpu}" != "false" ]]; then
        echo "Unsupported GPU selection '${gpu}'; expected true or false." >&2
        return 1
    fi

    if [[ "${arch}" == "aarch64" ]]; then
        if [[ "${os_ver}" != "22.04" ]]; then
            echo "Unsupported ARM64 container Ubuntu version '${os_ver}'; only 22.04 is available." >&2
            return 1
        fi
        if [[ "${gpu}" == "true" ]]; then
            if [[ ! -r /etc/nv_tegra_release ]]; then
                echo "ARM64 GPU images are only supported on Jetson Orin with JetPack 6.2.1 / L4T 36.4.3." >&2
                return 1
            fi

            local l4t_version
            l4t_version="$(sed -nE 's/.*R([0-9]+) \(release\), REVISION: ([0-9]+)\.([0-9]+).*/\1.\2.\3/p' /etc/nv_tegra_release)"
            if [[ "${l4t_version}" != "36.4.3" ]]; then
                echo "Unsupported Jetson L4T version '${l4t_version:-unknown}'; this image targets L4T 36.4.3." >&2
                return 1
            fi
            image_name="${APOLLO_REPO}:dev-aarch64-orin-jp6.2.1-l4t36.4.3-gpu"
        else
            image_name="${APOLLO_REPO}:dev-aarch64-22.04-cpu"
        fi
    elif [[ "${arch}" == "x86_64" ]]; then
        if [[ "$gpu" == "true" ]]; then
            if [[ "${os_ver}" != "22.04" ]]; then
                echo "Unsupported x86_64 GPU container Ubuntu version '${os_ver}'; the CUDA image uses 22.04." >&2
                return 1
            fi
            image_name="${APOLLO_REPO}:dev-x86_64-22.04-gpu"
        else
            if [[ "${os_ver}" != "20.04" && "${os_ver}" != "22.04" ]]; then
                echo "Unsupported x86_64 CPU image Ubuntu version '${os_ver}'; supported versions are 20.04 and 22.04." >&2
                return 1
            fi
            image_name="${APOLLO_REPO}:dev-x86_64-${os_ver}-cpu"
        fi
    else
        echo "Unsupported container architecture '${arch}'; supported architectures are x86_64 and aarch64." >&2
        return 1
    fi

    if [[ "${ensure_local}" == "true" ]]; then
        if ! docker_pull "${image_name}"; then
            echo "Failed to determine image."
            return 1
        fi
    else
        if ! resolve_image_name "${image_name}"; then
            echo "Failed to resolve image name."
            return 1
        fi
    fi
}

# Main function to call from whl.sh
function select_container() {
    if [[ $# -lt 3 || $# -gt 4 ]]; then
        echo "Usage: select_container <arch> <container-ubuntu-version> <gpu:true|false> [ensure-local:true|false]" >&2
        return 2
    fi

    local arch="$1"
    local os_ver="$2"
    local gpu="$3"
    local ensure_local="${4:-true}"

    if [[ "${ensure_local}" != "true" && "${ensure_local}" != "false" ]]; then
        echo "Unsupported ensure-local value '${ensure_local}'; expected true or false." >&2
        return 2
    fi

    resolve_apollo_repo || return
    determine_image "${arch}" "${os_ver}" "${gpu}" "${ensure_local}"
}

# Allow invocation of select_container from the command line for testing
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    select_container "$@"
fi
