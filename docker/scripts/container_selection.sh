#!/usr/bin/env bash

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

    RESOLVED_APOLLO_REPO="${APOLLO_REPO:-${default_repo}}"
}

function docker_pull() {
    local image_name="$1"

    if docker image inspect "${image_name}" >/dev/null 2>&1; then
        echo "Using local image '${image_name}'."
    else
        echo "Starting pull of docker image '${image_name}' ..."
        if ! docker pull "${image_name}"; then
            echo "Failed to pull docker image: '${image_name}'" >&2
            return 1
        fi
    fi

    APOLLO_IMAGE="${image_name}"
}

function resolve_image_name() {
    APOLLO_IMAGE="$1"
}

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
    if [[ "${os_ver}" != "${DEFAULT_CONTAINER_OS}" ]]; then
        echo "Unsupported container Ubuntu version '${os_ver}'; only ${DEFAULT_CONTAINER_OS} is available." >&2
        return 1
    fi

    case "${arch}.${gpu}" in
        x86_64.false)
            image_name="${RESOLVED_APOLLO_REPO}:dev-x86_64-22.04-cpu"
            ;;
        x86_64.true)
            image_name="${RESOLVED_APOLLO_REPO}:dev-x86_64-22.04-gpu"
            ;;
        aarch64.true)
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
            image_name="${RESOLVED_APOLLO_REPO}:dev-orin-jp6.2.1"
            ;;
        aarch64.false)
            echo "ARM CPU images are unsupported; only Orin GPU is supported." >&2
            return 1
            ;;
        *)
            echo "Unsupported host architecture/GPU combination: ${arch}/${gpu}" >&2
            return 1
            ;;
    esac

    if [[ "${ensure_local}" == "true" ]]; then
        docker_pull "${image_name}"
    else
        resolve_image_name "${image_name}"
    fi
}

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

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    select_container "$@"
fi
