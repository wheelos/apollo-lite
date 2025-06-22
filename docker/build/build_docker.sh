#!/usr/bin/env bash
set -euo pipefail

TAB="    " # 4 Spaces

APOLLO_REPO="apolloauto/apollo"
UBUNTU_LTS="20.04"

declare -A CUDA_LITE_VERSIONS
declare -A CUDNN_VERSIONS
declare -A TENSORRT_VERSIONS
# x86_64
CUDA_LITE_VERSIONS["x86_64"]="11.8.0"
CUDNN_VERSIONS["x86_64"]="8"
TENSORRT_VERSIONS["x86_64"]="8.6.1.6"
# aarch64
CUDA_LITE_VERSIONS["aarch64"]="10.2"
CUDNN_VERSIONS["aarch64"]="8.0.0.180"
TENSORRT_VERSIONS["aarch64"]="7.1.3"

SUPPORTED_ARCHS=( x86_64 aarch64 )
SUPPORTED_STAGES=( base cyber dev runtime )

HOST_ARCH="$(uname -m)"
INSTALL_MODE="download"
TARGET_GEOLOC="us"

TARGET_ARCH=
TARGET_STAGE=

DOCKERFILE=
PREV_IMAGE_TIMESTAMP=
USE_CACHE=1
DRY_RUN_ONLY=0

CUDA_LITE=
CUDNN_VERSION=
TENSORRT_VERSION=

IMAGE_IN=
IMAGE_OUT=
DEV_IMAGE_IN=

function check_experimental_docker() {
    local daemon_cfg="/etc/docker/daemon.json"
    local enabled="$(docker version -f '{{.Server.Experimental}}')"
    if [ "${enabled}" != "true" ]; then
        echo "Experimental features should be enabled to run Apollo docker build."
        echo "Please perform the following two steps to have it enabled:"
        echo "  1) Add '\"experimental\": true' to '${daemon_cfg}'"
        echo "     Example:"
        echo "       {"
        echo "           \"experimental\": true "
        echo "       }"
        echo "  2) Restart docker daemon. E.g., 'sudo systemctl restart docker'"
        exit 1
    fi
}

function cpu_arch_support_check() {
    local arch="$1"
    for entry in "${SUPPORTED_ARCHS[@]}"; do
        if [[ "${entry}" == "${arch}" ]]; then return; fi
    done
    echo "Unsupported CPU architecture: ${arch}. Exiting..."
    exit 1
}

function build_stage_check() {
    local stage="$1"
    for entry in "${SUPPORTED_STAGES[@]}"; do
        if [[ "${entry}" == "${stage}" ]]; then
            return
        fi
    done
    echo "Unsupported build stage: ${stage}. Exiting..."
    exit 1
}

function determine_target_arch_and_stage() {
    local dockerfile_base="$(basename "$1")"
    if [[ ! "${dockerfile_base}" =~ ^([a-zA-Z0-9_]+)\.([a-z0-9_]+)\.dockerfile$ ]]; then
        echo "Error: Expected Dockerfile name format '[prefix_]<target>.<arch>.dockerfile'" >&2
        echo "Got '${dockerfile_base}'. Exiting..." >&2
        exit 1
    fi

    local stage="${BASH_REMATCH[1]##*_}"
    local arch="${BASH_REMATCH[2]}"

    cpu_arch_support_check "${arch}"
    TARGET_ARCH="${arch}"

    if [[ "${TARGET_ARCH}" != "${HOST_ARCH}" ]]; then
        echo "[WARNING] HOST_ARCH(${HOST_ARCH}) != TARGET_ARCH(${TARGET_ARCH}) detected."
        echo "[WARNING] Make sure you have executed: docker run --rm --privileged multiarch/qemu-user-static --reset -p yes"
    fi
    build_stage_check "${stage}"
    TARGET_STAGE="${stage}"
}

function determine_cuda_versions() {
    local arch="$1"
    if [[ -n "${CUDA_LITE_VERSIONS[${arch}]:-}" ]]; then
        CUDA_LITE="${CUDA_LITE_VERSIONS[${arch}]}"
        CUDNN_VERSION="${CUDNN_VERSIONS[${arch}]}"
        TENSORRT_VERSION="${TENSORRT_VERSIONS[${arch}]}"
    else
        echo "Error: Unsupported arch for CUDA/CuDNN/TensorRT version picking: ${arch}" >&2
        exit 1
    fi
}

function determine_prev_image_timestamp() {
    if [[ -n "${PREV_IMAGE_TIMESTAMP}" ]]; then return; fi

    local arch="$1"
    local stage="$2"

    PREV_IMAGE_TIMESTAMP=""
}

function determine_images_in_out_name() {
    local arch="$1"
    local stage="$2"
    local timestamp="$3"

    local cudnn_ver="${CUDNN_VERSION%%.*}"
    local trt_ver="${TENSORRT_VERSION%%.*}"

    local base_image="${APOLLO_REPO}:cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}-${arch}"
    if [[ "${stage}" == "base" ]]; then
        IMAGE_IN="nvidia/cuda:${CUDA_LITE}-cudnn${cudnn_ver}-devel-ubuntu${UBUNTU_LTS}"
        IMAGE_OUT="${base_image}-${timestamp}"
    elif [[ "${stage}" == "cyber" ]]; then
        IMAGE_IN="${base_image}-${timestamp}"
        IMAGE_OUT="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
    elif [[ "${stage}" == "dev" ]]; then
        IMAGE_IN="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
        IMAGE_OUT="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
    elif [[ "${stage}" == "runtime" ]]; then
        IMAGE_IN="nvidia/cuda:${CUDA_LITE}-cudnn${cudnn_ver}-runtime-ubuntu${UBUNTU_LTS}"
        DEV_IMAGE_IN="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
        IMAGE_OUT="${APOLLO_REPO}:runtime-${arch}-${UBUNTU_LTS}-${timestamp}"
    else
        echo "Unknown build stage: ${stage}. Exiting..."
        exit 1
    fi
}

function determine_images_in_out() {
    local arch="$1"
    local stage="$2"
    local timestamp="$(date +%Y%m%d_%H%M)"

    determine_cuda_versions "${arch}"
    determine_prev_image_timestamp "${arch}" "${stage}"

    determine_images_in_out_name ${arch} "${stage}" "${timestamp}"
}

function print_usage() {
    local prog="$(basename "$0")"
    echo "Usage:"
    echo "${TAB}${prog} -f <Dockerfile> [Options]"
    echo "Options:"
    echo "${TAB}-c,--clean      Docker build without cache"
    echo "${TAB}-m,--mode       build | download (default download)"
    echo "${TAB}-g,--geo        Geo-specific mirrors (cn/us)"
    echo "${TAB}-t,--timestamp  Previous stage image timestamp (yyyymmdd_hhmm, optional)"
    echo "${TAB}--dry           Dry run (print but not build)"
    echo "${TAB}-h,--help       Show help"
    echo "Examples:"
    echo "${TAB}${prog} -f cyber.x86_64.dockerfile -m build -g cn"
    echo "${TAB}${prog} -f dev.aarch64.dockerfile -m download"
}

function check_opt_arg() {
    local opt="$1"
    local arg="$2"
    if [[ -z "${arg}" || "${arg}" =~ ^-.* ]]; then
        echo "Argument missing for option ${opt}. Exiting..."
        exit 1
    fi
}

function parse_arguments() {
    if [[ $# -eq 0 ]] || [[ "$1" == "--help" ]]; then
        print_usage
        exit 0
    fi
    while [[ $# -gt 0 ]]; do
        local opt="$1"
        shift
        case $opt in
            -f|--dockerfile)
                check_opt_arg "${opt}" "$1"
                DOCKERFILE="$1"
                shift
                ;;
            -m|--mode)
                check_opt_arg "${opt}" "$1"
                INSTALL_MODE="$1"
                shift
                ;;
            -g|--geo)
                check_opt_arg "${opt}" "$1"
                TARGET_GEOLOC="$1"
                shift
                ;;
            -c|--clean)
                USE_CACHE=0
                ;;
            -t|--timestamp)
                check_opt_arg "${opt}" "$1"
                PREV_IMAGE_TIMESTAMP="$1"
                shift
                ;;
            --dry)
                DRY_RUN_ONLY=1
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            *)
                echo "Unknown option: ${opt}"
                print_usage
                exit 1
                ;;
        esac
    done
}

function check_arguments() {
    if [[ "${INSTALL_MODE}" != "download" && "${INSTALL_MODE}" != "build" ]]; then
        echo "Unknown INSTALL_MODE: ${INSTALL_MODE}."
        exit 1
    fi
    if [[ -z "${DOCKERFILE}" ]]; then
        echo "Dockerfile not specified. Exiting..."
        exit 1
    fi
    determine_target_arch_and_stage "${DOCKERFILE}"
}

function docker_build_preview() {
    echo "===== Docker Build Preview (${TARGET_STAGE}) ====="
    echo "|  Generated image: ${IMAGE_OUT}"
    echo "|  FROM image: ${IMAGE_IN}"
    echo "|  Dockerfile: ${DOCKERFILE}"
    echo "|  TARGET_ARCH=${TARGET_ARCH}, HOST_ARCH=${HOST_ARCH}"
    echo "|  CUDA_LITE=${CUDA_LITE}, CUDNN_VERSION=${CUDNN_VERSION}, TENSORRT_VERSION=${TENSORRT_VERSION}"
    echo "=================================================="
}

function docker_build_run() {
    local extra_args="--squash"
    if [[ "${USE_CACHE}" -eq 0 ]]; then
        extra_args="${extra_args} --no-cache=true"
    fi
    local context="$(dirname "${BASH_SOURCE[0]}")"

    local build_args="--build-arg BASE_IMAGE=${IMAGE_IN}"

    if [[ "${TARGET_STAGE}" == "base" ]]; then
        build_args+=" --build-arg CUDA_LITE=${CUDA_LITE}"
        build_args+=" --build-arg CUDNN_VERSION=${CUDNN_VERSION}"
        build_args+=" --build-arg TENSORRT_VERSION=${TENSORRT_VERSION}"
    elif [[ "${TARGET_STAGE}" == "cyber" || "${TARGET_STAGE}" == "dev" ]]; then
        build_args+=" --build-arg INSTALL_MODE=${INSTALL_MODE}"
        build_args+=" --build-arg GEOLOC=${TARGET_GEOLOC}"
    elif [[ "${TARGET_STAGE}" == "runtime" ]]; then
        build_args+=" --build-arg GEOLOC=${TARGET_GEOLOC}"
        build_args+=" --build-arg CUDA_LITE=${CUDA_LITE}"
        build_args+=" --build-arg CUDNN_VERSION=${CUDNN_VERSION}"
        build_args+=" --build-arg TENSORRT_VERSION=${TENSORRT_VERSION}"
        build_args+=" --build-arg DEV_IMAGE_IN=${DEV_IMAGE_IN}"
    else
        echo "Unknown build stage: ${TARGET_STAGE}. Exiting..."
        exit 1
    fi

    set -x
    # TODO: Use --platform linux/arm64 if building for aarch64
    docker build --network=host ${extra_args} -t "${IMAGE_OUT}" \
            ${build_args} \
            -f "${DOCKERFILE}" \
            "${context}"
    set +x

    echo "=============================================="
    echo "✅ Docker image build succeeded!"
    echo "Image: ${IMAGE_OUT}"
    echo "=============================================="
}

function main() {
    parse_arguments "$@"
    check_arguments

    check_experimental_docker
    determine_images_in_out "${TARGET_ARCH}" "${TARGET_STAGE}"

    docker_build_preview
    if [[ "${DRY_RUN_ONLY}" -gt 0 ]]; then return; fi

    docker_build_run
}

main "$@"
