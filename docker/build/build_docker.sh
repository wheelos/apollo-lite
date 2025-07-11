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
SUPPORTED_HARDWARE=( cpu gpu ) # Added explicit 'gpu' to make it clear for check
SUPPORTED_CPU_STAGES=( cyber dev runtime )

HOST_ARCH="$(uname -m)"
INSTALL_MODE="download"
TARGET_GEOLOC="us"

TARGET_ARCH=
TARGET_STAGE=
TARGET_EXTRA=

DOCKERFILE=
PREV_IMAGE_TIMESTAMP="" # Initialize explicitly to empty string
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
    local enabled

    # Check if Docker daemon is running and responsive
    if ! docker info > /dev/null 2>&1; then
        echo "Error: Docker daemon is not running or not responsive. Please start Docker." >&2
        exit 1
    fi

    enabled="$(docker version -f '{{.Server.Experimental}}')"
    if [ "${enabled}" != "true" ]; then
        echo "Experimental features should be enabled to run Apollo docker build." >&2
        echo "Please perform the following two steps to have it enabled:" >&2
        echo "  1) Add '\"experimental\": true' to '${daemon_cfg}' (or your Docker daemon configuration file)" >&2
        echo "     Example:" >&2
        echo "       {" >&2
        echo "           \"experimental\": true " >&2
        echo "       }" >&2
        echo "  2) Restart docker daemon. E.g., 'sudo systemctl restart docker'" >&2
        exit 1
    fi
}

function cpu_arch_support_check() {
    local arch="$1"
    if [[ ! " ${SUPPORTED_ARCHS[*]} " =~ " ${arch} " ]]; then
        echo "Unsupported CPU architecture: ${arch}. Supported architectures are: ${SUPPORTED_ARCHS[*]}. Exiting..." >&2
        exit 1
    fi
}

function build_stage_check() {
    local stage="$1"
    if [[ ! " ${SUPPORTED_STAGES[*]} " =~ " ${stage} " ]]; then
        echo "Unsupported build stage: ${stage}. Supported stages are: ${SUPPORTED_STAGES[*]}. Exiting..." >&2
        exit 1
    fi
}

function hardware_stage_check() {
    local hardware="$1"
    local stage="$2"

    # Ensure hardware type is supported
    if [[ ! " ${SUPPORTED_HARDWARE[*]} " =~ " ${hardware} " ]]; then
        echo "Unsupported hardware type: ${hardware}. Supported types are: ${SUPPORTED_HARDWARE[*]}. Exiting..." >&2
        exit 1
    fi

    if [[ "${hardware}" == "cpu" ]]; then
        if [[ ! " ${SUPPORTED_CPU_STAGES[*]} " =~ " ${stage} " ]]; then
            echo "CPU mode only supports stages: ${SUPPORTED_CPU_STAGES[*]}. Got '${stage}'. Exiting..." >&2
            exit 1
        fi
    fi
    # No specific stage check needed for 'gpu' if all stages are allowed
}

function determine_target_arch_and_stage() {
    local dockerfile_base="$(basename "$1")"
    if [[ ! "${dockerfile_base}" =~ ^([a-zA-Z0-9_]+)\.([a-z0-9_]+)\.([a-zA-Z0-9_]+)\.dockerfile$ ]]; then
        echo "Error: Expected Dockerfile name format '<stage>.<arch>.<extra>.dockerfile'" >&2
        echo "Got '${dockerfile_base}'. Exiting..." >&2
        exit 1
    fi

    local stage="${BASH_REMATCH[1]}"
    local arch="${BASH_REMATCH[2]}"
    local extra="${BASH_REMATCH[3]}"

    cpu_arch_support_check "${arch}"
    build_stage_check "${stage}"
    hardware_stage_check "${extra}" "${stage}" # extra is the hardware type

    TARGET_ARCH="${arch}"
    TARGET_STAGE="${stage}"
    TARGET_EXTRA="${extra}"

    if [[ "${TARGET_ARCH}" != "${HOST_ARCH}" ]]; then
        echo "[WARNING] HOST_ARCH(${HOST_ARCH}) != TARGET_ARCH(${TARGET_ARCH}) detected. Cross-architecture build." >&2
        echo "[WARNING] Make sure you have executed: 'docker run --rm --privileged multiarch/qemu-user-static --reset -p yes' for QEMU setup." >&2
    fi
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

# determine_prev_image_timestamp removed as its logic is integrated into determine_images_in_out

function determine_images_in_out_name() {
    local arch="$1"
    local stage="$2"
    local timestamp="$3"

    if [[ "${TARGET_EXTRA}" == "cpu" ]]; then
        case "${stage}" in
            cyber)
                IMAGE_IN="ubuntu:${UBUNTU_LTS}"
                IMAGE_OUT="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
                ;;
            dev)
                IMAGE_IN="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
                IMAGE_OUT="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
                ;;
            runtime)
                IMAGE_IN="ubuntu:${UBUNTU_LTS}" # Or a more minimal ubuntu:focal-slim for runtime
                DEV_IMAGE_IN="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
                IMAGE_OUT="${APOLLO_REPO}:runtime-${arch}-${UBUNTU_LTS}-${timestamp}"
                ;;
            *)
                echo "Error: Unknown build stage for CPU mode: ${stage}. Exiting..." >&2
                exit 1
                ;;
        esac
        return
    fi

    # GPU mode logic
    local cudnn_ver="${CUDNN_VERSION%%.*}"
    local trt_ver="${TENSORRT_VERSION%%.*}"
    local base_image_name_prefix="${APOLLO_REPO}:cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}-${arch}"

    case "${stage}" in
        base)
            IMAGE_IN="nvidia/cuda:${CUDA_LITE}-cudnn${cudnn_ver}-devel-ubuntu${UBUNTU_LTS}"
            IMAGE_OUT="${base_image_name_prefix}-${timestamp}"
            ;;
        cyber)
            IMAGE_IN="${base_image_name_prefix}-${timestamp}"
            IMAGE_OUT="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
            ;;
        dev)
            IMAGE_IN="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
            IMAGE_OUT="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
            ;;
        runtime)
            IMAGE_IN="nvidia/cuda:${CUDA_LITE}-cudnn${cudnn_ver}-runtime-ubuntu${UBUNTU_LTS}"
            DEV_IMAGE_IN="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
            IMAGE_OUT="${APOLLO_REPO}:runtime-${arch}-${UBUNTU_LTS}-${timestamp}"
            ;;
        *)
            echo "Error: Unknown build stage for GPU mode: ${stage}. Exiting..." >&2
            exit 1
            ;;
    esac
}

function determine_images_in_out() {
    local current_timestamp="$(date +%Y%m%d_%H%M)"
    # Use user-provided timestamp if available, otherwise use current timestamp
    local effective_timestamp="${PREV_IMAGE_TIMESTAMP:-${current_timestamp}}"

    if [[ "${TARGET_EXTRA}" != "cpu" ]]; then
        determine_cuda_versions "${TARGET_ARCH}"
    fi
    determine_images_in_out_name "${TARGET_ARCH}" "${TARGET_STAGE}" "${effective_timestamp}"
}

function docker_build_preview() {
    echo "===== Docker Build Preview (${TARGET_STAGE}/${TARGET_EXTRA}) ====="
    echo "|  Image OUT: ${IMAGE_OUT}"
    echo "|  Base Image: ${IMAGE_IN}"
    echo "|  Dockerfile: ${DOCKERFILE}"
    echo "|  ARCH: ${TARGET_ARCH}, HOST: ${HOST_ARCH}"
    echo "|  MODE: ${INSTALL_MODE}, GEO: ${TARGET_GEOLOC}"
    if [[ "${TARGET_EXTRA}" != "cpu" ]]; then
        echo "|  CUDA: ${CUDA_LITE}, CuDNN: ${CUDNN_VERSION}, TensorRT: ${TENSORRT_VERSION}"
    fi
    if [[ -n "${DEV_IMAGE_IN}" ]]; then # Only show if DEV_IMAGE_IN is set (for runtime stage)
        echo "|  Dev Image IN: ${DEV_IMAGE_IN}"
    fi
    echo "=================================================="
}

function docker_build_run() {
    local extra_args_array=()
    [[ "${USE_CACHE}" -eq 0 ]] && extra_args_array+=( "--no-cache=true" )

    local context="$(dirname "${BASH_SOURCE[0]}")"

    local build_args_array=()
    build_args_array+=( "--build-arg" "BASE_IMAGE=${IMAGE_IN}" )

    # Common args based on TARGET_EXTRA or specific stages
    if [[ "${TARGET_EXTRA}" == "cpu" || "${TARGET_STAGE}" == "cyber" || "${TARGET_STAGE}" == "dev" || "${TARGET_STAGE}" == "runtime" ]]; then
        build_args_array+=( "--build-arg" "GEOLOC=${TARGET_GEOLOC}" )
    fi

    # INSTALL_MODE only for cyber and dev stages
    if [[ "${TARGET_STAGE}" == "cyber" || "${TARGET_STAGE}" == "dev" ]]; then
        build_args_array+=( "--build-arg" "INSTALL_MODE=${INSTALL_MODE}" )
    fi

    # CUDA/CuDNN/TensorRT versions for GPU base and runtime
    if [[ "${TARGET_EXTRA}" != "cpu" && ( "${TARGET_STAGE}" == "base" || "${TARGET_STAGE}" == "runtime" ) ]]; then
        build_args_array+=( "--build-arg" "CUDA_LITE=${CUDA_LITE}" )
        build_args_array+=( "--build-arg" "CUDNN_VERSION=${CUDNN_VERSION}" )
        build_args_array+=( "--build-arg" "TENSORRT_VERSION=${TENSORRT_VERSION}" )
    fi

    # DEV_IMAGE_IN specifically for runtime stage
    if [[ "${TARGET_STAGE}" == "runtime" ]]; then
        build_args_array+=( "--build-arg" "DEV_IMAGE_IN=${DEV_IMAGE_IN}" )
    fi

    set -x # Enable command tracing
    docker build --network=host "${extra_args_array[@]}" -t "${IMAGE_OUT}" \
        "${build_args_array[@]}" \
        -f "${DOCKERFILE}" \
        "${context}"
    set +x # Disable command tracing
    echo "✅ Build complete: ${IMAGE_OUT}"
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
                DOCKERFILE="$1"; shift ;;
            -m|--mode)
                INSTALL_MODE="$1"; shift ;;
            -g|--geo)
                TARGET_GEOLOC="$1"; shift ;;
            -c|--clean)
                USE_CACHE=0 ;;
            -t|--timestamp)
                PREV_IMAGE_TIMESTAMP="$1"; shift ;;
            --dry)
                DRY_RUN_ONLY=1 ;;
            -h|--help)
                print_usage; exit 0 ;;
            *)
                echo "Unknown option: ${opt}. Use -h for help." >&2; print_usage; exit 1 ;;
        esac
    done
    # Validate DOCKERFILE is provided
    if [[ -z "${DOCKERFILE}" ]]; then
        echo "Error: Dockerfile must be specified using -f or --dockerfile." >&2
        print_usage
        exit 1
    fi
}

function print_usage() {
    echo "Usage:"
    echo "${TAB}$0 -f <Dockerfile> [options]"
    echo "Options:"
    echo "${TAB}-c,--clean        Disable Docker cache (--no-cache)"
    echo "${TAB}-m,--mode         Apollo install mode (build | download). Default: ${INSTALL_MODE}"
    echo "${TAB}-g,--geo          Geographical location (cn | us) for software sources. Default: ${TARGET_GEOLOC}"
    echo "${TAB}-t,--timestamp    Specify a fixed timestamp for image tags (YYYYMMDD_HHMM). Default: current time"
    echo "${TAB}-s,--no-squash    Do not squash Docker layers. Default: Squash layers"
    echo "${TAB}--dry             Dry run only (print build commands without executing)"
    echo "${TAB}-h,--help         Show this help message"
}

function main() {
    export DOCKER_BUILDKIT=0
    parse_arguments "$@"
    determine_target_arch_and_stage "${DOCKERFILE}"
    check_experimental_docker
    determine_images_in_out
    docker_build_preview
    [[ "${DRY_RUN_ONLY}" -gt 0 ]] || docker_build_run
}

main "$@"
