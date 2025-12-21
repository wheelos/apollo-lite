#!/usr/bin/env bash
set -euo pipefail

TAB="    " # 4 Spaces

DOCKER_BUILDKIT=${DOCKER_BUILDKIT:-1}

APOLLO_REPO="${APOLLO_REPO:-wheelos/apollo}"
UBUNTU_LTS="${UBUNTU_LTS:-20.04}"

declare -A CUDA_LITE_VERSIONS
declare -A CUDNN_VERSIONS
declare -A TENSORRT_VERSIONS
# x86_64
CUDA_LITE_VERSIONS["x86_64"]="11.8.0"
CUDNN_VERSIONS["x86_64"]="8"
TENSORRT_VERSIONS["x86_64"]="8.6.1.6"
# aarch64
CUDA_LITE_VERSIONS["aarch64"]="11.4.3"
CUDNN_VERSIONS["aarch64"]="8.6.0.166"
TENSORRT_VERSIONS["aarch64"]="8.5.2"

SUPPORTED_ARCHS=( x86_64 aarch64 )
SUPPORTED_STAGES=( base cyber dev runtime )
# TODO(All): maybe ROCm support in the future
SUPPORTED_COMPUTE_PLATFORM=( cpu cuda )
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

LOCAL_HTTP_ADDR=${LOCAL_HTTP_ADDR:-http://172.17.0.1:8388}

function fail() {
    echo "Error: $1" >&2
    exit 1
}

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

function compute_platform_stage_check() {
    local platform="$1"
    local stage="$2"

    # Ensure compute platform is supported
    if [[ ! " ${SUPPORTED_COMPUTE_PLATFORM[*]} " =~ " ${platform} " ]]; then
        echo "Unsupported compute platform: ${platform}. Supported platforms are: ${SUPPORTED_COMPUTE_PLATFORM[*]}. Exiting..." >&2
        exit 1
    fi

    if [[ "${platform}" == "cpu" ]]; then
        if [[ ! " ${SUPPORTED_CPU_STAGES[*]} " =~ " ${stage} " ]]; then
            echo "CPU mode only supports stages: ${SUPPORTED_CPU_STAGES[*]}. Got '${stage}'. Exiting..." >&2
            exit 1
        fi
    fi
    # No specific stage check needed for 'cuda' if all stages are allowed
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
    compute_platform_stage_check "${extra}" "${stage}" # extra is the compute platform

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

function determine_cpu_img() {
  local arch="$1"
  local stage="$2"
  local timestamp="$3"
  local effective_timestamp="${PREV_IMAGE_TIMESTAMP:-$timestamp}"

  # Define the general image name format
  local cyber_img="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
  local dev_img="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
  local runtime_img="${APOLLO_REPO}:runtime-${arch}-${UBUNTU_LTS}-${timestamp}"

  # Define the previous stage image name
  local prev_cyber_img="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${effective_timestamp}"
  local prev_dev_img="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${effective_timestamp}"

  case "${stage}" in
    cyber)
      IMAGE_IN="ubuntu:${UBUNTU_LTS}"
      IMAGE_OUT="${cyber_img}"
      ;;
    dev)
      IMAGE_IN="${prev_cyber_img}"
      IMAGE_OUT="${dev_img}"
      ;;
    runtime)
      IMAGE_IN="ubuntu:${UBUNTU_LTS}"
      DEV_IMAGE_IN="${prev_dev_img}"
      IMAGE_OUT="${runtime_img}"
      ;;
    *)
      fail "Unknown build stage for CPU mode: '${stage}'"
      ;;
  esac
}

function determine_gpu_img() {
  local arch="$1"
  local stage="$2"
  local timestamp="$3"
  local effective_timestamp="${PREV_IMAGE_TIMESTAMP:-$timestamp}"

  # Extract the major version number from the determined global variables
  local cudnn_ver="${CUDNN_VERSION%%.*}"
  local trt_ver="${TENSORRT_VERSION%%.*}"

  # Define the prefix for the GPU base image, logic is more centralized
  local base_image_prefix="${APOLLO_REPO}:cuda${CUDA_LITE}-cudnn${cudnn_ver}-trt${trt_ver}-devel-${UBUNTU_LTS}-${arch}"
  local base_image_tagged="${base_image_prefix}-${timestamp}"
  local prev_base_image_tagged="${base_image_prefix}-${effective_timestamp}"

  # Define the general image name format (consistent with CPU mode)
  local cyber_img="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${timestamp}"
  local dev_img="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${timestamp}"
  local runtime_img="${APOLLO_REPO}:runtime-${arch}-${UBUNTU_LTS}-${timestamp}"

  # Define the previous stage image name (consistent with CPU mode)
  local prev_cyber_img="${APOLLO_REPO}:cyber-${arch}-${UBUNTU_LTS}-${effective_timestamp}"
  local prev_dev_img="${APOLLO_REPO}:dev-${arch}-${UBUNTU_LTS}-${effective_timestamp}"

  case "${stage}" in
    base)
      IMAGE_IN="nvidia/cuda:${CUDA_LITE}-cudnn${cudnn_ver}-devel-ubuntu${UBUNTU_LTS}"
      if [[ "${arch}" == "aarch64" ]]; then
        # Note: nvidia/cuda may not work for all arm64v8 hardware, here
        # we use a generic arm64v8 Ubuntu image as the base image. And
        # install CUDA/CuDNN/TensorRT manually in the Dockerfile.
        # See the Dockerfile(base.aarch64.dockerfile) for more details.
        # TODO(All): specified via args, such as orin or xavier
        IMAGE_IN="docker.io/arm64v8/ubuntu:${UBUNTU_LTS}"
      fi
      IMAGE_OUT="${base_image_tagged}"
      ;;
    cyber)
      IMAGE_IN="${prev_base_image_tagged}"
      IMAGE_OUT="${cyber_img}"
      ;;
    dev)
      IMAGE_IN="${prev_cyber_img}"
      IMAGE_OUT="${dev_img}"
      ;;
    runtime)
      IMAGE_IN="nvidia/cuda:${CUDA_LITE}-cudnn${cudnn_ver}-runtime-ubuntu${UBUNTU_LTS}"
      if [[ "${arch}" == "aarch64" ]]; then
        # Note: nvidia/cuda may not work for all arm64v8 hardware, here
        # we use a generic arm64v8 Ubuntu image as the base image. And
        # install CUDA/CuDNN/TensorRT manually in the Dockerfile.
        # See the Dockerfile(base.aarch64.dockerfile) for more details.
        # TODO(All): specified via args, such as orin or xavier
        IMAGE_IN="docker.io/arm64v8/ubuntu:${UBUNTU_LTS}"
      fi
      DEV_IMAGE_IN="${prev_dev_img}"
      IMAGE_OUT="${runtime_img}"
      ;;
    *)
      fail "Unknown build stage for GPU mode: '${stage}'"
      ;;
  esac
}

function determine_images_in_out() {
  # Use shell parameter expansion: if PREV_IMAGE_TIMESTAMP is not set, use the current timestamp
  local timestamp
  timestamp="$(date +%Y%m%d_%H%M)"

  # Dispatch to different handler functions according to TARGET_EXTRA
  if [[ "${TARGET_EXTRA}" == "cpu" ]]; then
    determine_cpu_img "${TARGET_ARCH}" "${TARGET_STAGE}" "${timestamp}"
  else
    # In GPU mode, determine CUDA version first, then determine image name
    determine_cuda_versions "${TARGET_ARCH}"
    determine_gpu_img "${TARGET_ARCH}" "${TARGET_STAGE}" "${timestamp}"
  fi
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
    build_args_array+=( "--build-arg" "LOCAL_HTTP_ADDR=${LOCAL_HTTP_ADDR}" )

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
            --cache-server)
                LOCAL_HTTP_ADDR="$1"; shift ;;
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
  echo "Usage: $0 -f <Dockerfile> [options]"
  echo "Options:"
  echo "${TAB}-c,--clean            Disable Docker cache"
  echo "${TAB}-m,--mode             Install mode (build|download), default: ${INSTALL_MODE}"
  echo "${TAB}-g,--geo              Geo location (cn|us), default: ${TARGET_GEOLOC}"
  echo "${TAB}-t,--timestamp        Image tag timestamp (YYYYMMDD_HHMM), default: now"
  echo "${TAB}--cache-server <URL>  Use a local HTTP server for caching packages (default: ${LOCAL_HTTP_ADDR})"
  echo "${TAB}--dry                 Dry run (show build commands only)"
  echo "${TAB}-h,--help             Show this help"
  echo
  echo "Tip: To pre-download packages, use a local HTTP server (python3 -m http.server 8388). Or set LOCAL_HTTP_ADDR."
}

function main() {
    parse_arguments "$@"
    determine_target_arch_and_stage "${DOCKERFILE}"
    check_experimental_docker
    determine_images_in_out
    docker_build_preview
    [[ "${DRY_RUN_ONLY}" -gt 0 ]] || docker_build_run
}

main "$@"
