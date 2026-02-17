#!/usr/bin/env bash
set -euo pipefail

TAB="    "

DOCKER_BUILDKIT="${DOCKER_BUILDKIT:-1}"
APOLLO_REPO="${APOLLO_REPO:-wheelos/apollo}"
UBUNTU_LTS="${UBUNTU_LTS:-22.04}"
TARGET_GEOLOC="us"
INSTALL_MODE="download"
LOCAL_HTTP_ADDR="${LOCAL_HTTP_ADDR:-http://172.17.0.1:8388}"
IMAGE_TAG="${IMAGE_TAG:-$(date +%Y%m%d_%H%M)}"
L4T_TAG="${L4T_TAG:-r36.4.0}"

DOCKERFILE=""
TARGET=""
USE_CACHE=1
DRY_RUN_ONLY=0
PUSH_IMAGE=0
LOAD_IMAGE=1

function print_usage() {
  echo "Usage: $0 -f <Dockerfile> [options]"
  echo "Options:"
  echo "${TAB}-f,--dockerfile        Dockerfile name (e.g., base.x86_64.cuda.dockerfile)"
  echo "${TAB}-m,--mode              Installer mode (build|download), default: ${INSTALL_MODE}"
  echo "${TAB}-g,--geo               Geolocation (cn|us), default: ${TARGET_GEOLOC}"
  echo "${TAB}-t,--timestamp         Image tag suffix, default: ${IMAGE_TAG}"
  echo "${TAB}--l4t-tag              Jetson L4T tag, default: ${L4T_TAG}"
  echo "${TAB}--cache-server <URL>   Local package cache URL, default: ${LOCAL_HTTP_ADDR}"
  echo "${TAB}-c,--clean             Disable Docker cache"
  echo "${TAB}--push                 Push images to registry"
  echo "${TAB}--no-load              Disable loading built images into local Docker"
  echo "${TAB}--dry                  Print bake plan only"
  echo "${TAB}-h,--help              Show this help"
}

function check_docker() {
  if ! docker info >/dev/null 2>&1; then
    echo "Error: Docker daemon is not running." >&2
    exit 1
  fi
  if ! docker buildx version >/dev/null 2>&1; then
    echo "Error: docker buildx is required." >&2
    exit 1
  fi
}

function parse_arguments() {
  if [[ $# -eq 0 ]]; then
    print_usage
    exit 0
  fi

  while [[ $# -gt 0 ]]; do
    case "$1" in
      -f|--dockerfile)
        DOCKERFILE="$2"; shift 2 ;;
      -m|--mode)
        INSTALL_MODE="$2"; shift 2 ;;
      -g|--geo)
        TARGET_GEOLOC="$2"; shift 2 ;;
      -t|--timestamp)
        IMAGE_TAG="$2"; shift 2 ;;
      --l4t-tag)
        L4T_TAG="$2"; shift 2 ;;
      --cache-server)
        LOCAL_HTTP_ADDR="$2"; shift 2 ;;
      -c|--clean)
        USE_CACHE=0; shift ;;
      --push)
        PUSH_IMAGE=1; shift ;;
      --no-load)
        LOAD_IMAGE=0; shift ;;
      --dry)
        DRY_RUN_ONLY=1; shift ;;
      -h|--help)
        print_usage; exit 0 ;;
      *)
        echo "Unknown option: $1" >&2
        print_usage
        exit 1 ;;
    esac
  done

  if [[ -z "${DOCKERFILE}" ]]; then
    echo "Error: Dockerfile must be specified by -f/--dockerfile" >&2
    exit 1
  fi
}

function dockerfile_to_target() {
  case "$(basename "${DOCKERFILE}")" in
    base.x86_64.cuda.dockerfile) TARGET="base-x86_64-cuda" ;;
    dev.x86_64.cuda.dockerfile) TARGET="dev-x86_64-cuda" ;;
    base.aarch64.cuda.dockerfile) TARGET="base-aarch64-cuda" ;;
    dev.aarch64.cuda.dockerfile) TARGET="dev-aarch64-cuda" ;;
    *)
      echo "Error: unsupported dockerfile '${DOCKERFILE}' for buildx bake workflow." >&2
      echo "Supported dockerfiles: base/dev for x86_64.cuda and aarch64.cuda" >&2
      exit 1 ;;
  esac
}

function run_bake() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
  local bake_file="${script_dir}/docker-bake.hcl"

  local set_args=()
  set_args+=("--set" "*.args.GEOLOC=${TARGET_GEOLOC}")
  set_args+=("--set" "*.args.INSTALL_MODE=${INSTALL_MODE}")
  set_args+=("--set" "*.args.LOCAL_HTTP_ADDR=${LOCAL_HTTP_ADDR}")
  set_args+=("--set" "*.args.APOLLO_DIST=stable")
  set_args+=("--set" "*.args.UBUNTU_LTS=${UBUNTU_LTS}")

  case "${TARGET}" in
    base-x86_64-cuda)
      set_args+=("--set" "${TARGET}.tags=${APOLLO_REPO}:cuda12.6.3-cudnn9-trt10-devel-${UBUNTU_LTS}-x86_64-${IMAGE_TAG},${APOLLO_REPO}:base-x86_64-${UBUNTU_LTS}-${IMAGE_TAG}")
      ;;
    dev-x86_64-cuda)
      set_args+=("--set" "${TARGET}.args.BASE_IMAGE=${APOLLO_REPO}:cuda12.6.3-cudnn9-trt10-devel-${UBUNTU_LTS}-x86_64-${IMAGE_TAG}")
      set_args+=("--set" "${TARGET}.tags=${APOLLO_REPO}:dev-x86_64-${UBUNTU_LTS}-${IMAGE_TAG}")
      ;;
    base-aarch64-cuda)
      set_args+=("--set" "${TARGET}.args.BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:${L4T_TAG}")
      set_args+=("--set" "${TARGET}.tags=${APOLLO_REPO}:jetpack-${L4T_TAG}-base-aarch64-${IMAGE_TAG},${APOLLO_REPO}:base-aarch64-${UBUNTU_LTS}-${IMAGE_TAG}")
      ;;
    dev-aarch64-cuda)
      set_args+=("--set" "${TARGET}.args.BASE_IMAGE=${APOLLO_REPO}:jetpack-${L4T_TAG}-base-aarch64-${IMAGE_TAG}")
      set_args+=("--set" "${TARGET}.tags=${APOLLO_REPO}:dev-aarch64-${UBUNTU_LTS}-${IMAGE_TAG}")
      ;;
  esac

  if [[ "${USE_CACHE}" -eq 0 ]]; then
    set_args+=("--set" "${TARGET}.no-cache=true")
  fi

  if [[ "${PUSH_IMAGE}" -eq 1 ]]; then
    set_args+=("--set" "${TARGET}.output=type=registry")
  elif [[ "${LOAD_IMAGE}" -eq 1 ]]; then
    set_args+=("--set" "${TARGET}.output=type=docker")
  fi

  echo "===== Buildx Bake Preview ====="
  echo "| target: ${TARGET}"
  echo "| dockerfile: ${DOCKERFILE}"
  echo "| geoloc: ${TARGET_GEOLOC}, install_mode: ${INSTALL_MODE}"
  echo "| image_tag: ${IMAGE_TAG}, l4t_tag: ${L4T_TAG}"
  echo "==============================="

  if [[ "${DRY_RUN_ONLY}" -eq 1 ]]; then
    docker buildx bake --file "${bake_file}" "${TARGET}" "${set_args[@]}" --print
  else
    docker buildx bake --file "${bake_file}" "${TARGET}" "${set_args[@]}"
  fi
}

function main() {
  parse_arguments "$@"
  check_docker
  dockerfile_to_target
  run_bake
}

main "$@"
