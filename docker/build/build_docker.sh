#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DOCKERFILE=""
GEOLOC="${GEOLOC:-cn}"
LOCAL_HTTP_ADDR="${LOCAL_HTTP_ADDR:-}"
PRINT_ONLY=0
PUSH=0
NO_CACHE=0
FORWARD_ARGS=()

usage() {
  cat <<'USAGE'
Usage: build_docker.sh -f <legacy-dev-dockerfile> [options]

Builds a direct base-to-dev image using Docker Buildx Bake.

Supported Dockerfile aliases:
  dev.x86_64.cpu.dockerfile   -> dev-amd64-cpu-u22
  dev.x86_64.cuda.dockerfile  -> dev-amd64-cuda-u22
  dev.x86_64.u22.dockerfile   -> dev-amd64-cuda-u22
  dev.aarch64.cpu.dockerfile  -> dev-arm64-cpu-u22
  dev.aarch64.l4t.dockerfile  -> dev-orin-jp621-l4t3643

Options:
  -f, --dockerfile <file>    Select a legacy Dockerfile alias.
  -g, --geo <geo>            Package mirror region (cn|us), default: cn.
  -c, --clean                Disable BuildKit cache.
  --cache-server <url>       HTTP artifact cache URL.
  --push                     Push the final image to its registry.
  --dry                      Print the expanded Bake configuration.
  --set <key=value>          Forward an additional Buildx Bake override.
  -h, --help                 Show this help.

Buildx Bake targets can also be invoked directly from the repository root.
USAGE
}

fail() {
  printf 'Error: %s\n' "$1" >&2
  exit 1
}

while (($#)); do
  case "$1" in
    -f|--dockerfile)
      (($# >= 2)) || fail "Missing value for $1"
      DOCKERFILE="$2"
      shift 2
      ;;
    -g|--geo)
      (($# >= 2)) || fail "Missing value for $1"
      GEOLOC="$2"
      shift 2
      ;;
    --cache-server)
      (($# >= 2)) || fail "Missing value for $1"
      LOCAL_HTTP_ADDR="$2"
      shift 2
      ;;
    --set)
      (($# >= 2)) || fail "Missing value for $1"
      FORWARD_ARGS+=(--set "$2")
      shift 2
      ;;
    -c|--clean)
      NO_CACHE=1
      shift
      ;;
    --push)
      PUSH=1
      shift
      ;;
    --dry)
      PRINT_ONLY=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -t|--timestamp)
      fail "Previous-stage timestamps are obsolete; Bake builds dev images directly from platform bases."
      ;;
    *)
      fail "Unknown option: $1"
      ;;
  esac
done

[[ -n "${DOCKERFILE}" ]] || {
  usage
  exit 1
}

case "${GEOLOC}" in
  cn|us)
    ;;
  *)
    fail "Unsupported geolocation: ${GEOLOC} (expected cn or us)"
    ;;
esac
export GEOLOC

case "$(basename "${DOCKERFILE}")" in
  dev.x86_64.cpu.dockerfile)
    TARGET="dev-amd64-cpu-u22"
    ;;
  dev.x86_64.cuda.dockerfile)
    TARGET="dev-amd64-cuda-u22"
    ;;
  dev.x86_64.u22.dockerfile)
    TARGET="dev-amd64-cuda-u22"
    ;;
  base.*.dockerfile)
    fail "Standalone base images are no longer published; Bake's base stage is included in each dev target."
    ;;
  dev.aarch64.cpu.dockerfile)
    TARGET="dev-arm64-cpu-u22"
    ;;
  dev.aarch64.l4t.dockerfile)
    TARGET="dev-orin-jp621-l4t3643"
    ;;
  dev.aarch64.nx.dockerfile)
    fail "Jetson Xavier NX is not supported by the Orin L4T 36.4 target."
    ;;
  dev.aarch64.cuda.dockerfile)
    fail "Generic aarch64 CUDA is not a supported target; use the Orin L4T target on Jetson."
    ;;
  cyber.*.dockerfile)
    fail "Cyber images have been removed; build a direct dev target instead."
    ;;
  *)
    fail "Unsupported Dockerfile alias: ${DOCKERFILE}"
    ;;
esac

BAKE_ARGS=(-f "${SCRIPT_DIR}/docker-bake.hcl")
BAKE_ARGS+=(--set "*.args.GEOLOC=${GEOLOC}")
if [[ -n "${LOCAL_HTTP_ADDR}" ]]; then
  BAKE_ARGS+=(--set "*.args.LOCAL_HTTP_ADDR=${LOCAL_HTTP_ADDR}")
fi
((NO_CACHE == 0)) || BAKE_ARGS+=(--no-cache)
((PUSH == 0)) || BAKE_ARGS+=(--push)
((PRINT_ONLY == 0)) || BAKE_ARGS+=(--print)
BAKE_ARGS+=("${FORWARD_ARGS[@]}" "${TARGET}")

cd "${SCRIPT_DIR}/../.."
DOCKER_BUILDKIT=1 exec docker buildx bake "${BAKE_ARGS[@]}"
