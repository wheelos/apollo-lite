#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
DOCKERFILE=""
GEOLOC="${GEOLOC:-cn}"
LOCAL_HTTP_ADDR="${LOCAL_HTTP_ADDR:-}"
RELEASE_VERSION="${RELEASE_VERSION:-}"
PRINT_ONLY=0
PUSH=0
CHECK_ONLY=0
NO_CACHE=0
FORWARD_ARGS=()

usage() {
  cat <<'USAGE'
Usage: build_docker.sh -f <image-alias> [options]

Builds the WheelOS developer image with Docker Buildx Bake.

Supported aliases:
  dev.x86_64.cpu.dockerfile   -> dev-amd64-cpu-u22
  dev.x86_64.cuda.dockerfile  -> dev-amd64-cuda-u22
  dev.aarch64.orin.dockerfile -> dev-orin-jp621-l4t3643

Options:
  -f, --dockerfile <alias>  Select an image target.
  -g, --geo <cn|us>         Select image registry and package mirrors.
  --cache-server <url>      Use the HTTP artifact cache.
  --release-version <tag>   Use a versioned platform-specific image tag.
  --set <key=value>         Forward a Buildx Bake override.
  --push                    Push the image (requires --release-version).
  --check                   Check the Dockerfile without exporting an image.
  -c, --clean               Disable BuildKit cache.
  --dry                     Print the expanded Bake configuration.
  -h, --help                Show this help.

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
    --release-version)
      (($# >= 2)) || fail "Missing value for $1"
      RELEASE_VERSION="$2"
      shift 2
      ;;
    --set)
      (($# >= 2)) || fail "Missing value for $1"
      FORWARD_ARGS+=(--set "$2")
      shift 2
      ;;
    --push)
      PUSH=1
      shift
      ;;
    --check)
      CHECK_ONLY=1
      shift
      ;;
    -c|--clean)
      NO_CACHE=1
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

if [[ -n "${RELEASE_VERSION}" && ! "${RELEASE_VERSION}" =~ ^[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}$ ]]; then
  fail "Invalid release version '${RELEASE_VERSION}' (expected a Docker tag component)."
fi
if ((PUSH == 1)) && [[ -z "${RELEASE_VERSION}" ]]; then
  fail "--push requires --release-version to avoid publishing a mutable dev tag."
fi
if ((PUSH == 1 && CHECK_ONLY == 1)); then
  fail "--check and --push cannot be used together."
fi
if ((PRINT_ONLY == 1 && CHECK_ONLY == 1)); then
  fail "--dry and --check cannot be used together."
fi

case "$(basename "${DOCKERFILE}")" in
  dev.x86_64.cpu.dockerfile)
    TARGET="dev-amd64-cpu-u22"
    ;;
  dev.x86_64.cuda.dockerfile)
    TARGET="dev-amd64-cuda-u22"
    ;;
  dev.aarch64.orin.dockerfile | dev.aarch64.l4t.dockerfile)
    TARGET="dev-orin-jp621-l4t3643"
    ;;
  base.*.dockerfile)
    fail "Standalone base images are no longer built; use a dev target."
    ;;
  cyber.*.dockerfile)
    fail "Cyber images have been removed; build a WheelOS dev target instead."
    ;;
  *)
    fail "Unsupported image alias: ${DOCKERFILE}"
    ;;
esac

export APOLLO_REPO="${APOLLO_REPO:-}"
export GEOLOC
export RELEASE_VERSION
BAKE_ARGS=(-f "${SCRIPT_DIR}/docker-bake.hcl")
BAKE_ARGS+=(--set "*.args.GEOLOC=${GEOLOC}")
if [[ -n "${LOCAL_HTTP_ADDR}" ]]; then
  BAKE_ARGS+=(--set "*.args.LOCAL_HTTP_ADDR=${LOCAL_HTTP_ADDR}")
fi
((NO_CACHE == 0)) || BAKE_ARGS+=(--no-cache)
((PRINT_ONLY == 0)) || BAKE_ARGS+=(--print)
((CHECK_ONLY == 0)) || BAKE_ARGS+=(--check)
BAKE_ARGS+=("${FORWARD_ARGS[@]}")
if ((CHECK_ONLY == 1)); then
  BAKE_ARGS+=(--set "${TARGET}.output=type=cacheonly")
elif ((PUSH == 1)); then
  BAKE_ARGS+=(--set "${TARGET}.output=type=image,push=true")
else
  BAKE_ARGS+=(--set "${TARGET}.output=type=docker")
fi
BAKE_ARGS+=("${TARGET}")

cd "${SCRIPT_DIR}/../.."
DOCKER_BUILDKIT=1 exec docker buildx bake "${BAKE_ARGS[@]}"
