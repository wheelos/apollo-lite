#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cat <<'EOF'
Usage: bash scripts/start_bazel_remote.sh

Start a host-local bazel-remote Docker container. Overrides:
  BAZEL_REMOTE_NAME          Container name (default: bazel-remote)
  BAZEL_REMOTE_PORT          Host loopback port (default: 19090)
  BAZEL_REMOTE_CACHE_DIR     Persistent cache directory (default: ~/.cache/bazel-remote)
  BAZEL_REMOTE_MAX_SIZE_GIB  Cache limit in GiB (default: 5)
  BAZEL_REMOTE_IMAGE         Container image (default: buchgr/bazel-remote-cache:v2.6.1)
EOF
  exit 0
fi

if [[ $# -ne 0 ]]; then
  echo "Unexpected arguments; use --help for configuration." >&2
  exit 2
fi

name="${BAZEL_REMOTE_NAME:-bazel-remote}"
port="${BAZEL_REMOTE_PORT:-19090}"
cache_dir="${BAZEL_REMOTE_CACHE_DIR:-${HOME}/.cache/bazel-remote}"
max_size="${BAZEL_REMOTE_MAX_SIZE_GIB:-5}"
image="${BAZEL_REMOTE_IMAGE:-buchgr/bazel-remote-cache:v2.6.1}"

if [[ ! "${port}" =~ ^[0-9]{1,5}$ ]] || (( 10#${port} < 1 || 10#${port} > 65535 )); then
  echo "BAZEL_REMOTE_PORT must be an integer from 1 to 65535." >&2
  exit 2
fi
if [[ ! "${max_size}" =~ ^[1-9][0-9]*$ ]]; then
  echo "BAZEL_REMOTE_MAX_SIZE_GIB must be a positive integer." >&2
  exit 2
fi
if [[ "${cache_dir}" != /* || "${cache_dir}" == *","* ]]; then
  echo "BAZEL_REMOTE_CACHE_DIR must be an absolute path without commas." >&2
  exit 2
fi

command -v docker >/dev/null || { echo "Docker is required." >&2; exit 1; }
command -v curl >/dev/null || { echo "curl is required for readiness checks." >&2; exit 1; }
docker info >/dev/null
if docker container inspect "${name}" >/dev/null 2>&1; then
  echo "Container ${name} already exists; inspect it rather than replacing it." >&2
  exit 1
fi

mkdir -p "${cache_dir}"
cache_dir="$(cd "${cache_dir}" && pwd -P)"
if [[ "${cache_dir}" == *","* ]]; then
  echo "Resolved cache directory must not contain commas: ${cache_dir}" >&2
  exit 2
fi
if [[ ! -w "${cache_dir}" ]]; then
  echo "Cache directory is not writable by the current user: ${cache_dir}" >&2
  exit 1
fi

docker run -d \
  --name "${name}" \
  --restart unless-stopped \
  --user "$(id -u):$(id -g)" \
  --publish "127.0.0.1:${port}:8080" \
  --mount "type=bind,src=${cache_dir},dst=/data" \
  "${image}" --dir /data --max_size "${max_size}" >/dev/null

for ((attempt = 0; attempt < 20; attempt++)); do
  if [[ "$(docker inspect --format '{{.State.Running}}' "${name}")" != "true" ]]; then
    break
  fi
  if curl --silent --show-error --fail --max-time 1 "http://127.0.0.1:${port}/status" >/dev/null 2>&1; then
    echo "Bazel remote cache ready: http://127.0.0.1:${port}"
    echo "Persistent cache: ${cache_dir} (limit: ${max_size} GiB)"
    exit 0
  fi
  sleep 1
done

echo "Bazel remote cache failed to become ready; inspect container ${name}." >&2
docker logs --tail 20 "${name}" >&2 || true
exit 1
