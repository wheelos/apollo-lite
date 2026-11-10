#!/usr/bin/env bash

set -euo pipefail

readonly VERSION="10.1.0"
INSTALL_DIR="${1:-/opt/apollo/sysroot/bin}"

case "$(uname -m)" in
  x86_64)
    ARCH="amd64"
    CHECKSUM="31b6a8aa1e5c746696788f428729701770ad91925873d8256cb885c60e12c77e"
    ;;
  aarch64|arm64)
    ARCH="arm64"
    CHECKSUM="38d2ed845f560b4a16ddee41de906508a95f8dc85b04e0851b0a71e3a70d3890"
    ;;
  *)
    echo "Unsupported Buildifier architecture: $(uname -m)" >&2
    exit 1
    ;;
esac

DOWNLOAD_DIR="$(mktemp -d)"
trap 'rm -f "${DOWNLOAD_DIR}/buildifier"; rmdir "${DOWNLOAD_DIR}"' EXIT

curl --fail --location --show-error --silent \
  --connect-timeout 15 --max-time 180 \
  "https://github.com/bazel-contrib/buildtools/releases/download/v${VERSION}/buildifier-linux-${ARCH}" \
  --output "${DOWNLOAD_DIR}/buildifier"
if ! echo "${CHECKSUM}  ${DOWNLOAD_DIR}/buildifier" | sha256sum --check --status; then
  echo "Buildifier ${VERSION} checksum verification failed (${ARCH})." >&2
  exit 1
fi
install -d "${INSTALL_DIR}"
install -m 0755 "${DOWNLOAD_DIR}/buildifier" "${INSTALL_DIR}/buildifier"
"${INSTALL_DIR}/buildifier" --version
