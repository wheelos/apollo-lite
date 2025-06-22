#!/bin/bash

# Exit immediately on error
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Determine target architecture
TARGET_ARCH=$(uname -m)

# Get Bazel version from file
CURR_DIR="$(pwd -P)"
BAZEL_VERSION=$(<"${CURR_DIR}/../../../.bazelversion")
SYSROOT_BIN_DIR="${SYSROOT_DIR}/bin"

install_bazel() {
  local pkg_name=$1
  local checksum=$2
  local download_link="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${pkg_name}"

  download_if_not_cached "${pkg_name}" "${checksum}" "${download_link}"

  if [[ "${pkg_name}" == *.deb ]]; then
    apt_get_update_and_install g++ unzip zip
    dpkg -i "${pkg_name}"
  else
    cp -f "${pkg_name}" "${SYSROOT_BIN_DIR}/bazel"
    chmod a+x "${SYSROOT_BIN_DIR}/bazel"
  fi

  rm -f "${pkg_name}"
}

cleanup() {
  apt-get clean
  rm -rf /var/lib/apt/lists/*
}

trap cleanup EXIT

echo "Installing Bazel ${BAZEL_VERSION}..."

case "$TARGET_ARCH" in
  x86_64)
    BAZEL_PKG="bazel_${BAZEL_VERSION}-linux-x86_64.deb"
    BAZEL_CHECKSUM="ac6249d1192aea9feaf49dfee2ab50c38cee2454b00cf29bbec985a11795c025"
    install_bazel "${BAZEL_PKG}" "${BAZEL_CHECKSUM}"
    ;;

  aarch64)
    BAZEL_PKG="bazel-${BAZEL_VERSION}-linux-arm64"
    BAZEL_CHECKSUM="2d86d36db0c9af15747ff02a80e6db11a45d68f868ea8f62f489505c474f0099"
    install_bazel "${BAZEL_PKG}" "${BAZEL_CHECKSUM}"

    cp /opt/apollo/rcfiles/bazel_completion.bash /etc/bash_completion.d/bazel
    ;;

  *)
    echo "Error: Target architecture ${TARGET_ARCH} not supported yet"
    exit 1
    ;;
esac

info "Done installing Bazel ${BAZEL_VERSION}"
