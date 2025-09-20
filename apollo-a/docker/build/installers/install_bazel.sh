#!/bin/bash

# Exit immediately on error
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Determine target architecture
TARGET_ARCH=$(uname -m)

# Get Bazel version from file
CURR_DIR="$(pwd -P)"
BAZELISK_VERSION="${BAZELISK_VERSION:-v1.26.0}"
BAZELISK_CHECKSUM_AMD64="${BAZELISK_CHECKSUM_AMD64:-6539c12842ad76966f3d493e8f80d67caa84ec4a000e220d5459833c967c12bc}"
BAZELISK_CHECKSUM_ARM64="${BAZELISK_CHECKSUM_ARM64:-54f85ef4c23393f835252cc882e5fea596e8ef3c4c2056b059f8067cd19f0351}"

BAZEL_VERSION="${BAZEL_VERSION:-$(cat "${CURR_DIR}/../../../.bazelversion" 2>/dev/null || echo "7.6.1")}"
BAZEL_CHECKSUM_AMD64="${BAZEL_CHECKSUM_AMD64:-ac6249d1192aea9feaf49dfee2ab50c38cee2454b00cf29bbec985a11795c025}"
BAZEL_CHECKSUM_ARM64="${BAZEL_CHECKSUM_ARM64:-2d86d36db0c9af15747ff02a80e6db11a45d68f868ea8f62f489505c474f0099}"
SYSROOT_BIN_DIR="${SYSROOT_DIR}/bin"

install_bazelisk() {
  local pkg_name=$1
  local version=$2
  local checksum=$3
  local download_link="https://github.com/bazelbuild/bazelisk/releases/download/${version}/${pkg_name}"
  download_if_not_cached "${pkg_name}" "${checksum}" "${download_link}"
  if [[ "${pkg_name}" == *.deb ]]; then
    apt_get_update_and_install ./${pkg_name}
  else
    cp -f "${pkg_name}" "${SYSROOT_BIN_DIR}/bazelisk"
    chmod +x "${SYSROOT_BIN_DIR}/bazelisk"
    # TODO(All): use bazelisk to install bazel
    # ln -snf "${SYSROOT_BIN_DIR}/bazelisk" "${SYSROOT_BIN_DIR}/bazel"

  fi
  rm -f "${pkg_name}"
}

install_bazel() {
  local pkg_name=$1
  local checksum=$2
  local download_link="https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/${pkg_name}"

  download_if_not_cached "${pkg_name}" "${checksum}" "${download_link}"

  if [[ "${pkg_name}" == *.deb ]]; then
    apt_get_update_and_install ./${pkg_name}
  else
    # install directly to sysroot
    # TODO(All): use bazelisk to install bazel
    install -m755 "${pkg_name}" "${SYSROOT_BIN_DIR}/bazel"

    # for bazelisk
    mkdir -p ${HOME}/.cache/bazelisk/downloads/metadata/bazelbuild
    echo -n "${checksum}" > ${HOME}/.cache/bazelisk/downloads/metadata/bazelbuild/${pkg_name}
    mkdir -p ${HOME}/.cache/bazelisk/downloads/sha256/${checksum}/bin
    install -m755 "${pkg_name}" ${HOME}/.cache/bazelisk/downloads/sha256/${checksum}/bin/bazel
    install -m600 /dev/null ${HOME}/.cache/bazelisk/downloads/sha256/${checksum}/bin/bazel.lock
  fi

  rm -f "${pkg_name}"
}

cleanup() {
  apt-get clean
  rm -rf /var/lib/apt/lists/*
}

trap cleanup EXIT

echo "Installing Bazelisk[${BAZELISK_VERSION}] and Bazel[${BAZEL_VERSION}]..."

case "$TARGET_ARCH" in
  x86_64)
    install_bazelisk "bazelisk-linux-amd64" "${BAZELISK_VERSION}" "${BAZELISK_CHECKSUM_AMD64}"
    install_bazel "bazel-${BAZEL_VERSION}-linux-x86_64" "${BAZEL_CHECKSUM_AMD64}"
    ;;

  aarch64)
    install_bazelisk "bazelisk-linux-arm64" "${BAZELISK_VERSION}" "${BAZELISK_CHECKSUM_ARM64}"
    install_bazel "bazel-${BAZEL_VERSION}-linux-arm64" "${BAZEL_CHECKSUM_ARM64}"

    ;;

  *)
    echo "Error: Target architecture ${TARGET_ARCH} not supported yet"
    exit 1
    ;;
esac

mkdir -p /etc/bash_completion.d
cp /opt/apollo/rcfiles/bazel_completion.bash /etc/bash_completion.d/bazel

info "Done installing Bazel ${BAZEL_VERSION}"
