#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

set -eux

# Go to script directory and source base installer functions
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

# Argument parsing
INSTALL_MODE="dev"
COMPUTE_MODE="cpu"
ARCH_OVERRIDE="auto"

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --install-mode)
      INSTALL_MODE="$2"
      shift
      ;;
    --compute-mode)
      COMPUTE_MODE="$2"
      shift
      ;;
    --arch-mode)
      ARCH_OVERRIDE="$2"
      shift
      ;;
    *)
      error "Unknown parameter passed: $1"
      ;;
  esac
  shift
done

info "Starting minimal environment installation with:"
info "  INSTALL_MODE: ${INSTALL_MODE}"
info "  COMPUTE_MODE: ${COMPUTE_MODE}"
info "  ARCH_OVERRIDE: ${ARCH_OVERRIDE}"

# Architecture detection
CURRENT_ARCH="$(uname -m)"
TARGET_ARCH="${ARCH_OVERRIDE}"

if [[ "${TARGET_ARCH}" == "auto" ]]; then
  TARGET_ARCH="${CURRENT_ARCH}"
  info "Auto-detected architecture: ${TARGET_ARCH}"
fi

if [[ "${TARGET_ARCH}" != "x86_64" && "${TARGET_ARCH}" != "aarch64" ]]; then
  error "Unsupported architecture detected or specified: ${TARGET_ARCH}. Must be x86_64 or aarch64."
fi

# Base dependencies (all modes)
info "--- Installing base system dependencies ---"
apt_get_update_and_install \
  apt-utils \
  bc \
  curl \
  file \
  gawk \
  less \
  lsof \
  sed \
  software-properties-common \
  sudo \
  unzip \
  vim \
  wget \
  zip \
  xz-utils \
  zlib1g-dev # bazel deps

# Architecture-specific dependencies
if [[ "${TARGET_ARCH}" == "aarch64" ]]; then
  info "--- Installing aarch64 specific dependencies ---"
  apt_get_update_and_install kmod
fi

# Dev/build tools (dev mode only)
if [[ "${INSTALL_MODE}" == "dev" ]]; then
  info "--- Installing development/build tools (dev mode) ---"
  apt_get_update_and_install \
    build-essential \
    autoconf \
    automake \
    gdb \
    libtool \
    patch \
    pkg-config \
    linux-libc-dev
else
  info "--- Skipping development/build tools (runtime mode) ---"
fi

# Sudo configuration
info "--- Configuring Sudo: Allow sudo group to execute commands without password ---"
sudoers_file="/etc/sudoers"
if ! grep -q "^%sudo ALL=(ALL:ALL) NOPASSWD: ALL$" "${sudoers_file}"; then
  sed -i -e '/^%sudo.*/d' -e '$a\%sudo ALL=(ALL:ALL) NOPASSWD: ALL' "${sudoers_file}" ||
    error "Failed to configure sudoers. Manual intervention may be required."
  info "Sudoers configured for NOPASSWD for sudo group."
else
  info "Sudoers already configured for NOPASSWD for sudo group."
fi

# link Python 3 to /usr/bin/python
sudo ln -s /usr/bin/python3 /usr/bin/python

# Default shell configuration
info "--- Setting default shell to Bash ---"
chsh -s /bin/bash || warning "Failed to change default shell for current user to /bin/bash."

if [[ -L /bin/sh && "$(readlink /bin/sh)" != "/bin/bash" ]]; then
  warning "Symbolic link /bin/sh points to $(readlink /bin/sh). Recreating symlink to /bin/bash."
  rm /bin/sh || error "Failed to remove existing /bin/sh symlink."
  ln -sf /bin/bash /bin/sh || error "Failed to create /bin/sh symlink to /bin/bash."
elif [[ ! -L /bin/sh ]]; then
  info "/bin/sh is not a symlink. Creating symlink to /bin/bash."
  ln -sf /bin/bash /bin/sh || error "Failed to create /bin/sh symlink to /bin/bash."
else
  info "/bin/sh already points to /bin/bash."
fi

# Cleanup
info "--- Performing minimal cleanup ---"
apt-get clean || warning "Failed to clean apt cache."
rm -rf /var/lib/apt/lists/* || warning "Failed to remove apt lists."
rm -rf /tmp/* /var/tmp/* || warning "Failed to remove temporary files."

info "Minimal environment installation completed successfully."
