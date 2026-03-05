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

set -euo pipefail

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# TODO(All): detect if the installed version consists with the required version
if [[ -e '/usr/local/libtorch/lib/libtorch.so' ]]; then
  warning "LibTorch already installed, re-installation skipped."
  exit 0
fi

PYTORCH_VERSION="2.6.0"
TARGET_ARCH="$(uname -m)"
CUDA_SUPPORT=false
CUDA_VERSION_STR=""
CUDA_VERSION_TAG=""

if [ "${TARGET_ARCH}" = "x86_64" ] && command -v nvcc >/dev/null 2>&1; then
  CUDA_VERSION_STR=$(nvcc --version | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p')
  if [ -n "${CUDA_VERSION_STR}" ]; then
    CUDA_VERSION_TAG="cu$(echo ${CUDA_VERSION_STR} | sed 's/\.//g')"
    CUDA_SUPPORT=true
    ok "Found CUDA ${CUDA_VERSION_STR} on x86_64. PyTorch will be installed with GPU support."
  else
    warning "nvcc found, but CUDA version could not be determined. Falling back to CPU-only."
  fi
fi

# --- C++ LibTorch Installation  ---
function install_libtorch_cpp() {
  local INSTALL_DIR="/usr/local/libtorch"
  info "Starting LibTorch C++ ${PYTORCH_VERSION} installation for ${TARGET_ARCH}..."

  # ============================================================================
  # x86_64 Strategy: Download official pre-compiled .zip via caching function
  # ============================================================================
  if [ "${TARGET_ARCH}" = "x86_64" ]; then
    info "Executing x86_64 strategy: Downloading official package via cache."
    local BASE_URL="https://download.pytorch.org/libtorch"
    local PKG_NAME="" URL="" CHECKSUM=""

    if [ "$CUDA_SUPPORT" = true ]; then
      PKG_NAME="libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}+${CUDA_VERSION_TAG}.zip"
      URL="${BASE_URL}/${CUDA_VERSION_TAG}/${PKG_NAME}"
      # Example: CHECKSUM="419dba362eaf8f1d36849ceee17c3e2ff8ff12ac666b42d3ff02a164ebe090e9"
      CHECKSUM="TODO_ADD_SHA256_CHECKSUM_FOR_${PKG_NAME}"
    else
      PKG_NAME="libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip"
      URL="${BASE_URL}/cpu/${PKG_NAME}"
      CHECKSUM="TODO_ADD_SHA256_CHECKSUM_FOR_CPU_LIBTORCH"
    fi

    if [[ "${CHECKSUM}" == TODO_* ]]; then
        error "Checksum for ${PKG_NAME} is not set. Please edit the script and add the correct SHA256 value."
        return 1
    fi

    local DOWNLOAD_DIR="/tmp/libtorch_download"
    mkdir -p "${DOWNLOAD_DIR}"; pushd "${DOWNLOAD_DIR}" > /dev/null

    info "Downloading ${PKG_NAME} via cache-aware function..."
    if ! download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${URL}"; then
        error "Download failed for ${PKG_NAME}. Please check URL, checksum, and network."
        popd >/dev/null; rm -rf "${DOWNLOAD_DIR}"; return 1
    fi

    info "Extracting archive..."
    unzip -q "${PKG_NAME}"

    info "Installing to ${INSTALL_DIR}..."
    sudo rm -rf "${INSTALL_DIR}"
    sudo mv libtorch "${INSTALL_DIR}"

    popd >/dev/null; rm -rf "${DOWNLOAD_DIR}"

  # ============================================================================
  # aarch64 Strategy: Prioritize pre-compiled wheel, fallback to source build
  # ============================================================================
  elif [ "${TARGET_ARCH}" = "aarch64" ]; then
    info "Executing aarch64 strategy..."
    if ! _install_libtorch_from_wheel_aarch64; then
      warning "Pre-compiled wheel installation failed. Falling back to building from source."
      if ! _build_libtorch_from_source_aarch64; then
        error "LibTorch installation failed after both attempts."
        return 1
      fi
    fi
    # Common step for aarch64: Copy from Python site-packages
    info "Copying LibTorch C++ headers and libraries to final destination..."
    local PYTORCH_SITE
    PYTORCH_SITE=$(python3 -c "import torch, os; print(os.path.dirname(torch.__file__))")
    if [ -z "${PYTORCH_SITE}" ]; then
      error "Could not determine PyTorch installation location. Aborting."; return 1
    fi
    sudo rm -rf "${INSTALL_DIR}"; sudo mkdir -p "${INSTALL_DIR}"
    sudo cp -r "${PYTORCH_SITE}/include" "${INSTALL_DIR}/"
    sudo cp -r "${PYTORCH_SITE}/lib" "${INSTALL_DIR}/"

  else
    error "Unsupported architecture: ${TARGET_ARCH}"; return 1
  fi

  # ============================================================================
  # Post-install steps for all architectures
  # ============================================================================
  if [ -d "${INSTALL_DIR}/lib" ]; then
    ensure_ld_path "${INSTALL_DIR}/lib" # This function is in installer_base.sh
    sudo ldconfig
    ok "LibTorch C++ ${PYTORCH_VERSION} installed successfully at ${INSTALL_DIR}."
  else
    error "Installation failed — ${INSTALL_DIR}/lib directory missing."; return 1
  fi

  if [ -f "${INSTALL_DIR}/lib/libtorch.so" ]; then
    info "libtorch.so located successfully."
  else
    warning "libtorch.so not found — verify your build output."
  fi
}

# --- Helper: Install aarch64 from pre-compiled wheel via cache ---
function _install_libtorch_from_wheel_aarch64() {
  local PYTORCH_WHL_VERSION="2.6.0a0+git1eba9b3"
  local PKG_NAME="torch-${PYTORCH_WHL_VERSION}-cp310-cp310-linux_aarch64.whl"
  local DOWNLOAD_LINK="http://10.0.39.103:8080/build/aarch64/${PKG_NAME}"

  # IMPORTANT: Add the real SHA256 checksum for your wheel file.
  # Run this on your server: sha256sum torch-2.6.0a0+git1eba9b3-cp310-cp310-linux_aarch64.whl
  local CHECKSUM="828ffcf5b4185eef382b0f1e9bd479201c4cae1bb16abd84f45195cdb4eed205"

  info "Attempt 1/2: Trying to install from pre-compiled aarch64 wheel..."

  if [[ "${CHECKSUM}" == TODO_* ]]; then
      error "Checksum for ${PKG_NAME} is not set. Please edit the script."
      return 1
  fi

  local DOWNLOAD_DIR="/tmp/pytorch_wheel"
  mkdir -p "${DOWNLOAD_DIR}"; pushd "${DOWNLOAD_DIR}" > /dev/null

  if ! download_if_not_cached "${PKG_NAME}" "${CHECKSUM}" "${DOWNLOAD_LINK}"; then
      warning "Could not download pre-compiled wheel from ${DOWNLOAD_LINK}."
      popd >/dev/null; rm -rf "${DOWNLOAD_DIR}"; return 1
  fi

  info "Installing downloaded wheel via pip..."
  if ! pip3 install --no-cache-dir "${PKG_NAME}"; then
      error "pip3 install failed for the downloaded wheel."; popd >/dev/null; rm -rf "${DOWNLOAD_DIR}"; return 1
  fi

  ok "Pre-compiled wheel installed successfully."
  popd >/dev/null; rm -rf "${DOWNLOAD_DIR}"; return 0 # Success
}

# --- Helper: Build aarch64 from source (Fallback) ---
function _build_libtorch_from_source_aarch64() {
  info "Attempt 2/2: Building LibTorch from source. This will take a very long time."
  # Install build dependencies
  info "Installing source build dependencies..."
  sudo apt-get update && sudo apt-get install -y --no-install-recommends \
      git cmake ninja-build build-essential python3-dev python3-pip python3-setuptools \
      python3-wheel libopenblas-dev libomp-dev libjpeg-dev zlib1g-dev libffi-dev \
      && pip3 install --no-cache-dir numpy pyyaml typing_extensions sympy filelock \
      && sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

  # Clone PyTorch source
  local BUILD_DIR="/tmp/pytorch_build"
  rm -rf "${BUILD_DIR}"; mkdir -p "${BUILD_DIR}"; pushd "${BUILD_DIR}" > /dev/null
  info "Cloning PyTorch source v${PYTORCH_VERSION}..."
  git clone --recursive --single-branch --branch "v${PYTORCH_VERSION}" https://github.com/pytorch/pytorch.git
  cd pytorch

  # Configure and build
  info "Configuring build environment for Jetson..."
  export USE_CUDA=1 USE_CUDNN=1 USE_TENSORRT=1 TORCH_CUDA_ARCH_LIST="8.7"
  export USE_NCCL=0 USE_DISTRIBUTED=0 USE_QNNPACK=1 USE_PYTORCH_QNNPACK=1
  export BUILD_TEST=0 CMAKE_GENERATOR="Ninja" CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)

  info "Starting PyTorch wheel build. Log: /tmp/pytorch_build.log"
  if ! python3 setup.py bdist_wheel > /tmp/pytorch_build.log 2>&1; then
    error "Build from source failed. Check /tmp/pytorch_build.log"; popd >/dev/null; rm -rf "${BUILD_DIR}"; return 1
  fi

  ok "PyTorch source build completed. Installing the new wheel..."
  if ! pip3 install --no-cache-dir dist/*.whl; then
      error "pip3 install failed for the newly built wheel."; return 1
  fi

  popd >/dev/null; rm -rf "${BUILD_DIR}"; return 0 # Success
}

# --- Main Execution Flow ---
main() {
  # As per original script, Python installation is commented out by default.
  # install_pytorch_python
  install_libtorch_cpp
  info "✅ LibTorch installation process finished."
}

main "$@"
