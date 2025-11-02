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

set -e

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

# TODO(All): detect if the installed version consists with the required version
if [[ -e '/usr/local/libtorch/lib/libtorch.so' ]]; then
  warning "LibTorch already installed, re-installation skipped."
  exit 0
fi

# --- Unified Version Control ---
# Manage all Pytorch related component versions here
PYTORCH_VERSION="2.6.0"
TORCHVISION_VERSION="0.18.0" # Note: Version must be compatible with Pytorch
TORCHAUDIO_VERSION="2.4.0"  # Note: Version must be compatible with Pytorch

# --- Environment Detection ---
TARGET_ARCH="$(uname -m)"
CUDA_SUPPORT=false
CUDA_VERSION_STR="" # e.g., "11.8"
CUDA_VERSION_TAG="" # e.g., "cu118"

if [ "${TARGET_ARCH}" = "x86_64" ] && command -v nvcc >/dev/null 2>&1; then
  # Get CUDA major and minor version (e.g., 11.8)
  CUDA_VERSION_STR=$(nvcc --version | sed -n 's/.*release \([0-9]*\.[0-9]*\).*/\1/p')
  if [ -n "${CUDA_VERSION_STR}" ]; then
    # Format version for Pytorch whl URL (e.g., 11.8 -> cu118)
    CUDA_VERSION_TAG="cu$(echo ${CUDA_VERSION_STR} | sed 's/\.//g')"
    CUDA_SUPPORT=true
    ok "Found CUDA ${CUDA_VERSION_STR}. PyTorch will be installed with GPU support."
  else
    warning "nvcc found, but could not determine CUDA version. Falling back to CPU."
  fi
fi

# --- Python PyTorch Installation ---
function install_pytorch_python() {
  info "Installing Python PyTorch ${PYTORCH_VERSION}..."

  local INDEX_URL_FLAG=""
  if [ "$CUDA_SUPPORT" = true ]; then
    INDEX_URL_FLAG="--index-url https://download.pytorch.org/whl/${CUDA_VERSION_TAG}"
  else
    INDEX_URL_FLAG="--index-url https://download.pytorch.org/whl/cpu"
  fi

  pip3 install \
    torch==${PYTORCH_VERSION} \
    torchvision==${TORCHVISION_VERSION} \
    torchaudio==${TORCHAUDIO_VERSION} \
    ${INDEX_URL_FLAG}

  # Verify installation
  info "Verifying Python PyTorch installation..."
  python3 -c "
import torch
print(f'PyTorch Version: {torch.__version__}')
print(f'CUDA Available: {torch.cuda.is_available()}')
if torch.cuda.is_available():
  print(f'CUDA Version: {torch.version.cuda}')
  print(f'GPU Name: {torch.cuda.get_device_name(0)}')
"
  ok "Python PyTorch installation successful."
}

# --- C++ LibTorch Installation ---
function install_libtorch_cpp() {
  if [ -z "${PYTORCH_VERSION}" ]; then
    error "PYTORCH_VERSION is not set. Please export PYTORCH_VERSION before calling this function."
    return 1
  fi

  info "Installing LibTorch C++ ${PYTORCH_VERSION}..."
  local BASE_URL="https://download.pytorch.org/libtorch"
  local ARCHIVE=""
  local URL=""
  local INSTALL_DIR="/usr/local/libtorch"

  # x86_64: Official pre-built libtorch
  if [ "${TARGET_ARCH}" = "x86_64" ]; then
    info "Detected x86_64 architecture. Installing official precompiled package."

    if [ "$CUDA_SUPPORT" = true ]; then
      ARCHIVE="libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}+${CUDA_VERSION_TAG}.zip"
      URL="${BASE_URL}/${CUDA_VERSION_TAG}/${ARCHIVE}"
    else
      ARCHIVE="libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip"
      URL="${BASE_URL}/cpu/${ARCHIVE}"
    fi

    local DOWNLOAD_DIR="/tmp/libtorch_download"
    mkdir -p "${DOWNLOAD_DIR}"
    pushd "${DOWNLOAD_DIR}" > /dev/null

    info "Downloading LibTorch from ${URL}"
    if ! wget -q "${URL}" -O "${ARCHIVE}"; then
        error "Download failed: ${URL}"
        return 1
    fi

    unzip -q "${ARCHIVE}"

    info "Installing to ${INSTALL_DIR}..."
    sudo rm -rf "${INSTALL_DIR}"
    sudo mkdir -p "${INSTALL_DIR}"
    sudo mv libtorch/* "${INSTALL_DIR}/"

    popd > /dev/null
    rm -rf "${DOWNLOAD_DIR}"

  # aarch64: Build libtorch from source
  elif [ "${TARGET_ARCH}" = "aarch64" ]; then
    info "Detected aarch64 (Jetson) architecture. Building LibTorch from source..."

    # 0. Pre-check CUDA/cuDNN/TensorRT presence
    if [ ! -f /usr/include/cudnn.h ]; then
        warning "cuDNN header not found. Please install libcudnn9-dev before building."
    fi
    if [ ! -f /usr/include/NvInfer.h ]; then
        warning "TensorRT headers not found. Please ensure TensorRT 10.3 is installed."
    fi

    # 1. Install system dependencies
    info "Installing build dependencies..."
    apt-get update
    apt-get install -y --no-install-recommends \
        git cmake ninja-build build-essential \
        python3-dev python3-pip python3-setuptools python3-wheel \
        libopenblas-dev libomp-dev libjpeg-dev zlib1g-dev libffi-dev
    pip3 install --no-cache-dir numpy pyyaml typing_extensions sympy filelock
    apt-get clean && rm -rf /var/lib/apt/lists/*

    # 2. Clone PyTorch source
    local BUILD_DIR="/tmp/pytorch_build"
    mkdir -p "${BUILD_DIR}"
    pushd "${BUILD_DIR}" > /dev/null
    info "Cloning PyTorch source for version v${PYTORCH_VERSION}"
    git clone --recursive --single-branch --branch "v${PYTORCH_VERSION}" https://github.com/pytorch/pytorch.git
    cd pytorch

    # 3. Configure environment for Jetson Orin
    info "Configuring build environment (CUDA 12.6, cuDNN 9.3, TensorRT 10.3)..."
    export USE_CUDA=1
    export USE_CUDNN=1
    export USE_TENSORRT=1
    export TORCH_CUDA_ARCH_LIST="8.7"
    export USE_NCCL=0
    export USE_DISTRIBUTED=0
    export USE_QNNPACK=1
    export USE_PYTORCH_QNNPACK=1
    export BUILD_TEST=0
    export CMAKE_GENERATOR="Ninja"
    export CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)

    info "Starting PyTorch C++ library build..."
    info "TIP: Ensure swapfile (>=8GB) is enabled to prevent OOM during the build."
    python3 setup.py bdist_wheel > /tmp/pytorch_build.log 2>&1 || {
        error "Build failed. Check /tmp/pytorch_build.log for details."
        return 1
    }
    ok "PyTorch build completed."
    pip3 install dist/*.whl

    # 4. Copy libtorch headers and libs to /usr/local/libtorch
    info "Installing compiled LibTorch to ${INSTALL_DIR}..."
    sudo rm -rf "${INSTALL_DIR}"
    sudo mkdir -p "${INSTALL_DIR}"

    local PYTORCH_SITE=$(python3 -c "import torch; import os; print(os.path.dirname(torch.__file__))")
    sudo cp -r ${PYTORCH_SITE}/include "${INSTALL_DIR}"
    sudo cp -r ${PYTORCH_SITE}/lib "${INSTALL_DIR}"

    popd > /dev/null
    info "Cleaning up build directory..."
    rm -rf "${BUILD_DIR}"

  else
    error "Unsupported architecture: ${TARGET_ARCH}"
    return 1
  fi

  # -------------------- Post-install steps (all arches) ---------------------
  if [ -d "${INSTALL_DIR}/lib" ]; then
    ensure_ld_path "${INSTALL_DIR}/lib"
    ldconfig
    ok "LibTorch C++ ${PYTORCH_VERSION} installed successfully at ${INSTALL_DIR}."
  else
    error "LibTorch installation failed — ${INSTALL_DIR}/lib directory missing."
    return 1
  fi

  # Verify installation
  if [ -f "${INSTALL_DIR}/lib/libtorch.so" ]; then
    info "libtorch.so located successfully in ${INSTALL_DIR}/lib"
  else
    warning "libtorch.so not found — verify your build output."
  fi
}

# --- Main Execution Flow ---
main() {
  # TODO(daohu527): For inference, no python version is required
  # install_pytorch_python
  install_libtorch_cpp
  info "✅ All PyTorch components have been installed."
}

main "$@"
