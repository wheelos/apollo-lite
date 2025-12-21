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
  info "Installing LibTorch C++ ${PYTORCH_VERSION}..."
  local BASE_URL="https://download.pytorch.org/libtorch"
  local ARCHIVE=""
  local URL=""

  if [ "${TARGET_ARCH}" = "x86_64" ]; then
    if [ "$CUDA_SUPPORT" = true ]; then
      ARCHIVE="libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}+${CUDA_VERSION_TAG}.zip"
      CHECKSUM='36835d6c6315d741ad687632516f7bcd8efb6de3b57b61ca66b96f98e5ea30e8'
      URL="${BASE_URL}/${CUDA_VERSION_TAG}/${ARCHIVE}"
    else
      ARCHIVE="libtorch-cxx11-abi-shared-with-deps-${PYTORCH_VERSION}%2Bcpu.zip"
      CHECKSUM="6887b5186e466a6d5ca044a51d083bb03c48cb1b4952059b7ca51a5398fbafcc"
      URL="${BASE_URL}/cpu/${ARCHIVE}"
    fi
  elif [ "${TARGET_ARCH}" = "aarch64" ]; then
    # WARNING: Official pre-compiled LibTorch C++ package is not available for aarch64.
    # If needed, you must build from source. Skipping installation here.
    warning "Official pre-compiled LibTorch C++ is not available for aarch64."
    warning "We default install our own compiled for orin(l4t r35.2)"
    warning "If not suitable, please build from source yourself."

    # TODO(build): Docs on how to build libtorch on Jetson boards
    # References:
    #   https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
    #
    # Following content is an example to build libtorch source on orin:
    # 1. Downloading source of specific version libtorch and it's submodules:
    #     git clone --recursive --single-branch --branch release/1.11 --depth 1 https://github.com/pytorch/pytorch.git
    # 2. install py deps:
    #     pip3 install --no-cache-dir PyYAML typing typing-extensions
    # 3. set envs, e.g.:
    #     export USE_CUDA=1
    #     export TORCH_CUDA_ARCH_LIST="3.5;5.0;5.2;6.1;7.0;7.5;8.6;8.7"
    #     export BUILD_CAFFE2=1
    #     export USE_NCCL=0
    # 4. build
    #     python3 setup.py install
    # 5. packaging/install
    #     mkdir libtorch && cp -r include libtorch && cp -r lib libtorch/ && sudo mv libtorch /usr/local/

    # download the original pre-compiled libtorch for aarch64
    local pkg_version='1.11.0'
    local pkg_filename="libtorch_gpu-${pkg_version}-linux-${TARGET_ARCH}.tar.gz"
    local pkg_uri="https://apollo-pkg-beta.cdn.bcebos.com/archive/${pkg_filename}"
    local pkg_checksum='8413dd02c08fe7d0384e4ac0a66449f22c8fbe99f252015d6020174e8ecd5acf'
    download_if_not_cached "${pkg_filename}" "${pkg_checksum}" "${pkg_uri}"
    mkdir -p /usr/local/libtorch
    tar -xzf "${pkg_filename}" -C /usr/local/libtorch --strip-components=1

    # Add runtime library path
    ensure_ld_path "/usr/local/libtorch/lib"

    rm -rf "${pkg_filename}"

    ldconfig

    ok "LibTorch C++ pre-compiled package for jetson installed successfully."

    return 0 # Exit normally
  else
    error "Unsupported architecture: ${TARGET_ARCH}"
    return 1
  fi

  local DOWNLOAD_DIR="/tmp/libtorch_download"
  mkdir -p "${DOWNLOAD_DIR}"
  pushd "${DOWNLOAD_DIR}" > /dev/null

  info "Downloading from ${URL}"
  download_if_not_cached "${ARCHIVE}" "${CHECKSUM}" "${URL}"
  unzip -q "${ARCHIVE}"

  # Install to system path
  INSTALL_DIR="/usr/local/libtorch"
  info "Installing LibTorch to ${INSTALL_DIR}..."
  rm -rf "${INSTALL_DIR}" # Remove old version
  mkdir -p "${INSTALL_DIR}"
  # Use mv instead of cp -r for better efficiency
  mv libtorch/* "${INSTALL_DIR}/"

  # Add runtime library path
  ensure_ld_path "${INSTALL_DIR}/lib"

  # Clean up downloaded files
  popd > /dev/null
  rm -rf "${DOWNLOAD_DIR}"

  # Optionally: update dynamic linker cache
  ldconfig

  ok "LibTorch C++ ${PYTORCH_VERSION} installed successfully."
}

# --- Main Execution Flow ---
main() {
  # TODO(daohu527): For inference, no python version is required
  # install_pytorch_python
  install_libtorch_cpp
  info "✅ All PyTorch components have been installed."
}

main "$@"
