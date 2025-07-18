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

# Fail on first error, unset variables are errors, print commands and their arguments as they are executed.
set -eux

# Navigate to the script's directory and source the base installer functions.
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

# Define CMake version
CMAKE_VERSION="3.19.6"

info "Starting CMake installation (version ${CMAKE_VERSION})..."

# Determine target architecture.
TARGET_ARCH="$(uname -m)"

# ============================
# ARCHITECTURE-SPECIFIC CONFIGURATION
# ============================
CMAKE_SH_FILE=""
CMAKE_CHECKSUM=""

case "${TARGET_ARCH}" in
    "x86_64")
        CMAKE_SH_FILE="cmake-${CMAKE_VERSION}-Linux-x86_64.sh"
        CMAKE_CHECKSUM="d94155cef56ff88977306653a33e50123bb49885cd085edd471d70dfdfc4f859"
        ;;
    "aarch64")
        CMAKE_SH_FILE="cmake-${CMAKE_VERSION}-Linux-aarch64.sh"
        CMAKE_CHECKSUM="f383c2ef96e5de47c0a55957e9af0bdfcf99d3988c17103767c9ef1b3cd8c0a9"
        ;;
    *)
        error "Unsupported architecture for CMake installation: ${TARGET_ARCH}. Please add its configuration."
        ;;
esac

DOWNLOAD_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${CMAKE_SH_FILE}"

# ============================
# INSTALLATION PROCESS
# ============================

# Define installation directory based on SYSROOT_DIR
# Assuming SYSROOT_DIR is defined in installer_base.sh, e.g., /opt/apollo/sysroot
CMAKE_INSTALL_DIR="${SYSROOT_DIR}/cmake-${CMAKE_VERSION}"
CMAKE_BIN_PATH="${CMAKE_INSTALL_DIR}/bin/cmake"

# Create the installation directory if it does not exist, already created by installer_base.sh, here we confirm again
if [[ ! -d "${SYSROOT_DIR}" ]]; then
  mkdir -p "${SYSROOT_DIR}" || error "Failed to create directory: ${SYSROOT_DIR}"
fi

# Check if CMake is already installed at the target location
if [[ -f "${CMAKE_BIN_PATH}" ]]; then
    info "CMake version ${CMAKE_VERSION} for ${TARGET_ARCH} is already installed at ${CMAKE_INSTALL_DIR}. Skipping download and installation."
else
    info "Downloading CMake installer: ${CMAKE_SH_FILE} from ${DOWNLOAD_URL}..."
    download_if_not_cached "${CMAKE_SH_FILE}" "${CMAKE_CHECKSUM}" "${DOWNLOAD_URL}"

    info "Installing CMake to ${CMAKE_INSTALL_DIR}..."
    chmod a+x "${CMAKE_SH_FILE}" || error "Failed to make CMake installer executable."

    # Run the installer. --skip-license --prefix="${SYSROOT_DIR}"
    # Note: If SYSROOT_DIR is /opt/apollo/sysroot, CMake will install to /opt/apollo/sysroot/cmake-<VERSION>
    # The --prefix argument makes the installer extract to a specific path.
    ./"${CMAKE_SH_FILE}" --skip-license --prefix="${SYSROOT_DIR}" || error "Failed to install CMake."

    # Clean up the installer script
    info "Cleaning up CMake installer script..."
    rm -f "${CMAKE_SH_FILE}" || warn "Failed to remove CMake installer script: ${CMAKE_SH_FILE}."

    info "CMake version ${CMAKE_VERSION} installed successfully."
fi

# ============================
# SYMLINK MANAGEMENT
# ============================

###
# Function to create or update a symbolic link to CMake in /usr/local/bin
# This function ensures that the system-wide CMake link points to the newly installed version.
###
link_cmake_to_system_path() {
    local system_cmake_link="/usr/local/bin/cmake"
    local installed_cmake_bin="${CMAKE_BIN_PATH}" # Full path to the installed CMake binary

    if [[ -L "${system_cmake_link}" ]]; then
        # Check if the existing symlink points to the correct version
        local current_target="$(readlink "${system_cmake_link}")"
        if [[ "${current_target}" == "${installed_cmake_bin}" ]]; then
            info "System-wide CMake symlink ${system_cmake_link} already points to ${installed_cmake_bin}."
        else
            warn "Existing symlink ${system_cmake_link} points to ${current_target}. Updating to ${installed_cmake_bin}."
            rm "${system_cmake_link}" || error "Failed to remove old symlink ${system_cmake_link}."
            ln -sf "${installed_cmake_bin}" "${system_cmake_link}" || error "Failed to create symlink ${system_cmake_link}."
        fi
    elif [[ -e "${system_cmake_link}" ]]; then
        # It's a regular file or directory, not a symlink. This is unusual.
        warn "${system_cmake_link} exists but is not a symlink. Skipping automatic linking. Manual intervention may be required."
    else
        info "Creating system-wide CMake symlink: ${system_cmake_link} -> ${installed_cmake_bin}."
        ln -sf "${installed_cmake_bin}" "${system_cmake_link}" || error "Failed to create symlink ${system_cmake_link}."
    fi
}

# Call the symlink function
link_cmake_to_system_path

info "CMake installation script finished."
