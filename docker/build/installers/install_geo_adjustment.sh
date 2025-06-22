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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

TARGET_ARCH="$(uname -m)"
MY_GEO=$1; shift

info "My geolocation is ${MY_GEO}"

###
# Function to configure APT sources
# Uses geo and architecture to select the appropriate sources.list file.
# Prioritizes file-based configuration for consistency and clarity.
###
configure_apt_sources() {
    local geo=$1
    local arch=$2
    local sources_file="${RCFILES_DIR}/sources.list.${geo}.${arch}"
    local default_sources_file="${RCFILES_DIR}/sources.list.default.${arch}" # Add a default sources file

    info "Attempting to configure APT sources for ${geo^^} (${arch})..."

    if [[ -f "${sources_file}" ]]; then
        cp -f "${sources_file}" /etc/apt/sources.list
        info "Successfully configured APT sources for ${geo^^} (${arch}) from file: ${sources_file}"
    else
        error "No specific or default APT sources file found for ${geo^^} (${arch}). Please check ${RCFILES_DIR}."
    fi
}

###
# Function to configure PyPI mirror
# Sets the global index-url for pip and upgrades pip using the configured mirror.
# Handles both 'cn' for Tsinghua and 'us' for the official PyPI.
###
configure_pypi_mirror() {
    local geo=$1
    local pypi_mirror

    if [[ "${geo}" == "cn" ]]; then
        pypi_mirror="https://pypi.tuna.tsinghua.edu.cn/simple"
        info "Setting PyPI mirror to Tsinghua: ${pypi_mirror}"
    else # us or any other geo defaults to official
        pypi_mirror="https://pypi.org/simple"
        info "Setting PyPI mirror to official PyPI: ${pypi_mirror}"
    fi

    # Set the global index-url for pip
    python3 -m pip config set global.index-url "$pypi_mirror"
}

# Configure APT sources based on geolocation and architecture
configure_apt_sources "${MY_GEO}" "${TARGET_ARCH}"

# Configure PyPI mirror based on geolocation
configure_pypi_mirror "${MY_GEO}"
