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

# Exit on error, treat unset variables as errors, print commands as they execute.
set -eux

# Change to script directory and source base installer functions.
SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "${SCRIPT_DIR}"
. ./installer_base.sh

# Directory containing rcfiles (e.g., sources.list variants).
RCFILES_DIR="/opt/apollo/rcfiles"

# ============================
# Parse and validate arguments
# ============================
# Get geolocation parameter from first argument.
MY_GEO="${1:-}"
if [[ -z "${MY_GEO}" ]]; then
  error "Error: Geolocation parameter (MY_GEO) is required."
  exit 1
fi
MY_GEO="${MY_GEO,,}"
case "${MY_GEO}" in
  cn|us)
    ;;
  *)
    error "Unsupported geolocation '${MY_GEO}'; expected cn or us."
    exit 1
    ;;
esac

info "Starting geo-adjustment for geolocation: ${MY_GEO^^}"

# Detect system architecture (e.g., x86_64, aarch64).
TARGET_ARCH="$(uname -m)"
info "Detected system architecture: ${TARGET_ARCH}"

# ============================
# APT source configuration
# ============================
# Select the APT sources.list file matching geolocation, architecture, and OS.
configure_apt_sources() {
  local geo="$1"
  local arch="$2"
  local os_id="$(awk -F= '/^ID=/{print $2}' /etc/os-release | tr -d '"')"
  local os_version="$(awk -F= '/^VERSION_ID=/{print $2}' /etc/os-release | tr -d '"')"
  # Specific sources file for geo, arch, os_id, and os_version
  local sources_list_file="/etc/apt/sources.list"

  local preferred_source="${RCFILES_DIR}/sources.list.${geo}.${arch}.${os_id}.${os_version}"

  info "Configuring APT sources for ${geo^^} (${arch})..."

  if [[ -f "${preferred_source}" ]]; then
    info "Using APT sources file: ${preferred_source}"
    install -m 0644 "${preferred_source}" "${sources_list_file}" ||
      error "Failed to copy APT sources file '${preferred_source}' to '${sources_list_file}'."
  else
    warning "No suitable APT sources file found for ${geo^^} (${arch})."
    if [[ "${geo,,}" == "cn" ]]; then
      error "A Tsinghua APT source is required for China builds."
      exit 1
    fi
    warning "APT sources remain unchanged; downloads may be slow."
    # Uncomment below to enforce sources file requirement:
    # error "Critical: No suitable APT sources file found. Exiting."
    # exit 1
  fi
}

# ============================
# PyPI mirror configuration
# ============================
# Set pip global index-url to a mirror based on geolocation.
# Upgrades pip using the selected mirror.
configure_pypi_mirror() {
  apt_get_update_and_install python3 python3-pip python3-dev

  local geo="${1,,}"
  local pypi_mirror

  case "${geo}" in
    cn)
      pypi_mirror="https://pypi.tuna.tsinghua.edu.cn/simple"
      info "Setting PyPI mirror to Tsinghua: ${pypi_mirror}"
      ;;
    us | "")
      pypi_mirror="https://pypi.org/simple"
      info "Setting PyPI mirror to official PyPI: ${pypi_mirror}"
      ;;
    *)
      pypi_mirror="https://pypi.org/simple"
      warning "Unsupported geolocation '${geo}' for PyPI mirror. Defaulting to official PyPI: ${pypi_mirror}"
      ;;
  esac

  info "Upgrading pip using the configured mirror..."

  if ! python3 -m pip config set global.index-url "${pypi_mirror}"; then
    error "Failed to configure PyPI index-url."
  fi

  pip3_install -U setuptools || warning "Failed to upgrade setuptools."
  pip3_install -U wheel || warning "Failed to upgrade wheel."

  # Minimal cleanup
  info "--- Performing minimal cleanup ---"
  apt-get clean || warning "Failed to clean apt cache."
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* || warning "Failed to remove temporary files."
}

# Configure JavaScript package registries when those tools are present.
configure_npm_mirrors() {
  local geo="$1"
  local npm_registry

  case "${geo}" in
    cn)
      npm_registry="https://registry.npmmirror.com"
      ;;
    us)
      npm_registry="https://registry.npmjs.org"
      ;;
    *)
      error "Unsupported geolocation '${geo}' for npm registry."
      return 1
      ;;
  esac

  if command -v npm >/dev/null 2>&1; then
    info "Setting npm registry: ${npm_registry}"
    npm config set registry "${npm_registry}"
  fi

  if command -v yarn >/dev/null 2>&1; then
    info "Setting Yarn registry: ${npm_registry}"
    yarn config set registry "${npm_registry}"
  fi
}

# ============================
# Execute configuration steps
# ============================
configure_apt_sources "${MY_GEO}" "${TARGET_ARCH}"
configure_pypi_mirror "${MY_GEO}"
configure_npm_mirrors "${MY_GEO}"

info "Geolocation adjustment script finished."
