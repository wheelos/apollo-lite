#!/bin/bash

# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

set -euo pipefail

ARCH="$(uname -m)"

# --- Color definitions and logging functions ---
BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
WHITE='\033[34m'
NO_COLOR='\033[0m'

function info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function success() {
  (echo >&2 -e "[${GREEN}${BOLD}SUCCESS${NO_COLOR}] $*")
}

function error() {
  (echo >&2 -e "[${RED}${BOLD}ERROR${NO_COLOR}] $*")
}

# --- Precondition check functions (keeping generic checks to not affect strict install steps) ---

function check_sudo() {
  if ! command -v sudo &> /dev/null; then
    error "sudo command not found. Please install sudo or run this script as root."
    exit 1
  fi
}

function check_arch() {
  info "Checking system architecture..."
  if [ "${ARCH}" == "x86_64" ]; then
    info "Detected architecture: x86_64 (amd64)"
  elif [ "${ARCH}" == "aarch64" ]; then
    info "Detected architecture: aarch64 (arm64)"
  else
    error "This script does not support ${ARCH} architecture. NVIDIA Container Toolkit only supports x86_64 and aarch64."
    exit 1
  fi
}

function check_os() {
  info "Checking operating system type (Ubuntu only)..."
  if [ -f "/etc/os-release" ]; then
    . /etc/os-release
    if [ "$ID" == "ubuntu" ]; then
      info "Detected OS: $PRETTY_NAME ($VERSION_ID)"
      if [ -z "$VERSION_CODENAME" ]; then
        error "Cannot get Ubuntu codename. Please ensure OS is up-to-date or set VERSION_CODENAME manually."
        exit 1
      fi
    else
      error "This script only supports Ubuntu. Detected: $ID"
      exit 1
    fi
  else
    error "Cannot detect operating system. /etc/os-release not found."
    exit 1
  fi
}

function check_docker_installed() {
  info "Checking if Docker is installed and running..."
  if ! command -v docker &> /dev/null; then
    error "Docker is not installed or not in PATH. NVIDIA Container Toolkit depends on Docker."
    error "Please install Docker first. Refer to official guide: https://docs.docker.com/engine/install/ubuntu/"
    exit 1
  fi

  # Verify Docker daemon is active
  if ! sudo systemctl is-active --quiet docker; then
    error "Docker service is not running. Please start Docker: sudo systemctl start docker"
    exit 1
  fi
  info "Docker is installed and running."
}

# --- Installation function (strict adherence to provided steps) ---

function install_nvidia_container_toolkit() {
  info "Starting NVIDIA Container Toolkit installation (strict mode)..."

  # Prechecks (retain, not affecting core install sequence)
  check_sudo
  check_arch
  check_os
  check_docker_installed

  info "Installing required prerequisite packages..."
  sudo apt-get update || { error "Failed to update apt package list."; exit 1; }
  sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release || { error "Failed to install prerequisites."; exit 1; }
  success "Prerequisite packages installed."

  info ">>>> STEP 1: Configure production repository <<<<"
  # Execute provided command sequence
  sudo mkdir -p /usr/share/keyrings || { error "Failed to create directory /usr/share/keyrings."; exit 1; }
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg && \
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null
  if [ $? -ne 0 ]; then
    error "Failed to configure production repository. Possible network issue or failed GPG/repo list download."
    exit 1
  fi
  success "Production repository configured."

  info ">>>> STEP 2: (Optional) Configure experimental packages – skipped by default <<<<"
  # Skipping experimental step per user request. If needed:
  # uncomment experimental line manually with:
  # sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
  info "Skipped experimental package configuration."

  info ">>>> STEP 3: Update package list <<<<"
  sudo apt-get update || { error "Failed to update apt package list. Check repo configuration."; exit 1; }
  success "Apt package list updated."

  info ">>>> STEP 4: Install NVIDIA Container Toolkit packages <<<<"
  # Install specified version
  NVIDIA_CONTAINER_TOOLKIT_VERSION="1.17.8-1" # version hardcoded
  info "Installing NVIDIA Container Toolkit version: ${NVIDIA_CONTAINER_TOOLKIT_VERSION}..."
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION} || { error "Failed to install NVIDIA Container Toolkit packages."; exit 1; }
  success "NVIDIA Container Toolkit packages installed."

  info "Configuring Docker runtime after install..."
  sudo nvidia-ctk runtime configure --runtime=docker || { error "Failed to configure Docker runtime."; exit 1; }
  success "Docker daemon configured."

  info "Restarting Docker service to apply changes..."
  sudo systemctl restart docker || { error "Failed to restart Docker."; exit 1; }
  success "Docker service restarted."

  success "NVIDIA Container Toolkit installation successful!"
  info "You can verify with: docker run --rm --gpus all ubuntu nvidia-smi"
  info "If first run fails, it may need initialization—please try again."
}

# --- Uninstall function (keep generality for cleanup) ---

function uninstall_nvidia_container_toolkit() {
  info "Starting NVIDIA Container Toolkit uninstallation..."

  check_sudo

  info "Removing NVIDIA Container Toolkit packages..."
  # Purge all possible packages
  sudo apt-get -y purge nvidia-container-toolkit \
                     nvidia-container-toolkit-base \
                     libnvidia-container-tools \
                     libnvidia-container1 \
                     || true
  sudo apt-get -y autoremove --purge || true
  success "NVIDIA Container Toolkit-related packages removed."

  info "Cleaning up related files and configs..."
  # Remove GPG key file
  if [ -f "/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg" ]; then
    sudo rm -f "/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg"
    success "Removed NVIDIA Container Toolkit GPG key."
  fi

  # Remove repo source list
  if [ -f "/etc/apt/sources.list.d/nvidia-container-toolkit.list" ]; then
    sudo rm -f "/etc/apt/sources.list.d/nvidia-container-toolkit.list"
    success "Removed NVIDIA Container Toolkit repository list."
  fi

  info "Attempting to revert Docker daemon configuration..."
  if command -v nvidia-ctk &> /dev/null && [ "$(nvidia-ctk --version | grep -oP 'version \K[0-9]+\.[0-9]+' | cut -d'.' -f1)" -ge 1 ] && [ "$(nvidia-ctk --version | grep -oP 'version \K[0-9]+\.[0-9]+' | cut -d'.' -f2)" -ge 9 ] ; then
    info "Detected nvidia-ctk version ≥ 1.9, attempting with 'nvidia-ctk runtime configure --runtime=docker --unconfigure'."
    sudo nvidia-ctk runtime configure --runtime=docker --unconfigure || { error "Failed to revert Docker runtime. Please manually remove 'nvidia' runtime config from /etc/docker/daemon.json."; }
  else
    info "nvidia-ctk version < 1.9 or not installed. Please manually remove 'nvidia' runtime from /etc/docker/daemon.json."
    info "Typically remove 'default-runtime': 'nvidia' and 'runtimes': { 'nvidia': { … } } sections."
  fi
  success "Docker daemon runtime rollback attempted."

  info "Restarting Docker service..."
  sudo systemctl restart docker || { error "Failed to restart Docker."; }
  success "Docker service restarted."

  success "NVIDIA Container Toolkit uninstalled."
}

# --- Main function ---

function main() {
  if [ "$#" -eq 0 ]; then
    info "Usage: $0 {install|uninstall}"
    info "Defaulting to install."
    install_nvidia_container_toolkit
  else
    case "$1" in
      install)
        install_nvidia_container_toolkit
        ;;
      uninstall)
        uninstall_nvidia_container_toolkit
        ;;
      *)
        error "Invalid argument: $1"
        info "Usage: $0 {install|uninstall}"
        exit 1
        ;;
    esac
  fi
}

main "$@"
