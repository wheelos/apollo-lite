#!/usr/bin/env bash

# ==============================================================================
# Script Name: install_nvidia_container_toolkit.sh
# Description: Installs NVIDIA Container Toolkit on Ubuntu-based systems
#              to enable GPU support for Docker containers.
#              Includes checks for existing installation, pre-conditions, and
#              robust error handling.
# Author: WheelOS
# Date: June 24, 2025
# ==============================================================================

# --- Strict Mode ---
set -euo pipefail

# --- Global Variables ---
ARCH="$(uname -m)"
NVIDIA_TOOLKIT_KEYRING="/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg"
NVIDIA_TOOLKIT_LIST="/etc/apt/sources.list.d/nvidia-container-toolkit.list"
NVIDIA_CONTAINER_TOOLKIT_VERSION="1.17.8-1" # Hardcoded version

# --- Color Definitions for Output ---
BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[0;32m'
WHITE='\033[34m'
NO_COLOR='\033[0m'

# --- Logging Functions ---
# Prints an informational message to stderr.
info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

# Prints a success message to stderr.
success() {
  (echo >&2 -e "[${GREEN}${BOLD}SUCCESS${NO_COLOR}] $*")
}

# Prints an error message to stderr.
error() {
  (echo >&2 -e "[${RED}${BOLD}ERROR${NO_COLOR}] $*")
}

# --- Core Logic Functions ---

# Checks if NVIDIA Container Toolkit is already installed and functional.
# Returns 0 if installed, 1 otherwise.
is_nvidia_toolkit_already_installed() {
  info "Checking if NVIDIA Container Toolkit is already installed..."
  # Check if nvidia-container-toolkit package is installed
  if dpkg -s nvidia-container-toolkit &> /dev/null; then
    info "nvidia-container-toolkit package found."

    # Verify if Docker is configured to use the 'nvidia' runtime
    if sudo docker info --format '{{.Runtimes}}' | grep -q 'nvidia'; then
      info "Docker is configured with 'nvidia' runtime."
      info "NVIDIA Container Toolkit appears to be installed and configured. Skipping installation."
      return 0 # Already installed and configured
    else
      info "NVIDIA Container Toolkit package found, but Docker runtime is not configured. Attempting to configure."
      if sudo nvidia-ctk runtime configure --runtime=docker; then
        info "Docker runtime configured successfully."
        sudo systemctl restart docker &> /dev/null
        if [ $? -eq 0 ]; then
          info "Docker service restarted. NVIDIA Container Toolkit now appears functional. Skipping installation."
          return 0
        else
          error "Failed to restart Docker service after runtime configuration. Please check Docker status. Proceeding with full installation attempt."
          return 1
        fi
      else
        error "Failed to configure Docker runtime for NVIDIA Container Toolkit. Proceeding with full installation attempt."
        return 1
      fi
    fi
  fi

  info "NVIDIA Container Toolkit not detected. Proceeding with installation."
  return 1 # Not installed or not fully functional
}

# Checks pre-conditions necessary for NVIDIA Container Toolkit installation.
# Returns 0 if all pre-conditions are met, 1 otherwise.
check_nvidia_toolkit_pre_conditions() {
  info "Checking NVIDIA Container Toolkit pre-conditions..."

  # 1. Check for root privileges
  if [ "$(id -u)" -ne 0 ]; then
      error "This script must be run with root privileges (sudo)."
      return 1
  fi

  # 2. Check for supported OS (Ubuntu)
  if ! command -v lsb_release &> /dev/null; then
    error "lsb_release command not found. Cannot determine OS distribution. Please install 'lsb-release'."
    return 1
  fi
  if [ "$(lsb_release -is)" != "Ubuntu" ]; then
    error "Unsupported operating system: $(lsb_release -is). This script is designed for Ubuntu."
    return 1
  fi
  if [ -z "$(lsb_release -cs)" ]; then
    error "Cannot get Ubuntu codename. Please ensure OS is up-to-date or 'lsb_release -cs' returns a value."
    return 1
  fi
  info "Detected supported OS: Ubuntu ($(lsb_release -cs))."

  # 3. Check for supported architecture
  if [ "${ARCH}" == "x86_64" ]; then
    info "Detected supported architecture: x86_64 (amd64)."
  elif [ "${ARCH}" == "aarch64" ]; then
    info "Detected supported architecture: aarch64 (arm64)."
  else
    error "Unsupported architecture: ${ARCH}. NVIDIA Container Toolkit only officially supports x86_64 and aarch64."
    return 1
  fi

  # 4. Check if Docker is installed and running (Critical Dependency)
  info "Verifying Docker installation and status..."
  if ! command -v docker &> /dev/null; then
    error "Docker is not installed or not in PATH. NVIDIA Container Toolkit explicitly depends on Docker."
    error "Please install Docker first using 'install_docker.sh' script."
    return 1
  fi
  if ! sudo systemctl is-active --quiet docker; then
    error "Docker service is not running. NVIDIA Container Toolkit requires Docker service to be active."
    error "Please start Docker: 'sudo systemctl start docker' or ensure it's functional."
    return 1
  fi
  info "Docker is installed and running."

  # 5. Check if NVIDIA driver is installed (Indirect Dependency - Toolkit needs drivers)
  # While the toolkit itself can be installed without drivers, it's useless without them.
  # This check ensures a more complete functional environment.
  info "Verifying NVIDIA GPU driver installation..."
  if ! command -v nvidia-smi &> /dev/null; then
    error "NVIDIA GPU driver (nvidia-smi command) not found. NVIDIA Container Toolkit requires NVIDIA drivers to function."
    error "Please install NVIDIA GPU drivers first."
    return 1
  fi
  info "NVIDIA GPU driver (nvidia-smi) found."

  success "All NVIDIA Container Toolkit pre-conditions met."
  return 0
}

# Installs necessary prerequisite packages for NVIDIA Container Toolkit.
install_prereq_packages() {
  info "Updating apt package list..."
  sudo apt-get update
  if [ $? -ne 0 ]; then
    error "Failed to update apt package list. Please check network connection or source configuration."
    return 1
  fi

  info "Installing prerequisite packages (apt-transport-https, ca-certificates, curl, gnupg, lsb-release)..."
  sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
  if [ $? -ne 0 ]; then
    error "Failed to install prerequisite packages. Please check internet connectivity or package availability."
    return 1
  fi
  success "Prerequisite packages installed."
  return 0
}

# Configures the NVIDIA Container Toolkit repository and installs packages.
setup_nvidia_toolkit_repo_and_install() {
  info "Configuring production repository for NVIDIA Container Toolkit..."

  # Create apt keyrings directory if it doesn't exist
  sudo mkdir -p /usr/share/keyrings
  if [ $? -ne 0 ]; then
    error "Failed to create directory /usr/share/keyrings."
    return 1
  fi

  # Download and dearmor the GPG key
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o "${NVIDIA_TOOLKIT_KEYRING}"
  if [ $? -ne 0 ]; then
    error "Failed to download or dearmor NVIDIA Container Toolkit GPG key to ${NVIDIA_TOOLKIT_KEYRING}. Please check network or GPG tool."
    return 1
  fi
  info "NVIDIA Container Toolkit GPG key added to ${NVIDIA_TOOLKIT_KEYRING}."

  # Add repository list, using sed to include signed-by attribute
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee "${NVIDIA_TOOLKIT_LIST}" > /dev/null
  if [ $? -ne 0 ]; then
    error "Failed to set up NVIDIA Container Toolkit repository in ${NVIDIA_TOOLKIT_LIST}. Check network or repository URL."
    return 1
  fi
  info "NVIDIA Container Toolkit stable repository added to ${NVIDIA_TOOLKIT_LIST}."

  info "Skipping optional experimental package configuration (as per standard practice)."

  info "Updating apt package list with new repository..."
  sudo apt-get update
  if [ $? -ne 0 ]; then
    error "Failed to update apt package list after adding NVIDIA Container Toolkit repository. Check repository configuration."
    return 1
  fi
  success "Apt package list updated."

  info "Installing NVIDIA Container Toolkit packages (version: ${NVIDIA_CONTAINER_TOOLKIT_VERSION})..."
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
  if [ $? -ne 0 ]; then
    error "Failed to install NVIDIA Container Toolkit packages (version ${NVIDIA_CONTAINER_TOOLKIT_VERSION}). Please check package availability or version."
    return 1
  fi
  success "NVIDIA Container Toolkit packages installed."
  return 0
}

# Configures Docker runtime and restarts Docker service.
configure_docker_runtime_and_restart() {
  info "Configuring Docker runtime for NVIDIA Container Toolkit..."
  sudo nvidia-ctk runtime configure --runtime=docker
  if [ $? -ne 0 ]; then
    error "Failed to configure Docker runtime for NVIDIA Container Toolkit."
    return 1
  fi
  success "Docker daemon configured for NVIDIA runtime."

  info "Restarting Docker service to apply changes..."
  sudo systemctl restart docker
  if [ $? -ne 0 ]; then
    error "Failed to restart Docker service. Please check 'systemctl status docker.service'."
    return 1
  fi
  success "Docker service restarted."
  return 0
}

# --- Main Installation Orchestration Function ---
# This is the primary entry point for installing NVIDIA Container Toolkit.
install_nvidia_container_toolkit() {
  info "Starting NVIDIA Container Toolkit installation process..."

  # 1. Check if NVIDIA Container Toolkit is already installed
  if is_nvidia_toolkit_already_installed; then
    success "NVIDIA Container Toolkit is already functional. Exiting installation script successfully."
    return 0 # Exit with success as nothing needs to be done
  fi

  # 2. Check pre-conditions before proceeding with installation
  if ! check_nvidia_toolkit_pre_conditions; then
    error "NVIDIA Container Toolkit pre-conditions not met. Aborting installation."
    return 1 # Exit with error as pre-conditions failed
  fi

  # 3. Install prerequisite packages for the toolkit
  if ! install_prereq_packages; then
    error "Prerequisite package installation failed. Aborting NVIDIA Container Toolkit installation."
    return 1
  fi

  # 4. Setup NVIDIA Container Toolkit repository and install core packages
  if ! setup_nvidia_toolkit_repo_and_install; then
    error "NVIDIA Container Toolkit repository setup or package installation failed. Aborting."
    return 1
  fi

  # 5. Configure Docker runtime and restart Docker service
  if ! configure_docker_runtime_and_restart; then
    error "Docker runtime configuration or service restart failed. Aborting NVIDIA Container Toolkit installation."
    return 1
  fi

  success "NVIDIA Container Toolkit installation completed successfully!"
  info "You can verify the installation by running: 'docker run --rm --gpus all ubuntu nvidia-smi'"
  info "If the first run fails, it may need initialization - please try again."
  return 0 # Final success
}

# --- Uninstallation Function ---
# This function handles the uninstallation of NVIDIA Container Toolkit.
uninstall_nvidia_container_toolkit() {
  info "Starting NVIDIA Container Toolkit uninstallation process..."

  # It's good practice to ensure root privileges even for uninstall, if not enforced by parent script
  if [ "$(id -u)" -ne 0 ]; then
      error "This script must be run with root privileges (sudo) for uninstallation."
      return 1
  fi

  info "Removing NVIDIA Container Toolkit packages..."
  # Purge all possible packages, allow failure if not installed
  sudo apt-get -y purge nvidia-container-toolkit \
                     nvidia-container-toolkit-base \
                     libnvidia-container-tools \
                     libnvidia-container1 || true
  sudo apt-get -y autoremove --purge || true
  success "NVIDIA Container Toolkit-related packages removed."

  info "Cleaning up related files and configs..."
  # Remove GPG key file
  if [ -f "${NVIDIA_TOOLKIT_KEYRING}" ]; then
    sudo rm -f "${NVIDIA_TOOLKIT_KEYRING}"
    info "Removed NVIDIA Container Toolkit GPG key: ${NVIDIA_TOOLKIT_KEYRING}."
  fi

  # Remove repo source list
  if [ -f "${NVIDIA_TOOLKIT_LIST}" ]; then
    sudo rm -f "${NVIDIA_TOOLKIT_LIST}"
    info "Removed NVIDIA Container Toolkit repository list: ${NVIDIA_TOOLKIT_LIST}."
  fi

  info "Attempting to revert Docker daemon configuration..."
  # Check if nvidia-ctk is available and version >= 1.9 for proper unconfigure command
  if command -v nvidia-ctk &> /dev/null; then
    # Safely get version string, then extract major.minor
    local ctk_version_raw=$(nvidia-ctk --version 2>&1 | grep -oP 'version \K[0-9]+\.[0-9]+' || echo "0.0")
    local ctk_major=$(echo "$ctk_version_raw" | cut -d'.' -f1)
    local ctk_minor=$(echo "$ctk_version_raw" | cut -d'.' -f2)

    if (( ctk_major > 1 || (ctk_major == 1 && ctk_minor >= 9) )); then
      info "Detected nvidia-ctk version ${ctk_version_raw} (>= 1.9), attempting with 'nvidia-ctk runtime configure --runtime=docker --unconfigure'."
      sudo nvidia-ctk runtime configure --runtime=docker --unconfigure
      if [ $? -ne 0 ]; then
        error "Failed to revert Docker runtime using nvidia-ctk. Please manually remove 'nvidia' runtime config from /etc/docker/daemon.json."
      else
        success "Docker runtime configuration reverted."
      fi
    else
      info "nvidia-ctk version ${ctk_version_raw} (< 1.9) detected or unparseable. Manual intervention may be needed."
      error "Please manually remove 'nvidia' runtime from /etc/docker/daemon.json if it exists."
      info "Typically remove 'default-runtime': 'nvidia' and 'runtimes': { 'nvidia': { ... } } sections."
    fi
  else
    error "nvidia-ctk command not found. Cannot automatically revert Docker runtime configuration."
    info "Please manually remove 'nvidia' runtime from /etc/docker/daemon.json if it exists."
    info "Typically remove 'default-runtime': 'nvidia' and 'runtimes': { 'nvidia': { ... } } sections."
  fi

  info "Restarting Docker service to finalize changes..."
  sudo systemctl restart docker
  if [ $? -ne 0 ]; then
    error "Failed to restart Docker service. Please check Docker status. Manual restart may be required."
  else
    success "Docker service restarted."
  fi

  success "NVIDIA Container Toolkit uninstallation completed."
  return 0
}

# --- Main Script Entry Point ---
# Parses command-line arguments to either install or uninstall the toolkit.
main() {
  if [ "$#" -eq 0 ]; then
    info "Defaulting to 'install' mode as no valid argument was provided."
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
        return 1 # Return 1 for invalid arguments
        ;;
    esac
  fi
}

# Execute the main function with all command-line arguments
main "$@"
