#!/usr/bin/env bash

# ==============================================================================
# Script Name: install_docker.sh
# Description: Installs Docker Engine, CLI, Containerd, Buildx, and Compose
#              plugins on a Ubuntu-based system.
#              Includes checks for existing installation, pre-conditions, and
#              robust error handling.
# Author: WheelOS
# Date: June 24, 2025
# ==============================================================================

set -euo pipefail

# --- Global Variables ---
ARCH="$(uname -m)"
DOCKER_GPG_KEYRING="/etc/apt/keyrings/docker.gpg"
DOCKER_REPO_LIST="/etc/apt/sources.list.d/docker.list"
# Link for architectural support issues, as suggested in original script
ISSUES_LINK="https://github.com/ApolloAuto/apollo/issues"

# --- Colors for Output ---
BOLD='\033[1m'
RED='\033[0;31m'
WHITE='\033[34m'
NO_COLOR='\033[0m'

# --- Logging Functions ---
# Prints an informational message to stderr.
info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

# Prints an error message to stderr.
error() {
  (echo >&2 -e "[${RED}ERROR${NO_COLOR}] $*")
}

# --- Core Logic Functions ---

# Checks if Docker is already installed by verifying the 'docker' command.
# Returns 0 if Docker is installed, 1 otherwise.
is_docker_already_installed() {
  info "Checking if Docker is already installed..."
  if command -v docker &> /dev/null; then
    if sudo systemctl is-active --quiet docker; then
      info "Docker is detected and running. Installation will be skipped."
      return 0 # Docker installed and active
    else
      info "Docker command found, but service is not active. Attempting to start service."
      sudo systemctl start docker && sudo systemctl enable docker &> /dev/null
      if [ $? -eq 0 ]; then
        info "Docker service started and enabled. Installation will be skipped."
        return 0
      else
        error "Docker command found, but service could not be started. Please investigate. Proceeding with installation attempt."
        return 1 # Docker detected, but not fully functional, so attempt reinstall
      fi
    fi
  fi
  info "Docker not detected. Proceeding with installation."
  return 1 # Docker not installed
}

# Checks pre-conditions necessary for Docker installation.
# Returns 0 if all pre-conditions are met, 1 otherwise.
check_docker_pre_conditions() {
  info "Checking Docker installation pre-conditions..."

  # 1. Check for supported OS (Ubuntu)
  if ! command -v lsb_release &> /dev/null; then
    error "lsb_release command not found. Cannot determine OS distribution. Please install 'lsb-release'."
    return 1
  fi
  if [ "$(lsb_release -is)" != "Ubuntu" ]; then
    error "Unsupported operating system: $(lsb_release -is). This script is designed for Ubuntu."
    return 1
  fi

  # 2. Check for supported architecture
  local arch_alias=""
  if [ "${ARCH}" == "x86_64" ]; then
    arch_alias="amd64"
  elif [ "${ARCH}" == "aarch64" ]; then
    arch_alias="arm64"
  else
    error "Unsupported architecture: ${ARCH}. You can create an Issue at ${ISSUES_LINK}."
    return 1
  fi
  info "Detected supported architecture: ${ARCH} (alias: ${arch_alias})."

  # 3. Check for root privileges (handled by main script calling with sudo, but good to double check)
  if [ "$(id -u)" -ne 0 ]; then
      error "This script must be run with root privileges (sudo)."
      return 1
  fi

  info "All Docker pre-conditions met."
  return 0
}

# Installs necessary prerequisite packages for Docker.
install_prereq_packages() {
  info "Updating apt package list..."
  sudo apt-get -y update
  if [ $? -ne 0 ]; then
    error "Failed to update apt package list. Please check network connection or source configuration."
    return 1
  fi

  info "Installing prerequisite packages (apt-transport-https, ca-certificates, curl, gnupg, software-properties-common)..."
  sudo apt-get -y install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    software-properties-common
  if [ $? -ne 0 ]; then
    error "Failed to install prerequisite packages. Please check internet connectivity or package availability."
    return 1
  fi
  info "Prerequisite packages installed."
  return 0
}

# Sets up the Docker APT repository and installs Docker components.
setup_docker_repo_and_install() {
  local arch_alias=
  if [ "${ARCH}" == "x86_64" ]; then
    arch_alias="amd64"
  elif [ "${ARCH}" == "aarch64" ]; then
    arch_alias="arm64"
  fi # Pre-condition check already validated supported architecture, so no 'else' needed here.

  info "Adding Docker official GPG key..."
  # Create apt keyrings directory if it doesn't exist
  sudo install -m 0755 -d /etc/apt/keyrings
  if [ $? -ne 0 ]; then
    error "Failed to create /etc/apt/keyrings directory."
    return 1
  fi

  # Download and dearmor the GPG key into the keyrings directory
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o "${DOCKER_GPG_KEYRING}"
  if [ $? -ne 0 ]; then
    error "Failed to download or dearmor Docker GPG key to ${DOCKER_GPG_KEYRING}. Please check network connection or GPG tool availability."
    return 1
  fi
  info "Docker GPG key added to ${DOCKER_GPG_KEYRING}."

  info "Setting up Docker stable repository..."
  # Add Docker repository to apt sources list using new signed-by syntax
  echo "deb [arch=${arch_alias} signed-by=${DOCKER_GPG_KEYRING}] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee "${DOCKER_REPO_LIST}" > /dev/null
  if [ $? -ne 0 ]; then
    error "Failed to set up Docker repository in ${DOCKER_REPO_LIST}. Please check system info or repository URL."
    return 1
  fi
  info "Docker stable repository added to ${DOCKER_REPO_LIST}."

  info "Updating apt package list with new repository..."
  sudo apt-get update
  if [ $? -ne 0 ]; then
    error "Failed to update apt package list after adding Docker repository. Please check repository configuration."
    return 1
  fi
  info "Apt cache updated."

  info "Installing Docker Engine, CLI, Containerd, Buildx, and Compose plugins..."
  # Install Docker Engine and related components including official recommended plugins
  sudo apt-get install -y docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin
  if [ $? -ne 0 ]; then
    error "Failed to install Docker components. Please check dependencies or try again."
    return 1
  fi
  info "Docker Engine, CLI, Containerd, Buildx, and Compose plugins installed."
  return 0
}

# Applies post-installation settings for Docker.
post_install_settings() {
  info "Applying post-installation settings..."
  # Add current user to 'docker' group to allow running docker commands without sudo
  info "Adding current user '$USER' to 'docker' group. You may need to logout and login again (or run 'newgrp docker') for changes to take effect."
  sudo usermod -aG docker "$USER"
  if [ $? -ne 0 ]; then
    error "Failed to add user '$USER' to 'docker' group. Manual intervention may be required."
    return 1
  fi

  # Restart Docker service to apply changes
  info "Restarting Docker service..."
  sudo systemctl restart docker
  if [ $? -ne 0 ]; then
    error "Failed to restart Docker service. Please check 'systemctl status docker.service'."
    return 1
  fi
  info "Docker service restarted."
  info "Post-installation settings applied."
  return 0
}

# --- Main Installation Function ---
# This is the primary entry point for installing Docker.
install_docker() {
  info "Starting Docker installation process..."

  # 1. Check if Docker is already installed
  if is_docker_already_installed; then
    info "Docker is already functional. Exiting Docker installation script successfully."
    return 0 # Exit with success as Docker is already set up
  fi

  # 2. Check pre-conditions before proceeding with installation
  if ! check_docker_pre_conditions; then
    error "Docker pre-conditions not met. Aborting Docker installation."
    return 1 # Exit with error as pre-conditions failed
  fi

  # 3. Install prerequisite packages
  if ! install_prereq_packages; then
    error "Prerequisite package installation failed. Aborting Docker installation."
    return 1
  fi

  # 4. Set up Docker repository and install components
  if ! setup_docker_repo_and_install; then
    error "Docker repository setup or component installation failed. Aborting Docker installation."
    return 1
  fi

  # 5. Apply post-installation settings
  if ! post_install_settings; then
    error "Docker post-installation settings failed. Aborting Docker installation."
    return 1
  fi

  info "Docker installation completed successfully!"
  info "IMPORTANT: You need to logout and login again (or run 'newgrp docker') for user '$USER' to use Docker without 'sudo'."
  info "You can verify the installation by running 'docker run hello-world' after re-logging."
  return 0 # Final success
}

# --- Main Uninstallation Function ---
# This function handles the uninstallation of Docker.
uninstall_docker() {
  info "Starting Docker uninstallation process..."
  info "Attempting to stop and disable Docker service..."
  sudo systemctl stop docker &> /dev/null || true
  sudo systemctl disable docker &> /dev/null || true
  info "Docker service stopped."

  info "Removing Docker components..."
  # Try removing all known Docker packages. Use '|| true' to continue even if some packages are not installed.
  sudo apt-get -y remove docker docker-engine docker.io docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin || true
  sudo apt-get -y purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin || true
  sudo apt-get -y autoremove --purge || true # Remove residual configs and unused dependencies
  info "Docker components removed."

  info "Cleaning up Docker repository files..."
  # Remove GPG key file
  if [ -f "${DOCKER_GPG_KEYRING}" ]; then
    sudo rm -f "${DOCKER_GPG_KEYRING}"
    info "Removed ${DOCKER_GPG_KEYRING}."
  fi

  # Remove repository sources list file
  if [ -f "${DOCKER_REPO_LIST}" ]; then
    sudo rm -f "${DOCKER_REPO_LIST}"
    info "Removed ${DOCKER_REPO_LIST}."
  fi

  info "Updating apt cache after cleanup..."
  sudo apt-get -y update || true # Update apt cache, allow failure as it's cleanup
  info "Apt cache updated."

  info "Docker uninstallation completed."
  return 0 # Always return 0 for uninstallation success
}

# --- Script Entry Point ---
# Handles command-line arguments to either install or uninstall Docker.
main() {
  case "$1" in
    install)
      install_docker
      ;;
    uninstall)
      uninstall_docker
      ;;
    *)
      info "Usage: $0 {install|uninstall}"
      info "Defaulting to 'install' mode as no valid argument was provided."
      install_docker
      ;;
  esac
}

main "$@"
