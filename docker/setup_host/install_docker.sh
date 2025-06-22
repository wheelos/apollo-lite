#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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

ARCH="$(uname -m)"

BOLD='\033[1m'
RED='\033[0;31m'
WHITE='\033[34m'
NO_COLOR='\033[0m'

function info() {
  (echo >&2 -e "[${WHITE}${BOLD}INFO${NO_COLOR}] $*")
}

function error() {
  (echo >&2 -e "[${RED}ERROR${NO_COLOR}] $*")
}

function install_prereq_packages() {
  info "Updating apt package list..."
  sudo apt-get -y update || { error "Failed to update apt package list. Please check network connection or source configuration."; exit 1; }
  info "Installing prerequisite packages..."

  sudo apt-get -y install \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg \
    software-properties-common || { error "Failed to install prerequisite packages."; exit 1; }

  info "Prerequisite packages installed."
}

function setup_docker_repo_and_install() {
  local issues_link="https://github.com/ApolloAuto/apollo/issues"
  local arch_alias=
  if [ "${ARCH}" == "x86_64" ]; then
    arch_alias="amd64"
  elif [ "${ARCH}" == "aarch64" ]; then
    arch_alias="arm64"
  else
    error "Current architecture ${ARCH} is not supported." \
          "You can create an Issue at ${issues_link}."
    exit 1
  fi

  info "Adding Docker official GPG key..."
  # Create apt keyrings directory if it doesn't exist
  sudo install -m 0755 -d /etc/apt/keyrings || { error "Failed to create /etc/apt/keyrings directory."; exit 1; }
  # Download and dearmor the GPG key into the keyrings directory, replacing old apt-key add method
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  if [ $? -ne 0 ]; then
    error "Failed to download or dearmor Docker GPG key. Please check network connection or GPG tool availability."
    exit 1
  fi
  info "Docker GPG key added to /etc/apt/keyrings/docker.gpg."

  info "Setting up Docker stable repository..."
  # Add Docker repository to apt sources list using new signed-by syntax
  echo \
    "deb [arch=${arch_alias} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  if [ $? -ne 0 ]; then
    error "Failed to set up Docker repository. Please check system info or repository URL."
    exit 1
  fi
  info "Docker stable repository added to /etc/apt/sources.list.d/docker.list."

  info "Updating apt package list with new repository..."
  sudo apt-get update
  if [ $? -ne 0 ]; then
    error "Failed to update apt package list after adding Docker repository. Please check repository configuration."
    exit 1
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
    exit 1
  fi
  info "Docker Engine, CLI, Containerd, Buildx, and Compose plugins installed."
}

function post_install_settings() {
  info "Applying post-installation settings..."
  # Add current user to 'docker' group to allow running docker commands without sudo
  info "Adding current user '$USER' to 'docker' group. You may need to logout and login again for changes to take effect."
  sudo usermod -aG docker $USER || { error "Failed to add user to 'docker' group."; }

  # Restart Docker service to apply changes
  info "Restarting Docker service..."
  sudo systemctl restart docker || { error "Failed to restart Docker service. Please check systemctl status."; }
  info "Docker service restarted."
  info "Post-installation settings applied."
}

function install_docker() {
  info "Starting Docker installation process..."
  install_prereq_packages
  setup_docker_repo_and_install
  post_install_settings
  info "Docker installation completed. Please logout and login again (or run 'newgrp docker') to use Docker without 'sudo'."
  info "You can verify installation by running 'docker run hello-world'."
}

function uninstall_docker() {
  info "Starting Docker uninstallation process..."
  info "Removing Docker components..."
  # Try removing all known Docker packages. Use '|| true' to continue even if some packages are not installed.
  sudo apt-get -y remove docker docker-engine docker.io docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin || true
  sudo apt-get -y purge docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin || true
  sudo apt-get -y autoremove --purge || true # Remove residual configs and unused dependencies
  info "Docker components removed."

  info "Cleaning up Docker repository files..."
  # Remove GPG key file
  if [ -f "/etc/apt/keyrings/docker.gpg" ]; then
    sudo rm -f "/etc/apt/keyrings/docker.gpg"
    info "Removed /etc/apt/keyrings/docker.gpg."
  fi

  # Remove repository sources list file
  if [ -f "/etc/apt/sources.list.d/docker.list" ]; then
    sudo rm -f "/etc/apt/sources.list.d/docker.list"
    info "Removed /etc/apt/sources.list.d/docker.list."
  fi

  info "Updating apt cache after cleanup..."
  sudo apt-get -y update || true
  info "Apt cache updated."

  info "Docker uninstallation completed."
}

function main() {
  case $1 in
    install)
      install_docker
      ;;
    uninstall)
      uninstall_docker
      ;;
    *)
      info "Usage: $0 {install|uninstall}"
      install_docker
      ;;
  esac
}

main "$@"
