#!/bin/bash

# Docker Mirror Registry Configuration Script
# This script configures Docker to use multiple Chinese mirror registries for faster image pulls.
# It updates the "registry-mirrors" setting in daemon.json (no backup performed),
# and restarts the Docker service.

# --- Configuration ---
# Provide your preferred Docker mirror URLs in this array:
DOCKER_MIRROR_URLS=(
    "https://docker.m.daocloud.io"
    "https://dockerproxy.com"
)

# --- Script Start ---

echo "--- Checking Docker service status ---"
if ! systemctl is-active --quiet docker; then
    echo "Docker service is not running. Please start it first (e.g., sudo systemctl start docker)."
    exit 1
fi

DAEMON_JSON_PATH="/etc/docker/daemon.json"

echo "--- Configuring Docker Mirror Registries ---"

# Read current daemon.json content or start with an empty JSON object
CURRENT_DAEMON_JSON=$(cat "$DAEMON_JSON_PATH" 2>/dev/null || echo "{}")

# Check if 'jq' is installed.
if ! command -v jq &> /dev/null; then
    echo "Error: 'jq' is not installed. Please install 'jq' to proceed."
    echo "  For Debian/Ubuntu: sudo apt-get install jq"
    echo "  For CentOS/RHEL: sudo yum install epel-release && sudo yum install jq"
    exit 1
fi

# Convert the mirror URLs array to a JSON array string
NEW_MIRRORS_JSON=$(printf '%s\n' "${DOCKER_MIRROR_URLS[@]}" | jq -R . | jq -s .)

# Use jq to update/add the "registry-mirrors" key, preserving other keys.
NEW_DAEMON_JSON=$(echo "$CURRENT_DAEMON_JSON" | jq --argjson new_mirrors "$NEW_MIRRORS_JSON" '. + {"registry-mirrors": $new_mirrors}')

# Write the new daemon.json
echo "$NEW_DAEMON_JSON" | sudo tee "$DAEMON_JSON_PATH" > /dev/null

echo "--- Docker daemon.json configuration complete ---"
echo "New daemon.json content:"
cat "$DAEMON_JSON_PATH"
echo ""

echo "--- Restarting Docker service to apply changes ---"
sudo systemctl daemon-reload
sudo systemctl restart docker

echo "--- Verifying Docker service status ---"
if systemctl is-active --quiet docker; then
    echo "Docker service restarted successfully!"
else
    echo "Docker service failed to restart. Please check logs: sudo journalctl -xe | grep docker"
    exit 1
fi

echo "--- Verifying mirror configuration ---"
echo "You can check 'docker info' for 'Registry Mirrors' or try 'docker pull hello-world'."
docker info | grep "Registry Mirrors"

echo ""
echo "Script execution finished."
