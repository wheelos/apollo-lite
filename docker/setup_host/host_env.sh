#!/usr/bin/env bash

###############################################################################
# Copyright 2025 The WheelOS Team. All Rights Reserved.
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

LOCAL_BIN_PATH="$HOME/.local/bin"
BASHRC="$HOME/.bashrc"
MARKER="# [apollo-deploy] Added by host_env.sh"

echo "Setup host environment..."

mkdir -p "$LOCAL_BIN_PATH"

if ! grep -qF "$MARKER" "$BASHRC"; then
  echo "Writing PATH update snippet to $BASHRC..."
  cat << EOF >> "$BASHRC"
$MARKER
if [ -d "\$HOME/.local/bin" ] && [[ ":\$PATH:" != *":\$HOME/.local/bin:"* ]]; then
    export PATH="\$HOME/.local/bin:\$PATH"
fi
EOF
else
  echo "Environment variables already configured in $BASHRC."
fi

# Apply immediately to the current shell
if [ -d "$LOCAL_BIN_PATH" ] && [[ ":$PATH:" != *":$LOCAL_BIN_PATH:"* ]]; then
  export PATH="$LOCAL_BIN_PATH:$PATH"
  echo "Updated PATH for current shell."
fi

echo "Current PATH: $PATH"
