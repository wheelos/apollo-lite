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

set -euo pipefail

geo="${1:-us}"

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

if [[ "$geo" == "cn" ]]; then
  echo "📍 China region, switching mirror source to Taobao (npm mirror)"
  yarn config set registry https://registry.npmmirror.com/
  npm config set registry https://registry.npmmirror.com/
fi

curl -fsSL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt-get install -y nodejs

echo "✅ Node.js installation completed:"
node -v
npm -v
echo "▼ Yarn registry"
yarn config get registry

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
