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

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Install Yarn using the recommended method for modern Debian/Ubuntu
# Ref: https://yarnpkg.com/getting-started/install#debian-ubuntu-linux

info "Installing yarn ..."

# 1. Download and dearmor the GPG key, then save it to /usr/share/keyrings/
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | gpg --dearmor | tee /usr/share/keyrings/yarn-archive-keyring.gpg >/dev/null

# 2. Add the Yarn repository to sources.list.d, referencing the new keyring file
echo "deb [signed-by=/usr/share/keyrings/yarn-archive-keyring.gpg] https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list >/dev/null

# 3. Update apt and install yarn
apt_get_update_and_install yarn

info "Successfully installed yarn"

apt-get clean
rm -fr /etc/apt/sources.list.d/yarn.list
