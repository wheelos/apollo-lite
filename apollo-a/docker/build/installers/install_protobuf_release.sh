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
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

# Notes on Protobuf Installer:
# 1) protobuf for cpp didn't need to be pre-installed into system
# 2) protobuf for python should be provided for cyber

VERSION="29.0"

PROTOC_NAME="protoc"
CHECKSUM="c0eb373db646ac4850d34b8dfa40dbfcc3e96530b873dc8209c9a3f17be6a6c5"
DOWNLOAD_LINK="https://github.com/wheelos/wheel.os/releases/download/v1.0.0/${PROTOC_NAME}"

download_if_not_cached "$PROTOC_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"
sudo cp protoc /usr/local/bin/ && chmod +x /usr/local/bin/protoc

PKG_NAME="protobuf.tar.gz"
CHECKSUM="04ab708746c9d8b43f582056b43a3d7ea46c9ae1b05353b35354d9e35063c716"
DOWNLOAD_LINK="https://github.com/wheelos/wheel.os/releases/download/v1.0.0/${PKG_NAME}"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"
pip3_install ${PKG_NAME}

ok "Successfully installed protobuf, VERSION=${VERSION}"

# Clean up.
rm -fr ${PROTOC_NAME} ${PKG_NAME}
