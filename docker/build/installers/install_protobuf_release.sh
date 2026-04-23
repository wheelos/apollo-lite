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

ARCH="$(uname -m)"
VERSION="29.0"

# install protoc
if [[ "${ARCH}" == "aarch64" ]]; then
    pkg="protoc-${VERSION}-linux-aarch_64.zip"
    download_link="https://github.com/protocolbuffers/protobuf/releases/download/v${VERSION}/protoc-${VERSION}-linux-aarch_64.zip"
    checksum="305f1be5ae7b2f39451870b312b45c1e0ba269901c83ba16d85f9f9d1441b348"
    download_if_not_cached "${pkg}" "${checksum}" "${download_link}"
    unzip "${pkg}" "bin/protoc" -d /usr/local
    rm -rf "${pkg}"
elif [[ "${ARCH}" == "x86_64" ]]; then
    pkg="protoc-${VERSION}-linux-x86_64.zip"
    download_link="https://github.com/protocolbuffers/protobuf/releases/download/v${VERSION}/protoc-${VERSION}-linux-x86_64.zip"
    checksum="3c51065af3b9a606d9e18a1bf628143734ff4b9e69725d6459857430ba7a78df"
    download_if_not_cached "${pkg}" "${checksum}" "${download_link}"
    unzip "${pkg}" "bin/protoc" -d /usr/local
    rm -rf "${pkg}"
fi

# install protobuf for python
PKG_NAME="protobuf.tar.gz"
CHECKSUM="04ab708746c9d8b43f582056b43a3d7ea46c9ae1b05353b35354d9e35063c716"
DOWNLOAD_LINK="https://github.com/wheelos/wheel.os/releases/download/v1.0.0/${PKG_NAME}"
download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"
pip3_install ${PKG_NAME}
ok "Successfully installed protobuf, VERSION=${VERSION}"
rm -fr ${PKG_NAME}
