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
# 2) protobuf for python should be provided for cyber testcases

VERSION="29.0"

PKG_NAME="protobuf-${VERSION}.tar.gz"
CHECKSUM="10a0d58f39a1a909e95e00e8ba0b5b1dc64d02997f741151953a2b3659f6e78c"
DOWNLOAD_LINK="https://github.com/protocolbuffers/protobuf/archive/refs/tags/v${VERSION}.tar.gz"

download_if_not_cached "$PKG_NAME" "$CHECKSUM" "$DOWNLOAD_LINK"

tar xzf ${PKG_NAME}

pushd protobuf-${VERSION}

# Install protoc compiler.
bazel build :protoc :protobuf
sudo cp bazel-bin/protoc /usr/local/bin

# Add in "cyber/setup.bash" to set up the environment.
# export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

bazel build //python/dist:source_wheel
pip install bazel-bin/python/dist/protobuf-*.tar.gz

popd

ok "Successfully installed protobuf, VERSION=${VERSION}"

# Clean up.
rm -fr ${PKG_NAME}  protobuf-${VERSION}
