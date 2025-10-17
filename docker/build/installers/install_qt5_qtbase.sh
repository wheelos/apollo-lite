#!/usr/bin/env bash

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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

CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
. ${CURR_DIR}/installer_base.sh

QT5_PREFIX="/usr/local/qt5"

# References:
# 1) http://www.linuxfromscratch.org/blfs/view/svn/x/qt5.html
# 2) https://src.fedoraproject.org/rpms/qt5-qtbase/tree/master
# 3) https://launchpad.net/ubuntu/+source/qtbase-opensource-src/5.12.8+dfsg-0ubuntu1
apt_get_update_and_install \
    libicu-dev \
    libdbus-1-dev \
    libfontconfig1-dev \
    libfreetype6-dev \
    libgl1-mesa-dev  \
    libharfbuzz-dev \
    libjpeg-dev \
    libpcre3-dev \
    libpng-dev \
    libsqlite3-dev \
    libssl-dev \
    libvulkan-dev \
    libxcb1-dev \
    libexpat1-dev \
    zlib1g-dev \
    libxcb-image0-dev \
    libxcb-keysyms1-dev \
    libxcb-render-util0-dev \
    libxcb-shm0-dev \
    libxcb-util1 \
    libxcb-xinerama0-dev \
    libxcb-xkb-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    qtbase5-dev

mkdir ${QT5_PREFIX} && ln -snf /usr/include/aarch64-linux-gnu/qt5 ${QT5_PREFIX}/include

ok "Successfully installed Qt5 qtbase from apt."

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*

