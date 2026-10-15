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

cd "$(dirname "${BASH_SOURCE[0]}")"
. ./installer_base.sh

ARCH="$(uname -m)"

# Disabled:
#   apt-file
apt_get_update_and_install \
    apt-utils \
    bc      \
    curl    \
    file    \
    gawk    \
    git     \
    less    \
    lsof    \
    python3     \
    python3-pip \
    python3-distutils \
    sed         \
    software-properties-common \
    sudo    \
    unzip   \
    vim     \
    wget    \
    zip     \
    xz-utils

if [[ "${ARCH}" == "aarch64" ]]; then
    apt-get -y install kmod
fi

MY_STAGE=
if [[ -f /etc/apollo.conf ]]; then
    MY_STAGE="$(awk -F '=' '/^stage=/ {print $2}' /etc/apollo.conf 2>/dev/null)"
fi

apt_get_update_and_install \
    build-essential \
    autoconf    \
    automake    \
    gdb         \
    libtool     \
    patch       \
    pkg-config      \
    libexpat1-dev   \
    linux-libc-dev


##----------------##
##    SUDO        ##
##----------------##
sed -i /etc/sudoers -re 's/^%sudo.*/%sudo ALL=(ALL:ALL) NOPASSWD: ALL/g'

##----------------##
## default shell  ##
##----------------##
chsh -s /bin/bash
ln -s /bin/bash /bin/sh -f

# # Kick down the ladder
# apt-get -y autoremove python3-pip

# Clean up cache to reduce layer size.
apt-get clean && \
    rm -rf /var/lib/apt/lists/*
