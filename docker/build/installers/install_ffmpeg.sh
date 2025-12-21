#!/usr/bin/env bash

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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

cd $( dirname "${BASH_SOURCE[0]}")
. ./installer_base.sh


VERSION="7.0.2"
CHECKSUM="8646515b638a3ad303e23af6a3587734447cb8fc0a0c064ecdb8e95c4fd8b389"

check_ffmpeg_installed() {
    local ffmpeg_bin="${SYSROOT_DIR}/bin/ffmpeg"

    if [[ -x "$ffmpeg_bin" ]]; then
        info "FFmpeg found at ${ffmpeg_bin}, skipping compilation."
        return 0
    else
        info "FFmpeg not found at ${ffmpeg_bin}, will compile and install."
        return 1
    fi
}

install_dependencies() {
    info "Installing build-time and run-time dependencies for FFmpeg..."
    # Combine all dependencies. The package manager will handle what's already installed.
    # Using modern library versions available in standard repos (e.g., Ubuntu 20.04/22.04).
    # Removed libnuma-dev as it's often a transitive dependency and should be handled
    # by packages that actually need it (like libipopt-dev).
    apt_get_update_and_install \
        nasm yasm \
        libx264-dev libx265-dev libvpx-dev \
        libfdk-aac-dev libmp3lame-dev libopus-dev \
        libtheora-dev libvorbis-dev \
        libass-dev libfreetype6-dev libva-dev libvdpau-dev
}

compile_ffmpeg() {
    info "Compiling and installing FFmpeg version ${VERSION}..."
    local pkg_name="ffmpeg-${VERSION}.tar.xz"
    local download_link="http://ffmpeg.org/releases/${pkg_name}"

    download_if_not_cached "${pkg_name}" "${CHECKSUM}" "${download_link}"

    info "Extracting ${pkg_name}..."
    tar -xf "${pkg_name}"

    pushd "ffmpeg-${VERSION}"
        # Configure FFmpeg with recommended flags.
        # --extra-cflags="-I${SYSROOT_DIR}/include" --extra-ldflags="-L${SYSROOT_DIR}/lib"
        # can help find dependencies in our custom prefix if needed.
        ./configure \
            --prefix="${SYSROOT_DIR}" \
            --pkg-config-flags="--static" \
            --extra-cflags="-I${SYSROOT_DIR}/include" \
            --extra-ldflags="-L${SYSROOT_DIR}/lib" \
            --enable-gpl \
            --enable-version3 \
            --enable-nonfree \
            --enable-shared \
            --disable-static \
            --disable-debug \
            --enable-libass \
            --enable-libfdk-aac \
            --enable-libfreetype \
            --enable-libmp3lame \
            --enable-libopus \
            --enable-libtheora \
            --enable-libvorbis \
            --enable-libvpx \
            --enable-libx264 \
            --enable-libx265

        info "Building with $(nproc) jobs..."
        make -j"$(nproc)"

        info "Installing..."
        make install
    popd
}

cleanup() {
    info "Cleaning up..."
    rm -rf "ffmpeg-${VERSION}.tar.xz" "ffmpeg-${VERSION}"

    if [[ -n "${CLEAN_DEPS}" ]]; then
        info "Removing build-time dependencies..."
        # We only list packages that are purely for building. The runtime libraries
        # (like libx264-164) will be kept as they are dependencies of the -dev packages.
        # The package manager is smart enough to not remove required runtime libs.
        apt_get_remove \
            nasm yasm \
            libx264-dev libx265-dev libvpx-dev \
            libfdk-aac-dev libmp3lame-dev libopus-dev \
            libtheora-dev libvorbis-dev \
            libass-dev libfreetype6-dev libva-dev libvdpau-dev
    fi
}

main() {
    if ! check_ffmpeg_installed; then
        install_dependencies
        compile_ffmpeg
        ldconfig
        cleanup
        info "FFmpeg ${VERSION} installation completed successfully."
    else
        info "Skipping FFmpeg installation."
    fi

    info "Verify with: ffmpeg -version"
}

# Run the main function.
main
