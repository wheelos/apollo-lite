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

if ldconfig -p | grep -q libopencv_core; then
    info "OpenCV was already installed"
    exit 0
fi

WORKHORSE="cpu"

# 1) Install OpenCV via apt
# apt-get -y update && \
#    apt-get -y install \
#    libopencv-core-dev \
#    libopencv-imgproc-dev \
#    libopencv-imgcodecs-dev \
#    libopencv-highgui-dev \
#    libopencv-dev
# 2) Build OpenCV from source
# RTFM: https://src.fedoraproject.org/rpms/opencv/blob/master/f/opencv.spec

apt_get_update_and_install \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libgtk2.0-dev \
    libv4l-dev \
    libeigen3-dev \
    libopenblas-dev \
    libatlas-base-dev \
    libxvidcore-dev \
    libx264-dev \
    libopenni-dev \
    libwebp-dev

pip3_install numpy

VERSION="${OPENCV_VERSION:-4.8.0}"

git clone --depth 1 --branch "${VERSION}" https://github.com/opencv/opencv.git "opencv-${VERSION}"

# https://stackoverflow.com/questions/12427928/configure-and-build-opencv-to-custom-ffmpeg-install
# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${SYSROOT_DIR}/lib
# export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:${SYSROOT_DIR}/lib/pkgconfig
# export PKG_CONFIG_LIBDIR=$PKG_CONFIG_LIBDIR:${SYSROOT_DIR}/lib

# libgtk-3-dev libtbb2 libtbb-dev
# -DWITH_GTK=ON -DWITH_TBB=ON

GPU_OPTIONS="-DWITH_CUDA=OFF"

# keep opencv_contrib disabled in docker baseline for smaller and faster builds.

TARGET_ARCH="$(uname -m)"

EXTRA_OPTIONS=
if [ "${TARGET_ARCH}" = "x86_64" ]; then
    EXTRA_OPTIONS="${EXTRA_OPTIONS} -DCPU_BASELINE=SSE4"
fi

EXTRA_OPTIONS="${EXTRA_OPTIONS} -DBUILD_opencv_world=OFF"

# -DBUILD_LIST=core,highgui,improc
pushd "opencv-${VERSION}"
    [[ ! -e build ]] && mkdir build
    pushd build
        cmake .. \
            -DCMAKE_INSTALL_PREFIX="${SYSROOT_DIR}" \
            -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_SHARED_LIBS=ON \
            -DWITH_NATIVE_OPTIMIZED_CODE=ON \
            -DENABLE_PRECOMPILED_HEADERS=OFF \
            -DOPENCV_GENERATE_PKGCONFIG=ON \
            -DBUILD_EXAMPLES=OFF \
            -DBUILD_DOCS=OFF \
            -DBUILD_TESTS=OFF \
            -DBUILD_PERF_TESTS=OFF \
            -DBUILD_JAVA=OFF \
            -DBUILD_PROTOBUF=OFF \
            -DPROTOBUF_UPDATE_FILES=ON \
            -DINSTALL_C_EXAMPLES=OFF \
            -DWITH_QT=OFF \
            -DWITH_GTK=ON \
            -DWITH_GTK_2_X=ON \
            -DWITH_IPP=OFF \
            -DWITH_ITT=OFF \
            -DWITH_TBB=OFF \
            -DWITH_EIGEN=ON \
            -DWITH_FFMPEG=ON \
            -DWITH_LIBV4L=ON \
            -DWITH_OPENMP=ON \
            -DWITH_OPENNI=ON \
            -DWITH_OPENCL=ON \
            -DWITH_WEBP=ON \
            -DOpenGL_GL_PREFERENCE=GLVND \
            -DBUILD_opencv_python2=OFF \
            -DBUILD_opencv_python3=ON \
            -DBUILD_NEW_PYTHON_SUPPORT=ON \
            -DPYTHON_DEFAULT_EXECUTABLE="$(which python3)" \
            -DOPENCV_PYTHON3_INSTALL_PATH="/usr/local/lib/python$(py3_version)/dist-packages" \
            -DOPENCV_ENABLE_NONFREE=ON \
            -DCV_TRACE=OFF \
            ${GPU_OPTIONS} \
            ${EXTRA_OPTIONS}
        make -j$(nproc)
        make install
    popd
popd

ldconfig
ok "Successfully installed OpenCV ${VERSION}."

rm -rf opencv*

if [[ -n "${CLEAN_DEPS}" ]]; then
    apt_get_remove \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libv4l-dev \
        libeigen3-dev \
        libopenblas-dev \
        libatlas-base-dev \
        libxvidcore-dev \
        libx264-dev \
        libgtk2.0-dev \
        libopenni-dev
    apt_get_update_and_install libgtk2.0-0
fi
