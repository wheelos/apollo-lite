#ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.4.1
ARG BASE_IMAGE=nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04
FROM ${BASE_IMAGE}

ARG TENSORRT_VERSION="8.5.2"
ARG PATCH_SUFFIX="-1+cuda11.4"
ARG GEOLOC
ARG DEBIAN_FRONTEND=noninteractive

LABEL maintainer="WheelOS <developer@wheelos.cn>"

# update ca-certificates and gnupg2 for https source first
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -qq -y --no-install-recommends \
        ca-certificates \
        gnupg2 && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# consistent with the host system, for jetson orin, if not, please modify it manually
ADD https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
RUN chmod +r /etc/apt/trusted.gpg.d/jetson-ota-public.asc
COPY rcfiles/nvidia-jetson-common-r35.4-main.list /etc/apt/sources.list.d/nvidia-jetson-common-r35.4-main.list

# change source list for cn
COPY rcfiles/sources.list.tsinghua.aarch64.ubuntu.20.04 /etc/apt/sources.list

# Add NVIDIA Tegra library paths
RUN echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf

#
# Install nvidia-cuda-dev for CUDA developer packages
# Use nvidia-cuda if need CUDA runtime only
#
# RUN --mount=type=bind,source=rcfiles/sources.list.local.aarch64.ubuntu.20.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        nvidia-cuda-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

#
# Install nvidia-cudnn8-dev for CuDNN developer packages
# Use nvidia-cudnn8 if need CuDNN runtime only
#
# RUN --mount=type=bind,source=rcfiles/sources.list.local.aarch64.ubuntu.20.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        nvidia-cudnn8-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

#
# Install nvidia-tensorrt-dev for TensorRT developer packages
# Use nvidia-tensorrt if need TensorRT runtime only
#
# RUN --mount=type=bind,source=rcfiles/sources.list.local.aarch64.ubuntu.20.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        nvidia-tensorrt-dev && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

ENV TENSORRT_VERSION=${TENSORRT_VERSION}
