ARG BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r36.4.0
FROM ${BASE_IMAGE}

ARG CUDA_TOOLKIT_VERSION="12.6.68"
ARG CUDNN_VERSION="9.3.0.75"
ARG TENSORRT_VERSION="10.3.0.30"
ARG VPI_VERSION="3.2.4"
ARG VULKAN_VERSION="1.3.204"
ARG GEOLOC
ARG DEBIAN_FRONTEND=noninteractive

LABEL maintainer="WheelOS <developer@wheelos.cn>"

# update ca-certificates and gnupg2 for https source first
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -qq -y --no-install-recommends \
        ca-certificates \
        gnupg2 \
        curl \
        libvulkan1 \
        vulkan-tools && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# Add NVIDIA Tegra library paths
RUN echo "/usr/lib/aarch64-linux-gnu/tegra" >> /etc/ld.so.conf.d/nvidia-tegra.conf && \
    echo "/usr/lib/aarch64-linux-gnu/tegra-egl" >> /etc/ld.so.conf.d/nvidia-tegra.conf

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

ENV CUDA_TOOLKIT_VERSION=${CUDA_TOOLKIT_VERSION}
ENV CUDNN_VERSION=${CUDNN_VERSION}
ENV TENSORRT_VERSION=${TENSORRT_VERSION}
ENV VPI_VERSION=${VPI_VERSION}
ENV VULKAN_VERSION=${VULKAN_VERSION}
