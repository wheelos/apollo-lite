ARG BASE_IMAGE=nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04
FROM ${BASE_IMAGE}

ARG CUDA_TOOLKIT_VERSION="12.6.68"
ARG CUDNN_VERSION="9.3.0.75"
ARG TENSORRT_VERSION="10.3.0.30"
ARG VPI_VERSION="3.2.4"
ARG VULKAN_VERSION="1.3.204"

LABEL maintainer="WheelOS <developer@wheelos.cn>"

ENV DEBIAN_FRONTEND=noninteractive

# Note:
# The `--mount` option is used to bind mount the local sources.list file into the container during the build process.
# We can use it to speed up the apt-get update process by using a local mirror.
# RUN --mount=type=bind,source=rcfiles/sources.list.local.x86_64.ubuntu.20.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2 \
        libvulkan1 \
        vulkan-tools && \
    (apt-get install -y --no-install-recommends nvidia-tensorrt-dev \
      || apt-get install -y --no-install-recommends tensorrt-dev \
      || apt-get install -y --no-install-recommends \
          libnvinfer-dev \
          libnvinfer-plugin-dev \
          libnvonnxparsers-dev \
          libnvparsers-dev) && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_TOOLKIT_VERSION=${CUDA_TOOLKIT_VERSION}
ENV CUDNN_VERSION=${CUDNN_VERSION}
ENV TENSORRT_VERSION=${TENSORRT_VERSION}
ENV VPI_VERSION=${VPI_VERSION}
ENV VULKAN_VERSION=${VULKAN_VERSION}
