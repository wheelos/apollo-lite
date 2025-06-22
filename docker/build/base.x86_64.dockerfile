ARG BASE_IMAGE=nvidia/cuda:11.8.0-cudnn8-devel-ubuntu20.04
FROM ${BASE_IMAGE}

ARG TENSORRT_VERSION="8.6.1.6"
ARG PATCH_SUFFIX="-1+cuda11.8"

LABEL maintainer="WheelOS <developer@wheelos.cn>"

COPY rcfiles/sources.list.tsinghua /etc/apt/sources.list

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        libnvinfer8="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-plugin8="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-vc-plugin8="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-headers-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-headers-plugin-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-plugin-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvonnxparsers8="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvonnxparsers-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvparsers8="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvparsers-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        python3-libnvinfer="${TENSORRT_VERSION}${PATCH_SUFFIX}" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV TENSORRT_VERSION=${TENSORRT_VERSION}
