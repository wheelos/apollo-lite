FROM nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04

ARG TENSORRT_VERSION="10.9.0.34"
ARG PATCH_SUFFIX="-1+cuda12.8"

LABEL maintainer="WheelOS <developer@wheelos.cn>"

COPY rcfiles/sources.list.tsinghua.x86_64.ubuntu.22.04 /etc/apt/sources.list

ENV DEBIAN_FRONTEND=noninteractive

# Note:
# The `--mount` option is used to bind mount the local sources.list file into the container during the build process.
# We can use it to speed up the apt-get update process by using a local mirror.
RUN --mount=type=bind,source=rcfiles/sources.list.local.x86_64.ubuntu.22.04,target=/etc/apt/sources.list \
    --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        libnvinfer10="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-plugin10="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-vc-plugin10="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-headers-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-headers-plugin-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvinfer-plugin-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvonnxparsers10="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        libnvonnxparsers-dev="${TENSORRT_VERSION}${PATCH_SUFFIX}" \
        python3-libnvinfer="${TENSORRT_VERSION}${PATCH_SUFFIX}" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /usr/lib/x86_64-linux-gnu/libnvinfer_builder_resource* /usr/lib/x86_64-linux-gnu/libnvinfer_static.a

ENV TENSORRT_VERSION=${TENSORRT_VERSION}
