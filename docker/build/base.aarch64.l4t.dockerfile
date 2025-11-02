FROM nvcr.io/nvidia/l4t-tensorrt:r10.3.0-devel

ENV DEBIAN_FRONTEND=noninteractive

LABEL org.opencontainers.image.authors="WheelOS <developer@wheelos.cn>" \
      org.opencontainers.image.title="WheelOS Base Builder" \
      org.opencontainers.image.description="Build environment for WheelOS base"
