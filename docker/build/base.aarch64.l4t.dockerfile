# Base image with CUDA 12.6 and TensorRT 10.3 for JetPack 6.2.1
# https://docs.nvidia.com/jetson/jetpack/release-notes/index.html
FROM nvcr.io/nvidia/l4t-tensorrt:r10.3.0-devel

ENV DEBIAN_FRONTEND=noninteractive

# Install cuDNN 9.3 using best practices for security and image size
RUN apt-get update && \
    # Install prerequisite tools
    apt-get install -y --no-install-recommends ca-certificates curl gnupg2 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# consistent with the host system, for jetson orin, if not, please modify it manually
ADD https://repo.download.nvidia.com/jetson/jetson-ota-public.asc /etc/apt/trusted.gpg.d/jetson-ota-public.asc
RUN chmod +r /etc/apt/trusted.gpg.d/jetson-ota-public.asc
COPY rcfiles/nvidia-jetson-common-r36.4-main.list /etc/apt/sources.list.d/nvidia-jetson-common-r36.4-main.list

# change source list for cn
COPY rcfiles/sources.list.tsinghua.aarch64.ubuntu.22.04 /etc/apt/sources.list

# Note:
# The `--mount` option is used to bind mount the local sources.list file into the container during the build process.
# We can use it to speed up the apt-get update process by using a local mirror.
# RUN --mount=type=bind,source=rcfiles/sources.list.local.aarch64.ubuntu.22.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
RUN apt-get update && \
    # Install cuDNN runtime and development libraries, avoiding recommended packages
    apt-get install -y --no-install-recommends \
        libcudnn9 libcudnn9-dev && \
    # Clean up APT cache and temporary files to reduce image size
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

LABEL org.opencontainers.image.authors="WheelOS <developer@wheelos.cn>" \
      org.opencontainers.image.title="WheelOS Base Builder with cuDNN" \
      org.opencontainers.image.description="Build environment for WheelOS base, including CUDA, TensorRT, and cuDNN."

# Verify the installation (optional but recommended)
RUN echo "Verifying cuDNN installation..." && \
    dpkg -l | grep libcudnn9 && \
    ls /usr/include/cudnn.h

CMD ["/bin/bash"]
