# Base image with CUDA 12.6 and TensorRT 10.3 for JetPack 6.2.1
# https://docs.nvidia.com/jetson/jetpack/release-notes/index.html
FROM nvcr.io/nvidia/l4t-tensorrt:r10.3.0-devel

ENV DEBIAN_FRONTEND=noninteractive

# Install cuDNN 9.3 using best practices for security and image size
RUN apt-get update && \
    # Install prerequisite tools
    apt-get install -y --no-install-recommends ca-certificates curl gnupg2 && \
    # Add NVIDIA's GPG key securely to a dedicated keyring file
    curl -fsSL http://l4t-repo.nvidia.com/jetson-ota-internal.key | gpg --dearmor -o /usr/share/keyrings/nvidia-l4t-apt-keyring.gpg && \
    # Add the L4T repositories for JetPack 6.x (r36.4) and Orin (t234)
    echo "deb [signed-by=/usr/share/keyrings/nvidia-l4t-apt-keyring.gpg] http://l4t-repo.nvidia.com/common r36.4 main" > /etc/apt/sources.list.d/nvidia-l4t.list && \
    echo "deb [signed-by=/usr/share/keyrings/nvidia-l4t-apt-keyring.gpg] http://l4t-repo.nvidia.com/t234 r36.4 main" >> /etc/apt/sources.list.d/nvidia-l4t.list && \
    # Update package list again with the new repositories
    apt-get update && \
    # Install cuDNN runtime and development libraries, avoiding recommended packages
    apt-get install -y --no-install-recommends libcudnn9 libcudnn9-dev && \
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
