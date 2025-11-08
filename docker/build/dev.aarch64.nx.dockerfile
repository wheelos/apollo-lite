FROM wheelos/apollo:cuda12.6-cudnn9-trt10-devel-22.04-aarch64-20251103_1226
ENV CUDA_LITE 12.6

ENV CUDA_VERSION 12.6.0

ENV PATH /usr/local/cuda/bin:/opt/apollo/sysroot/bin:${PATH}

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=${CUDA_LITE}"

ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

ENV CUDNN_VERSION 9.3.0

ENV TENSORRT_VERSION 10.3.0

COPY rcfiles /opt/apollo/rcfiles
COPY installers /opt/apollo/installers

RUN rm -f /etc/apt/sources.list && cp /opt/apollo/rcfiles/sources.list.cn.aarch64.ubuntu.22.04 /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends sudo gnupg2 curl ca-certificates \
    && rm -rf /var/lib/apt/lists/* /usr/local/bin/cmake /usr/local/share/cmake-3.14

RUN apt update && DEBIAN_FRONTEND=noninteractive TZ="Asian/China" apt-get -y install tzdata

RUN bash /opt/apollo/installers/install_minimal_environment.sh cn
RUN bash /opt/apollo/installers/install_bazel.sh
RUN bash /opt/apollo/installers/install_cmake.sh build

RUN bash /opt/apollo/installers/install_llvm_clang.sh
RUN bash /opt/apollo/installers/install_visualizer_deps.sh

RUN bash /opt/apollo/installers/install_modules_base.sh
RUN bash /opt/apollo/installers/install_ordinary_modules.sh --all
RUN bash /opt/apollo/installers/install_gpu_support.sh
RUN bash /opt/apollo/installers/install_release_deps.sh

RUN bash /opt/apollo/installers/post_install.sh dev

RUN bash /opt/apollo/installers/install_cyber_deps.sh
