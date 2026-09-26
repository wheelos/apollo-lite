# syntax=docker/dockerfile:1.7

ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE} AS base

ARG BASE_VARIANT=cpu-u22
ARG GEOLOC=cn
ENV DEBIAN_FRONTEND=noninteractive

COPY docker/build/rcfiles /opt/apollo/rcfiles

RUN set -eux; \
    case "${GEOLOC}" in \
      cn|us) \
        ;; \
      *) \
        echo "Unsupported GEOLOC: ${GEOLOC}" >&2; \
        exit 1; \
        ;; \
    esac; \
    case "${BASE_VARIANT}" in \
      cpu-u20|cpu-u22|cpu-arm64-u22) \
        ;; \
      cuda-u22) \
        case "${GEOLOC}" in \
          cn) \
            cp /opt/apollo/rcfiles/sources.list.cn.x86_64.ubuntu.22.04 /etc/apt/sources.list; \
            ;; \
          us) \
            cp /opt/apollo/rcfiles/sources.list.us.x86_64.ubuntu.22.04 /etc/apt/sources.list; \
            ;; \
        esac; \
        apt-get update; \
        apt-get install -y --no-install-recommends \
          libnvinfer10="10.9.0.34-1+cuda12.8" \
          libnvinfer-plugin10="10.9.0.34-1+cuda12.8" \
          libnvinfer-vc-plugin10="10.9.0.34-1+cuda12.8" \
          libnvinfer-dev="10.9.0.34-1+cuda12.8" \
          libnvinfer-headers-dev="10.9.0.34-1+cuda12.8" \
          libnvinfer-headers-plugin-dev="10.9.0.34-1+cuda12.8" \
          libnvinfer-plugin-dev="10.9.0.34-1+cuda12.8" \
          libnvonnxparsers10="10.9.0.34-1+cuda12.8" \
          libnvonnxparsers-dev="10.9.0.34-1+cuda12.8" \
          python3-libnvinfer="10.9.0.34-1+cuda12.8"; \
        rm -f /usr/lib/x86_64-linux-gnu/libnvinfer_static.a; \
        apt-get clean; \
        rm -rf /var/lib/apt/lists/*; \
        ;; \
      orin-jp621-l4t3643) \
        case "${GEOLOC}" in \
          cn) \
            cp /opt/apollo/rcfiles/sources.list.cn.aarch64.ubuntu.22.04 /etc/apt/sources.list; \
            ;; \
          us) \
            cp /opt/apollo/rcfiles/sources.list.us.aarch64.ubuntu.22.04 /etc/apt/sources.list; \
            ;; \
        esac; \
        apt-get update; \
        apt-get install -y --no-install-recommends ca-certificates curl gnupg2; \
        curl -fsSL https://repo.download.nvidia.com/jetson/jetson-ota-public.asc \
          -o /etc/apt/trusted.gpg.d/jetson-ota-public.asc; \
        chmod +r /etc/apt/trusted.gpg.d/jetson-ota-public.asc; \
        cp /opt/apollo/rcfiles/nvidia-jetson-common-r36.4-main.list \
          /etc/apt/sources.list.d/nvidia-jetson-common-r36.4-main.list; \
        apt-get update; \
        apt-get install -y --no-install-recommends libcudnn9 libcudnn9-dev; \
        apt-get clean; \
        rm -rf /var/lib/apt/lists/*; \
        ;; \
      *) \
        echo "Unsupported BASE_VARIANT: ${BASE_VARIANT}" >&2; \
        exit 1; \
        ;; \
    esac

FROM base AS dev

ARG APOLLO_DIST=stable
ARG GEOLOC=cn
ARG LOCAL_HTTP_ADDR
ARG CLEAN_DEPS
ARG GPU_SUPPORT=0
ARG DEBIAN_FRONTEND=noninteractive

ENV APOLLO_DIST=${APOLLO_DIST} \
    DEBIAN_FRONTEND=${DEBIAN_FRONTEND} \
    LOCAL_HTTP_ADDR=${LOCAL_HTTP_ADDR} \
    PATH="/opt/apollo/sysroot/bin:${PATH}"

COPY docker/build/installers/installer_base.sh \
     docker/build/installers/install_geo_adjustment.sh \
     docker/build/installers/install_minimal_environment.sh \
     docker/build/installers/install_bazel.sh \
     docker/build/installers/install_cmake.sh \
     docker/build/installers/install_llvm_clang.sh \
     docker/build/installers/install_protobuf_release.sh \
     docker/build/installers/install_visualizer_deps.sh \
     docker/build/installers/install_qt.sh \
     docker/build/installers/install_qt5_qtbase.sh \
     docker/build/installers/post_install.sh \
     docker/build/installers/py3_requirements.txt \
     /opt/apollo/installers/
COPY .bazelversion /opt/apollo/bazelversion

RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    set -eux; \
    bash /opt/apollo/installers/install_geo_adjustment.sh "${GEOLOC}"; \
    bash /opt/apollo/installers/install_minimal_environment.sh; \
    BAZEL_VERSION="$(cat /opt/apollo/bazelversion)" \
      bash /opt/apollo/installers/install_bazel.sh; \
    bash /opt/apollo/installers/install_cmake.sh; \
    bash /opt/apollo/installers/install_llvm_clang.sh; \
    bash /opt/apollo/installers/install_protobuf_release.sh; \
    bash /opt/apollo/installers/install_visualizer_deps.sh

COPY docker/build/installers/install_modules_base.sh \
     docker/build/installers/install_ordinary_modules.sh \
     docker/build/installers/install_drivers_deps.sh \
     docker/build/installers/install_dreamview_deps.sh \
     docker/build/installers/install_gpu_support.sh \
     docker/build/installers/install_release_deps.sh \
     docker/build/installers/install_ffmpeg.sh \
     docker/build/installers/install_openh264.sh \
     docker/build/installers/install_patchelf.sh \
     docker/build/installers/install_python_modules.sh \
     docker/build/installers/install_perception_deps.sh \
     docker/build/installers/install_adolc.sh \
     docker/build/installers/install_ipopt.sh \
     docker/build/installers/install_libtorch.sh \
     /opt/apollo/installers/

RUN --mount=type=cache,target=/root/.cache/pip,sharing=locked \
    set -eux; \
    bash /opt/apollo/installers/install_modules_base.sh; \
    GPU_SUPPORT="${GPU_SUPPORT}" bash /opt/apollo/installers/install_ordinary_modules.sh --all; \
    if [ "${GPU_SUPPORT}" = "1" ]; then \
      bash /opt/apollo/installers/install_gpu_support.sh; \
    fi; \
    bash /opt/apollo/installers/install_release_deps.sh; \
    bash /opt/apollo/installers/post_install.sh dev

COPY docker/build/rcfiles/setup.sh /opt/apollo/neo/setup.sh
RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc

WORKDIR /apollo
