ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG APOLLO_DIST
ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE
ARG LOCAL_HTTP_ADDR
ARG DEBIAN_FRONTEND=noninteractive

ENV APOLLO_DIST=${APOLLO_DIST} \
    PATH="/opt/apollo/sysroot/bin:${PATH}"

COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

# adjust source list by geo first
RUN bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}

# Note:
# The `--mount` option is used to bind mount the local sources.list file into the container during the build process.
# We can use it to speed up the apt-get update process by using a local mirror.
# RUN --mount=type=bind,source=rcfiles/sources.list.local.aarch64.ubuntu.20.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
#     --mount=type=bind,source=rcfiles/pip.conf.local,target=/root/.config/pip/pip.conf \
RUN bash /opt/apollo/installers/install_minimal_environment.sh ${GEOLOC} \
    && bash /opt/apollo/installers/install_bazel.sh \
    && bash /opt/apollo/installers/install_cmake.sh ${INSTALL_MODE} \
    && bash /opt/apollo/installers/install_llvm_clang.sh \
    && bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE} \
    && bash /opt/apollo/installers/install_visualizer_deps.sh ${INSTALL_MODE} \
    && bash /opt/apollo/installers/post_install.sh cyber

WORKDIR /apollo
