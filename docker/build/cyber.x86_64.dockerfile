ARG BASE_IMAGE
FROM ${BASE_IMAGE} as dev

ARG APOLLO_DIST
ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE
ARG DEBIAN_FRONTEND=noninteractive

ENV APOLLO_DIST=${APOLLO_DIST} \
    PATH="/opt/apollo/sysroot/bin:${PATH}"

COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

RUN set -eux; \
    bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}; \
    bash /opt/apollo/installers/install_minimal_environment.sh ${GEOLOC}; \
    bash /opt/apollo/installers/install_cmake.sh; \
    bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_llvm_clang.sh; \
    bash /opt/apollo/installers/install_qa_tools.sh; \
    bash /opt/apollo/installers/install_visualizer_deps.sh; \
    bash /opt/apollo/installers/install_bazel.sh; \
    bash /opt/apollo/installers/post_install.sh cyber

WORKDIR /apollo

# ---------------------------------

FROM ${BASE_IMAGE} as runtime

ARG APOLLO_DIST
ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE
ARG DEBIAN_FRONTEND=noninteractive

ENV APOLLO_DIST=${APOLLO_DIST} \
    PATH="/opt/apollo/sysroot/bin:${PATH}"

COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

RUN set -eux; \
    bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}; \
    bash /opt/apollo/installers/install_minimal_environment.sh ${GEOLOC}; \
    bash /opt/apollo/installers/install_bazel.sh; \
    bash /opt/apollo/installers/install_cmake.sh; \
    bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_visualizer_deps.sh; \
    bash /opt/apollo/installers/post_install.sh cyber

WORKDIR /apollo
