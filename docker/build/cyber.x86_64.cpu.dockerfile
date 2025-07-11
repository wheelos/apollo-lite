FROM ubuntu:20.04 AS base

LABEL maintainer="WheelOS <developer@wheelos.cn>"

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# Development Stage: Contains all tools and dependencies required to build and develop Apollo
# -----------------------------------------------------------------------------
FROM base AS dev

ARG APOLLO_DIST
ARG GEOLOC
ARG INSTALL_MODE
ARG COMPUTE_MODE="cpu"
ENV APOLLO_DIST=${APOLLO_DIST} \
  PATH="/opt/apollo/sysroot/bin:${PATH}"

COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates

RUN set -eux; \
  bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}; \
  bash /opt/apollo/installers/install_minimal_environment.sh; \
  bash /opt/apollo/installers/install_bazel.sh; \
  bash /opt/apollo/installers/install_cmake.sh; \
  bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE}; \
  bash /opt/apollo/installers/post_install.sh cyber; \
  # Clean up build-time files and directories that are not needed in the final image
  rm -rf /opt/apollo/installers /opt/apollo/rcfiles

WORKDIR /apollo

# -----------------------------------------------------------------------------
# Runtime Stage: Contains only the minimal dependencies required to run Apollo
# -----------------------------------------------------------------------------
FROM base AS runtime

ARG APOLLO_DIST
ARG GEOLOC
ARG INSTALL_MODE

ENV APOLLO_DIST=${APOLLO_DIST} \
  PATH="/opt/apollo/sysroot/bin:${PATH}"

# Copy Apollo installation scripts and configuration files
COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates

# Run Apollo runtime installation scripts
RUN set -eux; \
  bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}; \
  bash /opt/apollo/installers/install_minimal_environment.sh; \
  bash /opt/apollo/installers/install_bazel.sh; \
  bash /opt/apollo/installers/install_cmake.sh; \
  bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE}; \
  bash /opt/apollo/installers/post_install.sh cyber; \
  \
  # Clean up installation files and directories
  rm -rf /opt/apollo/installers /opt/apollo/rcfiles

WORKDIR /apollo
