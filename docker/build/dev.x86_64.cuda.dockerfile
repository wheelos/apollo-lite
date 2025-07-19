ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG CLEAN_DEPS
ARG APOLLO_DIST
ARG INSTALL_MODE
ARG LOCAL_HTTP_ADDR
ARG DEBIAN_FRONTEND=noninteractive

COPY installers /opt/apollo/installers

# Note:
# The `--mount` option is used to bind mount the local sources.list file into the container during the build process.
# We can use it to speed up the apt-get update process by using a local mirror.
# RUN --mount=type=bind,source=rcfiles/sources.list.local.x86_64.ubuntu.20.04,target=/etc/apt/sources.list \
#     --mount=type=bind,source=rcfiles/wheelos.cn.public.gpg,target=/opt/apollo/rcfiles/wheelos.cn.public.gpg \
#     --mount=type=bind,source=rcfiles/pip.conf.local,target=/root/.config/pip/pip.conf \
RUN bash -eux /opt/apollo/installers/install_modules_base.sh \
    && bash -eux /opt/apollo/installers/install_ordinary_modules.sh --all \
    && bash -eux /opt/apollo/installers/install_drivers_deps.sh ${INSTALL_MODE} \
    && bash -eux /opt/apollo/installers/install_dreamview_deps.sh ${GEOLOC} \
    && bash -eux /opt/apollo/installers/install_gpu_support.sh \
    && bash -eux /opt/apollo/installers/install_release_deps.sh \
    && bash -eux /opt/apollo/installers/post_install.sh dev

COPY rcfiles/setup.sh /opt/apollo/neo/

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc
