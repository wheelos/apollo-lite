ARG BASE_IMAGE
FROM ${BASE_IMAGE} as dev

ARG GEOLOC
ARG CLEAN_DEPS
ARG APOLLO_DIST
ARG INSTALL_MODE

COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_modules_base.sh; \
    bash /opt/apollo/installers/install_ordinary_modules.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_drivers_deps.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_dreamview_deps.sh ${GEOLOC}; \
    bash /opt/apollo/installers/install_contrib_deps.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_gpu_support.sh; \
    bash /opt/apollo/installers/install_release_deps.sh; \
    bash /opt/apollo/installers/post_install.sh dev

COPY rcfiles/setup.sh /opt/apollo/neo/

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc

# ---------------------------------

FROM ${BASE_IMAGE} as runtime

ARG GEOLOC
ARG CLEAN_DEPS
ARG APOLLO_DIST
ARG INSTALL_MODE

COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_modules_base.sh; \
    bash /opt/apollo/installers/install_ordinary_modules.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_drivers_deps.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_dreamview_deps.sh ${GEOLOC}; \
    bash /opt/apollo/installers/install_contrib_deps.sh ${INSTALL_MODE}; \
    bash /opt/apollo/installers/install_gpu_support.sh; \
    bash /opt/apollo/installers/install_release_deps.sh; \
    bash /opt/apollo/installers/post_install.sh release

COPY rcfiles/setup.sh /opt/apollo/neo/

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc
