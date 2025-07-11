ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS dev

ARG GEOLOC
ARG CLEAN_DEPS
ARG APOLLO_DIST
ARG INSTALL_MODE

ARG INSTALL_MODULES="common,dreamview,map,monitor,planning,routing,task_manager,tools,transform"

COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_ordinary_modules.sh --modules ${INSTALL_MODULES}; \
  bash /opt/apollo/installers/install_release_deps.sh; \
  bash /opt/apollo/installers/post_install.sh dev

COPY rcfiles/setup.sh /opt/apollo/neo/

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc

# ---------------------------------

FROM ${BASE_IMAGE} AS runtime

ARG GEOLOC
ARG CLEAN_DEPS
ARG APOLLO_DIST
ARG INSTALL_MODE

ARG INSTALL_MODULES="common,dreamview,map,monitor,planning,routing,task_manager,tools,transform"

COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_ordinary_modules.sh --modules ${INSTALL_MODULES}; \
  bash /opt/apollo/installers/install_release_deps.sh; \
  bash /opt/apollo/installers/post_install.sh dev

COPY rcfiles/setup.sh /opt/apollo/neo/

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc
