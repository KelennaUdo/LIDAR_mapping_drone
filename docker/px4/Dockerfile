# syntax=docker/dockerfile:1.7

FROM ubuntu:24.04

ARG PX4_VERSION=v1.17.0
ARG HOST_UID=1000
ARG HOST_GID=1000

LABEL org.opencontainers.image.title="PX4 SITL Environment"
LABEL org.opencontainers.image.version="${PX4_VERSION}"
LABEL org.opencontainers.image.description="PX4 SITL and Gazebo dependencies; PX4 source is mounted at runtime"

ENV DEBIAN_FRONTEND=noninteractive
ENV RUNS_IN_DOCKER=true

# PX4 owns the dependency list. The named build context exposes only the
# matching release's Tools/setup directory and does not copy PX4 source into
# the resulting image.
RUN --mount=type=bind,from=px4_setup,source=/,target=/tmp/px4-setup,readonly \
    test -f /tmp/px4-setup/ubuntu.sh && \
    test -f /tmp/px4-setup/requirements.txt && \
    bash /tmp/px4-setup/ubuntu.sh --no-nuttx && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Match the host user so builds written to the external source directory are
# not owned by root. Some Ubuntu base images already reserve UID/GID 1000, so
# reuse that account instead of assuming both identifiers are free.
RUN set -e; \
    existing_user="$(getent passwd "${HOST_UID}" | cut -d: -f1 || true)"; \
    if [ -n "$existing_user" ]; then \
      usermod --login px4 --home /home/px4 --move-home \
        --shell /bin/bash "$existing_user"; \
    else \
      if ! getent group "${HOST_GID}" >/dev/null; then \
        groupadd --gid "${HOST_GID}" px4; \
      fi; \
      useradd --create-home --uid "${HOST_UID}" --gid "${HOST_GID}" \
        --shell /bin/bash px4; \
    fi

ENV HOME=/home/px4

USER px4
WORKDIR /workspace/PX4-Autopilot

CMD ["bash"]
