# syntax=docker/dockerfile:1.7

# Three targets:
#   dev     - devcontainer: toolchain, ROS dev packages, non-root user
#   build   - compiles the source and installs into /opt/livo-slam
#   runtime - slim image carrying only the install tree and its runtime deps

ARG ROS_DISTRO=jazzy
ARG LLVM_VERSION=22
ARG GCC_VERSION=14
ARG INSTALL_PREFIX=/opt/livo-slam

# ---------------------------------------------------------------- dev --------
FROM ros:${ROS_DISTRO}-ros-base AS dev

ARG ROS_DISTRO
ARG LLVM_VERSION
ARG GCC_VERSION
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# apt.llvm.org keeps only the newest build of each major version, so pinning an
# exact clang patch version breaks the image as soon as they publish a rebuild.
# The major version is what matters: clang-format output is stable within it.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates gnupg wget \
 && wget -qO /etc/apt/trusted.gpg.d/apt.llvm.org.asc https://apt.llvm.org/llvm-snapshot.gpg.key \
 && echo "deb http://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
      > /etc/apt/sources.list.d/llvm.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      clang-${LLVM_VERSION} \
      clangd-${LLVM_VERSION} \
      clang-format-${LLVM_VERSION} \
      clang-tidy-${LLVM_VERSION} \
      lld-${LLVM_VERSION} \
      lldb-${LLVM_VERSION} \
      g++-${GCC_VERSION} \
      cmake ninja-build ccache git gdb sudo \
      ros-${ROS_DISTRO}-cv-bridge \
      ros-${ROS_DISTRO}-image-transport \
 && rm -rf /var/lib/apt/lists/*

# Ubuntu 24.04 ships a stock "ubuntu" account already sitting on uid 1000, which
# collides with the host uid we want the dev user to own.
RUN existing="$(getent passwd ${USER_UID} | cut -d: -f1)" \
 && if [ -n "${existing}" ] && [ "${existing}" != "${USERNAME}" ]; then \
      userdel -r "${existing}" 2>/dev/null || true; \
    fi \
 && getent group ${USER_GID} >/dev/null || groupadd -g ${USER_GID} ${USERNAME} \
 && getent passwd ${USERNAME} >/dev/null \
    || useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc \
 && install -d -o ${USER_UID} -g ${USER_GID} /workspace /data

ENV CC=clang-${LLVM_VERSION} \
    CXX=clang++-${LLVM_VERSION} \
    GCC_INSTALL_DIR=/usr/lib/gcc/x86_64-linux-gnu/${GCC_VERSION}

USER ${USERNAME}
WORKDIR /workspace

# -------------------------------------------------------------- build --------
FROM dev AS build

ARG INSTALL_PREFIX

USER root
WORKDIR /src
COPY . .

# FetchContent downloads and ccache live on cache mounts so an image rebuild does
# not refetch and recompile every dependency.
RUN --mount=type=cache,target=/cache/fetchcontent,sharing=locked \
    --mount=type=cache,target=/cache/ccache,sharing=locked \
    CCACHE_DIR=/cache/ccache \
    cmake --preset release -DFETCHCONTENT_BASE_DIR=/cache/fetchcontent \
 && cmake --build --preset release \
 && cmake --install build/release --prefix ${INSTALL_PREFIX}

# ------------------------------------------------------------ runtime --------
FROM ros:${ROS_DISTRO}-ros-core AS runtime

ARG ROS_DISTRO
ARG INSTALL_PREFIX
ARG USERNAME=livo

ENV DEBIAN_FRONTEND=noninteractive

# ros-core lacks rosbag2, which ros-base carries. Adding just the pieces we link
# against keeps this image well under the 887 MB of a full ros-base.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rosbag2-cpp \
      ros-${ROS_DISTRO}-rosbag2-storage-mcap \
      ros-${ROS_DISTRO}-cv-bridge \
 && rm -rf /var/lib/apt/lists/* \
 && useradd -m -s /bin/bash ${USERNAME} \
 && install -d -o ${USERNAME} -g ${USERNAME} /data

COPY --from=build ${INSTALL_PREFIX} ${INSTALL_PREFIX}

ENV PATH=${INSTALL_PREFIX}/bin:${PATH} \
    LD_LIBRARY_PATH=${INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH}

USER ${USERNAME}
WORKDIR /data

# /ros_entrypoint.sh from the base image already sources setup.bash, which is what
# makes the rosidl typesupport and rosbag2 storage plugins loadable at runtime.
CMD ["livo_slam"]
