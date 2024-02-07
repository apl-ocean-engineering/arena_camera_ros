ARG ROS_VERSION=noetic

#====================================================================
# Preliminary image with dependencies
#====================================================================
#
## This should work on both amd64 and arm64 through the magic of multi-arch?
FROM ros:${ROS_VERSION}-perception AS deps

# This should be set automatically by the build engine
ARG TARGETPLATFORM
ARG ARENA_SDK_URL_BASE=https://s3.us-east-1.wasabisys.com/amarburg-east/LucidVision_SDK

# Install the LucidVision SDK
WORKDIR /tmp

RUN apt-get update && apt-get install -y \
    git \
    python3-catkin-tools \
    python3-vcstool \
    ros-noetic-diagnostic-updater \
    ros-noetic-roslint \
    ros-noetic-web-video-server \
    wget \
    && apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${TARGETPLATFORM}" = "linux/amd64" ]; then \
      ARENA_SDK_FILE=ArenaSDK_v0.1.68_Linux_x64.tar.gz; \
    elif [ "${TARGETPLATFORM}" = "linux/arm64" ]; then \
      ARENA_SDK_FILE=ArenaSDK_v0.1.49_Linux_ARM64.tar.gz; \
    else \
      echo "Unknown target platform ${TARGETPLATFORM}"; \
      exit; \
    fi && \
    ARENA_SDK_URL=${ARENA_SDK_URL_BASE}/${ARENA_SDK_FILE} && \
    echo "Downloading ${ARENA_SDK_URL}" && \
    wget $ARENA_SDK_URL && \
    cd /usr/local && \
    tar -xf /tmp/$ARENA_SDK_FILE && \
    rm /tmp/$ARENA_SDK_FILE

ENV ARENA_ROOT=/usr/local/ArenaSDK
# The name of the directory in the tarball changes by architecture,
# so create a convenience symlink
RUN ln -s /usr/local/ArenaSDK_* ${ARENA_ROOT}
WORKDIR ${ARENA_ROOT}
RUN sh -c "sudo sh Arena_SDK_*.conf"

#====================================================================
# Intermediate image with prereq steps for running in CI
#====================================================================
FROM deps as ci

## ~~ Create non-root user ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
  #
  # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

# Install the default ROS entrypoint
COPY --chmod=0755 ros_entrypoint.sh /home/$USERNAME/ros_entrypoint.sh
ENTRYPOINT [ "/home/ros/ros_entrypoint.sh" ]
CMD ["/bin/bash"]

#====================================================================
# Intermediate image with cloned Git repos
#====================================================================
FROM ci as clone

ARG WS_DIR=/home/$USERNAME/ros_ws
ONBUILD WORKDIR ${WS_DIR}/src

ARG ARENA_CAMERA_ROS_REPO=https://github.com/apl-ocean-engineering/arena_camera_ros.git
ARG ARENA_CAMERA_ROS_BRANCH=main
ONBUILD RUN echo "Cloning from ${ARENA_CAMERA_ROS_BRANCH} branch ${ARENA_CAMERA_ROS_REPO}"

# This will break the Docker cache when the repo changes at github.com
ONBUILD ADD https://api.github.com/repos/apl-ocean-engineering/arena_camera_ros/git/refs/heads/${ARENA_CAMERA_ROS_BRANCH} /tmp/version.json
ONBUILD RUN git clone --depth 1 -b ${ARENA_CAMERA_ROS_BRANCH} ${ARENA_CAMERA_ROS_REPO}

#====================================================================
# Bring in ROS dependencies but don't buil
#====================================================================
#
FROM clone as prebuild

ARG WS_DIR
WORKDIR ${WS_DIR}/src

RUN vcs import --shallow --skip-existing < arena_camera_ros/arena_camera_ros.repos

WORKDIR ${WS_DIR}

# This is quite expensive to run on every build...
# RUN bash -c "apt-get update \
#               && source /opt/ros/noetic/setup.bash \
#               && rosdep install -y --ignore-src \
#                     --skip-keys=arena_sdk \
#                     --from-paths src/ \
#               && rm -rf /var/lib/apt/lists/*"


#====================================================================
# Final build
#====================================================================
#
FROM prebuild as arena_camera_ros

ARG WS_DIR
LABEL Version=0.1
LABEL Name=arena_camera_ros

RUN bash -c "~/ros_entrypoint.sh \
                catkin build"

# Convert ARG to ENV
ENV ROS_WS ${WS_DIR}
CMD ["roslaunch", "arena_camera", "arena_camera_nodelet.launch"]
