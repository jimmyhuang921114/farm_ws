FROM nvidia/cuda:12.6.3-devel-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=developer
ARG USER_UID=1000
ARG USER_GID=1000
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales curl gpg lsb-release ca-certificates software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    curl -s --compressed https://koide3.github.io/ppa/ubuntu2204/KEY.gpg \
      | gpg --dearmor | tee /etc/apt/trusted.gpg.d/koide3_ppa.gpg >/dev/null && \
    echo "deb [signed-by=/etc/apt/trusted.gpg.d/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" \
      > /etc/apt/sources.list.d/koide3_ppa.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-desktop ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rqt ros-humble-rqt-image-view ros-humble-rqt-gui-py \
      ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap \
      ros-humble-velodyne ros-humble-realsense2-camera \
      ros-humble-realsense2-description ros-humble-rviz-imu-plugin \
      ros-humble-image-transport-plugins \
      python3-colcon-common-extensions python3-rosdep python3-vcstool \
      python3-pip python3-pytest python3-yaml python3-pyqt5 python3-pyudev \
      build-essential cmake ninja-build git pkg-config sudo usbutils \
      iproute2 iputils-ping tcpdump v4l-utils \
      libiridescence-dev libboost-all-dev libglfw3-dev libmetis-dev \
      libomp-dev libfmt-dev libspdlog-dev libglm-dev libpng-dev libjpeg-dev \
      libgtsam-points-cuda12.6-dev && \
    rosdep init 2>/dev/null || true && ldconfig && \
    rm -rf /var/lib/apt/lists/*

# livox_ros_driver2 requires the native SDK headers and shared library.
# Build outside the vendored source tree and retain only the installation.
COPY src/Livox-SDK2 /tmp/Livox-SDK2
RUN cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2-build \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/Livox-SDK2-build --parallel 2 && \
    cmake --install /tmp/Livox-SDK2-build && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2 /tmp/Livox-SDK2-build

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENV ROS_DOMAIN_ID=40 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp QT_X11_NO_MITSHM=1 \
    FARM_WS_CONTAINER_NAME=farm_ws_glim_ui_dev
WORKDIR /workspace/farm_ws
USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
