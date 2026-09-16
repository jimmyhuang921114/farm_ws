FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive

ARG USERNAME=work
ARG USER_UID=1000
ARG USER_GID=1000

ENV TZ=Asia/Taipei
ENV ROS_DISTRO=humble

# ============================================================
# Base dependencies
# ============================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    ca-certificates \
    software-properties-common \
    build-essential \
    cmake \
    git \
    vim \
    nano \
    sudo \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv \
    python3-opencv \
    v4l-utils \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8


# ============================================================
# ROS 2 Humble repository
# ============================================================

RUN curl -sSL \
    https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list


# ============================================================
# ROS 2 packages
# ============================================================

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-vision-msgs \
    ros-humble-rqt-image-view \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*


# ============================================================
# Python packages
# ============================================================

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --ignore-installed \
        sympy \
        ultralytics \
        "lap>=0.5.12" \
        numpy \
        scipy


# ============================================================
# Create work user
# ============================================================

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd \
        --uid ${USER_UID} \
        --gid ${USER_GID} \
        --create-home \
        --shell /bin/bash \
        ${USERNAME}

# Passwordless sudo
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" \
    > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}


# ============================================================
# Workspace
# ============================================================

RUN mkdir -p /workspace && \
    chown -R ${USERNAME}:${USERNAME} /workspace


# ============================================================
# ROS environment
# ============================================================

RUN echo "source /opt/ros/humble/setup.bash" \
        >> /home/${USERNAME}/.bashrc && \
    echo 'if [ -f /workspace/install/setup.bash ]; then' \
        >> /home/${USERNAME}/.bashrc && \
    echo '    source /workspace/install/setup.bash' \
        >> /home/${USERNAME}/.bashrc && \
    echo 'fi' \
        >> /home/${USERNAME}/.bashrc

RUN chown ${USERNAME}:${USERNAME} \
    /home/${USERNAME}/.bashrc


# ============================================================
# Default user
# ============================================================

USER ${USERNAME}

WORKDIR /workspace

CMD ["/bin/bash"]