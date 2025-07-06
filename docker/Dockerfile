ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

LABEL maintainer="Max Domitrovic <max@domitrovic.com>"

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    TZ=Etc/UTC \
    DISPLAY=host.docker.internal:0

SHELL ["/bin/bash", "-c"]

# System & dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    sudo \
    curl \
    wget \
    git \
    vim \
    nano \
    bash-completion \
    lsb-release \
    locales \
    gosu \
    gedit \
    htop \
    libserial-dev \
    iputils-ping \
    usbutils \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-argcomplete \
    python3-networkx \
    x11-apps \
    mesa-utils \
    libgl1 \
    libglu1-mesa \
    libx11-dev \
    libxcb-xinerama0 \
    libtbb-dev \
    libceres-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 setup
RUN rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# GUI tools (RViz, RQT, etc.)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-demo-nodes-cpp \
    ros-${ROS_DISTRO}-demo-nodes-py \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt-reconfigure \
    ros-${ROS_DISTRO}-bondcpp || true && \
    rm -rf /var/lib/apt/lists/*

# Create workspace folders
RUN mkdir -p /root/ros2_ws/src /root/shared/ros2

# Copy optional dev config
COPY docker/workspace.sh /root/
COPY docker/entrypoint.sh /root/
COPY docker/bash_aliases.txt /root/.bashrc_aliases

RUN chmod +x /root/*.sh && \
    cat /root/.bashrc_aliases >> /root/.bashrc

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]