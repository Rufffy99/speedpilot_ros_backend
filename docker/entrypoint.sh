#!/bin/bash

set -e  # exit on error

ROS_DISTRO="jazzy"
ROS_WS="/root/ros2_ws"
SHARED_ROS2="/root/shared/ros2"
ROS_DOMAIN_ID_FILE="$SHARED_ROS2/ros_domain_id.txt"

# X11 GUI support
export DISPLAY=${DISPLAY:-host.docker.internal:0}
export XDG_RUNTIME_DIR=/tmp/runtime-$USER
mkdir -p $XDG_RUNTIME_DIR
chmod 700 $XDG_RUNTIME_DIR

# Ensure ROS_DOMAIN_ID file exists
if [ ! -f "$ROS_DOMAIN_ID_FILE" ]; then
  echo "0" > "$ROS_DOMAIN_ID_FILE"
  echo "Created $ROS_DOMAIN_ID_FILE with default value 0"
fi
export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")

# Source ROS
source /opt/ros/$ROS_DISTRO/setup.bash

# If built already, source workspace
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
fi

# Source bashrc
if [ -f /root/.bashrc ]; then
  source /root/.bashrc
fi

# Optional: build if not built yet
if [ ! -d "$ROS_WS/build" ]; then
  echo "First-time colcon build..."
  cd $ROS_WS
  colcon build || echo "Build failed, check logs"
fi

cd $ROS_WS

exec "$@"