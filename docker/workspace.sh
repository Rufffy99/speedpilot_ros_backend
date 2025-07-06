#!/bin/bash
set -e

ROS_DISTRO="jazzy"
ROS_WS="/root/ros2_ws"

# Source ROS
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev

# Setup workspace
mkdir -p $ROS_WS/src
cd $ROS_WS/src

# Optionally clone required packages
if [ ! -d "moveit_task_constructor" ]; then
  git clone https://github.com/ros-planning/moveit_task_constructor.git
  cd moveit_task_constructor
  git checkout jazzy || git checkout main || git checkout rolling
  cd ..
fi

cd $ROS_WS

# ROS 2 dependencies
rosdep update || echo "Warning: rosdep update failed"
rosdep install --from-path src --rosdistro $ROS_DISTRO -y \
    --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts" \
    || echo "Some dependencies failed"

# Optional code fixes (example for known issues)
# Replace with your own project-specific patches as needed
PATCH_FILE="$ROS_WS/src/moveit_task_constructor/core/src/storage.cpp"
if grep -q "getParent()" "$PATCH_FILE"; then
  echo "Patching storage.cpp..."
  sed -i '/getParent()/,+3c\    this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);' "$PATCH_FILE"
fi

# Build
colcon build --packages-skip mycobot_mtc_pick_place_demo || true
colcon build --packages-select mycobot_mtc_pick_place_demo --cmake-args -Wno-dev || true
colcon build || true

source install/setup.bash

echo "✅ Workspace ready"