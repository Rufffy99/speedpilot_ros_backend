#!/bin/bash
set -e

# === Container-Spezifischer Build ===
if [ "$INSIDE_DEVCONTAINER" = "true" ]; then
    echo "🔧 Inside devcontainer: running colcon build..."
    cd /root/ros2_ws
    colcon build \
        --base-paths . \
        --build-base build \
        --install-base install 
    exit 0
fi

# === Hostseitiger Docker-Image-Build ===
echo "🛠️ Running on host: building image + setting up folders"

SCRIPT_PATH=$(dirname "$(realpath "$0")")
PARENT_PATH=$(dirname "$SCRIPT_PATH")
SHARED_ROS2="$HOME/automaticaddison/shared/ros2"
IMAGE_NAME="manipulation:latest"

mkdir -p "$SHARED_ROS2"
sudo docker build --no-cache -f "$SCRIPT_PATH/Dockerfile" -t "$IMAGE_NAME" "$PARENT_PATH"