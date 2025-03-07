#!/bin/bash
set -e  # Exit on error, but handle specific failures manually

# Set ROS 2 distribution as a variable
ROS_DISTRO="jazzy"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies for MongoDB and PCL
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev

# Install MongoDB (following official MongoDB installation for Ubuntu)
# curl -fsSL https://www.mongodb.org/static/pgp/server-8.0.asc | \
#    gpg -o /usr/share/keyrings/mongodb-server-8.0.gpg \
#    --dearmor
# echo "deb [ arch=amd64,arm64 signed-by=/usr/share/keyrings/mongodb-server-8.0.gpg ] https://repo.mongodb.org/apt/ubuntu noble/mongodb-org/8.0 multiverse" | \
#    tee /etc/apt/sources.list.d/mongodb-org-8.0.list
# apt-get update && apt-get install -y mongodb-org

# Start and enable MongoDB service
# Note: systemctl might not work in all Docker environments, so we'll add error handling
# systemctl start mongod || echo "Warning: Could not start MongoDB service. This is expected in some Docker environments."
# systemctl enable mongod || echo "Warning: Could not enable MongoDB service. This is expected in some Docker environments."

# Navigate to the workspace
cd /root/ros2_ws/src

# Install MoveIt Task Constructor if not already present
if [ ! -d "moveit_task_constructor" ]; then
    echo "Cloning MoveIt Task Constructor..."
    git clone https://github.com/moveit/moveit_task_constructor.git
    cd moveit_task_constructor
    
    # Ensure we are on a compatible branch
    git fetch origin
    git checkout jazzy || git checkout iron || git checkout rolling
    
    # Reset to a known good commit (if needed)
    git reset --hard bad8e13254010a7e72681adc247c34393dd24df2
    
    cd ..
fi

# Navigate back to the workspace root
cd /root/ros2_ws

# Install ROS 2 dependencies for all packages
echo "Installing ROS 2 dependencies..."
rosdep update || echo "Warning: Failed to update rosdep"

# Install dependencies, skipping problematic ones
rosdep install --from-path src --rosdistro $ROS_DISTRO -y \
    --skip-keys "actionlib catkin message_generation rviz rosparam_shortcuts" \
    || echo "Warning: Some dependencies could not be resolved"

# Apply known fixes after dependencies are installed

# Fix storage.cpp
echo "Fixing storage.cpp..."
cd /root/ros2_ws/src/moveit_task_constructor
if [ -f core/src/storage.cpp ]; then
    # Create backup
    cp core/src/storage.cpp core/src/storage.cpp.backup

    # Replace the problematic lines using sed
    sed -i '/if (this->end()->scene()->getParent() == this->start()->scene())/,+3c\    this->end()->scene()->getPlanningSceneDiffMsg(t.scene_diff);' core/src/storage.cpp || echo "Warning: Could not modify storage.cpp"
fi

# Fix cartesian_path.cpp
echo "Fixing cartesian_path.cpp..."
if [ -f core/src/solvers/cartesian_path.cpp ]; then
    # Create backup
    cp core/src/solvers/cartesian_path.cpp core/src/solvers/cartesian_path.cpp.backup

    # Make the replacement
    sed -i 's/moveit::core::JumpThreshold(props.get<double>("jump_threshold")), is_valid,/moveit::core::JumpThreshold::relative(props.get<double>("jump_threshold")), is_valid,/' core/src/solvers/cartesian_path.cpp || echo "Warning: Could not modify cartesian_path.cpp"
fi

cd /root/ros2_ws

# Fix PCL warning - this needs to come after rosdep install
echo "Fixing PCL warnings..."
find /usr/include/pcl* -path "*/sample_consensus/impl/sac_model_plane.hpp" -exec sed -i 's/^\(\s*\)PCL_ERROR ("\[pcl::SampleConsensusModelPlane::isSampleGood\] Sample points too similar or collinear!\\n");/\1\/\/ PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Sample points too similar or collinear!\\n");/' {} \;

# Build the packages
echo "Building packages..."
# First build without the problematic package
colcon build --packages-skip mycobot_mtc_pick_place_demo || echo "Warning: Initial colcon build failed, check logs"
source install/setup.bash

# Then build the problematic package with warning suppression
colcon build --packages-select mycobot_mtc_pick_place_demo --cmake-args -Wno-dev || echo "Warning: Selective colcon build failed, check logs"
source install/setup.bash

# Final build of everything
colcon build || echo "Warning: Final colcon build failed, check logs"
source install/setup.bash

echo "Workspace setup completed!"