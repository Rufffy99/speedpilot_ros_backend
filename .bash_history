pwd
pwd
apt-get update
apt-get install -y python3-rpi.gpio
ls
cd ros2_ws/
ls
colcon build --symlink-install && source install/setup.bash 
clear
colcon build --symlink-install && source install/setup.bash 
clear
colcon build --symlink-install && source install/setup.bash 
rm -rf build/ install/ log/
colcon build --symlink-install && source install/setup.bash 
clear
rm -rf build/ install/ log/
colcon build --symlink-install && source install/setup.bash 
ls install/custom_msgs/share/custom_msgs/local_setup.bash
clear
ros2 interface list | grep custom_msgs
ros2 interface show custom_msgs/msg/VehicleCommand
clear
cd ~/ros2_ws
rm -rf src/sllidar_ros2/.git
git add src/sllidar_ros2
git commit -m "Integriert sllidar_ros2 ins Hauptrepository"
git add src/sllidar_ros2 src/custom_msgs/CMakeLists.txt src/custom_msgs/package.xml src/ros2_bridge/ros2_bridge/bridge_node.py
echo ".vscode/" >> .gitignore
cd
ls
git switch dev
clear
git -help
git --help
clear
git checkout dev
git pull
clear
git checkout dev
git fetch origin
git rebase origin/main
git rebase --abort
git pull
clear
git fetch origin
git rebase origin/main
git status
git rebase --continue
git push
git pull
git push --force-with-lease
clear
git pull
git fetch
git pull
clear
git checkout dev
git fetch origin
git reset --hard origin/main
Git push --force-with-lease
git push --force-with-lease
clear
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
colcon build --symlink-install --parallel-workers 1
colcon build --symlink-install
clear
colcon build --symlink-install --parallel-workers 1
source install/setup.bash
cd src/
git submodule add https://github.com/dein-username/slam_toolbox.git
git commit -m "Add slam_toolbox as submodule"
git submodule update --init --recursive
cd slam_toolbox/
git checkout -b jazzy-patch
git status
git add src/slam_toolbox_common.cpp
git commit -m "Fix build issue for ROS2 Jazzy compatibility"
cd ../..
git status
git add src/slam_toolbox
git commit -m "Update slam_toolbox submodule to latest patch"
git push
cd src/slam_toolbox
git push origin jazzy-patch
cd ~/ros2_ws/src/slam_toolbox
git remote set-url origin https://github.com/Rufffy99/slam_toolbox.git
git remote -v
git push origin jazzy-patch
git diff > slam_toolbox_patch.diff
cd ~/ros2_ws/
git add src/slam_toolbox
git commit -m "Patched slam_toolbox for ROS 2 Jazzy compatibility (e.g. updated includes and CMake for build)"
git push
cd ~/ros2_ws/src/slam_toolbox
rm -rf .git
cd ~/ros2_ws
git add src/slam_toolbox
git commit -m "Integrated patched slam_toolbox into main repo"
git push
cd ..
ls
tree -L 4
clear
tree -L 3 -P '*.cpp' -P '*.hpp' -P '*.h' -P '*.py' -P 'CMakeLists.txt' -P 'package.xml' -P '*' --prune
tree -L 4 -P '*.cpp' -P '*.hpp' -P '*.h' -P '*.py' -P 'CMakeLists.txt' -P 'package.xml' -P '*' --prune
clear
tree -I 'build|log|install|__pycache__|.*|*.log|*.so|*.pyc|*.o|*.a' -P '*.cpp|*.hpp|*.h|*.py|CMakeLists.txt|package.xml' -L 4 --prune
tree -I 'build|log|install|__pycache__|.*|*.log|*.so|*.pyc|*.o|*.a' -P '*.cpp|*.hpp|*.h|*.py|CMakeLists.txt|package.xml' -L 5 --prune
clear
tree -I 'build|log|install|__pycache__|.*|*.log|*.so|*.pyc|*.o|*.a' -P '*.cpp|*.hpp|*.h|*.py|CMakeLists.txt|package.xml' -L 5 --prune
tree -L 1
git push 
git switch feature/ultrasonic
git fetch
git switch ultrasonic
git switch feature/ultrasonic
ls
clear
cd ros2_ws/
ls
cd src/
ros2 pkg create --build-type ament_python ultrasonic_sensor_node
ros2 pkg create --build-type ament_python ultrasonic_sensor
cd ..
colcon build
source install/setup.bash
ros2 run ultrasonic_sensor ultrasonic_node
git switch dev
pip install protobuf
pwd
