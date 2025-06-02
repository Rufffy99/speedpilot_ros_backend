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
