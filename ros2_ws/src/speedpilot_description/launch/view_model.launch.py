"""
Launch file to display the car model in RViz.

This launch file:
1. Processes the XACRO file to generate the robot description.
2. Launches joint_state_publisher_gui to simulate joint states.
3. Launches robot_state_publisher to publish TF frames.
4. Starts RViz2 for visualizing the robot model.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Locate xacro file
    pkg_share = get_package_share_directory('speedpilot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')

    # Process xacro file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        # Joint state publisher with GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])