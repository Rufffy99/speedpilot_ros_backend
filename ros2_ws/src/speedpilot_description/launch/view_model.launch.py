"""
Launch file to display the car model in RViz.

This launch description performs the following steps:
1. Starts the 'robot_state_publisher' node:
    - Reads a URDF/Xacro file and sets its content as the 'robot_description' parameter.
    - Ensure to update the file path 'PATH/TO/car.urdf.xacro' with the correct location of your robot's URDF/Xacro file.
2. Starts the 'rviz2' node for visualizing the robot state.

This setup integrates the robot state publishing with RViz visualization,
facilitating the debugging and development of the robot model.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    """
    Generate and return a ROS 2 LaunchDescription that configures the robot simulation.

    This launch description includes a node for publishing the robot state using a URDF/XACRO file
    and a node for starting the RViz2 visualization tool.

    Returns:
        LaunchDescription: The launch description containing nodes for robot state publishing and visualization.
    """
    pkg_share = get_package_share_directory('speedpilot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'car.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config.toxml()}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
