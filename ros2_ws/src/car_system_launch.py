#!/usr/bin/env python3
"""Launch both ros_bridge and car_controller nodes."""

from launch import LaunchDescription, LaunchService

from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate a LaunchDescription for starting key ROS2 nodes.

    This function constructs and returns a LaunchDescription instance that encapsulates
    a list of Node definitions. The nodes included are:
        - A bridge node from the 'ros2_bridge' package with the executable 'bridge_node'.
        - A car controller node from the 'car_controller' package with the executable 'controller_node'.
    Both nodes are configured to output their logs to the screen.
    Returns:
        LaunchDescription: An object containing the launch information for the defined nodes.
    """
    return LaunchDescription([
        Node(
            package='ros2_bridge',
            executable='bridge_node',
            name='ros2_bridge',
            output='screen'
        ),
        Node(
            package='car_controller',
            executable='controller_node',
            name='car_controller',
            output='screen'
        )
    ])


def main():
    """
    Execute the main function to initialize and run the ROS 2 launch service.

    This function performs the following steps:
    1. Creates an instance of the LaunchService.
    2. Generates a launch description using the generate_launch_description() function.
    3. Includes the generated launch description in the LaunchService.
    4. Runs the LaunchService and returns its exit code.
    Returns:
        int: The exit code produced by the launch_service.run() method.
    """
    launch_service = LaunchService()
    launch_description = generate_launch_description()
    launch_service.include_launch_description(launch_description)
    return launch_service.run()


if __name__ == '__main__':
    import sys
    sys.exit(main())
