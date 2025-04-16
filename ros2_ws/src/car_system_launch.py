#!/usr/bin/env python3
"""Launch both ros_bridge and car_controller nodes."""

from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description(): # noqa
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


if __name__ == '__main__':
    generate_launch_description()
