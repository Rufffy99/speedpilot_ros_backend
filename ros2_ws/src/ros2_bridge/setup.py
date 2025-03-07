from setuptools import setup # noqa
"""
Setup script for the ros2_bridge package.

This script uses setuptools to package the ros2_bridge module, which provides
a WebSocket bridge for a Flutter app to communicate with ROS2.

Attributes:
    package_name (str): The name of the package.
"""

package_name = 'ros2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['websocket-server'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max@domitrovic.com',
    description='ROS2 WebSocket Bridge für Flutter-App',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = ros2_bridge.bridge_node:main',
        ],
    },
)
