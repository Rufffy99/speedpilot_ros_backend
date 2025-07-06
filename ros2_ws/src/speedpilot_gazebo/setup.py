from setuptools import setup

package_name = 'speedpilot_gazebo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/turtlebot3_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max Domitrovic',
    maintainer_email='max@example.com',
    description='Gazebo launch and setup for Speedpilot simulation',
    license='Apache License 2.0',
)