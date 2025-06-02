from setuptools import find_packages, setup # noqa

package_name = 'ultrasonic_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='max@domitrovic.com',
    description='ROS 2 node for publishing distance measurements from an ultrasonic sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_node = ultrasonic_sensor.ultrasonic_node:main',
        ],
    },
)
