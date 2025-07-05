from setuptools import find_packages, setup

package_name = 'speedpilot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/speedpilot_description']),
        ('share/speedpilot_description', ['package.xml']),
        ('share/speedpilot_description/launch', ['launch/view_model.launch.py']),
        ('share/speedpilot_description/urdf', ['urdf/car.urdf.xacro']),  # ← wichtig!
    ],
    install_requires=['setuptools', 'ament_index_python', 'xacro'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='max@domitrovic.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
