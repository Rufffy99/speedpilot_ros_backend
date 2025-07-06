from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Pfade zu Packages
    pkg_desc = FindPackageShare('speedpilot_description')
    pkg_ros_gz = FindPackageShare('ros_gz_sim')
    pkg_gazebo = FindPackageShare('speedpilot_gazebo')

    # URDF/Xacro-Datei
    urdf_file = PathJoinSubstitution([pkg_desc, 'urdf', 'car.urdf.xacro'])

    # Welt-Datei
    world_file = PathJoinSubstitution([pkg_gazebo, 'worlds', 'turtlebot3_world.world'])

    gz_sim_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg_ros_gz, 'launch', 'gz_sim.launch.py'])
    ),
    launch_arguments={
        'gz_args': [TextSubstitution(text='-r '), world_file]
    }.items()
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': TextSubstitution(text='-g')
        }.items()
    )

    # robot_state_publisher mit URDF starten
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }],
        output='screen'
    )

    # Roboter im Gazebo spawnen
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'speedpilot', '-topic', 'robot_description'],
        output='screen'
    )

    # GZ_SIM_RESOURCE_PATH setzen für Modelle, Texturen etc.
    set_gz_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=PathJoinSubstitution([pkg_desc])
    )

    return LaunchDescription([
        set_gz_resources,
        gz_sim_server,
        gz_sim_gui,
        robot_state_pub,
        spawn_entity
    ])