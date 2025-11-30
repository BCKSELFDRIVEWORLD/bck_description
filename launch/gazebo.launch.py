import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('bck_description')
    gazebo_share = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_share, 'urdf', 'guzergah_agv.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Start Gazebo (use the gazebo_ros packaged launch)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Publish robot_description
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawn the robot in Gazebo using the robot_description topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'guzergah_agv'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        spawn_entity,
    ])
