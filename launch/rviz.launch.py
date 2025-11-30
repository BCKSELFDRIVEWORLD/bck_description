import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('bck_description')
    # Use the no-gazebo xacro (no gazebo plugins) for RViz visualization
    xacro_file = os.path.join(pkg_share, 'urdf', 'guzergah_agv_no_gazebo.xacro')

    # Note: include a trailing space after 'xacro' so Command joins correctly
    robot_description_content = Command(['xacro ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': False}],
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'robot.rviz')
    rviz_args = ['-d', rviz_config] if os.path.exists(rviz_config) else []

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args,
    )

    return LaunchDescription([
        rsp_node,
        rviz_node,
    ])
