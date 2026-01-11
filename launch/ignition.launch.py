import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import Command

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project_description = get_package_share_directory('bck_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the URDF file from "description" package
    xacro_file = os.path.join(pkg_project_description, 'urdf', 'bck_agv.xacro')
    robot_description = Command(['xacro ', xacro_file])

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_description, 'rviz', 'robot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Ignition -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # TF (Ignition -> ROS2)
            '/model/bck_agv/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # TF Static (Ignition -> ROS2)
            '/model/bck_agv/pose_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            # CMD Vel (ROS2 -> Ignition)
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Odometry (Ignition -> ROS2)
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # Lidar Scan (Ignition -> ROS2)
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # Joint States (Ignition -> ROS2)
            '/model/bck_agv/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
        remappings=[
            ('/model/bck_agv/joint_state', '/joint_states'),
            ('/model/bck_agv/pose', '/tf'),
            ('/model/bck_agv/pose_static', '/tf_static'),
        ]
    )

    # Spawn the robot using ros_gz_sim create
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'bck_agv',
                   '-allow_renaming', 'true'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        gz_sim,
        robot_state_publisher,
        rviz,
        bridge,
        spawn
    ])

