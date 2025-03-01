import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_bot_description = get_package_share_directory('ddr_bot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_world = get_package_share_directory('nav_world')
    pkg_share = get_package_share_directory('locomotion_control')
    xacro_file = os.path.join(pkg_bot_description, 'urdf', 'ddr_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    robot_localization_file_path = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'ddr_nav_config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': True}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    # Spawn Urdf
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ddr_bot',
            '-topic', 'robot_description',
            '-z', '0.20',
        ],
        output='screen'
    )

    # Start Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Start robot localization using an Extended Kalman filter
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}])

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument('world',
            default_value=[os.path.join(pkg_sim_world, 'worlds', 'cafe.world'), ''],
            description='SDF world file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo,
        urdf_spawn_node,
        robot_localization_node,
        rviz_node
    ])