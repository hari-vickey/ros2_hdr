import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
    # HDR Navigation Map and Configuration
    pkg_hdr_navigation = FindPackageShare(package='hdr_navigation').find('hdr_navigation')
    static_map_file = os.path.join(pkg_hdr_navigation, 'maps', 'smalltown.yaml')

    return LaunchDescription([
        Node(
            parameters=[{'yaml_filename': static_map_file}, {'use_sim_time': True}],
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'),
        Node(
            parameters=[{'use_sim_time': True}, {'autostart': True},
                        {'node_names': ['map_server']}],
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen')
    ])