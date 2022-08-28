import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_bot_description = get_package_share_directory('duke_bot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_sim_world = get_package_share_directory('sim_world')

    xacro_file = os.path.join(pkg_bot_description, 'urdf', 'duke_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    # Spawn Urdf
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'duke_bot',
            '-topic', 'robot_description',
            '-x', '-1.5',
            '-y', '-0.6',
            '-z', '0.147',
            '-Y', '-2.35'
        ],
        output='screen'
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller_1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel1_drive_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel2_drive_controller'],
        output='screen'
    )

    load_joint_trajectory_controller_3 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel3_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world',
          default_value=[os.path.join(pkg_sim_world, 'worlds', 'arena.world'), ''],
          description='SDF world file'),
        robot_state_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=urdf_spawn_node,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller_1],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller_1,
                on_exit=[load_joint_trajectory_controller_2],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller_2,
                on_exit=[load_joint_trajectory_controller_3],
            )
        ),
        gazebo,
        urdf_spawn_node,
    ])