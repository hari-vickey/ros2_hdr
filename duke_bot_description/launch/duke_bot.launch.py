
import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        ),
        launch_arguments={
            'pause': 'false'
        }.items()
    )

    duke_bot_description_path = os.path.join(
        get_package_share_directory('duke_bot_description'))

    xacro_file = os.path.join(duke_bot_description_path,
                              'urdf',
                              'duke_bot.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments= [
            '-entity', 'duke_bot',
            '-topic', 'robot_description',
            '-z', '0.027',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
    ])
