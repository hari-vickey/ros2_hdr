# Import Dependencies
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

    # ROS Gazebo Package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # Robot Description
    pkg_bot_description = FindPackageShare(package='ddr_bot_description').find('ddr_bot_description')
    xacro_file = os.path.join(pkg_bot_description, 'urdf', 'ddr_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Simulation World
    pkg_sim_world = FindPackageShare(package='nav_world').find('nav_world')
    world_file_name = 'smalltown.world'
    world_path = os.path.join(pkg_sim_world, 'worlds', world_file_name)

    # HDR Navigation Map and Configuration
    pkg_hdr_navigation = FindPackageShare(package='hdr_navigation').find('hdr_navigation')
    robot_localization_file_path = os.path.join(pkg_hdr_navigation, 'config' , 'ekf.yaml')
    rviz_config_file = os.path.join(pkg_hdr_navigation, 'config', 'ddr_nav_config.rviz')
    static_map_file = os.path.join(pkg_hdr_navigation, 'maps', 'smalltown.yaml')
    params_file = os.path.join(pkg_hdr_navigation, 'config', 'ddr_nav_params.yaml')

    # ROS2 Navigation2 Setup
    pkg_navigation2_bt = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_file = os.path.join(pkg_navigation2_bt, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml')
    pkg_navigation2 = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(pkg_navigation2, 'launch')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Launch Arguments
    use_map = LaunchConfiguration('use_map')
    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    use_world = LaunchConfiguration('use_world')
    use_params = LaunchConfiguration('use_params')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    use_nav_stack = LaunchConfiguration('use_nav_stack')

    # List of Arguments and their default Values with description
    declare_use_map_cmd = DeclareLaunchArgument(
      name='use_map', default_value=static_map_file,
      description='Launch a Map')
    declare_use_slam_cmd = DeclareLaunchArgument(
      name='use_slam', default_value='False',
      description='Whether to run SLAM')

    declare_use_rviz_cmd = DeclareLaunchArgument(
      name='use_rviz', default_value='True',
      description='Whether to start RVIZ')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
      name='use_sim_time', default_value='True',
      description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator', default_value='True',
        description='Whether to start the simulator')

    declare_use_world_cmd = DeclareLaunchArgument(
        name='use_world', default_value=world_path,
        description='World model file to load')

    declare_use_params_cmd = DeclareLaunchArgument(
        name='use_params', default_value=params_file,
        description='Parameters file to Load for Navigation')

    declare_use_nav_cmd = DeclareLaunchArgument(
        name='use_nav_stack', default_value='True',
        description='Whether to Launch ROS2 Navigation Stack')

    # Start Gazebo
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': use_world, 'pause': 'false'}.items())

    # Start Robot State Publisher Node
    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 'use_sim_time': use_sim_time}
        ],
        remappings=remappings)

    # Start Joint State Publisher Node
    start_joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])

    # Start Urdf Spawner
    start_urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ddr_bot',
            '-topic', 'robot_description',
            '-z', '0.20',
        ],
        output='screen')

    # Start Robot Localization Node
    start_robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, {'use_sim_time': LaunchConfiguration('use_sim_time')}])

    # Start Rviz
    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    # Start ROS 2 Navigation Stack
    start_navigation_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        condition=IfCondition(use_nav_stack),
        launch_arguments = {'slam': 'True',
                            'map': use_map,
                            'use_sim_time': use_sim_time,
                            'params_file': use_params,
                            'slam_params_file': use_params}.items())

    # Start ROS 2 Slam Node
    # This Node publishes map -> odom transformation
    # Set Parameters appropriately
    start_nav2_slam_node = Node(
        condition=IfCondition(use_slam),
        parameters=[use_params, {'use_sim_time': use_sim_time}],
        package = 'slam_toolbox',
        executable = 'async_slam_toolbox_node',
        name = 'slam_toolbox',
        output = 'screen')

    # Start ROS 2 Map Server
    start_map_server_node = Node(
        condition=IfCondition(use_nav_stack),
        parameters=[{'yaml_filename': use_map}, {'use_sim_time': use_sim_time}],
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen')

    # Start Lifecycle manager for navigation nodes
    start_nav2_lifecycle_manager = Node(
        condition=IfCondition(use_nav_stack),
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True},
                    {'node_names': ['map_server']}],
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen')

    # Create Launch Description
    ld = LaunchDescription()

    # Launch Options
    ld.add_action(declare_use_nav_cmd)
    ld.add_action(declare_use_map_cmd)
    ld.add_action(declare_use_slam_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_world_cmd)
    ld.add_action(declare_use_params_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)

    # Launch nodes
    ld.add_action(start_gazebo)
    ld.add_action(start_rviz_node)
    ld.add_action(start_robot_state_publisher_node)
    ld.add_action(start_joint_state_publisher_node)
    ld.add_action(start_urdf_spawn_node)
    ld.add_action(start_robot_localization_node)
    ld.add_action(start_nav2_slam_node)
    ld.add_action(start_map_server_node)
    ld.add_action(start_nav2_lifecycle_manager)
    ld.add_action(start_navigation_nodes)

    return ld