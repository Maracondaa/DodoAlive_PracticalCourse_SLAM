import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # --- Paths ---
    pkg_dodo_config = get_package_share_directory('dodo_config')
    pkg_rplidar_ros = get_package_share_directory('rplidar_ros')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_path = os.path.join(pkg_dodo_config, 'config', 'nav2_params.yaml')
    cartographer_config_dir = os.path.join(pkg_dodo_config, 'config')
    rviz_config_path = os.path.join(pkg_dodo_config, 'rviz', 'dodo_slam_2d.rviz')

    # --- Robot Description (URDF) ---
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('dodo_description'), 'urdf', 'dodo.urdf.xacro']),
        ]),
        value_type=str
    )

    # --- SENSORS & STATE ---

    # RPLIDAR C1 (publishing /scan)
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rplidar_ros, 'launch', 'rplidar_c1_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_b4ba5730465aee11afa481dc8ffcc75d-if00-port0',
            'frame_id': 'laser',
        }.items()
    )

    # Nicla IMU (publishing /nicla/imu)
    nicla_imu_node = Node(
        package='nicla_imu',
        executable='nicla_imu_node',
        name='nicla_imu',
        parameters=[{'use_sim_time': False}]
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,
            'publish_frequency': 30.0
        }]
    )


    # --- SLAM (Cartographer) ---

    cart_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'rviz' : False,
            'publish_tf' : True,
        }],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'dodo_slam_2d.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/nicla/imu'),
        ]
    )

    occ_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
            'use_map_builder_hashes': True,
        }],
        arguments=['--ros-args', '-p', 'durability:=transient_local'],
        remappings = [
            ('submap_list', '/submap_list'),
            ('map', '/map')
        ]
    )

    # --- NAVIGATION & UI ---

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rplidar_node,
        nicla_imu_node,
        cart_node,
        occ_grid_node,
        rviz_node,
        TimerAction(period=5.0, actions=[nav2_node])
    ])
