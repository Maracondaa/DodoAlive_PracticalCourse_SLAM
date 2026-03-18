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
    pkg_realsense_ros = get_package_share_directory('realsense2_camera')

    ekf_config_path = os.path.join(pkg_dodo_config, 'config', 'dodo_ekf.yaml')

    # Robot Description (URDF)
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

    # RealSense Camera (publishing /camera/realsense/...)
    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_realsense_ros, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': 'camera',
            'camera_name': 'realsense',
            'pointcloud.enable': 'false',
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'enable_depth': 'true',
            'enable_color': 'true',
            'align_depth.enable': 'true',

            #IMU disabled for now

            #'unite_imu_method': '0',
            # 'hold_back_imu_for_frames': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',

            'global_time_enabled': 'true',
            'base_time_error': 'true',
            'enable_sync': 'true',
            'host_performance_mode': 'false',

            'tf_publish_rate': '100.0',
            'initial_reset': 'true',
        }.items()
    )

    rgbd_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.01,
            'sync_queue_size': 20,
        }],
        remappings=[
            ('rgb/image', '/camera/realsense/color/image_raw'),
            ('depth/image', '/camera/realsense/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/realsense/color/camera_info'),
            ('rgbd_image', '/rgbd_image')
        ]
    )

    # Visual Odometry from RealSense camera (publishing /odometry/visual)
    rgbd_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'approx_sync': True,
            'subscribe_rgbd': True,
            'wait_for_transform': 0.5,
            'tf_delay': 0.05,

            'Reg/Force3DoF': 'False',

            'Vis/MinInliers': '25',
            'Vis/InlierDistance': '0.05',
            'Vis/FeatureType': '6',
            'Vis/EstimationType': '1',
            'Vis/MaxFeatures': '600',
            'Vis/MinDepth': '0.2',

            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'True',
            'Odom/GuessSmoothingDelay' : '2',
            'Odom/ResetCountdown' : '1',
            'Odom/VisKeyFrameThr' : '150',
            'Odom/KeyFrameThr' : '0.3',
            'Odom/FilteringStrategy': '1',
        }],
        remappings=[
            ('rgbd_image', '/rgbd_image'),
            ('odom', '/odometry/visual'),
        ]
    )

    # Extended Kalman Filter (publishing /odometry/ekf)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            ('odometry/filtered', '/odometry/ekf')
        ]
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
            'publish_frequency': 100.0
        }]
    )


    # --- SLAM (RTABMAP) ---

    rtabmap_slam_node = Node(
    package='rtabmap_slam',
    executable='rtabmap',
    output='screen',
    arguments=['--delete_db_on_start'],
    parameters=[{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',

        'subscribe_rgbd': True,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan': True,

        'approx_sync': True,
        'sync_queue_size': 20,
        'odom_sensor_sync': False,

        'publish_tf': True,
        'tf_delay': 0.05,
        'wait_for_transform': 0.5,

        'map_filter_radius': 0.0,
        'map_cleanup': True,
        'map_always_update': False,
        'map_empty_ray_tracing': True,
        'latch': True,

        'RGBD/NeighborLinkRefining': 'True',
        'RGBD/OptimizeFromGraphEnd': 'True',
        'RGBD/ProximityBySpace': 'True',
        'RGBD/ProximityMaxGraphDepth' : '50',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.03',
        'RGBD/LinearUpdate': '0.03',


        'Grid/FromDepth': 'True',
        'Grid/FromScan': 'False',
        'Grid/Sensor': '0',
        'Grid/3D': 'False',
        'Grid/RangeMax': '4.0',
        'Grid/RangeMin': '0.2',
        'Grid/DepthDecimation': '2',

        'Reg/Force3DoF': 'False',
        'Reg/Strategy': '2',

        'Optimizer/Strategy': '2',


        'Mem/DepthAsMask': 'False',

    }],
    remappings=[
        ('rgbd_image', '/rgbd_image'),
        ('scan', '/scan'),
        ('odom', '/odometry/ekf')
        ]
    )


    # --- NAVIGATION & UI ---

    rviz_node = Node(
    package='rtabmap_viz',
    executable='rtabmap_viz',
    output='screen',
    parameters=[{
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',

        'subscribe_rgbd': True,
        'subscribe_rgb': False,
        'subscribe_depth': False,
        'subscribe_scan': True,

        'approx_sync': True,
        'sync_queue_size': 20,
        'odom_sensor_sync': False,

        'publish_tf': False,
        'wait_for_transform': 0.5,

        'map_always_update': False,
        'latch': True,
    }],
    remappings=[
        ('rgbd_image', '/rgbd_image'),
        ('scan', '/scan'),
        ('odom', '/odometry/ekf')
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        realsense_camera_node,
        rplidar_node,
        nicla_imu_node,
        TimerAction(
            period=3.0,
            actions=[rgbd_sync_node]
        ),
        TimerAction(
            period=4.0,
            actions=[rgbd_odom_node]
        ),
        TimerAction(
            period=5.0,
            actions=[ekf_node]
        ),
        TimerAction(
            period=6.0,
            actions=[rtabmap_slam_node, rviz_node]
        ),
    ])

