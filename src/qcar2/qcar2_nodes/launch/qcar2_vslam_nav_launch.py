# =============================================================================
# qcar2_vslam_and_nav_bringup_launch.py
#
# Completely rewritten from qcar2_slam_and_nav_bringup_launch.py
# Reason: The original file had a deep dependency chain (bringup -> cartographer -> qcar2_launch)
#         It included Cartographer components unnecessarily and had an incorrect launch order.
#
# Preserved from original file:
#   - All original argument names (namespace, use_sim_time, params_file, etc.)
#   - Original config file path (qcar2_slam_and_nav.yaml)
#   - Original Nav2 bringup group pattern (component_container_isolated + navigation_launch.py)
#
# Changes:
#   - Replaced Cartographer + Isaac ROS with RTAB-Map (RGB-D SLAM)
#     Reason: ros-humble-isaac-ros-nitros cannot be installed via apt on Jetson host OS.
#     RTAB-Map is readily available: sudo apt install ros-humble-rtabmap-ros
#   - RTAB-Map handles both visual odometry (odom -> base_link) + 2D map (/map) in one node.
#   - All nodes are consolidated into a single file. No nested IncludeLaunchDescription.
#   - Added device_type argument (physical / virtual).
#   - Added localization_mode argument (New mapping / Use existing map).
#   - Corrected launch sequence (declare args -> hardware -> TF -> SLAM -> Nav2).
#
# Topic names (Verified from source code):
#   rgbd.cpp          -> /camera/color_image (BGR8,   1280x720)
#                     -> /camera/depth_image (MONO16, 1280x720)
#   qcar2_hardware    -> /qcar2_imu, /qcar2_joint, /qcar2_battery
#                     <- /qcar2_motor_speed_cmd, /qcar2_led_cmd
#   lidar.cpp         -> /scan  (frame: base_scan)
#
# RTAB-Map node pair:
#   rgbd_odometry  : Visual odometry from RGB-D + IMU
#                    input:  /camera/color_image, /camera/depth_image, /qcar2_imu
#                    output: TF odom -> base_link
#   rtabmap        : Loop closure + occupancy grid + map correction
#                    input:  /camera/color_image, /camera/depth_image,
#                            /scan, /qcar2_imu, /rtabmap/odom
#                    output: /map (OccupancyGrid), TF map -> odom
#
# TF tree:
#   map <- [rtabmap] <- odom <- [rgbd_odometry] <- base_link
#                                                  ├── base_scan  (lidar, x=+0.1m yaw=180)
#                                                  └── imu
#   map_rotated <- [static] <- map
#
# Prerequisites:
#   sudo apt install ros-humble-rtabmap-ros ros-humble-nav2-bringup
#
# Usage:
#   Physical - Mapping (default):
#     ros2 launch qcar2_nodes qcar2_vslam_and_nav_bringup_launch.py
#
#   Physical - Localization (Use existing map):
#     ros2 launch qcar2_nodes qcar2_vslam_and_nav_bringup_launch.py \
#          localization_mode:=true
#
#   Virtual / QLABS:
#     ros2 launch qcar2_nodes qcar2_vslam_and_nav_bringup_launch.py \
#          device_type:=virtual use_sim_time:=true
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # Package directories
    bringup_dir     = get_package_share_directory('qcar2_nodes')
    nav2_dir        = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================

    # Original arguments (interface-compatible with qcar2_slam_and_nav_bringup_launch.py)
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'qcar2_slam_and_nav.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    # New arguments
    declare_device_type = DeclareLaunchArgument(
        'device_type',
        default_value='physical',
        description='"physical" = Real QCar2 | "virtual" = Gazebo / QLABS')

    declare_localization_mode = DeclareLaunchArgument(
        'localization_mode',
        default_value='false',
        description='true = Localize on existing map, do not create new map')

    declare_rtabmap_db = DeclareLaunchArgument(
        'rtabmap_db',
        default_value=os.path.join(os.path.expanduser('~'), 'rtabmap.db'),
        description='Path to RTAB-Map database (.db) storing map + keyframes')

    declare_rtabmap_args = DeclareLaunchArgument(
        'rtabmap_args',
        default_value='',
        description='Extra arguments passed to the rtabmap node')

    # LaunchConfiguration references
    namespace         = LaunchConfiguration('namespace')
    use_namespace      = LaunchConfiguration('use_namespace')
    use_sim_time       = LaunchConfiguration('use_sim_time')
    params_file        = LaunchConfiguration('params_file')
    autostart          = LaunchConfiguration('autostart')
    use_composition    = LaunchConfiguration('use_composition')
    use_respawn        = LaunchConfiguration('use_respawn')
    log_level          = LaunchConfiguration('log_level')
    device_type        = LaunchConfiguration('device_type')
    localization_mode  = LaunchConfiguration('localization_mode')
    rtabmap_db         = LaunchConfiguration('rtabmap_db')
    rtabmap_args       = LaunchConfiguration('rtabmap_args')

    # Helper conditions
    is_virtual    = PythonExpression(["'", device_type, "' == 'virtual'"])
    is_mapping    = UnlessCondition(localization_mode)
    is_localizing = IfCondition(localization_mode)

    # Nav2 params rewrite (Same as original)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    # =========================================================================
    # LAYER 1 - QCar2 HARDWARE SENSOR NODES
    # device_type automatically switches URIs in each node
    # =========================================================================
    lidar_node = Node(
        package='qcar2_nodes',
        executable='lidar',
        name='Lidar',
        parameters=[{'device_type': device_type}],
        output='screen')

    rgbd_node = Node(
        package='qcar2_nodes',
        executable='rgbd',
        name='RealsenseCamera',
        parameters=[{'device_type': device_type}],
        output='screen')

    qcar2_hardware_node = Node(
        package='qcar2_nodes',
        executable='qcar2_hardware',
        name='qcar2_hardware',
        parameters=[{'device_type': device_type}],
        output='screen')

    # =========================================================================
    # LAYER 2 - TF BROADCASTERS
    #
    # Physical (fixed_lidar_frame):
    #   base_link -> base_scan  x=+0.1m, yaw=-180 (LiDAR facing backwards)
    #   base_link -> imu        origin, identity
    #
    # Virtual (fixed_lidar_frame_virtual):
    #   base_link -> base_scan  x=+0.1m, yaw=0
    #
    # Static: map_rotated -> map (Used for path visualization in nav_to_pose)
    # =========================================================================
    fixed_lidar_frame_node = Node(
        condition=UnlessCondition(is_virtual),
        package='qcar2_nodes',
        executable='fixed_lidar_frame',
        name='fixed_lidar_frame',
        output='screen')

    fixed_lidar_frame_virtual_node = Node(
        condition=IfCondition(is_virtual),
        package='qcar2_nodes',
        executable='fixed_lidar_frame_virtual',
        name='fixed_lidar_frame_virtual',
        output='screen')

    map_rotation_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_rotation_tf',
        arguments=['0', '0', '0', '3.14159', '0', '0', 'map_rotated', 'map'],
        output='screen')

    # =========================================================================
    # LAYER 3 - RTAB-Map VISUAL ODOMETRY (rgbd_odometry)
    #
    # Function: Computes visual odometry from RGB-D + IMU
    #           publish TF: odom -> base_link
    #
    # Input topics (Actual QCar2 names from rgbd.cpp):
    #   /camera/color_image  -> remapped: rgb/image
    #   /camera/depth_image  -> remapped: depth/image
    #   /qcar2_imu           -> remapped: imu (optional, improves accuracy)
    #
    # Note: RTAB-Map does not strictly require camera_info for remapping if using approx_sync.
    #       However, for accurate rectification, consider adding camera_info_manager.
    # =========================================================================
    rtabmap_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time':         use_sim_time,
            # TF frames
            'frame_id':             'base_link',
            'odom_frame_id':        'odom',
            'publish_tf':           True,
            # Sync
            'approx_sync':          True, # D435 RGB + Depth timestamps do not perfectly sync
            'approx_sync_max_interval': 0.05,
            # Algorithm
            'Odom/Strategy':        '0', # F2M (Frame to Map) - Accurate indoors
            'Odom/ResetCountdown':  '1', # Reset if tracking is lost
            'Vis/FeatureType':      '6', # GFTT/BRIEF - Fast on Orin AGX
            'Vis/MaxFeatures':      '600',
            'Vis/MinInliers':       '15',
            # Depth range for D435 (reliable: 0.1-5m)
            'Reg/Force3DoF':        'true', # QCar2 moves on flat ground, lock pitch/roll
        }],
        remappings=[
            # QCar2 topic names (from rgbd.cpp) -> RTAB-Map expected names
            ('rgb/image',       '/camera/color_image'),
            ('depth/image',     '/camera/depth_image'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('imu',             '/qcar2_imu'),
            ('odom',            '/rtabmap/odom'),
        ])

    # =========================================================================
    # LAYER 4a - RTAB-Map SLAM (Mapping mode: localization_mode:=false)
    #
    # Function: Loop closure + create/update occupancy grid map
    #           publish TF: map -> odom (pose graph correction)
    #
    # Input:
    #   /camera/color_image   RGB  (from rgbd.cpp)
    #   /camera/depth_image   Depth (from rgbd.cpp)
    #   /scan                 LiDAR (from lidar.cpp, frame: base_scan)
    #   /qcar2_imu            IMU (from qcar2_hardware.cpp)
    #   /rtabmap/odom         Odometry (from rgbd_odometry above)
    #
    # Output:
    #   /map                  nav_msgs/OccupancyGrid (for Nav2)
    #   TF: map -> odom
    #
    # arguments='--delete_db_on_start' deletes the old map every time it runs.
    # Remove this flag if you want to resume a previous mapping session.
    # =========================================================================
    rtabmap_slam_node = Node(
        condition=is_mapping,
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':   use_sim_time,
            # TF frames
            'frame_id':       'base_link',
            'odom_frame_id':  'odom',
            'map_frame_id':   'map',
            # Sensor subscriptions
            'subscribe_depth':      True,
            'subscribe_rgb':        True,
            'subscribe_scan':       True, # Fuse RPLidar A2M12
            'subscribe_odom_info':  False,
            'approx_sync':          True,
            # Database
            'database_path':        rtabmap_db,
            # Map update rate
            'Rtabmap/DetectionRate': '1', # Hz - Save CPU
            # 2D occupancy grid for Nav2
            'Grid/Sensor':          '2', # 0=LiDAR, 1=depth, 2=both
            'Grid/3D':              'false',
            'Grid/RangeMin':        '0.1',
            'Grid/RangeMax':        '5.0',
            'Grid/FootprintRadius': '0.19', # QCar2 ~19cm half-width
            # Loop closure
            'Mem/IncrementalMemory':    'true',
            'Mem/InitWMWithAllNodes':   'false',
            'Reg/Force3DoF':            'true',
        }],
        remappings=[
            ('rgb/image',       '/camera/color_image'),
            ('depth/image',     '/camera/depth_image'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan',            '/scan'),
            ('imu',             '/qcar2_imu'),
            ('odom',            '/rtabmap/odom'),
        ],
        arguments=['--delete_db_on_start', rtabmap_args])

    # =========================================================================
    # LAYER 4b - RTAB-Map LOCALIZATION (localization_mode:=true)
    #
    # Load existing map from rtabmap_db and localize current position.
    # Does not add new nodes to the map (Mem/IncrementalMemory = false).
    # =========================================================================
    rtabmap_localization_node = Node(
        condition=is_localizing,
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':   use_sim_time,
            'frame_id':       'base_link',
            'odom_frame_id':  'odom',
            'map_frame_id':   'map',
            'subscribe_depth':      True,
            'subscribe_rgb':        True,
            'subscribe_scan':       True,
            'approx_sync':          True,
            'database_path':        rtabmap_db,
            'Rtabmap/DetectionRate': '1',
            'Grid/Sensor':          '2',
            'Grid/3D':              'false',
            'Grid/RangeMin':        '0.1',
            'Grid/RangeMax':        '5.0',
            'Grid/FootprintRadius': '0.19',
            # Important: Do not add new nodes
            'Mem/IncrementalMemory':    'false',
            'Mem/InitWMWithAllNodes':   'true',
            'Reg/Force3DoF':            'true',
        }],
        remappings=[
            ('rgb/image',       '/camera/color_image'),
            ('depth/image',     '/camera/depth_image'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan',            '/scan'),
            ('imu',             '/qcar2_imu'),
            ('odom',            '/rtabmap/odom'),
        ],
        arguments=[rtabmap_args])

    # =========================================================================
    # LAYER 5 - NAV2 COMMAND CONVERTER
    #   /cmd_vel_nav (geometry_msgs/Twist) -> /qcar2_motor_speed_cmd (MotorCommands)
    #   nav2_qcar_command_convert.cpp subscribes /cmd_vel_nav, publishes /qcar2_motor_speed_cmd
    # =========================================================================
    nav2_qcar2_converter_node = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
        output='screen')

    # =========================================================================
    # LAYER 6 - NAV2 NAVIGATION STACK
    # Maintained original pattern from qcar2_slam_and_nav_bringup_launch.py
    # RTAB-Map provides /map + TF map->odom. Only navigation_launch.py is required.
    # =========================================================================
    nav2_bringup_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'namespace':         namespace,
                'use_sim_time':      use_sim_time,
                'autostart':         autostart,
                'params_file':       params_file,
                'use_composition':   use_composition,
                'use_respawn':       use_respawn,
                'container_name':    'nav2_container',
            }.items()),
    ])

    # =========================================================================
    # BUILD LaunchDescription
    # Correct sequence: env -> args -> hardware -> TF -> SLAM -> Nav2
    # =========================================================================
    ld = LaunchDescription()

    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'))

    # Declare all arguments first
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_log_level)
    ld.add_action(declare_device_type)
    ld.add_action(declare_localization_mode)
    ld.add_action(declare_rtabmap_db)
    ld.add_action(declare_rtabmap_args)

    # Layer 1: Hardware
    ld.add_action(lidar_node)
    ld.add_action(rgbd_node)
    ld.add_action(qcar2_hardware_node)

    # Layer 2: TF
    ld.add_action(fixed_lidar_frame_node)
    ld.add_action(fixed_lidar_frame_virtual_node)
    ld.add_action(map_rotation_tf_node)

    # Layer 3: RTAB-Map Visual Odometry
    ld.add_action(rtabmap_odom_node)

    # Layer 4: RTAB-Map SLAM / Localization
    ld.add_action(rtabmap_slam_node)
    ld.add_action(rtabmap_localization_node)

    # Layer 5: Nav2 command bridge
    ld.add_action(nav2_qcar2_converter_node)

    # Layer 6: Nav2
    ld.add_action(nav2_bringup_group)

    return ld