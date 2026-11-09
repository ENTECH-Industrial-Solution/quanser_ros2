# Starts the QCar2 camera nodes, the hardware node, the lidar, wheel odometry and the web
# interface. Mapping and navigation are separate, one at a time (see slam_launch.py and
# nav_launch.py, run as qcar2-slam.service / qcar2-nav@<map>.service by "qcar-nav"):
#
#   qcar-nav slam              # build a map with cartographer, then save it on the web page
#   qcar-nav nav <map>         # map_server + AMCL + Nav2 on ~/maps/<map>.yaml
#   qcar-nav stop
#
#   ros2 launch qcar2_web qcar2_web_launch.py
#   ros2 launch qcar2_web qcar2_web_launch.py start_lidar:=false        # no lidar
#   ros2 launch qcar2_web qcar2_web_launch.py start_hardware:=false   # qcar2_hardware already running elsewhere
#   ros2 launch qcar2_web qcar2_web_launch.py max_speed:=0.3 password:=secret
#   ros2 launch qcar2_web qcar2_web_launch.py person_model:=/path/to/yolov8n.pt
#   ros2 launch qcar2_web qcar2_web_launch.py stream_max_width:=480 jpeg_quality:=60   # weak Wi-Fi
#
# At boot this runs as two services (see system/install.sh): qcar2-web.service (web UI only)
# and qcar2-robot.service (everything else). Stop both first with "qcar-mode stop-all"
# before launching by hand.
#
# TF: wheel_odom publishes odom -> base_link; cartographer (SLAM) or AMCL (navigation)
# publishes map -> odom.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# QCar2 CSI camera numbers (from pal/products/qcar_config.json, cartype 2)
CSI_CAMERAS = {'right': 0, 'back': 1, 'front': 2, 'left': 3}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('host', default_value='0.0.0.0',
                              description='0.0.0.0 = listen on every interface'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('password', default_value='',
                              description='Empty = no password (anyone on the LAN can drive)'),
        DeclareLaunchArgument('max_speed', default_value='0.5', description='m/s at 100%'),
        DeclareLaunchArgument('max_steer', default_value='0.5', description='rad, capped at 0.6'),
        DeclareLaunchArgument('invert_steering', default_value='false'),
        DeclareLaunchArgument('stream_max_width', default_value='640',
                              description='Camera views sent to the browser at most this wide (px), 0 = full size'),
        DeclareLaunchArgument('jpeg_quality', default_value='70',
                              description='JPEG quality of the camera views (lower = less Wi-Fi traffic)'),
        DeclareLaunchArgument('max_fps', default_value='15'),
        DeclareLaunchArgument('start_web', default_value='true',
                              description='false = only the robot nodes (qcar2-robot.service)'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_csi', default_value='true'),
        DeclareLaunchArgument('csi_fps', default_value='15.0',
                              description='CSI camera frame rate; only the web views use them, '
                                          'and they send at most max_fps'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('person_model',
                              default_value='~/Documents/Quanser_demo/libraries/resources/'
                                            'pretrained_models/yolov8s-seg.engine',
                              description='YOLOv8 .engine/.pt used to find people (follow mode)'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Lidar and its frames (needed for SLAM and navigation)'),
        DeclareLaunchArgument('start_odom', default_value='true',
                              description='Wheel + gyro odometry (odom -> base_link)'),
    ]

    csi_nodes = [
        Node(
            package='qcar2_nodes',
            executable='csi',
            name=f'csi_{side}',
            namespace=f'qcar2/csi_{side}',
            parameters=[{'camera_num': num, 'frame_width': 820, 'frame_height': 410,
                         'frame_rate': ParameterValue(LaunchConfiguration('csi_fps'),
                                                      value_type=float)}],
            condition=IfCondition(LaunchConfiguration('start_csi')),
        )
        for side, num in CSI_CAMERAS.items()
    ]

    realsense = Node(
        package='qcar2_nodes',
        executable='rgbd',
        name='realsense',
        namespace='qcar2/realsense',
        condition=IfCondition(LaunchConfiguration('start_realsense')),
    )

    hardware = Node(
        package='qcar2_nodes',
        executable='qcar2_hardware',
        name='qcar2_hardware',
        condition=IfCondition(LaunchConfiguration('start_hardware')),
    )

    web = Node(
        package='qcar2_web',
        executable='web_server',
        name='qcar2_web',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_web')),
        parameters=[{
            'host': ParameterValue(LaunchConfiguration('host'), value_type=str),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'password': ParameterValue(LaunchConfiguration('password'), value_type=str),
            'max_speed': ParameterValue(LaunchConfiguration('max_speed'), value_type=float),
            'max_steer': ParameterValue(LaunchConfiguration('max_steer'), value_type=float),
            'invert_steering': ParameterValue(LaunchConfiguration('invert_steering'), value_type=bool),
            'command_timeout': 0.5,
            'stream_max_width': ParameterValue(LaunchConfiguration('stream_max_width'), value_type=int),
            'jpeg_quality': ParameterValue(LaunchConfiguration('jpeg_quality'), value_type=int),
            'max_fps': ParameterValue(LaunchConfiguration('max_fps'), value_type=float),
            'person_model': ParameterValue(LaunchConfiguration('person_model'), value_type=str),
        }],
    )

    lidar_on = IfCondition(LaunchConfiguration('start_lidar'))
    # At boot the lidar node used to fail with "A GPIO is in use by another application"
    # (qcar2_hardware opens the HIL card at the same moment) and then exit for good. Start
    # it a few seconds after the hardware node and restart it whenever it exits.
    lidar = TimerAction(period=4.0, actions=[
        Node(package='qcar2_nodes', executable='lidar', name='Lidar', condition=lidar_on,
             respawn=True, respawn_delay=3.0)])
    sensor_tf = Node(package='qcar2_nodes', executable='fixed_lidar_frame',
                     name='fixed_lidar_frame', condition=lidar_on)
    # fixed_lidar_frame publishes no frame for /qcar2_imu ("imu"), so cartographer drops all
    # IMU data and never starts mapping. Cartographer also aborts unless the IMU frame sits at
    # its tracking frame (base_scan), so put it there, axes aligned with base_link (the lidar
    # is turned 180 degrees).
    imu_tf = Node(package='tf2_ros', executable='static_transform_publisher', name='imu_frame',
                  arguments=['--yaw', '3.14159265359', '--frame-id', 'base_scan',
                             '--child-frame-id', 'imu'],
                  condition=lidar_on)
    odom = Node(package='qcar2_odom', executable='wheel_odom', name='wheel_odom',
                condition=IfCondition(LaunchConfiguration('start_odom')),
                respawn=True, respawn_delay=2.0)

    return LaunchDescription(args + csi_nodes + [realsense, hardware, web, lidar, sensor_tf, imu_tf,
                                                 odom])
