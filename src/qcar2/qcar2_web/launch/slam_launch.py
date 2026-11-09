# Mapping mode with the lidar: cartographer builds a map from the lidar (+ gyro). Needs the
# robot nodes (qcar2_web_launch.py: hardware, lidar, wheel odometry). Save the map from the web
# page; it goes to ~/maps/<name>.yaml/.pgm (+ .pbstream) and can then be used by nav_launch.py.
#
# The options come from the web page ("ปรับค่า" tab, saved in ~/.ros/qcar2_web_tuning.json);
# they are read once, here, so a change needs a new mapping run.
#
#   ros2 launch qcar2_web slam_launch.py          (or: qcar-nav slam)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from qcar2_web import tuning


def _nodes(context):
    config_dir = os.path.join(get_package_share_directory('qcar2_web'), 'config')
    values = tuning.load_values('cartographer')
    with open(os.path.join(config_dir, 'qcar2_web_2d.lua')) as f:
        lua = tuning.cartographer_lua(f.read(), values)
    out_dir = os.path.expanduser('~/.ros/qcar2_web_cartographer')
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, 'qcar2_web_2d.lua'), 'w') as f:
        f.write(lua)
    return [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', out_dir,
                       '-configuration_basename', 'qcar2_web_2d.lua'],
            remappings=[('/imu', '/qcar2_imu')]),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', str(values['resolution']),
                       '-publish_period_sec', '1.0']),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_nodes)])
