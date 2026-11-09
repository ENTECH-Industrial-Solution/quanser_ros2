"""Pieces shared by vslam_launch.py and vnav_launch.py (imported from the launch files)."""

import os

from launch_ros.actions import Node

from qcar2_web import obstacle, tuning

OBSTACLE_CONFIG = '~/.ros/qcar2_web_obstacle.json'


def camera_tf_node():
    """Static transform base_link -> realsense_optical from the camera mounting on the obstacle page."""
    cfg = obstacle.load_config(os.path.expanduser(OBSTACLE_CONFIG))
    x, y, z, qx, qy, qz, qw = tuning.camera_transform(cfg)
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name='realsense_frame',
        arguments=['--x', f'{x:.4f}', '--y', f'{y:.4f}', '--z', f'{z:.4f}',
                   '--qx', f'{qx:.6f}', '--qy', f'{qy:.6f}', '--qz', f'{qz:.6f}', '--qw', f'{qw:.6f}',
                   '--frame-id', 'base_link', '--child-frame-id', 'realsense_optical'])


def rtabmap_remappings(map_topic='map'):
    return [('rgb/image', '/vslam/rgb/image'), ('depth/image', '/vslam/depth/image'),
            ('rgb/camera_info', '/vslam/rgb/camera_info'), ('scan', '/scan'),
            ('map', map_topic)]


def bridge_node():
    return Node(package='qcar2_web', executable='rgbd_bridge', name='rgbd_bridge', output='screen')
