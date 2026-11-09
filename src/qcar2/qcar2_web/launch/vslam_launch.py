# Mapping mode with the camera (V-SLAM): RTAB-Map builds the map from the RealSense colour +
# depth images; the wheel/gyro odometry from qcar2_odom (odom -> base_link) gives it the motion
# between frames and the camera recognises places it has seen before (loop closure).
# Optionally the lidar draws the 2D grid (grid_source on the web page).
#
# rgbd_bridge prepares the RealSense images (the RealSense node publishes no camera_info and
# its depth is not aligned with its colour image), and a static transform places the camera
# on the car from the mounting saved by the obstacle page.
#
# Save the map from the web page; it goes to ~/maps/<name>.yaml/.pgm (the grid for Nav2) and
# ~/maps/<name>.db (RTAB-Map's database, for localisation by vnav_launch.py).
# The options come from the web page ("ปรับค่า" tab); a change needs a new mapping run.
#
#   ros2 launch qcar2_web vslam_launch.py          (or: qcar-nav vslam)

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from qcar2_web import tuning
from qcar2_web.launch_util import bridge_node, camera_tf_node, rtabmap_remappings


def _nodes(context):
    ros, rtab = tuning.rtabmap_parameters(tuning.load_values('vslam'), localization=False)
    os.makedirs(os.path.dirname(ros['database_path']), exist_ok=True)
    return [
        bridge_node(),
        camera_tf_node(),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[ros, rtab], remappings=rtabmap_remappings(),
             arguments=['--delete_db_on_start']),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_nodes)])
