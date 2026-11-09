# Navigation mode on a map made by the camera (V-SLAM), with Nav2:
#   map_server:  the saved grid (~/maps/<name>.yaml/.pgm), which Nav2's costmaps use
#   RTAB-Map in localisation mode: loads ~/maps/<name>.db and finds where the car is by
#       recognising what the camera sees (publishes map -> odom). It adds nothing to the map.
#   navigation_launch.py: planner, controller, behaviours, bt_navigator
# Obstacles still come from the lidar (/scan), as in nav_launch.py.
#
# The car is localised once the camera sees a place that was in the map: drive slowly and
# look around, or give a start pose on the web page ("ตั้งตำแหน่งรถ").
#
#   ros2 launch qcar2_web vnav_launch.py map:=$HOME/maps/office.yaml   (or: qcar-nav vnav office)

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from qcar2_web import tuning
from qcar2_web.launch_util import bridge_node, camera_tf_node, rtabmap_remappings

TUNED_PARAMS = '~/.ros/qcar2_web_nav2_params.yaml'


def _nodes(context):
    map_yaml = os.path.expanduser(LaunchConfiguration('map').perform(context))
    db = os.path.splitext(map_yaml)[0] + '.db'
    if not os.path.isfile(db):
        raise RuntimeError(f'{db} not found: this map was not made with the camera (V-SLAM)')
    ros, rtab = tuning.rtabmap_parameters(tuning.load_values('vslam'), localization=True)
    # A working copy: RTAB-Map writes to its database when it closes, the saved map stays as it was.
    os.makedirs(os.path.dirname(ros['database_path']), exist_ok=True)
    shutil.copyfile(db, ros['database_path'])

    base = os.path.join(get_package_share_directory('qcar2_web'), 'config', 'nav2_params.yaml')
    params_file = tuning.write_nav2_params(base, TUNED_PARAMS)
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    return [
        bridge_node(),
        camera_tf_node(),
        # RTAB-Map's own grid is not used here (the saved one is): keep it off /map.
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[ros, rtab], remappings=rtabmap_remappings('rtabmap/map')),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', respawn=True, respawn_delay=2.0,
             parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['map_server']}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'navigation_launch.py')),
            launch_arguments={'use_sim_time': 'false', 'autostart': 'true',
                              'params_file': params_file, 'use_composition': 'False',
                              'use_respawn': 'True'}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the map .yaml (with a .db next to it)'),
        OpaqueFunction(function=_nodes),
    ])
