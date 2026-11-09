# Navigation mode on a saved map, with the standard Nav2 bringup:
#   localization_launch.py: map_server (the map) + AMCL (where the car is on it, from the lidar)
#   navigation_launch.py:   planner, controller, behaviours, bt_navigator
# Needs the robot nodes (qcar2_web_launch.py: hardware, lidar, wheel odometry). Goals come
# from the web page; its 'nav' mode turns Nav2's /cmd_vel_nav into motor commands, so
# nav2_qcar2_converter is not started.
#
# The Nav2 values saved from the web page (~/.ros/qcar2_web_tuning.json) are written over
# config/nav2_params.yaml when this starts.
#
#   ros2 launch qcar2_web nav_launch.py map:=$HOME/maps/office.yaml   (or: qcar-nav nav office)
# For a map made by the camera (V-SLAM) see vnav_launch.py.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from qcar2_web import tuning

TUNED_PARAMS = '~/.ros/qcar2_web_nav2_params.yaml'


def nav2_params(context):
    """The params file to use: the one given, else the package's with the web tuning applied."""
    given = LaunchConfiguration('params_file').perform(context)
    if given:
        return given
    base = os.path.join(get_package_share_directory('qcar2_web'), 'config', 'nav2_params.yaml')
    return tuning.write_nav2_params(base, TUNED_PARAMS)


def _include(context):
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    common = {
        'use_sim_time': 'false',
        'autostart': 'true',
        'params_file': nav2_params(context),
        'use_composition': 'False',
        'use_respawn': 'True',
    }
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'localization_launch.py')),
            launch_arguments={**common, 'map': LaunchConfiguration('map')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'navigation_launch.py')),
            launch_arguments=common.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the map .yaml'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 parameters (default: package file + web tuning)'),
        OpaqueFunction(function=_include),
    ])
