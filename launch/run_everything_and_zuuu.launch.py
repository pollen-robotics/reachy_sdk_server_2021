import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    reachy_sdk_server_launch_dir = get_package_share_directory('reachy_sdk_server')

    zuuu_description_launch_dir = os.path.join(
        get_package_share_directory('zuuu_description'), 'launch')

    # Launch arguments
    arguments = [
        DeclareLaunchArgument(name='use_sim_time', default_value='False',
                              description='Flag to enable use_sim_time'),
    ]

    # Launch files to call
    launches = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(reachy_sdk_server_launch_dir, 'run_everything.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zuuu_description_launch_dir, 'zuuu_bringup_low_level_only.launch.py')),
        ),
    ]

    return LaunchDescription(arguments + launches)
