import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('reachy_description'),
        'urdf/reachy.URDF',
    )

    return LaunchDescription([
        # See: https://github.com/ros2/rosidl_python/issues/79
        SetEnvironmentVariable('PYTHONOPTIMIZE', '1'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf],
        ),
        Node(
            package='reachy_kinematics',
            executable='arm_kinematics_service',
        ),

        Node(
            package='reachy_controllers',
            executable='joint_state_controller',
        ),
        Node(
            package='reachy_controllers',
            executable='camera_publisher',
        ),
        Node(
            package='reachy_focus',
            executable='camera_focus',
        ),
        Node(
            package='reachy_controllers',
            executable='camera_zoom_service',
        ),

        Node(
            package='reachy_sdk_server',
            executable='reachy_sdk_server',
        ),
        Node(
            package='reachy_sdk_server',
            executable='camera_server',
        ),
    ])
