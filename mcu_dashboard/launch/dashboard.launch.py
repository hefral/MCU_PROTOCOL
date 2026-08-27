"""Launch the standalone MCU dashboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bridge_ns = LaunchConfiguration('bridge_ns')
    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_ns',
            default_value='/mcu_bridge',
            description='Namespace of the running mcu_bridge node.',
        ),
        Node(
            package='mcu_dashboard',
            executable='mcu_dashboard',
            name='mcu_dashboard',
            output='screen',
            parameters=[{'bridge_ns': bridge_ns}],
        ),
    ])
