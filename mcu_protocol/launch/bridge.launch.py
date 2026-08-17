"""启动 mcu_bridge。

用法：
    ros2 launch mcu_protocol bridge.launch.py
    ros2 launch mcu_protocol bridge.launch.py device:=/dev/ttyUSB1
    ros2 launch mcu_protocol bridge.launch.py device:=/tmp/mcu_sim   # 接 proto_sim.py

命令行传入的 device 会覆盖 config/bridge.yaml 里的值 —— 换设备号不必改文件。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    device = LaunchConfiguration('device')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyUSB0',
            description='串口设备路径。插拔会变号，稳定做法是用 /dev/serial/by-id/ 下的路径',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('mcu_protocol'), 'config', 'bridge.yaml']
            ),
            description='参数文件路径',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='日志级别。调试链路问题时用 debug',
        ),
        Node(
            package='mcu_protocol',
            executable='mcu_bridge',
            name='mcu_bridge',
            # 节点名必须与 config/bridge.yaml 里的键一致，否则参数被静默忽略。
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'device': device}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
