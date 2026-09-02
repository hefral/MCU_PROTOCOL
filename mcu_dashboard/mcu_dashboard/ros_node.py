"""ROS-facing node for the dashboard."""

import math

from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

from mcu_protocol_msgs.msg import (
    McuCommand,
    McuDiagnostics,
    McuEnv,
    McuImuRaw,
    McuLinkStatus,
)


class DashboardNode(Node):
    def __init__(self, store):
        super().__init__('mcu_dashboard')
        self.store = store
        self.bridge_ns = str(
            self.declare_parameter('bridge_ns', '/mcu_bridge').value
        ).rstrip('/')

        self.create_subscription(
            McuImuRaw,
            f'{self.bridge_ns}/imu_filtered',
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            McuEnv,
            f'{self.bridge_ns}/env',
            self._on_env,
            qos_profile_sensor_data,
        )

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            McuDiagnostics,
            f'{self.bridge_ns}/diagnostics',
            self._on_diagnostics,
            reliable,
        )

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            McuLinkStatus,
            f'{self.bridge_ns}/link_status',
            self.store.update_link,
            latched,
        )

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_pub = self.create_publisher(
            McuCommand, f'{self.bridge_ns}/cmd', command_qos
        )

    def _on_imu(self, msg: McuImuRaw) -> None:
        self.store.update_imu(msg.seq, msg.last_cmd_seq, msg.data)

    def _on_env(self, msg: McuEnv) -> None:
        self.store.update_env(
            msg.seq, msg.last_cmd_seq, msg.temperature, msg.pressure
        )

    def _on_diagnostics(self, msg: McuDiagnostics) -> None:
        self.store.update_diagnostics(msg)

    def publish_command(self, values) -> None:
        values = tuple(float(value) for value in values)
        if len(values) != 8:
            raise ValueError('motor command must contain exactly eight values')
        if not all(math.isfinite(value) for value in values):
            raise ValueError('motor command contains NaN or Inf')

        # The UI input range is -1..1. This transport method intentionally does not
        # clamp, so it cannot silently change a command generated elsewhere.
        msg = McuCommand()
        msg.values = values
        self.command_pub.publish(msg)
        self.store.note_command_published()
