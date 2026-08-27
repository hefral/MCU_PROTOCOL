"""Application entry point."""

import sys
import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from PyQt5.QtWidgets import QApplication

from .main_window import MainWindow
from .model import TelemetryStore
from .ros_node import DashboardNode


def main(args=None):
    rclpy.init(args=args)
    app = QApplication([sys.argv[0]])
    app.setApplicationName('MCU Dashboard')

    store = TelemetryStore()
    node = DashboardNode(store)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ros():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    spin_thread = threading.Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    window = MainWindow(node, store)
    window.show()
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: app.quit())
    result = app.exec()

    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    return result


if __name__ == '__main__':
    raise SystemExit(main())
