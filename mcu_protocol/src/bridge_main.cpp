// mcu_bridge 可执行入口。
//
// 用多线程执行器：命令定时器（20 Hz）和状态定时器（1 Hz）都会写日志、发布消息，
// 单线程执行器下若某个回调偶发变慢，会把命令定时器一起拖延 —— 而命令定时器迟到
// 200 ms 就会让 MCU 置过期位。串口读在自己的线程里，不受执行器影响。

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "mcu_protocol/bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<mcu_protocol::BridgeNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
