// 串口桥接节点。把前面三层（协议、解析、串口、状态机）组装成一个 rclcpp 节点。
//
// ## 线程结构
//
// 读串口走**独立线程**而不是 ROS 定时器轮询。20 Hz 双帧下每秒约 1720 字节，定时器
// 轮询要么周期短到浪费 CPU，要么攒出延迟；阻塞读能在字节到达时立刻处理。
//
//   读线程：poll/read -> Parser::feed -> LinkState::onUplinkFrame -> 发布遥测
//   命令定时器（20 Hz，ROS 执行器线程）：取最新 ~/cmd -> 编码 -> 写串口
//   状态定时器（1 Hz，ROS 执行器线程）：兜底发布 link_status
//
// 只有两个线程，所以锁的职责很好划：
//   - state_mutex_ 保护 LinkState 与计数器（读线程写，两个定时器读）
//   - cmd_mutex_ 保护最新命令（订阅回调写，命令定时器读）
//   - 串口写只在命令定时器线程发生，握手应答也在那里发 —— 读线程**不写串口**，
//     这样 SerialPort::writeAll 不需要加锁。读线程发现该握手了，只置一个标志。
//
// ## 命令超时的处置：停发，交给 MCU 看门狗
//
// 上游超过 cmd_timeout_ms 没有新命令时，本节点**停止发送命令帧**，而不是继续
// 重发最后一条、也不是自己发一帧零命令。理由是让 MCU 那条已经实测过的安全路径
// 真正跑起来（200 ms 降级 -> 500 ms 输出归零 -> 2 s 重整握手）。继续重发会让
// MCU 以为链路健康，把「上游节点挂了」伪装成「一切正常」，这是最坏的失效模式。

#ifndef MCU_PROTOCOL__BRIDGE_NODE_HPP_
#define MCU_PROTOCOL__BRIDGE_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "mcu_protocol_msgs/msg/mcu_command.hpp"
#include "mcu_protocol_msgs/msg/mcu_diagnostics.hpp"
#include "mcu_protocol_msgs/msg/mcu_env.hpp"
#include "mcu_protocol_msgs/msg/mcu_imu_raw.hpp"
#include "mcu_protocol_msgs/msg/mcu_link_status.hpp"

#include "mcu_protocol/link_state.hpp"
#include "mcu_protocol/parser.hpp"
#include "mcu_protocol/protocol.hpp"
#include "mcu_protocol/serial_port.hpp"

namespace mcu_protocol
{

class BridgeNode : public rclcpp::Node
{
public:
  explicit BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~BridgeNode() override;

private:
  // --- 读线程
  void readLoop();
  void onFrame(const Frame & f, int64_t now_ms);
  void handleEvents(const LinkEvents & ev, const Frame & f);

  // --- 定时器（ROS 执行器线程）
  void onCommandTimer();
  void onStatusTimer();

  // --- 订阅回调
  void onCommand(const mcu_protocol_msgs::msg::McuCommand::SharedPtr msg);

  void publishLinkStatus();

  /// 填上行公共头部到消息。stamp 用本机接收时刻 —— MCU 时间另存字段，
  /// 两个时间基准不能混：mcu_time_ms 在 49.7 天回绕，且与本机时钟无关联。
  template<typename MsgT>
  void fillHeader(MsgT & msg, const Frame & f) const;

  int64_t nowMs() const;

  // --- 参数
  std::string device_;
  int baud_ = static_cast<int>(kBaudRate);
  int cmd_timeout_ms_ = 150;
  int command_rate_hz_ = 20;
  bool publish_diagnostics_ = true;

  // --- 串口与协议（仅读线程访问 port_/parser_，写串口见类注释）
  SerialPort port_;
  Parser parser_;

  mutable std::mutex state_mutex_;
  LinkState link_;

  std::mutex cmd_mutex_;
  mcu_protocol_msgs::msg::McuCommand latest_cmd_;
  bool have_cmd_ = false;
  int64_t latest_cmd_ms_ = 0;
  bool commands_flowing_ = false;

  /// 读线程发现 MCU 在等握手时置起，命令定时器线程消费后清掉。
  /// 用 atomic 而不是走 state_mutex_：只是一个标志，不值得为它扩大临界区。
  std::atomic<bool> want_handshake_{false};
  /// MCU 在 HELLO 里声明的版本，供应答时回填与校验。
  std::atomic<uint8_t> mcu_version_{0};

  uint16_t cmd_seq_ = 1;       ///< 下行命令序号，仅命令定时器线程访问
  uint16_t ack_seq_ = 1;       ///< HELLO_ACK 序号，同上

  /// 上一次诊断帧里的 MCU 侧丢字节计数，仅读线程访问（诊断帧在那里处理）。
  ///
  /// 存在的理由：这两个计数器是**累加型、只增不减**，所以「非零就告警」等于
  /// 电平触发 —— 板子历史上溢出过一次，之后每帧诊断都会命中，告警永久刷屏。
  /// 实测就是这样：上电两小时前的注错测试留下 rx_overrun=22，此后每 10 s
  /// 重复一次同样的 WARN，而那 22 次早已过去。改成只在**增量**时报。
  uint32_t prev_rx_overrun_ = 0;
  uint32_t prev_rx_dma_lost_ = 0;
  bool have_prev_diag_counts_ = false;
  uint16_t last_echo_seen_ = 0;
  bool downlink_healthy_ = false;

  std::atomic<bool> running_{false};
  std::thread read_thread_;

  rclcpp::Publisher<mcu_protocol_msgs::msg::McuImuRaw>::SharedPtr pub_imu_;
  rclcpp::Publisher<mcu_protocol_msgs::msg::McuEnv>::SharedPtr pub_env_;
  rclcpp::Publisher<mcu_protocol_msgs::msg::McuDiagnostics>::SharedPtr pub_diag_;
  rclcpp::Publisher<mcu_protocol_msgs::msg::McuLinkStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<mcu_protocol_msgs::msg::McuCommand>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace mcu_protocol

#endif  // MCU_PROTOCOL__BRIDGE_NODE_HPP_
