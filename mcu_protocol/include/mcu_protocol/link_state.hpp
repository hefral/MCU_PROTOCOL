// 链路状态机。与 ROS 无关，因此可以在主机上单独测试状态跃迁 —— 握手时序和
// 安全态判定这类逻辑，用真实硬件复现一次要几十秒，用单元测试是几微秒。
//
// 这里刻意只做「判断」不做「动作」：什么时候该发 HELLO_ACK、什么时候该打日志，
// 由本类给出布尔量和边沿事件，实际的发送和打印在节点里。这样测试不需要串口，
// 也不需要 rclcpp。

#ifndef MCU_PROTOCOL__LINK_STATE_HPP_
#define MCU_PROTOCOL__LINK_STATE_HPP_

#include <cstdint>

#include "mcu_protocol/protocol.hpp"

namespace mcu_protocol
{

/// 本机侧观测到的链路统计。与 MCU 诊断帧里的计数器是**两套独立观测**，
/// 对照着看才能定位问题出在哪一段：本机 crc_errors 涨而 MCU rx_crc_err 不涨，
/// 说明是上行方向受扰；反过来则是下行方向。
struct LinkCounters
{
  uint64_t uplink_frames = 0;
  uint64_t uplink_gaps = 0;    ///< 按 seq 推算的丢帧数（不含回绕误算）
  uint64_t crc_errors = 0;
  uint64_t resyncs = 0;
  uint64_t reconnects = 0;
  uint64_t commands_sent = 0;
};

/// 一次状态更新产生的边沿事件。节点据此打日志 —— 边沿触发而非每帧触发，
/// 20 Hz 下每帧打一条会把日志刷得没法看，真正的异常反而被埋掉。
struct LinkEvents
{
  bool entered_safe_state = false;
  bool left_safe_state = false;
  bool entered_cmd_stale = false;
  bool left_cmd_stale = false;
  bool handshake_completed = false;
  bool handshake_lost = false;      ///< MCU 重新置了「未握手」位（复位或 2 s 重整）
  bool version_mismatch_started = false;
  bool uplink_lost = false;         ///< 超过容忍时间没收到任何上行帧
  bool uplink_restored = false;
  bool mcu_restarted = false;       ///< MCU 时间戳倒退，说明它复位过

  /// 有任何一个事件发生？节点用它决定是否需要立刻发一次 link_status。
  bool any() const noexcept;
};

/// 链路状态。只被节点的一个线程更新（读线程），发布时加锁拷走快照。
class LinkState
{
public:
  /// 收到一帧上行帧时调用。返回本次产生的边沿事件。
  ///
  /// 缺口检测**按帧型各自记账**。契约 1.4 明确每帧型各有一个序号计数器，这也是
  /// V2 相对 V1 的修正项之一：若按全局序号算，IMU seq=5 / ENV seq=5 / IMU seq=6
  /// 交错到达会被判成满屏丢帧。
  LinkEvents onUplinkFrame(const Frame & f, int64_t now_ms);

  /// 没有帧到来时也要周期调用，用于判断上行中断。
  LinkEvents onTick(int64_t now_ms);

  /// 串口刚打开/重连后调用，清掉所有「上一次」记忆。
  void onReconnect(int64_t now_ms);

  // --- 供发布与判断使用的快照
  uint8_t status() const noexcept { return status_; }
  bool safeState() const noexcept { return (status_ & kStatusSafeState) != 0; }
  bool cmdStale() const noexcept { return (status_ & kStatusCmdStale) != 0; }
  bool notHandshaked() const noexcept { return (status_ & kStatusNotHandshaked) != 0; }
  bool versionMismatch() const noexcept { return (status_ & kStatusVersionMismatch) != 0; }

  /// MCU 已确认握手（收到过上行帧且其未握手位为 0）。
  bool handshaked() const noexcept { return got_frame_ && !notHandshaked(); }
  bool uplinkAlive() const noexcept { return uplink_alive_; }

  /// MCU 回报的最近一次被接受的命令序号 —— 下行方向是否真的通了，看这个，
  /// 不要看「我发出去了多少」。
  uint16_t lastCmdSeqEcho() const noexcept { return last_cmd_seq_echo_; }
  int64_t lastFrameMs() const noexcept { return last_frame_ms_; }

  LinkCounters & counters() noexcept { return counters_; }
  const LinkCounters & counters() const noexcept { return counters_; }

  /// 多久没收到上行帧就判上行中断。20 Hz 双帧下正常间隔 50 ms，取 250 ms 有
  /// 足够余量，又比 MCU 自己的 500 ms 安全态阈值早，能先看到链路问题。
  static constexpr int64_t kUplinkTimeoutMs = 250;

private:
  uint8_t status_ = kStatusNotHandshaked;
  bool got_frame_ = false;

  /// 每帧型一个「上一次的序号」，索引即类型号（0x01~0x0F，取 kTypeMax + 1 够用）。
  bool have_prev_seq_[kTypeMax + 1] = {};
  uint16_t prev_seq_[kTypeMax + 1] = {};

  uint16_t last_cmd_seq_echo_ = 0;

  /// 上一帧的 MCU 时间戳。倒退即 MCU 复位过 —— 这比「缺口大得离谱」可靠得多：
  /// 复位后序号从 0 重来，prev=5000 遇上 seq=0 按模算会得到 61536，凭空报六万
  /// 多丢帧。时间戳倒退是个明确信号，据此清掉序号记忆而不计入缺口。
  bool have_prev_time_ = false;
  uint32_t prev_mcu_time_ms_ = 0;

  int64_t last_frame_ms_ = 0;
  bool uplink_alive_ = false;
  LinkCounters counters_;
};

}  // namespace mcu_protocol

#endif  // MCU_PROTOCOL__LINK_STATE_HPP_
