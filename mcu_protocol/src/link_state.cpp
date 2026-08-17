#include "mcu_protocol/link_state.hpp"

namespace mcu_protocol
{

bool LinkEvents::any() const noexcept
{
  return entered_safe_state || left_safe_state ||
         entered_cmd_stale || left_cmd_stale ||
         handshake_completed || handshake_lost ||
         version_mismatch_started ||
         uplink_lost || uplink_restored ||
         mcu_restarted;
}

LinkEvents LinkState::onUplinkFrame(const Frame & f, int64_t now_ms)
{
  LinkEvents ev;

  // --- 先判 MCU 是否复位过。必须在算序号缺口之前判，否则复位会被误报成
  //     数万帧丢失。
  if (have_prev_time_ && f.mcu_time_ms < prev_mcu_time_ms_) {
    ev.mcu_restarted = true;
    for (size_t i = 0; i <= kTypeMax; ++i) {
      have_prev_seq_[i] = false;
    }
  }
  have_prev_time_ = true;
  prev_mcu_time_ms_ = f.mcu_time_ms;

  // --- 序号缺口，按帧型各自记账（契约 1.4）。
  if (f.type <= kTypeMax) {
    if (have_prev_seq_[f.type]) {
      const uint16_t gap = seqGap(f.seq, prev_seq_[f.type]);
      // gap == 1 是正常推进；gap == 0 是重复帧（不该出现，但不算丢帧）。
      if (gap > 1) {
        counters_.uplink_gaps += gap - 1;
      }
    }
    prev_seq_[f.type] = f.seq;
    have_prev_seq_[f.type] = true;
  }

  ++counters_.uplink_frames;
  last_cmd_seq_echo_ = f.last_cmd_seq;
  last_frame_ms_ = now_ms;

  if (!uplink_alive_) {
    // 首帧不算「恢复」—— 那是初次建立，没有对应的 lost 事件。
    ev.uplink_restored = got_frame_;
    uplink_alive_ = true;
  }

  // --- 状态字节的边沿。这是 MCU 权威回报的状态，不是本机推测的。
  const uint8_t prev = got_frame_ ? status_ : kStatusNotHandshaked;
  const uint8_t now = f.status;

  const auto rose = [prev, now](uint8_t bit) {
      return (now & bit) != 0 && (prev & bit) == 0;
    };
  const auto fell = [prev, now](uint8_t bit) {
      return (now & bit) == 0 && (prev & bit) != 0;
    };

  ev.entered_safe_state = rose(kStatusSafeState);
  ev.left_safe_state = fell(kStatusSafeState);
  ev.entered_cmd_stale = rose(kStatusCmdStale);
  ev.left_cmd_stale = fell(kStatusCmdStale);
  ev.version_mismatch_started = rose(kStatusVersionMismatch);

  // 未握手位由 1 变 0 即握手完成；由 0 变 1 说明 MCU 复位或做了 2 s 重整，
  // 本机必须重新握手，否则它会一直发 HELLO 而我们一直不应答。
  ev.handshake_completed = fell(kStatusNotHandshaked);
  ev.handshake_lost = rose(kStatusNotHandshaked);

  // 首帧就带「已握手」的情形：本机刚启动而 MCU 早已和上一次运行握上。
  // 这仍然是一次握手完成，下游需要知道。
  if (!got_frame_ && (now & kStatusNotHandshaked) == 0) {
    ev.handshake_completed = true;
  }

  status_ = now;
  got_frame_ = true;
  return ev;
}

LinkEvents LinkState::onTick(int64_t now_ms)
{
  LinkEvents ev;
  if (uplink_alive_ && (now_ms - last_frame_ms_) > kUplinkTimeoutMs) {
    uplink_alive_ = false;
    ev.uplink_lost = true;
    // 上行断了，状态字节的信息立刻过期。保守起见按「未握手」对待：链路恢复后
    // 必须重新握手，而不是拿断线前的旧状态继续跑。
    status_ = kStatusNotHandshaked;
    for (size_t i = 0; i <= kTypeMax; ++i) {
      have_prev_seq_[i] = false;
    }
    have_prev_time_ = false;
  }
  return ev;
}

void LinkState::onReconnect(int64_t now_ms)
{
  status_ = kStatusNotHandshaked;
  got_frame_ = false;
  uplink_alive_ = false;
  for (size_t i = 0; i <= kTypeMax; ++i) {
    have_prev_seq_[i] = false;
  }
  have_prev_time_ = false;
  last_cmd_seq_echo_ = 0;
  last_frame_ms_ = now_ms;
  ++counters_.reconnects;
}

}  // namespace mcu_protocol
