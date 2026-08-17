// 链路状态机测试。这些跃迁在真实硬件上复现一次要几十秒（等安全态 500 ms、
// 等 2 s 重整、拔插串口），在这里是几微秒 —— 而且能构造出硬件上很难稳定重现的
// 情形：序号正好在回绕点、MCU 在半帧处复位、首帧就已握手。

#include <gtest/gtest.h>

#include <cstdint>

#include "mcu_protocol/link_state.hpp"

using namespace mcu_protocol;  // NOLINT

namespace
{

/// 构造一个上行帧视图。载荷内容与状态机无关，故留空。
Frame uplink(uint8_t type, uint16_t seq, uint32_t mcu_ms, uint8_t status, uint16_t last_cmd = 0)
{
  Frame f;
  f.type = type;
  f.seq = seq;
  f.mcu_time_ms = mcu_ms;
  f.last_cmd_seq = last_cmd;
  f.status = status;
  return f;
}

constexpr uint8_t kImu = static_cast<uint8_t>(FrameType::kImu);
constexpr uint8_t kEnv = static_cast<uint8_t>(FrameType::kEnv);

}  // namespace

// ---------------------------------------------------------------- 握手

TEST(LinkState, StartsUnhandshaked)
{
  LinkState s;
  EXPECT_TRUE(s.notHandshaked());
  EXPECT_FALSE(s.handshaked());
  EXPECT_FALSE(s.uplinkAlive());
}

TEST(LinkState, HandshakeCompletesOnNotHandshakedBitFalling)
{
  LinkState s;
  auto ev = s.onUplinkFrame(uplink(kImu, 0, 1000, kStatusNotHandshaked), 0);
  EXPECT_FALSE(ev.handshake_completed);
  EXPECT_FALSE(s.handshaked());

  ev = s.onUplinkFrame(uplink(kImu, 1, 1050, 0), 50);
  EXPECT_TRUE(ev.handshake_completed);
  EXPECT_TRUE(s.handshaked());
}

TEST(LinkState, FirstFrameAlreadyHandshakedStillReportsCompletion)
{
  // 本机刚启动而 MCU 早已和上一次运行握上。下游节点必须知道链路是可用的，
  // 否则它会一直等一个永远不会到来的「握手完成」。
  LinkState s;
  const auto ev = s.onUplinkFrame(uplink(kImu, 400, 20000, 0), 0);
  EXPECT_TRUE(ev.handshake_completed);
  EXPECT_TRUE(s.handshaked());
}

TEST(LinkState, HandshakeLostWhenMcuReassertsBit)
{
  // MCU 在安全态里待满 2 s 会退回未握手态并重新广播 HELLO。本机必须察觉，
  // 否则它一直不应答，而 MCU 一直在等 —— 链路静默地卡死。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);
  const auto ev = s.onUplinkFrame(uplink(kImu, 1, 1050, kStatusNotHandshaked), 50);
  EXPECT_TRUE(ev.handshake_lost);
  EXPECT_FALSE(s.handshaked());
}

// ---------------------------------------------------------------- 安全态边沿

TEST(LinkState, SafeStateIsEdgeTriggeredNotLevelTriggered)
{
  // 20 Hz 下若按电平触发，安全态期间每秒会产生 40 条事件。日志会被刷得没法看，
  // 真正的异常反而被埋掉。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);

  auto ev = s.onUplinkFrame(uplink(kImu, 1, 1050, kStatusSafeState), 50);
  EXPECT_TRUE(ev.entered_safe_state);
  EXPECT_TRUE(s.safeState());

  // 持续处于安全态：不再产生事件。
  for (int i = 2; i < 10; ++i) {
    ev = s.onUplinkFrame(uplink(kImu, static_cast<uint16_t>(i), 1000 + 50 * i, kStatusSafeState),
        50 * i);
    EXPECT_FALSE(ev.entered_safe_state) << "第 " << i << " 帧不应重复报进入安全态";
    EXPECT_FALSE(ev.left_safe_state);
  }

  ev = s.onUplinkFrame(uplink(kImu, 10, 1500, 0), 500);
  EXPECT_TRUE(ev.left_safe_state);
  EXPECT_FALSE(s.safeState());
}

TEST(LinkState, CmdStaleAndSafeStateAreIndependentBits)
{
  // 契约 1.4：200~500 ms 是降级态（bit1，仍在执行），≥500 ms 才是安全态
  // （bit0，输出全零）。下游对这两者的处置不同 —— 降级态不需要重置控制器。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);

  auto ev = s.onUplinkFrame(uplink(kImu, 1, 1050, kStatusCmdStale), 50);
  EXPECT_TRUE(ev.entered_cmd_stale);
  EXPECT_FALSE(ev.entered_safe_state);
  EXPECT_TRUE(s.cmdStale());
  EXPECT_FALSE(s.safeState());

  // 继续恶化：两位同时置起。
  ev = s.onUplinkFrame(uplink(kImu, 2, 1100, kStatusCmdStale | kStatusSafeState), 100);
  EXPECT_TRUE(ev.entered_safe_state);
  EXPECT_FALSE(ev.entered_cmd_stale) << "cmd_stale 已经是 1，不该重复报";
}

TEST(LinkState, SafeStateSupersedesCmdStalePerContract)
{
  // 契约 1.4：bit0 与 bit1 互斥，安全态取代过期态。所以恶化到安全态时 MCU 会
  // **清掉 bit1、置起 bit0**，于是同一帧里既有 entered_safe_state 也有
  // left_cmd_stale。节点若无条件地为 left_cmd_stale 打「过期状态解除」，那条
  // INFO 会紧跟在安全态告警后面，读起来像坏消息之后接了个好消息。
  //
  // 这个组合是接模拟器实跑时看出来的，日志里就是这么挨着出现的两行。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);
  s.onUplinkFrame(uplink(kImu, 1, 1200, kStatusCmdStale), 200);

  const auto ev = s.onUplinkFrame(uplink(kImu, 2, 1500, kStatusSafeState), 500);
  EXPECT_TRUE(ev.entered_safe_state);
  EXPECT_TRUE(ev.left_cmd_stale) << "MCU 清 bit1 置 bit0，两个事件必然同帧出现";
  EXPECT_TRUE(s.safeState());
  EXPECT_FALSE(s.cmdStale());
}

TEST(LinkState, SafeStateBitFallsWhenMcuRearmsWithoutRecovery)
{
  // bit0 落下有两条完全不同的原因，节点必须能区分：
  //   a) 命令恢复，MCU 重新执行 —— 状态字节转 0；
  //   b) 安全态满 2 s，MCU 退回未握手态重整 —— 状态字节变成 0x04，
  //      **输出仍然是零**，只是不再报「安全态」而是报「未握手」。
  // 情形 b 说成「恢复执行命令」是错的。区分依据是同帧的未握手位。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);
  ASSERT_TRUE(s.onUplinkFrame(uplink(kImu, 1, 1500, kStatusSafeState), 500).entered_safe_state);

  // 情形 b：2 s 后 MCU 重整。
  const auto ev = s.onUplinkFrame(uplink(kImu, 2, 3500, kStatusNotHandshaked), 2500);
  EXPECT_TRUE(ev.left_safe_state);
  EXPECT_TRUE(ev.handshake_lost) << "这才是这一刻真正发生的事";
  EXPECT_TRUE(s.notHandshaked());
  EXPECT_FALSE(s.handshaked());
}

TEST(LinkState, SafeStateBitFallsOnGenuineRecovery)
{
  // 情形 a：命令恢复，状态字节干净地转 0。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);
  s.onUplinkFrame(uplink(kImu, 1, 1500, kStatusSafeState), 500);

  const auto ev = s.onUplinkFrame(uplink(kImu, 2, 1600, 0), 600);
  EXPECT_TRUE(ev.left_safe_state);
  EXPECT_FALSE(ev.handshake_lost);
  EXPECT_TRUE(s.handshaked());
}

TEST(LinkState, VersionMismatchIsReported)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, kStatusNotHandshaked), 0);
  const auto ev = s.onUplinkFrame(
    uplink(kImu, 1, 1050, kStatusNotHandshaked | kStatusVersionMismatch), 50);
  EXPECT_TRUE(ev.version_mismatch_started);
  EXPECT_TRUE(s.versionMismatch());
}

// ---------------------------------------------------------------- 序号缺口

TEST(LinkState, GapsCountedPerFrameTypeNotGlobally)
{
  // 契约 1.4：每帧型各有一个序号计数器。IMU 和 ENV 交错到达时，按全局序号算
  // 会把正常流量判成满屏丢帧 —— 这是 V2 相对 V1 的修正项之一。
  LinkState s;
  int64_t t = 0;
  for (uint16_t i = 0; i < 20; ++i) {
    s.onUplinkFrame(uplink(kImu, i, 1000 + 50 * i, 0), t);
    s.onUplinkFrame(uplink(kEnv, i, 1000 + 50 * i, 0), t);
    t += 50;
  }
  EXPECT_EQ(0u, s.counters().uplink_gaps) << "交错的两个帧型各自连续，不该有缺口";
  EXPECT_EQ(40u, s.counters().uplink_frames);
}

TEST(LinkState, MissingFramesAreCounted)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 10, 1000, 0), 0);
  s.onUplinkFrame(uplink(kImu, 14, 1200, 0), 200);  // 丢了 11 12 13
  EXPECT_EQ(3u, s.counters().uplink_gaps);
}

TEST(LinkState, GapAcrossSeqWrapIsNotMisreported)
{
  // 2 字节序号在 20 Hz 下约 54.6 分钟绕一圈。长时间跑的系统必然经过这里，
  // 按有符号相减会在回绕点报 65535 帧丢失。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 65534, 1000, 0), 0);
  s.onUplinkFrame(uplink(kImu, 65535, 1050, 0), 50);
  s.onUplinkFrame(uplink(kImu, 0, 1100, 0), 100);
  s.onUplinkFrame(uplink(kImu, 1, 1150, 0), 150);
  EXPECT_EQ(0u, s.counters().uplink_gaps);
}

TEST(LinkState, DuplicateSeqIsNotCountedAsGap)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 5, 1000, 0), 0);
  s.onUplinkFrame(uplink(kImu, 5, 1000, 0), 50);
  EXPECT_EQ(0u, s.counters().uplink_gaps);
}

// ---------------------------------------------------------------- MCU 复位

TEST(LinkState, McuRestartDetectedByTimestampGoingBackwards)
{
  // 复位后序号从 0 重来。若不识别复位，prev=5000 遇上 seq=0 按模算得到 61536，
  // 凭空报六万多丢帧 —— 一眼假，但会污染整个统计。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 5000, 300000, 0), 0);

  const auto ev = s.onUplinkFrame(uplink(kImu, 0, 1000, kStatusNotHandshaked), 50);
  EXPECT_TRUE(ev.mcu_restarted);
  EXPECT_TRUE(ev.handshake_lost) << "复位后 MCU 重新置未握手位，本机须重新握手";
  EXPECT_EQ(0u, s.counters().uplink_gaps) << "复位不是丢帧";
}

TEST(LinkState, NormalTimestampAdvanceIsNotARestart)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);
  const auto ev = s.onUplinkFrame(uplink(kImu, 1, 1050, 0), 50);
  EXPECT_FALSE(ev.mcu_restarted);
}

// ---------------------------------------------------------------- 上行中断

TEST(LinkState, UplinkLostAfterTimeout)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 1000);
  EXPECT_TRUE(s.uplinkAlive());

  // 阈值之内：不动作。
  auto ev = s.onTick(1000 + LinkState::kUplinkTimeoutMs);
  EXPECT_FALSE(ev.uplink_lost);
  EXPECT_TRUE(s.uplinkAlive());

  ev = s.onTick(1000 + LinkState::kUplinkTimeoutMs + 1);
  EXPECT_TRUE(ev.uplink_lost);
  EXPECT_FALSE(s.uplinkAlive());
  EXPECT_TRUE(s.notHandshaked()) << "上行断了，状态字节立刻过期，须按未握手对待";
}

TEST(LinkState, UplinkLostFiresOnlyOnce)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 1000);
  ASSERT_TRUE(s.onTick(2000).uplink_lost);
  for (int64_t t = 2050; t < 5000; t += 50) {
    EXPECT_FALSE(s.onTick(t).uplink_lost) << "断线期间不该反复报，日志会被刷爆";
  }
}

TEST(LinkState, UplinkRestoredAfterLoss)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 1000);
  ASSERT_TRUE(s.onTick(2000).uplink_lost);

  const auto ev = s.onUplinkFrame(uplink(kImu, 1, 1050, 0), 3000);
  EXPECT_TRUE(ev.uplink_restored);
  EXPECT_TRUE(s.uplinkAlive());
}

TEST(LinkState, FirstFrameIsNotARestoration)
{
  // 初次建立链路没有对应的 lost 事件，报「恢复」会让下游困惑。
  LinkState s;
  const auto ev = s.onUplinkFrame(uplink(kImu, 0, 1000, 0), 0);
  EXPECT_FALSE(ev.uplink_restored);
}

TEST(LinkState, TickBeforeAnyFrameIsNoop)
{
  LinkState s;
  const auto ev = s.onTick(100000);
  EXPECT_FALSE(ev.uplink_lost);
  EXPECT_FALSE(ev.any());
}

// ---------------------------------------------------------------- 重连

TEST(LinkState, ReconnectClearsAllMemory)
{
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 5000, 300000, 0), 0);
  ASSERT_TRUE(s.handshaked());

  s.onReconnect(1000);
  EXPECT_TRUE(s.notHandshaked());
  EXPECT_FALSE(s.handshaked());
  EXPECT_FALSE(s.uplinkAlive());
  EXPECT_EQ(1u, s.counters().reconnects);

  // 重连后的第一帧序号从任意值开始，不该被算成缺口。
  s.onUplinkFrame(uplink(kImu, 77, 500, kStatusNotHandshaked), 1100);
  EXPECT_EQ(0u, s.counters().uplink_gaps);
}

// ---------------------------------------------------------------- 下行确认

TEST(LinkState, LastCmdSeqEchoTracksMcuAcceptance)
{
  // 下行方向是否真的通了，看 MCU 回报的 last_cmd_seq，不要看「我发出去了多少」：
  // 写进内核缓冲区不等于 MCU 收到了、更不等于 CRC 校验通过了。
  LinkState s;
  s.onUplinkFrame(uplink(kImu, 0, 1000, 0, 41), 0);
  EXPECT_EQ(41u, s.lastCmdSeqEcho());
  s.onUplinkFrame(uplink(kImu, 1, 1050, 0, 42), 50);
  EXPECT_EQ(42u, s.lastCmdSeqEcho());
}

TEST(LinkEventsAny, DetectsEachEvent)
{
  LinkEvents ev;
  EXPECT_FALSE(ev.any());
  ev.entered_safe_state = true;
  EXPECT_TRUE(ev.any());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
