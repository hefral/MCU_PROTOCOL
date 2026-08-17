// 协议层单元测试。用契约文档 §3 的参考帧 —— 那些字节由 MCU 端代码实际生成，
// 所以这套测试通过意味着本端与固件在字节级一致，而不只是自己和自己一致。
//
// 这一层不含 ROS 头文件，因此这些测试在主机上几秒跑完，不需要板子也不需要串口。

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mcu_protocol/parser.hpp"
#include "mcu_protocol/protocol.hpp"

using namespace mcu_protocol;  // NOLINT

namespace
{

std::vector<uint8_t> fromHex(const std::string & hex)
{
  std::vector<uint8_t> out;
  for (size_t i = 0; i < hex.size();) {
    if (hex[i] == ' ') {
      ++i;
      continue;
    }
    out.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    i += 2;
  }
  return out;
}

// 契约文档 §3 的参考帧，由 MCU 端代码实际生成。
const char * kHelloHex =
  "AA 55 03 11 00 00 E8 03 00 00 00 00 04 02 0C 02 14 14 00 00 00 95 1C";
const char * kEnvHex =
  "AA 55 02 11 07 00 40 E2 01 00 2A 00 00 33 33 9B 40 A0 F5 C3 48 AF CD";
const char * kHelloAckHex = "AA 55 11 06 01 00 02 01 00 00 B7 62";
const char * kCmdHex =
  "AA 55 10 22 01 00 00 00 00 00 00 00 80 3E 00 00 00 BF 00 00 80 3F"
  " 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 E5 2A";

/// 收集解析出的帧，供断言使用。payload 指向解析器内部缓冲，必须在回调里拷走。
struct Collected
{
  uint8_t type;
  uint16_t seq;
  uint32_t mcu_time_ms;
  uint16_t last_cmd_seq;
  uint8_t status;
  std::vector<uint8_t> payload;
};

std::vector<Collected> parseAll(Parser & p, const std::vector<uint8_t> & bytes, int64_t now_ms = 0)
{
  std::vector<Collected> got;
  auto cb = [&got](const Frame & f) {
      got.push_back(
        Collected{f.type, f.seq, f.mcu_time_ms, f.last_cmd_seq, f.status,
          std::vector<uint8_t>(f.payload, f.payload + f.payload_len)});
    };
  p.feed(bytes.data(), bytes.size(), now_ms, cb);
  return got;
}

}  // namespace

// ---------------------------------------------------------------- CRC

TEST(Crc, SelfTestVector)
{
  // 契约 1.2 的自检向量。写错任何一个 CRC 参数这里就会失败。
  const std::string s = "123456789";
  EXPECT_EQ(0x4B37, crc16Modbus(reinterpret_cast<const uint8_t *>(s.data()), s.size()));
}

TEST(Crc, ReferenceFramesCheckOut)
{
  // 覆盖范围是从类型字节起 len + 2 字节。把帧头也算进去、或漏掉类型字节，
  // 都会让这四帧全部失败。
  for (const char * hex : {kHelloHex, kEnvHex, kHelloAckHex, kCmdHex}) {
    const auto raw = fromHex(hex);
    const uint8_t len = raw[3];
    ASSERT_EQ(raw.size(), 4u + len + 2u) << "整帧长应为 4 + len + 2: " << hex;
    const uint16_t want =
      static_cast<uint16_t>(raw[4 + len] | (static_cast<uint16_t>(raw[5 + len]) << 8));
    EXPECT_EQ(want, crc16Modbus(raw.data() + 2, static_cast<size_t>(len) + 2u)) << hex;
  }
}

TEST(Crc, TypeByteIsCovered)
{
  // 类型字节必须被 CRC 覆盖 —— 它决定整帧怎么解释。不保护它意味着单个比特翻转
  // 能把遥测帧变成命令帧且校验仍然通过。这是 V2 修正 V1 的关键回归项。
  auto raw = fromHex(kEnvHex);
  const uint16_t before = crc16Modbus(raw.data() + 2, static_cast<size_t>(raw[3]) + 2u);
  raw[2] ^= 0x01;  // 翻转类型字节的一个比特
  const uint16_t after = crc16Modbus(raw.data() + 2, static_cast<size_t>(raw[3]) + 2u);
  EXPECT_NE(before, after);
}

// ---------------------------------------------------------------- 解码

TEST(Decode, HelloFieldsMatchContract)
{
  Parser p;
  const auto got = parseAll(p, fromHex(kHelloHex));
  ASSERT_EQ(1u, got.size());
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHello), got[0].type);
  EXPECT_EQ(0u, got[0].seq);
  EXPECT_EQ(1000u, got[0].mcu_time_ms);          // E8 03 00 00 小端 = 1000 ms
  EXPECT_EQ(0u, got[0].last_cmd_seq);
  EXPECT_EQ(kStatusNotHandshaked, got[0].status); // 0x04 未握手

  Frame f;
  f.type = got[0].type;
  f.payload = got[0].payload.data();
  f.payload_len = got[0].payload.size();
  const auto h = decodeHello(f);
  ASSERT_TRUE(h.has_value());
  EXPECT_EQ(kProtocolVersion, h->version);
  EXPECT_EQ(kImuFloats, h->imu_floats);
  EXPECT_EQ(kEnvFloats, h->env_floats);
  EXPECT_EQ(20u, h->uplink_hz);
  EXPECT_EQ(20u, h->downlink_hz);
}

TEST(Decode, EnvFloatsAreLittleEndian)
{
  // 若解出 -1.6e38 之类的值，是字节序错了（指南 §3）。
  Parser p;
  const auto got = parseAll(p, fromHex(kEnvHex));
  ASSERT_EQ(1u, got.size());

  Frame f;
  f.type = got[0].type;
  f.payload = got[0].payload.data();
  f.payload_len = got[0].payload.size();
  const auto e = decodeEnv(f);
  ASSERT_TRUE(e.has_value());
  EXPECT_NEAR(4.85f, (*e)[0], 1e-4f);        // 33 33 9B 40
  EXPECT_NEAR(401325.0f, (*e)[1], 1e-1f);    // A0 F5 C3 48
}

// ---------------------------------------------------------------- 编码

TEST(Encode, HelloAckMatchesReferenceBytes)
{
  // 参考帧是 seq=1, version=2, cmd_layout=1。字节级相等，不是「长度对就行」。
  const auto got = encodeHelloAck(1, kProtocolVersion, 1);
  EXPECT_EQ(fromHex(kHelloAckHex), got);
  EXPECT_EQ(12u, got.size());
}

TEST(Encode, CommandMatchesReferenceBytes)
{
  // 参考帧 seq=1，8 个 float 依次为 0, 0.25, -0.5, 1.0, 0, 0, 0, 0。
  const std::array<float, kCommandFloats> vals{0.0f, 0.25f, -0.5f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const auto got = encodeCommand(1, vals);
  EXPECT_EQ(fromHex(kCmdHex), got);
  // MCU 对命令帧做严格长度校验，多一字节少一字节都直接拒收。
  EXPECT_EQ(40u, got.size());
}

TEST(Encode, RoundTripThroughParser)
{
  Parser p;
  auto bytes = encodeHelloAck(7);
  const auto cmd = encodeCommand(9, {1.5f, -2.5f, 0.0f, 0.125f, 0.0f, 0.0f, 0.0f, 99.5f});
  bytes.insert(bytes.end(), cmd.begin(), cmd.end());

  const auto got = parseAll(p, bytes);
  ASSERT_EQ(2u, got.size());
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHelloAck), got[0].type);
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kCommand), got[1].type);
}

// ------------------------------------------------- 硬约束 2：不得依赖读取边界

TEST(Parser, ByteAtATimeYieldsSameFrames)
{
  // CH340 走 USB bulk，一次 read() 可能拿到半帧或三帧半。逐字节投喂是最严苛的
  // 切分方式：能过这一项，任何切分方式都能过。
  auto bytes = fromHex(kHelloHex);
  const auto env = fromHex(kEnvHex);
  bytes.insert(bytes.end(), env.begin(), env.end());

  Parser p;
  std::vector<uint8_t> types;
  auto cb = [&types](const Frame & f) {types.push_back(f.type);};
  for (size_t i = 0; i < bytes.size(); ++i) {
    p.feed(&bytes[i], 1, 0, cb);
  }
  ASSERT_EQ(2u, types.size());
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHello), types[0]);
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kEnv), types[1]);
  EXPECT_EQ(0u, p.stats().crc_errors);
}

TEST(Parser, CoalescedReadYieldsAllFrames)
{
  // 反向情形：三帧半一次到达。
  auto bytes = fromHex(kHelloHex);
  for (const char * hex : {kEnvHex, kHelloHex, kEnvHex}) {
    const auto x = fromHex(hex);
    bytes.insert(bytes.end(), x.begin(), x.end());
  }
  const auto half = fromHex(kHelloHex);
  bytes.insert(bytes.end(), half.begin(), half.begin() + 8);  // 半帧

  Parser p;
  const auto got = parseAll(p, bytes);
  EXPECT_EQ(4u, got.size());
  EXPECT_TRUE(p.pending()) << "残留的半帧应被识别为未完成候选帧";
}

TEST(Parser, LeadingGarbageIsDiscardedOneByteAtATime)
{
  std::vector<uint8_t> bytes{0x12, 0x34, 0xAA, 0x55, 0xFF, 0x10};  // 含越界类型伪帧头
  const auto hello = fromHex(kHelloHex);
  bytes.insert(bytes.end(), hello.begin(), hello.end());

  Parser p;
  const auto got = parseAll(p, bytes);
  ASSERT_EQ(1u, got.size());
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHello), got[0].type);
  EXPECT_GT(p.stats().bad_headers, 0u) << "0xFF 是永久非法类型，应被范围检查拦下";
}

TEST(Parser, SyncPairInPayloadDoesNotBreakParsing)
{
  // 伪帧头声称的长度**能被后续字节填满**，所以 CRC 真的会被计算并失败。这一项
  // 检验的是失败之后怎么走：必须从帧头下一字节重扫，而不是跳过整个「声称的」
  // 帧长 —— 那两个 AA 55 可能本来就是载荷，真帧头就在你想跳过的区间里。
  //
  // 跳整帧的实现会从第 14 字节继续，正好落在 HELLO 帧内部，于是只能捞回 ENV
  // 一帧。所以 2 与 1 能区分这两种实现。
  std::vector<uint8_t> bytes{0xAA, 0x55, 0x01, 0x08};  // 声称 14 B 的伪帧头
  const auto hello = fromHex(kHelloHex);
  bytes.insert(bytes.end(), hello.begin(), hello.end());
  const auto env = fromHex(kEnvHex);
  bytes.insert(bytes.end(), env.begin(), env.end());

  Parser p;
  const auto got = parseAll(p, bytes);
  ASSERT_EQ(2u, got.size()) << "被伪帧头挡住的真帧必须最终被交付";
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHello), got[0].type);
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kEnv), got[1].type);
  EXPECT_GT(p.stats().crc_errors, 0u);
}

TEST(Parser, SingleBitFlipIsCaught)
{
  auto bytes = fromHex(kHelloHex);
  bytes[10] ^= 0x02;  // 翻转载荷里的一个比特

  Parser p;
  const auto got = parseAll(p, bytes);
  EXPECT_EQ(0u, got.size());
  EXPECT_GT(p.stats().crc_errors, 0u);
}

TEST(Parser, OversizedLengthIsRejectedAsBadHeader)
{
  // len > 128 判伪帧头，且必须在「等待数据」之前判 —— 否则解析器会为一个不可能
  // 的长度傻等下去。
  std::vector<uint8_t> bytes{0xAA, 0x55, 0x01, 0xFF};
  const auto hello = fromHex(kHelloHex);
  bytes.insert(bytes.end(), hello.begin(), hello.end());

  Parser p;
  const auto got = parseAll(p, bytes);
  ASSERT_EQ(1u, got.size());
  EXPECT_GT(p.stats().bad_headers, 0u);
}

// ------------------------------------------- 契约 3.1：帧间空闲超时（安全相关）

TEST(Parser, IdleTimeoutRescuesFrameStuckBehindFakeHeader)
{
  // 这是整个解析器最关键的一项，也是 MCU 端主机测试发现过真实缺陷的地方。
  //
  // 伪帧头 AA 55 01 39 声称一帧 63 字节，解析器认下它并等待剩余字节。此时一帧
  // CRC 完全正确的帧已经排在它后面进了缓冲区，却不会被交付 —— 因为按字节内容
  // 无法区分「真帧被截断（该等）」和「伪帧头挡住真帧且不会再有新字节（该重扫）」。
  // 只有时间能区分。
  std::vector<uint8_t> bytes{0xAA, 0x55, 0x01, 0x39};
  const auto hello = fromHex(kHelloHex);
  bytes.insert(bytes.end(), hello.begin(), hello.end());

  Parser p;
  std::vector<uint8_t> types;
  auto cb = [&types](const Frame & f) {types.push_back(f.type);};

  p.feed(bytes.data(), bytes.size(), 1000, cb);
  ASSERT_TRUE(types.empty()) << "此刻还不该交付：候选帧尚未超时";
  ASSERT_TRUE(p.pending());

  // 超时未到，不得动作。
  p.tick(1000 + kIdleTimeoutMs - 1, cb);
  EXPECT_TRUE(types.empty());

  // 超时到达：强制重同步，被挡住的真帧应被救回。
  p.tick(1000 + kIdleTimeoutMs, cb);
  ASSERT_EQ(1u, types.size());
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHello), types[0]);
  EXPECT_EQ(1u, p.stats().idle_resyncs);
}

TEST(Parser, IdleResyncDropsOnlyOneByte)
{
  // 只丢 1 字节、不清空缓冲区（契约 3.1 规则 3）。若实现成清空缓冲区，
  // 上面那一项会「看起来也过」—— 但排在伪帧头后面的真帧会被一起丢掉。
  // 这里用两帧来区分这两种实现。
  std::vector<uint8_t> bytes{0xAA, 0x55, 0x01, 0x39};
  for (const char * hex : {kHelloHex, kEnvHex}) {
    const auto x = fromHex(hex);
    bytes.insert(bytes.end(), x.begin(), x.end());
  }

  Parser p;
  std::vector<uint8_t> types;
  auto cb = [&types](const Frame & f) {types.push_back(f.type);};
  p.feed(bytes.data(), bytes.size(), 0, cb);
  p.tick(kIdleTimeoutMs, cb);

  ASSERT_EQ(2u, types.size()) << "清空缓冲区的实现会在这里只剩 0 帧";
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kHello), types[0]);
  EXPECT_EQ(static_cast<uint8_t>(FrameType::kEnv), types[1]);
}

TEST(Parser, IdleTimerTracksLastByteNotCandidateStart)
{
  // 计时基准是最近一次收到字节的时刻。若错按「候选帧头出现的时刻」计时，
  // 持续但缓慢的字节流会被反复误判为滞留。
  std::vector<uint8_t> head{0xAA, 0x55, 0x01, 0x39};
  Parser p;
  std::vector<uint8_t> types;
  auto cb = [&types](const Frame & f) {types.push_back(f.type);};

  p.feed(head.data(), head.size(), 0, cb);
  // 持续缓慢喂字节：每 10 ms 一个，永远不该触发超时。
  const uint8_t filler = 0x00;
  for (int64_t t = 10; t <= 100; t += 10) {
    p.feed(&filler, 1, t, cb);
    p.tick(t, cb);
  }
  EXPECT_EQ(0u, p.stats().idle_resyncs);
}

TEST(Parser, TickWithoutPendingIsNoop)
{
  Parser p;
  auto cb = [](const Frame &) {};
  p.tick(100000, cb);
  EXPECT_EQ(0u, p.stats().idle_resyncs);
  EXPECT_EQ(0u, p.stats().resyncs);
}

// ---------------------------------------------------------------- 序号回绕

TEST(SeqGap, WrapsAsUint16)
{
  // 2 字节序号在 20 Hz 下约 54.6 分钟绕一圈。按有符号整数直接相减会在回绕点
  // 得到 -65535，误报大量丢帧。
  EXPECT_EQ(1u, seqGap(1, 0));
  EXPECT_EQ(1u, seqGap(0, 65535));       // 回绕点：正常推进
  EXPECT_EQ(2u, seqGap(0, 65534));       // 回绕点：丢了一帧
  EXPECT_EQ(1u, seqGap(32768, 32767));
  EXPECT_EQ(0u, seqGap(5, 5));           // 重复帧
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
