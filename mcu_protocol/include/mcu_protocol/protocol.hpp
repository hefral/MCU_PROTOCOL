// STM32F103 <-> ROS 2 串口协议 V2 的编解码。
//
// 本文件及 protocol.cpp / parser.cpp 不包含任何 ROS 头文件。这条纪律是从 MCU 端
// 抄来的（那边 proto_crc / proto_frame 不含 HAL 头），理由相同：CRC 参数、字节序、
// 长度语义、重同步逻辑这些最容易写错又最难在现场定位的东西全在这一层，不依赖
// 运行时环境就能在主机上用契约文档的参考帧做单元测试，一轮迭代几秒。
//
// 线上格式以 docs/STM32_ROS2_串口通信方案.md 为准。

#ifndef MCU_PROTOCOL__PROTOCOL_HPP_
#define MCU_PROTOCOL__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace mcu_protocol
{

// ---------------------------------------------------------------- 契约常量

constexpr uint32_t kBaudRate = 921600;
constexpr uint8_t kSyncByte0 = 0xAA;
constexpr uint8_t kSyncByte1 = 0x55;

constexpr uint8_t kProtocolVersion = 2;

/// 长度字段上界（契约 1.1）。超限即判伪帧头。
constexpr uint8_t kMaxLen = 128;
/// 整帧上界：4 + 128 + 2。
constexpr size_t kMaxFrame = 134;

/// 类型号取值域（契约 1.3）。0x00 与 0xFF 永久保留为非法值。
constexpr uint8_t kTypeMin = 0x01;
constexpr uint8_t kTypeMax = 0x1F;

/// 上行公共头部：序号(2)+时间戳(4)+last_cmd_seq(2)+状态(1)。
constexpr size_t kUplinkHeaderLen = 9;

enum class FrameType : uint8_t
{
  kImu = 0x01,
  kEnv = 0x02,
  kHello = 0x03,
  kDiagnostics = 0x04,
  kCommand = 0x10,
  kHelloAck = 0x11,
};

/// 上行帧型的判据：类型号落在 0x01~0x0F。下行为 0x10 起。
constexpr bool isUplink(uint8_t type) { return type >= kTypeMin && type <= 0x0F; }

// --- 状态字节位定义（契约 1.4）
constexpr uint8_t kStatusSafeState = 0x01;       ///< bit0 已进安全态，8 路输出全零
constexpr uint8_t kStatusCmdStale = 0x02;        ///< bit1 命令过期 200~500 ms，仍在执行
constexpr uint8_t kStatusNotHandshaked = 0x04;   ///< bit2 尚未握手
constexpr uint8_t kStatusVersionMismatch = 0x08; ///< bit3 协议版本不匹配

/// 帧间空闲超时（契约 3.1）。两端均须实现，且是安全相关而非优化项。
constexpr int kIdleTimeoutMs = 20;

/// MCU 期望的下行命令周期。
constexpr int kCommandPeriodMs = 50;

/// 命令帧的 8 个 float。语义由 cmd_layout 声明（契约 §6 待定）。
constexpr size_t kCommandFloats = 8;
constexpr size_t kImuFloats = 12;
constexpr size_t kEnvFloats = 2;

// ---------------------------------------------------------------- CRC

/// CRC-16/MODBUS: poly 0x8005(反射 0xA001), init 0xFFFF, refin/refout=true,
/// xorout 0x0000。自检 "123456789" -> 0x4B37。
///
/// 覆盖范围是从类型字节起 len + 2 字节，即 frame[2 .. 4+len-1]。两个最常见的
/// 错误是把帧头 AA 55 也算进去，以及漏掉类型字节只从长度字段开始算 —— 类型
/// 字节必须被覆盖，它决定整帧怎么解释，不保护它意味着单个比特翻转能把遥测帧
/// 变成命令帧且校验仍然通过。
uint16_t crc16Modbus(const uint8_t * data, size_t len) noexcept;

// ---------------------------------------------------------------- 解析结果

/// 一帧解析成功后的通用视图。payload 指向调用方缓冲区内部，仅在本次回调有效。
struct Frame
{
  uint8_t type = 0;
  uint8_t len = 0;

  // --- 上行公共头部，仅当 isUplink(type) 时有效
  uint16_t seq = 0;
  uint32_t mcu_time_ms = 0;
  uint16_t last_cmd_seq = 0;
  uint8_t status = 0;

  /// 载荷：上行为公共头部之后的字节，下行为长度字段之后的全部字节。
  const uint8_t * payload = nullptr;
  size_t payload_len = 0;
};

struct HelloPayload
{
  uint8_t version = 0;
  uint8_t imu_floats = 0;
  uint8_t env_floats = 0;
  uint8_t uplink_hz = 0;
  uint8_t downlink_hz = 0;
};

struct DiagnosticsPayload
{
  uint32_t rx_frames_ok = 0;
  uint32_t rx_crc_err = 0;
  uint32_t rx_resync = 0;
  uint32_t rx_overrun = 0;
  uint32_t rx_dma_lost = 0;
  uint32_t tx_dropped = 0;
  uint32_t last_cmd_age_ms = 0;
};

std::optional<HelloPayload> decodeHello(const Frame & f) noexcept;
std::optional<DiagnosticsPayload> decodeDiagnostics(const Frame & f) noexcept;
std::optional<std::array<float, kImuFloats>> decodeImu(const Frame & f) noexcept;
/// 返回 {温度, 压力}。顺序由 MCU 在 HELLO 里声明（env_floats=2）。
std::optional<std::array<float, kEnvFloats>> decodeEnv(const Frame & f) noexcept;

// ---------------------------------------------------------------- 编码

/// HELLO_ACK（0x11，12 B）。version 必须填 2 —— 填错 MCU 进「版本不匹配」态：
/// 不进正常模式、状态字节置 bit3、继续发 HELLO。这是刻意的，静默按不兼容格式
/// 运行是最坏结果。
std::vector<uint8_t> encodeHelloAck(
  uint16_t seq, uint8_t version = kProtocolVersion, uint8_t cmd_layout = 1);

/// 命令帧（0x10，精确 40 B）。MCU 对命令帧做严格长度校验，多一字节少一字节都
/// 直接拒收，不会部分解释。
std::vector<uint8_t> encodeCommand(uint16_t seq, const std::array<float, kCommandFloats> & values);

/// 序号按 uint16 取模比较。直接相减在回绕点会得到巨大的差值，误报大量丢帧。
/// 返回相邻两帧之间的间隔，正常为 1。
constexpr uint16_t seqGap(uint16_t current, uint16_t previous) noexcept
{
  return static_cast<uint16_t>(current - previous);
}

}  // namespace mcu_protocol

#endif  // MCU_PROTOCOL__PROTOCOL_HPP_
