#include "mcu_protocol/protocol.hpp"

#include <cstring>

namespace mcu_protocol
{

uint16_t crc16Modbus(const uint8_t * data, size_t len) noexcept
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ data[i]);
    for (int bit = 0; bit < 8; ++bit) {
      if (crc & 1u) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001u);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

// ---------------------------------------------------------------- 解码

std::optional<HelloPayload> decodeHello(const Frame & f) noexcept
{
  if (f.type != static_cast<uint8_t>(FrameType::kHello) || f.payload_len != 8) {
    return std::nullopt;
  }
  HelloPayload h;
  h.version = f.payload[0];
  h.imu_floats = f.payload[1];
  h.env_floats = f.payload[2];
  h.uplink_hz = f.payload[3];
  h.downlink_hz = f.payload[4];
  return h;
}

std::optional<DiagnosticsPayload> decodeDiagnostics(const Frame & f) noexcept
{
  if (f.type != static_cast<uint8_t>(FrameType::kDiagnostics) || f.payload_len != 28) {
    return std::nullopt;
  }
  uint32_t v[7];
  std::memcpy(v, f.payload, sizeof(v));
  DiagnosticsPayload d;
  d.rx_frames_ok = v[0];
  d.rx_crc_err = v[1];
  d.rx_resync = v[2];
  d.rx_overrun = v[3];
  d.rx_dma_lost = v[4];
  d.tx_dropped = v[5];
  d.last_cmd_age_ms = v[6];
  return d;
}

std::optional<std::array<float, kImuFloats>> decodeImu(const Frame & f) noexcept
{
  if (f.type != static_cast<uint8_t>(FrameType::kImu) ||
    f.payload_len != kImuFloats * sizeof(float))
  {
    return std::nullopt;
  }
  std::array<float, kImuFloats> out{};
  std::memcpy(out.data(), f.payload, f.payload_len);
  return out;
}

std::optional<std::array<float, kEnvFloats>> decodeEnv(const Frame & f) noexcept
{
  if (f.type != static_cast<uint8_t>(FrameType::kEnv) ||
    f.payload_len != kEnvFloats * sizeof(float))
  {
    return std::nullopt;
  }
  std::array<float, kEnvFloats> out{};
  std::memcpy(out.data(), f.payload, f.payload_len);
  return out;
}

// ---------------------------------------------------------------- 编码

namespace
{

/// body = 长度字段之后到载荷结束的字节。整帧 = 4 + len + 2（契约 1.1）。
std::vector<uint8_t> wrap(uint8_t type, const uint8_t * body, size_t body_len)
{
  std::vector<uint8_t> out;
  out.reserve(6 + body_len);
  out.push_back(kSyncByte0);
  out.push_back(kSyncByte1);
  out.push_back(type);
  out.push_back(static_cast<uint8_t>(body_len));
  out.insert(out.end(), body, body + body_len);

  // CRC 覆盖类型字节起 len + 2 字节，存放在帧尾 2 字节小端。
  const uint16_t crc = crc16Modbus(out.data() + 2, body_len + 2);
  out.push_back(static_cast<uint8_t>(crc & 0xFFu));
  out.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFu));
  return out;
}

}  // namespace

std::vector<uint8_t> encodeHelloAck(uint16_t seq, uint8_t version, uint8_t cmd_layout)
{
  // len = 6: seq(2) + version(1) + cmd_layout(1) + reserved(2)
  uint8_t body[6] = {};
  std::memcpy(body + 0, &seq, 2);
  body[2] = version;
  body[3] = cmd_layout;
  body[4] = 0;
  body[5] = 0;
  return wrap(static_cast<uint8_t>(FrameType::kHelloAck), body, sizeof(body));
}

std::vector<uint8_t> encodeCommand(uint16_t seq, const std::array<float, kCommandFloats> & values)
{
  // len = 34: seq(2) + 8 * float32(32)。整帧精确 40 B。
  uint8_t body[2 + kCommandFloats * sizeof(float)] = {};
  std::memcpy(body + 0, &seq, 2);
  std::memcpy(body + 2, values.data(), kCommandFloats * sizeof(float));
  return wrap(static_cast<uint8_t>(FrameType::kCommand), body, sizeof(body));
}

}  // namespace mcu_protocol
