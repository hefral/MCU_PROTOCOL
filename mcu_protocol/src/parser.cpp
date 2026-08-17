#include "mcu_protocol/parser.hpp"

#include <cstring>

namespace mcu_protocol
{

void Parser::feed(const uint8_t * data, size_t len, int64_t now_ms, const FrameCallback & cb)
{
  if (data != nullptr && len > 0) {
    buf_.insert(buf_.end(), data, data + len);
    // 计时基准是**最近一次收到字节的时刻**，不是候选帧头出现的时刻 —— 否则
    // 持续但缓慢的字节流会被反复误判为滞留。
    last_byte_ms_ = now_ms;
  }
  while (tryOne(cb)) {
  }
}

void Parser::tick(int64_t now_ms, const FrameCallback & cb)
{
  if (!pending()) {
    return;
  }
  if (now_ms - last_byte_ms_ < kIdleTimeoutMs) {
    return;
  }
  // 强制重同步。只丢 1 字节，不清空缓冲区（契约 3.1 规则 3）。
  dropOneByte();
  ++stats_.idle_resyncs;
  while (tryOne(cb)) {
  }
}

bool Parser::pending() const noexcept
{
  if (buf_.size() < 4) {
    return false;
  }
  if (buf_[0] != kSyncByte0 || buf_[1] != kSyncByte1) {
    return false;
  }
  const size_t total = 4u + static_cast<size_t>(buf_[3]) + 2u;
  return buf_.size() < total;
}

void Parser::reset() noexcept
{
  buf_.clear();
}

void Parser::dropOneByte()
{
  buf_.erase(buf_.begin());
  ++stats_.resyncs;
}

bool Parser::tryOne(const FrameCallback & cb)
{
  // 判定顺序严格对应契约 §3，每一步都不能省。

  // 1. 不足 2 字节 -> 等待。
  if (buf_.size() < 2) {
    return false;
  }

  // 2. 找 AA 55。不匹配就只丢 1 个字节再重找 —— 不是丢 2 个。
  if (buf_[0] != kSyncByte0 || buf_[1] != kSyncByte1) {
    dropOneByte();
    return true;
  }

  // 3. 不足 4 字节 -> 等待（还不知道帧长）。
  if (buf_.size() < 4) {
    return false;
  }

  const uint8_t type = buf_[2];
  const uint8_t len = buf_[3];

  // 4. 类型与长度范围检查。这一步不能省：它把可冒充帧头的字节值从 254 个压到
  //    31 个。省掉它的后果在 MCU 端主机测试里出现过真实缺陷 —— 连续 AA 55 洪流
  //    中 type=0xAA/len=0x55 构成语法完美的候选头，声称 91 字节，把真帧扣在
  //    缓冲区里不交付。
  if (len > kMaxLen || type < kTypeMin || type > kTypeMax) {
    dropOneByte();
    ++stats_.bad_headers;
    return true;
  }

  // 5. 整帧长 = 4 + len + 2。不够 -> 等待。
  const size_t total = 4u + static_cast<size_t>(len) + 2u;
  if (buf_.size() < total) {
    return false;
  }

  // 6. 校验 CRC。失败则丢 1 字节重扫，**不要跳过整帧** —— 这两个 AA 55 可能
  //    本来就是载荷，真帧头就在你想跳过的区间里。
  const uint16_t want =
    static_cast<uint16_t>(buf_[4 + len] | (static_cast<uint16_t>(buf_[5 + len]) << 8));
  const uint16_t got = crc16Modbus(buf_.data() + 2, static_cast<size_t>(len) + 2u);
  if (want != got) {
    dropOneByte();
    ++stats_.crc_errors;
    return true;
  }

  // 7. 通过 -> 交付，消费整帧。
  Frame f;
  f.type = type;
  f.len = len;

  const uint8_t * body = buf_.data() + 4;
  if (isUplink(type) && len >= kUplinkHeaderLen) {
    // 上行公共头部 13 B（含帧头与长度），全部小端。
    std::memcpy(&f.seq, body + 0, 2);
    std::memcpy(&f.mcu_time_ms, body + 2, 4);
    std::memcpy(&f.last_cmd_seq, body + 6, 2);
    f.status = body[8];
    f.payload = body + kUplinkHeaderLen;
    f.payload_len = static_cast<size_t>(len) - kUplinkHeaderLen;
  } else {
    f.payload = body;
    f.payload_len = len;
  }

  ++stats_.frames_ok;
  if (cb) {
    cb(f);
  }

  // 回调之后才擦除：payload 指向 buf_ 内部，回调期间必须有效。
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(total));
  return true;
}

}  // namespace mcu_protocol
