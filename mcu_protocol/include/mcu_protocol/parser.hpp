// 纯字节流帧解析器。不含 ROS 头文件，可在主机上直接单元测试。
//
// 关键纪律（契约 0.1 硬约束 2）：**不得假设帧边界与 read() 边界对齐。** CH340 走
// USB bulk 传输，会任意切分或聚合字节流 —— 一次 read() 可能拿到半帧，也可能拿到
// 三帧半。按读取块定界会在低流量时「恰好能用」，上了 20 Hz 双帧就开始随机丢帧。

#ifndef MCU_PROTOCOL__PARSER_HPP_
#define MCU_PROTOCOL__PARSER_HPP_

#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

#include "mcu_protocol/protocol.hpp"

namespace mcu_protocol
{

/// 解析器自身的计数，用于和 MCU 诊断帧对照。
struct ParserStats
{
  uint64_t frames_ok = 0;
  uint64_t crc_errors = 0;
  uint64_t resyncs = 0;        ///< 总重同步次数（含下面两类）
  uint64_t idle_resyncs = 0;   ///< 其中由 20 ms 空闲超时触发的
  uint64_t bad_headers = 0;    ///< 其中由类型/长度越界判定的
};

/// 纯字节流状态机。喂字节、拿帧，不关心字节从哪来。
class Parser
{
public:
  using FrameCallback = std::function<void(const Frame &)>;

  Parser() = default;

  /// 喂入任意长度的字节，对每个解析出的帧调用 cb。
  /// now_ms 是单调时钟毫秒，用于空闲超时计时。
  void feed(const uint8_t * data, size_t len, int64_t now_ms, const FrameCallback & cb);

  /// 没有字节到来时也必须周期调用，否则空闲超时永远不触发。
  ///
  /// 这不是可选的优化。CRC 淘汰伪帧的前提是「后续字节要继续到来」，对端停发时
  /// 这个前提就不成立：载荷里出现 AA 55、紧随两字节恰好构成合法的类型+长度，
  /// 解析器就会认下这个候选头并等待剩余字节 —— 而一帧 CRC 完全正确的帧可能
  /// 已经排在它后面进了缓冲区，却不会被交付。仅凭字节内容无法区分「真帧被截断
  /// （该等）」和「伪帧头挡住真帧且不会再有新字节（该立刻重扫）」，只有时间能区分。
  void tick(int64_t now_ms, const FrameCallback & cb);

  /// 持有一个尚未收满的候选帧？
  bool pending() const noexcept;

  /// 丢弃全部缓冲。仅用于串口重连 —— MCU 复位或掉电瞬间可能在半帧处截断，
  /// 重连后残留的半帧会和新字节拼成一帧由两次上电拼接出的乱码。
  void reset() noexcept;

  const ParserStats & stats() const noexcept { return stats_; }
  size_t bufferedBytes() const noexcept { return buf_.size(); }

private:
  /// 尝试从缓冲区头部取出一帧。
  /// 返回 true 表示消费了字节（可能产出帧，也可能只是丢弃了 1 字节），
  /// 返回 false 表示数据不够，需要等更多字节。
  bool tryOne(const FrameCallback & cb);

  /// 丢弃 1 个字节并计数。**只丢 1 个，不是 2 个，也不清空缓冲区** ——
  /// 真的帧头可能从第 2 个字节开始，已经排在伪帧头后面的真帧必须被救回。
  void dropOneByte();

  std::vector<uint8_t> buf_;
  int64_t last_byte_ms_ = 0;
  ParserStats stats_;
};

}  // namespace mcu_protocol

#endif  // MCU_PROTOCOL__PARSER_HPP_
