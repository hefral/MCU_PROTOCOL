// POSIX 串口封装。同样不含 ROS 头文件 —— 换个宿主环境或写个纯 C++ 工具都能用。
//
// 这一层只有两件事必须做对，但都很难在现场定位：
//
// 1. **必须 cfmakeraw()。** 默认的行规程模式会改写 0x0A/0x0D（ONLCR/ICRNL）、
//    把 0x11/0x13 当流控吃掉（IXON）、按行缓冲。协议载荷是二进制浮点，出现这些
//    字节值是常态 —— 症状是「大部分帧正常，偶尔 CRC 错」，而且只在某些数值下
//    复现，极难猜到是终端设置。
//
// 2. **不要按 read() 返回值定界帧。** read() 拿到多少字节是 USB 调度的结果，
//    与帧边界无关。本类只负责把字节搬出来，定界完全交给 Parser。

#ifndef MCU_PROTOCOL__SERIAL_PORT_HPP_
#define MCU_PROTOCOL__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace mcu_protocol
{

/// 阻塞式读、非阻塞式写的串口。单线程读 + 单线程写是安全的（内核各自加锁），
/// 但不要两个线程同时调 write()。
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// 打开并按协议要求配置（921600 8N1、raw、无流控）。
  /// 失败返回 false，错误原因写入 error。
  bool open(const std::string & device, uint32_t baud, std::string & error);

  void close() noexcept;
  bool isOpen() const noexcept { return fd_ >= 0; }

  /// 阻塞读，最多等 timeout_ms。
  /// 返回读到的字节数；0 表示超时（不是错误，对端可能只是没在发）；
  /// -1 表示串口出错或被拔掉，调用方应关闭并重连。
  ssize_t read(uint8_t * buf, size_t len, int timeout_ms);

  /// 全量写。返回 false 表示写失败（含被拔掉），调用方应重连。
  ///
  /// 写不完就重试直到写完或出错：921600 波特下 40 B 的命令帧约 0.43 ms，正常不会
  /// 被截断，但 USB 转串口在缓冲区满时确实会部分写入。半帧发出去在 MCU 那边表现
  /// 为 CRC 错 + 重同步，而不是「命令没生效」，排查方向会被带偏。
  bool writeAll(const uint8_t * data, size_t len);

  const std::string & device() const noexcept { return device_; }

private:
  int fd_ = -1;
  std::string device_;
};

}  // namespace mcu_protocol

#endif  // MCU_PROTOCOL__SERIAL_PORT_HPP_
