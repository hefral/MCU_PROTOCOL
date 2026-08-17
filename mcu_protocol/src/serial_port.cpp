#include "mcu_protocol/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace mcu_protocol
{

namespace
{

/// Linux 的 B* 常量。协议规定 921600，其余几个留着方便调试期降速试验。
bool baudConstant(uint32_t baud, speed_t & out)
{
  switch (baud) {
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
    case 460800: out = B460800; return true;
    case 921600: out = B921600; return true;
    default: return false;
  }
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, uint32_t baud, std::string & error)
{
  close();

  speed_t speed;
  if (!baudConstant(baud, speed)) {
    error = "不支持的波特率 " + std::to_string(baud) + "（协议要求 921600）";
    return false;
  }

  // O_NOCTTY：不要把串口当成控制终端，否则 Ctrl-C 之类的信号会打到本进程。
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if (fd_ < 0) {
    error = "打开 " + device + " 失败: " + std::strerror(errno);
    return false;
  }

  // 独占打开。两个节点同时开同一个口都在发命令帧，MCU 收到的是两路交错的序号，
  // 表现为「命令时而生效时而不生效」—— 拿到这个错误比事后查要省很多时间。
  if (::ioctl(fd_, TIOCEXCL) < 0) {
    error = "设置独占访问失败: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  struct termios tio;
  if (::tcgetattr(fd_, &tio) < 0) {
    error = "tcgetattr 失败: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  // 这一行是整个文件里最重要的一行。见头文件注释。
  ::cfmakeraw(&tio);

  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);

  tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);   // 1 位停止
  tio.c_cflag &= ~static_cast<tcflag_t>(PARENB);   // 无校验
  tio.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);  // 无硬件流控
  tio.c_cflag |= CLOCAL | CREAD;                   // 忽略 modem 线，允许接收
  tio.c_cflag = (tio.c_cflag & ~static_cast<tcflag_t>(CSIZE)) | CS8;

  // 由 poll() 管超时，这里设成「有多少拿多少，不等」。VMIN=0/VTIME=0 配合 poll
  // 才不会在 poll 说可读之后又阻塞在 read 里。
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tio) < 0) {
    error = "tcsetattr 失败: " + std::string(std::strerror(errno));
    close();
    return false;
  }

  // 丢掉开口瞬间内核里攒下的字节：可能是上次运行的残留，或 MCU 复位过程中的
  // 半帧。留着它们会和新字节拼成跨复位的乱码帧。
  ::tcflush(fd_, TCIOFLUSH);

  device_ = device;
  return true;
}

void SerialPort::close() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::read(uint8_t * buf, size_t len, int timeout_ms)
{
  if (fd_ < 0) {
    return -1;
  }

  struct pollfd pfd{};
  pfd.fd = fd_;
  pfd.events = POLLIN;

  const int pr = ::poll(&pfd, 1, timeout_ms);
  if (pr < 0) {
    // EINTR 不是错误 —— 信号打断了等待，按超时处理，下一轮继续。
    return (errno == EINTR) ? 0 : -1;
  }
  if (pr == 0) {
    return 0;  // 超时
  }
  // 设备被拔掉时 poll 报 POLLHUP/POLLERR/POLLNVAL 而不是可读，
  // 不判这个会陷入「poll 立刻返回 -> read 返回 0」的忙等死循环。
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    return -1;
  }

  const ssize_t n = ::read(fd_, buf, len);
  if (n < 0) {
    if (errno == EINTR || errno == EAGAIN) {
      return 0;
    }
    return -1;
  }
  if (n == 0) {
    // 已经 poll 说可读却读到 0 字节，通常意味着对端消失。
    return -1;
  }
  return n;
}

bool SerialPort::writeAll(const uint8_t * data, size_t len)
{
  if (fd_ < 0) {
    return false;
  }
  size_t done = 0;
  while (done < len) {
    const ssize_t n = ::write(fd_, data + done, len - done);
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      if (errno == EAGAIN) {
        // 内核缓冲满。等它排空一点再继续，不要空转。
        struct pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLOUT;
        if (::poll(&pfd, 1, 50) <= 0) {
          return false;
        }
        continue;
      }
      return false;
    }
    done += static_cast<size_t>(n);
  }
  return true;
}

}  // namespace mcu_protocol
