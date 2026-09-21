// 串口打开失败的**分类**测试。
//
// 为什么值得单独测：现场报「打不开串口」时，EBUSY（口被别的进程独占）和 ENOENT
// （设备还没枚举出来）的处理方向正好相反 —— 前者要去杀掉占用者，后者只要等。
// 而内核给的错误字符串非常含糊，二者都长得像「串口坏了」，于是现场会去反复拔插
// USB。这里用 pty 把两种情况都稳定造出来，锁住 openErrno() 的语义：一旦哪天有人
// 把 errno 的记录挪了位置或提前清零，这两个测试会立刻红。
//
// 不需要硬件：pty 是内核提供的虚拟终端，且支持 TIOCEXCL，正好能复现独占冲突。

#include <gtest/gtest.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <string>

#include "mcu_protocol/serial_port.hpp"

using namespace mcu_protocol;  // NOLINT

namespace
{

/// 一对 pty。析构时关掉 master，避免泄漏 fd 影响后续测试。
class Pty
{
public:
  Pty()
  {
    master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (master_ < 0) {
      return;
    }
    if (::grantpt(master_) != 0 || ::unlockpt(master_) != 0) {
      return;
    }
    const char * name = ::ptsname(master_);
    if (name != nullptr) {
      slave_ = name;
    }
  }

  ~Pty()
  {
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  Pty(const Pty &) = delete;
  Pty & operator=(const Pty &) = delete;

  bool valid() const { return master_ >= 0 && !slave_.empty(); }
  const std::string & slave() const { return slave_; }

private:
  int master_ = -1;
  std::string slave_;
};

}  // namespace

/// 设备不存在时，错误必须是 ENOENT —— 调用方据此选择「等枚举」而不是「找占用者」。
TEST(SerialPort, OpenMissingDeviceReportsEnoent)
{
  SerialPort port;
  std::string err;

  EXPECT_FALSE(port.open("/dev/mcu_definitely_absent", 921600, err));
  EXPECT_EQ(port.openErrno(), ENOENT);
  EXPECT_FALSE(port.isOpen());
}

/// 口被别的进程以 TIOCEXCL 独占时，错误必须是 EBUSY。
/// 这一条对应的正是「上次 launch 没退干净、新 launch 一直 Device or resource busy」
/// 的现场故障。
TEST(SerialPort, OpenExclusivelyHeldReportsEbusy)
{
  Pty pty;
  ASSERT_TRUE(pty.valid()) << "无法创建 pty，测试环境异常";

  const int holder = ::open(pty.slave().c_str(), O_RDWR | O_NOCTTY);
  ASSERT_GE(holder, 0);
  ASSERT_EQ(::ioctl(holder, TIOCEXCL), 0) << "pty 不支持 TIOCEXCL，无法复现独占冲突";

  SerialPort port;
  std::string err;
  EXPECT_FALSE(port.open(pty.slave(), 921600, err));
  EXPECT_EQ(port.openErrno(), EBUSY);
  EXPECT_FALSE(port.isOpen());

  ::close(holder);
}

/// 正常路径：pty 空闲时应能打开并取得独占，且 openErrno() 必须被清零
/// —— 否则上一次失败的 errno 会残留下来，把调用方骗进错误的分支。
TEST(SerialPort, OpenFreePtySucceedsAndClearsErrno)
{
  Pty pty;
  ASSERT_TRUE(pty.valid()) << "无法创建 pty，测试环境异常";

  SerialPort port;
  std::string err;

  // 先用一次失败把 errno 写成 ENOENT。
  EXPECT_FALSE(port.open("/dev/mcu_definitely_absent", 921600, err));
  ASSERT_EQ(port.openErrno(), ENOENT);

  EXPECT_TRUE(port.open(pty.slave(), 921600, err)) << err;
  EXPECT_TRUE(port.isOpen());
  EXPECT_TRUE(port.exclusive());
  EXPECT_EQ(port.openErrno(), 0);

  port.close();
  EXPECT_FALSE(port.isOpen());
  // close() 之后独占标志必须复位，见 serial_port.cpp 的说明。
  EXPECT_FALSE(port.exclusive());
}
