# mcu_protocol — 上层节点接入指南

桥接节点 `mcu_bridge` 把 MCU 的串口协议翻译成话题。**上层节点不需要知道串口、
帧格式、CRC、握手的存在** —— 订阅遥测、往 `~/cmd` 发命令即可。

怎么启动、怎么验证通信正常，见[仓库根 README](../README.md)。本文档讲**怎么用**。

## 话题总览

节点默认名 `mcu_bridge`，所以 `~/x` 实际是 `/mcu_bridge/x`。

| 话题 | 方向 | 类型 | QoS |
|---|---|---|---|
| `~/imu_raw` | 出 | `McuImuRaw` | SensorData（**best_effort**，深度 5） |
| `~/env` | 出 | `McuEnv` | SensorData（**best_effort**，深度 5） |
| `~/diagnostics` | 出 | `McuDiagnostics` | reliable，深度 10 |
| `~/link_status` | 出 | `McuLinkStatus` | reliable + **transient_local**，深度 1 |
| `~/cmd` | **入** | `McuCommand` | reliable，深度 1 |

## QoS：订阅端不匹配会静默收不到

**这是最容易踩的坑。** `~/imu_raw` 和 `~/env` 是 best_effort 发布的。用默认
（reliable）QoS 去订阅**不会匹配**，结果是话题存在、`ros2 topic hz` 有数据，
而你的回调一次都不触发，且没有任何报错。

```cpp
// 正确：遥测用 SensorDataQoS
sub_ = create_subscription<mcu_protocol_msgs::msg::McuImuRaw>(
  "/mcu_bridge/imu_raw", rclcpp::SensorDataQoS(),
  std::bind(&MyNode::onImu, this, std::placeholders::_1));
```

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(McuImuRaw, '/mcu_bridge/imu_raw',
                         self.on_imu, qos_profile_sensor_data)
```

`~/link_status` 是 **transient_local（latched）** 的：晚启动的节点一订阅就立刻
拿到当前状态，不必等下一次变化。订阅端也要声明 `transient_local` 才能收到这个
缓存值，否则只能等下一次发布（最长 1 秒）。

---

# 实时查看：mcu_monitor

```bash
ros2 run mcu_protocol mcu_monitor          # 与 mcu_bridge 分开的终端，它会占满整屏
ros2 run mcu_protocol mcu_monitor --ros-args -p bridge_ns:=/mcu_bridge -p refresh_hz:=5.0
```

一屏显示四路话题：链路状态位（连同后果，不只是位名）、16 个 IMU float、温压、
诊断计数器。**不要和 `ros2 launch` 放同一个终端** —— 桥接节点的日志会插进重画里
把画面搅烂。

三处与 `ros2 topic echo` 的差别，都是为了回答 echo 回答不了的问题：

- **刷新率与数据率解耦**（固定 5 Hz 取最新一帧）。20 Hz 下 16 个浮点滚屏读不了。
- **显示实测帧率与序号空洞**。上行是否健康的判据是帧率、零空洞、零校验错，
  echo 一个都不给。
- **明确标出「已停止」**。静止的画面和活着的数据在屏幕上完全一样 —— 超过判停阈值
  （三个周期，最少 0.5 s）没有新帧就写明，并显示距上一帧多久。

诊断计数器显示的是**自监视启动以来的增量**，不是「非零就告警」：这些计数器自 MCU
上电起累加，非零只说明那条路径曾经执行过，可能来自更早。判断链路是否正在恶化要看
增量。

需要精确的单帧内容、或者要把数据喂给别的程序时，仍然用 `ros2 topic echo`
（记得带 `--qos-reliability best_effort`，否则遥测话题匹配不上，会一直空等）。

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `bridge_ns` | `/mcu_bridge` | 桥接节点命名空间。launch 里改了节点名要一起改，否则四路话题全是「等待首帧」 |
| `refresh_hz` | `5.0` | 屏幕重画频率，与数据速率无关 |
| `expected_uplink_hz` | `20` | 仅用于「实测 vs 期望」对照。改了固件上行节奏就要改它，否则屏幕上的「期望」是一句假话 |

它是 Python 调试工具，只 `exec_depend` rclpy，不在控制路径上；源文件不带 `.py`
后缀的理由写在 `CMakeLists.txt` 的注释里（与 `--symlink-install` 有关）。

---

# 下发命令

## 最小示例

```bash
ros2 topic pub /mcu_bridge/cmd mcu_protocol_msgs/msg/McuCommand \
  "{values: [0,0,0,0,0,0,0,0]}" -r 20
```

```cpp
#include "mcu_protocol_msgs/msg/mcu_command.hpp"

pub_ = create_publisher<mcu_protocol_msgs::msg::McuCommand>("/mcu_bridge/cmd", 1);

mcu_protocol_msgs::msg::McuCommand cmd;
cmd.values = {1.5F, -2.25F, 3.125F, 10.0F, -0.5F, 100.0F, 0.001F, -999.875F};
pub_->publish(cmd);
```

`values` 是 `float32[8]` 定长数组，C++ 里是 `std::array<float, 8>`，长度由类型
系统保证。

## 发布频率：随意，但不要为了「保活」而空转

桥接节点做**零阶保持** —— 只记住最新一条，由它自己的 20 Hz 定时器发往 MCU。
上层用 5 Hz 还是 100 Hz 都可以，线上始终是 MCU 期望的 20 Hz。

**但停发是有含义的。** 超过 `cmd_timeout_ms`（默认 150 ms）没有新消息，桥接节点
**停止发送命令帧**，让 MCU 看门狗接手。不要为了「保持链路健康」而空转发布 ——
那会绕过两端的看门狗，把「上层挂了」伪装成「一切正常」。

如果你的控制器本来就是低频的，让它保持低频；桥接节点会补齐到 20 Hz。

## 命令合法性：非法命令整条丢弃

含 `NaN` 或 `Inf` 的命令会被**整条丢弃**，下发命令归零，并打 ERROR 日志。

这一条**真的会发生** —— 控制律除零、积分饱和、未初始化内存都会产出 NaN。它们打包
进命令帧后 CRC 完全正确，MCU 会照单执行。把 NaN 当电机指令发下去比长度错危险
得多，所以这里拦掉。

被丢弃时**不刷新新鲜度时间戳**：这条消息不是一条有效命令，不该让链路看起来健康。
于是行为是「立即降为零输出，并在 `cmd_timeout_ms` 用尽后停发」，MCU 看门狗照常
接手。若刷新时间戳，持续发非法命令的上游就会被无限期掩盖成「一切正常」。

---

# 上层必须处理的两件事

## 1. 安全状态 → 重置控制器状态

`~/link_status` 的 `safe_state` 为 true 时，**MCU 的 8 路输出已经全部归零**。

此时控制器里累积的积分项、滤波器状态**不再对应现实** —— 它们是按「输出在执行」
的假设累积的，而输出已经是零。不重置会导致命令恢复瞬间输出跳变。

```cpp
void MyNode::onLinkStatus(const McuLinkStatus::SharedPtr s)
{
  if (s->safe_state && !was_safe_) {
    integral_ = 0.0;        // 重置积分项
    filter_.reset();        // 重置滤波器
    RCLCPP_WARN(get_logger(), "MCU 安全态，控制器状态已重置");
  }
  was_safe_ = s->safe_state;
}
```

注意用**边沿检测**（`&& !was_safe_`）而不是电平判断。安全态会持续存在，按电平写
会每帧都重置一次。

## 2. 未握手 → 命令会被拒收

`not_handshaked` 为 true 时 MCU **拒收**命令帧。这不是错误状态，而是正常的启动
阶段或安全态重整阶段（安全态满 2 s 后 MCU 退回未握手，重新广播 HELLO）。

上层不需要做任何事 —— 桥接节点会自动应答。但如果你在等「命令生效」，要知道这个
阶段发出去的命令不会被执行。

**`version_mismatch` 为 true 是另一回事**：协议版本不匹配，MCU 永不进入正常模式，
重连也不会修好。这是配置错误，需要更新两端之一的固件/代码。

---

# 状态字段速查

`~/link_status` 里的字段分两类，混淆它们会误判故障方向：

**MCU 报告的（来自状态字节）**

| 字段 | 含义 | 上层动作 |
|---|---|---|
| `safe_state` | 输出**已全部归零**（≥500 ms 无命令） | 重置积分项与滤波器 |
| `cmd_stale` | 200~500 ms 无命令，**仍在执行最新命令** | 无需动作，这是早期信号 |
| `not_handshaked` | 命令会被拒收 | 等待，节点会自动握手 |
| `version_mismatch` | 永不进入正常模式 | 人工介入，更新固件或代码 |

`safe_state` 与 `cmd_stale` **互斥** —— 安全态取代过期态（MCU 会清 bit1 置 bit0）。
所以不要写成 `if (stale) ... else if (safe)`。

**本端观测的**

| 字段 | 含义 |
|---|---|
| `serial_open` | 串口是否打开（USB 拔出时 false） |
| `handshaked` | 本端认定的握手完成 |
| `downlink_healthy` | `last_cmd_seq` 是否在推进 —— **下行通道是否真的通** |
| `commands_flowing` | 上游 `~/cmd` 是否在超时内有新消息 |

`downlink_healthy` 判据是 **MCU 的回声**，不是「本端发了多少」。写进内核缓冲区
不等于 MCU 收到，更不等于 CRC 通过。上行正常而 `downlink_healthy` 为 false 是
可能的 —— 那说明下行单向故障（例如 TX 线断）。

---

# 遥测数据是原始浮点数组

`McuImuRaw.data` 是 `float32[16]`，当前顺序和单位为：

```text
[0] ax_g       [1] ay_g       [2] az_g
[3] gx_deg_s   [4] gy_deg_s   [5] gz_deg_s
[6] mx_uT      [7] my_uT      [8] mz_uT
[9] roll_deg  [10] pitch_deg [11] yaw_deg
[12] quaternion_w [13] quaternion_x [14] quaternion_y [15] quaternion_z
```

IMU 完整帧为 79 B，`len=0x49`，数据区从完整帧偏移 13 开始；四元数 W/X/Y/Z
分别位于字节 61..64、65..68、69..72、73..76，均为小端 `float32`。协议版本为 V3，
HELLO_ACK 必须回复版本 3，`cmd_layout` 保持不变。

`McuEnv` 的两个字段依次为 `temperature`（°C）和 `pressure`（Pa）。
对应环境数据顺序为 `[0] temperature_degC`、`[1] pressure_Pa`。
消息仍刻意不映射成 `sensor_msgs/Imu`，因为坐标系、时间语义和姿态表示仍属于上层约定；
需要标准消息时，在上层转换节点中完成映射。

`header.stamp` 是**本机收到该帧的时刻**，不是 MCU 时间。两个时间基准不能混：
`mcu_time_ms` 是 MCU 复位后的毫秒数，49.7 天回绕，且与本机时钟无关联。做时间
对齐用 `header.stamp`；判断 MCU 是否复位用 `mcu_time_ms` 是否跳回小值。

`seq` 是**按帧型独立**的序号，IMU 和 ENV 各有自己的计数，互不相干，回绕归零。

---

# 接入范式

## 独立节点（推荐）

桥接节点单独跑，上层节点通过话题通信。进程隔离，上层崩溃不影响串口链路 —— 而且
上层崩溃后命令自然停发，MCU 看门狗会正确地把输出归零。

```bash
ros2 launch mcu_protocol bridge.launch.py
ros2 run my_package my_controller
```

## 组件式加载（当前不支持）

`bridge_node.cpp` 编译成 `libmcu_bridge_component.so`，但**没有注册
`RCLCPP_COMPONENTS_REGISTER_NODE`**，所以不能用 `ros2 component load` 或
`ComposableNodeContainer` 加载。`CMakeLists.txt` 里的 `find_package(rclcpp_components)`
是为将来预留的，不代表当前可用。

若要支持，需在 `bridge_node.cpp` 末尾加注册宏并在 CMake 里调用
`rclcpp_components_register_node`。

## 直接链接组件库

需要在同一进程内构造 `BridgeNode` 时：

```cmake
find_package(mcu_protocol REQUIRED)
target_link_libraries(my_node mcu_bridge_component)
```

```cpp
#include "mcu_protocol/bridge_node.hpp"
auto bridge = std::make_shared<mcu_protocol::BridgeNode>();
```

**必须用 `MultiThreadedExecutor`。** 单线程执行器下，某个回调偶发变慢会拖延
20 Hz 命令定时器 —— 迟到 200 ms 就会让 MCU 置过期位。串口读在节点自己的线程里，
不受执行器影响。节点未使用自定义 callback group，所以回调默认互斥。

---

# 线程模型（改动本节点时需要知道）

```
读线程          poll/read → Parser::feed → LinkState → 发布遥测
命令定时器      20 Hz，取最新 ~/cmd → 编码 → 写串口
状态定时器      1 Hz，兜底发布 link_status
```

- `state_mutex_` 保护链路状态与计数器（读线程写，两个定时器读）
- `cmd_mutex_` 保护最新命令（订阅回调写，命令定时器读）
- **串口写只发生在命令定时器线程**，握手应答也在那里发。读线程**不写串口**，
  所以 `SerialPort::writeAll` 不需要加锁。读线程发现该握手了只置一个原子标志。

读串口用独立线程而不是 ROS 定时器轮询：20 Hz 双帧下每秒约 1720 字节，定时器轮询
要么周期短到浪费 CPU，要么攒出延迟。
