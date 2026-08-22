# MCU_PROTOCOL — STM32F103VET6 ↔ ROS 2 串口桥接

把 STM32F103VET6 上的自定义二进制协议接进 ROS 2。上行遥测发布为话题，下行命令
从话题接收，上层节点不需要知道串口的存在。

- **`mcu_protocol_msgs`** — 五个消息定义，无代码依赖
- **`mcu_protocol`** — 桥接节点 `mcu_bridge`，含协议、解析、串口、链路状态四层

上层怎么用（话题、QoS、接入范式、代码示例）见
[`mcu_protocol/README.md`](mcu_protocol/README.md)。本文档讲**怎么跑起来**和
**怎么确认通信正常**。

## 环境

ROS 2 Jazzy。串口需要 `dialout` 组权限：

```bash
sudo usermod -aG dialout $USER     # 之后需重新登录才生效
```

## 构建

```bash
cd ~/ros2_ws/MCU_PROTOCOL
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动

```bash
ros2 launch mcu_protocol bridge.launch.py
ros2 launch mcu_protocol bridge.launch.py device:=/dev/ttyUSB1
ros2 launch mcu_protocol bridge.launch.py log_level:=debug
```

参数在 [`mcu_protocol/config/bridge.yaml`](mcu_protocol/config/bridge.yaml)，
每一项都有注释说明改动后果。命令行传入的 `device` 覆盖文件里的值。

**设备号会变。** 插拔顺序不同，`/dev/ttyUSB0` 可能变成 `ttyUSB1`。稳定做法是用
by-id 路径：

```bash
ls -l /dev/serial/by-id/
ros2 launch mcu_protocol bridge.launch.py \
  device:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

---

# 测试通信是否正常

四个层次，从不接硬件到接硬件。**出问题时按顺序往下走** —— 跳过前面的层次会把
「测试写错了」和「链路真有问题」混在一起，这两件事在现场表现完全一样。

## 层次 1：主机单元测试（不接硬件）

CRC、帧编解码、解析器重同步、链路状态机的纯逻辑测试。

```bash
cd ~/ros2_ws/MCU_PROTOCOL
colcon test --packages-select mcu_protocol
colcon test-result --verbose
```

预期 **45 项全过**。这里失败说明代码逻辑坏了，不必去查线。

## 层次 2：接模拟器（不接硬件）

固件仓库里有一个照契约独立实现的 MCU 模拟器，跑在虚拟串口（pty）上。它的存在
理由是：**验证测试本身是对的**。硬件上一条测试失败时，只有模拟器能区分「固件有
问题」和「测试写错了」。

```bash
# 终端 1：启动模拟器，它会打印 pty 路径
python3 ~/stm32_ws/F103VET6/PROTOCOL/tools/proto_sim.py

# 终端 2：用上面打印的路径启动桥接节点
ros2 launch mcu_protocol bridge.launch.py device:=/dev/pts/N
```

> 模拟器位于**固件仓库**（`stm32_ws/F103VET6/PROTOCOL/tools/`），不在本仓库内。
> 它是照文档独立写的，不是固件的翻译，所以两边同时错的可能性很低。

接模拟器时会出现「未取得串口独占」告警，这是正常的 —— pty 的另一端被模拟器持有。
真实 USB 串口上不应出现这条。

## 层次 3：接真实硬件 — 确认上行

上行走通的判据不是「有话题」，而是**帧率、零丢帧、零校验错**。这三样加上链路状态位
一屏看全：

```bash
# 终端 2（不要和 launch 同一个终端，日志会把重画搅烂）
ros2 run mcu_protocol mcu_monitor
```

它固定 5 Hz 重画，显示实测帧率、序号空洞、状态位的**后果**、诊断计数器的**增量**，
并在超过判停阈值无新帧时明确写「已停止」—— 静止的画面和活着的数据在屏幕上完全
一样，这是最容易自欺的一种失效。细节见
[`mcu_protocol/README.md`](mcu_protocol/README.md#实时查看mcu_monitor)。

要精确的单帧内容或要把数据喂给别的程序时用原生工具：

```bash
ros2 topic hz /mcu_bridge/imu_raw          # 预期 20 Hz
ros2 topic echo /mcu_bridge/link_status --once
```

`link_status` 是 latched 的，一订阅就该立刻拿到值。健康状态：

```yaml
status: 0
safe_state: false
not_handshaked: false      # 握手已完成
version_mismatch: false
serial_open: true
handshaked: true
uplink_gaps: 0             # 上行序号无空洞
crc_errors: 0
resyncs: 0
```

`reconnects` 非零不一定是问题 —— 它是累计值，可能来自本次运行之前。

## 层次 4：接真实硬件 — 确认下行

**这一层最容易自欺。** 「我发出去了 N 帧」证明不了任何事：写进内核缓冲区不等于
MCU 收到了，更不等于 CRC 校验通过了。唯一可信的判据是 **MCU 自己的回声**。

未收到任何 `~/cmd` 时，节点**刻意不发送命令帧**，MCU 因此停在安全态、8 路输出
全零。这是设计行为，日志会解释原因。先发一组命令：

```bash
ros2 topic pub /mcu_bridge/cmd mcu_protocol_msgs/msg/McuCommand \
  "{values: [-1.0, -0.75, -0.5, -0.25, 0.25, 0.5, 0.75, 1.0]}" -r 20
```

然后看 MCU 的回声：

```bash
ros2 topic echo /mcu_bridge/diagnostics --once
```

下行通了的判据：

| 字段 | 期望 | 含义 |
|---|---|---|
| `last_cmd_seq` | 持续推进 | MCU 收到了且 **CRC 通过** |
| `rx_frames_ok` | 按 20/秒增长 | 帧被接受的速率 |
| `last_cmd_age_ms` | 约 50 ms | 一个命令周期 |
| `rx_crc_err` | 0 | 非零说明线路损坏或 CRC 算法不一致 |
| `rx_overrun` | 稳态不增长 | 增长说明 MCU 主循环被拖延 |
| `tx_dropped` | 0 | 非零说明 MCU 帧池满 |

`ros2 topic echo /mcu_bridge/link_status` 里 `downlink_healthy: true` 是同一件事的
布尔化 —— 它的实现就是「`last_cmd_seq` 是否在推进」。

### 上面那组数值不是随便挑的

八个通道互不相同、量级横跨 0.001 ~ 999.875、正负交错。这样一来**字节序反转、
偏移差 4 字节、两个字段调换**，任何一种都会在打印里立刻显形。随机数或全零做不到
这一点：全零经过任何错误变换后仍然是全零，看起来完全正确。

通道 6 的 `0.001` 是精度探针。若它显示成 `0.000`，问题不在数值传输，而在浮点打印
（newlib-nano 默认不含浮点转换，需要 `-u _printf_float` 链接选项）。

## 层次 5：确认安全路径

这是安全相关行为，**必须实测**，不能只看代码。停掉命令发布进程（`Ctrl-C`），
观察日志时序：

```
+0 ms      MCU 命令过期（200~500 ms 无新命令），仍在执行最新命令
+300 ms    MCU 进入安全状态：8 路输出已全部归零
+2000 ms   MCU 回到未握手状态（安全态满 2 s 重整）
           收到 HELLO → 已发送 HELLO_ACK → 握手完成
```

若命令仍未恢复，这个循环会重复 —— 这是正确行为，不是抖动。

**「退出安全状态」和「退回未握手」必须被区分开。** 两者都会让状态字节的 bit0
落下，但后者的输出**仍然是零**。把它报成「恢复执行命令」与事实相反，这是本项目
修过的一个真实缺陷。

---

# 常见故障与判据

## 输出一直是零

先看日志有没有这条：

```
尚未收到任何 ~/cmd，因此**不发送命令帧** —— MCU 会持续处于安全状态
```

有，就是**正常的**：没有上游命令时节点刻意停发，让 MCU 的看门狗跑起来。发一组
命令即可（见层次 4）。

## 帧率正常、序号连续，但载荷全是零

**链路是好的，数据源没接上。** 判据是「其他字段仍在动」：`seq` 推进、`mcu_time_ms`
增长、`crc_errors` 与 `resyncs` 为零，只有 12 个 IMU float 和温压是 0。

这种情况**不要去查线和波特率**。线或波特率坏掉的表现是 CRC 错、重同步增长、
帧率不足 —— 不是干净的零。零是一个**被正确传输的值**。

原因在 MCU 侧：`Proto_Session_SetImu` / `SetEnv` 没被调用，或调用它们的传感器读取
路径还没接上。固件里 `PROTO_SYNTHETIC_TELEMETRY` 为 1 时 `main.c` 会填一组合成值
（`imu[i] = i + (tick%1000)/1000`，温压为 `25+斜坡` 和常数 `101325`）。**若连这组
合成值都读不到，说明板子上烧的不是当前构建的固件** —— 重新烧一次再看。

合成值这样设计是有理由的：整数部分是通道号、小数部分是慢斜坡，一帧就能同时确认
字段顺序和浮点解码。全零做不到 —— 它经过任何错误变换后仍然是全零。

## 全帧 CRC 失败

波特率不一致。协议固定 921600，改 `baud` 必须同时改固件。

## `resyncs` 持续增长

字节流里有非协议数据。检查是否有第二个进程在写同一个串口：

```bash
fuser -v /dev/ttyUSB0
```

少量 resync（个位数，仅启动时）是正常的 —— 那是上电瞬间的残留字节。

## 「未取得串口独占」告警

真实串口上出现这条，说明**别的进程也能同时往这个口写命令帧**。两个进程同时发
命令时，MCU 收到两路交错的序号，表现为「命令时而生效时而不生效」，而
`last_cmd_seq` 会在两个序列间跳，看起来像丢帧。这种故障事后极难定位。

接 pty 模拟器时这条属正常（另一端持有 master fd）。

## `rx_overrun` 增长

MCU 主循环被拖延，UART 数据来不及取走。注意这个计数器**只增不减**，且自 MCU 上电
起累加 —— 节点只在**新增**时告警，历史值只在首帧报一次 INFO。节点启停瞬间增长
是正常的（节点关闭后 MCU 仍在发，无人读取即 ORE）。

## 上行正常但命令不生效

看 `not_handshaked`。握手未完成时 MCU **拒收**命令帧。若 `version_mismatch` 为
true，则是协议版本不匹配 —— 这是配置错误，重连不会修好，需要更新两端之一。

## 设备拔出后重连失败

已修复（`close()` 里补 `TIOCNXCL`）。若仍出现 `EBUSY`，检查是否有残留进程持有该
tty：`fuser -v /dev/ttyUSB0`。

---

# 调试口（USART2）

固件把调试信息输出到 **USART2（PA2/PA3，115200 8N1）**，与协议口物理分离。

**USART1 上除协议帧外不得出现任何字节** —— 多一个 ASCII 字符就会让解析器失步。
所以调试信息不能走协议口，需要第二个 USB-TTL 适配器接 PA2/PA3。

调试口每 500 ms 打印一行，含 8 个命令浮点数、`seq`、`age`：

```
[12345] cmd seq=1212 age=53ms n=617 | 1.500 -2.250 3.125 10.000 -0.500 100.000 0.001 -999.875
```

打印的是**已应用**的输出值。未握手或安全态下即使收到过命令也读全零 —— 同行的
`age=` 和 `seq=` 正是用来区分「没收到」和「收到了但被看门狗归零」。

---

# 设计取舍

三处可能引起疑问的决定，都是刻意的：

**上行不映射成 `sensor_msgs/Imu`。** 契约 §6 的 float 顺序与单位尚未定稿，现在
映射等于把一个待定约定固化进消息类型。保持 `float32[12]` 原始数组，映射留给上层。

**命令超时后停发，不重发旧命令。** 让 MCU 那条已实测的安全路径真正跑起来。继续
重发会让 MCU 以为链路健康，把「上游节点挂了」伪装成「一切正常」—— 这是最坏的
失效模式。

**非法命令整条丢弃并归零，且不刷新新鲜度时间戳。** 按前 N 个填、余下保持旧值等于
替上游决定未指定通道的输出值，而上游此刻显然处于错误状态。不刷新时间戳是为了让
持续发非法命令的上游**不会**被伪装成健康链路。
