# mcu_dashboard

独立的 MCU 遥测与 8 路电机测试窗口。它只通过 ROS 2 话题连接已经运行的
`mcu_bridge`，不打开串口，也不参与协议解析。桥接节点可以继续由另一个终端或
launch 进程运行；窗口关闭不会影响桥接节点的串口线程。

## 构建与启动

在工作区根目录执行：

```bash
cd ~/ros2_ws/MCU_PROTOCOL
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select mcu_dashboard
source install/setup.bash
```

确认 `mcu_bridge` 已经运行后，在另一个终端启动窗口：

```bash
ros2 run mcu_dashboard mcu_dashboard
```

也可以使用 launch 文件：

```bash
ros2 launch mcu_dashboard dashboard.launch.py
ros2 launch mcu_dashboard dashboard.launch.py bridge_ns:=/other_bridge
```

`bridge_ns` 默认是 `/mcu_bridge`，必须与正在运行的桥接节点命名空间一致。窗口发现
话题需要一点时间；如果刚启动时显示“等待”，先确认：

```bash
ros2 node list --no-daemon --spin-time 5
ros2 topic list --no-daemon --spin-time 5 -t
```

Python 图形依赖为 `PyQt5` 和 `pyqtgraph`。当前开发机已安装；新机器上可用：

```bash
sudo apt install python3-pyqt5 python3-pyqtgraph
```

## 界面布局

窗口是单屏左右布局：

- 左侧为传感器区域，显示温度、绝对压力、淡水深度、IMU 数值表和 4 组 XYZ 曲线。
- 右侧为电机测试区域，8 路电机从上到下排列，每路包含滑块、三位小数输入框和归零按钮。
- 顶部状态条显示串口、握手、上行、命令流、下行回声和 MCU 安全状态。
- 底部诊断栏显示 MCU 接收帧、CRC、重同步、丢字节和最近命令年龄等累计值。

窗口默认大小为 `1500x920`，最小大小为 `1280x880`。较小屏幕下右侧电机区域可以独立滚动。

## 传感器数据

### IMU

`McuImuRaw.data` 的 12 个 float 按四组显示，界面和当前 MCU 数据契约使用以下顺序：

```text
[0] ax_g       [1] ay_g       [2] az_g
[3] gx_deg_s   [4] gy_deg_s   [5] gz_deg_s
[6] mx_uT      [7] my_uT      [8] mz_uT
[9] roll_deg  [10] pitch_deg [11] yaw_deg
```

界面分别显示加速度（`g`）、角速度（`deg/s`）、磁力计（`uT`）和姿态角（`deg`），表格单元格
和曲线图例使用对应字段名。曲线显示最近一段历史数据；顶部“窗口”可选择 `10 s`、`30 s` 或
`60 s`，垃圾桶按钮清空历史曲线，“暂停曲线”只暂停绘图，不停止 ROS 数据接收。

### 温度、压力和深度

- `McuEnv` 的环境数组顺序为：

  ```text
  [0] temperature_degC
  [1] pressure_Pa
  ```

- 温度显示单位为 `°C`。
- 压力按绝对压力接收，显示单位为 `hPa`；例如 `99600 Pa` 显示为 `996.00 hPa`。
- 深度显示单位为 `m`，按淡水静水压力计算：

  ```text
  depth = (pressure_pa - surface_pressure_pa) / (density * 9.80665)
  ```

- 默认水面绝压为 `1013.25 hPa`，淡水密度为 `997.0 kg/m³`。
- 将设备放在水面或已知基准位置后，点击深度右侧的勾选按钮，可用当前压力设置水面零点。
- 原始绝对压力始终保留；负深度表示当前压力低于设定的水面基准，不代表协议数据被修改。

## 电机测试

电机面板默认处于“未使能，不发布命令”状态，打开窗口不会自动向 MCU 下发任何命令。

### 单路调节

- 每路输入范围为 `-1.000` 到 `+1.000`，滑块和数值框双向同步。
- 单路归零按钮只将该路界面值设为 `0.000`。
- “载入”可以快速载入“全部为零”“正向阶梯”“正负交替”三种预设。
- 当前界面按 8 路原始数组发送，不假定电机名称、方向或物理单位。

### 使能、停止和回声

1. 确认 MCU 已握手、串口已打开，并确认输出引脚处于可测量状态。
2. 调整 8 路数值，先建议使用全零或很小的单路值。
3. 勾选“启用 20 Hz 命令发布”。窗口随后以 20 Hz 往 `/mcu_bridge/cmd` 发布最新的 8 路数组。
4. 观察顶部“回声”状态和底部 MCU 诊断。`回声 · 推进` 与 `last_cmd_seq` 推进表示 MCU 已收到
   且 CRC 校验通过；本地“已发布”计数不能代替 MCU 回声。
5. 停止测试时点击“停止并归零”或取消使能。窗口会先发送 4 帧全零，然后停止发布。

GUI 发布器不会额外对值做二次限幅，只校验数组长度和有限浮点值；桥接节点仍会拒绝 `NaN/Inf`。
关闭窗口时会尽力补发 3 帧全零，但进程异常退出时不要依赖它，桥接节点的 `150 ms` 上游超时和
MCU 自身看门狗才是最终的归零路径。

## 真机测试建议

本窗口不拥有串口，真机测试前先单独启动桥接节点，并确认没有第二个程序占用 `/dev/ttyUSB0`。
可以先只测零命令和反馈链路：

```bash
ros2 topic echo /mcu_bridge/link_status --once \
  --qos-durability transient_local --qos-reliability reliable
ros2 topic echo /mcu_bridge/diagnostics --once
```

首次测试建议使用示波器或万用表观察 MCU 引脚，逐路改变一个小幅值，再点击停止并确认输出回零。
不要把“界面滑块改变”当成 MCU 已执行；必须同时看到 `last_cmd_seq` 持续推进、`rx_frames_ok` 增长、
`rx_crc_err` 为零。

## ROS 话题与 QoS

窗口订阅和发布的默认话题如下：

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `/mcu_bridge/imu_raw` | `McuImuRaw` | SensorData / best effort |
| 订阅 | `/mcu_bridge/env` | `McuEnv` | SensorData / best effort |
| 订阅 | `/mcu_bridge/link_status` | `McuLinkStatus` | reliable + transient local |
| 订阅 | `/mcu_bridge/diagnostics` | `McuDiagnostics` | reliable |
| 发布 | `/mcu_bridge/cmd` | `McuCommand` | reliable |

如果界面显示等待而 `ros2 topic echo` 能看到数据，优先检查 `bridge_ns` 和 QoS；传感器话题用默认
reliable 订阅会与桥接端 best-effort 发布不匹配。
