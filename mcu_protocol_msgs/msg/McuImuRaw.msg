# MCU 上行 IMU 帧（类型 0x01，20 Hz，整帧 63 B）。
#
# 12 个 float 的排列顺序与单位约定尚未确定（契约 §6 待定项），所以这里保持原始
# 数组，不映射 sensor_msgs/Imu。猜错字段映射会让下游拿到「看起来对」的错数据 ——
# 那比拿不到数据更难查。语义定下来后再加标准消息，本消息保持不变。

std_msgs/Header header      # stamp 为本机收到该帧的时刻，非 MCU 时间

uint16 seq                  # MCU 侧该帧型的独立序号，回绕归零（各帧型互不相干）
uint32 mcu_time_ms          # MCU 复位后的 HAL_GetTick() 毫秒；49.7 天回绕且不补偿
uint16 last_cmd_seq         # MCU 最近收到的 CRC 正确命令帧序号
uint8 status                # 状态字节，位含义解码见 McuLinkStatus

float32[12] data
