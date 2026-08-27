# MCU 上行 IMU 帧（类型 0x01，20 Hz，整帧 63 B）。
#
# 12 个 float 的排列顺序和单位如下。消息仍保持原始数组，不映射 sensor_msgs/Imu，
# 以便协议层不绑定具体的坐标系或姿态表示。
#
# [0] ax_g       [1] ay_g       [2] az_g
# [3] gx_deg_s   [4] gy_deg_s   [5] gz_deg_s
# [6] mx_uT      [7] my_uT      [8] mz_uT
# [9] roll_deg  [10] pitch_deg [11] yaw_deg

std_msgs/Header header      # stamp 为本机收到该帧的时刻，非 MCU 时间

uint16 seq                  # MCU 侧该帧型的独立序号，回绕归零（各帧型互不相干）
uint32 mcu_time_ms          # MCU 复位后的 HAL_GetTick() 毫秒；49.7 天回绕且不补偿
uint16 last_cmd_seq         # MCU 最近收到的 CRC 正确命令帧序号
uint8 status                # 状态字节，位含义解码见 McuLinkStatus

float32[12] data
