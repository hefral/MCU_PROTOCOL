# MCU 上行温压帧（类型 0x02，20 Hz，整帧 23 B）。
#
# 两个 float 的顺序由 MCU 在 HELLO 帧里声明（env_floats=2），当前固定为温度、压力。
# 单位为摄氏度和帕斯卡；上位机界面可将压力显示为 hPa，并据绝对压力计算淡水深度。

std_msgs/Header header      # stamp 为本机收到该帧的时刻，非 MCU 时间

uint16 seq                  # 独立于 IMU 的序号，回绕归零
uint32 mcu_time_ms
uint16 last_cmd_seq
uint8 status

float32 temperature         # 摄氏度
float32 pressure            # 帕斯卡
