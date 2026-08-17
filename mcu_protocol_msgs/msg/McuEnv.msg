# MCU 上行温压帧（类型 0x02，20 Hz，整帧 23 B）。
#
# 两个 float 的顺序由 MCU 在 HELLO 帧里声明（env_floats=2），当前固定为
# 温度、压力。单位约定属于契约 §6 待定项，下面标注的是 MCU 端现有实现的取值
# 量级（参考帧解出 4.85 与 401325.0），正式单位定下来前不要据此换算深度。

std_msgs/Header header      # stamp 为本机收到该帧的时刻，非 MCU 时间

uint16 seq                  # 独立于 IMU 的序号，回绕归零
uint32 mcu_time_ms
uint16 last_cmd_seq
uint8 status

float32 temperature         # 摄氏度
float32 pressure            # 帕斯卡
