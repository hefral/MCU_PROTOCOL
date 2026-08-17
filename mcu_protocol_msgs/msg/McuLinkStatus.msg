# 链路状态汇总：把 MCU 状态字节解码成具体字段，外加本端观测到的链路健康度。
#
# 这个话题用 transient_local(latched) QoS 发布 —— 晚启动的节点一订阅就能立刻拿到
# 当前状态，不必等下一帧。控制类节点应当订阅它，而不是自己去解 status 字节。

std_msgs/Header header

uint8 status                # MCU 状态字节原值（上行帧偏移 12）

# --- status 的位解码。bit0 与 bit1 不会同时为真：安全态取代过期态，因为
#     「已归零」和「仍在执行旧命令」同时上报会让输出状态含义不明。
bool safe_state             # bit0: 命令超时 >= 500 ms，MCU 8 路输出全零
bool cmd_stale              # bit1: 命令过期 200~500 ms，仍在执行最新命令
bool not_handshaked         # bit2: 尚未完成握手，命令会被拒收
bool version_mismatch       # bit3: 协议版本不匹配，MCU 不会进入正常模式

# --- 本端观测
bool serial_open            # 串口是否打开（USB 拔出时为 false）
bool handshaked             # 本端认定的握手状态：MCU 停发 HELLO 且状态字节转 0
bool downlink_healthy       # last_cmd_seq 是否在推进；否则下行通道故障（上行仍可能正常）
bool commands_flowing       # 上游 ~/cmd 是否在 cmd_timeout_ms 内有新消息

uint32 uplink_gaps          # 本端检出的上行序号空洞累计数（按 uint16 取模比较）
uint32 crc_errors           # 本端解析出的 CRC 错误累计数
uint32 resyncs              # 本端重同步累计数
uint32 reconnects           # 串口重连次数

# --- 给控制节点的动作提示
#
# safe_state 转为 true 时，控制算法应重置积分项、滤波器、状态机，防止 MCU 恢复
# 瞬间的输出冲击 —— MCU 的输出已经归零，控制器里累积的状态不再对应现实。
#
# cmd_stale 是调查下行抖动的信号，不需要复位控制器：MCU 仍在执行最新命令。
#
# version_mismatch 是配置错误而非链路故障，重连不会修好它。
