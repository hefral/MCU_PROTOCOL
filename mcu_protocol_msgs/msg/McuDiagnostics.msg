# MCU 上行诊断帧（类型 0x04，1 Hz，整帧 43 B）。链路质量的唯一窗口。
#
# 七个字段全是 MCU 侧的绝对累计计数（除 last_cmd_age_ms），不是速率。判断链路
# 是否在恶化要看增量，不是看非零 —— 非零只说明那条路径曾经执行过。

std_msgs/Header header

uint16 seq
uint32 mcu_time_ms
uint16 last_cmd_seq
uint8 status

uint32 rx_frames_ok         # CRC 通过的下行帧数
uint32 rx_crc_err           # CRC 失败帧数：你发的帧有损坏，或 CRC 算错
uint32 rx_resync            # 重同步次数：字节流里有垃圾或伪帧头（少量正常）
uint32 rx_overrun           # UART ORE/NE/FE：MCU 主循环被拖延
uint32 rx_dma_lost          # 环形缓冲被 DMA 追尾而丢弃的字节数
uint32 tx_dropped           # MCU 帧池满而丢弃的遥测帧数
uint32 last_cmd_age_ms      # 距上一条合法命令的毫秒数；首条命令前等于 MCU 运行时长

# rx_overrun 或 rx_dma_lost 持续增长是需要立刻处理的信号：MCU 侧真的丢了字节，
# 不只是某帧格式不对。
#
# rx_crc_err 增长而 rx_frames_ok 恒为 0 ⇒ 本端 CRC 实现错了；
# 两者同时增长 ⇒ 偶发干扰或发送侧有间歇性问题。
