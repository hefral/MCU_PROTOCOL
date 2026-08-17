#include "mcu_protocol/bridge_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

namespace mcu_protocol
{

using namespace std::chrono_literals;  // NOLINT

BridgeNode::BridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("mcu_bridge", options)
{
  device_ = declare_parameter<std::string>("device", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", static_cast<int>(kBaudRate));
  cmd_timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", 150);
  command_rate_hz_ = declare_parameter<int>("command_rate_hz", 20);
  publish_diagnostics_ = declare_parameter<bool>("publish_diagnostics", true);

  if (command_rate_hz_ <= 0 || command_rate_hz_ > 200) {
    RCLCPP_WARN(
      get_logger(), "command_rate_hz=%d 不合理，改用 20（MCU 期望 20 Hz）", command_rate_hz_);
    command_rate_hz_ = 20;
  }
  if (cmd_timeout_ms_ < 3 * kCommandPeriodMs) {
    // 阈值小于三个命令周期时，正常的调度抖动就会被判成上游断流，导致命令时断
    // 时续 —— MCU 那边看到的是反复进出降级态。
    RCLCPP_WARN(
      get_logger(), "cmd_timeout_ms=%d 小于三个命令周期（%d ms），调度抖动会被误判为上游断流",
      cmd_timeout_ms_, 3 * kCommandPeriodMs);
  }

  // 遥测用 SensorDataQoS：best_effort + 深度 5。20 Hz 的传感器数据重传旧帧没有
  // 价值，宁可丢也不要堆积延迟。
  pub_imu_ = create_publisher<mcu_protocol_msgs::msg::McuImuRaw>(
    "~/imu_raw", rclcpp::SensorDataQoS());
  pub_env_ = create_publisher<mcu_protocol_msgs::msg::McuEnv>(
    "~/env", rclcpp::SensorDataQoS());
  if (publish_diagnostics_) {
    // 诊断是 1 Hz 的低频关键信息，用可靠传输，丢一帧就少一秒的链路视野。
    pub_diag_ = create_publisher<mcu_protocol_msgs::msg::McuDiagnostics>("~/diagnostics", 10);
  }

  // link_status 用 transient_local(latched)：晚启动的节点一订阅就拿到当前状态，
  // 不必等下一次变化 —— 对「现在是不是安全态」这种问题，等一秒是不可接受的。
  {
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    pub_status_ = create_publisher<mcu_protocol_msgs::msg::McuLinkStatus>("~/link_status", qos);
  }

  // 命令入口。深度 1 + reliable：只有最新一条有意义（零阶保持），堆积旧命令
  // 反而危险。其他节点往这里发就行，不需要知道串口的存在。
  sub_cmd_ = create_subscription<mcu_protocol_msgs::msg::McuCommand>(
    "~/cmd", 1, std::bind(&BridgeNode::onCommand, this, std::placeholders::_1));

  const auto period = std::chrono::milliseconds(1000 / command_rate_hz_);
  cmd_timer_ = create_wall_timer(period, std::bind(&BridgeNode::onCommandTimer, this));
  status_timer_ = create_wall_timer(1s, std::bind(&BridgeNode::onStatusTimer, this));

  running_ = true;
  read_thread_ = std::thread(&BridgeNode::readLoop, this);

  RCLCPP_INFO(
    get_logger(), "mcu_bridge 启动：device=%s baud=%d 命令 %d Hz 超时 %d ms",
    device_.c_str(), baud_, command_rate_hz_, cmd_timeout_ms_);
  // 首次发布一次状态，让订阅者立刻有个初始值（latched）。
  publishLinkStatus();
}

BridgeNode::~BridgeNode()
{
  running_ = false;
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  port_.close();
}

int64_t BridgeNode::nowMs() const
{
  // 用 steady_clock：链路超时判定不能受系统时钟跳变（NTP 校时、手动改表）影响。
  // 消息 header.stamp 另用 now()，那是给下游做时间对齐用的。
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ---------------------------------------------------------------- 读线程

void BridgeNode::readLoop()
{
  uint8_t buf[512];
  bool warned_open_fail = false;

  while (running_) {
    if (!port_.isOpen()) {
      std::string err;
      if (!port_.open(device_, static_cast<uint32_t>(baud_), err)) {
        // 打开失败会持续存在（设备没插、权限不足），每次都打一条会刷爆日志。
        // 只在状态变化时打一次，之后靠 link_status 的 serial_open=false 反映。
        if (!warned_open_fail) {
          RCLCPP_ERROR(get_logger(), "%s；1 s 后重试", err.c_str());
          warned_open_fail = true;
          publishLinkStatus();
        }
        std::this_thread::sleep_for(1s);
        continue;
      }
      warned_open_fail = false;
      // 重连后必须清解析器：MCU 掉电瞬间可能在半帧处截断，残留的半帧会和新
      // 字节拼成一帧跨两次上电的乱码。
      parser_.reset();
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        link_.onReconnect(nowMs());
      }
      want_handshake_ = false;
      mcu_version_ = 0;
      RCLCPP_INFO(get_logger(), "串口已打开：%s @ %d", device_.c_str(), baud_);
      publishLinkStatus();
    }

    const ssize_t n = port_.read(buf, sizeof(buf), 50);
    const int64_t now = nowMs();

    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "串口读失败（设备可能已拔出），关闭并重连");
      port_.close();
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        link_.onReconnect(now);
      }
      publishLinkStatus();
      std::this_thread::sleep_for(500ms);
      continue;
    }

    auto cb = [this, now](const Frame & f) {onFrame(f, now);};

    if (n > 0) {
      parser_.feed(buf, static_cast<size_t>(n), now, cb);
    }
    // 无论有没有收到字节都要 tick：空闲超时的存在意义正是「对端停发时把被伪帧头
    // 挡住的真帧救出来」，只在有字节时调等于没实现它。
    parser_.tick(now, cb);

    // 上行中断检测同理，必须在没有帧到来时也推进。
    LinkEvents ev;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      ev = link_.onTick(now);
      link_.counters().crc_errors = parser_.stats().crc_errors;
      link_.counters().resyncs = parser_.stats().resyncs;
    }
    if (ev.uplink_lost) {
      RCLCPP_WARN(
        get_logger(), "上行中断：超过 %ld ms 未收到任何有效帧（MCU 复位、掉电或线路故障）",
        static_cast<long>(LinkState::kUplinkTimeoutMs));
      publishLinkStatus();
    }
  }
}

void BridgeNode::onFrame(const Frame & f, int64_t now_ms)
{
  if (!isUplink(f.type)) {
    // 上行链路上出现下行帧型：接线接成了回环，或者两个节点在抢同一个串口。
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "收到下行帧型 0x%02X：串口可能接成了回环，或有另一个进程在写这个口", f.type);
    return;
  }

  LinkEvents ev;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    ev = link_.onUplinkFrame(f, now_ms);
    link_.counters().crc_errors = parser_.stats().crc_errors;
    link_.counters().resyncs = parser_.stats().resyncs;
  }

  switch (static_cast<FrameType>(f.type)) {
    case FrameType::kImu: {
      const auto vals = decodeImu(f);
      if (!vals) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "IMU 帧载荷长度异常（%zu B，期望 %zu B），已丢弃",
          f.payload_len, kImuFloats * sizeof(float));
        break;
      }
      mcu_protocol_msgs::msg::McuImuRaw msg;
      fillHeader(msg, f);
      std::copy(vals->begin(), vals->end(), msg.data.begin());
      pub_imu_->publish(msg);
      break;
    }

    case FrameType::kEnv: {
      const auto vals = decodeEnv(f);
      if (!vals) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "温压帧载荷长度异常（%zu B），已丢弃", f.payload_len);
        break;
      }
      mcu_protocol_msgs::msg::McuEnv msg;
      fillHeader(msg, f);
      msg.temperature = (*vals)[0];
      msg.pressure = (*vals)[1];
      pub_env_->publish(msg);
      break;
    }

    case FrameType::kHello: {
      const auto h = decodeHello(f);
      if (!h) {
        RCLCPP_WARN(get_logger(), "HELLO 帧载荷长度异常（%zu B），已丢弃", f.payload_len);
        break;
      }
      mcu_version_ = h->version;
      if (h->version != kProtocolVersion) {
        // 不应答。静默按不兼容格式跑是最坏结果，宁可停在这里让人看到。
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "协议版本不匹配：MCU 声明 V%u，本端实现 V%u。不发送 HELLO_ACK —— "
          "这是配置错误，重连不会修好，需要更新两端之一的固件/代码",
          h->version, kProtocolVersion);
        break;
      }
      // MCU 还在广播 HELLO 说明它在等应答。应答由命令定时器线程发出（本线程
      // 不写串口），这里只置标志。
      want_handshake_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "收到 HELLO：V%u IMU %u float / 温压 %u float，上行 %u Hz 期望下行 %u Hz",
        h->version, h->imu_floats, h->env_floats, h->uplink_hz, h->downlink_hz);
      break;
    }

    case FrameType::kDiagnostics: {
      const auto d = decodeDiagnostics(f);
      if (!d) {
        RCLCPP_WARN(get_logger(), "诊断帧载荷长度异常（%zu B），已丢弃", f.payload_len);
        break;
      }
      if (pub_diag_) {
        mcu_protocol_msgs::msg::McuDiagnostics msg;
        fillHeader(msg, f);
        msg.rx_frames_ok = d->rx_frames_ok;
        msg.rx_crc_err = d->rx_crc_err;
        msg.rx_resync = d->rx_resync;
        msg.rx_overrun = d->rx_overrun;
        msg.rx_dma_lost = d->rx_dma_lost;
        msg.tx_dropped = d->tx_dropped;
        msg.last_cmd_age_ms = d->last_cmd_age_ms;
        pub_diag_->publish(msg);
      }
      // 这两项持续增长意味着 MCU 侧真的丢了字节，不只是某帧格式不对。
      if (d->rx_overrun > 0 || d->rx_dma_lost > 0) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "MCU 侧丢字节：rx_overrun=%u rx_dma_lost=%u（主循环被拖延或环形缓冲追尾）",
          d->rx_overrun, d->rx_dma_lost);
      }
      break;
    }

    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "未知上行帧型 0x%02X，已丢弃", f.type);
      break;
  }

  handleEvents(ev, f);
}

// -------------------------------------------------- 事件日志（边沿触发）

void BridgeNode::handleEvents(const LinkEvents & ev, const Frame & f)
{
  // 边沿触发，不是电平触发。20 Hz 下按电平打日志，安全态期间每秒 40 条，
  // 真正的异常会被自己刷掉。

  if (ev.entered_safe_state) {
    // 这是本节点最重要的一条日志：MCU 的 8 路输出**已经全部归零**。
    RCLCPP_WARN(
      get_logger(),
      "MCU 进入安全状态：8 路输出已全部归零（距上一条合法命令 >= 500 ms）。"
      "MCU 时间 %u ms，最近接受的命令序号 %u。"
      "控制节点应重置积分项与滤波器 —— MCU 输出已归零，控制器里累积的状态不再对应现实",
      f.mcu_time_ms, f.last_cmd_seq);
  }
  if (ev.left_safe_state) {
    RCLCPP_INFO(
      get_logger(), "MCU 退出安全状态，恢复执行命令（MCU 时间 %u ms）", f.mcu_time_ms);
  }

  if (ev.entered_cmd_stale) {
    // 降级态：MCU 仍在执行最新命令，不需要重置控制器。这是下行抖动的早期信号。
    RCLCPP_WARN(
      get_logger(),
      "MCU 命令过期（200~500 ms 无新命令），仍在执行最新命令。"
      "再无命令将于 500 ms 进入安全状态");
  }
  if (ev.left_cmd_stale) {
    RCLCPP_INFO(get_logger(), "MCU 命令过期状态解除");
  }

  if (ev.mcu_restarted) {
    RCLCPP_WARN(
      get_logger(), "MCU 已复位（时间戳从大值跳回 %u ms），序号统计已重置", f.mcu_time_ms);
  }

  if (ev.handshake_completed) {
    RCLCPP_INFO(get_logger(), "握手完成，MCU 开始接受命令帧");
  }
  if (ev.handshake_lost) {
    // MCU 在安全态里待满 2 s 会退回未握手态重新广播 HELLO。本端不重新应答的话，
    // 双方会静默地互相等待。
    RCLCPP_WARN(get_logger(), "MCU 回到未握手状态（复位或安全态满 2 s 重整），等待重新握手");
  }

  if (ev.uplink_restored) {
    RCLCPP_INFO(get_logger(), "上行恢复");
  }

  if (ev.any()) {
    // 状态一变就立刻发一次，下游不必等 1 Hz 兜底。
    publishLinkStatus();
  }
}

// ---------------------------------------------------------------- 命令下行

void BridgeNode::onCommand(const mcu_protocol_msgs::msg::McuCommand::SharedPtr msg)
{
  // --- 命令合法性校验。任何一项不通过：整条丢弃 + 下发命令归零 + 打日志。
  //
  // 按前 N 个填、余下保持旧值是更坏的选择 —— 那等于替上游决定了未指定通道的输出
  // 值，而上游此刻显然处于错误状态。
  //
  // 归零之后**刻意不刷新新鲜度时间戳**：这条消息不是一条有效命令，不该让链路看
  // 起来健康。于是行为是「立即降为零输出，并在 cmd_timeout_ms 用尽后停发」，MCU
  // 看门狗照常接手（200 ms 过期位 / 500 ms 输出归零 / 2 s 重整握手）。若刷新时间
  // 戳，上游持续发非法命令就会被无限期掩盖成「一切正常」。
  const char * reject = nullptr;
  char detail[96] = {};

  if (msg->values.size() != kCommandFloats) {
    // 当前 McuCommand.values 是 float32[8]（定长），生成为 std::array<float, 8>，
    // 因此这条分支实际不可达：长度由类型系统保证，上游写错在编译期就会失败，
    // Python 侧赋值也会先抛断言。保留它是为了在消息定义某天改成 float32[]
    // （变长）时不至于静默地少一层校验。
    reject = "字段长度错误";
    std::snprintf(
      detail, sizeof(detail), "收到 %zu 个 float，协议要求恰好 %zu 个",
      msg->values.size(), kCommandFloats);
  } else {
    for (size_t i = 0; i < msg->values.size(); ++i) {
      if (!std::isfinite(msg->values[i])) {
        // 这一条**真的会发生**：上游控制律除零、积分饱和、未初始化内存都会产出
        // NaN/Inf。它们打包进命令帧后 CRC 完全正确，MCU 会照单执行 —— 把 NaN
        // 当电机指令发下去比长度错危险得多。
        reject = "含非有限数值";
        std::snprintf(
          detail, sizeof(detail), "values[%zu] = %f（NaN 或 Inf）", i,
          static_cast<double>(msg->values[i]));
        break;
      }
    }
  }

  if (reject != nullptr) {
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      std::fill(latest_cmd_.values.begin(), latest_cmd_.values.end(), 0.0F);
    }
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "~/cmd %s：%s。整条命令已丢弃，下发命令已全部归零；"
      "若上游持续无有效命令，%d ms 后停发并交由 MCU 看门狗处理",
      reject, detail, cmd_timeout_ms_);
    return;
  }

  std::lock_guard<std::mutex> lk(cmd_mutex_);
  latest_cmd_ = *msg;
  latest_cmd_ms_ = nowMs();
  have_cmd_ = true;
}

void BridgeNode::onCommandTimer()
{
  if (!port_.isOpen()) {
    return;
  }

  // 握手应答优先。串口写只在本线程发生，所以 writeAll 不需要加锁。
  if (want_handshake_.exchange(false)) {
    const auto frame = encodeHelloAck(ack_seq_++, kProtocolVersion, 1);
    if (!port_.writeAll(frame.data(), frame.size())) {
      RCLCPP_ERROR(get_logger(), "发送 HELLO_ACK 失败");
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000, "已发送 HELLO_ACK（V%u）", kProtocolVersion);
  }

  // 取最新命令。零阶保持：上游发布频率自由，由本定时器按固定周期送出。
  std::array<float, kCommandFloats> values{};
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    const int64_t age = nowMs() - latest_cmd_ms_;
    fresh = have_cmd_ && age <= cmd_timeout_ms_;
    if (commands_flowing_ != fresh) {
      commands_flowing_ = fresh;
      if (!fresh) {
        // 停发的后果必须说清楚，否则现场只会看到「MCU 突然不动了」。
        RCLCPP_WARN(
          get_logger(),
          "上游 %ld ms 未发布 ~/cmd（阈值 %d ms），**停止发送命令帧**。"
          "MCU 将按其看门狗降级：200 ms 置过期位、500 ms 输出归零、2 s 后重整握手。"
          "这是刻意的 —— 继续重发旧命令会让 MCU 以为链路健康",
          static_cast<long>(age), cmd_timeout_ms_);
      } else {
        RCLCPP_INFO(get_logger(), "上游命令恢复，重新开始发送命令帧");
      }
    }
    if (fresh) {
      std::copy(latest_cmd_.values.begin(), latest_cmd_.values.end(), values.begin());
    }
  }

  if (!fresh) {
    return;  // 停发。交给 MCU 看门狗。
  }

  const auto frame = encodeCommand(cmd_seq_, values);
  if (!port_.writeAll(frame.data(), frame.size())) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "发送命令帧失败（设备可能已拔出）");
    return;
  }
  ++cmd_seq_;

  std::lock_guard<std::mutex> lk(state_mutex_);
  ++link_.counters().commands_sent;
}

void BridgeNode::onStatusTimer()
{
  // 下行健康度：看 MCU 回报的 last_cmd_seq 是否在推进，而不是「我发出去了多少」。
  // 写进内核缓冲区不等于 MCU 收到了，更不等于 CRC 校验通过了 —— 只有 MCU 的回声
  // 能证明下行真的通了。
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    const uint16_t echo = link_.lastCmdSeqEcho();
    const bool advancing = (echo != last_echo_seen_);
    last_echo_seen_ = echo;
    downlink_healthy_ = advancing && link_.handshaked();
  }
  publishLinkStatus();  // 1 Hz 兜底
}

// ---------------------------------------------------------------- 状态发布

void BridgeNode::publishLinkStatus()
{
  mcu_protocol_msgs::msg::McuLinkStatus msg;
  msg.header.stamp = now();
  msg.header.frame_id = device_;

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    msg.status = link_.status();
    msg.safe_state = link_.safeState();
    msg.cmd_stale = link_.cmdStale();
    msg.not_handshaked = link_.notHandshaked();
    msg.version_mismatch = link_.versionMismatch();
    msg.handshaked = link_.handshaked();

    const auto & c = link_.counters();
    msg.uplink_gaps = static_cast<uint32_t>(c.uplink_gaps);
    msg.crc_errors = static_cast<uint32_t>(c.crc_errors);
    msg.resyncs = static_cast<uint32_t>(c.resyncs);
    msg.reconnects = static_cast<uint32_t>(c.reconnects);
  }

  msg.serial_open = port_.isOpen();
  msg.downlink_healthy = downlink_healthy_;
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    msg.commands_flowing = commands_flowing_;
  }

  pub_status_->publish(msg);
}

template<typename MsgT>
void BridgeNode::fillHeader(MsgT & msg, const Frame & f) const
{
  msg.header.stamp = now();
  msg.header.frame_id = "mcu";
  msg.seq = f.seq;
  msg.mcu_time_ms = f.mcu_time_ms;
  msg.last_cmd_seq = f.last_cmd_seq;
  msg.status = f.status;
}

}  // namespace mcu_protocol
