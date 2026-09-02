// IMU telemetry validity gate and causal filter. This layer deliberately has no
// ROS dependency so the safety behavior can be unit-tested without a running node.

#ifndef MCU_PROTOCOL__IMU_FILTER_HPP_
#define MCU_PROTOCOL__IMU_FILTER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

#include "mcu_protocol/protocol.hpp"

namespace mcu_protocol
{

enum class ImuRejectReason
{
  kNone,
  kNonFinite,
  kQuaternionNorm,
  kOrientationRate,
};

const char * imuRejectReasonText(ImuRejectReason reason) noexcept;

struct ImuFilterConfig
{
  double accel_cutoff_hz = 5.0;
  double gyro_cutoff_hz = 5.0;
  double mag_cutoff_hz = 2.0;
  double orientation_cutoff_hz = 5.0;
  double quaternion_norm_min = 0.8;
  double quaternion_norm_max = 1.2;
  double max_orientation_rate_deg_s = 720.0;
  double nominal_period_s = 0.05;
};

struct ImuFilterResult
{
  bool accepted = false;
  ImuRejectReason reason = ImuRejectReason::kNone;
  std::array<float, kImuFloats> values{};
};

class ImuFilter
{
public:
  explicit ImuFilter(const ImuFilterConfig & config = ImuFilterConfig());

  void setConfig(const ImuFilterConfig & config);
  void reset() noexcept;

  ImuFilterResult process(
    const std::array<float, kImuFloats> & input, uint32_t mcu_time_ms);

private:
  using Quaternion = std::array<double, 4>;

  double samplePeriod(uint32_t mcu_time_ms) noexcept;
  static double lowPassAlpha(double cutoff_hz, double dt_s) noexcept;
  static Quaternion normalized(const Quaternion & q, double norm) noexcept;
  static double dot(const Quaternion & a, const Quaternion & b) noexcept;
  static Quaternion negated(const Quaternion & q) noexcept;
  static Quaternion blend(const Quaternion & from, const Quaternion & to, double alpha) noexcept;
  static std::array<float, 3> eulerDegrees(const Quaternion & q) noexcept;
  double median(size_t channel) const noexcept;

  ImuFilterConfig config_;
  std::array<std::array<double, 9>, 3> scalar_history_{};
  size_t history_count_ = 0;
  size_t history_next_ = 0;
  std::array<double, 9> filtered_scalars_{};
  bool have_filtered_scalars_ = false;

  Quaternion filtered_quaternion_{{1.0, 0.0, 0.0, 0.0}};
  Quaternion previous_input_quaternion_{{1.0, 0.0, 0.0, 0.0}};
  bool have_quaternion_ = false;

  uint32_t previous_time_ms_ = 0;
  bool have_time_ = false;
};

}  // namespace mcu_protocol

#endif  // MCU_PROTOCOL__IMU_FILTER_HPP_
