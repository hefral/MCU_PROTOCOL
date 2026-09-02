#include "mcu_protocol/imu_filter.hpp"

#include <algorithm>
#include <cmath>

namespace mcu_protocol
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansToDegrees = 180.0 / kPi;

double clamp(double value, double low, double high) noexcept
{
  return std::max(low, std::min(value, high));
}

}  // namespace

const char * imuRejectReasonText(ImuRejectReason reason) noexcept
{
  switch (reason) {
    case ImuRejectReason::kNone:
      return "none";
    case ImuRejectReason::kNonFinite:
      return "NaN/Inf";
    case ImuRejectReason::kQuaternionNorm:
      return "quaternion norm";
    case ImuRejectReason::kOrientationRate:
      return "orientation jump";
    default:
      return "unknown";
  }
}

ImuFilter::ImuFilter(const ImuFilterConfig & config)
: config_(config)
{
}

void ImuFilter::setConfig(const ImuFilterConfig & config)
{
  config_ = config;
  reset();
}

void ImuFilter::reset() noexcept
{
  scalar_history_ = {};
  history_count_ = 0;
  history_next_ = 0;
  filtered_scalars_ = {};
  have_filtered_scalars_ = false;
  filtered_quaternion_ = {{1.0, 0.0, 0.0, 0.0}};
  previous_input_quaternion_ = {{1.0, 0.0, 0.0, 0.0}};
  have_quaternion_ = false;
  previous_time_ms_ = 0;
  have_time_ = false;
}

double ImuFilter::samplePeriod(uint32_t mcu_time_ms) noexcept
{
  double dt_s = config_.nominal_period_s;
  if (have_time_) {
    const uint32_t elapsed_ms = mcu_time_ms - previous_time_ms_;
    if (elapsed_ms > 0U) {
      dt_s = static_cast<double>(elapsed_ms) / 1000.0;
    }
  }
  return clamp(dt_s, 0.001, 1.0);
}

double ImuFilter::lowPassAlpha(double cutoff_hz, double dt_s) noexcept
{
  if (cutoff_hz <= 0.0) {
    return 1.0;
  }
  const double rc_s = 1.0 / (2.0 * kPi * cutoff_hz);
  return dt_s / (rc_s + dt_s);
}

ImuFilter::Quaternion ImuFilter::normalized(const Quaternion & q, double norm) noexcept
{
  return {{q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm}};
}

double ImuFilter::dot(const Quaternion & a, const Quaternion & b) noexcept
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

ImuFilter::Quaternion ImuFilter::negated(const Quaternion & q) noexcept
{
  return {{-q[0], -q[1], -q[2], -q[3]}};
}

ImuFilter::Quaternion ImuFilter::blend(
  const Quaternion & from, const Quaternion & to, double alpha) noexcept
{
  Quaternion out{{
    from[0] + alpha * (to[0] - from[0]),
    from[1] + alpha * (to[1] - from[1]),
    from[2] + alpha * (to[2] - from[2]),
    from[3] + alpha * (to[3] - from[3]),
  }};
  const double norm = std::sqrt(dot(out, out));
  if (norm <= 0.0 || !std::isfinite(norm)) {
    return from;
  }
  return normalized(out, norm);
}

std::array<float, 3> ImuFilter::eulerDegrees(const Quaternion & q) noexcept
{
  const double w = q[0];
  const double x = q[1];
  const double y = q[2];
  const double z = q[3];

  const double roll = std::atan2(
    2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
  const double pitch = std::asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0));
  const double yaw = std::atan2(
    2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

  return {{
    static_cast<float>(roll * kRadiansToDegrees),
    static_cast<float>(pitch * kRadiansToDegrees),
    static_cast<float>(yaw * kRadiansToDegrees),
  }};
}

double ImuFilter::median(size_t channel) const noexcept
{
  if (history_count_ < 3U) {
    const size_t latest = (history_next_ + 2U) % 3U;
    return scalar_history_[latest][channel];
  }
  std::array<double, 3> values{{
    scalar_history_[0][channel],
    scalar_history_[1][channel],
    scalar_history_[2][channel],
  }};
  std::sort(values.begin(), values.end());
  return values[1];
}

ImuFilterResult ImuFilter::process(
  const std::array<float, kImuFloats> & input, uint32_t mcu_time_ms)
{
  ImuFilterResult result;
  for (const float value : input) {
    if (!std::isfinite(value)) {
      result.reason = ImuRejectReason::kNonFinite;
      return result;
    }
  }

  Quaternion quaternion{{input[12], input[13], input[14], input[15]}};
  const double quaternion_norm = std::sqrt(dot(quaternion, quaternion));
  if (quaternion_norm < config_.quaternion_norm_min ||
    quaternion_norm > config_.quaternion_norm_max)
  {
    result.reason = ImuRejectReason::kQuaternionNorm;
    return result;
  }
  quaternion = normalized(quaternion, quaternion_norm);

  const double dt_s = samplePeriod(mcu_time_ms);
  if (have_quaternion_) {
    double input_dot = dot(previous_input_quaternion_, quaternion);
    if (input_dot < 0.0) {
      quaternion = negated(quaternion);
      input_dot = -input_dot;
    }
    const double angle_deg =
      2.0 * std::acos(clamp(input_dot, -1.0, 1.0)) * kRadiansToDegrees;
    if (config_.max_orientation_rate_deg_s > 0.0 &&
      angle_deg / dt_s > config_.max_orientation_rate_deg_s)
    {
      result.reason = ImuRejectReason::kOrientationRate;
      return result;
    }
  }

  for (size_t channel = 0; channel < 9U; ++channel) {
    scalar_history_[history_next_][channel] = input[channel];
  }
  history_next_ = (history_next_ + 1U) % 3U;
  history_count_ = std::min<size_t>(history_count_ + 1U, 3U);

  for (size_t channel = 0; channel < 9U; ++channel) {
    const double cutoff = channel < 3U ? config_.accel_cutoff_hz :
      channel < 6U ? config_.gyro_cutoff_hz : config_.mag_cutoff_hz;
    const double sample = median(channel);
    if (!have_filtered_scalars_) {
      filtered_scalars_[channel] = sample;
    } else {
      const double alpha = lowPassAlpha(cutoff, dt_s);
      filtered_scalars_[channel] += alpha * (sample - filtered_scalars_[channel]);
    }
    result.values[channel] = static_cast<float>(filtered_scalars_[channel]);
  }
  have_filtered_scalars_ = true;

  if (!have_quaternion_) {
    filtered_quaternion_ = quaternion;
  } else {
    filtered_quaternion_ = blend(
      filtered_quaternion_, quaternion,
      lowPassAlpha(config_.orientation_cutoff_hz, dt_s));
  }
  previous_input_quaternion_ = quaternion;
  have_quaternion_ = true;
  previous_time_ms_ = mcu_time_ms;
  have_time_ = true;

  const auto euler = eulerDegrees(filtered_quaternion_);
  result.values[9] = euler[0];
  result.values[10] = euler[1];
  result.values[11] = euler[2];
  for (size_t index = 0; index < 4U; ++index) {
    result.values[12U + index] = static_cast<float>(filtered_quaternion_[index]);
  }

  result.accepted = true;
  result.reason = ImuRejectReason::kNone;
  return result;
}

}  // namespace mcu_protocol
