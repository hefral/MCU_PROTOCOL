#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#include "mcu_protocol/imu_filter.hpp"

using namespace mcu_protocol;  // NOLINT

namespace
{

std::array<float, kImuFloats> sample()
{
  return {{
    0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 0.0f,
    40.0f, 5.0f, -10.0f,
    99.0f, 99.0f, 99.0f,
    1.0f, 0.0f, 0.0f, 0.0f,
  }};
}

ImuFilterConfig unfilteredConfig()
{
  ImuFilterConfig config;
  config.accel_cutoff_hz = 0.0;
  config.gyro_cutoff_hz = 0.0;
  config.mag_cutoff_hz = 0.0;
  config.orientation_cutoff_hz = 0.0;
  config.max_orientation_rate_deg_s = 0.0;
  return config;
}

}  // namespace

TEST(ImuFilter, RejectsNonFiniteInput)
{
  ImuFilter filter;
  auto input = sample();
  input[3] = std::numeric_limits<float>::quiet_NaN();

  const auto result = filter.process(input, 1000U);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(ImuRejectReason::kNonFinite, result.reason);
}

TEST(ImuFilter, RejectsInvalidQuaternionNorm)
{
  ImuFilter filter;
  auto input = sample();
  input[12] = 0.0f;

  const auto result = filter.process(input, 1000U);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(ImuRejectReason::kQuaternionNorm, result.reason);
}

TEST(ImuFilter, NormalizesQuaternionAndRecomputesEuler)
{
  ImuFilter filter(unfilteredConfig());
  auto input = sample();
  const float root_half = std::sqrt(0.5f) * 0.9f;
  input[12] = root_half;
  input[15] = root_half;

  const auto result = filter.process(input, 1000U);
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(1.0, std::hypot(result.values[12], result.values[15]), 1e-6);
  EXPECT_NEAR(0.0, result.values[9], 1e-4);
  EXPECT_NEAR(0.0, result.values[10], 1e-4);
  EXPECT_NEAR(90.0, result.values[11], 1e-3);
}

TEST(ImuFilter, ThreePointMedianRemovesSingleScalarSpike)
{
  ImuFilter filter(unfilteredConfig());
  auto input = sample();
  input[2] = 1.0f;
  ASSERT_TRUE(filter.process(input, 1000U).accepted);
  ASSERT_TRUE(filter.process(input, 1050U).accepted);

  input[2] = 0.0f;
  const auto spike = filter.process(input, 1100U);
  ASSERT_TRUE(spike.accepted);
  EXPECT_FLOAT_EQ(1.0f, spike.values[2]);

  input[2] = 1.0f;
  const auto recovered = filter.process(input, 1150U);
  ASSERT_TRUE(recovered.accepted);
  EXPECT_FLOAT_EQ(1.0f, recovered.values[2]);
}

TEST(ImuFilter, MedianAllowsAChangeThatPersists)
{
  ImuFilter filter(unfilteredConfig());
  auto input = sample();
  input[0] = 0.0f;
  filter.process(input, 1000U);
  filter.process(input, 1050U);

  input[0] = 1.0f;
  EXPECT_FLOAT_EQ(0.0f, filter.process(input, 1100U).values[0]);
  EXPECT_FLOAT_EQ(1.0f, filter.process(input, 1150U).values[0]);
}

TEST(ImuFilter, TreatsQuaternionSignAsTheSameOrientation)
{
  ImuFilter filter(unfilteredConfig());
  ASSERT_TRUE(filter.process(sample(), 1000U).accepted);

  auto same = sample();
  same[12] = -1.0f;
  const auto result = filter.process(same, 1050U);
  ASSERT_TRUE(result.accepted);
  EXPECT_NEAR(1.0, result.values[12], 1e-6);
  EXPECT_NEAR(0.0, result.values[9], 1e-6);
}

TEST(ImuFilter, RejectsImplausibleOrientationRate)
{
  auto config = unfilteredConfig();
  config.max_orientation_rate_deg_s = 180.0;
  ImuFilter filter(config);
  ASSERT_TRUE(filter.process(sample(), 1000U).accepted);

  auto jumped = sample();
  jumped[12] = std::sqrt(0.5f);
  jumped[15] = std::sqrt(0.5f);
  const auto result = filter.process(jumped, 1050U);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(ImuRejectReason::kOrientationRate, result.reason);
}

TEST(ImuFilter, RejectedSampleDoesNotPolluteMedianHistory)
{
  ImuFilter filter(unfilteredConfig());
  auto input = sample();
  input[0] = 2.0f;
  filter.process(input, 1000U);
  filter.process(input, 1050U);

  auto rejected = input;
  rejected[0] = 100.0f;
  rejected[12] = 0.0f;
  EXPECT_FALSE(filter.process(rejected, 1100U).accepted);

  const auto result = filter.process(input, 1150U);
  ASSERT_TRUE(result.accepted);
  EXPECT_FLOAT_EQ(2.0f, result.values[0]);
}

TEST(ImuFilter, RateGateCanRecoverAfterRejectedFrames)
{
  auto config = unfilteredConfig();
  config.max_orientation_rate_deg_s = 180.0;
  ImuFilter filter(config);
  ASSERT_TRUE(filter.process(sample(), 1000U).accepted);

  auto turned = sample();
  turned[12] = std::sqrt(0.5f);
  turned[15] = std::sqrt(0.5f);
  EXPECT_FALSE(filter.process(turned, 1050U).accepted);

  // Rejected frames do not advance the reference time. Once enough real time has
  // elapsed, the same orientation is physically plausible and must be accepted.
  const auto recovered = filter.process(turned, 2000U);
  EXPECT_TRUE(recovered.accepted);
}

TEST(ImuFilter, ResetDropsOldScalarAndQuaternionHistory)
{
  ImuFilter filter(unfilteredConfig());
  auto before = sample();
  before[0] = 8.0f;
  filter.process(before, 1000U);
  filter.process(before, 1050U);

  filter.reset();
  auto after = sample();
  after[0] = -3.0f;
  const auto result = filter.process(after, 10U);
  ASSERT_TRUE(result.accepted);
  EXPECT_FLOAT_EQ(-3.0f, result.values[0]);
}

TEST(ImuFilter, ScalarLowPassSmoothsPersistentChange)
{
  auto config = unfilteredConfig();
  config.accel_cutoff_hz = 1.0;
  ImuFilter filter(config);
  auto input = sample();
  filter.process(input, 1000U);
  filter.process(input, 1050U);

  input[0] = 1.0f;
  EXPECT_FLOAT_EQ(0.0f, filter.process(input, 1100U).values[0]);
  const auto result = filter.process(input, 1150U);
  EXPECT_GT(result.values[0], 0.0f);
  EXPECT_LT(result.values[0], 1.0f);
}

TEST(ImuFilter, QuaternionLowPassKeepsUnitNormAndSmoothsOrientation)
{
  auto config = unfilteredConfig();
  config.orientation_cutoff_hz = 1.0;
  ImuFilter filter(config);
  ASSERT_TRUE(filter.process(sample(), 1000U).accepted);

  auto turned = sample();
  constexpr double yaw_deg = 10.0;
  constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
  turned[12] = static_cast<float>(std::cos(0.5 * yaw_deg * degrees_to_radians));
  turned[15] = static_cast<float>(std::sin(0.5 * yaw_deg * degrees_to_radians));
  const auto result = filter.process(turned, 1050U);

  ASSERT_TRUE(result.accepted);
  EXPECT_GT(result.values[11], 0.0f);
  EXPECT_LT(result.values[11], yaw_deg);
  const double norm = std::sqrt(
    result.values[12] * result.values[12] + result.values[13] * result.values[13] +
    result.values[14] * result.values[14] + result.values[15] * result.values[15]);
  EXPECT_NEAR(norm, 1.0, 1e-6);
}
