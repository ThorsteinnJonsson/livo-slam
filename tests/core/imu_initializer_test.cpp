#include "core/imu/imu_initializer.h"

#include <gtest/gtest.h>

#include <vector>

namespace livo_slam {
namespace {

ImuInitParams SmallWindow() {
  ImuInitParams params;
  params.window_size = 20;
  return params;
}

std::vector<ImuSample> MakeSamples(std::size_t count, double noise,
                                   double start_time = 0.0) {
  std::vector<ImuSample> samples;
  samples.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    ImuSample sample;
    sample.timestamp = start_time + 0.0025 * static_cast<double>(i);
    sample.linear_acceleration = Eigen::Vector3d(0.0, 0.0, 9.81) +
                                 Eigen::Vector3d::Constant(sign * noise);
    sample.angular_velocity = Eigen::Vector3d(0.01, 0.02, 0.03) +
                              Eigen::Vector3d::Constant(sign * noise);
    samples.push_back(sample);
  }
  return samples;
}

TEST(ImuInitializer, CollectsUntilWindowIsFull) {
  ImuInitializer initializer(SmallWindow());
  const auto samples = MakeSamples(10, 0.0);

  initializer.Add(samples);

  EXPECT_EQ(initializer.status(), InitStatus::kCollecting);
  EXPECT_FALSE(initializer.init_result().has_value());
}

TEST(ImuInitializer, InitializesOnStationaryWindow) {
  ImuInitializer initializer(SmallWindow());
  initializer.Add(MakeSamples(20, 0.0));

  ASSERT_EQ(initializer.status(), InitStatus::kInitialized);
  ASSERT_TRUE(initializer.init_result().has_value());

  const InitResult& result = *initializer.init_result();
  EXPECT_NEAR(result.accel_mean_norm, 9.81, 1e-9);
  EXPECT_NEAR(result.gyro_bias.x(), 0.01, 1e-9);
  EXPECT_NEAR(result.gyro_bias.z(), 0.03, 1e-9);
  EXPECT_NEAR(result.gravity.z(), -9.81, 1e-9);
}

TEST(ImuInitializer, RejectsMovingWindowAndRecoversWhenStill) {
  ImuInitializer initializer(SmallWindow());

  initializer.Add(MakeSamples(40, 1.0));
  EXPECT_EQ(initializer.status(), InitStatus::kNotStationary);
  EXPECT_FALSE(initializer.init_result().has_value());

  initializer.Add(MakeSamples(40, 0.0, 1.0));
  EXPECT_EQ(initializer.status(), InitStatus::kInitialized);
  EXPECT_TRUE(initializer.init_result().has_value());
}

TEST(ImuInitializer, IgnoresSamplesOnceInitialized) {
  ImuInitializer initializer(SmallWindow());
  initializer.Add(MakeSamples(20, 0.0));
  ASSERT_EQ(initializer.status(), InitStatus::kInitialized);

  const double first_timestamp = initializer.init_result()->timestamp;
  initializer.Add(MakeSamples(40, 1.0, 10.0));

  EXPECT_EQ(initializer.status(), InitStatus::kInitialized);
  EXPECT_DOUBLE_EQ(initializer.init_result()->timestamp, first_timestamp);
}

TEST(ImuInitializer, ResultTimestampIsLastSampleOfWindow) {
  ImuInitializer initializer(SmallWindow());
  const auto samples = MakeSamples(20, 0.0);
  initializer.Add(samples);

  ASSERT_TRUE(initializer.init_result().has_value());
  EXPECT_DOUBLE_EQ(initializer.init_result()->timestamp,
                   samples.back().timestamp);
}

}  // namespace
}  // namespace livo_slam
