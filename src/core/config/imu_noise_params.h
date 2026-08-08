#pragma once

namespace livo_slam {

struct ImuNoiseParams {
  double gyro_noise_std = 0.3162;
  double accel_noise_std = 0.3162;
  double gyro_random_walk_std = 0.01;
  double accel_random_walk_std = 0.01;
};

}  // namespace livo_slam
