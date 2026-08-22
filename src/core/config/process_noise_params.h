#pragma once

namespace livo_slam {

struct ProcessNoiseParams {
  double gyro_noise_std = 0.3162;
  double accel_noise_std = 0.3162;
  double gyro_random_walk_std = 0.01;
  double accel_random_walk_std = 0.01;
  double inv_exposure_random_walk_std = 0.1;
};

}  // namespace livo_slam
