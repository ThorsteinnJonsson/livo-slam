#pragma once

#include <cstddef>

namespace livo_slam {

struct ImuInitParams {
  std::size_t window_size = 200;
  double gravity_magnitude = 9.81;
  double max_accel_var = 0.05;
  double max_gyro_var = 0.01;
};

}  // namespace livo_slam
