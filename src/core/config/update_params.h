#pragma once

namespace livo_slam {

struct LidarUpdateParams {
  int max_iterations = 5;
  double point_noise_std = 0.03;
  double rotation_epsilon = 0.001;
  double position_epsilon = 0.001;
};

}  // namespace livo_slam
