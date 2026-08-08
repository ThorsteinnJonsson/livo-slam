#pragma once

namespace livo_slam {

struct InitialStateParams {
  double rotation_std = 0.017;
  double position_std = 0.001;
  double velocity_std = 0.01;
  double gyro_bias_std = 0.001;
  double accel_bias_std = 0.01;
  double gravity_std = 0.01;
};

}  // namespace livo_slam
