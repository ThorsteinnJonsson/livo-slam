#pragma once

namespace livo_slam {

struct CameraParams {
  // t_imu = t_cam + time_shift
  double time_shift = 0.0;
};

}  // namespace livo_slam
