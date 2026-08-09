#include "deskew.h"

namespace livo_slam {

Cloud Deskew(const Cloud& cloud, const Trajectory& trajectory,
             const State& reference, double reference_time,
             const Sophus::SE3d& imu_from_lidar) {
  (void)trajectory;
  (void)reference;
  (void)reference_time;
  (void)imu_from_lidar;
  return cloud;
}

}  // namespace livo_slam
