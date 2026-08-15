#include "lidar_update.h"

#include "core/estimator/iekf.h"
#include "core/estimator/lidar_residual.h"

namespace livo_slam {

Estimate UpdateWithLidar(const Estimate& prior, const Cloud& cloud_imu,
                         const VoxelMap& map, const LidarUpdateParams& params) {
  return prior;
}

}  // namespace livo_slam
