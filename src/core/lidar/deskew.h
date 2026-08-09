#pragma once

#include <sophus/se3.hpp>

#include "core/types/cloud.h"
#include "core/types/state.h"
#include "core/types/trajectory.h"

namespace livo_slam {

Cloud Deskew(const Cloud& cloud, const Trajectory& trajectory,
             const State& reference, double reference_time,
             const Sophus::SE3d& imu_from_lidar);

}  // namespace livo_slam
