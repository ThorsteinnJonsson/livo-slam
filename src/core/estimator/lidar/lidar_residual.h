#pragma once

#include <Eigen/Core>
#include <vector>

#include "core/map/voxel_map.h"
#include "core/types/cloud.h"
#include "core/types/state.h"

namespace livo_slam {

struct PointToPlaneMatch {
  Eigen::Vector3d point_imu = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();
  double distance = 0.0;
};

std::vector<PointToPlaneMatch> BuildLidarMatches(const Cloud& cloud_imu,
                                                 const State& state,
                                                 const VoxelMap& map);

}  // namespace livo_slam
