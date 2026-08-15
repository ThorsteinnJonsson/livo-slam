#pragma once

#include "core/config/update_params.h"
#include "core/map/voxel_map.h"
#include "core/types/cloud.h"
#include "core/types/estimate.h"

namespace livo_slam {

Estimate UpdateWithLidar(const Estimate& prior, const Cloud& cloud_imu,
                         const VoxelMap& map, const LidarUpdateParams& params);

}  // namespace livo_slam
