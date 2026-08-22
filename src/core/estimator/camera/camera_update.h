#pragma once

#include <opencv2/core.hpp>

#include "core/config/camera_params.h"
#include "core/config/update_params.h"
#include "core/map/voxel_map.h"
#include "core/types/cloud.h"
#include "core/types/estimate.h"

namespace livo_slam {

Estimate UpdateWithCamera(const Estimate& prior,
                          const std::vector<cv::Mat>& image_pyramid,
                          const std::vector<Point>& points_imu,
                          const VoxelMap& map,
                          const CameraParams& camera_params,
                          const CameraUpdateParams& update_params);
}