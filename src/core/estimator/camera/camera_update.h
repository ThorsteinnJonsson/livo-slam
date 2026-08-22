#pragma once

#include "core/config/camera_params.h"
#include "core/config/update_params.h"
#include "core/map/voxel_map.h"
#include "core/types/cloud.h"
#include "core/types/estimate.h"
#include "core/types/image_frame.h"

namespace livo_slam {

Estimate UpdateWithCamera(const Estimate& prior, const ImageFrame& image_frame,
                          const Cloud& cloud_imu, const VoxelMap& map,
                          const CameraParams& camera_params,
                          const CameraUpdateParams& update_params);
}