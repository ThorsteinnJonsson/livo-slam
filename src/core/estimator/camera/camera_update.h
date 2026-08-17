#pragma once

#include "core/map/voxel_map.h"
#include "core/types/estimate.h"
#include "core/types/image_frame.h"
namespace livo_slam {

Estimate UpdateWithCamera(const Estimate& prior, const ImageFrame& image_frame,
                          const VoxelMap& map);
}