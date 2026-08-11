#pragma once

#include <array>
#include <cstddef>

#include "core/config/map_params.h"

namespace livo_slam {

class VoxelMap {
 public:
  VoxelMap();

 private:
  double voxel_size_;
  std::array<std::size_t, VoxelMapParams::kMaxLayerDepth> layer_init_points_;
};
}  // namespace livo_slam