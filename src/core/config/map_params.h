#pragma once

#include <array>
#include <cstddef>

namespace livo_slam {

struct VoxelMapParams {
  // Each voxel can be subdivided like an octree to kMaxLayerDepth
  static constexpr std::size_t kMaxLayerDepth = 3;

  double voxel_size = 0.5;
  std::array<std::size_t, kMaxLayerDepth> layer_init_points = {5, 5, 5};
};

}  // namespace livo_slam
