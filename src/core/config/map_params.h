#pragma once

#include <cstddef>

namespace livo_slam {

struct VoxelMapParams {
  // Each voxel can be subdivided like an octree to kMaxLayerDepth
  static constexpr std::size_t kMaxLayerDepth = 3;
};

}  // namespace livo_slam
