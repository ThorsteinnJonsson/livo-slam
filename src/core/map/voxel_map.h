#pragma once

#include <array>
#include <cstddef>

#include "core/config/map_params.h"
#include "core/map/voxel_octree.h"
#include "core/types/cloud.h"

namespace livo_slam {

class VoxelMap {
 public:
  explicit VoxelMap(const VoxelMapParams& params);

  void Insert(const Cloud& cloud_world);

 private:
  double voxel_size_;
  std::array<std::size_t, VoxelMapParams::kMaxLayerDepth> layer_init_points_;

  std::unordered_map<std::int64_t, VoxelOctree> voxels_;
};
}  // namespace livo_slam