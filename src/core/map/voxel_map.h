#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include "core/config/map_params.h"
#include "core/map/voxel_octree.h"
#include "core/types/cloud.h"

namespace livo_slam {

class VoxelMap {
 public:
  explicit VoxelMap(const VoxelMapParams& params);

  void Insert(const Cloud& cloud_world);

  std::size_t VoxelCount() const { return voxels_.size(); }
  bool Empty() const { return voxels_.empty(); }

 private:
  VoxelMapParams params_;

  std::unordered_map<std::int64_t, VoxelOctree> voxels_;
};
}  // namespace livo_slam