#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>

#include "core/config/map_params.h"
#include "core/map/voxel_octree.h"
#include "core/types/cloud.h"

namespace livo_slam {

class VoxelMap {
 public:
  explicit VoxelMap(const VoxelMapParams& params);

  void Insert(const Cloud& cloud_world);

  // Returns the plane the point should be matched against, or nullopt when
  // no voxel near the point holds a suitable one.
  std::optional<Plane> FindPlane(const Eigen::Vector3d& point_world) const;

  std::size_t VoxelCount() const { return voxels_.size(); }
  bool Empty() const { return voxels_.empty(); }

 private:
  VoxelMapParams params_;

  std::unordered_map<std::int64_t, VoxelOctree> voxels_;
};
}  // namespace livo_slam