#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>

#include "core/config/map_params.h"
#include "core/map/voxel_octree.h"
#include "core/types/cloud.h"
#include "core/types/visual_point.h"

namespace livo_slam {

// VoxelMap is the local map that stores the map points and enables fast lookup
// for point-to-plane correspondences.
// All clouds/points passed to VoxelMap are assumed to be in the world frame.
class VoxelMap {
 public:
  explicit VoxelMap(const VoxelMapParams& params);

  void Insert(const Cloud& cloud);

  // Returns the plane the point should be matched against, or nullopt when
  // no voxel near the point holds a suitable one.
  std::optional<Plane> FindPlane(const Eigen::Vector3d& point) const;

  // Returns the visual points visible from a given cloud.
  std::vector<const VisualPoint*> CollectVisualPoints(const Cloud& cloud) const;

  // For visualization
  std::vector<Plane> CollectPlanes() const;

  std::size_t VoxelCount() const { return voxels_.size(); }
  bool Empty() const { return voxels_.empty(); }

 private:
  // Returns the plane in one voxel, if it holds one within max_match_distance.
  std::optional<Plane> PlaneAt(const Eigen::Vector3i& voxel_index,
                               const Eigen::Vector3d& point) const;

 private:
  VoxelMapParams params_;

  std::unordered_map<std::int64_t, VoxelOctree> voxels_;
};
}  // namespace livo_slam