#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include "core/config/map_params.h"
#include "core/types/cloud.h"
#include "core/types/plane.h"

namespace livo_slam {

// Voxel octree is the datastructure at each voxel in the voxel map. It can be
// subdivided into smaller units, which are also VoxelOctree.
// All points passed to this datastructure are assumed to be in the world frame.
class VoxelOctree {
 public:
  VoxelOctree(const Eigen::Vector3d& center, double quarter_length,
              std::size_t layer);

  void Insert(const Point& point, const VoxelMapParams& params);
  void Update(const VoxelMapParams& params);

  // Descends to the deepest node covering the point that holds a plane.
  // Returns nullptr when no node along the path has one.
  const VoxelOctree* FindCorrespondence(const Eigen::Vector3d& point) const;

  // For visualization
  void CollectPlanes(std::vector<Plane>& planes) const;

  const std::optional<Plane>& plane() const { return plane_; }
  std::size_t PointCount() const { return points_.size(); }

 private:
  void InitPlane(const VoxelMapParams& params);
  void Subdivide(const VoxelMapParams& params);

  std::size_t ChildIndex(const Eigen::Vector3d& point) const;
  // Finds the child voxel octree (one layer down), creates it if it doesn't
  // exist.
  VoxelOctree& GetChild(std::size_t index);
  bool IsSubdivided() const;

 private:
  // Geometry
  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();  // Center of the node.
  double quarter_length_ = 0.0;  // Children are at center_ ± quarter_length_.
  std::size_t layer_ = 0;

  // Data
  std::vector<Point> points_;
  std::optional<Plane> plane_;
  std::array<std::unique_ptr<VoxelOctree>, 8> children_;

  // State
  bool initialized_ =
      false;  // Not enough points yet for plane fit. Not a
              // plane if initialized and plane_ is std::nullopt.
  bool update_enabled_ = true;
  std::size_t new_points_ =
      0;  // New points added this frame, used to determine if we should attempt
          // to fit a plane.
          // - Not yet init + enough points → fit plane or subdivide.
          // - Already a plane → refit with the new points.
          // - Init but not planar → recurse into children.
};

}  // namespace livo_slam