#pragma once

#include <Eigen/Core>
#include <memory>

#include "core/config/map_params.h"
#include "core/types/cloud.h"
#include "core/types/plane.h"

namespace livo_slam {

class VoxelOctree {
 public:
  VoxelOctree(const Eigen::Vector3d& center, double quarter_length,
              std::size_t layer);

  void Insert(const Point& point_world, const VoxelMapParams& params);
  void InitPlane();
  void Subdivide();
  int ChildIndex(const Eigen::Vector3d& point_world) const;

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
};

}  // namespace livo_slam