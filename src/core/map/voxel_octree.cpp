#include "voxel_octree.h"

#include "core/config/map_params.h"

namespace livo_slam {

VoxelOctree::VoxelOctree(const Eigen::Vector3d& center, double quarter_length,
                         std::size_t layer)
    : center_(center), quarter_length_(quarter_length), layer_(layer) {}

void VoxelOctree::Insert(const Point& point_world) {
  if (!update_enabled_) return;

  if (IsSubdivided()) {
    const auto child_index = ChildIndex(point_world.position);
    VoxelOctree& child = GetChild(child_index);
    child.Insert(point_world);
  } else {
    points_.push_back(point_world);
    ++new_points_;
  }
}

void VoxelOctree::Update(const VoxelMapParams& params) {
  if (new_points_ == 0) {
    // Early return, nothing to update
    return;
  }

  // OUTLINE
  // If subdivided
  //    Update each child
  //    Return
  //
  // If new_points_ == 0, return
  //
  // If not initialized and points_.size() > layer_init_points[layer_]
  //    InitPlane, initialized_ = true
  //    If no plane and layer_ + 1 < kMaxLayerDepth
  //       Subdivide, then Update each child
  // Else if plane_
  //    InitPlane again to refit with the new points
  //    If points_.size() > max_points_per_voxel, freeze and free points_
  //
  // If points_.size() > max_points_per_voxel
  //    update_enabled_ = false, and free points_ if plane_
  //
  // new_points_ = 0
}

void VoxelOctree::InitPlane() {}

void VoxelOctree::Subdivide() {}

std::size_t VoxelOctree::ChildIndex(const Eigen::Vector3d& point_world) const {
  const std::size_t x = point_world.x() > center_.x() ? 1 : 0;
  const std::size_t y = point_world.y() > center_.y() ? 1 : 0;
  const std::size_t z = point_world.z() > center_.z() ? 1 : 0;
  return 4 * x + 2 * y + z;
}

VoxelOctree& VoxelOctree::GetChild(std::size_t index) {
  auto& child = children_[index];
  if (!child) {
    const Eigen::Vector3d offset((index & 4 ? 1.0 : -1.0) * quarter_length_,
                                 (index & 2 ? 1.0 : -1.0) * quarter_length_,
                                 (index & 1 ? 1.0 : -1.0) * quarter_length_);
    child = std::make_unique<VoxelOctree>(center_ + offset,
                                          quarter_length_ / 2.0, layer_ + 1);
  }
  return *child;
}

bool VoxelOctree::IsSubdivided() const {
  return initialized_ && !plane_ && layer_ + 1 < VoxelMapParams::kMaxLayerDepth;
}

}  // namespace livo_slam