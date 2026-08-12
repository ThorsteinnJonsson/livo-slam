#include "voxel_octree.h"

#include <Eigen/Eigenvalues>

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
  if (IsSubdivided()) {
    for (const auto& child : children_) {
      if (child) {
        child->Update(params);
      }
    }
    return;
  }

  if (new_points_ == 0) {
    // Early return, nothing to update
    return;
  }

  if (!initialized_ && points_.size() > params.layer_init_points[layer_]) {
    InitPlane(params);
    initialized_ = true;
  } else if (plane_) {
    InitPlane(params);  // Refit with new points
  }

  if (initialized_ && !plane_ && layer_ + 1 < VoxelMapParams::kMaxLayerDepth) {
    Subdivide();
    for (const auto& child : children_) {
      if (child) {
        child->Update(params);
      }
    }
  }

  if (points_.size() > params.max_points_per_voxel) {
    update_enabled_ = false;
    if (plane_) {
      points_.clear();
    }
  }

  new_points_ = 0;
}

void VoxelOctree::InitPlane(const VoxelMapParams& params) {
  plane_.reset();
  if (points_.size() < 3) {
    return;
  }

  const double count = static_cast<double>(points_.size());

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const Point& point : points_) {
    centroid += point.position;
  }
  centroid /= count;

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const Point& point : points_) {
    const Eigen::Vector3d offset = point.position - centroid;
    covariance += offset * offset.transpose();
  }
  covariance /= count;

  // Eigenvalues come out in increasing order, so the first one is the
  // mean squared distance to the best-fit plane.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  const double variance = solver.eigenvalues()(0);
  if (variance > params.max_plane_variance) {
    return;
  }

  Plane plane;
  plane.centroid = centroid;
  plane.normal = solver.eigenvectors().col(0);
  plane.variance = variance;
  plane_ = plane;
}

void VoxelOctree::Subdivide() {
  for (const Point& point : points_) {
    const auto child_index = ChildIndex(point.position);
    VoxelOctree& child = GetChild(child_index);
    child.Insert(point);
  }
  points_.clear();
}

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